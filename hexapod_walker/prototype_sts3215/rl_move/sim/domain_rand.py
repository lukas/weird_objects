"""Per-episode domain randomization for the sim twin.

Randomizes exactly the quantities we cannot pin down (or that drift):

Physics / geometry (mutated on the loaded MjModel — no XML rebuild):
- chassis mass / CoM (battery position, wiring) + per-link mass jitter
- leg geometry: global link-length scale (print / CAD error) and
  independent per-leg per-segment variation (assembly tolerance).
  The policy-side IK keeps NOMINAL lengths — the mismatch is the point.
- foot / ground friction, contact compliance (table vs. carpet)
- ground slope (tilted gravity vector)
- actuator kp / kv scale (unit-to-unit servo spread), torque/voltage scale

Actuation / sensing (applied in the env, not the model):
- command latency, deadband, velocity-ceiling scale, dropped SyncWrites
- joint zero bias (encoder / set_zero error — the thing that bit us on
  2026-08-06), encoder noise
- IMU mount misalignment (random rotation chassis→IMU, hits tilt AND
  gyro), tilt/gyro bias + noise
- IMU mount POSITION (the IMU could be bolted anywhere on the robot).
  Position doesn't change static tilt, but an off-center IMU feels
  lever-arm accelerations whenever the body rotates, corrupting the
  accel-derived tilt exactly while the robot is leaning. The sim env
  computes the accelerometer at the randomized point so this shows up.
- action noise

Ranges are data-driven where possible: ``DomainRandomizer.from_params``
widens kp/kv/latency ranges using the measured joint-to-joint spread that
``fit_motor_model.py`` stores in ``sim_model.json``; everything else uses
the conservative defaults below.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .servo_model import AXES, N_JOINTS, SimServoParams

DEG2RAD = math.pi / 180.0
N_LEGS = 6
G0 = 9.80665

# Frozen-joint DOF damping (N·m·s/rad). Seized-gearbox approximation:
# implicit (unconditionally stable) viscous lock. Against the fitted
# joint kv range (0.02-3.0) this is a 150-25000x stiffening; measured
# creep under a worst-case full-body drop-settle load is ~0.05 rad
# over 2 s (bent knee, whole robot landing on it) and far less under
# ordinary stance loads — effectively frozen at episode (15-60 s)
# scale with zero stepper changes (dof_damping is per-world model DR).
FROZEN_DOF_DAMPING = 500.0


def _rot_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


@dataclass
class RandRanges:
    """Half-widths / bounds for each randomized quantity."""
    mass_scale: tuple[float, float] = (0.85, 1.20)
    com_offset_m: float = 0.012          # chassis CoM shift, each of x/y
    leg_mass_jitter_pct: float = 0.10    # per leg-link, on top of mass_scale
    link_len_scale_pct: float = 0.02     # global print/CAD length error
    link_len_leg_pct: float = 0.012      # per-leg per-segment assembly spread
    friction_scale: tuple[float, float] = (0.6, 1.4)
    contact_stiff_scale: tuple[float, float] = (0.7, 2.0)  # solref timeconst
    ground_tilt_deg: float = 2.0         # floor slope via gravity vector
    kp_scale_pct: float = 0.20           # ± around fitted kp, per joint
    kv_scale_pct: float = 0.25
    torque_scale: tuple[float, float] = (0.80, 1.05)  # battery sag / spread
    latency_scale: tuple[float, float] = (0.7, 1.8)
    deadband_scale: tuple[float, float] = (0.5, 1.8)
    vel_scale: tuple[float, float] = (0.85, 1.10)
    cmd_drop_prob_max: float = 0.05      # lost SyncWrite per control tick
    # Start pose: how the human placed the robot this episode.
    placement_noise_deg: float = 2.0     # per-joint hand-placement slop
    bad_start_prob: float = 0.25         # episodes with badly-off joints
    bad_start_max_joints: int = 3        # how many joints can be way off
    bad_start_deg: tuple[float, float] = (8.0, 35.0)  # offset magnitude
    joint_zero_bias_deg: float = 1.0     # per-joint set_zero error
    # Logical-zero drift frame (operator directive 08-10, GPT handoff §7):
    # 0 = legacy, zero bias corrupts only the ENCODER READS — but that
    # leaves a permanent cmd-vs-read residual the policy can exploit,
    # which hardware never shows. 1 = the bias is a FRAME SHIFT: position
    # commands are translated into the same drifted frame the reads come
    # from (set_zero done on a slumped pose shifts BOTH). Reads and
    # commands stay self-consistent; the drift is only visible through
    # physics (gravity/contacts/IMU) — exactly the failure that dropped
    # the robot on 08-09. Flag, not a range: never scaled by dr-scale.
    zero_drift_cmd_frame: float = 0.0
    encoder_noise_deg: float = 0.09      # ~1 LSB of the 12-bit encoder
    # IMU could be installed anywhere: gross orientation is canonicalized
    # once by imu_calibrate, so rotation DR covers the RESIDUAL error;
    # position DR covers the full plausible mounting envelope (chassis
    # deck to raised platform) because no calibration removes lever-arm
    # acceleration effects.
    imu_mount_deg: float = 10.0          # residual mount rotation after calib
    imu_pos_xy_m: float = 0.07           # mount offset from chassis center
    imu_pos_z_m: tuple[float, float] = (-0.02, 0.10)  # below deck … platform
    imu_bias_deg: float = 1.0            # extra roll/pitch bias (calib error)
    tilt_noise_deg: float = 0.3
    gyro_bias_deg_s: float = 0.5
    gyro_noise_deg_s: float = 0.5
    action_noise: float = 0.02
    # Tipped start (roll recovery) — default ON everywhere, operator
    # ruling 08-10 after the dep-vref1-r1 hardware walk rolled away
    # monotonically to the 25° trip with zero corrective response
    # (rl_docs/HARDWARE.md "runaway roll"): plant-family episodes
    # sometimes BEGIN at a sustained body roll (asymmetric leg fold,
    # settled under gravity) while the tilt reference stays LEVEL, so
    # the policy sees the lean in obs and is paid to level out. Applied
    # in sim_env at plant/park starts only (never belly-rise), capped by
    # the run's safety envelope; see HexapodSimEnv._tipped_offset_rad.
    tipped_start_prob: float = 0.30
    tipped_start_deg: tuple[float, float] = (6.0, 18.0)  # target body roll
    # Rise rocking (hardware 08-11, bench_blast camera sessions; dose
    # + shape recalibrated 08-12 from open-loop replay of all 10
    # recorded stand failures — see sim_env._rise_rock_offset and
    # rl_move/sim/replay_trace.py): the real rise is FLAT through the
    # belly curl, then rolls 0→10.6° in the last ~1.2 s of the height
    # ramp as the belly unloads onto a near-diagonal foot pair, and
    # trips tilt_roll 10/10 at the same tick, while the sim's rise
    # stays ≤2.7° under BOTH actuator fits (joints track the tapes at
    # ~1° RMSE; μ 2→50 and CoM shifts to 10 mm change nothing — the
    # gap is a support knife-edge, not actuator/friction/CoM). Axis:
    # rise-mode episodes sometimes carry a one-side hip/knee fold bias
    # on the PHYSICAL servo command, RAMP-GATED by rise progress
    # (sim_env._rise_rock_offset; same fold→roll mapping as the
    # tipped start). Encoders read the true (drooped) angles and the
    # tilt reference stays level, so the policy is paid to close the
    # command-vs-read loop and level out — the exact skill the
    # hardware curl demands. Replay calibration: ~18° dose crosses
    # the 10° trip band with the recorded failure's own shape on the
    # branch that removes the catching foot. Default OFF (opt-in via
    # dr.rise_rock_*).
    rise_rock_prob: float = 0.0
    rise_rock_deg: tuple[float, float] = (8.0, 18.0)     # target body roll
    # Walk takeoff kick (hardware 08-11, bench_report over 18 walks):
    # EVERY hardware walk crosses 5° roll within 0.6-1.5 s of gait
    # start and peaks 13-27°, at sustained roll RATES of 11-46 °/s
    # (median 24, p90 45; gyro spikes to ~90) — while sim plant-start
    # episodes take off level, so surviving the transient is luck.
    # Static leans do NOT close the gap (cw-dep-tip1-takeoff25-r1:
    # child==parent at the matched 20-25° dose, lever closed) — the
    # gap is dynamic. Axis: walk-mode episodes sometimes get a
    # TRANSIENT one-side fold pulse on the PHYSICAL servo command over
    # the first ~second of gait (sim_env._walk_kick_offset, half-sine
    # ramp in and out, net-zero terminal offset).
    # DOSE CEILING MEASURED 08-12 (replay_trace calibration session):
    # the fold pulse SATURATES — frozen-plant response is 5.4° peak at
    # a 14° draw, 6.8-9.8° at a 30° draw, roll rates capped ~10 °/s by
    # the planted opposite feet + the servo write profile. It CANNOT
    # reach the hardware takeoff regime at any dose, which retro-
    # explains cw-dep-tip1-kick1's 0/24-falls-both null: the arm never
    # tested the hypothesis. Kept for reproducibility; superseded by
    # dr.walk_push_* (base torque pulse) below.
    walk_kick_prob: float = 0.0
    walk_kick_deg: tuple[float, float] = (8.0, 18.0)     # peak target roll
    walk_kick_s: tuple[float, float] = (0.5, 1.2)        # pulse duration
    # Walk takeoff PUSH (08-12, the mechanism the kick could not
    # deliver): half-sine roll TORQUE pulse on the chassis
    # (xfrc_applied) over the first ~second of walk-mode episodes —
    # a true roll-rate disturbance that bypasses the actuator path,
    # so it can reproduce the measured takeoff regime. Open-loop walk
    # replays (replay_trace on the 08-11 tapes) show the recorded
    # actions ALREADY rock the sim plant 8-25° — the transient is the
    # gait's own load transfer; this axis forces closed-loop rollouts
    # to visit those states instead of relying on takeoff luck.
    # Dose CALIBRATED policy-in-the-loop (tip1 walking fwd 0.05 m/s,
    # DR0 det, 08-12): the planted 6-foot stance absorbs any
    # plausible torque (2.2 Nm -> 0.2°) — the pulse lands when it
    # overlaps a tripod swing phase, so the duration must cover the
    # first gait cycle. 2.0 Nm/1.5 s: peaks 3-6°, no falls (soft);
    # 2.6 Nm/1.5 s: peaks {2.6, 3.2, 12.3, 30.2-fall} = the hardware
    # coin-flip regime, 5° crossings 0.56-0.76 s; 3.2 Nm: 3/4 falls
    # at 64-105 °/s (beyond the tapes' 11-46). Works on BOTH stacks:
    # C envs apply the xfrc in _advance; the MJX vec envs read each
    # shim's per-tick torque and hand it to the batched stepper
    # (plumbed 08-12). Default OFF (opt-in via dr.walk_push_*).
    walk_push_prob: float = 0.0
    walk_push_nm: tuple[float, float] = (2.0, 3.0)       # peak |torque|
    walk_push_s: tuple[float, float] = (0.8, 1.5)        # pulse duration
    # Per-joint FAULT INJECTION (AMP brief §8; M0 checklist "fault
    # injection works"). With prob fault_prob an episode carries ONE
    # fault, drawn from fault_mix = (weakened joint, frozen joint,
    # disabled leg):
    #   - weakened: one joint's servo at fault_weak_scales strength
    #     (kp + torque limit scaled; 0.0 = dead servo, free-swinging
    #     against its kv backdrive damping);
    #   - frozen: one joint's actuator force zeroed and its DOF locked
    #     with FROZEN_DOF_DAMPING (seized-gearbox approximation — it
    #     creeps ~2 deg/15 s under a 0.5 N·m gravity load, close
    #     enough to "frozen at current position" at episode scale);
    #   - disabled leg: all 3 joints of one leg dead (scale 0.0).
    # Implementation is PURE MjModel field edits (actuator_gainprm/
    # biasprm/forcerange, dof_damping) — every touched field is in
    # mjx_backend.MODEL_DR_FIELDS, so the per-world model-DR upload
    # carries faults to the batched GPU stacks with ZERO stepper
    # changes. Default OFF (opt-in via dr.fault_*); the draw is
    # GUARDED so fault_prob=0 keeps the legacy rng stream bit-exact.
    fault_prob: float = 0.0
    fault_weak_scales: tuple[float, ...] = (0.7, 0.4, 0.2, 0.0)
    fault_mix: tuple[float, float, float] = (0.45, 0.25, 0.30)
    # Mid-episode external PUSH (AMP brief §7.4/§9.3, M3 push-recovery
    # curriculum). Distinct from dr.walk_push_* above (a fixed roll
    # TORQUE confined to the first ~1.5s that reproduces the measured
    # hardware TAKEOFF wobble): this is a random-direction horizontal
    # FORCE pulse fired once at a random point later in a walk-mode
    # episode, on a policy that is already walking, not taking off --
    # the actual "shove it mid-stride and see if it recovers" test the
    # brief and the M3/§9.3 cross-engine gate ask for ("recovers from
    # moderate pushes"). Half-sine ramp in/out like every pulse in this
    # file (net momentum, never a step discontinuity in xfrc). Direction
    # is drawn in the WORLD frame: walk episodes already visit every
    # heading via yaw-cmd, so a world-frame draw already covers
    # lateral/fore-aft/diagonal relative to the robot per brief §7.4
    # without needing a per-tick body-frame rotation. Default OFF
    # (opt-in via dr.ext_push_prob); guarded draw, same convention as
    # tipped/rock/kick/push/fault above (probability follows the
    # curriculum, the dose menu does not).
    ext_push_prob: float = 0.0
    ext_push_n: tuple[float, float] = (10.0, 25.0)       # peak |force| N
    ext_push_dur_s: tuple[float, float] = (0.15, 0.4)    # pulse duration
    ext_push_start_s: tuple[float, float] = (1.5, 9.0)   # delay from ep start

    def scaled(self, s: float) -> "RandRanges":
        """Curriculum knob: shrink every range toward nominal by ``s``.

        s=1 is full randomization, s=0 is the calibrated nominal sim.
        Sensor NOISE floors (encoder, tilt, gyro noise) are kept at full
        strength even at s=0 — real sensors are always noisy; it's the
        structural/bias randomization that makes early learning hard.
        """
        s = max(0.0, min(1.0, float(s)))

        def pair(lo: float, hi: float) -> tuple[float, float]:
            return (1.0 + (lo - 1.0) * s, 1.0 + (hi - 1.0) * s)

        return RandRanges(
            mass_scale=pair(*self.mass_scale),
            com_offset_m=self.com_offset_m * s,
            leg_mass_jitter_pct=self.leg_mass_jitter_pct * s,
            link_len_scale_pct=self.link_len_scale_pct * s,
            link_len_leg_pct=self.link_len_leg_pct * s,
            friction_scale=pair(*self.friction_scale),
            contact_stiff_scale=pair(*self.contact_stiff_scale),
            ground_tilt_deg=self.ground_tilt_deg * s,
            kp_scale_pct=self.kp_scale_pct * s,
            kv_scale_pct=self.kv_scale_pct * s,
            torque_scale=pair(*self.torque_scale),
            latency_scale=pair(*self.latency_scale),
            deadband_scale=pair(*self.deadband_scale),
            vel_scale=pair(*self.vel_scale),
            cmd_drop_prob_max=self.cmd_drop_prob_max * s,
            placement_noise_deg=self.placement_noise_deg * s,
            bad_start_prob=self.bad_start_prob * s,
            bad_start_max_joints=self.bad_start_max_joints,
            bad_start_deg=(self.bad_start_deg[0] * s,
                           self.bad_start_deg[1] * s),
            joint_zero_bias_deg=self.joint_zero_bias_deg * s,
            zero_drift_cmd_frame=self.zero_drift_cmd_frame,
            encoder_noise_deg=self.encoder_noise_deg,
            imu_mount_deg=self.imu_mount_deg * s,
            imu_pos_xy_m=self.imu_pos_xy_m * s,
            imu_pos_z_m=(self.imu_pos_z_m[0] * s, self.imu_pos_z_m[1] * s),
            imu_bias_deg=self.imu_bias_deg * s,
            tilt_noise_deg=self.tilt_noise_deg,
            gyro_bias_deg_s=self.gyro_bias_deg_s * s,
            gyro_noise_deg_s=self.gyro_noise_deg_s,
            action_noise=self.action_noise * s,
            # Probability follows the curriculum; the DOSE does not
            # (like the sensor noise floors): a champion at dr 0.35
            # must still see real 6-18° leans, not homeopathic 2-6°
            # ones — a shrunken dose never visits the states the
            # hardware actually fails in.
            tipped_start_prob=self.tipped_start_prob * s,
            tipped_start_deg=self.tipped_start_deg,
            # Same convention as tipped: probability follows the
            # curriculum, the dose does not.
            rise_rock_prob=self.rise_rock_prob * s,
            rise_rock_deg=self.rise_rock_deg,
            walk_kick_prob=self.walk_kick_prob * s,
            walk_kick_deg=self.walk_kick_deg,
            walk_kick_s=self.walk_kick_s,
            walk_push_prob=self.walk_push_prob * s,
            walk_push_nm=self.walk_push_nm,
            walk_push_s=self.walk_push_s,
            # Same convention as tipped/rock/push: probability follows
            # the curriculum, the dose (strength menu / mix) does not —
            # a half-strength fault is a different, easier fault.
            fault_prob=self.fault_prob * s,
            fault_weak_scales=self.fault_weak_scales,
            fault_mix=self.fault_mix,
            # Same convention as walk_push/fault: probability follows
            # the curriculum, the dose (force/duration/timing menu)
            # does not -- a homeopathic shove never visits the states
            # the hardware push-recovery gate actually needs.
            ext_push_prob=self.ext_push_prob * s,
            ext_push_n=self.ext_push_n,
            ext_push_dur_s=self.ext_push_dur_s,
            ext_push_start_s=self.ext_push_start_s,
        )


@dataclass
class EpisodeRandomization:
    """One sampled episode's perturbations."""
    mass_scale: float
    com_offset_m: np.ndarray             # (3,)
    leg_mass_scale: np.ndarray           # (6, 3) coxa/femur/tibia bodies
    link_scale: np.ndarray               # (6, 3) coxa/femur/tibia lengths
    friction_scale: float
    contact_stiff_scale: float
    gravity_vec: np.ndarray              # (3,) tilted, |g| = 9.80665
    kp_scale: np.ndarray                 # (18,)
    kv_scale: np.ndarray                 # (18,)
    torque_scale: float
    latency_scale: float
    deadband_scale: float
    vel_scale: float
    cmd_drop_prob: float
    start_offset_rad: np.ndarray         # (18,) placement noise (+ bad start)
    bad_start_joints: list[int]          # joints that start way off
    joint_zero_bias_rad: np.ndarray      # (18,)
    zero_drift_cmd_frame: bool           # bias shifts cmd frame too
    encoder_noise_rad: float
    imu_mount_rot: np.ndarray            # (3, 3) chassis → IMU frame
    imu_pos_m: np.ndarray                # (3,) IMU offset from chassis origin
    imu_bias_rad: np.ndarray             # (2,) roll, pitch
    tilt_noise_rad: float
    gyro_bias_rad_s: np.ndarray          # (3,)
    gyro_noise_rad_s: float
    action_noise: float
    # Signed target body roll for a tipped start (0 = level episode).
    # + rolls the body toward its right side (legs 3-5), − toward the
    # left. sim_env maps this to an asymmetric leg-fold start offset at
    # plant/park starts and keeps the tilt reference LEVEL.
    tipped_roll_deg: float = 0.0
    # Signed target body roll for the rise-rock command bias (0 = no
    # rocking this episode; rise-mode episodes only, same sign
    # convention as tipped_roll_deg).
    rise_rock_roll_deg: float = 0.0
    # Signed PEAK target roll + pulse duration for the walk takeoff
    # kick (0 = no kick this episode; walk-mode episodes only, same
    # sign convention as tipped_roll_deg).
    walk_kick_roll_deg: float = 0.0
    walk_kick_dur_s: float = 0.0
    # Signed PEAK chassis roll torque (N·m) + pulse duration for the
    # walk takeoff push (0 = no push this episode; walk-mode episodes
    # only; + rolls the body toward its right side, same convention
    # as tipped_roll_deg).
    walk_push_peak_nm: float = 0.0
    walk_push_dur_s: float = 0.0
    # Fault injection (dr.fault_*, see RandRanges). fault_mode "" =
    # healthy episode (all fault fields inert, apply_fault_to_model
    # is a no-op). "weak"/"frozen" carry ONE joint index in
    # fault_joints; "leg" carries that leg's 3 joint indices.
    # fault_scale is the strength multiplier for weak/leg (0.0 = dead
    # servo) and is ignored for frozen.
    fault_mode: str = ""
    fault_joints: tuple[int, ...] = ()
    fault_scale: float = 1.0
    # Mid-episode external push (dr.ext_push_*, see RandRanges): peak
    # |force| N + pulse duration + delay from episode start + world-
    # frame direction (0 = no push this episode; walk-mode episodes
    # only, see sim_env._ext_push_force_n).
    ext_push_peak_n: float = 0.0
    ext_push_dur_s: float = 0.0
    ext_push_start_s: float = 0.0
    ext_push_dir_rad: float = 0.0

    def fault_health(self) -> np.ndarray:
        """(18,) health vector per AMP brief §8.2: 1.0 healthy, 0.0
        disabled/frozen, intermediate = degraded strength. Deployable
        as actor obs once M4 wiring lands; also handy for eval
        reports."""
        h = np.ones(N_JOINTS, dtype=np.float32)
        if self.fault_mode:
            v = 0.0 if self.fault_mode == "frozen" else float(self.fault_scale)
            for j in self.fault_joints:
                h[j] = v
        return h

    def apply_fault_to_model(self, model) -> None:
        """Apply the episode's fault as pure MjModel field edits.

        Call AFTER ``servo_model.apply_params_to_model`` (which SETS
        actuator gain/bias/forcerange rows each reset — these edits
        multiply/override them). Touches only fields in
        ``mjx_backend.MODEL_DR_FIELDS`` (actuator_gainprm/biasprm/
        forcerange, dof_damping) so the per-world model-DR path uploads
        faults to the batched stacks unchanged. No-op when healthy.
        """
        if not self.fault_mode:
            return
        from .servo_model import _act_id, joint_names, joint_qvel_addrs

        names = joint_names()
        dadr = joint_qvel_addrs(model)
        for j in self.fault_joints:
            pa = _act_id(model, names[j])
            va = _act_id(model, names[j] + "_d")
            if self.fault_mode == "frozen":
                # Seized gearbox: servo can't move it, backdrive locked.
                model.actuator_forcerange[pa] = (0.0, 0.0)
                model.actuator_forcerange[va] = (0.0, 0.0)
                model.dof_damping[dadr[j]] = FROZEN_DOF_DAMPING
            else:
                s = float(self.fault_scale)
                model.actuator_gainprm[pa, 0] *= s
                model.actuator_biasprm[pa, 1] *= s
                model.actuator_forcerange[pa] *= s
                model.actuator_forcerange[va] *= s
                # scale 0.0 = dead servo: joint free-swings against its
                # existing kv damping + frictionloss (backdrive), which
                # stay untouched.

    def apply_to_model(self, model, *, chassis_bid: int) -> None:
        """Mutate a (freshly restored) MjModel in place."""
        import mujoco

        def bid(name: str) -> int:
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

        def gid(name: str) -> int:
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

        def sid(name: str) -> int:
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)

        # Mass: global scale + chassis CoM shift + per-link jitter.
        model.body_mass[:] *= self.mass_scale
        model.body_inertia[:] *= self.mass_scale
        model.body_ipos[chassis_bid] += self.com_offset_m

        # Leg geometry + per-link mass. Link lengths in the MJCF are just
        # attachment offsets: coxa length = L{i}_femur body x, femur length
        # = L{i}_tibia body x, tibia length = pad-hinge body x.
        for i in range(N_LEGS):
            b_yaw = bid(f"L{i}_yaw")
            b_fem = bid(f"L{i}_femur")
            b_tib = bid(f"L{i}_tibia")
            b_pad = bid(f"L{i}_pad")
            s_coxa, s_femur, s_tibia = self.link_scale[i]

            model.body_pos[b_fem, 0] *= s_coxa
            model.body_pos[b_tib, 0] *= s_femur
            model.body_pos[b_pad, 0] *= s_tibia
            # CoM of each link moves with its length.
            model.body_ipos[b_yaw, 0] *= s_coxa
            model.body_ipos[b_fem, 0] *= s_femur
            model.body_ipos[b_tib, 0] *= s_tibia

            for k, b in enumerate((b_yaw, b_fem, b_tib)):
                model.body_mass[b] *= self.leg_mass_scale[i, k]
                model.body_inertia[b] *= self.leg_mass_scale[i, k]

        # Ground: sliding friction, contact compliance, slope (via gravity).
        model.geom_friction[:, 0] *= self.friction_scale
        model.geom_solref[:, 0] *= self.contact_stiff_scale
        model.opt.gravity[:] = self.gravity_vec

    def summary(self) -> dict:
        tilt = math.degrees(math.acos(
            min(1.0, -float(self.gravity_vec[2]) / G0)))
        return {
            "mass_scale": round(self.mass_scale, 3),
            "com_offset_mm": [round(v * 1000, 1) for v in self.com_offset_m],
            "link_scale_range": [round(float(np.min(self.link_scale)), 3),
                                 round(float(np.max(self.link_scale)), 3)],
            "friction_scale": round(self.friction_scale, 3),
            "ground_tilt_deg": round(tilt, 2),
            "kp_scale_mean": round(float(np.mean(self.kp_scale)), 3),
            "torque_scale": round(self.torque_scale, 3),
            "latency_scale": round(self.latency_scale, 3),
            "deadband_scale": round(self.deadband_scale, 3),
            "cmd_drop_prob": round(self.cmd_drop_prob, 3),
            "imu_pos_mm": [round(v * 1000, 1) for v in self.imu_pos_m],
            "zero_bias_max_deg": round(
                float(np.max(np.abs(self.joint_zero_bias_rad))) / DEG2RAD, 2),
            "zero_drift_cmd_frame": bool(self.zero_drift_cmd_frame),
            "bad_start_joints": [int(j) for j in self.bad_start_joints],
            "start_offset_max_deg": round(
                float(np.max(np.abs(self.start_offset_rad))) / DEG2RAD, 1),
            "tipped_roll_deg": round(self.tipped_roll_deg, 1),
            "rise_rock_roll_deg": round(self.rise_rock_roll_deg, 1),
            "walk_kick_roll_deg": round(self.walk_kick_roll_deg, 1),
            "walk_kick_dur_s": round(self.walk_kick_dur_s, 2),
            "walk_push_peak_nm": round(self.walk_push_peak_nm, 2),
            "walk_push_dur_s": round(self.walk_push_dur_s, 2),
            "fault": ("none" if not self.fault_mode else
                      f"{self.fault_mode}:j{list(self.fault_joints)}"
                      f"@{round(self.fault_scale, 2)}"),
        }


class DomainRandomizer:
    def __init__(self, ranges: RandRanges | None = None, *,
                 scale: float = 1.0):
        self.scale = float(scale)
        self.ranges = (ranges or RandRanges()).scaled(self.scale)

    @classmethod
    def from_params(cls, params: SimServoParams, *,
                    scale: float = 1.0) -> "DomainRandomizer":
        """Widen ranges with the measured joint-to-joint spread, if fitted."""
        r = RandRanges()
        spreads = [params.spread.get(ax, {}) for ax in AXES]
        rise = [s.get("rise_ms_pct") for s in spreads if s.get("rise_ms_pct")]
        delay = [s.get("delay_ms_pct") for s in spreads
                 if s.get("delay_ms_pct")]
        if rise:
            # Rise-time spread across joints ≈ effective kp/kv spread.
            r.kp_scale_pct = max(r.kp_scale_pct, 1.5 * max(rise))
            r.kv_scale_pct = max(r.kv_scale_pct, 1.5 * max(rise))
        if delay:
            hi = 1.0 + 2.0 * max(delay)
            r.latency_scale = (max(0.3, min(r.latency_scale[0], 2.0 - hi)),
                               max(r.latency_scale[1], hi))
        return cls(r, scale=scale)

    def sample(self, rng: np.random.Generator) -> EpisodeRandomization:
        r = self.ranges
        u = rng.uniform

        # Leg lengths: one global scale (systematic print/CAD error) times
        # independent per-leg per-segment spread (assembly tolerance).
        global_len = u(1.0 - r.link_len_scale_pct, 1.0 + r.link_len_scale_pct)
        link_scale = global_len * u(
            1.0 - r.link_len_leg_pct, 1.0 + r.link_len_leg_pct, (N_LEGS, 3))

        # Ground slope: random azimuth, tilt up to ground_tilt_deg.
        tilt = u(0.0, r.ground_tilt_deg) * DEG2RAD
        az = u(0.0, 2.0 * math.pi)
        gravity = G0 * np.array([
            math.sin(tilt) * math.cos(az),
            math.sin(tilt) * math.sin(az),
            -math.cos(tilt)])

        mnt = r.imu_mount_deg * DEG2RAD
        imu_mount_rot = _rot_rpy(u(-mnt, mnt), u(-mnt, mnt), u(-mnt, mnt))

        # Start pose: hand-placement slop on every joint, plus (sometimes)
        # a few joints that are WAY off — slipped zero / operator error,
        # the 2026-08-06 scenario. The env holds this pose at reset like
        # the hardware does; the policy must cope from a degraded stance.
        start_offset = u(-r.placement_noise_deg, r.placement_noise_deg,
                         N_JOINTS) * DEG2RAD
        bad_joints: list[int] = []
        if rng.random() < r.bad_start_prob:
            n_bad = int(rng.integers(1, r.bad_start_max_joints + 1))
            bad_joints = list(rng.choice(N_JOINTS, size=n_bad,
                                         replace=False))
            for j in bad_joints:
                mag = u(*r.bad_start_deg) * DEG2RAD
                start_offset[j] = mag * (1 if rng.random() < 0.5 else -1)

        # Tipped start: draws are GUARDED so configs with the axis off
        # (prob 0, incl. dr_scale=0) keep the legacy rng stream —
        # same convention as the walk park bank.
        tipped_roll = 0.0
        if r.tipped_start_prob > 0.0 and rng.random() < r.tipped_start_prob:
            tipped_roll = float(u(*r.tipped_start_deg))
            if rng.random() < 0.5:
                tipped_roll = -tipped_roll

        # Rise rock: same guarded-draw convention (axis off = legacy
        # rng stream, bit-exact).
        rise_rock = 0.0
        if r.rise_rock_prob > 0.0 and rng.random() < r.rise_rock_prob:
            rise_rock = float(u(*r.rise_rock_deg))
            if rng.random() < 0.5:
                rise_rock = -rise_rock

        # Walk takeoff kick: same guarded-draw convention.
        walk_kick, walk_kick_s = 0.0, 0.0
        if r.walk_kick_prob > 0.0 and rng.random() < r.walk_kick_prob:
            walk_kick = float(u(*r.walk_kick_deg))
            walk_kick_s = float(u(*r.walk_kick_s))
            if rng.random() < 0.5:
                walk_kick = -walk_kick

        # Walk takeoff push: same guarded-draw convention.
        walk_push, walk_push_s = 0.0, 0.0
        if r.walk_push_prob > 0.0 and rng.random() < r.walk_push_prob:
            walk_push = float(u(*r.walk_push_nm))
            walk_push_s = float(u(*r.walk_push_s))
            if rng.random() < 0.5:
                walk_push = -walk_push

        # Fault injection: same guarded-draw convention (fault_prob=0
        # keeps the legacy rng stream bit-exact).
        fault_mode, fault_joints, fault_scale = "", (), 1.0
        if r.fault_prob > 0.0 and rng.random() < r.fault_prob:
            mix = np.asarray(r.fault_mix, dtype=float)
            mix = mix / mix.sum()
            pick = rng.random()
            if pick < mix[0]:
                fault_mode = "weak"
                fault_joints = (int(rng.integers(N_JOINTS)),)
                fault_scale = float(
                    r.fault_weak_scales[
                        int(rng.integers(len(r.fault_weak_scales)))])
            elif pick < mix[0] + mix[1]:
                fault_mode = "frozen"
                fault_joints = (int(rng.integers(N_JOINTS)),)
            else:
                fault_mode = "leg"
                leg = int(rng.integers(N_LEGS))
                fault_joints = (3 * leg, 3 * leg + 1, 3 * leg + 2)
                fault_scale = 0.0

        # Mid-episode external push: same guarded-draw convention
        # (ext_push_prob=0 keeps the legacy rng stream bit-exact).
        ext_push, ext_push_dur, ext_push_start, ext_push_dir = (
            0.0, 0.0, 0.0, 0.0)
        if r.ext_push_prob > 0.0 and rng.random() < r.ext_push_prob:
            ext_push = float(u(*r.ext_push_n))
            ext_push_dur = float(u(*r.ext_push_dur_s))
            ext_push_start = float(u(*r.ext_push_start_s))
            ext_push_dir = float(u(0.0, 2.0 * math.pi))

        return EpisodeRandomization(
            mass_scale=u(*r.mass_scale),
            com_offset_m=np.array([
                u(-r.com_offset_m, r.com_offset_m),
                u(-r.com_offset_m, r.com_offset_m),
                0.0]),
            leg_mass_scale=u(1.0 - r.leg_mass_jitter_pct,
                             1.0 + r.leg_mass_jitter_pct, (N_LEGS, 3)),
            link_scale=link_scale,
            friction_scale=u(*r.friction_scale),
            contact_stiff_scale=u(*r.contact_stiff_scale),
            gravity_vec=gravity,
            kp_scale=u(1.0 - r.kp_scale_pct, 1.0 + r.kp_scale_pct, N_JOINTS),
            kv_scale=u(1.0 - r.kv_scale_pct, 1.0 + r.kv_scale_pct, N_JOINTS),
            torque_scale=u(*r.torque_scale),
            latency_scale=u(*r.latency_scale),
            deadband_scale=u(*r.deadband_scale),
            vel_scale=u(*r.vel_scale),
            cmd_drop_prob=u(0.0, r.cmd_drop_prob_max),
            start_offset_rad=start_offset,
            bad_start_joints=bad_joints,
            joint_zero_bias_rad=u(
                -r.joint_zero_bias_deg, r.joint_zero_bias_deg,
                N_JOINTS) * DEG2RAD,
            zero_drift_cmd_frame=bool(r.zero_drift_cmd_frame),
            encoder_noise_rad=r.encoder_noise_deg * DEG2RAD,
            imu_mount_rot=imu_mount_rot,
            imu_pos_m=np.array([
                u(-r.imu_pos_xy_m, r.imu_pos_xy_m),
                u(-r.imu_pos_xy_m, r.imu_pos_xy_m),
                u(*r.imu_pos_z_m)]),
            imu_bias_rad=u(-r.imu_bias_deg, r.imu_bias_deg, 2) * DEG2RAD,
            tilt_noise_rad=r.tilt_noise_deg * DEG2RAD,
            gyro_bias_rad_s=u(
                -r.gyro_bias_deg_s, r.gyro_bias_deg_s, 3) * DEG2RAD,
            gyro_noise_rad_s=r.gyro_noise_deg_s * DEG2RAD,
            action_noise=r.action_noise,
            tipped_roll_deg=tipped_roll,
            rise_rock_roll_deg=rise_rock,
            walk_kick_roll_deg=walk_kick,
            walk_kick_dur_s=walk_kick_s,
            walk_push_peak_nm=walk_push,
            walk_push_dur_s=walk_push_s,
            fault_mode=fault_mode,
            fault_joints=fault_joints,
            fault_scale=fault_scale,
            ext_push_peak_n=ext_push,
            ext_push_dur_s=ext_push_dur,
            ext_push_start_s=ext_push_start,
            ext_push_dir_rad=ext_push_dir,
        )
