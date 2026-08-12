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
    # Rise rocking (hardware 08-11, bench_blast camera sessions): the
    # real belly curl rocks 10°+ and trips tilt_roll 5/5 at the same
    # tick while the sim's curl stays <2° under BOTH actuator fits
    # (probed 08-11) — a systematic one-side droop the nominal sim
    # cannot produce. Axis: rise-mode episodes sometimes carry a
    # persistent one-side hip/knee fold bias on the PHYSICAL servo
    # command (sim_env._rise_rock_offset; same fold→roll mapping as
    # the tipped start). Encoders read the true (drooped) angles and
    # the tilt reference stays level, so the policy is paid to close
    # the command-vs-read loop and level out — the exact skill the
    # hardware curl demands. Default OFF (opt-in via dr.rise_rock_*).
    rise_rock_prob: float = 0.0
    rise_rock_deg: tuple[float, float] = (6.0, 15.0)     # target body roll

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
        )
