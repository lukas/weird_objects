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
    encoder_noise_rad: float
    imu_mount_rot: np.ndarray            # (3, 3) chassis → IMU frame
    imu_pos_m: np.ndarray                # (3,) IMU offset from chassis origin
    imu_bias_rad: np.ndarray             # (2,) roll, pitch
    tilt_noise_rad: float
    gyro_bias_rad_s: np.ndarray          # (3,)
    gyro_noise_rad_s: float
    action_noise: float

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
        # = L{i}_tibia body x, tibia length = foot geom/site x.
        for i in range(N_LEGS):
            b_yaw = bid(f"L{i}_yaw")
            b_fem = bid(f"L{i}_femur")
            b_tib = bid(f"L{i}_tibia")
            g_foot = gid(f"L{i}_foot")
            s_foot = sid(f"L{i}_foot_site")
            s_coxa, s_femur, s_tibia = self.link_scale[i]

            model.body_pos[b_fem, 0] *= s_coxa
            model.body_pos[b_tib, 0] *= s_femur
            model.geom_pos[g_foot, 0] *= s_tibia
            model.site_pos[s_foot, 0] *= s_tibia
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
            "bad_start_joints": [int(j) for j in self.bad_start_joints],
            "start_offset_max_deg": round(
                float(np.max(np.abs(self.start_offset_rad))) / DEG2RAD, 1),
        }


class DomainRandomizer:
    def __init__(self, ranges: RandRanges | None = None):
        self.ranges = ranges or RandRanges()

    @classmethod
    def from_params(cls, params: SimServoParams) -> "DomainRandomizer":
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
        return cls(r)

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
        )
