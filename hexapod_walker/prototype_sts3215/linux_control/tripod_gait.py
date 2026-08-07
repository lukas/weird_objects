"""Open-loop tripod gait for the STS3215 hexapod (no MuJoCo / numpy).

Port of ``mujoco_prototype.TripodGait`` + ``_leg_ik`` with pure stdlib math
so it can run on the Uno Q with only the Feetech bus SDK vendored.
"""
from __future__ import annotations

import math

# Geometry (mm) — keep in sync with hexapod_prototype / mujoco_prototype.
COXA_MM = 12.5
FEMUR_MM = 90.0
TIBIA_MM = 128.0
CHASSIS_FLAT_TO_FLAT_MM = 200.0  # matches hexapod_prototype.CHASSIS_FLAT_TO_FLAT
# Legacy gait crouch (used only if plant pose can't be loaded).
STANCE_FEMUR_DEG = -25.0
STANCE_TIBIA_DEG = 60.0

M = 0.001
COXA = COXA_MM * M
FEMUR = FEMUR_MM * M
TIBIA = TIBIA_MM * M
STANCE_FEMUR = math.radians(STANCE_FEMUR_DEG)
STANCE_TIBIA = math.radians(STANCE_TIBIA_DEG)
LEG_RADIAL = (CHASSIS_FLAT_TO_FLAT_MM / 2.0) * M


def _plant_hip_knee_deg() -> tuple[float, float]:
    """Stand-plant hip/knee (same source as ``standing_pose_degrees``)."""
    try:
        from feetech_bus import (DEFAULT_STAND_HIP_DEG, DEFAULT_STAND_KNEE_DEG,
                                 load_plant_pose)
        plant = load_plant_pose()
        return (float(plant.get("hip_deg", DEFAULT_STAND_HIP_DEG)),
                float(plant.get("knee_deg", DEFAULT_STAND_KNEE_DEG)))
    except Exception:
        return STANCE_FEMUR_DEG, STANCE_TIBIA_DEG


def _leg_ik(target_xyz_in_yaw_frame):
    u = float(target_xyz_in_yaw_frame[0]) - COXA
    w = -float(target_xyz_in_yaw_frame[2])
    L = math.hypot(u, w)
    if L > FEMUR + TIBIA - 1e-6 or L < abs(FEMUR - TIBIA) + 1e-6:
        return None
    cos_pt = (L * L - FEMUR * FEMUR - TIBIA * TIBIA) / (2 * FEMUR * TIBIA)
    cos_pt = max(-1.0, min(1.0, cos_pt))
    pt = math.acos(cos_pt)
    p = math.atan2(w, u) - math.atan2(
        TIBIA * math.sin(pt), FEMUR + TIBIA * math.cos(pt))
    return p, pt


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class TripodGait:
    MAX_VX = 0.28
    MAX_VY = 0.20
    MAX_OMEGA = 1.0
    SCALE_PERIOD_MIN = 0.40
    SCALE_PERIOD_MAX = 2.00
    SCALE_LIFT_MIN = 0.30
    SCALE_LIFT_MAX = 2.50
    SCALE_STRIDE_MIN = 0.30
    SCALE_STRIDE_MAX = 1.80
    STANCE_RADIUS_SCALE_MIN = 0.55
    STANCE_RADIUS_SCALE_MAX = 1.05

    def __init__(
        self,
        *,
        period: float = 0.75,
        lift: float = 0.025,
        ramp: float = 0.35,
        vx: float = 0.0,
        vy: float = 0.0,
        omega: float = 0.0,
        period_scale: float = 1.0,
        lift_scale: float = 1.0,
        stride_scale: float = 1.0,
        stance_radius_scale: float = 1.0,
    ):
        self.period = period
        self.lift = lift
        self.ramp = max(ramp, 1e-3)
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.period_scale = period_scale
        self.lift_scale = [_clip(float(lift_scale), self.SCALE_LIFT_MIN,
                                 self.SCALE_LIFT_MAX)] * 6
        self.stride_scale = stride_scale
        self.leg_angles = [(i + 0.5) * math.pi / 3.0 for i in range(6)]
        self.stance_radius_scale = _clip(
            stance_radius_scale,
            self.STANCE_RADIUS_SCALE_MIN,
            self.STANCE_RADIUS_SCALE_MAX,
        )
        self._phase_offset = math.pi / 2.0
        self._vx_smooth = vx
        self._vy_smooth = vy
        self._om_smooth = omega
        self._last_t = None
        self._phase = 0.0
        self._elapsed = 0.0
        # Foot plant matches stand home (+20°/+80° or learned plant), not the
        # old crouch −25°/+60° — otherwise walk yanks hips up off the bench.
        self.sync_plant_stance()

    def sync_plant_stance(self, hip_deg: float | None = None,
                          knee_deg: float | None = None) -> None:
        """Set IK foot height / radius from stand-plant hip & knee degrees."""
        if hip_deg is None or knee_deg is None:
            h, k = _plant_hip_knee_deg()
            hip_deg = h if hip_deg is None else hip_deg
            knee_deg = k if knee_deg is None else knee_deg
        self.plant_hip_deg = float(hip_deg)
        self.plant_knee_deg = float(knee_deg)
        p = math.radians(self.plant_hip_deg)
        pt = math.radians(self.plant_hip_deg + self.plant_knee_deg)
        self.foot_neutral_x = COXA + FEMUR * math.cos(p) + TIBIA * math.cos(pt)
        self.foot_neutral_z = -FEMUR * math.sin(p) - TIBIA * math.sin(pt)
        self._foot_radius = LEG_RADIAL + self.foot_neutral_x
        self._foot_radius_eff = self._foot_radius * self.stance_radius_scale
        self._fallback = (0.0, p, math.radians(self.plant_knee_deg))

    def set_velocity(self, *, vx=None, vy=None, omega=None):
        if vx is not None:
            self.vx = _clip(float(vx), -self.MAX_VX, self.MAX_VX)
        if vy is not None:
            self.vy = _clip(float(vy), -self.MAX_VY, self.MAX_VY)
        if omega is not None:
            self.omega = _clip(float(omega), -self.MAX_OMEGA, self.MAX_OMEGA)

    def set_lift_mm(self, lift_mm: float) -> None:
        self.lift = max(0.004, min(0.050, float(lift_mm) * 0.001))

    def set_scales(self, *, period_scale=None, lift_scale=None, stride_scale=None):
        if period_scale is not None:
            self.period_scale = _clip(
                float(period_scale), self.SCALE_PERIOD_MIN, self.SCALE_PERIOD_MAX)
        if lift_scale is not None:
            s = _clip(float(lift_scale), self.SCALE_LIFT_MIN, self.SCALE_LIFT_MAX)
            self.lift_scale = [s] * 6
        if stride_scale is not None:
            self.stride_scale = _clip(
                float(stride_scale), self.SCALE_STRIDE_MIN, self.SCALE_STRIDE_MAX)

    def stop(self):
        self.set_velocity(vx=0.0, vy=0.0, omega=0.0)

    def reset_phase(self, *, phase: float = 0.0, t: float = 0.0):
        self._phase = phase % (2 * math.pi)
        self._elapsed = 0.0
        self._last_t = t
        self._vx_smooth = self.vx
        self._vy_smooth = self.vy
        self._om_smooth = self.omega

    def neutral_pose_deg(self) -> list[float]:
        """Standing plant — same as ``standing_pose_degrees()`` (learned/default)."""
        try:
            from feetech_bus import standing_pose_degrees
            return standing_pose_degrees()
        except Exception:
            out: list[float] = []
            for _ in range(6):
                out.extend([0.0, self.plant_hip_deg, self.plant_knee_deg])
            return out

    def _advance(self, t: float) -> float:
        if self._last_t is None:
            self._last_t = t
            return 0.0
        dt = max(0.0, t - self._last_t)
        self._last_t = t
        self._elapsed += dt
        self._phase = (
            self._phase
            + 2 * math.pi * dt / max(self.period * self.period_scale, 0.05)
        ) % (2 * math.pi)
        return dt

    def _smoothed_command(self, dt: float):
        if dt <= 0.0:
            return self._vx_smooth, self._vy_smooth, self._om_smooth
        tau = 0.15
        a = 1.0 - math.exp(-dt / tau)
        self._vx_smooth += a * (self.vx - self._vx_smooth)
        self._vy_smooth += a * (self.vy - self._vy_smooth)
        self._om_smooth += a * (self.omega - self._om_smooth)
        return self._vx_smooth, self._vy_smooth, self._om_smooth

    def _foot_target_in_body(self, i: int, vx, vy, omega):
        t_eff = max(self.period * self.period_scale, 0.05)
        ramp_amp = min(self._elapsed / self.ramp, 1.0)
        tripod = 0 if i % 2 == 0 else 1
        phi = (self._phase + self._phase_offset + tripod * math.pi) % (2 * math.pi)
        if phi < math.pi:
            s = phi / math.pi
            prog = -0.5 + s
            dz = self.lift * self.lift_scale[i] * ramp_amp * math.sin(math.pi * s)
        else:
            s = (phi - math.pi) / math.pi
            prog = 0.5 - s
            dz = 0.0
        a_i = self.leg_angles[i]
        sa, ca = math.sin(a_i), math.cos(a_i)
        v_x_at = vx - omega * self._foot_radius_eff * sa
        v_y_at = vy + omega * self._foot_radius_eff * ca
        dx = prog * v_x_at * t_eff / 2.0 * ramp_amp * self.stride_scale
        dy = prog * v_y_at * t_eff / 2.0 * ramp_amp * self.stride_scale
        return dx, dy, dz

    def desired_deg(self, t: float) -> list[float]:
        """18 joint angles in degrees for time ``t`` (seconds)."""
        dt = self._advance(t)
        vx, vy, omega = self._smoothed_command(dt)
        out: list[float] = []
        for i, a in enumerate(self.leg_angles):
            dx_b, dy_b, dz_b = self._foot_target_in_body(i, vx, vy, omega)
            fx_b = self._foot_radius_eff * math.cos(a) + dx_b
            fy_b = self._foot_radius_eff * math.sin(a) + dy_b
            yaw_origin_x = LEG_RADIAL * math.cos(a)
            yaw_origin_y = LEG_RADIAL * math.sin(a)
            rx = fx_b - yaw_origin_x
            ry = fy_b - yaw_origin_y
            ca, sa = math.cos(a), math.sin(a)
            x_yaw = ca * rx + sa * ry
            y_yaw = -sa * rx + ca * ry
            yaw_angle = math.atan2(y_yaw, x_yaw)
            r_planar = math.hypot(x_yaw, y_yaw)
            ik = _leg_ik((r_planar, 0.0, self.foot_neutral_z + dz_b))
            if ik is None:
                yaw, pitch, knee = self._fallback
            else:
                pitch, knee = ik
                yaw = yaw_angle
            out.extend([math.degrees(yaw), math.degrees(pitch),
                        math.degrees(knee)])
        return out
