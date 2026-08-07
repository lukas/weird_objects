"""Fixed-foot body IK: body pose offsets → 18 joint targets (radians).

Reuses ``tripod_gait._leg_ik`` and plant geometry. Yaw joints participate
in planar XY body shifts.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_LINUX = Path(__file__).resolve().parents[1] / "linux_control"
if str(_LINUX) not in sys.path:
    sys.path.insert(0, str(_LINUX))

from tripod_gait import (  # noqa: E402
    COXA, FEMUR, TIBIA, LEG_RADIAL, _leg_ik,
)

N_JOINTS = 18
N_LEGS = 6


def leg_azimuths() -> list[float]:
    return [(i + 0.5) * math.pi / 3.0 for i in range(N_LEGS)]


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


@dataclass
class BodyOffset:
    roll: float = 0.0    # rad
    pitch: float = 0.0   # rad
    height: float = 0.0  # m (body +Z)
    x: float = 0.0       # m
    y: float = 0.0       # m


def body_offset_from_action(action: np.ndarray, *,
                            max_roll: float, max_pitch: float,
                            max_h: float, max_x: float, max_y: float
                            ) -> BodyOffset:
    a = np.asarray(action, dtype=float).reshape(5)
    a = np.clip(a, -1.0, 1.0)
    return BodyOffset(
        roll=float(a[0]) * max_roll,
        pitch=float(a[1]) * max_pitch,
        height=float(a[2]) * max_h,
        x=float(a[3]) * max_x,
        y=float(a[4]) * max_y,
    )


def fk_foot_body(yaw: float, hip: float, knee: float, leg_az: float
                 ) -> np.ndarray:
    """Foot position in body frame (metres) for one leg."""
    # Foot in yaw-frame (x out along coxa, z up-ish with plant convention).
    pt = hip + knee
    x_yaw = COXA + FEMUR * math.cos(hip) + TIBIA * math.cos(pt)
    z_yaw = -FEMUR * math.sin(hip) - TIBIA * math.sin(pt)
    # Rotate yaw joint about body Z at the yaw origin, then place origin.
    c, s = math.cos(leg_az + yaw), math.sin(leg_az + yaw)
    # Point in horizontal plane relative to yaw origin, along (leg_az+yaw).
    # Standard gait: yaw_angle rotates the planar foot about yaw axis.
    c0, s0 = math.cos(leg_az), math.sin(leg_az)
    # Yaw-frame x along leg_az when yaw=0; apply yaw about vertical at origin.
    c_y, s_y = math.cos(yaw), math.sin(yaw)
    x_local = c_y * x_yaw   # y_yaw assumed 0 at IK solution
    y_local = s_y * x_yaw
    # Rotate local XY into body by leg_az.
    bx = c0 * x_local - s0 * y_local + LEG_RADIAL * c0
    by = s0 * x_local + c0 * y_local + LEG_RADIAL * s0
    bz = z_yaw
    return np.array([bx, by, bz], dtype=float)


def fk_all_feet(q_rad: np.ndarray) -> np.ndarray:
    """Return (6,3) foot positions in body frame."""
    q = np.asarray(q_rad, dtype=float).reshape(N_JOINTS)
    az = leg_azimuths()
    feet = np.zeros((N_LEGS, 3), dtype=float)
    for i in range(N_LEGS):
        yaw, hip, knee = q[3 * i], q[3 * i + 1], q[3 * i + 2]
        feet[i] = fk_foot_body(yaw, hip, knee, az[i])
    return feet


def _body_to_world(p_body: np.ndarray, offset: BodyOffset) -> np.ndarray:
    """Transform point from body frame to world (nominal-aligned) frame."""
    R = rot_z(0.0) @ rot_y(offset.pitch) @ rot_x(offset.roll)
    t = np.array([offset.x, offset.y, offset.height], dtype=float)
    return R @ p_body + t


def _world_to_body(p_world: np.ndarray, offset: BodyOffset) -> np.ndarray:
    R = rot_z(0.0) @ rot_y(offset.pitch) @ rot_x(offset.roll)
    t = np.array([offset.x, offset.y, offset.height], dtype=float)
    return R.T @ (p_world - t)


def ik_leg_from_foot_body(foot_body: np.ndarray, leg_az: float
                          ) -> tuple[float, float, float] | None:
    """Body-frame foot → (yaw, hip, knee) rad, or None if unreachable."""
    c, s = math.cos(leg_az), math.sin(leg_az)
    # Vector from yaw origin to foot in body XY.
    ox, oy = LEG_RADIAL * c, LEG_RADIAL * s
    rx, ry = float(foot_body[0]) - ox, float(foot_body[1]) - oy
    # Into yaw frame aligned with leg_az.
    x_yaw = c * rx + s * ry
    y_yaw = -s * rx + c * ry
    yaw = math.atan2(y_yaw, x_yaw)
    r_planar = math.hypot(x_yaw, y_yaw)
    z = float(foot_body[2])
    ik = _leg_ik((r_planar, 0.0, z))
    if ik is None:
        return None
    hip, knee = ik
    return float(yaw), float(hip), float(knee)


@dataclass
class IKResult:
    ok: bool
    q_rad: np.ndarray
    reason: str = ""


class FixedFootBodyIK:
    """Freeze world feet at reset; map body offsets → joint targets."""

    def __init__(self):
        self.q_nominal = np.zeros(N_JOINTS, dtype=float)
        self.feet_world = np.zeros((N_LEGS, 3), dtype=float)
        self._ready = False

    @property
    def ready(self) -> bool:
        return self._ready

    def reset(self, q_nominal_rad: np.ndarray) -> None:
        self.q_nominal = np.asarray(q_nominal_rad, dtype=float).reshape(N_JOINTS).copy()
        # At nominal body (= identity offset), body frame == world frame.
        self.feet_world = fk_all_feet(self.q_nominal)
        self._ready = True

    def solve(self, offset: BodyOffset) -> IKResult:
        if not self._ready:
            return IKResult(False, self.q_nominal.copy(), "ik_not_reset")
        az = leg_azimuths()
        q = np.zeros(N_JOINTS, dtype=float)
        for i in range(N_LEGS):
            # Feet frozen in world; express in the *moved* body frame.
            p_body = _world_to_body(self.feet_world[i], offset)
            sol = ik_leg_from_foot_body(p_body, az[i])
            if sol is None:
                return IKResult(
                    False, self.q_nominal.copy(), f"ik_fail_leg_{i}")
            q[3 * i:3 * i + 3] = sol
        return IKResult(True, q, "")


def foot_world_error(q_rad: np.ndarray, feet_world_ref: np.ndarray,
                     offset: BodyOffset) -> float:
    """Max foot position error (m) after FK under body offset.

    Interprets ``q`` as joints for the offset body; maps feet to world and
    compares to the frozen reference.
    """
    feet_body = fk_all_feet(q_rad)
    err = 0.0
    for i in range(N_LEGS):
        pw = _body_to_world(feet_body[i], offset)
        err = max(err, float(np.linalg.norm(pw - feet_world_ref[i])))
    return err
