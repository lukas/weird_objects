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
# roll, pitch, height, x, y, curl
N_ACT = 6


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
    # 0..1: curl-in RATE. Positive values ratchet the foot anchors from
    # where they started toward the plant footprint (full travel in
    # ~2.5 s at curl=1); the anchors NEVER slide back out within an
    # episode. From a zero pose (legs straight out) raising the body
    # with pinned feet is geometrically impossible — the hip-foot
    # distance would exceed full leg extension — so standing up REQUIRES
    # dragging the feet inward first. Feet are nearly unloaded when the
    # belly rests on the ground, so the drag is physically cheap.
    #
    # Rate-with-ratchet instead of direct position (changed after run
    # 04): when the anchor tracked the INSTANTANEOUS curl value, a brief
    # exploratory curl pulse yanked the feet in for one tick and
    # snapped them back the next — pure jerk, punished by action-delta/
    # gyro penalties, zero lasting progress — so PPO learned to pin the
    # channel negative exactly on the starts that needed it (measured:
    # curl −0.5 on belly starts, +0.9 where curl is a no-op). With the
    # ratchet, every pulse leaves permanent progress that the curl-
    # progress reward pays immediately.
    curl: float = 0.0


def body_offset_from_action(action: np.ndarray, *,
                            max_roll: float, max_pitch: float,
                            max_h: float, max_x: float, max_y: float
                            ) -> BodyOffset:
    a = np.asarray(action, dtype=float).reshape(N_ACT)
    a = np.clip(a, -1.0, 1.0)
    return BodyOffset(
        roll=float(a[0]) * max_roll,
        pitch=float(a[1]) * max_pitch,
        height=float(a[2]) * max_h,
        x=float(a[3]) * max_x,
        y=float(a[4]) * max_y,
        # Only positive action curls: action 0 must stay "hold current
        # stance" (zero-action baselines, safety holds, PPO's initial
        # near-zero policy all rely on that convention).
        curl=max(float(a[5]), 0.0),
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
    """Freeze world feet at reset; map body offsets → joint targets.

    If a plant pose is supplied at reset, the ``curl`` channel of the
    offset RATCHETS the foot anchors (XY only — Z stays on the ground
    where each foot started) from their start positions toward the plant
    footprint: each solve() advances the internal curl fraction by
    ``CURL_RATE_PER_TICK * curl`` and it never decreases. curl=1 held
    for ~2.5 s reaches the footprint. When the robot already starts at
    the plant, the channel is close to inert.
    """

    # Fraction of the start→plant travel per solve() at curl=1.
    # solve() runs once per 40 ms control tick → full curl in ~2.5 s,
    # matching the rise task's 3 s hold phase.
    CURL_RATE_PER_TICK = 0.016

    def __init__(self):
        self.q_nominal = np.zeros(N_JOINTS, dtype=float)
        self.feet_world = np.zeros((N_LEGS, 3), dtype=float)
        self.feet_plant_xy: np.ndarray | None = None
        self.curl_frac = 0.0
        self._ready = False

    @property
    def ready(self) -> bool:
        return self._ready

    def reset(self, q_nominal_rad: np.ndarray,
              plant_q_rad: np.ndarray | None = None) -> None:
        self.q_nominal = np.asarray(q_nominal_rad, dtype=float).reshape(N_JOINTS).copy()
        # At nominal body (= identity offset), body frame == world frame.
        self.feet_world = fk_all_feet(self.q_nominal)
        if plant_q_rad is not None:
            plant_feet = fk_all_feet(
                np.asarray(plant_q_rad, dtype=float).reshape(N_JOINTS))
            self.feet_plant_xy = plant_feet[:, :2].copy()
        else:
            self.feet_plant_xy = None
        self.curl_frac = 0.0
        self._ready = True

    def _anchor(self, i: int) -> np.ndarray:
        p = self.feet_world[i].copy()
        if self.feet_plant_xy is not None and self.curl_frac > 0.0:
            c = self.curl_frac
            p[:2] = (1.0 - c) * p[:2] + c * self.feet_plant_xy[i]
        return p

    def solve(self, offset: BodyOffset) -> IKResult:
        if not self._ready:
            return IKResult(False, self.q_nominal.copy(), "ik_not_reset")
        # Ratchet: positive curl advances the anchors toward the plant
        # footprint; nothing ever slides them back out.
        if offset.curl > 0.0 and self.feet_plant_xy is not None:
            self.curl_frac = min(
                self.curl_frac
                + self.CURL_RATE_PER_TICK * min(offset.curl, 1.0), 1.0)
        az = leg_azimuths()
        q = np.zeros(N_JOINTS, dtype=float)
        for i in range(N_LEGS):
            # Anchors (frozen or curled) in world; express in the *moved*
            # body frame.
            p_body = _world_to_body(self._anchor(i), offset)
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
