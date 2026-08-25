"""Tripod-step transition between two upright walk-ready poses.

Use this only when the current upright stance is far from the desired
walk-ready pose and a direct all-six-leg glide would scrape feet. The normal
RL start pose is the simulator's explicit walk-start pose, not the mutable
plant_pose.json calibration file.

This planner alternates tripods so each leg's large horizontal move happens
while that leg is lifted. While one tripod is in the air, the support tripod
can also bend vertically at a fixed foot radius; that lowers the body without
scraping the support feet and lets the swing tripod reach farther per stage.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from tripod_gait import FEMUR_MM, HIP_LIMIT_DEG, KNEE_LIMIT_DEG, TIBIA_MM

N_JOINTS = 18
TRIPOD_GROUPS = ((0, 2, 4), (1, 3, 5))
LIFT_HIP_DEG = 10.0
LIFT_KNEE_DEG = 14.0
LIFT_Z_MM = 24.0
MAX_STAGE_DELTA_DEG = 10.0
MIN_STAGES = 4
MAX_STAGES = 4


@dataclass(frozen=True)
class TransitionFrame:
    q_deg: list[float]
    seconds: float
    phase: str
    stage: int
    legs: tuple[int, ...]


def _as_pose(q_deg: list[float] | tuple[float, ...]) -> list[float]:
    if len(q_deg) != N_JOINTS:
        raise ValueError(f"pose must have {N_JOINTS} joints")
    return [float(v) for v in q_deg]


def _stage_count(start_deg: list[float], target_deg: list[float]) -> int:
    worst = max(abs(a - b) for a, b in zip(start_deg, target_deg))
    return max(MIN_STAGES, min(MAX_STAGES,
                               int(math.ceil(worst / MAX_STAGE_DELTA_DEG))))


def _lerp(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [x + (y - x) * alpha for x, y in zip(a, b)]


def _foot_rz_mm(hip_deg: float, knee_deg: float) -> tuple[float, float]:
    hip = math.radians(hip_deg)
    knee = math.radians(knee_deg)
    r = FEMUR_MM * math.cos(hip) + TIBIA_MM * math.cos(knee)
    z = -FEMUR_MM * math.sin(hip) - TIBIA_MM * math.sin(knee)
    return r, z


def _ik_hip_knee(r_mm: float, z_mm: float) -> tuple[float, float] | None:
    """Absolute-tibia hip/knee IK from the hip pivot in the leg plane."""
    u = float(r_mm)
    w = -float(z_mm)
    reach = math.hypot(u, w)
    if reach < 1e-6 or reach > FEMUR_MM + TIBIA_MM - 1e-6:
        return None
    if reach < abs(FEMUR_MM - TIBIA_MM) + 1e-6:
        return None
    gamma = math.atan2(w, u)
    ca = (reach * reach + FEMUR_MM ** 2 - TIBIA_MM ** 2) / (
        2.0 * reach * FEMUR_MM)
    ca = max(-1.0, min(1.0, ca))
    alpha = math.acos(ca)
    candidates = []
    for hip_r in (gamma - alpha, gamma + alpha):
        knee_r = math.atan2(w - FEMUR_MM * math.sin(hip_r),
                            u - FEMUR_MM * math.cos(hip_r))
        hip = math.degrees(hip_r)
        knee = math.degrees(knee_r)
        if (HIP_LIMIT_DEG[0] <= hip <= HIP_LIMIT_DEG[1]
                and KNEE_LIMIT_DEG[0] <= knee <= KNEE_LIMIT_DEG[1]):
            score = abs(knee - hip)
            if knee < hip:
                score += 100.0
            candidates.append((score, hip, knee))
    if not candidates:
        return None
    _score, hip, knee = min(candidates, key=lambda row: row[0])
    return float(hip), float(knee)


def _lifted_hip_knee(hip_deg: float, knee_deg: float) -> tuple[float, float]:
    r, z = _foot_rz_mm(hip_deg, knee_deg)
    hk = _ik_hip_knee(r, z + LIFT_Z_MM)
    if hk is not None:
        return hk
    return hip_deg - LIFT_HIP_DEG, knee_deg - LIFT_KNEE_DEG


def _lift_pose(current: list[float], legs: tuple[int, ...]) -> list[float]:
    q = list(current)
    for leg in legs:
        base = 3 * leg
        q[base + 1], q[base + 2] = _lifted_hip_knee(
            current[base + 1], current[base + 2])
    return q


def _swing_pose(current: list[float], target: list[float],
                legs: tuple[int, ...]) -> list[float]:
    q = list(current)
    for leg in legs:
        base = 3 * leg
        hip, knee = _lifted_hip_knee(target[base + 1], target[base + 2])
        q[base + 0] = target[base + 0]
        q[base + 1] = hip
        q[base + 2] = knee
    return q


def _support_compress_pose(current: list[float], target: list[float],
                           legs: tuple[int, ...]) -> list[float]:
    """Bend loaded support legs vertically without moving foot radius."""
    q = list(current)
    for leg in legs:
        base = 3 * leg
        r_now, _z_now = _foot_rz_mm(current[base + 1],
                                    current[base + 2])
        _r_target, z_target = _foot_rz_mm(target[base + 1],
                                          target[base + 2])
        hk = _ik_hip_knee(r_now, z_target)
        if hk is not None:
            q[base + 1], q[base + 2] = hk
    return q


def _place_pose(current: list[float], target: list[float],
                legs: tuple[int, ...]) -> list[float]:
    q = list(current)
    for leg in legs:
        base = 3 * leg
        q[base:base + 3] = target[base:base + 3]
    return q


def build_tripod_plant_transition(
    start_deg: list[float] | tuple[float, ...],
    target_deg: list[float] | tuple[float, ...],
    *,
    lift_s: float = 0.45,
    support_s: float = 0.75,
    swing_s: float = 0.60,
    place_s: float = 0.60,
    settle_s: float = 1.00,
) -> list[TransitionFrame]:
    """Return tripod lift/place frames from ``start_deg`` to ``target_deg``.

    The planner advances toward the plant in several small stages. Within
    each stage, tripod A lifts in place, the opposite/support tripod bends
    vertically at a fixed foot radius, tripod A swings while unloaded, and
    then places onto the new partial target; tripod B then does the same.
    The final frame is just a re-hold of the target.
    """
    start = _as_pose(start_deg)
    target = _as_pose(target_deg)
    stages = _stage_count(start, target)
    frames: list[TransitionFrame] = []
    current = list(start)
    for stage in range(1, stages + 1):
        partial = _lerp(start, target, stage / stages)
        for legs in TRIPOD_GROUPS:
            support = tuple(leg for leg in range(6) if leg not in legs)
            lift = _lift_pose(current, legs)
            frames.append(TransitionFrame(
                q_deg=lift, seconds=lift_s, phase="lift",
                stage=stage, legs=legs))
            compressed = _support_compress_pose(lift, partial, support)
            frames.append(TransitionFrame(
                q_deg=compressed, seconds=support_s, phase="support",
                stage=stage, legs=support))
            swing = _swing_pose(compressed, partial, legs)
            frames.append(TransitionFrame(
                q_deg=swing, seconds=swing_s, phase="swing",
                stage=stage, legs=legs))
            placed = _place_pose(swing, partial, legs)
            frames.append(TransitionFrame(
                q_deg=placed, seconds=place_s, phase="place",
                stage=stage, legs=legs))
            current = placed
    frames.append(TransitionFrame(
        q_deg=list(target), seconds=settle_s, phase="settle",
        stage=stages, legs=()))
    return frames
