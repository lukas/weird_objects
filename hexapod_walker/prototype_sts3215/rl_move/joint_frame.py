"""Joint-frame compatibility for MuJoCo policies and the real robot.

The physical robot's logical command/readback frame uses an
absolute-tibia knee: hip is the femur angle in the leg plane and knee is
the tibia angle in that same leg plane. The MuJoCo model's knee qpos is
the physical hinge angle relative to the femur.

Default policy frame is ``robot_abs`` because new policies should train
and deploy in the same logical frame the real robot exposes. Older
checkpoints trained directly against the MuJoCo hinge can opt into
``model_rel`` as an explicit compatibility mode.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

N_JOINTS = 18
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

FRAME_ROBOT_ABS = "robot_abs"
FRAME_MODEL_REL = "model_rel"

_ALIASES = {
    "": FRAME_ROBOT_ABS,
    "robot": FRAME_ROBOT_ABS,
    "robot_abs": FRAME_ROBOT_ABS,
    "robot_absolute": FRAME_ROBOT_ABS,
    "absolute": FRAME_ROBOT_ABS,
    "absolute_tibia": FRAME_ROBOT_ABS,
    "absolute_knee": FRAME_ROBOT_ABS,
    "hardware": FRAME_ROBOT_ABS,
    "model": FRAME_MODEL_REL,
    "model_rel": FRAME_MODEL_REL,
    "sim": FRAME_MODEL_REL,
    "sim_rel": FRAME_MODEL_REL,
    "mujoco": FRAME_MODEL_REL,
    "mujoco_rel": FRAME_MODEL_REL,
    "relative": FRAME_MODEL_REL,
    "relative_knee": FRAME_MODEL_REL,
    "legacy": FRAME_MODEL_REL,
    "legacy_mujoco": FRAME_MODEL_REL,
}

_PROTO = Path(__file__).resolve().parents[1]
for _path in (_PROTO / "motor_setup", _PROTO / "linux_control"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))


def normalize_joint_frame(frame: str | None) -> str:
    """Return the canonical policy-frame name.

    ``None`` means the new/correct default: robot logical absolute-tibia.
    Use ``model_rel`` only for older checkpoints trained in MuJoCo's
    native femur-relative knee frame.
    """
    key = (frame or "").strip().lower().replace("-", "_")
    if key not in _ALIASES:
        raise ValueError(
            f"unknown joint frame {frame!r}; expected "
            f"{FRAME_ROBOT_ABS!r} or {FRAME_MODEL_REL!r}")
    return _ALIASES[key]


def _as_joint_array(q: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    return np.asarray(q, dtype=float).reshape(N_JOINTS).copy()


def robot_abs_to_model_rel(q_robot_abs: np.ndarray | list[float]) -> list[float]:
    """Robot logical absolute-tibia -> MuJoCo femur-relative knee.

    Unit-agnostic: degrees in gives degrees out; radians in gives
    radians out.
    """
    q = _as_joint_array(q_robot_abs)
    for leg in range(6):
        hip_j = 3 * leg + 1
        knee_j = 3 * leg + 2
        q[knee_j] = q[knee_j] - q[hip_j]
    return [float(v) for v in q]


def model_rel_to_robot_abs(q_model_rel: np.ndarray | list[float]) -> list[float]:
    """MuJoCo femur-relative knee -> robot logical absolute-tibia."""
    q = _as_joint_array(q_model_rel)
    for leg in range(6):
        hip_j = 3 * leg + 1
        knee_j = 3 * leg + 2
        q[knee_j] = q[knee_j] + q[hip_j]
    return [float(v) for v in q]


def robot_abs_rad_to_model_rel_rad(q_robot_abs_rad: np.ndarray | list[float]) -> np.ndarray:
    return np.asarray(robot_abs_to_model_rel(q_robot_abs_rad), dtype=float)


def model_rel_rad_to_robot_abs_rad(q_model_rel_rad: np.ndarray | list[float]) -> np.ndarray:
    return np.asarray(model_rel_to_robot_abs(q_model_rel_rad), dtype=float)


def robot_abs_deg_to_model_rel_rad(q_robot_abs_deg: np.ndarray | list[float]) -> np.ndarray:
    return robot_abs_rad_to_model_rel_rad(
        np.asarray(q_robot_abs_deg, dtype=float) * DEG2RAD)


def model_rel_rad_to_robot_abs_deg(q_model_rel_rad: np.ndarray | list[float]) -> list[float]:
    return [float(v) for v in model_rel_rad_to_robot_abs_rad(q_model_rel_rad) * RAD2DEG]


def convert_joint_frame_rad(q_rad: np.ndarray | list[float], *,
                            src: str | None, dst: str | None) -> np.ndarray:
    """Convert one 18-joint vector between policy frames."""
    src_f = normalize_joint_frame(src)
    dst_f = normalize_joint_frame(dst)
    q = _as_joint_array(q_rad)
    if src_f == dst_f:
        return q
    if src_f == FRAME_ROBOT_ABS and dst_f == FRAME_MODEL_REL:
        return robot_abs_rad_to_model_rel_rad(q)
    if src_f == FRAME_MODEL_REL and dst_f == FRAME_ROBOT_ABS:
        return model_rel_rad_to_robot_abs_rad(q)
    raise AssertionError(f"unhandled joint-frame conversion {src_f}->{dst_f}")


def robot_abs_rad_to_policy_rad(q_robot_abs_rad: np.ndarray | list[float],
                                joint_frame: str | None) -> np.ndarray:
    return convert_joint_frame_rad(q_robot_abs_rad, src=FRAME_ROBOT_ABS,
                                   dst=joint_frame)


def policy_rad_to_robot_abs_rad(q_policy_rad: np.ndarray | list[float],
                                joint_frame: str | None) -> np.ndarray:
    return convert_joint_frame_rad(q_policy_rad, src=joint_frame,
                                   dst=FRAME_ROBOT_ABS)


def model_rel_rad_to_policy_rad(q_model_rel_rad: np.ndarray | list[float],
                                joint_frame: str | None) -> np.ndarray:
    return convert_joint_frame_rad(q_model_rel_rad, src=FRAME_MODEL_REL,
                                   dst=joint_frame)


def policy_rad_to_model_rel_rad(q_policy_rad: np.ndarray | list[float],
                                joint_frame: str | None) -> np.ndarray:
    return convert_joint_frame_rad(q_policy_rad, src=joint_frame,
                                   dst=FRAME_MODEL_REL)


def policy_joint_frame_from_meta(meta: dict[str, Any] | None,
                                 cfg: dict[str, Any] | None = None) -> str:
    """Resolve a policy's frame, defaulting to the real robot frame."""
    raw = None
    if meta:
        raw = meta.get("joint_frame") or meta.get("policy_joint_frame")
    if raw is None and cfg is not None:
        try:
            from rl_move.config import cfg_get
            raw = cfg_get(cfg, "compat", "policy_joint_frame", default=None)
        except Exception:
            raw = None
    return normalize_joint_frame(raw)


def robot_stand_degrees() -> list[float]:
    """Robot stand/plant pose in logical robot degrees."""
    try:
        from feetech_bus import standing_pose_degrees
        q = [float(v) for v in standing_pose_degrees()]
        if len(q) == N_JOINTS:
            return q
    except Exception:
        pass
    return [0.0, 19.0, 28.0] * 6


# Back-compatible names used by the first MuJoCo-web bridge.
robot_abs_rad_to_sim_rad = robot_abs_rad_to_model_rel_rad
robot_abs_deg_to_sim_rad = robot_abs_deg_to_model_rel_rad
sim_rad_to_robot_abs_rad = model_rel_rad_to_robot_abs_rad
sim_rad_to_robot_abs_deg = model_rel_rad_to_robot_abs_deg
