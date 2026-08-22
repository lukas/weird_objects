"""Backward-compatible imports for the shared joint-frame bridge."""
from __future__ import annotations

from rl_move.joint_frame import (  # noqa: F401
    DEG2RAD,
    FRAME_MODEL_REL,
    FRAME_ROBOT_ABS,
    N_JOINTS,
    RAD2DEG,
    convert_joint_frame_rad,
    model_rel_rad_to_policy_rad,
    model_rel_rad_to_robot_abs_deg as sim_rad_to_robot_abs_deg,
    model_rel_rad_to_robot_abs_rad as sim_rad_to_robot_abs_rad,
    model_rel_to_robot_abs,
    normalize_joint_frame,
    policy_joint_frame_from_meta,
    policy_rad_to_model_rel_rad,
    policy_rad_to_robot_abs_rad,
    robot_abs_deg_to_model_rel_rad as robot_abs_deg_to_sim_rad,
    robot_abs_rad_to_model_rel_rad as robot_abs_rad_to_sim_rad,
    robot_abs_rad_to_policy_rad,
    robot_abs_to_model_rel,
    robot_stand_degrees,
)
