from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from rl_move.joint_frame import (
    FRAME_MODEL_REL,
    FRAME_ROBOT_ABS,
    model_rel_rad_to_policy_rad,
    model_rel_rad_to_robot_abs_rad,
    normalize_joint_frame,
    policy_joint_frame_from_meta,
    policy_rad_to_model_rel_rad,
    robot_abs_rad_to_model_rel_rad,
    robot_abs_rad_to_policy_rad,
)
from rl_move.robot_state import RobotState

ROOT = Path(__file__).resolve().parents[2]
LINUX = ROOT / "linux_control"
if str(LINUX) not in sys.path:
    sys.path.insert(0, str(LINUX))

import rl_policy  # noqa: E402


def _pose_rad() -> np.ndarray:
    q = np.zeros(18)
    for leg in range(6):
        q[3 * leg + 0] = 0.01 * leg
        q[3 * leg + 1] = 0.2 + 0.01 * leg
        q[3 * leg + 2] = 0.55 + 0.02 * leg
    return q


def test_robot_abs_model_rel_roundtrip():
    q_abs = _pose_rad()
    q_rel = robot_abs_rad_to_model_rel_rad(q_abs)
    for leg in range(6):
        hip = 3 * leg + 1
        knee = 3 * leg + 2
        assert q_rel[knee] == q_abs[knee] - q_abs[hip]
    np.testing.assert_allclose(model_rel_rad_to_robot_abs_rad(q_rel),
                               q_abs)


def test_policy_frame_defaults_to_robot_abs_and_accepts_legacy_aliases():
    assert policy_joint_frame_from_meta({}) == FRAME_ROBOT_ABS
    assert policy_joint_frame_from_meta(
        {}, {"compat": {"policy_joint_frame": "model_rel"}}
    ) == FRAME_MODEL_REL
    assert normalize_joint_frame("legacy_mujoco") == FRAME_MODEL_REL
    assert normalize_joint_frame("absolute_tibia") == FRAME_ROBOT_ABS


def test_policy_conversion_helpers_match_declared_frame():
    q_abs = _pose_rad()
    q_rel = robot_abs_rad_to_model_rel_rad(q_abs)
    np.testing.assert_allclose(
        robot_abs_rad_to_policy_rad(q_abs, FRAME_ROBOT_ABS), q_abs)
    np.testing.assert_allclose(
        robot_abs_rad_to_policy_rad(q_abs, FRAME_MODEL_REL), q_rel)
    np.testing.assert_allclose(
        policy_rad_to_model_rel_rad(q_abs, FRAME_ROBOT_ABS), q_rel)
    np.testing.assert_allclose(
        model_rel_rad_to_policy_rad(q_rel, FRAME_ROBOT_ABS), q_abs)


def test_robot_runner_state_view_uses_policy_frame():
    q_abs = _pose_rad()
    state = RobotState(
        timestamp=1.0,
        joint_position=q_abs,
        joint_velocity=q_abs * 0.1,
        imu_roll=0.0,
        imu_pitch=0.0,
        imu_yaw=0.0,
        imu_gyro=np.zeros(3),
        imu_accel=np.zeros(3),
        commanded_position=q_abs + 0.01,
    )
    view = rl_policy._state_for_policy_frame(state, FRAME_MODEL_REL)
    np.testing.assert_allclose(
        view.joint_position, robot_abs_rad_to_model_rel_rad(q_abs))
    np.testing.assert_allclose(
        view.joint_velocity, robot_abs_rad_to_model_rel_rad(q_abs * 0.1))
    np.testing.assert_allclose(
        view.commanded_position,
        robot_abs_rad_to_model_rel_rad(q_abs + 0.01))
    np.testing.assert_allclose(state.joint_position, q_abs)
