"""SimHexapodJointGoalEnv — the goal task with RAW JOINT actions.

Same observations (modulo prev-action width), goals, rewards, safety
layer and servo model as ``SimHexapodGoalEnv``; the only change is the
action space: 18 channels in [-1, 1], one per joint, mapped to an
ABSOLUTE joint target across the full hardware axis range:

    q_target[j] = center(axis) + a[j] * half_range(axis)

with (lo, hi) from ``AXIS_LIMITS_DEG`` (yaw ±35°, hip −80..30°, knee
−20..150°). a = 0 therefore commands the mid-range pose, and every
reachable pose is expressible — including the flat zero pose and the
plant, which the body-IK action space can only reach through the curl
ratchet. The SafetyLayer's per-tick rate limit (max_delta_q_deg) and
axis clipping still apply downstream, exactly as they would on
hardware, so "raw" never means "unfiltered".

There is no IK and no foot anchoring here: the policy owns foot
placement. That makes the task harder to explore (18 dims vs 6) but
removes the structural ceiling the IK imposes on gait discovery.

A policy trained on the body-IK task cannot be loaded directly (both
obs and action widths change); use BC distillation to warm-start
(see ``distill_joint_policy.py``).
"""
from __future__ import annotations

import numpy as np

from rl_move.env import GOAL_DIM
from rl_move.robot_state import DEG2RAD, N_JOINTS
from rl_move.safety import AXIS_LIMITS_DEG

from .goal_task import SimHexapodGoalEnv
from .sim_env import N_OBS

try:
    import gymnasium as _gym
except Exception:  # pragma: no cover
    _gym = None

# Per-joint affine map [-1,1] -> radians, from the hardware axis limits.
_CENTER_RAD = np.array([
    (AXIS_LIMITS_DEG[j % 3][0] + AXIS_LIMITS_DEG[j % 3][1]) * 0.5 * DEG2RAD
    for j in range(N_JOINTS)])
_HALF_RAD = np.array([
    (AXIS_LIMITS_DEG[j % 3][1] - AXIS_LIMITS_DEG[j % 3][0]) * 0.5 * DEG2RAD
    for j in range(N_JOINTS)])


def action_to_q_rad(action: np.ndarray) -> np.ndarray:
    """Map a clipped [-1,1]^18 action to absolute joint targets (rad)."""
    return _CENTER_RAD + np.asarray(action, dtype=float) * _HALF_RAD


def q_rad_to_action(q_rad: np.ndarray) -> np.ndarray:
    """Inverse map (used by the BC distillation to label joint targets)."""
    return np.clip((np.asarray(q_rad, dtype=float) - _CENTER_RAD)
                   / _HALF_RAD, -1.0, 1.0)


class SimHexapodJointGoalEnv(SimHexapodGoalEnv):
    """Goal-conditioned twin with raw 18-joint actions (obs 59 + 9)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.n_act = N_JOINTS
        self._prev_action = np.zeros(self.n_act, dtype=float)
        if _gym is not None:
            self.action_space = _gym.spaces.Box(
                -1.0, 1.0, shape=(self.n_act,), dtype=np.float32)
            self.observation_space = _gym.spaces.Box(
                -np.inf, np.inf,
                shape=(N_OBS - 6 + self.n_act + GOAL_DIM,),
                dtype=np.float32)

    def _act_to_q(self, clipped: np.ndarray):
        return action_to_q_rad(clipped), True, ""
