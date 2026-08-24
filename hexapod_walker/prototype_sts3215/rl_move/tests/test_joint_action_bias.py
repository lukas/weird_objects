"""joint_task action-centering bias (2026-08-24, walkcurr rung-1 dig-in).

Root cause found while triaging `cw-walkcurr-pf-fwd6-hgt2-pdw05` (and,
retroactively, every one of the 20+ FAILed rung-1 arms before it): the
raw-joint action space's zero point (`a=0` -> `action_to_q_rad(0)` ->
the hardware AXIS mid-range, hip=-25deg/knee=65deg) is NOT anywhere
near the settled standing pose (`q_nom` measured at reset: hip~16deg,
knee~85deg, matching the semantics bank's `WALK_PLANT=(20,80)`).
Direct probe (no policy, no reward, no training): stepping
`SimHexapodJointWalkEnv` with a CONSTANT all-zero action sinks the
chassis -110mm over 2s while roll/pitch stay EXACTLY 0 the whole
run -- bit-for-bit the "belly_sit" signature every RND / height-gate /
park-duty rung-1 arm converged to. A freshly initialized (small-weight,
near-zero-mean) PPO policy reproduces this by construction, regardless
of reward mechanism -- these tests pin the physical defect and the
fix (a cfg-gated, default-off, bit-exact-when-off action bias) so no
future rung can rediscover it via a fresh multi-arm reward-engineering
sweep.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import (  # noqa: E402
    _CENTER_RAD, _HALF_RAD, action_to_q_rad)
from test_task_semantics import _make_walk_env, SLIPWALK_OVERRIDES  # noqa: E402

# The fix dose used by the launched arm: shift a=0 from the hardware
# mid-range toward the semantics bank's WALK_PLANT=(20, 80) stance
# (yaw unchanged -- already centered at 0).
FIX_BIAS_OVERRIDES = {
    ("goal", "joint_action_bias_hip_deg"): 45.0,   # -25 -> 20
    ("goal", "joint_action_bias_knee_deg"): 15.0,  # 65 -> 80
}


def _zero_action_height_drop_mm(overrides: dict, n_steps: int = 50) -> float:
    """Roll out `n_steps` of a constant all-zero action; return the
    chassis height drop (mm, positive = sank) at the end."""
    ov = dict(SLIPWALK_OVERRIDES)
    ov.update(overrides)
    env = _make_walk_env(0, ov)
    env.reset()
    zref = env._z0
    h_mm = 0.0
    roll_deg = pitch_deg = 0.0
    for _ in range(n_steps):
        _obs, _r, term, trunc, _info = env.step(np.zeros(env.n_act))
        h_mm = (float(env.data.xpos[env._chassis_bid, 2]) - zref) * 1000.0
        roll_deg = float(np.degrees(env._state.imu_roll))
        pitch_deg = float(np.degrees(env._state.imu_pitch))
        if term or trunc:
            break
    env.close()
    return -h_mm, roll_deg, pitch_deg


def test_default_bias_is_zero_and_inactive():
    """No cfg keys set -> bias vector all-zero and the fast-path flag
    off (bit-exact legacy: `_act_to_q` reduces to the original
    one-liner)."""
    env = _make_walk_env(0, SLIPWALK_OVERRIDES)
    assert not env._joint_action_bias_active
    assert np.allclose(env._joint_action_bias, 0.0)
    env.close()


def test_bias_zero_is_bit_exact():
    """With the bias cfg keys explicitly at 0.0, `_act_to_q` must
    produce the IDENTICAL q_rad as the un-biased mapping for a batch
    of random actions (legacy path, not just the zero action)."""
    ov = dict(SLIPWALK_OVERRIDES)
    ov.update({("goal", "joint_action_bias_hip_deg"): 0.0,
               ("goal", "joint_action_bias_knee_deg"): 0.0,
               ("goal", "joint_action_bias_yaw_deg"): 0.0})
    env = _make_walk_env(0, ov)
    rng = np.random.default_rng(0)
    for _ in range(20):
        a = rng.uniform(-1.0, 1.0, env.n_act)
        q_biased, ok, _ = env._act_to_q(a)
        assert ok
        assert np.allclose(q_biased, action_to_q_rad(a))
    env.close()


def test_bias_shifts_zero_action_target():
    """A nonzero bias moves the a=0 target by exactly the requested
    degrees on the biased axes, leaves the other axis untouched, and
    the mapping stays a pure translation (a=+1/-1 still lands on the
    biased-but-still-clipped mapping, never explodes past the
    original a=0 point by more than the bias)."""
    env = _make_walk_env(0, {**SLIPWALK_OVERRIDES, **FIX_BIAS_OVERRIDES})
    q0, ok, _ = env._act_to_q(np.zeros(env.n_act))
    assert ok
    q0_deg = np.degrees(q0).reshape(6, 3)
    # yaw (axis 0) untouched; hip (axis 1) +45; knee (axis 2) +15.
    assert np.allclose(q0_deg[:, 0], 0.0, atol=1e-6)
    assert np.allclose(q0_deg[:, 1], -25.0 + 45.0, atol=1e-6)
    assert np.allclose(q0_deg[:, 2], 65.0 + 15.0, atol=1e-6)
    env.close()


def test_bias_clips_into_axis_range():
    """An extreme bias combined with an extreme action must still land
    inside the hardware axis limits (clip, never raise/NaN)."""
    ov = dict(SLIPWALK_OVERRIDES)
    ov.update({("goal", "joint_action_bias_hip_deg"): 200.0,
               ("goal", "joint_action_bias_knee_deg"): -300.0})
    env = _make_walk_env(0, ov)
    q, ok, _ = env._act_to_q(np.ones(env.n_act))
    assert ok
    q_deg = np.degrees(q).reshape(6, 3)
    assert np.all(q_deg[:, 1] <= 30.0 + 1e-6)
    assert np.all(q_deg[:, 2] <= 150.0 + 1e-6)
    q2, ok2, _ = env._act_to_q(-np.ones(env.n_act))
    assert ok2
    assert np.all(np.degrees(q2).reshape(6, 3)[:, 1] >= -80.0 - 1e-6)
    env.close()


def test_zero_action_collapses_without_the_fix():
    """PIN THE DEFECT: constant all-zero action (no policy at all)
    sinks the chassis well past the height-gate's 25mm/60mm cutoffs
    within 2s (50 steps), staying level the whole time (roll/pitch
    ~0) -- the exact belly_sit signature every rung-1 RND/height-gate
    arm converged to independent of reward mechanism."""
    drop_mm, roll_deg, pitch_deg = _zero_action_height_drop_mm({})
    assert drop_mm > 90.0, (
        f"expected the known ~110mm zero-action collapse, got {drop_mm}mm "
        "-- defect may have regressed/moved, re-diagnose before trusting "
        "the fix test below")
    assert abs(roll_deg) < 2.0 and abs(pitch_deg) < 2.0, (
        "collapse should stay level (roll/pitch near 0) -- a tilted "
        "result is a different failure mode than the one this bias fixes")


def test_bias_fixes_zero_action_collapse():
    """THE FIX: with the bias re-centering a=0 on the honest plant
    stance, the same constant all-zero-action rollout must stay close
    to the reference height (a real stand, not a sink) over the same
    2s window."""
    drop_mm, roll_deg, pitch_deg = _zero_action_height_drop_mm(
        FIX_BIAS_OVERRIDES)
    assert drop_mm < 30.0, (
        f"bias did not fix the zero-action collapse: still sank "
        f"{drop_mm}mm")
    assert abs(roll_deg) < 5.0 and abs(pitch_deg) < 5.0
