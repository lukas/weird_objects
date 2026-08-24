"""Unit tests for `goal.walk_stop_freeze_s`
(2026-08-24, joyfullcurr10-stopsettle-probe dig-in / structural
stop-hold lever).

The stopsettle-probe diagnostic measured the V6 b1 cert's residual
stop-tick creep as a genuine POST-grace floor -- excluding the exact
0.4s `reward.walk_stop_grace_s` transient a checkpoint was trained
under barely moved the metric (stop_speed_settled_m_s 0.03107 vs raw
stop_speed_m_s 0.03264, ~5%) -- which refutes the entire stop-speed/
stop-current REWARD-PRICING lever both by dose (joyfullcurr9/10) and
by measurement methodology, per that run's own pre-registered gate
text. `goal.walk_stop_freeze_s` is the named next lever: a STRUCTURAL
override (`sim_env._walk_stop_freeze_override`) that discards the
policy's own action and re-issues the previous tick's own safe
command once a walk/quadwalk stop segment has been commanded for more
than the threshold, exactly like a real "stop and hold" supervisory
mode -- not a price, a hold.

These tests call `_walk_stop_freeze_override` directly against a
scripted, monkeypatched `_current_goal()` rather than driving a full
scripted-gait rollout: the hook's own contract (freeze/exempt/reset)
is deterministic and cheap to pin exactly; a real gait's per-tick
joint deltas can legitimately go quiet on their own (e.g. mid-stance)
which would make a rollout-level test flaky/ambiguous about WHY the
command stopped changing.

Fast (~seconds): one env instance, no PPO, no physics rollout.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import N_JOINTS  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

FC_GOAL = {
    ("goal", "walk_yaw_cmd"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.06,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_heading_max_rad"): 0.0,
    ("goal", "walk_stop_frac"): 0.0,
}


class _FakeGoal:
    def __init__(self, vx_ref=0.0, vy_ref=0.0, wz_ref=0.0):
        self.vx_ref = vx_ref
        self.vy_ref = vy_ref
        self.wz_ref = wz_ref


def _walk_env(freeze_s=None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in FC_GOAL.items():
        cfg.setdefault(sec, {})[leaf] = val
    if freeze_s is not None:
        cfg.setdefault("goal", {})["walk_stop_freeze_s"] = freeze_s
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=8.0, seed=0, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    env.reset(seed=0)
    assert getattr(env._goal_traj, "mode", "") == "walk"
    return env


def _tick(env, q_safe, goal):
    """Simulate one _step_begin tick's worth of the override contract:
    call the hook, then latch the result into self._cmd exactly like
    the real call site does when the tick is not terminated."""
    env._current_goal = lambda: goal  # noqa: E731 - deliberate monkeypatch
    out = env._walk_stop_freeze_override(np.asarray(q_safe, dtype=float))
    env._cmd = out.copy()
    return out


def test_default_off_is_bit_exact_identity():
    """walk_stop_freeze_s absent (default 0.0) must return q_safe
    completely unchanged, for both stop and moving goals."""
    env = _walk_env(freeze_s=None)
    stop_goal = _FakeGoal(0.0, 0.0, 0.0)
    for i in range(20):
        q = np.full(N_JOINTS, 0.01 * i)
        out = _tick(env, q, stop_goal)
        assert np.array_equal(out, q)
    env.close()


def test_freeze_engages_after_threshold_and_holds():
    """A sustained stop command must leave q_safe untouched for ticks
    before the threshold, then pin the command to whatever it was AT
    the moment the threshold elapsed for every tick after, regardless
    of what new q_safe values keep being proposed."""
    thr = 0.5
    env = _walk_env(freeze_s=thr)
    stop_goal = _FakeGoal(0.0, 0.0, 0.0)
    n_thr = int(round(thr / env.dt))
    outs = []
    for i in range(n_thr + 20):
        q = np.full(N_JOINTS, 0.01 * (i + 1))   # keeps changing every tick
        out = _tick(env, q, stop_goal)
        outs.append(out.copy())
    # Every tick before the threshold elapses must pass q_safe through.
    for i in range(n_thr - 1):
        assert np.array_equal(outs[i], np.full(N_JOINTS, 0.01 * (i + 1)))
    # Once frozen, the held command must be IDENTICAL across all
    # remaining ticks (pinned), not tracking the still-changing
    # proposals.
    frozen_ticks = outs[n_thr + 2:]
    assert len(frozen_ticks) >= 5
    for out in frozen_ticks[1:]:
        assert np.array_equal(out, frozen_ticks[0])
    env.close()


def test_turn_in_place_is_exempt_from_freeze():
    """A nonzero commanded turn (wz_ref != 0) must never be frozen --
    that IS the commanded motion, same exemption as the stop-speed/
    stop-current reward charges."""
    thr = 0.5
    env = _walk_env(freeze_s=thr)
    turn_goal = _FakeGoal(0.0, 0.0, 0.3)
    n_thr = int(round(thr / env.dt))
    for i in range(n_thr + 20):
        q = np.full(N_JOINTS, 0.01 * (i + 1))
        out = _tick(env, q, turn_goal)
        assert np.array_equal(out, q), (
            f"tick {i}: turn-in-place command was frozen")
    env.close()


def test_resumed_walking_resets_the_timer():
    """A resumed walk command must clear the freeze accumulator so a
    later stop needs a fresh grace window before re-engaging --
    mirrors reward._walk_stop_cmd_s's own reset rule."""
    thr = 0.5
    env = _walk_env(freeze_s=thr)
    stop_goal = _FakeGoal(0.0, 0.0, 0.0)
    n_thr = int(round(thr / env.dt))
    for i in range(n_thr + 5):
        _tick(env, np.full(N_JOINTS, 0.01 * (i + 1)), stop_goal)
    assert env._walk_stop_freeze_cmd_s >= thr
    move_goal = _FakeGoal(0.06, 0.0, 0.0)
    _tick(env, np.full(N_JOINTS, 0.5), move_goal)
    assert env._walk_stop_freeze_cmd_s == pytest.approx(0.0)
    env.close()


def test_mode_gate_ignores_non_walk_goal_mode():
    """The hook must be a no-op outside walk/quadwalk mode even if the
    threshold is set (defends the hook against future goal types
    reusing the same env class)."""
    thr = 0.5
    env = _walk_env(freeze_s=thr)
    env._goal_traj.mode = "hold"
    stop_goal = _FakeGoal(0.0, 0.0, 0.0)
    n_thr = int(round(thr / env.dt))
    for i in range(n_thr + 5):
        q = np.full(N_JOINTS, 0.01 * (i + 1))
        out = _tick(env, q, stop_goal)
        assert np.array_equal(out, q)
    env.close()
