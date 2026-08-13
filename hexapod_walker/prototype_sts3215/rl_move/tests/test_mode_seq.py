"""goal.mode_seq tests (TRANSITIONS_DIRECTIVE CODE item 1, 08-13).

One episode = K back-to-back mode segments (rise -> {hold|walk} ->
{walk|lower} -> rise ...) with per-switch re-anchoring of the height
frame on the CURRENT state. These tests lock:

1. default OFF = bit-exact legacy (no plan, identical obs stream to a
   cfg without the keys);
2. the sampled plan follows the command grammar and the first segment
   is start-kind compatible;
3. a mid-episode switch re-anchors _z0 at the current chassis height,
   re-derives the goal bookkeeping (_h_target/_is_rise/ratchets), keeps
   the height reference continuous in absolute terms (blend window),
   and flips the obs mode one-hot;
4. walk segments arrive on the episode clock (shifted arrays) and the
   walk income accumulators restart;
5. every new per-episode attr rides mjx_host.SNAP_ATTRS (pool-restore
   lesson, commit 65edba7).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.mjx_host import SNAP_ATTRS  # noqa: E402
from rl_move.sim.walk_task import (  # noqa: E402
    MODE_ONEHOT_ORDER, SimHexapodJointWalkEnv,
)

SEQ_MODES = ("rise", "walk", "hold", "lower")


def _make_env(seed: int = 0, *, seq: bool | None = True,
              episode_seconds: float = 12.0,
              seg_s=(3.0, 4.0), mix: dict | None = None,
              extra_cfg: dict | None = None) -> SimHexapodJointWalkEnv:
    cfg = load_config()
    g = cfg.setdefault("goal", {})
    if seq is not None:
        g["mode_seq"] = 1.0 if seq else 0.0
    if seq:
        g["mode_seq_segment_s_min"] = seg_s[0]
        g["mode_seq_segment_s_max"] = seg_s[1]
    cfg.setdefault("obs", {})["mode_onehot"] = 1.0
    for k, v in (extra_cfg or {}).items():
        sect, name = k.split(".", 1)
        cfg.setdefault(sect, {})[name] = v
    env = SimHexapodJointWalkEnv(cfg, seed=seed,
                                 episode_seconds=episode_seconds)
    if mix is not None:
        gen = env._goal_gen
        for a in [a for a in vars(gen) if a.startswith("p_")]:
            setattr(gen, a, 0.0)
        gen.p_walk = 0.0
        for m, p in mix.items():
            setattr(gen, f"p_{m}", p)
    return env


# ---------------------------------------------------------------------------
# 1. default off = legacy exact
# ---------------------------------------------------------------------------

def test_off_by_default_no_plan():
    env = _make_env(seq=None, episode_seconds=5.0)
    env.reset()
    assert env._seq_plan is None
    assert env._seq_pose_anchor is None


def test_explicit_zero_matches_unset_stream():
    a = _make_env(seed=3, seq=None, episode_seconds=5.0)
    b = _make_env(seed=3, seq=False, episode_seconds=5.0)
    oa, _ = a.reset()
    ob, _ = b.reset()
    np.testing.assert_array_equal(oa, ob)
    rng = np.random.default_rng(0)
    for _ in range(30):
        act = rng.uniform(-0.2, 0.2, a.action_space.shape[0])
        ra = a.step(act)
        rb = b.step(act)
        np.testing.assert_array_equal(ra[0], rb[0])
        assert ra[1] == rb[1]
    assert b._seq_plan is None


# ---------------------------------------------------------------------------
# 2. plan grammar + first-segment start compatibility
# ---------------------------------------------------------------------------

def test_plan_grammar_and_start_kinds():
    nxt = SimHexapodJointWalkEnv.SEQ_NEXT
    for seed in range(12):
        env = _make_env(seed=seed, episode_seconds=20.0)
        env.reset()
        plan = env._seq_plan
        assert plan is not None and 2 <= len(plan) <= 5
        modes = [p["mode"] for p in plan]
        assert all(m in SEQ_MODES for m in modes)
        for a, b in zip(modes, modes[1:]):
            assert b in nxt[a], f"illegal transition {a}->{b}"
        ticks = [p["tick"] for p in plan]
        assert ticks == sorted(ticks) and ticks[0] == 0
        assert ticks[-1] < env.episode_steps
        # first segment start-kind compatibility
        start = getattr(env._goal_traj, "start_at", "plant")
        if modes[0] == "rise":
            assert start in ("zero", "crouch")
        elif modes[0] == "lower":
            assert start in ("plant", "zero")
        elif modes[0] == "hold":
            assert start == "plant"
        else:   # walk: its own spawn draws
            assert start in ("plant", "park", "gait")
        env.close()


# ---------------------------------------------------------------------------
# 3. the switch: re-anchor + bookkeeping + blend continuity + one-hot
# ---------------------------------------------------------------------------

def test_lower_to_rise_switch_reanchors():
    # p_lower=1 -> first segment lower (plant start, belly draw off);
    # the grammar forces lower -> rise: the directive's #1 trap case.
    env = _make_env(seed=1, episode_seconds=10.0, seg_s=(3.0, 3.5),
                    mix={"lower": 1.0})
    obs, _ = env.reset()
    assert env._seq_plan[0]["mode"] == "lower"
    assert env._seq_plan[1]["mode"] == "rise"
    assert env._h_target < 0.0
    stand_z0 = env._seq_stand_z
    assert stand_z0 is not None
    sw = int(env._seq_plan[1]["tick"])
    zeros = np.zeros(env.action_space.shape[0])
    prev_abs = env._z0 + env._current_goal().height_ref
    onehot_prev = obs[-len(MODE_ONEHOT_ORDER):]
    assert onehot_prev[MODE_ONEHOT_ORDER.index("lower")] == 1.0
    for _ in range(sw + 5):
        obs, _r, term, trunc, _info = env.step(zeros)
        assert not (term or trunc)
        cur_abs = env._z0 + env._current_goal().height_ref
        # absolute-ref continuity through the switch (blend window)
        assert abs(cur_abs - prev_abs) < 0.02
        prev_abs = cur_abs
        if env._seq_idx == 1:
            break
    assert env._seq_idx == 1
    # _z0 re-anchored at the CURRENT chassis height, not the reset one
    z_now = float(env.data.xpos[env._chassis_bid, 2])
    assert abs(env._z0 - z_now) < 5e-3
    # goal bookkeeping re-derived for the new segment
    assert env._is_rise and not env._is_lower_bc
    assert env._h_target > 0.0
    assert env._score_best is None or env._step_i > sw + 1
    assert env._rsi_ref_tick0 is None
    assert env._seq_stand_z == pytest.approx(env._z0 + env._h_target)
    # obs mode one-hot flipped lower -> rise
    onehot = obs[-len(MODE_ONEHOT_ORDER):]
    assert onehot[MODE_ONEHOT_ORDER.index("rise")] == 1.0
    assert onehot[MODE_ONEHOT_ORDER.index("lower")] == 0.0
    env.close()


def test_hold_segment_pose_anchor_captured():
    # rise -> {hold|walk}: find a seed whose plan has a hold at idx 1.
    for seed in range(30):
        env = _make_env(seed=seed, episode_seconds=10.0,
                        seg_s=(3.0, 3.5), mix={"rise": 1.0})
        env.reset()
        if env._seq_plan[1]["mode"] == "hold":
            break
        env.close()
    else:
        pytest.skip("no rise->hold plan in 30 seeds")
    assert env._seq_pose_anchor is None
    zeros = np.zeros(env.action_space.shape[0])
    sw = int(env._seq_plan[1]["tick"])
    for _ in range(sw + 5):
        env.step(zeros)
        if env._seq_idx == 1:
            break
    assert env._seq_idx == 1 and env._is_hold_bc
    # hold BC base = the pose carried INTO the segment, not reset q_nom
    assert env._seq_pose_anchor is not None
    np.testing.assert_allclose(env._seq_pose_anchor,
                               np.asarray(env.data.qpos[env._qadr]),
                               atol=0.2)
    env.close()


# ---------------------------------------------------------------------------
# 4. walk segments: episode-clock arrays + accumulator restart
# ---------------------------------------------------------------------------

def test_walk_segment_traj_shifted_and_state_reset():
    env = _make_env(seed=0, episode_seconds=10.0, mix={"hold": 1.0})
    env.reset()
    n = env.episode_steps + 1
    tick = 60
    env._ls_prog_m = 1.23
    env._gait_cmd_tick = 99
    env._z0 = float(env.data.xpos[env._chassis_bid, 2])
    traj, h_target, ramp_i0 = env._seq_segment_traj("walk", tick)
    assert len(traj.vx) == n and len(traj.height) == n
    # head is padded with the schedule's first value (a zero hold)
    assert float(np.abs(traj.vx[:tick]).max()) == 0.0
    assert float(np.abs(traj.vx).max()) > 0.0
    g = traj.at(tick)
    assert g.vx_ref == 0.0            # settle head at the switch
    env._seq_reset_mode_state("walk", ramp_i0, h_target)
    assert env._ls_prog_m == 0.0 and env._gait_cmd_tick == 0
    assert env._walk_bc_gait is None  # bc_anchor_coef unset here
    assert not env._is_rise and not env._is_hold_bc
    env.close()


def test_rise_segment_aims_at_stand_anchor():
    env = _make_env(seed=0, episode_seconds=10.0, mix={"hold": 1.0})
    env.reset()
    env._z0 = 0.080                     # pretend the body is at 80 mm
    env._seq_stand_z = 0.130            # last commanded stand: 130 mm
    traj, h_target, ramp_i0 = env._seq_segment_traj("rise", 40)
    assert h_target == pytest.approx(0.050)   # aims BACK at the stand
    assert float(traj.height[:40 + 1].max()) == 0.0   # hold head
    assert float(traj.height[-1]) == pytest.approx(0.050)
    assert ramp_i0 > 40
    assert env._seq_stand_z == pytest.approx(0.130)
    env.close()


# ---------------------------------------------------------------------------
# 5. pool-restore safety
# ---------------------------------------------------------------------------

def test_seq_attrs_in_snap_attrs():
    for a in ("_seq_plan", "_seq_idx", "_seq_stand_z", "_seq_seg_end",
              "_seq_pose_anchor"):
        assert a in SNAP_ATTRS, f"{a} missing from mjx_host.SNAP_ATTRS"
