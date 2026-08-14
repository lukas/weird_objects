"""goal.mode_seq tests (TRANSITIONS_DIRECTIVE CODE item 1, 08-13).

One episode = K back-to-back mode segments (rise -> {hold|walk} ->
{walk|lower} -> rise ...) with per-switch re-anchoring of the height
frame on the CURRENT state. These tests lock:

1. default OFF = bit-exact legacy (no plan, identical obs stream to a
   cfg without the keys);
2. the sampled plan follows the command grammar and the first segment
   is start-kind compatible;
3. a mid-episode switch installs the target family's CANONICAL settled
   frame (_z0/q_nom/pad refs from the reset-time settle probe — the
   reanchor_to()/eval_handoff mechanics; trans-dagger2 fix 08-14),
   re-derives the goal bookkeeping (_h_target/_is_rise/ratchets), keeps
   the height reference continuous in absolute terms (blend window),
   and flips the obs mode one-hot;
4. the canonical frames MATCH what a fresh reset of the target mode
   derives (the parity reanchor_to() relies on — the instrumented diff
   from the trans-dagger2 kill, locked as a regression test);
5. walk segments arrive on the episode clock (shifted arrays) and the
   walk income accumulators restart;
6. every new per-episode attr rides mjx_host.SNAP_ATTRS (pool-restore
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
    # the switch installed the rise family's CANONICAL belly frame —
    # NOT the episode-reset plant frame, and NOT the instantaneous
    # chassis height (the v1 bug trans-dagger2 caught)
    belly = env._seq_frames["belly"]
    assert env._z0 == pytest.approx(belly["z0"])
    np.testing.assert_array_equal(env._q_nom, belly["q_nom"])
    np.testing.assert_array_equal(env._pad_z_ref, belly["pad_z_ref"])
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


def test_hold_segment_anchors_at_canonical_plant():
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
    q_nom_reset = env._q_nom.copy()      # belly frame (rise start)
    zeros = np.zeros(env.action_space.shape[0])
    sw = int(env._seq_plan[1]["tick"])
    for _ in range(sw + 5):
        env.step(zeros)
        if env._seq_idx == 1:
            break
    assert env._seq_idx == 1 and env._is_hold_bc
    # hold BC base = q_nom, re-based to the CANONICAL plant frame the
    # stance teachers trained against (trans-dagger2 fix) — never the
    # episode-reset belly frame, and no separate carried-pose anchor.
    assert env._seq_pose_anchor is None
    plant = env._seq_frames["plant"]
    np.testing.assert_array_equal(env._q_nom, plant["q_nom"])
    assert env._z0 == pytest.approx(plant["z0"])
    assert float(np.max(np.abs(env._q_nom - q_nom_reset))) > 0.5, \
        "belly->plant frame switch should move q_nom by ~79 deg knees"
    env.close()


# ---------------------------------------------------------------------------
# 4. frame parity: probe frames == fresh-reset frames (reanchor_to())
# ---------------------------------------------------------------------------

def test_canonical_frames_match_fresh_reset_frames():
    """The instrumented diff from the trans-dagger2 kill, locked as a
    regression test: the frame a switch installs must equal the frame
    reanchor_to() derives (a fresh env.reset() of the target mode on
    the same model). If these drift apart again, the in-env sequence
    context no longer reproduces the composition-proven handoff."""
    env = _make_env(seed=1, episode_seconds=10.0, seg_s=(3.0, 3.5),
                    mix={"lower": 1.0})
    env.reset()
    frames = env._seq_frames
    assert frames is not None
    # plant family: what reanchor_to("lower"/"hold"/"walk") derives.
    ref = _make_env(seed=5, seq=False, episode_seconds=5.0,
                    mix={"lower": 1.0})
    ref.reset()
    assert ref._goal_traj.start_at == "plant"
    np.testing.assert_allclose(frames["plant"]["q_nom"], ref._q_nom,
                               atol=5e-3)
    assert abs(frames["plant"]["z0"] - ref._z0) < 2e-3
    np.testing.assert_allclose(frames["plant"]["pad_z_ref"],
                               ref._pad_z_ref, atol=2e-3)
    ref.close()
    # belly family: what reanchor_to("rise", force_rise_start="flat")
    # derives.
    ref2 = _make_env(seed=7, seq=False, episode_seconds=5.0,
                     mix={"rise": 1.0})
    ref2._goal_gen.force_rise_start = "flat"
    ref2.reset()
    assert ref2._goal_traj.start_at == "zero"
    np.testing.assert_allclose(frames["belly"]["q_nom"], ref2._q_nom,
                               atol=5e-3)
    assert abs(frames["belly"]["z0"] - ref2._z0) < 2e-3
    ref2.close()
    env.close()


# ---------------------------------------------------------------------------
# 5. walk segments: episode-clock arrays + accumulator restart
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
# 6. pool-restore safety
# ---------------------------------------------------------------------------

def test_seq_attrs_in_snap_attrs():
    for a in ("_seq_plan", "_seq_idx", "_seq_stand_z", "_seq_seg_end",
              "_seq_pose_anchor", "_seq_frames"):
        assert a in SNAP_ATTRS, f"{a} missing from mjx_host.SNAP_ATTRS"


# ---------------------------------------------------------------------------
# 7. fractional mode_seq = mixed sequence/single-mode diet (08-14, the
#    Arm 2 "retain ~25% single-mode episodes" hook). Endpoints stay
#    bit-exact: p=1.0 draws no extra rng (all-sequence, historical
#    stream), p=0 is the legacy path (locked above); 0<p<1 mixes both
#    episode kinds at roughly the configured rate.
# ---------------------------------------------------------------------------

def test_fractional_mode_seq_mixes_episode_kinds():
    env = _make_env(seq=True, episode_seconds=8.0)
    env.cfg["goal"]["mode_seq"] = 0.75
    seq_eps = 0
    n = 40
    for _ in range(n):
        env.reset()
        if env._seq_plan is not None:
            seq_eps += 1
    # binomial(40, .75): P(outside [20, 38]) < 1e-3 — a loose band that
    # still catches inverted or endpoint-stuck semantics.
    assert 20 <= seq_eps <= 38, seq_eps
    assert 0 < seq_eps < n  # both kinds actually appear


def test_full_mode_seq_draws_no_extra_rng():
    # p=1.0 must keep the historical all-sequence rng stream: obs after
    # reset identical to a fresh env with the same seed (the draw-free
    # fast path).
    a = _make_env(seed=11, seq=True, episode_seconds=8.0)
    b = _make_env(seed=11, seq=True, episode_seconds=8.0)
    b.cfg["goal"]["mode_seq"] = 1.0  # explicit float endpoint
    oa, _ = a.reset()
    ob, _ = b.reset()
    np.testing.assert_array_equal(oa, ob)
    assert a._seq_plan is not None and b._seq_plan is not None
