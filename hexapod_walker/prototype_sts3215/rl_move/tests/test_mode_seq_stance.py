"""goal.mode_seq_stance tests (cw-stand-postlower3 spec, 08-15).

Stance-only in-context sequencing on the joint_goal task: one episode
= K back-to-back STANCE segments (rise -> hold -> lower -> rise, the
operator grammar with walk removed) using the proven goal.mode_seq
switch machinery (canonical per-family frames, blend windows, per-
segment bookkeeping). Motivation: the postlower1/2 dig-in
(SESSION_BULK_GATE.md Cohort c2) proved cold single-mode spawns cannot
reproduce the in-session lower->rise transition context, and the bank
family trained on mechanically impossible rise targets; sequences give
the transition IN CONTEXT with the rise target anchored at the
sequence's own commanded stand height (_seq_stand_z). These tests lock:

1. default OFF = bit-exact legacy on the joint_goal task (no plan,
   identical obs stream to a cfg without the key);
2. the sampled plan is stance-only (never a walk segment) and follows
   the restricted grammar;
3. lower -> rise switch installs the canonical BELLY frame and the
   rise target is the REMAINING rise from that frame, never above the
   sequence's anchored stand height (the c2 impossible-target
   regression test) and never above gen.rise_m[1];
4. the stance key on the joint_walk task raises (its base-mode
   fallback would otherwise start stance sequences mid-draw);
5. the walk task's own goal.mode_seq behavior is untouched by the
   _seq_segment_traj delegation refactor (covered by the existing
   test_mode_seq.py bank, run alongside this file).
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
from rl_move.sim.joint_task import SimHexapodJointGoalEnv  # noqa: E402
from rl_move.sim.walk_task import SimHexapodJointWalkEnv  # noqa: E402

STANCE_MODES = ("rise", "hold", "lower")


def _make_env(seed: int = 0, *, seq: bool | None = True,
              episode_seconds: float = 12.0,
              seg_s=(3.0, 4.0), mix: dict | None = None,
              cls=SimHexapodJointGoalEnv):
    cfg = load_config()
    g = cfg.setdefault("goal", {})
    if seq is not None:
        g["mode_seq_stance"] = 1.0 if seq else 0.0
    if seq:
        g["mode_seq_segment_s_min"] = seg_s[0]
        g["mode_seq_segment_s_max"] = seg_s[1]
    env = cls(cfg, seed=seed, episode_seconds=episode_seconds)
    if mix is not None:
        gen = env._goal_gen
        for a in [a for a in vars(gen) if a.startswith("p_")]:
            setattr(gen, a, 0.0)
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
# 2. plan grammar: stance-only, restricted transitions
# ---------------------------------------------------------------------------

def test_plan_grammar_stance_only():
    nxt = SimHexapodJointGoalEnv.SEQ_NEXT_STANCE
    for seed in range(12):
        env = _make_env(seed=seed, episode_seconds=20.0)
        env.reset()
        plan = env._seq_plan
        assert plan is not None and 2 <= len(plan) <= 5
        modes = [p["mode"] for p in plan]
        assert all(m in STANCE_MODES for m in modes), \
            f"non-stance segment in {modes}"
        for a, b in zip(modes, modes[1:]):
            assert b in nxt[a], f"illegal transition {a}->{b}"
        ticks = [p["tick"] for p in plan]
        assert ticks == sorted(ticks) and ticks[0] == 0
        assert ticks[-1] < env.episode_steps
        env.close()


def test_max_segments_cap():
    env = _make_env(seed=2, episode_seconds=20.0, seg_s=(3.0, 3.5))
    env.cfg["goal"]["mode_seq_max_segments"] = 2
    env.reset()
    assert len(env._seq_plan) == 2
    env.close()


# ---------------------------------------------------------------------------
# 3. lower -> rise: canonical belly frame + reachable remaining-rise
#    target (the Cohort-c2 impossible-target regression test)
# ---------------------------------------------------------------------------

def test_lower_to_rise_targets_remaining_rise():
    env = _make_env(seed=1, episode_seconds=10.0, seg_s=(3.0, 3.5),
                    mix={"lower": 1.0})
    env.reset()
    assert env._seq_plan[0]["mode"] == "lower"
    assert env._seq_plan[1]["mode"] == "rise"
    assert env._h_target < 0.0
    stand_z0 = env._seq_stand_z          # settled plant height anchor
    assert stand_z0 is not None
    sw = int(env._seq_plan[1]["tick"])
    zeros = np.zeros(env.action_space.shape[0])
    prev_abs = env._z0 + env._current_goal().height_ref
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
    # canonical BELLY frame installed (not the plant reset frame, not
    # the instantaneous chassis height)
    belly = env._seq_frames["belly"]
    assert env._z0 == pytest.approx(belly["z0"])
    np.testing.assert_array_equal(env._q_nom, belly["q_nom"])
    # bookkeeping re-derived
    assert env._is_rise and not env._is_lower_bc
    # THE c2 regression test: the rise target is the REMAINING rise to
    # the sequence's own anchored stand height — reachable by
    # construction, never above the legacy amplitude cap.
    assert env._h_target > 0.0
    assert env._h_target <= env._goal_gen.rise_m[1] + 1e-9
    assert env._z0 + env._h_target <= stand_z0 + 1e-9
    assert env._z0 + env._h_target == pytest.approx(
        min(stand_z0, env._z0 + env._goal_gen.rise_m[1]))
    assert env._seq_stand_z == pytest.approx(env._z0 + env._h_target)
    env.close()


def test_rise_to_hold_anchors_at_canonical_plant():
    for seed in range(30):
        env = _make_env(seed=seed, episode_seconds=10.0,
                        seg_s=(3.0, 3.5), mix={"rise": 1.0})
        env.reset()
        if env._seq_plan[0]["mode"] == "rise":
            break
        env.close()
    else:
        pytest.skip("no rise-first plan in 30 seeds")
    assert env._seq_plan[1]["mode"] == "hold"   # grammar: rise -> hold
    zeros = np.zeros(env.action_space.shape[0])
    sw = int(env._seq_plan[1]["tick"])
    for _ in range(sw + 5):
        env.step(zeros)
        if env._seq_idx == 1:
            break
    assert env._seq_idx == 1 and env._is_hold_bc
    plant = env._seq_frames["plant"]
    np.testing.assert_array_equal(env._q_nom, plant["q_nom"])
    assert env._z0 == pytest.approx(plant["z0"])
    env.close()


# ---------------------------------------------------------------------------
# 4. the stance key is joint_goal-only
# ---------------------------------------------------------------------------

def test_walk_task_rejects_stance_key():
    env = _make_env(seed=0, episode_seconds=5.0,
                    cls=SimHexapodJointWalkEnv)
    with pytest.raises(ValueError, match="mode_seq_stance"):
        env.reset()
    env.close()
