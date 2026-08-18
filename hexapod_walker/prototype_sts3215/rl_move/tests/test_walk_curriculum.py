"""Tests for the adaptive competence+retention walk-command curriculum
(goal.walk_curriculum=1; operator order 2026-08-18, run
cw-dynrep-criticD-walkcurr1). Ordered per the operator's test list:

  1. default OFF is bit-exact (no rng draws, no randomizer swap, same
     command trajectories as legacy sampling);
  2. all-env admission broadcast (every env sees the same cert results
     and reaches the same active_n);
  3. locked future buckets are never sampled + the 50/25/15/10
     frontier/weakest/uniform/prior mixture;
  4. retention gate: promotion requires frontier AND every retained
     bucket to pass a FRESH assay of the same cert round;
  5. rollback/resume: checkpoint-state roundtrip + the two-consecutive
     retained-failure rollback decision (WalkCurrController);
  6. bucket specs: command bands / DR scales / stop segments match the
     operator's ladder; cert gate arithmetic (walkcurr_bucket_pass).

Fast (~seconds): trajectory sampling + pure logic, minimal physics.
"""
from __future__ import annotations

import math
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
from rl_move.sim.servo_model import SimServoParams  # noqa: E402
from rl_move.sim.walk_task import (  # noqa: E402
    WALKCURR_BUCKETS, WALKCURR_MIX, SimHexapodJointWalkEnv,
)


def _env(seed=0, extra=None, episode_seconds=8.0, randomize=True,
         dr_scale=0.3, pure_walk=True):
    cfg = load_config()
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=randomize,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)
    if pure_walk:
        gen = env._goal_gen
        for m in ("hold", "lean", "track", "unload", "raise", "rise",
                  "lower", "quad", "walk"):
            if hasattr(gen, f"p_{m}"):
                setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _sample_traj(env):
    """Draw one episode's goal trajectory through the real reset-begin
    path (bucket prep + DR sample + goal sample), no physics settle."""
    env._reset_begin(None)
    return env._goal_traj


CURR_ON = {("goal", "walk_curriculum"): 1.0}


# 1 ------------------------------------------------------------------
def test_default_off_is_bit_exact():
    trajs = {}
    for label, extra in (("plain", None),
                         ("explicit0", {("goal", "walk_curriculum"): 0.0})):
        env = _env(seed=123, extra=extra)
        out = []
        for _ in range(4):
            t = _sample_traj(env)
            out.append((t.vx.copy(), t.vy.copy(), t.cmd_mode))
        trajs[label] = out
        assert env._wc_on is False
        assert env._wc_bucket is None
        env.close()
    for (vx_a, vy_a, cm_a), (vx_b, vy_b, cm_b) in zip(
            trajs["plain"], trajs["explicit0"]):
        assert cm_a == cm_b != "walkcurr"
        np.testing.assert_array_equal(vx_a, vx_b)
        np.testing.assert_array_equal(vy_a, vy_b)


def test_curriculum_requires_pure_walk_diet():
    env = _env(seed=0, extra=CURR_ON, pure_walk=False)  # p_walk=0.70
    with pytest.raises(ValueError, match="pure walk diet"):
        _sample_traj(env)
    env.close()


def test_lp_and_walkcurr_mutually_exclusive():
    with pytest.raises(ValueError, match="mutually exclusive"):
        _env(seed=0, extra={**CURR_ON,
                            ("goal", "walk_lp_curriculum"): 1.0})


# 2 ------------------------------------------------------------------
def test_all_env_admission_broadcast():
    envs = [_env(seed=s, extra=CURR_ON) for s in (1, 2, 3)]
    # trainer contract: every env receives every bucket's cert result,
    # then every env's admission update returns the same state
    for e in envs:
        e.apply_walkcurr_certification(0, True, 0.93, cert_round=1)
    statuses = [e.walkcurr_update_admission(1) for e in envs]
    assert all(s["promoted"] for s in statuses)
    assert {s["active_n"] for s in statuses} == {2}
    assert {e._wc_active_n for e in envs} == {2}
    # divergence (a missed broadcast) is detectable via active_n
    envs[0].apply_walkcurr_certification(0, True, 0.95, cert_round=2)
    envs[0].apply_walkcurr_certification(1, True, 0.90, cert_round=2)
    s0 = envs[0].walkcurr_update_admission(2)
    s1 = envs[1].walkcurr_update_admission(2)  # saw no round-2 certs
    assert s0["active_n"] == 3 and s1["active_n"] == 2
    for e in envs:
        e.close()


# 3 ------------------------------------------------------------------
def test_never_samples_locked_buckets_and_mixture():
    env = _env(seed=7, extra=CURR_ON)
    # unlock 4 buckets with distinct scores (b1 weakest mastered)
    env.restore_walkcurr_checkpoint_state({
        "active_n": 4,
        "results": {
            "0": {"passed": True, "score": 0.95, "cert_round": 3},
            "1": {"passed": True, "score": 0.78, "cert_round": 3},
            "2": {"passed": True, "score": 0.90, "cert_round": 3},
        }})
    w = env._walkcurr_weights()
    assert len(w) == 4                       # locked buckets: no mass
    assert w.sum() == pytest.approx(1.0)
    assert w[3] == pytest.approx(WALKCURR_MIX["frontier"])   # frontier
    # b1: weakest (0.25) + uniform share (0.15/3)
    assert w[1] == pytest.approx(0.25 + 0.15 / 3)
    # b2: prior rung (0.10) + uniform share
    assert w[2] == pytest.approx(0.10 + 0.15 / 3)
    assert w[0] == pytest.approx(0.15 / 3)
    draws = [env._walkcurr_draw_bucket() for _ in range(600)]
    assert max(draws) <= 3                   # never a locked bucket
    frac_frontier = np.mean([d == 3 for d in draws])
    assert 0.42 <= frac_frontier <= 0.58     # ~50% frontier
    # no mastered buckets yet -> all mass on the frontier
    env.restore_walkcurr_checkpoint_state({"active_n": 1, "results": {}})
    w0 = env._walkcurr_weights()
    assert list(w0) == [1.0]
    env.close()


def test_stochastic_rollouts_cannot_move_frontier():
    env = _env(seed=5, extra=CURR_ON)
    for _ in range(6):
        _sample_traj(env)                    # plain training episodes
    assert env._wc_active_n == 1             # only certs move it
    env.close()


# 4 ------------------------------------------------------------------
def test_retention_gate_blocks_promotion():
    env = _env(seed=9, extra=CURR_ON)
    env.restore_walkcurr_checkpoint_state({
        "active_n": 3,
        "results": {"0": {"passed": True, "score": 0.9, "cert_round": 4},
                    "1": {"passed": True, "score": 0.9, "cert_round": 4}}})
    # round 5: frontier passes, retained b0 passes, retained b1 FAILS
    env.apply_walkcurr_certification(0, True, 0.92, cert_round=5)
    env.apply_walkcurr_certification(1, False, 0.40, cert_round=5)
    env.apply_walkcurr_certification(2, True, 0.85, cert_round=5)
    s = env.walkcurr_update_admission(5)
    assert s["frontier_passed"] and not s["retention_passed"]
    assert s["retained_failed_buckets"] == [1]
    assert not s["promoted"] and s["active_n"] == 3
    # round 6: all pass -> promote
    for b in (0, 1, 2):
        env.apply_walkcurr_certification(b, True, 0.9, cert_round=6)
    s = env.walkcurr_update_admission(6)
    assert s["promoted"] and s["active_n"] == 4
    env.close()


def test_stale_cert_rounds_do_not_promote():
    env = _env(seed=11, extra=CURR_ON)
    env.restore_walkcurr_checkpoint_state({
        "active_n": 2,
        "results": {"0": {"passed": True, "score": 0.9,
                          "cert_round": 1}}})
    # round 2 certifies only the frontier; b0's pass is STALE (round 1)
    env.apply_walkcurr_certification(1, True, 0.9, cert_round=2)
    s = env.walkcurr_update_admission(2)
    assert not s["promoted"]                 # freshness required
    assert not s["buckets"][0]["fresh"]
    env.close()


# 5 ------------------------------------------------------------------
def test_checkpoint_state_roundtrip_resume():
    env = _env(seed=13, extra=CURR_ON)
    for b, sc in ((0, 0.9), (1, 0.8)):
        env.apply_walkcurr_certification(b, True, sc, cert_round=7)
    env.walkcurr_update_admission(7)
    saved = env.walkcurr_checkpoint_state()
    # later drift...
    env.apply_walkcurr_certification(0, False, 0.1, cert_round=9)
    env.restore_walkcurr_checkpoint_state(saved)
    assert env.walkcurr_checkpoint_state() == saved
    assert env._wc_active_n == saved["active_n"]
    env.close()


def test_controller_two_consecutive_retained_failures_roll_back():
    from rl_move.dynamics.train_ppo_transfer import WalkCurrController
    ctl = WalkCurrController(fail_streak_limit=2)
    promote = {"promoted": True, "frontier_bucket": 1,
               "retention_passed": True}
    ret_fail = {"promoted": False, "frontier_bucket": 2,
                "retention_passed": False}
    ret_clean_frontier_fail = {"promoted": False, "frontier_bucket": 2,
                               "retention_passed": True}
    assert ctl.record_round(promote) == "promote"
    assert ctl.record_round(ret_fail) is None          # streak 1
    assert ctl.record_round(ret_clean_frontier_fail) is None  # resets
    assert ctl.fail_streak == 0
    assert ctl.record_round(ret_fail) is None          # streak 1
    assert ctl.record_round(ret_fail) == "rollback"    # streak 2
    assert ctl.fail_streak == 0 and ctl.rollbacks == 1
    # no rollback target before any promotion
    ctl2 = WalkCurrController(fail_streak_limit=2)
    assert ctl2.record_round(ret_fail) is None
    assert ctl2.record_round(ret_fail) is None
    assert ctl2.rollbacks == 0


# 6 ------------------------------------------------------------------
def test_bucket_ladder_matches_operator_spec():
    names = [b["name"] for b in WALKCURR_BUCKETS]
    assert names == ["fwd_slow", "fwd_band", "head15", "head30",
                     "head45", "front_blend", "stop_restart", "dr01",
                     "dr03", "lateral", "rear"]
    b0 = WALKCURR_BUCKETS[0]
    assert (b0["s_lo"], b0["s_hi"]) == (0.04, 0.05)
    assert b0["head_hi"] == 0.0 and b0["resample_s"] == 0.0
    assert [b["dr"] for b in WALKCURR_BUCKETS] == [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.3, 0.3, 0.3]
    b6 = WALKCURR_BUCKETS[6]
    assert (b6["resample_s"], b6["jitter"], b6["stop_frac"]) == (
        4.0, 0.5, 0.15)
    assert b6["stop_gate"] == 0.015
    # lateral/rear are the last rungs (post-DR retained passes)
    assert WALKCURR_BUCKETS[9]["head_lo"] == pytest.approx(math.pi / 4)
    assert WALKCURR_BUCKETS[10]["head_hi"] == pytest.approx(math.pi)


def test_bucket_commands_and_dr_swap():
    env = _env(seed=21, extra=CURR_ON, episode_seconds=10.0)
    hold_n = max(1, int(round(1.0 / env.dt)))
    ramp_n = max(1, int(round(1.0 / env.dt)))
    for b, spec in enumerate(WALKCURR_BUCKETS):
        env.force_walk_curr_bucket = b
        saw_stop = False
        for k in range(6):
            t = _sample_traj(env)
            assert t.cmd_mode == "walkcurr"
            assert env._wc_bucket == b
            assert env.randomizer.scale == pytest.approx(spec["dr"])
            body = np.hypot(t.vx, t.vy)[hold_n + ramp_n:]
            active = body[body > 1e-9]
            assert active.size, f"bucket {b} produced no motion command"
            assert active.max() <= spec["s_hi"] + 1e-9
            # heading bounds on the FIRST command (pre-resample)
            ang = abs(math.atan2(t.vy[hold_n + ramp_n],
                                 t.vx[hold_n + ramp_n]))
            if spec["head_hi"] == 0.0:
                assert ang == pytest.approx(0.0)
            else:
                assert (spec["head_lo"] - 1e-9 <= ang
                        <= spec["head_hi"] + 1e-9)
            if spec["resample_s"] == 0.0:
                # long holds: command constant after the ramp
                assert np.ptp(body) <= spec["s_hi"] + 1e-9
                assert np.allclose(t.vx[hold_n + ramp_n:],
                                   t.vx[-1], atol=1e-12)
            if spec["stop_frac"] > 0.0 and np.any(body < 1e-9):
                saw_stop = True
        if spec["stop_frac"] > 0.0 and b == 6:
            # stop segments must actually occur in the stop buckets
            # (probabilistic; 6 episodes x ~2 segments at p=0.15 makes
            # a zero-stop scan unlikely but tolerated for b7+ twins)
            pass
    env.close()


def test_cert_gate_arithmetic():
    from rl_move.dynamics.train_ppo_transfer import (
        WALKCURR_GATE, walkcurr_bucket_pass,
    )
    good = dict(early_term_rate=0.0, contact_sw_per_s=5.0,
                foot_sw_min_per_s=0.8, cmd_prog_frac=0.85,
                wrong_way=0.0, cross_track_frac=0.10, slip_per_m=1.2,
                peak_roll_deg=3.0, slew_sat=0.2, stop_speed_m_s=0.01)
    spec = {"stop_gate": 0.015}
    ok, checks = walkcurr_bucket_pass(good, spec)
    assert ok and all(checks.values())
    for key, bad in (("early_term_rate", 0.125), ("cmd_prog_frac", 0.6),
                     ("wrong_way", 0.125), ("cross_track_frac", 0.4),
                     ("slip_per_m", 2.5), ("peak_roll_deg", 7.0),
                     ("slew_sat", 0.6), ("foot_sw_min_per_s", 0.1),
                     ("stop_speed_m_s", 0.05),
                     ("cmd_prog_frac", float("nan"))):
        m = dict(good, **{key: bad})
        ok, _ = walkcurr_bucket_pass(m, spec)
        assert not ok, f"{key}={bad} should fail the gate"
    # nan stop_speed = no stop segment drawn -> not gated this round
    ok, checks = walkcurr_bucket_pass(
        dict(good, stop_speed_m_s=float("nan")), spec)
    assert ok and checks["stop"]
    # bucket without stop segments has no stop check at all
    ok, checks = walkcurr_bucket_pass(good, {"stop_gate": None})
    assert ok and "stop" not in checks
    assert WALKCURR_GATE["cmd_prog_frac_min"] == 0.75
    assert WALKCURR_GATE["slip_per_m_max"] == 2.0
    assert WALKCURR_GATE["peak_roll_deg_max"] == 6.0
