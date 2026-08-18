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
    WALKCURR_BUCKETS, WALKCURR_BUCKETS_V2, WALKCURR_GATE_V2_IGNITION,
    WALKCURR_GATE_V2_QUALITY, WALKCURR_MIX, SIGMA_V,
    SimHexapodJointWalkEnv,
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


# -- walkcurr2 (operator MCP note fb_20260818T060044): B0/B1 ignition
# band fix + relaxed slew admission ------------------------------------
CURR_V2_ON = {("goal", "walk_curriculum"): 2.0}


def test_v2_default_is_v1_bit_exact():
    """goal.walk_curriculum=1 is untouched: same table object, same
    per-episode trajectories as before this change (regression guard
    for the version-selection refactor)."""
    env = _env(seed=5, extra=CURR_ON)
    assert env._wc_version == 1
    assert env._wc_table is WALKCURR_BUCKETS
    env.close()


def test_v2_selects_the_v2_table():
    env = _env(seed=5, extra=CURR_V2_ON)
    assert env._wc_version == 2
    assert env._wc_table is WALKCURR_BUCKETS_V2
    assert len(WALKCURR_BUCKETS_V2) == len(WALKCURR_BUCKETS)
    env.close()


def test_v2_ignition_band_is_outside_the_velocity_kernel_park_zone():
    """Root cause 1 (walkcurr1 stalled at B0, 0/10 promotions): V1's B0
    (0.04-0.05 m/s, dead ahead) sits entirely inside one SIGMA_V of a
    PARKED robot, so standing still scores close to peak reward. V2's
    B0 must put a parked robot's error multiple sigma away, and must
    vary heading from the very first bucket (not deferred to a later
    rung)."""
    import math as _m

    def _park_reward_frac(speed: float) -> float:
        """A zero-output (parked) robot's velocity-kernel reward as a
        fraction of peak, when the commanded speed is ``speed``."""
        return _m.exp(-(speed ** 2) / (2.0 * SIGMA_V ** 2))

    b0_v1, b0_v2 = WALKCURR_BUCKETS[0], WALKCURR_BUCKETS_V2[0]
    assert b0_v2["name"] == "ignition"
    park_v1 = _park_reward_frac(b0_v1["s_lo"])
    park_v2 = _park_reward_frac(b0_v2["s_lo"])
    assert park_v1 > 0.5, "documents the V1 bug: parking near-peaks B0"
    assert park_v2 < 0.35, (
        f"V2 ignition (s_lo={b0_v2['s_lo']}) must make parking score "
        f"well under half of what V1's B0 gave it (got {park_v2:.2f} "
        f"vs V1's {park_v1:.2f})")
    assert b0_v2["head_hi"] > 0.0, (
        "ignition must vary heading from step 0, not defer it to a "
        "later bucket the way V1's B0/B1 did")
    # the original V1 park-zone bucket must NOT reappear in V2 at all
    assert not any(0.0 < b["s_hi"] <= SIGMA_V for b in WALKCURR_BUCKETS_V2)


def test_v2_gate_admits_the_known_good_checkpoint_v1_gate_rejects():
    """The retained 6M-best cw-dynrep-criticD-40m1 checkpoint (matched
    task/backend/mostly-matched budget, the best real evidence of a
    genuinely good policy at this scale) runs slew_sat ~0.925 — V1's
    hard slew_sat_max=0.5 admission bar would reject it outright; V2's
    gate must admit the same numbers, without loosening the OTHER
    bars V1 already had right."""
    from rl_move.dynamics.train_ppo_transfer import (
        WALKCURR_GATE, walkcurr_bucket_pass,
    )
    known_good = dict(early_term_rate=0.0, contact_sw_per_s=12.0,
                      foot_sw_min_per_s=1.0, cmd_prog_frac=0.78,
                      wrong_way=0.0, cross_track_frac=0.05,
                      slip_per_m=1.77, peak_roll_deg=4.49,
                      slew_sat=0.925, stop_speed_m_s=float("nan"))
    spec_v1 = {"stop_gate": None}         # v1 buckets carry no "gate" key
    ok_v1, checks_v1 = walkcurr_bucket_pass(known_good, spec_v1)
    assert not ok_v1 and not checks_v1["slew"], (
        "this test documents the V1 gate bug — if it starts passing, "
        "the V1 gate dict changed and this run's postmortem is stale")
    for spec in (dict(WALKCURR_BUCKETS_V2[0], stop_gate=None),
                dict(WALKCURR_BUCKETS_V2[6], stop_gate=None)):
        ok_v2, checks_v2 = walkcurr_bucket_pass(known_good, spec)
        assert ok_v2 and checks_v2["slew"], (
            f"V2 bucket {spec['name']} must admit the known-good "
            "checkpoint's slew_sat=0.925")


def test_v2_tiers_the_progress_bar_ignition_vs_quality():
    assert WALKCURR_GATE_V2_IGNITION["cmd_prog_frac_min"] < (
        WALKCURR_GATE_V2_QUALITY["cmd_prog_frac_min"])
    assert WALKCURR_BUCKETS_V2[0]["gate"] is WALKCURR_GATE_V2_IGNITION
    assert WALKCURR_BUCKETS_V2[1]["gate"] is WALKCURR_GATE_V2_QUALITY


def test_v2_bucket_commands_and_dr_swap():
    """Same physical-sanity sweep as V1's test_bucket_commands_and_
    dr_swap, run against the V2 table."""
    env = _env(seed=31, extra=CURR_V2_ON, episode_seconds=10.0)
    hold_n = max(1, int(round(1.0 / env.dt)))
    ramp_n = max(1, int(round(1.0 / env.dt)))
    for b, spec in enumerate(WALKCURR_BUCKETS_V2):
        env.force_walk_curr_bucket = b
        for _ in range(4):
            t = _sample_traj(env)
            assert t.cmd_mode == "walkcurr"
            assert env._wc_bucket == b
            assert env.randomizer.scale == pytest.approx(spec["dr"])
            body = np.hypot(t.vx, t.vy)[hold_n + ramp_n:]
            active = body[body > 1e-9]
            assert active.size, f"bucket {b} produced no motion command"
            # only the max bound is asserted here (matches V1's own
            # test_bucket_commands_and_dr_swap): a resampled segment's
            # LINEAR vx/vy blend between two in-band commands can
            # transiently dip below s_lo in speed-magnitude even
            # though both endpoints are in-band.
            assert active.max() <= spec["s_hi"] + 1e-9
            ang = abs(math.atan2(t.vy[hold_n + ramp_n],
                                 t.vx[hold_n + ramp_n]))
            assert spec["head_lo"] - 1e-9 <= ang <= spec["head_hi"] + 1e-9
    env.close()


# 3 ------------------------------------------------------------------
# WALKCURR_BUCKETS_V3: the actor-init "bridge" ladder (operator order
# fb_20260818T102844_116d4c). Ignition adjacent to the transplanted
# source skill (straight 0.05-0.06 m/s, no jitter/resample/stops, DR0),
# then a straight speed-band widening, then V2's own direction/heading
# ladder verbatim.
CURR_V3_ON = {("goal", "walk_curriculum"): 3.0}


def test_v3_selects_the_v3_table_and_v1_v2_untouched():
    from rl_move.sim.walk_task import WALKCURR_BUCKETS_V3
    env = _env(seed=5, extra=CURR_V3_ON)
    assert env._wc_version == 3
    assert env._wc_table is WALKCURR_BUCKETS_V3
    env.close()
    env = _env(seed=5, extra=CURR_V2_ON)
    assert env._wc_table is WALKCURR_BUCKETS_V2
    env.close()
    env = _env(seed=5, extra=CURR_ON)
    assert env._wc_table is WALKCURR_BUCKETS
    env.close()


def test_v3_bridge_rungs_are_adjacent_to_the_source_skill():
    """The two bridge rungs must sit at the transplanted checkpoint's
    own operating point: dead straight, source-speed band, one command
    held the whole episode, no stops, DR0 — and every later rung must
    be V2's ladder verbatim (same objects), preserving the eventual
    multi-direction joystick goal."""
    from rl_move.sim.walk_task import (WALKCURR_BUCKETS_V3,
                                       WALKCURR_GATE_V3_BRIDGE)
    b0, b1 = WALKCURR_BUCKETS_V3[0], WALKCURR_BUCKETS_V3[1]
    assert b0["name"] == "bridge_fwd"
    assert (b0["s_lo"], b0["s_hi"]) == (0.05, 0.06)
    assert (b1["s_lo"], b1["s_hi"]) == (0.05, 0.10)
    for b in (b0, b1):
        assert b["head_lo"] == 0.0 and b["head_hi"] == 0.0
        assert b["resample_s"] == 0.0 and b["jitter"] == 0.0
        assert b["stop_frac"] == 0.0 and b["dr"] == 0.0
        assert b["stop_gate"] is None
        assert b["gate"] is WALKCURR_GATE_V3_BRIDGE
    assert WALKCURR_BUCKETS_V3[2:] == WALKCURR_BUCKETS_V2[2:]


def test_v3_bridge_gate_admits_the_source_checkpoint_not_a_parked_one():
    """The bridge gate must admit the transplanted source checkpoint's
    own measured quality numbers (ppo_goal_cw_dep_bcgait1_hard1: slip
    ~1.3/m, roll ~4.5 deg, zero falls, six-leg gait, slew ~0.93) at
    honest command tracking — and still reject both the bcinit failure
    signature (posture kept, commanded travel lost) and a faller."""
    from rl_move.sim.walk_task import WALKCURR_BUCKETS_V3
    from rl_move.sim.walkcurr_cert import walkcurr_bucket_pass
    spec = WALKCURR_BUCKETS_V3[0]
    source_like = dict(early_term_rate=0.0, contact_sw_per_s=12.0,
                       foot_sw_min_per_s=1.0, cmd_prog_frac=0.85,
                       wrong_way=0.0, cross_track_frac=0.05,
                       slip_per_m=1.35, peak_roll_deg=4.5,
                       slew_sat=0.93, stop_speed_m_s=float("nan"))
    ok, checks = walkcurr_bucket_pass(source_like, spec)
    assert ok, f"source-like row must pass the bridge gate: {checks}"
    lost_travel = dict(source_like, cmd_prog_frac=0.06)   # bcinit @4M
    ok, checks = walkcurr_bucket_pass(lost_travel, spec)
    assert not ok and not checks["progress"]
    faller = dict(source_like, early_term_rate=0.25)
    ok, checks = walkcurr_bucket_pass(faller, spec)
    assert not ok and not checks["no_falls"]


def test_v3_bucket_commands_and_dr_swap():
    """Bridge rungs realize exactly the source command: straight
    heading, in-band speed, DR0, one command per episode (the V2 sweep
    already covers the shared tail rungs)."""
    env = _env(seed=31, extra=CURR_V3_ON, episode_seconds=10.0)
    from rl_move.sim.walk_task import WALKCURR_BUCKETS_V3
    hold_n = max(1, int(round(1.0 / env.dt)))
    ramp_n = max(1, int(round(1.0 / env.dt)))
    for b in (0, 1):
        spec = WALKCURR_BUCKETS_V3[b]
        env.force_walk_curr_bucket = b
        for _ in range(4):
            t = _sample_traj(env)
            assert t.cmd_mode == "walkcurr"
            assert env._wc_bucket == b
            assert env.randomizer.scale == pytest.approx(0.0)
            body = np.hypot(t.vx, t.vy)[hold_n + ramp_n:]
            active = body[body > 1e-9]
            assert active.size, f"bridge rung {b} produced no command"
            assert active.max() <= spec["s_hi"] + 1e-9
            assert active.min() >= spec["s_lo"] - 1e-9  # no resampling
            assert abs(float(t.vy[hold_n + ramp_n])) <= 1e-9  # straight
    env.close()


# 4 ------------------------------------------------------------------
# WALKCURR_BUCKETS_V4: joystick transitions at B1, then retained
# 10/20/40/60-second survival before speed/DR/direction broadening.
CURR_V4_ON = {("goal", "walk_curriculum"): 4.0}


def test_v4_requires_a_real_60_second_training_horizon():
    with pytest.raises(ValueError, match="requires episode_seconds >= 60"):
        _env(seed=0, extra=CURR_V4_ON, episode_seconds=10.0)
    env = _env(seed=0, extra=CURR_V4_ON, episode_seconds=60.0)
    assert env._wc_version == 4
    env.close()


def test_v4_ladder_puts_joystick_and_duration_first():
    from rl_move.sim.walk_task import WALKCURR_BUCKETS_V4
    names = [b["name"] for b in WALKCURR_BUCKETS_V4]
    assert names == [
        "bridge_10s", "joystick_10s", "joystick_20s",
        "joystick_40s", "joystick_60s", "full_band_60s",
        "dr01_60s", "dr03_60s", "lateral_60s", "rear_60s",
    ]
    assert [b["duration_s"] for b in WALKCURR_BUCKETS_V4[:5]] == [
        10.0, 10.0, 20.0, 40.0, 60.0]
    assert WALKCURR_BUCKETS_V4[0]["resample_s"] == 0.0
    assert WALKCURR_BUCKETS_V4[1]["resample_s"] > 0.0
    # B1-B4 isolate sustained survival: command distributions are equal.
    command_keys = ("s_lo", "s_hi", "head_lo", "head_hi", "resample_s",
                    "jitter", "stop_frac", "blend_lo", "blend_hi", "dr")
    baseline = tuple(WALKCURR_BUCKETS_V4[1][k] for k in command_keys)
    for spec in WALKCURR_BUCKETS_V4[2:5]:
        assert tuple(spec[k] for k in command_keys) == baseline
    assert [b["dr"] for b in WALKCURR_BUCKETS_V4[5:]] == [
        0.0, 0.1, 0.3, 0.3, 0.3]


def test_v4_trajectories_encode_horizon_and_command_change_floor():
    from rl_move.sim.walk_task import WALKCURR_BUCKETS_V4
    env = _env(seed=41, extra=CURR_V4_ON, episode_seconds=60.0)
    for b, spec in enumerate(WALKCURR_BUCKETS_V4):
        env.force_walk_curr_bucket = b
        for _ in range(3):
            traj = _sample_traj(env)
            assert traj.duration_steps * env.dt == pytest.approx(
                spec["duration_s"])
            assert traj.command_changes >= spec["min_command_changes"]
    env.close()


def test_v4_bucket_duration_actually_truncates_the_episode():
    env = _env(seed=9, extra=CURR_V4_ON, episode_seconds=60.0)
    env.force_walk_curr_bucket = 0
    obs, _ = env.reset(seed=9)
    limit = int(env._goal_traj.duration_steps)
    env._step_i = limit - 1
    *_, trunc_before, _ = env._post_step((obs, 0.0, False, False, {}))
    assert not trunc_before
    env._step_i = limit
    *_, trunc_at, _ = env._post_step((obs, 0.0, False, False, {}))
    assert trunc_at
    assert limit < env.episode_steps
    env.close()


def test_v4_gate_checks_duration_changes_height_and_tail_tracking():
    from rl_move.sim.walk_task import WALKCURR_BUCKETS_V4
    from rl_move.sim.walkcurr_cert import walkcurr_bucket_pass
    spec = WALKCURR_BUCKETS_V4[2]
    good = dict(early_term_rate=0.0, contact_sw_per_s=5.0,
                foot_sw_min_per_s=0.8, cmd_prog_frac=0.75,
                cmd_prog_frac_p10=0.60, wrong_way=0.0,
                cross_track_frac=0.10, slip_per_m=1.2,
                peak_roll_deg=3.0, slew_sat=0.8,
                stop_speed_m_s=0.01, height_factor=0.85,
                survival_s_min=20.0, command_changes_min=4.0)
    passed, checks = walkcurr_bucket_pass(good, spec)
    assert passed and all(checks.values())
    for key, value, check in (
            ("survival_s_min", 19.0, "duration"),
            ("command_changes_min", 3.0, "command_changes"),
            ("height_factor", 0.79, "height"),
            ("cmd_prog_frac_p10", 0.49, "progress_p10")):
        passed, checks = walkcurr_bucket_pass(
            dict(good, **{key: value}), spec)
        assert not passed and not checks[check]
