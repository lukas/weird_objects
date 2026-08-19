"""Unit tests for the 08-15 joystick-translation observability/pricing
keys (operator directive fb_20260815T114414, metric contract SIMPLIFIED
by fb_20260815T115650: no per-heading bins in training — fixed-direction
checks are held-out EVAL tools; headline is raw signed v_along only):

  - goal.walk_cmd_metrics (walk_task): raw SIGNED v_along / v_cross /
    cmd_speed / wrong_way info keys, emitted only
    on active-command ticks; with the default 0, those optional keys stay
    absent.
  - always-on walk direction telemetry: angular error on active-command
    ticks moving at least 0.01 m/s, plus valid-motion and wrong-way flags.
  - reward.term_cost_per_remaining_s (sim_env): early-fall horizon
    cost, k * remaining episode seconds added to the flat
    safety_termination_penalty on safety terminations only; default
    0.0 = legacy flat -10.
  - safety.walk_max_height_drop_mm (sim_env): optional walk-only belly
    collapse termination after safety.walk_height_grace_s; default off.

Fast (~seconds): short episodes, scripted actions, no PPO.
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
    walk_cmd_track_score,
    walk_direction_error_deg,
)

METRIC_KEYS = ("v_along_cmd_m_s", "v_cross_abs_m_s", "cmd_speed_m_s",
               "wrong_way")

FC_GOAL = {
    ("goal", "walk_yaw_cmd"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.03,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_heading_max_rad"): math.pi,
    ("goal", "walk_stop_frac"): 0.0,
    ("goal", "walk_cmd_resample_s"): 8.0,
    ("goal", "walk_cmd_resample_jitter"): 0.5,
    ("goal", "walk_cmd_blend_s_min"): 0.5,
    ("goal", "walk_cmd_blend_s_max"): 1.0,
    ("goal", "walk_yaw_zero_frac"): 1.0,
}


def _walk_env(seed=0, extra=None, episode_seconds=8.0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in {**FC_GOAL, **(extra or {})}.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _hold_action(env):
    return np.zeros(env.action_space.shape, dtype=np.float32)


def test_direct_command_score_orders_exact_park_cross_and_wrong_way():
    speed = 0.05
    exact = walk_cmd_track_score(speed, 0.0, speed, 0.0)[0]
    parked = walk_cmd_track_score(0.0, 0.0, speed, 0.0)[0]
    cross = walk_cmd_track_score(0.0, speed, speed, 0.0)[0]
    wrong = walk_cmd_track_score(-speed, 0.0, speed, 0.0)[0]
    assert (exact, parked, cross, wrong) == pytest.approx(
        (1.0, -1.0, -2.0, -3.0))
    assert walk_cmd_track_score(0.0, 0.0, 0.0, 0.0)[0] == 0.0
    assert walk_cmd_track_score(speed, 0.0, 0.0, 0.0)[0] < 0.0


def test_direction_error_exact_cross_wrong_and_stationary():
    speed = 0.05
    assert walk_direction_error_deg(
        speed, 0.0, speed, 0.0) == pytest.approx(0.0)
    assert walk_direction_error_deg(
        0.0, speed, speed, 0.0) == pytest.approx(90.0)
    assert walk_direction_error_deg(
        -speed, 0.0, speed, 0.0) == pytest.approx(180.0)
    assert walk_direction_error_deg(
        0.005, 0.0, speed, 0.0) is None
    assert walk_direction_error_deg(
        speed, 0.0, 0.0, 0.0) is None


def test_periodic_eval_reduces_direction_metrics_over_valid_motion():
    from rl_move.sim.train_ppo_sim import _rollout_stats

    class DirectionEnv:
        def reset(self):
            self.i = 0
            return np.zeros(1), {"goal_mode": "walk"}

        def step(self, _action):
            rows = (
                {"walk_direction_valid": 1.0,
                 "walk_direction_err_deg": 20.0,
                 "walk_direction_wrong_way": 0.0},
                {"walk_direction_valid": 0.0},
                {"walk_direction_valid": 1.0,
                 "walk_direction_err_deg": 100.0,
                 "walk_direction_wrong_way": 1.0},
            )
            info = {"roll_deg": 0.0, "pitch_deg": 0.0, **rows[self.i]}
            self.i += 1
            return np.zeros(1), 0.0, False, self.i == len(rows), info

    stats = _rollout_stats(DirectionEnv(), lambda _obs: np.zeros(1), 1)
    assert stats["walk_direction_valid_frac"] == pytest.approx(2.0 / 3.0)
    assert stats["walk_direction_err_deg_mean"] == pytest.approx(60.0)
    assert stats["walk_direction_err_deg_p90"] == pytest.approx(92.0)
    assert stats["walk_direction_wrong_way_frac"] == pytest.approx(0.5)


def _mode_traj(mode: str, seed: int = 0):
    env = _walk_env(seed=seed, episode_seconds=18.0, extra={
        ("goal", "walk_cmd_mode"): mode,
        ("goal", "walk_cmd_resample_s"): 2.0,
        ("goal", "walk_cmd_resample_jitter"): 0.0,
        ("goal", "walk_cmd_blend_s_min"): 0.0,
        ("goal", "walk_cmd_blend_s_max"): 0.0,
        ("goal", "walk_stop_frac"): 0.0,
        ("goal", "walk_cmd_sweep_period_s"): 12.0,
        ("goal", "walk_cmd_jitter_rad"): 0.25,
    })
    env.reset()
    traj = env._goal_traj
    env.close()
    return traj


def _cmd_at(traj, seconds: float) -> np.ndarray:
    i = int(round(seconds / 0.04))
    return np.array([traj.vx[i], traj.vy[i]], dtype=float)


def test_abrupt_flip_square_and_stop_go_modes():
    flip = _mode_traj("flip_180")
    square = _mode_traj("square")
    stop_go = _mode_traj("stop_go")
    samples = (3.0, 4.5, 6.5, 8.5)
    fv = [_cmd_at(flip, t) for t in samples]
    sv = [_cmd_at(square, t) for t in samples]
    zv = [_cmd_at(stop_go, t) for t in samples]
    for a, b in zip(fv, fv[1:]):
        assert np.dot(a, b) < -0.99 * np.linalg.norm(a) * np.linalg.norm(b)
    for a, b in zip(sv, sv[1:]):
        assert abs(np.dot(a, b)) < 1e-8
    assert [np.linalg.norm(v) > 1e-3 for v in zv] == [
        True, False, True, False]


def test_zero_blend_random_hold_switch_is_instant():
    traj = _mode_traj("random_hold")
    switch = int(round(4.0 / 0.04))
    old = np.array([traj.vx[switch - 1], traj.vy[switch - 1]])
    new = np.array([traj.vx[switch], traj.vy[switch]])
    after = np.array([traj.vx[switch + 1], traj.vy[switch + 1]])
    assert not np.allclose(old, new)
    assert np.allclose(new, after)


def test_sweep_circle_is_continuous_and_jitter_is_bounded():
    sweep = _mode_traj("sweep_circle")
    start = int(round(2.0 / 0.04))
    angles = np.unwrap(np.arctan2(sweep.vy[start:], sweep.vx[start:]))
    delta = np.diff(angles)
    assert np.max(np.abs(delta)) < 0.03
    assert abs(angles[-1] - angles[0]) > math.pi

    jitter = _mode_traj("jitter")
    vals = [_cmd_at(jitter, t) for t in (3.0, 4.5, 6.5, 8.5)]
    headings = np.unwrap([math.atan2(v[1], v[0]) for v in vals])
    assert np.max(np.abs(np.diff(headings))) <= 0.25 + 1e-9


def test_stress_mix_samples_each_concrete_mode():
    modes = {_mode_traj("stress_mix", seed).cmd_mode for seed in range(40)}
    assert modes == {"random_hold", "flip_180", "sweep_circle", "square",
                     "stop_go", "jitter"}


def test_metrics_keys_present_on_active_ticks_only():
    env = _walk_env(extra={("goal", "walk_cmd_metrics"): 1.0})
    env.reset()
    saw_active = False
    for step in range(100):        # 4 s: 1 s hold, 1 s ramp, then cmd
        _o, _r, term, trunc, info = env.step(_hold_action(env))
        goal = env._current_goal()
        s_ref = float(np.hypot(goal.vx_ref, goal.vy_ref))
        if s_ref > 1e-3:
            saw_active = True
            assert "walk_direction_valid" in info
            for k in METRIC_KEYS:
                assert k in info, f"missing {k} on active tick {step}"
            # fb_20260815T115650: per-heading bins are BANNED from the
            # training contract (held-out eval only) — none may leak.
            hbins = [k for k in info if k.startswith("v_along_hbin")]
            assert not hbins, f"per-heading bins are eval-only: {hbins}"
            # raw and signed: along on a held pose must be small, and
            # never clipped to a factor range
            assert abs(info["v_along_cmd_m_s"]) < 0.5
            assert info["wrong_way"] in (0.0, 1.0)
            assert abs(info["cmd_speed_m_s"] - s_ref) < 1e-9
            if info["walk_direction_valid"]:
                assert 0.0 <= info["walk_direction_err_deg"] <= 180.0
                assert info["walk_direction_wrong_way"] in (0.0, 1.0)
            else:
                assert "walk_direction_err_deg" not in info
                assert "walk_direction_wrong_way" not in info
        else:
            for k in METRIC_KEYS:
                assert k not in info, f"{k} leaked onto inactive tick"
            assert "walk_direction_valid" not in info
        if term or trunc:
            break
    env.close()
    assert saw_active, "no active-command ticks seen in 4 s"


def test_optional_signed_metrics_default_off_but_direction_is_on():
    env = _walk_env()              # walk_cmd_metrics unset -> 0
    env.reset()
    saw_active = False
    for step in range(80):
        _o, _r, term, trunc, info = env.step(_hold_action(env))
        goal = env._current_goal()
        active = float(np.hypot(goal.vx_ref, goal.vy_ref)) > 1e-3
        for k in info:
            assert not k.startswith("v_along"), k
            assert k not in METRIC_KEYS, k
        if active:
            saw_active = True
            assert "walk_direction_valid" in info
        if term or trunc:
            break
    env.close()
    assert saw_active


DIR_KEYS = ("walk_dir_valid", "walk_direction_err_deg")


def test_direction_telemetry_keys_and_bounds():
    """fb_20260815T192912: walk_direction_err_deg / walk_dir_valid ride
    the same walk_cmd_metrics gate; valid requires real motion
    (speed >= 5 mm/s), the deg key exists only on valid ticks, and it
    agrees with the signed along-speed key (wrong_way <=> err > 90)."""
    env = _walk_env(extra={("goal", "walk_cmd_metrics"): 1.0})
    env.reset()
    saw_active = False
    for step in range(100):
        _o, _r, term, trunc, info = env.step(_hold_action(env))
        goal = env._current_goal()
        s_ref = float(np.hypot(goal.vx_ref, goal.vy_ref))
        if s_ref > 1e-3:
            saw_active = True
            assert info["walk_dir_valid"] in (0.0, 1.0)
            if info["walk_dir_valid"] == 1.0:
                err = info["walk_direction_err_deg"]
                assert 0.0 <= err <= 180.0
                if info["wrong_way"] == 1.0:
                    assert err > 90.0
                else:
                    assert err <= 90.0
            else:
                assert "walk_direction_err_deg" not in info
        else:
            for k in DIR_KEYS:
                assert k not in info, f"{k} leaked onto inactive tick"
        if term or trunc:
            break
    env.close()
    assert saw_active, "no active-command ticks seen in 4 s"


def test_direction_telemetry_absent_when_metrics_off():
    env = _walk_env()              # walk_cmd_metrics unset -> 0
    env.reset()
    for step in range(80):
        _o, _r, term, trunc, info = env.step(_hold_action(env))
        for k in DIR_KEYS:
            assert k not in info, k
        if term or trunc:
            break
    env.close()


def test_fullcircle_sampler_contract():
    """Headings cover the circle (>=6 of 8 octants over 32 resets),
    every first-segment speed is in [0.03, 0.06], and wz is
    identically zero with walk_yaw_zero_frac=1 (obs channel kept)."""
    octants = set()
    for seed in range(32):
        env = _walk_env(seed=seed)
        env.reset()
        traj = env._goal_traj
        t3 = int(round(3.0 / env.dt))   # inside the first segment
        vx, vy = float(traj.vx[t3]), float(traj.vy[t3])
        sp = math.hypot(vx, vy)
        assert 0.03 - 1e-9 <= sp <= 0.06 + 1e-9, (
            f"seed {seed}: first-segment speed {sp}")
        octants.add(int((math.atan2(vy, vx) + math.pi)
                        / (math.pi / 4.0)) % 8)
        assert traj.wz is not None, "yaw obs channel must stay"
        assert float(np.max(np.abs(traj.wz))) == 0.0, (
            f"seed {seed}: wz not identically zero")
        env.close()
    assert len(octants) >= 6, (
        f"headings do not cover the circle: octants {sorted(octants)}")


def test_term_cost_default_off_is_flat_penalty():
    env = _walk_env()
    env.reset()
    bad = np.full(env.action_space.shape, np.nan, dtype=np.float32)
    _o, r, term, _tr, info = env.step(bad)
    assert term
    # legacy: flat safety_termination_penalty only (walk shaping may
    # add income on the tick; check the part, not the total)
    assert info["reward_termination"] == -10.0
    env.close()


def test_term_cost_scales_with_remaining_horizon():
    k = 12.0
    env = _walk_env(extra={("reward", "term_cost_per_remaining_s"): k},
                    episode_seconds=60.0)
    env.reset()
    n_fall = 150                    # fall at 6 s of 60 s
    for step in range(n_fall - 1):
        _o, _r, term, trunc, _i = env.step(_hold_action(env))
        assert not (term or trunc), f"unexpected end at {step}"
    bad = np.full(env.action_space.shape, np.nan, dtype=np.float32)
    _o, _r, term, _tr, info = env.step(bad)
    assert term
    expect = -(10.0 + k * max(env.episode_steps - n_fall, 0) * env.dt)
    assert abs(info["reward_termination"] - expect) < 1e-6, (
        f"{info['reward_termination']} != {expect}")
    env.close()


def test_walk_low_height_termination_is_opt_in_and_charges_horizon():
    legacy = _walk_env(episode_seconds=60.0)
    legacy.reset()
    legacy._z0 += 0.10
    _o, _r, term, _tr, info = legacy.step(_hold_action(legacy))
    assert not term, info.get("termination_reason")
    legacy.close()

    k = 12.0
    env = _walk_env(extra={
        ("safety", "walk_max_height_drop_mm"): 90.0,
        ("safety", "walk_height_grace_s"): 0.0,
        ("reward", "term_cost_per_remaining_s"): k,
    }, episode_seconds=60.0)
    env.reset()
    env._z0 += 0.10
    _o, _r, term, _tr, info = env.step(_hold_action(env))
    assert term
    assert info["termination_reason"] == "walk_low_height"
    expect = -(10.0 + k * (env.episode_steps - 1) * env.dt)
    assert info["reward_termination"] == pytest.approx(expect)
    env.close()


# --------------------------------------------------------------------------
# Bounded terminal cost (08-17, operator-approved fb_20260817T005114
# item 5): reward.term_cost_max caps the ADDED horizon component of
# the early-fall charge — the uncapped form reached ~-730 on an early
# 60 s fall and the critic never learned to predict the cliff
# (EV ~0 through 40M on cw-arch-joystick-long-scratch3). The flat
# safety_termination_penalty is never capped; default 0 = uncapped
# legacy bit-exact.

def test_term_cost_max_caps_horizon_charge():
    k, cap = 12.0, 60.0
    env = _walk_env(extra={
        ("reward", "term_cost_per_remaining_s"): k,
        ("reward", "term_cost_max"): cap,
    }, episode_seconds=60.0)
    env.reset()
    n_fall = 150                    # fall at 6 s: uncapped would be -658
    for _ in range(n_fall - 1):
        env.step(_hold_action(env))
    bad = np.full(env.action_space.shape, np.nan, dtype=np.float32)
    _o, _r, term, _tr, info = env.step(bad)
    assert term
    uncapped = k * max(env.episode_steps - n_fall, 0) * env.dt
    assert uncapped > cap           # the cap must actually bind here
    assert info["reward_termination"] == pytest.approx(-(10.0 + cap))
    env.close()


def test_term_cost_max_inert_when_not_binding():
    k, cap = 12.0, 1e6
    env = _walk_env(extra={
        ("reward", "term_cost_per_remaining_s"): k,
        ("reward", "term_cost_max"): cap,
    }, episode_seconds=60.0)
    env.reset()
    bad = np.full(env.action_space.shape, np.nan, dtype=np.float32)
    _o, _r, term, _tr, info = env.step(bad)
    assert term
    expect = -(10.0 + k * max(env.episode_steps - 1, 0) * env.dt)
    assert info["reward_termination"] == pytest.approx(expect)
    env.close()


# --------------------------------------------------------------------------
# stress_mix command curriculum (08-17, fb_20260817T005114 item 7):
# goal.walk_cmd_stage restricts which schedule families stress_mix may
# draw (cumulative tiers; stage 0 additionally forces heading 0 =
# pure forward/back). Default -1 must leave stress_mix draw-stream
# bit-exact; transitions stay instantaneous (blend cfg untouched).

def _stage_traj(stage, seed):
    env = _walk_env(seed=seed, episode_seconds=18.0, extra={
        ("goal", "walk_cmd_mode"): "stress_mix",
        ("goal", "walk_cmd_stage"): stage,
        ("goal", "walk_cmd_resample_s"): 2.0,
        ("goal", "walk_cmd_resample_jitter"): 0.0,
        ("goal", "walk_cmd_blend_s_min"): 0.0,
        ("goal", "walk_cmd_blend_s_max"): 0.0,
        ("goal", "walk_stop_frac"): 0.0,
    })
    env.reset()
    traj = env._goal_traj
    env.close()
    return traj


def test_cmd_stage_off_is_bitexact_stress_mix():
    for seed in range(4):
        a = _stage_traj(-1.0, seed)
        b = _stage_traj(-1.0, seed)   # determinism guard
        env = _walk_env(seed=seed, episode_seconds=18.0, extra={
            ("goal", "walk_cmd_mode"): "stress_mix",
            ("goal", "walk_cmd_resample_s"): 2.0,
            ("goal", "walk_cmd_resample_jitter"): 0.0,
            ("goal", "walk_cmd_blend_s_min"): 0.0,
            ("goal", "walk_cmd_blend_s_max"): 0.0,
            ("goal", "walk_stop_frac"): 0.0,
        })
        env.reset()
        c = env._goal_traj
        env.close()
        for other in (b, c):
            assert a.cmd_mode == other.cmd_mode
            assert np.array_equal(a.vx, other.vx)
            assert np.array_equal(a.vy, other.vy)


def test_cmd_stage0_is_pure_forward_back():
    from rl_move.sim.walk_task import WALK_CMD_STAGE_FAMILIES
    seen = set()
    for seed in range(10):
        traj = _stage_traj(0.0, seed)
        seen.add(traj.cmd_mode)
        assert traj.cmd_mode in WALK_CMD_STAGE_FAMILIES[0], traj.cmd_mode
        assert np.all(np.abs(traj.vy) < 1e-12), (
            f"seed {seed}: stage 0 must command the fwd/back axis only")
    assert len(seen) == 2, f"stage-0 families not both drawn: {seen}"


def test_cmd_stage1_adds_heading_families_cumulatively():
    from rl_move.sim.walk_task import WALK_CMD_STAGE_FAMILIES
    allowed = set(WALK_CMD_STAGE_FAMILIES[0]) | set(
        WALK_CMD_STAGE_FAMILIES[1])
    seen = set()
    for seed in range(16):
        traj = _stage_traj(1.0, seed)
        seen.add(traj.cmd_mode)
        assert traj.cmd_mode in allowed, traj.cmd_mode
    assert seen & set(WALK_CMD_STAGE_FAMILIES[1]), (
        f"stage 1 never drew a new family: {seen}")
    assert "jitter" not in seen


def test_cmd_stage_clips_to_full_mix():
    from rl_move.sim.walk_task import WALK_CMD_SCHEDULES
    seen = {_stage_traj(7.0, seed).cmd_mode for seed in range(24)}
    assert seen <= set(WALK_CMD_SCHEDULES)
    assert "jitter" in seen, f"full mix never drew jitter: {seen}"
