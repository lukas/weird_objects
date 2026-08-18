"""Tests for the all-GPU walk-curriculum port (operator order
2026-08-18, fb_20260818T065930_03b422 — cw-dynrep-criticD-walkcurr3):

  1. fail-closed backend guard: train_ppo_mjx._assert_gpu_physics
     REFUSES a SubprocVecEnv/DummyVecEnv (CPU C-MuJoCo physics) and a
     non-warp impl — an "all-GPU" launch cannot silently train CPU
     physics again;
  2. goal.walk_pure: pure-walk diet fixed at env CONSTRUCTION (no
     post-construction set_goal_mix needed); default OFF is bit-exact
     (p_walk stays 0.70);
  3. in-env walk probe (goal.walk_probe / walk_probe_on): default OFF
     and side-effect-free; when ON it reproduces the eval_task metric
     formulas from the same fields at the same times (external mirror
     comparison on a live C-env episode) and never changes obs/reward/
     termination;
  4. reward.term_penalty: default OFF bit-exact; when set, exactly the
     configured charge on term (not trunc) — the in-env twin of
     train_ppo_transfer's TRAINING-ONLY _term_penalty_wrapper;
  5. walkcurr_cert shared module: aggregation nan rules match
     eval_task's, failed_probe_row can only FAIL a gate, and the
     transfer trainer's re-imported names are the same objects;
  6. flush_reset_pools exists on both MJX vec envs (pool-staleness
     lever for admission changes).
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
from rl_move.sim.servo_model import SimServoParams  # noqa: E402
from rl_move.sim.walk_task import SimHexapodJointWalkEnv  # noqa: E402
from rl_move.sim.walkcurr_cert import (  # noqa: E402
    WALKCURR_GATE, aggregate_walk_probe, failed_probe_row,
    walkcurr_bucket_pass,
)


def _env(seed=0, extra=None, episode_seconds=8.0, randomize=False,
         dr_scale=0.0):
    cfg = load_config()
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    return SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=randomize,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)


# -- 1. fail-closed backend guard ------------------------------------


def test_gpu_physics_guard_refuses_cpu_vecenv():
    pytest.importorskip("stable_baselines3")
    import gymnasium as gym
    from stable_baselines3.common.vec_env import DummyVecEnv

    from rl_move.sim.train_ppo_mjx import _assert_gpu_physics

    venv = DummyVecEnv([lambda: gym.make("Pendulum-v1")])
    with pytest.raises(SystemExit, match="not MjxVecEnv"):
        _assert_gpu_physics(venv, "warp")
    venv.close()


def test_gpu_physics_guard_refuses_wrapped_cpu_vecenv():
    pytest.importorskip("stable_baselines3")
    import gymnasium as gym
    from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

    from rl_move.sim.train_ppo_mjx import _assert_gpu_physics

    venv = VecMonitor(DummyVecEnv([lambda: gym.make("Pendulum-v1")]))
    with pytest.raises(SystemExit, match="not MjxVecEnv"):
        _assert_gpu_physics(venv, "warp")
    venv.close()


def test_gpu_physics_guard_refuses_non_warp_impl():
    from rl_move.sim.train_ppo_mjx import _assert_gpu_physics

    from rl_move.sim import mjx_vec_env

    # bypass __init__ (no GPU here); the isinstance + impl checks are
    # what this test pins
    inst = object.__new__(mjx_vec_env.MjxVecEnv)
    inst.num_envs = 4
    with pytest.raises(SystemExit, match="impl"):
        _assert_gpu_physics(inst, None)


# -- 2. goal.walk_pure ------------------------------------------------


def test_walk_pure_off_is_legacy_default():
    env = _env()
    assert float(env._goal_gen.p_walk) == pytest.approx(0.70)
    env.close()


def test_walk_pure_sets_pure_diet_at_construction():
    env = _env(extra={("goal", "walk_pure"): 1.0})
    gen = env._goal_gen
    assert float(gen.p_walk) == 1.0
    for name in dir(gen):
        if name.startswith("p_") and name != "p_walk" \
                and isinstance(getattr(gen, name), (int, float)):
            assert float(getattr(gen, name)) == 0.0, name
    env.close()


def test_walk_pure_satisfies_walkcurr_admission():
    # the walkcurr episode guard demands p_walk == 1.0 at reset —
    # goal.walk_pure must satisfy it with zero post-construction pokes
    env = _env(extra={("goal", "walk_pure"): 1.0,
                      ("goal", "walk_curriculum"): 2.0})
    obs, _ = env.reset(seed=3)
    assert env._wc_bucket == 0    # frontier-only at start
    env.close()


# -- 3. in-env walk probe ---------------------------------------------


def test_probe_default_off():
    env = _env()
    assert env.walk_probe_on is False
    obs, _ = env.reset(seed=1)
    for _ in range(5):
        obs, r, term, trunc, info = env.step(
            np.zeros(env.action_space.shape, dtype=np.float32))
        assert "walk_probe" not in info
    assert env._wp is None
    env.close()


def _external_mirror_episode(env, seed, n_ticks=60):
    """Step a probe-ON env while measuring a metric subset externally
    with the exact eval_task reads (pad-body feet), then return
    (external dict, probe summary or None, rewards)."""
    obs, _ = env.reset(seed=seed)
    tr = env._tilt_ref0
    sat_limit = 0.98 * env.safety.max_dq
    prev_cmd = env.safety._last_safe.copy()
    prev_on = None
    peak_roll = 0.0
    sw = sat_jt = on_ticks = 0
    h_sum = 0.0
    ret = 0.0
    n = 0
    rng = np.random.default_rng(seed)
    info = {}
    for _ in range(n_ticks):
        act = rng.uniform(-0.3, 0.3,
                          env.action_space.shape).astype(np.float32)
        obs, r, term, trunc, info = env.step(act)
        ret += float(r)
        n += 1
        st = env._state
        peak_roll = max(peak_roll, abs(st.imu_roll - tr[0]))
        cmd = env.safety._last_safe
        sat_jt += int(np.sum(np.abs(cmd - prev_cmd) >= sat_limit))
        prev_cmd = cmd.copy()
        on = []
        for f in range(6):
            adr = env._touch_adr[f]
            is_on = bool(adr >= 0
                         and float(env.data.sensordata[adr]) > 0.5)
            if prev_on is not None and is_on != prev_on[f]:
                sw += 1
            on.append(is_on)
            on_ticks += int(is_on)
        prev_on = on
        h_sum += float(env.data.xpos[env._chassis_bid, 2])
        if term or trunc:
            break
    ext = dict(
        peak_roll_deg=peak_roll * 180.0 / np.pi,
        contact_sw_per_s=sw / max(n * env.dt, 1e-9),
        slew_sat=sat_jt / max(n * 18, 1),
        mean_h_m=h_sum / max(n, 1),
        ep_len=float(n), ret=ret)
    return ext, info.get("walk_probe"), (term, trunc)


def test_probe_matches_external_mirror():
    # probe ON via cfg; run until the episode ends (short horizon)
    env = _env(extra={("goal", "walk_pure"): 1.0,
                      ("goal", "walk_probe"): 1.0},
               episode_seconds=2.0)
    assert env.walk_probe_on is True
    ext, probe, (term, trunc) = _external_mirror_episode(
        env, seed=7, n_ticks=10_000)
    assert term or trunc, "episode must end within its horizon"
    assert probe is not None, "terminal info must carry walk_probe"
    assert probe["ep_len"] == ext["ep_len"]
    assert probe["return"] == pytest.approx(ext["ret"], abs=1e-9)
    assert probe["peak_roll_deg"] == pytest.approx(
        ext["peak_roll_deg"], abs=1e-9)
    assert probe["contact_sw_per_s"] == pytest.approx(
        ext["contact_sw_per_s"], abs=1e-9)
    assert probe["slew_sat"] == pytest.approx(ext["slew_sat"], abs=1e-9)
    assert probe["mean_h_m"] == pytest.approx(ext["mean_h_m"], abs=1e-9)
    assert probe["early_term"] == float(term)
    assert 0.0 <= probe["duty_factor"] <= 1.0
    env.close()


def test_probe_is_side_effect_free():
    # identical seed/actions with probe ON vs OFF -> identical obs,
    # rewards, terminations (measurement only)
    outs = []
    for probe in (0.0, 1.0):
        env = _env(extra={("goal", "walk_pure"): 1.0,
                          ("goal", "walk_probe"): probe},
                   episode_seconds=2.0)
        obs, _ = env.reset(seed=11)
        rng = np.random.default_rng(11)
        tr = []
        for _ in range(30):
            act = rng.uniform(-0.3, 0.3,
                              env.action_space.shape).astype(np.float32)
            obs, r, term, trunc, info = env.step(act)
            tr.append((obs.copy(), float(r), term, trunc))
            if term or trunc:
                break
        outs.append(tr)
        env.close()
    assert len(outs[0]) == len(outs[1])
    for (o0, r0, t0, u0), (o1, r1, t1, u1) in zip(*outs):
        assert np.array_equal(o0, o1)
        assert r0 == r1 and t0 == t1 and u0 == u1


# -- 4. reward.term_penalty -------------------------------------------


def test_term_penalty_charges_exactly_on_term():
    envs = {}
    for key, extra in (("off", {}),
                       ("on", {("reward", "term_penalty"): 30.0})):
        env = _env(extra={("goal", "walk_pure"): 1.0, **extra})
        env.reset(seed=5)
        env.step(np.zeros(env.action_space.shape, dtype=np.float32))
        envs[key] = env
    # synthetic terminal tick through the real _post_step stack:
    # identical state in both envs, so any difference == the charge
    base = dict(o=None)
    _, r_off_term, *_ = envs["off"]._post_step(
        (np.zeros(1), 0.0, True, False, {}))
    _, r_on_term, *_ = envs["on"]._post_step(
        (np.zeros(1), 0.0, True, False, {}))
    _, r_off_trunc, *_ = envs["off"]._post_step(
        (np.zeros(1), 0.0, False, True, {}))
    _, r_on_trunc, *_ = envs["on"]._post_step(
        (np.zeros(1), 0.0, False, True, {}))
    assert r_off_term - r_on_term == pytest.approx(30.0, abs=1e-9)
    assert r_on_trunc == pytest.approx(r_off_trunc, abs=1e-9)
    for env in envs.values():
        env.close()


# -- 5. shared cert module --------------------------------------------


def test_aggregate_nan_rules_match_eval_task():
    good = failed_probe_row()
    good.update({k: 1.0 for k in good if k not in ("early_term",)})
    good["early_term"] = 0.0
    nanrow = failed_probe_row()   # all-nan metrics, early_term 1
    agg = aggregate_walk_probe([good, nanrow])
    # command-conditional keys: nanmean (one good row -> its value)
    assert agg["stop_speed_m_s"] == pytest.approx(1.0)
    assert agg["wrong_way"] == pytest.approx(1.0)
    assert agg["cross_track_frac"] == pytest.approx(1.0)
    # historical keys: plain mean (nan poisons -> nan, as eval_task)
    assert np.isnan(agg["cmd_prog_frac"])
    assert agg["early_term_rate"] == pytest.approx(0.5)


def test_failed_probe_row_fails_every_gate():
    m = aggregate_walk_probe([failed_probe_row()])
    passed, checks = walkcurr_bucket_pass(
        m, dict(name="x", stop_gate=0.015), WALKCURR_GATE)
    assert not passed
    assert not checks["no_falls"]
    assert not checks["progress"]


def test_transfer_reimports_are_same_objects():
    from rl_move.dynamics import train_ppo_transfer as tpt
    from rl_move.sim import walkcurr_cert as wc
    assert tpt.WALKCURR_GATE is wc.WALKCURR_GATE
    assert tpt.walkcurr_bucket_pass is wc.walkcurr_bucket_pass
    assert tpt.WalkCurrController is wc.WalkCurrController


# -- 6. pool flush hooks ----------------------------------------------


def test_flush_reset_pools_exists_on_both_vec_envs():
    from rl_move.sim.mjx_sharded_vec_env import MjxShardedVecEnv
    from rl_move.sim.mjx_vec_env import MjxVecEnv
    assert callable(getattr(MjxVecEnv, "flush_reset_pools"))
    assert callable(getattr(MjxShardedVecEnv, "flush_reset_pools"))
