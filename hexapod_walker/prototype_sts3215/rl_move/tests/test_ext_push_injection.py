"""dr.ext_push_* mid-episode external push (AMP brief §7.4/§9.3, M3
push-recovery curriculum).

Contract under test:
- default OFF: ext_push_prob=0.0 draws nothing and keeps the legacy rng
  stream bit-exact (guarded-draw convention, same as fault/walk_push);
- sampled dose (peak force, duration, start delay, world-frame
  direction) stays within the configured menu;
- .scaled(s) shrinks the probability, not the dose menu (same
  convention as tipped/rock/kick/push/fault);
- sim_env._ext_push_force_n() is zero outside its [start, start+dur)
  window and outside walk mode, and inside the window returns a
  half-sine-ramped force along the drawn direction;
- end-to-end: a walk-mode HexapodSimEnv with ext_push_prob=1.0 resets
  and steps finite, actually writes a nonzero horizontal xfrc during
  the push window (and only then), and the push measurably perturbs
  the chassis trajectory vs. a push-off twin from the identical seed.
"""
import dataclasses
import math

import numpy as np
import pytest

from rl_move.sim.domain_rand import DomainRandomizer, EpisodeRandomization, RandRanges


def _er(**kw) -> EpisodeRandomization:
    """A neutral EpisodeRandomization with only ext_push fields set."""
    base = DomainRandomizer(scale=0.0).sample(np.random.default_rng(0))
    return dataclasses.replace(base, **kw)


def _walk_env(*, ext_push_prob: float = 0.0, seed: int = 0):
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    if ext_push_prob:
        cfg.setdefault("dr", {})["ext_push_prob"] = ext_push_prob
    env = SimHexapodJointWalkEnv(cfg, seed=seed, randomize=True)
    gen = env._goal_gen
    for name in dir(gen):
        if name.startswith("p_") and isinstance(getattr(gen, name),
                                                 (int, float)):
            setattr(gen, name, 0.0)
    gen.p_walk = 1.0
    return env


# ---------------------------------------------------------------- default off

def test_ext_push_prob_zero_draws_nothing_and_keeps_rng_stream():
    r1 = RandRanges()
    r2 = RandRanges(ext_push_n=(50.0, 60.0), ext_push_dur_s=(1.0, 2.0))
    s1 = DomainRandomizer(r1).sample(np.random.default_rng(9))
    s2 = DomainRandomizer(r2).sample(np.random.default_rng(9))
    assert s1.ext_push_peak_n == 0.0 and s2.ext_push_peak_n == 0.0
    assert s1.summary() == s2.summary()
    np.testing.assert_array_equal(s1.kp_scale, s2.kp_scale)
    np.testing.assert_array_equal(s1.start_offset_rad, s2.start_offset_rad)


def test_ext_push_force_n_zero_when_off():
    assert _er().ext_push_peak_n == 0.0


# ------------------------------------------------------------------- sampling

def test_sample_draws_dose_within_configured_ranges():
    r = RandRanges(ext_push_prob=1.0, ext_push_n=(10.0, 25.0),
                    ext_push_dur_s=(0.15, 0.4),
                    ext_push_start_s=(1.5, 9.0))
    dr = DomainRandomizer(r)
    rng = np.random.default_rng(3)
    dirs = []
    for _ in range(200):
        er = dr.sample(rng)
        assert er.ext_push_peak_n > 0.0
        assert 10.0 <= er.ext_push_peak_n <= 25.0
        assert 0.15 <= er.ext_push_dur_s <= 0.4
        assert 1.5 <= er.ext_push_start_s <= 9.0
        assert 0.0 <= er.ext_push_dir_rad < 2.0 * math.pi
        dirs.append(er.ext_push_dir_rad)
    # direction really is randomized (not pinned to one value/axis)
    assert np.std(dirs) > 1.0


def test_scaled_shrinks_probability_not_dose():
    r = RandRanges(ext_push_prob=0.6, ext_push_n=(10.0, 25.0))
    r2 = r.scaled(0.5)
    assert r2.ext_push_prob == pytest.approx(0.3)
    assert r2.ext_push_n == (10.0, 25.0)
    assert r2.ext_push_dur_s == r.ext_push_dur_s
    assert r2.ext_push_start_s == r.ext_push_start_s


# --------------------------------------------------------- force-window math

def test_force_zero_when_peak_or_duration_zero():
    env = _walk_env()
    env.reset(seed=0)
    env._ep_rand = _er(ext_push_peak_n=0.0, ext_push_dur_s=0.0)
    assert env._ext_push_force_n() == (0.0, 0.0)


def test_force_only_nonzero_inside_window_and_along_direction():
    env = _walk_env()
    env.reset(seed=0)
    env._goal_traj.mode = "walk"
    env._ep_rand = _er(ext_push_peak_n=20.0, ext_push_dur_s=0.2,
                       ext_push_start_s=1.0, ext_push_dir_rad=0.0)
    dt = env.dt
    # before the window: zero
    env._step_i = int(0.5 / dt)
    assert env._ext_push_force_n() == (0.0, 0.0)
    # mid-window: nonzero, pure +x (dir_rad=0), magnitude <= peak
    env._step_i = int(1.1 / dt)
    fx, fy = env._ext_push_force_n()
    assert fx > 0.0 and abs(fy) < 1e-6
    assert fx <= 20.0 + 1e-6
    # after the window: zero again
    env._step_i = int(2.0 / dt)
    assert env._ext_push_force_n() == (0.0, 0.0)


def test_force_only_fires_in_walk_mode():
    env = _walk_env()
    env.reset(seed=0)
    env._ep_rand = _er(ext_push_peak_n=20.0, ext_push_dur_s=0.2,
                       ext_push_start_s=1.0, ext_push_dir_rad=0.3)
    env._step_i = int(1.1 / env.dt)
    env._goal_traj.mode = "walk"
    assert env._ext_push_force_n() != (0.0, 0.0)
    env._goal_traj.mode = "hold"
    assert env._ext_push_force_n() == (0.0, 0.0)


def test_direction_components_match_angle():
    env = _walk_env()
    env.reset(seed=0)
    env._goal_traj.mode = "walk"
    ang = 0.9
    env._ep_rand = _er(ext_push_peak_n=15.0, ext_push_dur_s=0.3,
                       ext_push_start_s=0.5, ext_push_dir_rad=ang)
    env._step_i = int(0.65 / env.dt)   # window mid-point
    fx, fy = env._ext_push_force_n()
    mag = math.hypot(fx, fy)
    assert mag > 0.0
    assert fx == pytest.approx(mag * math.cos(ang), abs=1e-6)
    assert fy == pytest.approx(mag * math.sin(ang), abs=1e-6)


# ----------------------------------------------------------------- end to end

def test_env_reset_and_step_finite_with_push_on():
    env = _walk_env(ext_push_prob=1.0, seed=11)
    try:
        obs, _ = env.reset(seed=11)
        er = env._ep_rand
        assert er is not None and er.ext_push_peak_n > 0.0
        for _ in range(60):
            obs, rew, term, trunc, info = env.step(
                np.zeros(env.n_act, dtype=np.float32))
            assert np.all(np.isfinite(obs)) and np.isfinite(rew)
            if term or trunc:
                break
    finally:
        env.close()


def test_xfrc_nonzero_only_during_push_window():
    env = _walk_env(ext_push_prob=1.0, seed=11)
    try:
        env.reset(seed=11)
        er = env._ep_rand
        assert er.ext_push_peak_n > 0.0
        cbid = env._chassis_bid
        saw_nonzero = False
        saw_zero_after = False
        n_steps = int((er.ext_push_start_s + er.ext_push_dur_s) / env.dt) + 20
        for _ in range(n_steps):
            env.step(np.zeros(env.n_act, dtype=np.float32))
            t = env._step_i * env.dt
            fxy = env.data.xfrc_applied[cbid, 0:2].copy()
            if er.ext_push_start_s <= t < er.ext_push_start_s + er.ext_push_dur_s:
                if np.linalg.norm(fxy) > 1e-6:
                    saw_nonzero = True
            elif t >= er.ext_push_start_s + er.ext_push_dur_s + env.dt:
                if np.linalg.norm(fxy) < 1e-9:
                    saw_zero_after = True
        assert saw_nonzero, "push window never wrote a nonzero xfrc force"
        assert saw_zero_after, "xfrc force did not clear after the window"
    finally:
        env.close()


def test_push_measurably_perturbs_chassis_vs_push_off_twin():
    # Same seed, same everything, only ext_push_prob differs -> the
    # push-on twin's chassis trajectory must diverge from the push-off
    # twin's by more than integrator noise once the pulse has fired.
    env_on = _walk_env(ext_push_prob=1.0, seed=21)
    env_off = _walk_env(ext_push_prob=0.0, seed=21)
    try:
        env_on.reset(seed=21)
        env_off.reset(seed=21)
        er = env_on._ep_rand
        assert er.ext_push_peak_n > 0.0
        n_steps = int((er.ext_push_start_s + er.ext_push_dur_s) / env_on.dt) + 10
        a = np.zeros(env_on.n_act, dtype=np.float32)
        for _ in range(n_steps):
            env_on.step(a)
            env_off.step(a)
        p_on = env_on.data.xpos[env_on._chassis_bid, :2].copy()
        p_off = env_off.data.xpos[env_off._chassis_bid, :2].copy()
        assert np.linalg.norm(p_on - p_off) > 1e-4
    finally:
        env_on.close()
        env_off.close()
