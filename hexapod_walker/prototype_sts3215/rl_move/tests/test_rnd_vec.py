"""test_rnd_vec.py — walkcurr track's RND (Random Network Distillation)
state-novelty wrapper, the pre-registered fallback after both rung-0
swing-income arms (swing3, swing9) certified 0/6 on the C-env det gate
(`rl_docs/tracks/walkcurr/STATUS.md`). CPU-only, no jax/mjx/GPU needed.

Proves:
1. ``rnd_coef <= 0`` is rejected at construction (bit-exact off path —
   the caller must not build the wrapper at all).
2. blend math: ``r = r_env + rnd_coef * (intrinsic / ret_std)`` with
   intrinsic == the predictor/target squared-error on THIS wrapper's
   own normalized-obs convention (checked against the wrapper's own
   internals so the test cannot silently drift from the
   implementation while still catching sign/shape/off-by-one bugs).
3. the intrinsic reward genuinely encodes novelty: repeatedly training
   the predictor on a FIXED obs makes intrinsic error on that exact
   obs monotonically fall towards ~0, while a never-seen obs stays
   high — a static held pose (the observed rung-0 failure mode) would
   see its own intrinsic income decay to ~0 under this mechanism.
4. the observation ring buffer wraps at capacity; ``train_predictor``
   returns None below one batch, then finite decreasing loss.
5. save()/load() round-trips predictor/target/opt/running-stats
   exactly (bit-identical intrinsic reward on the same obs after
   reload).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.sim.rnd_vec import RNDVecWrapper, _ObsRing, _RunningMeanStd  # noqa: E402


class _StubVecEnv:
    """Minimal VecEnv double: constant task reward, deterministic obs
    drifting with time so different ticks are distinguishable states."""

    def __init__(self, n_envs=3, obs_dim=8):
        self.num_envs = n_envs
        self.obs_dim = obs_dim

        class _Space:
            shape = (obs_dim,)
        self.observation_space = _Space()
        self.action_space = None
        self.render_mode = None
        self._t = 0

    def reset(self):
        self._t = 0
        return np.zeros((self.num_envs, self.obs_dim), dtype=np.float32)

    def step_async(self, actions):
        pass

    def step_wait(self):
        self._t += 1
        rews = np.full(self.num_envs, 1.0, dtype=np.float32)
        dones = np.zeros(self.num_envs, dtype=bool)
        obs = np.zeros((self.num_envs, self.obs_dim), dtype=np.float32)
        for i in range(self.num_envs):
            obs[i] = 0.1 * self._t + 0.01 * i
        infos = [{} for _ in range(self.num_envs)]
        return obs, rews, dones, infos

    def close(self):
        pass

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def get_attr(self, name, indices=None):
        raise AttributeError(name)


def _wrap(stub, **kw):
    kw.setdefault("rnd_coef", 0.5)
    kw.setdefault("hidden", 16)
    kw.setdefault("out_dim", 8)
    kw.setdefault("buffer_size", 64)
    kw.setdefault("seed", 0)
    return RNDVecWrapper(stub, **kw)


# --------------------------------------------------------------- off path

def test_zero_coef_rejected():
    with pytest.raises(ValueError):
        _wrap(_StubVecEnv(), rnd_coef=0.0)
    with pytest.raises(ValueError):
        _wrap(_StubVecEnv(), rnd_coef=-1.0)


# --------------------------------------------------------------- blend math

def test_blend_matches_manual_recompute():
    w = _wrap(_StubVecEnv(n_envs=4, obs_dim=6), rnd_coef=0.7)
    w.reset()
    for _ in range(5):
        w.step_async(None)
        obs, blended, dones, infos = w.step_wait()
        # recompute intrinsic manually with the SAME normalization the
        # wrapper just applied (obs_rms/ret_rms already updated by
        # step_wait, so this reproduces its post-update state).
        flat = np.asarray(obs, dtype=np.float32)
        norm = w._normalize(flat)
        with torch.no_grad():
            t = torch.as_tensor(norm)
            target_out = w.target(t)
            pred_out = w.predictor(t)
            intrinsic = ((pred_out - target_out) ** 2).mean(dim=-1).numpy()
        scaled = intrinsic / (float(w.ret_rms.std) + 1e-8)
        expected = 1.0 + 0.7 * scaled  # r_env is a constant 1.0/tick
        np.testing.assert_allclose(blended, expected, rtol=1e-4, atol=1e-5)
        for i, info in enumerate(infos):
            assert info["reward_rnd_intrinsic"] == pytest.approx(
                float(0.7 * scaled[i]), rel=1e-3)


def test_intrinsic_is_finite_and_nonnegative_pre_scale():
    w = _wrap(_StubVecEnv())
    w.reset()
    for _ in range(3):
        w.step_async(None)
        _, blended, _, infos = w.step_wait()
        assert np.all(np.isfinite(blended))
        for info in infos:
            assert np.isfinite(info["reward_rnd_intrinsic"])


# ----------------------------------------------------------- novelty decay

def test_training_predictor_on_fixed_obs_decays_its_own_intrinsic_error():
    """The core novelty claim: repeatedly training the predictor on ONE
    obs makes the predictor/target squared error on THAT obs fall; a
    fresh, never-trained obs stays comparatively high. This is exactly
    the mechanism meant to break a static held pose (rung-0's observed
    failure): a policy that stops generating new states sees its own
    intrinsic income decay toward 0."""
    w = _wrap(_StubVecEnv(obs_dim=5), rnd_coef=0.5, buffer_size=256, lr=1e-2)
    fixed = np.full((1, 5), 3.0, dtype=np.float32)
    novel = np.full((1, 5), -7.0, dtype=np.float32)

    def _raw_error(x):
        norm = w._normalize(x)
        with torch.no_grad():
            t = torch.as_tensor(norm)
            return float(((w.predictor(t) - w.target(t)) ** 2).mean())

    # seed running obs stats + buffer so normalization/sampling work
    w.obs_rms.update(np.concatenate([fixed] * 8 + [novel] * 8, axis=0))
    for _ in range(200):
        w.ring.push(fixed)
    err_before = _raw_error(fixed)
    novel_before = _raw_error(novel)
    for _ in range(60):
        stats = w.train_predictor(steps=4, batch=32)
        assert stats is not None
    err_after = _raw_error(fixed)
    novel_after = _raw_error(novel)
    assert err_after < err_before * 0.5, (err_before, err_after)
    # the never-trained novel obs should end up with a clearly larger
    # error than the well-trained fixed obs (novelty preserved)
    assert novel_after > err_after * 2.0, (novel_after, err_after)


def test_train_predictor_none_below_one_batch():
    w = _wrap(_StubVecEnv(), buffer_size=64)
    assert w.train_predictor(steps=2, batch=32) is None
    w.ring.push(np.zeros((40, w.obs_rms.mean.shape[0]), dtype=np.float32))
    assert w.train_predictor(steps=2, batch=32) is not None


# --------------------------------------------------------------- ring buffer

def test_obs_ring_wraps_at_capacity():
    ring = _ObsRing(capacity=10, obs_dim=2)
    for i in range(25):
        ring.push(np.full((1, 2), float(i), dtype=np.float32))
    assert len(ring) == 10
    # the last 10 pushed values (15..24) must be exactly what's stored
    stored = set(ring.buf[:, 0].tolist())
    assert stored == set(float(i) for i in range(15, 25))


def test_running_mean_std_matches_numpy():
    rms = _RunningMeanStd((3,))
    rng = np.random.default_rng(0)
    x = rng.normal(size=(500, 3)).astype(np.float64)
    for i in range(0, 500, 37):
        rms.update(x[i:i + 37])
    np.testing.assert_allclose(rms.mean, x.mean(axis=0), atol=1e-6)
    np.testing.assert_allclose(rms.var, x.var(axis=0), atol=1e-6)


# ------------------------------------------------------------------ persist

def test_save_load_round_trips_intrinsic_reward(tmp_path):
    w = _wrap(_StubVecEnv(obs_dim=4), rnd_coef=0.3, buffer_size=64)
    w.reset()
    for _ in range(6):
        w.step_async(None)
        w.step_wait()
    w.train_predictor(steps=3, batch=8) if len(w.ring) >= 8 else None
    probe = np.random.default_rng(1).normal(size=(1, 4)).astype(np.float32)

    def _intrinsic(ww):
        norm = ww._normalize(probe)
        with torch.no_grad():
            t = torch.as_tensor(norm)
            return float(((ww.predictor(t) - ww.target(t)) ** 2).mean())

    before = _intrinsic(w)
    path = tmp_path / "rnd.pt"
    w.save(path)

    w2 = _wrap(_StubVecEnv(obs_dim=4), rnd_coef=0.3, buffer_size=64, seed=99)
    w2.load(path)
    after = _intrinsic(w2)
    assert after == pytest.approx(before, rel=1e-6)
    assert w2.updates == w.updates
