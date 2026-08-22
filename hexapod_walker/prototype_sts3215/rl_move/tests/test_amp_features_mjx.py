"""test_amp_features_mjx.py — AMP track M1: obs_style against the LIVE
batched MJX/Warp vec env (split from test_amp_features.py because a
module-level ``pytest.skip`` would otherwise also skip that file's
always-on CPU equivalence tests — same reason test_mjx_vec_env.py is
its own file).

Needs mujoco-mjx + jax (same guard/skip as test_mjx_vec_env.py); the
controller does not carry them — verified on a GPU pod instead (see
snapshot notes for the on-pod pytest run this cycle).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")  # see test_mjx_vec_env.py

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl_move.sim.mjx_backend import mjx_is_available  # noqa: E402

if not mjx_is_available():  # pragma: no cover
    pytest.skip("mujoco-mjx / jax not installed", allow_module_level=True)

from rl_move.sim.amp_features import (  # noqa: E402
    chassis_pad_gyro_ids,
    obs_style_batch,
)
from rl_move.sim.mjx_vec_env import MjxVecEnv  # noqa: E402
from rl_move.sim.walk_task import SimHexapodJointWalkEnv  # noqa: E402

B = 3


def test_mjx_vecenv_obs_style_batched():
    """First-ever computation of obs_style from the LIVE batched
    MJX/Warp vec env (not the offline CPU npz generator) — the
    prerequisite the discriminator's live-reward wiring needs."""
    venv = MjxVecEnv(SimHexapodJointWalkEnv, B,
                     env_kwargs=dict(randomize=False, episode_seconds=15.0),
                     seed=0, pool_per_env=1, desync_episodes=False)
    try:
        venv.reset()
        ids = chassis_pad_gyro_ids(venv.envs[0])
        neutral = np.stack([e.data.qpos[ids.qadr].copy() for e in venv.envs])
        for _ in range(5):
            actions = np.zeros((B, venv.action_space.shape[0]))
            venv.step_async(actions)
            venv.step_wait()
        style = obs_style_batch([e.data for e in venv.envs], ids, neutral)
        assert style.shape == (B, 60)
        assert np.all(np.isfinite(style))
    finally:
        venv.close()


def test_discriminator_on_real_mjx_rollout():
    """Feed ACTUAL MJX rollout transitions (not synthetic noise/
    shuffle) through the discriminator for the first time — closes
    amp_discriminator.py's documented gap directly."""
    import torch

    from rl_move.sim.amp_discriminator import (
        AMPDiscriminator, MotionLibrary, discriminator_loss)

    venv = MjxVecEnv(SimHexapodJointWalkEnv, B,
                     env_kwargs=dict(randomize=False, episode_seconds=15.0),
                     seed=1, pool_per_env=1, desync_episodes=False)
    try:
        venv.reset()
        ids = chassis_pad_gyro_ids(venv.envs[0])
        neutral = np.stack([e.data.qpos[ids.qadr].copy() for e in venv.envs])
        styles = []
        for _ in range(20):
            actions = np.zeros((B, venv.action_space.shape[0]))
            venv.step_async(actions)
            venv.step_wait()
            styles.append(obs_style_batch([e.data for e in venv.envs], ids, neutral))
        styles = np.stack(styles, axis=0)  # (T, B, 60)
    finally:
        venv.close()

    lib = MotionLibrary()
    assert lib.feat_dim == 60
    disc = AMPDiscriminator(lib.feat_dim)

    real_t, real_t1 = lib.sample_real_transitions(64, np.random.default_rng(0))
    real_t = torch.as_tensor(lib.normalize(real_t))
    real_t1 = torch.as_tensor(lib.normalize(real_t1))

    # REAL rollout transitions as "fake" (policy is zero-action, far
    # from the demo manifold -- exactly the untrained-policy case AMP
    # style reward must handle without NaN/inf).
    fake_t = styles[:-1].reshape(-1, 60)
    fake_t1 = styles[1:].reshape(-1, 60)
    fake_t = torch.as_tensor(lib.normalize(fake_t))
    fake_t1 = torch.as_tensor(lib.normalize(fake_t1))
    n = min(len(fake_t), 64)
    idx = np.random.default_rng(0).choice(len(fake_t), size=n, replace=False)
    fake_t, fake_t1 = fake_t[idx], fake_t1[idx]

    loss, stats = discriminator_loss(disc, real_t[:n], real_t1[:n], fake_t, fake_t1)
    assert np.isfinite(float(loss.detach()))
    for v in stats.values():
        assert np.isfinite(v)
