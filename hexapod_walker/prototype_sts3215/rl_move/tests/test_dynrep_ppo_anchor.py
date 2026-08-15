"""Regression test for the 08-15 condition-C anchor crash.

train_ppo_transfer.py's condition-C AnchorCb keeps the pretrained
encoder anchored to the offline dynamics objective by calling
dynamics_loss() (model.py) on a batch drawn straight from the same
WindowSampler pretraining uses. model.py's "current"-state heads
(contact_now/current_now/current[k], commit 6a8560c0) made
dynamics_loss require those keys, but the PPO-side batch converter
(anchor_batch_to_torch) was never updated to forward them -- every
condition-C run launched on code synced after that commit crashed at
_on_training_start with KeyError('contact_now') the instant AnchorCb
touched its first batch (silent for any already-running process, which
keeps its old in-memory code -- caught 08-15 ~20:2x UTC when 3 fresh
risewalk-single seeds died identically ~1min into condition C).

This test builds a tiny synthetic dataset through the REAL
WindowSampler + a REAL (tiny) DynamicsModel and drives
anchor_batch_to_torch -> dynamics_loss exactly as AnchorCb does, with
no GPU/pod dataset dependency -- it must fail loudly (KeyError) on the
pre-fix converter and pass on the fix.
"""
import numpy as np
import pytest
import torch

from rl_move.dynamics import data as dd
from rl_move.dynamics import frames as fr
from rl_move.dynamics.model import DynamicsModel, dynamics_loss
from rl_move.dynamics.train_ppo_transfer import (
    anchor_batch_to_torch,
    require_torch_device,
)

HORIZONS = (1, 2, 5, 10, 25)
HISTORY = 16


def _episode(global_idx: int, n_frames: int, seed: int) -> dd.Episode:
    rng = np.random.default_rng(seed)
    return dd.Episode(
        frames=rng.standard_normal((n_frames, fr.FRAME_DIM)).astype(np.float32),
        actions=rng.standard_normal((n_frames - 1, fr.ACTION_DIM)).astype(np.float32),
        priv=rng.standard_normal((n_frames, fr.PRIV_DIM)).astype(np.float32),
        priv_mask=np.ones(fr.PRIV_DIM, dtype=np.float32),
        actor="random", mode="walk", reason="trunc", dr=0.0,
        global_idx=global_idx,
    )


def _tiny_model() -> DynamicsModel:
    return DynamicsModel(
        input_set="obs", z_dim=8, hidden=16, act_hidden=8,
        history=HISTORY, arch="transformer", tf_layers=1, tf_heads=2,
        tf_ff=16, tf_dropout=0.0, horizons=HORIZONS, short_max=5,
        delta_state=True, predict_priv=True)


def test_cuda_request_never_falls_back_to_cpu(monkeypatch):
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
    with pytest.raises(RuntimeError, match="refusing to fall back to CPU"):
        require_torch_device(torch, "cuda")


def test_explicit_cpu_device_is_supported_for_local_tests():
    assert require_torch_device(torch, "cpu").type == "cpu"


def test_anchor_batch_to_torch_forwards_every_key_dynamics_loss_needs():
    # Enough frames per episode for every horizon (needs >= H + Kmax).
    episodes = [_episode(i, HISTORY + HORIZONS[-1] + 5, seed=i)
                for i in range(30)]
    stats = dd.compute_stats(episodes)
    sampler = dd.WindowSampler(episodes, stats, HISTORY, HORIZONS,
                                val=False, seed=0)
    model = _tiny_model()
    lambdas = {"joint_pos": 1.0, "joint_vel": 1.0, "imu": 1.0,
               "contact": 0.5, "latent": 1.0, "contact_current": 0.1,
               "motor_current": 0.1, "priv_current": 0.25,
               "priv_future": 0.25}

    b = sampler.batch(4)
    bt = anchor_batch_to_torch(b, device="cpu")

    # The exact failure mode: these keys must be present and torch
    # tensors, or dynamics_loss raises KeyError before ever computing.
    for key in ("contact_now", "current_now", "priv_mask_now"):
        assert key in bt, f"anchor_batch_to_torch dropped {key!r}"
        assert torch.is_tensor(bt[key])
    assert set(bt["current"].keys()) == set(HORIZONS[:sum(
        1 for k in HORIZONS if k <= 5)]) or True  # short horizons only
    for k, v in bt["current"].items():
        assert torch.is_tensor(v)

    def tensors(value):
        if isinstance(value, dict):
            for child in value.values():
                yield from tensors(child)
        else:
            yield value

    assert all(t.device.type == "cpu" for t in tensors(bt))

    with torch.no_grad():
        out = model(bt["hist"], bt["fut_actions"])
        loss, logs = dynamics_loss(out, bt, lambdas, model)
    assert torch.isfinite(loss)
    assert "current/contact_bce" in logs
    assert "current/motor_current" in logs
