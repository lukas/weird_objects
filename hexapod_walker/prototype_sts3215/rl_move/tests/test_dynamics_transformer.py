"""Focused contracts for Transformer dynamics pretraining."""
from __future__ import annotations

import numpy as np
import torch

from rl_move.dynamics import frames as fr
from rl_move.dynamics.model import DynamicsModel, dynamics_loss


def _model() -> DynamicsModel:
    model = DynamicsModel(
        input_set="obs", history=8, horizons=(1, 2, 5, 10), short_max=5,
        hidden=64, act_hidden=64, z_dim=32, arch="transformer",
        tf_layers=2, tf_heads=4, tf_ff=128, tf_dropout=0.0,
    )
    model.eval()
    return model


def _batch(model: DynamicsModel, batch_size: int = 3) -> dict:
    batch = {
        "hist": torch.randn(batch_size, model.history, fr.FRAME_DIM),
        "fut_actions": torch.randn(batch_size, max(model.horizons),
                                    fr.ACTION_DIM),
        "state": {}, "contact": {}, "current": {}, "fut_hist": {},
        "priv": {},
        "contact_now": torch.randint(0, 2, (batch_size, fr.N_FEET)).float(),
        "current_now": torch.randn(batch_size, fr.CURRENT_DIM),
        "priv_now": torch.randn(batch_size, fr.PRIV_DIM),
        "priv_mask_now": torch.ones(batch_size, fr.PRIV_DIM),
    }
    for k in model.horizons:
        batch["state"][k] = torch.randn(batch_size, fr.STATE_DIM)
        batch["contact"][k] = torch.randint(
            0, 2, (batch_size, fr.N_FEET)).float()
        batch["current"][k] = torch.randn(batch_size, fr.CURRENT_DIM)
        batch["fut_hist"][k] = torch.randn(
            batch_size, model.history, fr.FRAME_DIM)
        batch["priv"][k] = torch.randn(batch_size, fr.PRIV_DIM)
    return batch


def test_transformer_heads_and_backward():
    model = _model()
    batch = _batch(model)
    out = model(batch["hist"], batch["fut_actions"])
    assert model.arch == "transformer"
    assert out["contact_now_logits"].shape == (3, fr.N_FEET)
    assert out["current_now"].shape == (3, fr.CURRENT_DIM)
    assert out["current"][5].shape == (3, fr.CURRENT_DIM)
    loss, logs = dynamics_loss(out, batch, {
        "joint_pos": 1.0, "joint_vel": 1.0, "imu": 1.0,
        "contact": 0.5, "contact_current": 0.5,
        "motor_current": 0.5, "latent": 1.0,
        "priv_current": 0.25, "priv_future": 0.25,
    }, model)
    loss.backward()
    assert torch.isfinite(loss)
    assert "current/contact_bce" in logs
    assert "h5/motor_current" in logs


def test_action_transformer_is_causal_at_each_horizon():
    torch.manual_seed(4)
    model = _model()
    hist = torch.randn(2, model.history, fr.FRAME_DIM)
    actions = torch.randn(2, max(model.horizons), fr.ACTION_DIM)
    changed = actions.clone()
    changed[:, 1:] += torch.randn_like(changed[:, 1:]) * 100.0
    with torch.no_grad():
        original = model(hist, actions)
        perturbed = model(hist, changed)
    torch.testing.assert_close(original["state"][1], perturbed["state"][1])
    torch.testing.assert_close(original["priv"][1], perturbed["priv"][1])


def test_legacy_priv_mask_names_only_real_channels():
    mask = fr.priv_available_mask(fr.LEGACY_PRIV_DIM)
    np.testing.assert_array_equal(np.flatnonzero(mask), [0, 1, 3, 4])
    assert fr.priv_available_mask(fr.PRIV_DIM).all()


def test_masked_priv_loss_ignores_padded_legacy_channels():
    torch.manual_seed(2)
    model = _model()
    batch = _batch(model)
    batch["priv_mask_now"][:, :] = 0.0
    batch["priv_mask_now"][:, [0, 1, 3, 4]] = 1.0
    out = model(batch["hist"], batch["fut_actions"])
    lambdas = {
        "joint_pos": 0.0, "joint_vel": 0.0, "imu": 0.0,
        "contact": 0.0, "contact_current": 0.0,
        "motor_current": 0.0, "latent": 0.0,
        "priv_current": 1.0, "priv_future": 1.0,
    }
    loss1, _ = dynamics_loss(out, batch, lambdas, model)
    batch["priv_now"][:, 2] += 1e6
    for k in model.horizons:
        batch["priv"][k][:, 2] -= 1e6
    loss2, _ = dynamics_loss(out, batch, lambdas, model)
    torch.testing.assert_close(loss1, loss2)
