"""test_amp_discriminator.py — AMP track M1 bank for amp_discriminator.py.

Proves the discriminator core (rl_docs/AMP_LOCOMOTION.md §3.6/§5.2)
actually separates real motion-library transitions from synthetic
fakes, that the gradient penalty is finite and keeps the discriminator
from saturating instantly, and that the least-squares style reward
stays bounded — the M0/M1 "gradients flow through PPO and the
discriminator trains without instant saturation" checkbox (STATUS.md
Next item 4), scoped to the discriminator alone (not yet wired into
train_ppo_mjx's live env loop — see the module docstring).

CPU, plain torch/numpy — no GPU/Warp/MJX dependency. Runs in seconds.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl_move.sim.amp_discriminator import (  # noqa: E402
    DEFAULT_LIBRARY,
    AMPDiscriminator,
    MotionLibrary,
    discriminator_loss,
    gradient_penalty,
    style_reward,
    train_smoke,
)


@pytest.fixture(scope="module")
def library():
    if not DEFAULT_LIBRARY.exists():
        pytest.skip(f"motion library not built: {DEFAULT_LIBRARY}")
    return MotionLibrary(DEFAULT_LIBRARY)


def test_library_loads_and_normalizes(library):
    assert library.feat_dim == 60
    assert len(library) > 0
    # every clip loses exactly one transition (its last tick)
    assert len(library) == int(library.obs_style.shape[0]) - len(library.clip_lens)
    s_t, s_t1 = library.sample_real_transitions(64)
    assert s_t.shape == (64, 60)
    n_t = library.normalize(s_t)
    # normalized over the FULL dataset (not just this sample) should be
    # roughly zero-mean / unit-std, not exactly (sample != population)
    assert abs(float(n_t.mean())) < 1.0
    assert 0.2 < float(n_t.std()) < 5.0


def test_transitions_never_cross_clip_boundary(library):
    ends = set()
    for s0, ln in zip(library.clip_starts, library.clip_lens):
        ends.add(int(s0) + int(ln) - 1)  # last tick index of each clip
    assert not (set(library.transition_starts.tolist()) & ends)


def test_gradient_penalty_finite_and_nonneg(library):
    torch.manual_seed(0)
    disc = AMPDiscriminator(library.feat_dim)
    s_t, s_t1 = library.sample_real_transitions(32)
    s_t = torch.as_tensor(library.normalize(s_t))
    s_t1 = torch.as_tensor(library.normalize(s_t1))
    gp = gradient_penalty(disc, s_t, s_t1)
    assert torch.isfinite(gp)
    assert gp.item() >= 0.0


def test_discriminator_loss_finite_and_backprops(library):
    torch.manual_seed(0)
    disc = AMPDiscriminator(library.feat_dim)
    s_t, s_t1 = library.sample_real_transitions(32)
    f_t, f_t1 = library.sample_real_transitions(32)
    s_t = torch.as_tensor(library.normalize(s_t))
    s_t1 = torch.as_tensor(library.normalize(s_t1))
    # scramble the "fake" pair's dims to make it trivially distinguishable
    f_t = torch.as_tensor(library.normalize(f_t))[:, torch.randperm(60)]
    f_t1 = torch.as_tensor(library.normalize(f_t1))[:, torch.randperm(60)]
    loss, stats = discriminator_loss(disc, s_t, s_t1, f_t, f_t1)
    assert torch.isfinite(loss)
    loss.backward()
    grads = [p.grad for p in disc.parameters()]
    assert all(g is not None and torch.isfinite(g).all() for g in grads)
    assert all(k in stats for k in ("d_real_loss", "d_fake_loss", "gp", "d_real_mean", "d_fake_mean"))


def test_style_reward_bounded(library):
    torch.manual_seed(0)
    disc = AMPDiscriminator(library.feat_dim)
    s_t, s_t1 = library.sample_real_transitions(64)
    s_t = torch.as_tensor(library.normalize(s_t))
    s_t1 = torch.as_tensor(library.normalize(s_t1))
    r = style_reward(disc, s_t, s_t1)
    assert torch.isfinite(r).all()
    assert float(r.min()) >= 0.0
    assert float(r.max()) <= 1.0


@pytest.mark.parametrize("fake_kind", ["noise", "shuffled"])
def test_discriminator_separates_real_from_fake_after_training(library, fake_kind):
    """The core M1 claim: after a short training loop the discriminator
    scores held-out real transitions higher than fake ones, gradient
    penalty stays finite throughout (no saturation blowup), and the
    resulting style reward actually prefers real motion.
    """
    disc, history = train_smoke(library, steps=150, batch=128, seed=1, fake_kind=fake_kind)

    assert all(np.isfinite(h["loss"]) for h in history)
    assert all(np.isfinite(h["gp"]) for h in history)
    # no instant saturation: gradient penalty never explodes
    assert max(h["gp"] for h in history) < 1e4

    # held-out evaluation: fresh real transitions vs fresh fakes
    rng = np.random.default_rng(999)
    s_t, s_t1 = library.sample_real_transitions(256, rng)
    s_t = torch.as_tensor(library.normalize(s_t))
    s_t1 = torch.as_tensor(library.normalize(s_t1))
    if fake_kind == "noise":
        f_t = torch.randn(256, library.feat_dim)
        f_t1 = torch.randn(256, library.feat_dim)
    else:
        f_t, _ = library.sample_real_transitions(256, rng)
        _, f_t1 = library.sample_real_transitions(256, rng)
        f_t = torch.as_tensor(library.normalize(f_t))
        f_t1 = torch.as_tensor(library.normalize(f_t1))

    with torch.no_grad():
        d_real = disc(s_t, s_t1)
        d_fake = disc(f_t, f_t1)
        r_real = style_reward(disc, s_t, s_t1)
        r_fake = style_reward(disc, f_t, f_t1)

    assert float(d_real.mean()) > float(d_fake.mean()) + 0.5, (
        f"discriminator did not separate real ({d_real.mean():.3f}) from "
        f"fake ({d_fake.mean():.3f}) after training on fake_kind={fake_kind}"
    )
    assert float(r_real.mean()) > float(r_fake.mean()), (
        "style reward should prefer real transitions over fake ones "
        f"(real={r_real.mean():.3f}, fake={r_fake.mean():.3f})"
    )


def test_last_layer_bias_moves_from_init(library):
    """Sanity check that training actually updates weights (not a
    silent no-op optimizer/loss wiring bug)."""
    torch.manual_seed(0)
    disc0 = AMPDiscriminator(library.feat_dim)
    init_params = [p.clone() for p in disc0.parameters()]
    disc, _ = train_smoke(library, steps=50, batch=64, seed=0)
    moved = any(
        not torch.allclose(p0, p1)
        for p0, p1 in zip(init_params, disc.parameters())
    )
    assert moved
