"""GRU policy correctness + learning tests (post gru-r1..r4c forensics).

Four from-scratch GRU runs produced the identical leg-sacrifice paddle.
Before blaming exploration, these tests pin down that the GRU
implementation itself is sound:

1. `_process_sequence` (the ONE overridden method) is numerically
   identical to a hand-rolled per-step GRU loop, on both its code
   paths (fused no-reset fast path and the per-step reset loop);
2. episode_start=1 mid-sequence really zeroes the hidden state (a
   stale-state bug here would train on garbage credit assignment);
3. RecurrentPPO+GRU checkpoints survive a save/load round trip with
   bit-identical stateful predictions, and `load_checkpoint_auto`
   returns the right algo class for both MLP and GRU zips;
4. the whole training stack can actually LEARN a task that strictly
   requires memory (a memoryless policy is capped at a known score) —
   the test that would catch any silent state-threading bug in
   rollout collection or BPTT.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch as th
from torch import nn

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import gymnasium as gym  # noqa: E402
from gymnasium import spaces  # noqa: E402

from rl_move.sim.gru_policy import (  # noqa: E402
    GruActorCriticPolicy,
    is_recurrent_checkpoint,
    load_checkpoint_auto,
)


# ---------------------------------------------------------------------------
# 1+2. _process_sequence numerics
# ---------------------------------------------------------------------------

def _manual_gru_rollout(gru: nn.GRU, feats_tbf: th.Tensor,
                        starts_tb: th.Tensor, h0: th.Tensor) -> th.Tensor:
    """Reference implementation: per-step GRU with explicit resets."""
    h = h0.clone()
    outs = []
    for t in range(feats_tbf.shape[0]):
        h = (1.0 - starts_tb[t]).view(1, -1, 1) * h
        out, h = gru(feats_tbf[t].unsqueeze(0), h)
        outs.append(out.squeeze(0))
    return th.stack(outs)  # (T, B, H)


@pytest.mark.parametrize("with_resets", [False, True])
def test_process_sequence_matches_stepwise(with_resets):
    th.manual_seed(0)
    n_seq, t_len, n_in, n_hid = 3, 7, 5, 4
    gru = nn.GRU(n_in, n_hid)
    feats = th.randn(n_seq * t_len, n_in)
    starts = th.zeros(n_seq * t_len)
    if with_resets:
        # Resets at assorted mid-sequence points (sequence-major layout:
        # rows [i*t_len : (i+1)*t_len] belong to sequence i).
        starts[2] = 1.0
        starts[t_len + 5] = 1.0
        starts[2 * t_len + 1] = 1.0
    h0 = th.randn(1, n_seq, n_hid)
    c0 = th.zeros(1, n_seq, n_hid)

    with th.no_grad():
        out, (h_out, c_out) = GruActorCriticPolicy._process_sequence(
            feats, (h0, c0), starts, gru)
        feats_tbf = feats.reshape(n_seq, t_len, n_in).swapaxes(0, 1)
        starts_tb = starts.reshape(n_seq, t_len).swapaxes(0, 1)
        ref_tbf = _manual_gru_rollout(gru, feats_tbf, starts_tb, h0)

    ref_flat = ref_tbf.transpose(0, 1).reshape(n_seq * t_len, n_hid)
    assert th.allclose(out, ref_flat, atol=1e-6), \
        "batched _process_sequence diverges from per-step reference"
    assert th.allclose(h_out.squeeze(0), ref_tbf[-1], atol=1e-6)
    assert th.equal(c_out, c0), "unused cell state must pass through"


def test_episode_start_resets_hidden_state():
    """Post-reset outputs must equal a fresh-state run of the suffix."""
    th.manual_seed(1)
    t_len, n_in, n_hid, k = 9, 4, 6, 5
    gru = nn.GRU(n_in, n_hid)
    feats = th.randn(t_len, n_in)
    starts = th.zeros(t_len)
    starts[k] = 1.0
    h0 = th.randn(1, 1, n_hid)

    with th.no_grad():
        out, _ = GruActorCriticPolicy._process_sequence(
            feats, (h0, th.zeros_like(h0)), starts, gru)
        fresh, _ = gru(feats[k:].unsqueeze(1))  # default zero h0
    assert th.allclose(out[k:], fresh.squeeze(1), atol=1e-6), \
        "hidden state leaked across an episode boundary"


# ---------------------------------------------------------------------------
# Toy envs
# ---------------------------------------------------------------------------

class _TinyContinuousEnv(gym.Env):
    """Trivial continuous env for smoke/save-load tests."""

    def __init__(self):
        self.observation_space = spaces.Box(-1, 1, (3,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (2,), dtype=np.float32)
        self._t = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._t = 0
        return self.observation_space.sample(), {}

    def step(self, action):
        self._t += 1
        obs = self.observation_space.sample()
        return obs, 0.0, False, self._t >= 8, {}


class _MemoryEnv(gym.Env):
    """Reward requires remembering the first observation.

    obs = [signal, t/T]; signal = s0 in {-1,+1} at t=0, then 0.
    Every step after t=0 pays 1 - |a - s0| / 2 for a scalar action.
    A memoryless policy sees identical obs regardless of s0 for t>0,
    so its expected per-step reward is capped at 0.5; perfect memory
    scores 1.0.
    """

    T = 6

    def __init__(self):
        self.observation_space = spaces.Box(-1, 1, (2,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (1,), dtype=np.float32)
        self._t = 0
        self._s0 = 1.0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._t = 0
        self._s0 = float(self.np_random.choice([-1.0, 1.0]))
        return np.array([self._s0, 0.0], dtype=np.float32), {}

    def step(self, action):
        self._t += 1
        r = 1.0 - abs(float(action[0]) - self._s0) / 2.0
        obs = np.array([0.0, self._t / self.T], dtype=np.float32)
        return obs, r, False, self._t >= self.T, {}


# ---------------------------------------------------------------------------
# 3. save/load round trip + auto loader
# ---------------------------------------------------------------------------

def test_save_load_roundtrip_and_auto_loader(tmp_path):
    from sb3_contrib import RecurrentPPO
    from stable_baselines3 import PPO

    model = RecurrentPPO(
        GruActorCriticPolicy, _TinyContinuousEnv(),
        n_steps=8, batch_size=16, n_epochs=1, seed=0, device="cpu",
        policy_kwargs=dict(lstm_hidden_size=8))
    model.learn(64)
    gru_zip = tmp_path / "gru.zip"
    model.save(gru_zip)

    mlp = PPO("MlpPolicy", _TinyContinuousEnv(), n_steps=8, batch_size=16,
              seed=0, device="cpu")
    mlp_zip = tmp_path / "mlp.zip"
    mlp.save(mlp_zip)

    assert is_recurrent_checkpoint(gru_zip)
    assert not is_recurrent_checkpoint(mlp_zip)
    loaded = load_checkpoint_auto(gru_zip)
    assert isinstance(loaded, RecurrentPPO)
    assert isinstance(loaded.policy.lstm_actor, nn.GRU)
    assert isinstance(load_checkpoint_auto(mlp_zip), PPO)

    # Stateful predictions must match bit-for-bit across the round trip.
    obs_seq = [np.random.default_rng(i).standard_normal(3).astype(np.float32)
               for i in range(10)]
    for m in (model, loaded):
        m.policy.set_training_mode(False)

    def rollout(m):
        acts, state = [], None
        ep_start = np.ones((1,), dtype=bool)
        for o in obs_seq:
            a, state = m.predict(o, state=state, episode_start=ep_start,
                                 deterministic=True)
            ep_start = np.zeros((1,), dtype=bool)
            acts.append(a)
        return np.stack(acts)

    np.testing.assert_array_equal(rollout(model), rollout(loaded))


# ---------------------------------------------------------------------------
# 4. it must actually learn a memory task (slow, ~1-2 min CPU)
# ---------------------------------------------------------------------------

class _AnchorEnv(gym.Env):
    """18-action env emitting a constant ``bc_target`` every step —
    the pull direction is unambiguous, so the anchor either moves the
    policy mean toward it or the recurrent aux path is broken."""

    TARGET = 0.25

    def __init__(self):
        self.observation_space = spaces.Box(-1, 1, (3,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (18,), dtype=np.float32)
        self._t = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._t = 0
        return self.observation_space.sample(), {}

    def step(self, action):
        self._t += 1
        info = {"bc_target": np.full(18, self.TARGET, dtype=np.float32),
                "bc_mode": self._t % 2}
        return (self.observation_space.sample(), 0.0, False,
                self._t >= 8, info)


def test_bc_anchor_recurrent(tmp_path):
    """BC anchor on RecurrentPPO+GRU: hidden states ride into the ring,
    the aux step pulls the pi mean toward the target AT those states,
    the single-step mean matches the production stateful predict path,
    and the ring stays out of the checkpoint zip."""
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.vec_env import DummyVecEnv

    from rl_move.sim.bc_anchor import (
        make_bc_anchor_ppo_class,
        make_bc_collect_callback,
    )

    cls = make_bc_anchor_ppo_class(RecurrentPPO)
    venv = DummyVecEnv([_AnchorEnv for _ in range(4)])
    model = cls(
        GruActorCriticPolicy, venv,
        n_steps=32, batch_size=64, n_epochs=1, learning_rate=3e-3,
        ent_coef=0.0, seed=0, device="cpu",
        policy_kwargs=dict(lstm_hidden_size=8, net_arch=[32]))
    model.bc_coef = 5.0
    model.bc_minibatches = 8
    model.bc_batch_size = 256
    model.bc_buffer_cap = 4096
    model.learn(4096, callback=make_bc_collect_callback())

    n = model._bc_n
    assert n > 0, "collect callback never filled the anchor ring"
    assert model._bc_h.shape[1] == 8, "hidden state not stored per pair"
    assert float(np.abs(model._bc_h[:n]).max()) > 0.0, \
        "stored hidden states are all zero — lstm_states not captured"
    assert set(np.unique(model._bc_mode[:n])) == {0, 1}

    with th.no_grad():
        mean = model._bc_policy_mean(
            th.as_tensor(model._bc_obs[:n]), th.as_tensor(model._bc_h[:n]))
    mse = float(((mean - _AnchorEnv.TARGET) ** 2).mean())
    # Untrained pi mean is ~0 -> mse ~ TARGET^2 = 0.0625.
    assert mse < 0.02, (
        f"anchor did not pull the recurrent pi mean toward the target "
        f"(mse {mse:.4f} vs untrained ~{_AnchorEnv.TARGET ** 2:.4f})")

    # Single-step aux mean must match the stateful production path.
    h1 = model._bc_h[:1].reshape(1, 1, 8)
    a, _ = model.policy.predict(
        model._bc_obs[:1], state=(h1, np.zeros_like(h1)),
        episode_start=np.zeros(1, dtype=bool), deterministic=True)
    with th.no_grad():
        m1 = model._bc_policy_mean(
            th.as_tensor(model._bc_obs[:1]), th.as_tensor(model._bc_h[:1]))
    np.testing.assert_allclose(
        a[0], np.clip(m1.numpy()[0], -1, 1), atol=1e-5)

    # Ring buffers are rollout data, not model state.
    zip_path = tmp_path / "anchored.zip"
    model.save(zip_path)
    loaded = cls.load(zip_path, device="cpu")
    assert not hasattr(loaded, "_bc_obs"), \
        "anchor ring pickled into the checkpoint (save bloat)"


@pytest.mark.slow
def test_gru_learns_memory_task():
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.vec_env import DummyVecEnv

    venv = DummyVecEnv([_MemoryEnv for _ in range(16)])
    model = RecurrentPPO(
        GruActorCriticPolicy, venv,
        n_steps=24, batch_size=96, n_epochs=5, learning_rate=3e-4,
        ent_coef=0.0, gamma=0.9, seed=0, device="cpu",
        policy_kwargs=dict(lstm_hidden_size=32, net_arch=[]))
    model.learn(60_000)
    model.policy.set_training_mode(False)

    env = _MemoryEnv()
    rewards = []
    for ep in range(40):
        obs, _ = env.reset(seed=1000 + ep)
        state = None
        ep_start = np.ones((1,), dtype=bool)
        done = False
        while not done:
            a, state = model.predict(obs, state=state,
                                     episode_start=ep_start,
                                     deterministic=True)
            ep_start = np.zeros((1,), dtype=bool)
            obs, r, term, trunc, _ = env.step(a)
            rewards.append(r)
            done = term or trunc
    mean_r = float(np.mean(rewards))
    # Memoryless cap is 0.5/step; require clear water above it. A GRU
    # with broken state threading trains to ~0.5 and fails here.
    assert mean_r > 0.75, (
        f"GRU scored {mean_r:.3f}/step on the memory task; memoryless "
        f"cap is 0.5 — state threading through training is broken")
