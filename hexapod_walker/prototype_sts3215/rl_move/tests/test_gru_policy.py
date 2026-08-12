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
    _N_LOCO_SLOTS,
    DualGruActorCriticPolicy,
    GruActorCriticPolicy,
    N_MODE_OBS,
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


def _tiny_recurrent_bc_model(bs=4, hidden=8):
    """Minimal RecurrentPPO+BC-anchor model for gradient-flow probes
    (no .learn(), no rollout — just a policy + a hand-fed anchor pair,
    exactly the shape _bc_policy_mean needs)."""
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.vec_env import DummyVecEnv

    from rl_move.sim.bc_anchor import make_bc_anchor_ppo_class

    cls = make_bc_anchor_ppo_class(RecurrentPPO)
    venv = DummyVecEnv([_AnchorEnv for _ in range(2)])
    model = cls(
        GruActorCriticPolicy, venv,
        n_steps=8, batch_size=8, n_epochs=1, learning_rate=3e-3,
        ent_coef=0.0, seed=0, device="cpu",
        policy_kwargs=dict(lstm_hidden_size=hidden, net_arch=[16]))
    rng = np.random.default_rng(0)
    obs = th.as_tensor(
        rng.uniform(-1, 1, (bs, model.observation_space.shape[0]))
        .astype(np.float32))
    h = th.as_tensor(
        rng.uniform(-1, 1, (bs, hidden)).astype(np.float32))
    tgt = th.as_tensor(
        rng.uniform(-1, 1, (bs, 18)).astype(np.float32))
    return model, obs, h, tgt


def _trunk_and_head_params(model):
    trunk = (list(model.policy.features_extractor.parameters())
             + list(model.policy.lstm_actor.parameters()))
    head = (list(model.policy.mlp_extractor.parameters())
            + list(model.policy.action_net.parameters()))
    return trunk, head


def test_detach_trunk_default_off_is_bit_exact():
    """train.bc_anchor_detach_trunk unset -> byte-identical mean AND
    gradient into the GRU/feature-extractor as the pre-08-12 code path
    (the flag must never change default behavior)."""
    model, obs, h, tgt = _tiny_recurrent_bc_model()
    assert not hasattr(model, "bc_detach_trunk") or \
        not model.bc_detach_trunk, "default must be off"
    trunk, head = _trunk_and_head_params(model)
    for p in trunk + head:
        p.grad = None
    mean = model._bc_policy_mean(obs, h)
    th.nn.functional.mse_loss(mean, tgt).backward()
    trunk_grad_norm = sum(
        float(p.grad.abs().sum()) for p in trunk if p.grad is not None)
    head_grad_norm = sum(
        float(p.grad.abs().sum()) for p in head if p.grad is not None)
    assert trunk_grad_norm > 0.0, (
        "legacy path must backprop into the shared GRU/feature-"
        "extractor trunk (that is the mechanism cw-arch-gru-anchor2 "
        "measured causing the cross-mode walk freeze)")
    assert head_grad_norm > 0.0


def test_detach_trunk_stops_gradient_into_recurrent_core():
    """train.bc_anchor_detach_trunk=1 (08-12, cw-arch-gru-anchor2
    follow-up): the anchor loss must update the actor head
    (mlp_extractor + action_net) but leave the GRU cell and feature
    extractor with ZERO gradient from this loss — the lever that
    tests whether shared-trunk drift from stance-tick anchoring is
    what corrupts the walk-tick recurrent dynamics."""
    model, obs, h, tgt = _tiny_recurrent_bc_model()
    model.bc_detach_trunk = True
    trunk, head = _trunk_and_head_params(model)
    for p in trunk + head:
        p.grad = None
    mean = model._bc_policy_mean(obs, h)
    th.nn.functional.mse_loss(mean, tgt).backward()
    for p in trunk:
        assert p.grad is None or float(p.grad.abs().sum()) == 0.0, (
            "detach_trunk leaked gradient into the shared GRU/feature-"
            "extractor trunk")
    head_grad_norm = sum(
        float(p.grad.abs().sum()) for p in head if p.grad is not None)
    assert head_grad_norm > 0.0, (
        "detach_trunk must still train the actor head — a fully dead "
        "gradient is not a working lever, it's a no-op")


# ---------------------------------------------------------------------------
# 5. Dual-core (mode-gated) GRU — cw-arch-gru-anchor1..3 closure follow-up
# ---------------------------------------------------------------------------

def _onehot_tail(slot: int) -> np.ndarray:
    t = np.zeros(N_MODE_OBS, dtype=np.float32)
    t[slot] = 1.0
    return t


class _TinyDualEnv(gym.Env):
    """Continuous env whose obs tail carries the 6-wide mode one-hot.
    Episodes alternate between a locomotion-gated mode (slot 3, walk)
    and a stance-gated mode (slot 0, hold)."""

    def __init__(self):
        self.observation_space = spaces.Box(
            -1, 1, (3 + N_MODE_OBS,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (2,), dtype=np.float32)
        self._t = 0
        self._ep = 0

    def _obs(self):
        core = self.np_random.uniform(-1, 1, 3).astype(np.float32)
        slot = 3 if self._ep % 2 == 0 else 0
        return np.concatenate([core, _onehot_tail(slot)])

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._t = 0
        self._ep += 1
        return self._obs(), {}

    def step(self, action):
        self._t += 1
        return self._obs(), 0.0, False, self._t >= 8, {}


def _dual_model(hidden=8, env_ctor=_TinyDualEnv, **kw):
    from sb3_contrib import RecurrentPPO
    return RecurrentPPO(
        DualGruActorCriticPolicy, env_ctor(),
        n_steps=8, batch_size=16, n_epochs=1, seed=0, device="cpu",
        policy_kwargs=dict(lstm_hidden_size=hidden, net_arch=[16]), **kw)


def test_dual_slots_match_walk_task():
    """The gate reads the trailing _N_LOCO_SLOTS obs entries as the
    locomotion families. Frozen contract with walk_task — if either
    side reorders slots this fails before a run does."""
    from rl_move.sim.walk_task import (
        MODE_ONEHOT_ORDER, N_MODE_OBS as ENV_N, _MODE_FAMILY)
    assert ENV_N == N_MODE_OBS
    loco = MODE_ONEHOT_ORDER[-_N_LOCO_SLOTS:]
    assert loco == ("walk", "turn", "quad")
    stance = MODE_ONEHOT_ORDER[:-_N_LOCO_SLOTS]
    assert stance == ("hold", "rise", "lower")
    # Every goal-mix mode must land in the family the router expects:
    # getup is command-wise locomotion, everything attitude-ish stance.
    assert _MODE_FAMILY["getup"] == "walk"
    assert _MODE_FAMILY["track"] == "hold"


def test_dual_routing_selects_core():
    """Poison each core's action head with a distinct constant: the
    gate must pick core A's constant on locomotion obs and core B's on
    stance obs, per sample within one batch."""
    model = _dual_model()
    pol = model.policy
    pol.set_training_mode(False)
    with th.no_grad():
        pol.action_net.weight.zero_()
        pol.action_net.bias.fill_(-0.3)
        pol.action_net_b.weight.zero_()
        pol.action_net_b.bias.fill_(0.7)
    obs = th.as_tensor(np.stack([
        np.concatenate([np.ones(3, np.float32) * 0.1, _onehot_tail(3)]),
        np.concatenate([np.ones(3, np.float32) * 0.1, _onehot_tail(0)]),
        np.concatenate([np.ones(3, np.float32) * 0.1, _onehot_tail(1)]),
        np.concatenate([np.ones(3, np.float32) * 0.1, _onehot_tail(4)]),
    ]))
    h = th.zeros(2, 4, 8)
    starts = th.ones(4)
    with th.no_grad():
        dist, _ = pol.get_distribution(obs, (h, th.zeros_like(h)), starts)
        mean = dist.distribution.mean
    exp = th.tensor([[-0.3, -0.3], [0.7, 0.7], [0.7, 0.7], [-0.3, -0.3]])
    assert th.allclose(mean, exp, atol=1e-6), (
        f"mode routing broken: {mean} vs {exp} (rows: walk->A, hold->B, "
        f"rise->B, turn->A)")


def _dual_param_groups(pol):
    core_a = (list(pol.lstm_actor.core_a.parameters())
              + list(pol.lstm_critic.core_a.parameters())
              + list(pol.mlp_extractor.parameters())
              + list(pol.action_net.parameters())
              + list(pol.value_net.parameters()))
    core_b = (list(pol.lstm_actor.core_b.parameters())
              + list(pol.lstm_critic.core_b.parameters())
              + list(pol.mlp_extractor_b.parameters())
              + list(pol.action_net_b.parameters())
              + list(pol.value_net_b.parameters()))
    return core_a, core_b


@pytest.mark.parametrize("loco", [True, False])
def test_dual_gradient_isolation(loco):
    """THE property the architecture exists for: a batch gated entirely
    to one family must leave the other core (GRU cells AND heads) with
    exactly zero gradient — walk ticks can never train the stance core
    or vice versa."""
    from sb3_contrib.common.recurrent.type_aliases import RNNStates

    model = _dual_model()
    pol = model.policy
    slot = 3 if loco else 0
    obs = th.as_tensor(np.stack([np.concatenate([
        np.random.default_rng(i).uniform(-1, 1, 3).astype(np.float32),
        _onehot_tail(slot)]) for i in range(8)]))
    actions = th.zeros(8, 2)
    h = th.zeros(2, 8, 8)
    states = RNNStates((h, th.zeros_like(h)), (h.clone(), th.zeros_like(h)))
    starts = th.ones(8)

    core_a, core_b = _dual_param_groups(pol)
    for p in core_a + core_b:
        p.grad = None
    values, log_prob, entropy = pol.evaluate_actions(
        obs, actions, states, starts)
    (values.sum() + log_prob.sum()).backward()

    hot, cold = (core_a, core_b) if loco else (core_b, core_a)
    hot_norm = sum(float(p.grad.abs().sum())
                   for p in hot if p.grad is not None)
    assert hot_norm > 0.0, "active core received no gradient"
    for p in cold:
        assert p.grad is None or float(p.grad.abs().sum()) == 0.0, (
            "gradient leaked into the gated-out core — the isolation "
            "the dual architecture exists for is broken")


def test_dual_save_load_stateful_roundtrip(tmp_path):
    from sb3_contrib import RecurrentPPO

    model = _dual_model()
    model.learn(64)
    zip_path = tmp_path / "dual.zip"
    model.save(zip_path)
    assert is_recurrent_checkpoint(zip_path)
    loaded = load_checkpoint_auto(zip_path)
    assert isinstance(loaded, RecurrentPPO)
    assert isinstance(loaded.policy, DualGruActorCriticPolicy)

    rng = np.random.default_rng(7)
    obs_seq = []
    for i in range(10):
        obs_seq.append(np.concatenate([
            rng.uniform(-1, 1, 3).astype(np.float32),
            _onehot_tail(3 if i % 2 == 0 else 0)]))
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
        assert state[0].shape == (2, 1, 8), \
            f"dual state facade broken: {state[0].shape}"
        return np.stack(acts)

    np.testing.assert_array_equal(rollout(model), rollout(loaded))


def test_dual_bptt_forward_matches_entry_points():
    """distill_gru's whole-episode path must agree with the production
    get_distribution path from zero states (same math, two routes)."""
    model = _dual_model()
    pol = model.policy
    pol.set_training_mode(False)
    t_len, b = 5, 3
    rng = np.random.default_rng(3)
    obs = np.zeros((t_len, b, 3 + N_MODE_OBS), dtype=np.float32)
    for k in range(b):
        slot = 3 if k % 2 == 0 else 0
        for t in range(t_len):
            obs[t, k] = np.concatenate([
                rng.uniform(-1, 1, 3).astype(np.float32),
                _onehot_tail(slot)])
    feats = th.as_tensor(obs)
    with th.no_grad():
        mu_bptt, _ = pol.bptt_forward(feats)
        # Reference: sequence-major flatten through get_distribution.
        flat = feats.transpose(0, 1).reshape(t_len * b, -1)
        starts = th.zeros(t_len * b)
        starts[::t_len] = 0.0  # sequences start fresh via zero h below
        h = th.zeros(2, b, 8)
        dist, _ = pol.get_distribution(flat, (h, th.zeros_like(h)), starts)
        mu_ref = dist.distribution.mean.reshape(b, t_len, -1).transpose(0, 1)
    assert th.allclose(mu_bptt, mu_ref, atol=1e-5), \
        "bptt_forward diverges from the production sequence path"


def test_dual_bc_anchor_mean_and_detach(tmp_path):
    """Recurrent BC anchor on the DUAL policy: the aux mean must match
    the stateful predict path at the stored hidden state, and
    detach_trunk must train only the per-core heads."""
    from stable_baselines3.common.vec_env import DummyVecEnv

    from sb3_contrib import RecurrentPPO

    from rl_move.sim.bc_anchor import make_bc_anchor_ppo_class

    class _DualAnchorEnv(_TinyDualEnv):
        def __init__(self):
            super().__init__()
            self.action_space = spaces.Box(-1, 1, (18,), dtype=np.float32)

    cls = make_bc_anchor_ppo_class(RecurrentPPO)
    venv = DummyVecEnv([_DualAnchorEnv for _ in range(2)])
    model = cls(
        DualGruActorCriticPolicy, venv,
        n_steps=8, batch_size=8, n_epochs=1, seed=0, device="cpu",
        policy_kwargs=dict(lstm_hidden_size=8, net_arch=[16]))
    pol = model.policy
    pol.set_training_mode(False)
    rng = np.random.default_rng(0)
    obs_np = np.stack([np.concatenate([
        rng.uniform(-1, 1, 3).astype(np.float32),
        _onehot_tail(3 if i % 2 == 0 else 0)]) for i in range(4)])
    h_np = rng.uniform(-1, 1, (4, 2 * 8)).astype(np.float32)

    with th.no_grad():
        mean = model._bc_policy_mean(
            th.as_tensor(obs_np), th.as_tensor(h_np))
    # Cross-check row 0 against the production stateful predict.
    h1 = h_np[:1].reshape(1, 2, 8).transpose(1, 0, 2)
    a, _ = pol.predict(
        obs_np[:1], state=(h1, np.zeros_like(h1)),
        episode_start=np.zeros(1, dtype=bool), deterministic=True)
    np.testing.assert_allclose(
        a[0], np.clip(mean.numpy()[0], -1, 1), atol=1e-5)

    # detach_trunk: gradient reaches BOTH cores' heads (mixed-mode
    # batch), never the feature extractor or either GRU cell.
    model.bc_detach_trunk = True
    trunk = (list(pol.features_extractor.parameters())
             + list(pol.lstm_actor.parameters()))
    heads = (list(pol.mlp_extractor.parameters())
             + list(pol.action_net.parameters())
             + list(pol.mlp_extractor_b.parameters())
             + list(pol.action_net_b.parameters()))
    for p in trunk + heads:
        p.grad = None
    tgt = th.as_tensor(rng.uniform(-1, 1, (4, 18)).astype(np.float32))
    mean = model._bc_policy_mean(th.as_tensor(obs_np), th.as_tensor(h_np))
    th.nn.functional.mse_loss(mean, tgt).backward()
    for p in trunk:
        assert p.grad is None or float(p.grad.abs().sum()) == 0.0, \
            "detach_trunk leaked gradient into a recurrent core"
    for group, name in ((list(pol.action_net.parameters()), "A"),
                        (list(pol.action_net_b.parameters()), "B")):
        norm = sum(float(p.grad.abs().sum())
                   for p in group if p.grad is not None)
        assert norm > 0.0, f"anchor gradient never reached head {name}"


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
