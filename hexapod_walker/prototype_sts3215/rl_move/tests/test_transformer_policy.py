"""Transformer policy correctness + learning tests.

Before spending GPU budget on a from-scratch transformer walk arm,
pin down that the implementation is sound (the GRU line taught us to
test the mechanism BEFORE blaming the recipe):

1. the frame-stack reshape maps env frames (stacked NEWEST-FIRST) onto
   time-ordered tokens correctly, and the causal mask really is causal
   (token t has zero gradient from frames newer than t);
2. the policy head reads the NEWEST token (the current tick);
3. PPO+transformer checkpoints survive a save/load round trip with
   bit-identical deterministic predictions, and ``load_checkpoint_auto``
   returns plain PPO for them (eval/viewer paths unchanged);
4. actor and critic really get SEPARATE transformers by default;
5. the whole training stack can LEARN a task that strictly requires
   using the history window (a current-frame policy is capped at a
   known score) — the test that would catch a silent trunk bug.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch as th

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import gymnasium as gym  # noqa: E402
from gymnasium import spaces  # noqa: E402

from rl_move.sim.gru_policy import (  # noqa: E402
    is_recurrent_checkpoint,
    load_checkpoint_auto,
)
from rl_move.sim.transformer_policy import (  # noqa: E402
    FrameStackTransformerExtractor,
    TransformerActorCriticPolicy,
)


def _extractor(k=4, w=3, **kw):
    space = spaces.Box(-1, 1, (k * w,), dtype=np.float32)
    kw.setdefault("d_model", 16)
    kw.setdefault("n_layers", 2)
    kw.setdefault("n_heads", 2)
    kw.setdefault("ff_dim", 32)
    return FrameStackTransformerExtractor(space, n_frames=k, **kw)


# ---------------------------------------------------------------------------
# 1. frame ordering + causal mask
# ---------------------------------------------------------------------------

def test_obs_width_mismatch_raises():
    with pytest.raises(ValueError, match="not a multiple"):
        FrameStackTransformerExtractor(
            spaces.Box(-1, 1, (13,), dtype=np.float32), n_frames=4)
    with pytest.raises(ValueError, match="history"):
        FrameStackTransformerExtractor(
            spaces.Box(-1, 1, (12,), dtype=np.float32), n_frames=1)


@pytest.mark.parametrize("token_pos", [0, 1, 3])
def test_causal_mask_and_frame_flip(token_pos):
    """Token at time position t must receive gradient ONLY from obs
    frames at time <= t. Env frames are newest-first, so obs frame
    index f sits at time position K-1-f: token t may see exactly the
    frames f >= K-1-t."""
    th.manual_seed(0)
    k, w = 4, 3
    ext = _extractor(k, w)
    obs = th.randn(1, k * w, requires_grad=True)
    tok = ext.tokens(obs)
    tok[0, token_pos].sum().backward()
    g = obs.grad.view(k, w).abs().sum(dim=1)  # per obs-frame grad mass
    for f in range(k):
        visible = f >= k - 1 - token_pos
        if visible:
            assert g[f] > 0.0, (
                f"token {token_pos} got no gradient from visible frame "
                f"{f} (flip mapping broken)")
        else:
            assert g[f] == 0.0, (
                f"token {token_pos} leaked gradient from FUTURE frame "
                f"{f} — causal mask broken")


def test_forward_reads_newest_token():
    th.manual_seed(1)
    k, w = 4, 3
    ext = _extractor(k, w)
    obs = th.randn(2, k * w)
    with th.no_grad():
        want = ext.out_norm(ext.tokens(obs)[:, -1])
        got = ext(obs)
    assert th.equal(got, want)
    # Newest frame (obs frame 0) must influence the output.
    obs2 = obs.clone()
    obs2[:, :w] += 1.0
    with th.no_grad():
        assert not th.allclose(ext(obs2), got)


# ---------------------------------------------------------------------------
# Toy envs
# ---------------------------------------------------------------------------

class _TinyStackEnv(gym.Env):
    """Trivial continuous env with a K*W frame-stack-shaped obs."""

    K, W = 4, 3

    def __init__(self):
        self.observation_space = spaces.Box(
            -1, 1, (self.K * self.W,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (2,), dtype=np.float32)
        self._t = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._t = 0
        return self.observation_space.sample(), {}

    def step(self, action):
        self._t += 1
        return (self.observation_space.sample(), 0.0, False,
                self._t >= 8, {})


class _StackedMemoryEnv(gym.Env):
    """Frame-stacked memory task (windowed twin of gru tests'
    _MemoryEnv).

    Each single-tick frame is [signal, t/T]; signal = s0 in {-1,+1} on
    the t=0 frame only, 0 after. The env stacks the last K=8 frames
    NEWEST-FIRST (zero-padded), exactly like sim_env._final_obs. With
    T=6 < K the s0 frame stays inside the window all episode, so a
    policy that attends over the window scores ~1.0/step while a
    current-frame-only policy is capped at 0.5.
    """

    T = 6
    K = 8
    W = 2

    def __init__(self):
        self.observation_space = spaces.Box(
            -1, 1, (self.K * self.W,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (1,), dtype=np.float32)
        self._t = 0
        self._s0 = 1.0
        self._hist: list[np.ndarray] = []

    def _obs(self):
        frames = list(reversed(self._hist[-self.K:]))  # newest first
        while len(frames) < self.K:
            frames.append(np.zeros(self.W, dtype=np.float32))
        return np.concatenate(frames)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._t = 0
        self._s0 = float(self.np_random.choice([-1.0, 1.0]))
        self._hist = [np.array([self._s0, 0.0], dtype=np.float32)]
        return self._obs(), {}

    def step(self, action):
        self._t += 1
        r = 1.0 - abs(float(action[0]) - self._s0) / 2.0
        self._hist.append(
            np.array([0.0, self._t / self.T], dtype=np.float32))
        return self._obs(), r, False, self._t >= self.T, {}


def _tiny_model(env_ctor=_TinyStackEnv, **pk):
    from stable_baselines3 import PPO
    pk.setdefault("n_frames", env_ctor.K)
    pk.setdefault("d_model", 16)
    pk.setdefault("n_layers", 2)
    pk.setdefault("n_heads", 2)
    pk.setdefault("ff_dim", 32)
    pk.setdefault("net_arch", [16])
    return PPO(TransformerActorCriticPolicy, env_ctor(),
               n_steps=8, batch_size=16, n_epochs=1, seed=0,
               device="cpu", policy_kwargs=pk)


# ---------------------------------------------------------------------------
# 3+4. save/load round trip, auto loader, separate extractors
# ---------------------------------------------------------------------------

def test_separate_actor_critic_transformers():
    model = _tiny_model()
    pol = model.policy
    assert isinstance(pol.pi_features_extractor,
                      FrameStackTransformerExtractor)
    assert isinstance(pol.vf_features_extractor,
                      FrameStackTransformerExtractor)
    assert pol.pi_features_extractor is not pol.vf_features_extractor, \
        "actor and critic must get SEPARATE transformers by default"
    # Both must be trained: the optimizer has to cover both extractors.
    opt_params = {id(p) for grp in pol.optimizer.param_groups
                  for p in grp["params"]}
    for name in ("pi_features_extractor", "vf_features_extractor"):
        for pname, p in getattr(pol, name).named_parameters():
            assert id(p) in opt_params, \
                f"{name}.{pname} missing from the optimizer"


def test_save_load_roundtrip_and_auto_loader(tmp_path):
    from stable_baselines3 import PPO

    model = _tiny_model()
    model.learn(64)
    tf_zip = tmp_path / "tf.zip"
    model.save(tf_zip)

    assert not is_recurrent_checkpoint(tf_zip)
    loaded = load_checkpoint_auto(tf_zip)
    assert isinstance(loaded, PPO)
    assert isinstance(loaded.policy, TransformerActorCriticPolicy)
    assert isinstance(loaded.policy.pi_features_extractor,
                      FrameStackTransformerExtractor)

    for m in (model, loaded):
        m.policy.set_training_mode(False)
    obs = np.stack([np.random.default_rng(i)
                    .uniform(-1, 1, 12).astype(np.float32)
                    for i in range(16)])
    a0, _ = model.predict(obs, deterministic=True)
    a1, _ = loaded.predict(obs, deterministic=True)
    np.testing.assert_array_equal(a0, a1)


def test_mirror_ppo_composes(tmp_path):
    """MirrorPPO only needs policy.get_distribution(obs) — verify the
    aux symmetry step runs against a transformer policy without error
    (maps here are identity perms on the toy env; the REAL maps are
    layout-locked by test_mirror.py)."""
    from rl_move.sim.mirror import make_mirror_ppo_class

    cls = make_mirror_ppo_class()
    model = cls(TransformerActorCriticPolicy, _TinyStackEnv(),
                n_steps=8, batch_size=16, n_epochs=1, seed=0,
                device="cpu",
                policy_kwargs=dict(n_frames=4, d_model=16, n_layers=1,
                                   n_heads=2, ff_dim=32, net_arch=[]))
    n_obs = 12
    model.mirror_coef = 1.0
    model.mirror_obs_perm = np.arange(n_obs)
    model.mirror_obs_sign = np.ones(n_obs, dtype=np.float32)
    model.mirror_act_perm = np.arange(2)
    model.mirror_act_sign = np.ones(2, dtype=np.float32)
    model.learn(32)  # raises if the aux step can't drive the policy


# ---------------------------------------------------------------------------
# 5. it must actually learn a history task (slow, ~1-2 min CPU)
# ---------------------------------------------------------------------------

@pytest.mark.slow
def test_transformer_learns_windowed_memory_task():
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv

    venv = DummyVecEnv([_StackedMemoryEnv for _ in range(16)])
    model = PPO(
        TransformerActorCriticPolicy, venv,
        n_steps=24, batch_size=96, n_epochs=5, learning_rate=3e-4,
        ent_coef=0.0, gamma=0.9, seed=0, device="cpu",
        policy_kwargs=dict(n_frames=_StackedMemoryEnv.K, d_model=32,
                           n_layers=2, n_heads=2, ff_dim=64,
                           net_arch=[]))
    model.learn(60_000)
    model.policy.set_training_mode(False)

    env = _StackedMemoryEnv()
    rewards = []
    for ep in range(40):
        obs, _ = env.reset(seed=1000 + ep)
        done = False
        while not done:
            a, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = env.step(a)
            rewards.append(r)
            done = term or trunc
    mean_r = float(np.mean(rewards))
    # Current-frame cap is 0.5/step; require clear water above it.
    assert mean_r > 0.75, (
        f"transformer scored {mean_r:.3f}/step on the windowed memory "
        f"task; current-frame cap is 0.5 — the attention trunk is not "
        f"using the history window")
