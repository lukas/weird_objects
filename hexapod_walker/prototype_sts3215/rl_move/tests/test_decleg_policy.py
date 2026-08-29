"""Unit tests for the decentralized per-leg actor policy (walkcurr
rung-1, Schilling IROS 2020 lever — operator ruling
fb_20260829T145710).  Fast, CPU-only, no MuJoCo."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

torch = pytest.importorskip("torch")
gym = pytest.importorskip("gymnasium")

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv  # noqa: E402

from rl_move.sim.decleg_policy import (  # noqa: E402
    ACT_PER_LEG, N_LEGS, DecLegActorCriticPolicy, joint_walk_leg_slices,
)

OBS_W = 72   # joint_walk env width (59 base + 11 goal + 2 vel feedback)
ACT_W = 18


def _policy(seed: int = 0) -> DecLegActorCriticPolicy:
    torch.manual_seed(seed)
    legs, shared = joint_walk_leg_slices(OBS_W)
    obs_space = gym.spaces.Box(-np.inf, np.inf, shape=(OBS_W,),
                               dtype=np.float32)
    act_space = gym.spaces.Box(-1.0, 1.0, shape=(ACT_W,),
                               dtype=np.float32)
    return DecLegActorCriticPolicy(
        obs_space, act_space, lambda _: 3e-4,
        net_arch=[128, 64, 32], leg_obs_idx=legs, shared_obs_idx=shared,
        leg_hidden=(64, 64))


def test_slices_partition():
    legs, shared = joint_walk_leg_slices(OBS_W)
    assert len(legs) == N_LEGS and all(len(ix) == 9 for ix in legs)
    flat = [j for ix in legs for j in ix]
    assert len(set(flat)) == 54          # disjoint locals
    # every obs dim is either local-to-exactly-one-leg or shared, never both
    assert not (set(flat) & set(shared))
    assert sorted(set(flat) | set(shared)) == list(range(OBS_W))
    # leg 2 owns q 6:9, qd 24:27, prev 47:50
    assert legs[2] == [6, 7, 8, 24, 25, 26, 47, 48, 49]


def test_slices_other_widths():
    for w in (68, 74):
        legs, shared = joint_walk_leg_slices(w)
        assert sorted({j for ix in legs for j in ix} | set(shared)) \
            == list(range(w))
    with pytest.raises(ValueError):
        joint_walk_leg_slices(47)


def test_strict_per_leg_decentralization():
    """Perturbing a leg-local obs dim must change ONLY that leg's
    action mean; a shared dim may change everything."""
    pol = _policy()
    pol.eval()
    legs, shared = joint_walk_leg_slices(OBS_W)
    base = torch.zeros(1, OBS_W)
    with torch.no_grad():
        a0 = pol.action_net(pol.mlp_extractor.forward_actor(base))
    for leg in range(N_LEGS):
        for dim in legs[leg]:
            obs = base.clone()
            obs[0, dim] = 1.7
            with torch.no_grad():
                a = pol.action_net(pol.mlp_extractor.forward_actor(obs))
            delta = (a - a0).abs().numpy().ravel()
            own = delta[leg * ACT_PER_LEG:(leg + 1) * ACT_PER_LEG]
            others = np.delete(delta,
                               range(leg * ACT_PER_LEG,
                                     (leg + 1) * ACT_PER_LEG))
            assert others.max() == 0.0, (
                f"obs dim {dim} (leg {leg} local) leaked into other "
                f"legs' actions: max |delta| {others.max()}")
            assert own.max() > 0.0, (
                f"obs dim {dim} did not reach its own leg {leg}")
    # shared dim reaches every leg (sanity that modules aren't blind)
    obs = base.clone()
    obs[0, shared[0]] = 1.7
    with torch.no_grad():
        a = pol.action_net(pol.mlp_extractor.forward_actor(obs))
    delta = (a - a0).abs().reshape(N_LEGS, ACT_PER_LEG).numpy()
    assert (delta.max(axis=1) > 0.0).all()


def test_critic_is_centralized():
    pol = _policy()
    base = torch.zeros(1, OBS_W)
    obs = base.clone()
    obs[0, 0] = 2.0   # leg-0 local dim
    with torch.no_grad():
        v0 = pol.predict_values(base)
        v1 = pol.predict_values(obs)
    assert (v0 - v1).abs().item() > 0.0


class _DummyJointEnv(gym.Env):
    observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(OBS_W,),
                                       dtype=np.float32)
    action_space = gym.spaces.Box(-1.0, 1.0, shape=(ACT_W,),
                                  dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return np.zeros(OBS_W, dtype=np.float32), {}

    def step(self, action):
        return (np.zeros(OBS_W, dtype=np.float32), 0.0, False, False,
                {})


def test_ppo_save_load_roundtrip(tmp_path):
    legs, shared = joint_walk_leg_slices(OBS_W)
    venv = DummyVecEnv([_DummyJointEnv])
    model = PPO(DecLegActorCriticPolicy, venv, n_steps=8, batch_size=8,
                seed=3, device="cpu",
                policy_kwargs=dict(net_arch=[128, 64, 32],
                                   leg_obs_idx=legs,
                                   shared_obs_idx=shared,
                                   leg_hidden=(64, 64)))
    obs = np.random.RandomState(0).randn(1, OBS_W).astype(np.float32)
    a_before, _ = model.predict(obs, deterministic=True)
    p = tmp_path / "decleg.zip"
    model.save(p)
    loaded = PPO.load(p, device="cpu")
    a_after, _ = loaded.predict(obs, deterministic=True)
    np.testing.assert_allclose(a_before, a_after, rtol=0, atol=0)
    assert isinstance(loaded.policy, DecLegActorCriticPolicy)


def test_optimizer_covers_all_params():
    """The swapped-in per-leg head must be optimizer-registered (the
    stock action_net was registered by super()._build and then
    replaced — a stale optimizer would silently freeze the head)."""
    pol = _policy()
    opt_params = {id(p) for g in pol.optimizer.param_groups
                  for p in g["params"]}
    model_params = {id(p) for p in pol.parameters()}
    assert model_params <= opt_params
    head_params = {id(p) for p in pol.action_net.parameters()}
    assert head_params <= opt_params
