"""Bank for actor_only_transplant (operator addendum
fb_20260818T085834_588d9a, walkcurr4 tournament arms B/C): copying
ONLY the actor weights of a plain single-frame checkpoint (e.g. the
scripted-gait BC-INIT or a gait-hardened RL checkpoint) into a fresh
condition-D PredictiveCriticPolicy over the history-stacked obs.

Proves:
  1. actor tensors that need no widening copy EXACTLY (log_std,
     action_net, mlp_extractor.policy_net's later layers);
  2. the first-layer weight matrix zero-pads the extra (older-history)
     columns, so the transplanted actor's action distribution is
     BIT-IDENTICAL to the source policy for ANY value of the extra
     history dims;
  3. every critic-marked tensor (mlp_extractor.value_net, value_net,
     value_gate, latent_adapter) and the frozen predictor snapshot
     (critic_predictor, obs_to_frames, snapshot_version) are left
     EXACTLY as the fresh construction built them — never read from
     the source checkpoint;
  4. a genuinely incompatible shape (not a tail-widening) raises;
  5. the CLI guards in train_ppo_mjx wire the flag correctly (requires
     --critic-encoder + --init-from, conflicts with
     --obs-pad-transplant, and the plain --critic-encoder + --init-from
     combo without the new flag is still refused).
"""
import numpy as np
import pytest
import torch as th
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from rl_move.dynamics import data as dd
from rl_move.dynamics import frames as fr
from rl_move.dynamics.model import DynamicsModel
from rl_move.dynamics.predictive_critic import (
    PredictiveCriticPolicy, PredictiveCriticPPO, actor_only_transplant,
    raw_policy_backbone_transplant,
)
from rl_move.sim.update_health import CRITIC_MARKERS
from rl_move.sim import train_ppo_mjx

HORIZONS = (1, 2, 5, 10, 25)
HISTORY = 16
FRAME_WIDTH = 72
OBS_DIM = HISTORY * FRAME_WIDTH


def _episode(global_idx: int, n_frames: int, seed: int) -> dd.Episode:
    rng = np.random.default_rng(seed)
    return dd.Episode(
        frames=rng.standard_normal((n_frames, fr.FRAME_DIM)).astype(
            np.float32),
        actions=rng.standard_normal((n_frames - 1, fr.ACTION_DIM)).astype(
            np.float32),
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


@pytest.fixture(scope="module")
def encoder_ckpt(tmp_path_factory):
    episodes = [_episode(i, HISTORY + HORIZONS[-1] + 6, seed=i)
                for i in range(8)]
    stats = dd.compute_stats(episodes)
    model = _tiny_model()
    path = tmp_path_factory.mktemp("enc") / "tiny_dyn_obs.pt"
    th.save({"layout_version": fr.LAYOUT_VERSION,
             "config": model.config(), "model": model.state_dict(),
             "stats": stats.to_dict(), "history": HISTORY}, path)
    return path


class _SmallEnv(gym.Env):
    """Plain single-frame obs (e.g. a bcgait1_hard1-style checkpoint)."""
    observation_space = spaces.Box(-10.0, 10.0, (FRAME_WIDTH,),
                                   dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, (fr.ACTION_DIM,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        return np.zeros(FRAME_WIDTH, dtype=np.float32), {}

    def step(self, action):
        return (np.zeros(FRAME_WIDTH, dtype=np.float32), 0.0, False,
                True, {})


class _BigEnv(gym.Env):
    """History-stacked obs (newest-first): condition-D's env contract."""
    observation_space = spaces.Box(-10.0, 10.0, (OBS_DIM,), dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, (fr.ACTION_DIM,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        return np.zeros(OBS_DIM, dtype=np.float32), {}

    def step(self, action):
        return (np.zeros(OBS_DIM, dtype=np.float32), 0.0, False, True, {})


def _old_model(seed=1):
    return PPO("MlpPolicy", DummyVecEnv([_SmallEnv]),
              policy_kwargs=dict(net_arch=[16, 16], log_std_init=-1.0),
              n_steps=8, batch_size=8, seed=seed, verbose=0, device="cpu")


def _new_model(encoder_ckpt, seed=2):
    return PredictiveCriticPPO(
        PredictiveCriticPolicy, DummyVecEnv([_BigEnv]),
        policy_kwargs=dict(
            net_arch=[16, 16], log_std_init=-1.0,
            predictor_ckpt=str(encoder_ckpt),
            frame_width=FRAME_WIDTH, history=HISTORY),
        n_steps=8, batch_size=8, seed=seed, verbose=0, device="cpu")


CRIT_MARKERS = CRITIC_MARKERS + ("value_gate", "latent_adapter")


def test_transplant_copies_unwidened_actor_tensors_exactly(encoder_ckpt):
    old = _old_model()
    new = _new_model(encoder_ckpt)
    copied = actor_only_transplant(old, new)
    sd_old, sd_new = old.policy.state_dict(), new.policy.state_dict()
    for name in ("log_std", "action_net.weight", "action_net.bias",
                 "mlp_extractor.policy_net.2.weight",
                 "mlp_extractor.policy_net.2.bias"):
        assert name in copied
        assert th.equal(sd_new[name], sd_old[name]), name


def test_transplant_zero_pads_widened_first_layer(encoder_ckpt):
    old = _old_model()
    new = _new_model(encoder_ckpt)
    actor_only_transplant(old, new)
    w_old = old.policy.state_dict()["mlp_extractor.policy_net.0.weight"]
    w_new = new.policy.state_dict()["mlp_extractor.policy_net.0.weight"]
    assert w_old.shape == (16, FRAME_WIDTH)
    assert w_new.shape == (16, OBS_DIM)
    assert th.equal(w_new[:, :FRAME_WIDTH], w_old)
    assert th.count_nonzero(w_new[:, FRAME_WIDTH:]) == 0
    b_old = old.policy.state_dict()["mlp_extractor.policy_net.0.bias"]
    b_new = new.policy.state_dict()["mlp_extractor.policy_net.0.bias"]
    assert th.equal(b_new, b_old)   # bias has no width dependency


def test_transplant_leaves_critic_and_frozen_encoder_untouched(encoder_ckpt):
    old = _old_model()
    new = _new_model(encoder_ckpt, seed=42)
    baseline = {n: p.detach().clone()
                for n, p in new.policy.state_dict().items()}
    old_keys = set(old.policy.state_dict())
    actor_only_transplant(old, new)
    sd_new = new.policy.state_dict()
    untouched_checked = 0
    for name, base_v in baseline.items():
        if any(m in name for m in CRIT_MARKERS) or name not in old_keys:
            assert th.equal(sd_new[name], base_v), (
                f"{name} was mutated by an actor-only transplant")
            untouched_checked += 1
    # sanity: the frozen encoder + critic residual keys were actually
    # exercised by the loop above (not vacuously true).
    assert any(n.startswith("critic_predictor.") for n in baseline)
    assert any(n.startswith("obs_to_frames.") for n in baseline)
    assert "snapshot_version" in baseline and "value_gate" in baseline
    assert untouched_checked >= 10


def test_transplant_matches_source_regardless_of_history_tail(encoder_ckpt):
    old = _old_model()
    new = _new_model(encoder_ckpt)
    actor_only_transplant(old, new)
    rng = np.random.default_rng(0)
    current_frame = rng.standard_normal(FRAME_WIDTH).astype(np.float32)
    for _ in range(3):
        older_tail = rng.standard_normal(OBS_DIM - FRAME_WIDTH).astype(
            np.float32)
        big_obs = th.as_tensor(
            np.concatenate([current_frame, older_tail])[None, :])
        small_obs = th.as_tensor(current_frame[None, :])
        old_mean = old.policy.get_distribution(small_obs).distribution.mean
        new_mean = new.policy.get_distribution(big_obs).distribution.mean
        assert th.allclose(old_mean, new_mean, atol=1e-5), (
            "transplanted actor must ignore the older-history tail at "
            "init and reproduce the source policy exactly")


def test_transplant_raises_on_incompatible_shape(encoder_ckpt):
    old = PPO("MlpPolicy", DummyVecEnv([_SmallEnv]),
             policy_kwargs=dict(net_arch=[32, 16], log_std_init=-1.0),
             n_steps=8, batch_size=8, seed=1, verbose=0, device="cpu")
    new = _new_model(encoder_ckpt)
    with pytest.raises(SystemExit, match="unexpected shape change"):
        actor_only_transplant(old, new)


def test_backbone_transplant_warm_starts_raw_critic_only(encoder_ckpt):
    old = _old_model()
    new = _new_model(encoder_ckpt, seed=42)
    baseline = {n: v.detach().clone()
                for n, v in new.policy.state_dict().items()}
    copied = raw_policy_backbone_transplant(old, new)
    sd_old, sd_new = old.policy.state_dict(), new.policy.state_dict()
    for name in ("action_net.weight", "value_net.weight",
                 "mlp_extractor.policy_net.2.weight",
                 "mlp_extractor.value_net.2.weight"):
        assert name in copied
        assert th.equal(sd_new[name], sd_old[name])
    for name in ("value_gate", "snapshot_version"):
        assert th.equal(sd_new[name], baseline[name])
    assert all(th.equal(v, baseline[n]) for n, v in sd_new.items()
               if n.startswith("critic_predictor."))
    w_old = sd_old["mlp_extractor.value_net.0.weight"]
    w_new = sd_new["mlp_extractor.value_net.0.weight"]
    assert th.equal(w_new[:, :FRAME_WIDTH], w_old)
    assert th.count_nonzero(w_new[:, FRAME_WIDTH:]) == 0


# -- CLI wiring (train_ppo_mjx.main early validation) -------------------

def _run_main_expect_exit(monkeypatch, argv, match):
    monkeypatch.setattr(train_ppo_mjx, "mjx_is_available", lambda: True)
    with pytest.raises(SystemExit, match=match):
        train_ppo_mjx.main(argv)


def test_actor_only_flag_requires_critic_encoder(monkeypatch, tmp_path):
    fake = tmp_path / "x.zip"
    fake.write_bytes(b"0")
    _run_main_expect_exit(
        monkeypatch,
        ["--run-name", "t", "--no-wandb", "--init-from", str(fake),
         "--init-from-actor-only"],
        "requires --critic-encoder")


def test_actor_only_flag_requires_init_from(monkeypatch, tmp_path):
    fake = tmp_path / "enc.pt"
    fake.write_bytes(b"0")
    _run_main_expect_exit(
        monkeypatch,
        ["--run-name", "t", "--no-wandb", "--init-from-actor-only",
         "--critic-encoder", str(fake)],
        "requires --init-from")


def test_actor_only_flag_conflicts_with_obs_pad_transplant(monkeypatch,
                                                            tmp_path):
    fake = tmp_path / "x.zip"
    fake.write_bytes(b"0")
    _run_main_expect_exit(
        monkeypatch,
        ["--run-name", "t", "--no-wandb", "--init-from", str(fake),
         "--init-from-actor-only", "--obs-pad-transplant", "2",
         "--critic-encoder", str(fake), "--critic-encoder-md5", "deadbeef",
         "--cfg-set", "obs.history_frames=16"],
        "does not compose with --obs-pad-transplant")


def test_critic_encoder_plain_init_from_still_refused_without_flag(
        monkeypatch, tmp_path):
    fake = tmp_path / "x.zip"
    fake.write_bytes(b"0")
    _run_main_expect_exit(
        monkeypatch,
        ["--run-name", "t", "--no-wandb", "--init-from", str(fake),
         "--critic-encoder", str(fake), "--critic-encoder-md5", "deadbeef",
         "--cfg-set", "obs.history_frames=16"],
        "not wired for a full-checkpoint --init-from")


def test_backbone_flag_requires_predictive_live(monkeypatch, tmp_path):
    fake = tmp_path / "x.zip"
    fake.write_bytes(b"0")
    _run_main_expect_exit(
        monkeypatch,
        ["--run-name", "t", "--no-wandb", "--init-from", str(fake),
         "--init-from-policy-backbone"],
        "requires --predictive-live")
