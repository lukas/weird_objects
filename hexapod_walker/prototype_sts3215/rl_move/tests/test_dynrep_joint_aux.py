"""Regression tests for the corrected condition-C joint PPO+auxiliary
update (operator directive fb_20260816T203212_af7c64).

The retired v1 mechanism (AnchorCb) ran out-of-band Adam steps on the
SHARED transformer after rollout collection and before PPO consumed the
buffer — measured harmful in the metrics1 cohort (C led at 1M, dead
last at 2M, approx_kl ~4x A/B). These tests prove, on tiny real
components (real DynamicsModel, real WindowSampler, real SB3 PPO loop):

  1. the shared transformer CANNOT be mutated out-of-band between
     rollout collection and the PPO update — JointAuxPPO raises;
  2. the head warmup keeps the encoder bit-frozen, then joint training
     moves it as part of the coordinated update;
  3. the total action-KL guard rejects/rolls back combined updates and
     stops the auxiliary after repeated rejections;
  4. auxiliary batches honor the online-primary / rehearsal-mix
     contract.
"""
import numpy as np
import pytest
import torch as th
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from rl_move.dynamics import data as dd
from rl_move.dynamics import frames as fr
from rl_move.dynamics.joint_aux import AuxConfig, JointAuxPPO
from rl_move.dynamics.model import DynamicsModel
from rl_move.dynamics.online_windows import (
    OnlineWindowBuffer, concat_batches, mixed_batch,
)
from rl_move.dynamics.sb3_encoder import DynFeaturesExtractor, set_group_lrs
from rl_move.dynamics.train_ppo_transfer import anchor_batch_to_torch

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
def corpus():
    episodes = [_episode(i, HISTORY + HORIZONS[-1] + 6, seed=i)
                for i in range(40)]
    stats = dd.compute_stats(episodes)
    return episodes, stats


@pytest.fixture()
def encoder_ckpt(tmp_path, corpus):
    _, stats = corpus
    model = _tiny_model()
    path = tmp_path / "tiny_dyn_obs.pt"
    th.save({"layout_version": fr.LAYOUT_VERSION,
             "config": model.config(), "model": model.state_dict(),
             "stats": stats.to_dict(), "history": HISTORY}, path)
    return path


class _TinyEnv(gym.Env):
    observation_space = spaces.Box(-10.0, 10.0, (OBS_DIM,),
                                   dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, (fr.ACTION_DIM,),
                              dtype=np.float32)

    def __init__(self):
        self._rng = np.random.default_rng(0)
        self._t = 0

    def _obs(self):
        return self._rng.standard_normal(OBS_DIM).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        self._t = 0
        return self._obs(), {}

    def step(self, action):
        self._t += 1
        return (self._obs(), float(action[0]) * 0.01, False,
                self._t >= 20, {})


def _build_model(encoder_ckpt, corpus, *, warmup=0, kl_guard=1e9,
                 stop_after=3, rehearsal_frac=0.25,
                 min_online_windows=10**9, seed=0):
    episodes, stats = corpus
    venv = DummyVecEnv([_TinyEnv])
    model = JointAuxPPO(
        "MlpPolicy", venv,
        policy_kwargs=dict(
            net_arch=[16, 16], log_std_init=-1.0,
            features_extractor_class=DynFeaturesExtractor,
            features_extractor_kwargs=dict(
                ckpt_path=str(encoder_ckpt), frame_width=FRAME_WIDTH,
                history=HISTORY, freeze=False)),
        n_steps=32, batch_size=32, n_epochs=2, learning_rate=3e-4,
        seed=seed, verbose=0, device="cpu")
    model.policy.features_extractor.reload_pretrained()
    set_group_lrs(model.policy, 3e-4, 0.1)
    cfg = AuxConfig(coef=1.0, batch_size=16,
                    rehearsal_frac=rehearsal_frac, warmup_steps=warmup,
                    kl_target=0.02, kl_guard=kl_guard,
                    stop_after=stop_after,
                    min_online_windows=min_online_windows, probe_obs=32)
    rehearsal = dd.WindowSampler(episodes, stats, HISTORY, HORIZONS,
                                 val=False, seed=0)
    buffer = OnlineWindowBuffer(stats, HISTORY, HORIZONS,
                                max_frames=10_000, seed=0)
    payloads = []
    model.configure_aux(cfg, buffer, rehearsal, anchor_batch_to_torch,
                        metrics_sink=lambda p, s: payloads.append(p))
    return model, payloads


def _dyn_params(model):
    return [p.detach().clone()
            for p in model.policy.features_extractor.dyn.parameters()]


def test_out_of_band_encoder_mutation_is_refused(encoder_ckpt, corpus):
    """The exact failure mode of the retired AnchorCb: touching the
    shared transformer between rollout collection and the PPO update
    must raise, not silently corrupt the old-policy assumptions."""
    model, _ = _build_model(encoder_ckpt, corpus)

    class RogueAnchorCb(BaseCallback):
        def _on_rollout_end(self) -> None:
            with th.no_grad():
                next(iter(self.model.policy.features_extractor
                          .dyn.parameters())).add_(1.0)

        def _on_step(self) -> bool:
            return True

    with pytest.raises(RuntimeError, match="OUT-OF-BAND"):
        model.learn(total_timesteps=64, callback=RogueAnchorCb())


def test_clean_learn_runs_and_reports_aux_metrics(encoder_ckpt, corpus):
    model, payloads = _build_model(encoder_ckpt, corpus)
    model.learn(total_timesteps=64)
    assert payloads, "aux metrics sink never called"
    last = payloads[-1]
    assert last["aux/active"] == 1
    assert np.isfinite(last["aux/action_kl_total"])
    assert np.isfinite(last["aux/latent_drift"])
    assert np.isfinite(last["aux/train/total"])
    # per-target prediction metrics (velocity/heading via priv groups,
    # contacts, currents, servo state) must be reported
    for key in ("aux/train/now/vel", "aux/train/now/yaw_heading",
                "aux/train/h1/joint_pos", "aux/train/h1/contact_bce",
                "aux/train/h1/motor_current",
                "aux/train/current/motor_current"):
        assert key in last, sorted(last)
    # rehearsal-only until the online buffer fills (min_online_windows
    # is huge in this fixture)
    assert last["aux/rehearsal_frac"] == 1.0
    assert last["aux/batches_accepted_total"] > 0


def test_head_warmup_freezes_encoder_then_joint_training_moves_it(
        encoder_ckpt, corpus):
    model, payloads = _build_model(encoder_ckpt, corpus, warmup=64)
    before = _dyn_params(model)
    model.learn(total_timesteps=32)   # still inside warmup
    mid = _dyn_params(model)
    assert all(th.equal(a, b) for a, b in zip(before, mid)), \
        "encoder moved during the head warmup"
    assert payloads[-1]["aux/active"] == 0
    model.learn(total_timesteps=64, reset_num_timesteps=False)
    after = _dyn_params(model)
    assert not all(th.equal(a, b) for a, b in zip(mid, after)), \
        "encoder never trained after the warmup"
    assert payloads[-1]["aux/encoder_unfrozen"] == 1
    assert payloads[-1]["aux/active"] == 1


def test_kl_guard_rejects_and_rolls_back_without_misattribution(
        encoder_ckpt, corpus):
    """guard=0: every combined update breaches, but so does every no-aux
    retry (any update moves the policy), so the breach is NOT
    attributable to the auxiliary — it must be rolled back and logged
    rejected, but must NOT advance the permanent-stop counter."""
    model, payloads = _build_model(encoder_ckpt, corpus, kl_guard=0.0,
                                   stop_after=2)
    model.learn(total_timesteps=96)
    rejected = [p for p in payloads if p["aux/rejected"]]
    assert rejected, "guard at 0.0 must reject aux-active updates"
    assert payloads[-1]["aux/updates_rejected_total"] >= 2
    assert all("aux/action_kl_retry" in p for p in rejected)
    assert payloads[-1]["aux/stopped"] == 0, \
        "non-attributable breaches must not stop the auxiliary"


def test_kl_guard_stops_aux_on_attributable_breaches(encoder_ckpt, corpus):
    """Scripted KL: the combined update breaches (0.5) while the no-aux
    retry is clean (0.001) -> attributable to aux -> counter advances
    and the auxiliary stops permanently at stop_after."""
    model, payloads = _build_model(encoder_ckpt, corpus, kl_guard=0.04,
                                   stop_after=2)
    seq = iter([0.5, 0.001] * 10)   # with-aux, retry, with-aux, retry...
    model._action_kl = lambda *a, **kw: next(seq)
    model.learn(total_timesteps=96)
    assert payloads[-1]["aux/stopped"] == 1
    assert payloads[-1]["aux/updates_rejected_total"] >= 2
    # post-stop updates run without aux
    assert payloads[-1]["aux/active"] == 0


def test_mixed_batch_honors_rehearsal_fraction(corpus):
    episodes, stats = corpus
    rehearsal = dd.WindowSampler(episodes, stats, HISTORY, HORIZONS,
                                 val=False, seed=0)
    buffer = OnlineWindowBuffer(stats, HISTORY, HORIZONS,
                                max_frames=10_000, seed=0)
    rng = np.random.default_rng(1)
    n_frames = HISTORY + HORIZONS[-1] + 8
    for _ in range(30):
        assert buffer.add_episode({
            "frames": rng.standard_normal((n_frames, fr.FRAME_DIM)),
            "actions": rng.standard_normal((n_frames - 1, fr.ACTION_DIM)),
            "priv": rng.standard_normal((n_frames, fr.PRIV_DIM)),
            "mode": "walk", "reason": "trunc", "dr": 0.3,
        })
    # too-short episodes are skipped, not crashed on
    assert not buffer.add_episode({
        "frames": rng.standard_normal((5, fr.FRAME_DIM)),
        "actions": rng.standard_normal((4, fr.ACTION_DIM)),
        "priv": rng.standard_normal((5, fr.PRIV_DIM)),
    })
    assert buffer.num_windows() > 64
    batch, frac = mixed_batch(buffer, rehearsal, 32, 0.25,
                              min_online_windows=64)
    assert frac == pytest.approx(0.25)
    assert batch["hist"].shape == (32, HISTORY, fr.FRAME_DIM)
    assert batch["state"][1].shape == (32, fr.STATE_DIM)
    # below the online floor -> honest rehearsal-only batch
    empty = OnlineWindowBuffer(stats, HISTORY, HORIZONS, seed=0)
    _, frac0 = mixed_batch(empty, rehearsal, 32, 0.25,
                           min_online_windows=64)
    assert frac0 == 1.0


def test_concat_batches_preserves_nested_keys(corpus):
    episodes, stats = corpus
    s = dd.WindowSampler(episodes, stats, HISTORY, HORIZONS, val=False,
                         seed=0)
    a, b = s.batch(4), s.batch(6)
    merged = concat_batches(a, b)
    assert merged["hist"].shape[0] == 10
    for key in ("state", "contact", "current", "priv", "fut_hist"):
        for k in HORIZONS:
            assert merged[key][k].shape[0] == 10
    assert merged["contact_now"].shape == (10, fr.N_FEET)


def test_save_excludes_aux_runtime_and_roundtrips(encoder_ckpt, corpus,
                                                  tmp_path):
    """Checkpoint hygiene (tfwalk-joint1 crash diagnosis, 08-17).

    Every C checkpoint pickled the rehearsal sampler + online window
    buffer into SB3's `data` blob (12.5GB per zip); the save-time RSS
    spike crossed the pods' memwatch 85GiB kill threshold and all three
    C arms were SIGKILLed MID-SAVE (0-byte husks). Also: checkpoints
    written after set_group_lrs() carry a two-group Adam state that a
    freshly constructed model refused to load ("different number of
    parameter groups"). Prove both are fixed: the saved `data` blob is
    small and aux-free, and a plain JointAuxPPO.load() round-trips to
    a working, identically-acting policy with the two-group optimizer.
    """
    import json
    import zipfile

    model, _ = _build_model(encoder_ckpt, corpus, warmup=0)
    model.learn(total_timesteps=64)
    path = tmp_path / "joint_aux_ckpt.zip"
    model.save(str(path))

    with zipfile.ZipFile(path) as z:
        data_size = z.getinfo("data").file_size
        saved_keys = set(json.loads(z.read("data")).keys())
    # pre-fix the blob was ~12.5GB on the real corpus; even this tiny
    # test corpus pushed it into the MBs. Post-fix: config-only.
    assert data_size < 512 * 1024, f"data blob {data_size}B — aux leaked"
    leaked = saved_keys & set(JointAuxPPO._AUX_RUNTIME_ATTRS)
    assert not leaked, f"aux runtime pickled into checkpoint: {leaked}"

    loaded = JointAuxPPO.load(str(path), device="cpu")  # died pre-fix
    groups = loaded.policy.optimizer.param_groups
    assert len(groups) == 2
    assert sorted(g.get("lr_scale", 1.0) for g in groups) == [0.1, 1.0]
    obs = np.zeros((1, OBS_DIM), dtype=np.float32)
    act_orig, _ = model.predict(obs, deterministic=True)
    act_loaded, _ = loaded.predict(obs, deterministic=True)
    assert np.allclose(act_orig, act_loaded, atol=1e-6)
    # un-configured loaded model must behave like plain ScaledLRPPO
    assert loaded._aux is None
