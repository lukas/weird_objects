"""Preflight bank for the LIVE predictive-critic (condition F, operator
order fb_20260817T210422_9df9c7, run cw-dynrep-livewalkrise1). Proves,
on tiny real components (real DynamicsModel / WindowSampler / SB3 PPO):

  1. LiveWindowStore window/target extraction is EXACTLY
     data.WindowSampler's (parity on a shared episode);
  2. command/mode bin tags are correct on constructed episodes
     (direction, speed, yaw, command-change, start-stop, fall,
     rise start kind incl. the post-lower bank);
  3. stratified predictor batches realize the 75% fresh (75/25
     walk/rise) + 25% rehearsal quotas, with the honest
     rehearsal-only fallback while the store warms up;
  4. rise coverage: rise windows exist, and the val_every split
     routes command-rich heldout windows for BOTH modes;
  5. live mode keeps the actor completely independent (zero
     action-KL proof metric, same obligation as E);
  6. the critic snapshot changes ONLY at snapshot-boundary
     crossings, and only when EVERY gate passes;
  7. the rejection path leaves the snapshot bit-identical to the
     pretrained checkpoint and logs every gate value;
  8. checkpoints stay small (no live runtime pickled) and
     round-trip bit-identically;
  9. the W&B payload schema carries the required key families
     (pred/train, pred/batch fractions, pred/gate, data/ composition,
     pred/bin per-bin errors);
 10. [CUDA pods only] store tensors, predictor batches and the online
     optimizer state all live on CUDA — the "true live CUDA
     collection+training" proof, run on a train pod at preflight.
"""
import json
import zipfile

import numpy as np
import pytest
import torch as th
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3.common.vec_env import DummyVecEnv

from rl_move.dynamics import data as dd
from rl_move.dynamics import frames as fr
from rl_move.dynamics.live_replay import (
    BIN_DIMS, CMD_DIR_BINS, FALL_BINS, META_KEYS, MODE_BINS,
    PROGRESS_BINS, RISE_START_BINS, SPEED_BINS, STARTSTOP_BINS,
    TRANS_BINS, YAW_BINS, LiveWindowStore, stratified_live_batch,
    prediction_accuracy_metrics, window_meta,
)
from rl_move.dynamics.model import DynamicsModel
from rl_move.dynamics.predictive_critic import (
    PredictiveCriticPolicy, PredictiveCriticPPO, PredictorConfig,
)
from rl_move.dynamics.train_ppo_transfer import anchor_batch_to_torch

HORIZONS = (1, 2, 5, 10, 25)
HISTORY = 16
FRAME_WIDTH = 72
OBS_DIM = HISTORY * FRAME_WIDTH
N_FRAMES = HISTORY + HORIZONS[-1] + 8


def _tiny_model() -> DynamicsModel:
    return DynamicsModel(
        input_set="obs", z_dim=8, hidden=16, act_hidden=8,
        history=HISTORY, arch="transformer", tf_layers=1, tf_heads=2,
        tf_ff=16, tf_dropout=0.0, horizons=HORIZONS, short_max=5,
        delta_state=True, predict_priv=True)


def _ep_dict(seed: int, mode: str = "walk", *, cmd=(0.1, 0.0, 0.0),
             cmd2=None, reason: str = "trunc", start_at: str = "plant",
             n_frames: int = N_FRAMES) -> dict:
    """Synthetic captured episode with a controllable command track.
    ``cmd2`` switches the command at the episode midpoint."""
    rng = np.random.default_rng(seed)
    frames = rng.standard_normal((n_frames, fr.FRAME_DIM)).astype(
        np.float32)
    frames[:, fr.CONTACT_SLICE] = 10.0        # all six feet planted
    priv = rng.standard_normal((n_frames, fr.PRIV_DIM)).astype(
        np.float32) * 0.01
    priv[:, 7:10] = np.asarray(cmd, dtype=np.float32)
    if cmd2 is not None:
        priv[n_frames // 2:, 7:10] = np.asarray(cmd2, dtype=np.float32)
    # achieved along-command velocity: perfect tracking
    priv[:, 10] = np.hypot(priv[:, 7], priv[:, 8])
    return {
        "frames": frames,
        "actions": rng.standard_normal(
            (n_frames - 1, fr.ACTION_DIM)).astype(np.float32),
        "priv": priv, "mode": mode, "reason": reason,
        "start_at": start_at, "q_nom": np.zeros(18, dtype=np.float32),
        "dr": 0.3,
    }


@pytest.fixture(scope="module")
def stats():
    eps = [dd.Episode(
        frames=np.random.default_rng(i).standard_normal(
            (N_FRAMES, fr.FRAME_DIM)).astype(np.float32),
        actions=np.random.default_rng(i).standard_normal(
            (N_FRAMES - 1, fr.ACTION_DIM)).astype(np.float32),
        priv=np.random.default_rng(i).standard_normal(
            (N_FRAMES, fr.PRIV_DIM)).astype(np.float32),
        priv_mask=np.ones(fr.PRIV_DIM, dtype=np.float32),
        actor="random", mode="walk", reason="trunc", dr=0.0,
        global_idx=i) for i in range(40)]
    return eps, dd.compute_stats(eps)


def _store(stats_fx, device="cpu", **kw) -> LiveWindowStore:
    _, st = stats_fx
    kw.setdefault("windows_per_episode", 10_000)
    kw.setdefault("val_every", 4)
    return LiveWindowStore(st, HISTORY, HORIZONS, th.device(device),
                           seed=0, **kw)


def _meta_of(store, ep, t):
    return window_meta(
        fr.upgrade_priv(np.asarray(ep["priv"], dtype=np.float32)),
        np.asarray(ep["frames"], dtype=np.float32), t, HISTORY,
        HORIZONS[-1], ep["mode"], ep["reason"], ep["start_at"])


def _bin(meta, dim):
    return BIN_DIMS[dim][int(meta[META_KEYS.index(dim)])]


def test_window_extraction_parity_with_windowsampler(stats):
    """A store-held window must be bit-identical to the real
    WindowSampler's extraction of the same (episode, t)."""
    _, st = stats
    store = _store(stats, val_every=10**9, mask_cmd_priv=False)
    ep = _ep_dict(0, "walk")
    assert store.add_episode(ep)
    episode = dd.Episode(
        frames=np.asarray(ep["frames"], dtype=np.float32),
        actions=np.asarray(ep["actions"], dtype=np.float32),
        priv=fr.upgrade_priv(np.asarray(ep["priv"], dtype=np.float32)),
        priv_mask=np.ones(fr.PRIV_DIM, dtype=np.float32),
        actor="x", mode="walk", reason="trunc", dr=0.3, global_idx=0)
    episode.split = "train"
    sampler = dd.WindowSampler([episode], st, HISTORY, HORIZONS,
                               split="train")
    n = store.num_windows("walk")
    assert n == len(sampler)
    got = store.groups[("walk", "train")].gather(np.arange(n))
    ref = sampler.batch(0, idx=np.arange(n))
    ref_t = anchor_batch_to_torch(ref, device="cpu")
    for k in ("hist", "fut_actions", "contact_now", "current_now",
              "priv_now", "priv_mask_now"):
        assert th.equal(got[k], ref_t[k]), f"{k} differs"
    for k in ("state", "contact", "current", "priv", "fut_hist"):
        for kk in ref_t[k]:
            assert th.equal(got[k][kk], ref_t[k][kk]), f"{k}[{kk}]"


def test_physical_accuracy_metrics_are_interpretable(stats):
    store = _store(stats, val_every=10**9)
    store.add_episode(_ep_dict(99))
    bt, _ = store.sample(8, 0)
    pred = {
        "priv_now": bt["priv_now"].clone(),
        "contact_now_logits": th.where(
            bt["contact_now"] > 0.5, 20.0, -20.0),
        "current_now": bt["current_now"].clone(),
        "state": {k: v.clone() for k, v in bt["state"].items()},
        "priv": {k: v.clone() for k, v in bt["priv"].items()},
        "contact_logits": {
            k: th.where(v > 0.5, 20.0, -20.0)
            for k, v in bt["contact"].items()},
        "current": {k: v.clone() for k, v in bt["current"].items()},
    }
    metrics = prediction_accuracy_metrics(pred, bt, stats[1], "physical/")
    assert metrics["physical/now/velocity_rmse_m_s"] == 0.0
    assert metrics["physical/now/heading_mae_deg"] == 0.0
    assert metrics["physical/now/contact_accuracy"] == 1.0
    assert metrics["physical/future/tilt_rmse_deg"] == 0.0
    assert metrics["physical/future/joint_pos_rmse_deg"] == 0.0
    assert metrics["physical/future/contact_accuracy"] == 1.0


def test_bin_meta_correctness(stats):
    store = _store(stats)
    t = HISTORY - 1
    m = _meta_of(store, _ep_dict(1, "walk", cmd=(0.10, 0.0, 0.0)), t)
    assert _bin(m, "mode") == "walk" and _bin(m, "cmd_dir") == "fwd"
    assert _bin(m, "speed") == "fast" and _bin(m, "yaw") == "yaw0"
    assert _bin(m, "trans") == "steady"
    assert _bin(m, "startstop") == "cruise"
    assert _bin(m, "contact") == "feet_6"
    assert _bin(m, "progress") == "prog_hi"
    assert _bin(m, "fall") == "clean" and _bin(m, "rise_start") == "na"
    m = _meta_of(store, _ep_dict(2, "walk", cmd=(-0.05, 0.0, 0.0)), t)
    assert _bin(m, "cmd_dir") == "back" and _bin(m, "speed") == "slow"
    m = _meta_of(store, _ep_dict(3, "walk", cmd=(0.0, 0.08, 0.0)), t)
    assert _bin(m, "cmd_dir") == "lat" and _bin(m, "speed") == "med"
    m = _meta_of(store, _ep_dict(4, "walk", cmd=(0.06, 0.06, 0.25)), t)
    assert _bin(m, "cmd_dir") == "diag" and _bin(m, "yaw") == "yaw_left"
    # command change at the episode midpoint: a window straddling it
    ep = _ep_dict(5, "walk", cmd=(0.1, 0.0, 0.0), cmd2=(0.0, 0.1, 0.0))
    m = _meta_of(store, ep, N_FRAMES // 2)
    assert _bin(m, "trans") == "cmd_change"
    assert _bin(m, "startstop") == "cruise"
    # start-stop: zero command then moving
    ep = _ep_dict(6, "walk", cmd=(0.0, 0.0, 0.0), cmd2=(0.1, 0.0, 0.0))
    m = _meta_of(store, ep, N_FRAMES // 2)
    assert _bin(m, "startstop") == "start_stop"
    m = _meta_of(store, _ep_dict(7, "walk", cmd=(0.0, 0.0, 0.0)), t)
    assert _bin(m, "cmd_dir") == "zero" and _bin(m, "speed") == "stop"
    assert _bin(m, "progress") == "prog_na"
    # rise start kinds + fall episodes
    m = _meta_of(store, _ep_dict(8, "rise", cmd=(0, 0, 0),
                                 start_at="rise_bank"), t)
    assert _bin(m, "mode") == "rise"
    assert _bin(m, "rise_start") == "post_lower"
    m = _meta_of(store, _ep_dict(9, "rise", start_at="crouch"), t)
    assert _bin(m, "rise_start") == "crouch"
    m = _meta_of(store, _ep_dict(10, "rise", start_at="zero",
                                 reason="term"), t)
    assert _bin(m, "rise_start") == "flat_bridge"
    assert _bin(m, "fall") == "fall_ep"


def test_live_windows_mask_exogenous_cmd_priv(stats):
    """Live windows must NOT supervise the exogenous command priv
    channels (7:14): future commands are unpredictable by design under
    mid-episode resampling, and the corpus priv_std for wz_ref is a
    clamped 0.001 (one live yaw command = ~300 sigma — the canary1
    cmd_track blowup). Rehearsal windows keep the full corpus mask."""
    eps_all, st = stats
    store = _store(stats, val_every=10**9)      # mask_cmd_priv default ON
    for i in range(4):
        store.add_episode(_ep_dict(900 + i, "walk", cmd=(0.1, 0, 0.3)))
    rehearsal = dd.WindowSampler(eps_all, st, HISTORY, HORIZONS,
                                 val=False, seed=0)
    batch, info = stratified_live_batch(
        store, rehearsal, anchor_batch_to_torch, "cpu", 32,
        rehearsal_frac=0.25, walk_frac=1.0, min_live_windows=16)
    mask = batch["priv_mask_now"]
    n_fresh = int(round(info["pred/batch_fresh_frac"] * 32))
    assert n_fresh > 0
    assert th.all(mask[:n_fresh, 7:14] == 0.0), \
        "live windows still supervise exogenous cmd channels"
    assert th.all(mask[:n_fresh, :7] == 1.0)
    assert th.all(mask[n_fresh:, :] == 1.0), \
        "rehearsal mask was clobbered"
    # the masked loss must ignore the exploding channels entirely
    from rl_move.dynamics.model import dynamics_loss
    model = _tiny_model()
    with th.no_grad():
        out = model(batch["hist"], batch["fut_actions"])
        # blow up the masked channels in the target: loss must not move
        loss_a, _ = dynamics_loss(out, batch, PredictorConfig().lambdas,
                                  model)
        batch2 = dict(batch)
        batch2["priv_now"] = batch["priv_now"].clone()
        batch2["priv_now"][:n_fresh, 7:14] += 1000.0
        loss_b, _ = dynamics_loss(out, batch2, PredictorConfig().lambdas,
                                  model)
    assert th.allclose(loss_a, loss_b), \
        "masked exogenous channels still leak into the loss"


def test_stratified_quotas_and_warmup_fallback(stats):
    eps_all, st = stats
    rehearsal = dd.WindowSampler(eps_all, st, HISTORY, HORIZONS,
                                 val=False, seed=0)
    store = _store(stats, val_every=10**9)
    # warmup: empty store -> 100% rehearsal, honestly reported
    batch, info = stratified_live_batch(
        store, rehearsal, anchor_batch_to_torch, "cpu", 32,
        min_live_windows=512)
    assert batch["hist"].shape[0] == 32
    assert info["pred/batch_rehearsal_frac"] == 1.0
    assert info["pred/batch_fresh_frac"] == 0.0
    for i in range(24):
        store.add_episode(_ep_dict(100 + i, "walk"))
        store.add_episode(_ep_dict(200 + i, "rise", cmd=(0, 0, 0),
                                   start_at="zero"))
    assert store.num_windows("walk") >= 128
    batch, info = stratified_live_batch(
        store, rehearsal, anchor_batch_to_torch, "cpu", 32,
        rehearsal_frac=0.25, walk_frac=0.75, min_live_windows=128)
    assert batch["hist"].shape[0] == 32
    assert info["pred/batch_rehearsal_frac"] == pytest.approx(8 / 32)
    assert info["pred/batch_fresh_frac"] == pytest.approx(24 / 32)
    assert info["pred/batch_fresh_walk_frac"] == pytest.approx(18 / 24)
    assert info["pred/batch_fresh_rise_frac"] == pytest.approx(6 / 24)


def test_rise_coverage_and_val_split(stats):
    store = _store(stats, val_every=4)
    for i in range(8):
        store.add_episode(_ep_dict(300 + i, "walk"))
        store.add_episode(_ep_dict(400 + i, "rise", cmd=(0, 0, 0),
                                   start_at=("rise_bank" if i % 2
                                             else "crouch")))
    assert store.num_windows("rise", "train") > 0
    assert store.num_windows("rise", "val") > 0      # every 4th episode
    assert store.num_windows("walk", "val") > 0
    rep = store.composition_report()
    assert rep["data/windows_rise_train"] > 0
    assert rep["data/added/rise_start/post_lower"] > 0
    assert rep["data/added_frac/mode/rise"] > 0.3
    vb = list(store.val_batches("rise", batch_size=64))
    assert vb and vb[0]["hist"].shape[1] == HISTORY


# ---- live PPO mechanism -------------------------------------------------

class _TinyEnv(gym.Env):
    observation_space = spaces.Box(-10.0, 10.0, (OBS_DIM,),
                                   dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, (fr.ACTION_DIM,),
                              dtype=np.float32)

    def __init__(self):
        self._rng = np.random.default_rng(0)
        self._t = 0

    def reset(self, *, seed=None, options=None):
        self._t = 0
        return self._rng.standard_normal(OBS_DIM).astype(np.float32), {}

    def step(self, action):
        self._t += 1
        return (self._rng.standard_normal(OBS_DIM).astype(np.float32),
                float(action[0]) * 0.01, False, self._t >= 20, {})


@pytest.fixture(scope="module")
def encoder_ckpt(tmp_path_factory, stats):
    _, st = stats
    model = _tiny_model()
    path = tmp_path_factory.mktemp("enc") / "tiny_dyn_obs.pt"
    th.save({"layout_version": fr.LAYOUT_VERSION,
             "config": model.config(), "model": model.state_dict(),
             "stats": st.to_dict(), "history": HISTORY}, path)
    return path


def _build_live(encoder_ckpt, stats, *, boundary=64, gate_fns=None,
                drift_guard=1e9, value_jump=1e9, action_kl=1e9,
                actor_residual=False, seed=0):
    eps_all, st = stats
    store = _store(stats, val_every=4)
    for i in range(12):
        store.add_episode(_ep_dict(500 + i, "walk"))
        store.add_episode(_ep_dict(600 + i, "rise", cmd=(0, 0, 0),
                                   start_at="zero"))
    rehearsal = dd.WindowSampler(eps_all, st, HISTORY, HORIZONS,
                                 val=False, seed=0)
    venv = DummyVecEnv([_TinyEnv])
    model = PredictiveCriticPPO(
        PredictiveCriticPolicy, venv,
        policy_kwargs=dict(
            net_arch=[16, 16], log_std_init=-1.0,
            predictor_ckpt=str(encoder_ckpt),
            frame_width=FRAME_WIDTH, history=HISTORY,
            actor_residual_enabled=actor_residual),
        n_steps=32, batch_size=32, n_epochs=2, learning_rate=3e-4,
        seed=seed, verbose=0, device="cpu")
    payloads = []
    cfg = PredictorConfig(
        mode="live", batch_size=16, rehearsal_frac=0.25,
        steps_per_iter=1, lr=1e-3, drift_guard=drift_guard,
        probe_obs=32, snapshot_boundary_steps=boundary,
        gate_heldout_band=0.15, gate_live_improve=0.0,
        gate_rise_band=0.05, gate_value_jump_frac=value_jump,
        gate_action_kl=action_kl)

    def live_batch_fn(n):
        return stratified_live_batch(
            store, rehearsal, anchor_batch_to_torch, "cpu", n,
            min_live_windows=64)

    if gate_fns is None:
        gate_fns = {
            "corpus_val": lambda m: 1.0,
            "live_val": lambda m, mode: (
                0.5 if m is model._online_dyn else 1.0),
            "pretrained_ref": lambda: 1.0,
        }
    model.configure_predictor(
        cfg, rehearsal, anchor_batch_to_torch, online_buffer=None,
        metrics_sink=lambda p, s: payloads.append(p),
        live_batch_fn=live_batch_fn, gate_fns=gate_fns)
    return model, payloads, store


def test_live_actor_isolation(encoder_ckpt, stats):
    model, payloads, _ = _build_live(encoder_ckpt, stats,
                                     boundary=10**9)
    model.learn(total_timesteps=96)
    assert payloads
    for p in payloads:
        assert p["pred/actor_kl_from_predictor"] == 0.0
        assert p["pred/updates_total"] > 0
        assert "pred/batch_fresh_frac" in p


def test_boundary_only_snapshot_changes(encoder_ckpt, stats):
    """No attempt before the boundary; exactly one gated attempt per
    boundary crossing; acceptance copies online -> snapshot and bumps
    the version."""
    model, payloads, _ = _build_live(encoder_ckpt, stats, boundary=10**9)
    model.learn(total_timesteps=96)
    assert int(model.policy.snapshot_version.item()) == 0
    assert all("pred/gate/attempted" not in p for p in payloads)

    model2, payloads2, _ = _build_live(encoder_ckpt, stats, boundary=64)
    model2.learn(total_timesteps=96)
    attempts = [p for p in payloads2 if "pred/gate/attempted" in p]
    assert len(attempts) == 1                 # ts 96: one crossing of 64
    assert attempts[0]["pred/gate/accepted"] == 1
    assert int(model2.policy.snapshot_version.item()) == 1
    # the accepted update moved the snapshot off the pretrained weights
    # (the online predictor keeps training after the copy, so equality
    # with it holds only at the accept instant)
    ck = th.load(encoder_ckpt, weights_only=False)
    ref = _tiny_model()
    ref.load_state_dict(ck["model"])
    assert not all(th.equal(a.detach(), b.detach()) for a, b in zip(
        model2.policy.critic_predictor.parameters(),
        ref.parameters())), "accepted snapshot never changed"


def test_rejection_path_keeps_snapshot_pinned(encoder_ckpt, stats):
    gate_fns = {
        "corpus_val": lambda m: 10.0,          # fails generic retention
        "live_val": lambda m, mode: 1.0,       # no improvement either
        "pretrained_ref": lambda: 1.0,
    }
    model, payloads, _ = _build_live(encoder_ckpt, stats, boundary=64,
                                     gate_fns=gate_fns)
    model.learn(total_timesteps=96)
    attempts = [p for p in payloads if "pred/gate/attempted" in p]
    assert len(attempts) == 1
    a = attempts[0]
    assert a["pred/gate/accepted"] == 0
    assert a["pred/gate/generic"] == 0
    assert a["pred/boundary_rejected_total"] == 1
    for k in ("pred/gate/corpus_val_candidate", "pred/gate/latent_drift",
              "pred/gate/value_jump", "pred/gate/live_walk_candidate"):
        assert k in a
    assert int(model.policy.snapshot_version.item()) == 0
    ck = th.load(encoder_ckpt, weights_only=False)
    ref = _tiny_model()
    ref.load_state_dict(ck["model"])
    assert all(th.equal(a_.detach(), b_.detach()) for a_, b_ in zip(
        model.policy.critic_predictor.parameters(), ref.parameters())), \
        "rejected update still mutated the snapshot"


def test_actor_action_kl_gate_rejects_unseen_snapshot(encoder_ckpt, stats):
    """An actor-connected snapshot cannot cross a boundary when it changes
    the live walking action distribution beyond the registered ceiling."""
    model, payloads, _ = _build_live(
        encoder_ckpt, stats, boundary=64, actor_residual=True,
        action_kl=0.0)
    with th.no_grad():
        model.policy.actor_gate.fill_(1.0)
    model.learn(total_timesteps=96)
    attempt = next(p for p in payloads if "pred/gate/attempted" in p)
    assert attempt["pred/gate/action_kl"] > 0.0
    assert attempt["pred/gate/action"] == 0
    assert attempt["pred/gate/accepted"] == 0
    assert int(model.policy.snapshot_version.item()) == 0


def test_checkpoint_roundtrip_no_live_runtime(encoder_ckpt, stats,
                                              tmp_path):
    model, _, _ = _build_live(encoder_ckpt, stats, boundary=64)
    model.learn(total_timesteps=96)
    path = tmp_path / "livecritic.zip"
    model.save(str(path))
    with zipfile.ZipFile(path) as z:
        data_size = z.getinfo("data").file_size
        saved_keys = set(json.loads(z.read("data")).keys())
    assert data_size < 512 * 1024, f"data blob {data_size}B"
    leaked = saved_keys & set(PredictiveCriticPPO._PRED_RUNTIME_ATTRS)
    assert not leaked, f"live runtime pickled: {leaked}"
    loaded = PredictiveCriticPPO.load(str(path), device="cpu")
    obs = np.random.default_rng(4).standard_normal(
        (6, OBS_DIM)).astype(np.float32)
    a0, _ = model.predict(obs, deterministic=True)
    a1, _ = loaded.predict(obs, deterministic=True)
    assert np.array_equal(a0, a1)
    obs_t = th.as_tensor(obs)
    assert th.equal(model.policy.predict_values(obs_t),
                    loaded.policy.predict_values(obs_t))
    assert int(loaded.policy.snapshot_version.item()) == int(
        model.policy.snapshot_version.item())


def test_wandb_schema_families(encoder_ckpt, stats):
    """Every key family the run's dashboards/gates read must exist."""
    model, payloads, store = _build_live(encoder_ckpt, stats,
                                         boundary=64)
    model.learn(total_timesteps=96)
    keys = set().union(*(p.keys() for p in payloads))
    for want in ("pred/train/total", "pred/batch_fresh_frac",
                 "pred/batch_walk_frac", "pred/batch_rise_frac",
                 "pred/actor_kl_from_predictor", "pred/snapshot_version",
                 "pred/gate/accepted", "pred/gate/live_rise_candidate",
                 "critic/gate", "critic/residual_abs_mean"):
        assert want in keys, f"missing payload family {want}"
    rep = store.composition_report()
    assert {"data/windows_walk_train", "data/windows_rise_val",
            "data/added_frac/mode/walk",
            "data/sampled_frac/cmd_dir/fwd"} <= set(rep)
    bins = store.bin_report(model._online_dyn,
                            PredictorConfig().lambdas, max_per_bin=64)
    assert any(k.startswith("pred/bin/walk/") and k.endswith("/now/vel")
               for k in bins), bins.keys()
    assert any(k.startswith("pred/bin/rise/") for k in bins)


@pytest.mark.skipif(not th.cuda.is_available(),
                    reason="CUDA preflight proof runs on a train pod")
def test_cuda_store_batches_and_optimizer(stats):
    """True live CUDA collection+training: store tensors, stratified
    batches and the online predictor's Adam state all on CUDA."""
    eps_all, st = stats
    dev = th.device("cuda")
    store = _store(stats, device="cuda", val_every=4)
    for i in range(8):
        store.add_episode(_ep_dict(700 + i, "walk"))
        store.add_episode(_ep_dict(800 + i, "rise", cmd=(0, 0, 0),
                                   start_at="zero"))
    for g in store.groups.values():
        for c in g.chunks:
            assert c["hist"].device.type == "cuda"
    rehearsal = dd.WindowSampler(eps_all, st, HISTORY, HORIZONS,
                                 val=False, seed=0)
    batch, info = stratified_live_batch(
        store, rehearsal, anchor_batch_to_torch, dev, 32,
        min_live_windows=64)
    assert batch["hist"].device.type == "cuda"
    assert batch["state"][1].device.type == "cuda"
    assert info["pred/batch_fresh_frac"] > 0
    model = _tiny_model().to(dev)
    opt = th.optim.Adam(model.parameters(), lr=1e-3)
    from rl_move.dynamics.model import dynamics_loss
    out = model(batch["hist"], batch["fut_actions"])
    loss, _ = dynamics_loss(out, batch, PredictorConfig().lambdas, model)
    opt.zero_grad()
    loss.backward()
    opt.step()
    for group_state in opt.state.values():
        assert group_state["exp_avg"].device.type == "cuda"
