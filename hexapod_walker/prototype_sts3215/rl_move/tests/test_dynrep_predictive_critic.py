"""Bank for the decoupled predictive-critic conditions D/E (operator
directive fb_20260817T052333_e5ae09). Proves, on tiny real components
(real DynamicsModel, real SB3 PPO loop, real WindowSampler):

  1. actor + raw critic are BIT-IDENTICAL to a plain condition-A
     PPO("MlpPolicy") at the same seed, and actor outputs/log-probs are
     bit-identical with the predictive branch enabled/disabled at the
     zero-init gate (values too: the residual starts as an exact no-op);
  2. PPO value gradients cannot touch either transformer: after
     training, snapshot params are bit-unchanged (frozen mode) while
     the gate/adapter DID learn;
  3. online predictor updates leave the actor untouched — mutated-param
     check + the explicit zero action-KL proof metric — and only the
     online predictor's own params move;
  4. the snapshot's identity is frozen across rollout+GAE+PPO (an
     out-of-band mutation raises) and version bumps happen only via the
     guarded between-iteration EMA update;
  5. the drift guard skips (and logs) oversized snapshot updates;
  6. checkpoints stay small (no runtime pickled) and round-trip to
     bit-identical actions AND values.
"""
import json
import zipfile

import numpy as np
import pytest
import torch as th
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from rl_move.dynamics import data as dd
from rl_move.dynamics import frames as fr
from rl_move.dynamics.model import DynamicsModel
from rl_move.dynamics.online_windows import OnlineWindowBuffer
from rl_move.dynamics.predictive_critic import (
    ObsToDynFrames, PredictiveCriticPolicy, PredictiveCriticPPO,
    PredictorConfig,
)
from rl_move.dynamics.sb3_encoder import DynFeaturesExtractor
from rl_move.dynamics.train_ppo_transfer import anchor_batch_to_torch
from rl_move.sim.update_health import (
    CRITIC_MARKERS, attach_actor_critic_lr, split_actor_critic_params,
)

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


@pytest.fixture(scope="module")
def encoder_ckpt(tmp_path_factory, corpus):
    _, stats = corpus
    model = _tiny_model()
    path = tmp_path_factory.mktemp("enc") / "tiny_dyn_obs.pt"
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


def _build(encoder_ckpt, corpus, *, mode="online", steps_per_iter=2,
           ema_tau=0.05, drift_guard=1e9, min_online_windows=10**9,
           seed=0, configure=True, actor_residual=False):
    episodes, stats = corpus
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
    if configure:
        cfg = PredictorConfig(
            mode=mode, batch_size=16, rehearsal_frac=0.25,
            steps_per_iter=steps_per_iter, lr=1e-3, ema_tau=ema_tau,
            drift_guard=drift_guard,
            min_online_windows=min_online_windows, probe_obs=32)
        rehearsal = dd.WindowSampler(episodes, stats, HISTORY, HORIZONS,
                                     val=False, seed=0)
        buffer = (OnlineWindowBuffer(stats, HISTORY, HORIZONS,
                                     max_frames=10_000, seed=0)
                  if mode == "online" else None)
        model.configure_predictor(
            cfg, rehearsal, anchor_batch_to_torch, online_buffer=buffer,
            metrics_sink=lambda p, s: payloads.append(p))
    return model, payloads


def _build_with_algo(encoder_ckpt, corpus, algo_cls, *, seed=0,
                     actor_residual=False):
    """Tiny configured build for cooperative PPO wrapper tests."""
    episodes, stats = corpus
    model = algo_cls(
        PredictiveCriticPolicy, DummyVecEnv([_TinyEnv]),
        policy_kwargs=dict(
            net_arch=[16, 16], log_std_init=-1.0,
            predictor_ckpt=str(encoder_ckpt),
            frame_width=FRAME_WIDTH, history=HISTORY,
            actor_residual_enabled=actor_residual),
        n_steps=32, batch_size=32, n_epochs=2, learning_rate=3e-4,
        seed=seed, verbose=0, device="cpu")
    cfg = PredictorConfig(mode="frozen", probe_obs=32)
    rehearsal = dd.WindowSampler(episodes, stats, HISTORY, HORIZONS,
                                 val=False, seed=0)
    model.configure_predictor(cfg, rehearsal, anchor_batch_to_torch)
    return model


def _named_clone(module):
    return {n: p.detach().clone() for n, p in module.named_parameters()}


def _snap_params(model):
    return [p.detach().clone()
            for p in model.policy.critic_predictor.parameters()]


def test_actor_and_raw_critic_bit_match_scratch_A(encoder_ckpt, corpus):
    """At the same seed the D/E policy's actor AND raw critic init
    bit-identical to a plain condition-A PPO('MlpPolicy') — the
    predictive modules are built after SB3's standard RNG draws."""
    model, _ = _build(encoder_ckpt, corpus, configure=False, seed=7)
    a = PPO("MlpPolicy", DummyVecEnv([_TinyEnv]),
            policy_kwargs=dict(net_arch=[16, 16], log_std_init=-1.0),
            n_steps=32, batch_size=32, n_epochs=2, learning_rate=3e-4,
            seed=7, verbose=0, device="cpu")
    ours = _named_clone(model.policy)
    theirs = _named_clone(a.policy)
    for name, p in theirs.items():
        assert name in ours, f"scratch-A param {name} missing"
        assert th.equal(ours[name], p), f"param {name} differs from A"
    obs = np.random.default_rng(1).standard_normal(
        (4, OBS_DIM)).astype(np.float32)
    act_ours, _ = model.predict(obs, deterministic=True)
    act_a, _ = a.predict(obs, deterministic=True)
    assert np.array_equal(act_ours, act_a), \
        "deterministic actions differ from scratch A"


def test_zero_gate_residual_is_bit_exact_noop(encoder_ckpt, corpus):
    model, _ = _build(encoder_ckpt, corpus, configure=False)
    policy = model.policy
    assert float(policy.value_gate.detach()) == 0.0
    obs = th.as_tensor(np.random.default_rng(2).standard_normal(
        (8, OBS_DIM)).astype(np.float32))
    acts = th.zeros((8, fr.ACTION_DIM))
    th.manual_seed(0)
    policy.residual_enabled = True
    v_on, lp_on, ent_on = policy.evaluate_actions(obs, acts)
    val_on = policy.predict_values(obs)
    th.manual_seed(0)
    a_on, fv_on, flp_on = policy(obs, deterministic=True)
    policy.residual_enabled = False
    v_off, lp_off, ent_off = policy.evaluate_actions(obs, acts)
    val_off = policy.predict_values(obs)
    th.manual_seed(0)
    a_off, fv_off, flp_off = policy(obs, deterministic=True)
    assert th.equal(a_on, a_off) and th.equal(flp_on, flp_off), \
        "actor output/log-prob changed with the predictive branch"
    assert th.equal(lp_on, lp_off) and th.equal(ent_on, ent_off)
    # at gate==0 the residual is exactly 0.0 -> values bit-equal too
    assert th.equal(v_on, v_off) and th.equal(val_on, val_off)
    assert th.equal(fv_on, fv_off)
    policy.residual_enabled = True


def test_zero_gate_actor_conditioning_is_bit_exact_noop(encoder_ckpt,
                                                         corpus):
    """The live architecture really has an actor transformer branch, but
    enabling it starts at exactly the same action distribution as raw A."""
    raw, _ = _build(encoder_ckpt, corpus, configure=False, seed=19)
    live, _ = _build(encoder_ckpt, corpus, configure=False, seed=19,
                     actor_residual=True)
    assert float(live.policy.actor_gate.detach()) == 0.0
    obs = np.random.default_rng(20).standard_normal(
        (8, OBS_DIM)).astype(np.float32)
    a_raw, _ = raw.predict(obs, deterministic=True)
    a_live, _ = live.predict(obs, deterministic=True)
    assert np.array_equal(a_raw, a_live)


def test_actor_consumes_stable_snapshot_without_transformer_gradients(
        encoder_ckpt, corpus):
    """Once the gate opens, actions depend on snapshot z; policy gradients
    reach the actor adapter/gate but never the transformer snapshot."""
    model, _ = _build(encoder_ckpt, corpus, configure=False,
                      actor_residual=True)
    policy = model.policy
    obs = th.as_tensor(np.random.default_rng(21).standard_normal(
        (8, OBS_DIM)).astype(np.float32))
    with th.no_grad():
        policy.actor_gate.fill_(0.25)
    dist = policy.get_distribution(obs).distribution
    loss = dist.mean.square().mean()
    policy.optimizer.zero_grad()
    loss.backward()
    assert policy.actor_gate.grad is not None
    assert any(p.grad is not None for p in
               policy.actor_latent_adapter.parameters())
    assert all(p.grad is None for p in policy.critic_predictor.parameters())

    mean_before = dist.mean.detach().clone()
    candidate = _tiny_model()
    candidate.load_state_dict(policy.critic_predictor.state_dict())
    with th.no_grad():
        next(iter(candidate.parameters())).add_(0.5)
        mean_after = policy.distribution_with_predictor(
            obs, candidate).distribution.mean
    assert not th.equal(mean_before, mean_after), \
        "opened actor gate ignored the candidate transformer snapshot"


def test_obs_conversion_parity_with_features_extractor(encoder_ckpt,
                                                       corpus):
    """The critic-side obs->frames->z path must equal the audited
    DynFeaturesExtractor wiring (frozen), so D/E read the exact latent
    B consumed."""
    model, _ = _build(encoder_ckpt, corpus, configure=False)
    policy = model.policy
    fe = DynFeaturesExtractor(
        _TinyEnv.observation_space, ckpt_path=str(encoder_ckpt),
        frame_width=FRAME_WIDTH, history=HISTORY, freeze=True)
    obs = th.as_tensor(np.random.default_rng(3).standard_normal(
        (5, OBS_DIM)).astype(np.float32))
    z_policy = policy.predictive_latent(obs)
    z_fe = fe(obs)[:, : policy.critic_predictor.z_dim]
    assert th.allclose(z_policy, z_fe, atol=1e-6)
    conv = ObsToDynFrames(dd.Stats.from_dict(
        th.load(encoder_ckpt, weights_only=False)["stats"]),
        FRAME_WIDTH, HISTORY)
    assert conv(obs).shape == (5, HISTORY, fr.FRAME_DIM)


def test_obs_conversion_ignores_appended_recovery_pose_tail(encoder_ckpt):
    """Widening 72->90 leaves the pretrained encoder input bit-identical."""
    stats = dd.Stats.from_dict(
        th.load(encoder_ckpt, weights_only=False)["stats"])
    rng = np.random.default_rng(31)
    wide = rng.standard_normal((3, HISTORY, 90)).astype(np.float32)
    narrow = wide[:, :, :FRAME_WIDTH].copy()
    z_wide = ObsToDynFrames(stats, 90, HISTORY)(
        th.as_tensor(wide.reshape(3, -1)))
    z_narrow = ObsToDynFrames(stats, FRAME_WIDTH, HISTORY)(
        th.as_tensor(narrow.reshape(3, -1)))
    assert th.equal(z_wide, z_narrow)


def test_frozen_predictive_policy_composes_with_bc_anchor(
        encoder_ckpt, corpus):
    """Recovery mentor updates cannot mutate either frozen snapshot."""
    from rl_move.sim.bc_anchor import make_bc_anchor_ppo_class

    combined = make_bc_anchor_ppo_class(PredictiveCriticPPO)
    model = _build_with_algo(
        encoder_ckpt, corpus, combined, actor_residual=True)
    model.bc_coef = 0.5
    model.bc_minibatches = 2
    model.bc_batch_size = 16
    model.bc_foot_z_coef = 0.0
    rng = np.random.default_rng(37)
    for _ in range(32):
        model._bc_push(
            rng.standard_normal(OBS_DIM).astype(np.float32),
            rng.uniform(-1.0, 1.0, fr.ACTION_DIM).astype(np.float32),
            mode=6)
    critic_before = _snap_params(model)
    actor_before = [p.detach().clone()
                    for p in model.policy.actor_predictor.parameters()]

    model.learn(total_timesteps=64)

    assert all(th.equal(a, b) for a, b in
               zip(critic_before, _snap_params(model)))
    assert all(th.equal(a, b.detach()) for a, b in
               zip(actor_before, model.policy.actor_predictor.parameters()))
    assert model._bc_n == 32


def test_ppo_value_grads_cannot_touch_transformers(encoder_ckpt, corpus):
    """Frozen mode (condition D): PPO trains actor/critic/adapter/gate;
    both the snapshot and (nonexistent) online predictor stay put and
    the gate/adapter demonstrably learn."""
    model, payloads = _build(encoder_ckpt, corpus, mode="frozen")
    snap_before = _snap_params(model)
    assert all(not p.requires_grad
               for p in model.policy.critic_predictor.parameters())
    opt_params = {id(p) for g in model.policy.optimizer.param_groups
                  for p in g["params"]}
    assert not (opt_params & {
        id(p) for p in model.policy.critic_predictor.parameters()}), \
        "snapshot params leaked into the PPO optimizer"
    model.learn(total_timesteps=96)
    assert all(th.equal(a, b) for a, b in
               zip(snap_before, _snap_params(model))), \
        "PPO update mutated the frozen snapshot"
    assert float(model.policy.value_gate.detach()) != 0.0, \
        "gate never learned (no value gradient reached it)"
    assert payloads and payloads[-1]["pred/snapshot_version"] == 0
    assert "critic/residual_abs_mean" in payloads[-1]
    assert "critic/gate" in payloads[-1]


def test_predictor_updates_leave_actor_untouched(encoder_ckpt, corpus):
    """Online mode: predictor trains with its own optimizer; the proof
    metric pred/actor_kl_from_predictor is exactly 0.0 and only the
    online predictor's params move (snapshot moves only via EMA)."""
    model, payloads = _build(encoder_ckpt, corpus, mode="online",
                             steps_per_iter=2, ema_tau=0.0)
    online_before = [p.detach().clone()
                     for p in model._online_dyn.parameters()]
    model.learn(total_timesteps=96)
    assert payloads
    for p in payloads:
        assert p["pred/actor_kl_from_predictor"] == 0.0
        assert p["pred/updates_total"] > 0
    assert not all(th.equal(a, b) for a, b in zip(
        online_before, model._online_dyn.parameters())), \
        "online predictor never trained"
    # ema_tau=0 -> candidate == snapshot -> accepted but a no-op;
    # snapshot must still equal the pretrained checkpoint
    ck = th.load(encoder_ckpt, weights_only=False)
    ref = _tiny_model()
    ref.load_state_dict(ck["model"])
    assert all(th.equal(a.detach(), b.detach()) for a, b in zip(
        model.policy.critic_predictor.parameters(), ref.parameters()))


def test_snapshot_updates_between_iterations_only(encoder_ckpt, corpus):
    """EMA accepted -> version advances across learn(); out-of-band
    mutation inside a rollout raises."""
    model, payloads = _build(encoder_ckpt, corpus, mode="online",
                             steps_per_iter=1, ema_tau=0.5,
                             drift_guard=1e9)
    model.learn(total_timesteps=96)
    versions = [p["pred/snapshot_version"] for p in payloads
                if "pred/snapshot_version" in p]
    assert versions[-1] >= 2, versions
    assert payloads[-1]["pred/ema_accepted_total"] >= 2
    assert int(model.policy.snapshot_version.item()) == versions[-1]

    model2, _ = _build(encoder_ckpt, corpus, mode="online",
                       steps_per_iter=1)

    class RogueSnapshotCb(BaseCallback):
        def _on_rollout_end(self) -> None:
            with th.no_grad():
                next(iter(self.model.policy.critic_predictor
                          .parameters())).add_(1.0)

        def _on_step(self) -> bool:
            return True

    with pytest.raises(RuntimeError, match="MUTATED inside"):
        model2.learn(total_timesteps=64, callback=RogueSnapshotCb())


def test_drift_guard_skips_oversized_updates(encoder_ckpt, corpus):
    """drift_guard=-1: every candidate is rejected -> version pinned at
    0, snapshot bit-equal to the pretrained checkpoint, rejections
    logged."""
    model, payloads = _build(encoder_ckpt, corpus, mode="online",
                             steps_per_iter=1, ema_tau=0.5,
                             drift_guard=-1.0)
    snap_before = _snap_params(model)
    model.learn(total_timesteps=96)
    assert payloads[-1]["pred/ema_rejected_total"] >= 2
    assert payloads[-1]["pred/ema_accepted_total"] == 0
    assert payloads[-1]["pred/snapshot_version"] == 0
    assert all(th.equal(a, b) for a, b in
               zip(snap_before, _snap_params(model)))
    assert all(np.isfinite(p["pred/snap_drift_step"]) for p in payloads
               if "pred/snap_drift_step" in p)


def test_save_roundtrip_small_and_bit_identical(encoder_ckpt, corpus,
                                                tmp_path):
    """No runtime pickled (tfwalk-joint1 12.5GB lesson); loaded model
    reproduces actions AND values (incl. residual) bit-for-bit and
    keeps the snapshot version."""
    model, _ = _build(encoder_ckpt, corpus, mode="online",
                      steps_per_iter=1, ema_tau=0.5, drift_guard=1e9)
    model.learn(total_timesteps=64)
    path = tmp_path / "predcritic.zip"
    model.save(str(path))
    with zipfile.ZipFile(path) as z:
        data_size = z.getinfo("data").file_size
        saved_keys = set(json.loads(z.read("data")).keys())
    assert data_size < 512 * 1024, f"data blob {data_size}B — runtime leaked"
    leaked = saved_keys & set(PredictiveCriticPPO._PRED_RUNTIME_ATTRS)
    assert not leaked, f"runtime pickled into checkpoint: {leaked}"

    loaded = PredictiveCriticPPO.load(str(path), device="cpu")
    assert loaded._pcfg is None      # un-configured = plain PPO behavior
    obs_np = np.random.default_rng(4).standard_normal(
        (6, OBS_DIM)).astype(np.float32)
    act_orig, _ = model.predict(obs_np, deterministic=True)
    act_load, _ = loaded.predict(obs_np, deterministic=True)
    assert np.array_equal(act_orig, act_load)
    obs_t = th.as_tensor(obs_np)
    assert th.equal(model.policy.predict_values(obs_t),
                    loaded.policy.predict_values(obs_t))
    assert int(loaded.policy.snapshot_version.item()) == int(
        model.policy.snapshot_version.item())


# -- update_health critic-marker extension (walkcurr2, root cause 3:
# train_ppo_transfer had no target-KL/epoch-cap/LR-decay update-health
# guardrails; porting update_health.py to conditions D/E/F requires the
# stock CRITIC_MARKERS to also cover this policy's two critic-only
# modules that share no substring with them, "value_gate" and
# "latent_adapter" — otherwise attach_actor_critic_lr would silently
# throttle them with the (smaller, decaying) ACTOR learning rate.) ----
PREDICTIVE_CRITIC_MARKERS = CRITIC_MARKERS + ("value_gate", "latent_adapter")


def test_stock_critic_markers_miss_predictive_critic_modules(encoder_ckpt,
                                                              corpus):
    """Pins the bug the extension fixes: with the plain SB3 markers,
    value_gate/latent_adapter land in the ACTOR group (wrong)."""
    model, _ = _build(encoder_ckpt, corpus, configure=False)
    policy = model.policy
    _actor, critic = split_actor_critic_params(policy)
    critic_ids = {id(p) for p in critic}
    assert id(policy.value_gate) not in critic_ids
    assert id(policy.latent_adapter[0].weight) not in critic_ids


def test_extended_critic_markers_cover_predictive_critic_modules(
        encoder_ckpt, corpus):
    """With the extended marker tuple, every predictive-critic-only
    module (value_net trunk+head, latent_adapter, value_gate) lands in
    the critic group and nothing else moves — actor still gets exactly
    the action-distribution params."""
    model, _ = _build(encoder_ckpt, corpus, configure=False)
    policy = model.policy
    actor, critic = split_actor_critic_params(
        policy, critic_markers=PREDICTIVE_CRITIC_MARKERS)
    actor_ids, critic_ids = {id(p) for p in actor}, {id(p) for p in critic}
    assert not (actor_ids & critic_ids)
    all_ids = {id(p) for p in policy.parameters() if p.requires_grad}
    assert actor_ids | critic_ids == all_ids
    assert id(policy.value_gate) in critic_ids
    assert id(policy.latent_adapter[0].weight) in critic_ids
    assert id(policy.latent_adapter[2].weight) in critic_ids
    assert id(policy.value_net.weight) in critic_ids
    assert id(policy.mlp_extractor.value_net[0].weight) in critic_ids
    # actor keeps the action-distribution params
    assert id(policy.log_std) in actor_ids
    assert id(policy.action_net.weight) in actor_ids
    assert id(policy.mlp_extractor.policy_net[0].weight) in actor_ids


def test_attach_actor_critic_lr_with_predictive_critic_markers(
        encoder_ckpt, corpus):
    """attach_actor_critic_lr wires cleanly on this policy end-to-end
    with the extended markers: separate optimizer groups, decay lever
    present, no crash on a real PPO.train() step."""
    model, _ = _build(encoder_ckpt, corpus, configure=False)
    attach_actor_critic_lr(model, actor_lr=1e-4, actor_lr_final=1e-5,
                          critic_lr=3e-4,
                          critic_markers=PREDICTIVE_CRITIC_MARKERS)
    n_actor_expected = sum(
        1 for n, p in model.policy.named_parameters()
        if p.requires_grad
        and not any(m in n for m in PREDICTIVE_CRITIC_MARKERS))
    assert model._ac_state["n_actor"] == n_actor_expected
    model.learn(total_timesteps=32)  # must not raise


def test_attach_actor_critic_lr_save_load_roundtrip_frozen_encoder(
        encoder_ckpt, corpus, tmp_path):
    """Regression (cw-dynrep-criticD-walkcurr3, 08-18): the pre-staged
    harness eval crashed on EVERY finished walkcurr checkpoint with
    "loaded state dict contains a parameter group that doesn't match
    the size of optimizer's group". Root cause: save_stock_optimizer
    (update_health.attach_actor_critic_lr) built the save-time stock
    optimizer over ALL model.policy.parameters() — including the
    frozen dynamics-transformer encoder (requires_grad=False,
    excluded from both the actor and critic optimizer groups) — 172
    tensors on walkcurr3, instead of the 18 trainable ones a freshly
    constructed policy's default optimizer actually contains. Every
    condition-D/criticD run with --actor-lr set (all of walkcurr1/2/3
    and the walkcurr4 canary tournament) saved an uneval-able
    checkpoint. test_attachments_survive_save_load_roundtrip (plain
    MlpPolicy, no frozen params) cannot catch this — it needs a real
    frozen submodule, which this fixture has."""
    model, _ = _build(encoder_ckpt, corpus, configure=False)
    attach_actor_critic_lr(model, actor_lr=1e-4, actor_lr_final=1e-5,
                          critic_lr=3e-4,
                          critic_markers=PREDICTIVE_CRITIC_MARKERS)
    model.learn(total_timesteps=32)
    n_trainable = sum(1 for p in model.policy.parameters()
                      if p.requires_grad)
    n_total = sum(1 for _ in model.policy.parameters())
    assert n_total > n_trainable, \
        "fixture must have a frozen submodule to exercise the bug"
    path = tmp_path / "roundtrip_frozen.zip"
    model.save(str(path))
    loaded = PredictiveCriticPPO.load(str(path), device="cpu")  # must not raise
    assert len(loaded.policy.optimizer.param_groups) == 1
    assert (len(loaded.policy.optimizer.param_groups[0]["params"])
            == n_trainable)
    obs_np = np.random.default_rng(1).standard_normal(
        (3, OBS_DIM)).astype(np.float32)
    loaded.predict(obs_np, deterministic=True)  # inference works


def test_load_checkpoint_auto_repairs_legacy_broken_optimizer(
        encoder_ckpt, corpus, tmp_path):
    """load_checkpoint_auto must still open a checkpoint saved by the
    OLD (pre-fix) save_stock_optimizer bug — e.g.
    ppo_goal_cw_dynrep_criticD_walkcurr3.zip, already on disk before
    this fix landed. Simulates the legacy bug (inflate the saved
    optimizer's single param group to cover every policy.parameters()
    tensor, not just the trainable ones) and checks
    load_checkpoint_auto's repair fallback opens it anyway, into a
    sibling *.optfix.zip that leaves the original untouched."""
    import zipfile
    import io
    import torch as th
    from rl_move.sim.gru_policy import (
        load_checkpoint_auto, repair_legacy_actor_critic_optimizer,
    )

    model, _ = _build(encoder_ckpt, corpus, configure=False)
    attach_actor_critic_lr(model, actor_lr=1e-4, actor_lr_final=1e-5,
                          critic_lr=3e-4,
                          critic_markers=PREDICTIVE_CRITIC_MARKERS)
    model.learn(total_timesteps=32)
    path = tmp_path / "legacy_broken.zip"
    model.save(str(path))
    n_total = sum(1 for _ in model.policy.parameters())

    # Corrupt it back to the pre-fix shape: inflate param_groups[0]
    # to n_total entries (what the old bug produced).
    with zipfile.ZipFile(path) as z:
        opt_sd = th.load(io.BytesIO(z.read("policy.optimizer.pth")),
                         map_location="cpu")
    opt_sd["param_groups"][0]["params"] = list(range(n_total))
    buf = io.BytesIO()
    with zipfile.ZipFile(path) as zin, \
            zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zout:
        for name in zin.namelist():
            if name == "policy.optimizer.pth":
                inner = io.BytesIO()
                th.save(opt_sd, inner)
                zout.writestr(name, inner.getvalue())
            else:
                zout.writestr(name, zin.read(name))
    path.write_bytes(buf.getvalue())
    orig_bytes = path.read_bytes()

    with pytest.raises(ValueError, match="parameter group"):
        PredictiveCriticPPO.load(str(path), device="cpu")

    loaded = load_checkpoint_auto(str(path), device="cpu")
    assert path.read_bytes() == orig_bytes, "original must be untouched"
    fixed = path.with_suffix(".optfix.zip")
    assert fixed.is_file()
    obs_np = np.random.default_rng(2).standard_normal(
        (3, OBS_DIM)).astype(np.float32)
    loaded.predict(obs_np, deterministic=True)

    # repair_legacy_actor_critic_optimizer itself: no-op on an
    # already-correct checkpoint (the fixed-code save path).
    good_path = tmp_path / "already_fine.zip"
    model.save(str(good_path))
    assert repair_legacy_actor_critic_optimizer(good_path) is False
