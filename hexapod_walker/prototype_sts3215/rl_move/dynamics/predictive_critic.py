"""predictive_critic.py — decoupled online world model feeding the
CRITIC only (operator directive fb_20260817T052333_e5ae09, filed after
the tfwalk-joint1 verdict: three cohorts agree the pretrained dynamics
transformer does not transfer to walking through the ACTOR under any
tried mechanism; Lukas: "ok try it" on the critic-only design).

Architecture contract (every clause below is asserted mechanically,
either here or in rl_move/tests/test_dynrep_predictive_critic.py):

  * The ACTOR is the proven scratch-A raw-observation policy: same
    inputs, same net_arch, and — because the predictive modules are
    constructed AFTER SB3's standard build consumes the RNG — the
    actor's initial weights are BIT-IDENTICAL to a plain condition-A
    ``PPO("MlpPolicy", ...)`` at the same seed. The actor NEVER
    consumes transformer latents and shares no parameters or optimizer
    state with any transformer.
  * The CRITIC is the raw-observation value branch (identical to A's)
    plus a stop-gradient predictive-latent residual:
        value(obs) = value_net(latent_vf(obs))
                     + gate * adapter(stopgrad(z_snapshot(obs)))
    with ``gate`` a learned scalar initialized EXACTLY to zero, so the
    model's behavior (actions, log-probs, values) starts bit-equivalent
    to scratch A. PPO value gradients reach ONLY the raw critic branch,
    the adapter, the gate, and the value head — never a transformer.
  * TWO transformer instances:
      - the ONLINE predictor (condition E) trains continuously with its
        OWN optimizer on fresh rollout windows + rehearsal from the
        pretraining corpus; its gradients update only itself;
      - the CRITIC SNAPSHOT (inference-only, requires_grad False) is
        what the residual branch reads. It is updated ATOMICALLY only
        BETWEEN complete rollout+PPO iterations via a guarded EMA:
        a candidate whose probe-latent drift exceeds the guard is
        logged and SKIPPED. Its identity is frozen (params bit-checked,
        version asserted) for the entire rollout, GAE computation and
        all PPO epochs, preserving actor log-prob semantics and
        value-clipping consistency. Condition D never updates it.
  * Checkpoint hygiene: all runtime (samplers, buffers, online
    predictor, its optimizer, probes) is excluded from SB3's pickled
    ``data`` blob (the tfwalk-joint1 12.5GB-zip lesson); the snapshot
    transformer lives INSIDE the policy so saves stay ~60MB and a
    round-tripped checkpoint reproduces values bit-for-bit. The online
    predictor's weights are saved separately by the trainer
    (``models/dyn_online_<name>.pt``), not in the SB3 zip.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np
import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy

from . import frames as fr
from .joint_aux import default_lambdas, priv_group_metrics
from .model import dynamics_loss
from .online_windows import mixed_batch
from .sb3_encoder import PROPRIO_DIM, load_dyn_checkpoint


class ObsToDynFrames(nn.Module):
    """Stacked policy obs (B, H*W, newest first) -> normalized 86-dim
    chronological frame windows (B, H, FRAME_DIM) for a pretrained
    dynamics encoder. Same contract as DynFeaturesExtractor.forward's
    preprocessing (parity is test-locked), factored out so the critic
    residual can feed a snapshot encoder that is NOT a features
    extractor (the actor must keep the raw obs)."""

    def __init__(self, stats, frame_width: int, history: int,
                 q_scale: float = 1.0, qd_scale: float = 2.0,
                 tilt_scale: float = 0.2, gyro_scale: float = 1.0):
        super().__init__()
        self.frame_width, self.history = int(frame_width), int(history)
        unscale = np.concatenate([
            np.full(18, q_scale), np.full(18, qd_scale),
            np.full(2, tilt_scale), np.full(3, gyro_scale),
            np.ones(18)]).astype(np.float32)
        self.register_buffer("unscale", th.as_tensor(unscale))
        cols = np.concatenate([np.arange(0, 41), np.arange(68, 86)])
        self.register_buffer("proprio_cols",
                             th.as_tensor(cols, dtype=th.long))
        self.register_buffer("f_mean", th.as_tensor(stats.mean))
        self.register_buffer("f_std", th.as_tensor(stats.std))

    def forward(self, obs: th.Tensor) -> th.Tensor:
        B = obs.shape[0]
        x = obs.view(B, self.history, self.frame_width)
        x = th.flip(x, dims=(1,))               # newest-first -> chrono
        proprio = x[..., :PROPRIO_DIM] * self.unscale
        frames86 = obs.new_zeros(B, self.history, fr.FRAME_DIM)
        frames86[..., self.proprio_cols] = proprio
        return (frames86 - self.f_mean) / self.f_std


class PredictiveCriticPolicy(ActorCriticPolicy):
    """Scratch-A actor + raw critic with a zero-gated stop-gradient
    predictive-latent residual (module docstring has the contract)."""

    def __init__(self, observation_space, action_space, lr_schedule,
                 *args, predictor_ckpt: str, frame_width: int,
                 history: int, adapter_hidden: int = 64,
                 residual_enabled: bool = True, **kwargs):
        # Plain-python attrs must exist before super().__init__ calls
        # our _build override.
        self._predictor_ckpt = str(predictor_ckpt)
        self._frame_width = int(frame_width)
        self._history = int(history)
        self._adapter_hidden = int(adapter_hidden)
        self.residual_enabled = bool(residual_enabled)
        super().__init__(observation_space, action_space, lr_schedule,
                         *args, **kwargs)

    def _build(self, lr_schedule) -> None:
        # Standard SB3 build first: consumes the SAME RNG draws as a
        # plain MlpPolicy, so actor + raw critic init bit-match
        # condition A at the same seed. Everything predictive is built
        # after and never touched by ortho_init.
        super()._build(lr_schedule)
        model, stats, ckpt_h = load_dyn_checkpoint(self._predictor_ckpt)
        if self._history != ckpt_h:
            raise ValueError(f"env history_frames={self._history} but "
                             f"predictor was pretrained with H={ckpt_h}")
        n_obs = int(np.prod(self.observation_space.shape))
        if n_obs != self._history * self._frame_width:
            raise ValueError(f"obs dim {n_obs} != history {self._history}"
                             f" x frame_width {self._frame_width}")
        self.obs_to_frames = ObsToDynFrames(stats, self._frame_width,
                                            self._history)
        self.critic_predictor = model          # the SNAPSHOT (frozen)
        for p in self.critic_predictor.parameters():
            p.requires_grad_(False)
        self.critic_predictor.eval()
        self.latent_adapter = nn.Sequential(
            nn.Linear(model.z_dim, self._adapter_hidden), nn.SiLU(),
            nn.Linear(self._adapter_hidden, 1))
        self.value_gate = nn.Parameter(th.zeros(()))
        self.register_buffer("snapshot_version",
                             th.zeros((), dtype=th.long))
        # Rebuild the optimizer over TRAINABLE params only: actor,
        # raw critic, adapter, gate — the frozen snapshot is excluded,
        # so no PPO optimizer state can ever touch a transformer.
        self.optimizer = self.optimizer_class(
            [p for p in self.parameters() if p.requires_grad],
            lr=lr_schedule(1), **self.optimizer_kwargs)

    def train(self, mode: bool = True):
        out = super().train(mode)
        if hasattr(self, "critic_predictor"):
            self.critic_predictor.eval()   # snapshot is inference-only
        return out

    # -- predictive residual -------------------------------------------

    def predictive_latent(self, obs: th.Tensor) -> th.Tensor:
        """Stop-gradient snapshot latent for the critic branch."""
        with th.no_grad():
            return self.critic_predictor.encode(
                self.obs_to_frames(obs)).detach()

    def value_residual(self, obs: th.Tensor) -> th.Tensor:
        if not self.residual_enabled:
            return th.zeros(obs.shape[0], 1, device=obs.device,
                            dtype=obs.dtype)
        return self.value_gate * self.latent_adapter(
            self.predictive_latent(obs))

    # -- SB3 API: values get the residual, actor path untouched --------

    def forward(self, obs: th.Tensor, deterministic: bool = False):
        actions, values, log_prob = super().forward(obs, deterministic)
        return actions, values + self.value_residual(obs), log_prob

    def evaluate_actions(self, obs: th.Tensor, actions: th.Tensor):
        values, log_prob, entropy = super().evaluate_actions(obs, actions)
        return values + self.value_residual(obs), log_prob, entropy

    def predict_values(self, obs: th.Tensor) -> th.Tensor:
        return super().predict_values(obs) + self.value_residual(obs)


def actor_only_transplant(old_model, new_model,
                          critic_markers: tuple[str, ...] | None = None,
                          ) -> list[str]:
    """Copy ONLY the actor-side weights of ``old_model`` (a plain
    ``ActorCriticPolicy`` checkpoint — e.g. a scripted-gait BC-INIT or
    a gait-hardened RL checkpoint trained on the plain single-frame
    obs) into ``new_model`` (a freshly-constructed condition-D
    ``PredictiveCriticPolicy`` over the SAME per-frame observation,
    history-stacked newest-first, e.g. ``obs.history_frames=16``).

    Every critic-marked tensor (default: SB3's ``value_net``/
    ``vf_features_extractor`` plus condition-D's ``value_gate``/
    ``latent_adapter``) and every frozen buffer that has no counterpart
    in ``old_model`` (``critic_predictor.*``, ``obs_to_frames.*``,
    ``snapshot_version``) is left exactly as the fresh construction
    built it — this function never even reads their old-model
    counterparts, so the frozen predictor snapshot and the critic
    residual start from the SAME zero-gated fresh state as a
    from-scratch condition-D run.

    The actor's first-layer weight matrix zero-pads the extra columns
    for the additional (OLDER-history) dims appended at the observation
    tail, so the transplanted actor reproduces ``old_model``'s action
    distribution bit-for-bit at init for ANY value of those dims —
    same obs-widening contract as ``train_ppo_sim.pad_obs_transplant``,
    scoped to the actor half only. Optimizer state is fresh (not
    copied).

    Operator addendum fb_20260818T085834_588d9a (walkcurr4 tournament
    arms B/C): actor-only initialization from a proven scripted-gait
    or gait-hardened checkpoint with the frozen condition-D critic
    left completely untouched. Returns the list of copied tensor
    names (for logging/tests).
    """
    from rl_move.sim.update_health import CRITIC_MARKERS as _BASE
    markers = (critic_markers if critic_markers is not None
               else _BASE + ("value_gate", "latent_adapter"))

    def _is_critic(name: str) -> bool:
        return any(m in name for m in markers)

    sd_old = old_model.policy.state_dict()
    sd_new = new_model.policy.state_dict()
    copied: list[str] = []
    with th.no_grad():
        for name, v_new in sd_new.items():
            if _is_critic(name) or name not in sd_old:
                continue  # critic / frozen-encoder-only: never touched
            v_old = sd_old[name]
            if v_new.shape == v_old.shape:
                v_new.copy_(v_old)
            elif (v_new.dim() == 2 and v_new.shape[0] == v_old.shape[0]
                  and v_new.shape[1] > v_old.shape[1]):
                v_new.zero_()
                v_new[:, :v_old.shape[1]].copy_(v_old)
            else:
                raise SystemExit(
                    f"actor_only_transplant: unexpected shape change "
                    f"for {name}: {tuple(v_old.shape)} -> "
                    f"{tuple(v_new.shape)}")
            copied.append(name)
    if not copied:
        raise SystemExit(
            "actor_only_transplant copied nothing — check the "
            "checkpoint/policy architectures and critic_markers")
    return copied


@dataclass
class PredictorConfig:
    mode: str = "frozen"            # "frozen" (D) | "online" (E) |
                                    # "live" (boundary-gated snapshot,
                                    # fb_20260817T210422_9df9c7)
    batch_size: int = 256
    rehearsal_frac: float = 0.25
    steps_per_iter: int = 8         # predictor grad steps per PPO iter
    lr: float = 1e-4                # online predictor Adam LR
    ema_tau: float = 0.05           # snapshot <- online EMA rate (E)
    drift_guard: float = 0.05       # max probe-z MSE(candidate, snap)
    min_online_windows: int = 1024
    probe_obs: int = 1024
    lambdas: dict = field(default_factory=default_lambdas)
    # -- "live" mode only: the critic-facing snapshot may change no
    # faster than snapshot_boundary_steps PPO-step boundaries, and only
    # if EVERY gate passes; otherwise the attempt is logged + skipped.
    snapshot_boundary_steps: int = 1_000_000
    gate_heldout_band: float = 0.15   # generic corpus-val retention vs
                                      # the pretrained reference
    gate_live_improve: float = 0.0    # candidate must BEAT the current
                                      # snapshot on live command-rich
                                      # walk heldout by > this fraction
    gate_rise_band: float = 0.05      # live rise heldout retention band
    gate_value_jump_frac: float = 0.10  # mean |dV| <= frac*(mean|V|+1)


class PredictiveCriticPPO(PPO):
    """PPO for conditions D (frozen snapshot) and E (online predictor +
    guarded EMA snapshot). Without ``configure_predictor`` it behaves
    exactly like plain PPO."""

    _pcfg: PredictorConfig | None = None

    _PRED_RUNTIME_ATTRS = (
        "_pcfg", "_online", "_rehearsal", "_to_torch", "_sink",
        "_online_dyn", "_pred_opt", "_probe_hist", "_z_pretrained",
        "_snap_guard", "_version_guard",
        "_live_batch_fn", "_gate_fns", "_boundary_done",
        "_boundary_accepted", "_boundary_rejected",
    )

    def _excluded_save_params(self) -> list[str]:
        return (super()._excluded_save_params()
                + list(self._PRED_RUNTIME_ATTRS))

    def configure_predictor(self, cfg: PredictorConfig, rehearsal_sampler,
                            batch_to_torch, online_buffer=None,
                            metrics_sink=None, live_batch_fn=None,
                            gate_fns=None) -> None:
        if cfg.mode not in ("frozen", "online", "live"):
            raise ValueError("mode must be 'frozen', 'online' or 'live'")
        if cfg.mode == "online" and online_buffer is None:
            raise ValueError("online mode requires an online buffer")
        if cfg.mode == "live":
            # live mode (fb_20260817T210422_9df9c7): the trainer supplies
            # the stratified CUDA batch source and the snapshot-gate
            # evaluators:
            #   live_batch_fn(n) -> (device batch, info dict)
            #   gate_fns["corpus_val"](model) -> float   (generic heldout)
            #   gate_fns["live_val"](model, mode) -> float | None
            #   gate_fns["pretrained_ref"] -> float      (set after the
            #       start-of-run measurement of the untouched model)
            if live_batch_fn is None or gate_fns is None:
                raise ValueError(
                    "live mode requires live_batch_fn and gate_fns")
        self._pcfg = cfg
        self._online = online_buffer
        self._rehearsal = rehearsal_sampler
        self._to_torch = batch_to_torch
        self._sink = metrics_sink
        self._live_batch_fn = live_batch_fn
        self._gate_fns = gate_fns
        self._boundary_done = 0
        self._boundary_accepted = 0
        self._boundary_rejected = 0
        self._snap_guard = None
        self._version_guard = None
        self._ema_accepted = 0
        self._ema_rejected = 0
        self._pred_updates_total = 0
        # Fixed probe batch (rehearsal corpus) for latent-drift metrics.
        probe = rehearsal_sampler.batch(min(cfg.batch_size, 256))
        self._probe_hist = self._to_torch(probe,
                                          device=self.device)["hist"]
        with th.no_grad():
            self._z_pretrained = self._snapshot_model().encode(
                self._probe_hist).detach().clone()
        if cfg.mode in ("online", "live"):
            # The online predictor starts as an exact copy of the
            # snapshot (= the pretrained checkpoint — in live mode the
            # run therefore STARTS as exact frozen D) and NEVER shares
            # parameters with it or with the policy.
            self._online_dyn = copy.deepcopy(self._snapshot_model())
            for p in self._online_dyn.parameters():
                p.requires_grad_(True)
            self._online_dyn.train()
            self._pred_opt = th.optim.Adam(self._online_dyn.parameters(),
                                           lr=cfg.lr)
        else:
            self._online_dyn = None
            self._pred_opt = None

    # -- helpers -------------------------------------------------------

    def _snapshot_model(self):
        return self.policy.critic_predictor

    def _actor_param_names(self) -> list[str]:
        """Parameters whose mutation would change the action
        distribution: everything trainable except the pure-critic
        modules (value_net, latent_adapter, value_gate)."""
        crit = ("value_net.", "latent_adapter.", "value_gate")
        return [n for n, p in self.policy.named_parameters()
                if p.requires_grad and not n.startswith(crit)]

    def _clone_params(self, module) -> list[th.Tensor]:
        return [p.detach().clone() for p in module.parameters()]

    @staticmethod
    def _params_equal(module, clones) -> bool:
        params = list(module.parameters())
        return (len(params) == len(clones)
                and all(th.equal(p.detach(), c)
                        for p, c in zip(params, clones)))

    def _probe_obs_batch(self) -> th.Tensor:
        obs = self.rollout_buffer.observations
        flat = obs.reshape(-1, obs.shape[-1])[: self._pcfg.probe_obs]
        return th.as_tensor(flat, device=self.device)

    def _dist_params(self, obs_t: th.Tensor):
        with th.no_grad():
            d = self.policy.get_distribution(obs_t).distribution
            return d.mean.detach().clone(), d.stddev.detach().clone()

    def _action_kl(self, obs_t: th.Tensor, old_mean, old_std) -> float:
        with th.no_grad():
            new = self.policy.get_distribution(obs_t).distribution
            old = th.distributions.Normal(old_mean, old_std)
            return float(th.distributions.kl_divergence(old, new)
                         .sum(dim=-1).mean())

    # -- snapshot identity guards ---------------------------------------

    def collect_rollouts(self, *args, **kwargs):
        if self._pcfg is not None:
            # Freeze the snapshot's identity from the FIRST collected
            # transition: params cloned + version recorded here, both
            # asserted at train() start and after the PPO epochs.
            self._snap_guard = self._clone_params(self._snapshot_model())
            self._version_guard = int(
                self.policy.snapshot_version.item())
        return super().collect_rollouts(*args, **kwargs)

    def _assert_snapshot_stable(self, where: str) -> None:
        if self._snap_guard is None:
            return
        version = int(self.policy.snapshot_version.item())
        if version != self._version_guard:
            raise RuntimeError(
                f"critic snapshot VERSION changed inside a rollout+PPO "
                f"iteration ({where}): {self._version_guard} -> "
                f"{version}; snapshot updates are only legal between "
                f"complete iterations (fb_20260817T052333_e5ae09)")
        if not self._params_equal(self._snapshot_model(),
                                  self._snap_guard):
            raise RuntimeError(
                f"critic snapshot transformer was MUTATED inside a "
                f"rollout+PPO iteration ({where}); the snapshot is "
                f"inference-only and may change only via the guarded "
                f"between-iteration EMA update "
                f"(fb_20260817T052333_e5ae09)")

    # -- online predictor training --------------------------------------

    def _train_predictor(self) -> dict:
        cfg = self._pcfg
        logs: dict[str, float] = {}
        sums: dict[str, float] = {}
        re_fracs = []
        live_infos: list[dict] = []
        for _ in range(max(cfg.steps_per_iter, 0)):
            if cfg.mode == "live":
                bt, info = self._live_batch_fn(cfg.batch_size)
                live_infos.append(info)
            else:
                batch, re_frac = mixed_batch(
                    self._online, self._rehearsal, cfg.batch_size,
                    cfg.rehearsal_frac, cfg.min_online_windows)
                bt = self._to_torch(batch, device=self.device)
            out = self._online_dyn(bt["hist"], bt["fut_actions"])
            loss, metrics = dynamics_loss(out, bt, cfg.lambdas,
                                          self._online_dyn)
            self._pred_opt.zero_grad()
            loss.backward()
            th.nn.utils.clip_grad_norm_(self._online_dyn.parameters(),
                                        1.0)
            self._pred_opt.step()
            self._pred_updates_total += 1
            re_fracs.append(info["pred/batch_rehearsal_frac"]
                            if cfg.mode == "live" else re_frac)
            metrics.update(priv_group_metrics(out, bt, prefix=""))
            for k, v in metrics.items():
                sums[k] = sums.get(k, 0.0) + v
        n = max(len(re_fracs), 1)
        logs.update({f"pred/train/{k}": v / n for k, v in sums.items()})
        if re_fracs:
            logs["pred/rehearsal_frac"] = float(np.mean(re_fracs))
        if live_infos:
            keys = live_infos[0].keys()
            logs.update({k: float(np.mean([i[k] for i in live_infos]))
                         for k in keys})
        logs["pred/updates_total"] = self._pred_updates_total
        return logs

    def _ema_snapshot_update(self) -> dict:
        """Guarded EMA of the critic snapshot toward the online
        predictor — the ONLY legal snapshot mutation, called strictly
        between complete rollout+PPO iterations."""
        cfg = self._pcfg
        snap = self._snapshot_model()
        candidate = copy.deepcopy(snap)
        with th.no_grad():
            for pc, po in zip(candidate.parameters(),
                              self._online_dyn.parameters()):
                pc.mul_(1.0 - cfg.ema_tau).add_(po.detach(),
                                                alpha=cfg.ema_tau)
            for bc, bo in zip(candidate.buffers(),
                              self._online_dyn.buffers()):
                bc.copy_(bo)
            candidate.eval()
            z_cand = candidate.encode(self._probe_hist)
            z_snap = snap.encode(self._probe_hist)
            drift_step = float(F.mse_loss(z_cand, z_snap))
            drift_total = float(F.mse_loss(z_cand, self._z_pretrained))
            online_drift = float(F.mse_loss(
                self._online_dyn.encode(self._probe_hist),
                self._z_pretrained))
        accepted = drift_step <= cfg.drift_guard
        if accepted:
            with th.no_grad():
                for ps, pc in zip(snap.parameters(),
                                  candidate.parameters()):
                    ps.copy_(pc)
                for bs, bc in zip(snap.buffers(), candidate.buffers()):
                    bs.copy_(bc)
                self.policy.snapshot_version.add_(1)
            self._ema_accepted += 1
        else:
            self._ema_rejected += 1
        return {
            "pred/snap_drift_step": drift_step,
            "pred/snap_drift_guard": cfg.drift_guard,
            "pred/snap_drift_from_pretrained": drift_total,
            "pred/online_drift_from_pretrained": online_drift,
            "pred/ema_accepted": int(accepted),
            "pred/ema_accepted_total": self._ema_accepted,
            "pred/ema_rejected_total": self._ema_rejected,
            "pred/snapshot_version": int(
                self.policy.snapshot_version.item()),
        }

    def _copy_into_snapshot(self, source) -> None:
        snap = self._snapshot_model()
        with th.no_grad():
            for ps, po in zip(snap.parameters(), source.parameters()):
                ps.copy_(po.detach())
            for bs, bo in zip(snap.buffers(), source.buffers()):
                bs.copy_(bo)
        snap.eval()
        self.policy.snapshot_version.add_(1)

    def _boundary_snapshot_update(self) -> dict:
        """LIVE mode: versioned critic-snapshot update, attempted at
        most once per snapshot_boundary_steps PPO-step boundary,
        strictly BETWEEN complete rollout+PPO iterations, and applied
        only if EVERY gate passes (fb_20260817T210422_9df9c7):

            generic  corpus-val loss of the candidate within
                     gate_heldout_band of the pretrained reference
            live_walk  candidate beats the CURRENT snapshot on the
                     command-rich live walk heldout by > gate_live_improve
            live_rise  candidate within gate_rise_band of the current
                     snapshot on the live rise heldout (retention)
            drift    probe-latent MSE(candidate, snapshot) <= drift_guard
            value    critic value jump from swapping the snapshot on the
                     probe obs <= gate_value_jump_frac * (mean|V|+1)

        A failed gate leaves the snapshot untouched (rejection path);
        the attempt and every gate value are logged either way. The
        candidate is the online predictor itself; acceptance copies its
        parameters into the snapshot and bumps snapshot_version."""
        cfg = self._pcfg
        boundary = int(self.num_timesteps // cfg.snapshot_boundary_steps)
        payload: dict[str, float] = {
            "pred/snapshot_version": int(
                self.policy.snapshot_version.item()),
            "pred/boundary_index": boundary,
            "pred/boundary_accepted_total": self._boundary_accepted,
            "pred/boundary_rejected_total": self._boundary_rejected,
        }
        if boundary <= self._boundary_done:
            return payload
        self._boundary_done = boundary
        cand = self._online_dyn
        snap = self._snapshot_model()
        gates: dict[str, bool] = {}
        ref = float(self._gate_fns["pretrained_ref"]())
        cand_corpus = float(self._gate_fns["corpus_val"](cand))
        gates["generic"] = (cand_corpus
                            <= ref * (1.0 + cfg.gate_heldout_band))
        cand_walk = self._gate_fns["live_val"](cand, "walk")
        snap_walk = self._gate_fns["live_val"](snap, "walk")
        gates["live_walk"] = (
            cand_walk is not None and snap_walk is not None
            and cand_walk < snap_walk * (1.0 - cfg.gate_live_improve))
        cand_rise = self._gate_fns["live_val"](cand, "rise")
        snap_rise = self._gate_fns["live_val"](snap, "rise")
        gates["live_rise"] = (
            cand_rise is not None and snap_rise is not None
            and cand_rise <= snap_rise * (1.0 + cfg.gate_rise_band))
        with th.no_grad():
            was_training = cand.training
            cand.eval()
            z_cand = cand.encode(self._probe_hist)
            z_snap = snap.encode(self._probe_hist)
            drift = float(F.mse_loss(z_cand, z_snap))
            gates["drift"] = drift <= cfg.drift_guard
            # value-jump: residual with candidate vs current snapshot
            # on the probe OBS through the critic's own adapter+gate.
            obs = self._probe_obs_batch()
            frames = self.policy.obs_to_frames(obs)
            res_snap = (self.policy.value_gate
                        * self.policy.latent_adapter(snap.encode(frames)))
            res_cand = (self.policy.value_gate
                        * self.policy.latent_adapter(cand.encode(frames)))
            v_now = self.policy.predict_values(obs)
            dv = float((res_cand - res_snap).abs().mean())
            v_scale = float(v_now.abs().mean()) + 1.0
            gates["value"] = dv <= cfg.gate_value_jump_frac * v_scale
            if was_training:
                cand.train()
        accepted = all(gates.values())
        if accepted:
            self._copy_into_snapshot(cand)
            self._boundary_accepted += 1
        else:
            self._boundary_rejected += 1
        payload.update({
            "pred/gate/attempted": 1,
            "pred/gate/accepted": int(accepted),
            "pred/gate/generic": int(gates["generic"]),
            "pred/gate/live_walk": int(gates["live_walk"]),
            "pred/gate/live_rise": int(gates["live_rise"]),
            "pred/gate/drift": int(gates["drift"]),
            "pred/gate/value": int(gates["value"]),
            "pred/gate/corpus_val_candidate": cand_corpus,
            "pred/gate/corpus_val_ref": ref,
            "pred/gate/live_walk_candidate": float(cand_walk)
            if cand_walk is not None else float("nan"),
            "pred/gate/live_walk_snapshot": float(snap_walk)
            if snap_walk is not None else float("nan"),
            "pred/gate/live_rise_candidate": float(cand_rise)
            if cand_rise is not None else float("nan"),
            "pred/gate/live_rise_snapshot": float(snap_rise)
            if snap_rise is not None else float("nan"),
            "pred/gate/latent_drift": drift,
            "pred/gate/value_jump": dv,
            "pred/snapshot_version": int(
                self.policy.snapshot_version.item()),
            "pred/boundary_accepted_total": self._boundary_accepted,
            "pred/boundary_rejected_total": self._boundary_rejected,
        })
        return payload

    # -- the coordinated iteration --------------------------------------

    def train(self) -> None:
        if self._pcfg is None:
            return super().train()
        self._assert_snapshot_stable("before predictor updates")
        payload: dict[str, float] = {}
        probe = self._probe_obs_batch()

        if self._pcfg.mode in ("online", "live"):
            # 1. online predictor updates: own optimizer, own params.
            #    Proof obligations: actor params bit-identical and
            #    actor action-KL exactly zero across these updates.
            actor_names = self._actor_param_names()
            pdict = dict(self.policy.named_parameters())
            actor_before = {n: pdict[n].detach().clone()
                            for n in actor_names}
            old_mean, old_std = self._dist_params(probe)
            payload.update(self._train_predictor())
            kl_from_pred = self._action_kl(probe, old_mean, old_std)
            payload["pred/actor_kl_from_predictor"] = kl_from_pred
            mutated = [n for n in actor_names
                       if not th.equal(pdict[n].detach(),
                                       actor_before[n])]
            if mutated or kl_from_pred != 0.0:
                raise RuntimeError(
                    f"online predictor update touched the ACTOR "
                    f"(mutated={mutated[:4]}, actor_kl={kl_from_pred}); "
                    f"the actor must be completely independent "
                    f"(fb_20260817T052333_e5ae09)")
            self._assert_snapshot_stable("after predictor updates")

        # 2. the standard PPO update (actor + raw critic + adapter +
        #    gate). The snapshot cannot move: requires_grad False,
        #    excluded from the optimizer, and bit-checked right after.
        super().train()
        self._assert_snapshot_stable("after PPO epochs")
        self._snap_guard = None
        self._version_guard = None

        # Critic-branch attribution on the probe observations.
        with th.no_grad():
            raw_v = self.policy.predict_values(probe) \
                - self.policy.value_residual(probe)
            res_v = self.policy.value_residual(probe)
            payload.update({
                "critic/gate": float(self.policy.value_gate.detach()),
                "critic/raw_value_abs_mean": float(raw_v.abs().mean()),
                "critic/residual_abs_mean": float(res_v.abs().mean()),
                "critic/adapter_out_abs_mean": float(
                    self.policy.latent_adapter(
                        self.policy.predictive_latent(probe))
                    .abs().mean()),
                "critic/value_mean": float(
                    (raw_v + res_v).mean()),
                "critic/return_mean": float(
                    np.mean(self.rollout_buffer.returns)),
            })
        if self._online is not None:
            payload["pred/online_windows"] = self._online.num_windows()
            payload["pred/online_frames"] = self._online.frames_total
            payload["pred/online_episodes_total"] = (
                self._online.episodes_added)

        # 3. atomic snapshot update BETWEEN iterations: E = guarded EMA
        #    every iteration; live = versioned boundary update behind
        #    the full gate battery; D = never.
        if self._pcfg.mode == "online":
            payload.update(self._ema_snapshot_update())
        elif self._pcfg.mode == "live":
            payload.update(self._boundary_snapshot_update())
        else:
            payload["pred/snapshot_version"] = int(
                self.policy.snapshot_version.item())

        self.last_pred_payload = payload
        if self._sink is not None:
            self._sink(payload, self.num_timesteps)
