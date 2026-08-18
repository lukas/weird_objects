"""joint_aux.py — corrected condition C: PPO + dynamics auxiliary loss
as ONE coordinated update (operator directive fb_20260816T203212_af7c64).

The metrics1 cohort showed the old condition-C mechanism (AnchorCb:
several out-of-band Adam steps on the shared transformer AFTER rollout
collection, BEFORE PPO consumed the buffer) is unstable: C led at 1M
then regressed to last at 2M with approx_kl ~0.085-0.089 vs A/B ~0.02.
Root cause: mutating the shared representation between collection and
the PPO update invalidates the old-policy/value assumptions. Merely
moving the callback after PPO would still ship an independently shifted
representation into the next rollout.

This module replaces it:

  * The future-state auxiliary loss joins the PPO loss INSIDE the PPO
    train block — same backward pass, same optimizer step, same LR
    schedule; the transformer sits in a scaled-LR param group
    (``set_group_lrs``), so it moves slower than the heads but only
    ever moves as part of the coordinated update.
  * A brief encoder-frozen head warmup lets actor/value heads learn to
    consume the pretrained latent before joint training starts.
  * Auxiliary batches are online-primary (fresh windows captured from
    the policy's own rollouts) with a 20-30% rehearsal mix from the
    recovered pretraining corpus.
  * The TOTAL policy movement of the combined update is guarded: the
    post-update action KL vs the pre-update policy (analytic diagonal
    Gaussian, summed over action dims, meaned over a fixed probe of
    rollout observations) is logged every update; when it exceeds the
    guard while the auxiliary was active, the whole update is ROLLED
    BACK (params + optimizer state) and redone without the auxiliary.
    Repeated consecutive rejections stop auxiliary training entirely.
  * A hard mutation guard makes the old failure mode impossible: the
    shared transformer's parameters are snapshotted when a rollout
    finishes and verified bit-identical when the PPO update begins —
    any out-of-band mutation (e.g. a legacy anchor callback) raises.
  * Latent drift (MSE of the current encoder's z vs the pretrained
    encoder's z on a fixed probe batch) is logged every update.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np
import torch as th
import torch.nn.functional as F
from gymnasium import spaces
from stable_baselines3.common.utils import explained_variance

from .model import dynamics_loss
from .online_windows import mixed_batch
from .sb3_encoder import ScaledLRPPO


def default_lambdas() -> dict:
    """Pretraining-equivalent loss weights (matches the retired AnchorCb
    plus the current-state heads so every supervised target — velocity/
    heading via priv, foot contacts, servo currents, motor/servo state —
    keeps training)."""
    return {"joint_pos": 1.0, "joint_vel": 1.0, "imu": 1.0,
            "contact": 0.5, "latent": 1.0,
            "contact_current": 0.5, "motor_current": 0.5,
            "priv_current": 0.25, "priv_future": 0.25}


# Privileged-target channel groups (frames.py PRIV_NAMES) for honest
# per-target reporting: velocity, heading/yaw, command tracking.
PRIV_GROUPS = {
    "vel": (0, 1, 2, 3),          # vx/vy/vz body + wz
    "height": (4,),               # chassis z
    "yaw_heading": (5, 6, 12, 13),  # sin/cos yaw_rel + cmd heading
    "cmd_track": (7, 8, 9, 10, 11),  # cmd refs + along/cross velocity
}


@dataclass
class AuxConfig:
    coef: float = 1.0
    batch_size: int = 256
    rehearsal_frac: float = 0.25
    warmup_steps: int = 50_000
    kl_target: float = 0.02
    kl_guard: float = 0.04
    stop_after: int = 3
    min_online_windows: int = 1024
    probe_obs: int = 1024
    lambdas: dict = field(default_factory=default_lambdas)


def priv_group_metrics(out: dict, bt: dict, prefix: str) -> dict:
    """Masked per-group MSE for current + future privileged targets."""
    logs: dict[str, float] = {}
    if "priv_now" not in out or "priv_now" not in bt:
        return logs
    mask = bt.get("priv_mask_now")
    if mask is None:
        mask = th.ones_like(bt["priv_now"])

    def group_mse(pred: th.Tensor, tgt: th.Tensor, cols) -> float:
        c = list(cols)
        d = (pred[:, c] - tgt[:, c]).square() * mask[:, c]
        return float((d.sum() / mask[:, c].sum().clamp_min(1)).detach())

    for name, cols in PRIV_GROUPS.items():
        logs[f"{prefix}now/{name}"] = group_mse(
            out["priv_now"], bt["priv_now"], cols)
        if out.get("priv"):
            vals = [group_mse(out["priv"][k], bt["priv"][k], cols)
                    for k in out["priv"]]
            logs[f"{prefix}future/{name}"] = float(np.mean(vals))
    return logs


class JointAuxPPO(ScaledLRPPO):
    """PPO whose dynamics auxiliary trains INSIDE the PPO update.

    Without ``configure_aux`` this class behaves exactly like
    ``ScaledLRPPO``. With it, every optimization minibatch adds
    ``coef * dynamics_loss(mixed online/rehearsal batch)`` to the PPO
    loss, guarded as documented in the module docstring.
    """

    _aux: AuxConfig | None = None

    # Aux runtime state must NEVER be pickled into SB3's `data` blob.
    # tfwalk-joint1 (08-16): every C checkpoint serialized the rehearsal
    # sampler + online window buffer (12.5GB `data` entry per zip), and
    # the save-time RSS spike (~20-30GB in <60s) crossed the pods'
    # memwatch 85GiB kill threshold — all three C arms were SIGKILLed
    # MID-SAVE, leaving 0-byte checkpoint husks (C-s5/s6 at ck500000,
    # C-s7 at its final ck1000000). Excluding names absent from
    # __dict__ is a no-op, so A/B (no configure_aux) saves stay
    # bit-identical.
    _AUX_RUNTIME_ATTRS = (
        "_aux", "_online", "_rehearsal", "_to_torch", "_sink", "_dyn",
        "_enc_snapshot", "_probe_hist", "_z_init",
    )

    def _excluded_save_params(self) -> list[str]:
        return (super()._excluded_save_params()
                + list(self._AUX_RUNTIME_ATTRS))

    def configure_aux(self, cfg: AuxConfig, online_buffer,
                      rehearsal_sampler, batch_to_torch,
                      metrics_sink=None) -> None:
        self._aux = cfg
        self._online = online_buffer
        self._rehearsal = rehearsal_sampler
        self._to_torch = batch_to_torch
        self._sink = metrics_sink
        self._dyn = self.policy.features_extractor.dyn
        self._enc_snapshot = None
        self._enc_unfrozen = cfg.warmup_steps <= 0
        self._aux_stopped = False
        self._consec_rejects = 0
        self._updates_accepted = 0
        self._updates_rejected = 0
        self._aux_batches_accepted = 0
        self._aux_batches_rejected = 0
        # Latent-drift probe: one fixed rehearsal batch + pretrained z.
        probe = rehearsal_sampler.batch(min(cfg.batch_size, 256))
        self._probe_hist = self._to_torch(probe, device=self.device)["hist"]
        with th.no_grad():
            self._z_init = self._dyn.encode(self._probe_hist).detach().clone()
        if not self._enc_unfrozen:
            self._set_encoder_frozen(True)

    # -- guards -----------------------------------------------------

    def collect_rollouts(self, *args, **kwargs):
        # Snapshot BEFORE collection starts: SB3 fires on_rollout_start/
        # on_step/on_rollout_end callbacks inside collect_rollouts, so a
        # post-return snapshot would capture (and thereby launder) any
        # callback mutation. This way the shared transformer must be
        # bit-identical from the first collected transition to the
        # moment the PPO update evaluates the old policy.
        if self._aux is not None:
            self._enc_snapshot = [p.detach().clone()
                                  for p in self._dyn.parameters()]
        return super().collect_rollouts(*args, **kwargs)

    def _assert_encoder_unmutated(self) -> None:
        if self._enc_snapshot is None:
            return
        params = list(self._dyn.parameters())
        if len(params) != len(self._enc_snapshot) or any(
                not th.equal(p.detach(), s)
                for p, s in zip(params, self._enc_snapshot)):
            raise RuntimeError(
                "shared dynamics transformer was mutated OUT-OF-BAND "
                "between rollout collection and the PPO update; "
                "out-of-band anchor updates are forbidden "
                "(fb_20260816T203212_af7c64) — the auxiliary objective "
                "must train inside the PPO train block")
        self._enc_snapshot = None

    def _set_encoder_frozen(self, frozen: bool) -> None:
        for p in self._dyn.parameters():
            p.requires_grad_(not frozen)

    # -- policy-movement probe ---------------------------------------

    def _probe_obs(self) -> th.Tensor:
        obs = self.rollout_buffer.observations
        flat = obs.reshape(-1, obs.shape[-1])[: self._aux.probe_obs]
        return th.as_tensor(flat, device=self.device)

    def _dist_params(self, obs_t: th.Tensor):
        with th.no_grad():
            d = self.policy.get_distribution(obs_t).distribution
            return d.mean.detach().clone(), d.stddev.detach().clone()

    def _action_kl(self, obs_t: th.Tensor, old_mean: th.Tensor,
                   old_std: th.Tensor) -> float:
        with th.no_grad():
            new = self.policy.get_distribution(obs_t).distribution
            old = th.distributions.Normal(old_mean, old_std)
            kl = th.distributions.kl_divergence(old, new)
            return float(kl.sum(dim=-1).mean())

    # -- rollback ------------------------------------------------------

    def _snapshot_policy(self):
        return ({k: v.detach().clone()
                 for k, v in self.policy.state_dict().items()},
                copy.deepcopy(self.policy.optimizer.state_dict()))

    def _restore_policy(self, snap) -> None:
        params, opt_state = snap
        self.policy.load_state_dict(params)
        self.policy.optimizer.load_state_dict(opt_state)

    # -- training ------------------------------------------------------

    def train(self) -> None:
        if self._aux is None:
            return super().train()
        self._assert_encoder_unmutated()
        cfg = self._aux
        if not self._enc_unfrozen and self.num_timesteps >= cfg.warmup_steps:
            self._set_encoder_frozen(False)
            self._enc_unfrozen = True
        aux_active = self._enc_unfrozen and not self._aux_stopped
        probe = self._probe_obs()
        old_mean, old_std = self._dist_params(probe)
        snap = self._snapshot_policy() if aux_active else None
        n_updates0 = self._n_updates
        aux_logs, n_aux_batches = self._train_once(aux_active)
        kl_total = self._action_kl(probe, old_mean, old_std)
        rejected = 0
        if aux_active and kl_total > cfg.kl_guard:
            # The COMBINED update moved the policy too far: roll back
            # params + optimizer state, redo without the auxiliary. The
            # retry gives ATTRIBUTION: only a breach that disappears
            # without the auxiliary counts toward stopping it — if the
            # no-aux redo breaches too, the movement is PPO's own and
            # stands (measured 08-16 on tfwalk-joint1-C-s5: PPO alone
            # ran approx_kl ~0.03 in this task's early phase, so
            # counting every combined breach stopped the auxiliary
            # permanently ~50k steps in, degenerating condition C into
            # a no-aux arm — an artifact, not the hypothesis).
            self._restore_policy(snap)
            self._n_updates = n_updates0
            kl_with_aux = kl_total
            self._train_once(False)
            kl_total = self._action_kl(probe, old_mean, old_std)
            rejected = 1
            self._updates_rejected += 1
            self._aux_batches_rejected += n_aux_batches
            aux_logs["aux/action_kl_rejected"] = kl_with_aux
            aux_logs["aux/action_kl_retry"] = kl_total
            if kl_total <= cfg.kl_guard:
                # breach attributable to the auxiliary
                self._consec_rejects += 1
                if self._consec_rejects >= cfg.stop_after:
                    self._aux_stopped = True
        elif aux_active:
            self._updates_accepted += 1
            self._aux_batches_accepted += n_aux_batches
            self._consec_rejects = 0
        with th.no_grad():
            drift = F.mse_loss(self._dyn.encode(self._probe_hist),
                               self._z_init)
        payload = {
            "aux/action_kl_total": kl_total,
            "aux/kl_target": cfg.kl_target,
            "aux/kl_guard": cfg.kl_guard,
            "aux/active": int(aux_active),
            "aux/rejected": rejected,
            "aux/updates_accepted_total": self._updates_accepted,
            "aux/updates_rejected_total": self._updates_rejected,
            "aux/batches_accepted_total": self._aux_batches_accepted,
            "aux/batches_rejected_total": self._aux_batches_rejected,
            "aux/stopped": int(self._aux_stopped),
            "aux/encoder_unfrozen": int(self._enc_unfrozen),
            "aux/latent_drift": float(drift),
            "aux/online_windows": self._online.num_windows(),
            "aux/online_frames": self._online.frames_total,
            "aux/online_episodes_total": self._online.episodes_added,
            **aux_logs,
        }
        self.last_aux_payload = payload
        if self._sink is not None:
            self._sink(payload, self.num_timesteps)

    def _train_once(self, aux_active: bool) -> tuple[dict, int]:
        """One full PPO optimization pass over the rollout buffer —
        SB3 2.9.0 ``PPO.train`` body with the auxiliary loss joined to
        each minibatch loss (same backward, same optimizer step)."""
        cfg = self._aux
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)
        clip_range = self.clip_range(self._current_progress_remaining)
        if self.clip_range_vf is not None:
            clip_range_vf = self.clip_range_vf(
                self._current_progress_remaining)

        entropy_losses = []
        pg_losses, value_losses = [], []
        clip_fractions = []
        aux_metric_sums: dict[str, float] = {}
        aux_metric_n = 0
        rehearsal_fracs = []
        n_aux_batches = 0

        continue_training = True
        for epoch in range(self.n_epochs):
            approx_kl_divs = []
            for rollout_data in self.rollout_buffer.get(self.batch_size):
                actions = rollout_data.actions
                if isinstance(self.action_space, spaces.Discrete):
                    actions = rollout_data.actions.long().flatten()

                values, log_prob, entropy = self.policy.evaluate_actions(
                    rollout_data.observations, actions)
                values = values.flatten()
                advantages = rollout_data.advantages
                if self.normalize_advantage and len(advantages) > 1:
                    advantages = (advantages - advantages.mean()) / (
                        advantages.std() + 1e-8)

                ratio = th.exp(log_prob - rollout_data.old_log_prob)
                policy_loss_1 = advantages * ratio
                policy_loss_2 = advantages * th.clamp(
                    ratio, 1 - clip_range, 1 + clip_range)
                policy_loss = -th.min(policy_loss_1, policy_loss_2).mean()

                pg_losses.append(policy_loss.item())
                clip_fraction = th.mean(
                    (th.abs(ratio - 1) > clip_range).float()).item()
                clip_fractions.append(clip_fraction)

                if self.clip_range_vf is None:
                    values_pred = values
                else:
                    values_pred = rollout_data.old_values + th.clamp(
                        values - rollout_data.old_values,
                        -clip_range_vf, clip_range_vf)
                value_loss = F.mse_loss(rollout_data.returns, values_pred)
                value_losses.append(value_loss.item())

                if entropy is None:
                    entropy_loss = -th.mean(-log_prob)
                else:
                    entropy_loss = -th.mean(entropy)
                entropy_losses.append(entropy_loss.item())

                loss = (policy_loss + self.ent_coef * entropy_loss
                        + self.vf_coef * value_loss)

                if aux_active:
                    batch, re_frac = mixed_batch(
                        self._online, self._rehearsal, cfg.batch_size,
                        cfg.rehearsal_frac, cfg.min_online_windows)
                    bt = self._to_torch(batch, device=self.device)
                    out = self._dyn(bt["hist"], bt["fut_actions"])
                    aux_loss, aux_metrics = dynamics_loss(
                        out, bt, cfg.lambdas, self._dyn)
                    loss = loss + cfg.coef * aux_loss
                    n_aux_batches += 1
                    rehearsal_fracs.append(re_frac)
                    for key, val in aux_metrics.items():
                        aux_metric_sums[key] = (
                            aux_metric_sums.get(key, 0.0) + val)
                    for key, val in priv_group_metrics(
                            out, bt, prefix="").items():
                        aux_metric_sums[key] = (
                            aux_metric_sums.get(key, 0.0) + val)
                    aux_metric_n += 1

                with th.no_grad():
                    log_ratio = log_prob - rollout_data.old_log_prob
                    approx_kl_div = th.mean(
                        (th.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                    approx_kl_divs.append(approx_kl_div)

                if (self.target_kl is not None
                        and approx_kl_div > 1.5 * self.target_kl):
                    continue_training = False
                    if self.verbose >= 1:
                        print(f"Early stopping at step {epoch} due to "
                              f"reaching max kl: {approx_kl_div:.2f}")
                    break

                self.policy.optimizer.zero_grad()
                loss.backward()
                th.nn.utils.clip_grad_norm_(self.policy.parameters(),
                                            self.max_grad_norm)
                self.policy.optimizer.step()

            self._n_updates += 1
            if not continue_training:
                break

        explained_var = explained_variance(
            self.rollout_buffer.values.flatten(),
            self.rollout_buffer.returns.flatten())

        self.logger.record("train/entropy_loss", np.mean(entropy_losses))
        self.logger.record("train/policy_gradient_loss", np.mean(pg_losses))
        self.logger.record("train/value_loss", np.mean(value_losses))
        self.logger.record("train/approx_kl", np.mean(approx_kl_divs))
        self.logger.record("train/clip_fraction", np.mean(clip_fractions))
        self.logger.record("train/loss", loss.item())
        self.logger.record("train/explained_variance", explained_var)
        if hasattr(self.policy, "log_std"):
            self.logger.record(
                "train/std", th.exp(self.policy.log_std).mean().item())
        self.logger.record("train/n_updates", self._n_updates,
                           exclude="tensorboard")
        self.logger.record("train/clip_range", clip_range)
        if self.clip_range_vf is not None:
            self.logger.record("train/clip_range_vf", clip_range_vf)

        aux_logs: dict[str, float] = {}
        if aux_metric_n:
            aux_logs = {f"aux/train/{k}": v / aux_metric_n
                        for k, v in aux_metric_sums.items()}
            aux_logs["aux/rehearsal_frac"] = float(
                np.mean(rehearsal_fracs))
        return aux_logs, n_aux_batches
