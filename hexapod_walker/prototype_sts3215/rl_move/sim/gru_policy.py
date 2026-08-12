"""GRU recurrent actor-critic policy for sb3-contrib's RecurrentPPO.

sb3-contrib 2.9.0 ships RecurrentPPO with LSTM cells only. This module
provides ``GruActorCriticPolicy``: the same RecurrentActorCriticPolicy,
but with ``nn.GRU`` in place of ``nn.LSTM`` for the actor (and, by
default, the critic).

Why this works without touching RecurrentPPO itself:

- RecurrentPPO only reads ``policy.lstm_actor.num_layers`` /
  ``.hidden_size`` (both exist on nn.GRU) and threads opaque
  ``(h, c)`` state tuples through rollout collection and the
  RecurrentRolloutBuffer.
- ALL cell-type-specific math lives in one static method,
  ``_process_sequence``, which we override. A GRU has no cell state,
  so the ``c`` slot of every state tuple is simply carried through
  untouched (it stays zeros forever). Slightly wasteful buffer memory,
  zero behavioral difference.

Checkpoints save/load through the normal SB3 zip path: the zip pickles
this class, so ``RecurrentPPO.load`` reconstructs it as long as
``rl_move.sim.gru_policy`` is importable (same convention as
``asym_policy.AsymActorCriticPolicy``).

Use ``load_checkpoint_auto`` to load a checkpoint without knowing
whether it is a plain PPO/MLP zip or a RecurrentPPO/GRU zip.
"""
from __future__ import annotations

from pathlib import Path

import torch as th
from torch import nn

from sb3_contrib.common.recurrent.policies import RecurrentActorCriticPolicy


class GruActorCriticPolicy(RecurrentActorCriticPolicy):
    """RecurrentActorCriticPolicy with GRU cells instead of LSTM.

    Constructor args are identical to RecurrentActorCriticPolicy
    (``lstm_hidden_size``, ``n_lstm_layers``, ``shared_lstm``,
    ``enable_critic_lstm``, ``lstm_kwargs`` keep their names so the
    kwargs stored in existing-style checkpoints stay compatible).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Replace the LSTMs the parent built with GRUs of the same
        # geometry. num_layers/hidden_size/input_size all match, so
        # RecurrentPPO's state-shape bookkeeping is unaffected.
        self.lstm_actor = nn.GRU(
            self.lstm_actor.input_size,
            self.lstm_actor.hidden_size,
            num_layers=self.lstm_actor.num_layers,
            **self.lstm_kwargs,
        )
        if self.lstm_critic is not None:
            self.lstm_critic = nn.GRU(
                self.lstm_critic.input_size,
                self.lstm_critic.hidden_size,
                num_layers=self.lstm_critic.num_layers,
                **self.lstm_kwargs,
            )
        # The parent's optimizer was built over the (now discarded)
        # LSTM parameters; rebuild it over the live module set.
        lr = self.optimizer.defaults["lr"]
        self.optimizer = self.optimizer_class(
            self.parameters(), lr=lr, **self.optimizer_kwargs)

    @staticmethod
    def _process_sequence(
        features: th.Tensor,
        lstm_states: tuple[th.Tensor, th.Tensor],
        episode_starts: th.Tensor,
        lstm: nn.GRU,
    ) -> tuple[th.Tensor, tuple[th.Tensor, th.Tensor]]:
        """GRU forward pass mirroring the parent's LSTM version.

        ``lstm_states`` is the (h, c) tuple RecurrentPPO carries; only
        h is used, c passes through untouched (always zeros).
        """
        h, c_unused = lstm_states[0], lstm_states[1]
        n_seq = h.shape[1]
        # Batch to sequence: (padded batch, feat) -> (len, n_seq, feat).
        features_sequence = features.reshape(
            (n_seq, -1, lstm.input_size)).swapaxes(0, 1)
        episode_starts = episode_starts.reshape((n_seq, -1)).swapaxes(0, 1)

        # No resets inside the sequence: single fused GRU call.
        if th.all(episode_starts == 0.0):
            gru_output, h = lstm(features_sequence, h)
            gru_output = th.flatten(
                gru_output.transpose(0, 1), start_dim=0, end_dim=1)
            return gru_output, (h, c_unused)

        gru_output = []
        for feat, episode_start in zip(
                features_sequence, episode_starts, strict=True):
            hidden, h = lstm(
                feat.unsqueeze(dim=0),
                # Reset hidden state where a new episode begins.
                (1.0 - episode_start).view(1, n_seq, 1) * h,
            )
            gru_output += [hidden]
        gru_output = th.flatten(
            th.cat(gru_output).transpose(0, 1), start_dim=0, end_dim=1)
        return gru_output, (h, c_unused)


# --- Dual-core (mode-gated) GRU -------------------------------------
#
# Born from the cw-arch-gru-anchor1..3 closure (08-12): a SINGLE shared
# GRU trunk cannot hold sharp anchored stance skills and a displacing
# walk at once — anchor3 proved the interference is PPO's own gradient
# through the shared trunk (walk froze even with the anchor gradient
# detached from it), while ft1 proved walk survives 10M steps of RL
# when nothing else competes for the trunk. So: give locomotion and
# stance their own complete cores (GRU + actor/critic heads each) and
# route per sample by the obs.mode_onehot tail. Both cores run every
# tick (their memories stay warm across mode changes); only the OUTPUT
# is selected, so each core receives gradient exclusively from its own
# skill family's ticks. Interference is gone by construction, yet it
# is still one checkpoint, one predict() interface, one brain to
# deploy.
#
# REQUIREMENT: the env must append the 6-wide skill-family one-hot at
# the obs tail (obs.mode_onehot=1). Slot order is frozen in
# walk_task.MODE_ONEHOT_ORDER = (hold, rise, lower, walk, turn, quad);
# locomotion = the last three slots (walk/turn/quad — getup rides the
# walk family), stance = the first three. test_gru_policy.py
# cross-checks this against walk_task so the two can never drift.

N_MODE_OBS = 6          # width of the obs-tail one-hot
_N_LOCO_SLOTS = 3       # trailing slots (walk, turn, quad) = core A


class _DualGRU(nn.Module):
    """Two parallel single-layer GRU cores behind one state facade.

    ``num_layers=2`` is a lie in the stacked-layer sense: it makes
    RecurrentPPO size its opaque state buffers as (2, n_envs, H) so
    row 0 threads core A (locomotion) and row 1 core B (stance).
    """

    def __init__(self, input_size: int, hidden_size: int, **gru_kwargs):
        super().__init__()
        self.core_a = nn.GRU(input_size, hidden_size, **gru_kwargs)
        self.core_b = nn.GRU(input_size, hidden_size, **gru_kwargs)
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = 2  # facade: 2 state rows, not stacked layers


class DualGruActorCriticPolicy(GruActorCriticPolicy):
    """Mode-gated dual-core GRU policy (locomotion core + stance core).

    Same constructor surface as GruActorCriticPolicy. Requires the
    default recurrent layout (critic GRU enabled, no shared_lstm, no
    SDE) and n_lstm_layers=1 per core.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.lstm_critic is None or self.shared_lstm:
            raise ValueError(
                "DualGruActorCriticPolicy requires enable_critic_lstm="
                "True and shared_lstm=False (the defaults)")
        if self.use_sde:
            raise ValueError("DualGruActorCriticPolicy does not support "
                             "use_sde")
        if self.lstm_actor.num_layers != 1:
            raise ValueError("DualGruActorCriticPolicy requires "
                             "n_lstm_layers=1 (one layer per core)")
        import copy

        self.lstm_actor = _DualGRU(
            self.lstm_actor.input_size, self.lstm_actor.hidden_size,
            **self.lstm_kwargs)
        self.lstm_critic = _DualGRU(
            self.lstm_critic.input_size, self.lstm_critic.hidden_size,
            **self.lstm_kwargs)
        # predict() sizes zero states off this: (rows, 1, H).
        self.lstm_hidden_state_shape = (
            2, 1, self.lstm_actor.hidden_size)
        # Core B's own heads. deepcopy keeps the architecture in sync
        # with whatever net_arch built core A's heads; the two diverge
        # immediately under their disjoint per-mode gradients.
        self.mlp_extractor_b = copy.deepcopy(self.mlp_extractor)
        self.action_net_b = copy.deepcopy(self.action_net)
        self.value_net_b = copy.deepcopy(self.value_net)
        lr = self.optimizer.defaults["lr"]
        self.optimizer = self.optimizer_class(
            self.parameters(), lr=lr, **self.optimizer_kwargs)

    # -- gating --------------------------------------------------

    @staticmethod
    def _gate(obs_or_feats: th.Tensor) -> th.Tensor:
        """(..., obs) -> (..., 1); 1.0 = locomotion family (core A).

        Reads the frozen obs-tail one-hot. Exactly one slot is lit per
        tick, so summing the trailing locomotion slots is exact.
        """
        loco = obs_or_feats[..., -_N_LOCO_SLOTS:]
        return loco.sum(dim=-1, keepdim=True).clamp(0.0, 1.0)

    def _dual_sequence(self, features, lstm_states, episode_starts, dual):
        """Run BOTH cores over the sequence; return their outputs and
        the repacked (h, c) with h rows [core_a, core_b]."""
        base = GruActorCriticPolicy._process_sequence
        h, c = lstm_states[0], lstm_states[1]
        out_a, (h_a, _) = base(
            features, (h[0:1], c[0:1]), episode_starts, dual.core_a)
        out_b, (h_b, _) = base(
            features, (h[1:2], c[1:2]), episode_starts, dual.core_b)
        return out_a, out_b, (th.cat([h_a, h_b], dim=0), c)

    def _actor_mean(self, out_a, out_b, gate):
        mu_a = self.action_net(self.mlp_extractor.forward_actor(out_a))
        mu_b = self.action_net_b(
            self.mlp_extractor_b.forward_actor(out_b))
        return gate * mu_a + (1.0 - gate) * mu_b

    def _critic_value(self, out_a, out_b, gate):
        v_a = self.value_net(self.mlp_extractor.forward_critic(out_a))
        v_b = self.value_net_b(
            self.mlp_extractor_b.forward_critic(out_b))
        return gate * v_a + (1.0 - gate) * v_b

    def _dist_from_mean(self, mean_actions):
        return self.action_dist.proba_distribution(
            mean_actions, self.log_std)

    # -- RecurrentPPO entry points ---------------------------------

    def forward(self, obs, lstm_states, episode_starts,
                deterministic: bool = False):
        features = self.extract_features(obs)
        if self.share_features_extractor:
            pi_features = vf_features = features
        else:
            pi_features, vf_features = features
        gate = self._gate(obs)
        pa, pb, st_pi = self._dual_sequence(
            pi_features, lstm_states.pi, episode_starts, self.lstm_actor)
        va, vb, st_vf = self._dual_sequence(
            vf_features, lstm_states.vf, episode_starts, self.lstm_critic)
        values = self._critic_value(va, vb, gate)
        distribution = self._dist_from_mean(self._actor_mean(pa, pb, gate))
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        actions = actions.reshape((-1, *self.action_space.shape))
        from sb3_contrib.common.recurrent.type_aliases import RNNStates
        return actions, values, log_prob, RNNStates(st_pi, st_vf)

    def get_distribution(self, obs, lstm_states, episode_starts):
        from stable_baselines3.common.policies import ActorCriticPolicy
        features = super(ActorCriticPolicy, self).extract_features(
            obs, self.pi_features_extractor)
        gate = self._gate(obs)
        pa, pb, st = self._dual_sequence(
            features, lstm_states, episode_starts, self.lstm_actor)
        return self._dist_from_mean(self._actor_mean(pa, pb, gate)), st

    def predict_values(self, obs, lstm_states, episode_starts):
        from stable_baselines3.common.policies import ActorCriticPolicy
        features = super(ActorCriticPolicy, self).extract_features(
            obs, self.vf_features_extractor)
        gate = self._gate(obs)
        va, vb, _ = self._dual_sequence(
            features, lstm_states, episode_starts, self.lstm_critic)
        return self._critic_value(va, vb, gate)

    def evaluate_actions(self, obs, actions, lstm_states, episode_starts):
        features = self.extract_features(obs)
        if self.share_features_extractor:
            pi_features = vf_features = features
        else:
            pi_features, vf_features = features
        gate = self._gate(obs)
        pa, pb, _ = self._dual_sequence(
            pi_features, lstm_states.pi, episode_starts, self.lstm_actor)
        va, vb, _ = self._dual_sequence(
            vf_features, lstm_states.vf, episode_starts, self.lstm_critic)
        distribution = self._dist_from_mean(self._actor_mean(pa, pb, gate))
        log_prob = distribution.log_prob(actions)
        values = self._critic_value(va, vb, gate)
        return values, log_prob, distribution.entropy()

    # -- auxiliary paths (distillation, BC anchor) ------------------

    def bptt_forward(self, feats: th.Tensor):
        """Whole-episode fused BPTT pass for distill_gru.train_student.

        ``feats`` is (T, B, obs) padded episodes starting at reset
        (zero initial hidden state is the truth). Returns (mu, value).
        """
        gate = self._gate(feats)
        out_a, _ = self.lstm_actor.core_a(feats)
        out_b, _ = self.lstm_actor.core_b(feats)
        mu = self._actor_mean(out_a, out_b, gate)
        v_a, _ = self.lstm_critic.core_a(feats)
        v_b, _ = self.lstm_critic.core_b(feats)
        value = self._critic_value(v_a, v_b, gate)
        return mu, value

    def bc_anchor_mean(self, th_obs: th.Tensor, th_h: th.Tensor,
                       detach_trunk: bool = False):
        """Policy mean at stored hidden states, for the BC anchor's
        auxiliary step (bc_anchor._bc_policy_mean delegates here).

        ``th_h`` is (B, 2*H) flat rows as stored by the anchor ring
        (row-major over the (2, B, H) state: core A then core B).
        With ``detach_trunk`` the feature extractor + both GRU cores
        run under no_grad and their outputs are detached, so the
        anchor loss only trains the per-core actor heads.
        """
        hidden = self.lstm_actor.hidden_size
        h = (th_obs.new_zeros((2, th_obs.shape[0], hidden))
             if th_h is None
             else th_h.reshape(th_obs.shape[0], 2, hidden)
             .transpose(0, 1).contiguous())
        starts = th.zeros(th_obs.shape[0], device=th_obs.device)
        gate = self._gate(th_obs)
        if detach_trunk:
            with th.no_grad():
                feats = self.extract_features(th_obs)
                if not self.share_features_extractor:
                    feats = feats[0]
                pa, pb, _ = self._dual_sequence(
                    feats, (h, th.zeros_like(h)), starts, self.lstm_actor)
            pa, pb = pa.detach(), pb.detach()
        else:
            feats = self.extract_features(th_obs)
            if not self.share_features_extractor:
                feats = feats[0]
            pa, pb, _ = self._dual_sequence(
                feats, (h, th.zeros_like(h)), starts, self.lstm_actor)
        return self._actor_mean(pa, pb, gate)


def is_recurrent_checkpoint(path: str | Path) -> bool:
    """True if the SB3 zip at ``path`` holds a recurrent policy."""
    from stable_baselines3.common.save_util import load_from_zip_file
    data, _, _ = load_from_zip_file(path, device="cpu", load_data=True)
    policy_class = data.get("policy_class")
    try:
        return bool(policy_class) and issubclass(
            policy_class, RecurrentActorCriticPolicy)
    except TypeError:
        return False


def load_checkpoint_auto(path: str | Path, device: str = "cpu", env=None):
    """Load an SB3 checkpoint as PPO or RecurrentPPO, whichever wrote it.

    Every eval/viewer code path that used to hardcode ``PPO.load``
    should go through this so GRU checkpoints just work.
    """
    if is_recurrent_checkpoint(path):
        from sb3_contrib import RecurrentPPO
        return RecurrentPPO.load(path, env=env, device=device)
    from stable_baselines3 import PPO
    return PPO.load(path, env=env, device=device)
