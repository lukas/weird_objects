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
