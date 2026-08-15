"""Causal-transformer actor-critic policy over the env-side frame stack.

Temporal-arch rung: the finite-history MLP (obs.history_frames=16,
cw-arch-hist16-r7/-s1) is the only from-scratch walk architecture that
has passed its gate, while every recurrent (GRU) from-scratch arm
collapsed to leg-sacrifice/paddle exploits. Hypothesis (operator-
sanctioned, 08-15): explicit attention over the same 16-frame window
is a genuinely different memory mechanism — the policy can *inspect*
"the command changed 8 ticks ago" instead of compressing it into
either a recurrent state (GRU) or fixed first-layer weights (MLP).

Design (small causal transformer, online PPO — NOT a Decision
Transformer):

- The env already stacks the last K single-tick frames NEWEST-FIRST
  into one flat obs (sim_env._final_obs); each frame carries the full
  proprio + prev-action + command/goal tail, so tokens naturally
  contain observation, previous action, vx/vy command and mode extras.
- ``FrameStackTransformerExtractor`` reshapes (K*W,) -> (K, W), flips
  to oldest-first so the causal mask reads left-to-right in time,
  embeds each frame to d_model, adds a learned positional embedding,
  runs pre-LN encoder layers under a causal mask, and returns the
  NEWEST token only. Stock (non-recurrent) PPO on top — no
  RecurrentPPO machinery, no hidden state to thread, minibatching and
  GAE identical to the hist16 MLP baseline.
- ``TransformerActorCriticPolicy`` defaults to SEPARATE actor and
  critic transformers (share_features_extractor=False); net_arch
  builds the usual sb3 heads on the newest-token features.

Checkpoints ride the normal SB3 zip path (the zip pickles this class;
``gru_policy.load_checkpoint_auto`` returns a plain PPO for them), so
eval_checkpoint / pod_eval / play / drive_policy work unchanged as
long as this module is importable.

Interplay with the rest of the stack:

- mirror.py's MirrorPPO only calls ``policy.get_distribution(obs)`` —
  composes unchanged.
- sb3's ortho_init sweeps every nn.Linear inside feature extractors
  with gain sqrt(2); that is the wrong init for a transformer's
  residual stream, so the policy restores standard PyTorch/xavier
  init on the extractor(s) after construction (heads keep sb3's
  usual ortho init).
"""
from __future__ import annotations

import numpy as np
import torch as th
from torch import nn

from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class FrameStackTransformerExtractor(BaseFeaturesExtractor):
    """(B, K*W) flat frame-stack obs -> (B, d_model) newest-token features.

    ``n_frames`` must match the env's obs.history_frames (the flat obs
    width must divide evenly into K frames — enforced here so an obs
    layout drift fails at construction, not silently).
    """

    def __init__(self, observation_space, n_frames: int,
                 d_model: int = 128, n_layers: int = 2, n_heads: int = 4,
                 ff_dim: int = 256):
        obs_dim = int(np.prod(observation_space.shape))
        if n_frames < 2:
            raise ValueError(
                f"n_frames={n_frames}: a transformer over a single frame "
                "has no history to attend over — run obs.history_frames>=2")
        if obs_dim % n_frames:
            raise ValueError(
                f"obs width {obs_dim} is not a multiple of n_frames "
                f"{n_frames} — obs.history_frames and --tf-* flags disagree "
                "with the env's obs layout")
        super().__init__(observation_space, features_dim=d_model)
        self.n_frames = int(n_frames)
        self.frame_width = obs_dim // self.n_frames
        self.embed = nn.Linear(self.frame_width, d_model)
        self.pos_embed = nn.Parameter(th.zeros(1, self.n_frames, d_model))
        layer = nn.TransformerEncoderLayer(
            d_model, n_heads, dim_feedforward=ff_dim, dropout=0.0,
            activation="gelu", batch_first=True, norm_first=True)
        self.encoder = nn.TransformerEncoder(
            layer, n_layers, enable_nested_tensor=False)
        # Pre-LN blocks leave the residual stream unnormalized; norm the
        # token we actually read (standard GPT-style final norm).
        self.out_norm = nn.LayerNorm(d_model)
        mask = th.triu(
            th.full((self.n_frames, self.n_frames), float("-inf")),
            diagonal=1)
        self.register_buffer("causal_mask", mask, persistent=False)
        self.reset_transformer_parameters()

    def reset_transformer_parameters(self) -> None:
        """Standard transformer init (xavier attention, default Linear/
        LayerNorm resets, small-normal positional embedding). Called at
        construction AND by TransformerActorCriticPolicy after sb3's
        ortho_init sweep has clobbered the Linears."""
        for m in self.modules():
            if isinstance(m, (nn.Linear, nn.LayerNorm)):
                m.reset_parameters()
            elif isinstance(m, nn.MultiheadAttention):
                m._reset_parameters()
        nn.init.normal_(self.pos_embed, std=0.02)

    def tokens(self, observations: th.Tensor) -> th.Tensor:
        """All K token outputs, time-ordered oldest->newest (B, K, D).

        Exposed for tests (causality checks need intermediate tokens);
        the policy path only reads ``forward`` = newest token.
        """
        b = observations.shape[0]
        # Env frames are stacked NEWEST-FIRST (frame 0 = current tick);
        # flip to oldest-first so position K-1 is "now" and the causal
        # mask reads left-to-right in time.
        x = observations.view(b, self.n_frames, self.frame_width).flip(1)
        x = self.embed(x) + self.pos_embed
        return self.encoder(x, mask=self.causal_mask)

    def forward(self, observations: th.Tensor) -> th.Tensor:
        return self.out_norm(self.tokens(observations)[:, -1])


class TransformerActorCriticPolicy(ActorCriticPolicy):
    """Stock sb3 ActorCriticPolicy with a causal-transformer trunk.

    Constructor keeps the transformer geometry as flat kwargs
    (n_frames, d_model, n_layers, n_heads, ff_dim) so they ride the
    checkpoint's policy_kwargs and PPO.load reconstructs the exact
    architecture. Actor and critic get SEPARATE transformers by
    default (share_features_extractor=False).
    """

    def __init__(self, observation_space, action_space, lr_schedule,
                 n_frames: int = 16, d_model: int = 128, n_layers: int = 2,
                 n_heads: int = 4, ff_dim: int = 256,
                 share_features_extractor: bool = False, **kwargs):
        tf_kwargs = dict(n_frames=n_frames, d_model=d_model,
                         n_layers=n_layers, n_heads=n_heads, ff_dim=ff_dim)
        super().__init__(
            observation_space, action_space, lr_schedule,
            share_features_extractor=share_features_extractor,
            features_extractor_class=FrameStackTransformerExtractor,
            features_extractor_kwargs=tf_kwargs,
            **kwargs)
        # Undo sb3's orthogonal-init sweep inside the transformer(s);
        # dedupe by id in case the extractor is shared.
        extractors = {}
        for name in ("features_extractor", "pi_features_extractor",
                     "vf_features_extractor"):
            fe = getattr(self, name, None)
            if isinstance(fe, FrameStackTransformerExtractor):
                extractors[id(fe)] = fe
        for fe in extractors.values():
            fe.reset_transformer_parameters()
