"""Asymmetric actor-critic policy (literature-review priority #1).

The walk env's observation carries the MEASURED body velocity in its
last two dims — privileged sim state that hardware does not have. The
plain deployable-obs baseline (goal.walk_obs_body_vel=0, cw-walk-nv)
blinds BOTH networks. This policy blinds only the ACTOR: a fixed
zero-mask over the privileged dims is applied on the actor path of the
MlpExtractor, while the critic (value net) sees the full observation.
The critic vanishes at deployment, so the saved policy's action path is
hardware-legal by construction — `predict()` gives identical actions
whatever the privileged dims contain.

Warm-start compatibility: the mask is a non-persistent buffer, so the
state_dict keys are IDENTICAL to the stock "MlpPolicy" with the same
net_arch — a champion checkpoint's weights transplant 1:1
(train_ppo_sim --asym-critic handles this; optimizer state is fresh).
Checkpoints saved from this policy load normally via PPO.load because
policy_kwargs (incl. privileged_idx) are stored in the model file.
"""
from __future__ import annotations

import torch

from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import MlpExtractor


class MaskedMlpExtractor(MlpExtractor):
    """MlpExtractor whose ACTOR path sees masked features.

    forward()/forward_actor() multiply features by the mask before the
    policy net; forward_critic() is untouched. The mask is registered
    as a non-persistent buffer: no state_dict entry, moves with .to().
    """

    def __init__(self, feature_dim, net_arch, activation_fn,
                 device="auto", actor_mask: torch.Tensor | None = None):
        super().__init__(feature_dim, net_arch, activation_fn, device)
        if actor_mask is None:
            actor_mask = torch.ones(feature_dim)
        self.register_buffer("actor_mask",
                             actor_mask.to(torch.float32).clone(),
                             persistent=False)

    def forward(self, features):
        return (self.policy_net(features * self.actor_mask),
                self.value_net(features))

    def forward_actor(self, features):
        return self.policy_net(features * self.actor_mask)

    # forward_critic inherited: full, unmasked features.


class AsymActorCriticPolicy(ActorCriticPolicy):
    """ActorCriticPolicy with privileged obs dims hidden from the actor.

    Extra policy_kwargs: ``privileged_idx`` — iterable of obs indices
    (negative allowed) zeroed on the actor path only.
    """

    def __init__(self, observation_space, action_space, lr_schedule,
                 *args, privileged_idx=(), **kwargs):
        self.privileged_idx = tuple(int(i) for i in privileged_idx)
        super().__init__(observation_space, action_space, lr_schedule,
                         *args, **kwargs)

    def _build_mlp_extractor(self) -> None:
        n = int(self.features_dim)
        mask = torch.ones(n, dtype=torch.float32)
        for i in self.privileged_idx:
            mask[i % n] = 0.0
        self.mlp_extractor = MaskedMlpExtractor(
            self.features_dim, net_arch=self.net_arch,
            activation_fn=self.activation_fn, device=self.device,
            actor_mask=mask)
