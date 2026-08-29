"""Decentralized per-leg actor policy (walkcurr rung-1, operator-registered
literature: Schilling et al., IROS 2020, arXiv:2005.11164).

Six independent per-leg actor modules (default 2x64) replace the single
centralized MLP actor: the action for leg i is a function ONLY of that
leg's local state (its 3 joint positions, 3 joint velocities, 3
previous-action dims) plus the shared body/command state (tilt, gyro,
goal/command tail).  No cross-leg connections exist anywhere on the
action path — coordination has to emerge through the body/environment,
which is the paper's central claim (their centralized baseline "got
stuck more often in local minima", exactly the walkcurr campaign's
15-classes-frozen signature; the decentralized arm reached the
centralized arm's final performance in less than half the training
time and out-performed it at convergence, p=.011).

The critic stays CENTRALIZED (full observation) — a standard
centralized-training/decentralized-execution split; the critic
vanishes at deployment so the saved action path is per-leg by
construction.

Obs layout contract (joint-family tasks, ``obs.history_frames == 1``):

    [ 0:18]  q_rel        (leg-major: leg i owns dims 3i..3i+2)
    [18:36]  qd           (same order)
    [36:38]  tilt
    [38:41]  gyro
    [41:59]  prev_action  (same order)
    [59: W]  goal/command tail (+ walk vel feedback etc.) — shared

``joint_walk_leg_slices(obs_width)`` computes the index sets; the
trainer fills them into policy_kwargs after the venv reports its obs
width, mirroring the asym-critic post-venv pattern.  policy_kwargs
(incl. the index lists) are stored in the model file, so PPO.load
reconstructs the exact same wiring for eval.
"""
from __future__ import annotations

import numpy as np
import torch
from torch import nn

from stable_baselines3.common.policies import ActorCriticPolicy

N_LEGS = 6
ACT_PER_LEG = 3


def joint_walk_leg_slices(obs_width: int
                          ) -> tuple[list[list[int]], list[int]]:
    """(leg_obs_idx, shared_obs_idx) for the joint-family obs layout.

    Works for any tail width (obs 68/72/74 variants): everything from
    dim 59 on is shared command/feedback state.
    """
    obs_width = int(obs_width)
    if obs_width < 59:
        raise ValueError(
            f"obs width {obs_width} is not a joint-family layout "
            "(need >= 59 dims: 18q+18qd+2tilt+3gyro+18prev)")
    legs = []
    for i in range(N_LEGS):
        legs.append([3 * i, 3 * i + 1, 3 * i + 2,
                     18 + 3 * i, 18 + 3 * i + 1, 18 + 3 * i + 2,
                     41 + 3 * i, 41 + 3 * i + 1, 41 + 3 * i + 2])
    shared = list(range(36, 41)) + list(range(59, obs_width))
    return legs, shared


def _mlp(in_dim: int, hidden: tuple[int, ...],
         activation_fn: type[nn.Module]) -> nn.Sequential:
    layers: list[nn.Module] = []
    last = in_dim
    for h in hidden:
        layers += [nn.Linear(last, int(h)), activation_fn()]
        last = int(h)
    return nn.Sequential(*layers)


class _PerLegHead(nn.Module):
    """Block-diagonal action head: leg i's latent chunk -> 3 action
    dims, leg-major concat (matches the joint/action ordering)."""

    def __init__(self, n_legs: int, latent_per_leg: int,
                 act_per_leg: int = ACT_PER_LEG):
        super().__init__()
        self.n_legs = int(n_legs)
        self.latent_per_leg = int(latent_per_leg)
        self.heads = nn.ModuleList(
            [nn.Linear(latent_per_leg, act_per_leg)
             for _ in range(n_legs)])

    def forward(self, latent: torch.Tensor) -> torch.Tensor:
        chunks = torch.split(latent, self.latent_per_leg, dim=-1)
        return torch.cat([h(c) for h, c in zip(self.heads, chunks)],
                         dim=-1)


class _DecLegExtractor(nn.Module):
    """MlpExtractor stand-in: per-leg actor towers + centralized
    value tower.  forward_actor output = concat of per-leg latents
    (leg-major), consumed by the block-diagonal _PerLegHead."""

    def __init__(self, feature_dim: int,
                 leg_obs_idx: list[list[int]],
                 shared_obs_idx: list[int],
                 leg_hidden: tuple[int, ...],
                 value_arch: tuple[int, ...],
                 activation_fn: type[nn.Module],
                 device: torch.device | str = "auto"):
        super().__init__()
        if not leg_obs_idx:
            raise ValueError("leg_obs_idx is empty — the trainer must "
                             "fill it (joint_walk_leg_slices)")
        n_local = len(leg_obs_idx[0])
        if any(len(ix) != n_local for ix in leg_obs_idx):
            raise ValueError("ragged leg_obs_idx")
        flat = [j for ix in leg_obs_idx for j in ix]
        if len(set(flat)) != len(flat):
            raise ValueError("leg_obs_idx sets overlap between legs")
        self.latent_dim_pi = int(leg_hidden[-1]) * len(leg_obs_idx)
        self.latent_dim_vf = int(value_arch[-1])
        in_dim = len(shared_obs_idx) + n_local
        self.leg_nets = nn.ModuleList(
            [_mlp(in_dim, tuple(leg_hidden), activation_fn)
             for _ in leg_obs_idx])
        self.value_net = _mlp(feature_dim, tuple(value_arch),
                              activation_fn)
        # Non-persistent index buffers: no state_dict entries (the
        # lists live in policy_kwargs), move with .to(device).
        for i, ix in enumerate(leg_obs_idx):
            self.register_buffer(
                f"_leg_idx_{i}",
                torch.as_tensor(list(ix), dtype=torch.long),
                persistent=False)
        self.register_buffer(
            "_shared_idx",
            torch.as_tensor(list(shared_obs_idx), dtype=torch.long),
            persistent=False)
        self.n_legs = len(leg_obs_idx)

    def forward(self, features: torch.Tensor
                ) -> tuple[torch.Tensor, torch.Tensor]:
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, features: torch.Tensor) -> torch.Tensor:
        shared = torch.index_select(features, -1, self._shared_idx)
        outs = []
        for i in range(self.n_legs):
            idx = getattr(self, f"_leg_idx_{i}")
            local = torch.index_select(features, -1, idx)
            outs.append(self.leg_nets[i](
                torch.cat([shared, local], dim=-1)))
        return torch.cat(outs, dim=-1)

    def forward_critic(self, features: torch.Tensor) -> torch.Tensor:
        return self.value_net(features)


class DecLegActorCriticPolicy(ActorCriticPolicy):
    """ActorCriticPolicy with a strictly decentralized per-leg actor.

    Extra policy_kwargs:
      ``leg_obs_idx``    six lists of obs indices (leg-local dims)
      ``shared_obs_idx`` obs indices every module sees (body+command)
      ``leg_hidden``     per-leg tower widths, default (64, 64)

    net_arch's vf side (or the plain list) is the centralized critic
    tower; the pi side is ignored (the per-leg towers replace it).
    """

    def __init__(self, observation_space, action_space, lr_schedule,
                 *args, leg_obs_idx=(), shared_obs_idx=(),
                 leg_hidden=(64, 64), **kwargs):
        self.leg_obs_idx = [list(map(int, ix)) for ix in leg_obs_idx]
        self.shared_obs_idx = list(map(int, shared_obs_idx))
        self.leg_hidden = tuple(int(h) for h in leg_hidden)
        super().__init__(observation_space, action_space, lr_schedule,
                         *args, **kwargs)

    def _value_arch(self) -> tuple[int, ...]:
        na = self.net_arch
        if isinstance(na, dict):
            vf = na.get("vf", [64, 64])
        elif na and isinstance(na[0], dict):   # legacy [dict] form
            vf = na[0].get("vf", [64, 64])
        else:
            vf = list(na) if na else [64, 64]
        return tuple(int(x) for x in vf)

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = _DecLegExtractor(
            self.features_dim, self.leg_obs_idx, self.shared_obs_idx,
            self.leg_hidden, self._value_arch(), self.activation_fn,
            device=self.device)

    def _build(self, lr_schedule) -> None:
        super()._build(lr_schedule)
        act_dim = int(np.prod(self.action_space.shape))
        n_legs = len(self.leg_obs_idx)
        if act_dim != n_legs * ACT_PER_LEG:
            raise ValueError(
                f"action dim {act_dim} != {n_legs} legs x "
                f"{ACT_PER_LEG} — decleg needs the raw joint action "
                "space (joint-family tasks)")
        # Replace the dense Linear(latent_pi, 18) — which would
        # re-couple the legs — with the block-diagonal per-leg head,
        # then rebuild the optimizer (super()._build registered the
        # old head's params).
        self.action_net = _PerLegHead(
            n_legs, int(self.leg_hidden[-1]), ACT_PER_LEG)
        if self.ortho_init:
            for h in self.action_net.heads:
                h.apply(lambda m: self.init_weights(m, gain=0.01))
        self.action_net = self.action_net.to(self.device)
        self.optimizer = self.optimizer_class(
            self.parameters(), lr=lr_schedule(1),
            **self.optimizer_kwargs)
