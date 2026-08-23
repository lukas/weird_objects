"""amp_discriminator.py — AMP track M1: discriminator + demo replay
buffer + style reward (rl_docs/AMP_LOCOMOTION.md §3.6, §5.2).

Consumes the motion library built by ``build_motion_library.py``
(``rl_move/sim/motion_library/teacher_v1.npz``'s ``obs_style`` field,
§4.5) directly — no new discriminator-feature definition here, reuses
the 60-dim (joint_pos_rel_neutral[18], joint_vel[18],
base_angular_velocity[3], projected_gravity[3],
foot_positions_rel_body[18]) vector the library already recorded.

Implements the standard AMP recipe (Peng et al. 2021, least-squares
GAN variant, matching the brief's §3.6/§5.2 requirements):

- ``MotionLibrary``: loads the npz, holds dataset mean/std for input
  normalization (§3.6 "normalize all discriminator inputs from the
  demonstration dataset"), and samples REAL (s_t, s_t+1) transitions
  — never crossing a clip boundary (``clip_starts``/``clip_lens``).
- ``AMPDiscriminator``: small MLP over the concatenated normalized
  transition (2 x 60-dim -> scalar logit), matching AsymActorCriticPolicy's
  plain-torch style (no framework dependency beyond torch).
- ``gradient_penalty``: R1-style penalty on REAL transitions only
  (standard AMP choice — penalizing only the real branch is what keeps
  the discriminator from saturating immediately, per the brief's
  explicit ask for "gradient penalty and discriminator regularization
  so the style reward does not saturate immediately").
- ``style_reward``: least-squares GAN style reward
  ``clip(1 - 0.25*(D(s,s')-1)^2, 0, 1)`` — bounded, does not blow up
  when the discriminator is confident the transition is fake (D->-1
  gives reward 0, not -inf, so a not-yet-competent policy still gets a
  well-behaved gradient instead of a cliff).
- ``discriminator_loss``: least-squares GAN discriminator loss (real
  wants D->1, fake wants D->-1) + gradient penalty term.

NOT YET WIRED into train_ppo_mjx.py's live reward loop — that requires
a live-loop change (blend style reward into the task reward each step,
run an online discriminator-update against the rollout buffer) that is
intentionally a separate, larger, more carefully-tested change, still
tracked in rl_docs/tracks/amp/STATUS.md.

The PREREQUISITE for that wiring — computing this SAME 60-dim
obs_style vector from the batched Warp/MJX env the live trainer
actually uses (not just the offline CPU npz generator) — is done:
see ``amp_features.py`` (cross-backend feature function; the MJX
per-env state mirror has no ``xquat`` field, so it needed an
xmat-based rewrite proven equivalent to this dataset's own extraction)
and ``test_amp_features_mjx.py`` (feeds a REAL MJX rollout's
transitions through this module's ``discriminator_loss`` for the first
time, replacing the synthetic noise/shuffle fakes below as the
"gradients flow, no instant saturation" check for that harder case).

CPU-only, plain torch (matches asym_policy.py's dependency footprint).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

DEFAULT_LIBRARY = Path(__file__).resolve().parent / "motion_library" / "teacher_v1.npz"


def apply_style_mask(x: np.ndarray, dims) -> np.ndarray:
    """Zero the named obs_style feature dims IN PLACE and return x.

    Empty ``dims`` is a strict no-op (bit-exact off path). Used to
    blind the discriminator to specific channels (e.g. dim 38 =
    base_angular_velocity z / yaw rate) on BOTH the real (library) and
    fake (policy) sides, so neither branch carries the masked signal.
    """
    if dims is not None and len(dims):
        x[..., np.asarray(dims, dtype=np.int64)] = 0.0
    return x


class MotionLibrary:
    """Loads a build_motion_library.py npz and samples real transitions.

    Transitions are (obs_style[i], obs_style[i+1]) pairs with i, i+1 in
    the SAME clip (never crossing a clip_starts/clip_lens boundary).
    Input normalization uses the dataset's own mean/std (§3.6), fit
    once at load time over every valid obs_style tick (not just
    transition starts) so the stats are not skewed by the small
    per-clip boundary exclusion.
    """

    def __init__(self, path: Path | str = DEFAULT_LIBRARY,
                 mask_dims=()):
        path = Path(path)
        d = np.load(path, allow_pickle=True)
        self.obs_style = np.asarray(d["obs_style"], dtype=np.float32)
        # Optional feature mask (default () = bit-exact legacy): zero
        # the named dims BEFORE fitting mean/std, so masked dims carry
        # mean 0 and hit the constant-dim std guard (std -> 1.0). The
        # same dims must be zeroed on the policy/fake side
        # (apply_style_mask in the AMP wrapper).
        self.mask_dims = tuple(sorted(int(i) for i in (mask_dims or ())))
        if self.mask_dims:
            if max(self.mask_dims) >= self.obs_style.shape[1] or min(self.mask_dims) < 0:
                raise ValueError(
                    f"mask_dims {self.mask_dims} out of range for "
                    f"obs_style dim {self.obs_style.shape[1]}")
            apply_style_mask(self.obs_style, self.mask_dims)
        self.clip_starts = np.asarray(d["clip_starts"], dtype=np.int64)
        self.clip_lens = np.asarray(d["clip_lens"], dtype=np.int64)
        self.feat_dim = self.obs_style.shape[1]
        # Global neutral pose (08-22 live-wiring convention): the
        # builder records per-clip neutrals (q_20260822T0900Z), but in
        # teacher_v1 every clip spawns from the SAME deterministic
        # stand init, so joint_position - joint_position_rel_neutral is
        # ONE pose (verified identical across all 15 clips to 1e-16).
        # The live trainer's env emits RAW joints (neutral=0) and the
        # AMPStyleVecWrapper subtracts THIS neutral, so both data paths
        # share the library's own convention. None when the npz
        # predates the raw-joint fields (fail loudly at use).
        self.neutral_pose = None
        if "joint_position" in d and "joint_position_rel_neutral" in d:
            jp = np.asarray(d["joint_position"], dtype=np.float64)
            rel = np.asarray(d["joint_position_rel_neutral"],
                             dtype=np.float64)
            neutrals = np.stack([jp[s] - rel[s] for s in self.clip_starts])
            spread = float(np.abs(neutrals - neutrals.mean(axis=0)).max())
            if spread > 1e-9:
                raise ValueError(
                    f"motion library {path} has per-clip neutrals that "
                    f"actually differ (max spread {spread:.3e}); the "
                    "live-wiring single-neutral convention no longer "
                    "holds — revisit q_20260822T0900Z before training")
            self.neutral_pose = neutrals[0].astype(np.float32)

        # valid transition start indices: every tick except the last
        # tick of each clip (that tick has no in-clip successor).
        starts = []
        for s0, ln in zip(self.clip_starts, self.clip_lens):
            if ln < 2:
                continue
            starts.extend(range(int(s0), int(s0) + int(ln) - 1))
        if not starts:
            raise ValueError(f"motion library {path} has no clip with >=2 ticks")
        self.transition_starts = np.asarray(starts, dtype=np.int64)

        mean = self.obs_style.mean(axis=0)
        std = self.obs_style.std(axis=0)
        std = np.where(std < 1e-6, 1.0, std)  # guard constant dims (e.g. always-zero feature)
        self.mean = mean.astype(np.float32)
        self.std = std.astype(np.float32)

    def normalize(self, x: np.ndarray | torch.Tensor):
        """Apply the dataset's fixed mean/std normalization."""
        if isinstance(x, torch.Tensor):
            mean = torch.as_tensor(self.mean, dtype=x.dtype, device=x.device)
            std = torch.as_tensor(self.std, dtype=x.dtype, device=x.device)
            return (x - mean) / std
        return (x - self.mean) / self.std

    def sample_real_transitions(self, n: int, rng: np.random.Generator | None = None):
        """Returns (s_t, s_t1) as float32 arrays, shape (n, feat_dim) each, RAW (not normalized)."""
        rng = rng or np.random.default_rng()
        idx = rng.choice(self.transition_starts, size=n, replace=True)
        s_t = self.obs_style[idx]
        s_t1 = self.obs_style[idx + 1]
        return s_t, s_t1

    def __len__(self):
        return len(self.transition_starts)


class AMPDiscriminator(nn.Module):
    """MLP discriminator over a normalized (s_t, s_t1) transition.

    Input: 2 * feat_dim (the two normalized 60-dim obs_style vectors,
    concatenated — "transitions across adjacent timesteps rather than
    isolated poses", §3.6). Output: a single unbounded logit; the
    least-squares GAN formulation (style_reward / discriminator_loss
    below) treats it as a score, not a probability, so no final
    sigmoid/tanh — matches the standard AMP implementation.
    """

    def __init__(self, feat_dim: int, hidden=(256, 128)):
        super().__init__()
        dims = [2 * feat_dim, *hidden, 1]
        layers = []
        for i in range(len(dims) - 2):
            layers.append(nn.Linear(dims[i], dims[i + 1]))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(dims[-2], dims[-1]))
        self.net = nn.Sequential(*layers)

    def forward(self, s_t: torch.Tensor, s_t1: torch.Tensor) -> torch.Tensor:
        x = torch.cat([s_t, s_t1], dim=-1)
        return self.net(x).squeeze(-1)


def gradient_penalty(disc: AMPDiscriminator, s_t: torch.Tensor, s_t1: torch.Tensor) -> torch.Tensor:
    """R1-style penalty: squared gradient norm of D on REAL transitions.

    Only the real branch is penalized (standard AMP choice) — this is
    the mechanism the brief asks for ("gradient penalty ... so the
    style reward does not saturate immediately"): without it the
    discriminator can drive real logits to +inf / fake logits to -inf
    in a handful of gradient steps and the style reward degenerates to
    a constant.
    """
    s_t = s_t.clone().requires_grad_(True)
    s_t1 = s_t1.clone().requires_grad_(True)
    d = disc(s_t, s_t1)
    grads = torch.autograd.grad(
        outputs=d.sum(), inputs=[s_t, s_t1], create_graph=True, retain_graph=True,
    )
    sq_norm = sum((g ** 2).sum(dim=-1) for g in grads)
    return sq_norm.mean()


def style_reward(disc: AMPDiscriminator, s_t: torch.Tensor, s_t1: torch.Tensor) -> torch.Tensor:
    """Least-squares GAN style reward, clipped to [0, 1] (Peng et al. 2021 eq.).

    r_style = clip(1 - 0.25 * (D(s,s') - 1)^2, 0, 1)

    D->1 (looks exactly like the demo data) gives reward 1; D->-1 or
    below/above the [0,2] logit window that keeps the bracket
    non-negative gives reward 0 — bounded both directions, no reward
    cliff for a policy that starts far from the demo manifold.
    """
    with torch.no_grad():
        d = disc(s_t, s_t1)
        r = 1.0 - 0.25 * (d - 1.0) ** 2
        return torch.clamp(r, min=0.0, max=1.0)


def discriminator_loss(disc: AMPDiscriminator, real_t, real_t1, fake_t, fake_t1,
                        gp_weight: float = 10.0):
    """Least-squares GAN discriminator loss + gradient penalty.

    L = 0.5 * E[(D(real)-1)^2] + 0.5 * E[(D(fake)+1)^2] + gp_weight * GP(real)

    Returns (total_loss, {"d_real_loss", "d_fake_loss", "gp", "d_real_mean", "d_fake_mean"}).
    """
    d_real = disc(real_t, real_t1)
    d_fake = disc(fake_t, fake_t1)
    d_real_loss = 0.5 * ((d_real - 1.0) ** 2).mean()
    d_fake_loss = 0.5 * ((d_fake + 1.0) ** 2).mean()
    gp = gradient_penalty(disc, real_t, real_t1)
    total = d_real_loss + d_fake_loss + gp_weight * gp
    stats = {
        "d_real_loss": float(d_real_loss.detach()),
        "d_fake_loss": float(d_fake_loss.detach()),
        "gp": float(gp.detach()),
        "d_real_mean": float(d_real.mean().detach()),
        "d_fake_mean": float(d_fake.mean().detach()),
    }
    return total, stats


def train_smoke(library: MotionLibrary, steps: int = 200, batch: int = 256,
                 gp_weight: float = 10.0, lr: float = 3e-4, seed: int = 0,
                 fake_kind: str = "noise"):
    """Standalone training loop (M1 item 4's "gradients flow / no instant
    saturation" smoke check) — trains the discriminator to separate
    REAL motion-library transitions from a synthetic "fake" distribution.

    fake_kind:
      "noise"    — real inputs' own per-dim mean/std, resampled i.i.d.
                   (destroys joint-to-joint and temporal structure);
      "shuffled" — real (s_t, s_t1) pairs with s_t1 drawn from an
                   unrelated random tick (breaks ONLY the temporal
                   adjacency, keeping single-tick statistics intact —
                   the harder discrimination task).

    Returns (disc, history) where history is a list of per-step stats
    dicts from discriminator_loss, for callers to assert monotone
    improvement / no NaN.
    """
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    disc = AMPDiscriminator(library.feat_dim)
    opt = torch.optim.Adam(disc.parameters(), lr=lr)
    history = []
    for _ in range(steps):
        s_t, s_t1 = library.sample_real_transitions(batch, rng)
        s_t = torch.as_tensor(library.normalize(s_t))
        s_t1 = torch.as_tensor(library.normalize(s_t1))

        if fake_kind == "noise":
            f_t = torch.randn(batch, library.feat_dim)
            f_t1 = torch.randn(batch, library.feat_dim)
        elif fake_kind == "shuffled":
            f_t, _ = library.sample_real_transitions(batch, rng)
            _, f_t1 = library.sample_real_transitions(batch, rng)
            f_t = torch.as_tensor(library.normalize(f_t))
            f_t1 = torch.as_tensor(library.normalize(f_t1))
        else:
            raise ValueError(f"unknown fake_kind {fake_kind!r}")

        loss, stats = discriminator_loss(disc, s_t, s_t1, f_t, f_t1, gp_weight=gp_weight)
        opt.zero_grad()
        loss.backward()
        opt.step()
        stats["loss"] = float(loss.detach())
        history.append(stats)
    return disc, history


def _main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--library", default=str(DEFAULT_LIBRARY))
    ap.add_argument("--steps", type=int, default=500)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--gp-weight", type=float, default=10.0)
    ap.add_argument("--fake-kind", default="shuffled", choices=["noise", "shuffled"])
    ap.add_argument("--out", default=None, help="optional path to save the trained discriminator state_dict")
    args = ap.parse_args()

    lib = MotionLibrary(args.library)
    print(f"[amp_discriminator] library={args.library} feat_dim={lib.feat_dim} "
          f"n_transitions={len(lib)}")
    disc, hist = train_smoke(lib, steps=args.steps, batch=args.batch,
                              gp_weight=args.gp_weight, fake_kind=args.fake_kind)
    first, last = hist[0], hist[-1]
    print(f"[amp_discriminator] step 0: loss={first['loss']:.4f} "
          f"d_real={first['d_real_mean']:.3f} d_fake={first['d_fake_mean']:.3f} gp={first['gp']:.4f}")
    print(f"[amp_discriminator] step {len(hist)-1}: loss={last['loss']:.4f} "
          f"d_real={last['d_real_mean']:.3f} d_fake={last['d_fake_mean']:.3f} gp={last['gp']:.4f}")
    if args.out:
        torch.save(disc.state_dict(), args.out)
        print(f"[amp_discriminator] saved -> {args.out}")


if __name__ == "__main__":
    _main()
