"""amp_style_vec.py — AMP style-reward VecEnv wrapper + online
discriminator training (rl_docs/AMP_LOCOMOTION.md §5.2/§7, M1 item 3's
"live reward-loop wiring", 2026-08-22).

Sits between the batched MJX/Warp vec env and VecMonitor in
``train_ppo_mjx.py`` when ``--amp-style-weight > 0`` (default 0 = the
wrapper is never constructed, bit-exact legacy). Each ``step_wait``:

1. reads the per-env 60-dim ``info["amp_obs_style"]`` the env emits
   when ``goal.amp_style_obs=1`` (``sim_env._post_step``; RAW joint
   angles in dims 0..17 — this wrapper subtracts the motion library's
   OWN neutral pose so the feature convention is defined in exactly
   one place, the library);
2. pairs it with the previous tick's vector into an AMP transition
   (s_t, s_t1), masked across episode boundaries: on ``done`` the
   MJX vec env pops a pooled reset in the SAME step (the returned obs
   is already the new episode's), so the pair ending in the terminal
   state is kept and the buffer is cleared so the first tick of the
   next episode never pairs with the old episode's last state;
3. computes the least-squares style reward (bounded [0, 1],
   ``amp_discriminator.style_reward``) on valid pairs and blends:
   ``r = task_weight * r_task + style_weight * r_style``
   (brief §5.2 mixtures: 0.7/0.3, 0.5/0.5, 0.3/0.7). Ticks without a
   valid pair (first tick of each episode) contribute style 0;
4. pushes valid pairs into a ring replay of POLICY transitions (the
   discriminator's "fake" side, brief §7 "replay_size").

``train_discriminator()`` (called by the trainer's rollout-end
callback) draws fake minibatches from that replay and real minibatches
from the motion library, applies ``discriminator_loss`` (least-squares
GAN + R1 gradient penalty on real) and returns mean stats for W&B.

CPU/plain-torch, same dependency footprint as amp_discriminator.py.
The discriminator state (net + optimizer + step count) round-trips via
``save()``/``load()`` so continuations don't restart it from scratch.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from stable_baselines3.common.vec_env import VecEnvWrapper

from .amp_discriminator import (AMPDiscriminator, MotionLibrary,
                                discriminator_loss, style_reward)


class _TransitionRing:
    """Fixed-capacity FIFO of (s_t, s_t1) float32 rows."""

    def __init__(self, capacity: int, feat_dim: int):
        self.capacity = int(capacity)
        self.s_t = np.zeros((self.capacity, feat_dim), dtype=np.float32)
        self.s_t1 = np.zeros((self.capacity, feat_dim), dtype=np.float32)
        self._n = 0          # rows ever pushed (saturates at capacity)
        self._i = 0          # next write index

    def __len__(self):
        return min(self._n, self.capacity)

    def push(self, s_t: np.ndarray, s_t1: np.ndarray) -> None:
        k = len(s_t)
        if k == 0:
            return
        if k >= self.capacity:            # degenerate: keep the tail
            s_t, s_t1, k = s_t[-self.capacity:], s_t1[-self.capacity:], \
                self.capacity
        end = self._i + k
        if end <= self.capacity:
            self.s_t[self._i:end] = s_t
            self.s_t1[self._i:end] = s_t1
        else:
            first = self.capacity - self._i
            self.s_t[self._i:] = s_t[:first]
            self.s_t1[self._i:] = s_t1[:first]
            self.s_t[:end - self.capacity] = s_t[first:]
            self.s_t1[:end - self.capacity] = s_t1[first:]
        self._i = end % self.capacity
        self._n = min(self._n + k, self.capacity)

    def sample(self, n: int, rng: np.random.Generator):
        m = len(self)
        idx = rng.integers(0, m, size=n)
        return self.s_t[idx], self.s_t1[idx]


class AMPStyleVecWrapper(VecEnvWrapper):
    """Blend an AMP discriminator style reward into the env reward.

    ``style_weight <= 0`` is rejected: the caller must simply not
    construct the wrapper (that is the bit-exact off path).
    """

    def __init__(self, venv, *, style_weight: float, task_weight: float,
                 motion_lib: str | Path | None = None,
                 replay_size: int = 500_000, disc_lr: float = 3e-4,
                 gp_weight: float = 10.0, seed: int = 0,
                 disc_init: str | Path | None = None):
        super().__init__(venv)
        if style_weight <= 0.0:
            raise ValueError("AMPStyleVecWrapper needs style_weight > 0; "
                             "for style off, do not wrap at all")
        self.style_weight = float(style_weight)
        self.task_weight = float(task_weight)
        self.lib = (MotionLibrary(motion_lib) if motion_lib
                    else MotionLibrary())
        if self.lib.neutral_pose is None:
            raise ValueError(
                "motion library lacks joint_position/"
                "joint_position_rel_neutral — cannot derive the neutral "
                "pose the live obs_style convention requires")
        self.gp_weight = float(gp_weight)
        torch.manual_seed(seed)
        self.disc = AMPDiscriminator(self.lib.feat_dim)
        self.opt = torch.optim.Adam(self.disc.parameters(), lr=disc_lr)
        self.disc_updates = 0
        if disc_init:
            self.load(disc_init)
        self.ring = _TransitionRing(replay_size, self.lib.feat_dim)
        self.rng = np.random.default_rng(seed)
        n = self.num_envs
        self._prev = np.zeros((n, self.lib.feat_dim), dtype=np.float32)
        self._prev_valid = np.zeros(n, dtype=bool)
        # rollout-window telemetry (popped by the trainer callback)
        self._stat_style_sum = 0.0
        self._stat_style_n = 0
        self._stat_pair_n = 0

    # ------------------------------------------------------------- env
    def reset(self):
        obs = self.venv.reset()
        self._prev_valid[:] = False
        return obs

    def _styles_from_infos(self, infos) -> np.ndarray:
        rows = []
        for i, info in enumerate(infos):
            v = info.get("amp_obs_style")
            if v is None:
                raise RuntimeError(
                    "info['amp_obs_style'] missing (env %d) — the env "
                    "must run with goal.amp_style_obs=1 when the AMP "
                    "style wrapper is active" % i)
            rows.append(v)
        s = np.asarray(rows, dtype=np.float32)
        # library-neutral convention: env emits RAW joints in dims 0..17
        s[:, :self.lib.neutral_pose.shape[0]] -= self.lib.neutral_pose
        return s

    def step_wait(self):
        obs, rews, dones, infos = self.venv.step_wait()
        cur = self._styles_from_infos(infos)
        valid = self._prev_valid.copy()
        style = np.zeros(len(rews), dtype=np.float32)
        if valid.any():
            with torch.no_grad():
                s_t = torch.as_tensor(self.lib.normalize(self._prev[valid]))
                s_t1 = torch.as_tensor(self.lib.normalize(cur[valid]))
                style[valid] = style_reward(
                    self.disc, s_t, s_t1).numpy().astype(np.float32)
            self.ring.push(self._prev[valid], cur[valid])
            self._stat_style_sum += float(style[valid].sum())
            self._stat_style_n += int(valid.sum())
            self._stat_pair_n += int(valid.sum())
        blended = (self.task_weight * np.asarray(rews, dtype=np.float32)
                   + self.style_weight * style)
        # expose per-env style reward for _Track's generic scalar sweep
        for i, info in enumerate(infos):
            info["reward_amp_style"] = float(style[i] * self.style_weight)
        self._prev = cur
        # On done, info["amp_obs_style"] is the TERMINAL state (the env
        # emitted it in _post_step, BEFORE the vec env popped the pooled
        # reset into obs), so the pair consumed above legitimately ends
        # in the terminal state — but pairing that terminal state with
        # the NEW episode's first tick would be a fake discontinuity:
        # invalidate it. Cost: the style reward skips exactly one tick
        # per episode (its first).
        self._prev_valid = ~np.asarray(dones, dtype=bool)
        return obs, blended, dones, infos

    # ---------------------------------------------------- discriminator
    def train_discriminator(self, steps: int, batch: int) -> dict | None:
        """K minibatch updates: real from the library, fake from the
        policy replay. Returns mean stats (None while replay too small
        to fill one batch — e.g. before the first rollout finishes)."""
        if len(self.ring) < batch:
            return None
        agg: dict[str, float] = {}
        for _ in range(int(steps)):
            real_t, real_t1 = self.lib.sample_real_transitions(
                batch, self.rng)
            fake_t, fake_t1 = self.ring.sample(batch, self.rng)
            loss, stats = discriminator_loss(
                self.disc,
                torch.as_tensor(self.lib.normalize(real_t)),
                torch.as_tensor(self.lib.normalize(real_t1)),
                torch.as_tensor(self.lib.normalize(fake_t)),
                torch.as_tensor(self.lib.normalize(fake_t1)),
                gp_weight=self.gp_weight)
            self.opt.zero_grad()
            loss.backward()
            self.opt.step()
            self.disc_updates += 1
            stats["loss"] = float(loss.detach())
            for k, v in stats.items():
                agg[k] = agg.get(k, 0.0) + v / float(steps)
        agg["replay_len"] = float(len(self.ring))
        agg["disc_updates"] = float(self.disc_updates)
        return agg

    def pop_rollout_stats(self) -> dict:
        out = {
            "style_reward_mean": (self._stat_style_sum
                                  / max(1, self._stat_style_n)),
            "pairs": float(self._stat_pair_n),
        }
        self._stat_style_sum = 0.0
        self._stat_style_n = 0
        self._stat_pair_n = 0
        return out

    # ------------------------------------------------------ persistence
    def save(self, path: str | Path) -> None:
        torch.save({"disc": self.disc.state_dict(),
                    "opt": self.opt.state_dict(),
                    "disc_updates": self.disc_updates,
                    "feat_dim": self.lib.feat_dim}, str(path))

    def load(self, path: str | Path) -> None:
        d = torch.load(str(path), map_location="cpu", weights_only=False)
        if d.get("feat_dim") != self.lib.feat_dim:
            raise ValueError(
                f"disc checkpoint feat_dim {d.get('feat_dim')} != "
                f"library {self.lib.feat_dim}")
        self.disc.load_state_dict(d["disc"])
        self.opt.load_state_dict(d["opt"])
        self.disc_updates = int(d.get("disc_updates", 0))
