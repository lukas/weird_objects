"""rnd_vec.py — Random Network Distillation (RND) intrinsic-reward
VecEnv wrapper (Burda et al. 2018), built for the `walkcurr` track's
pre-registered fallback ("RND state-novelty stays the fallback if
rung-0 also freezes" — `rl_docs/tracks/walkcurr/STATUS.md`).

Both rung-0 swing-income arms (`swing3`, `swing9`) certified 0/6 on
the C-env det gate: swing3 converges to a static one-leg-planted
tripod-lean pose, swing9 converges to a static all-legs-airborne
hover. Neither is "frozen" in the literal zero-clip_fraction optimizer
-crush sense from rung-1 (both keep clip_fraction healthy and rising)
— the optimizer is fine, but PPO settles on a STATIC pose that collects
whatever income/charge balance is on offer without ever discovering
rhythmic 6-leg cycling. RND adds an exploration bonus that pays for
VISITING STATES THE POLICY HASN'T SEEN BEFORE (measured as prediction
error against a frozen random target network), which should make a
static held pose actively unprofitable — it stops generating fresh
states, so its intrinsic income decays to ~0 while any policy that
keeps moving keeps collecting it.

Mechanism (standard RND, matching the reference recipe used everywhere
else in this file's sibling `amp_style_vec.py`: sits between the
batched vec env and VecMonitor, blends into the env reward the
trainer's PPO actually sees, trains itself once per rollout via a
trainer callback):

1. A frozen (never trained) random-init target MLP over the raw obs.
2. A trainable predictor MLP, same architecture, different init.
3. Every tick: normalize obs with a running mean/std (Welford), run
   both nets (no grad), intrinsic = mean squared prediction error per
   env. Intrinsic reward is itself normalized by a running std (Burda
   et al. — otherwise its scale drifts as the predictor learns) before
   being added: ``r = r_env + rnd_coef * intrinsic / (std_intrinsic + eps)``.
4. Every rollout end (trainer callback, mirrors `_AMPDiscCb`): sample
   a batch of buffered normalized obs, take a few predictor gradient
   steps toward the frozen target's output (MSE loss) — this is what
   makes prediction error DECAY on repeatedly-visited states (the
   "novelty" signal) while staying high on states rarely seen.

``rnd_coef <= 0`` is bit-exact off: the caller must not construct the
wrapper at all (mirrors ``AMPStyleVecWrapper``'s own contract).
"""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
from stable_baselines3.common.vec_env import VecEnvWrapper


class _RunningMeanStd:
    """Welford running mean/var over the last axis, batched updates."""

    def __init__(self, shape, eps: float = 1e-4):
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = float(eps)

    def update(self, x: np.ndarray) -> None:
        batch_mean = x.mean(axis=0)
        batch_var = x.var(axis=0)
        batch_count = x.shape[0]
        delta = batch_mean - self.mean
        tot_count = self.count + batch_count
        new_mean = self.mean + delta * batch_count / tot_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + np.square(delta) * self.count * batch_count / tot_count
        self.mean = new_mean
        self.var = m2 / tot_count
        self.count = tot_count

    @property
    def std(self) -> np.ndarray:
        return np.sqrt(np.maximum(self.var, 1e-8))


class _RNDNet(nn.Module):
    def __init__(self, obs_dim: int, hidden: int, out_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, out_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class _ObsRing:
    """Fixed-capacity FIFO of raw (unnormalized) obs rows."""

    def __init__(self, capacity: int, obs_dim: int):
        self.capacity = int(capacity)
        self.buf = np.zeros((self.capacity, obs_dim), dtype=np.float32)
        self._n = 0
        self._i = 0

    def __len__(self):
        return min(self._n, self.capacity)

    def push(self, x: np.ndarray) -> None:
        k = len(x)
        if k == 0:
            return
        if k >= self.capacity:
            x, k = x[-self.capacity:], self.capacity
        end = self._i + k
        if end <= self.capacity:
            self.buf[self._i:end] = x
        else:
            first = self.capacity - self._i
            self.buf[self._i:] = x[:first]
            self.buf[:end - self.capacity] = x[first:]
        self._i = end % self.capacity
        self._n = min(self._n + k, self.capacity)

    def sample(self, n: int, rng: np.random.Generator) -> np.ndarray:
        m = len(self)
        idx = rng.integers(0, m, size=n)
        return self.buf[idx]


class RNDVecWrapper(VecEnvWrapper):
    """Blend an RND state-novelty intrinsic reward into the env reward.

    ``rnd_coef <= 0`` is rejected: the caller must simply not construct
    the wrapper (bit-exact off path, mirrors ``AMPStyleVecWrapper``).
    """

    def __init__(self, venv, *, rnd_coef: float, obs_dim: int | None = None,
                 hidden: int = 128, out_dim: int = 64, lr: float = 1e-4,
                 buffer_size: int = 200_000, seed: int = 0,
                 clip_obs: float = 5.0):
        super().__init__(venv)
        if rnd_coef <= 0.0:
            raise ValueError("RNDVecWrapper needs rnd_coef > 0; for RND "
                             "off, do not wrap at all")
        self.rnd_coef = float(rnd_coef)
        self.clip_obs = float(clip_obs)
        obs_dim = int(obs_dim if obs_dim is not None
                      else np.prod(venv.observation_space.shape))
        g = torch.Generator().manual_seed(int(seed))
        torch.manual_seed(int(seed))
        self.target = _RNDNet(obs_dim, hidden, out_dim)
        self.predictor = _RNDNet(obs_dim, hidden, out_dim)
        for p in self.target.parameters():
            p.requires_grad_(False)
        self.opt = torch.optim.Adam(self.predictor.parameters(), lr=lr)
        self.obs_rms = _RunningMeanStd((obs_dim,))
        self.ret_rms = _RunningMeanStd(())
        self.ring = _ObsRing(buffer_size, obs_dim)
        self.rng = np.random.default_rng(seed)
        self.updates = 0
        self._stat_intrinsic_sum = 0.0
        self._stat_intrinsic_n = 0
        del g

    # ------------------------------------------------------------- env
    def reset(self):
        return self.venv.reset()

    def _normalize(self, obs: np.ndarray) -> np.ndarray:
        z = (obs - self.obs_rms.mean) / self.obs_rms.std
        return np.clip(z, -self.clip_obs, self.clip_obs).astype(np.float32)

    def step_wait(self):
        obs, rews, dones, infos = self.venv.step_wait()
        flat = np.asarray(obs, dtype=np.float32).reshape(len(rews), -1)
        self.obs_rms.update(flat)
        norm = self._normalize(flat)
        with torch.no_grad():
            t = torch.as_tensor(norm)
            target_out = self.target(t)
            pred_out = self.predictor(t)
            intrinsic = ((pred_out - target_out) ** 2).mean(dim=-1).numpy()
        self.ret_rms.update(intrinsic)
        scaled = intrinsic / (float(self.ret_rms.std) + 1e-8)
        blended = np.asarray(rews, dtype=np.float32) + self.rnd_coef * scaled
        self.ring.push(flat)
        self._stat_intrinsic_sum += float(intrinsic.sum())
        self._stat_intrinsic_n += len(intrinsic)
        for i in range(len(infos)):
            infos[i]["reward_rnd_intrinsic"] = float(
                self.rnd_coef * scaled[i])
        return obs, blended, dones, infos

    # -------------------------------------------------------- predictor
    def train_predictor(self, steps: int, batch: int) -> dict | None:
        """K minibatch updates of the predictor toward the frozen
        target on buffered (already-visited) obs. Returns mean stats,
        or None while the buffer is too small to fill one batch."""
        if len(self.ring) < batch:
            return None
        total_loss = 0.0
        for _ in range(int(steps)):
            raw = self.ring.sample(batch, self.rng)
            norm = self._normalize(raw)
            x = torch.as_tensor(norm)
            with torch.no_grad():
                target_out = self.target(x)
            pred_out = self.predictor(x)
            loss = ((pred_out - target_out) ** 2).mean()
            self.opt.zero_grad()
            loss.backward()
            self.opt.step()
            self.updates += 1
            total_loss += float(loss.detach())
        return {"loss": total_loss / max(int(steps), 1),
                "buffer_len": float(len(self.ring)),
                "updates": float(self.updates)}

    def pop_rollout_stats(self) -> dict:
        n = max(self._stat_intrinsic_n, 1)
        out = {"intrinsic_mean": self._stat_intrinsic_sum / n,
               "n": self._stat_intrinsic_n}
        self._stat_intrinsic_sum = 0.0
        self._stat_intrinsic_n = 0
        return out

    # --------------------------------------------------------- persist
    def save(self, path) -> None:
        torch.save({"predictor": self.predictor.state_dict(),
                    "target": self.target.state_dict(),
                    "opt": self.opt.state_dict(),
                    "obs_mean": self.obs_rms.mean, "obs_var": self.obs_rms.var,
                    "obs_count": self.obs_rms.count,
                    "ret_var": self.ret_rms.var, "ret_count": self.ret_rms.count,
                    "updates": self.updates}, path)

    def load(self, path) -> None:
        ckpt = torch.load(path, map_location="cpu", weights_only=False)
        self.predictor.load_state_dict(ckpt["predictor"])
        self.target.load_state_dict(ckpt["target"])
        self.opt.load_state_dict(ckpt["opt"])
        self.obs_rms.mean = ckpt["obs_mean"]
        self.obs_rms.var = ckpt["obs_var"]
        self.obs_rms.count = ckpt["obs_count"]
        self.ret_rms.var = ckpt["ret_var"]
        self.ret_rms.count = ckpt["ret_count"]
        self.updates = ckpt["updates"]
