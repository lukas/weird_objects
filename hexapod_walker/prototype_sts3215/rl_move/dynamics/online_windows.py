"""online_windows.py — fresh dynamics windows from live PPO rollouts.

The corrected condition-C recipe (operator directive
fb_20260816T203212_af7c64) trains the shared transformer's future-state
auxiliary objective PRIMARILY on windows collected from the policy's own
GPU rollouts, with a 20-30% rehearsal mix from the recovered v5
pretraining corpus. This module supplies the online half:

    OnlineEpisodeCapture   gym wrapper that records the exact collector
                           frame/action/priv contract (frames.py v2)
                           inside each training env and emits the whole
                           episode in ``info["dynrep_episode"]`` on
                           termination/truncation.
    OnlineWindowBuffer     bounded FIFO of captured episodes that serves
                           dynamics-loss batches through the REAL
                           ``data.WindowSampler`` (identical window/
                           target semantics to pretraining; every online
                           episode is forced into the train split).
    concat_batches         merge an online batch with a rehearsal batch
                           (nested-dict aware) so one auxiliary batch
                           carries both sources.

Frames are normalized with the PRETRAINING statistics (the encoder
checkpoint's ``stats``), never re-fit online, so the auxiliary loss
stays numerically comparable to the pretraining/heldout values.
"""
from __future__ import annotations

import numpy as np

from . import data as dd
from . import frames as fr


import gymnasium as gym


class OnlineEpisodeCapture(gym.Wrapper):
    """Gym wrapper recording collector-contract episodes from a live env.

    Mirrors collect.py's convention exactly: frame 0 is captured right
    after reset with a zero prev_action; after every step the frame is
    captured with the env's executed action (``env._prev_action``), so
    ``actions[t]`` is the action executed FROM frame t and
    ``len(actions) == len(frames) - 1``. On termination/truncation the
    complete episode rides out in ``info["dynrep_episode"]`` (the only
    cross-process hop; ~120 KB per episode).
    """

    def __init__(self, env, dr_scale: float):
        super().__init__(env)
        self._dr_scale = float(dr_scale)
        self._frames: list[np.ndarray] = []
        self._priv: list[np.ndarray] = []
        self._actions: list[np.ndarray] = []
        self._mode = "?"
        self._start_at = "?"
        self._qnom: np.ndarray | None = None

    def reset(self, **kw):
        obs, info = self.env.reset(**kw)
        raw = self.env.unwrapped
        fr.reset_priv_episode(raw)
        self._frames = [fr.extract_frame(
            raw, np.zeros(fr.ACTION_DIM, dtype=np.float32))]
        self._priv = [fr.extract_priv(raw)]
        self._actions = []
        self._mode = str(getattr(raw._goal_traj, "mode", "?"))
        # start pose kind (rise stratification: flat/bridge = "zero",
        # "crouch", post-lower bank = "rise_bank"; walk = "plant"/"park")
        self._start_at = str(getattr(raw._goal_traj, "start_at", "?"))
        self._qnom = raw._q_nom.astype(np.float32).copy()
        return obs, info

    def step(self, action):
        obs, r, term, trunc, info = self.env.step(action)
        raw = self.env.unwrapped
        self._actions.append(raw._prev_action.astype(np.float32).copy())
        self._frames.append(fr.extract_frame(raw, raw._prev_action))
        self._priv.append(fr.extract_priv(raw))
        if term or trunc:
            info = dict(info)
            info["dynrep_episode"] = {
                "frames": np.stack(self._frames),
                "actions": np.stack(self._actions),
                "priv": np.stack(self._priv),
                "mode": self._mode,
                "start_at": self._start_at,
                "reason": "trunc" if trunc else "term",
                "q_nom": self._qnom,
                "dr": self._dr_scale,
            }
        return obs, r, term, trunc, info


class OnlineWindowBuffer:
    """Bounded FIFO of captured episodes serving WindowSampler batches."""

    def __init__(self, stats: dd.Stats, history: int,
                 horizons: tuple[int, ...], max_frames: int = 120_000,
                 seed: int = 0):
        self.stats = stats
        self.H = int(history)
        self.horizons = tuple(sorted(int(k) for k in horizons))
        self.max_frames = int(max_frames)
        self._seed = int(seed)
        self._eps: list[dd.Episode] = []
        self._frames_total = 0
        self._next_idx = 0
        self._sampler: dd.WindowSampler | None = None
        self._dirty = False
        self.episodes_added = 0
        self.episodes_skipped_short = 0

    @property
    def min_episode_frames(self) -> int:
        return self.H + max(self.horizons) + 1

    def add_episode(self, ep: dict) -> bool:
        """Add one captured episode; returns False for unusable ones."""
        frames = np.asarray(ep["frames"], dtype=np.float32)
        if len(frames) < self.min_episode_frames:
            self.episodes_skipped_short += 1
            return False
        episode = dd.Episode(
            frames=frames,
            actions=np.asarray(ep["actions"], dtype=np.float32),
            priv=fr.upgrade_priv(np.asarray(ep["priv"], dtype=np.float32)),
            priv_mask=np.ones(fr.PRIV_DIM, dtype=np.float32),
            actor="ppo_online", mode=str(ep.get("mode", "?")),
            reason=str(ep.get("reason", "trunc")),
            dr=float(ep.get("dr", 0.0)), global_idx=self._next_idx,
            q_nom=ep.get("q_nom"))
        # Online data is training data by definition — the heldout
        # prediction reference stays the pretraining corpus val split.
        episode.split = "train"
        episode.is_val = False
        self._next_idx += 1
        self._eps.append(episode)
        self._frames_total += len(frames)
        while self._eps and self._frames_total > self.max_frames:
            gone = self._eps.pop(0)
            self._frames_total -= len(gone.frames)
        self._dirty = True
        self.episodes_added += 1
        return True

    @property
    def frames_total(self) -> int:
        return self._frames_total

    def num_windows(self) -> int:
        return sum(dd.valid_window_count(len(e.frames), self.H,
                                         self.horizons)
                   for e in self._eps)

    def _ensure_sampler(self) -> dd.WindowSampler | None:
        if not self._eps:
            return None
        if self._sampler is None or self._dirty:
            self._seed += 1
            self._sampler = dd.WindowSampler(
                self._eps, self.stats, self.H, self.horizons,
                seed=self._seed, split="train")
            self._dirty = False
        return self._sampler

    def batch(self, n: int) -> dict:
        sampler = self._ensure_sampler()
        if sampler is None:
            raise ValueError("online window buffer is empty")
        return sampler.batch(n)


def concat_batches(a: dict, b: dict) -> dict:
    """Concatenate two WindowSampler batches along the batch axis."""
    out: dict = {}
    for key, value in a.items():
        if isinstance(value, dict):
            out[key] = {k: np.concatenate([v, b[key][k]])
                        for k, v in value.items()}
        else:
            out[key] = np.concatenate([value, b[key]])
    return out


def mixed_batch(online: OnlineWindowBuffer, rehearsal, n: int,
                rehearsal_frac: float, min_online_windows: int = 1024
                ) -> tuple[dict, float]:
    """One auxiliary batch: online-primary with a rehearsal mix.

    Returns (batch, actual_rehearsal_fraction). Until the online buffer
    holds ``min_online_windows`` valid windows the batch is 100%
    rehearsal (and the returned fraction says so honestly).
    """
    n = int(n)
    n_re = int(round(n * float(rehearsal_frac)))
    n_on = n - n_re
    if online.num_windows() < max(int(min_online_windows), 1):
        return rehearsal.batch(n), 1.0
    if n_re <= 0:
        return online.batch(n), 0.0
    return (concat_batches(online.batch(n_on), rehearsal.batch(n_re)),
            n_re / n)
