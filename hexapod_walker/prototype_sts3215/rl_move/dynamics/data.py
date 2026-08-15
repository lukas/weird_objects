"""data.py — window sampling over collected rollout shards.

Loads every shard_*.npz under a dataset dir, splits by EPISODE (val
fraction is a deterministic hash of the episode's global index, so the
split is stable across runs and processes), computes per-channel
normalization statistics on the TRAIN episodes only, and samples
training windows:

    history  frames[t-H+1 .. t]            (H, 86)  normalized
    actions  actions[t .. t+Kmax-1]        (Kmax, 18)
    per horizon k:
        state target   frames[t+k][0:44]   normalized (q,qd,tilt,gyro,accel)
        contact target frames[t+k][62:68] > CONTACT_THRESH_N
        priv_now       priv[t]             normalized privileged truths
        priv target    priv[t+k]           normalized privileged truths
        future window  frames[t+k-H+1 .. t+k]  (for the latent target)

Validity: t >= H-1 and t + Kmax <= F-1 (frames per episode F = steps+1;
the executed action exists for every frame except the terminal one).
Every continuous channel is standardized (mean/std) so no channel
dominates the loss merely by scale (brief requirement).
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import frames as fr

VAL_FRACTION = 0.1
STD_FLOOR = 1e-3


def _is_val(ep_global_idx: int) -> bool:
    h = hashlib.md5(f"dynrep-ep-{ep_global_idx}".encode()).digest()
    return (h[0] / 255.0) < VAL_FRACTION


@dataclass
class Stats:
    mean: np.ndarray
    std: np.ndarray
    priv_mean: np.ndarray | None = None
    priv_std: np.ndarray | None = None

    def normalize(self, frames: np.ndarray) -> np.ndarray:
        return (frames - self.mean) / self.std

    def normalize_priv(self, priv: np.ndarray) -> np.ndarray:
        if self.priv_mean is None or self.priv_std is None:
            return priv
        return (priv - self.priv_mean) / self.priv_std

    def denormalize_priv(self, priv: np.ndarray) -> np.ndarray:
        if self.priv_mean is None or self.priv_std is None:
            return priv
        return priv * self.priv_std + self.priv_mean

    def to_dict(self) -> dict:
        d = {"mean": self.mean.tolist(), "std": self.std.tolist()}
        if self.priv_mean is not None and self.priv_std is not None:
            d["priv_mean"] = self.priv_mean.tolist()
            d["priv_std"] = self.priv_std.tolist()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Stats":
        return cls(np.asarray(d["mean"], dtype=np.float32),
                   np.asarray(d["std"], dtype=np.float32),
                   (np.asarray(d["priv_mean"], dtype=np.float32)
                    if "priv_mean" in d else None),
                   (np.asarray(d["priv_std"], dtype=np.float32)
                    if "priv_std" in d else None))


@dataclass
class Episode:
    frames: np.ndarray      # (F, 86) float32
    actions: np.ndarray     # (F-1, 18) float32
    priv: np.ndarray        # (F, 4)
    actor: str
    mode: str
    reason: str
    dr: float
    global_idx: int
    q_nom: np.ndarray | None = None    # (18,) episode settled start pose
    is_val: bool = field(init=False)

    def __post_init__(self):
        self.is_val = _is_val(self.global_idx)


def load_dataset(path: Path | str) -> list[Episode]:
    path = Path(path)
    shards = sorted(path.glob("shard_*.npz"))
    if not shards:
        raise FileNotFoundError(f"no shard_*.npz under {path}")
    meta_path = path / "meta.json"
    if meta_path.exists():
        ver = json.loads(meta_path.read_text()).get("layout_version")
        if ver != fr.LAYOUT_VERSION:
            raise ValueError(
                f"dataset {path} has frame layout {ver!r}, code expects "
                f"{fr.LAYOUT_VERSION!r} — re-collect (v1 stored absolute "
                f"q; v2 stores obs-contract relative q)")
    eps: list[Episode] = []
    gidx = 0
    for shard in shards:
        z = np.load(shard, allow_pickle=False)
        f_off = a_off = 0
        for i in range(len(z["ep_frames"])):
            nf = int(z["ep_frames"][i])
            na = int(z["ep_actions"][i])
            eps.append(Episode(
                frames=z["frames"][f_off:f_off + nf],
                actions=z["actions"][a_off:a_off + na],
                priv=fr.upgrade_priv(z["priv"][f_off:f_off + nf]),
                actor=str(z["ep_actor"][i]), mode=str(z["ep_mode"][i]),
                reason=str(z["ep_reason"][i]), dr=float(z["ep_dr"][i]),
                global_idx=gidx,
                q_nom=(z["ep_qnom"][i] if "ep_qnom" in z else None)))
            f_off += nf
            a_off += na
            gidx += 1
    return eps


def compute_stats(eps: list[Episode]) -> Stats:
    train = np.concatenate([e.frames for e in eps if not e.is_val])
    priv = np.concatenate([e.priv for e in eps if not e.is_val])
    mean = train.mean(axis=0).astype(np.float32)
    std = np.maximum(train.std(axis=0), STD_FLOOR).astype(np.float32)
    priv_mean = priv.mean(axis=0).astype(np.float32)
    priv_std = np.maximum(priv.std(axis=0), STD_FLOOR).astype(np.float32)
    return Stats(mean, std, priv_mean, priv_std)


class WindowSampler:
    """Uniform sampling of valid (episode, t) windows in one split."""

    def __init__(self, eps: list[Episode], stats: Stats, history: int,
                 horizons: tuple[int, ...], val: bool,
                 seed: int = 0):
        self.stats = stats
        self.H = int(history)
        self.horizons = tuple(sorted(int(k) for k in horizons))
        self.Kmax = self.horizons[-1]
        self.rng = np.random.default_rng(seed)
        self.eps = [e for e in eps if e.is_val == val]
        self.index: list[tuple[int, int]] = []      # (ep_i, t)
        for i, e in enumerate(self.eps):
            F = len(e.frames)
            lo, hi = self.H - 1, F - 1 - self.Kmax
            for t in range(lo, hi + 1):
                self.index.append((i, t))
        if not self.index:
            raise ValueError(
                f"no valid windows (val={val}): need episodes with at "
                f"least H+Kmax={self.H + self.Kmax} frames")

    def __len__(self) -> int:
        return len(self.index)

    def batch(self, n: int, idx: np.ndarray | None = None) -> dict:
        """One batch as numpy arrays; ``idx`` overrides random draws
        (used for deterministic validation)."""
        if idx is None:
            idx = self.rng.integers(0, len(self.index), size=n)
        H, Kmax = self.H, self.Kmax
        D = fr.FRAME_DIM
        hist = np.empty((len(idx), H, D), dtype=np.float32)
        fut_a = np.empty((len(idx), Kmax, fr.ACTION_DIM), dtype=np.float32)
        state = {k: np.empty((len(idx), fr.STATE_DIM), dtype=np.float32)
                 for k in self.horizons}
        contact = {k: np.empty((len(idx), fr.N_FEET), dtype=np.float32)
                   for k in self.horizons}
        priv_now = np.empty((len(idx), fr.PRIV_DIM), dtype=np.float32)
        priv = {k: np.empty((len(idx), fr.PRIV_DIM), dtype=np.float32)
                for k in self.horizons}
        fut_hist = {k: np.empty((len(idx), H, D), dtype=np.float32)
                    for k in self.horizons}
        for row, j in enumerate(np.asarray(idx)):
            ep_i, t = self.index[int(j)]
            f = self.eps[ep_i].frames
            a = self.eps[ep_i].actions
            p = self.eps[ep_i].priv
            hist[row] = self.stats.normalize(f[t - H + 1:t + 1])
            fut_a[row] = a[t:t + Kmax]
            priv_now[row] = self.stats.normalize_priv(p[t])
            for k in self.horizons:
                tk = t + k
                nf = self.stats.normalize(f[tk])
                state[k][row] = nf[fr.STATE_SLICE]
                contact[k][row] = (f[tk][fr.CONTACT_SLICE]
                                   > fr.CONTACT_THRESH_N)
                priv[k][row] = self.stats.normalize_priv(p[tk])
                fut_hist[k][row] = self.stats.normalize(
                    f[tk - H + 1:tk + 1])
        return {"hist": hist, "fut_actions": fut_a, "state": state,
                "contact": contact, "priv_now": priv_now, "priv": priv,
                "fut_hist": fut_hist}

    def val_batches(self, n_windows: int, batch: int):
        """Deterministic, evenly spaced coverage of the split."""
        step = max(len(self.index) // max(n_windows, 1), 1)
        all_idx = np.arange(0, len(self.index), step)[:n_windows]
        for i0 in range(0, len(all_idx), batch):
            yield self.batch(0, idx=all_idx[i0:i0 + batch])


def describe(eps: list[Episode]) -> str:
    n_val = sum(e.is_val for e in eps)
    steps = sum(len(e.actions) for e in eps)
    actors: dict[str, int] = {}
    falls = 0
    for e in eps:
        actors[e.actor] = actors.get(e.actor, 0) + 1
        if "trunc" not in e.reason and e.reason != "end":
            falls += 1
    return (f"{len(eps)} episodes ({n_val} val), {steps} steps "
            f"({steps / 25 / 60:.1f} sim-min), "
            f"{falls} terminated early (falls/trips); "
            f"actors: {json.dumps(actors)}")
