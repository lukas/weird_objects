"""data.py — window sampling over collected rollout shards.

Loads every shard_*.npz under a dataset dir, splits by EPISODE (a
deterministic hash of the episode's global index produces disjoint
train/validation/test sets that are stable across runs and processes),
computes per-channel
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

SPLIT_VERSION = "episode-blake2b-80-10-10-v1"
TRAIN_FRACTION = 0.8
VAL_FRACTION = 0.1
TEST_FRACTION = 0.1
SPLITS = ("train", "val", "test")
STD_FLOOR = 1e-3


def split_for_episode(ep_global_idx: int) -> str:
    """Stable, whole-episode 80/10/10 split.

    Eight hash bytes avoid the coarse 1/256 buckets used by the old
    train/validation-only split. No frame or overlapping window from one
    episode can appear in more than one split.
    """
    digest = hashlib.blake2b(
        f"dynrep-ep-{int(ep_global_idx)}".encode(), digest_size=8).digest()
    unit = int.from_bytes(digest, "big") / float(1 << 64)
    if unit < TRAIN_FRACTION:
        return "train"
    if unit < TRAIN_FRACTION + VAL_FRACTION:
        return "val"
    return "test"


def is_val_episode(ep_global_idx: int) -> bool:
    """Backward-compatible predicate for the validation split."""
    return split_for_episode(ep_global_idx) == "val"


def is_test_episode(ep_global_idx: int) -> bool:
    return split_for_episode(ep_global_idx) == "test"


def _is_val(ep_global_idx: int) -> bool:
    """Backward-compatible private alias."""
    return is_val_episode(ep_global_idx)


def valid_window_count(n_frames: int, history: int,
                       horizons: tuple[int, ...]) -> int:
    """Number of distinct valid center timesteps in one episode."""
    if not horizons:
        raise ValueError("at least one prediction horizon is required")
    return max(int(n_frames) - int(history) - max(horizons) + 1, 0)


def window_budget(eps: list["Episode"], history: int,
                  horizons: tuple[int, ...]) -> dict[str, int]:
    """Distinct center-window counts for mechanical data-reuse checks."""
    out = {split: 0 for split in SPLITS}
    for episode in eps:
        out[episode.split] += valid_window_count(
            len(episode.frames), history, horizons)
    return out


def split_diagnostics(eps: list["Episode"], history: int,
                      horizons: tuple[int, ...]) -> dict[str, dict]:
    """Episode/window counts and marginal coverage for each split."""
    out = {
        split: {"episodes": 0, "windows": 0, "actors": {}, "dr": {},
                "modes": {}}
        for split in SPLITS
    }
    for episode in eps:
        row = out[episode.split]
        row["episodes"] += 1
        row["windows"] += valid_window_count(
            len(episode.frames), history, horizons)
        for key, value in (("actors", episode.actor),
                           ("dr", f"{episode.dr:g}"),
                           ("modes", episode.mode)):
            row[key][value] = row[key].get(value, 0) + 1
    return out


def validate_split_coverage(diagnostics: dict[str, dict]) -> None:
    """Refuse datasets whose held-out splits miss a collected stratum."""
    for split in SPLITS:
        if diagnostics[split]["episodes"] <= 0:
            raise ValueError(f"dataset split {split!r} has no episodes")
        if diagnostics[split]["windows"] <= 0:
            raise ValueError(f"dataset split {split!r} has no valid windows")
    for category in ("actors", "dr", "modes"):
        expected = set().union(
            *(set(diagnostics[s][category]) for s in SPLITS))
        for split in SPLITS:
            missing = sorted(expected - set(diagnostics[split][category]))
            if missing:
                raise ValueError(
                    f"dataset split {split!r} is missing {category}: "
                    f"{', '.join(missing)}")


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
    priv: np.ndarray        # (F, PRIV_DIM), upgraded if needed
    priv_mask: np.ndarray   # (PRIV_DIM,), 1 only for real labels
    actor: str
    mode: str
    reason: str
    dr: float
    global_idx: int
    q_nom: np.ndarray | None = None    # (18,) episode settled start pose
    split: str = field(init=False)
    is_val: bool = field(init=False)

    def __post_init__(self):
        self.split = split_for_episode(self.global_idx)
        self.is_val = self.split == "val"


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
            raw_priv = z["priv"][f_off:f_off + nf]
            eps.append(Episode(
                frames=z["frames"][f_off:f_off + nf],
                actions=z["actions"][a_off:a_off + na],
                priv=fr.upgrade_priv(raw_priv),
                priv_mask=fr.priv_available_mask(raw_priv.shape[-1]),
                actor=str(z["ep_actor"][i]), mode=str(z["ep_mode"][i]),
                reason=str(z["ep_reason"][i]), dr=float(z["ep_dr"][i]),
                global_idx=gidx,
                q_nom=(z["ep_qnom"][i] if "ep_qnom" in z else None)))
            f_off += nf
            a_off += na
            gidx += 1
    return eps


def compute_stats(eps: list[Episode]) -> Stats:
    train_eps = [e for e in eps if e.split == "train"]
    if not train_eps:
        raise ValueError("cannot compute normalization without train episodes")
    train = np.concatenate([e.frames for e in train_eps])
    mean = train.mean(axis=0).astype(np.float32)
    std = np.maximum(train.std(axis=0), STD_FLOOR).astype(np.float32)
    priv_mean = np.zeros(fr.PRIV_DIM, dtype=np.float32)
    priv_std = np.ones(fr.PRIV_DIM, dtype=np.float32)
    for j in range(fr.PRIV_DIM):
        available = [e.priv[:, j] for e in train_eps if e.priv_mask[j]]
        if available:
            values = np.concatenate(available)
            priv_mean[j] = values.mean()
            priv_std[j] = max(float(values.std()), STD_FLOOR)
    return Stats(mean, std, priv_mean, priv_std)


def full_priv_fraction(eps: list[Episode]) -> float:
    """Fraction of episodes carrying all current privileged labels."""
    return float(np.mean([bool(np.all(e.priv_mask)) for e in eps]))


class WindowSampler:
    """Uniform sampling of valid (episode, t) windows in one split."""

    def __init__(self, eps: list[Episode], stats: Stats, history: int,
                 horizons: tuple[int, ...], val: bool | None = None,
                 seed: int = 0, *, split: str | None = None):
        self.stats = stats
        self.H = int(history)
        self.horizons = tuple(sorted(int(k) for k in horizons))
        self.Kmax = self.horizons[-1]
        self.rng = np.random.default_rng(seed)
        if split is None:
            if val is None:
                raise ValueError("pass split='train', 'val', or 'test'")
            split = "val" if val else "train"
        if split not in SPLITS:
            raise ValueError(f"unknown split {split!r}; expected one of {SPLITS}")
        self.split = split
        self.eps = [e for e in eps if e.split == split]
        self.index: list[tuple[int, int]] = []      # (ep_i, t)
        for i, e in enumerate(self.eps):
            F = len(e.frames)
            lo, hi = self.H - 1, F - 1 - self.Kmax
            for t in range(lo, hi + 1):
                self.index.append((i, t))
        if not self.index:
            raise ValueError(
                f"no valid windows (split={split}): need episodes with at "
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
        priv_mask_now = np.empty((len(idx), fr.PRIV_DIM), dtype=np.float32)
        priv = {k: np.empty((len(idx), fr.PRIV_DIM), dtype=np.float32)
                for k in self.horizons}
        current_now = np.empty((len(idx), fr.CURRENT_DIM), dtype=np.float32)
        contact_now = np.empty((len(idx), fr.N_FEET), dtype=np.float32)
        current = {k: np.empty((len(idx), fr.CURRENT_DIM), dtype=np.float32)
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
            priv_mask_now[row] = self.eps[ep_i].priv_mask
            current_now[row] = hist[row, -1, fr.CURRENT_SLICE]
            contact_now[row] = (f[t][fr.CONTACT_SLICE]
                                > fr.CONTACT_THRESH_N)
            for k in self.horizons:
                tk = t + k
                nf = self.stats.normalize(f[tk])
                state[k][row] = nf[fr.STATE_SLICE]
                contact[k][row] = (f[tk][fr.CONTACT_SLICE]
                                   > fr.CONTACT_THRESH_N)
                priv[k][row] = self.stats.normalize_priv(p[tk])
                current[k][row] = nf[fr.CURRENT_SLICE]
                fut_hist[k][row] = self.stats.normalize(
                    f[tk - H + 1:tk + 1])
        return {"hist": hist, "fut_actions": fut_a, "state": state,
                "contact": contact, "contact_now": contact_now,
                "current": current, "current_now": current_now,
                "priv_now": priv_now, "priv_mask_now": priv_mask_now,
                "priv": priv,
                "fut_hist": fut_hist}

    def val_batches(self, n_windows: int, batch: int):
        """Deterministic, evenly spaced coverage of the split."""
        step = max(len(self.index) // max(n_windows, 1), 1)
        all_idx = np.arange(0, len(self.index), step)[:n_windows]
        for i0 in range(0, len(all_idx), batch):
            yield self.batch(0, idx=all_idx[i0:i0 + batch])


class GpuWindowSampler:
    """Window sampler whose training batches are gathered on CUDA.

    Shards and normalization statistics are loaded once on the host, then
    all frame/action tensors, random index selection, gathers, targets, and
    metrics stay on the selected GPU.
    """

    def __init__(self, eps: list[Episode], stats: Stats, history: int,
                 horizons: tuple[int, ...], val: bool | None = None,
                 device=None, seed: int = 0, *, split: str | None = None):
        import torch

        self.stats = stats
        self.H = int(history)
        self.horizons = tuple(sorted(int(k) for k in horizons))
        self.Kmax = self.horizons[-1]
        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise ValueError("GpuWindowSampler requires a CUDA device")
        if split is None:
            if val is None:
                raise ValueError("pass split='train', 'val', or 'test'")
            split = "val" if val else "train"
        if split not in SPLITS:
            raise ValueError(f"unknown split {split!r}; expected one of {SPLITS}")
        self.split = split
        selected = [e for e in eps if e.split == split]
        frames, actions, priv = [], [], []
        frame_centers, action_centers, masks = [], [], []
        foff = aoff = 0
        for e in selected:
            frames.append(stats.normalize(e.frames).astype(np.float32))
            actions.append(e.actions.astype(np.float32))
            priv.append(stats.normalize_priv(e.priv).astype(np.float32))
            lo, hi = self.H - 1, len(e.frames) - 1 - self.Kmax
            if hi >= lo:
                ts = np.arange(lo, hi + 1, dtype=np.int64)
                frame_centers.append(ts + foff)
                action_centers.append(ts + aoff)
                masks.append(np.broadcast_to(e.priv_mask, (len(ts),
                                                           fr.PRIV_DIM)))
            foff += len(e.frames)
            aoff += len(e.actions)
        if not frame_centers:
            raise ValueError(f"no valid windows (split={split}): need episodes "
                             f"with at least H+Kmax={self.H + self.Kmax} "
                             "frames")
        self.frames = torch.as_tensor(np.concatenate(frames),
                                      device=self.device)
        self.actions = torch.as_tensor(np.concatenate(actions),
                                       device=self.device)
        self.priv = torch.as_tensor(np.concatenate(priv), device=self.device)
        self.frame_centers = torch.as_tensor(np.concatenate(frame_centers),
                                             device=self.device)
        self.action_centers = torch.as_tensor(np.concatenate(action_centers),
                                              device=self.device)
        self.priv_masks = torch.as_tensor(np.concatenate(masks),
                                          device=self.device)
        self.frame_mean = torch.as_tensor(stats.mean, device=self.device)
        self.frame_std = torch.as_tensor(stats.std, device=self.device)
        self.hist_offsets = torch.arange(-self.H + 1, 1,
                                         device=self.device)
        self.action_offsets = torch.arange(self.Kmax, device=self.device)
        self.generator = torch.Generator(device=self.device)
        self.generator.manual_seed(seed)

    def __len__(self) -> int:
        return int(self.frame_centers.numel())

    def batch(self, n: int, idx=None) -> dict:
        import torch

        if idx is None:
            idx = torch.randint(len(self), (n,), device=self.device,
                                generator=self.generator)
        else:
            idx = torch.as_tensor(idx, dtype=torch.long, device=self.device)
        fc = self.frame_centers.index_select(0, idx)
        ac = self.action_centers.index_select(0, idx)
        hist = self.frames[fc[:, None] + self.hist_offsets[None, :]]
        fut_actions = self.actions[ac[:, None] + self.action_offsets[None, :]]
        state, contact, current, priv, fut_hist = {}, {}, {}, {}, {}
        for k in self.horizons:
            target = self.frames[fc + k]
            state[k] = target[:, fr.STATE_SLICE]
            current[k] = target[:, fr.CURRENT_SLICE]
            raw_contact = (target[:, fr.CONTACT_SLICE]
                           * self.frame_std[fr.CONTACT_SLICE]
                           + self.frame_mean[fr.CONTACT_SLICE])
            contact[k] = (raw_contact > fr.CONTACT_THRESH_N).float()
            priv[k] = self.priv[fc + k]
            future_center = fc + k
            fut_hist[k] = self.frames[
                future_center[:, None] + self.hist_offsets[None, :]]
        raw_contact_now = (hist[:, -1, fr.CONTACT_SLICE]
                           * self.frame_std[fr.CONTACT_SLICE]
                           + self.frame_mean[fr.CONTACT_SLICE])
        return {
            "hist": hist, "fut_actions": fut_actions,
            "state": state, "contact": contact, "current": current,
            "contact_now": (raw_contact_now > fr.CONTACT_THRESH_N).float(),
            "current_now": hist[:, -1, fr.CURRENT_SLICE],
            "priv_now": self.priv[fc],
            "priv_mask_now": self.priv_masks.index_select(0, idx),
            "priv": priv, "fut_hist": fut_hist,
        }

    def val_batches(self, n_windows: int, batch: int):
        import torch

        step = max(len(self) // max(n_windows, 1), 1)
        all_idx = torch.arange(0, len(self), step, device=self.device)
        all_idx = all_idx[:n_windows]
        for i0 in range(0, len(all_idx), batch):
            yield self.batch(0, idx=all_idx[i0:i0 + batch])


def describe(eps: list[Episode]) -> str:
    split_counts = {split: sum(e.split == split for e in eps)
                    for split in SPLITS}
    steps = sum(len(e.actions) for e in eps)
    actors: dict[str, int] = {}
    falls = 0
    for e in eps:
        actors[e.actor] = actors.get(e.actor, 0) + 1
        if "trunc" not in e.reason and e.reason != "end":
            falls += 1
    return (f"{len(eps)} episodes (train/val/test "
            f"{split_counts['train']}/{split_counts['val']}/"
            f"{split_counts['test']}), {steps} steps "
            f"({steps / 25 / 60:.1f} sim-min), "
            f"{falls} terminated early (falls/trips); "
            f"actors: {json.dumps(actors)}")
