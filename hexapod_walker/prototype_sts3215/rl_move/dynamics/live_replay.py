"""live_replay.py — command-rich stratified CUDA replay for the LIVE
predictive-critic run (operator order fb_20260817T210422_9df9c7,
run cw-dynrep-livewalkrise1).

Extends the condition-E online path into true live CUDA
collection+training:

    LiveWindowStore   holds dynamics windows extracted from live PPO
                      rollout episodes as CUDA-resident tensors,
                      stratified by goal MODE (walk / rise), each window
                      tagged with coarse pre-registered bins:
                      command direction / speed / yaw, command-change
                      and start-stop transitions inside the window span,
                      contact (feet-on-ground) phase, progress-vs-slip
                      proxy, fall episodes, rise start kind (flat-or-
                      bridge / crouch / post-lower bank) and DR level.
                      Every ``val_every``-th accepted episode per mode
                      goes to a VAL store (never trained) — that is the
                      command-rich heldout the snapshot gates read.
    stratified_batch  75%% fresh windows (75/25 walk/rise target inside
                      the fresh part) + 25%% recovered-v5 rehearsal, all
                      returned as tensors on the training device, with
                      honest realized fractions.
    bin_report        per-bin composition counters + per-bin prediction
                      errors (velocity vector, heading, tilt, contact,
                      joints, servo currents — current-state AND
                      future-horizon heads) for W&B.

Window/target semantics are EXACTLY data.WindowSampler's: extraction
goes through a single-episode WindowSampler (parity is what the test
bank locks), then the numpy batch is moved to the device once and kept
there. Frames are normalized with the PRETRAINING stats, never re-fit.
"""
from __future__ import annotations

import math

import numpy as np
import torch as th

from . import data as dd
from . import frames as fr

# --- pre-registered coarse bins -------------------------------------
MODE_BINS = ("walk", "rise", "other")
CMD_DIR_BINS = ("zero", "fwd", "back", "lat", "diag")
SPEED_BINS = ("stop", "slow", "med", "fast")
YAW_BINS = ("yaw0", "yaw_left", "yaw_right")
TRANS_BINS = ("steady", "cmd_change")
STARTSTOP_BINS = ("cruise", "start_stop")
CONTACT_BINS = ("feet_le2", "feet_3", "feet_4to5", "feet_6")
PROGRESS_BINS = ("prog_na", "prog_lo", "prog_hi")
FALL_BINS = ("clean", "fall_ep")
RISE_START_BINS = ("na", "flat_bridge", "crouch", "post_lower")

BIN_DIMS = {
    "mode": MODE_BINS, "cmd_dir": CMD_DIR_BINS, "speed": SPEED_BINS,
    "yaw": YAW_BINS, "trans": TRANS_BINS, "startstop": STARTSTOP_BINS,
    "contact": CONTACT_BINS, "progress": PROGRESS_BINS,
    "fall": FALL_BINS, "rise_start": RISE_START_BINS,
}
META_KEYS = tuple(BIN_DIMS)

STOP_SPEED = 0.015        # m/s: below = commanded stop
SLOW_SPEED = 0.06
MED_SPEED = 0.09
YAW_EPS = 0.02            # rad/s
CMD_CHANGE_EPS = 0.01     # command-vector change that counts as a change

# priv sidecar columns (frames.PRIV_NAMES)
_PRIV_CMD = slice(7, 10)          # vx_ref, vy_ref, wz_ref
_PRIV_V_ALONG = 10
# Priv channels 7:14 (cmd refs + cmd-projected velocity + cmd heading)
# are EXOGENOUS under command-rich live collection: mid-episode
# resampling makes future commands unpredictable by design, and the
# pretraining corpus never commanded yaw (its priv_std for wz_ref is a
# clamped 0.001, so one live wz_ref=0.3 normalizes to ~300 sigma —
# measured on cw-dynrep-livewalkrise1-canary1: the cmd_track loss group
# hit ~500-580 while every physical head stayed 0.3-1.5, tripling the
# online predictor's corpus-val). Live windows therefore mask these
# channels out of the supervised priv targets; rehearsal windows keep
# the full corpus mask so pretraining-comparable heldout numbers are
# untouched.
_PRIV_EXOGENOUS = slice(7, 14)

# error-report channel groups within the 44-dim STATE target
_STATE_TILT = (36, 37)
_STATE_JOINT_POS = tuple(range(0, 18))
_STATE_JOINT_VEL = tuple(range(18, 36))
# priv groups for per-bin reporting. NOTE: heading = yaw_rel sin/cos
# only — the cmd-heading channels (12, 13) are exogenous-masked on live
# windows (see _PRIV_EXOGENOUS) and would report untrained noise.
_PRIV_VEL = (0, 1, 2, 3)
_PRIV_HEADING = (5, 6)


def _mode_bin(mode: str) -> int:
    if mode == "walk":
        return MODE_BINS.index("walk")
    if mode == "rise":
        return MODE_BINS.index("rise")
    return MODE_BINS.index("other")


def _cmd_dir_bin(vx: float, vy: float) -> int:
    if math.hypot(vx, vy) < STOP_SPEED:
        return CMD_DIR_BINS.index("zero")
    a = abs(math.degrees(math.atan2(vy, vx)))
    if a <= 22.5:
        return CMD_DIR_BINS.index("fwd")
    if a >= 157.5:
        return CMD_DIR_BINS.index("back")
    if 67.5 <= a <= 112.5:
        return CMD_DIR_BINS.index("lat")
    return CMD_DIR_BINS.index("diag")


def _speed_bin(vx: float, vy: float) -> int:
    s = math.hypot(vx, vy)
    if s < STOP_SPEED:
        return SPEED_BINS.index("stop")
    if s < SLOW_SPEED:
        return SPEED_BINS.index("slow")
    if s < MED_SPEED:
        return SPEED_BINS.index("med")
    return SPEED_BINS.index("fast")


def _yaw_bin(wz: float) -> int:
    if abs(wz) < YAW_EPS:
        return YAW_BINS.index("yaw0")
    return YAW_BINS.index("yaw_left" if wz > 0 else "yaw_right")


def _rise_start_bin(mode: str, start_at: str) -> int:
    if mode != "rise":
        return RISE_START_BINS.index("na")
    if start_at == "rise_bank":
        return RISE_START_BINS.index("post_lower")
    if start_at == "crouch":
        return RISE_START_BINS.index("crouch")
    return RISE_START_BINS.index("flat_bridge")


def window_meta(ep_priv: np.ndarray, ep_frames: np.ndarray, t: int,
                history: int, k_max: int, mode: str, reason: str,
                start_at: str) -> np.ndarray:
    """Coarse bin indices (int16, order META_KEYS) for the window
    anchored at frame ``t``. ``ep_priv``/``ep_frames`` are the RAW
    (unnormalized) episode arrays."""
    cmd = ep_priv[:, _PRIV_CMD]
    vx, vy, wz = (float(cmd[t, 0]), float(cmd[t, 1]), float(cmd[t, 2]))
    span = cmd[max(t - history + 1, 0): t + k_max + 1]
    d_cmd = float(np.max(np.abs(span - cmd[t]))) if len(span) else 0.0
    speeds = np.hypot(span[:, 0], span[:, 1])
    moving = speeds >= STOP_SPEED
    startstop = bool(moving.any() and (~moving).any())
    n_on = int(np.sum(ep_frames[t, fr.CONTACT_SLICE]
                      > fr.CONTACT_THRESH_N))
    if n_on <= 2:
        contact_bin = CONTACT_BINS.index("feet_le2")
    elif n_on == 3:
        contact_bin = CONTACT_BINS.index("feet_3")
    elif n_on <= 5:
        contact_bin = CONTACT_BINS.index("feet_4to5")
    else:
        contact_bin = CONTACT_BINS.index("feet_6")
    s_cmd = math.hypot(vx, vy)
    if s_cmd < STOP_SPEED:
        prog_bin = PROGRESS_BINS.index("prog_na")
    else:
        fut = ep_priv[t: t + k_max + 1]
        s_fut = np.hypot(fut[:, 7], fut[:, 8])
        ok = s_fut >= STOP_SPEED
        ratio = (float(np.mean(fut[ok, _PRIV_V_ALONG] / s_fut[ok]))
                 if ok.any() else 0.0)
        prog_bin = PROGRESS_BINS.index("prog_hi" if ratio >= 0.5
                                       else "prog_lo")
    return np.array([
        _mode_bin(mode),
        _cmd_dir_bin(vx, vy),
        _speed_bin(vx, vy),
        _yaw_bin(wz),
        TRANS_BINS.index("cmd_change" if d_cmd > CMD_CHANGE_EPS
                         else "steady"),
        STARTSTOP_BINS.index("start_stop" if startstop else "cruise"),
        contact_bin,
        prog_bin,
        FALL_BINS.index("fall_ep" if reason == "term" else "clean"),
        _rise_start_bin(mode, start_at),
    ], dtype=np.int16)


_TENSOR_KEYS = ("hist", "fut_actions", "contact_now", "current_now",
                "priv_now", "priv_mask_now")
_DICT_KEYS = ("state", "contact", "current", "priv", "fut_hist")


def batch_to_device(b: dict, device) -> dict:
    """WindowSampler numpy batch -> torch tensors on ``device``."""
    out = {}
    for k in _TENSOR_KEYS:
        out[k] = th.as_tensor(b[k], device=device)
    for k in _DICT_KEYS:
        out[k] = {kk: th.as_tensor(vv, device=device)
                  for kk, vv in b[k].items()}
    return out


def cat_device_batches(parts: list[dict]) -> dict:
    parts = [p for p in parts if p is not None]
    if len(parts) == 1:
        return parts[0]
    out = {}
    for k in _TENSOR_KEYS:
        out[k] = th.cat([p[k] for p in parts])
    for k in _DICT_KEYS:
        out[k] = {kk: th.cat([p[k][kk] for p in parts])
                  for kk in parts[0][k]}
    return out


class _Group:
    """One (mode, split) stratum: a list of CUDA tensor chunks."""

    def __init__(self, max_windows: int):
        self.max_windows = int(max_windows)
        self.chunks: list[dict] = []          # device batches
        self.metas: list[np.ndarray] = []     # (n, len(META_KEYS)) int16
        self.n = 0

    def add(self, chunk: dict, meta: np.ndarray) -> None:
        self.chunks.append(chunk)
        self.metas.append(meta)
        self.n += len(meta)
        while self.n > self.max_windows and len(self.chunks) > 1:
            gone = self.metas.pop(0)
            self.chunks.pop(0)
            self.n -= len(gone)

    def gather(self, idx: np.ndarray) -> dict:
        """Global indices -> one device batch."""
        sizes = np.array([len(m) for m in self.metas])
        bounds = np.cumsum(sizes)
        chunk_of = np.searchsorted(bounds, idx, side="right")
        local = idx - np.concatenate([[0], bounds[:-1]])[chunk_of]
        parts = []
        for ci in np.unique(chunk_of):
            sel = th.as_tensor(local[chunk_of == ci],
                               dtype=th.long,
                               device=self.chunks[int(ci)]["hist"].device)
            c = self.chunks[int(ci)]
            part = {k: c[k].index_select(0, sel) for k in _TENSOR_KEYS}
            for k in _DICT_KEYS:
                part[k] = {kk: vv.index_select(0, sel)
                           for kk, vv in c[k].items()}
            parts.append(part)
        return cat_device_batches(parts)

    def meta_all(self) -> np.ndarray:
        if not self.metas:
            return np.zeros((0, len(META_KEYS)), dtype=np.int16)
        return np.concatenate(self.metas)


class LiveWindowStore:
    """Stratified CUDA-resident window store for live rollouts."""

    def __init__(self, stats: dd.Stats, history: int,
                 horizons: tuple[int, ...], device,
                 max_walk_windows: int = 30_000,
                 max_rise_windows: int = 12_000,
                 max_val_windows: int = 5_000,
                 windows_per_episode: int = 64,
                 val_every: int = 8, seed: int = 0,
                 mask_cmd_priv: bool = True):
        self.stats = stats
        self.H = int(history)
        self.horizons = tuple(sorted(int(k) for k in horizons))
        self.Kmax = self.horizons[-1]
        self.device = device
        self.windows_per_episode = int(windows_per_episode)
        self.val_every = int(val_every)
        self.mask_cmd_priv = bool(mask_cmd_priv)
        self.rng = np.random.default_rng(seed)
        self.groups = {
            ("walk", "train"): _Group(max_walk_windows),
            ("rise", "train"): _Group(max_rise_windows),
            ("walk", "val"): _Group(max_val_windows),
            ("rise", "val"): _Group(max_val_windows),
        }
        self._accepted = {"walk": 0, "rise": 0}
        self.episodes_added = 0
        self.episodes_skipped = 0
        # cumulative ADDED composition (train+val), keyed bin dim -> counts
        self.added_counts = {d: np.zeros(len(b), dtype=np.int64)
                             for d, b in BIN_DIMS.items()}
        # cumulative SAMPLED composition (train draws)
        self.sampled_counts = {d: np.zeros(len(b), dtype=np.int64)
                               for d, b in BIN_DIMS.items()}
        self._next_idx = 0
        self._frame_mean = th.as_tensor(stats.mean, device=device)
        self._frame_std = th.as_tensor(stats.std, device=device)
        self._priv_mean = th.as_tensor(stats.priv_mean, device=device)
        self._priv_std = th.as_tensor(stats.priv_std, device=device)

    @property
    def min_episode_frames(self) -> int:
        return self.H + self.Kmax + 1

    def _mode_key(self, mode: str) -> str | None:
        if mode in ("walk", "rise"):
            return mode
        return None

    def add_episode(self, ep: dict) -> bool:
        frames = np.asarray(ep["frames"], dtype=np.float32)
        mode = str(ep.get("mode", "?"))
        key = self._mode_key(mode)
        if key is None or len(frames) < self.min_episode_frames:
            self.episodes_skipped += 1
            return False
        priv_mask = np.ones(fr.PRIV_DIM, dtype=np.float32)
        if self.mask_cmd_priv:
            priv_mask[_PRIV_EXOGENOUS] = 0.0
        actions = np.asarray(ep["actions"], dtype=np.float32)
        priv = fr.upgrade_priv(np.asarray(ep["priv"], dtype=np.float32))
        n_valid = dd.valid_window_count(len(frames), self.H,
                                        self.horizons)
        take = min(n_valid, self.windows_per_episode)
        pos = (np.arange(n_valid) if take == n_valid else
               np.sort(self.rng.choice(n_valid, size=take,
                                       replace=False)))
        batch = self._device_episode_batch(
            frames, actions, priv, priv_mask, pos)
        start_at = str(ep.get("start_at", "?"))
        metas = np.stack([
            window_meta(priv, frames,
                        int(p) + self.H - 1, self.H, self.Kmax,
                        mode, str(ep.get("reason", "trunc")), start_at)
            for p in pos])
        for col, dim in enumerate(META_KEYS):
            np.add.at(self.added_counts[dim], metas[:, col], 1)
        self._accepted[key] += 1
        split = ("val" if self._accepted[key] % self.val_every == 0
                 else "train")
        self.groups[(key, split)].add(batch_to_device(batch, self.device),
                                      metas)
        self._next_idx += 1
        self.episodes_added += 1
        return True

    def _device_episode_batch(self, frames: np.ndarray,
                              actions: np.ndarray,
                              priv: np.ndarray,
                              priv_mask: np.ndarray,
                              pos: np.ndarray) -> dict:
        """Build WindowSampler-equivalent targets directly on ``device``.

        Live collection arrives as host observations because the task shim
        computes reward/labels there. Normalization, window gathering and
        every supervised target are assembled after one upload and remain
        on the predictor device (CUDA in production).
        """
        raw_f = th.as_tensor(frames, device=self.device)
        a = th.as_tensor(actions, device=self.device)
        p = th.as_tensor(priv, device=self.device)
        f = (raw_f - self._frame_mean) / self._frame_std
        pn = (p - self._priv_mean) / self._priv_std
        centers = (th.as_tensor(pos, dtype=th.long, device=self.device)
                   + self.H - 1)
        hist_off = th.arange(-self.H + 1, 1, device=self.device)
        act_off = th.arange(self.Kmax, device=self.device)
        hist = f[centers[:, None] + hist_off[None, :]]
        fut_actions = a[centers[:, None] + act_off[None, :]]
        state, contact, current, priv_fut, fut_hist = {}, {}, {}, {}, {}
        for k in self.horizons:
            tk = centers + k
            target = f[tk]
            state[k] = target[:, fr.STATE_SLICE]
            current[k] = target[:, fr.CURRENT_SLICE]
            contact[k] = (raw_f[tk, fr.CONTACT_SLICE]
                          > fr.CONTACT_THRESH_N).float()
            priv_fut[k] = pn[tk]
            fut_hist[k] = f[tk[:, None] + hist_off[None, :]]
        mask = th.as_tensor(priv_mask, device=self.device).expand(
            len(pos), -1)
        return {
            "hist": hist,
            "fut_actions": fut_actions,
            "state": state,
            "contact": contact,
            "contact_now": (raw_f[centers, fr.CONTACT_SLICE]
                            > fr.CONTACT_THRESH_N).float(),
            "current": current,
            "current_now": hist[:, -1, fr.CURRENT_SLICE],
            "priv_now": pn[centers],
            "priv_mask_now": mask,
            "priv": priv_fut,
            "fut_hist": fut_hist,
        }

    def num_windows(self, mode: str, split: str = "train") -> int:
        return self.groups[(mode, split)].n

    def sample(self, n_walk: int, n_rise: int
               ) -> tuple[dict | None, dict]:
        """Stratified train draw; short groups shrink honestly."""
        parts, realized = [], {"walk": 0, "rise": 0}
        for mode, want in (("walk", n_walk), ("rise", n_rise)):
            g = self.groups[(mode, "train")]
            take = min(int(want), g.n)
            if take <= 0:
                continue
            idx = self.rng.integers(0, g.n, size=take)
            parts.append(g.gather(idx))
            realized[mode] = take
            meta = g.meta_all()[idx]
            for col, dim in enumerate(META_KEYS):
                np.add.at(self.sampled_counts[dim], meta[:, col], 1)
        if not parts:
            return None, realized
        return cat_device_batches(parts), realized

    def val_batches(self, mode: str, batch_size: int = 256,
                    max_windows: int = 2048):
        g = self.groups[(mode, "val")]
        n = min(g.n, int(max_windows))
        for i0 in range(0, n, int(batch_size)):
            yield g.gather(np.arange(i0, min(i0 + batch_size, n)))

    # -- reporting ----------------------------------------------------

    def composition_report(self) -> dict:
        out = {
            "data/episodes_added": self.episodes_added,
            "data/episodes_skipped": self.episodes_skipped,
            "data/windows_walk_train": self.num_windows("walk"),
            "data/windows_rise_train": self.num_windows("rise"),
            "data/windows_walk_val": self.num_windows("walk", "val"),
            "data/windows_rise_val": self.num_windows("rise", "val"),
        }
        for dim, bins in BIN_DIMS.items():
            tot_a = max(int(self.added_counts[dim].sum()), 1)
            tot_s = max(int(self.sampled_counts[dim].sum()), 1)
            for i, b in enumerate(bins):
                out[f"data/added/{dim}/{b}"] = int(
                    self.added_counts[dim][i])
                out[f"data/added_frac/{dim}/{b}"] = (
                    float(self.added_counts[dim][i]) / tot_a)
                out[f"data/sampled_frac/{dim}/{b}"] = (
                    float(self.sampled_counts[dim][i]) / tot_s)
        return out

    def bin_report(self, model, lambdas: dict,
                   max_per_bin: int = 256) -> dict:
        """Per-bin prediction errors of ``model`` on the VAL stores:
        current+future velocity vector / heading (priv heads), tilt,
        contact, joint pos/vel and servo-current errors."""
        model_was_training = model.training
        model.eval()
        out: dict[str, float] = {}
        with th.no_grad():
            for mode in ("walk", "rise"):
                g = self.groups[(mode, "val")]
                if g.n == 0:
                    continue
                meta = g.meta_all()
                for col, dim in enumerate(META_KEYS):
                    for bi, bname in enumerate(BIN_DIMS[dim]):
                        rows = np.flatnonzero(meta[:, col] == bi)
                        if len(rows) < 16:
                            continue
                        if len(rows) > max_per_bin:
                            rows = self.rng.choice(rows, max_per_bin,
                                                   replace=False)
                        bt = g.gather(rows)
                        pred = model(bt["hist"], bt["fut_actions"])
                        pfx = f"pred/bin/{mode}/{dim}={bname}"
                        out.update(_bin_errors(pred, bt, pfx))
                        out.update(prediction_accuracy_metrics(
                            pred, bt, self.stats, prefix=f"{pfx}/physical/"))
                        out[f"{pfx}/n"] = float(len(rows))
        if model_was_training:
            model.train()
        return out


def _sel_mse(a: th.Tensor, b: th.Tensor, cols) -> float:
    c = list(cols)
    return float((a[:, c] - b[:, c]).square().mean())


def _bin_errors(pred: dict, bt: dict, pfx: str) -> dict:
    bce = th.nn.functional.binary_cross_entropy_with_logits
    ks = list(bt["state"])
    out = {
        f"{pfx}/now/vel": _sel_mse(pred["priv_now"], bt["priv_now"],
                                   _PRIV_VEL),
        f"{pfx}/now/heading": _sel_mse(pred["priv_now"], bt["priv_now"],
                                       _PRIV_HEADING),
        f"{pfx}/now/contact": float(bce(pred["contact_now_logits"],
                                        bt["contact_now"])),
        f"{pfx}/now/current": float(
            (pred["current_now"] - bt["current_now"]).square().mean()),
        f"{pfx}/fut/vel": float(np.mean(
            [_sel_mse(pred["priv"][k], bt["priv"][k], _PRIV_VEL)
             for k in pred["priv"]])),
        f"{pfx}/fut/heading": float(np.mean(
            [_sel_mse(pred["priv"][k], bt["priv"][k], _PRIV_HEADING)
             for k in pred["priv"]])),
        f"{pfx}/fut/tilt": float(np.mean(
            [_sel_mse(pred["state"][k], bt["state"][k], _STATE_TILT)
             for k in pred["state"]])),
        f"{pfx}/fut/joint_pos": float(np.mean(
            [_sel_mse(pred["state"][k], bt["state"][k], _STATE_JOINT_POS)
             for k in pred["state"]])),
        f"{pfx}/fut/joint_vel": float(np.mean(
            [_sel_mse(pred["state"][k], bt["state"][k], _STATE_JOINT_VEL)
             for k in pred["state"]])),
        f"{pfx}/fut/contact": float(np.mean(
            [float(bce(pred["contact_logits"][k], bt["contact"][k]))
             for k in pred["contact_logits"]])),
        f"{pfx}/fut/current": float(np.mean(
            [float((pred["current"][k] - bt["current"][k])
                   .square().mean()) for k in pred["current"]])),
    }
    return out


@th.no_grad()
def prediction_accuracy_metrics(pred: dict, bt: dict, stats: dd.Stats,
                                prefix: str = "") -> dict:
    """Interpretable physical-unit accuracy for W&B.

    Losses remain normalized for optimization. These metrics denormalize
    predictions and labels so a curve answers concrete questions such as
    "velocity RMSE in m/s" and "which feet are touching?".
    """
    frame_std = th.as_tensor(stats.std, device=bt["hist"].device)
    priv_mean = th.as_tensor(stats.priv_mean, device=bt["hist"].device)
    priv_std = th.as_tensor(stats.priv_std, device=bt["hist"].device)
    state_std = frame_std[fr.STATE_SLICE]
    current_std = frame_std[fr.CURRENT_SLICE]

    def rmse_scaled(a, b, scale, cols) -> float:
        c = list(cols)
        return float(th.sqrt((((a[:, c] - b[:, c]) * scale[c]) ** 2)
                             .mean()))

    def heading_mae_deg(a, b) -> float:
        pa = a[:, 5:7] * priv_std[5:7] + priv_mean[5:7]
        pb = b[:, 5:7] * priv_std[5:7] + priv_mean[5:7]
        da = th.atan2(pa[:, 0], pa[:, 1])
        db = th.atan2(pb[:, 0], pb[:, 1])
        delta = th.atan2(th.sin(da - db), th.cos(da - db)).abs()
        return float(delta.mean() * (180.0 / math.pi))

    def contact_acc(logits, target) -> float:
        return float(((logits.sigmoid() >= 0.5) == (target >= 0.5))
                     .float().mean())

    out = {
        f"{prefix}now/velocity_rmse_m_s": rmse_scaled(
            pred["priv_now"], bt["priv_now"], priv_std, (0, 1, 2)),
        f"{prefix}now/yaw_rate_rmse_rad_s": rmse_scaled(
            pred["priv_now"], bt["priv_now"], priv_std, (3,)),
        f"{prefix}now/height_rmse_mm": 1000.0 * rmse_scaled(
            pred["priv_now"], bt["priv_now"], priv_std, (4,)),
        f"{prefix}now/heading_mae_deg": heading_mae_deg(
            pred["priv_now"], bt["priv_now"]),
        f"{prefix}now/contact_accuracy": contact_acc(
            pred["contact_now_logits"], bt["contact_now"]),
        f"{prefix}now/current_rmse_a": rmse_scaled(
            pred["current_now"], bt["current_now"], current_std,
            range(fr.CURRENT_DIM)),
    }
    horizons = list(pred["state"])
    if horizons:
        vals: dict[str, list[float]] = {
            "velocity_rmse_m_s": [], "yaw_rate_rmse_rad_s": [],
            "height_rmse_mm": [], "heading_mae_deg": [],
            "tilt_rmse_deg": [], "joint_pos_rmse_deg": [],
            "joint_vel_rmse_deg_s": [], "contact_accuracy": [],
            "current_rmse_a": [],
        }
        for k in horizons:
            vals["velocity_rmse_m_s"].append(rmse_scaled(
                pred["priv"][k], bt["priv"][k], priv_std, (0, 1, 2)))
            vals["yaw_rate_rmse_rad_s"].append(rmse_scaled(
                pred["priv"][k], bt["priv"][k], priv_std, (3,)))
            vals["height_rmse_mm"].append(1000.0 * rmse_scaled(
                pred["priv"][k], bt["priv"][k], priv_std, (4,)))
            vals["heading_mae_deg"].append(heading_mae_deg(
                pred["priv"][k], bt["priv"][k]))
            vals["tilt_rmse_deg"].append((180.0 / math.pi) * rmse_scaled(
                pred["state"][k], bt["state"][k], state_std, (36, 37)))
            vals["joint_pos_rmse_deg"].append(
                (180.0 / math.pi) * rmse_scaled(
                    pred["state"][k], bt["state"][k], state_std,
                    range(18)))
            vals["joint_vel_rmse_deg_s"].append(
                (180.0 / math.pi) * rmse_scaled(
                    pred["state"][k], bt["state"][k], state_std,
                    range(18, 36)))
            vals["contact_accuracy"].append(contact_acc(
                pred["contact_logits"][k], bt["contact"][k]))
            vals["current_rmse_a"].append(rmse_scaled(
                pred["current"][k], bt["current"][k], current_std,
                range(fr.CURRENT_DIM)))
        out.update({f"{prefix}future/{name}": float(np.mean(rows))
                    for name, rows in vals.items()})
    return out


def stratified_live_batch(store: LiveWindowStore, rehearsal,
                          batch_to_torch, device, n: int,
                          rehearsal_frac: float = 0.25,
                          walk_frac: float = 0.75,
                          min_live_windows: int = 512
                          ) -> tuple[dict, dict]:
    """One predictor batch: 75% fresh stratified (75/25 walk/rise
    target) + 25% v5 rehearsal, on ``device``. Falls back toward
    rehearsal honestly while the live stores warm up."""
    n = int(n)
    n_re = int(round(n * float(rehearsal_frac)))
    n_fresh = n - n_re
    n_walk = int(round(n_fresh * float(walk_frac)))
    n_rise = n_fresh - n_walk
    total_live = (store.num_windows("walk") + store.num_windows("rise"))
    if total_live < int(min_live_windows):
        fresh, realized = None, {"walk": 0, "rise": 0}
    else:
        fresh, realized = store.sample(n_walk, n_rise)
    n_got = realized["walk"] + realized["rise"]
    re_batch = batch_to_torch(rehearsal.batch(n - n_got), device=device)
    batch = cat_device_batches([fresh, re_batch])
    info = {
        "pred/batch_fresh_frac": n_got / max(n, 1),
        "pred/batch_rehearsal_frac": (n - n_got) / max(n, 1),
        "pred/batch_walk_frac": realized["walk"] / max(n, 1),
        "pred/batch_rise_frac": realized["rise"] / max(n, 1),
        "pred/batch_fresh_walk_frac": (
            realized["walk"] / max(n_got, 1)),
        "pred/batch_fresh_rise_frac": (
            realized["rise"] / max(n_got, 1)),
    }
    return batch, info
