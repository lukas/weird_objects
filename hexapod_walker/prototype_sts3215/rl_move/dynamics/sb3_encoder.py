"""sb3_encoder.py — feed a pretrained dynamics encoder from env obs.

The env (with ``obs.history_frames = H``) emits a stacked observation
of H frames, NEWEST FIRST, each frame = 59 proprio dims (q_rel, qd,
tilt, gyro, prev_action — scaled by the obs config) + the goal/command
tail. The pretrained encoder (``--input-set obs``) was trained on
CHRONOLOGICAL windows of physical-unit frames normalized by dataset
statistics. This features extractor bridges the two contracts:

    stacked obs (B, H*W)
      -> reshape (B, H, W), flip to chronological
      -> un-scale proprio to physical units (q_scale, qd_scale,
         tilt_scale, gyro_scale from the obs config)
      -> scatter into the 86-dim v2 frame layout, apply dataset
         normalization
      -> encoder -> z (128)
    features = concat(z, goal/command tail of the newest frame)

so the policy still sees its commands while the body-dynamics context
arrives through z. Freeze for condition B; leave trainable (with a
scaled-down LR via ``ScaledLRPPO``) for condition C.

Known, accepted distribution shift: at episode start the env fills the
history stack with H copies of the first frame; pretraining windows
never contained that padding.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from . import frames as fr
from .data import Stats
from .model import DynamicsModel

PROPRIO_DIM = 59


def load_dyn_checkpoint(ckpt_path: str | Path):
    ckpt = torch.load(Path(ckpt_path), map_location="cpu",
                      weights_only=False)
    if ckpt.get("layout_version") != fr.LAYOUT_VERSION:
        raise ValueError(
            f"encoder checkpoint has frame layout "
            f"{ckpt.get('layout_version')!r}, code expects "
            f"{fr.LAYOUT_VERSION!r}")
    cfg = ckpt["config"]
    if cfg["input_set"] != "obs":
        raise ValueError(
            "PPO wiring requires an encoder trained with --input-set "
            f"obs (got {cfg['input_set']!r}): the policy obs only "
            "carries the 59 proprio dims")
    model = DynamicsModel(input_set=cfg["input_set"], z_dim=cfg["z_dim"],
                          hidden=cfg.get("hidden", 256),
                          act_hidden=cfg.get("act_hidden", 128),
                          gru_layers=cfg.get("gru_layers", 1),
                          history=cfg.get("history", ckpt.get("history", 16)),
                          arch=cfg.get("arch", "gru"),
                          tf_layers=cfg.get("tf_layers", 4),
                          tf_heads=cfg.get("tf_heads", 8),
                          tf_ff=cfg.get("tf_ff", 1024),
                          tf_dropout=cfg.get("tf_dropout", 0.0),
                          horizons=tuple(cfg["horizons"]),
                          short_max=cfg["short_max"],
                          delta_state=cfg.get("delta_state", False),
                          predict_priv=cfg.get(
                              "predict_priv",
                              any(k.startswith("priv_")
                                  for k in ckpt["model"])))
    model.load_state_dict(ckpt["model"], strict=False)
    stats = Stats.from_dict(ckpt["stats"])
    return model, stats, int(ckpt["history"])


class DynFeaturesExtractor(BaseFeaturesExtractor):
    """z(dyn encoder over obs history) ++ newest frame's goal tail."""

    def __init__(self, observation_space, ckpt_path: str,
                 frame_width: int, history: int, freeze: bool = True,
                 q_scale: float = 1.0, qd_scale: float = 2.0,
                 tilt_scale: float = 0.2, gyro_scale: float = 1.0):
        model, stats, ckpt_h = load_dyn_checkpoint(ckpt_path)
        if history != ckpt_h:
            raise ValueError(f"env history_frames={history} but encoder "
                             f"was pretrained with H={ckpt_h}")
        n_obs = int(np.prod(observation_space.shape))
        if n_obs != history * frame_width:
            raise ValueError(f"obs dim {n_obs} != history {history} x "
                             f"frame_width {frame_width}")
        tail_dim = frame_width - PROPRIO_DIM
        super().__init__(observation_space, model.z_dim + tail_dim)
        self.ckpt_path = str(ckpt_path)
        self.dyn = model
        self.frame_width, self.history = frame_width, history
        self.freeze = freeze
        # Per-channel obs -> physical unscale for the 59 proprio dims.
        unscale = np.concatenate([
            np.full(18, q_scale), np.full(18, qd_scale),
            np.full(2, tilt_scale), np.full(3, gyro_scale),
            np.ones(18)]).astype(np.float32)
        self.register_buffer("unscale", torch.as_tensor(unscale))
        # Proprio channel j lands at frame column proprio_cols[j]
        # (q,qd,tilt,gyro at 0:41; prev_action at 68:86 — frames.py v2).
        cols = np.concatenate([np.arange(0, 41), np.arange(68, 86)])
        self.register_buffer("proprio_cols",
                             torch.as_tensor(cols, dtype=torch.long))
        self.register_buffer("f_mean", torch.as_tensor(stats.mean))
        self.register_buffer("f_std", torch.as_tensor(stats.std))
        if freeze:
            for p in self.dyn.parameters():
                p.requires_grad_(False)
            self.dyn.eval()

    def reload_pretrained(self) -> None:
        """Restore the pretrained encoder weights.

        MUST be called after constructing a FRESH sb3 policy:
        ActorCriticPolicy's default ``ortho_init=True`` orthogonally
        re-initializes every Linear in the features extractor right
        after construction, silently wiping the pretrained weights
        (the GRUs survive — worse, because it looks half-trained).
        ``PPO.load`` restores saved params afterwards, so warm-started
        runs don't need this."""
        model, _, _ = load_dyn_checkpoint(self.ckpt_path)
        self.dyn.load_state_dict(model.state_dict())
        if self.freeze:
            for p in self.dyn.parameters():
                p.requires_grad_(False)
            self.dyn.eval()

    def train(self, mode: bool = True):
        super().train(mode)
        if self.freeze:
            self.dyn.eval()
        return self

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        B = obs.shape[0]
        x = obs.view(B, self.history, self.frame_width)
        x = torch.flip(x, dims=(1,))            # newest-first -> chrono
        proprio = x[..., :PROPRIO_DIM] * self.unscale
        frames86 = obs.new_zeros(B, self.history, fr.FRAME_DIM)
        frames86[..., self.proprio_cols] = proprio
        frames86 = (frames86 - self.f_mean) / self.f_std
        if self.freeze:
            with torch.no_grad():
                z = self.dyn.encode(frames86)
        else:
            z = self.dyn.encode(frames86)
        tail = x[:, -1, PROPRIO_DIM:]           # newest frame's goal dims
        return torch.cat([z, tail], dim=-1)


class ScaledLRPPO(PPO):
    """PPO whose LR schedule respects per-param-group ``lr_scale``
    (condition C: encoder fine-tunes slower than the policy head)."""

    def _update_learning_rate(self, optimizers) -> None:
        lr = self.lr_schedule(self._current_progress_remaining)
        self.logger.record("train/learning_rate", lr)
        if not isinstance(optimizers, (list, tuple)):
            optimizers = [optimizers]
        for opt in optimizers:
            for group in opt.param_groups:
                group["lr"] = lr * group.get("lr_scale", 1.0)

    def set_parameters(self, load_path_or_dict, exact_match: bool = True,
                       device="auto") -> None:
        # Checkpoint round-trip fix (tfwalk-joint1 diagnosis, 08-17):
        # checkpoints written after set_group_lrs() carry a TWO-group
        # Adam state ({heads, encoder w/ lr_scale}); a freshly
        # constructed model has SB3's single default group, so torch
        # refuses the optimizer restore ("loaded state dict has a
        # different number of parameter groups") and `.load()` dies.
        # Rebuild the matching group structure (same deterministic
        # rest/encoder split as set_group_lrs) before delegating.
        if isinstance(load_path_or_dict, dict):
            opt_state = load_path_or_dict.get("policy.optimizer")
            saved = (opt_state or {}).get("param_groups", [])
            have = self.policy.optimizer.param_groups
            if (len(saved) == 2 and len(have) != 2
                    and all("lr_scale" in g for g in saved)):
                base = next((g["lr"] for g in saved
                             if g.get("lr_scale", 1.0) == 1.0),
                            saved[0]["lr"])
                enc_scale = next((g["lr_scale"] for g in saved
                                  if g.get("lr_scale", 1.0) != 1.0), 1.0)
                set_group_lrs(self.policy, base, enc_scale)
        super().set_parameters(load_path_or_dict, exact_match=exact_match,
                               device=device)


def set_group_lrs(policy, base_lr: float, encoder_lr_scale: float):
    """Rebuild the policy optimizer with the features extractor's
    trainable params in a scaled-LR group (call after PPO init/load)."""
    enc_ids = {id(p) for p in policy.features_extractor.parameters()}
    enc = [p for p in policy.parameters()
           if id(p) in enc_ids and p.requires_grad]
    rest = [p for p in policy.parameters()
            if id(p) not in enc_ids and p.requires_grad]
    policy.optimizer = torch.optim.Adam(
        [{"params": rest, "lr": base_lr, "lr_scale": 1.0},
         {"params": enc, "lr": base_lr * encoder_lr_scale,
          "lr_scale": encoder_lr_scale}],
        lr=base_lr, eps=1e-5)
