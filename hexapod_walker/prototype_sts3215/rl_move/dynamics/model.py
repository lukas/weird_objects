"""Action-conditioned dynamics representation network.

    per-frame features (86 or the 59-dim obs subset)
          |
    frame MLP: Linear(d,256) SiLU Linear(256,256) SiLU
          |
    causal Transformer encoder over H history frames
          |
    z = Linear(256, 128)          <- the representation
          |
    causal action Transformer over the future action sequence; its token
    at step k conditions the horizon-k prediction (the model is
    ACTION-CONDITIONED: same z, different future actions -> different
    predictions).
          |
    short horizons (k <= short_max): MLP head -> raw physical state
        (normalized q, qd, tilt, gyro, accel = 44) + 6 contact logits,
        one output layer per horizon.
    privileged heads: z -> current privileged simulator truths, and
        [z, future actions] -> future privileged truths at every horizon.
        These are supervised targets only; privileged state is never an
        input channel.
    long horizons: MLP head -> future latent z (target =
        stop_gradient(encode(future_history))).

The legacy GRU architecture remains loadable for old checkpoints, but new
pretraining uses the Transformer architecture.
"""
from __future__ import annotations

import torch
import torch.nn as nn

from . import frames as fr


class DynamicsModel(nn.Module):
    def __init__(self, input_set: str = "full", z_dim: int = 128,
                 hidden: int = 256, act_hidden: int = 128,
                 gru_layers: int = 1,
                 history: int = 16, arch: str = "transformer",
                 tf_layers: int = 4, tf_heads: int = 8,
                 tf_ff: int = 1024, tf_dropout: float = 0.0,
                 horizons: tuple[int, ...] = (1, 2, 5, 10, 25),
                 short_max: int = 5, delta_state: bool = True,
                 predict_priv: bool = True):
        super().__init__()
        # Short-horizon heads predict the state DELTA from the newest
        # history frame (residual parametrization): persistence is the
        # zero-output baseline, so the head can only add value — this
        # is what lets the obs-input variant beat a strong full-history
        # linear predictor at k=1 where dynamics are locally linear.
        self.delta_state = bool(delta_state)
        self.predict_priv = bool(predict_priv)
        if input_set not in fr.INPUT_SETS:
            raise ValueError(f"input_set must be one of "
                             f"{sorted(fr.INPUT_SETS)}")
        self.input_set = input_set
        self.register_buffer(
            "input_idx",
            torch.as_tensor(fr.INPUT_SETS[input_set], dtype=torch.long))
        in_dim = len(fr.INPUT_SETS[input_set])
        self.horizons = tuple(sorted(int(k) for k in horizons))
        self.short = tuple(k for k in self.horizons if k <= short_max)
        self.long = tuple(k for k in self.horizons if k > short_max)
        self.z_dim = z_dim
        self.history = int(history)
        self.arch = str(arch)
        self.tf_layers = int(tf_layers)
        self.tf_heads = int(tf_heads)
        self.tf_ff = int(tf_ff)
        self.tf_dropout = float(tf_dropout)
        if self.arch not in ("transformer", "gru"):
            raise ValueError("arch must be 'transformer' or 'gru'")
        if hidden % self.tf_heads or act_hidden % self.tf_heads:
            raise ValueError("hidden and act_hidden must be divisible by "
                             "tf_heads")

        self.hidden = int(hidden)
        self.act_hidden = int(act_hidden)
        self.gru_layers = int(gru_layers)
        self.frame_mlp = nn.Sequential(
            nn.Linear(in_dim, hidden), nn.SiLU(),
            nn.Linear(hidden, hidden), nn.SiLU())
        if self.arch == "transformer":
            frame_layer = nn.TransformerEncoderLayer(
                d_model=hidden, nhead=self.tf_heads,
                dim_feedforward=self.tf_ff, dropout=self.tf_dropout,
                activation="gelu", batch_first=True, norm_first=True)
            self.frame_transformer = nn.TransformerEncoder(
                frame_layer, num_layers=self.tf_layers,
                norm=nn.LayerNorm(hidden))
            self.frame_pos = nn.Parameter(torch.zeros(1, self.history,
                                                       hidden))
            self.action_in = nn.Linear(fr.ACTION_DIM, act_hidden)
            action_layer = nn.TransformerEncoderLayer(
                d_model=act_hidden, nhead=self.tf_heads,
                dim_feedforward=self.tf_ff, dropout=self.tf_dropout,
                activation="gelu", batch_first=True, norm_first=True)
            self.action_transformer = nn.TransformerEncoder(
                action_layer, num_layers=self.tf_layers,
                norm=nn.LayerNorm(act_hidden))
            self.action_pos = nn.Parameter(torch.zeros(
                1, max(self.horizons), act_hidden))
            nn.init.normal_(self.frame_pos, std=0.02)
            nn.init.normal_(self.action_pos, std=0.02)
        else:
            self.gru = nn.GRU(hidden, hidden, num_layers=self.gru_layers,
                              batch_first=True)
            self.act_gru = nn.GRU(fr.ACTION_DIM, act_hidden,
                                  batch_first=True)
        self.to_z = nn.Linear(hidden, z_dim)

        head_in = z_dim + act_hidden
        self.short_trunk = nn.Sequential(
            nn.Linear(head_in, hidden), nn.SiLU())
        self.short_out = nn.ModuleDict({
            str(k): nn.Linear(hidden, fr.STATE_DIM + fr.N_FEET)
            for k in self.short})
        self.current_out = nn.ModuleDict({
            str(k): nn.Linear(hidden, fr.CURRENT_DIM) for k in self.short})
        self.contact_current_out = nn.Sequential(
            nn.Linear(z_dim, hidden), nn.SiLU(),
            nn.Linear(hidden, fr.N_FEET))
        self.current_current_out = nn.Sequential(
            nn.Linear(z_dim, hidden), nn.SiLU(),
            nn.Linear(hidden, fr.CURRENT_DIM))
        self.long_trunk = nn.Sequential(
            nn.Linear(head_in, hidden), nn.SiLU())
        self.long_out = nn.ModuleDict({
            str(k): nn.Linear(hidden, z_dim) for k in self.long})
        if self.predict_priv:
            self.priv_current_out = nn.Sequential(
                nn.Linear(z_dim, hidden), nn.SiLU(),
                nn.Linear(hidden, fr.PRIV_DIM))
            self.priv_trunk = nn.Sequential(
                nn.Linear(head_in, hidden), nn.SiLU())
            self.priv_out = nn.ModuleDict({
                str(k): nn.Linear(hidden, fr.PRIV_DIM)
                for k in self.horizons})

    def encode(self, hist: torch.Tensor) -> torch.Tensor:
        """(B, H, FRAME_DIM) normalized frames -> (B, z_dim)."""
        x = hist.index_select(-1, self.input_idx)
        feats = self.frame_mlp(x)
        if self.arch == "transformer":
            if feats.shape[1] > self.history:
                raise ValueError(f"history length {feats.shape[1]} exceeds "
                                 f"configured maximum {self.history}")
            feats = feats + self.frame_pos[:, :feats.shape[1]]
            encoded = self.frame_transformer(feats)
            summary = encoded[:, -1]
        else:
            _, h_n = self.gru(feats)
            summary = h_n[-1]
        return self.to_z(summary)

    def forward(self, hist: torch.Tensor, fut_actions: torch.Tensor
                ) -> dict:
        """Returns {"z", "state" {k: (B,44)}, "contact_logits"
        {k: (B,6)}, "z_pred" {k: (B,z)}}."""
        z = self.encode(hist)
        if self.arch == "transformer":
            length = fut_actions.shape[1]
            a = self.action_in(fut_actions) + self.action_pos[:, :length]
            causal = torch.triu(torch.ones(length, length, dtype=torch.bool,
                                            device=a.device), diagonal=1)
            a_seq = self.action_transformer(a, mask=causal)
        else:
            a_seq, _ = self.act_gru(fut_actions)
        base = hist[:, -1, fr.STATE_SLICE]       # newest frame's state
        out = {"z": z, "state": {}, "contact_logits": {},
               "current": {}, "z_pred": {},
               "contact_now_logits": self.contact_current_out(z),
               "current_now": self.current_current_out(z)}
        if self.predict_priv:
            out["priv_now"] = self.priv_current_out(z)
            out["priv"] = {}
        for k in self.short:
            h = self.short_trunk(torch.cat([z, a_seq[:, k - 1]], dim=-1))
            y = self.short_out[str(k)](h)
            state = y[:, :fr.STATE_DIM]
            if self.delta_state:
                state = state + base
            out["state"][k] = state
            out["contact_logits"][k] = y[:, fr.STATE_DIM:]
            out["current"][k] = self.current_out[str(k)](h)
        for k in self.long:
            h = self.long_trunk(torch.cat([z, a_seq[:, k - 1]], dim=-1))
            out["z_pred"][k] = self.long_out[str(k)](h)
        if self.predict_priv:
            for k in self.horizons:
                h = self.priv_trunk(torch.cat([z, a_seq[:, k - 1]],
                                              dim=-1))
                out["priv"][k] = self.priv_out[str(k)](h)
        return out

    def config(self) -> dict:
        return {"input_set": self.input_set, "z_dim": self.z_dim,
                "hidden": self.hidden, "act_hidden": self.act_hidden,
                "gru_layers": self.gru_layers,
                "history": self.history, "arch": self.arch,
                "tf_layers": self.tf_layers, "tf_heads": self.tf_heads,
                "tf_ff": self.tf_ff, "tf_dropout": self.tf_dropout,
                "horizons": list(self.horizons),
                "short_max": max(self.short) if self.short else 0,
                "delta_state": self.delta_state,
                "predict_priv": self.predict_priv}


# Per-group column indices WITHIN the 44-dim state target, for
# per-channel loss weighting and error reporting.
STATE_GROUPS = {
    "joint_pos": slice(0, 18),
    "joint_vel": slice(18, 36),
    "imu": slice(36, 44),        # tilt(2) + gyro(3) + accel(3)
}


def dynamics_loss(out: dict, batch_t: dict, lambdas: dict,
                  model: DynamicsModel) -> tuple[torch.Tensor, dict]:
    """Multi-horizon loss per the brief. ``batch_t`` holds torch
    tensors (hist, fut_actions, state{k}, contact{k}, fut_hist{k}).
    Latent targets are stop-gradient encodings of the future history.
    Returns (total, {metric: float})."""
    mse = nn.functional.mse_loss
    bce = nn.functional.binary_cross_entropy_with_logits
    total = out["z"].new_zeros(())
    logs: dict[str, float] = {}
    l_contact_now = bce(out["contact_now_logits"], batch_t["contact_now"])
    l_current_now = mse(out["current_now"], batch_t["current_now"])
    total = total + (lambdas.get("contact_current", 0.0) * l_contact_now
                     + lambdas.get("motor_current", 0.0) * l_current_now)
    logs["current/contact_bce"] = float(l_contact_now.detach())
    logs["current/motor_current"] = float(l_current_now.detach())
    for k in model.short:
        tgt = batch_t["state"][k]
        pred = out["state"][k]
        l_pos = mse(pred[:, STATE_GROUPS["joint_pos"]],
                    tgt[:, STATE_GROUPS["joint_pos"]])
        l_vel = mse(pred[:, STATE_GROUPS["joint_vel"]],
                    tgt[:, STATE_GROUPS["joint_vel"]])
        l_imu = mse(pred[:, STATE_GROUPS["imu"]],
                    tgt[:, STATE_GROUPS["imu"]])
        l_con = bce(out["contact_logits"][k], batch_t["contact"][k])
        l_cur = mse(out["current"][k], batch_t["current"][k])
        total = total + (lambdas["joint_pos"] * l_pos
                         + lambdas["joint_vel"] * l_vel
                         + lambdas["imu"] * l_imu
                         + lambdas["contact"] * l_con
                         + lambdas.get("motor_current", 0.0) * l_cur)
        logs[f"h{k}/joint_pos"] = float(l_pos.detach())
        logs[f"h{k}/joint_vel"] = float(l_vel.detach())
        logs[f"h{k}/imu"] = float(l_imu.detach())
        logs[f"h{k}/contact_bce"] = float(l_con.detach())
        logs[f"h{k}/motor_current"] = float(l_cur.detach())
    if model.predict_priv and "priv_now" in batch_t:
        mask = batch_t.get("priv_mask_now")
        if mask is None:
            mask = torch.ones_like(batch_t["priv_now"])
        def masked_mse(pred, target):
            return ((pred - target).square() * mask).sum() / mask.sum().clamp_min(1)
        l_priv_now = masked_mse(out["priv_now"], batch_t["priv_now"])
        total = total + lambdas.get("priv_current", 0.0) * l_priv_now
        logs["priv/current"] = float(l_priv_now.detach())
        for k in model.horizons:
            l_priv = masked_mse(out["priv"][k], batch_t["priv"][k])
            total = total + lambdas.get("priv_future", 0.0) * l_priv
            logs[f"h{k}/priv"] = float(l_priv.detach())
    for k in model.long:
        with torch.no_grad():
            z_tgt = model.encode(batch_t["fut_hist"][k])
        l_z = mse(out["z_pred"][k], z_tgt)
        total = total + lambdas["latent"] * l_z
        logs[f"h{k}/latent"] = float(l_z.detach())
    logs["total"] = float(total.detach())
    return total, logs
