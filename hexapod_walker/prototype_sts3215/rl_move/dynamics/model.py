"""model.py — v1 dynamics-representation network (deliberately small).

    per-frame features (86 or the 59-dim obs subset)
          |
    frame MLP: Linear(d,256) SiLU Linear(256,256) SiLU
          |
    GRU(256 -> 256)  over H history frames
          |
    z = Linear(256, 128)          <- the representation
          |
    action GRU(18 -> 128) over the future action sequence; its hidden
    state at step k conditions the horizon-k prediction (the model is
    ACTION-CONDITIONED: same z, different future actions -> different
    predictions).
          |
    short horizons (k <= short_max): MLP head -> raw physical state
        (normalized q, qd, tilt, gyro, accel = 44) + 6 contact logits,
        one output layer per horizon.
    long horizons: MLP head -> future latent z (target =
        stop_gradient(encode(future_history))).

~1M parameters. No transformer, no VAE, no Dreamer — see the brief's
"Do Not Do Yet" list (rl_docs/DYNREP.md).
"""
from __future__ import annotations

import torch
import torch.nn as nn

from . import frames as fr


class DynamicsModel(nn.Module):
    def __init__(self, input_set: str = "full", z_dim: int = 128,
                 hidden: int = 256, act_hidden: int = 128,
                 horizons: tuple[int, ...] = (1, 2, 5, 10, 25),
                 short_max: int = 5, delta_state: bool = True):
        super().__init__()
        # Short-horizon heads predict the state DELTA from the newest
        # history frame (residual parametrization): persistence is the
        # zero-output baseline, so the head can only add value — this
        # is what lets the obs-input variant beat a strong full-history
        # linear predictor at k=1 where dynamics are locally linear.
        self.delta_state = bool(delta_state)
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

        self.frame_mlp = nn.Sequential(
            nn.Linear(in_dim, hidden), nn.SiLU(),
            nn.Linear(hidden, hidden), nn.SiLU())
        self.gru = nn.GRU(hidden, hidden, batch_first=True)
        self.to_z = nn.Linear(hidden, z_dim)
        self.act_gru = nn.GRU(fr.ACTION_DIM, act_hidden, batch_first=True)

        head_in = z_dim + act_hidden
        self.short_trunk = nn.Sequential(
            nn.Linear(head_in, hidden), nn.SiLU())
        self.short_out = nn.ModuleDict({
            str(k): nn.Linear(hidden, fr.STATE_DIM + fr.N_FEET)
            for k in self.short})
        self.long_trunk = nn.Sequential(
            nn.Linear(head_in, hidden), nn.SiLU())
        self.long_out = nn.ModuleDict({
            str(k): nn.Linear(hidden, z_dim) for k in self.long})

    def encode(self, hist: torch.Tensor) -> torch.Tensor:
        """(B, H, FRAME_DIM) normalized frames -> (B, z_dim)."""
        x = hist.index_select(-1, self.input_idx)
        feats = self.frame_mlp(x)
        _, h_n = self.gru(feats)
        return self.to_z(h_n[-1])

    def forward(self, hist: torch.Tensor, fut_actions: torch.Tensor
                ) -> dict:
        """Returns {"z", "state" {k: (B,44)}, "contact_logits"
        {k: (B,6)}, "z_pred" {k: (B,z)}}."""
        z = self.encode(hist)
        a_seq, _ = self.act_gru(fut_actions)     # (B, Kmax, act_hidden)
        base = hist[:, -1, fr.STATE_SLICE]       # newest frame's state
        out = {"z": z, "state": {}, "contact_logits": {}, "z_pred": {}}
        for k in self.short:
            h = self.short_trunk(torch.cat([z, a_seq[:, k - 1]], dim=-1))
            y = self.short_out[str(k)](h)
            state = y[:, :fr.STATE_DIM]
            if self.delta_state:
                state = state + base
            out["state"][k] = state
            out["contact_logits"][k] = y[:, fr.STATE_DIM:]
        for k in self.long:
            h = self.long_trunk(torch.cat([z, a_seq[:, k - 1]], dim=-1))
            out["z_pred"][k] = self.long_out[str(k)](h)
        return out

    def config(self) -> dict:
        return {"input_set": self.input_set, "z_dim": self.z_dim,
                "horizons": list(self.horizons),
                "short_max": max(self.short) if self.short else 0,
                "delta_state": self.delta_state}


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
        total = total + (lambdas["joint_pos"] * l_pos
                         + lambdas["joint_vel"] * l_vel
                         + lambdas["imu"] * l_imu
                         + lambdas["contact"] * l_con)
        logs[f"h{k}/joint_pos"] = float(l_pos.detach())
        logs[f"h{k}/joint_vel"] = float(l_vel.detach())
        logs[f"h{k}/imu"] = float(l_imu.detach())
        logs[f"h{k}/contact_bce"] = float(l_con.detach())
    for k in model.long:
        with torch.no_grad():
            z_tgt = model.encode(batch_t["fut_hist"][k])
        l_z = mse(out["z_pred"][k], z_tgt)
        total = total + lambdas["latent"] * l_z
        logs[f"h{k}/latent"] = float(l_z.detach())
    logs["total"] = float(total.detach())
    return total, logs
