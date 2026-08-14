"""eval_model.py — held-out prediction quality vs the brief's baselines.

The v1 gate (rl_docs/DYNREP.md G1): before any PPO wiring, the neural
model must beat BOTH baselines on held-out windows at every horizon:

    persistence   predict the current state unchanged
    linear        ridge regression from [flattened history, flattened
                  future actions] -> future state (fit on train windows)

Reports, per horizon: normalized state MSE (model / persistence /
linear), physical errors (joint pos RMSE deg, joint vel RMSE deg/s,
tilt RMSE deg), contact accuracy + F1 (model vs persistence), and the
latent-prediction MSE against the "latent unchanged" reference.
JSON report -> rl_move/dynamics/logs/, table -> stdout.

    ../../.venv/bin/python -m rl_move.dynamics.eval_model \
        --ckpt rl_move/dynamics/models/dyn_v1.pt \
        --data rl_move/dynamics/datasets/v1

--dump-latents also writes val-split latent embeddings + episode labels
(actor, goal mode, termination, mean tilt/contact) for the organization
analysis the brief asks for (upright/fallen, stable/unstable clusters).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl_move.dynamics import data as dd          # noqa: E402
from rl_move.dynamics import frames as fr        # noqa: E402
from rl_move.dynamics.model import (             # noqa: E402
    STATE_GROUPS, DynamicsModel,
)

LOG_DIR = ROOT / "rl_move" / "dynamics" / "logs"
RAD2DEG = 180.0 / np.pi


def load_model(ckpt_path: Path):
    import torch
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    cfg = ckpt["config"]
    model = DynamicsModel(input_set=cfg["input_set"],
                          z_dim=cfg["z_dim"],
                          hidden=cfg.get("hidden", 256),
                          act_hidden=cfg.get("act_hidden", 128),
                          gru_layers=cfg.get("gru_layers", 1),
                          horizons=tuple(cfg["horizons"]),
                          short_max=cfg["short_max"],
                          delta_state=cfg.get("delta_state", False))
    model.load_state_dict(ckpt["model"])
    model.eval()
    stats = dd.Stats.from_dict(ckpt["stats"])
    return model, stats, ckpt


def fit_linear(train_s: dd.WindowSampler, horizons, n_rows: int,
               batch: int, input_idx: np.ndarray,
               ridge_rel: float = 1e-3):
    """Closed-form ridge: [hist flat, future actions flat, 1] ->
    [state(44), contact(6)] per horizon. The history features are
    restricted to ``input_idx`` — the SAME frame columns the neural
    model receives — so the baseline never has an information
    advantage (or handicap) over the model it gates. Features are
    ~unit variance (normalized frames, [-1,1] actions) so the ridge
    strength scales with the row count (lambda = ridge_rel * n).
    Solved in float64 — the ~1800-dim float32 normal equations are
    ill-conditioned."""
    Xs, Ys = [], {k: [] for k in horizons}
    got = 0
    while got < n_rows:
        b = train_s.batch(min(batch, n_rows - got))
        B = b["hist"].shape[0]
        Xs.append(np.concatenate(
            [b["hist"][:, :, input_idx].reshape(B, -1),
             b["fut_actions"].reshape(B, -1)], axis=1))
        for k in horizons:
            Ys[k].append(np.concatenate([b["state"][k],
                                         b["contact"][k]], axis=1))
        got += B
    X = np.concatenate(Xs).astype(np.float64)
    X = np.concatenate([X, np.ones((len(X), 1))], axis=1)
    lam = ridge_rel * len(X)
    XtX = X.T @ X + lam * np.eye(X.shape[1])
    W = {}
    for k in horizons:
        Y = np.concatenate(Ys[k]).astype(np.float64)
        W[k] = np.linalg.solve(XtX, X.T @ Y).astype(np.float32)
    return W


def _linear_predict(W: np.ndarray, b: dict,
                    input_idx: np.ndarray) -> np.ndarray:
    B = b["hist"].shape[0]
    X = np.concatenate([b["hist"][:, :, input_idx].reshape(B, -1),
                        b["fut_actions"].reshape(B, -1),
                        np.ones((B, 1), dtype=np.float32)], axis=1)
    return X @ W


def _phys_errors(pred_n: np.ndarray, tgt_n: np.ndarray,
                 stats: dd.Stats) -> dict[str, float]:
    """Un-normalize and report physical RMSEs."""
    std = stats.std[fr.STATE_SLICE]
    err = (pred_n - tgt_n) * std
    q = err[:, STATE_GROUPS["joint_pos"]]
    qd = err[:, STATE_GROUPS["joint_vel"]]
    tilt = err[:, 36:38]
    return {
        "joint_pos_rmse_deg": float(np.sqrt(np.mean(q ** 2)) * RAD2DEG),
        "joint_vel_rmse_deg_s": float(np.sqrt(np.mean(qd ** 2))
                                      * RAD2DEG),
        "tilt_rmse_deg": float(np.sqrt(np.mean(tilt ** 2)) * RAD2DEG),
    }


def _contact_metrics(pred: np.ndarray, tgt: np.ndarray) -> dict:
    tp = float(np.sum((pred > 0.5) & (tgt > 0.5)))
    fp = float(np.sum((pred > 0.5) & (tgt <= 0.5)))
    fn = float(np.sum((pred <= 0.5) & (tgt > 0.5)))
    acc = float(np.mean((pred > 0.5) == (tgt > 0.5)))
    f1 = 2 * tp / max(2 * tp + fp + fn, 1e-9)
    return {"contact_acc": acc, "contact_f1": f1}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--val-windows", type=int, default=16384)
    ap.add_argument("--linear-rows", type=int, default=50000)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--dump-latents", action="store_true")
    ap.add_argument("--k1-ridge-tol", type=float, default=0.05,
                    help="G1.1 (revised gate, recorded 2026-08-13): at "
                         "the shortest horizon the model may be within "
                         "this relative margin of the matched ridge "
                         "baseline (locally-linear 1-step dynamics are "
                         "a ridge home game); every other horizon must "
                         "still beat both baselines outright.")
    args = ap.parse_args()

    import torch

    model, stats, ckpt = load_model(ROOT / args.ckpt)
    history = int(ckpt["history"])
    horizons = tuple(model.horizons)
    eps = dd.load_dataset(ROOT / args.data)
    print(dd.describe(eps))
    train_s = dd.WindowSampler(eps, stats, history, horizons, val=False,
                               seed=1)
    val_s = dd.WindowSampler(eps, stats, history, horizons, val=True,
                             seed=1)
    input_idx = fr.INPUT_SETS[ckpt["config"]["input_set"]]
    print(f"fitting linear baseline on {args.linear_rows} train "
          f"windows (input set '{ckpt['config']['input_set']}', "
          f"{len(input_idx)} frame dims — matched to the model) ...")
    W = fit_linear(train_s, horizons, args.linear_rows, args.batch,
                   input_idx)

    acc: dict = {k: {"model_se": 0.0, "pers_se": 0.0, "lin_se": 0.0,
                     "n": 0,
                     "model_pred": [], "pers_pred": [], "lin_pred": [],
                     "tgt": [], "con_pred": [], "con_pers": [],
                     "con_tgt": [], "z_se": 0.0, "z_pers_se": 0.0,
                     "z_n": 0}
                 for k in horizons}
    lat_z, lat_labels = [], []
    for b in val_s.val_batches(args.val_windows, args.batch):
        bt_hist = torch.as_tensor(b["hist"])
        bt_act = torch.as_tensor(b["fut_actions"])
        with torch.no_grad():
            out = model(bt_hist, bt_act)
        last_state = b["hist"][:, -1, fr.STATE_SLICE]
        last_contact = (b["hist"][:, -1, fr.CONTACT_SLICE]
                        * stats.std[fr.CONTACT_SLICE]
                        + stats.mean[fr.CONTACT_SLICE]
                        ) > fr.CONTACT_THRESH_N
        for k in horizons:
            a = acc[k]
            tgt = b["state"][k]
            lin = _linear_predict(W[k], b, input_idx)
            a["n"] += len(tgt)
            a["pers_se"] += float(np.sum((last_state - tgt) ** 2))
            a["lin_se"] += float(
                np.sum((lin[:, :fr.STATE_DIM] - tgt) ** 2))
            a["pers_pred"].append(last_state)
            a["lin_pred"].append(lin[:, :fr.STATE_DIM])
            a["tgt"].append(tgt)
            a["con_tgt"].append(b["contact"][k])
            a["con_pers"].append(last_contact.astype(np.float32))
            if k in model.short:
                pred = out["state"][k].numpy()
                a["model_se"] += float(np.sum((pred - tgt) ** 2))
                a["model_pred"].append(pred)
                a["con_pred"].append(
                    torch.sigmoid(out["contact_logits"][k]).numpy())
            else:
                with torch.no_grad():
                    z_tgt = model.encode(
                        torch.as_tensor(b["fut_hist"][k])).numpy()
                zp = out["z_pred"][k].numpy()
                z_now = out["z"].numpy()
                a["z_se"] += float(np.sum((zp - z_tgt) ** 2))
                a["z_pers_se"] += float(np.sum((z_now - z_tgt) ** 2))
                a["z_n"] += len(zp)
        if args.dump_latents:
            lat_z.append(out["z"].numpy())
            fN = b["hist"][:, -1]
            lat_labels.append(np.stack([
                fN[:, 36] * stats.std[36] + stats.mean[36],   # roll
                fN[:, 37] * stats.std[37] + stats.mean[37],   # pitch
                np.sum((b["hist"][:, -1, fr.CONTACT_SLICE]
                        * stats.std[fr.CONTACT_SLICE]
                        + stats.mean[fr.CONTACT_SLICE])
                       > fr.CONTACT_THRESH_N, axis=1),          # n feet on
            ], axis=1))

    report: dict = {"ckpt": args.ckpt, "data": args.data,
                    "val_windows": args.val_windows,
                    "config": ckpt["config"], "horizons": {}}
    dim = fr.STATE_DIM
    k1 = min(model.short) if model.short else None
    print(f"\n{'k':>4} {'model':>10} {'persist':>10} {'linear':>10} "
          f"{'beats?':>7}   physical (model)")
    for k in horizons:
        a = acc[k]
        n = a["n"] * dim
        pers = a["pers_se"] / n
        lin = a["lin_se"] / n
        row: dict = {"persistence_mse": pers, "linear_mse": lin}
        if k in model.short:
            mdl = a["model_se"] / n
            row["model_mse"] = mdl
            row.update({f"model_{kk}": v for kk, v in _phys_errors(
                np.concatenate(a["model_pred"]),
                np.concatenate(a["tgt"]), stats).items()})
            row.update({f"persistence_{kk}": v for kk, v in _phys_errors(
                np.concatenate(a["pers_pred"]),
                np.concatenate(a["tgt"]), stats).items()})
            row.update({f"model_{kk}": v for kk, v in _contact_metrics(
                np.concatenate(a["con_pred"]),
                np.concatenate(a["con_tgt"])).items()})
            row.update({f"persistence_{kk}": v
                        for kk, v in _contact_metrics(
                            np.concatenate(a["con_pers"]),
                            np.concatenate(a["con_tgt"])).items()})
            beats = mdl < pers and mdl < lin
            row["beats_baselines"] = beats
            # G1.1: shortest horizon gets the ridge tolerance.
            if k == k1:
                row["beats_revised"] = (
                    mdl < pers and mdl <= lin * (1.0 + args.k1_ridge_tol))
                row["k1_ridge_margin"] = float(mdl / lin - 1.0)
            else:
                row["beats_revised"] = beats
            print(f"{k:>4} {mdl:>10.4f} {pers:>10.4f} {lin:>10.4f} "
                  f"{'YES' if beats else 'NO':>7}   "
                  f"q {row['model_joint_pos_rmse_deg']:.2f}deg  "
                  f"tilt {row['model_tilt_rmse_deg']:.2f}deg  "
                  f"contact {row['model_contact_acc']:.3f}")
        else:
            zn = max(a["z_n"] * model.z_dim, 1)
            row["latent_mse"] = a["z_se"] / zn
            row["latent_persistence_mse"] = a["z_pers_se"] / zn
            beats = row["latent_mse"] < row["latent_persistence_mse"]
            row["beats_baselines"] = beats
            row["beats_revised"] = beats
            print(f"{k:>4} {'(latent)':>10} {'':>10} {'':>10} "
                  f"{'YES' if beats else 'NO':>7}   "
                  f"z-mse {row['latent_mse']:.4f} vs unchanged-z "
                  f"{row['latent_persistence_mse']:.4f}  "
                  f"[state ref: pers {pers:.4f} lin {lin:.4f}]")
        report["horizons"][k] = row
    report["gate_g1_pass"] = all(
        report["horizons"][k]["beats_baselines"] for k in horizons)
    report["gate_g1_1_pass"] = all(
        report["horizons"][k]["beats_revised"] for k in horizons)
    report["g1_1_k1_ridge_tol"] = args.k1_ridge_tol
    print(f"\nGATE G1 (legacy: beat persistence + linear at every "
          f"horizon): {'PASS' if report['gate_g1_pass'] else 'FAIL'}")
    print(f"GATE G1.1 (revised 2026-08-13: k={k1} within "
          f"{args.k1_ridge_tol:.0%} of matched ridge, all other "
          f"horizons beat both baselines): "
          f"{'PASS' if report['gate_g1_1_pass'] else 'FAIL'}")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_path = LOG_DIR / f"eval_{Path(args.ckpt).stem}_{ts}.json"
    out_path.write_text(json.dumps(report, indent=2) + "\n")
    print(f"report: {out_path}")
    if args.dump_latents:
        lat_path = LOG_DIR / f"latents_{Path(args.ckpt).stem}_{ts}.npz"
        np.savez_compressed(
            lat_path, z=np.concatenate(lat_z),
            labels=np.concatenate(lat_labels),
            label_names=np.array(["roll_rad", "pitch_rad", "n_feet_on"]))
        print(f"latents: {lat_path}")


if __name__ == "__main__":
    main()
