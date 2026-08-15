"""train.py — dynamics-representation pretraining on collected shards.

    ../../.venv/bin/python -m rl_move.dynamics.train \
        --data rl_move/dynamics/datasets/v1 \
        --name dyn_v1 --steps 20000

Logs per-horizon train/val losses (CSV next to the checkpoint +
stdout), keeps the best-val checkpoint, and stores the normalization
stats and model config inside the .pt so eval/PPO wiring can rebuild
everything from one file. The persistence baseline's val loss is
printed alongside so collapse is visible immediately (a model that
never beats "predict no change" has learned nothing — do not connect
PPO to it; see rl_docs/DYNREP.md gate G1).

Optional --wandb logs to the campaign project with tag track:dynrep.
"""
from __future__ import annotations

import argparse
import csv
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
    DynamicsModel, dynamics_loss,
)

MODEL_DIR = ROOT / "rl_move" / "dynamics" / "models"


def _to_torch(batch: dict, device):
    import torch

    def t(x):
        return torch.as_tensor(x, device=device)
    return {"hist": t(batch["hist"]),
            "fut_actions": t(batch["fut_actions"]),
            "state": {k: t(v) for k, v in batch["state"].items()},
            "contact": {k: t(v) for k, v in batch["contact"].items()},
            "priv_now": t(batch["priv_now"]),
            "priv": {k: t(v) for k, v in batch["priv"].items()},
            "fut_hist": {k: t(v) for k, v in batch["fut_hist"].items()}}


def persistence_val_loss(sampler: dd.WindowSampler, n_windows: int,
                         batch: int) -> dict[str, float]:
    """State MSE of "predict the current state unchanged" on the val
    split — the collapse detector printed next to training logs."""
    sums: dict[str, float] = {}
    n = 0
    for b in sampler.val_batches(n_windows, batch):
        cur_state = b["hist"][:, -1, fr.STATE_SLICE]
        cur_priv = b["priv_now"]
        for k in sampler.horizons:
            err = float(np.mean((cur_state - b["state"][k]) ** 2))
            sums[f"h{k}/state"] = sums.get(f"h{k}/state", 0.0) + err
            perr = float(np.mean((cur_priv - b["priv"][k]) ** 2))
            sums[f"h{k}/priv"] = sums.get(f"h{k}/priv", 0.0) + perr
        n += 1
    return {key: v / max(n, 1) for key, v in sums.items()}


def evaluate(model, sampler: dd.WindowSampler, lambdas: dict, device,
             n_windows: int, batch: int) -> dict[str, float]:
    import torch
    model.eval()
    sums: dict[str, float] = {}
    n = 0
    with torch.no_grad():
        for b in sampler.val_batches(n_windows, batch):
            bt = _to_torch(b, device)
            out = model(bt["hist"], bt["fut_actions"])
            _, logs = dynamics_loss(out, bt, lambdas, model)
            for key, v in logs.items():
                sums[key] = sums.get(key, 0.0) + v
            n += 1
    model.train()
    return {key: v / max(n, 1) for key, v in sums.items()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", required=True)
    ap.add_argument("--name", default="dyn_v1")
    ap.add_argument("--steps", type=int, default=20000)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--history", type=int, default=16)
    ap.add_argument("--horizons", default="1,2,5,10,25")
    ap.add_argument("--short-max", type=int, default=5)
    ap.add_argument("--z-dim", type=int, default=128)
    # Scaling knobs (operator next-steps 08-13: representation scaling
    # curve small ~1M / medium ~5M / large ~15-20M x context x data).
    ap.add_argument("--hidden", type=int, default=256)
    ap.add_argument("--act-hidden", type=int, default=128)
    ap.add_argument("--gru-layers", type=int, default=1)
    ap.add_argument("--input-set", choices=sorted(fr.INPUT_SETS),
                    default="full")
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--lr-final-frac", type=float, default=1.0,
                    help="cosine-decay the LR to lr*frac by the last "
                         "step (1.0 = constant LR)")
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--warmup", type=int, default=500)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--val-every", type=int, default=1000)
    ap.add_argument("--val-windows", type=int, default=8192)
    ap.add_argument("--log-every", type=int, default=100)
    ap.add_argument("--lam-joint-pos", type=float, default=1.0)
    ap.add_argument("--lam-joint-vel", type=float, default=1.0)
    ap.add_argument("--lam-imu", type=float, default=1.0)
    ap.add_argument("--lam-contact", type=float, default=0.5)
    ap.add_argument("--lam-latent", type=float, default=1.0)
    ap.add_argument("--lam-priv-current", type=float, default=0.25)
    ap.add_argument("--lam-priv-future", type=float, default=0.25)
    ap.add_argument("--no-priv-heads", action="store_true",
                    help="disable privileged-truth prediction heads "
                         "(legacy ablation; inputs are never privileged)")
    ap.add_argument("--wandb", action="store_true")
    args = ap.parse_args()

    import torch
    torch.manual_seed(args.seed)
    device = ("mps" if torch.backends.mps.is_available()
              else "cuda" if torch.cuda.is_available() else "cpu")
    horizons = tuple(int(k) for k in args.horizons.split(","))
    lambdas = {"joint_pos": args.lam_joint_pos,
               "joint_vel": args.lam_joint_vel,
               "imu": args.lam_imu, "contact": args.lam_contact,
               "latent": args.lam_latent,
               "priv_current": args.lam_priv_current,
               "priv_future": args.lam_priv_future}

    eps = dd.load_dataset(ROOT / args.data)
    print(dd.describe(eps))
    stats = dd.compute_stats(eps)
    train_s = dd.WindowSampler(eps, stats, args.history, horizons,
                               val=False, seed=args.seed)
    val_s = dd.WindowSampler(eps, stats, args.history, horizons,
                             val=True, seed=args.seed)
    print(f"windows: train {len(train_s)}, val {len(val_s)}; "
          f"device {device}")

    pers = persistence_val_loss(val_s, args.val_windows, args.batch)
    pers_line = " ".join(f"{k}={v:.4f}" for k, v in sorted(pers.items()))
    print(f"persistence baseline (val state/priv MSE): {pers_line}")

    model = DynamicsModel(input_set=args.input_set, z_dim=args.z_dim,
                          hidden=args.hidden, act_hidden=args.act_hidden,
                          gru_layers=args.gru_layers,
                          horizons=horizons,
                          short_max=args.short_max,
                          predict_priv=not args.no_priv_heads).to(device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"model: {n_params / 1e6:.2f}M params, input_set="
          f"{args.input_set}, z={args.z_dim}, horizons={horizons}")
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr,
                            weight_decay=args.weight_decay)

    run = None
    if args.wandb:
        import wandb
        run = wandb.init(project="hexapod-balance", group="dynrep",
                         tags=["track:dynrep"], name=args.name,
                         config={**vars(args), "n_params": n_params,
                                 "dataset": dd.describe(eps)},
                         notes="WHAT THIS RUN IS TRYING TO LEARN: a "
                               "task-independent latent of the body's "
                               "dynamics via action-conditioned "
                               "multi-horizon prediction (dynrep v1).")

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    ckpt_path = MODEL_DIR / f"{args.name}.pt"
    log_path = MODEL_DIR / f"{args.name}_log.csv"
    log_f = open(log_path, "w", newline="")
    log_w = None
    best_val = float("inf")
    t0 = time.time()

    def save(path: Path):
        torch.save({"model": model.state_dict(),
                    "config": model.config(),
                    "stats": stats.to_dict(),
                    "history": args.history,
                    "lambdas": lambdas,
                    "layout_version": fr.LAYOUT_VERSION,
                    "args": vars(args)}, path)

    import math
    for step in range(1, args.steps + 1):
        if step <= args.warmup:
            lr_now = args.lr * step / args.warmup
        else:
            frac = (step - args.warmup) / max(args.steps - args.warmup, 1)
            lo = args.lr * args.lr_final_frac
            lr_now = lo + 0.5 * (args.lr - lo) * (1 + math.cos(
                math.pi * frac))
        for g in opt.param_groups:
            g["lr"] = lr_now
        bt = _to_torch(train_s.batch(args.batch), device)
        out = model(bt["hist"], bt["fut_actions"])
        loss, logs = dynamics_loss(out, bt, lambdas, model)
        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        opt.step()

        if step % args.log_every == 0:
            print(f"step {step:6d}  loss {logs['total']:.4f}  "
                  f"({(time.time() - t0):.0f}s)")
            if run is not None:
                run.log({f"train/{k}": v for k, v in logs.items()},
                        step=step)
        if step % args.val_every == 0 or step == args.steps:
            vlogs = evaluate(model, val_s, lambdas, device,
                             args.val_windows, args.batch)
            line = " ".join(f"{k}={v:.4f}"
                            for k, v in sorted(vlogs.items())
                            if k != "total")
            print(f"  val @ {step}: total={vlogs['total']:.4f}  {line}")
            row = {"step": step, **{f"val/{k}": v
                                    for k, v in vlogs.items()},
                   **{f"train/{k}": v for k, v in logs.items()}}
            if log_w is None:
                log_w = csv.DictWriter(log_f, fieldnames=list(row))
                log_w.writeheader()
            log_w.writerow(row)
            log_f.flush()
            if run is not None:
                run.log({f"val/{k}": v for k, v in vlogs.items()},
                        step=step)
            if vlogs["total"] < best_val:
                best_val = vlogs["total"]
                save(ckpt_path)
                print(f"  saved best -> {ckpt_path.name} "
                      f"(val {best_val:.4f})")

    save(MODEL_DIR / f"{args.name}_final.pt")
    log_f.close()
    if run is not None:
        run.finish()
    print(f"done in {(time.time() - t0) / 60:.1f} min; best val "
          f"{best_val:.4f}; checkpoints: {ckpt_path} + _final.pt; "
          f"log: {log_path}")
    print("next: python -m rl_move.dynamics.eval_model "
          f"--ckpt {ckpt_path.relative_to(ROOT)} --data {args.data}")


if __name__ == "__main__":
    main()
