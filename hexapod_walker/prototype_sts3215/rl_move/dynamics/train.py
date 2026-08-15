"""CUDA-first Transformer pretraining for the dynamics representation."""
from __future__ import annotations

import argparse
import copy
import csv
import math
import os
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
    STATE_GROUPS, DynamicsModel, dynamics_loss,
)

MODEL_DIR = ROOT / "rl_move" / "dynamics" / "models"
LOG_DIR = ROOT / "rl_move" / "dynamics" / "logs"
WANDB_ENV_FILE = ROOT / "rl_move" / "sim" / "wandb.env"
RAD2DEG = 180.0 / math.pi


def _to_torch(value, device):
    import torch

    if isinstance(value, dict):
        return {k: _to_torch(v, device) for k, v in value.items()}
    if torch.is_tensor(value):
        return value.to(device)
    return torch.as_tensor(value, device=device)


def _contact_metrics(logits, target) -> dict[str, float]:
    import torch

    prob = logits.sigmoid()
    pred = prob > 0.5
    truth = target > 0.5
    tp = (pred & truth).sum().float()
    fp = (pred & ~truth).sum().float()
    fn = (~pred & truth).sum().float()
    ece = prob.new_zeros(())
    for lo in torch.linspace(0.0, 0.9, 10, device=prob.device):
        mask = (prob >= lo) & (prob < lo + 0.1)
        if mask.any():
            ece += mask.float().mean() * torch.abs(
                prob[mask].mean() - truth[mask].float().mean())
    return {
        "contact_acc": float((pred == truth).float().mean().detach()),
        "contact_f1": float((2 * tp / (2 * tp + fp + fn).clamp_min(1)).detach()),
        "contact_brier": float(((prob - target).square()).mean().detach()),
        "contact_ece": float(ece.detach()),
    }


def _physical_metrics(out: dict, batch: dict, stats: dd.Stats,
                      model: DynamicsModel) -> dict[str, float]:
    """Interpretable metrics, calculated on the training device."""
    import torch

    fstd = torch.as_tensor(stats.std, device=out["z"].device)
    pmean = torch.as_tensor(stats.priv_mean, device=out["z"].device)
    pstd = torch.as_tensor(stats.priv_std, device=out["z"].device)
    metrics: dict[str, float] = {}

    def rmse(x):
        return float(torch.sqrt(torch.mean(x.square())).detach())

    def add_contacts(prefix, logits, target):
        for key, value in _contact_metrics(logits, target).items():
            metrics[f"{prefix}/{key}"] = value

    def add_priv(prefix, pred_n, tgt_n, mask):
        pred = pred_n * pstd + pmean
        tgt = tgt_n * pstd + pmean
        err = pred - tgt

        def available(cols):
            return bool(torch.all(mask[:, cols] > 0.5))

        if available([0, 1]):
            metrics[f"{prefix}/vxy_rmse_m_s"] = rmse(err[:, [0, 1]])
        if available([2]):
            metrics[f"{prefix}/vz_rmse_m_s"] = rmse(err[:, 2])
        if available([3]):
            metrics[f"{prefix}/wz_rmse_rad_s"] = rmse(err[:, 3])
        if available([4]):
            metrics[f"{prefix}/chassis_z_rmse_m"] = rmse(err[:, 4])
        if available([5, 6]):
            ph = torch.atan2(pred[:, 5], pred[:, 6])
            th = torch.atan2(tgt[:, 5], tgt[:, 6])
            dh = torch.atan2(torch.sin(ph - th), torch.cos(ph - th))
            metrics[f"{prefix}/heading_rmse_deg"] = rmse(dh) * RAD2DEG
        if available([10]):
            metrics[f"{prefix}/along_rmse_m_s"] = rmse(err[:, 10])
        if available([11]):
            metrics[f"{prefix}/cross_rmse_m_s"] = rmse(err[:, 11])

    add_contacts("current", out["contact_now_logits"], batch["contact_now"])
    cur_err = ((out["current_now"] - batch["current_now"])
               * fstd[fr.CURRENT_SLICE])
    metrics["current/motor_current_rmse_a"] = rmse(cur_err)
    if model.predict_priv:
        add_priv("current", out["priv_now"], batch["priv_now"],
                 batch["priv_mask_now"])

    for k in model.short:
        err = ((out["state"][k] - batch["state"][k])
               * fstd[fr.STATE_SLICE])
        metrics[f"h{k}/joint_pos_rmse_deg"] = (
            rmse(err[:, STATE_GROUPS["joint_pos"]]) * RAD2DEG)
        metrics[f"h{k}/joint_vel_rmse_deg_s"] = (
            rmse(err[:, STATE_GROUPS["joint_vel"]]) * RAD2DEG)
        metrics[f"h{k}/tilt_rmse_deg"] = rmse(err[:, 36:38]) * RAD2DEG
        cur_err = ((out["current"][k] - batch["current"][k])
                   * fstd[fr.CURRENT_SLICE])
        metrics[f"h{k}/motor_current_rmse_a"] = rmse(cur_err)
        add_contacts(f"h{k}", out["contact_logits"][k],
                     batch["contact"][k])
    if model.predict_priv:
        for k in model.horizons:
            add_priv(f"h{k}", out["priv"][k], batch["priv"][k],
                     batch["priv_mask_now"])
    return metrics


def persistence_val_loss(sampler, n_windows: int,
                         batch_size: int) -> dict[str, float]:
    import torch

    sums: dict[str, float] = {}
    n = 0
    for b in sampler.val_batches(n_windows, batch_size):
        cur_state = b["hist"][:, -1, fr.STATE_SLICE]
        cur_priv = b["priv_now"]
        is_torch = torch.is_tensor(cur_state)
        for k in sampler.horizons:
            state_err = (cur_state - b["state"][k]) ** 2
            priv_err = (cur_priv - b["priv"][k]) ** 2
            mask = b["priv_mask_now"]
            if is_torch:
                serr = float(state_err.mean())
                perr = float((priv_err * mask).sum()
                             / mask.sum().clamp_min(1))
            else:
                serr = float(np.mean(state_err))
                perr = float(np.sum(priv_err * mask) / max(mask.sum(), 1))
            sums[f"h{k}/state"] = sums.get(f"h{k}/state", 0.0) + serr
            sums[f"h{k}/priv"] = sums.get(f"h{k}/priv", 0.0) + perr
        n += 1
    return {key: value / max(n, 1) for key, value in sums.items()}


def evaluate(model, target_model, sampler, lambdas: dict, device,
             stats: dd.Stats,
             n_windows: int, batch_size: int) -> dict[str, float]:
    import torch

    model.eval()
    sums: dict[str, float] = {}
    n = 0
    with torch.no_grad():
        for batch in sampler.val_batches(n_windows, batch_size):
            bt = _to_torch(batch, device)
            out = model(bt["hist"], bt["fut_actions"])
            _, logs = dynamics_loss(out, bt, lambdas, model, target_model)
            logs.update(_physical_metrics(out, bt, stats, model))
            for key, value in logs.items():
                sums[key] = sums.get(key, 0.0) + value
            n += 1
    model.train()
    return {key: value / max(n, 1) for key, value in sums.items()}


def _load_wandb_env() -> None:
    if not WANDB_ENV_FILE.is_file():
        return
    for raw in WANDB_ENV_FILE.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        if key.strip() and value.strip():
            os.environ.setdefault(key.strip(), value.strip())


def _init_wandb(args, config: dict):
    if args.no_wandb:
        return None
    _load_wandb_env()
    try:
        import wandb
    except ImportError as exc:
        raise RuntimeError("W&B is required; install wandb or use "
                           "--no-wandb for a smoke test") from exc
    has_key = bool(os.environ.get("WANDB_API_KEY"))
    if not has_key:
        try:
            has_key = bool(wandb.api.api_key)
        except Exception:
            has_key = False
    if not has_key:
        raise RuntimeError(f"W&B is required but no key was found in "
                           f"{WANDB_ENV_FILE}")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    run = wandb.init(
        entity=os.environ.get("WANDB_ENTITY", "l2k2"),
        project=os.environ.get("WANDB_PROJECT", "hexapod-balance"),
        dir=str(LOG_DIR), group="dynrep-pretrain", job_type="phase-1",
        name=args.name, config=config,
        notes=(args.notes or "Transformer phase-1 representation learning: "
               "infer current body state and predict future servo, contact, "
               "velocity, and heading state under future actions."))
    run.define_metric("global_step")
    run.define_metric("train/*", step_metric="global_step")
    run.define_metric("val/*", step_metric="global_step")
    run.define_metric("data/*", summary="last")
    print(f"[wandb] logging to {run.url or 'offline run dir'}")
    return run


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", required=True)
    ap.add_argument("--name", default="dyn_transformer_v1")
    ap.add_argument("--notes", default="")
    ap.add_argument("--steps", type=int, default=40000)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--history", type=int, default=16)
    ap.add_argument("--horizons", default="1,2,5,10,25")
    ap.add_argument("--short-max", type=int, default=5)
    ap.add_argument("--z-dim", type=int, default=256)
    ap.add_argument("--hidden", type=int, default=512)
    ap.add_argument("--act-hidden", type=int, default=256)
    ap.add_argument("--arch", choices=("transformer", "gru"),
                    default="transformer")
    ap.add_argument("--gru-layers", type=int, default=1)
    ap.add_argument("--tf-layers", type=int, default=4)
    ap.add_argument("--tf-heads", type=int, default=8)
    ap.add_argument("--tf-ff", type=int, default=1024)
    ap.add_argument("--tf-dropout", type=float, default=0.0)
    ap.add_argument("--input-set", choices=sorted(fr.INPUT_SETS),
                    default="obs")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--allow-cpu", action="store_true",
                    help="smoke tests only; production pretraining is CUDA")
    ap.add_argument("--allow-legacy-priv", action="store_true",
                    help="allow old four-label shards (missing metrics masked)")
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--lr-final-frac", type=float, default=0.1)
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--target-ema", type=float, default=0.995,
                    help="EMA decay for stable long-horizon latent targets")
    ap.add_argument("--warmup", type=int, default=500)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--val-every", type=int, default=1000)
    ap.add_argument("--val-windows", type=int, default=8192)
    ap.add_argument("--log-every", type=int, default=100)
    ap.add_argument("--max-window-reuse", type=float, default=2.0,
                    help="hard cap on planned optimizer draws / distinct "
                         "training-window centers")
    ap.add_argument("--allow-excess-window-reuse", action="store_true",
                    help="explicit smoke/debug override for the data budget")
    ap.add_argument("--lam-joint-pos", type=float, default=1.0)
    ap.add_argument("--lam-joint-vel", type=float, default=1.0)
    ap.add_argument("--lam-imu", type=float, default=1.0)
    ap.add_argument("--lam-contact", type=float, default=0.5)
    ap.add_argument("--lam-contact-current", type=float, default=0.5)
    ap.add_argument("--lam-motor-current", type=float, default=0.5)
    ap.add_argument("--lam-latent", type=float, default=1.0)
    ap.add_argument("--lam-priv-current", type=float, default=0.25)
    ap.add_argument("--lam-priv-future", type=float, default=0.25)
    ap.add_argument("--no-priv-heads", action="store_true")
    ap.add_argument("--no-wandb", action="store_true")
    args = ap.parse_args()

    import torch

    torch.manual_seed(args.seed)
    if args.device != "cuda" and not args.allow_cpu:
        raise RuntimeError("production pretraining requires --device cuda; "
                           "use --allow-cpu only for a smoke test")
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but torch.cuda.is_available() "
                           "is false; use the GPU torch environment")
    device = torch.device(args.device)
    if device.type == "cuda":
        torch.set_float32_matmul_precision("high")
        gpu_name = torch.cuda.get_device_name(device)
    else:
        gpu_name = "none (CPU smoke override)"

    horizons = tuple(int(k) for k in args.horizons.split(","))
    lambdas = {
        "joint_pos": args.lam_joint_pos, "joint_vel": args.lam_joint_vel,
        "imu": args.lam_imu, "contact": args.lam_contact,
        "contact_current": args.lam_contact_current,
        "motor_current": args.lam_motor_current, "latent": args.lam_latent,
        "priv_current": args.lam_priv_current,
        "priv_future": args.lam_priv_future,
    }

    eps = dd.load_dataset(ROOT / args.data)
    coverage = dd.full_priv_fraction(eps)
    if coverage < 1.0 and not args.allow_legacy_priv:
        raise RuntimeError(f"dataset has full 14-label supervision for only "
                           f"{coverage:.1%} of episodes; collect with current "
                           "frames.py or pass --allow-legacy-priv for analysis")
    print(dd.describe(eps))
    print(f"privileged-label coverage: {coverage:.1%}")
    stats = dd.compute_stats(eps)
    budget = dd.window_budget(eps, args.history, horizons)
    planned_draws = args.steps * args.batch
    planned_reuse = planned_draws / max(budget["train"], 1)
    if (planned_reuse > args.max_window_reuse
            and not args.allow_excess_window_reuse):
        required = math.ceil(planned_draws / args.max_window_reuse)
        raise RuntimeError(
            f"DATA BUDGET REFUSED: {planned_draws:,} optimizer draws over "
            f"only {budget['train']:,} distinct train-window centers = "
            f"{planned_reuse:.1f}x planned reuse (limit "
            f"{args.max_window_reuse:.1f}x). Collect at least "
            f"{required:,} train windows with collect_mjx, or use "
            "--allow-excess-window-reuse only for an intentional smoke/debug "
            "run. Model capacity is not the problem; the old run starved a "
            "13.6M-parameter Transformer on a frozen tiny corpus.")
    sampler_cls = dd.GpuWindowSampler if device.type == "cuda" else dd.WindowSampler
    sampler_kw = {"device": device} if device.type == "cuda" else {}
    train_s = sampler_cls(eps, stats, args.history, horizons, val=False,
                          seed=args.seed, **sampler_kw)
    val_s = sampler_cls(eps, stats, args.history, horizons, val=True,
                        seed=args.seed, **sampler_kw)
    print(f"windows: train {len(train_s)}, val {len(val_s)}; "
          f"planned draws={planned_draws:,}; reuse={planned_reuse:.2f}x; "
          f"device={device}; gpu={gpu_name}; "
          f"gpu_resident_batches={device.type == 'cuda'}")

    pers = persistence_val_loss(val_s, args.val_windows, args.batch)
    print("persistence baseline: " + " ".join(
        f"{key}={value:.4f}" for key, value in sorted(pers.items())))

    model = DynamicsModel(
        input_set=args.input_set, z_dim=args.z_dim, hidden=args.hidden,
        act_hidden=args.act_hidden, gru_layers=args.gru_layers,
        history=args.history, arch=args.arch, tf_layers=args.tf_layers,
        tf_heads=args.tf_heads, tf_ff=args.tf_ff,
        tf_dropout=args.tf_dropout, horizons=horizons,
        short_max=args.short_max, predict_priv=not args.no_priv_heads,
    ).to(device)
    if not 0.0 <= args.target_ema < 1.0:
        raise ValueError("--target-ema must be in [0, 1)")
    target_model = copy.deepcopy(model).eval()
    target_model.requires_grad_(False)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"model: {n_params / 1e6:.2f}M params; arch={args.arch}; "
          f"input_set={args.input_set}; z={args.z_dim}")
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr,
                            weight_decay=args.weight_decay)

    config = {**vars(args), "n_params": n_params,
              "dataset_summary": dd.describe(eps),
              "full_priv_fraction": coverage, "torch": torch.__version__,
              "cuda": torch.version.cuda, "gpu": gpu_name,
              "gpu_resident_batches": device.type == "cuda",
              "data_train_windows": budget["train"],
              "data_val_windows": budget["val"],
              "data_planned_draws": planned_draws,
              "data_planned_window_reuse": planned_reuse}
    run = _init_wandb(args, config)
    if run is not None:
        run.log({"global_step": 0,
                 "data/train_windows": budget["train"],
                 "data/val_windows": budget["val"],
                 "data/planned_draws": planned_draws,
                 "data/planned_window_reuse": planned_reuse})

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    ckpt_path = MODEL_DIR / f"{args.name}.pt"
    log_path = MODEL_DIR / f"{args.name}_log.csv"
    log_f = open(log_path, "w", newline="")
    log_w = None
    best_val = float("inf")
    t0 = time.time()

    def save(path: Path):
        torch.save({"model": model.state_dict(), "config": model.config(),
                    "stats": stats.to_dict(), "history": args.history,
                    "lambdas": lambdas,
                    "layout_version": fr.LAYOUT_VERSION,
                    "target_model": target_model.state_dict(),
                    "target_ema": args.target_ema,
                    "args": vars(args)}, path)

    for step in range(1, args.steps + 1):
        if step <= args.warmup:
            lr_now = args.lr * step / max(args.warmup, 1)
        else:
            frac = (step - args.warmup) / max(args.steps - args.warmup, 1)
            lo = args.lr * args.lr_final_frac
            lr_now = lo + 0.5 * (args.lr - lo) * (1 + math.cos(math.pi * frac))
        for group in opt.param_groups:
            group["lr"] = lr_now
        bt = _to_torch(train_s.batch(args.batch), device)
        with torch.autocast(device_type="cuda", dtype=torch.bfloat16,
                            enabled=device.type == "cuda"):
            out = model(bt["hist"], bt["fut_actions"])
            loss, logs = dynamics_loss(out, bt, lambdas, model, target_model)
        opt.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        opt.step()
        with torch.no_grad():
            for target, online in zip(target_model.parameters(),
                                      model.parameters()):
                target.lerp_(online, 1.0 - args.target_ema)

        if step % args.log_every == 0:
            physical = _physical_metrics(out, bt, stats, model)
            elapsed = time.time() - t0
            print(f"step {step:6d} loss {logs['total']:.4f} "
                  f"({elapsed:.0f}s, {step / max(elapsed, 1):.1f} step/s)")
            if run is not None:
                payload = {f"train/{key}": value
                           for key, value in {**logs, **physical}.items()}
                payload.update({"global_step": step, "train/lr": lr_now,
                                "system/gpu_memory_gb":
                                torch.cuda.max_memory_allocated() / 1e9
                                if device.type == "cuda" else 0.0})
                run.log(payload)

        if step % args.val_every == 0 or step == args.steps:
            vlogs = evaluate(model, target_model, val_s, lambdas,
                             device, stats,
                             args.val_windows, args.batch)
            print(f"  val @ {step}: total={vlogs['total']:.4f}; "
                  f"vxy={vlogs.get('current/vxy_rmse_m_s', float('nan')):.4f} "
                  f"m/s; heading={vlogs.get('current/heading_rmse_deg', float('nan')):.2f} "
                  f"deg; contact_f1={vlogs['current/contact_f1']:.3f}; "
                  f"current={vlogs['current/motor_current_rmse_a']:.3f} A")
            row = {"step": step, **{f"val/{key}": value
                                     for key, value in vlogs.items()}}
            if log_w is None:
                log_w = csv.DictWriter(log_f, fieldnames=list(row))
                log_w.writeheader()
            log_w.writerow(row)
            log_f.flush()
            if run is not None:
                run.log({"global_step": step,
                         **{f"val/{key}": value
                            for key, value in vlogs.items()}})
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
          f"{best_val:.4f}; checkpoints: {ckpt_path} + _final.pt")


if __name__ == "__main__":
    main()
