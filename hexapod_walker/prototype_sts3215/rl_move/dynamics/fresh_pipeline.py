"""Orchestrator entry point: GPU data generation, then Transformer train.

The two stages use separate W&B runs: ``<name>-data`` exposes collection
throughput/fresh-window progress immediately, then ``<name>`` is the actual
phase-1 training run.  Training starts only after the requested reuse budget
has been met and rechecks that budget itself.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--name", required=True)
    ap.add_argument("--steps", type=int, required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--history", type=int, default=16)
    ap.add_argument("--horizons", default="1,2,5,10,25")
    ap.add_argument("--max-window-reuse", type=float, default=2.0)
    ap.add_argument("--collect-n-envs", type=int, default=2048)
    ap.add_argument("--collect-host-workers", type=int, default=24)
    ap.add_argument("--collect-pool-per-env", type=int, default=2)
    ap.add_argument("--collect-shard-episodes", type=int, default=2048)
    ap.add_argument("--collect-impl", choices=("auto", "warp", "default"),
                    default="warp")
    ap.add_argument("--collect-seed", type=int, default=0)
    ap.add_argument("--collect-episodes", type=int, default=0)
    ap.add_argument("--collect-compressed", action="store_true")
    ap.add_argument("--notes", default="")
    args, train_extra = ap.parse_known_args()

    python = sys.executable
    collect = [
        python, "-m", "rl_move.dynamics.collect_mjx",
        "--name", f"{args.name}-data",
        "--out", args.data,
        "--optimizer-steps", str(args.steps),
        "--batch", str(args.batch),
        "--history", str(args.history),
        "--horizons", args.horizons,
        "--max-window-reuse", str(args.max_window_reuse),
        "--n-envs", str(args.collect_n_envs),
        "--host-workers", str(args.collect_host_workers),
        "--pool-per-env", str(args.collect_pool_per_env),
        "--shard-episodes", str(args.collect_shard_episodes),
        "--impl", args.collect_impl,
        "--seed", str(args.collect_seed),
        "--episodes", str(args.collect_episodes),
        "--actor-device", "cuda",
        "--notes", (args.notes or
                     f"Fresh GPU simulator data for {args.name}; collection "
                     "continues until the planned window reuse is within "
                     f"{args.max_window_reuse:.2f}x."),
    ]
    if args.collect_compressed:
        collect.append("--compressed")
    print("[fresh-pipeline] stage 1/2: GPU MJX data generation", flush=True)
    subprocess.run(collect, check=True)

    train = [
        python, "-m", "rl_move.dynamics.train",
        "--name", args.name,
        "--steps", str(args.steps),
        "--data", args.data,
        "--batch", str(args.batch),
        "--history", str(args.history),
        "--horizons", args.horizons,
        "--max-window-reuse", str(args.max_window_reuse),
    ]
    if args.notes:
        train += ["--notes", args.notes]
    train += train_extra
    print("[fresh-pipeline] stage 2/2: full-size Transformer training",
          flush=True)
    os.execv(python, train)


if __name__ == "__main__":
    main()
