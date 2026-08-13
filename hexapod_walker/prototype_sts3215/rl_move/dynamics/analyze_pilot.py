"""analyze_pilot — aggregate A/B/C pilot eval CSVs across seeds.

Reads logs/ppo_pilot_{phase}_{cond}_s{seed}_eval.csv (written by
train_ppo_transfer.py via run_pilot.sh) and reports, per condition:

  * hold phase:  steps-to-threshold on the hold return (sample
    efficiency), final hold return.
  * lower phase: steps-to-threshold on lower (transfer speed), final
    lower return, final hold return (retention of the old task while
    learning the new one).

All numbers are mean +/- half-range across seeds (n is small; no
stderr theater). A plot of the per-seed curves lands next to the logs.

    python -m rl_move.dynamics.analyze_pilot [--seeds 0 1 2]
        [--hold-threshold -120] [--lower-threshold -40] [--plot]
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

LOG_DIR = Path(__file__).resolve().parent / "logs"
CONDITIONS = ("A", "B", "C")
PHASES = ("hold", "lower")


def load_curve(phase: str, cond: str, seed: int) -> dict[str, np.ndarray]:
    path = LOG_DIR / f"ppo_pilot_{phase}_{cond}_s{seed}_eval.csv"
    if not path.exists():
        raise FileNotFoundError(path)
    cols: dict[str, list[float]] = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                cols.setdefault(k, []).append(float(v) if v else np.nan)
    return {k: np.asarray(v) for k, v in cols.items()}


def steps_to_threshold(steps: np.ndarray, ret: np.ndarray,
                       thr: float) -> float:
    """First eval step at which the return reaches thr (nan = never)."""
    hit = np.nonzero(ret >= thr)[0]
    return float(steps[hit[0]]) if hit.size else float("nan")


def _fmt(vals: list[float]) -> str:
    v = np.asarray(vals, dtype=float)
    if np.all(np.isnan(v)):
        return "never"
    mid = np.nanmean(v)
    hr = (np.nanmax(v) - np.nanmin(v)) / 2.0
    never = int(np.sum(np.isnan(v)))
    s = f"{mid:9.1f} +/- {hr:7.1f}"
    if never:
        s += f" ({never}/{len(v)} never)"
    return s


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--hold-threshold", type=float, default=-120.0)
    ap.add_argument("--lower-threshold", type=float, default=-40.0)
    ap.add_argument("--plot", action="store_true",
                    help="write per-seed curves to logs/pilot_sweep.png")
    args = ap.parse_args()
    thr = {"hold": args.hold_threshold, "lower": args.lower_threshold}

    curves: dict[tuple, dict] = {}
    for ph in PHASES:
        for c in CONDITIONS:
            for s in args.seeds:
                try:
                    curves[(ph, c, s)] = load_curve(ph, c, s)
                except FileNotFoundError as e:
                    print(f"  [missing] {e}")

    for ph in PHASES:
        task = ph                      # the task being TRAINED this phase
        print(f"\n== phase: {ph} (train task '{task}', "
              f"threshold {thr[task]}) ==")
        hdr = (f"{'cond':>4}  {'steps-to-thr':>28}  {'final ' + task:>20}"
               + (f"  {'final hold (retention)':>24}" if ph == "lower"
                  else ""))
        print(hdr)
        for c in CONDITIONS:
            stt, fin, ret_hold = [], [], []
            for s in args.seeds:
                cu = curves.get((ph, c, s))
                if cu is None:
                    continue
                stt.append(steps_to_threshold(
                    cu["step"], cu[f"{task}/return"], thr[task]))
                fin.append(cu[f"{task}/return"][-1])
                if ph == "lower":
                    ret_hold.append(cu["hold/return"][-1])
            line = f"{c:>4}  {_fmt(stt):>28}  {_fmt(fin):>20}"
            if ph == "lower":
                line += f"  {_fmt(ret_hold):>24}"
            print(line)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
        panels = [("hold", "hold"), ("lower", "lower"), ("lower", "hold")]
        titles = ["phase 1: hold return", "phase 2: lower return",
                  "phase 2: hold retention"]
        colors = {"A": "tab:red", "B": "tab:blue", "C": "tab:green"}
        for ax, (ph, task), title in zip(axes.flat, panels, titles):
            for c in CONDITIONS:
                for s in args.seeds:
                    cu = curves.get((ph, c, s))
                    if cu is None:
                        continue
                    ax.plot(cu["step"], cu[f"{task}/return"],
                            color=colors[c], alpha=0.6,
                            label=c if s == args.seeds[0] else None)
            ax.axhline(thr[task], color="gray", ls=":", lw=1)
            ax.set_title(title)
            ax.legend()
        axes.flat[3].axis("off")
        fig.suptitle("dynrep pilot sweep — A scratch / B frozen / "
                     "C anchored (thin lines = seeds)")
        fig.tight_layout()
        out = LOG_DIR / "pilot_sweep.png"
        fig.savefig(out, dpi=110)
        print(f"\nplot -> {out}")


if __name__ == "__main__":
    main()
