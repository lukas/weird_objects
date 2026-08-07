"""Validate the calibrated sim against the hardware battery CSV.

Loads a ``motor_dyn_*.csv`` from the hardware run, re-runs every step
phase in the fixed-base sim with the fitted ``sim_model.json`` params,
and reports per-joint metric deltas (rise / settle / overshoot / delay)
plus an optional trace-overlay PNG per axis.

This is the gate before trusting PPO-in-sim: if the sim's step responses
do not match the hardware's within tolerance, fix the fit first.

Run (from prototype_sts3215/):
    ../../.venv/bin/python -m rl_move.sim.replay_compare \
        --csv linux_control/logs/motor_dyn_YYYYMMDD_HHMMSS.csv [--plot]
"""
from __future__ import annotations

import argparse
import csv
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .fit_motor_model import _StepSim  # noqa: E402
from .servo_model import AXES, SimServoParams  # noqa: E402

METRICS = ("delay_ms", "rise_ms", "settle_ms", "overshoot_deg")
# |sim − hw| tolerance to call a metric matched. A metric also passes on
# near-zero BIAS (≤ tol/3) even if per-joint scatter pushes mean|Δ| over:
# the sim carries one value per axis while real joints spread (2026-08-07
# battery: delay 147–226 ms across hips) — that spread is what the DR
# latency/kp/kv scales absorb during training, not a calibration error.
TOL = {"delay_ms": 30.0, "rise_ms": 40.0, "settle_ms": 80.0,
       "overshoot_deg": 1.0}
_STEP_RE = re.compile(r"^step([+-])(\d+(?:\.\d+)?)r\d+$")


def load_step_phases(csv_path: Path) -> dict:
    """Group rows: {(joint, phase): [rows...]} for step phases only."""
    groups: dict[tuple[int, str], list[dict]] = defaultdict(list)
    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            phase = row.get("phase", "")
            if not _STEP_RE.match(phase) or not row.get("present_deg"):
                continue
            groups[(int(row["joint"]), phase)].append({
                "t_s": float(row["t_s"]),
                "present_deg": float(row["present_deg"]),
                "cmd_deg": float(row["cmd_deg"]),
                "speed_deg_s": float(row["speed_deg_s"] or 0),
            })
    return groups


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", type=Path, required=True,
                    help="hardware motor_dyn_*.csv")
    ap.add_argument("--sim-model", type=Path, default=None,
                    help="sim_model.json (default: fitted one)")
    ap.add_argument("--speed-counts", type=float, default=None,
                    help="profile speed in counts/s (default: from "
                         "sim_model.json)")
    ap.add_argument("--acc-units", type=float, default=15.0,
                    help="profile acc in Feetech register units "
                         "(battery uses ACC=15)")
    ap.add_argument("--plot", action="store_true",
                    help="write per-axis overlay PNGs next to the CSV")
    args = ap.parse_args(argv)

    from motor_dynamics import _fit_step

    params = (SimServoParams.load(args.sim_model) if args.sim_model
              else SimServoParams.load())
    if params.source == "defaults":
        print("[replay] WARNING: sim_model.json is defaults — run "
              "fit_motor_model on real data first for a meaningful check")
    speed_deg_s = ((args.speed_counts or params.speed_counts_s)
                   * 360.0 / 4096.0)

    groups = load_step_phases(args.csv)
    if not groups:
        raise SystemExit(f"no step phases found in {args.csv}")

    sim = _StepSim()
    rows_out = []
    deltas: dict[str, list[float]] = defaultdict(list)
    plots: dict[str, tuple] = {}

    for (joint, phase), rows in sorted(groups.items()):
        m = _STEP_RE.match(phase)
        assert m is not None
        amp = float(m.group(2)) * (1 if m.group(1) == "+" else -1)
        # Hardware rows are absolute time; re-zero on the phase start.
        t0 = rows[0]["t_s"]
        hw_rows = [{**r, "t_s": r["t_s"] - t0} for r in rows]
        hw_fit = _fit_step(hw_rows, amp=amp, t_cmd=0.0)
        sim_rows = sim.run_step(params, joint, amp, speed_deg_s=speed_deg_s,
                                acc_units=args.acc_units)
        sim_fit = _fit_step(sim_rows, amp=amp, t_cmd=0.0)

        rec = {"joint": joint, "phase": phase}
        for k in METRICS:
            h, s = hw_fit.get(k), sim_fit.get(k)
            rec[f"hw_{k}"], rec[f"sim_{k}"] = h, s
            if h is not None and s is not None:
                deltas[k].append(s - h)
        rows_out.append(rec)

        axis = AXES[joint % 3]
        if args.plot and axis not in plots:
            plots[axis] = (joint, phase, hw_rows, sim_rows)

    # Report
    print(f"[replay] {len(rows_out)} step phases from {args.csv.name} "
          f"(sim params: {params.source})")
    hdr = f"{'joint':>5} {'phase':>12}"
    for k in METRICS:
        hdr += f" {'hw_' + k:>12} {'sim_' + k:>12}"
    print(hdr)
    for rec in rows_out:
        line = f"{rec['joint']:>5} {rec['phase']:>12}"
        for k in METRICS:
            h, s = rec[f"hw_{k}"], rec[f"sim_{k}"]
            line += (f" {h if h is not None else '—':>12}"
                     f" {s if s is not None else '—':>12}")
        print(line)

    print("\nper-metric |sim − hw|:")
    all_ok = True
    for k in METRICS:
        if not deltas[k]:
            print(f"  {k:>14}: no data")
            continue
        mean_abs = statistics.mean(abs(d) for d in deltas[k])
        bias = statistics.mean(deltas[k])
        ok = mean_abs <= TOL[k] or abs(bias) <= TOL[k] / 3.0
        all_ok = all_ok and ok
        note = ("OK" if mean_abs <= TOL[k]
                else "OK (unbiased; scatter → DR)" if ok else "MISMATCH")
        print(f"  {k:>14}: mean|Δ|={mean_abs:8.2f}  bias={bias:+8.2f}  "
              f"tol={TOL[k]:g}  {note}")
    print(f"\n[replay] {'PASS' if all_ok else 'FAIL'} — "
          + ("sim matches hardware within tolerance"
             if all_ok else "re-fit or widen DR before trusting sim training"))

    if args.plot and plots:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[replay] matplotlib not installed — skipping plots")
            return 0 if all_ok else 1
        fig, axs = plt.subplots(1, len(plots), figsize=(5 * len(plots), 4))
        if len(plots) == 1:
            axs = [axs]
        for axplt, (axis, (joint, phase, hw_rows, sim_rows)) in zip(
                axs, sorted(plots.items())):
            # Normalize both traces to start at 0° (different rest poses).
            h0 = hw_rows[0]["present_deg"]
            s0 = sim_rows[0]["present_deg"]
            axplt.plot([r["t_s"] for r in hw_rows],
                       [r["present_deg"] - h0 for r in hw_rows],
                       "o-", ms=3, label="hardware")
            axplt.plot([r["t_s"] for r in sim_rows],
                       [r["present_deg"] - s0 for r in sim_rows],
                       "-", label="sim (fitted)")
            axplt.set_title(f"{axis} — joint {joint} {phase}")
            axplt.set_xlabel("t (s)")
            axplt.set_ylabel("present (deg)")
            axplt.legend()
            axplt.grid(alpha=0.3)
        out = args.csv.with_suffix(".replay.png")
        fig.tight_layout()
        fig.savefig(out, dpi=110)
        print(f"[replay] wrote {out}")

    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
