#!/usr/bin/env python3
"""analyze_rise_walk — event-aligned rise→walk hardware session report.

Operator RISE_WALK_NEXT_48H directive (08-13), "P3 — Prepare the
hardware validation package now": make the eventual bench session
answer sim-vs-hardware questions immediately. Reads the EXISTING 25 Hz
per-tick tapes (rl_stand_*/rl_walk_* CSVs: attitude, gyro, refs,
measured q + post-SafetyLayer cmd ×18, raw action ×18, per-servo
current, phase) — no robot access, no new logging required.

It aligns each tape around the directive's named events

    rise start / rise completion        (height_ref ramp edges)
    locomotion-policy enable            (walk-tape tick 0, or phase)
    velocity-ramp start                 (first |v*_ref| > 0)
    first gait cycle                    (dominant-joint cmd period)
    command back to zero                (v_ref returns to 0)

and reports, over the first --win-s (default 1 s) after each event,
THE SAME metric set as the sim-side transition gate
(rl_move/sim/eval_transition_gate.py) so the two compare cell by cell:

    peak |roll| / |pitch| (rel. to pre-event), peak roll rate
    max joint-target discontinuity (deg/tick, post-SafetyLayer)
    slew-saturation fraction (|Δcmd| pinned at safety.max_delta_q_deg)
    max simultaneous saturated joints
    peak single / summed servo current
    cmd-vs-measured tracking RMS
    time-to-stable after enable, falls (bench_report tail rule)

Usage:

    # one session dir (stand tape then walk tape, chronological):
    uv run python -m rl_move.scripts.analyze_rise_walk \
        rl_move/hardware_traces/bench_blast_<stamp>/ --plots out/

    # explicit tapes, plus sim comparison:
    uv run python -m rl_move.scripts.analyze_rise_walk rise.csv walk.csv \
        --sim-gate logs/ckpt_eval/transition_gate_baseline.json

Writes <out>/rise_walk_metrics.json and per-tape event-aligned PNGs.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

TICK_LIMIT_DEG = 1.5      # safety.max_delta_q_deg deployed value
FELL_TAIL_DEG = 10.0      # bench_report verdict rule
STABLE_TILT_DEG = 3.0     # match eval_transition_gate defaults
STABLE_GYRO_DPS = 30.0
STABLE_HOLD_S = 0.3


def load_tape(path: Path) -> dict:
    with path.open() as f:
        rows = [r for r in csv.DictReader(f) if r.get("t_s")]
    # The 3 s read-only post-episode tail (10 Hz, servos limp) logs
    # stale/zero commands — keeping it would fabricate huge Δcmd and
    # tracking-error spikes. The run rows still contain any fall.
    rows = [r for r in rows if r.get("phase") != "tail"]
    if len(rows) < 5:
        raise SystemExit(f"{path}: only {len(rows)} non-tail rows")

    def col(name, default=0.0):
        return np.array([float(r.get(name) or default) for r in rows])

    n_j = sum(1 for k in rows[0] if k.startswith("cmd")
              and k.endswith("_deg"))
    q = np.stack([col(f"q{j}_deg") for j in range(n_j)], axis=1)
    cmd = np.stack([col(f"cmd{j}_deg") for j in range(n_j)], axis=1)
    cur_cols = [k for k in rows[0]
                if k.startswith("cur") and k.endswith("_a")]
    cur = (np.stack([col(c) for c in cur_cols], axis=1)
           if cur_cols else np.zeros((len(rows), 0)))
    return {
        "file": path.name,
        "t": col("t_s"),
        "phase": [r.get("phase", "") for r in rows],
        "roll": col("roll_deg"), "pitch": col("pitch_deg"),
        "gyro_x": col("gyro_x_dps"),
        "h_ref": col("height_ref_mm"),
        "vx_ref": col("vx_ref_mps"), "vy_ref": col("vy_ref_mps"),
        "q": q, "cmd": cmd, "cur": cur,
    }


def detect_events(tape: dict, kind: str) -> dict[str, float | None]:
    """Directive event timestamps on one tape (None = not present)."""
    t = tape["t"]
    ev: dict[str, float | None] = {}
    h = tape["h_ref"]
    dh = np.abs(h - h[0])
    if kind == "rise" and np.max(dh) > 1.0:
        i0 = int(np.argmax(dh > 1.0))
        ev["rise_start"] = float(t[i0])
        plateau = np.abs(h - h[-1]) < 0.5
        i1 = len(h) - 1
        for i in range(i0, len(h)):
            if plateau[i:].all():
                i1 = i
                break
        ev["rise_complete"] = float(t[i1])
    if kind == "walk":
        ev["policy_enable"] = float(t[0])
        v = np.hypot(tape["vx_ref"], tape["vy_ref"])
        moving = v > 1e-6
        if moving.any():
            ir = int(np.argmax(moving))
            ev["ramp_start"] = float(t[ir])
            after = np.where(~moving[ir:])[0]
            if after.size:
                ev["cmd_zero"] = float(t[ir + int(after[0])])
            # first gait cycle: dominant oscillating joint's cmd
            # completes one full period after the ramp starts.
            seg = tape["cmd"][ir:]
            if len(seg) > 10:
                d = seg - seg.mean(axis=0)
                j = int(np.argmax(d.var(axis=0)))
                s = np.signbit(d[:, j])
                flips = np.where(s[1:] != s[:-1])[0]
                if flips.size >= 3:   # two half-periods = one cycle
                    ev["first_gait_cycle"] = float(t[ir + int(flips[2])])
                ev["gait_joint"] = float(j)
    return ev


def window_metrics(tape: dict, t0: float, win_s: float) -> dict:
    """The eval_transition_gate W-metric set on [t0, t0+win_s)."""
    t = tape["t"]
    m = (t >= t0) & (t < t0 + win_s)
    if not m.any():
        return {}
    pre = t < t0
    roll0 = float(tape["roll"][pre][-1]) if pre.any() \
        else float(tape["roll"][m][0])
    pitch0 = float(tape["pitch"][pre][-1]) if pre.any() \
        else float(tape["pitch"][m][0])
    dq = np.abs(np.diff(tape["cmd"], axis=0, prepend=tape["cmd"][:1]))
    sat = dq >= 0.999 * TICK_LIMIT_DEG
    cur = tape["cur"]
    out = {
        "peak_roll_deg": round(float(np.max(np.abs(
            tape["roll"][m] - roll0))), 2),
        "peak_pitch_deg": round(float(np.max(np.abs(
            tape["pitch"][m] - pitch0))), 2),
        "peak_roll_rate_deg_s": round(float(np.max(np.abs(
            tape["gyro_x"][m]))), 1),
        "max_target_disc_deg": round(float(np.max(dq[m])), 2),
        "sat_joint_frac": round(float(np.mean(sat[m])), 3),
        "max_sat_frac": round(float(np.max(np.mean(sat[m], axis=1))), 3),
        "track_rmse_deg": round(float(np.sqrt(np.mean(
            (tape["q"][m] - tape["cmd"][m]) ** 2))), 2),
    }
    if cur.shape[1]:
        out["peak_servo_a"] = round(float(np.max(cur[m])), 2)
        out["peak_sum_a"] = round(float(np.max(cur[m].sum(axis=1))), 2)
    return out


def time_to_stable(tape: dict, t0: float, until: float | None) -> float | None:
    t = tape["t"]
    hi = until if until is not None else float(t[-1])
    m = (t >= t0) & (t < hi)
    idx = np.where(m)[0]
    if idx.size == 0:
        return None
    roll0 = float(tape["roll"][idx[0]])
    pitch0 = float(tape["pitch"][idx[0]])
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.04
    need = max(1, int(round(STABLE_HOLD_S / dt)))
    run = 0
    for i in idx:
        quiet = (abs(tape["roll"][i] - roll0) <= STABLE_TILT_DEG
                 and abs(tape["pitch"][i] - pitch0) <= STABLE_TILT_DEG
                 and abs(tape["gyro_x"][i]) <= STABLE_GYRO_DPS)
        run = run + 1 if quiet else 0
        if run >= need:
            return round(float(t[i]) - t0 - STABLE_HOLD_S, 2)
    return None


def fell(tape: dict) -> bool:
    t = tape["t"]
    tail = t >= (t[-1] - 1.0)
    rel = np.abs(tape["roll"] - tape["roll"][0])
    return bool(np.mean(rel[tail]) >= FELL_TAIL_DEG)


def plot_tape(tape: dict, events: dict, wins: dict, out_png: Path,
              win_s: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = tape["t"]
    dq = np.abs(np.diff(tape["cmd"], axis=0, prepend=tape["cmd"][:1]))
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, tape["roll"], label="roll")
    axes[0].plot(t, tape["pitch"], label="pitch")
    axes[0].set_ylabel("deg")
    axes[1].plot(t, tape["gyro_x"], label="gyro_x", color="tab:red")
    axes[1].set_ylabel("dps")
    axes[2].plot(t, dq.max(axis=1), label="max |Δcmd|",
                 color="tab:purple")
    axes[2].axhline(TICK_LIMIT_DEG, ls="--", c="k", lw=0.8,
                    label=f"slew limit {TICK_LIMIT_DEG}°")
    axes[2].plot(t, (dq >= 0.999 * TICK_LIMIT_DEG).mean(axis=1)
                 * TICK_LIMIT_DEG, alpha=0.5, label="sat frac (scaled)")
    axes[2].set_ylabel("deg/tick")
    if tape["cur"].shape[1]:
        axes[3].plot(t, tape["cur"].max(axis=1), label="max servo A")
        axes[3].plot(t, tape["cur"].sum(axis=1) / 10.0,
                     label="sum A / 10")
    ax2 = axes[3].twinx()
    ax2.plot(t, tape["vx_ref"], c="tab:green", lw=1.0, label="vx_ref")
    ax2.set_ylabel("m/s")
    axes[3].set_ylabel("A")
    axes[3].set_xlabel("t [s]")
    colors = {"rise_start": "tab:blue", "rise_complete": "tab:cyan",
              "policy_enable": "tab:orange", "ramp_start": "tab:green",
              "first_gait_cycle": "tab:olive", "cmd_zero": "tab:gray"}
    for name, tv in events.items():
        if name == "gait_joint" or tv is None:
            continue
        for ax in axes:
            ax.axvline(tv, color=colors.get(name, "k"), lw=1.0,
                       alpha=0.8)
            if name in wins:
                ax.axvspan(tv, tv + win_s,
                           color=colors.get(name, "k"), alpha=0.08)
        axes[0].text(tv, axes[0].get_ylim()[1], name, rotation=90,
                     fontsize=7, va="top")
    for ax in axes:
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(alpha=0.3)
    fig.suptitle(f"{tape['file']}  (windows = {win_s}s after event)")
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("inputs", nargs="+",
                    help="session dir(s) and/or rl_*.csv tapes; a dir "
                         "expands to its rl_stand_*/rl_walk_* tapes in "
                         "chronological order")
    ap.add_argument("--win-s", type=float, default=1.0)
    ap.add_argument("--plots", type=Path, default=None,
                    help="dir for event-aligned PNGs (default: next to "
                         "the metrics json)")
    ap.add_argument("--out", type=Path, default=None,
                    help="metrics json (default <first-dir>/"
                         "rise_walk_metrics.json)")
    ap.add_argument("--sim-gate", type=Path, default=None,
                    help="eval_transition_gate output json to print a "
                         "sim-vs-hardware W0 comparison against")
    args = ap.parse_args()

    tapes: list[Path] = []
    for inp in args.inputs:
        p = Path(inp)
        if p.is_dir():
            tapes += sorted(p.glob("rl_stand_*.csv"))
            tapes += sorted(p.glob("rl_walk_*.csv"))
        else:
            tapes.append(p)
    if not tapes:
        raise SystemExit("no rl_*.csv tapes found in inputs")

    report: dict = {"win_s": args.win_s, "tapes": []}
    for path in tapes:
        kind = "walk" if "walk" in path.name else "rise"
        tape = load_tape(path)
        events = detect_events(tape, kind)
        wins = {}
        for name in ("rise_start", "rise_complete", "policy_enable",
                     "ramp_start", "first_gait_cycle", "cmd_zero"):
            tv = events.get(name)
            if tv is not None:
                wins[name] = window_metrics(tape, tv, args.win_s)
        rec = {"file": str(path), "kind": kind,
               "events": {k: v for k, v in events.items()
                          if k != "gait_joint"},
               "windows": wins,
               "fell": fell(tape)}
        if kind == "walk":
            rec["time_to_stable_s"] = time_to_stable(
                tape, events["policy_enable"], events.get("ramp_start"))
        report["tapes"].append(rec)
        plot_dir = args.plots or (tapes[0].parent / "rise_walk_plots")
        plot_tape(tape, events, wins, plot_dir / f"{path.stem}.png",
                  args.win_s)
        print(f"[{path.name}] kind={kind} events="
              f"{ {k: v for k, v in events.items() if k != 'gait_joint'} }"
              f" fell={rec['fell']}")
        for name, w in wins.items():
            print(f"    {name:18s} {w}")

    # ---- sim-vs-hardware: same window metric names, cell by cell -------
    if args.sim_gate is not None:
        sim = json.loads(args.sim_gate.read_text())
        sim_w0 = [r["windows"]["activate"] for r in sim.get("records", [])]
        hw_w0 = [r["windows"].get("policy_enable") for r in report["tapes"]
                 if r["kind"] == "walk"]
        hw_w0 = [w for w in hw_w0 if w]
        keys = ("peak_roll_deg", "peak_roll_rate_deg_s",
                "max_target_disc_deg", "max_sat_frac", "track_rmse_deg")
        if sim_w0 and hw_w0:
            print(f"\n=== sim vs hardware, takeover window "
                  f"(medians, n_sim={len(sim_w0)} n_hw={len(hw_w0)}) ===")
            print(f"{'metric':26s}{'sim':>10s}{'hw':>10s}{'hw/sim':>9s}")
            for k in keys:
                sv = float(np.median([w.get(k, math.nan)
                                      for w in sim_w0]))
                hv = float(np.median([w.get(k, math.nan)
                                      for w in hw_w0]))
                ratio = hv / sv if sv else math.inf
                print(f"{k:26s}{sv:10.2f}{hv:10.2f}{ratio:9.2f}")
            print("(hw/sim >> 1 on roll/saturation = the sim still "
                  "underestimates the takeoff transient — actuator/"
                  "compliance modeling gap, see COMPLIANCE.md item 7)")

    out = args.out or (tapes[0].parent / "rise_walk_metrics.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1))
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
