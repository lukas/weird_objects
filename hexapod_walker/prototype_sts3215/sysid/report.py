"""Reality Gap Report: automatic hardware-vs-sim scorecard.

Consumes fixed sysid datasets (hardware traces from
``linux_control/sysid_runner.py``), replays every trace open-loop in the
suspended sim under one or more servo-parameter sets, and emits a
versioned report: headline gap metrics, per-segment tables, latency /
jitter distributions, per-servo variation, and overlay plots.

Per the plan: a simulator change is successful only if it improves
HELD-OUT hardware reproduction — these numbers are the contract, and
they are versioned alongside the parameter files (hashes in the
header).

Run (from prototype_sts3215/, repo .venv)::

    uv run python -m sysid.report --csv sysid/datasets/<dir>/*.csv \
        --servo-params air loaded
"""
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import subprocess
import time
from pathlib import Path

import numpy as np

from . import PROTO_DIR, REPORT_DIR
from sysid_protocol import axis_of, protocol_hash  # noqa: E402
from . import trace as trace_mod
from .metrics import (compare_joint_streams, latency_stats,
                      segment_metrics, tick_jitter_stats)
from .replay import SuspendedSim, load_params, replay_trace


def _git_rev() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=PROTO_DIR,
            capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return "unknown"


def _file_hash(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()[:12]
    except Exception:
        return "n/a"


def _param_files(sels: list[str]) -> dict[str, str]:
    from rl_move.sim.servo_model import LOADED_MODEL_PATH, SIM_MODEL_PATH
    out = {}
    for s in sels:
        p = (SIM_MODEL_PATH if s == "air"
             else LOADED_MODEL_PATH if s == "loaded" else Path(s))
        out[s] = f"{p.name} ({_file_hash(Path(p))})"
    return out


def analyze_trace(tr: dict, param_sets: dict, sims: dict) -> dict:
    """Hardware stats + per-param-set sim comparison for one trace."""
    segs = trace_mod.segments(tr)
    hz = float((tr.get("protocol") or {}).get("hz", 25.0))
    rec: dict = {
        "trace": tr["name"],
        "protocol": (tr.get("protocol") or {}).get("name"),
        "protocol_hash": (protocol_hash(tr["protocol"])
                          if tr.get("protocol") else None),
        "ticks": int(len(tr["t"])),
        "tick_jitter": tick_jitter_stats(tr["t_send"], hz),
        "read_rtt": latency_stats(
            list((tr["t_recv"] - tr["t_send"]) * 1000.0)),
    }

    # Hardware step/sine metrics (+ latency distribution over all reps).
    hw_m = {}
    step_lat: list[float] = []
    for seg in segs:
        m = segment_metrics(tr, seg)
        hw_m[seg["index"]] = m
        if seg["kind"] == "step" and m.get("latency_ms") is not None:
            step_lat.append(m["latency_ms"])
    rec["cmd_to_motion_latency"] = latency_stats(step_lat)

    # Per-servo variation (Phase 6): joints with >=2 step segments.
    per_joint: dict[int, list[float]] = {}
    for seg in segs:
        m = hw_m[seg["index"]]
        if seg["kind"] == "step" and m.get("t90_ms") is not None:
            per_joint.setdefault(seg["joint"], []).append(m["t90_ms"])
    variation = {}
    for axis in ("yaw", "hip", "knee"):
        meds = {j: statistics.median(v) for j, v in per_joint.items()
                if axis_of(j) == axis and v}
        if len(meds) >= 2:
            med = statistics.median(meds.values())
            variation[axis] = {
                "n_joints": len(meds),
                "t90_median_ms": round(med, 1),
                "t90_spread_pct": round(100.0 * statistics.pstdev(
                    meds.values()) / max(med, 1e-6), 1),
                "outliers": sorted(j for j, v in meds.items()
                                   if v > 1.5 * med or v < med / 1.5),
            }
    rec["servo_variation"] = variation

    # Sim comparison per parameter set.
    rec["sim"] = {}
    rec["segments"] = []
    q_sims = {}
    for tag, params in param_sets.items():
        q_sim = replay_trace(tr, params, sim=sims[tag])
        q_sims[tag] = q_sim
        rec["sim"][tag] = compare_joint_streams(tr["t"], tr["q"], q_sim)
    for seg in segs:
        row = {"label": seg["label"], "kind": seg["kind"],
               "joint": seg["joint"], "hw": hw_m[seg["index"]]}
        for tag in param_sets:
            row[f"sim_{tag}"] = segment_metrics(tr, seg, q=q_sims[tag])
        rec["segments"].append(row)
    rec["_q_sims"] = q_sims
    return rec


def _fmt_metric(m: dict, keys: tuple[str, ...]) -> str:
    if not m or not m.get("ok"):
        return "—"
    return " ".join(f"{k.split('_')[0]}={m[k]}" for k in keys
                    if m.get(k) is not None)


def write_markdown(out_dir: Path, meta: dict, recs: list[dict]) -> Path:
    lines = ["# Hexapod Sim Reality Report", ""]
    lines += [f"- generated: {meta['generated']}",
              f"- git: `{meta['git']}`"]
    for tag, desc in meta["param_files"].items():
        lines.append(f"- servo params `{tag}`: {desc}")
    lines.append("")
    tags = list(meta["param_files"])

    lines.append("## Headline gap metrics")
    lines.append("")
    hdr = "| trace | metric | hardware | " + " | ".join(
        f"sim {t}" for t in tags) + " |"
    lines += [hdr, "|" + "---|" * (3 + len(tags))]
    for r in recs:
        for metric, unit in (("q_rmse_deg", "deg"),
                             ("qd_rmse_deg_s", "deg/s")):
            row = [r["trace"], f"unloaded {metric}", "—"]
            row += [str(r["sim"][t].get(metric, "—")) for t in tags]
            lines.append("| " + " | ".join(row) + " |")
        lat = r["cmd_to_motion_latency"]
        if lat.get("n"):
            lines.append(
                f"| {r['trace']} | cmd→motion latency (hw) | "
                f"median {lat.get('p50_ms')} ms "
                f"(p10 {lat.get('p10_ms')} / p90 {lat.get('p90_ms')} / "
                f"max {lat.get('max_ms')}, n={lat['n']}) | "
                + " | ".join("—" for _ in tags) + " |")
    lines.append("")

    lines.append("## Timing (hardware)")
    lines.append("")
    for r in recs:
        j = r["tick_jitter"]
        rtt = r["read_rtt"]
        lines.append(
            f"- **{r['trace']}**: tick median {j.get('median_ms')} ms "
            f"(nominal {j.get('nominal_ms')}, p95 {j.get('p95_ms')}, "
            f"late {j.get('late_pct')}%); send→recv RTT median "
            f"{rtt.get('p50_ms')} ms (p95 {rtt.get('p95_ms')})")
    lines.append("")

    any_var = any(r["servo_variation"] for r in recs)
    if any_var:
        lines.append("## Servo-to-servo variation (t90)")
        lines.append("")
        for r in recs:
            for axis, v in r["servo_variation"].items():
                lines.append(
                    f"- {r['trace']} [{axis}]: median "
                    f"{v['t90_median_ms']} ms over {v['n_joints']} joints,"
                    f" spread {v['t90_spread_pct']}%"
                    + (f", OUTLIERS: joints {v['outliers']}"
                       if v["outliers"] else ""))
        lines.append("")

    lines.append("## Per-segment detail")
    lines.append("")
    step_keys = ("latency_ms", "t90_ms", "overshoot_deg", "tracking_pct")
    sine_keys = ("gain", "phase_lag_ms", "rmse_deg")
    for r in recs:
        lines.append(f"### {r['trace']}")
        lines.append("")
        hdr = "| segment | hardware | " + " | ".join(
            f"sim {t}" for t in tags) + " |"
        lines += [hdr, "|" + "---|" * (2 + len(tags))]
        for row in r["segments"]:
            keys = step_keys if row["kind"] == "step" else (
                sine_keys if row["kind"] == "sine"
                else ("rmse_moving_deg",))
            cells = [row["label"], _fmt_metric(row["hw"], keys)]
            cells += [_fmt_metric(row.get(f"sim_{t}"), keys) for t in tags]
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")

    path = out_dir / "reality_gap.md"
    path.write_text("\n".join(lines))
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", type=Path, nargs="+", required=True)
    ap.add_argument("--servo-params", nargs="+", default=["air", "loaded"],
                    help="parameter sets to score "
                         "('air'|'loaded'|'defaults'|path)")
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir or (REPORT_DIR / f"reality_gap_{stamp}")
    out_dir.mkdir(parents=True, exist_ok=True)

    param_sets = {s: load_params(s) for s in args.servo_params}
    sims = {s: SuspendedSim(p) for s, p in param_sets.items()}
    meta = {
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "git": _git_rev(),
        "param_files": _param_files(args.servo_params),
        "traces": [str(p) for p in args.csv],
    }

    recs = []
    for p in args.csv:
        tr = trace_mod.load(p)
        print(f"[report] {tr['name']} ({len(tr['t'])} ticks)")
        rec = analyze_trace(tr, param_sets, sims)
        for tag in param_sets:
            print(f"  sim {tag}: {rec['sim'][tag]}")
        if not args.no_plots:
            from .plots import overlay_trace
            overlay_trace(tr, rec["_q_sims"],
                          out_dir / f"{p.stem}.overlay.png")
        rec.pop("_q_sims")
        recs.append(rec)

    (out_dir / "reality_gap.json").write_text(
        json.dumps({"meta": meta, "traces": recs}, indent=1))
    md = write_markdown(out_dir, meta, recs)
    print(f"[report] wrote {md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
