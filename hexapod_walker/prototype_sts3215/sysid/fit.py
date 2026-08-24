"""Fit per-axis actuator parameters against sysid hardware traces.

Phase 3 of the sysid plan: fit ONLY actuator/timing parameters (kp, kv,
frictionloss, latency, velocity ceiling, deadband) against unloaded
(suspended) actuator experiments — never contact parameters (those are
fit later, with these frozen; see the plan's "Important constraint").

Method (same family as ``rl_move/sim/fit_loaded_actuator.py``):

1. Measure step/sine metrics per segment of the hardware trace(s);
   median across repeats -> one target per (joint, amp | amp+freq).
2. Split targets into a FIT set and a HELD-OUT set (by step amplitude
   and sine frequency).
3. Per axis, coordinate-descent the ``AxisParams`` fields, scoring each
   candidate by re-simulating the fit-set experiments in the suspended
   sim (``sysid.replay``) and comparing metrics.
4. Report held-out metric losses and full-trace replay RMSE, fitted vs
   the starting parameter set — a fit only counts if held-out
   reproduction improves.
5. If the traces cover several joints of an axis (servo_spread
   protocol), store the per-joint variation as the DR ``spread``.

Output: a ``sim_model*.json`` (SimServoParams schema) selectable via
``--cfg-set bus.servo_params=<path>``, with the evidence trail embedded.

Run (from prototype_sts3215/, repo .venv)::

    uv run python -m sysid.fit --csv sysid/datasets/<dir>/*.csv \
        --out rl_move/sim/sim_model_sysid.json
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import numpy as np

from . import PROTO_DIR  # noqa: F401  (bootstraps sys.path)
from sysid_protocol import AXES, axis_of  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402
from . import trace as trace_mod
from .metrics import compare_joint_streams, segment_metrics
from .replay import SuspendedSim, load_params, replay_protocol, replay_trace

# Step metric weights (fit_loaded_actuator heritage) + sine additions.
STEP_KEYS = ("latency_ms", "t90_ms", "peak_vel_deg_s", "tracking_pct",
             "overshoot_deg")
STEP_W = {"latency_ms": 1.0, "t90_ms": 1.5, "peak_vel_deg_s": 1.0,
          "tracking_pct": 2.0, "overshoot_deg": 0.5}
SINE_W = {"gain": 2.0, "phase_lag_ms": 1.0, "rmse_deg": 1.0}

HOLDOUT_STEP_AMPS = (10.0,)
HOLDOUT_SINE_FREQS = (0.5,)

FIT_FIELDS = ("kp", "kv", "frictionloss", "latency_ms",
              "vel_max_deg_s", "deadband_deg")


# ---------------------------------------------------------------------------
# Targets: measured metrics grouped over repeats
# ---------------------------------------------------------------------------

def build_targets(traces: list[dict]) -> list[dict]:
    """One target per (joint, kind, amp[, freq]): median metrics across
    repeats, plus a representative segment for the sim twin."""
    groups: dict[tuple, dict] = {}
    for tr in traces:
        for seg in trace_mod.segments(tr):
            if seg["kind"] not in ("step", "sine"):
                continue
            m = segment_metrics(tr, seg)
            if not m.get("ok"):
                continue
            spec = seg["spec"]
            key = (seg["joint"], seg["kind"], float(spec["amp_deg"]),
                   float(spec.get("freq_hz", 0.0)))
            g = groups.setdefault(key, {
                "joint": seg["joint"], "kind": seg["kind"],
                "axis": axis_of(seg["joint"]),
                "amp_deg": float(spec["amp_deg"]),
                "freq_hz": float(spec.get("freq_hz", 0.0)),
                "spec": dict(spec), "metrics": [],
                "home_q_deg": tr["q"][seg["sl"].start].tolist(),
            })
            g["metrics"].append(m)
    out = []
    for g in groups.values():
        keys = STEP_KEYS if g["kind"] == "step" else tuple(SINE_W)
        med = {}
        for k in keys:
            vals = [m[k] for m in g["metrics"] if m.get(k) is not None]
            med[k] = statistics.median(vals) if vals else None
        g["target"] = med
        g["n_reps"] = len(g["metrics"])
        out.append(g)
    return out


def split_targets(targets: list[dict]) -> tuple[list[dict], list[dict]]:
    fit_set, holdout = [], []
    for g in targets:
        held = ((g["kind"] == "step"
                 and abs(g["amp_deg"]) in HOLDOUT_STEP_AMPS)
                or (g["kind"] == "sine"
                    and g["freq_hz"] in HOLDOUT_SINE_FREQS))
        (holdout if held else fit_set).append(g)
    return fit_set, holdout


# ---------------------------------------------------------------------------
# Sim twin of one target + loss
# ---------------------------------------------------------------------------

def _mini_protocol(g: dict) -> dict:
    return {"sysid_protocol": 1, "name": "fit_seg", "hz": 25.0,
            "segments": [g["spec"]]}


def sim_target_metrics(g: dict, params: SimServoParams,
                       sim: SuspendedSim) -> dict:
    tr = replay_protocol(_mini_protocol(g), params,
                         q0_deg=np.asarray(g["home_q_deg"]), sim=sim)
    seg = trace_mod.segments(tr)[0]
    return segment_metrics(tr, seg)


def loss_one(g: dict, sim_m: dict) -> float:
    if not sim_m.get("ok"):
        return 8.0
    total = 0.0
    if g["kind"] == "step":
        amp = abs(g["amp_deg"])
        for k in STEP_KEYS:
            tv, sv = g["target"].get(k), sim_m.get(k)
            if tv is None:
                continue
            if sv is None:
                total += 4.0
                continue
            if k == "tracking_pct":
                lsb_pct = 100.0 * 0.088 / amp
                err = max(0.0, abs((100.0 - sv) - (100.0 - tv)) - lsb_pct)
                total += STEP_W[k] * (err / max(100.0 - tv, 2.0)) ** 2
            else:
                floor = 20.0 if k.endswith("_ms") else 2.0
                total += STEP_W[k] * ((sv - tv) / max(abs(tv), floor)) ** 2
    else:
        for k, w in SINE_W.items():
            tv, sv = g["target"].get(k), sim_m.get(k)
            if tv is None:
                continue
            if sv is None:
                total += 4.0
                continue
            floor = {"gain": 0.05, "phase_lag_ms": 15.0,
                     "rmse_deg": 0.3}[k]
            total += w * ((sv - tv) / max(abs(tv), floor)) ** 2
    return total


# ---------------------------------------------------------------------------
# Per-axis coordinate descent
# ---------------------------------------------------------------------------

def fit_axis(axis: str, fit_set: list[dict], params: SimServoParams,
             sim: SuspendedSim, *, rounds: int = 4,
             verbose: bool = True) -> float:
    """Mutates ``params.axes[axis]`` in place; returns final loss."""
    ax = params.axes[axis]
    mine = [g for g in fit_set if g["axis"] == axis]
    if not mine:
        if verbose:
            print(f"  [{axis}] no fit targets — untouched")
        return float("nan")

    def total() -> float:
        return sum(loss_one(g, sim_target_metrics(g, params, sim))
                   for g in mine)

    # Seed from the better of the starting point and the known fitted
    # sets (air / loaded): the loss surface is not monotonic in kp when
    # gravity sag dominates, and a bad basin wastes the whole descent.
    seeds = [("start", {f: getattr(ax, f) for f in FIT_FIELDS})]
    for tag in ("air", "loaded"):
        try:
            cand = load_params(tag).axes[axis]
            seeds.append((tag, {f: getattr(cand, f) for f in FIT_FIELDS}))
        except Exception:
            pass
    best, best_seed = None, None
    for tag, vals in seeds:
        for f, v in vals.items():
            setattr(ax, f, float(v))
        lo = total()
        if best is None or lo < best:
            best, best_seed = lo, vals
    for f, v in best_seed.items():
        setattr(ax, f, float(v))

    grids = {
        "kp": np.geomspace(max(ax.kp / 8.0, 2.0), 1200.0, 11),
        "kv": np.geomspace(0.05, 6.0, 8),
        "frictionloss": np.array([0.0, 0.02, 0.05, 0.1, 0.2]),
        "latency_ms": np.linspace(5.0, 160.0, 8),
        "vel_max_deg_s": np.linspace(20.0, 90.0, 8),
        "deadband_deg": np.linspace(0.05, 0.6, 8),
    }
    if verbose:
        print(f"  [{axis}] start loss {best:.3f} "
              f"({len(mine)} targets)")
    for rnd in range(rounds):
        improved = False
        for key, grid in grids.items():
            cur = getattr(ax, key)
            for v in grid:
                if abs(v - cur) < 1e-9:
                    continue
                setattr(ax, key, float(v))
                lo = total()
                if lo < best - 1e-6:
                    best, cur, improved = lo, float(v), True
                else:
                    setattr(ax, key, cur)
        # Refine grids around the current point.
        grids["kp"] = np.geomspace(ax.kp / 1.5, ax.kp * 1.5, 5)
        grids["kv"] = np.geomspace(max(ax.kv / 1.5, 0.02), ax.kv * 1.5, 5)
        grids["frictionloss"] = np.linspace(
            max(0.0, ax.frictionloss - 0.04), ax.frictionloss + 0.04, 5)
        grids["latency_ms"] = np.linspace(
            max(2.0, ax.latency_ms - 20.0), ax.latency_ms + 20.0, 5)
        grids["vel_max_deg_s"] = np.linspace(
            max(15.0, ax.vel_max_deg_s - 10.0), ax.vel_max_deg_s + 10.0, 5)
        grids["deadband_deg"] = np.linspace(
            max(0.02, ax.deadband_deg - 0.08), ax.deadband_deg + 0.08, 5)
        if verbose:
            print(f"  [{axis}] round {rnd + 1}: loss={best:.3f} "
                  f"kp={ax.kp:.1f} kv={ax.kv:.3f} "
                  f"fric={ax.frictionloss:.3f} lat={ax.latency_ms:.0f}ms "
                  f"vel={ax.vel_max_deg_s:.1f} dbd={ax.deadband_deg:.2f}")
        if not improved:
            break
    return best


# ---------------------------------------------------------------------------
# Spread (Phase 6) + validation
# ---------------------------------------------------------------------------

def per_joint_spread(targets: list[dict]) -> dict[str, dict]:
    """Relative per-joint spread of latency/rise per axis (DR widths)."""
    out: dict[str, dict] = {}
    for axis in AXES:
        per_joint: dict[int, dict[str, list[float]]] = {}
        for g in targets:
            if g["axis"] != axis or g["kind"] != "step":
                continue
            d = per_joint.setdefault(g["joint"], {"lat": [], "rise": []})
            if g["target"].get("latency_ms") is not None:
                d["lat"].append(g["target"]["latency_ms"])
            if g["target"].get("t90_ms") is not None:
                d["rise"].append(g["target"]["t90_ms"])
        if len(per_joint) < 2:
            continue
        lat = [statistics.median(d["lat"]) for d in per_joint.values()
               if d["lat"]]
        rise = [statistics.median(d["rise"]) for d in per_joint.values()
                if d["rise"]]
        sp = {}
        if len(lat) >= 2 and statistics.median(lat) > 0:
            sp["delay_ms_pct"] = round(
                statistics.pstdev(lat) / statistics.median(lat), 3)
        if len(rise) >= 2 and statistics.median(rise) > 0:
            sp["rise_ms_pct"] = round(
                statistics.pstdev(rise) / statistics.median(rise), 3)
        if sp:
            sp["from_joints"] = sorted(per_joint)
            out[axis] = sp
    return out


def validate(targets: list[dict], fitted: SimServoParams,
             base: SimServoParams, sim: SuspendedSim, *,
             tag: str, verbose: bool = True) -> dict:
    out = {}
    for g in targets:
        lf = loss_one(g, sim_target_metrics(g, fitted, sim))
        lb = loss_one(g, sim_target_metrics(g, base, sim))
        label = (f"j{g['joint']}_{g['kind']}{g['amp_deg']:+g}"
                 + (f"@{g['freq_hz']:g}Hz" if g["kind"] == "sine" else ""))
        out[label] = {"loss_fitted": round(lf, 3),
                      "loss_base": round(lb, 3)}
        if verbose:
            print(f"  [{tag}] {label:>24}: fitted {lf:.3f} vs base {lb:.3f}"
                  f"  {'OK' if lf <= lb else 'WORSE'}")
    return out


def validate_full_replay(traces: list[dict], fitted: SimServoParams,
                         base: SimServoParams, sim: SuspendedSim,
                         verbose: bool = True) -> dict:
    out = {}
    for tr in traces:
        res = {}
        for tag, prm in (("fitted", fitted), ("base", base)):
            q_sim = replay_trace(tr, prm, sim=sim)
            res[tag] = compare_joint_streams(tr["t"], tr["q"], q_sim)
        out[tr["name"]] = res
        if verbose:
            print(f"  [replay] {tr['name']}: fitted q RMSE "
                  f"{res['fitted'].get('q_rmse_deg')} deg vs base "
                  f"{res['base'].get('q_rmse_deg')} deg")
    return out


# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", type=Path, nargs="+", required=True,
                    help="hardware sysid trace CSV(s)")
    ap.add_argument("--start-params", default="air",
                    help="starting point: 'air'|'loaded'|'defaults'|path")
    ap.add_argument("--out", type=Path,
                    default=PROTO_DIR / "rl_move/sim/sim_model_sysid.json")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--axes", default="",
                    help="comma list to fit (default: all with data)")
    args = ap.parse_args(argv)

    traces = [trace_mod.load(p) for p in args.csv]
    targets = build_targets(traces)
    if not targets:
        raise SystemExit("no usable step/sine segments in the traces")
    fit_set, holdout = split_targets(targets)
    print(f"[targets] {len(fit_set)} fit / {len(holdout)} held out "
          f"(from {sum(g['n_reps'] for g in targets)} segments)")

    base = load_params(args.start_params)
    params = load_params(args.start_params)
    params.source = (f"sysid fit from {', '.join(p.name for p in args.csv)}"
                     f" (base: {args.start_params})")
    params.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    sim = SuspendedSim(params)

    want = ([a.strip() for a in args.axes.split(",") if a.strip()]
            or list(AXES))
    losses = {}
    for axis in want:
        losses[axis] = fit_axis(axis, fit_set, params, sim,
                                rounds=args.rounds)

    print("[validate] held-out targets:")
    hold_res = validate(holdout, params, base, sim, tag="holdout")
    print("[validate] full-trace replay:")
    replay_res = validate_full_replay(traces, params, base, sim)

    params.spread = {**base.spread, **per_joint_spread(targets)}
    path = params.save(args.out)
    blob = json.loads(path.read_text())
    blob["fit_evidence"] = {
        "traces": [str(p) for p in args.csv],
        "fit_loss": {a: (None if v != v else round(v, 3))
                     for a, v in losses.items()},
        "holdout": hold_res,
        "full_replay": {
            name: {tag: r.get("q_rmse_deg") for tag, r in res.items()}
            for name, res in replay_res.items()},
        "targets": [
            {k: g[k] for k in ("joint", "kind", "amp_deg", "freq_hz",
                               "n_reps", "target")}
            for g in targets],
    }
    path.write_text(json.dumps(blob, indent=2))
    print(f"[fit] wrote {path}")
    print("      select in envs with --cfg-set "
          f"bus.servo_params={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
