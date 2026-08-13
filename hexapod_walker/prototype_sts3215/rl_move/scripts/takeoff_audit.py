#!/usr/bin/env python3
"""takeoff_audit — the walk-takeoff fingerprint, per the 08-13 ruling.

Operator ruling 08-13 (hw/STATUS.md): STOP reward/DR sweeps at takeoff;
INSTRUMENT the first ~1.5 s after gait start (roll rate, per-foot
loading, which feet break contact, command ramp phase), THEN design a
staged gait-entry transition. This script is the instrument; findings
+ the staged-entry design live in rl_docs/TAKEOFF.md (per-foot contact
story = §1b there). The entry mechanism itself is
safety.entry_slew_ramp_s / entry_slew_start_deg (rl_move/safety.py).

For every on-robot walk tape (``rl_walk_*.csv``) it computes the
command-side takeoff fingerprint bench_report's roll-shape table does
not cover:

- engagement discontinuity: the policy's RAW first-tick joint targets
  (reconstructed from the logged actions, pre-safety) vs the measured
  start pose — how big a whole-body reconfiguration the policy asked
  for at v_cmd = 0,
- slew saturation: per tick, how many of the 18 joints move at exactly
  the SafetyLayer's 1.5 deg/tick clamp; max simultaneous + the mean
  over the window,
- action saturation (|a| >= 0.99) at tick 0 and over the window,
- attitude: peak |roll_rel| / roll rate / pitch in the window and over
  the full tape, the 5-deg crossing time and WHERE IN THE COMMAND RAMP
  it happens (v_ref magnitude at crossing: 0 = still the zero-command
  settle hold),
- commanded-vs-measured tracking error over the window,
- currents: peak per-servo and peak whole-body sum in the window,
- time-to-stable (first tick after which |roll_rel| stays <= 3 deg for
  a full second) and the fell/recovered verdict.

``--replay`` additionally runs the open-loop matched sim replay
(replay_trace machinery, recorded cmd stream through ServoProfile +
free-base MuJoCo) to get what hardware cannot log: per-foot loading at
engagement, which feet break contact first, contact transitions and
loaded-foot slip inside the window.

Reads ONLY local files — never the robot.

Run (from prototype_sts3215/, repo .venv):
    python -m rl_move.scripts.takeoff_audit --all
    python -m rl_move.scripts.takeoff_audit --all --replay --md-out \
        rl_docs/TAKEOFF_AUDIT.md
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (str(_PROTO),):
    if p not in sys.path:
        sys.path.insert(0, p)

from rl_move.config import cfg_get, load_config          # noqa: E402
from rl_move.safety import AXIS_LIMITS_DEG               # noqa: E402

TRACES = _RL / "hardware_traces"
N_JOINTS = 18

# Raw policy targets: q_prop = CENTER + action * HALF (rl_policy.py).
_CENTER_DEG = np.array([
    (AXIS_LIMITS_DEG[j % 3][0] + AXIS_LIMITS_DEG[j % 3][1]) * 0.5
    for j in range(N_JOINTS)])
_HALF_DEG = np.array([
    (AXIS_LIMITS_DEG[j % 3][1] - AXIS_LIMITS_DEG[j % 3][0]) * 0.5
    for j in range(N_JOINTS)])

STABLE_DEG = 3.0          # "stable" = |roll_rel| at/below this ...
STABLE_HOLD_S = 1.0       # ... sustained this long
CROSS_DEG = 5.0           # transient-onset crossing (bench_report's)
ACT_SAT = 0.99            # |action| at/beyond this = saturated
# CSV cmd values are rounded to 0.01 deg; a clamped step reads exactly
# the limit, so anything within 0.06 deg of it counts as pinned.
SLEW_EPS_DEG = 0.06


def _f(row: dict, key: str, default: float = math.nan) -> float:
    v = row.get(key, "")
    return float(v) if v not in ("", None) else default


def load_tape(csv_path: Path) -> dict:
    """Run + tail rows of an on-robot rl_walk CSV + its sidecar."""
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    run = [r for r in rows if r.get("phase") == "run"]
    tail = [r for r in rows if r.get("phase") == "tail"]
    if len(run) < 25:
        raise ValueError(f"{csv_path.name}: only {len(run)} run ticks")
    tp = {
        "path": csv_path,
        "name": csv_path.name,
        "t": np.array([_f(r, "t_s") for r in run]),
        "roll": np.array([_f(r, "roll_deg") for r in run]),
        "pitch": np.array([_f(r, "pitch_deg") for r in run]),
        "gyro_x": np.array([_f(r, "gyro_x_dps") for r in run]),
        "gyro_y": np.array([_f(r, "gyro_y_dps") for r in run]),
        "vx_ref": np.array([_f(r, "vx_ref_mps") for r in run]),
        "vy_ref": np.array([_f(r, "vy_ref_mps") for r in run]),
        "q": np.array([[_f(r, f"q{j}_deg") for j in range(N_JOINTS)]
                       for r in run]),
        "cmd": np.array([[_f(r, f"cmd{j}_deg") for j in range(N_JOINTS)]
                         for r in run]),
        "act": np.array([[_f(r, f"act{j}") for j in range(N_JOINTS)]
                         for r in run]),
        "cur": np.array([[_f(r, f"cur{j}_a") for j in range(N_JOINTS)]
                         for r in run]),
        "tail_roll": np.array([_f(r, "roll_deg") for r in tail]),
    }
    summary = {}
    sp = csv_path.with_name(csv_path.stem + "_summary.json")
    if sp.exists():
        summary = json.loads(sp.read_text())
    # Many tapes have no per-episode sidecar but ARE indexed in the
    # bench_blast session summary (walks[].csv), which carries the
    # policy tag, tilt reference and the session's own fell verdict.
    sess_walk = {}
    sess_path = csv_path.parent / "summary.json"
    if sess_path.exists():
        try:
            sess = json.loads(sess_path.read_text())
            sess_walk = next((w for w in sess.get("walks", [])
                              if w.get("csv") == csv_path.name), {})
        except (OSError, json.JSONDecodeError):
            pass
    tp["summary"] = summary
    params = summary.get("params", {})
    res = summary.get("result", {}) or sess_walk.get("result", {})
    ref = (params.get("tilt_ref_deg") or res.get("tilt_ref_deg")
           or [tp["roll"][0], tp["pitch"][0]])
    tp["ref_roll"], tp["ref_pitch"] = float(ref[0]), float(ref[1])
    pol = params.get("policy", {})
    tp["policy"] = (pol.get("source", "").rsplit("/", 1)[-1]
                    .removeprefix("ppo_goal_").removesuffix(".zip")
                    or sess_walk.get("tag", "?"))
    tp["vx_cmd"] = float(params.get("vx", res.get("vx_cmd", math.nan)))
    tp["vy_cmd"] = float(params.get("vy", res.get("vy_cmd", math.nan)))
    tp["result_ok"] = res.get("ok")
    tp["result_fell"] = res.get("fell")
    tp["result_error"] = res.get("error")
    return tp


def _first_sustained_below(rel: np.ndarray, t: np.ndarray,
                           thresh: float, hold_s: float) -> float | None:
    """First time from which |rel| stays <= thresh for hold_s."""
    ok = np.abs(rel) <= thresh
    for i in range(len(t)):
        if not ok[i]:
            continue
        j = np.searchsorted(t, t[i] + hold_s)
        if j > len(t) - 1:
            # window runs past the tape; only accept if it held to the
            # end AND the tape end is at least hold_s/2 past t[i]
            if ok[i:].all() and t[-1] - t[i] >= hold_s * 0.5:
                return float(t[i])
            return None
        if ok[i:j + 1].all():
            return float(t[i])
    return None


def hardware_metrics(tp: dict, window_s: float,
                     slew_limit_deg: float) -> dict:
    t = tp["t"]
    w = t <= window_s
    roll_rel = tp["roll"] - tp["ref_roll"]
    pitch_rel = tp["pitch"] - tp["ref_pitch"]

    # Engagement discontinuity: raw policy targets vs measured pose.
    q_prop = _CENTER_DEG + tp["act"] * _HALF_DEG          # (n, 18) deg
    d0 = np.abs(q_prop[0] - tp["q"][0])
    # Per-tick raw-target jumps (how discontinuous the policy's own
    # request stream is, before the safety clamp smooths it).
    d_prop = np.abs(np.diff(q_prop, axis=0))

    # Slew saturation on the POST-safety command stream.
    d_cmd = np.abs(np.diff(tp["cmd"], axis=0))            # (n-1, 18)
    sat = d_cmd >= (slew_limit_deg - SLEW_EPS_DEG)
    n_sat = sat.sum(axis=1)                               # per tick
    w1 = w[1:]

    # 5-deg crossing + command-ramp phase there.
    vref = np.hypot(tp["vx_ref"], tp["vy_ref"])
    cross_i = next((i for i in range(len(t))
                    if abs(roll_rel[i]) >= CROSS_DEG), None)
    t_cross = float(t[cross_i]) if cross_i is not None else None
    vref_cross = float(vref[cross_i]) if cross_i is not None else None
    phase_cross = None
    if cross_i is not None:
        vmax = float(np.max(vref)) or 1.0
        phase_cross = ("settle" if vref_cross < 1e-6 else
                       "ramp" if vref_cross < 0.98 * vmax else "hold")

    cur = tp["cur"]
    have_cur = np.isfinite(cur).any()
    cur_sum = np.nansum(np.abs(cur), axis=1) if have_cur else None

    track = np.abs(tp["cmd"] - tp["q"])                    # same tick

    i_peak = int(np.argmax(np.abs(roll_rel)))
    tail = tp["tail_roll"]
    tail_rel_mean = (float(np.mean(np.abs(tail - tp["ref_roll"])))
                     if len(tail) else None)
    if tp["result_fell"] is not None:
        fell = bool(tp["result_fell"]) or tp["result_ok"] is False
    else:
        fell = (tp["result_ok"] is False
                or (tail_rel_mean is not None and tail_rel_mean >= 10.0))

    # Time-to-stable is measured FROM THE TRANSIENT ONSET (the 5-deg
    # crossing): "how long until the takeoff excursion settled", not
    # "was the tape quiet at t=0" (it always is for a beat).
    if cross_i is None:
        t_stable = 0.0
    else:
        after = slice(cross_i, None)
        ts = _first_sustained_below(
            roll_rel[after], t[after], STABLE_DEG, STABLE_HOLD_S)
        t_stable = None if ts is None else round(ts, 2)

    return {
        "tape": tp["name"],
        "policy": tp["policy"],
        "cmd_mps": [tp["vx_cmd"], tp["vy_cmd"]],
        "n_run_ticks": int(len(t)),
        "window_s": window_s,
        # -- engagement discontinuity
        "raw_target_jump_t0_max_deg": round(float(d0.max()), 1),
        "raw_target_jump_t0_mean_deg": round(float(d0.mean()), 1),
        "joints_beyond_slew_t0": int((d0 >= slew_limit_deg).sum()),
        "raw_target_jump_win_max_deg": round(
            float(d_prop[w1].max()) if w1.any() else math.nan, 1),
        # -- saturation
        "act_sat_frac_t0": round(
            float((np.abs(tp["act"][0]) >= ACT_SAT).mean()), 2),
        "act_sat_frac_win": round(
            float((np.abs(tp["act"][w]) >= ACT_SAT).mean()), 2),
        "slew_sat_max_joints": int(n_sat[w1].max()) if w1.any() else 0,
        "slew_sat_mean_joints_win": round(
            float(n_sat[w1].mean()) if w1.any() else math.nan, 1),
        "slew_sat_frac_ticks_ge12_win": round(
            float((n_sat[w1] >= 12).mean()) if w1.any() else math.nan, 2),
        # -- attitude
        "roll_peak_win_deg": round(float(np.abs(roll_rel[w]).max()), 1),
        "roll_peak_full_deg": round(float(abs(roll_rel[i_peak])), 1),
        "t_roll_peak_s": round(float(t[i_peak]), 2),
        "rollrate_peak_win_dps": round(
            float(np.abs(tp["gyro_x"][w]).max()), 1),
        "rollrate_peak_full_dps": round(
            float(np.abs(tp["gyro_x"]).max()), 1),
        "pitch_peak_win_deg": round(float(np.abs(pitch_rel[w]).max()), 1),
        "t_cross5_s": t_cross,
        "vref_at_cross_mps": vref_cross,
        "ramp_phase_at_cross": phase_cross,
        # -- tracking + effort
        "track_err_win_max_deg": round(float(track[w].max()), 1),
        "track_err_win_rmse_deg": round(
            float(np.sqrt(np.mean(track[w] ** 2))), 2),
        "cur_servo_peak_win_a": round(
            float(np.nanmax(cur[w])), 2) if have_cur else None,
        "cur_sum_peak_win_a": round(
            float(np.max(cur_sum[w])), 2) if have_cur else None,
        # -- outcome
        "t_stable_s": t_stable,
        "tail_roll_rel_mean_deg": (round(tail_rel_mean, 1)
                                   if tail_rel_mean is not None else None),
        "fell": bool(fell),
        "result_error": tp["result_error"],
    }


def replay_metrics(tp: dict, window_s: float, params_tag: str) -> dict:
    """Matched open-loop sim replay: the per-foot contact story."""
    from rl_move.sim.replay_trace import _ReplaySim
    from rl_move.sim.servo_model import SimServoParams

    sim = _ReplaySim(SimServoParams.load())
    res = sim.replay({"q": tp["q"], "cmd": tp["cmd"], "t": tp["t"]})
    t = tp["t"]
    w = t <= window_s
    f = res["foot_f"]                                     # (n, 6) N
    on = f > 0.5
    xy = res["foot_xyz"][:, :, :2]
    slip = np.zeros_like(f)
    slip[1:] = (np.linalg.norm(np.diff(xy, axis=0), axis=2)
                * (on[1:] & on[:-1]))

    first_break = {}
    transitions = 0
    for foot in range(6):
        ch = np.flatnonzero(np.diff(on[w][:, foot].astype(int)) != 0)
        transitions += len(ch)
        lift = np.flatnonzero(on[w][:, foot][:-1] & ~on[w][:, foot][1:])
        if len(lift):
            first_break[f"L{foot}"] = round(float(t[lift[0] + 1]), 2)
    sim_roll_rel = res["roll"] - res["ref_roll"]
    return {
        "replay_params": params_tag,
        "foot_load_t0_n": [round(float(x), 2) for x in f[0]],
        "first_liftoff_s": dict(sorted(first_break.items(),
                                       key=lambda kv: kv[1])),
        "contact_transitions_win": int(transitions),
        "min_feet_on_win": int(on[w].sum(axis=1).min()),
        "loaded_slip_win_mm": round(float(slip[w].sum() * 1000.0), 1),
        "sim_roll_peak_win_deg": round(
            float(np.abs(sim_roll_rel[w]).max()), 1),
        "sim_roll_peak_full_deg": round(
            float(np.abs(sim_roll_rel).max()), 1),
    }


_TABLE_COLS = (
    ("tape", "tape"), ("policy", "policy"), ("fell", "fell"),
    ("jump0", "raw_target_jump_t0_max_deg"),
    ("sat0", "act_sat_frac_t0"),
    ("slew12+", "slew_sat_frac_ticks_ge12_win"),
    ("cross5s", "t_cross5_s"), ("@phase", "ramp_phase_at_cross"),
    ("pk_roll", "roll_peak_full_deg"), ("pk_t", "t_roll_peak_s"),
    ("pk_rate", "rollrate_peak_full_dps"),
    ("stable_s", "t_stable_s"),
)


def _fmt(v) -> str:
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "FELL" if v else "ok"
    if isinstance(v, float):
        return f"{v:.2f}".rstrip("0").rstrip(".")
    return str(v)


def render_table(recs: list[dict], md: bool = False) -> str:
    heads = [h for h, _ in _TABLE_COLS]
    rows = [[_fmt(r.get(k)) for _, k in _TABLE_COLS] for r in recs]
    if md:
        out = ["| " + " | ".join(heads) + " |",
               "|" + "|".join("---" for _ in heads) + "|"]
        out += ["| " + " | ".join(row) + " |" for row in rows]
        return "\n".join(out)
    widths = [max(len(h), *(len(r[i]) for r in rows))
              for i, h in enumerate(heads)]
    out = ["  ".join(h.ljust(widths[i]) for i, h in enumerate(heads))]
    out += ["  ".join(r[i].ljust(widths[i]) for i in range(len(heads)))
            for r in rows]
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", type=Path, nargs="*", default=[])
    ap.add_argument("--all", action="store_true",
                    help="every rl_walk_*.csv under hardware_traces/")
    ap.add_argument("--window-s", type=float, default=1.5)
    ap.add_argument("--replay", action="store_true",
                    help="matched open-loop sim replay per tape "
                         "(per-foot loading / contact breaks; needs "
                         "mujoco, ~seconds per tape)")
    ap.add_argument("--json-out", type=Path, default=None)
    ap.add_argument("--md-out", type=Path, default=None)
    args = ap.parse_args(argv)

    paths = list(args.csv)
    if args.all:
        paths += sorted(TRACES.glob("**/rl_walk_*.csv"))
    if not paths:
        ap.error("no tapes: pass --csv or --all")

    cfg = load_config(str(_RL / "config.yaml"))
    slew = float(cfg_get(cfg, "safety", "max_delta_q_deg", default=1.5))

    recs = []
    for p in paths:
        try:
            tp = load_tape(p)
        except ValueError as e:
            print(f"skip: {e}")
            continue
        rec = hardware_metrics(tp, args.window_s, slew)
        if args.replay:
            rec["replay"] = replay_metrics(tp, args.window_s, "air")
        recs.append(rec)

    print(render_table(recs))
    n_fell = sum(1 for r in recs if r["fell"])
    crossings = [r["t_cross5_s"] for r in recs
                 if r["t_cross5_s"] is not None]
    phases = [r["ramp_phase_at_cross"] for r in recs
              if r["ramp_phase_at_cross"]]
    agg = {
        "n_tapes": len(recs),
        "n_fell": n_fell,
        "cross5_n": len(crossings),
        "cross5_median_s": (round(float(np.median(crossings)), 2)
                            if crossings else None),
        "cross_phase_counts": {ph: phases.count(ph)
                               for ph in ("settle", "ramp", "hold")},
        "jump0_median_deg": round(float(np.median(
            [r["raw_target_jump_t0_max_deg"] for r in recs])), 1),
        "act_sat0_median": round(float(np.median(
            [r["act_sat_frac_t0"] for r in recs])), 2),
        "slew_sat_ge12_median": round(float(np.median(
            [r["slew_sat_frac_ticks_ge12_win"] for r in recs])), 2),
    }
    print(f"\n{agg['n_tapes']} tapes, {n_fell} fell; 5-deg crossing in "
          f"{agg['cross5_n']} (median {agg['cross5_median_s']} s, "
          f"phase counts {agg['cross_phase_counts']}); median t0 raw "
          f"target jump {agg['jump0_median_deg']} deg, median t0 action "
          f"saturation {agg['act_sat0_median']}, median frac of window "
          f"ticks with >=12 slew-pinned joints "
          f"{agg['slew_sat_ge12_median']}")

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(
            {"window_s": args.window_s, "aggregate": agg, "tapes": recs},
            indent=1))
        print(f"wrote {args.json_out}")
    if args.md_out:
        args.md_out.parent.mkdir(parents=True, exist_ok=True)
        lines = ["# Walk-takeoff audit", "",
                 f"Window: engagement -> {args.window_s} s. "
                 f"Generated by `rl_move.scripts.takeoff_audit`.", "",
                 render_table(recs, md=True), "",
                 "```json", json.dumps(agg, indent=1), "```", ""]
        args.md_out.write_text("\n".join(lines))
        print(f"wrote {args.md_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
