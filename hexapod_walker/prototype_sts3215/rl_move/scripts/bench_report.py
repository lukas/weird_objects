#!/usr/bin/env python3
"""bench_report — every bench_blast session, one markdown table.

The fast way to read a night of hardware sessions without opening a
single summary.json by hand:

    python -m rl_move.scripts.bench_report                # all sessions
    python -m rl_move.scripts.bench_report --since 20260811_19
    python -m rl_move.scripts.bench_report --out report.md

For each session it reads summary.json (walks, transitions, recoveries,
turn signs, aborts) and every pulled rl_walk CSV (per-tick roll trace)
and prints:

- a per-walk table: policy, direction, takeoff-transient timing
  (when |roll| first crosses 5 deg, when it peaks), peak, tail
  (mean |roll| over the last second), and the verdict fell/recovered/
  clean — the fell/tail split the "runaway" flag conflates,
- a per-rise table: tick + max relative roll of every learned stand
  attempt (the mid-curl trip signature),
- per-policy fell-rate tallies across all selected sessions.

Companion to bench_blast.py (writes the sessions) and video_review.py
(cuts the camera frames). Reads ONLY local files — never the robot.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

TRACES = Path(__file__).resolve().parents[1] / "hardware_traces"
TRANSIENT_DEG = 5.0     # |roll| crossing that marks the takeoff transient
FELL_TAIL_DEG = 10.0    # mean |roll| over the last second => ended down


def walk_csv_metrics(path: Path) -> dict | None:
    """Roll-trace shape of one episode CSV."""
    try:
        with path.open() as f:
            rows = [r for r in csv.DictReader(f) if r.get("roll_deg")]
    except OSError:
        return None
    if len(rows) < 10:
        return {"file": path.name, "error": f"only {len(rows)} rows"}
    t = [float(r["t_s"]) for r in rows]
    roll = [float(r["roll_deg"]) for r in rows]
    roll0 = roll[0]                       # start-relative: mounts differ
    rel = [abs(x - roll0) for x in roll]
    peak = max(rel)
    i_peak = rel.index(peak)
    t_cross = next((t[i] for i, x in enumerate(rel) if x >= TRANSIENT_DEG),
                   None)
    tail_from = t[-1] - 1.0
    tail = [x for i, x in enumerate(rel) if t[i] >= tail_from]
    tail_mean = sum(tail) / len(tail) if tail else 0.0
    cur_cols = [k for k in rows[0] if k.startswith("cur") and
                k.endswith("_a")]
    max_cur = max((max(abs(float(r[c] or 0)) for c in cur_cols)
                   for r in rows), default=0.0)
    rot60 = any((r.get("rot60_k") or "0") not in ("0", "") for r in rows)
    return {
        "file": path.name, "dur_s": round(t[-1], 1), "ticks": len(rows),
        "t_transient_s": None if t_cross is None else round(t_cross, 2),
        "t_peak_s": round(t[i_peak], 2), "peak_deg": round(peak, 1),
        "tail_deg": round(tail_mean, 1), "max_cur_a": round(max_cur, 2),
        "rot60": rot60,
        "verdict": ("FELL" if tail_mean >= FELL_TAIL_DEG else
                    "recovered" if peak >= FELL_TAIL_DEG else "clean"),
    }


def load_session(d: Path) -> dict | None:
    sp = d / "summary.json"
    if not sp.exists():
        return None
    s = json.loads(sp.read_text())
    csv_metrics = {}
    for c in sorted(d.glob("rl_walk_*.csv")):
        m = walk_csv_metrics(c)
        if m:
            csv_metrics[c.name] = m
    return {"dir": d.name, "summary": s, "csv": csv_metrics}


def fmt(v) -> str:
    if v is None:
        return "—"
    if isinstance(v, float) and math.isnan(v):
        return "—"
    return str(v)


def report(sessions: list[dict]) -> str:
    out: list[str] = []
    walks_by_policy: dict[str, list[str]] = {}
    rises: list[tuple[str, dict]] = []

    for sess in sessions:
        s = sess["summary"]
        out.append(f"\n## {sess['dir']}")
        ab = s.get("ab_tally")
        aborted = next((e["text"] for e in s.get("events", [])
                        if "abort" in e["text"].lower()), None)
        if aborted:
            out.append(f"- **aborted:** {aborted}")
        if ab:
            out.append(f"- ab_tally: {json.dumps(ab)}")

        walks = s.get("walks", [])
        if walks:
            out.append("\n| walk | dir | csv | transient@ | peak@ | peak | "
                       "tail | maxA | verdict |")
            out.append("|---|---|---|---|---|---|---|---|---|")
            for w in walks:
                tag = w.get("tag", "?")
                pol = tag.split("-r")[0]
                m = sess["csv"].get(w.get("csv") or "", {})
                if not m:      # fall back to the closest unclaimed csv
                    m = next((v for v in sess["csv"].values()
                              if v not in walks_by_policy.get("_used", [])),
                             {})
                vx = w.get("vx", w.get("vx_mps"))
                d = ("fwd" if (vx or 0) > 0 else
                     "back" if (vx or 0) < 0 else "?")
                verdict = m.get("verdict", "?")
                out.append(
                    f"| {tag} | {d} | {fmt(m.get('file'))[8:23]} | "
                    f"{fmt(m.get('t_transient_s'))}s | "
                    f"{fmt(m.get('t_peak_s'))}s | "
                    f"{fmt(m.get('peak_deg'))} | {fmt(m.get('tail_deg'))} | "
                    f"{fmt(m.get('max_cur_a'))} | **{verdict}** |")
                walks_by_policy.setdefault(pol, []).append(verdict)

        trans = s.get("transitions", [])
        for tr in trans:
            if tr.get("mode") != "stand":
                continue
            res = tr.get("result") or {}
            rises.append((sess["dir"], res))
        turns = s.get("turnsign", [])
        for t in turns:
            out.append(f"- turnsign omega {t.get('omega'):+}: "
                       f"{t.get('observed', '?')}"
                       + (f" ({t['error']})" if t.get("error") else ""))
        recs = s.get("recoveries", [])
        if recs:
            out.append(f"- recoveries: {len(recs)} "
                       f"(after {[r.get('after') for r in recs]})")

    if rises:
        out.append("\n## Learned rise attempts (all selected sessions)")
        out.append("\n| session | ok | error | ticks | max roll | maxA |")
        out.append("|---|---|---|---|---|---|")
        for d, r in rises:
            out.append(f"| {d[-6:]} | {r.get('ok')} | "
                       f"{fmt(r.get('error'))} | {fmt(r.get('ticks'))} | "
                       f"{fmt(r.get('tilt_rel_max_deg'))} | "
                       f"{fmt(r.get('max_current_a'))} |")

    out.append("\n## Per-policy walk verdicts")
    for pol, vs in sorted(walks_by_policy.items()):
        if pol.startswith("_"):
            continue
        n = len(vs)
        fell = sum(1 for v in vs if v == "FELL")
        rec = sum(1 for v in vs if v == "recovered")
        clean = sum(1 for v in vs if v == "clean")
        out.append(f"- **{pol}**: {n} walks — {fell} fell, "
                   f"{rec} recovered a transient, {clean} clean")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--since", default="",
                    help="only sessions whose stamp sorts >= this "
                         "(e.g. 20260811_19)")
    ap.add_argument("--out", default="", help="also write markdown here")
    args = ap.parse_args()

    dirs = sorted(d for d in TRACES.glob("bench_blast_*") if d.is_dir()
                  and d.name.removeprefix("bench_blast_") >= args.since)
    sessions = [s for d in dirs if (s := load_session(d))]
    if not sessions:
        print("no sessions found")
        return 1
    md = f"# bench_report — {len(sessions)} session(s)\n" + report(sessions)
    print(md)
    if args.out:
        Path(args.out).write_text(md + "\n")
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
