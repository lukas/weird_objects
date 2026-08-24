"""Build the standard versioned sysid protocol files.

Protocols are deterministic experiment definitions (schema + materializer
in ``linux_control/sysid_protocol.py`` — shared with the on-robot
runner). The standard set covers the sysid plan:

- ``steps_air_v1``    — Phase 1 step ladders (±5/10/20/40°, clamped to
  axis limits) on one representative joint per axis (leg 2, matching
  motor_dynamics.FULL_LEG).
- ``sines_air_v1``    — Phase 1 frequency response (0.25/0.5/1/2 Hz at
  two amplitudes) on the same joints.
- ``servo_spread_v1`` — Phase 6 per-servo variation: a short ±10° step
  battery on EVERY joint.
- ``champion`` (CLI)  — Phase 8 suspended champion replay: package the
  ``cmd*_deg`` stream of a recorded episode CSV (hardware or sim) as an
  absolute whole-body ``traj`` protocol.

Build (from prototype_sts3215/, repo .venv)::

    uv run python -m sysid.protocols build
    uv run python -m sysid.protocols champion --csv <rl_*.csv> --out <name> \
        [--start-tick 0 --ticks 400]
    uv run python -m sysid.protocols show sysid/protocols/steps_air_v1.json
"""
from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

from . import PROTOCOL_DIR  # noqa: F401  (bootstraps sys.path)
from sysid_protocol import (  # noqa: E402
    AXIS_LIMITS_DEG, DEFAULT_HZ, N_JOINTS, axis_of, duration_s,
    materialize, protocol_hash,
)

# Representative joints: leg 2 (same leg motor_dynamics gives the full
# battery), joints = leg*3 + axis.
REP_JOINTS = {"yaw": 6, "hip": 7, "knee": 8}

STEP_AMPS = (5.0, 10.0, 20.0, 40.0)
SINE_FREQS = (0.25, 0.5, 1.0, 2.0)
SINE_AMPS = (4.0, 8.0)
LIMIT_MARGIN_DEG = 3.0


def _amp_ok(joint: int, amp: float) -> bool:
    """Is ``home(0°) + amp`` inside the axis limits with margin?

    Step/sine commands are relative to the segment home; the standard
    protocols assume the bench zero pose (legs straight out, all 0°).
    """
    lo, hi = AXIS_LIMITS_DEG[axis_of(joint)]
    return lo + LIMIT_MARGIN_DEG <= amp <= hi - LIMIT_MARGIN_DEG


def _doc(name: str, description: str, segments: list[dict],
         **overrides) -> dict:
    doc = {
        "sysid_protocol": 1,
        "name": name,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "description": description,
        "hz": DEFAULT_HZ,
        "segments": segments,
    }
    doc.update(overrides)
    return doc


def build_steps(joints: dict[str, int] | None = None,
                amps: tuple[float, ...] = STEP_AMPS,
                repeats: int = 5, *, hold_s: float = 2.0,
                pre_s: float = 0.8, rest_s: float = 1.6,
                name: str = "steps_air_v1") -> dict:
    # pre/rest are generous on purpose: each segment's home is the
    # present pose at segment start, so the previous return must have
    # fully settled (a ±40 deg loaded knee step needs >1 s) or the next
    # step's latency/tracking metrics are polluted.
    """Phase 1 step ladders: ±amp × repeats per representative joint."""
    joints = joints or REP_JOINTS
    segs: list[dict] = []
    for ax, j in joints.items():
        for rep in range(repeats):
            for a in amps:
                for signed in (+a, -a):
                    if not _amp_ok(j, signed):
                        continue
                    segs.append({
                        "kind": "step", "joint": j, "amp_deg": signed,
                        "pre_s": pre_s, "hold_s": hold_s, "rest_s": rest_s,
                        "label": f"L{j // 3}_{ax}_step{signed:+g}_r{rep}",
                    })
    return _doc(name,
                f"Phase 1 unloaded step ladders ±{'/'.join(f'{a:g}' for a in amps)} deg, "
                f"{repeats} repeats, joints {sorted(joints.values())}. "
                "Robot on stand, feet OFF the ground; the runner glides "
                "the legs to bench zero by itself.",
                segs, home_deg=[0.0] * N_JOINTS)


def build_sines(joints: dict[str, int] | None = None,
                freqs: tuple[float, ...] = SINE_FREQS,
                amps: tuple[float, ...] = SINE_AMPS, *,
                name: str = "sines_air_v1") -> dict:
    """Phase 1 frequency response: sines per joint × freq × amplitude."""
    joints = joints or REP_JOINTS
    segs: list[dict] = []
    for ax, j in joints.items():
        for a in amps:
            if not (_amp_ok(j, a) and _amp_ok(j, -a)):
                continue
            for f in freqs:
                cycles = max(3, int(round(3 * f)))
                segs.append({
                    "kind": "sine", "joint": j, "amp_deg": a,
                    "freq_hz": f, "cycles": cycles,
                    "pre_s": 0.6, "rest_s": 0.8,
                    "label": f"L{j // 3}_{ax}_sine{a:g}@{f:g}Hz",
                })
    return _doc(name,
                "Phase 1 unloaded frequency response "
                f"({'/'.join(f'{f:g}' for f in freqs)} Hz at "
                f"{'/'.join(f'{a:g}' for a in amps)} deg). Robot on "
                "stand, feet OFF the ground; the runner glides the legs "
                "to bench zero by itself.",
                segs, home_deg=[0.0] * N_JOINTS)


def build_servo_spread(amp: float = 10.0, repeats: int = 3, *,
                       name: str = "servo_spread_v1") -> dict:
    """Phase 6: reduced ±amp battery on EVERY joint (per-servo spread)."""
    segs: list[dict] = []
    for j in range(N_JOINTS):
        for rep in range(repeats):
            for signed in (+amp, -amp):
                if not _amp_ok(j, signed):
                    continue
                segs.append({
                    "kind": "step", "joint": j, "amp_deg": signed,
                    "pre_s": 0.5, "hold_s": 1.2, "rest_s": 0.6,
                    "label": (f"L{j // 3}_{axis_of(j)}_"
                              f"step{signed:+g}_r{rep}"),
                })
    return _doc(name,
                f"Phase 6 per-servo variation: ±{amp:g} deg × {repeats} "
                "on every joint. Robot on stand, feet OFF the ground; "
                "the runner glides the legs to bench zero by itself.",
                segs, home_deg=[0.0] * N_JOINTS)


def build_champion_traj(csv_path: Path, *, start_tick: int = 0,
                        ticks: int = 0, name: str = "",
                        start_tol_deg: float = 12.0) -> dict:
    """Phase 8: package an episode CSV's cmd stream as a traj protocol.

    Reads ``cmd*_deg`` (post-safety commands the servos actually
    received) from a hardware or sim ``rl_*.csv`` trace. Replay it
    SUSPENDED (Test A) — the runner glides to the first row by itself,
    then verifies the pose before streaming.
    """
    rows = [r for r in csv.DictReader(open(csv_path))
            if r.get("phase", "run") == "run"]
    if not rows:
        raise SystemExit(f"{csv_path}: no run-phase rows")
    rows = rows[start_tick:start_tick + ticks if ticks else None]
    t0 = float(rows[0]["t_s"])
    t_s = [round(float(r["t_s"]) - t0, 4) for r in rows]
    q_deg = [[float(r[f"cmd{j}_deg"]) for j in range(N_JOINTS)]
             for r in rows]
    name = name or f"champion_{csv_path.stem}"
    return _doc(
        name,
        f"Phase 8 suspended champion replay of {csv_path.name} "
        f"ticks {start_tick}..{start_tick + len(rows)}. TEST A: robot "
        "suspended, feet off the ground, operator watching. The runner "
        "glides to the trajectory's start pose by itself; requires "
        "force=true.",
        [{"kind": "traj", "t_s": t_s, "q_deg": q_deg,
          "start_tol_deg": start_tol_deg,
          "label": f"traj_{csv_path.stem}"}])


def save(doc: dict, out_dir: Path = PROTOCOL_DIR) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{doc['name']}.json"
    path.write_text(json.dumps(doc, indent=1, sort_keys=True))
    print(f"  {path.name}: {len(doc['segments'])} segments, "
          f"{duration_s(doc):.0f} s @ {doc['hz']:g} Hz, "
          f"hash {protocol_hash(doc)}")
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="write the standard protocol files")
    b.add_argument("--repeats", type=int, default=5,
                   help="step repeats per amp/direction (plan asks ~20; "
                        "5 keeps the first bench session short)")
    b.add_argument("--out-dir", type=Path, default=PROTOCOL_DIR)
    c = sub.add_parser("champion", help="traj protocol from an episode CSV")
    c.add_argument("--csv", type=Path, required=True)
    c.add_argument("--start-tick", type=int, default=0)
    c.add_argument("--ticks", type=int, default=0, help="0 = all")
    c.add_argument("--out", default="", help="protocol name")
    c.add_argument("--out-dir", type=Path, default=PROTOCOL_DIR)
    s = sub.add_parser("show", help="summarize a protocol file")
    s.add_argument("path", type=Path)
    args = ap.parse_args(argv)

    if args.cmd == "build":
        save(build_steps(repeats=args.repeats), args.out_dir)
        save(build_sines(), args.out_dir)
        save(build_servo_spread(), args.out_dir)
    elif args.cmd == "champion":
        save(build_champion_traj(args.csv, start_tick=args.start_tick,
                                 ticks=args.ticks, name=args.out),
             args.out_dir)
    else:
        doc = json.loads(args.path.read_text())
        mat = materialize(doc)
        print(f"{doc['name']}  hash {protocol_hash(doc)}  "
              f"{len(mat['ticks'])} ticks @ {mat['hz']:g} Hz "
              f"({len(mat['ticks']) / mat['hz']:.0f} s)")
        for lbl in mat["seg_labels"]:
            print(f"  {lbl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
