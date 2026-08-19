#!/usr/bin/env python3
"""Measure feedback / control-tick rates over the Feetech bus.

Read-only by default — safe to run any time (no torque, no motion):

    python3 bus_bench.py                 # bench every read path, 3 s each
    python3 bus_bench.py --seconds 5

Optionally bench the combined write+snapshot tick ('S'). This DOES
SyncWrite — it re-commands the PRESENT pose at hold speed, so nothing
moves, but only run it when the operator asks for it:

    python3 bus_bench.py --step

Interpreting results (2026-08-19 stream-bridge upgrade): with STREAM
firmware the MCU free-runs acquisition and every read here is served
from its RAM caches — expect read_snapshot/read_all_positions well over
100 Hz. On legacy firmware each call blocks on 18 servo replies; the
old measured tick (write + positions + IMU) was >20 ms (~40 Hz ceiling,
the reason rl_move ran at 25 Hz).
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from mcu_feetech_bus import open_feetech_bus  # noqa: E402


def bench(label: str, fn, seconds: float) -> None:
    times: list[float] = []
    ok = 0
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        try:
            r = fn()
        except Exception:
            r = None
        times.append(time.monotonic() - t0)
        if r:
            ok += 1
    if not times:
        print(f"  {label:<22} no samples")
        return
    mean = statistics.fmean(times)
    print(f"  {label:<22} {1.0 / mean:7.1f} Hz   "
          f"mean {mean * 1000:6.2f} ms   "
          f"p max {max(times) * 1000:6.2f} ms   "
          f"ok {ok}/{len(times)}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--step", action="store_true",
                    help="ALSO bench step_all (SyncWrites the present "
                         "pose at hold speed — servos should not move, "
                         "but this touches the bus as a writer)")
    args = ap.parse_args(argv)

    bus, port = open_feetech_bus(args.port)
    streaming = getattr(bus, "streaming", False)
    print(f"bus: {port}   stream mode: {'ON' if streaming else 'OFF'}")
    print(f"benching {args.seconds:.0f} s per path ...")

    if hasattr(bus, "read_snapshot"):
        bench("read_snapshot (S n=0)", bus.read_snapshot, args.seconds)
    if hasattr(bus, "read_all_positions"):
        bench("read_all_positions", bus.read_all_positions, args.seconds)
    if hasattr(bus, "read_all_feedback"):
        bench("read_all_feedback", bus.read_all_feedback, args.seconds)
    if hasattr(bus, "read_imu"):
        bench("read_imu", bus.read_imu, args.seconds)

    if args.step and hasattr(bus, "step_all"):
        pose = bus.read_all_positions()
        if not isinstance(pose, dict) or len(pose) < 18:
            print("  step bench skipped: incomplete position read")
        else:
            degrees = [float(pose[j]) for j in range(18)]
            # Hold speed / gentle acc: re-commanding the present pose.
            bench("step_all (write+snap)",
                  lambda: bus.step_all(degrees, speed=250, acc=40),
                  args.seconds)

    try:
        bus.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
