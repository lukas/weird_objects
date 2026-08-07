#!/usr/bin/env python3
"""Step B: measure RobotStateEstimator acquisition timing on hardware."""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.config import load_config  # noqa: E402
from rl_move.env import open_bus  # noqa: E402
from rl_move.robot_state import RobotStateEstimator  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--hz", type=float, default=50.0)
    args = ap.parse_args()

    cfg = load_config(args.config)
    bus, port = open_bus(cfg)
    print(f"[probe] bus={port}")
    est = RobotStateEstimator(bus, cfg)

    # Warmup
    for _ in range(5):
        est.update()

    n = max(1, int(args.seconds * args.hz))
    period = 1.0 / args.hz
    t_pos, t_imu, t_tot, t_fb = [], [], [], []
    rolls, pitches = [], []
    n_fb = 0
    bus_fail = imu_fail = 0
    t_next = time.monotonic()
    for i in range(n):
        t_next += period
        st = est.update()
        tm = est.last_timing
        t_pos.append(tm.t_pos * 1000)
        t_imu.append(tm.t_imu * 1000)
        t_tot.append(tm.t_total * 1000)
        if tm.did_full_feedback:
            t_fb.append(tm.t_fb * 1000)
            n_fb += 1
        if not st.bus_ok:
            bus_fail += 1
        if not st.imu_ok:
            imu_fail += 1
        rolls.append(st.imu_roll)
        pitches.append(st.imu_pitch)
        remain = t_next - time.monotonic()
        if remain > 0:
            time.sleep(remain)

    def pct(xs, p):
        if not xs:
            return float("nan")
        s = sorted(xs)
        return s[min(len(s) - 1, int(round((p / 100) * (len(s) - 1))))]

    print(f"[probe] ticks={n}  target={args.hz} Hz")
    print(f"  read_all_positions  mean={statistics.mean(t_pos):.2f} ms  "
          f"p95={pct(t_pos,95):.2f} ms  max={max(t_pos):.2f} ms")
    print(f"  read_imu            mean={statistics.mean(t_imu):.2f} ms  "
          f"p95={pct(t_imu,95):.2f} ms  max={max(t_imu):.2f} ms")
    print(f"  combined acq        mean={statistics.mean(t_tot):.2f} ms  "
          f"p95={pct(t_tot,95):.2f} ms  max={max(t_tot):.2f} ms")
    if t_fb:
        print(f"  full_feedback (n={n_fb}) mean={statistics.mean(t_fb):.2f} ms  "
              f"max={max(t_fb):.2f} ms")
    else:
        print("  full_feedback: none this run")
    print(f"  bus_fail_ticks={bus_fail}  imu_fail_ticks={imu_fail}")
    print(f"  roll mean={statistics.mean(rolls):+.4f} rad "
          f"({statistics.mean(rolls)*180/3.14159:+.2f} deg)  "
          f"pitch mean={statistics.mean(pitches):+.4f} rad "
          f"({statistics.mean(pitches)*180/3.14159:+.2f} deg)")
    q = est.update().joint_position
    print(f"  sample q_deg[0:3]={[(q[i]*180/3.14159) for i in range(3)]}")
    try:
        bus.close()
    except Exception:
        pass
    return 0 if bus_fail == 0 and imu_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
