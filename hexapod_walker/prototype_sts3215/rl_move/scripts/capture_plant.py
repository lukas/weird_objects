#!/usr/bin/env python3
"""Capture the current 18-joint stance as the RL plant (no motion).

Physically put the robot in a wide, stable, feet-down pose first, then run
this. Writes ``plant_pose.json`` with full ``joints_deg[18]`` so balance
startup can freeze / verify against a real stance — not default +20/+80.
"""
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

from feetech_bus import save_plant_pose  # noqa: E402
from rl_move.config import load_config  # noqa: E402
from rl_move.env import open_bus  # noqa: E402
from rl_move.robot_state import RAD2DEG, RobotStateEstimator  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--samples", type=int, default=8,
                    help="median over N reads (default 8)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    print("[capture] opening bus (stop hexapod-web first if needed)")
    bus, port = open_bus(cfg)
    print(f"[capture] bus={port}")
    est = RobotStateEstimator(bus, cfg)

    samples = []
    st = None
    for _ in range(max(3, args.samples)):
        st = est.update(want_full_feedback=True)
        samples.append(st.joint_position.copy())
        time.sleep(0.05)
    assert st is not None

    # Median per joint (radians → deg).
    import numpy as np
    stack = np.stack(samples, axis=0)
    q_deg = (np.median(stack, axis=0) * RAD2DEG).tolist()

    hips = [q_deg[i] for i in range(1, 18, 3)]
    knees = [q_deg[i] for i in range(2, 18, 3)]
    hip_med = float(statistics.median(hips))
    knee_med = float(statistics.median(knees))

    print(f"[capture] tilt roll={st.imu_roll*RAD2DEG:+.1f}° "
          f"pitch={st.imu_pitch*RAD2DEG:+.1f}°")
    for leg in range(6):
        y, h, k = q_deg[leg * 3: leg * 3 + 3]
        print(f"  L{leg} yaw/hip/knee={y:+6.1f}/{h:+6.1f}/{k:+6.1f}")
    print(f"[capture] median hip/knee={hip_med:+.1f}/{knee_med:+.1f}")

    path = save_plant_pose(
        hip_med, knee_med,
        extra={
            "joints_deg": [round(float(x), 3) for x in q_deg],
            "source": "manual_capture",
            "contact_found": True,
            "imu_roll_deg": round(float(st.imu_roll * RAD2DEG), 3),
            "imu_pitch_deg": round(float(st.imu_pitch * RAD2DEG), 3),
        },
    )
    print(f"[capture] saved → {path}")
    print("[capture] Next: keep robot near this pose; balance env will "
          "hold current joints and refuse if far from plant.")
    try:
        bus.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
