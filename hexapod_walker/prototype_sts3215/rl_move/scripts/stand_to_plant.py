#!/usr/bin/env python3
"""DISABLED by default — do not auto-stand to +20/+80.

Phase-1 policy (2026-08-06): physically set a stable stance, then
``uv run python -m rl_move.scripts.capture_plant``. Balance startup holds
current joints and refuses if far from that snapshot.

This script only runs with ``--force`` *and* a captured ``joints_deg``
plant (never the knee-limit default +80°).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from feetech_bus import load_plant_pose, standing_pose_degrees  # noqa: E402
from rl_move.config import cfg_get, load_config  # noqa: E402
from rl_move.env import open_bus  # noqa: E402
from rl_move.robot_state import DEG2RAD, RAD2DEG, RobotStateEstimator  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true",
                    help="required: blend only to a captured joints_deg plant")
    args = ap.parse_args()

    plant = load_plant_pose()
    if not args.force:
        print("[stand] REFUSED: auto stand-to-plant is disabled.")
        print("  Physically set a stable feet-down stance, then:")
        print("    uv run python -m rl_move.scripts.capture_plant")
        print("  Balance will hold current pose (no blend).")
        print("  (Emergency blend to a *captured* plant: --force)")
        return 2
    if plant.get("joints_deg") is None:
        print("[stand] REFUSED: --force needs plant_pose.json with joints_deg.")
        print("  Default +20/+80 tips the robot — will not use it.")
        print("  Run: uv run python -m rl_move.scripts.capture_plant")
        return 2

    cfg = load_config(args.config)
    max_tilt = math.radians(float(
        cfg_get(cfg, "episode", "preflight_max_tilt_deg", default=12)))
    speed = int(cfg_get(cfg, "bus", "stand_speed", default=200))
    acc = int(cfg_get(cfg, "bus", "write_acc", default=20))

    print("[stand] opening bus (stop hexapod-web first if needed)")
    bus, port = open_bus(cfg)
    print(f"[stand] bus={port}")
    est = RobotStateEstimator(bus, cfg)

    for _ in range(5):
        st = est.update()
        time.sleep(0.04)

    q0 = st.joint_position.copy()
    q1 = np.array(standing_pose_degrees(), dtype=float) * DEG2RAD
    print(f"[stand] tilt roll={st.imu_roll*RAD2DEG:+.1f}° "
          f"pitch={st.imu_pitch*RAD2DEG:+.1f}°")
    print(f"[stand] target = captured plant ({plant.get('path')})")
    print(f"[stand] current L0="
          f"{(q0[0]*RAD2DEG):+.1f}/{(q0[1]*RAD2DEG):+.1f}/{(q0[2]*RAD2DEG):+.1f}")
    print(f"[stand] max |Δq|={float(np.max(np.abs(q1-q0))*RAD2DEG):.1f}°  "
          f"blend={args.seconds}s  dry_run={args.dry_run}")

    if abs(st.imu_roll) > max_tilt or abs(st.imu_pitch) > max_tilt:
        print("[stand] ABORT: already tilted — upright the chassis first")
        try:
            bus.close()
        except Exception:
            pass
        return 2

    if args.dry_run:
        print("[stand] dry-run done (no motion)")
        bus.close()
        return 0

    try:
        if hasattr(bus, "enable_all_torque"):
            bus.enable_all_torque(True)
    except Exception as e:
        print(f"[stand] torque enable warn: {e}")

    n = max(2, int(args.seconds / 0.05))
    t0 = time.monotonic()
    for i in range(n):
        a = (i + 1) / n
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        q = (1.0 - s) * q0 + s * q1
        bus.write_all((q * RAD2DEG).tolist(), speed=speed, acc=acc)
        time.sleep(0.05)
        st = est.update()
        if abs(st.imu_roll) > max_tilt or abs(st.imu_pitch) > max_tilt:
            print(f"[stand] TILT during blend — limp "
                  f"(roll={st.imu_roll*RAD2DEG:+.1f}° "
                  f"pitch={st.imu_pitch*RAD2DEG:+.1f}°)")
            try:
                bus.enable_all_torque(False)
            except Exception:
                pass
            bus.close()
            return 1
        if (i + 1) % 20 == 0 or i + 1 == n:
            print(f"  {100*a:5.1f}%  tilt=({st.imu_roll*RAD2DEG:+.1f}°, "
                  f"{st.imu_pitch*RAD2DEG:+.1f}°)  "
                  f"L0 hip/knee="
                  f"{st.joint_position[1]*RAD2DEG:+.1f}/"
                  f"{st.joint_position[2]*RAD2DEG:+.1f}")

    bus.write_all((q1 * RAD2DEG).tolist(), speed=speed, acc=acc)
    print(f"[stand] holding plant {args.hold}s …")
    t_end = time.monotonic() + args.hold
    while time.monotonic() < t_end:
        st = est.update()
        time.sleep(0.05)
        if abs(st.imu_roll) > max_tilt or abs(st.imu_pitch) > max_tilt:
            print("[stand] TILT during hold — limp")
            try:
                bus.enable_all_torque(False)
            except Exception:
                pass
            bus.close()
            return 1

    st = est.update(want_full_feedback=True)
    print(f"[stand] DONE in {time.monotonic()-t0:.1f}s  "
          f"tilt=({st.imu_roll*RAD2DEG:+.1f}°, {st.imu_pitch*RAD2DEG:+.1f}°)")
    try:
        bus.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
