#!/usr/bin/env python3
"""Scripted ±1° body roll then pitch with feet planted."""
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

from rl_move.config import cfg_get, load_config  # noqa: E402
from rl_move.env import HexapodBalanceEnv, open_bus  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--amp-deg", type=float, default=1.0)
    ap.add_argument("--period", type=float, default=4.0)
    ap.add_argument("--cycles", type=float, default=2.0)
    args = ap.parse_args()

    cfg = load_config(args.config)
    motion = bool(cfg_get(cfg, "bus", "enable_motion", default=False))
    print(f"[balance_sine] enable_motion={motion}  "
          f"hold_current={cfg_get(cfg, 'episode', 'hold_current_pose')}")
    bus, port = open_bus(cfg)
    print(f"[balance_sine] bus={port}")
    env = HexapodBalanceEnv(bus, cfg, log=True)
    hz = float(cfg_get(cfg, "control", "hz", default=25))
    dt = 1.0 / hz
    max_roll = float(cfg_get(cfg, "actions", "max_roll_deg", default=3))
    max_pitch = float(cfg_get(cfg, "actions", "max_pitch_deg", default=3))
    amp = args.amp_deg
    a_roll = amp / max(max_roll, 1e-6)
    a_pitch = amp / max(max_pitch, 1e-6)

    def run_axis(axis: int, scale: float, label: str) -> bool:
        obs, info = env.reset()
        del obs
        print(f"[balance_sine] {label} ±{amp}°  ep={info.get('episode')}  "
              f"start_tilt=({info.get('roll_deg', float('nan')):+.1f}°, "
              f"{info.get('pitch_deg', float('nan')):+.1f}°)")
        n = int(args.cycles * args.period * hz)
        terminated = truncated = False
        overruns = 0
        for i in range(n):
            if terminated:
                print(f"  early stop: {info.get('termination_reason')}")
                return False
            t0 = time.monotonic()
            phase = 2 * math.pi * (i / max(1, args.period * hz))
            action = np.zeros(5, dtype=float)
            action[axis] = scale * math.sin(phase)
            obs, rew, terminated, truncated, info = env.step(action)
            del obs, rew, truncated
            remain = dt - (time.monotonic() - t0)
            if remain > 0:
                time.sleep(remain)
            else:
                overruns += 1
        st = env._state
        print(f"  done {label}; end_tilt=({st.imu_roll*180/math.pi:+.2f}°, "
              f"{st.imu_pitch*180/math.pi:+.2f}°) overruns={overruns}/{n}")
        return not terminated

    try:
        ok = run_axis(0, a_roll, "roll") and run_axis(1, a_pitch, "pitch")
    except Exception as e:
        print(f"[balance_sine] ABORT: {e}")
        ok = False
        try:
            env._limp()
        except Exception:
            pass
    env.close()
    try:
        bus.close()
    except Exception:
        pass
    print("[balance_sine]", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
