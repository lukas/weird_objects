#!/usr/bin/env python3
"""Hold action=0 and log while a human gently perturbs the chassis."""
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
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()
    cfg = load_config(args.config)
    bus, _ = open_bus(cfg)
    env = HexapodBalanceEnv(bus, cfg, log=True)
    obs, info = env.reset()
    del obs
    print(f"[perturb] ep={info.get('episode')} — nudge gently for {args.seconds}s")
    hz = float(cfg_get(cfg, "control", "hz", default=50))
    dt = 1.0 / hz
    zeros = np.zeros(5)
    peak_r = peak_p = 0.0
    t_end = time.monotonic() + args.seconds
    t_next = time.monotonic()
    while time.monotonic() < t_end:
        t_next += dt
        obs, rew, term, trunc, info = env.step(zeros)
        del obs, rew, trunc
        st = env._state
        peak_r = max(peak_r, abs(st.imu_roll))
        peak_p = max(peak_p, abs(st.imu_pitch))
        if term:
            print("terminated:", info.get("termination_reason"))
            break
        remain = t_next - time.monotonic()
        if remain > 0:
            time.sleep(remain)
        else:
            t_next = time.monotonic()
    print(f"[perturb] peak roll={peak_r*180/math.pi:.2f}° "
          f"pitch={peak_p*180/math.pi:.2f}°")
    env.close()
    try:
        bus.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
