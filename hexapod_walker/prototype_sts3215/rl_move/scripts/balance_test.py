#!/usr/bin/env python3
"""Zero-action stationary balance infrastructure test (Milestone 1)."""
from __future__ import annotations

import argparse
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
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--no-log", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    motion = bool(cfg_get(cfg, "bus", "enable_motion", default=False))
    print("[balance_test] opening bus (stop hexapod-web first if needed)")
    print(f"[balance_test] enable_motion={motion}  "
          f"hold_current={cfg_get(cfg, 'episode', 'hold_current_pose')}")
    bus, port = open_bus(cfg)
    print(f"[balance_test] bus={port}")
    env = HexapodBalanceEnv(bus, cfg, log=not args.no_log)
    hz = float(cfg_get(cfg, "control", "hz", default=50))
    dt = 1.0 / hz
    zeros = np.zeros(5, dtype=float)

    ok = True
    try:
        for ep in range(args.episodes):
            obs, info = env.reset()
            print(f"[balance_test] episode {info.get('episode')}  "
                  f"obs_dim={obs.shape[0]}  "
                  f"roll={info.get('roll_deg', float('nan')):+.1f}°  "
                  f"pitch={info.get('pitch_deg', float('nan')):+.1f}°  "
                  f"hold_current={info.get('hold_current')}")
            terminated = truncated = False
            step = 0
            overruns = 0
            rew_sum = 0.0
            t_next = time.monotonic()
            while not terminated and not truncated and step < env.episode_steps:
                t0 = time.monotonic()
                obs, rew, terminated, truncated, info = env.step(zeros)
                rew_sum += rew
                step += 1
                # Pace only the remainder — env.step already did the work.
                remain = dt - (time.monotonic() - t0)
                if remain > 0:
                    time.sleep(remain)
                else:
                    overruns += 1
            reason = info.get("termination_reason") or ""
            st = env._state
            print(f"  steps={step} reward_sum={rew_sum:.3f} overruns={overruns} "
                  f"term={terminated} trunc={truncated} reason={reason!r}  "
                  f"end_tilt=({st.imu_roll*180/3.14159:+.1f}°, "
                  f"{st.imu_pitch*180/3.14159:+.1f}°)")
            if terminated and reason not in ("",):
                ok = False
    except Exception as e:
        print(f"[balance_test] ABORT: {e}")
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
    print("[balance_test]", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
