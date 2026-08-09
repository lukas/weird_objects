"""Harvest the policy's OWN park states as reset poses (cycle 27).

Rationale: synthetic tripod-park starts (goal.walk_park_start_frac,
cycle 24) taught exits from SYNTHETIC parks (park-exit 11/12) while the
policy's own park — entered from its own dynamics under stochastic
actions — survived full update parity (cw-walk-parkstart-mjx-c1,
cycle 27). This script rolls the champion out STOCHASTICALLY from
normal starts, detects parked ticks (commanded speed >= s_ref_min but
trailing-window mean achieved speed below a stall threshold), and
snapshots the 18 joint angles into an npz bank that sim_env's
start_at=="park" branch can sample (cfg goal.walk_park_bank +
goal.walk_park_bank_frac).

Harvest with a seed FAR from the eval harness stream (default 1000):
training on the literal gate-eval episodes would contaminate the gate.

Usage:
  python -m rl_move.sim.harvest_park_states CKPT.zip \
      --episodes 60 --seed 1000 --out rl_move/sim/park_banks/NAME.npz \
      [--episode-seconds 15] [--cfg-set goal.walk_speed_max_m_s=0.06 ...]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (_PROTO, _PROTO / "linux_control"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .servo_model import SimServoParams        # noqa: E402
from .walk_task import SimHexapodJointWalkEnv  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--stall-speed", type=float, default=0.015,
                    help="trailing-window mean speed below this = parked")
    ap.add_argument("--window-s", type=float, default=2.0)
    ap.add_argument("--min-t-s", type=float, default=4.0,
                    help="ignore the command hold+ramp at episode start")
    ap.add_argument("--snapshot-every-s", type=float, default=0.5)
    args = ap.parse_args()

    from stable_baselines3 import PPO

    cfg_kw = {}
    if args.cfg_set:
        from rl_move.config import load_config
        cfg = load_config()
        for spec in args.cfg_set:
            key, val = spec.split("=", 1)
            sect, name = key.split(".", 1)
            try:
                parsed: float | str = float(val)
            except ValueError:
                parsed = val.strip()
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.load(),
        randomize=args.dr_scale > 0, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed, **cfg_kw)
    # walk-only mix
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)

    model = PPO.load(args.checkpoint, device="cpu")

    win_n = max(1, int(round(args.window_s / env.dt)))
    snap_n = max(1, int(round(args.snapshot_every_s / env.dt)))
    min_i = int(round(args.min_t_s / env.dt))

    bank, ep_stats = [], []
    t0 = time.time()
    for ep in range(args.episodes):
        obs, _ = env.reset()
        speeds, snaps, parked_ticks = [], 0, 0
        done, i, since_snap = False, 0, snap_n
        while not done:
            a, _ = model.predict(obs, deterministic=False)
            obs, _r, term, trunc, info = env.step(a)
            done = term or trunc
            i += 1
            speeds.append(float(info.get("walk_speed", 0.0)))
            goal = env._current_goal()
            s_ref = float(np.hypot(goal.vx_ref, goal.vy_ref)) if goal else 0.0
            since_snap += 1
            if (i >= min_i and s_ref >= 0.02 and len(speeds) >= win_n
                    and float(np.mean(speeds[-win_n:])) < args.stall_speed):
                parked_ticks += 1
                if since_snap >= snap_n:
                    bank.append(env.data.qpos[env._qadr].copy())
                    snaps += 1
                    since_snap = 0
        g = env._current_goal()
        cmd = ((round(float(g.vx_ref), 4), round(float(g.vy_ref), 4))
               if g else (0.0, 0.0))
        ep_stats.append({"ep": ep, "parked_ticks": parked_ticks,
                         "snapshots": snaps, "cmd_vx_vy": cmd,
                         "mean_speed": round(float(np.mean(speeds)), 4)})
        print(f"[harvest] ep {ep}: parked_ticks={parked_ticks} "
              f"snaps={snaps} mean_speed={np.mean(speeds):.4f} "
              f"cmd={cmd}", flush=True)

    q_rad = np.asarray(bank, dtype=np.float64)
    parked_eps = sum(1 for e in ep_stats if e["snapshots"] > 0)
    md5 = hashlib.md5(args.checkpoint.read_bytes()).hexdigest()
    meta = {"checkpoint": str(args.checkpoint), "checkpoint_md5": md5,
            "episodes": args.episodes, "seed": args.seed,
            "dr_scale": args.dr_scale,
            "stall_speed": args.stall_speed, "window_s": args.window_s,
            "min_t_s": args.min_t_s, "cfg_set": args.cfg_set,
            "parked_episodes": parked_eps, "n_states": len(q_rad),
            "ep_stats": ep_stats}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, q_rad=q_rad, meta=json.dumps(meta))
    print(f"[harvest] {len(q_rad)} states from {parked_eps}/"
          f"{args.episodes} parked episodes -> {args.out} "
          f"({time.time() - t0:.0f}s)")
    if len(q_rad):
        # How far outside the synthetic distribution are we? Synthetic
        # lifts one tripod's hips 10-25 deg off plant with knee jitter
        # -5..10 deg. Report per-joint deltas from plant.
        from .sim_env import DEG2RAD
        plant = env._plant_deg * DEG2RAD
        d_deg = (q_rad - plant[None, :]) / DEG2RAD
        for j, name in ((0, "yaw"), (1, "hip"), (2, "knee")):
            v = d_deg[:, j::3].ravel()
            print(f"[harvest] {name} delta-from-plant deg: "
                  f"mean {v.mean():+.1f} p5 {np.percentile(v, 5):+.1f} "
                  f"p95 {np.percentile(v, 95):+.1f}")


if __name__ == "__main__":
    main()
