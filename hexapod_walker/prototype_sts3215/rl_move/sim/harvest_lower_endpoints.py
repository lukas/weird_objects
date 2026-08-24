"""Harvest the stance policy's OWN settled lower-endpoint poses.

Rationale (08-14, SESSION_BULK_GATE): at n=600 held-out joystick
sessions the hierarchical product baseline's ONLY det failures (10/10)
and weakest sto stratum (0.801, over_current-dominated) were POST-LOWER
rises — rising again out of the pose its own lower skill actually
settles into — while synthetic-start first rises were 300/300. Stance
training has never SEEN those states: rise episodes spawn synthetic
flat/partial/crouch poses. This script rolls the stance checkpoint
STOCHASTICALLY through lower-only episodes and snapshots the settled
endpoint joint angles into an npz bank that sim_env's
start_at=="rise_bank" branch samples (cfg goal.rise_start_bank +
goal.rise_start_bank_frac) — the exact mechanism class that fixed the
walk park habit (harvest_park_states, cycle 27).

Harvest with a seed FAR from every eval stream (gate evals use seeds
0/1; the bulk session banks use 900000../910000.. (retired) and future
cohorts pre-register fresh ones): default 5000.

Usage:
  uv run python -m rl_move.sim.harvest_lower_endpoints CKPT.zip \
      --episodes 300 --seed 5000 --out rl_move/sim/park_banks/NAME.npz \
      [--episode-seconds 15] [--cfg-set actions.max_height_mm=115 ...] \
      [--h-err-tol-mm 15]
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

from .servo_model import SimServoParams   # noqa: E402
from .joint_task import SimHexapodJointGoalEnv  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--episodes", type=int, default=300)
    ap.add_argument("--seed", type=int, default=5000)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--h-err-tol-mm", type=float, default=15.0,
                    help="keep endpoints within this of the commanded "
                         "lower target (settled, not mid-fight)")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    cfg_kw = {}
    if args.cfg_set:
        from rl_move.config import load_config
        from .train_ppo_sim import _parse_cfg_set
        cfg = load_config()
        for key, parsed in _parse_cfg_set(args.cfg_set).items():
            sect, name = key.split(".", 1)
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg

    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(cfg_kw.get("cfg")),
        randomize=args.dr_scale > 0, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed, **cfg_kw)
    # lower-only mix
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "lower" else 0.0)

    model = PPO.load(args.checkpoint, device="cpu")

    bank, bank_qpos, bank_qvel, bank_zstand, ep_stats = [], [], [], [], []
    kept = fell = unsettled = 0
    t0 = time.time()
    for ep in range(args.episodes):
        obs, _ = env.reset()
        done, term, info = False, False, {}
        while not done:
            a, _ = model.predict(obs, deterministic=False)
            obs, _r, term, trunc, info = env.step(a)
            done = term or trunc
        h_err = float(info.get("height_err_mm", 1e9))
        h_mm = float(info.get("height_mm", 1e9))
        ok = (not term) and abs(h_err) <= args.h_err_tol_mm
        if ok:
            bank.append(env.data.qpos[env._qadr].copy())
            # Full-state twin (08-14, postlower2 dig-in): the joints-only
            # bank re-planted by _place_at_plant + slip/limp settle proved
            # OFF-DISTRIBUTION — even the harvested policy's own parent
            # rises 0/12 from the reconstruction vs 0.801/0.967 from real
            # in-session post-lower states. Save the exact settled state
            # so goal.rise_start_bank_exact can restore it verbatim.
            bank_qpos.append(np.asarray(env.data.qpos, dtype=float).copy())
            bank_qvel.append(np.asarray(env.data.qvel, dtype=float).copy())
            # Standing anchor (08-14 postlower2 dig-in, the REAL bug):
            # the rise task's height band is BELLY-anchored (z0 +
            # 108-114mm) but bank spawns settle ~50mm ABOVE the belly —
            # anchoring the band at the spawn commanded an impossible
            # ~190-213mm chassis height and both postlower arms trained
            # on it (strain-at-max-current, c2 regression). The lower
            # episode STARTS standing, so its own _z0 is the exact
            # height this endpoint should rise back to:
            # goal.rise_start_bank_anchor_stand rewrites the schedule to
            # z_stand - z0_spawn.
            bank_zstand.append(float(env._z0))
            kept += 1
        elif term:
            fell += 1
        else:
            unsettled += 1
        ep_stats.append({"ep": ep, "kept": bool(ok), "terminated": bool(term),
                         "height_mm": round(h_mm, 1),
                         "height_err_mm": round(h_err, 1)})
        if ep % 25 == 0 or ok is False:
            print(f"[harvest] ep {ep}: kept={ok} term={term} "
                  f"h={h_mm:.1f}mm err={h_err:.1f}mm", flush=True)

    q_rad = np.asarray(bank, dtype=np.float64)
    md5 = hashlib.md5(args.checkpoint.read_bytes()).hexdigest()
    meta = {"kind": "lower_endpoints",
            "checkpoint": str(args.checkpoint), "checkpoint_md5": md5,
            "episodes": args.episodes, "seed": args.seed,
            "dr_scale": args.dr_scale, "cfg_set": args.cfg_set,
            "h_err_tol_mm": args.h_err_tol_mm,
            "kept": kept, "fell": fell, "unsettled": unsettled,
            "ep_stats": ep_stats}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, q_rad=q_rad,
             qpos_full=np.asarray(bank_qpos, dtype=np.float64),
             qvel_full=np.asarray(bank_qvel, dtype=np.float64),
             z_stand=np.asarray(bank_zstand, dtype=np.float64),
             meta=json.dumps(meta))
    print(f"[harvest] kept {kept}/{args.episodes} settled lower "
          f"endpoints (fell {fell}, unsettled {unsettled}) -> {args.out} "
          f"({time.time() - t0:.0f}s)")
    if len(q_rad):
        # Distribution sanity: deltas from the belly-flat zero pose the
        # synthetic flat rise start uses — how different is the real
        # post-lower state from what rise training has been seeing?
        from .sim_env import DEG2RAD
        d_deg = q_rad / DEG2RAD
        for j, name in ((0, "yaw"), (1, "hip"), (2, "knee")):
            v = d_deg[:, j::3].ravel()
            print(f"[harvest] {name} vs flat-zero deg: "
                  f"mean {v.mean():+.1f} p5 {np.percentile(v, 5):+.1f} "
                  f"p95 {np.percentile(v, 95):+.1f}", flush=True)


if __name__ == "__main__":
    main()
