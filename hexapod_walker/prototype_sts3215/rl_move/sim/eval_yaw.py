"""Yaw-command tracking eval (walk_yaw_cmd lineage gate).

The generic harness report has no wz fields, so the yawcmd gate
("commanded-turn |wz_err| med <= 0.10 rad/s AND wz_ref=0 segments
|wz| med <= 0.05 rad/s") was unmeasurable — flagged DIG-IN on
cw-walk-yawcmd1-rr1/-s1 (2026-08-10). This drives the policy through a
scripted yaw panel the way eval_drive drives the linear panel:

  fwd-hold / stop-hold            wz_ref = 0 (heading hold)
  arc-left/right (0.15, max)      forward + commanded turn
  tip-left/right (max)            turn in place, vx = vy = 0
  yaw-flip-stress                 random instant (vx, wz) changes

Per scenario: falls, median |wz - wz_ref| (blend-skipped). Aggregate
gate = turn-segment |wz_err| med, zero-segment |wz| med, zero falls.

    uv run python -m rl_move.sim.eval_yaw <ckpt.zip> \
        --speed 0.05 --wz-max 0.3 [--cfg-set k=v ...] [--out out.json]

Exit 0 = gate passed, 1 = failed.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
from pathlib import Path

import numpy as np


def scenarios(s: float, w: float) -> list[tuple[str, list]]:
    """(name, [(seconds, vx, vy, wz), ...])."""
    half = w / 2.0
    return [
        ("fwd-hold",      [(1.0, 0, 0, 0), (6.0,  s, 0.0,  0.0)]),
        ("stop-hold",     [(1.0, 0, 0, 0), (6.0, 0.0, 0.0,  0.0)]),
        ("arc-left",      [(1.0, 0, 0, 0), (6.0,  s, 0.0,  half)]),
        ("arc-right",     [(1.0, 0, 0, 0), (6.0,  s, 0.0, -half)]),
        ("arc-left-max",  [(1.0, 0, 0, 0), (6.0,  s, 0.0,  w)]),
        ("arc-right-max", [(1.0, 0, 0, 0), (6.0,  s, 0.0, -w)]),
        ("tip-left",      [(1.0, 0, 0, 0), (6.0, 0.0, 0.0,  w)]),
        ("tip-right",     [(1.0, 0, 0, 0), (6.0, 0.0, 0.0, -w)]),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--wz-max", type=float, default=0.3,
                    help="trained yaw envelope (goal.walk_yaw_max_rad_s)")
    ap.add_argument("--blend-skip-s", type=float, default=1.5,
                    help="seconds ignored after each command change "
                         "(training blends commands up to 1 s)")
    ap.add_argument("--turn-gate", type=float, default=0.10)
    ap.add_argument("--hold-gate", type=float, default=0.05)
    ap.add_argument("--flip-episodes", type=int, default=2)
    ap.add_argument("--flip-seconds", type=float, default=15.0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

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
        params=SimServoParams.from_cfg(cfg_kw.get("cfg")),
        randomize=args.dr_scale > 0,
        dr_scale=args.dr_scale, episode_seconds=600.0, seed=args.seed,
        **cfg_kw)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        setattr(gen, f"p_{m}", 0.0)
    model = PPO.load(args.checkpoint, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape} — wrong --cfg-set for this ckpt?")

    turn_abs_err: list[float] = []   # |wz - wz_ref|, wz_ref != 0
    hold_abs_wz: list[float] = []    # |wz|, wz_ref == 0

    def run_schedule(phases) -> dict:
        obs, _ = env.reset()
        falls, t = 0, 0.0
        seg_turn, seg_hold = [], []
        for seconds, vx, vy, wz in phases:
            t_seg = 0.0
            for _ in range(max(1, int(seconds / env.dt))):
                traj = env._goal_traj
                if traj is not None and hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                    if getattr(traj, "wz", None) is not None:
                        traj.wz[:] = wz
                    else:
                        traj.wz = np.full_like(np.asarray(traj.vx), wz)
                a, _ = model.predict(obs, deterministic=True)
                obs, _r, term, trunc, _info = env.step(a)
                t_seg += env.dt
                t += env.dt
                if t_seg >= args.blend_skip_s:
                    wz_meas = env._body_wz()
                    if abs(wz) > 1e-6:
                        seg_turn.append(abs(wz_meas - wz))
                    else:
                        seg_hold.append(abs(wz_meas))
                if term or trunc:
                    falls += 1
                    obs, _ = env.reset()
        turn_abs_err.extend(seg_turn)
        hold_abs_wz.extend(seg_hold)
        med = (float(np.median(seg_turn)) if seg_turn
               else float(np.median(seg_hold)) if seg_hold else float("nan"))
        return {"falls": falls, "wz_med": round(med, 4),
                "seconds": round(t, 1)}

    results, falls = {}, 0
    for name, phases in scenarios(args.speed, args.wz_max):
        r = run_schedule(phases)
        results[name] = r
        falls += r["falls"]
        kind = ("|wz_err|" if "arc" in name or "tip" in name else "|wz|")
        print(f"{name:14s} falls={r['falls']} {kind} med={r['wz_med']}")

    rng = np.random.default_rng(args.seed)
    for ep in range(args.flip_episodes):
        phases, t = [(1.0, 0.0, 0.0, 0.0)], 1.0
        while t < args.flip_seconds:
            seg = float(rng.uniform(0.5, 2.0))
            vx = 0.0 if rng.random() < 0.3 else args.speed
            wz = (0.0 if rng.random() < 0.4
                  else float(rng.uniform(-args.wz_max, args.wz_max)))
            phases.append((seg, vx, 0.0, wz))
            t += seg
        r = run_schedule(phases)
        results[f"yaw-flip-{ep}"] = r
        falls += r["falls"]
        print(f"yaw-flip-{ep}    falls={r['falls']}")

    turn_med = float(np.median(turn_abs_err)) if turn_abs_err else float("nan")
    hold_med = float(np.median(hold_abs_wz)) if hold_abs_wz else float("nan")
    ok = (turn_med <= args.turn_gate and hold_med <= args.hold_gate
          and falls == 0)
    print(f"turn |wz_err| med = {turn_med:.4f} rad/s (gate <= "
          f"{args.turn_gate})")
    print(f"hold |wz|     med = {hold_med:.4f} rad/s (gate <= "
          f"{args.hold_gate})")
    print(f"falls = {falls}")
    print(f"YAW GATE: {'PASS' if ok else 'FAIL'}")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(
            {"gate": "PASS" if ok else "FAIL",
             "turn_wz_err_med": turn_med, "hold_wz_med": hold_med,
             "falls": falls, "gates": {"turn": args.turn_gate,
                                       "hold": args.hold_gate},
             "envelope": {"speed": args.speed, "wz_max": args.wz_max},
             "scenarios": results}, indent=1))
        print(f"wrote {args.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
