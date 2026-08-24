"""Joystick-gate eval: scripted direction panel + randomized flip stress.

The generic harness samples commands from the env's own training
distribution — it never guarantees direction coverage (cw-walk-backforth
was gated without a single scripted reverse segment) and never tests
abrupt flips. This eval drives the policy the way an operator's joystick
will (operator binding target, 2026-08-09):

  named scenarios  fwd / back / left / right / diag-fl / diag-fr /
                   stop-go, each a scripted command schedule
  flip-stress      N episodes of random INSTANT command changes every
                   0.5-2.0 s inside a configurable envelope

Per scenario it reports falls (the gate), mean tracking error, and
distance covered. JOYSTICK GATE = zero falls across everything inside
the policy's trained envelope; out-of-envelope scenarios are reported
for information (expected to be rough until coverage lands).

    cd prototype_sts3215 && uv run python -m rl_move.sim.eval_drive \
        rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip \
        --speed 0.05 --heading-max-deg 45 [--cfg-set k=v ...]

Exit code 0 = gate passed (no in-envelope falls), 1 = failed.
"""
from __future__ import annotations

import os

# Cap math threads before numpy import (same reason as
# eval_checkpoint: unbounded per-process pools thrashed the
# controller's node, 08-09).
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
from pathlib import Path

import numpy as np


def scenarios(speed: float, fast: float) -> list[tuple[str, list]]:
    """(name, [(seconds, vx, vy), ...]) command schedules."""
    s = speed
    return [
        ("fwd",       [(1.0, 0, 0), (6.0,  s, 0.0)]),
        ("back",      [(1.0, 0, 0), (6.0, -s, 0.0)]),
        ("left",      [(1.0, 0, 0), (6.0, 0.0,  s)]),
        ("right",     [(1.0, 0, 0), (6.0, 0.0, -s)]),
        ("diag-fl",   [(1.0, 0, 0), (6.0, s * 0.707,  s * 0.707)]),
        ("diag-fr",   [(1.0, 0, 0), (6.0, s * 0.707, -s * 0.707)]),
        ("stop-go",   [(1.0, 0, 0), (2.0, s, 0), (2.0, 0, 0),
                       (2.0, s, 0), (2.0, 0, 0)]),
        ("fast-fwd",  [(1.0, 0, 0), (6.0, fast, 0.0)]),
        ("fwd-back-flip", [(1.0, 0, 0), (2.5, s, 0), (2.5, -s, 0),
                           (2.5, s, 0)]),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--speed", type=float, default=0.05,
                    help="panel command speed (m/s)")
    ap.add_argument("--fast", type=float, default=0.08,
                    help="fast-fwd scenario speed")
    ap.add_argument("--heading-max-deg", type=float, default=45.0,
                    help="policy's trained heading envelope; scenarios "
                         "outside it are informational, not gating")
    ap.add_argument("--speed-max", type=float, default=0.06,
                    help="policy's trained speed ceiling (envelope check)")
    ap.add_argument("--flip-episodes", type=int, default=3)
    ap.add_argument("--flip-seconds", type=float, default=15.0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V", help="env cfg overrides (own-cfg eval)")
    ap.add_argument("--strips", type=Path, default=None,
                    help="dir for ~1 fps frame-strip PNGs, one per named "
                         "scenario (video evidence for the direction "
                         "panel; flip-stress episodes are not stripped)")
    ap.add_argument("--rot60", action="store_true",
                    help="wrap the policy in the rot-60 canonicalizer "
                         "(rot60.Rot60Policy): commands are rotated into "
                         "the +/-30deg wedge and legs relabeled, so a "
                         "wedge-trained policy drives the full circle. "
                         "Walk obs frame (72) only.")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    # Same cfg-before-construction rule as eval_checkpoint (cycle 11:
    # overrides can change obs WIDTH, baked in __init__).
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
        render_mode="rgb_array" if args.strips else None,
        **cfg_kw)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        setattr(gen, f"p_{m}", 0.0)
    model = PPO.load(args.checkpoint, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape} — wrong --cfg-set for this ckpt?")
    if args.rot60:
        from rl_move.config import cfg_get
        from .rot60 import Rot60Policy
        ts = float(cfg_get(env.cfg, "obs", "tilt_scale", default=0.2))
        model = Rot60Policy(model, tilt_scale=ts)

    h_env = math.radians(args.heading_max_deg)

    def in_envelope(vx: float, vy: float) -> bool:
        sp = math.hypot(vx, vy)
        if sp < 0.005:
            return True
        return (abs(math.atan2(vy, vx)) <= h_env + 1e-6
                and sp <= args.speed_max + 0.005)

    def run_schedule(phases, strip_name: str | None = None) -> dict:
        obs, _ = env.reset()
        if hasattr(model, "reset"):
            model.reset()   # rot60 sector state is per-episode
        falls, err_sum, err_n, t = 0, 0.0, 0, 0.0
        p0 = np.array(env.data.qpos[:2], dtype=float)
        dist = 0.0
        strip: list = []
        next_frame_t = 0.0
        for seconds, vx, vy in phases:
            for _ in range(max(1, int(seconds / env.dt))):
                traj = env._goal_traj
                if traj is not None and hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                a, _ = model.predict(obs, deterministic=True)
                obs, _r, term, trunc, _info = env.step(a)
                v = env._body_vel_xy()
                err_sum += math.hypot(v[0] - vx, v[1] - vy)
                err_n += 1
                t += env.dt
                if strip_name is not None and t >= next_frame_t:
                    strip.append(env.render())
                    next_frame_t += 1.0
                if term or trunc:
                    falls += 1
                    p1 = np.array(env.data.qpos[:2], dtype=float)
                    dist += float(np.hypot(*(p1 - p0)))
                    obs, _ = env.reset()
                    if hasattr(model, "reset"):
                        model.reset()
                    p0 = np.array(env.data.qpos[:2], dtype=float)
        p1 = np.array(env.data.qpos[:2], dtype=float)
        dist += float(np.hypot(*(p1 - p0)))
        if strip_name is not None and strip:
            import imageio.v2 as imageio
            args.strips.mkdir(parents=True, exist_ok=True)
            imageio.imwrite(args.strips / f"{strip_name}.png",
                            np.hstack(strip))
        return {"falls": falls, "tracking_err": err_sum / max(err_n, 1),
                "seconds": round(t, 1), "dist_m": round(dist, 3)}

    rng = np.random.default_rng(args.seed)
    results, gate_falls = {}, 0
    for name, phases in scenarios(args.speed, args.fast):
        gating = all(in_envelope(vx, vy) for _sec, vx, vy in phases)
        r = run_schedule(
            phases, strip_name=name if args.strips else None)
        r["gating"] = gating
        results[name] = r
        if gating:
            gate_falls += r["falls"]
        tag = "GATE" if gating else "info"
        print(f"[{tag}] {name:14s} falls={r['falls']} "
              f"trk_err={r['tracking_err']:.3f} m/s")

    # flip stress: random instant flips inside the envelope
    for ep in range(args.flip_episodes):
        phases, t = [(1.0, 0.0, 0.0)], 1.0
        while t < args.flip_seconds:
            seg = float(rng.uniform(0.5, 2.0))
            if rng.random() < 0.2:
                vx = vy = 0.0
            else:
                sp = float(rng.uniform(0.02, args.speed_max))
                ang = float(rng.uniform(-h_env, h_env))
                vx, vy = sp * math.cos(ang), sp * math.sin(ang)
            phases.append((seg, vx, vy))
            t += seg
        r = run_schedule(phases)
        results[f"flip-stress-{ep}"] = {**r, "gating": True}
        gate_falls += r["falls"]
        print(f"[GATE] flip-stress-{ep}  falls={r['falls']} "
              f"trk_err={r['tracking_err']:.3f} m/s")

    verdict = "PASS" if gate_falls == 0 else "FAIL"
    print(f"JOYSTICK GATE: {verdict} ({gate_falls} in-envelope fall(s); "
          f"envelope heading<={args.heading_max_deg:.0f}deg "
          f"speed<={args.speed_max})")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(
            {"gate": verdict, "gate_falls": gate_falls,
             "envelope": {"heading_max_deg": args.heading_max_deg,
                          "speed_max": args.speed_max},
             "scenarios": results}, indent=1))
        print(f"wrote {args.out}")
    return 0 if gate_falls == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
