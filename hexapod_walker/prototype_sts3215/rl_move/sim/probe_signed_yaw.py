"""Signed yaw-response + zero-command stillness probe (multitask track).

`eval_yaw.py` reports MEDIAN ABSOLUTE wz error per scenario, which
cannot distinguish "yaw response absent" from "sign-correct response
buried under a constant drift" — the fork that decides the
FAIL(acquisition) branch of the cw-mt-widen lineage (first needed for
cw-mt-widen1, 2026-08-12). This probe reports the SIGNED median wz and
the median planar speed per scenario, deterministic, DR 0:

  stop-hold   vx=0  wz=0      -> speed_med is the stillness verdict
  fwd-hold    vx=s  wz=0      -> baseline drift while walking
  tip-left    vx=0  wz=+w     |  tip_left.wz - tip_right.wz is the
  tip-right   vx=0  wz=-w     |  yaw differential (sign response)
  arc-left    vx=s  wz=+w/2   |
  arc-right   vx=s  wz=-w/2   |  same, while walking

    uv run python -m rl_move.sim.probe_signed_yaw <ckpt.zip> \
        --speed 0.05 --wz-max 0.15 [--cfg-set k=v ...] [--out out.json]

Read: acquisition of "stop" = stop-hold speed_med well under the walk
speed; acquisition of yaw sign = positive (tip-left - tip-right)
differential of meaningful size vs the command span.
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--wz-max", type=float, default=0.15)
    ap.add_argument("--seconds", type=float, default=7.0)
    ap.add_argument("--blend-skip-s", type=float, default=1.5)
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
        randomize=False, dr_scale=0.0, episode_seconds=600.0,
        seed=args.seed, **cfg_kw)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        setattr(gen, f"p_{m}", 0.0)
    model = PPO.load(args.checkpoint, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape} — wrong --cfg-set for this ckpt?")

    def run(vx: float, wz: float) -> dict:
        obs, _ = env.reset()
        wzs, spd, falls, t = [], [], 0, 0.0
        for _ in range(int(args.seconds / env.dt)):
            traj = env._goal_traj
            if traj is not None and hasattr(traj, "vx"):
                traj.vx[:] = vx
                traj.vy[:] = 0.0
                if getattr(traj, "wz", None) is not None:
                    traj.wz[:] = wz
                else:
                    traj.wz = np.full_like(np.asarray(traj.vx), wz)
            a, _ = model.predict(obs, deterministic=True)
            obs, _r, term, trunc, _info = env.step(a)
            t += env.dt
            if t >= args.blend_skip_s:
                wzs.append(float(env._body_wz()))
                spd.append(float(np.linalg.norm(env.data.qvel[:2])))
            if term or trunc:
                falls += 1
                obs, _ = env.reset()
        return {"wz_signed_med": round(float(np.median(wzs)), 4),
                "speed_med": round(float(np.median(spd)), 4),
                "falls": falls}

    s, w = args.speed, args.wz_max
    out = {}
    for name, vx, wz in [("stop-hold", 0.0, 0.0), ("fwd-hold", s, 0.0),
                         ("tip-left", 0.0, w), ("tip-right", 0.0, -w),
                         ("arc-left", s, w / 2), ("arc-right", s, -w / 2)]:
        out[name] = run(vx, wz)
        print(f"{name:10s} {out[name]}", flush=True)
    out["yaw_differential_tip"] = round(
        out["tip-left"]["wz_signed_med"] - out["tip-right"]["wz_signed_med"], 4)
    out["yaw_differential_arc"] = round(
        out["arc-left"]["wz_signed_med"] - out["arc-right"]["wz_signed_med"], 4)
    print("yaw_differential tip", out["yaw_differential_tip"],
          "arc", out["yaw_differential_arc"])
    if args.out:
        args.out.write_text(json.dumps(out, indent=1))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
