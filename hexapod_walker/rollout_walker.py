"""Roll out a trained PPO policy in the MuJoCo viewer (or headless).

Loads ``policies/<tag>/<tag>.zip`` (and the matching VecNormalize
statistics) and drives the residual gait controller live, so you can
watch the trained walker handle randomized terrain and obstacles.

Usage
-----

    # interactive viewer
    ./.venv/bin/mjpython hexapod_walker/rollout_walker.py \\
        --policy hexapod_walker/policies/walker_ppo/walker_ppo.zip \\
        --vx 0.4

    # headless (prints scoreboard)
    ./.venv/bin/python hexapod_walker/rollout_walker.py \\
        --policy hexapod_walker/policies/walker_ppo/walker_ppo.zip \\
        --headless --episodes 5

The policy receives observations from the *real* environment and outputs
joint residuals; the keyboard / CLI ``--vx --vy --omega`` flags set the
commanded twist that the policy is asked to track.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import mujoco

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_env as he  # noqa: E402
import mujoco_walker as MW  # noqa: E402

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.vec_env import (  # noqa: E402
    DummyVecEnv,
    VecNormalize,
)


def _load_env_cfg(policy_path):
    import json
    cfg_path = os.path.join(os.path.dirname(policy_path), "env_cfg.json")
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            return json.load(f)
    return None


def _make_eval_env(args, env_cfg=None):
    extra = {}
    if env_cfg:
        scalar_keys = (
            "residual_scale", "gait_period", "action_filter_tau",
            "gait_action", "gait_action_filter_tau",
        )
        for key in scalar_keys:
            if key in env_cfg:
                extra[key] = env_cfg[key]
        for key in ("period_scale_range", "lift_scale_range",
                    "stride_scale_range"):
            if key in env_cfg:
                extra[key] = tuple(env_cfg[key])

    def _thunk():
        env = he.HexapodWalkerEnv(
            episode_seconds=args.duration,
            obstacle_count=args.obstacles,
            terrain_enabled=not args.no_terrain,
            terrain_seed=args.terrain_seed,
            obstacle_seed=args.obstacle_seed,
            randomize_command=False,
            terminate_on_fall=False,
            **extra,
        )
        return env
    return _thunk


def _load_policy(args, venv):
    model = PPO.load(args.policy, env=venv, device=args.device)
    vn_path = os.path.join(os.path.dirname(args.policy), "vec_normalize.pkl")
    if os.path.exists(vn_path):
        venv = VecNormalize.load(vn_path, venv)
        venv.training = False
        venv.norm_reward = False
        print(f"Loaded VecNormalize stats from {vn_path}")
        model.set_env(venv)
    return model, venv


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", required=True,
                    help="Path to a saved PPO .zip")
    ap.add_argument("--vx", type=float, default=0.3)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--omega", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--obstacles", type=int, default=12)
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--terrain-seed", type=int, default=42)
    ap.add_argument("--obstacle-seed", type=int, default=7)
    ap.add_argument("--device", type=str, default="auto")
    args = ap.parse_args()

    env_cfg = _load_env_cfg(args.policy)
    if env_cfg:
        print(f"Loaded env config from env_cfg.json: "
              f"gait_period={env_cfg.get('gait_period', '?')}, "
              f"residual_scale={env_cfg.get('residual_scale', '?')}, "
              f"filter_tau={env_cfg.get('action_filter_tau', '?')}")
    venv = DummyVecEnv([_make_eval_env(args, env_cfg=env_cfg)])
    model, venv = _load_policy(args, venv)

    print(f"Loaded policy {args.policy}")

    # Reach into the underlying env so we can set the command.
    inner = venv.envs[0]
    while hasattr(inner, "env"):
        inner = inner.env
    if isinstance(inner, he.HexapodWalkerEnv):
        target_env = inner
    else:
        raise RuntimeError("Could not unwrap HexapodWalkerEnv")

    if args.headless:
        for ep in range(args.episodes):
            obs = venv.reset()
            target_env.set_command(args.vx, args.vy, args.omega)
            done = np.array([False])
            total_r = 0.0
            steps = 0
            last_xy = target_env.data.body(target_env.idx.chassis_body).xpos[:2].copy()
            p0 = last_xy.copy()
            while not done[0]:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, info = venv.step(action)
                total_r += float(reward[0])
                steps += 1
                # SubprocVecEnv / DummyVecEnv auto-reset the env when
                # ``done`` fires, so the post-step xpos goes back to the
                # spawn pad.  We fetch the *pre-reset* xpos from info.
                if not done[0]:
                    last_xy = info[0]["chassis_xy"]
                else:
                    last_xy = info[0].get("chassis_xy", last_xy)
            disp = np.hypot(last_xy[0] - p0[0], last_xy[1] - p0[1])
            print(f"  ep {ep}: steps={steps}  return={total_r:+.2f}  "
                  f"travelled={disp:.2f} m  end_xy=({last_xy[0]:+.2f},"
                  f"{last_xy[1]:+.2f})")
        venv.close()
        return

    # ---- Interactive viewer rollout ---------------------------------
    import mujoco.viewer as viewer
    obs = venv.reset()
    target_env.set_command(args.vx, args.vy, args.omega)

    DV = 0.10
    DOM = 0.05

    def status():
        c = target_env._cmd
        print(f"  command:  vx={c[0]:+.2f}  vy={c[1]:+.2f}  ω={c[2]:+.2f}",
              flush=True)

    _GK_RIGHT = 262;  _GK_LEFT = 263
    _GK_DOWN = 264;   _GK_UP = 265
    _GK_PG_UP = 266;  _GK_PG_DN = 267
    _GK_HOME = 268;   _GK_END = 269
    _GK_BACKTICK = 96

    def cb(keycode):
        c = target_env._cmd
        kc = int(keycode)
        if kc == _GK_UP:
            target_env.set_command(vx=c[0] + DV)
        elif kc == _GK_DOWN:
            target_env.set_command(vx=c[0] - DV)
        elif kc == _GK_LEFT:
            target_env.set_command(vy=c[1] + DV)
        elif kc == _GK_RIGHT:
            target_env.set_command(vy=c[1] - DV)
        elif kc == _GK_PG_UP:
            target_env.set_command(omega=c[2] + DOM)
        elif kc == _GK_PG_DN:
            target_env.set_command(omega=c[2] - DOM)
        elif kc == _GK_HOME:
            target_env.set_command(vx=0.0, vy=0.0, omega=0.0)
        elif kc == _GK_END:
            target_env.reset(options={"command": (args.vx, args.vy, args.omega)})
        elif kc == _GK_BACKTICK:
            print("Arrow keys: vx/vy.  PgUp/Dn: omega.  Home: stop.  End: reset.")
        else:
            return
        status()

    print("Trained-policy rollout.  Arrow keys / PgUp/PgDn drive the command.")
    status()
    with viewer.launch_passive(target_env.model, target_env.data,
                               key_callback=cb) as v:
        last_status = -1.0
        while v.is_running():
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = venv.step(action)
            if done[0]:
                obs = venv.reset()
                target_env.set_command(args.vx, args.vy, args.omega)
                print("(episode reset)")
            v.sync()
            t = float(target_env.data.time)
            if t - last_status > 1.0:
                p = target_env.data.body(target_env.idx.chassis_body).xpos
                print(f"  t={t:6.2f}s  pos=({p[0]:+6.2f},{p[1]:+6.2f},"
                      f"{p[2]:+5.2f})  reward={float(reward[0]):+.3f}",
                      flush=True)
                last_status = t
            time.sleep(max(0.0, target_env.model.opt.timestep - 1e-4))


if __name__ == "__main__":
    main()
