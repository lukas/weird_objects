"""Roll out a trained STS walking policy in MuJoCo (with stall guard).

Examples
--------

    ./.venv/bin/python hexapod_walker/sts/rollout.py \\
        --policy hexapod_walker/sts/policies/sts_ppo/sts_ppo.zip \\
        --headless --episodes 3

    ./.venv/bin/mjpython hexapod_walker/sts/rollout.py \\
        --policy hexapod_walker/sts/policies/sts_ppo/sts_ppo.zip --vx 0.10
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.join(os.path.dirname(THIS_DIR), "prototype_sts3215")
FULLSIZE_DIR = os.path.join(os.path.dirname(THIS_DIR), "fullsize_v1")
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, PROTO_DIR)
sys.path.insert(0, FULLSIZE_DIR)

import mujoco_prototype as MP  # noqa: E402
import sts_env as SE  # noqa: E402

MP.TORQUE_LIMIT = 2.70
sys.modules["mujoco_walker"] = MP
sys.modules["hexapod_env"] = SE


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", required=True, help="Path to SB3 .zip")
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--omega", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--obstacles", type=int, default=8)
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--jam", action="store_true",
                    help="Force a mid-episode joint jam to demo stall stop")
    ap.add_argument("--mode", choices=("walk", "posture"), default="walk")
    ap.add_argument("--posture", choices=("sit", "stand"), default="stand",
                    help="With --mode posture: stand = stand up from sprawl, "
                         "sit = sit down from stance")
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    policy_dir = os.path.dirname(os.path.abspath(args.policy))
    cfg_path = os.path.join(policy_dir, "env_cfg.json")
    cfg = {}
    if os.path.isfile(cfg_path):
        with open(cfg_path) as f:
            cfg = json.load(f)

    def _make():
        return SE.StsWalkerEnv(
            episode_seconds=args.duration,
            obstacle_count=args.obstacles if args.mode == "walk" else 0,
            terrain_enabled=(not args.no_terrain) and args.mode == "walk",
            randomize_command=False,
            residual_scale=cfg.get("residual_scale", 0.055),
            gait_period=cfg.get("gait_period", 0.82),
            action_filter_tau=cfg.get("action_filter_tau", 0.14),
            gait_action=cfg.get("gait_action", True),
            per_leg_lift=cfg.get("per_leg_lift", True),
            stub_w=cfg.get("stub_w", 0.5),
            period_scale_range=tuple(cfg.get("period_scale_range", (0.88, 1.38))),
            lift_scale_range=tuple(cfg.get("lift_scale_range", (0.65, 1.75))),
            stride_scale_range=tuple(cfg.get("stride_scale_range", (0.52, 1.18))),
            gait_action_filter_tau=cfg.get("gait_action_filter_tau", 0.38),
            jam_event_prob=1.0 if args.jam else 0.0,
            posture_task_prob=1.0 if args.mode == "posture" else 0.0,
            posture_flip_prob=0.0,
            render_mode=None if args.headless else "human",
        )

    venv = DummyVecEnv([_make])
    norm_path = os.path.join(policy_dir, "vec_normalize.pkl")
    if os.path.isfile(norm_path):
        venv = VecNormalize.load(norm_path, venv)
        venv.training = False
        venv.norm_reward = False

    model = PPO.load(args.policy, env=venv, device="cpu")

    for ep in range(args.episodes):
        # Unwrap VecNormalize → DummyVecEnv → env
        raw = venv
        while hasattr(raw, "venv"):
            raw = raw.venv
        env = raw.envs[0]
        # Reset underlying env with mode options, then sync vec obs.
        raw_obs, _ = env.reset(
            seed=args.seed + ep,
            options={
                "command": (args.vx, args.vy, args.omega),
                "mode": args.mode,
                "posture": args.posture,
            },
        )
        if args.mode == "walk":
            env.set_command(vx=args.vx, vy=args.vy, omega=args.omega)
        if args.jam:
            env._jam_joint = 5
            env._jam_steps_left = -int(1.5 * env.control_hz)
        # Push normalized obs into the VecEnv wrapper stack.
        obs = np.expand_dims(raw_obs, 0)
        if hasattr(venv, "normalize_obs"):
            obs = venv.normalize_obs(obs)

        steps = int(args.duration * env.control_hz)
        ret = 0.0
        t0 = time.time()
        info = {"chassis_xy": (0.0, 0.0)}
        for k in range(steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, rewards, dones, infos = venv.step(action)
            ret += float(rewards[0])
            info = infos[0]
            if not args.headless:
                env.render()
            if info.get("stall"):
                print(f"  [ep {ep}] STALL at step {k} "
                      f"joints={list(np.flatnonzero(info.get('stall_joints', [])))} "
                      f"relax={info.get('relax')}")
            if dones[0]:
                break
        xy = info.get("chassis_xy", (0, 0))
        print(f"episode {ep}: mode={info.get('mode', args.mode)}  "
              f"return={ret:.2f}  xy=({xy[0]:.3f},{xy[1]:.3f})  "
              f"z={info.get('chassis_z', float('nan')):.3f}  "
              f"bus≈{info.get('bus_current_a', float('nan')):.1f}A  "
              f"peak≈{info.get('peak_current_a', float('nan')):.1f}A  "
              f"stall={info.get('stall', False)}  "
              f"wall={time.time() - t0:.1f}s")
        if not args.headless and hasattr(env, "_viewer") and env._viewer is not None:
            time.sleep(0.3)

    venv.close()


if __name__ == "__main__":
    main()
