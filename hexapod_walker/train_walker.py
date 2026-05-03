"""Train a residual gait policy with Stable Baselines 3 (PPO).

The agent's job is to output a small per-joint correction on top of the
analytic IK gait so the walker tracks an arbitrary commanded body twist
(vx, vy, omega) more accurately on rough terrain with obstacles.

Examples
--------

  # quick smoke train (~ 30 s on CPU)
  ./.venv/bin/python hexapod_walker/train_walker.py --steps 5000

  # ~ 10-30 min, multi-env vectorised
  ./.venv/bin/python hexapod_walker/train_walker.py \
      --steps 250000 --n-envs 8

  # then watch a trained policy
  ./.venv/bin/mjpython hexapod_walker/rollout_walker.py \
      --policy hexapod_walker/policies/walker_ppo.zip --vx 0.4

The script saves (policy, vec_normalize, training-curve CSV) under
hexapod_walker/policies/<run_tag>/.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_env as he  # noqa: E402

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import (  # noqa: E402
    CallbackList,
    CheckpointCallback,
)
from stable_baselines3.common.logger import configure  # noqa: E402
from stable_baselines3.common.vec_env import (  # noqa: E402
    DummyVecEnv,
    SubprocVecEnv,
    VecMonitor,
    VecNormalize,
)


def make_env(rank: int, *, base_seed: int, episode_seconds: float,
             obstacle_count: int, terrain_enabled: bool,
             residual_scale: float, gait_period: float,
             action_filter_tau: float, delta_w: float,
             progress_w: float, cmd_speed_bias: float,
             vx_max: float, vy_max: float, omega_max: float,
             dr_mass_pct: float, dr_friction_pct: float,
             dr_motor_latency_ms: float, dr_joint_bias_rad: float,
             dr_action_noise: float, dr_velocity_kick: float,
             terrain_level_max: float, terrain_level_min: float,
             curriculum_episodes: int):
    def _thunk():
        env = he.HexapodWalkerEnv(
            episode_seconds=episode_seconds,
            obstacle_count=obstacle_count,
            terrain_enabled=terrain_enabled,
            terrain_seed=base_seed + rank,
            obstacle_seed=(base_seed + rank) * 7919,
            randomize_command=True,
            terminate_on_fall=True,
            residual_scale=residual_scale,
            gait_period=gait_period,
            action_filter_tau=action_filter_tau,
            delta_w=delta_w,
            progress_w=progress_w,
            cmd_speed_bias=cmd_speed_bias,
            cmd_vx_range=(-vx_max * 0.6, vx_max),
            cmd_vy_range=(-vy_max, vy_max),
            cmd_omega_range=(-omega_max, omega_max),
            dr_mass_pct=dr_mass_pct,
            dr_friction_pct=dr_friction_pct,
            dr_motor_latency_ms=dr_motor_latency_ms,
            dr_joint_bias_rad=dr_joint_bias_rad,
            dr_action_noise=dr_action_noise,
            dr_velocity_kick=dr_velocity_kick,
            terrain_level_max=terrain_level_max,
            terrain_level_min=terrain_level_min,
            curriculum_episodes=curriculum_episodes,
        )
        return env
    return _thunk


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--steps", type=int, default=200_000,
                    help="Total environment steps to train")
    ap.add_argument("--n-envs", type=int, default=4,
                    help="Number of parallel environments")
    ap.add_argument("--episode-seconds", type=float, default=8.0)
    ap.add_argument("--obstacle-count", type=int, default=8)
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", type=str, default="walker_ppo")
    ap.add_argument("--out-dir", type=str,
                    default=os.path.join(THIS_DIR, "policies"))
    ap.add_argument("--device", type=str, default="auto")
    ap.add_argument("--learning-rate", type=float, default=3e-4)
    ap.add_argument("--n-steps", type=int, default=2048,
                    help="PPO rollout length per env")
    ap.add_argument("--batch-size", type=int, default=512)
    ap.add_argument("--n-epochs", type=int, default=10)
    ap.add_argument("--ent-coef", type=float, default=0.0,
                    help="PPO entropy coefficient (0 = greedy)")
    ap.add_argument("--log-std-init", type=float, default=-1.5,
                    help="Initial log-std of the Gaussian policy "
                         "(-1.5 -> std~0.22, so initial actions stay close "
                         "to the bare gait)")
    ap.add_argument("--residual-scale", type=float, default=0.05,
                    help="Per-joint residual cap (rad)")
    ap.add_argument("--gait-period", type=float, default=0.85,
                    help="Gait cycle period in seconds (faster = quicker walk)")
    ap.add_argument("--action-filter-tau", type=float, default=0.08,
                    help="Time constant of the per-step LPF on the residual")
    ap.add_argument("--delta-w", type=float, default=1.5,
                    help="Penalty weight on |action - prev_action|^2 (smoothness)")
    ap.add_argument("--progress-w", type=float, default=15.0,
                    help="Reward weight per metre travelled in commanded dir")
    ap.add_argument("--cmd-speed-bias", type=float, default=0.5,
                    help="0 = uniform commands; 1 = biased to high speeds")
    ap.add_argument("--vx-max", type=float, default=0.6)
    ap.add_argument("--vy-max", type=float, default=0.4)
    ap.add_argument("--omega-max", type=float, default=0.20)
    ap.add_argument("--net-arch", type=str, default="128,128",
                    help="Comma-separated MLP widths for the policy net")
    # Domain randomisation
    ap.add_argument("--dr-mass-pct",         type=float, default=0.0,
                    help="Chassis mass scaling magnitude (e.g. 0.25 = ±25%%)")
    ap.add_argument("--dr-friction-pct",     type=float, default=0.0,
                    help="Ground sliding-friction scaling magnitude (±)")
    ap.add_argument("--dr-motor-latency-ms", type=float, default=0.0,
                    help="Motor command delay in ms (uniform 0..value per ep)")
    ap.add_argument("--dr-joint-bias-rad",   type=float, default=0.0,
                    help="Per-joint position bias magnitude (rad)")
    ap.add_argument("--dr-action-noise",     type=float, default=0.0,
                    help="Std of gaussian noise added to the residual (units of action_space)")
    ap.add_argument("--dr-velocity-kick",    type=float, default=0.0,
                    help="Initial random body-velocity kick at reset (m/s)")
    # Terrain curriculum
    ap.add_argument("--terrain-level-max",  type=float, default=1.0)
    ap.add_argument("--terrain-level-min",  type=float, default=0.0)
    ap.add_argument("--curriculum-episodes", type=int,   default=0,
                    help="Episodes over which terrain level ramps min->max "
                         "(0 = no curriculum, sample [0, max] uniform)")
    ap.add_argument("--resume", type=str, default=None,
                    help="Path to a saved .zip to continue training")
    args = ap.parse_args()

    out_dir = os.path.join(args.out_dir, args.tag)
    os.makedirs(out_dir, exist_ok=True)
    print(f"Run dir: {out_dir}")

    env_fns = [make_env(rank=i, base_seed=args.seed,
                        episode_seconds=args.episode_seconds,
                        obstacle_count=args.obstacle_count,
                        terrain_enabled=not args.no_terrain,
                        residual_scale=args.residual_scale,
                        gait_period=args.gait_period,
                        action_filter_tau=args.action_filter_tau,
                        delta_w=args.delta_w,
                        progress_w=args.progress_w,
                        cmd_speed_bias=args.cmd_speed_bias,
                        vx_max=args.vx_max,
                        vy_max=args.vy_max,
                        omega_max=args.omega_max,
                        dr_mass_pct=args.dr_mass_pct,
                        dr_friction_pct=args.dr_friction_pct,
                        dr_motor_latency_ms=args.dr_motor_latency_ms,
                        dr_joint_bias_rad=args.dr_joint_bias_rad,
                        dr_action_noise=args.dr_action_noise,
                        dr_velocity_kick=args.dr_velocity_kick,
                        terrain_level_max=args.terrain_level_max,
                        terrain_level_min=args.terrain_level_min,
                        curriculum_episodes=args.curriculum_episodes)
               for i in range(args.n_envs)]
    if args.n_envs > 1:
        # SubprocVecEnv on macOS needs spawn; SB3 handles that.
        venv = SubprocVecEnv(env_fns)
    else:
        venv = DummyVecEnv(env_fns)
    venv = VecMonitor(venv)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True,
                        gamma=0.99, clip_obs=10.0)

    if args.resume and os.path.exists(args.resume):
        print(f"Resuming from {args.resume}")
        model = PPO.load(args.resume, env=venv, device=args.device)
    else:
        model = PPO(
            policy="MlpPolicy",
            env=venv,
            learning_rate=args.learning_rate,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=args.ent_coef,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=dict(
                net_arch=[int(x) for x in args.net_arch.split(",")],
                log_std_init=args.log_std_init,
            ),
            verbose=1,
            seed=args.seed,
            device=args.device,
        )

    log_formats = ["stdout", "csv"]
    try:
        import torch.utils.tensorboard  # noqa: F401
        log_formats.append("tensorboard")
    except ImportError:
        pass
    logger = configure(out_dir, log_formats)
    model.set_logger(logger)

    ckpt = CheckpointCallback(save_freq=max(1, args.steps // (args.n_envs * 4)),
                              save_path=out_dir, name_prefix="ckpt",
                              save_vecnormalize=True)
    callbacks = CallbackList([ckpt])

    print(f"Training {args.steps} steps across {args.n_envs} parallel envs ...")
    t0 = time.time()
    model.learn(total_timesteps=args.steps, callback=callbacks,
                progress_bar=False)
    dt = time.time() - t0
    print(f"Done in {dt:.1f} s ({args.steps/dt:.1f} env-steps/s)")

    save_path = os.path.join(out_dir, args.tag + ".zip")
    model.save(save_path)
    venv.save(os.path.join(out_dir, "vec_normalize.pkl"))

    import json
    env_cfg = {
        "residual_scale":      args.residual_scale,
        "gait_period":         args.gait_period,
        "action_filter_tau":   args.action_filter_tau,
        "delta_w":             args.delta_w,
        "progress_w":          args.progress_w,
        "cmd_speed_bias":      args.cmd_speed_bias,
        "vx_max":              args.vx_max,
        "vy_max":              args.vy_max,
        "omega_max":           args.omega_max,
        "net_arch":            args.net_arch,
        "log_std_init":        args.log_std_init,
        "ent_coef":            args.ent_coef,
        "total_steps":         args.steps,
        "dr_mass_pct":         args.dr_mass_pct,
        "dr_friction_pct":     args.dr_friction_pct,
        "dr_motor_latency_ms": args.dr_motor_latency_ms,
        "dr_joint_bias_rad":   args.dr_joint_bias_rad,
        "dr_action_noise":     args.dr_action_noise,
        "dr_velocity_kick":    args.dr_velocity_kick,
        "terrain_level_max":   args.terrain_level_max,
        "terrain_level_min":   args.terrain_level_min,
        "curriculum_episodes": args.curriculum_episodes,
    }
    with open(os.path.join(out_dir, "env_cfg.json"), "w") as f:
        json.dump(env_cfg, f, indent=2)
    print(f"Saved policy → {save_path}")
    print(f"Saved VecNormalize stats → {out_dir}/vec_normalize.pkl")
    print(f"Saved env config → {out_dir}/env_cfg.json")


if __name__ == "__main__":
    main()
