"""Probe: leg-odometry estimator vs privileged body velocity, in sim.

THE decisive pre-launch check for the estimator rung (operator
2026-08-11). Rolls a real walk champion at 25 Hz and, every tick, runs
``rl_move.estimator.LegOdometryVelocity`` on exactly the signals the
robot would have (the env's DR-corrupted observed state: encoder
positions, gyro, accel tilt) while recording the privileged simulator
body velocity the mode-1 champions train on. If the estimate tracks
the truth here — through a real learned gait's stance breaks, slips
and wobble, not an idealized tripod — then walk_obs_body_vel=3 gives
a deployable policy the same signal the privileged champions exploit.

Run from prototype_sts3215:
    uv run python -m rl_move.sim.probe_estimator \
        --policy rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

CMD_V = 0.05


def make_env(seed: int, episode_seconds: float, dr_scale: float):
    from rl_move.config import load_config
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    # Privileged velocity obs (mode 1) — the champion's own contract;
    # the estimator runs alongside, off the observed state only.
    cfg.setdefault("goal", {})["walk_obs_body_vel"] = 1.0
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=dr_scale > 0.0,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def pin_cmd(env, vx: float, vy: float) -> None:
    traj = env._goal_traj
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = vx
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = vx * ramp
    traj.vy[:] = vy
    traj.vy[:hold_n] = 0.0
    traj.vy[hold_n:hold_n + ramp_n] = vy * ramp
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0


def rollout(model, seed: int, episode_seconds: float, dr_scale: float,
            vx: float, vy: float) -> dict:
    from rl_move.estimator import LegOdometryVelocity

    env = make_env(seed, episode_seconds, dr_scale)
    obs, _ = env.reset()
    pin_cmd(env, vx, vy)
    est = LegOdometryVelocity(dt=env.dt)

    t_settle = 2.0
    v_true_l, v_est_l = [], []
    fk_us = []
    step = 0
    while True:
        st = env._state          # DR-corrupted observed state (robot's view)
        t0 = time.perf_counter()
        v_est = est.update(st.joint_position, st.imu_gyro,
                           st.imu_roll, st.imu_pitch)
        fk_us.append((time.perf_counter() - t0) * 1e6)
        if step * env.dt >= t_settle:
            v_true_l.append(env._body_vel_xy().copy())
            v_est_l.append(v_est)
        action, _ = model.predict(obs, deterministic=True)
        obs, _r, term, trunc, _info = env.step(action)
        step += 1
        if term or trunc:
            break
    v_true = np.array(v_true_l)
    v_est = np.array(v_est_l)
    err = v_est - v_true
    # Correlation on the x axis (the driven one for forward commands).
    def corr(a, b):
        if np.std(a) < 1e-9 or np.std(b) < 1e-9:
            return float("nan")
        return float(np.corrcoef(a, b)[0, 1])
    return {
        "seed": seed, "dr": dr_scale, "cmd": [vx, vy],
        "ticks": len(v_true), "terminated": bool(term),
        "rmse_x_mps": float(np.sqrt(np.mean(err[:, 0] ** 2))),
        "rmse_y_mps": float(np.sqrt(np.mean(err[:, 1] ** 2))),
        "bias_x_mps": float(np.mean(err[:, 0])),
        "bias_y_mps": float(np.mean(err[:, 1])),
        "corr_x": corr(v_est[:, 0], v_true[:, 0]),
        "corr_y": corr(v_est[:, 1], v_true[:, 1]),
        "true_mean_x": float(np.mean(v_true[:, 0])),
        "est_mean_x": float(np.mean(v_est[:, 0])),
        "true_mean_y": float(np.mean(v_true[:, 1])),
        "est_mean_y": float(np.mean(v_est[:, 1])),
        "fk_us_per_tick": float(np.mean(fk_us)),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=str(
        ROOT / "rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip"))
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    from stable_baselines3 import PPO
    model = PPO.load(args.policy, device="cpu")

    rows = []
    for dr in (0.0, 0.5):
        for seed, (vx, vy) in zip((11, 12, 13),
                                  ((CMD_V, 0.0), (CMD_V, 0.0),
                                   (0.035, 0.035))):
            r = rollout(model, seed, args.seconds, dr, vx, vy)
            rows.append(r)
            print(f"dr={dr} seed={seed} cmd=({vx},{vy}) ticks={r['ticks']}"
                  f" | x: true {r['true_mean_x']:+.3f} est"
                  f" {r['est_mean_x']:+.3f} rmse {r['rmse_x_mps']:.3f}"
                  f" corr {r['corr_x']:.2f}"
                  f" | y: true {r['true_mean_y']:+.3f} est"
                  f" {r['est_mean_y']:+.3f} rmse {r['rmse_y_mps']:.3f}"
                  f" corr {r['corr_y']:.2f}"
                  f" | fk {r['fk_us_per_tick']:.0f}us")
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=1))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
