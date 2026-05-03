"""Benchmark a residual-gait policy (or the bare IK gait baseline) on a
fixed test suite of commanded twists.

Reports per-command:
  * mean tracking error in body-frame velocity (||v_body - v_cmd||)
  * mean tracking error in yaw rate
  * mean horizontal distance travelled
  * fall rate (% episodes that ended in a fall)
  * mean undiscounted return

and an aggregate "tracking score" = mean exp(-2 v_err) + 0.5 exp(-3 w_err),
matching the per-step reward shaping used in training.

Usage
-----

    # zero-residual baseline (= the bare IK gait)
    ./.venv/bin/python hexapod_walker/eval_walker.py --baseline

    # a saved PPO policy
    ./.venv/bin/python hexapod_walker/eval_walker.py \\
        --policy hexapod_walker/policies/walker_v1/walker_v1.zip

The same set of (command, terrain_seed, obstacle_seed) triples is used
for every run, so two evaluations are directly comparable.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_env as he  # noqa: E402


# A small fixed test set: variety of twists × a handful of terrains.
TEST_COMMANDS = [
    (0.30, 0.00,  0.00),   # straight forward
    (0.40, 0.00,  0.00),   # faster forward
    (-0.20, 0.00, 0.00),   # backward
    (0.00, 0.30,  0.00),   # strafe +Y
    (0.00, -0.30, 0.00),   # strafe -Y
    (0.30, 0.00,  0.10),   # forward + left curve
    (0.30, 0.00, -0.10),   # forward + right curve
    (0.20, 0.20,  0.00),   # diagonal
    (0.00, 0.00,  0.15),   # spin in place
    (0.00, 0.00, -0.15),   # spin opposite
]
TERRAIN_SEEDS = [101, 202, 303]    # 3 different terrains per command
OBSTACLE_SEEDS = [11, 22, 33]


def _make_env(*, episode_seconds, obstacle_count, terrain_enabled,
              terrain_seed, obstacle_seed, env_cfg=None):
    extra = {}
    if env_cfg:
        for key in ("residual_scale", "gait_period", "action_filter_tau"):
            if key in env_cfg:
                extra[key] = env_cfg[key]
    return he.HexapodWalkerEnv(
        episode_seconds=episode_seconds,
        obstacle_count=obstacle_count,
        terrain_enabled=terrain_enabled,
        terrain_seed=terrain_seed,
        obstacle_seed=obstacle_seed,
        randomize_command=False,
        terminate_on_fall=False,
        **extra,
    )


def _load_policy(policy_path, device="auto"):
    import json
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    cfg_path = os.path.join(os.path.dirname(policy_path), "env_cfg.json")
    env_cfg = None
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            env_cfg = json.load(f)

    # Tiny dummy env just to satisfy SB3 loader; we'll re-bind it.
    def _dummy():
        return _make_env(episode_seconds=1.0, obstacle_count=0,
                         terrain_enabled=False, terrain_seed=0,
                         obstacle_seed=0, env_cfg=env_cfg)
    venv = DummyVecEnv([_dummy])
    model = PPO.load(policy_path, env=venv, device=device)

    vn_path = os.path.join(os.path.dirname(policy_path), "vec_normalize.pkl")
    if os.path.exists(vn_path):
        norm = VecNormalize.load(vn_path, venv)
        norm.training = False
        norm.norm_reward = False
    else:
        norm = None
    return model, norm, env_cfg


def _evaluate_one(env, policy, normaliser, command, max_steps):
    """Run one episode with the given policy + command.  Returns metrics."""
    obs, _ = env.reset(options={"command": command})
    env.set_command(*command)

    # Trackers
    v_err_sum = 0.0
    w_err_sum = 0.0
    return_sum = 0.0
    p0 = env.data.body(env.idx.chassis_body).xpos[:2].copy()
    fell = False
    steps = 0

    for _ in range(max_steps):
        if policy is None:
            action = np.zeros(env.ACT_DIM, dtype=np.float32)
        else:
            obs_in = obs.astype(np.float32).reshape(1, -1)
            if normaliser is not None:
                obs_in = normaliser.normalize_obs(obs_in)
            action, _ = policy.predict(obs_in, deterministic=True)
            action = np.asarray(action).reshape(env.ACT_DIM)
        obs, reward, term, trunc, info = env.step(action)
        return_sum += reward
        steps += 1

        # Body-frame velocity tracking error
        quat = env.data.qpos[3:7]
        v_world = np.asarray(env.data.qvel[0:3], dtype=np.float64)
        w_world = np.asarray(env.data.qvel[3:6], dtype=np.float64)
        v_body = he._world_to_body(*quat, *v_world)
        w_body = he._world_to_body(*quat, *w_world)
        v_err_sum += math.hypot(v_body[0] - command[0],
                                v_body[1] - command[1])
        w_err_sum += abs(w_body[2] - command[2])
        if info.get("fell"):
            fell = True
        if term or trunc:
            break

    p1 = env.data.body(env.idx.chassis_body).xpos[:2].copy()
    disp = math.hypot(p1[0] - p0[0], p1[1] - p0[1])

    return {
        "steps": steps,
        "return": return_sum,
        "v_err_mean": v_err_sum / max(1, steps),
        "w_err_mean": w_err_sum / max(1, steps),
        "distance": disp,
        "fell": fell,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", type=str, default=None,
                    help="Path to a saved PPO .zip (omit for IK-only baseline)")
    ap.add_argument("--baseline", action="store_true",
                    help="Force IK-only baseline (zero residual)")
    ap.add_argument("--episode-seconds", type=float, default=6.0)
    ap.add_argument("--obstacles", type=int, default=4)
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--device", type=str, default="auto")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    env_cfg = None
    if args.baseline or args.policy is None:
        policy = None
        normaliser = None
        label = "IK baseline (zero residual)"
    else:
        policy, normaliser, env_cfg = _load_policy(args.policy, args.device)
        label = f"policy={os.path.basename(args.policy)}"

    print(f"=== Evaluating {label} ===")
    print(f"    episode_seconds={args.episode_seconds}, "
          f"obstacles={args.obstacles}, terrain={'on' if not args.no_terrain else 'off'}")
    print()

    rows = []
    per_cmd = {}
    for cmd in TEST_COMMANDS:
        per_cmd[cmd] = []
        for ts, os_ in zip(TERRAIN_SEEDS, OBSTACLE_SEEDS):
            env = _make_env(episode_seconds=args.episode_seconds,
                            obstacle_count=args.obstacles,
                            terrain_enabled=not args.no_terrain,
                            terrain_seed=ts, obstacle_seed=os_,
                            env_cfg=env_cfg)
            max_steps = int(round(args.episode_seconds * env.control_hz))
            r = _evaluate_one(env, policy, normaliser, cmd, max_steps)
            per_cmd[cmd].append(r)
            rows.append({"cmd": cmd, "terrain": ts, **r})
            env.close()
            if not args.quiet:
                print(f"  cmd=({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f})  "
                      f"terrain={ts:3d}  steps={r['steps']:3d}  "
                      f"d={r['distance']:.2f}m  v_err={r['v_err_mean']:.3f}  "
                      f"w_err={r['w_err_mean']:.3f}  fell={r['fell']}")

    # Per-command summary
    print()
    print(f"{'command':<22}  {'d̄ (m)':>8}  {'v_err':>7}  {'w_err':>7}  "
          f"{'falls':>6}  {'track':>6}")
    print("-" * 70)
    track_scores = []
    for cmd, results in per_cmd.items():
        d = np.mean([r["distance"] for r in results])
        v = np.mean([r["v_err_mean"] for r in results])
        w = np.mean([r["w_err_mean"] for r in results])
        fall = sum(r["fell"] for r in results) / len(results)
        track = math.exp(-2 * v) + 0.5 * math.exp(-3 * w)
        track_scores.append(track)
        cmd_str = f"({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f})"
        print(f"{cmd_str:<22}  {d:>8.2f}  {v:>7.3f}  {w:>7.3f}  "
              f"{fall*100:>5.0f}%  {track:>6.3f}")

    # Aggregate
    all_v = np.mean([r["v_err_mean"] for r in rows])
    all_w = np.mean([r["w_err_mean"] for r in rows])
    all_d = np.mean([r["distance"] for r in rows])
    all_fall = sum(r["fell"] for r in rows) / len(rows)
    all_ret = np.mean([r["return"] for r in rows])
    all_track = float(np.mean(track_scores))
    print("-" * 70)
    print(f"{'AGGREGATE':<22}  {all_d:>8.2f}  {all_v:>7.3f}  {all_w:>7.3f}  "
          f"{all_fall*100:>5.0f}%  {all_track:>6.3f}")
    print(f"  mean return per episode: {all_ret:+.2f}")
    print(f"  tracking score (higher is better, max 1.5):  {all_track:.3f}")


if __name__ == "__main__":
    main()
