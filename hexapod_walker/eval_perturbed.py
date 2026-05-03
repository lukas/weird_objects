"""Evaluate a policy against the IK baseline under physical perturbations.

The point of domain randomisation is to make the policy robust to
perturbations the original simulator did not include — heavier rider,
slippery ground, motor delay, mis-calibrated joints, random initial
shoves.  This script runs the same fixed test suite as eval_walker.py
but turns one DR knob at a time on the EVAL env so we can see how each
policy holds up.
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

# Use the same fixed test suite as eval_walker for direct comparability.
TEST_COMMANDS = [
    (0.30, 0.00,  0.00),
    (0.40, 0.00,  0.00),
    (0.00, 0.30,  0.00),
    (0.30, 0.00,  0.10),
    (0.20, 0.20,  0.00),
]
TERRAIN_SEEDS = [101, 202, 303]
OBSTACLE_SEEDS = [11, 22, 33]


PERTURBATIONS = {
    "nominal":      dict(),
    "+25 % mass":   dict(dr_mass_pct=0.25),
    "−40 % friction": dict(dr_friction_pct=0.4),
    "60 ms motor latency": dict(dr_motor_latency_ms=60),
    "joint bias 1.5°":     dict(dr_joint_bias_rad=0.025),
    "0.15 m/s vel kick":   dict(dr_velocity_kick=0.15),
    "all combined":        dict(dr_mass_pct=0.25, dr_friction_pct=0.4,
                                dr_motor_latency_ms=60,
                                dr_joint_bias_rad=0.025,
                                dr_velocity_kick=0.10),
}


def _make_env(extra, terrain_seed, obstacle_seed, episode_seconds=6.0):
    return he.HexapodWalkerEnv(
        episode_seconds=episode_seconds,
        obstacle_count=4,
        terrain_enabled=True,
        terrain_seed=terrain_seed,
        obstacle_seed=obstacle_seed,
        randomize_command=False,
        terminate_on_fall=False,
        # Eval defaults: gait_period=1.0, residual_scale=0.06, filter=0.08
        **extra,
    )


def _load_policy(policy_path, device="auto"):
    if policy_path is None:
        return None, None, None
    import json
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    cfg_path = os.path.join(os.path.dirname(policy_path), "env_cfg.json")
    env_cfg = None
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            env_cfg = json.load(f)

    extra = {}
    if env_cfg:
        for k in ("residual_scale", "gait_period", "action_filter_tau"):
            if k in env_cfg:
                extra[k] = env_cfg[k]

    def _dummy():
        return _make_env({**extra}, terrain_seed=0, obstacle_seed=0,
                         episode_seconds=1.0)
    venv = DummyVecEnv([_dummy])
    model = PPO.load(policy_path, env=venv, device=device)
    vn_path = os.path.join(os.path.dirname(policy_path), "vec_normalize.pkl")
    norm = None
    if os.path.exists(vn_path):
        norm = VecNormalize.load(vn_path, venv)
        norm.training = False
        norm.norm_reward = False
    return model, norm, env_cfg


def _evaluate_one(env, policy, norm, command, max_steps):
    obs, _ = env.reset(options={"command": command})
    env.set_command(*command)
    v_err_sum = 0.0
    w_err_sum = 0.0
    fell = False
    steps = 0
    p0 = env.data.body(env.idx.chassis_body).xpos[:2].copy()
    for _ in range(max_steps):
        if policy is None:
            action = np.zeros(env.ACT_DIM, dtype=np.float32)
        else:
            obs_in = obs.astype(np.float32).reshape(1, -1)
            if norm is not None:
                obs_in = norm.normalize_obs(obs_in)
            action, _ = policy.predict(obs_in, deterministic=True)
            action = np.asarray(action).reshape(env.ACT_DIM)
        obs, _, term, trunc, info = env.step(action)
        steps += 1
        quat = env.data.qpos[3:7]
        v_world = np.asarray(env.data.qvel[0:3], dtype=np.float64)
        w_world = np.asarray(env.data.qvel[3:6], dtype=np.float64)
        v_body = he._world_to_body(*quat, *v_world)
        w_body = he._world_to_body(*quat, *w_world)
        v_err_sum += math.hypot(v_body[0] - command[0], v_body[1] - command[1])
        w_err_sum += abs(w_body[2] - command[2])
        if info.get("fell"): fell = True
        if term or trunc: break
    p1 = env.data.body(env.idx.chassis_body).xpos[:2].copy()
    disp = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    return dict(
        steps=steps, distance=disp, fell=fell,
        v_err=v_err_sum / max(1, steps),
        w_err=w_err_sum / max(1, steps),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policies", type=str, nargs="+", required=True,
                    help="Paths to PPO .zip files; use 'baseline' for IK-only")
    args = ap.parse_args()

    policies = []
    for p in args.policies:
        if p == "baseline":
            policies.append(("baseline (IK)", None, None, None))
        else:
            label = os.path.basename(p).replace(".zip", "")
            model, norm, _ = _load_policy(p)
            policies.append((label, model, norm, p))

    rows = {label: {} for label, _, _, _ in policies}
    for pert_name, pert_cfg in PERTURBATIONS.items():
        for label, model, norm, _ in policies:
            v_errs, w_errs, dists, falls = [], [], [], []
            for cmd in TEST_COMMANDS:
                for ts, os_ in zip(TERRAIN_SEEDS, OBSTACLE_SEEDS):
                    env = _make_env(pert_cfg, ts, os_, episode_seconds=6.0)
                    max_steps = int(round(6.0 * env.control_hz))
                    r = _evaluate_one(env, model, norm, cmd, max_steps)
                    v_errs.append(r["v_err"])
                    w_errs.append(r["w_err"])
                    dists.append(r["distance"])
                    falls.append(int(r["fell"]))
                    env.close()
            track = (np.mean([math.exp(-2 * v) for v in v_errs])
                     + 0.5 * np.mean([math.exp(-3 * w) for w in w_errs]))
            rows[label][pert_name] = dict(
                v_err=np.mean(v_errs),
                w_err=np.mean(w_errs),
                distance=np.mean(dists),
                fall_rate=np.mean(falls),
                track=track,
            )

    # Print a wide table: rows = perturbations, cols = policies (track score).
    pol_labels = [label for label, _, _, _ in policies]
    print()
    print("Tracking score (higher is better, max ≈ 1.5):")
    header = f"{'perturbation':<28}" + "".join(
        f"  {l:>14}" for l in pol_labels)
    print(header)
    print("-" * len(header))
    for pert in PERTURBATIONS:
        line = f"{pert:<28}"
        for label in pol_labels:
            r = rows[label][pert]
            line += f"  {r['track']:>14.3f}"
        print(line)

    print()
    print("Mean distance per 6 s episode (m):")
    print(header)
    print("-" * len(header))
    for pert in PERTURBATIONS:
        line = f"{pert:<28}"
        for label in pol_labels:
            line += f"  {rows[label][pert]['distance']:>14.2f}"
        print(line)

    print()
    print("Fall rate (%):")
    print(header)
    print("-" * len(header))
    for pert in PERTURBATIONS:
        line = f"{pert:<28}"
        for label in pol_labels:
            line += f"  {rows[label][pert]['fall_rate']*100:>13.0f}%"
        print(line)


if __name__ == "__main__":
    main()
