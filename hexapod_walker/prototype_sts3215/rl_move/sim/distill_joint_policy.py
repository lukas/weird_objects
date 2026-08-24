"""Distill the body-IK goal policy into a raw joint-space policy (BC).

Warm-starting the joint-action task from the trained IK policy is
impossible directly — both the obs width (56 -> 68) and the action head
(6 -> 18) change — so this script does the honest equivalent:

1. Roll out the teacher in its own env (``SimHexapodGoalEnv``, DR on).
   At every step, record the STUDENT's view of the same underlying
   state (same proprioception/goal, prev-action in joint-action units)
   and the label: the safety-filtered 18-joint command the teacher's
   IK actually issued (``env._cmd``), normalized by the joint-space
   action map.
2. Supervised-train a fresh ``SimHexapodJointGoalEnv`` PPO policy
   (actor MSE on labels, critic MSE on the teacher's value estimate)
   so PPO fine-tuning starts from the teacher's behavior instead of
   mid-range flailing.
3. Save an SB3 zip loadable via ``train_ppo_sim --task joint_goal
   --init-from <out>``.

Run (from prototype_sts3215/):
    uv run python -m rl_move.sim.distill_joint_policy \
        --teacher rl_move/sim/policies/ppo_goal.zip \
        --episodes 300 --epochs 25
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from rl_move.env import build_obs

from .goal_task import SimHexapodGoalEnv
from .joint_task import SimHexapodJointGoalEnv, q_rad_to_action
from .servo_model import SimServoParams

POLICY_DIR = Path(__file__).resolve().parent / "policies"


def collect(teacher, env: SimHexapodGoalEnv, episodes: int,
            stochastic_frac: float, rng: np.random.Generator):
    """Teacher rollouts -> (student_obs, action_label, teacher_value)."""
    import torch

    obs_l, act_l, val_l = [], [], []
    mode_counts: dict[str, int] = {}
    for ep in range(episodes):
        deterministic = rng.random() >= stochastic_frac
        t_obs, info = env.reset()
        mode = info.get("goal_mode", "?")
        mode_counts[mode] = mode_counts.get(mode, 0) + 1
        a_prev_student = np.zeros(18, dtype=float)
        done = False
        while not done:
            # Student obs: same state/goal, prev-action in joint units.
            s_obs = build_obs(env.cfg, env._state, env._q_nom,
                              a_prev_student, goal=env._current_goal(),
                              tilt_ref=env._tilt_ref0)
            t_act, _ = teacher.predict(t_obs, deterministic=deterministic)
            with torch.no_grad():
                t_val = teacher.policy.predict_values(
                    torch.as_tensor(t_obs[None]).float()).item()
            t_obs, _, term, trunc, _ = env.step(t_act)
            # env._cmd is the safety-filtered joint command that was
            # actually issued this tick — the cleanest possible label.
            label = q_rad_to_action(env._cmd)
            obs_l.append(s_obs)
            act_l.append(label)
            val_l.append(t_val)
            a_prev_student = label
            done = term or trunc
    print(f"[distill] modes: {mode_counts}")
    return (np.asarray(obs_l, dtype=np.float32),
            np.asarray(act_l, dtype=np.float32),
            np.asarray(val_l, dtype=np.float32))


def train_student(student, obs: np.ndarray, act: np.ndarray,
                  val: np.ndarray, epochs: int, batch: int = 512,
                  lr: float = 1e-3) -> float:
    import torch

    policy = student.policy
    opt = torch.optim.Adam(policy.parameters(), lr=lr)
    obs_t = torch.as_tensor(obs)
    act_t = torch.as_tensor(act)
    val_t = torch.as_tensor(val).unsqueeze(-1)
    n = len(obs)
    # Critic targets are in return units (100s); scale that term so it
    # doesn't drown the actor loss but the value net still learns the
    # true return scale PPO will see.
    v_scale = 1.0 / max(float(np.var(val)), 1.0)
    last_a = last_c = float("nan")
    for ep in range(epochs):
        perm = torch.randperm(n)
        tot_a, tot_c, nb = 0.0, 0.0, 0
        for i in range(0, n, batch):
            idx = perm[i:i + batch]
            o, a, v = obs_t[idx], act_t[idx], val_t[idx]
            feats = policy.extract_features(o)
            if policy.share_features_extractor:
                lat_pi, lat_vf = policy.mlp_extractor(feats)
            else:  # pragma: no cover - default config shares
                lat_pi = policy.mlp_extractor.forward_actor(feats)
                lat_vf = policy.mlp_extractor.forward_critic(feats)
            mu = policy.action_net(lat_pi)
            v_pred = policy.value_net(lat_vf)
            loss_a = torch.nn.functional.mse_loss(mu, a)
            loss_c = torch.nn.functional.mse_loss(v_pred, v)
            loss = loss_a + 0.5 * v_scale * loss_c
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot_a += float(loss_a)
            tot_c += float(loss_c)
            nb += 1
        last_a, last_c = tot_a / max(nb, 1), tot_c / max(nb, 1)
        if ep % 5 == 0 or ep == epochs - 1:
            print(f"[distill] epoch {ep:3d}  actor_mse {last_a:.5f}  "
                  f"critic_mse {last_c:.1f}")
    return last_a


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--teacher", type=Path,
                    default=POLICY_DIR / "ppo_goal.zip")
    ap.add_argument("--out", type=Path,
                    default=POLICY_DIR / "ppo_joint_goal_bc.zip")
    ap.add_argument("--episodes", type=int, default=300)
    ap.add_argument("--stochastic-frac", type=float, default=0.3,
                    help="fraction of collection episodes rolled out with "
                         "sampling noise (state coverage off the teacher's "
                         "deterministic tube)")
    ap.add_argument("--epochs", type=int, default=25)
    ap.add_argument("--dr-scale", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    from stable_baselines3 import PPO

    rng = np.random.default_rng(args.seed)
    params = SimServoParams.load()
    teacher = PPO.load(args.teacher, device="cpu")
    print(f"[distill] teacher: {args.teacher}")

    t_env = SimHexapodGoalEnv(params=params, randomize=True,
                              dr_scale=args.dr_scale,
                              episode_seconds=10.0, seed=args.seed)
    t0 = time.monotonic()
    obs, act, val = collect(teacher, t_env, args.episodes,
                            args.stochastic_frac, rng)
    t_env.close()
    print(f"[distill] {len(obs)} samples in {time.monotonic() - t0:.0f}s")

    s_env = SimHexapodJointGoalEnv(params=params, randomize=True,
                                   dr_scale=args.dr_scale,
                                   episode_seconds=10.0, seed=args.seed)
    student = PPO(
        "MlpPolicy", s_env,
        n_steps=256, batch_size=2048, learning_rate=3e-4, gamma=0.99,
        gae_lambda=0.95, ent_coef=1e-3, clip_range=0.2,
        policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0),
        seed=args.seed, device="cpu", verbose=0)

    actor_mse = train_student(student, obs, act, val, args.epochs)
    # RMS in normalized action units; half-ranges are 35/55/85 deg, so
    # ~0.02 units ≈ 1 deg RMS on the worst axis.
    print(f"[distill] final actor RMS {np.sqrt(actor_mse):.4f} action "
          f"units (~{np.sqrt(actor_mse) * 85:.1f} deg on the knee axis)")

    student.save(args.out)
    print(f"[distill] saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
