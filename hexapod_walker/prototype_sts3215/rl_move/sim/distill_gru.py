"""Distill the MLP champions into one recurrent GRU policy (BC).

Why: gru-r1..r4c showed from-scratch GRU+PPO at discovery budget (2M)
reliably finds the leg-sacrifice paddle before it finds walking. The
MLP reference (hist16-r7) needed 40M walk-only steps — nothing walks
from scratch at 2M. Instead of exploring, imitate: both champions are
single-frame 18-joint policies acting in the SAME action space as the
student, so BC labels are simply the teacher's actions.

Teachers (both run inside the student's walk env, the play.py bridge):
- walk episodes:  ppo_goal_cw_walk_longdist_r2 on the full walk obs;
- rise/lower/hold episodes: ppo_goal_cw_stance_dr10 on obs[:68]
  (walk env obs = stance obs + [vx_ref, vy_ref, vx_meas, vy_meas]).

The student is a RecurrentPPO GruActorCriticPolicy trained by BPTT
over WHOLE episodes (hidden state starts at zero only at true episode
starts — no mid-episode chunking, which would teach a bogus
"zero state mid-episode" convention). Saved as a normal SB3 zip for
``train_ppo_mjx --gru --init-from`` RL fine-tuning.

Run (from prototype_sts3215/):
    ../../.venv/bin/python -m rl_move.sim.distill_gru \
        --episodes 400 --epochs 30
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from .servo_model import SimServoParams
from .walk_task import SimHexapodJointWalkEnv

POLICY_DIR = Path(__file__).resolve().parent / "policies"

# The gru-r3/r4c env config (anti-cheat + joystick stack) so demo obs /
# goal-command distributions match what RL fine-tuning will see.
R3_CFG = {
    "reward.k_step_event": 1.0,
    "reward.k_drag_loaded": 10.0,
    "reward.k_park_duty": 1.0,
    "reward.walk_kernel_prog_gate": 1.0,
    "reward.walk_anchor_gate": 1.0,
    "reward.anchor_tol_mm": 10.0,
    "goal.walk_speed_min_m_s": 0.05,
    "goal.walk_speed_max_m_s": 0.06,
    "goal.walk_park_start_frac": 0.25,
    "goal.walk_heading_max_rad": 0.7854,
    "goal.walk_cmd_resample_s": 1.5,
    "goal.walk_cmd_resample_jitter": 0.6,
    "goal.walk_cmd_blend_s_min": 0.1,
    "goal.walk_cmd_blend_s_max": 1.0,
    "goal.walk_stop_frac": 0.2,
}

# r3/r4c training diet; episode counts per mode follow these weights.
DIET = {"walk": 0.60, "rise": 0.15, "lower": 0.15, "hold": 0.10}


def _build_cfg() -> dict:
    from rl_move.config import load_config
    cfg = load_config()
    for dotted, val in R3_CFG.items():
        node = cfg
        *path, leaf = dotted.split(".")
        for k in path:
            node = node.setdefault(k, {})
        node[leaf] = val
    return cfg


def _make_env(args, cfg: dict, params) -> SimHexapodJointWalkEnv:
    return SimHexapodJointWalkEnv(
        params=params, randomize=True, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed, cfg=cfg)


def collect(envs: dict, teachers: dict, episodes_by_mode: dict[str, int],
            stochastic_frac: float, rng: np.random.Generator):
    """Teacher rollouts -> list of per-episode (obs, act, val) arrays.

    ``envs`` maps mode -> env (stance modes may use the teacher's
    native episode length). Prints per-mode teacher episode returns —
    if the TEACHER scores badly in this env context, BC on more of its
    demos cannot help and the context is what needs fixing.
    """
    import torch

    episodes = []
    t0 = time.monotonic()
    for mode, n_ep in episodes_by_mode.items():
        teacher, n_t_obs = teachers["walk" if mode == "walk" else "stance"]
        env = envs[mode]
        env.set_goal_mix({m: (1.0 if m == mode else 0.0) for m in DIET})
        returns = []
        for _ in range(n_ep):
            deterministic = rng.random() >= stochastic_frac
            obs, info = env.reset()
            got = info.get("goal_mode", mode)
            obs_l, act_l, val_l = [], [], []
            done, ep_ret = False, 0.0
            while not done:
                t_obs = obs[:n_t_obs]
                act, _ = teacher.predict(t_obs, deterministic=deterministic)
                with torch.no_grad():
                    val = teacher.policy.predict_values(
                        torch.as_tensor(t_obs[None]).float()).item()
                obs_l.append(obs)
                act_l.append(np.clip(act, -1.0, 1.0))
                val_l.append(val)
                obs, r, term, trunc, _ = env.step(act)
                ep_ret += r
                done = term or trunc
            returns.append(ep_ret)
            episodes.append((got,
                             np.asarray(obs_l, dtype=np.float32),
                             np.asarray(act_l, dtype=np.float32),
                             np.asarray(val_l, dtype=np.float32)))
        print(f"[distill-gru] {mode}: {n_ep} eps, teacher return "
              f"med {np.median(returns):.0f} min {min(returns):.0f} "
              f"({time.monotonic() - t0:.0f}s elapsed)")
    return episodes


def collect_dagger(envs: dict, student, teachers: dict,
                   episodes_by_mode: dict[str, int]):
    """DAgger round: STUDENT drives (stateful, deterministic), TEACHER
    labels every visited state. Fixes BC compounding error — the
    student learns recoveries on its own trajectory distribution."""
    import torch

    episodes = []
    t0 = time.monotonic()
    for mode, n_ep in episodes_by_mode.items():
        teacher, n_t_obs = teachers["walk" if mode == "walk" else "stance"]
        env = envs[mode]
        env.set_goal_mix({m: (1.0 if m == mode else 0.0) for m in DIET})
        for _ in range(n_ep):
            obs, info = env.reset()
            got = info.get("goal_mode", mode)
            state, ep_start = None, np.ones((1,), dtype=bool)
            obs_l, act_l, val_l = [], [], []
            done = False
            while not done:
                t_obs = obs[:n_t_obs]
                label, _ = teacher.predict(t_obs, deterministic=True)
                with torch.no_grad():
                    val = teacher.policy.predict_values(
                        torch.as_tensor(t_obs[None]).float()).item()
                obs_l.append(obs)
                act_l.append(np.clip(label, -1.0, 1.0))
                val_l.append(val)
                act, state = student.predict(
                    obs, state=state, episode_start=ep_start,
                    deterministic=True)
                ep_start = np.zeros((1,), dtype=bool)
                obs, _, term, trunc, _ = env.step(act)
                done = term or trunc
            episodes.append((got,
                             np.asarray(obs_l, dtype=np.float32),
                             np.asarray(act_l, dtype=np.float32),
                             np.asarray(val_l, dtype=np.float32)))
        print(f"[distill-gru] dagger {mode}: {n_ep} eps "
              f"({time.monotonic() - t0:.0f}s elapsed)")
    return episodes


def train_student(student, episodes, epochs: int, batch_eps: int = 8,
                  lr: float = 1e-3) -> float:
    """Sequence BC: BPTT over whole padded episodes, masked loss."""
    import torch
    import torch.nn.functional as F

    policy = student.policy
    policy.set_training_mode(True)
    opt = torch.optim.Adam(policy.parameters(), lr=lr)

    seqs = [(torch.as_tensor(o), torch.as_tensor(a), torch.as_tensor(v))
            for _, o, a, v in episodes]
    all_vals = np.concatenate([v for _, _, _, v in episodes])
    v_scale = 1.0 / max(float(np.var(all_vals)), 1.0)
    n = len(seqs)
    last_a = float("nan")
    for ep in range(epochs):
        perm = np.random.permutation(n)
        tot_a = tot_c = 0.0
        nb = 0
        for i in range(0, n, batch_eps):
            batch = [seqs[j] for j in perm[i:i + batch_eps]]
            t_max = max(o.shape[0] for o, _, _ in batch)
            b = len(batch)
            obs_p = torch.zeros(t_max, b, batch[0][0].shape[1])
            act_p = torch.zeros(t_max, b, batch[0][1].shape[1])
            val_p = torch.zeros(t_max, b, 1)
            mask = torch.zeros(t_max, b, 1)
            for k, (o, a, v) in enumerate(batch):
                t = o.shape[0]
                obs_p[:t, k] = o
                act_p[:t, k] = a
                val_p[:t, k, 0] = v
                mask[:t, k, 0] = 1.0

            feats = policy.extract_features(
                obs_p.reshape(t_max * b, -1)).reshape(t_max, b, -1)
            # Whole episodes from reset: zero initial hidden state is
            # the truth, so a plain fused GRU call is exact BPTT.
            lat_pi, _ = policy.lstm_actor(feats)
            lat_vf, _ = policy.lstm_critic(feats)
            mu = policy.action_net(policy.mlp_extractor.forward_actor(lat_pi))
            v_pred = policy.value_net(
                policy.mlp_extractor.forward_critic(lat_vf))
            m_sum = mask.sum()
            loss_a = (F.mse_loss(mu, act_p, reduction="none")
                      * mask).sum() / (m_sum * mu.shape[-1])
            loss_c = (F.mse_loss(v_pred, val_p, reduction="none")
                      * mask).sum() / m_sum
            loss = loss_a + 0.5 * v_scale * loss_c
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            opt.step()
            tot_a += float(loss_a.detach())
            tot_c += float(loss_c.detach())
            nb += 1
        last_a = tot_a / max(nb, 1)
        if ep % 5 == 0 or ep == epochs - 1:
            print(f"[distill-gru] epoch {ep:3d}  actor_mse {last_a:.5f}  "
                  f"critic_mse {tot_c / max(nb, 1):.1f}")
    policy.set_training_mode(False)
    return last_a


def quick_probe(student, env, modes=("walk", "rise", "hold"),
                n_ep: int = 2) -> None:
    """Stateful deterministic sanity rollouts (not the real harness)."""
    for mode in modes:
        env.set_goal_mix({m: (1.0 if m == mode else 0.0) for m in DIET})
        rews = []
        for _ in range(n_ep):
            obs, _ = env.reset()
            state, ep_start = None, np.ones((1,), dtype=bool)
            done, tot = False, 0.0
            while not done:
                a, state = student.predict(
                    obs, state=state, episode_start=ep_start,
                    deterministic=True)
                ep_start = np.zeros((1,), dtype=bool)
                obs, r, term, trunc, _ = env.step(a)
                tot += r
                done = term or trunc
            rews.append(tot)
        print(f"[distill-gru] probe {mode}: ep returns "
              f"{[f'{r:.0f}' for r in rews]}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--walk-teacher", type=Path,
                    default=POLICY_DIR / "ppo_goal_cw_walk_longdist_r2.zip")
    ap.add_argument("--stance-teacher", type=Path,
                    default=POLICY_DIR / "ppo_goal_cw_stance_dr10.zip")
    ap.add_argument("--out", type=Path,
                    default=POLICY_DIR / "ppo_goal_cw_gru_bc.zip")
    ap.add_argument("--episodes", type=int, default=400,
                    help="total demo episodes, split per --mix")
    ap.add_argument("--mix", type=str, default=None,
                    help="demo mix, e.g. walk=0.3,rise=0.35,lower=0.15,"
                         "hold=0.2 (default: the r3 training diet). "
                         "bc2/bc3 post-mortem (r3-cfg harness): walk "
                         "below ~0.5 degrades the gait (bc3 walk=0.40 -> "
                         "gait_valid 1/6, sacrificed legs); rise never "
                         "clones regardless of volume (60->270 eps, best "
                         "1/6) — compounding error on a knife-edge "
                         "maneuver, not data poverty. Fix rise with RL, "
                         "not more demos.")
    ap.add_argument("--dagger-rounds", type=int, default=0,
                    help="after BC, N rounds of student-drives/teacher-"
                         "labels data aggregation. bc2 used 2 rounds "
                         "(confounded with 10s stance demos): det "
                         "lower/hold collapsed vs bc1 and rise did not "
                         "improve — no evidence it helps here; default 0")
    ap.add_argument("--dagger-episodes", type=int, default=150,
                    help="episodes per DAgger round (same --mix split)")
    ap.add_argument("--stance-episode-seconds", type=float, default=15.0,
                    help="episode length for rise/lower/hold demos. Keep "
                         "equal to --episode-seconds: bc2 collected these "
                         "at the stance teacher's native 10s and the "
                         "student regressed on the 15s eval context")
    ap.add_argument("--stochastic-frac", type=float, default=0.3)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--gru-hidden-size", type=int, default=256)
    ap.add_argument("--dr-scale", type=float, default=0.5)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    from sb3_contrib import RecurrentPPO
    from stable_baselines3 import PPO

    from .gru_policy import GruActorCriticPolicy

    rng = np.random.default_rng(args.seed)
    params = SimServoParams.load()
    cfg = _build_cfg()

    walk_teacher = PPO.load(args.walk_teacher, device="cpu")
    stance_teacher = PPO.load(args.stance_teacher, device="cpu")
    n_walk = int(walk_teacher.observation_space.shape[0])
    n_stance = int(stance_teacher.observation_space.shape[0])
    print(f"[distill-gru] teachers: walk obs {n_walk}, stance obs {n_stance}")

    env = _make_env(args, cfg, params)
    n_env_obs = int(env.observation_space.shape[0])
    if n_env_obs != n_walk:
        raise SystemExit(f"walk teacher obs {n_walk} != env obs {n_env_obs}")
    if n_stance >= n_env_obs:
        raise SystemExit(f"stance teacher obs {n_stance} not a prefix of "
                         f"env obs {n_env_obs}")
    # Stance demos run at the stance teacher's native episode length.
    import copy as _copy
    stance_args = _copy.copy(args)
    stance_args.episode_seconds = args.stance_episode_seconds
    stance_env = _make_env(stance_args, cfg, params)
    envs = {"walk": env, "rise": stance_env, "lower": stance_env,
            "hold": stance_env}

    mix = DIET
    if args.mix:
        mix = {k: float(v) for k, v in
               (kv.split("=") for kv in args.mix.split(","))}
        unknown = set(mix) - set(DIET)
        if unknown:
            raise SystemExit(f"--mix unknown modes: {unknown}")
    episodes_by_mode = {m: max(1, round(args.episodes * w))
                        for m, w in mix.items() if w > 0}
    teachers = {"walk": (walk_teacher, n_walk),
                "stance": (stance_teacher, n_stance)}
    episodes = collect(envs, teachers, episodes_by_mode,
                       args.stochastic_frac, rng)
    n_steps_total = sum(len(a) for _, _, a, _ in episodes)
    print(f"[distill-gru] {len(episodes)} episodes, "
          f"{n_steps_total} transitions")

    student = RecurrentPPO(
        GruActorCriticPolicy, env,
        n_steps=256, batch_size=8192, n_epochs=5, learning_rate=3e-4,
        gamma=0.99, gae_lambda=0.95, ent_coef=0.01, clip_range=0.2,
        target_kl=0.02,
        policy_kwargs=dict(lstm_hidden_size=args.gru_hidden_size),
        seed=args.seed, device="cpu", verbose=0)

    actor_mse = train_student(student, episodes, args.epochs)
    print(f"[distill-gru] BC actor RMS {np.sqrt(actor_mse):.4f} action "
          f"units (~{np.sqrt(actor_mse) * 85:.1f} deg on the knee axis)")

    dagger_by_mode = {m: max(1, round(args.dagger_episodes * w))
                      for m, w in mix.items() if w > 0}
    for rnd in range(args.dagger_rounds):
        new_eps = collect_dagger(envs, student, teachers, dagger_by_mode)
        episodes.extend(new_eps)
        actor_mse = train_student(student, episodes,
                                  max(args.epochs // 2, 5))
        print(f"[distill-gru] dagger round {rnd + 1}: dataset "
              f"{len(episodes)} eps, actor RMS {np.sqrt(actor_mse):.4f}")

    quick_probe(student, env)
    env.close()
    stance_env.close()

    # BC never trains log_std (default 0.0 = full-range noise, which
    # wrecks both the sto harness pass and early RL fine-tune rollouts).
    student.policy.log_std.data.fill_(-1.5)
    student.save(args.out)
    print(f"[distill-gru] saved {args.out} (log_std -1.5)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
