"""train_ppo_transfer.py — the A/B/C representation-transfer experiment.

Three PPO conditions, identical env / reward / budget / seeds; the ONLY
variable is where the policy's representation comes from
(rl_docs/DYNREP.md "First experimental comparison"):

    A  scratch    MlpPolicy on the raw stacked obs (H=16 frames)
    B  frozen     pretrained dyn encoder (dyn_v2_obs) -> frozen z ->
                  policy/value heads learn
    C  anchored   as B, but the encoder fine-tunes at a scaled-down LR
                  AND keeps training on the predictive objective: after
                  every rollout a few dynamics-loss steps on the
                  OFFLINE pretraining dataset anchor the encoder
                  (v1 approximation of L_total = L_PPO + lambda*L_dyn;
                  an online-window anchor is a documented follow-up).

Local pilot tasks (Mac-scale budgets; the brief's walk/yaw tasks need
pod-scale steps): "hold" (quiet plant stance) then "lower" (controlled
descent to belly = return toward the zero pose). Transfer protocol:
train task 1, checkpoint, warm-start task 2 with --init-from; the eval
callback measures BOTH tasks at every eval so retention curves come
for free.

    ../../.venv/bin/python -m rl_move.dynamics.train_ppo_transfer \
        --condition B --task hold --steps 150000 --seed 0 \
        --name pilot_hold_B

Outputs: models/ppo_<name>.zip + logs/ppo_<name>_eval.csv
(step, per-task mean return / episode length / early-termination rate).
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config                  # noqa: E402
from rl_move.sim.servo_model import SimServoParams      # noqa: E402

MODEL_DIR = ROOT / "rl_move" / "dynamics" / "models"
LOG_DIR = ROOT / "rl_move" / "dynamics" / "logs"
DEFAULT_ENCODER = "rl_move/dynamics/models/dyn_v2_obs.pt"

HISTORY = 16
FRAME_WIDTH = 72          # walk-env frame: 59 proprio + 13 goal/cmd
TASKS = ("hold", "lower")
TASK_GOALS = {"hold": {"hold": 1.0}, "lower": {"lower": 1.0}}
ALL_MODES = ("hold", "lean", "track", "unload", "raise", "rise",
             "lower", "quad", "walk")


def make_task_env(task: str, seed: int, dr_scale: float,
                  episode_seconds: float):
    """One walk-family env pinned to a single goal mode. Uses the walk
    env class for every task so obs width (72) and checkpoints are
    interchangeable across tasks."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    cfg.setdefault("obs", {})["history_frames"] = HISTORY
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=dr_scale > 0.0,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)
    gen = env._goal_gen
    for mode in ALL_MODES:
        if hasattr(gen, f"p_{mode}"):
            setattr(gen, f"p_{mode}",
                    float(TASK_GOALS[task].get(mode, 0.0)))
    return env


def _term_penalty_wrapper(env, penalty: float):
    """TRAINING-ONLY shaping: subtract a one-time penalty on early
    termination (tip/trip). The pilot's hold task has no alive bonus,
    so at small budgets every condition discovered that tipping over
    ~35 ticks in beats holding badly (measured 08-12: return -228 ->
    ~+1 by suicide). The terminal charge removes that escape without
    adding per-tick income. Evals run the RAW env so metrics stay
    comparable across phases and with the 08-12 pilot."""
    import gymnasium as gym

    class TermPenaltyWrapper(gym.Wrapper):
        def step(self, action):
            obs, r, term, trunc, info = self.env.step(action)
            if term and not trunc:
                r = float(r) - penalty
            return obs, r, term, trunc, info

    return TermPenaltyWrapper(env)


def _env_factory(task: str, seed: int, dr_scale: float,
                 episode_seconds: float, term_penalty: float):
    def _make():
        env = make_task_env(task, seed, dr_scale, episode_seconds)
        if term_penalty > 0.0:
            env = _term_penalty_wrapper(env, term_penalty)
        return env
    return _make


def eval_task(model, task: str, episodes: int, dr_scale: float,
              episode_seconds: float, seed0: int = 10_000) -> dict:
    """Deterministic episodes on fixed seeds -> mean return / length /
    early-termination (fall/trip) rate."""
    rets, lens, terms = [], [], 0
    env = make_task_env(task, seed0, dr_scale, episode_seconds)
    for i in range(episodes):
        obs, _ = env.reset(seed=seed0 + i)
        ret, n = 0.0, 0
        while True:
            act, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = env.step(act)
            ret += float(r)
            n += 1
            if term or trunc:
                terms += int(term)
                break
        rets.append(ret)
        lens.append(n)
    env.close()
    return {"return": float(np.mean(rets)),
            "ep_len": float(np.mean(lens)),
            "early_term_rate": terms / episodes}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--condition", required=True, choices=("A", "B", "C"))
    ap.add_argument("--task", required=True, choices=TASKS)
    ap.add_argument("--name", required=True)
    ap.add_argument("--steps", type=int, default=150_000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--term-penalty", type=float, default=30.0,
                    help="training-only one-time reward penalty on "
                         "early termination (0 disables); evals are "
                         "always raw")
    ap.add_argument("--dr-scale", type=float, default=0.3)
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--encoder", default=DEFAULT_ENCODER)
    ap.add_argument("--init-from", default=None,
                    help="warm-start checkpoint (transfer phase)")
    ap.add_argument("--eval-every", type=int, default=25_000)
    ap.add_argument("--eval-episodes", type=int, default=4)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--encoder-lr-scale", type=float, default=0.1)
    ap.add_argument("--anchor-data",
                    default="rl_move/dynamics/datasets/v2")
    ap.add_argument("--anchor-batches", type=int, default=4)
    ap.add_argument("--anchor-batch-size", type=int, default=256)
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from rl_move.dynamics import data as dd
    from rl_move.dynamics.model import dynamics_loss
    from rl_move.dynamics.sb3_encoder import (
        DynFeaturesExtractor, ScaledLRPPO, set_group_lrs,
    )

    torch.set_num_threads(2)
    venv = SubprocVecEnv(
        [_env_factory(args.task, args.seed * 1000 + i, args.dr_scale,
                      args.episode_seconds, args.term_penalty)
         for i in range(args.n_envs)])

    common = dict(
        n_steps=256, batch_size=min(2048, 256 * args.n_envs),
        learning_rate=args.lr, gamma=0.99, gae_lambda=0.95,
        ent_coef=1e-3, clip_range=0.2, seed=args.seed, verbose=0,
        device="cpu")
    enc_kwargs = dict(ckpt_path=str(ROOT / args.encoder),
                      frame_width=FRAME_WIDTH, history=HISTORY)

    if args.condition == "A":
        model = PPO("MlpPolicy", venv,
                    policy_kwargs=dict(net_arch=[128, 128],
                                       log_std_init=-1.0), **common)
    else:
        freeze = args.condition == "B"
        cls = PPO if freeze else ScaledLRPPO
        model = cls("MlpPolicy", venv, policy_kwargs=dict(
            net_arch=[128, 128], log_std_init=-1.0,
            features_extractor_class=DynFeaturesExtractor,
            features_extractor_kwargs={**enc_kwargs, "freeze": freeze},
        ), **common)
        # sb3's ortho_init wipes the extractor's Linears — restore.
        model.policy.features_extractor.reload_pretrained()
    if args.init_from:
        # Warm-start = policy WEIGHTS only, into the freshly built
        # model: every condition gets a fresh optimizer at the task
        # switch (also avoids the group-count mismatch that a full
        # PPO.load hits on condition C's two-group optimizer).
        from stable_baselines3.common.save_util import load_from_zip_file
        _, params, _ = load_from_zip_file(str(ROOT / args.init_from),
                                          device="cpu")
        model.policy.load_state_dict(params["policy"])
    if args.condition == "C":
        set_group_lrs(model.policy, args.lr, args.encoder_lr_scale)

    n_train = sum(p.numel() for p in model.policy.parameters()
                  if p.requires_grad)
    print(f"[{args.name}] condition {args.condition} task {args.task} "
          f"seed {args.seed}: {n_train / 1e6:.2f}M trainable params, "
          f"obs {venv.observation_space.shape}, "
          f"init_from={args.init_from}, term_penalty={args.term_penalty}")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = LOG_DIR / f"ppo_{args.name}_eval.csv"
    csv_f = open(csv_path, "w", newline="")
    csv_w = csv.DictWriter(csv_f, fieldnames=[
        "step", "wall_s",
        *[f"{t}/{m}" for t in TASKS
          for m in ("return", "ep_len", "early_term_rate")],
        "anchor_loss"])
    csv_w.writeheader()
    t0 = time.time()
    anchor_state = {"loss": float("nan")}

    def run_evals(step: int):
        row = {"step": step, "wall_s": round(time.time() - t0, 1),
               "anchor_loss": anchor_state["loss"]}
        for t in TASKS:
            m = eval_task(model, t, args.eval_episodes, args.dr_scale,
                          args.episode_seconds)
            row.update({f"{t}/{k}": round(v, 3) for k, v in m.items()})
        csv_w.writerow(row)
        csv_f.flush()
        print(f"  eval @ {step}: " + "  ".join(
            f"{t} ret={row[f'{t}/return']:.1f} "
            f"term={row[f'{t}/early_term_rate']:.2f}" for t in TASKS)
            + (f"  anchor={anchor_state['loss']:.3f}"
               if args.condition == "C" else ""))

    class EvalCb(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = args.eval_every

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                run_evals(self.num_timesteps)
                self._next += args.eval_every
            return True

    callbacks = [EvalCb()]

    if args.condition == "C":
        eps = dd.load_dataset(ROOT / args.anchor_data)
        stats = dd.Stats(
            model.policy.features_extractor.f_mean.numpy(),
            model.policy.features_extractor.f_std.numpy())
        dyn = model.policy.features_extractor.dyn
        sampler = dd.WindowSampler(eps, stats, HISTORY, dyn.horizons,
                                   val=False, seed=args.seed)
        lambdas = {"joint_pos": 1.0, "joint_vel": 1.0, "imu": 1.0,
                   "contact": 0.5, "latent": 1.0}
        anchor_opt = torch.optim.Adam(
            dyn.parameters(), lr=args.lr * args.encoder_lr_scale)

        class AnchorCb(BaseCallback):
            """Continue the predictive objective on the offline
            dataset after every PPO rollout (task-independent anchor
            for the shared encoder)."""

            def _on_training_start(self) -> None:
                # Loss of the untouched pretrained model — if this is
                # far above the pretraining val total, the checkpoint
                # or normalization wiring is wrong.
                with torch.no_grad():
                    b = sampler.batch(args.anchor_batch_size)
                    bt = {
                        "hist": torch.as_tensor(b["hist"]),
                        "fut_actions": torch.as_tensor(b["fut_actions"]),
                        "state": {k: torch.as_tensor(v)
                                  for k, v in b["state"].items()},
                        "contact": {k: torch.as_tensor(v)
                                    for k, v in b["contact"].items()},
                        "fut_hist": {k: torch.as_tensor(v)
                                     for k, v in b["fut_hist"].items()},
                    }
                    out = dyn(bt["hist"], bt["fut_actions"])
                    loss, _ = dynamics_loss(out, bt, lambdas, dyn)
                print(f"  anchor loss at start (pretrained, untouched): "
                      f"{float(loss):.3f}")

            def _on_rollout_end(self) -> None:
                dyn.train()
                losses = []
                for _ in range(args.anchor_batches):
                    b = sampler.batch(args.anchor_batch_size)
                    bt = {
                        "hist": torch.as_tensor(b["hist"]),
                        "fut_actions": torch.as_tensor(b["fut_actions"]),
                        "state": {k: torch.as_tensor(v)
                                  for k, v in b["state"].items()},
                        "contact": {k: torch.as_tensor(v)
                                    for k, v in b["contact"].items()},
                        "fut_hist": {k: torch.as_tensor(v)
                                     for k, v in b["fut_hist"].items()},
                    }
                    out = dyn(bt["hist"], bt["fut_actions"])
                    loss, _ = dynamics_loss(out, bt, lambdas, dyn)
                    anchor_opt.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(dyn.parameters(), 1.0)
                    anchor_opt.step()
                    losses.append(float(loss.detach()))
                anchor_state["loss"] = float(np.mean(losses))

            def _on_step(self) -> bool:
                return True

        callbacks.append(AnchorCb())

    run_evals(0)
    model.learn(total_timesteps=args.steps, callback=callbacks,
                reset_num_timesteps=True, progress_bar=False)
    run_evals(model.num_timesteps)
    out = MODEL_DIR / f"ppo_{args.name}.zip"
    model.save(str(out))
    csv_f.close()
    venv.close()
    print(f"[{args.name}] done in {(time.time() - t0) / 60:.1f} min "
          f"-> {out}\n  eval log: {csv_path}")


if __name__ == "__main__":
    main()
