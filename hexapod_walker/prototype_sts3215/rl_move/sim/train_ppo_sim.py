"""PPO training on the MuJoCo twin (stable-baselines3).

Trains entirely in sim with domain randomization on. Two tasks:

- ``--task goal`` (default): goal-conditioned lean / reference-tracking /
  weight-shift (54-dim obs = 46 + 8-dim goal, 5-dim body-offset action).
- ``--task balance``: the plain hold-level task (46-dim obs).

Policy I/O matches the hardware ``HexapodBalanceEnv`` conventions, so a
trained policy can later be evaluated on the robot — gated, as always,
on the operator explicitly asking for hardware motion.

Run (from prototype_sts3215/):
    # ~1 min smoke
    ../../.venv/bin/python -m rl_move.sim.train_ppo_sim --smoke

    # real run
    ../../.venv/bin/python -m rl_move.sim.train_ppo_sim \
        --steps 2000000 --n-envs 8

    # evaluate a checkpoint (sim only)
    ../../.venv/bin/python -m rl_move.sim.train_ppo_sim \
        --eval rl_move/sim/policies/ppo_balance.zip

Weights & Biases: runs log to wandb entity ``l2k2``, project
``hexapod-balance`` automatically when a key is available — put it in
``rl_move/sim/wandb.env`` (gitignored; see the placeholder there), or
``wandb login`` once, or export WANDB_API_KEY. ``--no-wandb`` disables.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .servo_model import SimServoParams  # noqa: E402
from .sim_env import SimHexapodBalanceEnv  # noqa: E402
from .goal_task import SimHexapodGoalEnv  # noqa: E402

ENV_CLASSES = {"balance": SimHexapodBalanceEnv, "goal": SimHexapodGoalEnv}

POLICY_DIR = Path(__file__).resolve().parent / "policies"
WANDB_ENV_FILE = Path(__file__).resolve().parent / "wandb.env"
WANDB_ENTITY_DEFAULT = "l2k2"
WANDB_PROJECT_DEFAULT = "hexapod-balance"


def _warn_if_defaults(params: SimServoParams) -> None:
    if params.source == "defaults":
        print("[train] WARNING: sim_model.json is defaults (no hardware "
              "fit yet). Fine for pipeline checks; re-train after "
              "fit_motor_model runs on real battery data.")


def _load_wandb_env() -> None:
    """Pull KEY=VALUE lines from wandb.env into os.environ (no override)."""
    if not WANDB_ENV_FILE.is_file():
        return
    for raw in WANDB_ENV_FILE.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, val = line.partition("=")
        key, val = key.strip(), val.strip()
        if key and val:
            os.environ.setdefault(key, val)


def _init_wandb(args, params: SimServoParams):
    """Start a W&B run, or return None (with the reason printed)."""
    if args.no_wandb:
        return None
    _load_wandb_env()
    try:
        import wandb
    except ImportError:
        print("[wandb] not installed — pip install wandb (logging skipped)")
        return None
    has_key = bool(os.environ.get("WANDB_API_KEY"))
    if not has_key:  # `wandb login` stores the key in ~/.netrc
        try:
            has_key = bool(wandb.api.api_key)
        except Exception:
            has_key = False
    if not has_key:
        print(f"[wandb] no API key — fill in {WANDB_ENV_FILE} "
              "(or `wandb login`); logging skipped")
        return None

    config = {
        "task": args.task,
        "steps": args.steps,
        "n_envs": args.n_envs,
        "seed": args.seed,
        "episode_seconds": args.episode_seconds,
        "domain_randomization": not args.no_dr,
        "sim_model_source": params.source,
        "sim_model_timestamp": params.timestamp,
        "net_arch": [128, 128],
        "n_steps": 256,
        "learning_rate": 3e-4,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "ent_coef": 1e-3,
        "clip_range": 0.2,
    }
    for axis, ax in params.axes.items():
        config[f"servo_{axis}"] = {
            "kp": ax.kp, "kv": ax.kv, "frictionloss": ax.frictionloss,
            "latency_ms": ax.latency_ms, "vel_max_deg_s": ax.vel_max_deg_s,
            "deadband_deg": ax.deadband_deg,
        }
    run = wandb.init(
        entity=os.environ.get("WANDB_ENTITY", WANDB_ENTITY_DEFAULT),
        project=os.environ.get("WANDB_PROJECT", WANDB_PROJECT_DEFAULT),
        dir=str(POLICY_DIR),
        config=config,
        sync_tensorboard=True,  # SB3 scalars (losses, ep_rew_mean, fps)
    )
    print(f"[wandb] logging to {run.url or 'offline run dir'}")
    return run


def _make_reward_parts_callback():
    """Log per-rollout means of the env's reward components and tilt."""
    from stable_baselines3.common.callbacks import BaseCallback

    class RewardPartsCallback(BaseCallback):
        KEYS = ("reward_roll", "reward_pitch", "reward_gyro",
                "reward_action", "reward_action_delta", "reward_current",
                "reward_unload", "reward_termination")

        def __init__(self):
            super().__init__()
            self._acc: dict[str, list[float]] = {}

        def _on_step(self) -> bool:
            for info in self.locals.get("infos", ()):
                for k in self.KEYS:
                    if k in info:
                        self._acc.setdefault(k, []).append(float(info[k]))
                for k in ("roll_deg", "pitch_deg"):
                    if k in info:
                        self._acc.setdefault(f"abs_{k}", []).append(
                            abs(float(info[k])))
                if "track_err_deg" in info:
                    self._acc.setdefault("track_err_deg", []).append(
                        float(info["track_err_deg"]))
            return True

        def _on_rollout_end(self) -> None:
            import wandb
            if not self._acc:
                return
            # No step= here: wandb ignores explicit steps under
            # tensorboard sync. Log global_step as a value instead so
            # env/ charts can share SB3's x-axis.
            payload = {f"env/{k}": float(np.mean(v))
                       for k, v in self._acc.items()}
            payload["global_step"] = self.num_timesteps
            wandb.log(payload)
            self._acc = {}

    return RewardPartsCallback()


def train(args) -> int:
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import CheckpointCallback

    params = SimServoParams.load()
    _warn_if_defaults(params)
    POLICY_DIR.mkdir(parents=True, exist_ok=True)
    run = _init_wandb(args, params)

    env_cls = ENV_CLASSES[args.task]

    def env_fn():
        return env_cls(
            params=params,
            randomize=not args.no_dr,
            episode_seconds=args.episode_seconds,
        )

    venv = make_vec_env(env_fn, n_envs=args.n_envs, seed=args.seed)
    model = PPO(
        "MlpPolicy", venv,
        n_steps=256,
        batch_size=min(2048, 256 * args.n_envs),
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=1e-3,
        clip_range=0.2,
        policy_kwargs=dict(net_arch=[128, 128]),
        seed=args.seed,
        verbose=1,
        device="cpu",  # MLP this small is faster on CPU than MPS
        tensorboard_log=str(POLICY_DIR / "tb") if run else None,
    )
    name = f"ppo_{args.task}"
    callbacks = [CheckpointCallback(
        save_freq=max(10_000 // args.n_envs, 1000),
        save_path=str(POLICY_DIR), name_prefix=name)]
    if run:
        callbacks.append(_make_reward_parts_callback())
    t0 = time.monotonic()
    model.learn(total_timesteps=args.steps, callback=callbacks,
                progress_bar=False)
    out = POLICY_DIR / f"{name}.zip"
    model.save(out)
    print(f"[train] {args.steps} steps in {time.monotonic() - t0:.0f}s → "
          f"{out}")
    venv.close()

    # Quick post-train eval (DR on) so the run ends with a number.
    stats = evaluate(out, episodes=10, no_dr=args.no_dr,
                     episode_seconds=args.episode_seconds, task=args.task)
    if run:
        import wandb
        for k, v in stats.items():
            run.summary[f"eval/{k}"] = v
        art = wandb.Artifact(name, type="model")
        art.add_file(str(out))
        run.log_artifact(art)
        run.finish()
    return 0


def evaluate(policy_path: Path, *, episodes: int = 10, no_dr: bool = False,
             episode_seconds: float | None = None,
             task: str = "balance") -> dict:
    from stable_baselines3 import PPO

    params = SimServoParams.load()
    _warn_if_defaults(params)
    env_cls = ENV_CLASSES[task]
    env = env_cls(
        params=params, randomize=not no_dr,
        episode_seconds=episode_seconds, seed=1234)
    model = PPO.load(policy_path, device="cpu")

    returns, tilts, track_errs, survived = [], [], [], 0
    for ep in range(episodes):
        obs, _ = env.reset()
        ret, max_tilt, done = 0.0, 0.0, False
        ep_track: list[float] = []
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            ret += r
            max_tilt = max(max_tilt, abs(info["roll_deg"]),
                           abs(info["pitch_deg"]))
            if "track_err_deg" in info:
                ep_track.append(float(info["track_err_deg"]))
            done = term or trunc
        returns.append(ret)
        tilts.append(max_tilt)
        if ep_track:
            track_errs.append(float(np.mean(ep_track)))
        survived += 0 if term else 1
    track_msg = (f" | track err {np.mean(track_errs):.2f}°"
                 if track_errs else "")
    print(f"[eval] {policy_path.name}: return "
          f"{np.mean(returns):+.3f} ± {np.std(returns):.3f} | "
          f"max tilt {np.mean(tilts):.2f}° | "
          f"survived {survived}/{episodes}{track_msg}")

    # Baseline: zero action, same seeds.
    env2 = env_cls(
        params=params, randomize=not no_dr,
        episode_seconds=episode_seconds, seed=1234)
    base = []
    for ep in range(episodes):
        obs, _ = env2.reset()
        ret, done = 0.0, False
        while not done:
            obs, r, term, trunc, _ = env2.step(np.zeros(5))
            ret += r
            done = term or trunc
        base.append(ret)
    print(f"[eval] zero-action baseline: {np.mean(base):+.3f} "
          f"± {np.std(base):.3f}")
    env.close()
    env2.close()
    stats = {
        "return_mean": float(np.mean(returns)),
        "return_std": float(np.std(returns)),
        "max_tilt_deg_mean": float(np.mean(tilts)),
        "survived": survived,
        "episodes": episodes,
        "baseline_return_mean": float(np.mean(base)),
    }
    if track_errs:
        stats["track_err_deg_mean"] = float(np.mean(track_errs))
    return stats


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--task", choices=sorted(ENV_CLASSES), default="goal",
                    help="balance = hold level; goal = goal-conditioned "
                         "lean / track / weight-shift (default)")
    ap.add_argument("--steps", type=int, default=1_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=10.0,
                    help="sim episodes can be longer than hardware's 5 s")
    ap.add_argument("--no-dr", action="store_true",
                    help="disable domain randomization (debug only)")
    ap.add_argument("--smoke", action="store_true",
                    help="10k steps, 2 envs — pipeline check")
    ap.add_argument("--eval", type=Path, default=None,
                    help="evaluate a saved policy instead of training")
    ap.add_argument("--no-wandb", action="store_true",
                    help="disable Weights & Biases logging")
    args = ap.parse_args(argv)

    if args.eval:
        evaluate(args.eval, no_dr=args.no_dr,
                 episode_seconds=args.episode_seconds, task=args.task)
        return 0
    if args.smoke:
        args.steps, args.n_envs = 10_000, 2
    return train(args)


if __name__ == "__main__":
    raise SystemExit(main())
