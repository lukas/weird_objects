"""Collect a dynamics dataset with batched GPU MJX/Warp physics.

Unlike ``collect.py``'s sequential C-MuJoCo path, this collector runs
thousands of independent robots on one accelerator and sizes the corpus from
the *training window budget*.  A 40k x 512 pretraining run with a maximum
reuse of 2 therefore collects at least 10.24M distinct train-window centers
before the Transformer starts.

The simulator's host shims still execute safety, reward, sensor corruption,
and frame extraction.  Physics and checkpoint-policy inference use the GPU;
the resulting dataset is consumed by ``GpuWindowSampler`` entirely on CUDA.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config                       # noqa: E402
from rl_move.dynamics import data as dd                      # noqa: E402
from rl_move.dynamics import frames as fr                    # noqa: E402
from rl_move.dynamics.collect import (                       # noqa: E402
    ACTOR_PROFILE, DEFAULT_MIX, DEFAULT_STANCE_CKPT, DEFAULT_WALK_CKPT,
    DR_CHOICES, DR_PROBS, _GaitActor, _next_shard_idx,
)
from rl_move.dynamics.collector_env import (                 # noqa: E402
    DynrepCollectWalkEnv,
)
from rl_move.sim.joint_task import q_rad_to_action           # noqa: E402
from rl_move.sim.servo_model import SimServoParams           # noqa: E402

WANDB_ENV_FILE = ROOT / "rl_move" / "sim" / "wandb.env"


def _load_wandb_env() -> None:
    if not WANDB_ENV_FILE.is_file():
        return
    for raw in WANDB_ENV_FILE.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        os.environ.setdefault(key.strip(), value.strip())


def _init_wandb(args, config: dict):
    if args.no_wandb:
        return None
    _load_wandb_env()
    import wandb
    run = wandb.init(
        entity=os.environ.get("WANDB_ENTITY", "l2k2"),
        project=os.environ.get("WANDB_PROJECT", "hexapod-balance"),
        group="dynrep-pretrain", job_type="gpu-data-generation",
        name=args.name, config=config,
        notes=(args.notes or "GPU MJX/Warp generation of fresh dynamics "
               "episodes sized to the Transformer optimizer budget."),
    )
    run.define_metric("global_step")
    run.define_metric("data/*", step_metric="global_step")
    print(f"[wandb] data generation logging to {run.url}")
    return run


def _exact_categories(rng: np.random.Generator, values, probs, n: int):
    """Allocate a shuffled categorical fleet with near-exact proportions."""
    probs = np.asarray(probs, dtype=float)
    probs = probs / probs.sum()
    raw = probs * n
    counts = np.floor(raw).astype(int)
    for i in np.argsort(-(raw - counts))[: n - int(counts.sum())]:
        counts[i] += 1
    out = np.concatenate([
        np.repeat(np.asarray(values, dtype=object)[i], count)
        for i, count in enumerate(counts)
    ])
    rng.shuffle(out)
    return out


def _parse_mix(raw: str | None) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_MIX)
    mix = {key: float(value) for key, value in
           (item.split("=", 1) for item in raw.split(","))}
    unknown = set(mix) - set(DEFAULT_MIX)
    if unknown:
        raise ValueError(f"unknown actor(s) in --mix: {sorted(unknown)}")
    if any(value < 0 for value in mix.values()) or sum(mix.values()) <= 0:
        raise ValueError("--mix weights must be nonnegative and sum above zero")
    return mix


class ActorFleet:
    """Original five-actor collection recipe over a vector environment."""

    def __init__(self, names: np.ndarray, *, dt: float, seed: int,
                 walk_ckpt: Path, stance_ckpt: Path, device: str):
        self.names = np.asarray(names, dtype=object)
        self.n = len(self.names)
        self.dt = float(dt)
        self.rngs = [np.random.default_rng(seed * 1_000_003 + i)
                     for i in range(self.n)]
        self.episode = np.zeros(self.n, dtype=np.int64)
        self.tick = np.zeros(self.n, dtype=np.int64)
        self.random_a = np.zeros((self.n, fr.ACTION_DIM), dtype=np.float32)
        self.random_mu = np.zeros_like(self.random_a)
        self.random_next = np.zeros(self.n, dtype=np.float32)
        self.gaits: dict[int, _GaitActor] = {}
        self.goal_traj: dict[int, object] = {}
        self.groups = {
            name: np.flatnonzero(self.names == name)
            for name in sorted(set(str(x) for x in self.names))
        }

        self.ckpts = {}
        ckpt_paths = {"walk_ckpt": walk_ckpt, "stance_ckpt": stance_ckpt}
        if any(name in self.groups for name in ckpt_paths):
            import torch
            from stable_baselines3 import PPO
            if device.startswith("cuda") and not torch.cuda.is_available():
                raise RuntimeError("checkpoint actors requested CUDA but "
                                   "torch.cuda.is_available() is false")
            for name, path in ckpt_paths.items():
                if name not in self.groups:
                    continue
                if not path.is_file():
                    raise FileNotFoundError(
                        f"required {name} checkpoint is missing: {path}")
                self.ckpts[name] = PPO.load(str(path), device=device)

    def reset(self, indices, initial, goal_traj=None) -> None:
        for pos, i0 in enumerate(indices):
            i = int(i0)
            frame, _priv, _mode, qnom = initial[pos]
            self.tick[i] = 0
            if self.names[i] == "random":
                q_abs = np.asarray(frame[:fr.N_JOINTS]) + np.asarray(qnom)
                a0 = q_rad_to_action(q_abs).astype(np.float32)
                self.random_a[i] = a0
                self.random_mu[i] = a0
                self.random_next[i] = 0.0
            elif self.names[i] in ("tripod", "noslip"):
                noise = (0.0, 0.05, 0.10)[int(self.episode[i] % 3)]
                self.gaits[i] = _GaitActor(
                    str(self.names[i]), self.rngs[i], noise)
                if goal_traj is not None:
                    self.goal_traj[i] = goal_traj[pos]

    def completed(self, indices) -> None:
        self.episode[np.asarray(indices, dtype=int)] += 1

    def actions(self, obs: np.ndarray) -> np.ndarray:
        actions = np.zeros((self.n, fr.ACTION_DIM), dtype=np.float32)

        for i0 in self.groups.get("random", ()):
            i = int(i0)
            rng = self.rngs[i]
            t = self.tick[i] * self.dt
            if t >= self.random_next[i]:
                self.random_mu[i] = rng.uniform(-1.0, 1.0, fr.ACTION_DIM)
                self.random_next[i] = t + rng.uniform(1.0, 3.0)
            a = self.random_a[i]
            a = a + 3.0 * (self.random_mu[i] - a) * self.dt
            a = a + 0.8 * np.sqrt(self.dt) * rng.normal(
                0.0, 1.0, fr.ACTION_DIM)
            self.random_a[i] = np.clip(a, -1.0, 1.0)
            actions[i] = self.random_a[i]

        for name in ("tripod", "noslip"):
            for i0 in self.groups.get(name, ()):
                i = int(i0)
                traj = self.goal_traj[i]
                env = SimpleNamespace(dt=self.dt, _goal_traj=traj)
                actions[i] = self.gaits[i](
                    self.tick[i] * self.dt, obs[i], env)

        for name, width in (("stance_ckpt", 68), ("walk_ckpt", 72)):
            group = self.groups.get(name)
            if group is None:
                continue
            for deterministic in (False, True):
                idx = group[(self.episode[group] % 2 == 0) == deterministic]
                if not len(idx):
                    continue
                pred, _ = self.ckpts[name].predict(
                    np.asarray(obs[idx, :width]), deterministic=deterministic)
                pred = np.asarray(pred, dtype=np.float32)
                noisy = (self.episode[idx] % 4) == 3
                for row in np.flatnonzero(noisy):
                    pred[row] += self.rngs[int(idx[row])].normal(
                        0.0, 0.05, fr.ACTION_DIM)
                actions[idx] = np.clip(pred, -1.0, 1.0)

        self.tick += 1
        return actions


def _write_shard(out: Path, idx: int, episodes: list[dict],
                 compressed: bool) -> Path:
    path = out / f"shard_{idx:04d}.npz"
    save = np.savez_compressed if compressed else np.savez
    save(
        path,
        frames=np.concatenate([e["frames"] for e in episodes]),
        actions=np.concatenate([e["actions"] for e in episodes]),
        priv=np.concatenate([e["priv"] for e in episodes]),
        ep_frames=np.array([len(e["frames"]) for e in episodes],
                           dtype=np.int64),
        ep_actions=np.array([len(e["actions"]) for e in episodes],
                            dtype=np.int64),
        ep_actor=np.array([e["actor"] for e in episodes]),
        ep_mode=np.array([e["mode"] for e in episodes]),
        ep_reason=np.array([e["reason"] for e in episodes]),
        ep_dr=np.array([e["dr"] for e in episodes], dtype=np.float32),
        ep_seed=np.array([e["seed"] for e in episodes], dtype=np.int64),
        ep_qnom=np.stack([e["qnom"] for e in episodes]),
    )
    return path


def _initial(venv, indices=None):
    return venv.env_method("dynrep_initial", indices=indices)


def _goal_traj(venv, indices=None):
    return venv.env_method("dynrep_goal_traj", indices=indices)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="dynrep-mjx-data")
    ap.add_argument("--notes", default="")
    ap.add_argument("--no-wandb", action="store_true")
    ap.add_argument("--optimizer-steps", type=int, default=40000)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--max-window-reuse", type=float, default=2.0)
    ap.add_argument("--history", type=int, default=16)
    ap.add_argument("--horizons", default="1,2,5,10,25")
    ap.add_argument("--episodes", type=int, default=0,
                    help="optional minimum episode count")
    ap.add_argument("--n-envs", type=int, default=2048)
    ap.add_argument("--host-workers", type=int, default=24)
    ap.add_argument("--impl", choices=("auto", "warp", "default"),
                    default="warp")
    ap.add_argument("--pool-per-env", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--walk-episode-seconds", type=float, default=12.0)
    ap.add_argument("--shard-episodes", type=int, default=2048)
    ap.add_argument("--compressed", action="store_true",
                    help="smaller shards but slower collection")
    ap.add_argument("--walk-ckpt", default=str(DEFAULT_WALK_CKPT))
    ap.add_argument("--stance-ckpt", default=str(DEFAULT_STANCE_CKPT))
    ap.add_argument("--mix", default=None)
    ap.add_argument("--actor-device", default="cuda")
    args = ap.parse_args()

    if args.optimizer_steps <= 0 or args.batch <= 0:
        raise SystemExit("--optimizer-steps and --batch must be positive")
    if args.max_window_reuse <= 0:
        raise SystemExit("--max-window-reuse must be positive")
    horizons = tuple(int(k) for k in args.horizons.split(","))
    target_train_windows = int(np.ceil(
        args.optimizer_steps * args.batch / args.max_window_reuse))
    run = _init_wandb(args, {
        **vars(args), "target_train_windows": target_train_windows,
        "collector": "mjx_sharded_gpu", "split_version": dd.SPLIT_VERSION,
    })
    if run is not None:
        run.log({"global_step": 0,
                 "data/target_train_windows": target_train_windows,
                 "data/max_window_reuse": args.max_window_reuse})

    # JAX must see these before the MJX backend is imported.  Disable its
    # large default preallocation so CUDA Torch checkpoint actors coexist.
    os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.70")
    import jax
    from rl_move.sim.mjx_sharded_vec_env import MjxShardedVecEnv
    if not any(d.platform in ("gpu", "cuda") for d in jax.devices()):
        raise RuntimeError(f"GPU MJX collection required; JAX devices are "
                           f"{jax.devices()}")
    impl = args.impl
    if impl == "auto":
        try:
            import mujoco_warp  # noqa: F401
            impl = "warp"
        except ImportError:
            impl = "default"
    impl_arg = None if impl == "default" else impl

    rng = np.random.default_rng(args.seed)
    mix = _parse_mix(args.mix)
    names = sorted(mix)
    actors = _exact_categories(
        rng, names, [mix[name] for name in names], args.n_envs)
    drs = _exact_categories(rng, DR_CHOICES, DR_PROBS, args.n_envs).astype(
        np.float32)
    profiles = np.array([ACTOR_PROFILE[str(name)] for name in actors],
                        dtype=object)

    cfg = load_config()
    params = SimServoParams.from_cfg(cfg)
    t0 = time.time()
    venv = MjxShardedVecEnv(
        DynrepCollectWalkEnv, args.n_envs,
        env_kwargs=dict(cfg=cfg, params=params, randomize=True, dr_scale=1.0,
                        episode_seconds=args.walk_episode_seconds),
        host_workers=args.host_workers, seed=args.seed, impl=impl_arg,
        pool_per_env=args.pool_per_env, desync_episodes=False, model_dr=True,
    )
    for profile in sorted(set(profiles)):
        for dr in DR_CHOICES:
            idx = np.flatnonzero((profiles == profile) & np.isclose(drs, dr))
            if len(idx):
                venv.env_method("dynrep_configure", str(profile), float(dr),
                                indices=idx.tolist())
    walk_idx = np.flatnonzero(profiles == "walk").tolist()
    other_idx = np.flatnonzero(profiles != "walk").tolist()
    if walk_idx:
        venv.set_attr("episode_steps", int(round(
            args.walk_episode_seconds / 0.04)), indices=walk_idx)
    if other_idx:
        venv.set_attr("episode_steps", int(round(
            args.episode_seconds / 0.04)), indices=other_idx)
    obs = venv.reset()
    initial = _initial(venv)

    gait_idx = np.flatnonzero(np.isin(actors, ("tripod", "noslip")))
    gait_goal = _goal_traj(venv, indices=gait_idx.tolist()) \
        if len(gait_idx) else []
    goal_by_idx = {int(i): goal for i, goal in zip(gait_idx, gait_goal)}
    fleet = ActorFleet(
        actors, dt=0.04, seed=args.seed,
        walk_ckpt=Path(args.walk_ckpt), stance_ckpt=Path(args.stance_ckpt),
        device=args.actor_device,
    )
    fleet.reset(range(args.n_envs), initial,
                [goal_by_idx.get(i) for i in range(args.n_envs)])

    max_steps = int(round(max(args.episode_seconds,
                              args.walk_episode_seconds) / 0.04))
    frame_buf = np.empty((args.n_envs, max_steps + 1, fr.FRAME_DIM),
                         dtype=np.float32)
    action_buf = np.empty((args.n_envs, max_steps, fr.ACTION_DIM),
                          dtype=np.float32)
    priv_buf = np.empty((args.n_envs, max_steps + 1, fr.PRIV_DIM),
                        dtype=np.float32)
    lengths = np.ones(args.n_envs, dtype=np.int32)
    modes = np.empty(args.n_envs, dtype=object)
    qnoms = np.empty((args.n_envs, fr.N_JOINTS), dtype=np.float32)
    for i, (frame, priv, mode, qnom) in enumerate(initial):
        frame_buf[i, 0] = frame
        priv_buf[i, 0] = priv
        modes[i] = mode
        qnoms[i] = qnom

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    existing_shards = sorted(out.glob("shard_*.npz"))
    if existing_shards:
        existing = dd.load_dataset(out)
        old_budget = dd.window_budget(existing, args.history, horizons)
        completed = len(existing)
        total_steps = sum(len(ep.actions) for ep in existing)
        train_windows = old_budget["train"]
        val_windows = old_budget["val"]
        test_windows = old_budget["test"]
        counts: dict[str, int] = {}
        for ep in existing:
            counts[ep.actor] = counts.get(ep.actor, 0) + 1
        print(f"append: found {completed} episodes and "
              f"{train_windows:,} train windows in {out}")
    else:
        completed = total_steps = 0
        train_windows = val_windows = test_windows = 0
        counts = {}
    started_completed = completed
    started_steps = total_steps
    started_train_windows = train_windows
    shard_idx = _next_shard_idx(out)
    shard: list[dict] = []
    last_report = time.time()
    try:
        while (completed < args.episodes
               or train_windows < target_train_windows):
            actions = fleet.actions(obs)
            obs, _rews, dones, infos = venv.step(actions)
            rows = np.arange(args.n_envs)
            write_at = lengths.copy()
            frame_buf[rows, write_at] = np.stack(
                [info["dynrep_frame"] for info in infos])
            priv_buf[rows, write_at] = np.stack(
                [info["dynrep_priv"] for info in infos])
            action_buf[rows, write_at - 1] = np.stack(
                [info["dynrep_action"] for info in infos])
            lengths += 1

            done_idx = np.flatnonzero(dones)
            accepted = []
            for i0 in done_idx:
                if (completed >= args.episodes
                        and train_windows >= target_train_windows):
                    break
                i = int(i0)
                nf = int(lengths[i])
                na = nf - 1
                reason = str(infos[i].get("termination_reason")
                             or ("trunc" if infos[i].get(
                                 "TimeLimit.truncated") else "term"))
                episode = dict(
                    frames=frame_buf[i, :nf].copy(),
                    actions=action_buf[i, :na].copy(),
                    priv=priv_buf[i, :nf].copy(), actor=str(actors[i]),
                    mode=str(modes[i]), reason=reason, dr=float(drs[i]),
                    seed=args.seed * 1_000_003 + completed,
                    qnom=qnoms[i].copy(),
                )
                shard.append(episode)
                valid = dd.valid_window_count(nf, args.history, horizons)
                split = dd.split_for_episode(completed)
                if split == "train":
                    train_windows += valid
                elif split == "val":
                    val_windows += valid
                else:
                    test_windows += valid
                completed += 1
                total_steps += na
                counts[str(actors[i])] = counts.get(str(actors[i]), 0) + 1
                accepted.append(i)
                if len(shard) >= args.shard_episodes:
                    path = _write_shard(out, shard_idx, shard,
                                        args.compressed)
                    print(f"wrote {path.name}: {len(shard)} episodes")
                    shard_idx += 1
                    shard = []

            if len(done_idx):
                fleet.completed(done_idx)
                new_initial = _initial(venv, indices=done_idx.tolist())
                gait_done = [int(i) for i in done_idx
                             if actors[int(i)] in ("tripod", "noslip")]
                new_goals = _goal_traj(venv, indices=gait_done) \
                    if gait_done else []
                new_goal_by_idx = dict(zip(gait_done, new_goals))
                fleet.reset(done_idx, new_initial,
                            [new_goal_by_idx.get(int(i)) for i in done_idx])
                for pos, i0 in enumerate(done_idx):
                    i = int(i0)
                    frame, priv, mode, qnom = new_initial[pos]
                    frame_buf[i, 0] = frame
                    priv_buf[i, 0] = priv
                    modes[i] = mode
                    qnoms[i] = qnom
                    lengths[i] = 1

            now = time.time()
            if now - last_report >= 15.0:
                elapsed = max(now - t0, 1e-6)
                reuse = (args.optimizer_steps * args.batch
                         / max(train_windows, 1))
                print(f"progress: {completed} eps, {total_steps:,} steps, "
                      f"train_windows={train_windows:,}/"
                      f"{target_train_windows:,}, planned_reuse={reuse:.2f}, "
                      f"{total_steps / elapsed:,.0f} env-step/s")
                if run is not None:
                    run.log({
                        "global_step": total_steps,
                        "data/episodes": completed,
                        "data/env_steps": total_steps,
                        "data/train_windows": train_windows,
                        "data/val_windows": val_windows,
                        "data/test_windows": test_windows,
                        "data/target_train_windows": target_train_windows,
                        "data/planned_window_reuse": reuse,
                        "data/env_steps_per_second": total_steps / elapsed,
                    })
                last_report = now
    finally:
        venv.close()

    if shard:
        path = _write_shard(out, shard_idx, shard, args.compressed)
        print(f"wrote {path.name}: {len(shard)} episodes")

    meta_path = out / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {
        "layout_version": fr.LAYOUT_VERSION, "frame_dim": fr.FRAME_DIM,
        "action_dim": fr.ACTION_DIM, "priv_dim": fr.PRIV_DIM,
        "priv_names": list(fr.PRIV_NAMES), "dt": 0.04, "runs": [],
    }
    meta["runs"].append({
        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
        "collector": "mjx_sharded_gpu", "impl": impl,
        "jax_devices": [str(d) for d in jax.devices()],
        "episodes": completed - started_completed, "seed": args.seed,
        "actor_counts_total": counts,
        "total_steps": total_steps - started_steps,
        "train_windows_added": train_windows - started_train_windows,
        "split_version": dd.SPLIT_VERSION,
        "train_windows": train_windows, "val_windows": val_windows,
        "test_windows": test_windows,
        "target_train_windows": target_train_windows,
        "planned_window_reuse": (args.optimizer_steps * args.batch
                                  / train_windows),
        "optimizer_steps": args.optimizer_steps, "batch": args.batch,
        "max_window_reuse": args.max_window_reuse,
        "n_envs": args.n_envs, "host_workers": args.host_workers,
        "mix": mix, "dr_choices": list(DR_CHOICES),
        "walk_ckpt": args.walk_ckpt, "stance_ckpt": args.stance_ckpt,
    })
    meta_path.write_text(json.dumps(meta, indent=2) + "\n")
    elapsed = time.time() - t0
    if run is not None:
        run.log({
            "global_step": total_steps,
            "data/episodes": completed,
            "data/env_steps": total_steps,
            "data/train_windows": train_windows,
            "data/val_windows": val_windows,
            "data/test_windows": test_windows,
            "data/target_train_windows": target_train_windows,
            "data/planned_window_reuse": (
                args.optimizer_steps * args.batch / train_windows),
            "data/env_steps_per_second": total_steps / max(elapsed, 1e-6),
            "data/complete": 1,
        })
        run.finish()
    print(f"done: {completed} eps, {total_steps:,} steps, "
          f"{train_windows:,} train windows in {elapsed / 60:.1f} min; "
          f"planned reuse "
          f"{args.optimizer_steps * args.batch / train_windows:.2f}x -> {out}")


if __name__ == "__main__":
    main()
