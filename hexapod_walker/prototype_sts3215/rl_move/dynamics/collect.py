"""collect.py — diverse simulator rollouts for dynamics pretraining.

Goal: model the HEXAPOD'S dynamics, not one policy's trajectory
distribution. Each episode samples an ACTOR (what drives the joints)
and an env flavor (goal mix, DR scale), so the dataset covers standing,
walking, falls, recoveries, random flailing, and perturbed episodes —
failures included (tip terminations are kept, with the terminal frame).

Actors:
    random       OU-smoothed random joint actions (exploration noise
                 with temporal correlation; the safety layer's rate
                 limit applies downstream exactly as on hardware)
    tripod       scripted TripodGait teacher, +Gaussian action noise
    noslip       scripted NoSlipGait teacher, +Gaussian action noise
    stance_ckpt  a stance checkpoint (obs 68) on hold/rise/lower/lean
                 goals, det/sto alternating, sometimes +action noise
    walk_ckpt    a walk checkpoint (obs 72) on walk goals, det/sto
                 alternating, sometimes +action noise

Per episode DR scale is sampled from {0, 0.3, 0.6, 1.0} — DR supplies
perturbations too (tipped starts, walk kick/push, rise rock).

Output: npz shards under --out (append-safe; existing shards are kept):

    shard_XXX.npz:
        frames    (F_total, 86) float32   see frames.py
        actions   (A_total, 18) float32   executed [-1,1] actions
        priv      (F_total, 14) float32   privileged targets; never input
        ep_frames (E,) int64              frames per episode (= steps+1)
        ep_actions(E,) int64              actions per episode
        ep_actor / ep_mode / ep_reason (E,) str
        ep_dr     (E,) float32
        ep_seed   (E,) int64
    meta.json: layout version, dt, args, per-actor episode counts.

Usage (from prototype_sts3215/):

    ../../.venv/bin/python -m rl_move.dynamics.collect \
        --out rl_move/dynamics/datasets/v1 --episodes 400 --seed 0

Smoke: --episodes 10 --episode-seconds 4.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config                      # noqa: E402
from rl_move.robot_state import DEG2RAD                     # noqa: E402
from rl_move.dynamics import frames as fr                   # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action          # noqa: E402
from rl_move.sim.probe_walk_income import WALK_PLANT        # noqa: E402
from rl_move.sim.servo_model import SimServoParams          # noqa: E402

POLICY_DIR = ROOT / "rl_move" / "sim" / "policies"
DEFAULT_WALK_CKPT = POLICY_DIR / "ppo_goal_cw_walk_longdist_r2.zip"
DEFAULT_STANCE_CKPT = POLICY_DIR / "ppo_goal_cw_stance_dr10.zip"

# actor -> sampling weight (renormalized if a checkpoint is missing).
DEFAULT_MIX = {
    "random": 0.30,
    "tripod": 0.15,
    "noslip": 0.10,
    "stance_ckpt": 0.20,
    "walk_ckpt": 0.25,
}
DR_CHOICES = (0.0, 0.3, 0.6, 1.0)
DR_PROBS = (0.4, 0.2, 0.2, 0.2)

# Goal-mode mixes (applied to the generator's p_* attrs, hasattr-guarded
# like probe_walk_income does). "mixed" spreads mass over every mode so
# random-action episodes start from plants, bellies, crouches and
# mid-stride spawns alike.
GOAL_PROFILES = {
    "walk": {"walk": 1.0},
    "stance": {"hold": 0.10, "lean": 0.15, "track": 0.15, "raise": 0.15,
               "rise": 0.30, "lower": 0.15, "unload": 0.0, "quad": 0.0,
               "walk": 0.0},
    "mixed": {"hold": 0.10, "lean": 0.10, "track": 0.10, "raise": 0.10,
              "rise": 0.25, "lower": 0.10, "unload": 0.05, "quad": 0.0,
              "walk": 0.20},
}
ACTOR_PROFILE = {"random": "mixed", "tripod": "walk", "noslip": "walk",
                 "stance_ckpt": "stance", "walk_ckpt": "walk"}


def make_env(profile: str, dr_scale: float, episode_seconds: float,
             seed: int):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=dr_scale > 0.0,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)
    gen = env._goal_gen
    all_modes = ("hold", "lean", "track", "unload", "raise", "rise",
                 "lower", "quad", "walk")
    for mode in all_modes:
        if hasattr(gen, f"p_{mode}"):
            setattr(gen, f"p_{mode}",
                    float(GOAL_PROFILES[profile].get(mode, 0.0)))
    return env


class _OUActor:
    """Ornstein-Uhlenbeck action noise around a slowly resampled
    setpoint: temporally correlated flailing, not white jitter."""

    def __init__(self, rng: np.random.Generator, dt: float,
                 a0: np.ndarray):
        self.rng, self.dt = rng, dt
        self.a = a0.copy()
        self.mu = a0.copy()
        self.theta, self.sigma = 3.0, 0.8
        self._next_mu = 0.0

    def __call__(self, t: float, obs, env) -> np.ndarray:
        if t >= self._next_mu:
            self.mu = self.rng.uniform(-1.0, 1.0, self.a.shape)
            self._next_mu = t + self.rng.uniform(1.0, 3.0)
        self.a = self.a + self.theta * (self.mu - self.a) * self.dt \
            + self.sigma * np.sqrt(self.dt) * self.rng.normal(
                0.0, 1.0, self.a.shape)
        self.a = np.clip(self.a, -1.0, 1.0)
        return self.a


class _GaitActor:
    def __init__(self, kind: str, rng: np.random.Generator,
                 noise: float):
        self.rng, self.noise = rng, noise
        if kind == "noslip":
            from noslip_gait import NoSlipGait
            self.gait = NoSlipGait()
            self.gait.sync_plant_stance(*WALK_PLANT)
        else:
            from tripod_gait import TripodGait
            self.gait = TripodGait(vx=0.0)
            self.gait.sync_plant_stance(*WALK_PLANT)
            self.gait.reset_phase()

    def __call__(self, t: float, obs, env) -> np.ndarray:
        traj = env._goal_traj
        i = min(int(round(t / env.dt)), len(traj.vx) - 1)
        self.gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
        a = q_rad_to_action(np.asarray(self.gait.desired_deg(t)) * DEG2RAD)
        if self.noise > 0.0:
            a = a + self.rng.normal(0.0, self.noise, a.shape)
        return np.clip(a, -1.0, 1.0)


class _CkptActor:
    def __init__(self, model, obs_width: int, deterministic: bool,
                 rng: np.random.Generator, noise: float):
        self.model, self.w = model, obs_width
        self.det, self.rng, self.noise = deterministic, rng, noise

    def __call__(self, t: float, obs, env) -> np.ndarray:
        a, _ = self.model.predict(np.asarray(obs)[: self.w],
                                  deterministic=self.det)
        if self.noise > 0.0:
            a = a + self.rng.normal(0.0, self.noise, a.shape)
        return np.clip(np.asarray(a, dtype=float), -1.0, 1.0)


def _load_ckpt(path: Path):
    from stable_baselines3 import PPO
    return PPO.load(str(path), device="cpu")


def _make_actor(name: str, ep: int, rng: np.random.Generator, env,
                ckpts: dict):
    if name == "random":
        a0 = q_rad_to_action(env._state.joint_position)
        return _OUActor(rng, env.dt, a0)
    if name in ("tripod", "noslip"):
        noise = (0.0, 0.05, 0.10)[ep % 3]
        return _GaitActor(name, rng, noise)
    obs_w = 68 if name == "stance_ckpt" else 72
    det = ep % 2 == 0
    noise = 0.05 if ep % 4 == 3 else 0.0
    return _CkptActor(ckpts[name], obs_w, det, rng, noise)


def rollout(env, actor_factory, rng: np.random.Generator):
    """One episode. Returns (frames F=T+1, actions T, priv F, mode,
    term_reason). ``actor_factory(env)`` is called after reset so
    actors can read the sampled start pose."""
    obs, _ = env.reset()
    fr.reset_priv_episode(env)
    actor = actor_factory(env)
    mode = getattr(env._goal_traj, "mode", "?")
    prev_a = np.zeros(fr.ACTION_DIM)
    q_nom = env._q_nom.copy()
    frames = [fr.extract_frame(env, prev_a)]
    priv = [fr.extract_priv(env)]
    actions = []
    step, reason = 0, "end"
    while True:
        a = np.asarray(actor(step * env.dt, obs, env), dtype=np.float32)
        obs, _, term, trunc, info = env.step(a)
        actions.append(a)
        prev_a = a.astype(float)
        frames.append(fr.extract_frame(env, prev_a))
        priv.append(fr.extract_priv(env))
        step += 1
        if term or trunc:
            reason = str(info.get("termination")
                         or info.get("term_reason")
                         or ("trunc" if trunc else "term"))
            break
    return (np.stack(frames), np.stack(actions).astype(np.float32),
            np.stack(priv), str(mode), reason,
            q_nom.astype(np.float32))


def _next_shard_idx(out: Path) -> int:
    existing = sorted(out.glob("shard_*.npz"))
    return (int(existing[-1].stem.split("_")[1]) + 1) if existing else 0


def _write_shard(out: Path, idx: int, eps: list[dict]) -> Path:
    path = out / f"shard_{idx:03d}.npz"
    np.savez_compressed(
        path,
        frames=np.concatenate([e["frames"] for e in eps]),
        actions=np.concatenate([e["actions"] for e in eps]),
        priv=np.concatenate([e["priv"] for e in eps]),
        ep_frames=np.array([len(e["frames"]) for e in eps], dtype=np.int64),
        ep_actions=np.array([len(e["actions"]) for e in eps],
                            dtype=np.int64),
        ep_actor=np.array([e["actor"] for e in eps]),
        ep_mode=np.array([e["mode"] for e in eps]),
        ep_reason=np.array([e["reason"] for e in eps]),
        ep_dr=np.array([e["dr"] for e in eps], dtype=np.float32),
        ep_seed=np.array([e["seed"] for e in eps], dtype=np.int64),
        ep_qnom=np.stack([e["qnom"] for e in eps]),
    )
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--episodes", type=int, default=400)
    ap.add_argument("--seed", type=int, default=0)
    # >= 10 s: rise goals ramp after a ~5 s hold (goal.rise_hold_s) and
    # goal sampling raises on episodes shorter than hold + ramp.
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--walk-episode-seconds", type=float, default=12.0)
    ap.add_argument("--shard-episodes", type=int, default=100)
    ap.add_argument("--walk-ckpt", default=str(DEFAULT_WALK_CKPT))
    ap.add_argument("--stance-ckpt", default=str(DEFAULT_STANCE_CKPT))
    ap.add_argument("--mix", default=None,
                    help="override actor mix, e.g. random=0.5,tripod=0.5")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)

    mix = dict(DEFAULT_MIX)
    if args.mix:
        mix = {k: float(v) for k, v in
               (kv.split("=") for kv in args.mix.split(","))}
    ckpts: dict[str, object] = {}
    for actor, path in (("walk_ckpt", args.walk_ckpt),
                        ("stance_ckpt", args.stance_ckpt)):
        if mix.get(actor, 0.0) <= 0.0:
            continue
        p = Path(path) if Path(path).is_absolute() else ROOT / path
        if p.exists():
            ckpts[actor] = _load_ckpt(p)
        else:
            print(f"WARNING: {actor} checkpoint missing ({p}); "
                  f"reassigning its weight to random")
            mix["random"] = mix.get("random", 0.0) + mix.pop(actor)
    # noslip_gait.py is laptop-local (never committed); degrade the same
    # way a missing checkpoint does so pod-side collection works. Bit-
    # exact when the module is importable (the operator's machine).
    if mix.get("noslip", 0.0) > 0.0:
        try:
            import noslip_gait  # noqa: F401
        except ImportError:
            print("WARNING: noslip_gait module missing; "
                  "reassigning its weight to tripod")
            mix["tripod"] = mix.get("tripod", 0.0) + mix.pop("noslip")
    names = sorted(mix)
    probs = np.array([mix[n] for n in names])
    probs = probs / probs.sum()

    envs: dict[tuple, object] = {}
    shard: list[dict] = []
    shard_idx = _next_shard_idx(out)
    counts: dict[str, int] = {}
    t0 = time.time()
    total_steps = 0
    for ep in range(args.episodes):
        actor_name = str(rng.choice(names, p=probs))
        dr = float(rng.choice(DR_CHOICES, p=DR_PROBS))
        profile = ACTOR_PROFILE[actor_name]
        ep_s = (args.walk_episode_seconds if profile == "walk"
                else args.episode_seconds)
        key = (profile, dr, ep_s)
        if key not in envs:
            envs[key] = make_env(profile, dr, ep_s,
                                 seed=args.seed * 100_003 + len(envs))
        env = envs[key]
        frames, actions, priv, mode, reason, qnom = rollout(
            env, lambda e, n=actor_name, i=ep: _make_actor(n, i, rng, e,
                                                           ckpts), rng)
        shard.append(dict(frames=frames, actions=actions, priv=priv,
                          actor=actor_name, mode=mode, reason=reason,
                          dr=dr, seed=args.seed, qnom=qnom))
        counts[actor_name] = counts.get(actor_name, 0) + 1
        total_steps += len(actions)
        print(f"ep {ep:4d} actor={actor_name:12s} mode={mode:6s} "
              f"dr={dr:.1f} steps={len(actions):4d} end={reason}")
        if len(shard) >= args.shard_episodes:
            p = _write_shard(out, shard_idx, shard)
            print(f"  wrote {p.name} ({len(shard)} eps)")
            shard_idx += 1
            shard = []
    if shard:
        p = _write_shard(out, shard_idx, shard)
        print(f"  wrote {p.name} ({len(shard)} eps)")
    for env in envs.values():
        env.close()

    meta_path = out / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {
        "layout_version": fr.LAYOUT_VERSION, "frame_dim": fr.FRAME_DIM,
        "action_dim": fr.ACTION_DIM, "priv_dim": fr.PRIV_DIM,
        "priv_names": list(fr.PRIV_NAMES), "dt": 0.04, "runs": []}
    meta["priv_dim"] = fr.PRIV_DIM
    meta["priv_names"] = list(fr.PRIV_NAMES)
    meta["runs"].append({
        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
        "episodes": args.episodes, "seed": args.seed,
        "actor_counts": counts, "total_steps": total_steps,
        "mix": mix,
        "walk_ckpt": args.walk_ckpt, "stance_ckpt": args.stance_ckpt,
    })
    meta_path.write_text(json.dumps(meta, indent=2) + "\n")
    dt_min = (time.time() - t0) / 60.0
    print(f"done: {args.episodes} eps, {total_steps} steps "
          f"({total_steps / 25 / 60:.1f} sim-min) in {dt_min:.1f} min "
          f"-> {out}")


if __name__ == "__main__":
    main()
