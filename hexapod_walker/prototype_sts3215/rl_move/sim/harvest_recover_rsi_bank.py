"""Harvest a checkpoint's OWN successful recovery rollouts as an
ON-PATH RSI bank for the harvested-bank RECOVER RSI variant
(goal.recover_rsi_bank_path; sim_env._recover_rsi_bank spawn branch,
walk_task._sample_recover's recover_rsi_bank flag).

Rationale (08-16, tangle-wall mechanism fix after cw-recover-any7/
any11/any12 all plateau on the same tangle-family kinds at a
statistically stable ~0.25-0.44 CERT band, closing the curriculum-
weight lever): the existing rise-mode RSI trick (goal.rise_rsi_frac,
extended to recover's zero family as goal.recover_rsi_frac) spawns
episodes on random waypoints of a SINGLE recorded belly->plant
reference trajectory. That reference only exists (and only makes
sense) for a monotonic-height rise; tangle's untangling motion has no
such single reference, and no one hand-choreographed it. But tangle
DOES already succeed sometimes (the 0.25-0.44 band means occasional
genuine recoveries happen) — so instead of a hand-built reference, this
script harvests ON-PATH poses directly from a checkpoint's own
successful tangle-family rollouts: roll the checkpoint stochastically
through forced episodes of the target kind(s), and for every episode
that reaches recover_success, keep a subsample of the joint poses
visited in the MIDDLE portion of that successful trajectory (skip the
first slice — that's just the kind's own start distribution, already
covered by ordinary training — and the last slice — that's the final
settled stance, essentially the same terminal pose the "zero"/plant
families already provide). The result is a plain (K,18) q_rad bank,
the same npz contract as goal.recover_start_bank / goal.rise_start_bank
— sim_env._recover_rsi_bank() loads it, the RSI spawn branch samples a
random row + small joint noise, exactly like the other harvested-bank
starts in this file family.

Usage:
  uv run python -m rl_move.sim.harvest_recover_rsi_bank CKPT.zip \
      --kinds tangle,tangle_deep,tangle_mid,tangle_mild \
      --episodes-per-kind 400 --seed 7000 \
      --out rl_move/sim/park_banks/NAME.npz \
      [--episode-seconds 16] [--cfg-set k=v ...] \
      [--window-lo 0.15] [--window-hi 0.85] [--stride 8] \
      [--max-poses-per-episode 6]

Harvest with a seed far from every eval/train stream (gate evals use
seeds 0/1; CERT/training streams are internal) — default 7000.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (_PROTO, _PROTO / "linux_control"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .servo_model import SimServoParams   # noqa: E402
from .walk_task import SimHexapodJointWalkEnv  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--kinds", type=str, required=True,
                    help="comma list of recover start_kind values to "
                         "force and harvest from")
    ap.add_argument("--episodes-per-kind", type=int, default=400)
    ap.add_argument("--seed", type=int, default=7000)
    ap.add_argument("--episode-seconds", type=float, default=16.0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--window-lo", type=float, default=0.15,
                    help="skip ticks before this fraction of the "
                         "episode's elapsed-to-success time (avoid the "
                         "kind's own raw start pose)")
    ap.add_argument("--window-hi", type=float, default=0.85,
                    help="skip ticks after this fraction (avoid the "
                         "final settled stance, already covered by "
                         "other families)")
    ap.add_argument("--stride", type=int, default=8,
                    help="keep every Nth in-window tick (avoid near-"
                         "duplicate adjacent poses dominating the bank)")
    ap.add_argument("--max-poses-per-episode", type=int, default=6,
                    help="cap how many poses one successful episode "
                         "contributes (avoid a few long episodes "
                         "dominating the bank)")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    cfg_kw = {}
    if args.cfg_set:
        from rl_move.config import load_config
        from .train_ppo_sim import _parse_cfg_set
        cfg = load_config()
        for key, parsed in _parse_cfg_set(args.cfg_set).items():
            sect, name = key.split(".", 1)
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg_kw.get("cfg")),
        randomize=args.dr_scale > 0, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed, **cfg_kw)
    # recover-only mix (mirrors harvest_lower_endpoints's per-mode gate)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk", "recover"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "recover" else 0.0)

    model = PPO.load(args.checkpoint, device="cpu")

    kinds = [k.strip() for k in args.kinds.split(",") if k.strip()]
    bank: list[np.ndarray] = []
    per_kind_stats: dict[str, dict] = {}
    t0 = time.time()
    for kind in kinds:
        env.force_recover_start = kind
        kept_eps = succ = 0
        for ep in range(args.episodes_per_kind):
            obs, _ = env.reset()
            traj_q: list[np.ndarray] = []
            done = False
            info: dict = {}
            while not done:
                traj_q.append(env.data.qpos[env._qadr].copy())
                # Deterministic, not stochastic: this campaign's own
                # "sto collapses to 0/N everywhere" recover finding
                # (action-noise vs the strict consecutive-hold success
                # criterion, documented on every recover run so far) is
                # a harness/reward artifact, not a real capability gap
                # — stochastic rollouts would starve this harvester of
                # successes even on kinds the checkpoint genuinely
                # solves. Small joint noise still applies at spawn
                # (sim_env's +/-2deg), so the harvested bank is not a
                # single point regardless.
                a, _ = model.predict(obs, deterministic=True)
                obs, _r, term, trunc, info = env.step(a)
                done = term or trunc
            success = bool(
                info.get("recover_success", 0.0) > 0.0
                or info.get("termination_reason") == "recover_success")
            if success:
                succ += 1
                n = len(traj_q)
                lo = int(args.window_lo * n)
                hi = max(lo + 1, int(args.window_hi * n))
                window = traj_q[lo:hi:max(1, args.stride)]
                window = window[: args.max_poses_per_episode]
                if window:
                    bank.extend(window)
                    kept_eps += 1
            if ep % 50 == 0:
                print(f"[harvest] {kind} ep {ep}/{args.episodes_per_kind}: "
                      f"succ={succ} bank_size={len(bank)}", flush=True)
        per_kind_stats[kind] = {
            "episodes": args.episodes_per_kind, "successes": succ,
            "episodes_contributed": kept_eps}
        print(f"[harvest] {kind}: {succ}/{args.episodes_per_kind} "
              f"succeeded, {kept_eps} contributed poses "
              f"(cert-band sanity: success_fraction="
              f"{succ / args.episodes_per_kind:.3f})", flush=True)

    if len(bank) == 0:
        print("[harvest] WARNING: empty bank — the source checkpoint "
              "never succeeded on any requested kind at this episode "
              "budget; nothing written, do not wire a bank into a "
              "training cfg without one", flush=True)
        return
    q_rad = np.asarray(bank, dtype=np.float64)
    if q_rad.ndim != 2 or q_rad.shape[1] != 18:
        raise RuntimeError(f"bad harvested bank shape {q_rad.shape}")
    md5 = hashlib.md5(args.checkpoint.read_bytes()).hexdigest()
    meta = {"kind": "recover_rsi_bank", "source_kinds": kinds,
            "checkpoint": str(args.checkpoint), "checkpoint_md5": md5,
            "episodes_per_kind": args.episodes_per_kind,
            "seed": args.seed, "dr_scale": args.dr_scale,
            "cfg_set": args.cfg_set, "window_lo": args.window_lo,
            "window_hi": args.window_hi, "stride": args.stride,
            "max_poses_per_episode": args.max_poses_per_episode,
            "per_kind_stats": per_kind_stats, "total_poses": len(q_rad)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, q_rad=q_rad, meta=json.dumps(meta))
    print(f"[harvest] wrote {len(q_rad)} on-path poses from "
          f"{sum(s['episodes_contributed'] for s in per_kind_stats.values())} "
          f"successful episodes -> {args.out} "
          f"({time.time() - t0:.0f}s)", flush=True)


if __name__ == "__main__":
    main()
