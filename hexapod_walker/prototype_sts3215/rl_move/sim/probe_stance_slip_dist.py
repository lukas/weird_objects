"""probe_stance_slip_dist.py — per-STANCE loaded-travel distribution
for a checkpoint under an exact --cfg-set env stack (dig-in tool,
2026-08-22, phasedir7/7b step-function root-cause).

QUESTION IT ANSWERS: where does a policy's completed-stance loaded
foot travel actually sit relative to reward.drag_stance_allow_mm —
i.e. is the k_drag_stance charge a CHEAT DISCRIMINATOR (honest gait
pays rarely) or a TAX ON TRAVEL (honest gait pays on most stances)?

The 08-11 audit (probe_drag_audit.py) calibrated allow=6mm against
the SCRIPTED teacher at the old 128 mm plant ("honest pays on <5% of
stances"). At the measured tibia-150 plant the honest phase clone's
per-stance MEAN is ~9.5 mm (pd7 A/B reports), so 6 mm taxes every
stance of the target behavior — this probe measures the full
distribution so the allowance can be recalibrated to the honest
band's upper tail instead of a stale constant.

Accounting is EXACTLY walk_task.py's k_drag_stance semantics:
touchdown resets the accumulator, only ticks sliding faster than
drag_stance_tick_floor_mm accumulate, liftoff completes a stance;
feet still loaded at episode end contribute their running totals.
For each candidate allowance it reports the fraction of stances that
pay, the episode excess (m), the charge at --k, and charge/income
(income = return minus the env's own reward_drag_stance term, so the
probe works whether or not the stack already enables the charge).

Usage (pod or controller; CPU, det, DR-0):
    python3 -m rl_move.sim.probe_stance_slip_dist \
        --checkpoint rl_move/sim/policies/<ckpt>.zip \
        --cfg-set goal.walk_phase_obs=1 ... \
        --episodes 4 --allowances 6,8,10,12,14,16 --k 8000 \
        --out logs/ckpt_eval/<tag>.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--task", default="joint_walk")
    ap.add_argument("--episodes", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--allowances", default="6,8,10,12,14,16",
                    help="candidate drag_stance_allow_mm values")
    ap.add_argument("--k", type=float, default=8000.0,
                    help="k_drag_stance ($/m) used to price candidates")
    ap.add_argument("--tick-floor-mm", type=float, default=0.25)
    # TRAINING-REGIME knobs (2026-08-22 phasedir8 dig-in): the 08-22
    # det/DR-0 calibration did not transfer to the run's actual
    # optimization regime (sto std 0.13, dr_scale 0.35, tipped starts
    # 0.30) — these let the probe measure the SAME distribution under
    # that regime. Defaults keep the original det/DR-0 behavior.
    ap.add_argument("--dr-scale", type=float, default=0.0,
                    help="env domain-randomization scale (0 = off)")
    ap.add_argument("--action-noise-std", type=float, default=0.0,
                    help="Gaussian noise added to the deterministic "
                         "action, emulating a forced PPO log_std "
                         "(e.g. 0.135 for --warm-log-std-override -2)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    from rl_move.sim.eval_checkpoint import ENV_CLASSES
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.train_ppo_sim import _parse_cfg_set

    cfg_kw = {}
    if args.cfg_set:
        from rl_move.config import load_config
        cfg = load_config()
        for key, parsed in _parse_cfg_set(args.cfg_set).items():
            sect, name = key.split(".", 1)
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg

    env_cls = ENV_CLASSES[args.task]
    env = env_cls(params=SimServoParams.from_cfg(cfg_kw.get("cfg")),
                  randomize=args.dr_scale > 0.0, dr_scale=args.dr_scale,
                  episode_seconds=args.episode_seconds,
                  seed=args.seed, render_mode=None, **cfg_kw)
    gen = env._goal_gen
    for a in list(vars(type(gen))) + list(vars(gen)):
        if a.startswith("p_"):
            setattr(gen, a, 1.0 if a == "p_walk" else 0.0)

    from rl_move.sim.gru_policy import load_checkpoint_auto
    model = load_checkpoint_auto(Path(args.checkpoint), device="cpu")
    n_model = int(model.observation_space.shape[0])
    n_env = int(env.observation_space.shape[0])
    if n_model != n_env:
        raise SystemExit(f"obs width mismatch: ckpt {n_model} vs env "
                         f"{n_env}; pass the run's own --cfg-set stack")

    floor_m = args.tick_floor_mm / 1000.0
    allowances = [float(x) for x in args.allowances.split(",")]
    episodes = []
    all_stances: list[float] = []
    for ep in range(args.episodes):
        obs, _ = env.reset()
        acc = [0.0] * 6
        prev_on = [False] * 6
        prev_xy = [None] * 6
        stances: list[float] = []
        ret, drag_paid, fwd = 0.0, 0.0, 0.0
        speeds = []
        noise_rng = np.random.default_rng(77_000 + args.seed * 100 + ep)
        while True:
            act, _ = model.predict(obs, deterministic=True)
            if args.action_noise_std > 0.0:
                act = np.clip(
                    act + noise_rng.normal(
                        0.0, args.action_noise_std, size=np.shape(act)),
                    env.action_space.low, env.action_space.high)
            obs, r, term, trunc, info = env.step(act)
            ret += float(r)
            drag_paid += float(info.get("reward_drag_stance", 0.0))
            if "v_along_cmd_m_s" in info:
                speeds.append(float(info["v_along_cmd_m_s"]))
            for f in range(6):
                adr = env._touch_adr[f]
                on = adr >= 0 and float(env.data.sensordata[adr]) > 0.5
                xy = env.data.xpos[env._pad_bids[f], :2].copy()
                if on and not prev_on[f]:
                    acc[f] = 0.0                       # touchdown reset
                elif on and prev_on[f] and prev_xy[f] is not None:
                    s = float(np.linalg.norm(xy - prev_xy[f]))
                    if s > floor_m:
                        acc[f] += s
                elif not on and prev_on[f]:
                    stances.append(acc[f])             # liftoff closes
                    acc[f] = 0.0
                prev_on[f], prev_xy[f] = on, xy
            if term or trunc:
                break
        stances.extend(a for a in acc if a > 0.0)      # still-loaded feet
        st = np.array(stances) if stances else np.zeros(1)
        income = ret - drag_paid
        row = {
            "episode": ep,
            "return": round(ret, 1),
            "income_ex_drag_stance": round(income, 1),
            "reward_drag_stance_paid": round(drag_paid, 1),
            "n_stances": int(len(st)),
            "stance_travel_mm": {
                "mean": round(float(np.mean(st)) * 1000, 2),
                "median": round(float(np.median(st)) * 1000, 2),
                "p75": round(float(np.percentile(st, 75)) * 1000, 2),
                "p90": round(float(np.percentile(st, 90)) * 1000, 2),
                "p95": round(float(np.percentile(st, 95)) * 1000, 2),
                "max": round(float(np.max(st)) * 1000, 2),
            },
            "allowances": {},
        }
        for a_mm in allowances:
            a = a_mm / 1000.0
            excess = float(np.sum(np.maximum(st - a, 0.0)))
            charge = args.k * excess
            row["allowances"][f"{a_mm:g}mm"] = {
                "frac_stances_paying": round(float(np.mean(st > a)), 3),
                "excess_m": round(excess, 4),
                "charge_at_k": round(charge, 1),
                "charge_over_income": (round(charge / income, 3)
                                       if income > 0 else None),
            }
        episodes.append(row)
        all_stances.extend(stances)
        print(f"ep{ep}: n_stances={row['n_stances']} "
              f"median={row['stance_travel_mm']['median']}mm "
              f"p90={row['stance_travel_mm']['p90']}mm "
              f"income={row['income_ex_drag_stance']}")
    st = np.array(all_stances) if all_stances else np.zeros(1)
    pooled = {
        "n_stances": int(len(st)),
        "mean": round(float(np.mean(st)) * 1000, 2),
        "median": round(float(np.median(st)) * 1000, 2),
        "p75": round(float(np.percentile(st, 75)) * 1000, 2),
        "p90": round(float(np.percentile(st, 90)) * 1000, 2),
        "p95": round(float(np.percentile(st, 95)) * 1000, 2),
        "max": round(float(np.max(st)) * 1000, 2),
        "frac_paying_at": {
            f"{a:g}mm": round(float(np.mean(st > a / 1000.0)), 3)
            for a in allowances},
    }
    out = {"checkpoint": args.checkpoint, "task": args.task,
           "seed": args.seed, "episodes": episodes,
           "k": args.k, "tick_floor_mm": args.tick_floor_mm,
           "pooled_stance_travel_mm": pooled}
    print(json.dumps(pooled, indent=1))
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(out, indent=1))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
