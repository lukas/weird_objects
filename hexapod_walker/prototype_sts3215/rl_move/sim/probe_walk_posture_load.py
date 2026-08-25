"""probe_walk_posture_load.py — posture + hip-load fingerprint of a
walking checkpoint (operator kick 08-24 evening, load-probe session).

The operator's local probes found the walk champion
(ppo_goal_cw_walk_longdist_r2) walking in a sprawled crouch at body z
~70 mm instead of the ~110 mm plant stance; statically holding that
crouch costs 1.1-1.5 N*m per stance hip vs 0.23 N*m at plant pose, the
hip-pitch servos rail the 2.2 N*m clamp on ~2.5% of ticks (always in
stance, only on the LEFT legs L0-L2, 30-50% left/right lean). This
probe makes those exact quantities a repeatable harness so the
effort-pricing arms (cw-walk-posture-eff*) can be gated on them:

  - steady-state body z (absolute mm + offset vs episode anchor z0)
  - per-hip-pitch STANCE |qfrc_actuator|: mean, p95
  - clamp-rail duty: fraction of ticks with |tau| > rail threshold
    (default 2.15 N*m, just under the 2.2 N*m clamp), overall and
    stance-only
  - left (L0-L2) vs right (L3-L5) stance-hip mean torque + asymmetry %
  - swing-tick hip tau mean (contrast; rails should never be in swing)

Deterministic rollout, fixed forward command (default 0.055 m/s, the
champion's production speed), steady-state window after the pin ramp.
Reward-stack cfg does not affect dynamics; obs layout is the default
(longdist/anchorgate lineage adds no obs keys). Runs on the PRIMITIVE
model family by default (the probed lineage predates the 08-24 mesh
flip); --model-source mesh for mesh-era checkpoints.

    uv run python -m rl_move.sim.probe_walk_posture_load \
        --ckpt rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

LEFT, RIGHT = (0, 1, 2), (3, 4, 5)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="rl_move/sim/policies/"
                    "ppo_goal_cw_walk_longdist_r2.zip")
    ap.add_argument("--cmd", type=float, default=0.055)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--steady-from-s", type=float, default=3.0,
                    help="skip the pin hold (1 s) + ramp (1 s) + settle")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rail-nm", type=float, default=2.15,
                    help="|tau| above this counts as clamp-railed")
    ap.add_argument("--model-source", default="primitive",
                    choices=["primitive", "mesh", "mesh_mjx"])
    ap.add_argument("--set", action="append", default=[],
                    metavar="SEC.KEY=VAL", help="extra cfg overrides")
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    # model family must be pinned BEFORE the env import chain touches it
    os.environ["HEXAPOD_MODEL_SOURCE"] = args.model_source

    import rl_move.sim.probe_walk_income as pwi
    pwi.EPISODE_S = max(pwi.EPISODE_S, args.seconds + 1.0)
    extra = []
    for kv in args.set:
        key, val = kv.split("=", 1)
        sec, leaf = key.split(".", 1)
        try:
            val = float(val)
        except ValueError:
            pass
        extra.append(((sec, leaf), val))
    # longdist/anchorgate speed band; harmless for other lineages
    stack = {("goal", "walk_speed_min_m_s"): 0.05,
             ("goal", "walk_speed_max_m_s"): 0.06}
    env = pwi.make_env(args.seed, stack, extra_sets=tuple(extra))
    obs, _ = env.reset()
    pwi.pin_command(env, args.cmd, 0.0)

    from stable_baselines3 import PPO
    model = PPO.load(str(ROOT / args.ckpt) if not os.path.isabs(args.ckpt)
                     else args.ckpt, device="cpu")

    m = env.model
    pitch_dof = [int(m.joint(f"L{leg}_pitch").dofadr[0]) for leg in range(6)]
    z0 = float(env._z0)

    z_abs, z_off = [], []
    tau_stance = [[] for _ in range(6)]   # |tau| on this leg's contact ticks
    tau_swing = [[] for _ in range(6)]
    rail_ticks = [0] * 6                  # railed ticks (any phase)
    rail_stance_ticks = [0] * 6
    stance_ticks = [0] * 6
    n_ticks = 0
    step = 0
    while True:
        t = step * env.dt
        act, _ = model.predict(obs, deterministic=True)
        obs, _, term, trunc, _ = env.step(act)
        if t >= args.steady_from_s:
            n_ticks += 1
            z = float(env.data.xpos[env._chassis_bid, 2])
            z_abs.append(z * 1000.0)
            z_off.append((z - z0) * 1000.0)
            for f in range(6):
                tau = abs(float(env.data.qfrc_actuator[pitch_dof[f]]))
                adr = env._touch_adr[f]
                on = adr >= 0 and float(env.data.sensordata[adr]) > 0.5
                railed = tau > args.rail_nm
                if railed:
                    rail_ticks[f] += 1
                if on:
                    stance_ticks[f] += 1
                    tau_stance[f].append(tau)
                    if railed:
                        rail_stance_ticks[f] += 1
                else:
                    tau_swing[f].append(tau)
        step += 1
        if term or trunc or t >= args.seconds:
            break

    def _agg(samples):
        if not samples:
            return {"mean": None, "p95": None}
        a = np.asarray(samples)
        return {"mean": round(float(a.mean()), 3),
                "p95": round(float(np.percentile(a, 95)), 3)}

    per_leg = {}
    for f in range(6):
        per_leg[f"L{f}"] = {
            "stance": _agg(tau_stance[f]),
            "swing": _agg(tau_swing[f]),
            "stance_ticks": stance_ticks[f],
            "rail_duty_all": round(rail_ticks[f] / max(n_ticks, 1), 4),
            "rail_duty_stance": round(
                rail_stance_ticks[f] / max(stance_ticks[f], 1), 4),
        }
    left_means = [np.mean(tau_stance[f]) for f in LEFT if tau_stance[f]]
    right_means = [np.mean(tau_stance[f]) for f in RIGHT if tau_stance[f]]
    lmean = float(np.mean(left_means)) if left_means else float("nan")
    rmean = float(np.mean(right_means)) if right_means else float("nan")
    asym_pct = (abs(lmean - rmean) / max(min(lmean, rmean), 1e-9) * 100.0
                if left_means and right_means else float("nan"))
    all_stance = [s for leg in tau_stance for s in leg]
    out = {
        "ckpt": args.ckpt,
        "cmd_m_s": args.cmd,
        "seconds": args.seconds,
        "seed": args.seed,
        "model_source": args.model_source,
        "terminated_early": bool(term or trunc) and t < args.seconds,
        "n_steady_ticks": n_ticks,
        "body_z_abs_mm": {"mean": round(float(np.mean(z_abs)), 1),
                          "min": round(float(np.min(z_abs)), 1),
                          "p05": round(float(np.percentile(z_abs, 5)), 1)},
        "body_z_off_mm_vs_anchor": round(float(np.mean(z_off)), 1),
        "hip_rail_nm": args.rail_nm,
        "hip_rail_duty_all_legs": round(
            sum(rail_ticks) / max(n_ticks * 6, 1), 4),
        "stance_hip_tau": _agg(all_stance),
        "stance_hip_tau_left_mean": round(lmean, 3),
        "stance_hip_tau_right_mean": round(rmean, 3),
        "stance_hip_lr_asym_pct": round(asym_pct, 1),
        "per_leg": per_leg,
    }
    print(json.dumps(out, indent=2))
    if args.json_out:
        Path(args.json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_out).write_text(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
