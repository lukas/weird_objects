"""probe_drag_audit.py — the drag CHARGE-MAGNITUDE AUDIT (GAIT.md P3
lever 2 prerequisite, pre-registered 08-11; run by operator session).

QUESTION: what does reward.k_drag_loaded actually cost a typical
paddle tick, as a fraction of that tick's income? dragstep1 FAILED at
k=40 but env/reward_drag never exceeded ~0.09/tick against ~1/tick of
income — the "big penalty" was effectively small. Two suspects:
  (1) magnitude: k is simply 10-40x too small;
  (2) the 0.5 mm/tick per-foot deadband: at champion speeds
      (0.05-0.065 m/s, slip/m ~1-1.5) per-foot loaded slip is
      ~0.3-0.8 mm/tick — RIGHT AT the deadband, so much of the
      skating may be priced at literally zero.

WHAT IT DOES: rolls the scripted references from probe_walk_income
(honest gait / paddle / flag-leg sac1) plus optional checkpoints under
the trans1 stack, and measures the per-foot per-tick LOADED-slip
distribution directly from world-frame pad positions + touch sensors
(the exact quantities the reward reads). Reports:
  - slip/tick distribution and the fraction under 0.5 / 0.2 / 0.1 mm
    deadbands (how much skating is currently free);
  - actual income/tick (return minus the drag term itself);
  - the AUDIT-DERIVED k for each candidate deadband such that the
    paddle's drag charge/tick = TARGET_X x its income/tick, and what
    that same (k, deadband) costs the honest gait (must stay small —
    hardware slip is not failure, GAIT.md constitution).

    ../../.venv/bin/python -m rl_move.sim.probe_drag_audit \
        --out rl_move/sim/logs/probe_drag_audit.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.probe_walk_income import (  # noqa: E402
    CMD_V, PIN, VEL_SCALE, WALK_PLANT, make_env, pin_command,
)

TARGET_X = 2.5                 # paddle tick charge = 2.5x its income
DEADBANDS_MM = (0.5, 0.2, 0.1, 0.0)
# Per-stance allowance candidates for the STRUCTURAL charge (charge
# only accumulated loaded travel beyond this, per stance period).
ALLOWANCES_MM = (3.0, 5.0, 6.0, 8.0)
# Per-tick accumulation floor: contact-solver micro-jitter on a
# motionless loaded foot runs ~0.2 mm/tick and must not integrate
# (measured: a 2 s quiet stance accrues ~10 mm/foot of pure jitter).
# Matches reward.drag_stance_tick_floor_mm in walk_task.
TICK_FLOOR_MM = 0.25


def rollout(policy: str, seed: int, stack_name: str = "trans1") -> dict:
    from tripod_gait import TripodGait

    if policy.startswith("ckptplain:"):
        # Legacy flat-obs checkpoint (e.g. the longdist_r2 champion —
        # THE canonical learned skater): default cfg, privileged vel
        # obs, no history stack. Reward terms don't steer a frozen
        # policy; only the obs contract must match.
        from rl_move.sim.probe_estimator import make_env as make_plain
        env = make_plain(seed, 15.0, 0.0)
        policy = "ckpt:" + policy[len("ckptplain:"):]
    else:
        env = make_env(seed, __import__(
            "rl_move.sim.probe_walk_income", fromlist=["STACKS"]
        ).STACKS[stack_name])
    obs, _ = env.reset()
    pin_command(env, CMD_V, 0.0)
    traj = env._goal_traj
    n = len(traj.vx)

    model, gait = None, None
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    if policy.startswith("ckpt:"):
        from stable_baselines3 import PPO
        model = PPO.load(str(ROOT / policy[5:]), device="cpu")
    else:
        gait = TripodGait(vx=0.0)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()
    scale = VEL_SCALE.get(policy, 1.0)

    slips = []                       # every loaded per-foot per-tick slip (m)
    # Per-STANCE accumulated loaded travel (structural-charge candidate):
    # accumulate slip from touchdown to liftoff; a foot that never lifts
    # contributes its (huge) running total at episode end.
    stance_acc = [0.0] * 6
    stance_travels = []              # completed stances' accumulated slip
    prev_on = [False] * 6
    prev_xy = [None] * 6
    total = 0.0
    drag_sum = 0.0
    step = 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if model is not None:
            act, _ = model.predict(obs, deterministic=True)
        else:
            gait.set_velocity(vx=float(traj.vx[i]) * scale,
                              vy=float(traj.vy[i]) * scale, omega=0.0)
            q = np.asarray(gait.desired_deg(t)) * DEG2RAD
            for leg in PIN.get(policy, ()):
                q[3 * leg:3 * leg + 3] = plant_rad[3 * leg:3 * leg + 3]
            act = q_rad_to_action(q)
        obs, r, term, trunc, info = env.step(act)
        total += float(r)
        drag_sum += float(info.get("reward_drag", 0.0))
        for f in range(6):
            adr = env._touch_adr[f]
            on = adr >= 0 and float(env.data.sensordata[adr]) > 0.5
            xy = env.data.xpos[env._pad_bids[f], :2].copy()
            if on and prev_on[f] and prev_xy[f] is not None:
                s = float(np.linalg.norm(xy - prev_xy[f]))
                slips.append(s)
                if s > TICK_FLOOR_MM / 1000.0:
                    stance_acc[f] += s
            elif not on and prev_on[f]:
                stance_travels.append(stance_acc[f])   # liftoff closes it
                stance_acc[f] = 0.0
            prev_on[f], prev_xy[f] = on, xy
        step += 1
        if term or trunc:
            break

    # Feet still in stance at episode end contribute their running total
    # (a never-lifted paddle anchor IS the pathology being priced).
    stance_travels.extend(a for a in stance_acc if a > 0.0)
    st_arr = np.array(stance_travels) if stance_travels else np.zeros(1)
    slips_arr = np.array(slips) if slips else np.zeros(1)
    income_tick = (total - drag_sum) / max(step, 1)
    out = {
        "policy": policy, "seed": seed, "ticks": step,
        "income_per_tick": income_tick,
        "actual_drag_per_tick": drag_sum / max(step, 1),
        "loaded_slip_mm": {
            "mean": float(np.mean(slips_arr)) * 1000,
            "median": float(np.median(slips_arr)) * 1000,
            "p90": float(np.percentile(slips_arr, 90)) * 1000,
        },
        "stance_travel_mm": {
            "n_stances": int(len(st_arr)),
            "median": float(np.median(st_arr)) * 1000,
            "p90": float(np.percentile(st_arr, 90)) * 1000,
            "max": float(np.max(st_arr)) * 1000,
            "total_m": float(np.sum(st_arr)),
        },
        "deadbands": {},
        "stance_allowances": {},
    }
    for a_mm in ALLOWANCES_MM:
        a = a_mm / 1000.0
        excess = float(np.sum(np.maximum(st_arr - a, 0.0)))
        out["stance_allowances"][f"{a_mm}mm"] = {
            "frac_stances_free": float(np.mean(st_arr <= a)),
            "excess_m_per_episode": excess,
            # k (per metre of over-allowance travel) that prices the
            # episode's structural drag at TARGET_X x episode income.
            "k_for_target": (TARGET_X * income_tick * step / excess
                             if excess > 1e-6 and income_tick > 0
                             else None),
        }
    for d_mm in DEADBANDS_MM:
        d = d_mm / 1000.0
        charged = slips_arr[slips_arr > d]
        # Chargeable slip per TICK at this deadband (sum over feet).
        rate = float(np.sum(charged)) / max(step, 1)
        out["deadbands"][f"{d_mm}mm"] = {
            "frac_slip_ticks_free": float(np.mean(slips_arr <= d)),
            "chargeable_m_per_tick": rate,
            # k that makes drag charge = TARGET_X x income for THIS run
            "k_for_target": (TARGET_X * income_tick / rate
                             if rate > 1e-9 and income_tick > 0
                             else None),
        }
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policies", default="gait,paddle,sac1")
    ap.add_argument("--seeds", default="0,1")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    rows = []
    for pol in args.policies.split(","):
        for seed in (int(s) for s in args.seeds.split(",")):
            r = rollout(pol.strip(), seed)
            rows.append(r)
            s = r["loaded_slip_mm"]
            print(f"\n{pol} seed {seed}: {r['ticks']} ticks, "
                  f"income {r['income_per_tick']:.3f}/tick, actual drag "
                  f"{r['actual_drag_per_tick']:+.4f}/tick")
            print(f"  loaded slip/foot/tick: mean {s['mean']:.3f} mm, "
                  f"median {s['median']:.3f}, p90 {s['p90']:.3f}")
            st = r["stance_travel_mm"]
            print(f"  per-STANCE travel: n {st['n_stances']}, median "
                  f"{st['median']:.2f} mm, p90 {st['p90']:.2f}, max "
                  f"{st['max']:.1f}, episode total {st['total_m']:.3f} m")
            for a, v in r["stance_allowances"].items():
                k = v["k_for_target"]
                print(f"  allowance {a:>5}: {v['frac_stances_free']*100:5.1f}% "
                      f"stances free, excess {v['excess_m_per_episode']:.3f} "
                      f"m/ep, k for {TARGET_X}x income = "
                      + (f"{k:.0f}" if k else "n/a"))
            for d, v in r["deadbands"].items():
                k = v["k_for_target"]
                print(f"  deadband {d:>6}: {v['frac_slip_ticks_free']*100:5.1f}% "
                      f"of slip ticks FREE, chargeable "
                      f"{v['chargeable_m_per_tick']*1000:.3f} mm/tick, "
                      f"k for {TARGET_X}x income = "
                      + (f"{k:.0f}" if k else "n/a"))
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=1))
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
