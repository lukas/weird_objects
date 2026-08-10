"""Contact-pricing calibration: scripted-gait replay vs the tape measure.

Open problem 1 (RL_PLAN): sim prices sliding as FREE, so learned gaits
skate. The operator measured the real robot with a tape (2026-08-10,
``hardware_traces/tape_20260810_summary.json``): the scripted tripod
gait travels 0.50-0.51 of its commanded distance, invariant to speed
(30/50 mm/s) and swing lift (25/40 mm), on bare concrete with rubber
tips. Reward-side anti-slip shaping is a CLOSED move (10+ failed arms);
the sanctioned fix is to calibrate the PHYSICS so the same scripted
gait loses the same fraction of its stride in sim.

This script replays the EXACT hardware gait generator
(``linux_control/tripod_gait.py`` — the stdlib port that ran on the
robot, plant stance +20/+80) through the sim env's full servo/safety
stack at 25 Hz, and sweeps the foot-ground slide friction
(cfg ``env.foot_friction_slide``, see ``sim_env.set_foot_ground_friction``)
until sim travel ratio matches hardware:

    .venv/bin/python -m rl_move.sim.calibrate_slip                # sweep
    .venv/bin/python -m rl_move.sim.calibrate_slip --mu 0.6      # one point
    .venv/bin/python -m rl_move.sim.calibrate_slip --servo-params loaded

Output per (mu, vx): travel ratio (net chassis XY displacement /
commanded distance), plus mean synthesized servo current while walking
and while parked (hardware truth: walking 0.31-0.42 A mean is CHEAPER
than standing 0.59 A). A mu is a match when the ratio lands in
0.45-0.55 at BOTH speeds (speed invariance is the hardware's own
discriminator).

The calibrated value is then a per-run cfg:
    --cfg-set env.foot_friction_slide=<mu*>
Gate 0 (RL_PLAN) requires re-running this check whenever sim contact
params change.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402

# Hardware truth (tape_20260810_summary.json).
HW_RATIO = (0.50, 0.51)
HW_RATIO_BAND = (0.45, 0.55)
PLANT_HIP_DEG = 20.0
PLANT_KNEE_DEG = 80.0
HOLD_S = 1.5          # settle at plant before the gait starts
WALK_S = 10.0         # matches the 301 mm @ 30 mm/s tape runs


def _make_env(mu: float, servo_params: str, seed: int = 0):
    from rl_move.sim.joint_task import SimHexapodJointGoalEnv
    from rl_move.sim.servo_model import SimServoParams

    cfg = load_config()
    if servo_params:
        cfg.setdefault("bus", {})["servo_params"] = servo_params
    if mu > 0:
        cfg.setdefault("env", {})["foot_friction_slide"] = float(mu)
    # Replicate the HARDWARE write profile the tape runs used
    # (drive_controller WALK_SPEED=1500 steps/s ≈132°/s, acc 30) instead
    # of the training env's bench-gentled 400/20 + 1.5°/tick slew —
    # otherwise the sim swing foot can't complete its return at speed
    # and the comparison measures the write profile, not the contact.
    cfg.setdefault("bus", {})["write_speed"] = 1500
    cfg["bus"]["write_acc"] = 30
    cfg.setdefault("safety", {})["max_delta_q_deg"] = 8.0
    env = SimHexapodJointGoalEnv(
        cfg=cfg, params=SimServoParams.from_cfg(cfg), randomize=False,
        dr_scale=0.0, episode_seconds=HOLD_S + WALK_S + 1.0, seed=seed,
        plant_deg=[0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    return env


def replay(mu: float, vx: float, *, lift_mm: float = 25.0,
           servo_params: str = "", seed: int = 0) -> dict:
    """One scripted-gait episode; returns travel/current stats."""
    from tripod_gait import TripodGait

    from rl_move.sim.joint_task import q_rad_to_action

    env = _make_env(mu, servo_params, seed)
    env.reset()
    gait = TripodGait(vx=vx)
    gait.sync_plant_stance(PLANT_HIP_DEG, PLANT_KNEE_DEG)
    gait.set_lift_mm(lift_mm)
    gait.reset_phase()

    plant_rad = np.array([0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6) * DEG2RAD
    hold_steps = int(round(HOLD_S / env.dt))
    walk_steps = int(round(WALK_S / env.dt))
    cur_hold, cur_walk = [], []
    xy0 = None
    xy = env.data.xpos[env._chassis_bid, :2].copy()
    terminated = False
    for step in range(hold_steps + walk_steps):
        if step < hold_steps:
            act = q_rad_to_action(plant_rad)
        else:
            t = (step - hold_steps) * env.dt
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t)) * DEG2RAD)
            if xy0 is None:
                xy0 = env.data.xpos[env._chassis_bid, :2].copy()
        _obs, _r, term, trunc, info = env.step(act)
        xy = env.data.xpos[env._chassis_bid, :2].copy()
        (cur_hold if step < hold_steps else cur_walk).append(
            info.get("mean_current_a", 0.0))
        if term or trunc:
            terminated = term
            break
    travel_m = float(np.linalg.norm(xy - (xy0 if xy0 is not None else xy)))
    cmd_m = vx * WALK_S
    env.close()
    return {
        "mu": mu, "vx": vx, "lift_mm": lift_mm,
        "travel_m": round(travel_m, 4),
        "cmd_m": round(cmd_m, 4),
        "ratio": round(travel_m / cmd_m, 3) if cmd_m > 0 else None,
        "cur_walk_a": round(float(np.mean(cur_walk)), 3) if cur_walk else None,
        "cur_hold_a": round(float(np.mean(cur_hold)), 3) if cur_hold else None,
        "terminated": terminated,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mu", type=float, default=None,
                    help="single friction value (default: sweep)")
    ap.add_argument("--sweep", type=str,
                    default="0,1.5,1.2,1.0,0.8,0.6,0.5,0.4,0.3,0.2",
                    help="comma list of mu values (0 = XML default 2.0)")
    ap.add_argument("--vx", type=str, default="0.03,0.05",
                    help="comma list of speeds m/s (hardware: 0.03,0.05)")
    ap.add_argument("--lift-mm", type=float, default=25.0)
    ap.add_argument("--servo-params", default="",
                    help="'' = air fit, 'loaded' = 08-10 loaded bench fit")
    args = ap.parse_args()

    mus = [args.mu] if args.mu is not None else [
        float(x) for x in args.sweep.split(",")]
    vxs = [float(x) for x in args.vx.split(",")]

    print(f"hardware truth: ratio {HW_RATIO[0]}-{HW_RATIO[1]} at both "
          f"speeds; walking current 0.31-0.42 A < standing 0.59 A")
    print(f"servo params: {args.servo_params or 'air fit (default)'}")
    hdr = (f"{'mu':>5} " + " ".join(
        f"| vx={v:0.2f}: ratio travel/cmd curW curH" for v in vxs))
    print(hdr)
    for mu in mus:
        cells = []
        ratios = []
        for vx in vxs:
            r = replay(mu, vx, lift_mm=args.lift_mm,
                       servo_params=args.servo_params)
            ratios.append(r["ratio"])
            term = " TERM" if r["terminated"] else ""
            cells.append(
                f"| {r['ratio']:5.3f} {r['travel_m']:.3f}/{r['cmd_m']:.3f} "
                f"{r['cur_walk_a']:.2f} {r['cur_hold_a']:.2f}{term}")
        ok = all(HW_RATIO_BAND[0] <= x <= HW_RATIO_BAND[1] for x in ratios)
        tag = "  <-- MATCH" if ok else ""
        mu_s = f"{mu:5.2f}" if mu > 0 else "  XML"
        print(f"{mu_s} " + " ".join(cells) + tag)


if __name__ == "__main__":
    main()
