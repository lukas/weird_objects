"""calibrate_walk_height.py — measure the HONEST gait's height band.

Operator-approved direction (fb_20260817T005114_775298, item 6): the
90 mm walk_max_height_drop_mm cutoff on cw-arch-joystick-long-scratch3
was guessed, and video showed the policy living 40-77 mm low without
structural income suppression. Both walk-height knobs must instead be
CALIBRATED from the height distribution the hardware-proven scripted
gait actually rides:

- reward.walk_height_sigma_mm  <- spread of the honest gait's chassis
  height around the episode anchor (z - z0), so the Gaussian income
  gate pays the honest gait ~1.0 and the measured crouch band ~0.
- safety.walk_max_height_drop_mm <- below ANYTHING the honest gait
  ever visits (min h_rel minus margin), so the collapse termination
  can never clip a legitimate step.

Usage (controller CPU, ~1 min):

    python3 -m rl_move.sim.calibrate_walk_height \
        --teachers tripod noslip_clean --seconds 12 --seeds 3

Prints per-teacher stats and a recommended (sigma_mm, drop_mm) pair.
Pure measurement tool — imports the probe_walk_income env/teacher
stack unchanged, writes nothing, trains nothing.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.probe_walk_income import (  # noqa: E402
    STACKS, WALK_PLANT, make_env, pin_command,
)

DIRS = {"forward": (1.0, 0.0), "backward": (-1.0, 0.0),
        "crab_left": (0.0, 1.0)}


def measure(teacher: str, seed: int, direction: str, seconds: float,
            cmd: float, stack_name: str, skip_s: float) -> np.ndarray:
    from tripod_gait import TripodGait

    env = make_env(seed, STACKS[stack_name])
    env.reset()
    ux, uy = DIRS[direction]
    pin_command(env, ux * cmd, uy * cmd)
    traj = env._goal_traj
    n = len(traj.vx)
    if teacher.startswith("noslip"):
        from noslip_gait import NoSlipGait
        gait = (NoSlipGait.clamp_fit() if teacher == "noslip_clean"
                else NoSlipGait())
    else:
        gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    if hasattr(gait, "reset_phase"):
        gait.reset_phase()
    h_rel = []
    steps = int(round(seconds / env.dt))
    for step in range(steps):
        t = step * env.dt
        i = min(step, n - 1)
        gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
        act = q_rad_to_action(
            np.asarray(gait.desired_deg(t)) * DEG2RAD)
        _o, _r, term, trunc, _info = env.step(act)
        if t >= skip_s:
            h_rel.append(
                float(env.data.xpos[env._chassis_bid, 2]) - env._z0)
        if term or trunc:
            break
    env.close()
    return np.asarray(h_rel)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--teachers", nargs="+", default=["tripod"])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--skip-s", type=float, default=3.0,
                    help="drop the settle + gait-entry transient")
    ap.add_argument("--cmd", type=float, default=0.04)
    ap.add_argument("--stack", default="trans1", choices=list(STACKS))
    ap.add_argument("--margin-mm", type=float, default=10.0,
                    help="termination margin below the honest minimum")
    args = ap.parse_args()

    all_h = []
    for teacher in args.teachers:
        rows = []
        for seed in range(args.seeds):
            for direction in DIRS:
                h = measure(teacher, seed, direction, args.seconds,
                            args.cmd, args.stack, args.skip_s)
                if len(h):
                    rows.append(h)
        h = np.concatenate(rows) * 1000.0  # mm
        all_h.append(h)
        q = np.percentile(h, [0.1, 1, 5, 50, 95, 99, 99.9])
        print(f"{teacher}: n={len(h)} ticks  mean={h.mean():+.1f}mm "
              f"std={h.std():.1f}mm  min={h.min():+.1f} max={h.max():+.1f}")
        print(f"  p0.1={q[0]:+.1f} p1={q[1]:+.1f} p5={q[2]:+.1f} "
              f"p50={q[3]:+.1f} p95={q[4]:+.1f} p99={q[5]:+.1f} "
              f"p99.9={q[6]:+.1f}")
    h = np.concatenate(all_h)
    lo, hi = float(np.percentile(h, 0.1)), float(np.percentile(h, 99.9))
    # sigma: honest band half-width so the honest gait keeps >=~0.88
    # income (1 sigma at the band edge); crouch (-50mm) then keeps <=~0.1.
    half = max(abs(h.mean() - lo), abs(hi - h.mean()))
    sigma = max(math.ceil(half), 5)
    drop = math.ceil(-min(float(h.min()), 0.0) + args.margin_mm)
    print(f"\nRECOMMEND: reward.walk_height_sigma_mm={sigma} "
          f"(honest band {lo:+.1f}..{hi:+.1f} mm around mean "
          f"{h.mean():+.1f})")
    print(f"RECOMMEND: safety.walk_max_height_drop_mm={drop} "
          f"(honest min {h.min():+.1f} mm, margin {args.margin_mm} mm)")


if __name__ == "__main__":
    main()
