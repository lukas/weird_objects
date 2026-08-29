"""Measure the scripted tripod teacher's per-tick direction-error floor.

WHAT THIS PROBE ANSWERS (standwalk coursedisp-c1 DIG-IN, 2026-08-29):
is the eval harness's `direction_err_mean_deg` headline — a PER-TICK
INSTANTANEOUS velocity-angle statistic — achievable at low values by a
KNOWN-GOOD course-follower on the CURRENT model family/cadence, or is
it floored by honest intra-stride sway?  The joystick track measured a
~35 deg tick-level floor on the PRIMITIVE family at 25 Hz and its DONE
gate judges deltas against that floor (CURRENT_TRUTHS).  No one has
measured the floor on the MESH family at 100 Hz, which is what every
standwalk-track policy is judged on.

Method: roll the hardware-proven scripted TripodGait (via the
sim_gait_compat convention boundary, same as build_motion_library)
through real physics at a fixed forward command, and report over the
commanded ticks:
  - per-tick instantaneous direction error (exact harness definition:
    walk_task.walk_direction_error_deg on _body_vel_xy vs command);
  - trailing-window NET-DISPLACEMENT direction error (the
    k_walk_course_disp mechanism's own quantity, default 1.5 s);
  - whole-rollout net path direction error + mean speed + fall flag.

Read-only diagnostic: no shared behavior changes, no cfg keys.
Usage (controller-ok, single env, ~1-2 min per rollout):
  uv run python -m rl_move.sim.probe_dir_floor \
      --model-source mesh --hz 100 --vx 0.08 --seconds 60
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-source", default="mesh",
                    choices=["mesh", "mesh_mjx", "primitive"])
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--max-delta-q-deg", type=float, default=None,
                    help="slew clamp per tick (default: 37.5deg/s / hz, "
                         "the rate-invariant physical contract)")
    ap.add_argument("--vx", type=float, default=0.08)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--window-s", type=float, default=1.5)
    ap.add_argument("--min-speed", type=float, default=0.005,
                    help="dir-err validity threshold m/s (5e-3 = the "
                         "joint_walk env's own inline emitter)")
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    # Same override pattern the pinned test suite uses; must be set
    # before the env/config imports below resolve anything.
    os.environ["HEXAPOD_MODEL_SOURCE"] = args.model_source
    os.environ["HEXAPOD_CONTROL_HZ"] = ("%g" % args.hz)

    from rl_move.config import load_config
    from rl_move.robot_state import DEG2RAD
    from rl_move.sim.joint_task import q_rad_to_action
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import (
        SimHexapodJointWalkEnv, walk_direction_error_deg)
    from rl_move.sim.probe_walk_income import WALK_PLANT
    from sim_gait_compat import TripodGait

    max_dq = (args.max_delta_q_deg if args.max_delta_q_deg is not None
              else 37.5 / args.hz)
    cfg = load_config()
    cfg.setdefault("safety", {})["max_delta_q_deg"] = float(max_dq)

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=args.seconds + 2.0,
        seed=args.seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    env.reset()

    gait = TripodGait(vx=0.0, lift=0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    gait.set_velocity(vx=args.vx, vy=0.0, omega=0.0)
    gait.reset_phase()

    dt = env.dt
    n = int(round(args.seconds / dt))
    win_ticks = max(int(round(args.window_s / dt)), 1)
    hist: deque = deque(maxlen=win_ticks + 1)

    tick_errs, win_errs, speeds = [], [], []
    n_cmd, n_valid = 0, 0
    fell = False
    xy0 = env.data.xpos[env._chassis_bid, :2].copy()
    # Six-leg gait-validity + loaded-slip telemetry: a floor measured
    # on a gliding/dragging rollout would be unrepresentative.
    prev_on = [False] * 6
    prev_xy = [None] * 6
    touchdowns = [0] * 6
    slip_m = 0.0
    for step in range(n):
        t = step * dt
        act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        _obs, _r, term, trunc, _info = env.step(act)
        for f in range(6):
            adr = env._touch_adr[f]
            on = bool(adr >= 0 and env.data.sensordata[adr] > 0.5)
            xy_world = env.data.xpos[env._pad_bids[f], :2].copy()
            if on and not prev_on[f]:
                touchdowns[f] += 1
            if on and prev_on[f] and prev_xy[f] is not None:
                slip_m += float(np.linalg.norm(xy_world - prev_xy[f]))
            prev_xy[f] = xy_world
            prev_on[f] = on
        v = env._body_vel_xy()
        n_cmd += 1
        err = walk_direction_error_deg(
            float(v[0]), float(v[1]), args.vx, 0.0,
            min_speed_m_s=args.min_speed)
        if err is not None:
            n_valid += 1
            tick_errs.append(err)
        speeds.append(float(np.hypot(*v)))
        bxy = env.data.xpos[env._chassis_bid, :2]
        hist.append((float(bxy[0]), float(bxy[1])))
        if len(hist) == hist.maxlen:
            dx = hist[-1][0] - hist[0][0]
            dy = hist[-1][1] - hist[0][1]
            d = math.hypot(dx, dy)
            if d / args.window_s >= 0.02:  # mechanism's own min speed
                # World +x == commanded direction at start; matches the
                # k_walk_course_disp dot-product convention for the
                # heading_max_rad=0 forward-only command family.
                win_errs.append(math.degrees(math.acos(
                    max(-1.0, min(1.0, dx / d)))))
        if term:
            fell = True
            break
        if trunc:
            break

    xy1 = env.data.xpos[env._chassis_bid, :2].copy()
    net = xy1 - xy0
    net_err = (math.degrees(math.atan2(net[1], net[0]))
               if np.hypot(*net) > 1e-6 else float("nan"))
    out = {
        "model_source": args.model_source, "hz": args.hz,
        "max_delta_q_deg": max_dq, "vx_cmd": args.vx,
        "seconds": args.seconds, "seed": args.seed, "fell": fell,
        "ticks": n_cmd, "dir_valid_frac": round(n_valid / max(n_cmd, 1), 4),
        "tick_dir_err_mean_deg": round(float(np.mean(tick_errs)), 2)
            if tick_errs else None,
        "tick_dir_err_med_deg": round(float(np.median(tick_errs)), 2)
            if tick_errs else None,
        "tick_dir_err_p90_deg": round(float(np.percentile(tick_errs, 90)), 2)
            if tick_errs else None,
        "win_dir_err_mean_deg": round(float(np.mean(win_errs)), 2)
            if win_errs else None,
        "win_dir_err_med_deg": round(float(np.median(win_errs)), 2)
            if win_errs else None,
        "window_s": args.window_s,
        "net_path_dir_err_deg": round(abs(net_err), 2),
        "net_disp_m": round(float(np.hypot(*net)), 4),
        "mean_speed_m_s": round(float(np.mean(speeds)), 4),
        "touchdowns_per_leg": touchdowns,
        "slip_per_m": round(slip_m / max(float(np.hypot(*net)), 1e-6), 3),
    }
    env.close()
    print(json.dumps(out, indent=1))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
