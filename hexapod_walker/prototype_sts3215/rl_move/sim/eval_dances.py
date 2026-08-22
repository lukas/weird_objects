#!/usr/bin/env python3
"""Render the demos-page standing dances in the MuJoCo twin.

Streams each dance's pose function into the sim exactly like the robot's
20 Hz web stream — through the fitted ``ServoProfile`` (bus latency +
trapezoid profile + deadband) — and writes one mp4 per dance plus a
stability verdict (max tilt, fell or not).

    python eval_dances.py                       # all dances, 12 s each
    python eval_dances.py --dances stand_hi --seconds 20 --speed 1.5
    python eval_dances.py --out /tmp/dances

Plain python (cv2 render path), NOT mjpython.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_SIM = Path(__file__).resolve().parent
_RL = _SIM.parents[0]
_PROTO = _SIM.parents[2]
for p in (_PROTO, _PROTO / "linux_control", _PROTO / "motor_setup",
          _RL.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.body_ik import fk_all_feet  # noqa: E402
from rl_move.robot_state import DEG2RAD, N_JOINTS  # noqa: E402
from rl_move.safety import AXIS_LIMITS_DEG  # noqa: E402
from rl_move.joint_frame import (  # noqa: E402
    robot_abs_deg_to_sim_rad, robot_stand_degrees,
)
from rl_move.sim.servo_model import (  # noqa: E402
    SIM_MODEL_PATH, ServoProfile, SimServoParams, apply_params_to_model,
    build_model, joint_qpos_addrs, position_actuator_ids,
)

# Robot logical plant pose.  The command stream is converted to MuJoCo's
# physical hinge frame at the servo-command boundary below.
ROBOT_PLANT_DEG = robot_stand_degrees()

CTRL_HZ = 20.0                 # the web stream's pursuit rate
STREAM_ACC_UNITS = 200         # stream_pose_fn max_acc register units


def clip_limits(q_rad: np.ndarray) -> np.ndarray:
    q = q_rad.copy()
    for j in range(N_JOINTS):
        lo, hi = AXIS_LIMITS_DEG[j % 3]
        q[j] = float(np.clip(q[j], lo * DEG2RAD, hi * DEG2RAD))
    return q


def place_at_plant(mujoco, model, data, qadr, pos_act,
                   q_rad: np.ndarray) -> None:
    """qpos at ``q_rad`` with the chassis at foot-contact height
    (mirror of SimHexapodBalanceEnv._place_at_plant)."""
    import mujoco_prototype as MP
    feet = fk_all_feet(q_rad)
    foot_drop = float(np.min(feet[:, 2]))
    base_z = MP.YAW_OUTPUT_HEIGHT - foot_drop + MP.FOOT_R + 0.002
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = (0.0, 0.0, base_z)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.qpos[qadr] = q_rad
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.ctrl[pos_act] = q_rad
    mujoco.mj_forward(model, data)
    for _ in range(40):
        worst = min((float(data.contact[ci].dist)
                     for ci in range(data.ncon)), default=0.0)
        if worst > -1e-4:
            break
        data.qpos[2] += -worst + 0.001
        mujoco.mj_forward(model, data)


def up_z(data, chassis_bid: int) -> float:
    """z-component of the chassis up axis (1 = level, <=0 = on side)."""
    return float(data.xmat[chassis_bid].reshape(3, 3)[2, 2])


def run_dance(name: str, *, seconds: float, speed: float,
              out_dir: Path, fps: int, width: int, height: int) -> dict:
    import cv2
    import mujoco
    import inplace_demos as D

    # Anchor the dances to the robot's real captured plant.
    D._stand_zero_pose = lambda: list(ROBOT_PLANT_DEG)
    pose_fn = D.make_stand_pose_fn(name)

    model = build_model(mesh_visuals=False, flat_terrain=True)
    params = SimServoParams.load(SIM_MODEL_PATH)
    apply_params_to_model(model, params)
    data = mujoco.MjData(model)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    chassis_bid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    q_plant = clip_limits(robot_abs_deg_to_sim_rad(ROBOT_PLANT_DEG))
    place_at_plant(mujoco, model, data, qadr, pos_act, q_plant)
    profile = ServoProfile(params, q_plant)

    h = model.opt.timestep
    substeps = max(1, int(round(1.0 / CTRL_HZ / h)))
    db = profile.deadband_rad

    def advance_tick():
        for _ in range(substeps):
            target = profile.tick(h)
            q = data.qpos[qadr]
            err = target - q
            eff = q + np.sign(err) * np.maximum(np.abs(err) - db, 0.0)
            data.ctrl[pos_act] = eff
            mujoco.mj_step(model, data)

    # settle onto the plant before dancing
    for _ in range(int(1.0 * CTRL_HZ)):
        profile.command(q_plant, acc_units=STREAM_ACC_UNITS)
        advance_tick()

    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 0.62
    cam.elevation = -18.0

    out_dir.mkdir(parents=True, exist_ok=True)
    vpath = out_dir / f"dance_{name}.mp4"
    vw = cv2.VideoWriter(str(vpath), cv2.VideoWriter_fourcc(*"mp4v"),
                         fps, (width, height))

    n_ticks = int(seconds * CTRL_HZ)
    render_every = max(1, int(round(CTRL_HZ / fps)))
    min_up = 1.0
    max_tilt_deg = 0.0
    fell_at = None
    for i in range(n_ticks):
        t = i / CTRL_HZ * speed          # dance/demo time
        q_goal = clip_limits(robot_abs_deg_to_sim_rad(pose_fn(t)))
        profile.command(q_goal, acc_units=STREAM_ACC_UNITS)
        advance_tick()
        uz = up_z(data, chassis_bid)
        min_up = min(min_up, uz)
        max_tilt_deg = max(max_tilt_deg,
                           math.degrees(math.acos(max(-1.0, min(1.0, uz)))))
        if fell_at is None and uz < 0.60:      # > ~53° = gone
            fell_at = t
        if i % render_every == 0:
            cam.azimuth = 145.0 + 12.0 * math.sin(i / CTRL_HZ * 0.25)
            cam.lookat[:] = (float(data.qpos[0]), float(data.qpos[1]), 0.10)
            renderer.update_scene(data, camera=cam)
            frame = renderer.render()
            vw.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    vw.release()
    renderer.close()

    verdict = "FELL" if fell_at is not None else (
        "wobbly" if max_tilt_deg > 15.0 else "stable")
    return {"name": name, "video": str(vpath), "max_tilt_deg": max_tilt_deg,
            "min_up": min_up, "fell_at_s": fell_at, "verdict": verdict}


def main() -> None:
    import inplace_demos as D
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dances", default="all",
                    help="comma list or 'all' (= STAND_STREAM_DEMOS)")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--out", default=str(_SIM / "dance_videos"))
    args = ap.parse_args()

    names = (list(D.STAND_STREAM_DEMOS) if args.dances == "all"
             else [s.strip() for s in args.dances.split(",") if s.strip()])
    out_dir = Path(args.out)
    results = []
    for n in names:
        r = run_dance(n, seconds=args.seconds, speed=args.speed,
                      out_dir=out_dir, fps=args.fps,
                      width=args.width, height=args.height)
        results.append(r)
        print(f"{r['verdict']:7s} {n:13s} max_tilt={r['max_tilt_deg']:5.1f}° "
              f"min_up={r['min_up']:.2f} "
              f"{'fell@%.1fs' % r['fell_at_s'] if r['fell_at_s'] else ''} "
              f"→ {r['video']}", flush=True)
    n_fell = sum(1 for r in results if r["verdict"] == "FELL")
    print(f"\n{len(results)} dances, {n_fell} fell — videos in {out_dir}")


if __name__ == "__main__":
    main()
