#!/usr/bin/env python3
"""Render the ROCKING CHAIR demo (ground-start) in the MuJoCo twin.

eval_dances.py anchors the standing dances to the plant; the rock is a
sitting show, so this starts from the zero pose (legs straight out,
belly on the floor), streams ``make_pose_rock`` through the fitted
``ServoProfile`` exactly like the robot's 20 Hz web stream, and writes
an mp4 plus a rocking verdict: peak-to-peak chassis pitch during the
steady phase, roll excursion, and whether it tipped over.

    python eval_rock.py                     # 30 s at 1x
    python eval_rock.py --seconds 40 --speed 1.25

Plain python (cv2 render path), NOT mjpython.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

# eval_dances bootstraps sys.path for rl_move / motor_setup imports.
from eval_dances import (  # noqa: E402
    CTRL_HZ, STREAM_ACC_UNITS, clip_limits, place_at_plant, up_z,
)

from rl_move.joint_frame import robot_abs_deg_to_sim_rad  # noqa: E402
from rl_move.robot_state import N_JOINTS  # noqa: E402
from rl_move.sim.servo_model import (  # noqa: E402
    SIM_MODEL_PATH, ServoProfile, SimServoParams, apply_params_to_model,
    build_model, joint_qpos_addrs, position_actuator_ids,
)

_SIM = Path(__file__).resolve().parent


def pitch_roll_deg(data, chassis_bid: int) -> tuple[float, float]:
    """Chassis pitch (nose-down positive) and roll, degrees."""
    r = data.xmat[chassis_bid].reshape(3, 3)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -r[2, 0]))))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, r[2, 1]))))
    return pitch, roll


def run_rock(*, seconds: float, speed: float, out_dir: Path,
             fps: int, width: int, height: int) -> dict:
    import cv2
    import mujoco
    import inplace_demos as D

    pose_fn = D.make_pose_rock(seconds * speed)

    model = build_model(mesh_visuals=False, flat_terrain=True)
    params = SimServoParams.load(SIM_MODEL_PATH)
    apply_params_to_model(model, params)
    data = mujoco.MjData(model)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    chassis_bid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    q_zero = np.zeros(N_JOINTS)
    place_at_plant(mujoco, model, data, qadr, pos_act, q_zero)
    profile = ServoProfile(params, q_zero)

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

    # settle onto the belly before the show
    for _ in range(int(1.5 * CTRL_HZ)):
        profile.command(q_zero, acc_units=STREAM_ACC_UNITS)
        advance_tick()

    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 0.72
    cam.elevation = -14.0
    cam.azimuth = 100.0          # near side-on: the pitch rock reads best

    out_dir.mkdir(parents=True, exist_ok=True)
    vpath = out_dir / "rock.mp4"
    vw = cv2.VideoWriter(str(vpath), cv2.VideoWriter_fourcc(*"mp4v"),
                         fps, (width, height))

    n_ticks = int(seconds * CTRL_HZ)
    render_every = max(1, int(round(CTRL_HZ / fps)))
    min_up = 1.0
    fell_at = None
    pitches: list[tuple[float, float]] = []
    max_roll = 0.0
    for i in range(n_ticks):
        t = i / CTRL_HZ * speed          # demo time
        q_goal = clip_limits(robot_abs_deg_to_sim_rad(pose_fn(t)))
        profile.command(q_goal, acc_units=STREAM_ACC_UNITS)
        advance_tick()
        uz = up_z(data, chassis_bid)
        min_up = min(min_up, uz)
        if fell_at is None and uz < 0.60:      # > ~53 deg = gone
            fell_at = t
        pitch, roll = pitch_roll_deg(data, chassis_bid)
        pitches.append((t, pitch))
        max_roll = max(max_roll, abs(roll))
        if i % render_every == 0:
            cam.lookat[:] = (float(data.qpos[0]), float(data.qpos[1]), 0.08)
            renderer.update_scene(data, camera=cam)
            frame = renderer.render()
            cv2.putText(frame, f"t={t:5.1f}s pitch={pitch:+5.1f}",
                        (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
            vw.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    vw.release()
    renderer.close()

    # Steady rocking window: after spread + ramp, before the fade.
    total = seconds * speed
    t_lo = D.ROCK_SPREAD_S + D.ROCK_RAMP_CYCLES / D.ROCK_HZ
    t_hi = total - D.ROCK_FADE_S
    steady = [p for t, p in pitches if t_lo <= t <= t_hi]
    pp = (max(steady) - min(steady)) if steady else 0.0
    crossings = sum(1 for a, b in zip(steady, steady[1:])
                    if (a < 0.0) != (b < 0.0))
    verdict = ("FELL" if fell_at is not None
               else "rocking" if pp >= 6.0 and crossings >= 4
               else "flat")
    return {"video": str(vpath), "pitch_pp_deg": pp,
            "crossings": crossings, "max_roll_deg": max_roll,
            "min_up": min_up, "fell_at_s": fell_at, "verdict": verdict}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--out", default=str(_SIM / "dance_videos"))
    args = ap.parse_args()

    r = run_rock(seconds=args.seconds, speed=args.speed,
                 out_dir=Path(args.out), fps=args.fps,
                 width=args.width, height=args.height)
    print(f"{r['verdict']:8s} pitch_pp={r['pitch_pp_deg']:5.1f} deg  "
          f"crossings={r['crossings']}  max_roll={r['max_roll_deg']:4.1f} deg  "
          f"min_up={r['min_up']:.2f} "
          f"{'fell@%.1fs' % r['fell_at_s'] if r['fell_at_s'] else ''} "
          f"-> {r['video']}")


if __name__ == "__main__":
    main()
