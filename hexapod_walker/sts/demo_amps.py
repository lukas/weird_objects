"""Watch sit → stand → walk → sit with live motor + bus current.

Opens a MuJoCo viewer and cycles the three behaviors.  Current draw is
shown two ways (both matter for the STS3215 build):

  * MAX/motor  — hottest single servo (stall = 2.7 A @ 12 V)
  * TOTAL bus  — sum of all 18 servos (+ logic)

Terminal bars + chassis tint track the more stressed of the two
(fraction of stall per motor, or fraction of the bus budget).

Uses the open-loop cosine posture blend + residual-zero tripod gait, so
no trained policy is required.  Defaults to the realistic chassis mass
from ``standup_current_sim.py`` so holding/stand-up currents aren't
under-reported.

Examples
--------

    ./.venv/bin/mjpython hexapod_walker/sts/demo_amps.py

    ./.venv/bin/mjpython hexapod_walker/sts/demo_amps.py --vx 0.12 --walk-s 10
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import mujoco
import mujoco.viewer

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.join(os.path.dirname(THIS_DIR), "prototype_sts3215")
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, PROTO_DIR)

import mujoco_prototype as MP  # noqa: E402
import posture as P  # noqa: E402
from sts_sensors import (  # noqa: E402
    StsSensorBank,
    SensorNoiseConfig,
    I_STALL_A,
    I_IDLE_A,
)
from standup_current_sim import REALISTIC_CHASSIS_MASS, LOGIC_12V_A  # noqa: E402

MP.TORQUE_LIMIT = 2.70
MP.CHASSIS_MASS = REALISTIC_CHASSIS_MASS

# Per-motor scale: nameplate stall.
I_MOTOR_STALL = I_STALL_A          # 2.7 A
I_MOTOR_SOFT = P.I_SOFT_A         # comfort band used in posture training

# Bus scale: 18 × stall would be 48.6 A; real fuse/PDB budget is lower.
# Bar fills against a practical trunk budget, not the theoretical max.
N_MOTORS = 18
I_BUS_BUDGET = 24.0                # A — practical gentle-operation budget
I_BUS_SOFT = P.I_TRUNK_SOFT_A      # ~12 A soft


def _heat_color(frac: float) -> tuple[float, float, float, float]:
    """Cool blue → yellow → red as stress fraction 0..1 rises."""
    u = float(np.clip(frac, 0.0, 1.0))
    if u < 0.5:
        t = u / 0.5
        r, g, b = 0.15 + 0.7 * t, 0.45 + 0.4 * t, 0.85 - 0.5 * t
    else:
        t = (u - 0.5) / 0.5
        r, g, b = 0.85 + 0.15 * t, 0.85 - 0.7 * t, 0.35 - 0.3 * t
    return (r, g, b, 1.0)


def _bar(value: float, full: float, soft: float, width: int = 22) -> str:
    filled = int(round(np.clip(value / full, 0.0, 1.0) * width))
    if value < soft:
        tone = "\033[32m"
    elif value < 0.75 * full:
        tone = "\033[33m"
    else:
        tone = "\033[31m"
    return f"{tone}{'█' * filled}{'░' * (width - filled)}\033[0m"


def _set_targets(data, joints: np.ndarray):
    for i in range(6):
        b = i * 6
        data.ctrl[b + 0] = joints[3 * i + 0]
        data.ctrl[b + 1] = joints[3 * i + 1]
        data.ctrl[b + 2] = joints[3 * i + 2]


def _joint_dof_and_acts(model):
    qpos, qvel, pos_act = [], [], []
    for i in range(6):
        for k, kind in enumerate(("yaw", "pitch", "knee")):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{kind}")
            qpos.append(model.jnt_qposadr[j])
            qvel.append(model.jnt_dofadr[j])
            pos_act.append(i * 6 + k)
    return (np.asarray(qpos, np.int32),
            np.asarray(qvel, np.int32),
            np.asarray(pos_act, np.int32))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sit-s", type=float, default=5.0, help="sit-down blend time")
    ap.add_argument("--stand-s", type=float, default=5.0, help="stand-up blend time")
    ap.add_argument("--walk-s", type=float, default=8.0, help="walk phase duration")
    ap.add_argument("--hold-s", type=float, default=1.5, help="hold / settle after each blend")
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--loops", type=int, default=0,
                    help="number of sit/stand/walk cycles (0 = forever)")
    ap.add_argument("--gain-mult", type=float, default=1.0,
                    help="PD gain multiplier (5 ≈ real STS3215 stiffness)")
    ap.add_argument("--light-chassis", action="store_true",
                    help="use as-modeled 0.55 kg chassis instead of 1.30 kg")
    ap.add_argument("--realtime", action="store_true", default=True)
    ap.add_argument("--no-realtime", action="store_false", dest="realtime")
    args = ap.parse_args()

    if args.light_chassis:
        MP.CHASSIS_MASS = 0.55
    MP.KP_YAW = 18.0 * args.gain_mult
    MP.KP_PITCH = 26.0 * args.gain_mult
    MP.KP_KNEE = 24.0 * args.gain_mult

    model, data, _ = MP.build_world(
        terrain_enabled=False, obstacle_count=0, terrain_seed=0, obstacle_seed=0,
    )
    qadr, dadr, pos_act = _joint_dof_and_acts(model)
    sensors = StsSensorBank(SensorNoiseConfig(
        pos_std_rad=0.0, vel_std_rad_s=0.0, load_std_pct=0.0,
        current_std_a=0.0, volt_std_v=0.0, temp_std_c=0.0, dropout_prob=0.0,
    ))
    sensors.reset()

    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    total_mass = float(model.body_subtreemass[chassis])
    chassis_geoms = [
        g for g in range(model.ngeom)
        if model.geom_bodyid[g] == chassis and model.geom_rgba[g, 3] > 0.5
    ]
    base_rgba = {g: model.geom_rgba[g].copy() for g in chassis_geoms}

    sit = P.sit_joints()
    stand = P.stand_joints()
    gait = MP.TripodGait(period=0.82, lift=0.025, ramp=0.4,
                         vx=args.vx, vy=0.0, omega=0.0)
    P.set_joint_qpos(model, data, sit, chassis_z=0.012)
    _set_targets(data, sit)

    phases = [
        ("HOLD SIT", "hold_sit", args.hold_s),
        ("STAND UP", "stand_up", args.stand_s),
        ("HOLD STAND", "hold_stand", args.hold_s),
        ("WALK", "walk", args.walk_s),
        ("SIT DOWN", "sit_down", args.sit_s),
    ]

    dt = float(model.opt.timestep)
    print("demo_amps: sit → stand → walk → sit   (Ctrl-C or close viewer to quit)")
    print(f"  robot mass ≈ {total_mass:.2f} kg   chassis={MP.CHASSIS_MASS:.2f} kg   "
          f"gain_mult={args.gain_mult:g}")
    print(f"  MAX/motor  soft={I_MOTOR_SOFT:.2f}A  stall={I_MOTOR_STALL:.1f}A  "
          f"(idle≈{I_IDLE_A:.2f}A)")
    print(f"  TOTAL bus  soft={I_BUS_SOFT:.0f}A  budget={I_BUS_BUDGET:.0f}A  "
          f"(18×stall={N_MOTORS * I_MOTOR_STALL:.0f}A theoretical)  "
          f"+{LOGIC_12V_A:.2f}A logic")
    print()
    print(f"  {'phase':<11}  {'MAX/motor':<32}  {'TOTAL bus':<34}  z")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.55
        viewer.cam.elevation = -22
        viewer.cam.azimuth = 140
        loop = 0
        while viewer.is_running():
            for phase_name, kind, duration in phases:
                phase_t = 0.0
                gait.reset_phase()
                # Blend from the ACTUAL current pose (not a kinematic ideal) so
                # walk → sit doesn't slam through a discontinuous jump — that
                # was the source of the spurious ~29 A spike.
                q_now = np.asarray(data.qpos[qadr], dtype=np.float64).copy()
                if kind == "stand_up":
                    start_pose, goal_pose = q_now, stand.copy()
                elif kind == "sit_down":
                    start_pose, goal_pose = q_now, sit.copy()
                elif kind == "hold_sit":
                    start_pose = goal_pose = sit.copy()
                elif kind == "hold_stand":
                    # Settle back to stance before the next sit / after stand-up.
                    start_pose, goal_pose = q_now, stand.copy()
                else:
                    start_pose = goal_pose = stand.copy()

                phase_max_motor = 0.0
                phase_max_bus = 0.0
                hottest_joint = -1

                while phase_t < duration and viewer.is_running():
                    wall0 = time.time()
                    if kind in ("stand_up", "sit_down", "hold_stand", "hold_sit"):
                        u = phase_t / max(duration, 1e-6)
                        # Holds ease to the pose then stay; blends run full u.
                        if kind.startswith("hold"):
                            u = min(1.0, phase_t / max(0.6, duration * 0.4))
                        targets = P.blend_joints(start_pose, goal_pose, u)
                    elif kind == "walk":
                        yaws, pitches, knees = gait.desired(phase_t)
                        targets = np.empty(18, dtype=np.float64)
                        for i in range(6):
                            targets[3 * i + 0] = yaws[i]
                            targets[3 * i + 1] = pitches[i]
                            targets[3 * i + 2] = knees[i]
                    else:
                        targets = goal_pose

                    _set_targets(data, targets)
                    mujoco.mj_step(model, data)
                    phase_t += dt

                    q = np.asarray(data.qpos[qadr], dtype=np.float64)
                    qd = np.asarray(data.qvel[dadr], dtype=np.float64)
                    tau = np.asarray(data.actuator_force[pos_act], dtype=np.float64)
                    fb = sensors.sample(q, qd, tau, dt=dt)

                    i_motors = fb[:, 4]                         # (18,) amps each
                    i_max = float(i_motors.max())               # hottest servo
                    j_hot = int(i_motors.argmax())
                    i_bus = float(i_motors.sum()) + LOGIC_12V_A  # total pack draw
                    z = float(data.body(chassis).xpos[2])

                    if i_max > phase_max_motor:
                        phase_max_motor = i_max
                        hottest_joint = j_hot
                    phase_max_bus = max(phase_max_bus, i_bus)

                    # Tint by whichever is closer to its limit.
                    stress = max(i_max / I_MOTOR_STALL, i_bus / I_BUS_BUDGET)
                    rgba = _heat_color(stress)
                    for g, base in base_rgba.items():
                        model.geom_rgba[g] = 0.35 * base + 0.65 * np.asarray(rgba)

                    if int(phase_t / dt) % max(1, int(0.05 / dt)) == 0:
                        leg, axis = divmod(j_hot, 3)
                        axis_name = ("yaw", "hip", "knee")[axis]
                        print(
                            f"\r{phase_name:<11}  "
                            f"{_bar(i_max, I_MOTOR_STALL, I_MOTOR_SOFT)} "
                            f"{i_max:4.2f}A L{leg}/{axis_name:<4}  "
                            f"{_bar(i_bus, I_BUS_BUDGET, I_BUS_SOFT)} "
                            f"{i_bus:5.1f}A  "
                            f"z={z:5.3f}   ",
                            end="", flush=True,
                        )

                    viewer.sync()
                    if args.realtime:
                        delay = dt - (time.time() - wall0)
                        if delay > 0:
                            time.sleep(delay)

                leg, axis = divmod(max(hottest_joint, 0), 3)
                axis_name = ("yaw", "hip", "knee")[axis]
                print(
                    f"\r{phase_name:<11}  "
                    f"phase-peak MAX/motor={phase_max_motor:4.2f}A "
                    f"(L{leg}/{axis_name})   "
                    f"TOTAL bus={phase_max_bus:5.1f}A"
                    f"{' ' * 20}"
                )

            loop += 1
            if args.loops and loop >= args.loops:
                break

    print("done.")


if __name__ == "__main__":
    main()
