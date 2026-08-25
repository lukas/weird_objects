#!/usr/bin/env python3
"""MuJoCo check for STEP-stand -> RL walk-ready plant acquisition.

The hardware path lives in ``linux_control.walk_ready_transition`` and is
called by the RL page's Stand Up button. This script tests that exact frame
planner through the fitted servo profile and compares it with the old
all-six-leg glide baseline.

    uv run --active python -m rl_move.sim.eval_walk_ready_transition
    uv run --active python -m rl_move.sim.eval_walk_ready_transition --json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
for p in (_ROOT, _ROOT / "linux_control",
          _ROOT / "linux_control" / "urt2_setup", _ROOT.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from linux_control.walk_ready_transition import (  # noqa: E402
    TransitionFrame, build_tripod_plant_transition,
)
from rl_move.robot_state import DEG2RAD, N_JOINTS, RAD2DEG  # noqa: E402
from rl_move.safety import AXIS_LIMITS_DEG  # noqa: E402
from rl_move.joint_frame import (  # noqa: E402
    robot_abs_deg_to_sim_rad, robot_stand_degrees, sim_rad_to_robot_abs_deg,
)
from rl_move.sim.eval_dances import place_at_plant, up_z  # noqa: E402
from rl_move.sim.servo_model import (  # noqa: E402
    COUNTS_PER_DEG, LOADED_MODEL_PATH, SIM_MODEL_PATH, ServoProfile,
    SimServoParams, apply_params_to_model, build_model, joint_qpos_addrs,
    position_actuator_ids,
)

CTRL_HZ = 20.0
CONTACT_N = 0.5
FALL_UP_Z = 0.60


@dataclass(frozen=True)
class SimFrame:
    frame: TransitionFrame
    speed_counts_s: float
    acc_units: float


def clip_limits(q_rad: np.ndarray) -> np.ndarray:
    q = q_rad.copy()
    for j in range(N_JOINTS):
        lo, hi = AXIS_LIMITS_DEG[j % 3]
        q[j] = float(np.clip(q[j], lo * DEG2RAD, hi * DEG2RAD))
    return q


def load_step_final() -> list[float]:
    path = _ROOT / "linux_control" / "standup_modes.json"
    data = json.loads(path.read_text())
    frames = data["modes"]["step"]["keyframes"]
    return [float(v) for v in frames[-1]["q_deg"]]


def measured_walk_ready_plant() -> list[float]:
    """Best available plant pose for the test.

    ``robot_stand_degrees`` reads a saved plant if one exists. This checkout
    usually does not carry the robot-local plant file, so fall back to the
    last captured hardware plant from the RL state instead of the generic
    +19/+28 fallback. The fallback is deliberately explicit in the report.
    """
    q = [float(v) for v in robot_stand_degrees()]
    generic = [0.0, 19.0, 28.0] * 6
    if len(q) == N_JOINTS and max(abs(a - b) for a, b in zip(q, generic)) > 1e-6:
        return q
    return [
        0.088, 15.908, 43.242,
        0.264, 16.611, 43.945,
        -0.088, 16.260, 43.242,
        0.000, 16.699, 44.297,
        -0.088, 15.557, 43.682,
        0.088, 15.820, 43.418,
    ]


def make_tripod_frames(start: list[float], target: list[float]) -> list[SimFrame]:
    out = []
    for f in build_tripod_plant_transition(start, target):
        if f.phase == "lift":
            speed, acc = 360.0, 45.0
        elif f.phase == "support":
            speed, acc = 180.0, 25.0
        elif f.phase == "swing":
            speed, acc = 320.0, 40.0
        elif f.phase == "settle":
            speed, acc = 180.0, 20.0
        else:
            speed, acc = 260.0, 35.0
        out.append(SimFrame(f, speed, acc))
    return out


def make_glide_frames(start: list[float], target: list[float]) -> list[SimFrame]:
    worst = max(abs(a - b) for a, b in zip(start, target))
    seconds = max(1.5, worst / (260.0 / COUNTS_PER_DEG))
    frame = TransitionFrame(
        q_deg=list(target), seconds=seconds, phase="glide",
        stage=1, legs=())
    return [SimFrame(frame, 260.0, 35.0)]


def _touch_addrs(model) -> list[int]:
    import mujoco
    out = []
    for i in range(6):
        sid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, f"L{i}_foot_t")
        out.append(int(model.sensor_adr[sid]) if sid >= 0 else -1)
    return out


def _pad_bids(model) -> list[int]:
    import mujoco
    return [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_pad")
        for i in range(6)
    ]


def _contact_forces(data, touch_addr: list[int]) -> np.ndarray:
    vals = []
    for adr in touch_addr:
        vals.append(0.0 if adr < 0 else max(0.0, float(data.sensordata[adr])))
    return np.asarray(vals, dtype=float)


def _bad_contacts(mujoco, model, data, samples: set[str]) -> int:
    ground = {"floor", "terrain"}
    foot = {f"L{i}_foot" for i in range(6)}
    bad = 0
    for ci in range(data.ncon):
        c = data.contact[ci]
        names = []
        for gid in (int(c.geom1), int(c.geom2)):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            names.append(name or f"geom:{gid}")
        pair = set(names)
        if pair & ground:
            other = [n for n in names if n not in ground]
            if other and other[0] in foot:
                continue
        bad += 1
        if len(samples) < 8:
            samples.add("/".join(names))
    return bad


def run_transition(name: str, frames: list[SimFrame], *,
                   start_deg: list[float], target_deg: list[float],
                   loaded_params: bool) -> dict:
    import mujoco

    model = build_model(mesh_visuals=False, flat_terrain=True)
    params = SimServoParams.load(
        LOADED_MODEL_PATH if loaded_params else SIM_MODEL_PATH)
    apply_params_to_model(model, params)
    data = mujoco.MjData(model)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    chassis_bid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    touch_addr = _touch_addrs(model)
    pad_bids = _pad_bids(model)

    q_start = clip_limits(robot_abs_deg_to_sim_rad(start_deg))
    q_target = clip_limits(robot_abs_deg_to_sim_rad(target_deg))
    place_at_plant(mujoco, model, data, qadr, pos_act, q_start)
    profile = ServoProfile(params, q_start)

    h = float(model.opt.timestep)
    substeps = max(1, int(round(1.0 / CTRL_HZ / h)))
    db = profile.deadband_rad
    min_up = 1.0
    max_tilt_deg = 0.0
    fell_at_s = None
    bad_contact_count = 0
    bad_contact_samples: set[str] = set()
    max_support_slip_m = 0.0
    max_moving_loaded_slip_m = 0.0
    max_support_force_n = 0.0
    max_moving_force_n = 0.0
    foot_air_ticks = [0] * 6
    frame_reports = []
    t = 0.0

    def advance_tick(moving: tuple[int, ...],
                     support_anchor: list[np.ndarray | None],
                     moving_anchor: list[np.ndarray | None]) -> None:
        nonlocal t, min_up, max_tilt_deg, fell_at_s
        nonlocal bad_contact_count, max_support_slip_m
        nonlocal max_moving_loaded_slip_m, max_support_force_n
        nonlocal max_moving_force_n
        moving_set = set(moving)
        for _ in range(substeps):
            target = profile.tick(h)
            q = data.qpos[qadr]
            err = target - q
            eff = q + np.sign(err) * np.maximum(np.abs(err) - db, 0.0)
            data.ctrl[pos_act] = eff
            mujoco.mj_step(model, data)
            t += h

            uz = up_z(data, chassis_bid)
            min_up = min(min_up, uz)
            max_tilt_deg = max(
                max_tilt_deg,
                math.degrees(math.acos(max(-1.0, min(1.0, uz)))))
            if fell_at_s is None and uz < FALL_UP_Z:
                fell_at_s = t

            bad_contact_count += _bad_contacts(
                mujoco, model, data, bad_contact_samples)
            forces = _contact_forces(data, touch_addr)
            for leg in range(6):
                xy = np.asarray(data.xpos[pad_bids[leg], :2], dtype=float)
                on = forces[leg] > CONTACT_N
                if not on:
                    foot_air_ticks[leg] += 1
                    continue
                if leg in moving_set:
                    max_moving_force_n = max(max_moving_force_n, forces[leg])
                    if moving_anchor[leg] is None:
                        moving_anchor[leg] = xy.copy()
                    max_moving_loaded_slip_m = max(
                        max_moving_loaded_slip_m,
                        float(np.linalg.norm(xy - moving_anchor[leg])))
                else:
                    max_support_force_n = max(max_support_force_n, forces[leg])
                    if support_anchor[leg] is None:
                        support_anchor[leg] = xy.copy()
                    max_support_slip_m = max(
                        max_support_slip_m,
                        float(np.linalg.norm(xy - support_anchor[leg])))

    # Let the STEP final pose settle before the transition begins.
    for _ in range(int(1.0 * CTRL_HZ)):
        profile.command(q_start, acc_units=35.0)
        advance_tick((), [None] * 6, [None] * 6)

    for i, sf in enumerate(frames, 1):
        q_goal = clip_limits(robot_abs_deg_to_sim_rad(sf.frame.q_deg))
        speed_deg_s = sf.speed_counts_s / COUNTS_PER_DEG
        n_ticks = max(1, int(round(sf.frame.seconds * CTRL_HZ)))
        support_anchor = [None] * 6
        moving_anchor = [None] * 6
        slip0 = max_support_slip_m
        move_slip0 = max_moving_loaded_slip_m
        air0 = list(foot_air_ticks)
        for _ in range(n_ticks):
            profile.command(
                q_goal, speed_deg_s=speed_deg_s, acc_units=sf.acc_units)
            advance_tick(sf.frame.legs, support_anchor, moving_anchor)
        frame_reports.append({
            "idx": i,
            "phase": sf.frame.phase,
            "stage": sf.frame.stage,
            "legs": list(sf.frame.legs),
            "support_slip_mm": round((max_support_slip_m - slip0) * 1000, 2),
            "moving_loaded_slip_mm": round(
                (max_moving_loaded_slip_m - move_slip0) * 1000, 2),
            "air_ticks": [foot_air_ticks[j] - air0[j] for j in range(6)],
        })

    for _ in range(int(0.8 * CTRL_HZ)):
        profile.command(q_target, speed_deg_s=180.0 / COUNTS_PER_DEG,
                        acc_units=20.0)
        advance_tick((), [None] * 6, [None] * 6)

    q_final_robot = sim_rad_to_robot_abs_deg(data.qpos[qadr])
    final_err = max(abs(a - b) for a, b in zip(q_final_robot, target_deg))
    verdict = "PASS"
    issues = []
    if fell_at_s is not None:
        verdict = "FAIL"
        issues.append(f"fell at {fell_at_s:.2f}s")
    if max_tilt_deg > 12.0:
        verdict = "WARN" if verdict == "PASS" else verdict
        issues.append(f"tilt {max_tilt_deg:.1f} deg")
    if max_support_slip_m > 0.025:
        verdict = "WARN" if verdict == "PASS" else verdict
        issues.append(f"support slip {max_support_slip_m * 1000:.1f} mm")
    if bad_contact_count:
        verdict = "WARN" if verdict == "PASS" else verdict
        issues.append(f"{bad_contact_count} non-foot contacts")

    return {
        "name": name,
        "verdict": verdict,
        "issues": issues,
        "frames": len(frames),
        "duration_s": round(t, 3),
        "max_tilt_deg": round(max_tilt_deg, 2),
        "min_up": round(min_up, 4),
        "fell_at_s": None if fell_at_s is None else round(fell_at_s, 3),
        "max_support_slip_mm": round(max_support_slip_m * 1000, 2),
        "max_moving_loaded_slip_mm": round(max_moving_loaded_slip_m * 1000, 2),
        "max_support_force_n": round(max_support_force_n, 2),
        "max_moving_force_n": round(max_moving_force_n, 2),
        "bad_contacts": int(bad_contact_count),
        "bad_contact_samples": sorted(bad_contact_samples),
        "final_max_joint_err_deg": round(float(final_err), 2),
        "final_body_z_m": round(float(data.qpos[2]), 4),
        "final_xy_m": [round(float(data.qpos[0]), 4),
                       round(float(data.qpos[1]), 4)],
        "frame_reports": frame_reports,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true",
                    help="print full JSON instead of summary lines")
    ap.add_argument("--air-params", action="store_true",
                    help="use sim_model.json instead of loaded actuator fit")
    ap.add_argument("--baseline-only", action="store_true",
                    help="run only the old all-six glide baseline")
    ap.add_argument("--tripod-only", action="store_true",
                    help="run only the new tripod-step transition")
    args = ap.parse_args()

    start = load_step_final()
    target = measured_walk_ready_plant()
    jobs = []
    if not args.tripod_only:
        jobs.append(("glide", make_glide_frames(start, target)))
    if not args.baseline_only:
        jobs.append(("tripod", make_tripod_frames(start, target)))
    results = [
        run_transition(n, fs, start_deg=start, target_deg=target,
                       loaded_params=not args.air_params)
        for n, fs in jobs
    ]

    payload = {
        "ok": all(r["verdict"] != "FAIL" for r in results),
        "start_step_final_deg": [round(x, 3) for x in start],
        "target_walk_ready_deg": [round(x, 3) for x in target],
        "params": "air" if args.air_params else "loaded",
        "results": results,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
        return
    for r in results:
        detail = "; ".join(r["issues"]) if r["issues"] else "clean"
        print(
            f"{r['verdict']:4s} {r['name']:6s} frames={r['frames']:2d} "
            f"tilt={r['max_tilt_deg']:4.1f}deg "
            f"support_slip={r['max_support_slip_mm']:5.1f}mm "
            f"moving_loaded_slip={r['max_moving_loaded_slip_mm']:5.1f}mm "
            f"final_err={r['final_max_joint_err_deg']:4.1f}deg "
            f"- {detail}",
            flush=True,
        )


if __name__ == "__main__":
    main()
