"""Probe quad gait sensitivity to small geometry/contact mismatches.

This is an approximate MuJoCo perturbation sweep for the operator question:
"what happens if the sim robot is a little wrong?"  It mutates the compiled
model in small, local ways and then runs the same quad rear+walk command path
as ``quad_play``:

- move one or more pad bodies outward to mimic a longer foot/tibia chain,
- start the chassis slightly higher than the contact solve,
- override slide friction,
- rotate the synthetic IMU pitch axis before feeding live trim.

It is not a CAD-regeneration substitute. The inertias and IK constants stay
nominal, so treat it as a fast sensitivity smoke, not a final hardware model.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

_SIM = Path(__file__).resolve().parent
_PROTO = _SIM.parents[1]
for _p in (_PROTO, _PROTO / "linux_control", _PROTO / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import mujoco  # noqa: E402
from imu_calibrate import apply_imu_calib, imu_body_frame_from_roll_pitch  # noqa: E402
from inplace_demos import QuadPitchTrim  # noqa: E402
from rl_move.sim.eval_dances import up_z  # noqa: E402
from rl_move.sim.quad_play import Player  # noqa: E402


def _imu_angles(roll_deg: float, pitch_deg: float) -> dict[str, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    az = 1.0
    ay = math.tan(roll) * az
    h = math.hypot(ay, az)
    ax = -math.tan(pitch) * h
    return {
        "ax_g": ax, "ay_g": ay, "az_g": az,
        "gx_dps": 0.0, "gy_dps": 0.0, "gz_dps": 0.0,
    }


def _rpy_deg(pl: Player) -> tuple[float, float, float]:
    rot = pl.data.xmat[pl.chassis].reshape(3, 3)
    roll = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
    pitch = math.degrees(math.atan2(
        -rot[2, 0], math.hypot(rot[2, 1], rot[2, 2])))
    yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    return roll, pitch, yaw


def _off_axis_sensor_from_body_pitch(
        body_pitch_deg: float, axis_deg: float) -> tuple[float, float]:
    theta = math.radians(float(axis_deg))
    lean = -float(body_pitch_deg)
    return lean * math.cos(theta), lean * math.sin(theta)


def _make_trim(target_pitch_deg: float, imu_axis_deg: float
               ) -> tuple[QuadPitchTrim, dict[str, Any]]:
    roll, pitch = _off_axis_sensor_from_body_pitch(
        target_pitch_deg, imu_axis_deg)
    body_frame = imu_body_frame_from_roll_pitch(
        roll, pitch, expected_pitch_deg=target_pitch_deg,
        samples=12, source="quad_geometry_probe")
    if not body_frame.get("ok"):
        raise RuntimeError(str(body_frame.get("error") or "bad body frame"))
    calib = {
        "gyro_bias_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
        "accel_bias_g": {"x": 0.0, "y": 0.0, "z": 0.0},
        "body_frame": body_frame,
    }
    return QuadPitchTrim(
        expected_pitch_deg=target_pitch_deg, gait="walk"), calib


def _mutate(pl: Player, case: dict[str, Any]) -> None:
    if case.get("mu") is not None:
        pl.model.geom_friction[:, 0] = float(case["mu"])
    for leg, dx in case.get("pad_dx", []):
        bid = mujoco.mj_name2id(
            pl.model, mujoco.mjtObj.mjOBJ_BODY, f"L{int(leg)}_pad")
        if bid < 0:
            raise RuntimeError(f"missing L{leg}_pad body")
        pl.model.body_pos[bid, 0] += float(dx)
    pl.reset()
    if case.get("z_offset"):
        pl.data.qpos[2] += float(case["z_offset"])
        mujoco.mj_forward(pl.model, pl.data)


def run_case(case: dict[str, Any], *, seconds: float,
             trim: bool, trim_target: str,
             imu_axis_deg: float, imu_noise_deg: float,
             seed: int) -> dict[str, Any]:
    pl = Player()
    _mutate(pl, case)
    pl.cmd_rear(then_walk=True)
    for _ in range(int(round(9.0 * 25))):
        pl.step()
        if pl.state == Player.WALK:
            break

    target_pitch = -24.0
    roll0, pitch0, _yaw0 = _rpy_deg(pl)
    if trim_target == "observed":
        target_pitch = pitch0

    trim_ctl = None
    calib = None
    if trim:
        trim_ctl, calib = _make_trim(target_pitch, imu_axis_deg)
        pl.gait.trim_fn = trim_ctl.pose_trim

    x0 = float(pl.data.qpos[0])
    rng = np.random.default_rng(seed)
    fell = False
    abort = ""
    min_pitch = 999.0
    max_pitch = -999.0
    max_roll = abs(roll0)
    max_tilt = 0.0
    max_trim = 0.0
    max_dx = 0.0
    min_speed = 1.0
    samples = 0
    for i in range(int(round(seconds * 25))):
        if case.get("push_n") and i == int(round(2.0 * 25)):
            pl.push_left = 0.35
            pl.data.xfrc_applied[pl.chassis][0] = float(case["push_n"])
        pl.step()
        roll, pitch, _yaw = _rpy_deg(pl)
        tilt = math.degrees(math.acos(max(-1.0, min(
            1.0, up_z(pl.data, pl.chassis)))))
        min_pitch = min(min_pitch, pitch)
        max_pitch = max(max_pitch, pitch)
        max_roll = max(max_roll, abs(roll))
        max_tilt = max(max_tilt, tilt)
        samples += 1
        if trim_ctl and calib:
            noisy_pitch = pitch + float(rng.normal(0.0, imu_noise_deg))
            sroll, spitch = _off_axis_sensor_from_body_pitch(
                noisy_pitch, imu_axis_deg)
            imu = apply_imu_calib(_imu_angles(sroll, spitch), calib)
            trim_ctl.update(imu, i / 25.0)
            data = trim_ctl.event_data()
            max_trim = max(max_trim, abs(float(data["pitch_trim_deg"])))
            max_dx = max(max_dx, abs(float(data["body_dx_trim_mm"])))
            min_speed = min(min_speed, float(data["speed_scale"]))
            if trim_ctl.abort_reason:
                abort = trim_ctl.abort_reason
                fell = True
                break
        if up_z(pl.data, pl.chassis) < 0.55:
            fell = True
            break

    return {
        "case": case["name"],
        "fell_or_guarded": fell,
        "abort": abort or None,
        "walk_state": int(pl.state),
        "x_mm": round(1000.0 * (float(pl.data.qpos[0]) - x0), 1),
        "max_tilt_deg": round(max_tilt, 1),
        "pitch_range_deg": [round(min_pitch, 1), round(max_pitch, 1)],
        "max_abs_roll_deg": round(max_roll, 1),
        "trim_target_deg": round(target_pitch, 2) if trim else None,
        "max_trim_deg": round(max_trim, 1),
        "max_trim_dx_mm": round(max_dx, 1),
        "min_speed_scale": round(min_speed, 2),
        "samples": samples,
    }


def default_cases() -> list[dict[str, Any]]:
    return [
        {"name": "baseline"},
        {"name": "imu_75deg_off"},
        {"name": "high_start_10mm", "z_offset": 0.010},
        {"name": "high_start_20mm", "z_offset": 0.020},
        {"name": "L1_foot_10mm_long", "pad_dx": [(1, 0.010)]},
        {"name": "L2_foot_10mm_long", "pad_dx": [(2, 0.010)]},
        {"name": "all_feet_10mm_long",
         "pad_dx": [(i, 0.010) for i in range(6)]},
        {"name": "mu_0p6", "mu": 0.6},
        {"name": "mu_0p35", "mu": 0.35},
        {"name": "push_4N", "push_n": 4.0},
        {"name": "L1_long_high_mu0p6",
         "pad_dx": [(1, 0.010)], "z_offset": 0.010, "mu": 0.6},
    ]


def _print_table(rows: list[dict[str, Any]]) -> None:
    print("case                  guard  x_mm  tilt  pitch_min/max  roll  "
          "target  trim dx  speed abort")
    for r in rows:
        p0, p1 = r["pitch_range_deg"]
        print(
            f"{r['case'][:21]:21s} "
            f"{str(r['fell_or_guarded']):5s} "
            f"{r['x_mm']:6.1f} "
            f"{r['max_tilt_deg']:5.1f} "
            f"{p0:6.1f}/{p1:4.1f} "
            f"{r['max_abs_roll_deg']:5.1f} "
            f"{'' if r['trim_target_deg'] is None else r['trim_target_deg']:>6} "
            f"{r['max_trim_deg']:4.1f} "
            f"{r['max_trim_dx_mm']:4.1f} "
            f"{r['min_speed_scale']:5.2f} "
            f"{r['abort'] or ''}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--no-trim", action="store_true")
    ap.add_argument("--trim-target", choices=("observed", "commanded"),
                    default="observed")
    ap.add_argument("--imu-axis-deg", type=float, default=37.0)
    ap.add_argument("--imu-noise-deg", type=float, default=0.25)
    ap.add_argument("--seed", type=int, default=20260821)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rows = []
    for case in default_cases():
        c = dict(case)
        if c["name"] == "imu_75deg_off":
            axis = 75.0
        else:
            axis = args.imu_axis_deg
        rows.append(run_case(
            c, seconds=args.seconds, trim=not args.no_trim,
            trim_target=args.trim_target, imu_axis_deg=axis,
            imu_noise_deg=args.imu_noise_deg, seed=args.seed))
    if args.json:
        print(json.dumps(rows, indent=2))
    else:
        _print_table(rows)


if __name__ == "__main__":
    main()
