"""Off-robot smoke tests for live quad pitch trim."""
from __future__ import annotations

import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
for _p in (_ROOT / "motor_setup", _HERE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from imu_calibrate import apply_imu_calib, imu_body_frame_from_roll_pitch  # noqa: E402
from inplace_demos import QuadPitchTrim  # noqa: E402
from quad_walk import make_quad_walk_pose_fn  # noqa: E402


def _imu_angles(roll_deg: float, pitch_deg: float) -> dict:
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


def _imu_pitch(pitch_deg: float) -> dict:
    return _imu_angles(0.0, pitch_deg)


def _imu_roll(roll_deg: float) -> dict:
    return _imu_angles(roll_deg, 0.0)


def test_trim_sign_and_direction() -> None:
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    for i in range(3):
        trim.update(_imu_pitch(+24.0), i * 0.25)
    assert trim.ready
    assert trim.sign_to_cmd == -1.0
    assert abs((trim.target_pitch_deg or 0.0) + 24.0) < 0.5

    trim.update(_imu_pitch(+18.0), 1.0)
    trim.update(_imu_pitch(+18.0), 1.25)
    forward = trim.event_data()
    assert forward["err_deg"] > 0.0
    assert forward["pitch_trim_deg"] < 0.0
    assert forward["body_dx_trim_mm"] < 0.0

    trim.update(_imu_pitch(+30.0), 1.75)
    trim.update(_imu_pitch(+30.0), 2.0)
    backward = trim.event_data()
    assert backward["err_deg"] < 0.0
    assert backward["pitch_trim_deg"] > forward["pitch_trim_deg"]


def test_trim_infers_roll_axis_when_imu_is_rotated() -> None:
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    for i in range(3):
        trim.update(_imu_roll(+24.0), i * 0.25)
    assert trim.ready
    assert trim.event_data()["imu_axis"] == "roll"
    assert trim.sign_to_cmd == -1.0

    trim.update(_imu_roll(+18.0), 1.0)
    trim.update(_imu_roll(+18.0), 1.25)
    forward = trim.event_data()
    assert forward["err_deg"] > 0.0
    assert forward["pitch_trim_deg"] < 0.0
    assert forward["body_dx_trim_mm"] < 0.0


def test_trim_projects_mixed_roll_pitch_axis() -> None:
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    for i in range(3):
        trim.update(_imu_angles(+17.0, +17.0), i * 0.25)
    assert trim.ready
    data = trim.event_data()
    assert data["imu_axis"] == "mix"
    assert abs(data["imu_axis_roll"] - 0.707) < 0.03
    assert abs(data["imu_axis_pitch"] - 0.707) < 0.03
    assert abs((trim.target_pitch_deg or 0.0) + math.hypot(17.0, 17.0)) < 0.5

    # Sag forward along the learned mixed axis; it should correct the same
    # way as the pure pitch/roll cases.
    fwd = 18.0 / math.sqrt(2.0)
    trim.update(_imu_angles(fwd, fwd), 1.0)
    trim.update(_imu_angles(fwd, fwd), 1.25)
    forward = trim.event_data()
    assert forward["err_deg"] > 0.0
    assert forward["pitch_trim_deg"] < 0.0
    assert forward["body_dx_trim_mm"] < 0.0


def test_saved_body_frame_handles_off_axis_imu() -> None:
    body_frame = imu_body_frame_from_roll_pitch(
        17.0, 17.0, expected_pitch_deg=-24.0, samples=10)
    assert body_frame["ok"]
    assert body_frame["pitch_axis"] == "mix"

    calib = {
        "gyro_bias_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
        "accel_bias_g": {"x": 0.0, "y": 0.0, "z": 0.0},
        "body_frame": body_frame,
    }
    leaned = apply_imu_calib(_imu_angles(17.0, 17.0), calib)
    assert leaned["body_frame_calibrated"]
    assert abs(leaned["body_pitch_deg"] + math.hypot(17.0, 17.0)) < 0.5
    assert abs(leaned["body_roll_deg"]) < 0.5

    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    trim.update(leaned, 0.0)
    assert trim.ready
    data = trim.event_data()
    assert data["body_frame_mode"]
    assert data["calibration_source"] == "imu_body_frame"

    fwd = 18.0 / math.sqrt(2.0)
    trim.update(apply_imu_calib(_imu_angles(fwd, fwd), calib), 0.5)
    trim.update(apply_imu_calib(_imu_angles(fwd, fwd), calib), 0.75)
    forward = trim.event_data()
    assert forward["err_deg"] > 0.0
    assert forward["pitch_trim_deg"] < 0.0
    assert forward["body_dx_trim_mm"] < 0.0


def test_body_frame_trim_targets_measured_rear_lean() -> None:
    body_frame = imu_body_frame_from_roll_pitch(
        13.0, 13.0, expected_pitch_deg=-24.0, samples=10)
    assert body_frame["ok"]
    assert abs(body_frame["body_pitch_target_deg"]
               + math.hypot(13.0, 13.0)) < 0.5
    calib = {
        "gyro_bias_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
        "accel_bias_g": {"x": 0.0, "y": 0.0, "z": 0.0},
        "body_frame": body_frame,
    }
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    lean = 13.0
    trim.update(apply_imu_calib(_imu_angles(lean, lean), calib), 0.0)
    data = trim.event_data()
    assert data["body_frame_mode"]
    assert abs((data["target_pitch_deg"] or 0.0)
               + math.hypot(13.0, 13.0)) < 0.5
    assert abs(data["err_deg"]) < 0.5
    assert abs(data["pitch_trim_deg"]) < 0.5


def test_recovery_engages_and_releases() -> None:
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    for i in range(3):
        trim.update(_imu_pitch(+24.0), i * 0.25)
    assert trim.ready

    # Tip forward: the rear lean decays 24 -> 12 deg (err ~ +12 deg,
    # inside the recovery band, below the 15-deg fall guard).
    t = 1.0
    for _ in range(12):
        trim.update(_imu_pitch(+12.0), t)
        t += 0.25
    data = trim.event_data()
    assert data["recovering"]
    assert trim.abort_reason is None
    # Beyond the normal +/-5 deg / 12 mm caps: full recovery authority.
    assert data["pitch_trim_deg"] < -5.5
    assert data["body_dx_trim_mm"] < -12.5

    # Lean comes back to target: recovery releases and the walk resumes.
    for _ in range(6):
        trim.update(_imu_pitch(+24.0), t)
        t += 0.25
    data = trim.event_data()
    assert not data["recovering"]
    assert data["speed_scale"] >= 0.55
    assert trim.abort_reason is None
    assert data["recover_count"] == 1


def test_recovery_backward_tip_pushes_nose_down() -> None:
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    for i in range(3):
        trim.update(_imu_pitch(+24.0), i * 0.25)
    assert trim.ready

    # Tipping backward: lean grows past the target (24 -> 36 deg).
    t = 1.0
    for _ in range(12):
        trim.update(_imu_pitch(+36.0), t)
        t += 0.25
    data = trim.event_data()
    assert data["recovering"]
    assert data["err_deg"] < 0.0
    assert data["pitch_trim_deg"] > 5.5      # push the nose back down
    assert data["body_dx_trim_mm"] > 12.5    # and the body forward
    assert trim.abort_reason is None


def test_recovery_timeout_goes_limp() -> None:
    trim = QuadPitchTrim(expected_pitch_deg=-24.0, gait="walk")
    for i in range(3):
        trim.update(_imu_pitch(+24.0), i * 0.25)
    assert trim.ready

    t = 1.0
    for _ in range(48):      # ~12 s of sustained ~12 deg tip
        trim.update(_imu_pitch(+12.0), t)
        t += 0.25
        if trim.abort_reason:
            break
    assert trim.abort_reason is not None
    assert "recovery_timeout" in trim.abort_reason


def test_pose_factory_accepts_trim() -> None:
    base = [0.0, 20.0, 80.0] * 6
    trim = {"body_dx_m": -0.005, "pitch_rad": -0.05}
    fn = make_quad_walk_pose_fn(
        base, 30.0, gait="walk", phase="walk", trim_fn=lambda: trim)
    pose = fn(1.0)
    assert len(pose) == 18
    assert all(isinstance(x, float) for x in pose)


def _main() -> int:
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
