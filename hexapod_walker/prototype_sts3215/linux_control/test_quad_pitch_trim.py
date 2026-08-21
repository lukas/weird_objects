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

from inplace_demos import QuadPitchTrim  # noqa: E402
from quad_walk import make_quad_walk_pose_fn  # noqa: E402


def _imu_angles(roll_deg: float, pitch_deg: float) -> dict:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    az = 1.0
    ay = math.tan(roll) * az
    h = math.hypot(ay, az)
    ax = -math.tan(pitch) * h
    return {"ax_g": ax, "ay_g": ay, "az_g": az}


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
