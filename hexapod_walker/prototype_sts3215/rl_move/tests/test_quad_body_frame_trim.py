"""MuJoCo smoke test for calibrated off-axis quad balance trim."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parents[1]
for _p in (_ROOT, _ROOT / "motor_setup", _ROOT / "linux_control"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from imu_calibrate import apply_imu_calib, imu_body_frame_from_roll_pitch  # noqa: E402
from inplace_demos import (  # noqa: E402
    LIVE_SPEED_MIN,
    QuadPitchTrim,
)

try:
    from rl_move.sim.quad_play import Player  # noqa: E402
except ImportError as e:
    Player = None
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None


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


def _body_pitch_deg(pl: Player) -> float:
    rot = pl.data.xmat[pl.chassis].reshape(3, 3)
    return math.degrees(math.atan2(
        -rot[2, 0], math.hypot(rot[2, 1], rot[2, 2])))


def _off_axis_sensor_from_body_pitch(body_pitch_deg: float,
                                     axis_deg: float = 37.0
                                     ) -> tuple[float, float]:
    theta = math.radians(axis_deg)
    lean = -float(body_pitch_deg)
    return lean * math.cos(theta), lean * math.sin(theta)


def _calib_from_known_rear(expected_pitch_deg: float) -> dict:
    roll, pitch = _off_axis_sensor_from_body_pitch(expected_pitch_deg)
    body_frame = imu_body_frame_from_roll_pitch(
        roll, pitch, expected_pitch_deg=expected_pitch_deg,
        samples=12, source="mujoco_off_axis_rear")
    assert body_frame["ok"]
    assert body_frame["pitch_axis"] == "mix"
    return {
        "gyro_bias_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
        "accel_bias_g": {"x": 0.0, "y": 0.0, "z": 0.0},
        "body_frame": body_frame,
    }


def test_mujoco_quad_walk_uses_calibrated_off_axis_trim() -> None:
    if Player is None:
        print(f"SKIP MuJoCo quad trim smoke: {_IMPORT_ERROR}")
        return
    pl = Player()
    pl.model.geom_friction[:, 0] *= 0.6
    pl.model.actuator_forcerange[pl.pos_act] *= 0.85
    pl.cmd_rear(then_walk=True)
    # Budget the entry from the gait's own timing (run_headless does the
    # same): the 08-22 speed-cap retune stretched rear-up entry to
    # ~ENTRY_TOTAL_S / speed_eff (~13.7 s) and the old fixed 9.0 s
    # budget went stale — it timed out in ENTRY even under nominal
    # physics.
    import quad_walk as QW
    entry_s = QW.ENTRY_TOTAL_S / max(pl.speed_eff, 0.05) + 4.0
    for _ in range(int(entry_s * 25)):
        pl.step()
        if pl.state == Player.WALK:
            break
    assert pl.state == Player.WALK

    expected_pitch = -24.0
    calib = _calib_from_known_rear(expected_pitch)
    trim = QuadPitchTrim(expected_pitch_deg=expected_pitch, gait="walk")
    pl.gait.trim_fn = trim.pose_trim

    rng = np.random.default_rng(20260821)
    min_pitch_trim = 0.0
    min_dx_trim = 0.0
    min_speed_scale = 1.0
    max_err = 0.0
    for i in range(int(4.0 * 25)):
        if i == 25:
            pl.push_left = 0.35
        pl.step()
        body_pitch = _body_pitch_deg(pl)
        noisy_pitch = body_pitch + float(rng.normal(0.0, 0.35))
        sensor_roll, sensor_pitch = _off_axis_sensor_from_body_pitch(
            noisy_pitch)
        imu = apply_imu_calib(_imu_angles(sensor_roll, sensor_pitch), calib)
        assert imu["body_frame_calibrated"]
        trim.update(imu, i / 25.0)
        data = trim.event_data()
        min_pitch_trim = min(min_pitch_trim, data["pitch_trim_deg"])
        min_dx_trim = min(min_dx_trim, data["body_dx_trim_mm"])
        min_speed_scale = min(min_speed_scale, data["speed_scale"])
        max_err = max(max_err, abs(data["err_deg"]))

    assert trim.ready
    assert trim.body_frame_mode
    assert min_pitch_trim < -0.5
    assert min_dx_trim < -1.0
    assert min_speed_scale < 0.95
    assert max_err > 3.0

    for t, body_pitch in ((5.0, -3.0), (5.25, 0.0)):
        sensor_roll, sensor_pitch = _off_axis_sensor_from_body_pitch(
            body_pitch)
        trim.update(apply_imu_calib(
            _imu_angles(sensor_roll, sensor_pitch), calib), t)
    assert trim.abort_reason is not None


def test_mujoco_quad_walk_brace_holds_and_recovers_from_tip() -> None:
    """Closed loop at the streamer's sparse (~4 Hz) IMU cadence: a
    sustained ~8 deg tip (sensor fault injection on top of live walk
    physics) must engage recovery — finish the step, brace-hold frozen
    at all-stance, full trim authority — bring the lean back, release,
    and never trip the limp guard.  Without recovery this exact run
    limps on the fall guard at the first sustained tip."""
    if Player is None:
        print(f"SKIP MuJoCo quad recovery smoke: {_IMPORT_ERROR}")
        return
    from rl_move.sim.quad_play import all_stance

    pl = Player()
    pl.model.geom_friction[:, 0] *= 0.6
    pl.model.actuator_forcerange[pl.pos_act] *= 0.85
    pl.cmd_rear(then_walk=True)
    # Budget the entry from the gait's own timing (run_headless does the
    # same): the 08-22 speed-cap retune stretched rear-up entry to
    # ~ENTRY_TOTAL_S / speed_eff (~13.7 s) and the old fixed 9.0 s
    # budget went stale — it timed out in ENTRY even under nominal
    # physics.
    import quad_walk as QW
    entry_s = QW.ENTRY_TOTAL_S / max(pl.speed_eff, 0.05) + 4.0
    for _ in range(int(entry_s * 25)):
        pl.step()
        if pl.state == Player.WALK:
            break
    assert pl.state == Player.WALK

    expected_pitch = -24.0
    calib = _calib_from_known_rear(expected_pitch)
    trim = QuadPitchTrim(expected_pitch_deg=expected_pitch, gait="walk")
    pl.gait.trim_fn = trim.pose_trim

    def _fault_deg(t_s: float) -> float:
        # Looks to the trim like the nose sagged 8 deg toward level:
        # slow ramp in over 2 s, hold 3 s, ramp out by 12 s.
        if t_s < 6.0 or t_s >= 12.0:
            return 0.0
        if t_s < 8.0:
            return 8.0 * (t_s - 6.0) / 2.0
        if t_s < 11.0:
            return 8.0
        return 8.0 * (12.0 - t_s)

    rng = np.random.default_rng(20260821)
    dt = 1.0 / 25.0
    imu_every = 6           # ~0.24 s: stream_pose_fn's IMU sample pace
    releases = 0
    prev_recovering = False
    min_pitch_trim = 0.0
    frozen_s = 0.0
    for i in range(int(16.0 * 25)):
        t_s = i * dt
        if trim.ready:
            if trim.recovering:
                # Mirror stream_pose_fn's brace-hold clock: finish the
                # step at <= 1x, freeze at the next all-stance window.
                if all_stance(pl.gait, pl.tw):
                    pl.speed = 0.0
                    frozen_s += dt
                else:
                    pl.speed = 1.0
            else:
                pl.speed = max(LIVE_SPEED_MIN, min(3.0, trim.speed_scale))
        pl.step()
        if i % imu_every:
            continue
        body_pitch = _body_pitch_deg(pl)
        noisy = (body_pitch + _fault_deg(t_s)
                 + float(rng.normal(0.0, 0.2)))
        sensor_roll, sensor_pitch = _off_axis_sensor_from_body_pitch(noisy)
        imu = apply_imu_calib(_imu_angles(sensor_roll, sensor_pitch), calib)
        trim.update(imu, t_s)
        if prev_recovering and not trim.recovering:
            releases += 1
        prev_recovering = trim.recovering
        min_pitch_trim = min(min_pitch_trim, trim.pitch_trim_deg)
        assert trim.abort_reason is None, trim.abort_reason

    assert trim.recover_count >= 1
    assert releases >= 1
    # Brace-hold actually froze the gait clock on planted feet.
    assert frozen_s > 1.0
    # Recovery used more than the normal +/-5 deg trim cap.
    assert min_pitch_trim < -5.5
    # Fault gone: recovery released and the walk is still live.
    assert not trim.recovering
    assert pl.state == Player.WALK


def _main() -> int:
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
