"""Off-robot tests for the all-in-one calibration checkup coordinator.

Run locally:  python3 linux_control/test_calibration_checkup.py
No hardware: tests monkeypatch the motion phases and assert sequencing.
"""
from __future__ import annotations

import sys
import tempfile
import types
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from bench_api import BenchAPI  # noqa: E402


def _module(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


def _phase(phases: list[dict], name: str) -> dict:
    for phase in phases:
        if phase.get("name") == name:
            return phase
    raise AssertionError(f"missing phase {name}: {phases}")


def _run_checkup(*, sweep_res: dict,
                 body_res: dict | None = None,
                 traction_res: dict | None = None,
                 plant_exc: Exception | None = None,
                 sweep_exc: Exception | None = None) -> tuple[dict, list[str]]:
    calls: list[str] = []
    old_geometry = sys.modules.get("geometry_plant")
    old_imu = sys.modules.get("imu_calibrate")

    def run_geometry_plant(*_args, **_kwargs) -> dict:
        calls.append("plant")
        if plant_exc is not None:
            raise plant_exc
        return {"ok": True, "mode": "geometry_plant", "msg": "plant ok"}

    def run_geometry_contact_sweep(*_args, **_kwargs) -> dict:
        calls.append("sweep")
        if sweep_exc is not None:
            raise sweep_exc
        return sweep_res

    def run_imu_calibrate(*_args, **_kwargs) -> dict:
        calls.append("imu")
        return {"ok": True, "mode": "imu", "msg": "imu ok"}

    sys.modules["geometry_plant"] = _module(
        "geometry_plant",
        run_geometry_plant=run_geometry_plant,
        run_geometry_contact_sweep=run_geometry_contact_sweep)
    sys.modules["imu_calibrate"] = _module(
        "imu_calibrate", run_imu_calibrate=run_imu_calibrate)

    try:
        api = BenchAPI(object())

        def safe_zero(**_kwargs) -> dict:
            calls.append("safe_zero")
            return {"ok": True, "stages_done": 1}

        def body_frame(*_args, **_kwargs) -> dict:
            calls.append("body")
            return body_res or {
                "ok": True,
                "mode": "imu_body_frame",
                "msg": "body ok",
            }

        def traction(*_args, **_kwargs) -> dict:
            calls.append("traction")
            return traction_res or {
                "ok": True,
                "mode": "traction_probe",
                "msg": "traction ok",
            }

        def proprio(*_args, **_kwargs) -> dict:
            calls.append("proprio")
            return {
                "ok": True,
                "mode": "proprioception_check",
                "msg": "zero max err 1.0deg",
                "max_abs_error_deg": 1.0,
            }

        def camera(*_args, **_kwargs) -> dict:
            calls.append("camera")
            return {
                "ok": True,
                "skipped": True,
                "non_blocking": True,
                "mode": "camera_witness",
                "msg": "camera witness not configured",
            }

        def report(**_kwargs) -> dict:
            calls.append("report")
            return {
                "path": "/tmp/calibration_report_test.json",
                "log_name": "calibration_report_test.json",
                "geometry": {},
                "imu": {},
                "actuators": {},
            }

        api._safe_zero_sync = safe_zero
        api._calibrate_quad_body_frame = body_frame
        api._run_leg_slip_probe = traction
        api._proprioception_check = proprio
        api._camera_witness_check = camera
        api._save_calibration_report = report

        return (
            api._run_calibration_checkup(
                None,
                clearance_mm=25.0,
                abort_check=lambda: False,
                on_progress=lambda _p: None),
            calls,
        )
    finally:
        if old_geometry is None:
            sys.modules.pop("geometry_plant", None)
        else:
            sys.modules["geometry_plant"] = old_geometry
        if old_imu is None:
            sys.modules.pop("imu_calibrate", None)
        else:
            sys.modules["imu_calibrate"] = old_imu


def test_partial_sweep_stops_dynamic_phases() -> None:
    result, calls = _run_checkup(sweep_res={
        "ok": False,
        "mode": "geometry_sweep",
        "msg": "dimension sweep partial",
    })
    phases = result["phases"]
    assert calls.count("safe_zero") == 1, calls
    assert "body" not in calls, calls
    assert "traction" not in calls, calls
    assert not result["ok"], result
    assert "geometry_sweep: dimension sweep partial" == result["error"]
    assert _phase(phases, "imu_body_frame")["skipped"] is True
    assert "dimension sweep" in _phase(phases, "imu_body_frame")["summary"]
    assert _phase(phases, "traction_probe")["skipped"] is True
    assert _phase(phases, "return_zero")["skipped"] is True
    assert _phase(phases, "proprioception_check")["skipped"] is True
    assert "proprio" not in calls, calls


def test_sweep_command_failure_is_reported_as_phase_error() -> None:
    result, calls = _run_checkup(
        sweep_res={"ok": True, "mode": "geometry_sweep"},
        sweep_exc=RuntimeError("SyncWrite failed: fallback='ERR'"))
    phases = result["phases"]
    assert "sweep" in calls, calls
    assert "body" not in calls, calls
    sweep = _phase(phases, "geometry_sweep")
    assert sweep["aborted"] is True
    assert "SyncWrite failed" in sweep["summary"]
    assert result["error"].startswith("geometry_sweep: dimension sweep command failed")
    assert _phase(phases, "imu_body_frame")["skipped"] is True
    assert _phase(phases, "proprioception_check")["skipped"] is True


def test_plant_command_failure_is_reported_as_phase_error() -> None:
    result, calls = _run_checkup(
        sweep_res={"ok": True, "mode": "geometry_sweep"},
        plant_exc=RuntimeError("SyncWrite failed: fallback='ERR'"))
    phases = result["phases"]
    assert "plant" in calls, calls
    assert "sweep" not in calls, calls
    plant = _phase(phases, "geometry_plant")
    assert plant["aborted"] is True
    assert "SyncWrite failed" in plant["summary"]
    assert result["error"].startswith("geometry_plant: ground contact command failed")
    assert _phase(phases, "geometry_sweep")["skipped"] is True
    assert _phase(phases, "proprioception_check")["skipped"] is True


def test_body_frame_failure_skips_return_zero() -> None:
    result, calls = _run_checkup(
        sweep_res={"ok": True, "mode": "geometry_sweep", "msg": "sweep ok"},
        body_res={
            "ok": False,
            "mode": "imu_body_frame",
            "error": "rear-lean reading too small",
        })
    phases = result["phases"]
    assert calls.count("safe_zero") == 1, calls
    assert "body" in calls, calls
    assert "traction" not in calls, calls
    assert result["error"] == "imu_body_frame: rear-lean reading too small"
    assert _phase(phases, "traction_probe")["skipped"] is True
    assert _phase(phases, "return_zero")["skipped"] is True
    assert _phase(phases, "proprioception_check")["skipped"] is True
    assert "proprio" not in calls, calls


def test_clean_checkup_returns_zero() -> None:
    result, calls = _run_checkup(
        sweep_res={"ok": True, "mode": "geometry_sweep", "msg": "sweep ok"})
    assert result["ok"], result
    assert calls.count("safe_zero") == 2, calls
    assert "body" in calls, calls
    assert "traction" in calls, calls
    assert "proprio" in calls, calls
    assert "camera" in calls, calls
    assert _phase(result["phases"], "return_zero")["ok"] is True
    assert _phase(result["phases"], "proprioception_check")["ok"] is True
    assert _phase(result["phases"], "camera_witness")["skipped"] is True


def test_recoverable_traction_guard_still_returns_zero() -> None:
    result, calls = _run_checkup(
        sweep_res={"ok": True, "mode": "geometry_sweep", "msg": "sweep ok"},
        traction_res={
            "ok": False,
            "mode": "traction_probe",
            "recoverable": True,
            "guard_stop": True,
            "error": "L0 hover stopped: tilt delta > 8 deg",
        })
    assert not result["ok"], result
    assert result["error"] == (
        "traction_probe: L0 hover stopped: tilt delta > 8 deg")
    assert calls.count("safe_zero") == 2, calls
    assert "proprio" in calls, calls
    traction = _phase(result["phases"], "traction_probe")
    assert traction["aborted"] is False
    assert traction["recoverable"] is True
    assert _phase(result["phases"], "return_zero")["ok"] is True
    assert _phase(result["phases"], "proprioception_check")["ok"] is True


class FakeFeedbackBus:
    def __init__(self, *, off_joint: int | None = None,
                 off_deg: float = 0.0, current_a: float = 0.15):
        self.off_joint = off_joint
        self.off_deg = off_deg
        self.current_a = current_a

    def read_all_feedback(self):
        rows = {}
        for joint in range(18):
            rows[joint] = {
                "deg": self.off_deg if joint == self.off_joint else 0.0,
                "current_a": self.current_a,
                "speed_deg_s": 0.0,
                "load_pct": 1.0,
                "volt": 12.1,
                "temp_c": 32,
            }
        return rows


def test_proprioception_check_scores_expected_pose() -> None:
    api = BenchAPI(object())
    res = api._proprioception_check(
        FakeFeedbackBus(off_joint=5, off_deg=3.25),
        expected_pose=[0.0] * 18,
        expected_name="zero")
    assert res["ok"], res
    assert res["live_joints"] == 18
    assert res["max_abs_error_deg"] == 3.25
    assert res["worst_joints"][0]["joint"] == 5


def test_proprioception_check_flags_large_pose_error() -> None:
    api = BenchAPI(object())
    res = api._proprioception_check(
        FakeFeedbackBus(off_joint=8, off_deg=12.0),
        expected_pose=[0.0] * 18,
        expected_name="zero")
    assert not res["ok"], res
    assert "max err 12.0deg" in res["error"]


def test_manual_geometry_is_reported_separately_from_fk() -> None:
    api = BenchAPI(object())
    with tempfile.TemporaryDirectory() as td:
        api._manual_geometry_path = lambda: Path(td) / "geometry_manual.json"
        saved = api.set_manual_geometry(
            hip_pitch_height_mm=95.0,
            hip_center_radius_mm=114.0,
            femur_mm=88.0,
            tibia_mm=152.0,
        )
        assert saved["ok"], saved
        assert saved["hip_pitch_height_mm"] == 95.0

        api.plant_state = lambda: {
            "ok": True,
            "learned": True,
            "hip_deg": 18.0,
            "knee_deg": 33.0,
            "pose": [0.0, 18.0, 33.0] * 6,
        }
        sweep_samples = [
            {"accepted": True, "contact_detected": True, "leg": i % 6,
             "hip_deg": hip, "knee_deg": knee,
             "reason": "floor contact signal"}
            for i, (hip, knee) in enumerate((
                (18.0, 28.0), (14.0, 32.0), (22.0, 24.0),
                (17.0, 30.0), (21.0, 26.0), (15.0, 31.0),
            ))
        ]
        geom = api._geometry_report(
            geometry_sweep={"ok": True, "samples": sweep_samples},
            use_latest_sweep=False)
        summary = geom["summary"]
        assert geom["schema_version"] >= 4
        assert summary["manual_hip_pitch_height_mm"] == 95.0
        assert summary["manual_hip_center_radius_mm"] == 114.0
        assert summary["manual_center_minus_nominal_mm"] == 1.5
        assert summary["manual_relative_minus_manual_height_mm"] > 0.0
        hyp = summary["manual_zero_hypotheses"]
        assert hyp["ok"], hyp
        assert hyp["sample_count"] == len(sweep_samples)
        assert hyp["models"]["serial_best_pair"]["rms_error_mm"] < (
            hyp["models"]["serial_no_offset"]["rms_error_mm"])
        assert geom["mujoco_hint"]["neutral_foot_z_m"] == -0.095
        assert geom["mujoco_hint"]["per_leg_servo_height_m"]["0"] == 0.095
        assert geom["mujoco_hint"]["per_leg_servo_height_source"] == (
            "manual_operator_measurement")


def test_contact_sweep_mismatch_is_rejected_as_dimension_source() -> None:
    api = BenchAPI(object())
    with tempfile.TemporaryDirectory() as td:
        api._manual_geometry_path = lambda: Path(td) / "geometry_manual.json"
        assert api.set_manual_geometry(
            hip_pitch_height_mm=95.0,
            hip_center_radius_mm=114.0,
            femur_mm=88.0,
            tibia_mm=152.0,
        )["ok"]
        api.plant_state = lambda: {
            "ok": True,
            "learned": True,
            "hip_deg": 18.6,
            "knee_deg": 28.1,
            "pose": [0.0, 18.6, 28.1] * 6,
        }
        sweep_samples = []
        for leg in range(5):
            for hip, knee in (
                    (18.6, 28.1),
                    (22.6, 20.4),
                    (14.6, 36.2),
                    (27.6, 11.2)):
                sweep_samples.append({
                    "accepted": True,
                    "contact_detected": True,
                    "leg": leg,
                    "hip_deg": hip,
                    "knee_deg": knee,
                    "base_z_mm": -122.0,
                    "nominal_z_mm": -122.0,
                    "reason": "floor contact signal",
                })
        geom = api._geometry_report(
            geometry_sweep={"ok": True, "samples": sweep_samples},
            use_latest_sweep=False)
        assert geom["contact_sweep"]["status"] == "manual_geometry_mismatch"
        assert geom["contact_sweep"]["ok"] is False
        assert geom["summary"]["manual_height_mismatch"] is True
        assert geom["effective_fit"]["source"] == "plant_only"
        assert geom["effective_fit"]["rejected_contact_sweep_status"] == (
            "manual_geometry_mismatch")
        assert geom["mujoco_hint"]["per_leg_servo_height_m"]["0"] == 0.095


def _main() -> int:
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
