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
                 traction_res: dict | None = None) -> tuple[dict, list[str]]:
    calls: list[str] = []
    old_geometry = sys.modules.get("geometry_plant")
    old_imu = sys.modules.get("imu_calibrate")

    def run_geometry_plant(*_args, **_kwargs) -> dict:
        calls.append("plant")
        return {"ok": True, "mode": "geometry_plant", "msg": "plant ok"}

    def run_geometry_contact_sweep(*_args, **_kwargs) -> dict:
        calls.append("sweep")
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


def test_clean_checkup_returns_zero() -> None:
    result, calls = _run_checkup(
        sweep_res={"ok": True, "mode": "geometry_sweep", "msg": "sweep ok"})
    assert result["ok"], result
    assert calls.count("safe_zero") == 2, calls
    assert "body" in calls, calls
    assert "traction" in calls, calls
    assert _phase(result["phases"], "return_zero")["ok"] is True


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


def _main() -> int:
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
