"""Off-robot tests for touchdown zero hint math."""
from __future__ import annotations

import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from touchdown_zero import (  # noqa: E402
    analyze_touch_rows,
    expected_touch_angles,
    make_sweep_angles,
)


def test_expected_angles_match_measured_leg0_geometry() -> None:
    expected = expected_touch_angles(
        zero_tip_clearance_mm=88.0,
        femur_mm=90.0,
        tibia_mm=150.0,
        boot_diameter_mm=9.0,
    )
    assert math.isclose(expected["hip"].expected_touch_deg, 21.510, abs_tol=0.001)
    assert math.isclose(expected["knee"].pin_tip_deg or 0.0, 35.921, abs_tol=0.001)
    assert math.isclose(
        expected["knee"].boot_radius_deg or 0.0, 35.022, abs_tol=0.001)


def test_sweep_brackets_expected_contact() -> None:
    angles = make_sweep_angles(21.51, limit=(-80.0, 30.0))
    assert angles[0] == 0.0
    assert any(20.0 <= a <= 22.0 for a in angles)
    assert angles[-1] <= 30.0


def test_analysis_ignores_false_early_load_before_arm() -> None:
    rows = [
        {
            "cmd_deg": 0.0,
            "encoder_deg": 0.4,
            "current_delta_a": 0.0,
            "load_delta_pct": 9.0,
        },
        {
            "cmd_deg": 20.0,
            "encoder_deg": 19.95,
            "current_delta_a": 0.0,
            "load_delta_pct": 0.0,
        },
        {
            "cmd_deg": 22.0,
            "encoder_deg": 21.88,
            "current_delta_a": 0.006,
            "load_delta_pct": 4.0,
        },
        {
            "cmd_deg": 24.0,
            "encoder_deg": 23.47,
            "current_delta_a": 0.013,
            "load_delta_pct": 5.6,
        },
    ]
    analysis = analyze_touch_rows(
        "hip", rows, expected_touch_deg=21.510, arm_margin_deg=4.0)
    assert analysis.ok
    assert analysis.observed_touch_deg == 21.88
    assert analysis.contact_strength == "weak"
    assert math.isclose(
        analysis.encoder_zero_error_deg or 0.0, 0.370, abs_tol=0.001)
    assert analysis.command_compensation_deg == analysis.encoder_zero_error_deg


def test_analysis_reports_no_contact_when_only_prefloor_noise() -> None:
    rows = [
        {
            "cmd_deg": 0.0,
            "encoder_deg": 0.2,
            "current_delta_a": 0.08,
            "load_delta_pct": 12.0,
        },
        {
            "cmd_deg": 12.0,
            "encoder_deg": 11.9,
            "current_delta_a": 0.03,
            "load_delta_pct": 7.0,
        },
    ]
    analysis = analyze_touch_rows(
        "knee", rows, expected_touch_deg=35.920, arm_margin_deg=4.0)
    assert not analysis.ok
    assert analysis.status == "no_contact"
    assert analysis.encoder_zero_error_deg is None


if __name__ == "__main__":
    test_expected_angles_match_measured_leg0_geometry()
    test_sweep_brackets_expected_contact()
    test_analysis_ignores_false_early_load_before_arm()
    test_analysis_reports_no_contact_when_only_prefloor_noise()
    print("test_touchdown_zero: OK")
