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
    fit_zero_tip_height_from_contacts,
    make_refine_sweep_angles,
    make_sweep_angles,
    merge_repeat_analyses,
)


def test_expected_angles_match_measured_leg0_geometry() -> None:
    expected = expected_touch_angles(
        zero_tip_clearance_mm=88.0,
        femur_mm=90.0,
        tibia_mm=150.0,
        boot_diameter_mm=9.0,
    )
    assert math.isclose(expected["hip"].expected_touch_deg, 20.767, abs_tol=0.001)
    assert math.isclose(expected["hip"].pin_tip_deg or 0.0, 21.510, abs_tol=0.001)
    assert math.isclose(
        expected["hip"].boot_radius_deg or 0.0, 20.767, abs_tol=0.001)
    assert expected["hip"].model == "straight_leg_spherical_boot_tip"
    assert math.isclose(
        expected["knee"].expected_touch_deg, 35.022, abs_tol=0.001)
    assert math.isclose(
        expected["knee"].boot_radius_deg or 0.0, 35.022, abs_tol=0.001)
    assert math.isclose(expected["knee"].pin_tip_deg or 0.0, 35.921, abs_tol=0.001)
    assert expected["knee"].model == "spherical_boot_tip"


def test_expected_angles_use_pin_tip_without_boot() -> None:
    expected = expected_touch_angles(
        zero_tip_clearance_mm=88.0,
        femur_mm=90.0,
        tibia_mm=150.0,
        boot_diameter_mm=None,
    )
    assert math.isclose(
        expected["hip"].expected_touch_deg, 21.510, abs_tol=0.001)
    assert expected["hip"].boot_radius_deg is None
    assert expected["hip"].model == "straight_leg_pin_tip"
    assert math.isclose(
        expected["knee"].expected_touch_deg, 35.921, abs_tol=0.001)
    assert expected["knee"].boot_radius_deg is None
    assert expected["knee"].model == "pin_tip"


def test_sweep_brackets_expected_contact() -> None:
    angles = make_sweep_angles(21.51, limit=(-80.0, 30.0))
    assert angles[0] == 0.0
    assert any(20.0 <= a <= 22.0 for a in angles)
    assert angles[-1] <= 30.0


def test_refine_sweep_stays_near_contact_command() -> None:
    angles = make_refine_sweep_angles(
        43.02, backoff_deg=12.0, limit=(-20.0, 150.0))
    assert angles[0] == 31.02
    assert 35.22 in angles
    assert 43.02 in angles
    assert angles[-1] == 45.52
    assert all(31.0 <= a <= 46.0 for a in angles)


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
    assert analysis.contact_strength == "edge"
    assert math.isclose(
        analysis.encoder_zero_error_deg or 0.0, 0.370, abs_tol=0.001)
    assert analysis.command_compensation_deg == analysis.encoder_zero_error_deg


def test_analysis_uses_small_rise_before_firm_hip_push() -> None:
    rows = [
        {
            "cmd_deg": 22.51,
            "encoder_deg": 18.46,
            "current_delta_a": 0.0,
            "load_delta_pct": 0.0,
        },
        {
            "cmd_deg": 23.51,
            "encoder_deg": 20.74,
            "current_delta_a": 0.0,
            "load_delta_pct": 0.0,
        },
        {
            "cmd_deg": 24.51,
            "encoder_deg": 23.38,
            "current_delta_a": 0.0,
            "load_delta_pct": 2.0,
        },
        {
            "cmd_deg": 29.51,
            "encoder_deg": 25.49,
            "current_delta_a": 0.019,
            "load_delta_pct": 6.8,
        },
    ]
    analysis = analyze_touch_rows(
        "hip", rows, expected_touch_deg=21.510, arm_margin_deg=4.0)
    assert analysis.ok
    assert analysis.observed_touch_deg == 23.38
    assert analysis.contact_strength == "edge"
    assert analysis.note == "first rising-edge contact hint"


def test_analysis_uses_small_rise_before_larger_knee_load() -> None:
    rows = [
        {
            "cmd_deg": 43.92,
            "encoder_deg": 32.26,
            "current_delta_a": 0.0,
            "load_delta_pct": 2.4,
        },
        {
            "cmd_deg": 43.92,
            "encoder_deg": 35.07,
            "current_delta_a": 0.006,
            "load_delta_pct": 1.6,
        },
        {
            "cmd_deg": 43.92,
            "encoder_deg": 37.62,
            "current_delta_a": 0.006,
            "load_delta_pct": 3.6,
        },
    ]
    analysis = analyze_touch_rows(
        "knee", rows, expected_touch_deg=35.921, arm_margin_deg=4.0)
    assert analysis.ok
    assert analysis.observed_touch_deg == 35.07
    assert analysis.contact_strength == "edge"


def test_analysis_can_use_more_sensitive_local_refine_thresholds() -> None:
    rows = [
        {
            "cmd_deg": 35.22,
            "encoder_deg": 33.22,
            "current_delta_a": 0.006,
            "load_delta_pct": 2.8,
        },
        {
            "cmd_deg": 39.02,
            "encoder_deg": 35.95,
            "current_delta_a": 0.0,
            "load_delta_pct": 2.0,
        },
        {
            "cmd_deg": 40.52,
            "encoder_deg": 38.67,
            "current_delta_a": 0.006,
            "load_delta_pct": 2.8,
        },
    ]
    broad = analyze_touch_rows(
        "knee", rows, expected_touch_deg=35.021, arm_margin_deg=4.0)
    local = analyze_touch_rows(
        "knee", rows,
        expected_touch_deg=35.021,
        arm_margin_deg=4.0,
        weak_current_delta_a=0.008,
        weak_load_delta_pct=1.9,
    )
    assert broad.ok
    assert broad.observed_touch_deg == 38.67
    assert local.ok
    assert local.observed_touch_deg == 35.95


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
    assert analysis.status == "encoder_not_reached"
    assert analysis.encoder_zero_error_deg is None
    assert not analysis.reached_search_window


def test_analysis_reports_no_contact_after_encoder_reaches_window() -> None:
    rows = [
        {
            "cmd_deg": 32.0,
            "encoder_deg": 32.1,
            "current_delta_a": 0.0,
            "load_delta_pct": 0.0,
        },
        {
            "cmd_deg": 39.0,
            "encoder_deg": 39.0,
            "current_delta_a": 0.0,
            "load_delta_pct": 0.4,
        },
    ]
    analysis = analyze_touch_rows(
        "knee", rows, expected_touch_deg=35.920, arm_margin_deg=4.0)
    assert not analysis.ok
    assert analysis.status == "no_contact"
    assert analysis.reached_search_window
    assert analysis.max_encoder_deg == 39.0


def test_analysis_ignores_weak_knee_bump_before_expected_angle() -> None:
    rows = [
        {
            "cmd_deg": 43.92,
            "encoder_deg": 32.52,
            "current_delta_a": 0.0,
            "load_delta_pct": 2.0,
        },
        {
            "cmd_deg": 43.92,
            "encoder_deg": 35.24,
            "current_delta_a": 0.0,
            "load_delta_pct": 2.8,
        },
    ]
    analysis = analyze_touch_rows(
        "knee", rows, expected_touch_deg=35.921, arm_margin_deg=4.0)
    assert not analysis.ok
    assert analysis.status == "no_contact"
    assert analysis.reached_search_window
    assert analysis.max_encoder_deg == 35.24


def test_repeat_merge_accepts_two_close_touches() -> None:
    merged = merge_repeat_analyses(
        "knee",
        [
            {
                "ok": True,
                "status": "contact",
                "observed_touch_deg": 36.1,
                "observed_cmd_deg": 44.0,
                "contact_strength": "weak",
                "samples": 12,
                "armed_at_deg": 31.92,
                "max_encoder_deg": 36.1,
                "reached_search_window": True,
            },
            {
                "ok": True,
                "status": "contact",
                "observed_touch_deg": 36.8,
                "observed_cmd_deg": 44.0,
                "contact_strength": "weak",
                "samples": 13,
                "armed_at_deg": 31.92,
                "max_encoder_deg": 36.8,
                "reached_search_window": True,
            },
        ],
        expected_touch_deg=35.921,
        repeat_tolerance_deg=2.0)
    assert merged["ok"]
    assert merged["status"] == "contact"
    assert merged["chosen_repeats"] == [1, 2]
    assert merged["observed_touch_deg"] == 36.45
    assert merged["repeat_spread_deg"] == 0.7


def test_repeat_merge_takes_matching_pair_from_three_touches() -> None:
    merged = merge_repeat_analyses(
        "hip",
        [
            {
                "ok": True,
                "status": "contact",
                "observed_touch_deg": 21.2,
                "samples": 10,
                "armed_at_deg": 17.51,
                "max_encoder_deg": 21.2,
                "reached_search_window": True,
            },
            {
                "ok": True,
                "status": "contact",
                "observed_touch_deg": 25.6,
                "samples": 11,
                "armed_at_deg": 17.51,
                "max_encoder_deg": 25.6,
                "reached_search_window": True,
            },
            {
                "ok": True,
                "status": "contact",
                "observed_touch_deg": 25.1,
                "samples": 11,
                "armed_at_deg": 17.51,
                "max_encoder_deg": 25.1,
                "reached_search_window": True,
            },
        ],
        expected_touch_deg=21.51,
        repeat_tolerance_deg=2.0)
    assert merged["ok"]
    assert merged["chosen_repeats"] == [2, 3]
    assert merged["observed_touch_deg"] == 25.35
    assert merged["repeat_spread_deg"] == 0.5


def test_repeat_merge_rejects_three_disagreeing_touches() -> None:
    merged = merge_repeat_analyses(
        "hip",
        [
            {"ok": True, "observed_touch_deg": 20.0, "samples": 10},
            {"ok": True, "observed_touch_deg": 23.0, "samples": 10},
            {"ok": True, "observed_touch_deg": 26.2, "samples": 10},
        ],
        expected_touch_deg=21.51,
        repeat_tolerance_deg=2.0)
    assert not merged["ok"]
    assert merged["status"] == "repeat_mismatch"
    assert merged["repeat_spread_deg"] == 3.0


def test_height_fit_uses_boot_aware_knee_reading() -> None:
    fit = fit_zero_tip_height_from_contacts(
        [
            {
                "leg": 0,
                "hip": {
                    "ok": True,
                    "observed_touch_deg": 23.685,
                    "contact_strength": "edge_repeat",
                },
                "knee": {
                    "ok": True,
                    "observed_touch_deg": 35.025,
                    "contact_strength": "edge_repeat",
                },
            },
        ],
        input_height_mm=88.0,
        femur_mm=90.0,
        tibia_mm=150.0,
        boot_diameter_mm=9.0)
    assert fit["ok"]
    assert fit["status"] == "axis_disagreement"
    assert math.isclose(fit["knee_mean_height_mm"], 88.0, abs_tol=0.05)
    assert math.isclose(fit["hip_mean_height_mm"], 99.10, abs_tol=0.01)
    assert math.isclose(fit["recommended_height_mm"], 88.0, abs_tol=0.05)
    assert fit["axis_spread_mm"] > 8.0


if __name__ == "__main__":
    test_expected_angles_match_measured_leg0_geometry()
    test_expected_angles_use_pin_tip_without_boot()
    test_sweep_brackets_expected_contact()
    test_refine_sweep_stays_near_contact_command()
    test_analysis_ignores_false_early_load_before_arm()
    test_analysis_uses_small_rise_before_firm_hip_push()
    test_analysis_uses_small_rise_before_larger_knee_load()
    test_analysis_reports_no_contact_when_only_prefloor_noise()
    test_analysis_reports_no_contact_after_encoder_reaches_window()
    test_analysis_ignores_weak_knee_bump_before_expected_angle()
    test_repeat_merge_accepts_two_close_touches()
    test_repeat_merge_takes_matching_pair_from_three_touches()
    test_repeat_merge_rejects_three_disagreeing_touches()
    test_height_fit_uses_boot_aware_knee_reading()
    print("test_touchdown_zero: OK")
