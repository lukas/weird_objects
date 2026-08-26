"""Off-robot tests for the single-leg height probe."""
from __future__ import annotations

import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from leg_height_probe import (  # noqa: E402
    diagnose_height_probe,
    predict_drops,
    solve_hip_from_total_drop,
    straight_leg_touch_hip_for_clearance,
)


def test_default_stand_drop_residuals_are_zero():
    drops = predict_drops(20.0, 80.0)
    probe = diagnose_height_probe(
        hip_drop_mm=drops.hip_to_foot_mm,
        knee_drop_mm=drops.knee_to_foot_mm,
        calibrated_hip_deg=20.0,
        calibrated_knee_deg=80.0,
    )
    assert abs(probe.residual_mm["hip_to_foot"]) < 1e-6
    assert abs(probe.residual_mm["knee_to_foot"]) < 1e-6
    assert abs(probe.residual_mm["hip_to_knee"]) < 1e-6


def test_height_pair_exposes_tibia_supplement_branch():
    drops = predict_drops(20.0, 80.0)
    probe = diagnose_height_probe(
        hip_drop_mm=drops.hip_to_foot_mm,
        knee_drop_mm=drops.knee_to_foot_mm,
        calibrated_hip_deg=20.0,
        calibrated_knee_deg=80.0,
    )
    pairs = {
        (round(c.hip_deg or 0.0, 6), round(c.knee_deg or 0.0, 6))
        for c in probe.candidates
        if c.source == "measured hip+knee drops"
    }
    assert (20.0, 80.0) in pairs
    assert (20.0, 60.0) in pairs
    assert any("cannot distinguish" in w for w in probe.warnings)


def test_total_drop_with_fixed_knee_picks_calibrated_hip():
    drops = predict_drops(20.0, 80.0)
    hips = solve_hip_from_total_drop(drops.hip_to_foot_mm, 80.0)
    assert any(abs(h - 20.0) < 1e-9 for h in hips)


def test_unreachable_knee_drop_has_no_candidates():
    probe = diagnose_height_probe(
        hip_drop_mm=200.0,
        knee_drop_mm=160.0,  # tibia is 150 mm
        calibrated_hip_deg=20.0,
        calibrated_knee_deg=80.0,
    )
    assert not [
        c for c in probe.candidates
        if c.source == "measured hip+knee drops"
    ]
    assert any("tibia vertical reach" in w for w in probe.warnings)


def test_predict_drops_matches_known_formula():
    drops = predict_drops(20.0, 80.0)
    assert math.isclose(
        drops.hip_to_foot_mm,
        90.0 * math.sin(math.radians(20.0))
        + 150.0 * math.sin(math.radians(100.0)),
    )


def test_straight_leg_touch_from_88mm_clearance():
    touch = straight_leg_touch_hip_for_clearance(88.0)
    assert touch.reachable
    assert math.isclose(touch.hip_deg, 21.510, abs_tol=0.001)
    assert touch.knee_deg == 0.0

    probe = diagnose_height_probe(
        hip_drop_mm=None,
        knee_drop_mm=None,
        zero_tip_clearance_mm=88.0,
    )
    assert probe.straight_leg_touch
    assert probe.straight_leg_touch.hip_deg == touch.hip_deg


if __name__ == "__main__":
    test_default_stand_drop_residuals_are_zero()
    test_height_pair_exposes_tibia_supplement_branch()
    test_total_drop_with_fixed_knee_picks_calibrated_hip()
    test_unreachable_knee_drop_has_no_candidates()
    test_predict_drops_matches_known_formula()
    test_straight_leg_touch_from_88mm_clearance()
    print("test_leg_height_probe: OK")
