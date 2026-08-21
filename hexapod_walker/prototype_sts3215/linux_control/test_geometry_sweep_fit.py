"""Off-robot tests for multi-contact geometry fitting.

Run locally:  python3 linux_control/test_geometry_sweep_fit.py
No hardware: these tests synthesize floor contacts from known link geometry.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from geometry_plant import (FEMUR_MM, TIBIA_MM, fit_contact_sweep)  # noqa: E402


def _solve_knee_for_height(
        hip_deg: float, height_mm: float, *,
        femur_mm: float = FEMUR_MM, tibia_mm: float = TIBIA_MM,
        prefer_deg: float = 70.0) -> float:
    hip = math.radians(hip_deg)
    num = (height_mm - femur_mm * math.sin(hip)) / tibia_mm
    assert abs(num) <= 1.0, (hip_deg, height_mm)
    a = math.asin(max(-1.0, min(1.0, num)))
    knees = []
    for pt in (a, math.pi - a):
        knee = math.degrees(pt) - hip_deg
        if -20.0 <= knee <= 150.0:
            knees.append(knee)
    assert knees, (hip_deg, height_mm)
    return min(knees, key=lambda k: abs(k - prefer_deg))


def _samples_for_geometry(
        *, femur_mm: float = FEMUR_MM, tibia_mm: float = TIBIA_MM,
        hip_zero_deg: float = 0.0, knee_zero_deg: float = 0.0,
        leg0_only_offsets: bool = False) -> list[dict]:
    samples: list[dict] = []
    for leg in range(6):
        height = 112.0 + leg * 1.5
        h_off = hip_zero_deg if (leg == 0 or not leg0_only_offsets) else 0.0
        k_off = knee_zero_deg if (leg == 0 or not leg0_only_offsets) else 0.0
        for raw_hip in (2.0, 7.0, 12.0, 17.0, 22.0, 27.0):
            true_hip = raw_hip + h_off
            true_knee = _solve_knee_for_height(
                true_hip, height,
                femur_mm=femur_mm, tibia_mm=tibia_mm)
            samples.append({
                "accepted": True,
                "contact_detected": True,
                "leg": leg,
                "hip_deg": raw_hip,
                "knee_deg": true_knee - k_off,
            })
    return samples


def test_contact_fit_keeps_configured_links_scale_is_ambiguous() -> None:
    fit = fit_contact_sweep(_samples_for_geometry(
        femur_mm=94.0, tibia_mm=124.0))
    seg = fit["segment_fit"]
    links = seg["link_lengths_mm"]
    assert fit["ok"], fit
    assert seg["ok"], seg
    assert links["femur"] == FEMUR_MM, links
    assert links["tibia"] == TIBIA_MM, links
    assert seg["link_lengths_observable"] is False
    assert seg["status"] == "nominal_link_contact_height_fit"
    assert "independent body-height" in " ".join(seg["notes"])


def test_zero_offset_hint_recovers_leg_specific_offset() -> None:
    fit = fit_contact_sweep(_samples_for_geometry(
        hip_zero_deg=2.0, knee_zero_deg=-3.0, leg0_only_offsets=True))
    leg0 = fit["per_leg"][0]
    assert fit["ok"], fit
    assert abs(leg0["hip_zero_hint_deg"] - 2.0) < 0.4, leg0
    assert abs(leg0["knee_zero_hint_deg"] + 3.0) < 0.4, leg0
    assert leg0["zero_rms_residual_mm"] < 0.2, leg0


def test_single_plant_snapshot_is_marked_partial() -> None:
    samples = [
        {
            "accepted": True,
            "contact_detected": True,
            "leg": leg,
            "hip_deg": 18.0,
            "knee_deg": 80.0,
        }
        for leg in range(6)
    ]
    fit = fit_contact_sweep(samples)
    assert not fit["ok"], fit
    assert fit["status"] == "partial", fit
    assert fit["segment_fit"]["status"] == "not_enough_multi_pose_contacts"


def test_non_contact_and_prefloor_rows_are_rejected() -> None:
    samples = _samples_for_geometry()
    for row in samples:
        row["base_z_mm"] = -125.0
        row["nominal_z_mm"] = -125.0
    samples.append({
        "accepted": True,
        "contact_detected": False,
        "leg": 0,
        "hip_deg": 10.0,
        "knee_deg": 20.0,
        "base_z_mm": -125.0,
        "nominal_z_mm": -90.0,
        "reason": "reached solved floor pose without contact signal",
    })
    samples.append({
        "accepted": True,
        "contact_detected": True,
        "leg": 1,
        "hip_deg": 10.0,
        "knee_deg": 20.0,
        "base_z_mm": -125.0,
        "nominal_z_mm": -90.0,
        "reason": "ignored pre-floor resistance",
    })
    fit = fit_contact_sweep(samples)
    assert fit["ok"], fit
    assert fit["sample_count"] == len(samples) - 2


def _main() -> int:
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
