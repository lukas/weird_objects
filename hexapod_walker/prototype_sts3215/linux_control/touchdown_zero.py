"""Touchdown-based zero hints for the STS3215 hexapod.

This module is deliberately pure math / record shaping.  Hardware motion lives
in ``bench_api`` so the zero convention can be tested without a robot.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class TouchExpected:
    axis: str
    expected_touch_deg: float
    model: str
    pin_tip_deg: float | None = None
    boot_radius_deg: float | None = None


@dataclass(frozen=True)
class TouchAnalysis:
    ok: bool
    axis: str
    status: str
    expected_touch_deg: float
    observed_touch_deg: float | None
    observed_cmd_deg: float | None
    contact_strength: str | None
    encoder_zero_error_deg: float | None
    command_compensation_deg: float | None
    samples: int
    note: str


def _asin_deg(drop_mm: float, length_mm: float) -> float | None:
    if length_mm <= 0.0:
        raise ValueError("length must be positive")
    ratio = float(drop_mm) / float(length_mm)
    if abs(ratio) > 1.0 + 1e-9:
        return None
    return math.degrees(math.asin(max(-1.0, min(1.0, ratio))))


def expected_touch_angles(
        *,
        zero_tip_clearance_mm: float,
        femur_mm: float,
        tibia_mm: float,
        boot_diameter_mm: float | None = None) -> dict[str, TouchExpected]:
    """Expected hip-only and knee-only touchdown angles.

    ``zero_tip_clearance_mm`` is the measured vertical distance from the foot
    contact tip to the floor at logical hip=0/knee=0.
    """
    clearance = float(zero_tip_clearance_mm)
    femur = float(femur_mm)
    tibia = float(tibia_mm)
    hip = _asin_deg(clearance, femur + tibia)
    knee_pin = _asin_deg(clearance, tibia)
    if hip is None:
        raise ValueError("zero tip clearance exceeds hip+knee reach")
    if knee_pin is None:
        raise ValueError("zero tip clearance exceeds tibia reach")

    knee_model = "pin_tip"
    knee_expected = knee_pin
    knee_boot = None
    if boot_diameter_mm is not None and float(boot_diameter_mm) > 0.0:
        radius = float(boot_diameter_mm) * 0.5
        center_drop = max(0.0, clearance - radius)
        effective_len = max(1e-9, tibia - radius)
        knee_boot = _asin_deg(center_drop, effective_len)

    return {
        "hip": TouchExpected(
            axis="hip",
            expected_touch_deg=round(hip, 3),
            model="straight_leg_pin_tip",
            pin_tip_deg=round(hip, 3),
        ),
        "knee": TouchExpected(
            axis="knee",
            expected_touch_deg=round(knee_expected, 3),
            model=knee_model,
            pin_tip_deg=round(knee_pin, 3),
            boot_radius_deg=(
                None if knee_boot is None else round(knee_boot, 3)),
        ),
    }


def make_sweep_angles(
        expected_deg: float, *,
        start_deg: float = 0.0,
        arm_margin_deg: float = 4.0,
        overrun_deg: float = 5.0,
        limit: tuple[float, float] | None = None) -> list[float]:
    """Small bracket around the expected contact angle."""
    e = float(expected_deg)
    raw = [
        start_deg,
        max(start_deg, e - 12.0),
        max(start_deg, e - 6.0),
        max(start_deg, e - arm_margin_deg),
        e - 2.0,
        e - 1.0,
        e,
        e + 1.0,
        e + 2.0,
        e + 3.0,
        e + overrun_deg,
    ]
    lo, hi = limit if limit is not None else (-1e9, 1e9)
    out: list[float] = []
    for val in raw:
        v = max(float(lo), min(float(hi), float(val)))
        v = round(v, 2)
        if not out or abs(v - out[-1]) > 0.2:
            out.append(v)
    return out


def analyze_touch_rows(
        axis: str,
        rows: list[dict],
        *,
        expected_touch_deg: float,
        arm_margin_deg: float = 4.0,
        weak_current_delta_a: float = 0.012,
        weak_load_delta_pct: float = 2.4,
        firm_current_delta_a: float = 0.055,
        firm_load_delta_pct: float = 5.5) -> TouchAnalysis:
    """Find first useful touchdown and compute the encoder zero hint.

    Sign convention:
    ``encoder_zero_error_deg = observed_touch_deg - expected_touch_deg``.
    Approximate physical angle is therefore ``encoder_deg - error``; a command
    layer that wants physical angles would add ``command_compensation_deg``.
    """
    expected = float(expected_touch_deg)
    armed_at = expected - float(arm_margin_deg)
    first_weak = None
    first_firm = None
    for row in rows:
        try:
            encoder = float(row.get("encoder_deg"))
        except (TypeError, ValueError):
            continue
        current_delta = abs(float(row.get("current_delta_a") or 0.0))
        load_delta = abs(float(row.get("load_delta_pct") or 0.0))
        if encoder < armed_at:
            continue
        firm = (
            current_delta >= float(firm_current_delta_a)
            or load_delta >= float(firm_load_delta_pct)
        )
        weak = (
            firm
            or current_delta >= float(weak_current_delta_a)
            or load_delta >= float(weak_load_delta_pct)
        )
        if weak and first_weak is None:
            first_weak = row
        if firm and first_firm is None:
            first_firm = row
            break

    picked = first_weak
    strength = None
    if picked is None:
        status = "no_contact"
        note = "no armed current/load rise found"
        observed = None
        cmd = None
    else:
        current_delta = abs(float(picked.get("current_delta_a") or 0.0))
        load_delta = abs(float(picked.get("load_delta_pct") or 0.0))
        firm = (
            current_delta >= float(firm_current_delta_a)
            or load_delta >= float(firm_load_delta_pct)
        )
        strength = "firm" if firm else "weak"
        observed = round(float(picked["encoder_deg"]), 3)
        cmd_val = picked.get("cmd_deg")
        cmd = None if cmd_val is None else round(float(cmd_val), 3)
        status = "contact"
        note = "first armed contact hint"

    error = None if observed is None else round(observed - expected, 3)
    return TouchAnalysis(
        ok=picked is not None,
        axis=str(axis),
        status=status,
        expected_touch_deg=round(expected, 3),
        observed_touch_deg=observed,
        observed_cmd_deg=cmd,
        contact_strength=strength,
        encoder_zero_error_deg=error,
        command_compensation_deg=error,
        samples=len(rows),
        note=note,
    )


def analysis_dict(analysis: TouchAnalysis) -> dict:
    return asdict(analysis)


def expected_dicts(expected: dict[str, TouchExpected]) -> dict[str, dict]:
    return {axis: asdict(val) for axis, val in expected.items()}
