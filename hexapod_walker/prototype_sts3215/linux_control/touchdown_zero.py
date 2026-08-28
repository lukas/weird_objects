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
    armed_at_deg: float
    max_encoder_deg: float | None
    reached_search_window: bool
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
    hip_pin = _asin_deg(clearance, femur + tibia)
    knee_pin = _asin_deg(clearance, tibia)
    if hip_pin is None:
        raise ValueError("zero tip clearance exceeds hip+knee reach")
    if knee_pin is None:
        raise ValueError("zero tip clearance exceeds tibia reach")

    hip_model = "straight_leg_pin_tip"
    hip_expected = hip_pin
    hip_boot = None
    knee_model = "pin_tip"
    knee_expected = knee_pin
    knee_boot = None
    if boot_diameter_mm is not None and float(boot_diameter_mm) > 0.0:
        radius = float(boot_diameter_mm) * 0.5
        center_drop = max(0.0, clearance - radius)
        straight_effective_len = max(1e-9, femur + tibia - radius)
        hip_boot = _asin_deg(center_drop, straight_effective_len)
        if hip_boot is None:
            raise ValueError("zero tip clearance exceeds hip+knee+boot reach")
        hip_model = "straight_leg_spherical_boot_tip"
        hip_expected = hip_boot
        effective_len = max(1e-9, tibia - radius)
        knee_boot = _asin_deg(center_drop, effective_len)
        if knee_boot is None:
            raise ValueError("zero tip clearance exceeds tibia+boot reach")
        knee_model = "spherical_boot_tip"
        knee_expected = knee_boot

    return {
        "hip": TouchExpected(
            axis="hip",
            expected_touch_deg=round(hip_expected, 3),
            model=hip_model,
            pin_tip_deg=round(hip_pin, 3),
            boot_radius_deg=(
                None if hip_boot is None else round(hip_boot, 3)),
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
    ]
    if overrun_deg > 5.0:
        for extra in (5.0, 8.0, 12.0, 16.0):
            if extra < overrun_deg:
                raw.append(e + extra)
    raw.append(e + overrun_deg)
    lo, hi = limit if limit is not None else (-1e9, 1e9)
    out: list[float] = []
    for val in raw:
        v = max(float(lo), min(float(hi), float(val)))
        v = round(v, 2)
        if not out or abs(v - out[-1]) > 0.2:
            out.append(v)
    return out


def make_refine_sweep_angles(
        center_cmd_deg: float, *,
        backoff_deg: float = 10.0,
        overrun_deg: float = 2.5,
        limit: tuple[float, float] | None = None) -> list[float]:
    """Short near-contact sweep for repeat-mismatch refinement."""
    c = float(center_cmd_deg)
    mid_backoff = max(6.0, backoff_deg * 0.65)
    raw = [
        c - backoff_deg,
        c - mid_backoff,
        c - 4.0,
        c - 2.5,
        c - 1.0,
        c,
        c + 1.0,
        c + overrun_deg,
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
    edge_min_encoder = expected - min(float(arm_margin_deg), 2.5)
    edge_current_delta_a = min(float(weak_current_delta_a), 0.006)
    edge_load_delta_pct = min(float(weak_load_delta_pct), 1.5)
    edge_load_jump_pct = max(0.8, float(weak_load_delta_pct) * 0.75)
    edge_current_jump_a = max(0.006, float(weak_current_delta_a) * 0.75)
    edge_confirm_score = 1.45
    edge_follow_window_deg = 4.0
    first_edge = None
    first_weak = None
    first_firm = None
    max_encoder = None
    prev_armed: dict | None = None

    def deltas(row: dict) -> tuple[float, float]:
        return (
            abs(float(row.get("current_delta_a") or 0.0)),
            abs(float(row.get("load_delta_pct") or 0.0)),
        )

    def signal_score(current_delta: float, load_delta: float) -> float:
        cur_score = current_delta / max(float(weak_current_delta_a), 1e-9)
        load_score = load_delta / max(float(weak_load_delta_pct), 1e-9)
        return max(cur_score, load_score)

    for row in rows:
        try:
            encoder = float(row.get("encoder_deg"))
        except (TypeError, ValueError):
            continue
        max_encoder = (
            encoder if max_encoder is None else max(float(max_encoder), encoder))
        current_delta, load_delta = deltas(row)
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
        if prev_armed is not None:
            try:
                prev_encoder = float(prev_armed.get("encoder_deg"))
            except (TypeError, ValueError):
                prev_encoder = None
            if prev_encoder is not None:
                prev_current, prev_load = deltas(prev_armed)
                prev_score = signal_score(prev_current, prev_load)
                this_score = signal_score(current_delta, load_delta)
                prev_small = (
                    prev_current >= edge_current_delta_a
                    or prev_load >= edge_load_delta_pct
                )
                close_after = (
                    0.0 <= encoder - prev_encoder <= edge_follow_window_deg
                )
                confirmed_rise = (
                    firm
                    or this_score >= edge_confirm_score
                    or (
                        load_delta >= float(weak_load_delta_pct)
                        and load_delta - prev_load >= edge_load_jump_pct
                    )
                    or (
                        current_delta >= float(weak_current_delta_a)
                        and current_delta - prev_current >= edge_current_jump_a
                    )
                )
                if (
                        prev_small
                        and confirmed_rise
                        and this_score > prev_score
                        and close_after
                        and prev_encoder >= edge_min_encoder):
                    first_edge = prev_armed
                    break
        # Weak load/current bumps are common while the leg is still moving.
        # Treat them as "maybe" until the encoder reaches the predicted
        # touchdown angle; firm signals are still accepted from the armed
        # window so a real early hit can stop promptly.
        if weak and encoder >= expected and first_weak is None:
            first_weak = row
        if firm and first_firm is None:
            first_firm = row
            break
        prev_armed = row

    picked = first_edge or first_weak or first_firm
    strength = None
    reached = max_encoder is not None and max_encoder >= armed_at
    if picked is None:
        if reached:
            status = "no_contact"
            note = "no armed current/load rise found"
        else:
            status = "encoder_not_reached"
            note = "encoder never reached armed contact window"
        observed = None
        cmd = None
    else:
        current_delta, load_delta = deltas(picked)
        firm = (
            current_delta >= float(firm_current_delta_a)
            or load_delta >= float(firm_load_delta_pct)
        )
        strength = "edge" if picked is first_edge else (
            "firm" if firm else "weak")
        observed = round(float(picked["encoder_deg"]), 3)
        cmd_val = picked.get("cmd_deg")
        cmd = None if cmd_val is None else round(float(cmd_val), 3)
        status = "contact"
        note = (
            "first rising-edge contact hint"
            if picked is first_edge else "first armed contact hint")

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
        armed_at_deg=round(armed_at, 3),
        max_encoder_deg=(
            None if max_encoder is None else round(float(max_encoder), 3)),
        reached_search_window=bool(reached),
        note=note,
    )


def analysis_dict(analysis: TouchAnalysis) -> dict:
    return asdict(analysis)


def _height_from_touch_angle(
        axis: str,
        observed_deg: float,
        *,
        femur_mm: float,
        tibia_mm: float,
        boot_diameter_mm: float | None = None) -> float:
    theta = math.radians(float(observed_deg))
    if axis == "hip":
        reach = float(femur_mm) + float(tibia_mm)
        if boot_diameter_mm is not None and float(boot_diameter_mm) > 0.0:
            radius = float(boot_diameter_mm) * 0.5
            return radius + (reach - radius) * math.sin(theta)
        return reach * math.sin(theta)
    if axis == "knee":
        tibia = float(tibia_mm)
        if boot_diameter_mm is not None and float(boot_diameter_mm) > 0.0:
            radius = float(boot_diameter_mm) * 0.5
            return radius + (tibia - radius) * math.sin(theta)
        return tibia * math.sin(theta)
    raise ValueError(f"unknown touchdown axis {axis!r}")


def fit_zero_tip_height_from_contacts(
        per_leg: list[dict],
        *,
        input_height_mm: float,
        femur_mm: float,
        tibia_mm: float,
        boot_diameter_mm: float | None = None) -> dict:
    """Infer the zero-pose foot clearance implied by touchdown contacts."""
    rows: list[dict] = []
    by_axis: dict[str, list[float]] = {"hip": [], "knee": []}
    for leg_row in per_leg or []:
        leg = leg_row.get("leg")
        for axis in ("hip", "knee"):
            a = leg_row.get(axis)
            if not isinstance(a, dict) or not a.get("ok"):
                continue
            observed = a.get("observed_touch_deg")
            if observed is None:
                continue
            try:
                height = _height_from_touch_angle(
                    axis,
                    float(observed),
                    femur_mm=float(femur_mm),
                    tibia_mm=float(tibia_mm),
                    boot_diameter_mm=boot_diameter_mm)
            except (TypeError, ValueError):
                continue
            row = {
                "leg": leg,
                "axis": axis,
                "observed_touch_deg": round(float(observed), 3),
                "height_mm": round(height, 2),
                "contact_strength": a.get("contact_strength"),
            }
            rows.append(row)
            by_axis[axis].append(float(height))

    if not rows:
        return {
            "ok": False,
            "input_height_mm": round(float(input_height_mm), 2),
            "recommended_height_mm": None,
            "note": "no accepted touchdown contacts to fit height",
            "rows": [],
        }

    def mean(vals: list[float]) -> float | None:
        return None if not vals else sum(vals) / len(vals)

    heights = [float(r["height_mm"]) for r in rows]
    hip_mean = mean(by_axis["hip"])
    knee_mean = mean(by_axis["knee"])
    all_mean = mean(heights)
    assert all_mean is not None
    axis_spread = (
        None if hip_mean is None or knee_mean is None
        else abs(float(hip_mean) - float(knee_mean)))
    # The knee-only tap is the cleaner height measurement when the spherical
    # boot is known; the hip tap spans the whole leg and is easier to bias by
    # a late load edge or a hip zero offset.
    recommended = knee_mean if knee_mean is not None else all_mean
    delta = float(recommended) - float(input_height_mm)
    status = "ok"
    note = "touchdown-derived height is consistent"
    if axis_spread is not None and axis_spread > 5.0:
        status = "axis_disagreement"
        note = "hip and knee imply different heights; height alone is not the full error"
    elif abs(delta) > 3.0:
        status = "height_adjust"
        note = "touchdown contacts suggest updating zero tip height"

    return {
        "ok": True,
        "status": status,
        "input_height_mm": round(float(input_height_mm), 2),
        "recommended_height_mm": round(float(recommended), 2),
        "recommended_delta_mm": round(delta, 2),
        "mean_height_mm": round(float(all_mean), 2),
        "hip_mean_height_mm": (
            None if hip_mean is None else round(float(hip_mean), 2)),
        "knee_mean_height_mm": (
            None if knee_mean is None else round(float(knee_mean), 2)),
        "axis_spread_mm": (
            None if axis_spread is None else round(float(axis_spread), 2)),
        "sample_count": len(rows),
        "rows": rows,
        "note": note,
    }


def merge_repeat_analyses(
        axis: str,
        repeats: list[TouchAnalysis | dict],
        *,
        expected_touch_deg: float,
        repeat_tolerance_deg: float = 2.0) -> dict:
    """Choose a repeat-confirmed touchdown from two or three tap analyses."""
    repeat_dicts = [
        asdict(r) if isinstance(r, TouchAnalysis) else dict(r)
        for r in repeats
    ]
    expected = float(expected_touch_deg)
    contacts: list[tuple[int, float, dict]] = []
    for idx, row in enumerate(repeat_dicts):
        if not row.get("ok"):
            continue
        val = row.get("observed_touch_deg")
        if val is None:
            continue
        contacts.append((idx, float(val), row))

    total_samples = sum(int(r.get("samples") or 0) for r in repeat_dicts)
    max_encoder_vals = [
        float(r["max_encoder_deg"]) for r in repeat_dicts
        if r.get("max_encoder_deg") is not None
    ]
    reached = any(bool(r.get("reached_search_window")) for r in repeat_dicts)
    armed_vals = [
        float(r["armed_at_deg"]) for r in repeat_dicts
        if r.get("armed_at_deg") is not None
    ]
    out = {
        "ok": False,
        "axis": str(axis),
        "status": "repeat_incomplete",
        "expected_touch_deg": round(expected, 3),
        "observed_touch_deg": None,
        "observed_cmd_deg": None,
        "contact_strength": None,
        "encoder_zero_error_deg": None,
        "command_compensation_deg": None,
        "samples": total_samples,
        "armed_at_deg": None if not armed_vals else round(armed_vals[0], 3),
        "max_encoder_deg": (
            None if not max_encoder_vals else round(max(max_encoder_vals), 3)),
        "reached_search_window": bool(reached),
        "repeat_count": len(repeat_dicts),
        "contact_repeat_count": len(contacts),
        "repeat_tolerance_deg": round(float(repeat_tolerance_deg), 3),
        "repeat_spread_deg": None,
        "chosen_repeats": [],
        "repeats": repeat_dicts,
        "note": "need two matching touchdown measurements",
    }

    if len(contacts) < 2:
        statuses = [str(r.get("status") or "") for r in repeat_dicts]
        if contacts:
            out["observed_touch_deg"] = round(contacts[0][1], 3)
            out["encoder_zero_error_deg"] = round(contacts[0][1] - expected, 3)
            out["command_compensation_deg"] = out["encoder_zero_error_deg"]
        elif statuses and all(s == "encoder_not_reached" for s in statuses):
            out["status"] = "encoder_not_reached"
            out["note"] = "encoder never reached armed contact window"
        elif statuses and all(s == "no_contact" for s in statuses):
            out["status"] = "no_contact"
            out["note"] = "no repeated current/load rise found"
        return out

    best: tuple[float, int, int] | None = None
    for a in range(len(contacts)):
        for b in range(a + 1, len(contacts)):
            spread = abs(contacts[a][1] - contacts[b][1])
            if best is None or spread < best[0]:
                best = (spread, a, b)
    assert best is not None
    spread, ia, ib = best
    out["repeat_spread_deg"] = round(float(spread), 3)
    if spread > float(repeat_tolerance_deg):
        vals = [c[1] for c in contacts]
        out["status"] = "repeat_mismatch"
        out["observed_touch_deg"] = round(sum(vals) / len(vals), 3)
        out["encoder_zero_error_deg"] = round(
            float(out["observed_touch_deg"]) - expected, 3)
        out["command_compensation_deg"] = out["encoder_zero_error_deg"]
        out["chosen_repeats"] = [c[0] + 1 for c in contacts]
        out["note"] = "touchdown repeats disagree"
        return out

    chosen = [contacts[ia], contacts[ib]]
    observed = sum(c[1] for c in chosen) / len(chosen)
    cmd_vals = [
        float(c[2]["observed_cmd_deg"]) for c in chosen
        if c[2].get("observed_cmd_deg") is not None
    ]
    strengths = [str(c[2].get("contact_strength") or "") for c in chosen]
    out.update({
        "ok": True,
        "status": "contact",
        "observed_touch_deg": round(observed, 3),
        "observed_cmd_deg": (
            None if len(cmd_vals) != len(chosen)
            else round(sum(cmd_vals) / len(cmd_vals), 3)),
        "contact_strength": (
            "edge_repeat" if any(s == "edge" for s in strengths)
            else "firm_repeat" if any(s == "firm" for s in strengths)
            else "weak_repeat"),
        "encoder_zero_error_deg": round(observed - expected, 3),
        "command_compensation_deg": round(observed - expected, 3),
        "chosen_repeats": [c[0] + 1 for c in chosen],
        "note": "repeat-confirmed touchdown",
    })
    return out


def expected_dicts(expected: dict[str, TouchExpected]) -> dict[str, dict]:
    return {axis: asdict(val) for axis, val in expected.items()}
