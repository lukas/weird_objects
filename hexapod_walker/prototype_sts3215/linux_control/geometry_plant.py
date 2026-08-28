"""Geometry + contact plant finder for Phase-1 balance.

Target stance (operator intent):
  * search through the measured hip/knee stand family and continue deeper
    only if unloaded
  * final saved stance raises roughly the requested clearance from contact
  * abort instead of tipping if the body tilt grows while searching

Method:
  1. Soft-torque, yaw=0, hip=0; start with shallow knees (feet high).
  2. Slowly deepen knees, then hip, while watching current/load/lag.
  3. On multi-knee contact, hold and sample.
  4. Compute a taller pose: reduce knee so FK foot-z rises by
     ``clearance_mm``.
  5. Ease to that pose and save full ``joints_deg[18]`` plant.

Callable from hexapod-web (``/api/rl/find_plant``) — no SSH required.
"""
from __future__ import annotations

import math
import json
import statistics
import time
from pathlib import Path
from typing import Callable

from feetech_bus import (
    AXIS_LIMITS_DEG,
    N_JOINTS,
    load_plant_pose,
    save_plant_pose,
    standing_pose_degrees,
)

# Link lengths (mm) — match tripod_gait / hexapod_prototype.
COXA_MM = 12.5
FEMUR_MM = 90.0
TIBIA_MM = 150.0

# Search down through the measured standing family.  Knee is the tibia's
# absolute leg-plane angle, so the old +80° default would put the foot far
# below the measured floor height.
TARGET_HIP_DEG = 19.0
TARGET_KNEE_DEG = 30.0
START_HIP_DEG = 0.0
START_KNEE_DEG = 8.0            # start high; deepen hip/knee toward floor
CLEARANCE_MM = 25.0             # ~1 in above contact plane
REACH_SECONDS = 22.0
CLEARANCE_SECONDS = 6.0
REACH_TORQUE = 500
SAMPLE_DT = 0.08
SEARCH_HIP_DEG = 20.0
SEARCH_KNEE_DEG = 55.0

CONTACT_CURRENT_FLOOR_A = 0.16
CONTACT_CURRENT_DELTA_A = 0.08
CONTACT_LOAD_FLOOR_PCT = 16.0
CONTACT_LOAD_DELTA_PCT = 8.0
CONTACT_ARM_KNEE_DEG = 20.0     # arm before the measured stand region
CONTACT_MIN_KNEE_JOINTS = 3
CONTACT_WINDOW_S = 0.45
CONTACT_BASELINE_POSE_DEG = 12.0
CONTACT_MAX_KNEE_SPEED = 90.0
CONTACT_POSITION_LAG_DEG = 6.0
# Lag-only contact is useful when current/load are quiet, but it is also the
# easiest way to false-trigger while servos are simply catching up in air.
# Only let lag vote once the search reached the normal plant depth.
CONTACT_LAG_MIN_KNEE_DEG = 28.0
CONTACT_LAG_MAX_KNEE_SPEED = 55.0
MAX_TILT_DELTA_DEG = 24.0       # sustained search lean before limp
MAX_TILT_ABS_DEG = 40.0         # hard cap even if start/platform is tilted
TILT_TRIP_HOLD_S = 0.60         # ignore short accel/IMU bumps
SOFT_SUPPORT_MIN_KNEE_DEG = 36.0
SOFT_SUPPORT_MAX_KNEE_SPEED = 12.0
SOFT_SUPPORT_MIN_LEGS = 2
SOFT_SUPPORT_CURRENT_A = 0.05
SOFT_SUPPORT_LOAD_PCT = 8.0

SWEEP_HIP_OFFSETS_DEG = (-14.0, -9.0, -4.0, 4.0, 9.0, 14.0)
SWEEP_MAX_TARGETS_PER_LEG = 4
SWEEP_LIFT_MM = 18.0
SWEEP_OVERRUN_MM = 7.0
SWEEP_TORQUE = 620
SWEEP_MAX_TILT_DELTA_DEG = 18.0
SWEEP_HARD_TILT_DELTA_DEG = 32.0
SWEEP_TILT_RECOVERY_S = 0.55
SWEEP_MAX_TILT_SKIPS = 12
SWEEP_FLOOR_SETTLE_S = 1.2
SWEEP_FLOOR_SETTLE_DT = 0.08
SWEEP_MAX_LEG_CURRENT_A = 2.6
SWEEP_FLOOR_BAND_MM = 4.0
SWEEP_WEAK_CONTACT_CURRENT_RISE_A = 0.012
SWEEP_WEAK_CONTACT_LOAD_RISE_PCT = 2.4
SWEEP_CONTACT_CURRENT_RISE_A = 0.055
SWEEP_CONTACT_LOAD_RISE_PCT = 5.5
SWEEP_CONTACT_LAG_DEG = 4.0


def _imu_tilt_deg(bus) -> tuple[float | None, float | None]:
    """Best-effort roll/pitch degrees from MPU accel (IMUR)."""
    try:
        if not hasattr(bus, "read_imu"):
            return None, None
        imu = bus.read_imu()
        if not imu:
            return None, None
        ax = float(imu.get("ax_g", 0.0))
        ay = float(imu.get("ay_g", 0.0))
        az = float(imu.get("az_g", 0.0))
        # Same convention as rl_move complementary filter (accel only).
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll, pitch
    except Exception:
        pass
    return None, None


def _angle_delta_deg(now: float, base: float) -> float:
    """Small signed angle difference, robust to +/-180 wrap."""
    return (float(now) - float(base) + 180.0) % 360.0 - 180.0


def foot_z_mm(hip_deg: float, knee_deg: float) -> float:
    """Foot Z in yaw frame (mm); more negative = lower / farther down."""
    p = math.radians(hip_deg)
    k = math.radians(knee_deg)
    return -FEMUR_MM * math.sin(p) - TIBIA_MM * math.sin(k)


def knee_for_foot_z(hip_deg: float, z_mm: float) -> float | None:
    """Solve knee (deg) for desired foot z at fixed hip. None if unreachable."""
    p = math.radians(hip_deg)
    # z = -F sin(p) - T sin(k) -> sin(k) = -(z + F sin p)/T
    num = -(z_mm + FEMUR_MM * math.sin(p)) / TIBIA_MM
    if abs(num) > 1.0:
        return None
    a = math.asin(max(-1.0, min(1.0, num)))
    knee = math.degrees(a)
    if knee < -20.0 or knee > 150.0:
        knee2 = math.degrees(math.pi - a)
        if -20.0 <= knee2 <= 150.0:
            return float(knee2)
        return None
    return float(knee)


def foot_r_mm(hip_deg: float, knee_deg: float) -> float:
    """Foot radial reach in the yaw frame (mm), including coxa link."""
    p = math.radians(hip_deg)
    k = math.radians(knee_deg)
    return COXA_MM + FEMUR_MM * math.cos(p) + TIBIA_MM * math.cos(k)


def _foot_z_model_mm(
        hip_deg: float, knee_deg: float, *,
        femur_mm: float = FEMUR_MM, tibia_mm: float = TIBIA_MM,
        hip_zero_deg: float = 0.0, knee_zero_deg: float = 0.0) -> float:
    hip = math.radians(float(hip_deg) + float(hip_zero_deg))
    knee = math.radians(float(knee_deg) + float(knee_zero_deg))
    return (
        -float(femur_mm) * math.sin(hip)
        - float(tibia_mm) * math.sin(knee)
    )


def _solve_knees_for_foot_z(hip_deg: float, z_mm: float) -> list[float]:
    """Return all in-limit knee branches for ``foot_z_mm(hip, knee)=z``."""
    hip = math.radians(float(hip_deg))
    num = -(float(z_mm) + FEMUR_MM * math.sin(hip)) / TIBIA_MM
    if abs(num) > 1.0:
        return []
    a = math.asin(max(-1.0, min(1.0, num)))
    out: list[float] = []
    lo, hi = AXIS_LIMITS_DEG.get(2, (-20.0, 150.0))
    for pt in (a, math.pi - a):
        knee = math.degrees(pt)
        if lo <= knee <= hi and all(abs(knee - x) > 0.1 for x in out):
            out.append(float(knee))
    return sorted(out)


def _best_knee_for_foot_z(
        hip_deg: float, z_mm: float, *,
        prefer_deg: float | None = None) -> float | None:
    knees = _solve_knees_for_foot_z(hip_deg, z_mm)
    if not knees:
        return None
    if prefer_deg is None:
        return knees[0]
    return min(knees, key=lambda k: abs(k - float(prefer_deg)))


def _candidate_sweep_targets(
        base_hip_deg: float, base_knee_deg: float, base_z_mm: float,
        *, max_targets: int = SWEEP_MAX_TARGETS_PER_LEG) -> list[dict]:
    """Pick reachable same-floor contact targets near the plant pose."""
    hip_lo, hip_hi = AXIS_LIMITS_DEG.get(1, (-80.0, 40.0))
    knee_lo, knee_hi = AXIS_LIMITS_DEG.get(2, (-20.0, 150.0))
    raw: list[dict] = []
    for off in SWEEP_HIP_OFFSETS_DEG:
        hip = max(float(hip_lo), min(float(hip_hi), base_hip_deg + off))
        for knee in _solve_knees_for_foot_z(hip, base_z_mm):
            if not (knee_lo <= knee <= knee_hi):
                continue
            dist = abs(hip - base_hip_deg) + 0.35 * abs(knee - base_knee_deg)
            raw.append({
                "hip_deg": float(hip),
                "knee_deg": float(knee),
                "score": float(dist),
            })

    # Include the learned plant itself as a sanity/contact reference.
    raw.append({
        "hip_deg": float(base_hip_deg),
        "knee_deg": float(base_knee_deg),
        "score": 0.1,
    })
    dedup: list[dict] = []
    for row in sorted(raw, key=lambda r: r["score"]):
        if any(
                abs(row["hip_deg"] - got["hip_deg"]) < 0.5
                and abs(row["knee_deg"] - got["knee_deg"]) < 0.8
                for got in dedup):
            continue
        dedup.append(row)
        if len(dedup) >= max_targets:
            break
    return [
        {"hip_deg": round(r["hip_deg"], 3),
         "knee_deg": round(r["knee_deg"], 3)}
        for r in dedup
    ]


def _mean(vals: list[float]) -> float:
    vals = [float(v) for v in vals]
    return sum(vals) / len(vals) if vals else 0.0


def _rms(vals: list[float]) -> float:
    vals = [float(v) for v in vals]
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0


def _fit_segment_lengths(valid: list[dict]) -> dict:
    """Fit contact-height consistency with configured links.

    Vertical floor contacts alone do not identify absolute femur/tibia
    lengths: scaling femur, tibia, and body height together leaves the
    contact equations unchanged.  Earlier versions tried to regularize that
    underdetermined solve and could report nonsense "learned" link lengths
    when a boot edge or compliant contact tripped early.  Keep the historical
    return shape, but make the math honest: use the configured contact-point
    links and report per-leg height/residual diagnostics only.
    """
    legs = sorted({int(s["leg"]) for s in valid})
    if len(valid) < 10 or len(legs) < 3:
        return {
            "ok": False,
            "status": "not_enough_multi_pose_contacts",
            "sample_count": len(valid),
            "leg_count": len(legs),
            "link_lengths_mm": {
                "coxa": COXA_MM,
                "femur": FEMUR_MM,
                "tibia": TIBIA_MM,
            },
            "notes": [
                "Need several contact poses across multiple legs before "
                "contact-height residuals are meaningful.",
            ],
        }

    per_leg_height: dict[int, float] = {}
    for leg in legs:
        rows = [s for s in valid if int(s["leg"]) == leg]
        heights = []
        for s in rows:
            hip = math.radians(float(s["hip_deg"]))
            knee = math.radians(float(s["knee_deg"]))
            heights.append(
                FEMUR_MM * math.sin(hip)
                + TIBIA_MM * math.sin(knee))
        mean_h = _mean(heights)
        per_leg_height[leg] = mean_h
    residuals = []
    for s in valid:
        leg = int(s["leg"])
        hip = math.radians(float(s["hip_deg"]))
        knee = math.radians(float(s["knee_deg"]))
        pred_h = FEMUR_MM * math.sin(hip) + TIBIA_MM * math.sin(knee)
        residuals.append(pred_h - per_leg_height[leg])
    return {
        "ok": True,
        "status": "nominal_link_contact_height_fit",
        "sample_count": len(valid),
        "leg_count": len(legs),
        "link_lengths_mm": {
            "coxa": COXA_MM,
            "femur": FEMUR_MM,
            "tibia": TIBIA_MM,
        },
        "link_lengths_source": "configured_nominal_contact_model",
        "link_lengths_observable": False,
        "angle_convention": "absolute_tibia",
        "nominal_mm": {
            "coxa": COXA_MM,
            "femur": FEMUR_MM,
            "tibia": TIBIA_MM,
        },
        "per_leg_height_mm": {
            str(leg): round(per_leg_height[leg], 2)
            for leg in legs
        },
        "rms_residual_mm": round(_rms(residuals), 2),
        "max_abs_residual_mm": round(
            max([abs(x) for x in residuals] or [0.0]), 2),
        "notes": [
            "Vertical floor contacts do not identify absolute femur/tibia "
            "lengths without an independent body-height or scale measurement; "
            "configured contact-point links are used for diagnostics.",
            "Boot edges, footpad angle, floor compliance, and servo lag can "
            "move the first-contact pose away from the ideal tibia endpoint.",
        ],
    }


def _fit_zero_offsets_for_leg(rows: list[dict], *,
                              femur_mm: float, tibia_mm: float) -> dict:
    """Find small per-leg zero offsets that flatten contact-height residuals."""
    if len(rows) < 3:
        return {
            "ok": False,
            "status": "not_enough_poses",
            "hip_zero_hint_deg": 0.0,
            "knee_zero_hint_deg": 0.0,
        }

    def score(hip_off: float, knee_off: float) -> tuple[float, float]:
        zs = [
            _foot_z_model_mm(
                float(r["hip_deg"]), float(r["knee_deg"]),
                femur_mm=femur_mm, tibia_mm=tibia_mm,
                hip_zero_deg=hip_off, knee_zero_deg=knee_off)
            for r in rows
        ]
        mean_z = _mean(zs)
        resid = [z - mean_z for z in zs]
        penalty = 0.005 * (abs(hip_off) + abs(knee_off))
        return _rms(resid) + penalty, _rms(resid)

    best = (1e9, 1e9, 0.0, 0.0)
    for hip_i in range(-16, 17):
        hip_off = hip_i * 0.5
        for knee_i in range(-16, 17):
            knee_off = knee_i * 0.5
            total, resid = score(hip_off, knee_off)
            if total < best[0]:
                best = (total, resid, hip_off, knee_off)
    hip_center = best[2]
    knee_center = best[3]
    for hip_i in range(-6, 7):
        hip_off = hip_center + hip_i * 0.1
        for knee_i in range(-6, 7):
            knee_off = knee_center + knee_i * 0.1
            total, resid = score(hip_off, knee_off)
            if total < best[0]:
                best = (total, resid, hip_off, knee_off)

    hip_hint = best[2]
    knee_hint = best[3]
    zs = [
        _foot_z_model_mm(
            float(r["hip_deg"]), float(r["knee_deg"]),
            femur_mm=femur_mm, tibia_mm=tibia_mm,
            hip_zero_deg=hip_hint, knee_zero_deg=knee_hint)
        for r in rows
    ]
    mean_z = _mean(zs)
    return {
        "ok": True,
        "status": "relative_height_fit",
        "hip_zero_hint_deg": round(hip_hint, 2),
        "knee_zero_hint_deg": round(knee_hint, 2),
        "height_mm": round(-mean_z, 2),
        "rms_residual_mm": round(best[1], 2),
        "spread_mm": round(max(zs) - min(zs), 2),
    }


def fit_contact_sweep(samples: list[dict]) -> dict:
    """Summarize multi-pose floor contacts into effective geometry hints."""
    valid = []
    contact_strength_counts: dict[str, int] = {}
    for s in samples or []:
        try:
            if (
                    not bool(s.get("accepted", False))
                    or not bool(s.get("contact_detected", False))):
                continue
            base_z = s.get("base_z_mm")
            nominal_z = s.get("nominal_z_mm")
            if base_z is not None and nominal_z is not None:
                if float(nominal_z) > float(base_z) + SWEEP_FLOOR_BAND_MM:
                    continue
            reason = str(s.get("reason") or "").lower()
            if "without contact signal" in reason:
                continue
            valid.append({
                **s,
                "leg": int(s["leg"]),
                "hip_deg": float(s["hip_deg"]),
                "knee_deg": float(s["knee_deg"]),
            })
            strength = str(s.get("contact_strength") or "unlabeled")
            contact_strength_counts[strength] = (
                contact_strength_counts.get(strength, 0) + 1)
        except (KeyError, TypeError, ValueError):
            continue

    seg = _fit_segment_lengths(valid)
    links = seg.get("link_lengths_mm") or {}
    femur = float(links.get("femur", FEMUR_MM))
    tibia = float(links.get("tibia", TIBIA_MM))
    seg_heights = seg.get("per_leg_height_mm") or {}
    per_leg: list[dict] = []
    heights: list[float] = []
    max_zero = 0.0
    for leg in range(6):
        rows = [s for s in valid if int(s["leg"]) == leg]
        zs = [
            _foot_z_model_mm(
                float(s["hip_deg"]), float(s["knee_deg"]),
                femur_mm=femur, tibia_mm=tibia)
            for s in rows
        ]
        # Zero hints are intentionally against the nominal CAD links.  The
        # global segment fit and per-leg zero offsets can explain the same
        # residuals; separating them keeps the report honest.
        zero = _fit_zero_offsets_for_leg(
            rows, femur_mm=FEMUR_MM, tibia_mm=TIBIA_MM)
        try:
            height = float(seg_heights[str(leg)])
        except (KeyError, TypeError, ValueError):
            height = (
                float(zero.get("height_mm"))
                if zero.get("ok") and zero.get("height_mm") is not None
                else (-_mean(zs) if zs else None)
            )
        if height is not None:
            heights.append(float(height))
        max_zero = max(
            max_zero,
            abs(float(zero.get("hip_zero_hint_deg") or 0.0)),
            abs(float(zero.get("knee_zero_hint_deg") or 0.0)))
        per_leg.append({
            "leg": leg,
            "samples": len(rows),
            "servo_height_mm": (
                None if height is None else round(float(height), 2)),
            "height_spread_mm": (
                None if not zs else round(max(zs) - min(zs), 2)),
            "rms_residual_mm": (
                None if not zs else round(
                    _rms([z - _mean(zs) for z in zs]), 2)),
            "zero_rms_residual_mm": zero.get("rms_residual_mm"),
            "hip_zero_hint_deg": zero.get("hip_zero_hint_deg"),
            "knee_zero_hint_deg": zero.get("knee_zero_hint_deg"),
            "zero_status": zero.get("status"),
        })

    enough = (
        len(valid) >= 10
        and sum(1 for r in per_leg if r["samples"] >= 2) >= 3)
    ok = enough and bool(seg.get("ok"))
    status = "ok" if ok else "partial"
    if not valid:
        status = "no_contacts"
    elif enough and not seg.get("ok"):
        status = str(seg.get("status") or "unusable_fit")
    return {
        "ok": ok,
        "status": status,
        "sample_count": len(valid),
        "segment_fit": seg,
        "per_leg": per_leg,
        "summary": {
            "mean_servo_height_mm": (
                None if not heights else round(_mean(heights), 2)),
            "servo_height_spread_mm": (
                None if not heights else round(max(heights) - min(heights), 2)),
            "max_zero_hint_deg": round(max_zero, 2),
            "contact_strength_counts": contact_strength_counts,
        },
        "observability": [
            "Multiple floor contacts estimate contact-height consistency when "
            "interpreted through the configured contact-point link model.",
            "Zero offsets are hints from contact-height consistency, not an "
            "absolute encoder truth.",
            "Weak first-contact samples are useful for geometry; firm samples "
            "also include boot/chassis compliance under load.",
            "Absolute femur/tibia lengths, coxa length, and chassis width are "
            "not observable from vertical floor contact alone.",
        ],
    }


def search_pose_at(u: float) -> tuple[float, float]:
    """Piecewise downward search pose, shallow → normal plant → deeper."""
    u = max(0.0, min(1.0, float(u)))
    if u <= 0.55:
        a = u / 0.55
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        return (
            START_HIP_DEG + s * (0.0 - START_HIP_DEG),
            START_KNEE_DEG + s * (TARGET_KNEE_DEG - START_KNEE_DEG),
        )
    a = (u - 0.55) / 0.45
    s = 0.5 - 0.5 * math.cos(math.pi * a)
    return (
        0.0 + s * (SEARCH_HIP_DEG - 0.0),
        TARGET_KNEE_DEG + s * (SEARCH_KNEE_DEG - TARGET_KNEE_DEG),
    )


def run_geometry_contact_sweep(
    bus,
    *,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
    max_targets_per_leg: int = SWEEP_MAX_TARGETS_PER_LEG,
) -> dict:
    """Collect several same-floor contact poses for each leg.

    The current plant pose gives one floor contact.  This sweep keeps five
    feet planted, lifts one leg, then searches that leg back to the same
    floor height at a few nearby hip/knee combinations.  The collected
    contacts are enough to estimate effective hip height, femur/tibia scale,
    and likely zero offsets with residuals.
    """
    abort_check = abort_check or (lambda: False)
    try:
        from inplace_demos import (
            _enable_torque, _hold_here, _live_robot_ids, _set_torque_limit,
            _write_pose, ease_to_pose,
        )
    except ImportError as e:
        return {"ok": False, "error": f"inplace_demos missing: {e}",
                "mode": "geometry_sweep"}

    def _progress(msg: str, **extra) -> None:
        if on_progress:
            try:
                on_progress({"msg": msg, "mode": "geometry_sweep", **extra})
            except Exception:
                pass

    def clamp(x: float, lo: float, hi: float) -> float:
        return max(float(lo), min(float(hi), float(x)))

    def read_feedback(j: int) -> dict | None:
        try:
            return bus.read_feedback(j)
        except Exception:
            return None

    def read_pose() -> list[float] | None:
        row: list[float] = []
        for j in range(N_JOINTS):
            try:
                v = bus.read_position_deg(j)
            except Exception:
                v = None
            if v is None:
                return None
            row.append(float(v))
        return row

    def median_pose(samples: int = 4) -> list[float] | None:
        rows = []
        for _ in range(samples):
            q = read_pose()
            if q is not None:
                rows.append(q)
            time.sleep(0.025)
        if not rows:
            return None
        return [_median([r[j] for r in rows], rows[0][j])
                for j in range(N_JOINTS)]

    def tilt_delta(base: tuple[float, float] | None) -> tuple[float, float] | None:
        now = _imu_tilt_deg(bus)
        if now[0] is None or now[1] is None or base is None:
            return None
        return (
            _angle_delta_deg(float(now[0]), base[0]),
            _angle_delta_deg(float(now[1]), base[1]),
        )

    def settled_tilt_delta(
            base: tuple[float, float] | None,
            seconds: float = SWEEP_TILT_RECOVERY_S
    ) -> tuple[float, float] | None:
        rows: list[tuple[float, float]] = []
        deadline = time.monotonic() + max(0.05, float(seconds))
        while time.monotonic() < deadline:
            td = tilt_delta(base)
            if td is not None:
                rows.append(td)
            time.sleep(0.04)
        if not rows:
            return None
        return (
            _median([r for r, _p in rows], 0.0),
            _median([p for _r, p in rows], 0.0),
        )

    live = _live_robot_ids(bus)
    if len(live) < 12:
        return {"ok": False, "error": f"need more servos (live={len(live)})",
                "mode": "geometry_sweep", "live": sorted(live)}
    try:
        plant = load_plant_pose()
        base = [float(x) for x in standing_pose_degrees()]
    except Exception as e:
        return {"ok": False, "mode": "geometry_sweep",
                "error": f"plant pose unavailable: {e}"}
    if len(base) != N_JOINTS:
        return {"ok": False, "mode": "geometry_sweep",
                "error": "plant pose is not 18 joints"}

    hip_lo, hip_hi = AXIS_LIMITS_DEG.get(1, (-80.0, 40.0))
    knee_lo, knee_hi = AXIS_LIMITS_DEG.get(2, (-20.0, 150.0))
    samples: list[dict] = []
    target_plan: list[dict] = []

    _progress("dimension sweep: settle learned plant")
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, SWEEP_TORQUE)
    if not ease_to_pose(bus, base, abort_check=abort_check, seconds=2.0,
                        label="geometry dimension sweep plant"):
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_sweep",
                "error": "plant settle aborted"}
    time.sleep(0.2)

    tilts = []
    for _ in range(5):
        rp = _imu_tilt_deg(bus)
        if rp[0] is not None and rp[1] is not None:
            tilts.append((float(rp[0]), float(rp[1])))
        time.sleep(0.025)
    base_tilt = None
    if tilts:
        base_tilt = (
            _median([r for r, _p in tilts], 0.0),
            _median([p for _r, p in tilts], 0.0),
        )

    def make_pose(leg: int, hip: float, knee: float) -> list[float]:
        q = list(base)
        j = leg * 3
        q[j + 1] = clamp(hip, hip_lo, hip_hi)
        q[j + 2] = clamp(knee, knee_lo, knee_hi)
        return q

    def sample_contact(
            leg: int, goal: list[float], *, target: dict,
            detected: bool, reason: str, step: int,
            accepted: bool | None = None,
            tilt_trip: tuple[float, float] | None = None,
            state: dict | None = None,
            contact_strength: str | None = None) -> dict:
        q = median_pose(3) or goal
        j = leg * 3
        fb_rows = [read_feedback(j + k) or {} for k in range(3)]
        currents = [abs(float(r.get("current_a") or 0.0)) for r in fb_rows]
        loads = [float(r.get("load_pct") or 0.0) for r in fb_rows]
        row = {
            "leg": leg,
            "target_hip_deg": round(float(target["hip_deg"]), 3),
            "target_knee_deg": round(float(target["knee_deg"]), 3),
            "hip_deg": round(float(q[j + 1]), 3),
            "knee_deg": round(float(q[j + 2]), 3),
            "yaw_deg": round(float(q[j]), 3),
            "contact_detected": bool(detected),
            "accepted": bool(detected) if accepted is None else bool(accepted),
            "reason": reason,
            "search_step": int(step),
            "base_z_mm": round(float(target.get("base_z_mm", 0.0)), 2),
            "nominal_z_mm": round(foot_z_mm(q[j + 1], q[j + 2]), 2),
            "nominal_radial_mm": round(foot_r_mm(q[j + 1], q[j + 2]), 2),
            "max_leg_current_a": round(max(currents or [0.0]), 3),
            "max_leg_load_pct": round(max(loads or [0.0]), 1),
        }
        if contact_strength:
            row["contact_strength"] = contact_strength
        if state is not None:
            row["current_rise_a"] = round(
                float(state.get("current_rise") or 0.0), 3)
            row["load_rise_pct"] = round(
                float(state.get("load_rise") or 0.0), 1)
            row["position_lag_deg"] = round(
                float(state.get("lag") or 0.0), 2)
            row["foot_z_now_mm"] = round(
                float(state.get("z_now") or 0.0), 2)
            row["foot_z_cmd_mm"] = round(
                float(state.get("z_cmd") or 0.0), 2)
        td = tilt_delta(base_tilt)
        if td is not None:
            row["roll_delta_deg"] = round(td[0], 2)
            row["pitch_delta_deg"] = round(td[1], 2)
        if tilt_trip is not None:
            row["tilt_trip_delta_deg"] = [
                round(float(tilt_trip[0]), 2),
                round(float(tilt_trip[1]), 2),
            ]
        samples.append(row)
        return row

    def probe_target(leg: int, target: dict) -> tuple[bool, str | None]:
        hip = float(target["hip_deg"])
        knee = float(target["knee_deg"])
        base_z = float(target["base_z_mm"])
        start_knee = _best_knee_for_foot_z(
            hip, base_z + SWEEP_LIFT_MM, prefer_deg=knee)
        if start_knee is None:
            return False, "no lifted start pose"
        deep_knee = _best_knee_for_foot_z(
            hip, base_z - SWEEP_OVERRUN_MM, prefer_deg=knee)
        if deep_knee is None:
            # Some plant poses already sit near maximum vertical reach.  In
            # that case, probing to the solved floor pose is the safest stop.
            deep_knee = knee
        start = make_pose(leg, hip, start_knee)
        goal = make_pose(leg, hip, deep_knee)

        def retreat_probe_leg() -> None:
            try:
                _write_pose(bus, start, live, speed=140, acc=12)
                time.sleep(0.22)
            except Exception:
                pass
            _hold_here(bus, live)

        _write_pose(bus, start, live, speed=125, acc=12)
        time.sleep(0.25)
        base_fb = [read_feedback(leg * 3 + k) or {} for k in range(3)]
        base_current = max([
            abs(float(r.get("current_a") or 0.0)) for r in base_fb
        ] or [0.0])
        base_load = max([
            float(r.get("load_pct") or 0.0) for r in base_fb
        ] or [0.0])
        j = leg * 3

        def check_tilt_trip(step: int, goal_pose: list[float]) -> tuple[bool, str | None] | None:
            td = tilt_delta(base_tilt)
            if td is None or max(abs(td[0]), abs(td[1])) <= SWEEP_MAX_TILT_DELTA_DEG:
                return None
            hard_trip = (
                max(abs(td[0]), abs(td[1]))
                > SWEEP_HARD_TILT_DELTA_DEG
            )
            trip_reason = (
                "tilt backoff" if hard_trip else
                f"tilt skip (Δ{td[0]:+.1f}°/{td[1]:+.1f}°)")
            sample_contact(
                leg, goal_pose, target=target, detected=False,
                accepted=False, reason=trip_reason, step=step,
                tilt_trip=td)
            retreat_probe_leg()
            settled_td = settled_tilt_delta(base_tilt)
            if (settled_td is not None
                    and max(abs(settled_td[0]), abs(settled_td[1]))
                    > SWEEP_HARD_TILT_DELTA_DEG):
                return False, (
                    "tilt abort "
                    f"(Δ{settled_td[0]:+.1f}°/"
                    f"{settled_td[1]:+.1f}° sustained)")
            _progress(
                f"dimension sweep: L{leg} backed off after tilt "
                f"Δ{td[0]:+.1f}°/{td[1]:+.1f}°")
            return True, "tilt skip"

        def probe_state(commanded_pose: list[float]) -> dict:
            fb_rows = [read_feedback(j + k) or {} for k in range(3)]
            currents = [abs(float(r.get("current_a") or 0.0)) for r in fb_rows]
            loads = [float(r.get("load_pct") or 0.0) for r in fb_rows]
            hip_row = fb_rows[1] or {}
            knee_row = fb_rows[2] or {}
            hip_cmd = float(commanded_pose[j + 1])
            knee_cmd = float(commanded_pose[j + 2])
            hip_now = float(
                hip_row["deg"] if hip_row.get("deg") is not None
                else hip_cmd)
            knee_now = float(
                knee_row["deg"] if knee_row.get("deg") is not None
                else knee_cmd)
            lag = max(abs(hip_cmd - hip_now), abs(knee_cmd - knee_now))
            z_cmd = foot_z_mm(hip_cmd, knee_cmd)
            z_now = foot_z_mm(hip_now, knee_now)
            current_rise = max(0.0, max(currents or [0.0]) - base_current)
            load_rise = max(0.0, max(loads or [0.0]) - base_load)
            cmd_reached_floor = z_cmd <= base_z + SWEEP_FLOOR_BAND_MM
            measured_reached_floor = z_now <= base_z + SWEEP_FLOOR_BAND_MM
            weak_contact_signal = (
                current_rise >= SWEEP_WEAK_CONTACT_CURRENT_RISE_A
                or load_rise >= SWEEP_WEAK_CONTACT_LOAD_RISE_PCT
                or (measured_reached_floor and lag >= SWEEP_CONTACT_LAG_DEG)
            )
            firm_contact_signal = (
                current_rise >= SWEEP_CONTACT_CURRENT_RISE_A
                or load_rise >= SWEEP_CONTACT_LOAD_RISE_PCT
                or (measured_reached_floor and lag >= SWEEP_CONTACT_LAG_DEG)
            )
            return {
                "max_current": max(currents or [0.0]),
                "max_load": max(loads or [0.0]),
                "lag": lag,
                "z_cmd": z_cmd,
                "z_now": z_now,
                "current_rise": current_rise,
                "load_rise": load_rise,
                "cmd_reached_floor": cmd_reached_floor,
                "measured_reached_floor": measured_reached_floor,
                "contact_signal": weak_contact_signal,
                "weak_contact_signal": weak_contact_signal,
                "firm_contact_signal": firm_contact_signal,
            }

        detected = False
        reason = "no contact signal"
        last_goal = start
        steps = 12
        for step in range(1, steps + 1):
            if abort_check():
                _hold_here(bus, live)
                return False, "aborted"
            a = step / steps
            q = list(base)
            j = leg * 3
            q[j + 1] = hip
            q[j + 2] = start_knee + (deep_knee - start_knee) * a
            last_goal = q
            _write_pose(bus, q, live, speed=105, acc=10)
            time.sleep(0.075)

            tilt_result = check_tilt_trip(step, last_goal)
            if tilt_result is not None:
                return tilt_result

            state = probe_state(q)
            if state["max_current"] > SWEEP_MAX_LEG_CURRENT_A:
                sample_contact(
                    leg, last_goal, target=target, detected=False,
                    reason="current abort", step=step, state=state)
                retreat_probe_leg()
                return False, "current abort"

            if (state["cmd_reached_floor"] and state["contact_signal"]
                    and not state["measured_reached_floor"]):
                reason = (
                    "ignored pre-floor resistance "
                    f"(z {state['z_now']:.1f} vs floor {base_z:.1f})")
                continue
            if state["measured_reached_floor"] and state["contact_signal"]:
                firm = bool(state.get("firm_contact_signal"))
                sample_contact(
                    leg, last_goal, target=target,
                    detected=True, accepted=True,
                    reason=(
                        "firm floor contact signal" if firm
                        else "first-contact brush; backed off"),
                    step=step, state=state,
                    contact_strength="firm" if firm else "weak")
                retreat_probe_leg()
                return True, None
            if (step == steps and state["cmd_reached_floor"]
                    and state["measured_reached_floor"]):
                sample_contact(
                    leg, last_goal, target=target,
                    detected=False, accepted=False,
                    reason="measured floor pose reached without contact signal",
                    step=step, state=state)
                retreat_probe_leg()
                return True, None

        if foot_z_mm(last_goal[j + 1], last_goal[j + 2]) <= base_z + SWEEP_FLOOR_BAND_MM:
            deadline = time.monotonic() + SWEEP_FLOOR_SETTLE_S
            settle_step = steps
            last_state = None
            while time.monotonic() < deadline:
                if abort_check():
                    _hold_here(bus, live)
                    return False, "aborted"
                _write_pose(bus, last_goal, live, speed=95, acc=8)
                time.sleep(SWEEP_FLOOR_SETTLE_DT)
                settle_step += 1

                tilt_result = check_tilt_trip(settle_step, last_goal)
                if tilt_result is not None:
                    return tilt_result

                last_state = probe_state(last_goal)
                if last_state["max_current"] > SWEEP_MAX_LEG_CURRENT_A:
                    sample_contact(
                        leg, last_goal, target=target, detected=False,
                        reason="current abort", step=settle_step,
                        state=last_state)
                    retreat_probe_leg()
                    return False, "current abort"
                if (last_state["contact_signal"]
                        and not last_state["measured_reached_floor"]):
                    continue
                if (last_state["measured_reached_floor"]
                        and last_state["contact_signal"]):
                    firm = bool(last_state.get("firm_contact_signal"))
                    sample_contact(
                        leg, last_goal, target=target,
                        detected=True, accepted=True,
                        reason=(
                            "firm floor contact signal after settle" if firm
                            else "first-contact brush after settle; backed off"),
                        step=settle_step, state=last_state,
                        contact_strength="firm" if firm else "weak")
                    retreat_probe_leg()
                    return True, None
                if last_state["measured_reached_floor"]:
                    sample_contact(
                        leg, last_goal, target=target,
                        detected=False, accepted=False,
                        reason="measured floor pose reached without contact signal",
                        step=settle_step, state=last_state)
                    retreat_probe_leg()
                    return True, None

            if last_state is not None:
                if last_state["contact_signal"]:
                    reason = (
                        "ignored pre-floor resistance "
                        f"(z {last_state['z_now']:.1f} vs floor {base_z:.1f})")
                    detected = True
                else:
                    reason = (
                        "servo did not reach solved floor pose "
                        f"(z {last_state['z_now']:.1f} vs floor {base_z:.1f})")
                    detected = False
                sample_contact(
                    leg, last_goal, target=target,
                    detected=detected, accepted=False,
                    reason=reason, step=settle_step, state=last_state)
                retreat_probe_leg()
                return True, None

        sample_contact(
            leg, last_goal, target=target, detected=False,
            reason=reason, step=steps)
        retreat_probe_leg()
        return True, None

    tilt_skips = 0
    stop_sweep = False
    try:
        for leg in range(6):
            if stop_sweep:
                break
            if abort_check():
                _hold_here(bus, live)
                return {"ok": False, "aborted": True,
                        "mode": "geometry_sweep", "samples": samples}
            j = leg * 3
            base_hip = float(base[j + 1])
            base_knee = float(base[j + 2])
            base_z = foot_z_mm(base_hip, base_knee)
            targets = _candidate_sweep_targets(
                base_hip, base_knee, base_z,
                max_targets=max_targets_per_leg)
            for t in targets:
                t["base_z_mm"] = round(base_z, 3)
                t["leg"] = leg
                t["base_hip_deg"] = round(base_hip, 3)
                t["base_knee_deg"] = round(base_knee, 3)
            target_plan.extend(targets)
            _progress(
                f"dimension sweep: L{leg} {len(targets)} contact poses")
            for target in targets:
                ok, reason = probe_target(leg, target)
                if (not ok and (reason in ("aborted", "current abort")
                                or str(reason).startswith("tilt abort"))):
                    _set_torque_limit(bus, live, 1000)
                    return {
                        "ok": False,
                        "aborted": True,
                        "mode": "geometry_sweep",
                        "error": reason,
                        "samples": samples,
                        "target_plan": target_plan,
                        "fit": fit_contact_sweep(samples),
                    }
                if reason == "tilt skip":
                    tilt_skips += 1
                    _progress(
                        f"dimension sweep: L{leg} skipped after tilt backoff")
                    if tilt_skips >= SWEEP_MAX_TILT_SKIPS:
                        _progress(
                            "dimension sweep: stopping early after recovered "
                            "tilt backoffs")
                        stop_sweep = True
                    break
                _write_pose(bus, base, live, speed=125, acc=12)
                time.sleep(0.18)
        _progress("dimension sweep: return to plant")
        _write_pose(bus, base, live, speed=125, acc=12)
        time.sleep(0.35)
        _hold_here(bus, live)
    finally:
        _set_torque_limit(bus, live, 1000)

    fit = fit_contact_sweep(samples)
    log_dir = Path(__file__).resolve().parent / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"geometry_sweep_{stamp}.json"
    latest = log_dir / "geometry_sweep_latest.json"
    payload = {
        "ok": bool(fit.get("ok")),
        "mode": "geometry_sweep",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "plant": plant,
        "target_plan": target_plan,
        "samples": samples,
        "fit": fit,
        "tilt_skips": tilt_skips,
        "stopped_early": bool(stop_sweep),
        "path": str(path),
        "log_name": path.name,
        "latest": str(latest),
    }
    path.write_text(json.dumps(payload, indent=2) + "\n")
    latest.write_text(json.dumps(payload, indent=2) + "\n")
    summary = fit.get("summary") or {}
    msg = (
        f"dimension sweep {fit.get('status')}; "
        f"{fit.get('sample_count', 0)} contacts"
    )
    if summary.get("mean_servo_height_mm") is not None:
        msg += (
            f", height {summary['mean_servo_height_mm']:.1f}mm"
            f" spread {summary.get('servo_height_spread_mm', 0.0):.1f}mm"
        )
    if stop_sweep:
        msg += f"; stopped early after {tilt_skips} tilt backoff(s)"
    payload["msg"] = msg
    return payload


def _median(vals: list[float], default: float) -> float:
    return float(statistics.median(vals)) if vals else default


def run_geometry_plant(
    bus,
    *,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
    clearance_mm: float = CLEARANCE_MM,
    reach_seconds: float = REACH_SECONDS,
) -> dict:
    """Contact reach at hip=0 → raise by clearance → save joints_deg plant."""
    abort_check = abort_check or (lambda: False)

    try:
        from inplace_demos import (
            _enable_torque, _hold_here, _limp_all, _live_robot_ids,
            _set_torque_limit, _write_pose, ease_to_pose,
        )
    except ImportError as e:
        return {"ok": False, "error": f"inplace_demos missing: {e}",
                "mode": "geometry_plant"}

    live = _live_robot_ids(bus)
    if len(live) < 12:
        return {"ok": False, "error": f"need more servos (live={len(live)})",
                "mode": "geometry_plant", "live": sorted(live)}

    def _progress(msg: str, **extra):
        if on_progress:
            try:
                on_progress({"msg": msg, "mode": "geometry_plant", **extra})
            except Exception:
                pass

    def _pose(hip: float, knee: float) -> list[float]:
        out: list[float] = []
        for _ in range(6):
            out.extend([0.0, float(hip), float(knee)])
        return out

    def _fb(j: int):
        try:
            return bus.read_feedback(j)
        except Exception:
            return None

    clearance_mm = float(clearance_mm)
    _progress(f"soft torque → ground search "
              f"(to hip {SEARCH_HIP_DEG:+.0f}° / "
              f"knee {SEARCH_KNEE_DEG:.0f}° if needed)")
    _set_torque_limit(bus, live, REACH_TORQUE)
    _enable_torque(bus, live)
    if abort_check():
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}

    # Ease to start pose (not the tall hip0/knee80 stilt).
    start = _pose(START_HIP_DEG, START_KNEE_DEG)
    if not ease_to_pose(bus, start, abort_check=abort_check, seconds=3.0,
                        label="geo_start"):
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}
    base_tilts: list[tuple[float, float]] = []
    for _ in range(5):
        rp = _imu_tilt_deg(bus)
        if rp[0] is not None and rp[1] is not None:
            base_tilts.append((float(rp[0]), float(rp[1])))
        time.sleep(0.04)
    tilt_base = None
    if base_tilts:
        tilt_base = (
            _median([r for r, _p in base_tilts], 0.0),
            _median([p for _r, p in base_tilts], 0.0),
        )
        _progress(
            f"tilt baseline roll={tilt_base[0]:+.1f}° "
            f"pitch={tilt_base[1]:+.1f}°")

    # --- blend toward ground; contact usually appears before full depth ---
    _progress("reaching for ground (current/load/lag contact watch)…")
    n = max(2, int(reach_seconds / SAMPLE_DT))
    t0 = time.monotonic()
    base_I: list[float] = []
    base_L: list[float] = []
    hot_since: dict[int, float] = {}
    tilt_hot_since: float | None = None
    contact_found = False
    contact_knee = None
    contact_joints: list[int] = []
    last_hip_cmd = START_HIP_DEG
    last_knee_cmd = START_KNEE_DEG
    last_knee_now = 0.0
    last_knee_spd = 999.0
    last_support_legs: list[int] = []

    for i in range(n):
        if abort_check():
            _set_torque_limit(bus, live, 1000)
            return {"ok": False, "aborted": True, "mode": "geometry_plant"}
        a = (i + 1) / n
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        hip_cmd, knee_cmd = search_pose_at(s)
        last_hip_cmd, last_knee_cmd = hip_cmd, knee_cmd
        goal = _pose(hip_cmd, knee_cmd)
        _write_pose(bus, goal, live, speed=160, acc=12)
        time.sleep(SAMPLE_DT)

        roll_d, pitch_d = _imu_tilt_deg(bus)
        if roll_d is not None and pitch_d is not None:
            if tilt_base is not None:
                roll_trip = _angle_delta_deg(roll_d, tilt_base[0])
                pitch_trip = _angle_delta_deg(pitch_d, tilt_base[1])
            else:
                roll_trip = float(roll_d)
                pitch_trip = float(pitch_d)
            trip_raw = (
                max(abs(roll_trip), abs(pitch_trip)) > MAX_TILT_DELTA_DEG
                or max(abs(float(roll_d)), abs(float(pitch_d))) > MAX_TILT_ABS_DEG)
            now = time.monotonic()
            if trip_raw:
                if tilt_hot_since is None:
                    tilt_hot_since = now
            else:
                tilt_hot_since = None
            if tilt_hot_since is not None and now - tilt_hot_since >= TILT_TRIP_HOLD_S:
                _hold_here(bus, live)
                try:
                    _limp_all(bus, live)
                except Exception:
                    pass
                _set_torque_limit(bus, live, 1000)
                _progress(
                    f"TILT abort roll={roll_d:+.1f}° pitch={pitch_d:+.1f}° "
                    f"(Δ{roll_trip:+.1f}°/{pitch_trip:+.1f}°) — limp")
                return {
                    "ok": False,
                    "aborted": True,
                    "error": (
                        f"tilt abort ({roll_d:+.1f}°, {pitch_d:+.1f}°; "
                        f"Δ{roll_trip:+.1f}°/{pitch_trip:+.1f}°)"),
                    "mode": "geometry_plant",
                    "contact_found": contact_found,
                    "tilt_baseline_deg": (
                        None if tilt_base is None else
                        [round(tilt_base[0], 2), round(tilt_base[1], 2)]),
                    "tilt_delta_deg": [
                        round(roll_trip, 2), round(pitch_trip, 2)],
                }

        knee_presents: list[float] = []
        knee_speeds: list[float] = []
        for leg in range(6):
            jk = leg * 3 + 2
            fb = _fb(jk)
            if fb is None:
                continue
            knee_presents.append(float(fb["deg"]))
            knee_speeds.append(abs(float(fb.get("speed_deg_s") or 0)))

        knee_now = _median(knee_presents, 0.0)
        knee_spd = _median(knee_speeds, 999.0)
        last_knee_now = knee_now
        last_knee_spd = knee_spd
        i_thr = max(CONTACT_CURRENT_FLOOR_A,
                    (_median(base_I, 0.05) + CONTACT_CURRENT_DELTA_A))
        l_thr = max(CONTACT_LOAD_FLOOR_PCT,
                    (_median(base_L, 5.0) + CONTACT_LOAD_DELTA_PCT))
        armed = (knee_now >= CONTACT_ARM_KNEE_DEG
                 and knee_spd <= CONTACT_MAX_KNEE_SPEED)

        support_legs: set[int] = set()
        for leg in range(6):
            jk = leg * 3 + 2
            fb = _fb(jk)
            if fb is None:
                continue
            present = float(fb["deg"])
            cur = abs(float(fb.get("current_a") or 0.0))
            load = float(fb.get("load_pct") or 0.0)
            speed = abs(float(fb.get("speed_deg_s") or 0.0))
            lag = float(knee_cmd) - present
            if (abs(present) < CONTACT_BASELINE_POSE_DEG
                    and (time.monotonic() - t0) < 3.0):
                base_I.append(cur)
                base_L.append(load)
            lag_contact = (
                knee_cmd >= CONTACT_LAG_MIN_KNEE_DEG
                and lag >= CONTACT_POSITION_LAG_DEG
                and speed <= CONTACT_LAG_MAX_KNEE_SPEED)
            if armed and (cur >= i_thr or load >= l_thr or lag_contact):
                hot_since.setdefault(jk, time.monotonic())
            elif jk in hot_since:
                if time.monotonic() - hot_since[jk] > CONTACT_WINDOW_S:
                    del hot_since[jk]
            if (cur >= SOFT_SUPPORT_CURRENT_A
                    or load >= SOFT_SUPPORT_LOAD_PCT
                    or lag_contact):
                support_legs.add(leg)
            fh = _fb(leg * 3 + 1)
            if fh is not None:
                hcur = abs(float(fh.get("current_a") or 0.0))
                hload = float(fh.get("load_pct") or 0.0)
                if hcur >= SOFT_SUPPORT_CURRENT_A or hload >= SOFT_SUPPORT_LOAD_PCT:
                    support_legs.add(leg)
        last_support_legs = sorted(support_legs)

        t_wall = time.monotonic()
        agreed = [j for j, th in hot_since.items()
                  if t_wall - th <= CONTACT_WINDOW_S]
        if armed and len(agreed) >= CONTACT_MIN_KNEE_JOINTS:
            contact_found = True
            contact_joints = sorted(agreed)
            contact_knee = knee_now
            _hold_here(bus, live)
            _progress(f"contact @ knee≈{knee_now:.0f}° "
                      f"({len(agreed)} knees, I≥{i_thr:.2f}A / "
                      f"L≥{l_thr:.0f}%)")
            break

        if (i + 1) % 12 == 0:
            _progress(f"reach {100 * a:.0f}%  cmd hip {hip_cmd:+.0f}° / "
                      f"knee {knee_cmd:.0f}°  present knee≈{knee_now:.0f}°  "
                      f"hot={len(agreed)}")

    if abort_check():
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}

    if not contact_found or contact_knee is None:
        stable_full_reach = (
            last_hip_cmd >= SEARCH_HIP_DEG - 1.0
            and last_knee_cmd >= SEARCH_KNEE_DEG - 2.0
            and last_knee_now >= SOFT_SUPPORT_MIN_KNEE_DEG
            and last_knee_spd <= SOFT_SUPPORT_MAX_KNEE_SPEED
            and len(last_support_legs) >= SOFT_SUPPORT_MIN_LEGS)
        try:
            plant = load_plant_pose()
        except Exception:
            plant = {}
        using_existing = (
            stable_full_reach
            and bool(plant.get("learned"))
            and isinstance(plant.get("joints_deg"), list)
            and len(plant.get("joints_deg") or []) == N_JOINTS)
        if using_existing:
            pose = standing_pose_degrees()
            hip = float(plant.get("hip_deg", TARGET_HIP_DEG))
            knee = float(plant.get("knee_deg", TARGET_KNEE_DEG))
            _progress(
                "no sharp load spike, but reached stable support; "
                "using existing learned plant")
            if not ease_to_pose(
                    bus, pose, abort_check=abort_check, seconds=2.0,
                    label="existing learned plant"):
                _set_torque_limit(bus, live, 1000)
                return {"ok": False, "aborted": True,
                        "mode": "geometry_plant",
                        "contact_found": False,
                        "used_existing_plant": True}
            _set_torque_limit(bus, live, 1000)
            _hold_here(bus, live)
            return {
                "ok": True,
                "mode": "geometry_plant",
                "contact_found": False,
                "used_existing_plant": True,
                "saved": False,
                "soft_support_legs": last_support_legs,
                "searched_hip_deg": round(last_hip_cmd, 2),
                "searched_knee_deg": round(last_knee_cmd, 2),
                "present_knee_deg": round(last_knee_now, 2),
                "hip_deg": round(hip, 2),
                "knee_deg": round(knee, 2),
                "pose": pose,
                "msg": (
                    "no force spike; stable support reached, using existing "
                    f"plant hip {hip:+.1f}° / knee {knee:+.1f}°"),
            }
        _set_torque_limit(bus, live, 1000)
        _progress("no contact after full depth — not saving")
        return {
            "ok": False,
            "error": (
                "no ground contact detected after search to "
                f"hip {SEARCH_HIP_DEG:+.0f}° / knee {SEARCH_KNEE_DEG:.0f}°"),
            "mode": "geometry_plant",
            "contact_found": False,
            "stable_full_reach": stable_full_reach,
            "soft_support_legs": last_support_legs,
            "present_knee_deg": round(last_knee_now, 2),
            "searched_hip_deg": SEARCH_HIP_DEG,
            "searched_knee_deg": SEARCH_KNEE_DEG,
        }

    # Sample contact pose.
    time.sleep(0.4)
    hips: list[float] = []
    knees: list[float] = []
    for _ in range(6):
        for leg in range(6):
            fh = _fb(leg * 3 + 1)
            fk = _fb(leg * 3 + 2)
            if fh:
                hips.append(float(fh["deg"]))
            if fk:
                knees.append(float(fk["deg"]))
        time.sleep(0.05)
    hip_c = _median(hips, TARGET_HIP_DEG)
    knee_c = _median(knees, contact_knee)
    z_c = foot_z_mm(hip_c, knee_c)
    # Body-frame foot z is more negative when the chassis is higher above a
    # fixed floor.  After contact, raising the chassis by ``clearance_mm``
    # means the planted feet end up farther below the body.
    z_target = z_c - clearance_mm
    knee_up = knee_for_foot_z(TARGET_HIP_DEG, z_target)
    if knee_up is None:
        # Fallback: knock ~25° off contact knee.
        knee_up = max(START_KNEE_DEG, knee_c - 25.0)
    knee_up = max(START_KNEE_DEG, min(SEARCH_KNEE_DEG, float(knee_up)))

    _progress(f"raise ~{clearance_mm:.0f} mm → hip {TARGET_HIP_DEG:.0f}° / "
              f"knee {knee_up:.0f}° (was {knee_c:.0f}°)")
    tall = _pose(TARGET_HIP_DEG, knee_up)
    if not ease_to_pose(bus, tall, abort_check=abort_check,
                        seconds=CLEARANCE_SECONDS, label="geo_raise"):
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant",
                "contact_found": True}

    # Capture full 18-joint snapshot.
    time.sleep(0.5)
    samples: list[list[float]] = []
    for _ in range(8):
        row = []
        ok_row = True
        for j in range(N_JOINTS):
            d = bus.read_position_deg(j)
            if d is None:
                ok_row = False
                break
            row.append(float(d))
        if ok_row:
            samples.append(row)
        time.sleep(0.05)
    if not samples:
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "error": "failed to read joints after raise",
                "mode": "geometry_plant"}

    # Median per joint.
    q_deg = []
    for j in range(N_JOINTS):
        q_deg.append(_median([s[j] for s in samples], samples[0][j]))
    hip_med = _median([q_deg[i] for i in range(1, 18, 3)], TARGET_HIP_DEG)
    knee_med = _median([q_deg[i] for i in range(2, 18, 3)], knee_up)

    z_plant = foot_z_mm(hip_med, knee_med)
    body_raise_mm = z_c - z_plant
    min_raise_mm = max(8.0, 0.45 * clearance_mm)
    if body_raise_mm < min_raise_mm:
        _set_torque_limit(bus, live, 1000)
        _hold_here(bus, live)
        err = (f"raise solve too shallow ({body_raise_mm:.1f} mm; "
               f"wanted ~{clearance_mm:.0f} mm)")
        _progress(err + " — not saving")
        return {
            "ok": False,
            "error": err,
            "mode": "geometry_plant",
            "contact_found": True,
            "contact_knee_deg": round(knee_c, 2),
            "hip_deg": round(hip_med, 2),
            "knee_deg": round(knee_med, 2),
            "clearance_mm": clearance_mm,
            "body_raise_mm": round(body_raise_mm, 2),
            "contact_joints": contact_joints,
        }

    path = save_plant_pose(
        hip_med, knee_med,
        extra={
            "joints_deg": [round(x, 3) for x in q_deg],
            "source": "geometry_plant",
            "contact_found": True,
            "contact_knee_deg": round(knee_c, 3),
            "clearance_mm": clearance_mm,
            "body_raise_mm": round(body_raise_mm, 2),
            "contact_joints": contact_joints,
            "foot_z_contact_mm": round(z_c, 2),
            "foot_z_plant_mm": round(z_plant, 2),
            "target_hip_deg": TARGET_HIP_DEG,
            "target_knee_deg": TARGET_KNEE_DEG,
        },
    )
    _set_torque_limit(bus, live, 1000)
    _hold_here(bus, live)
    _progress(f"saved plant → {path}")
    return {
        "ok": True,
        "mode": "geometry_plant",
        "contact_found": True,
        "contact_knee_deg": round(knee_c, 2),
        "hip_deg": round(hip_med, 2),
        "knee_deg": round(knee_med, 2),
        "clearance_mm": clearance_mm,
        "body_raise_mm": round(body_raise_mm, 2),
        "joints_deg": [round(x, 3) for x in q_deg],
        "pose": standing_pose_degrees(),
        "path": str(path),
        "contact_joints": contact_joints,
        "msg": (f"plant hip {hip_med:+.1f}° / knee {knee_med:+.1f}° "
                f"(~{body_raise_mm:.0f} mm above contact)"),
    }
