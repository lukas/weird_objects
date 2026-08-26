"""Single-leg height probe for checking servo angles against tape readings.

This is a read-only bench diagnostic.  Put one leg in a known pose, read the
calibrated hip/knee degrees, then measure two vertical drops:

* hip pitch axis -> foot contact point
* knee pitch axis -> foot contact point

Both drops are positive downward in millimetres.  If the foot is on a flat
table, each drop is just that joint axis height above the table.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE / "urt2_setup", _HERE.parent / "motor_setup"):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

try:
    from feetech_bus import AXIS_LIMITS_DEG
except Exception:  # pragma: no cover - fallback for standalone copying
    AXIS_LIMITS_DEG = {1: (-80.0, 30.0), 2: (-20.0, 150.0)}

try:
    from tripod_gait import FEMUR_MM as DEFAULT_FEMUR_MM
    from tripod_gait import TIBIA_MM as DEFAULT_TIBIA_MM
except Exception:  # pragma: no cover - fallback for standalone copying
    DEFAULT_FEMUR_MM = 90.0
    DEFAULT_TIBIA_MM = 150.0


@dataclass(frozen=True)
class Drops:
    hip_to_foot_mm: float
    knee_to_foot_mm: float
    hip_to_knee_mm: float
    foot_x_from_hip_mm: float
    knee_x_from_hip_mm: float
    tibia_abs_deg: float


@dataclass(frozen=True)
class Candidate:
    source: str
    hip_deg: float | None = None
    knee_deg: float | None = None
    tibia_abs_deg: float | None = None
    angle_error_deg: float | None = None
    note: str = ""


@dataclass(frozen=True)
class StraightLegTouch:
    zero_tip_clearance_mm: float
    hip_deg: float | None
    knee_deg: float
    total_reach_mm: float
    reachable: bool
    note: str


@dataclass(frozen=True)
class HeightProbe:
    femur_mm: float
    tibia_mm: float
    hip_drop_mm: float | None
    knee_drop_mm: float | None
    calibrated_hip_deg: float | None
    calibrated_knee_deg: float | None
    calibrated_drops: Drops | None
    straight_leg_touch: StraightLegTouch | None
    residual_mm: dict[str, float | None]
    candidates: list[Candidate]
    warnings: list[str]


def _round(v: float | None, ndigits: int = 3) -> float | None:
    return None if v is None else round(float(v), ndigits)


def _unique(vals: Iterable[float], *, tol: float = 1e-7) -> list[float]:
    out: list[float] = []
    for v in vals:
        if not any(abs(v - old) <= tol for old in out):
            out.append(float(v))
    return out


def _in_limits(v: float, limits: tuple[float, float], *, slop: float = 1e-7) -> bool:
    lo, hi = limits
    return lo - slop <= v <= hi + slop


def _asin_angle_candidates(drop_mm: float, length_mm: float) -> list[float]:
    """Angles whose vertical projection is ``drop_mm`` for ``length_mm``.

    The two useful branches are theta and 180-theta.  For example, 80° and
    100° from horizontal have the same vertical drop; this is exactly the
    ambiguity a height-only tape measurement should expose.
    """
    if length_mm <= 0.0:
        raise ValueError("link length must be positive")
    ratio = drop_mm / length_mm
    if abs(ratio) > 1.0 + 1e-7:
        return []
    ratio = max(-1.0, min(1.0, ratio))
    a = math.degrees(math.asin(ratio))
    return _unique([a, 180.0 - a])


def predict_drops(
    hip_deg: float,
    knee_deg: float,
    *,
    femur_mm: float = DEFAULT_FEMUR_MM,
    tibia_mm: float = DEFAULT_TIBIA_MM,
) -> Drops:
    """Return vertical drops implied by calibrated hip/knee degrees."""
    hip = math.radians(float(hip_deg))
    tib_abs = math.radians(float(hip_deg) + float(knee_deg))
    hip_to_knee = femur_mm * math.sin(hip)
    knee_to_foot = tibia_mm * math.sin(tib_abs)
    return Drops(
        hip_to_foot_mm=hip_to_knee + knee_to_foot,
        knee_to_foot_mm=knee_to_foot,
        hip_to_knee_mm=hip_to_knee,
        foot_x_from_hip_mm=(
            femur_mm * math.cos(hip) + tibia_mm * math.cos(tib_abs)
        ),
        knee_x_from_hip_mm=femur_mm * math.cos(hip),
        tibia_abs_deg=float(hip_deg) + float(knee_deg),
    )


def straight_leg_touch_hip_for_clearance(
    zero_tip_clearance_mm: float,
    *,
    femur_mm: float = DEFAULT_FEMUR_MM,
    tibia_mm: float = DEFAULT_TIBIA_MM,
    hip_limits: tuple[float, float] = AXIS_LIMITS_DEG[1],
) -> StraightLegTouch:
    """Hip angle that lowers a straight knee=0 leg by ``zero_tip_clearance``.

    With hip=0/knee=0 the foot is horizontally straight out.  Keeping knee at
    zero turns the leg into one lever of length femur+tibia, so contact is:

        clearance = (femur + tibia) * sin(hip)
    """
    reach = float(femur_mm) + float(tibia_mm)
    clearance = float(zero_tip_clearance_mm)
    if reach <= 0.0:
        raise ValueError("combined link length must be positive")
    if abs(clearance) > reach + 1e-7:
        return StraightLegTouch(
            zero_tip_clearance_mm=round(clearance, 3),
            hip_deg=None,
            knee_deg=0.0,
            total_reach_mm=round(reach, 3),
            reachable=False,
            note="clearance exceeds straight-leg vertical reach",
        )
    ratio = max(-1.0, min(1.0, clearance / reach))
    cands = _unique([
        math.degrees(math.asin(ratio)),
        180.0 - math.degrees(math.asin(ratio)),
    ])
    in_range = [h for h in cands if _in_limits(h, hip_limits)]
    if not in_range:
        return StraightLegTouch(
            zero_tip_clearance_mm=round(clearance, 3),
            hip_deg=None,
            knee_deg=0.0,
            total_reach_mm=round(reach, 3),
            reachable=False,
            note=(
                "mathematically reachable, but not within the configured "
                f"hip limits {hip_limits[0]:.0f}..{hip_limits[1]:.0f} deg"
            ),
        )
    return StraightLegTouch(
        zero_tip_clearance_mm=round(clearance, 3),
        hip_deg=round(in_range[0], 3),
        knee_deg=0.0,
        total_reach_mm=round(reach, 3),
        reachable=True,
        note="expected touch angle with knee held at zero",
    )


def solve_hip_from_total_drop(
    hip_drop_mm: float,
    knee_deg: float,
    *,
    femur_mm: float = DEFAULT_FEMUR_MM,
    tibia_mm: float = DEFAULT_TIBIA_MM,
    hip_limits: tuple[float, float] = AXIS_LIMITS_DEG[1],
) -> list[float]:
    """Solve hip from hip->foot vertical drop while holding knee fixed."""
    knee = math.radians(float(knee_deg))
    a = femur_mm + tibia_mm * math.cos(knee)
    b = tibia_mm * math.sin(knee)
    r = math.hypot(a, b)
    if r <= 1e-9:
        return []
    x = float(hip_drop_mm) / r
    if abs(x) > 1.0 + 1e-7:
        return []
    x = max(-1.0, min(1.0, x))
    phi = math.atan2(b, a)
    base = math.asin(x)
    cands = [
        math.degrees(base - phi),
        math.degrees((math.pi - base) - phi),
    ]
    return [
        float(v) for v in _unique(cands)
        if _in_limits(v, hip_limits)
    ]


def diagnose_height_probe(
    *,
    hip_drop_mm: float | None,
    knee_drop_mm: float | None,
    calibrated_hip_deg: float | None = None,
    calibrated_knee_deg: float | None = None,
    zero_tip_clearance_mm: float | None = None,
    femur_mm: float = DEFAULT_FEMUR_MM,
    tibia_mm: float = DEFAULT_TIBIA_MM,
    hip_limits: tuple[float, float] = AXIS_LIMITS_DEG[1],
    knee_limits: tuple[float, float] = AXIS_LIMITS_DEG[2],
) -> HeightProbe:
    """Compare measured drops to the calibrated two-link leg model."""
    warnings: list[str] = []
    candidates: list[Candidate] = []

    calibrated_drops = None
    if calibrated_hip_deg is not None and calibrated_knee_deg is not None:
        calibrated_drops = predict_drops(
            calibrated_hip_deg,
            calibrated_knee_deg,
            femur_mm=femur_mm,
            tibia_mm=tibia_mm,
        )

    straight_leg_touch = None
    if zero_tip_clearance_mm is not None:
        straight_leg_touch = straight_leg_touch_hip_for_clearance(
            zero_tip_clearance_mm,
            femur_mm=femur_mm,
            tibia_mm=tibia_mm,
            hip_limits=hip_limits,
        )

    residual_mm = {
        "hip_to_foot": (
            None if hip_drop_mm is None or calibrated_drops is None
            else float(hip_drop_mm) - calibrated_drops.hip_to_foot_mm
        ),
        "knee_to_foot": (
            None if knee_drop_mm is None or calibrated_drops is None
            else float(knee_drop_mm) - calibrated_drops.knee_to_foot_mm
        ),
        "hip_to_knee": (
            None if hip_drop_mm is None or knee_drop_mm is None
            or calibrated_drops is None
            else (float(hip_drop_mm) - float(knee_drop_mm))
            - calibrated_drops.hip_to_knee_mm
        ),
    }

    if hip_drop_mm is not None and knee_drop_mm is not None:
        femur_drop = float(hip_drop_mm) - float(knee_drop_mm)
        hip_cands = [
            v for v in _asin_angle_candidates(femur_drop, femur_mm)
            if _in_limits(v, hip_limits)
        ]
        tib_abs_cands = _asin_angle_candidates(float(knee_drop_mm), tibia_mm)
        if not hip_cands:
            warnings.append(
                "hip_drop - knee_drop is outside the femur vertical reach"
            )
        if not tib_abs_cands:
            warnings.append("knee_drop is outside the tibia vertical reach")
        for h in hip_cands:
            for tib_abs in tib_abs_cands:
                k = tib_abs - h
                if not _in_limits(k, knee_limits):
                    continue
                err = None
                if calibrated_hip_deg is not None and calibrated_knee_deg is not None:
                    err = abs(h - calibrated_hip_deg) + abs(k - calibrated_knee_deg)
                candidates.append(Candidate(
                    source="measured hip+knee drops",
                    hip_deg=h,
                    knee_deg=k,
                    tibia_abs_deg=tib_abs,
                    angle_error_deg=err,
                    note=(
                        "height-only solution; tibia has acute/supplement "
                        "branch ambiguity"
                    ),
                ))

    if hip_drop_mm is not None and calibrated_knee_deg is not None:
        for h in solve_hip_from_total_drop(
            float(hip_drop_mm),
            calibrated_knee_deg,
            femur_mm=femur_mm,
            tibia_mm=tibia_mm,
            hip_limits=hip_limits,
        ):
            err = None if calibrated_hip_deg is None else abs(h - calibrated_hip_deg)
            candidates.append(Candidate(
                source="hip drop with calibrated knee fixed",
                hip_deg=h,
                knee_deg=float(calibrated_knee_deg),
                tibia_abs_deg=h + float(calibrated_knee_deg),
                angle_error_deg=err,
            ))

    if knee_drop_mm is not None and calibrated_hip_deg is not None:
        for tib_abs in _asin_angle_candidates(float(knee_drop_mm), tibia_mm):
            k = tib_abs - float(calibrated_hip_deg)
            if not _in_limits(k, knee_limits):
                continue
            err = None if calibrated_knee_deg is None else abs(k - calibrated_knee_deg)
            candidates.append(Candidate(
                source="knee drop with calibrated hip fixed",
                hip_deg=float(calibrated_hip_deg),
                knee_deg=k,
                tibia_abs_deg=tib_abs,
                angle_error_deg=err,
                note="two branches can fit the same knee-axis height",
            ))

    if (knee_drop_mm is not None
            and abs(float(knee_drop_mm)) <= tibia_mm + 1e-7):
        tib_branches = _asin_angle_candidates(float(knee_drop_mm), tibia_mm)
        if len(tib_branches) > 1:
            warnings.append(
                "knee-axis height alone cannot distinguish tibia angle "
                f"{tib_branches[0]:.1f}° from {tib_branches[1]:.1f}°"
            )

    candidates.sort(key=lambda c: (
        float("inf") if c.angle_error_deg is None else c.angle_error_deg,
        float("inf") if c.hip_deg is None else abs(c.hip_deg),
        float("inf") if c.knee_deg is None else abs(c.knee_deg),
    ))

    return HeightProbe(
        femur_mm=float(femur_mm),
        tibia_mm=float(tibia_mm),
        hip_drop_mm=_round(hip_drop_mm),
        knee_drop_mm=_round(knee_drop_mm),
        calibrated_hip_deg=_round(calibrated_hip_deg),
        calibrated_knee_deg=_round(calibrated_knee_deg),
        calibrated_drops=calibrated_drops,
        straight_leg_touch=straight_leg_touch,
        residual_mm={k: _round(v) for k, v in residual_mm.items()},
        candidates=[
            Candidate(
                source=c.source,
                hip_deg=_round(c.hip_deg),
                knee_deg=_round(c.knee_deg),
                tibia_abs_deg=_round(c.tibia_abs_deg),
                angle_error_deg=_round(c.angle_error_deg),
                note=c.note,
            )
            for c in candidates
        ],
        warnings=warnings,
    )


def _fmt(v: float | None, suffix: str = "") -> str:
    return "—" if v is None else f"{v:.2f}{suffix}"


def _print_text_report(probe: HeightProbe) -> None:
    print("Single-leg height probe")
    print(f"  links: femur {probe.femur_mm:.1f} mm, tibia {probe.tibia_mm:.1f} mm")
    print(
        "  measured drops: hip->foot "
        f"{_fmt(probe.hip_drop_mm, ' mm')}, knee->foot "
        f"{_fmt(probe.knee_drop_mm, ' mm')}"
    )

    if probe.calibrated_drops is not None:
        d = probe.calibrated_drops
        print(
            "  calibrated angles: hip "
            f"{_fmt(probe.calibrated_hip_deg, '°')}, knee "
            f"{_fmt(probe.calibrated_knee_deg, '°')}, "
            f"tibia abs {d.tibia_abs_deg:.2f}°"
        )
        print(
            "  model drops from calibrated angles: hip->foot "
            f"{d.hip_to_foot_mm:.2f} mm, knee->foot "
            f"{d.knee_to_foot_mm:.2f} mm, hip->knee "
            f"{d.hip_to_knee_mm:.2f} mm"
        )
        print(
            "  residual measured - model: hip->foot "
            f"{_fmt(probe.residual_mm['hip_to_foot'], ' mm')}, "
            "knee->foot "
            f"{_fmt(probe.residual_mm['knee_to_foot'], ' mm')}, "
            "hip->knee "
            f"{_fmt(probe.residual_mm['hip_to_knee'], ' mm')}"
        )

    if probe.straight_leg_touch is not None:
        s = probe.straight_leg_touch
        print(
            "  straight knee=0 touch from zero clearance: "
            f"{s.zero_tip_clearance_mm:.2f} mm drop over "
            f"{s.total_reach_mm:.1f} mm reach"
        )
        if s.reachable:
            print(
                f"    expected contact at hip {s.hip_deg:.2f}°, "
                f"knee {s.knee_deg:.1f}°"
            )
        else:
            print(f"    not reachable: {s.note}")

    if probe.candidates:
        print("  candidate angles from the measured drops:")
        for c in probe.candidates:
            err = "" if c.angle_error_deg is None else (
                f", error vs calibrated {c.angle_error_deg:.2f}°"
            )
            print(
                f"    {c.source}: hip {_fmt(c.hip_deg, '°')}, "
                f"knee {_fmt(c.knee_deg, '°')}, tibia abs "
                f"{_fmt(c.tibia_abs_deg, '°')}{err}"
            )
    else:
        print("  candidate angles from the measured drops: none")

    for w in probe.warnings:
        print(f"  note: {w}")


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Compare one leg's calibrated hip/knee angles to measured "
            "vertical drops from hip and knee axes."
        )
    )
    p.add_argument("--leg", default=None, help="Optional leg label for your notes.")
    p.add_argument("--hip-deg", type=float, default=None,
                   help="Calibrated/readback hip angle in degrees.")
    p.add_argument("--knee-deg", type=float, default=None,
                   help="Calibrated/readback knee angle in degrees.")
    p.add_argument("--hip-drop-mm", type=float, default=None,
                   help="Vertical drop from hip pitch axis to foot contact.")
    p.add_argument("--knee-drop-mm", type=float, default=None,
                   help="Vertical drop from knee pitch axis to foot contact.")
    p.add_argument("--zero-tip-clearance-mm", type=float, default=None,
                   help=(
                       "Foot-tip height above ground at hip=0/knee=0; reports "
                       "the hip angle that touches with knee held straight."
                   ))
    p.add_argument("--femur-mm", type=float, default=DEFAULT_FEMUR_MM)
    p.add_argument("--tibia-mm", type=float, default=DEFAULT_TIBIA_MM)
    p.add_argument("--json", action="store_true", help="Print machine JSON.")
    return p


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.hip_drop_mm is None and args.knee_drop_mm is None:
        if args.zero_tip_clearance_mm is None:
            raise SystemExit(
                "provide --hip-drop-mm, --knee-drop-mm, and/or "
                "--zero-tip-clearance-mm"
            )
    probe = diagnose_height_probe(
        hip_drop_mm=args.hip_drop_mm,
        knee_drop_mm=args.knee_drop_mm,
        calibrated_hip_deg=args.hip_deg,
        calibrated_knee_deg=args.knee_deg,
        zero_tip_clearance_mm=args.zero_tip_clearance_mm,
        femur_mm=args.femur_mm,
        tibia_mm=args.tibia_mm,
    )
    payload = asdict(probe)
    if args.leg is not None:
        payload["leg"] = args.leg
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        if args.leg is not None:
            print(f"Leg: {args.leg}")
        _print_text_report(probe)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
