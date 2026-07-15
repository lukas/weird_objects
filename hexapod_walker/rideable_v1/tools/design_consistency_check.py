#!/usr/bin/env python3
"""rideable_v1 design-consistency regression check (run by `make check`).

The rideable hexapod is a markdown design draft, so its "tests" are numeric:
this script re-derives the load / torque / drivetrain / mass / power numbers
from the machine-readable blocks in ``design_spec.yaml`` (+ the solved leg
geometry) and FAILS if anything drifts out of agreement — the exact failure
modes a feasibility review caught once and must not regress:

  * mass budget rows that don't sum, or optimistic leg/secondary masses;
  * moment-arm bookkeeping (vertical drop used as a horizontal arm);
  * a joint "peak" the chain physically cannot transmit;
  * brakes that can't hold the parked stance, or a hip brake quietly
    dropped from the BOM;
  * a 24 V rail too small to hold 12 brake coils released at once;
  * headline numbers in the .md docs drifting away from the spec.

It also verifies the generated scene declares the full 18-DOF joints[] block
and, with ``--buildviz``, runs the full BuildViz check suite (mesh overlap,
mating contact, watertightness, joint articulation…) and fails on any FAIL.

Run:
    ./run.sh hexapod_walker/rideable_v1/tools/design_consistency_check.py [--buildviz]
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
G = 9.80665

FAILURES: list[str] = []
CHECKS = 0


def check(ok: bool, label: str, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    status = "ok  " if ok else "FAIL"
    print(f"  [{status}] {label}" + (f" — {detail}" if detail else ""))
    if not ok:
        FAILURES.append(f"{label}: {detail}")


def close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol * max(abs(a), abs(b), 1e-9)


def main() -> int:
    spec = yaml.safe_load((PROJ / "design_spec.yaml").read_text())
    basis = spec["design_basis"]
    budget = spec["mass_budget"]
    power = spec["power_budget"]
    sec = spec["secondary_reduction"]
    brake = spec["brake"]
    kin = spec["leg_kinematics"]
    x15 = spec["actuators"]["hip_pitch"]

    # ---- loads --------------------------------------------------------------
    print("Loads / masses:")
    total = basis["total_operating_mass_kg"]
    dry = basis["vehicle_dry_mass_kg"]
    rider = basis["rider_mass_kg"]
    check(total == dry + rider, "total mass = dry + rider",
          f"{total} vs {dry}+{rider}")
    ground_kN = total * G / 1000.0
    check(close(basis["ground_weight_kN"], ground_kN, 0.02),
          "ground weight from total mass",
          f"spec {basis['ground_weight_kN']} vs {ground_kN:.2f} kN")
    per_leg = ground_kN / 3.0
    check(close(basis["nominal_per_leg_foot_load_kN"], per_leg, 0.03),
          "nominal per-leg load = weight / 3 (tripod)",
          f"spec {basis['nominal_per_leg_foot_load_kN']} vs {per_leg:.2f} kN")
    sf = basis["foot_load_safety_factor"]
    check(sf >= 1.4, "foot-load safety factor >= 1.4", f"SF {sf}")
    check(close(basis["design_per_leg_foot_load_kN"], per_leg * sf, 0.03),
          "design foot load = nominal x SF",
          f"spec {basis['design_per_leg_foot_load_kN']} vs {per_leg * sf:.2f} kN")

    rows = [v for k, v in budget.items()
            if k not in ("subtotal", "contingency", "total_dry")]
    check(sum(rows) == budget["subtotal"], "mass-budget rows sum to subtotal",
          f"{sum(rows)} vs {budget['subtotal']}")
    check(budget["subtotal"] + budget["contingency"] == budget["total_dry"],
          "subtotal + contingency = total dry")
    check(budget["total_dry"] == dry, "mass budget total = design-basis dry mass",
          f"{budget['total_dry']} vs {dry}")
    # The two rows the review found 2-3x optimistic — pin them to reality:
    check(budget["legs"] / 6 >= 8,
          "leg mass >= 8 kg each (steel truss reality floor)",
          f"{budget['legs'] / 6:.1f} kg/leg")
    check(budget["secondary_reductions"] / 12 >= 5,
          "secondary mass >= 5 kg/set (72T sprocket + shaft + housing)",
          f"{budget['secondary_reductions'] / 12:.1f} kg/set")

    # ---- moment arms from the SOLVED leg geometry ---------------------------
    print("Moment arms (vs solved IK geometry):")
    out_m = kin["foot_target_under_hip_mm"]["out"] / 1000.0
    down_m = kin["foot_target_under_hip_mm"]["down"] / 1000.0
    L1 = kin["femur_length_mm"] / 1000.0
    L2 = kin["tibia_length_mm"] / 1000.0
    d = math.hypot(out_m, down_m)
    phi = math.atan2(-down_m, out_m)
    a = math.acos(max(-1, min(1, (d * d + L1 * L1 - L2 * L2) / (2 * d * L1))))
    femur_ang = phi + a  # knee-up branch (matches rideable_viz_build.py)
    knee_x = L1 * math.cos(femur_ang)
    knee_arm_static = abs(knee_x - out_m)
    hip_arm = basis["hip_moment_arm_m"]
    knee_arm = basis["knee_moment_arm_m"]
    check(close(hip_arm["static"], out_m, 0.02),
          "hip static arm = tucked foot-out distance",
          f"spec {hip_arm['static']} vs {out_m:.3f} m")
    check(close(knee_arm["static"], knee_arm_static, 0.05),
          "knee static arm matches IK geometry",
          f"spec {knee_arm['static']} vs {knee_arm_static:.3f} m")
    check(hip_arm["worst_in_stride"] >= hip_arm["static"] + 0.1,
          "hip worst arm covers >= 0.1 m stride excursion")

    # ---- torques ------------------------------------------------------------
    print("Torques:")
    nom_load = basis["nominal_per_leg_foot_load_kN"]
    hip_nom_geom = nom_load * hip_arm["worst_in_stride"]
    hip = basis["hip_pitch_torque_kNm"]
    check(hip["nominal"] >= hip_nom_geom * 0.95,
          "hip nominal torque covers load x worst arm",
          f"spec {hip['nominal']} vs geometry {hip_nom_geom:.2f} kN·m")
    check(hip["design"] >= hip_nom_geom * 1.4,
          "hip design torque >= 1.4x geometry-implied",
          f"{hip['design']} vs 1.4x{hip_nom_geom:.2f}={1.4 * hip_nom_geom:.2f}")
    knee_t = basis["knee_torque_kNm"]
    check(knee_t["holding_nominal"] >= nom_load * knee_arm["static"] * 0.95,
          "knee nominal hold covers load x static arm",
          f"spec {knee_t['holding_nominal']} vs {nom_load * knee_arm['static']:.2f}")
    check(knee_t["holding_worst"] >= nom_load * knee_arm["worst_in_stride"] * 0.95,
          "knee worst hold covers load x worst arm",
          f"spec {knee_t['holding_worst']} vs {nom_load * knee_arm['worst_in_stride']:.2f}")
    check(knee_t["brake_bound"] >= knee_t["holding_worst"] * 1.2,
          "brake sizing bound >= 1.2x worst hold",
          f"{knee_t['brake_bound']} vs 1.2x{knee_t['holding_worst']}")

    # ---- drivetrain: chain-limited peak, brake path, speed, thermal ---------
    print("Drivetrain:")
    ratio = sec["ratio"]
    eta = sec["efficiency"]
    pitch_r_m = sec["driven_sprocket_pitch_dia_mm"] / 2000.0
    chain_work_kN = sec["chain_ultimate_kN"] / sec["chain_dynamic_sf"]
    chain_peak_kNm = chain_work_kN * pitch_r_m
    peak = basis["joint_peak_deliverable_kNm"]
    check(peak <= chain_peak_kNm * 1.02,
          "declared joint peak within chain working strength",
          f"spec {peak} vs chain {chain_peak_kNm:.2f} kN·m")
    check(peak >= hip["design"] * 1.25,
          "chain-limited peak >= 1.25x hip design torque",
          f"{peak} vs 1.25x{hip['design']}={1.25 * hip['design']:.2f}")
    cont = x15["rated_torque_Nm"] * ratio * eta
    check(close(basis["joint_cont_Nm"], cont, 0.02),
          "joint continuous = rated x ratio x eta",
          f"spec {basis['joint_cont_Nm']} vs {cont:.0f} N·m")
    # brake reflected torque: the bound at the joint, held on the fast shaft
    refl = knee_t["brake_bound"] * 1000.0 / (ratio * eta)
    check(brake["holding_torque_fast_shaft_Nm"] >= refl * 0.98,
          "brake holds the reflected joint bound",
          f"brake {brake['holding_torque_fast_shaft_Nm']} vs {refl:.0f} N·m")
    check(brake["static_rating_Nm"] >= 1.5 * brake["holding_torque_fast_shaft_Nm"],
          "brake bought at >= 1.5x working rating")
    check(brake["qty"] == 12, "12 brakes (knees AND hips are mandatory)",
          f"qty {brake['qty']}")
    joint_speed = x15["rated_speed_rpm"] / ratio * 6.0  # rpm -> deg/s
    check(joint_speed >= basis["gait_joint_speed_deg_s"] * 1.15,
          "joint speed >= 1.15x gait requirement",
          f"{joint_speed:.0f} vs {basis['gait_joint_speed_deg_s']} °/s")
    # walking stance hold happens on motor current (brakes only engage at rest)
    worst_static = hip_nom_geom * 1000.0
    check(worst_static <= 0.9 * cont,
          "worst walking stance hold <= 90% of continuous (thermal)",
          f"{worst_static:.0f} vs 0.9x{cont:.0f} N·m")

    # ---- power --------------------------------------------------------------
    print("Power:")
    coil_W = power["brake_coils"] * power["brake_coil_W_each"]
    check(power["rail_24V_W"] >= coil_W + 50,
          "24V rail covers 12 held brake coils + logic",
          f"{power['rail_24V_W']} W vs {coil_W}+50 W")
    check(power["main_fuse_A"] >= power["standup_transient_A"][1],
          "main fuse above the stand-up transient",
          f"{power['main_fuse_A']} A vs {power['standup_transient_A'][1]} A")
    check(power["pack_kWh"] / power["cruise_kW"] >= 1.0,
          "pack >= 1 h theoretical cruise",
          f"{power['pack_kWh'] / power['cruise_kW']:.1f} h")

    # ---- generated scene: full 18-DOF joints[] block ------------------------
    print("Scene (full_robot_viz):")
    scene_path = PROJ / "full_robot_viz" / "scene.json"
    scene = json.loads(scene_path.read_text())
    joints = scene.get("joints", [])
    check(len(joints) == 18, "scene declares 18 joints (6 legs x 3 DOF)",
          f"found {len(joints)}")
    check(all(j.get("type") == "revolute" and j.get("instances") and
              j.get("limits") for j in joints),
          "every joint is revolute with instances + limits")
    by_type: dict[str, int] = {}
    for inst in scene["instances"]:
        by_type[inst["partType"]] = by_type.get(inst["partType"], 0) + 1
    for pt, want in (("hip_brake", 6), ("knee_brake", 6),
                     ("hip_secondary", 6), ("knee_secondary", 6)):
        check(by_type.get(pt, 0) == want, f"scene has {want} x {pt}",
              f"found {by_type.get(pt, 0)}")

    # ---- doc drift: headline numbers must appear verbatim -------------------
    print("Docs quote the spec numbers:")
    doc_expectations = {
        "README.md": ["~460 kg", "~360 kg", "2.25 kN", "1.2 kN·m design",
                      "duplex #40", "≤ ~0.45 m"],
        "DRIVETRAIN.md": ["~1.6 kN·m", "duplex #40", "single point of failure",
                          "mandatory"],
        "STRUCTURE.md": ["2.25 kN", "10.7 kN", "~10 kg per leg"],
        "POWER_SYSTEM.md": ["400 W", "~100 A", "~3 kW"],
        "BOM.md": ["duplex #40", "all 12 required", "$29,010"],
    }
    for doc, needles in doc_expectations.items():
        text = (PROJ / doc).read_text()
        for needle in needles:
            check(needle in text, f"{doc} quotes {needle!r}")

    # ---- optional: the full BuildViz check suite (incl. joint checks) -------
    if "--buildviz" in sys.argv:
        print("BuildViz full check suite:")
        buildviz_root = Path(os.environ.get("BUILDVIZ_ROOT",
                                            Path.home() / "buildviz"))
        cli = buildviz_root / "bin" / "buildviz.mjs"
        res = subprocess.run(
            ["node", str(cli), "check", str(PROJ / "full_robot_viz"), "--json"],
            capture_output=True, text=True)
        try:
            report = json.loads(res.stdout)
            summary = report["results"]["checkSummary"]
            fails = [c for c in report["results"]["checks"]
                     if c["status"] == "fail"]
            joint_checks = [c for c in report["results"]["checks"]
                            if c["kind"].startswith("joint_")]
            check(res.returncode == 0 and summary["fail"] == 0,
                  "buildviz check: zero failing checks",
                  "; ".join(f"{c['kind']}: {c['label']}" for c in fails[:5])
                  or f"{summary['pass']} pass / {summary['warn']} warn")
            check(len(joint_checks) >= 18,
                  "joint-articulation checks ran for all DOFs",
                  f"{len(joint_checks)} joint checks")
        except (json.JSONDecodeError, KeyError) as exc:
            check(False, "buildviz check produced a readable report",
                  f"{exc}: {res.stderr[-300:]}")

    print(f"\n{CHECKS} checks, {len(FAILURES)} failure(s).")
    if FAILURES:
        for f in FAILURES:
            print(f"  ✗ {f}")
        return 1
    print("design_consistency_check: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
