#!/usr/bin/env python3
"""rideable_v2 design-consistency regression check (run by `make check`).

Like rideable_v1, this design is a markdown draft, so its "tests" are
numeric: re-derive the loads / torques / belt tensions / lock sizing /
power / cost figures from ``design_spec.yaml`` (+ the solved leg geometry)
and FAIL if anything drifts — specifically the failure modes v2's own
review corrected in the original brief and must not regress:

  * a mass budget that quietly gets optimistic (the hip margin dies at
    ~15 N·m per 10 kg dry);
  * "peak is comfortable" claims — the hip peak margin is THIN (1.16x)
    and must stay explicit;
  * standing-on-motor-current — parked hip static EXCEEDS continuous, so
    the 12 parking-pin locks are mandatory, not optional (and COTS
    friction brakes at this torque are 4-8 kg each — do not quietly
    reintroduce them without re-running the mass budget);
  * a joint peak the belt cannot transmit (tension vs the
    catalog-anchored allowable, >= 2x margin);
  * swing speed that doesn't fit under the no-load speed;
  * a 24 V rail too small for 12 held lock solenoids;
  * headline doc numbers drifting from the spec.

Run:
    ./run.sh hexapod_walker/rideable_v2/tools/design_consistency_check.py
"""

from __future__ import annotations

import json
import math
import os
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
    act = spec["actuator"]
    belt = spec["belt_stage"]
    lock = spec["load_hold"]
    kin = spec["leg_kinematics"]
    cost = spec["cost_rollup_usd"]

    # ---- loads / masses -----------------------------------------------------
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
          f"spec {basis['design_per_leg_foot_load_kN']} vs {per_leg * sf:.2f}")

    rows = [v for k, v in budget.items()
            if k not in ("subtotal", "contingency", "total_dry")]
    check(close(sum(rows), budget["subtotal"], 0.005),
          "mass-budget rows sum to subtotal",
          f"{sum(rows):.1f} vs {budget['subtotal']}")
    check(close(budget["subtotal"] + budget["contingency"],
                budget["total_dry"], 0.005),
          "subtotal + contingency = total dry")
    check(budget["total_dry"] == dry,
          "mass budget total = design-basis dry mass",
          f"{budget['total_dry']} vs {dry}")
    check(close(budget["actuators"], act["qty"] * act["mass_kg"], 0.02),
          "actuator mass row = qty x unit mass",
          f"{budget['actuators']} vs {act['qty']}x{act['mass_kg']}")
    check(close(budget["load_holds"], lock["qty"] * lock["mass_kg"], 0.05),
          "load-hold mass row = qty x unit mass",
          f"{budget['load_holds']} vs {lock['qty']}x{lock['mass_kg']}")

    # ---- moment arms from the SOLVED leg geometry ---------------------------
    print("Moment arms (vs solved IK geometry):")
    out_m = kin["foot_target_under_hip_mm"]["out"] / 1000.0
    down_m = kin["foot_target_under_hip_mm"]["down"] / 1000.0
    L1 = kin["femur_length_mm"] / 1000.0
    L2 = kin["tibia_length_mm"] / 1000.0
    d = math.hypot(out_m, down_m)
    check(abs(L1 - L2) < d < L1 + L2, "tucked foot target is reachable",
          f"d={d:.3f} vs [{abs(L1-L2):.2f}, {L1+L2:.2f}]")
    phi = math.atan2(-down_m, out_m)
    a = math.acos(max(-1, min(1, (d * d + L1 * L1 - L2 * L2) / (2 * d * L1))))
    femur_ang = phi + a  # knee-up branch
    knee_x = L1 * math.cos(femur_ang)
    knee_arm_static = abs(knee_x - out_m)
    hip_arm = basis["hip_moment_arm_m"]
    knee_arm = basis["knee_moment_arm_m"]
    check(close(hip_arm["static"], out_m, 0.02),
          "hip static arm = tucked foot-out distance",
          f"spec {hip_arm['static']} vs {out_m:.3f} m")
    check(close(knee_arm["static"], knee_arm_static, 0.1),
          "knee static arm matches IK geometry",
          f"spec {knee_arm['static']} vs {knee_arm_static:.3f} m")
    exc = basis["stride_excursion_m"]
    check(close(hip_arm["worst_in_stride"], hip_arm["static"] + exc, 0.02),
          "hip worst arm = static + stride excursion")
    check(close(knee_arm["worst_in_stride"], knee_arm["static"] + exc, 0.05),
          "knee worst arm = static + stride excursion")
    ring = kin["hip_yaw_ring_radius_mm"]
    coxa = kin["coxa_length_mm"]
    check(kin["foot_stance_radius_mm"] ==
          ring + coxa + kin["foot_target_under_hip_mm"]["out"],
          "foot stance radius = ring + coxa + foot-out")

    # ---- rollover -----------------------------------------------------------
    print("Rollover (CoM height vs support polygon):")
    r_foot = kin["foot_stance_radius_mm"] / 1000.0
    h = basis["com_height_m"]
    parked = math.degrees(math.atan((r_foot * math.cos(math.pi / 6)) / h))
    tripod = math.degrees(math.atan((r_foot / 2.0) / h))
    check(close(basis["rollover_deg"]["parked_6_feet"], parked, 0.05),
          "parked rollover from hexagon inradius",
          f"spec {basis['rollover_deg']['parked_6_feet']} vs {parked:.1f} deg")
    check(close(basis["rollover_deg"]["tripod"], tripod, 0.05),
          "tripod rollover from triangle inradius",
          f"spec {basis['rollover_deg']['tripod']} vs {tripod:.1f} deg")
    check(basis["rollover_deg"]["tripod"] >= 25,
          "tripod rollover >= 25 deg (ride-gait floor)")

    # ---- torques ------------------------------------------------------------
    print("Torques:")
    nom = basis["nominal_per_leg_foot_load_kN"]
    hip_t = basis["hip_pitch_torque_kNm"]
    knee_t = basis["knee_torque_kNm"]
    check(close(hip_t["static_parked"], nom * hip_arm["static"], 0.03),
          "hip parked static = load x static arm",
          f"spec {hip_t['static_parked']} vs {nom * hip_arm['static']:.3f}")
    check(close(hip_t["worst_in_stride"],
                nom * hip_arm["worst_in_stride"], 0.03),
          "hip worst-in-stride = load x worst arm",
          f"spec {hip_t['worst_in_stride']} "
          f"vs {nom * hip_arm['worst_in_stride']:.3f}")
    check(hip_t["design"] >= hip_t["worst_in_stride"],
          "hip design >= worst-in-stride")
    check(knee_t["holding_nominal"] >= nom * knee_arm["static"] * 0.95,
          "knee nominal hold covers load x static arm")
    check(close(knee_t["holding_worst"],
                nom * knee_arm["worst_in_stride"], 0.05),
          "knee worst hold = load x worst arm",
          f"spec {knee_t['holding_worst']} "
          f"vs {nom * knee_arm['worst_in_stride']:.3f}")
    check(knee_t["design"] >= knee_t["holding_worst"] * 1.3,
          "knee design >= 1.3x worst hold (uneven-share margin)")

    # ---- drivetrain: belt stages, margins, speeds ---------------------------
    print("Drivetrain (per joint):")
    eta = belt["efficiency"]
    check(0.85 <= eta <= 0.90, "belt efficiency in the briefed 85-90% band")
    design_Nm = {"hip_pitch": hip_t["design"] * 1000.0,
                 "knee": knee_t["design"] * 1000.0,
                 "hip_yaw": basis["hip_yaw_torque_kNm"]["max"] * 1000.0}
    min_margin = {"hip_pitch": 1.15, "knee": 1.4, "hip_yaw": 2.0}
    for name, j in belt["joints"].items():
        ratio = j["ratio"]
        peak = act["peak_torque_Nm"] * ratio * eta
        cont = act["rated_torque_Nm"] * ratio * eta
        check(close(j["joint_peak_Nm"], peak, 0.01),
              f"{name}: joint peak = 120 x {ratio} x {eta}",
              f"spec {j['joint_peak_Nm']} vs {peak:.0f} N·m")
        check(close(j["joint_cont_Nm"], cont, 0.01),
              f"{name}: joint continuous = 48 x {ratio} x {eta}",
              f"spec {j['joint_cont_Nm']} vs {cont:.0f} N·m")
        margin = j["joint_peak_Nm"] / design_Nm[name]
        check(margin >= min_margin[name],
              f"{name}: peak margin >= {min_margin[name]}x over design",
              f"{margin:.2f}x ({j['joint_peak_Nm']} vs {design_Nm[name]:.0f})")
        big_T = int(j["teeth"].split("→")[1].strip().rstrip("T"))
        pd = big_T * 8.0 / math.pi
        check(close(j["driven_pd_mm"], pd, 0.005),
              f"{name}: driven pulley PD = teeth x 8mm / pi",
              f"spec {j['driven_pd_mm']} vs {pd:.1f} mm")
        tension = j["joint_peak_Nm"] / (j["driven_pd_mm"] / 2000.0) / 1000.0
        check(close(j["peak_belt_tension_kN"], tension, 0.02),
              f"{name}: peak belt tension = peak / pitch radius",
              f"spec {j['peak_belt_tension_kN']} vs {tension:.2f} kN")
        check(tension * 2.0 <= belt["allowable_working_tension_kN"],
              f"{name}: >= 2x margin vs catalog-anchored belt allowable",
              f"{tension:.2f} kN vs allowable "
              f"{belt['allowable_working_tension_kN']} kN")
        rated = act["rated_speed_rpm_48V"] / ratio * 6.0
        noload = act["no_load_speed_rpm_48V"] / ratio * 6.0
        check(close(j["joint_speed_deg_s_rated"], rated, 0.02),
              f"{name}: rated joint speed", f"{rated:.0f} deg/s")
        check(close(j["joint_speed_deg_s_no_load"], noload, 0.02),
              f"{name}: no-load joint speed", f"{noload:.0f} deg/s")

    hip = belt["joints"]["hip_pitch"]
    check(basis["gait_stance_speed_deg_s"]
          <= 0.5 * hip["joint_speed_deg_s_rated"],
          "loaded stance sweep <= 50% of hip rated speed",
          f"{basis['gait_stance_speed_deg_s']} vs {hip['joint_speed_deg_s_rated']}")
    check(basis["gait_swing_speed_deg_s"]
          <= 0.9 * hip["joint_speed_deg_s_no_load"],
          "unloaded swing fits under 90% of hip no-load speed",
          f"{basis['gait_swing_speed_deg_s']} "
          f"vs 0.9x{hip['joint_speed_deg_s_no_load']}")

    # ---- gait thermal: hip RMS torque vs continuous rating -------------------
    # Stance hip torque = per-leg load x arm, arm sweeping static +- the stride
    # excursion; RMS over the sweep, then scaled by stance duty.  Tripod
    # (3 down, 50% duty) EXCEEDS hip continuous — that is a design property
    # the docs must state (tripod = maneuvering gait); sustained cruise is
    # ripple/wave (5 down, 5/6 duty), which must fit with margin.
    print("Gait thermal (hip):")
    exc = basis["stride_excursion_m"]
    a0, a1 = hip_arm["static"] - exc, hip_arm["static"] + exc
    arm_rms = math.sqrt((a0 * a0 + a0 * a1 + a1 * a1) / 3.0)
    total_kN = basis["ground_weight_kN"]
    hip_cont = hip["joint_cont_Nm"]
    tripod_rms = total_kN / 3.0 * 1000.0 * arm_rms * math.sqrt(0.5)
    ripple_rms = total_kN / 5.0 * 1000.0 * arm_rms * math.sqrt(5.0 / 6.0)
    check(tripod_rms > hip_cont,
          "tripod-walk hip RMS EXCEEDS continuous (why tripod is a "
          "maneuvering gait, not cruise — if this ever passes thermally, "
          "re-review the mass budget, don't delete the gait rule)",
          f"{tripod_rms:.0f} vs cont {hip_cont} N·m")
    check(ripple_rms <= 0.85 * hip_cont,
          "ripple/wave cruise hip RMS <= 85% of continuous",
          f"{ripple_rms:.0f} vs 0.85x{hip_cont} N·m")

    # ---- standing thermal → parking locks are mandatory ----------------------
    print("Load holding (the v1 lesson, joint-side parking pins):")
    parked_hip = hip_t["static_parked"] * 1000.0
    check(parked_hip > 0.85 * hip["joint_cont_Nm"],
          "parked hip static is NOT holdable on motor current "
          "(this is WHY the locks exist — if this ever passes thermally, "
          "re-review, don't delete the locks)",
          f"{parked_hip:.0f} vs cont {hip['joint_cont_Nm']} N·m")
    check(lock["qty"] == 12, "12 parking locks (hips AND knees mandatory)",
          f"qty {lock['qty']}")
    check(lock["rated_hold_Nm"] >= 1.5 * design_Nm["hip_pitch"],
          "lock rated hold >= 1.5x hip design torque",
          f"{lock['rated_hold_Nm']} vs 1.5x{design_Nm['hip_pitch']:.0f} N·m")
    # Pin double-shear capacity at the engage radius (hardened dowel,
    # conservative tau ~ 400 MPa) must dwarf the rated hold.
    pin_area_mm2 = math.pi / 4.0 * lock["pin_dia_mm"] ** 2
    shear_cap_Nm = 2 * pin_area_mm2 * 400.0 * lock["engage_radius_mm"] / 1000.0
    check(shear_cap_Nm >= 3.0 * lock["rated_hold_Nm"],
          "pin double-shear capacity >= 3x rated hold",
          f"{shear_cap_Nm:.0f} vs 3x{lock['rated_hold_Nm']} N·m")
    check(lock["max_backdrive_to_engage_deg"] <= lock["hole_pitch_deg"],
          "power-loss backdrive to engage <= one hole pitch",
          f"{lock['max_backdrive_to_engage_deg']} vs {lock['hole_pitch_deg']} deg")

    # ---- power --------------------------------------------------------------
    print("Power:")
    check(close(power["pack_kWh"],
                power["bus_V_nom"] * power["pack_Ah"] / 1000.0, 0.02),
          "pack energy = V_nom x Ah",
          f"{power['pack_kWh']} vs "
          f"{power['bus_V_nom'] * power['pack_Ah'] / 1000.0:.2f} kWh")
    coil_W = power["lock_solenoids"] * power["lock_solenoid_W_hold"]
    check(power["rail_24V_W"] >= coil_W + 50,
          "24V rail covers 12 held lock solenoids + logic",
          f"{power['rail_24V_W']} W vs {coil_W}+50 W")
    check(power["main_fuse_A"] >= power["standup_transient_A"][1],
          "main fuse above the stand-up transient")
    branch_rated = 3 * act["rated_current_A"]
    check(power["branch_fuse_A"] >= 1.25 * branch_rated,
          "leg branch fuse >= 125% of 3 rated actuator currents",
          f"{power['branch_fuse_A']} A vs 1.25x{branch_rated} A")
    check(power["branch_count"] == 6, "six fused leg branches")
    check(power["pack_kWh"] / power["cruise_kW"] >= 1.5,
          "pack >= 1.5 h theoretical cruise",
          f"{power['pack_kWh'] / power['cruise_kW']:.1f} h")

    # ---- cost roll-up -------------------------------------------------------
    print("Cost roll-up:")
    check(cost["actuators"] == act["qty"] * act["unit_usd"],
          "actuator bucket = qty x unit price",
          f"{cost['actuators']} vs {act['qty']}x{act['unit_usd']}")
    dt = (cost["actuators"] + cost["belt_stages"]
          + cost["joint_bearings"] + cost["load_holds"])
    check(cost["drivetrain_subtotal"] == dt,
          "drivetrain subtotal sums", f"{cost['drivetrain_subtotal']} vs {dt}")
    full = dt + sum(cost[k] for k in
                    ("legs", "feet", "chassis_rider", "battery_power",
                     "electronics_sensors", "hardware_misc", "consumables"))
    check(cost["full_build"] == full,
          "full-build total sums", f"{cost['full_build']} vs {full}")

    # ---- chain variant (CHAIN_VARIANT.md): Carvera-manufacturable drive -----
    # Same ratios as the belt baseline, ANSI #40-2 duplex, driven sprockets
    # as flat plates that must fit the Carvera's 240 mm Y axis.
    print("Chain variant (CHAIN_VARIANT.md):")
    CH_P = 12.7                  # ANSI 40 pitch, mm
    CH_UTS_KN = 27.8             # duplex ultimate (the v1/BOM figure)
    CH_ETA = 0.95                # lubricated roller chain
    CARVERA_Y_MM = 240.0
    chain_joints = {             # teeth (driver, driven), even link count
        "hip_pitch": (13, 52, 68),
        "knee": (14, 42, 62),
        "hip_yaw": (18, 36, 50),
    }
    for name, (n1, n2, links) in chain_joints.items():
        ratio = belt["joints"][name]["ratio"]
        check(n2 / n1 == ratio, f"chain {name}: {n1}T→{n2}T keeps ratio {ratio}")
        check(links % 2 == 0, f"chain {name}: even link count (no offset link)",
              f"{links} links")
        pd = CH_P / math.sin(math.pi / n2)
        od = CH_P * (0.6 + 1.0 / math.tan(math.pi / n2))
        check(od < CARVERA_Y_MM,
              f"chain {name}: driven OD fits the Carvera envelope",
              f"OD {od:.1f} vs {CARVERA_Y_MM:.0f} mm")
        peak_Nm = act["peak_torque_Nm"] * ratio * CH_ETA
        tension_kN = peak_Nm / (pd / 2000.0) / 1000.0
        check(tension_kN <= CH_UTS_KN / 6.0,
              f"chain {name}: peak tension within slow-drive UTS/6",
              f"{tension_kN:.2f} vs {CH_UTS_KN / 6.0:.2f} kN")
    chain_hip_peak = act["peak_torque_Nm"] * 4 * CH_ETA
    check(chain_hip_peak / design_Nm["hip_pitch"] >= 1.25,
          "chain variant: hip peak margin >= 1.25x (efficiency win)",
          f"{chain_hip_peak / design_Nm['hip_pitch']:.2f}x")
    chain_hip_cont = act["rated_torque_Nm"] * 4 * CH_ETA
    check(tripod_rms <= chain_hip_cont,
          "chain variant: tripod hip RMS fits the raised continuous rating",
          f"{tripod_rms:.0f} vs {chain_hip_cont:.0f} N·m")

    # ---- doc drift: headline numbers must appear verbatim -------------------
    print("Docs quote the spec numbers:")
    doc_expectations = {
        "README.md": ["~242 kg", "~165 kg", "1.19 kN", "418 N·m",
                      "No rider until", "parking pin", "ripple/wave"],
        "DRIVETRAIN.md": ["418 N·m", "Poly Chain", "3.28 kN", "parking pin",
                          "mandatory", "pilot bearing", "ripple/wave",
                          "r = 90 mm"],
        "STRUCTURE.md": ["1.19 kN", "535 N·m", "6061-T6", "Single-shear",
                         "fea_joint_shaft.py", "fea_leg_nodes.py"],
        "POWER_SYSTEM.md": ["12S", "100 A", "350 W", "30 A", "solenoid"],
        "BOM.md": ["$23,580", "all 12", "$16,020"],
        "PARTS.md": ["8MGT-896-36", "BFK458", "30205", "6905-2RS",
                     "r = 90 mm", "back-to-back"],
        "CHAIN_VARIANT.md": ["#40-2", "13T → 52T", "4.34 kN", "27.8 kN",
                             "217.6 mm", "cush", "Carvera", "r = 90 mm"],
    }
    for doc, needles in doc_expectations.items():
        text = (PROJ / doc).read_text()
        for needle in needles:
            check(needle in text, f"{doc} quotes {needle!r}")

    # ---- optional: the full BuildViz suite (static check + joint sweep) -----
    # `buildviz check` = static geometry (watertight / self-intersection /
    # disconnected bodies / mating contact); `buildviz sweep` = drives the
    # scene's 18-DOF joints[] through their ranges and re-checks interference
    # at every sampled pose (swept_overlap kinds only appear on failure).
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
            check(res.returncode == 0 and summary["fail"] == 0,
                  "buildviz check: zero failing checks",
                  "; ".join(f"{c['kind']}: {c['label']}" for c in fails[:5])
                  or f"{summary['pass']} pass / {summary['warn']} warn")
        except (json.JSONDecodeError, KeyError) as exc:
            check(False, "buildviz check produced a readable report",
                  f"{exc}: {res.stderr[-300:]}")
        res = subprocess.run(
            ["node", str(cli), "sweep", str(PROJ / "full_robot_viz"), "--json"],
            capture_output=True, text=True)
        try:
            report = json.loads(res.stdout)
            results = report["results"]
            overlaps = [p for p in results["pairs"] if p["kind"] == "overlap"]
            check(res.returncode == 0 and not overlaps,
                  "buildviz sweep: no interference anywhere in joint ranges",
                  "; ".join(f"{p['a']['partType']} x {p['b']['partType']}"
                            for p in overlaps[:5])
                  or f"{results['sampleCount']} poses clean")
            # 18 DOFs x >=8 samples each (+ home pose): prove the sweep really
            # exercised every joint, not a degenerate 1-pose run.
            check(results["sampleCount"] >= 18 * 8,
                  "sweep drove all 18 DOFs through their ranges",
                  f"{results['sampleCount']} sampled poses")
        except (json.JSONDecodeError, KeyError) as exc:
            check(False, "buildviz sweep produced a readable report",
                  f"{exc}: {res.stderr[-300:]}")

    # ---- chain-variant assembly stack-up (tools/chain_assembly_check.py) ----
    res = subprocess.run([sys.executable, str(HERE / "chain_assembly_check.py")],
                         capture_output=True, text=True)
    check(res.returncode == 0,
          "chain-variant assembly stack-up: all clearances/fits pass",
          (res.stdout.strip().splitlines()[-1] if res.stdout.strip() else
           res.stderr[-200:]))

    print(f"\n{CHECKS} checks, {len(FAILURES)} failure(s).")
    if FAILURES:
        for f in FAILURES:
            print(f"  ✗ {f}")
        return 1
    print("design_consistency_check: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
