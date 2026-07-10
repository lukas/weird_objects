"""Throwaway analysis: hip-pitch vs knee holding torque for prototype_v1.

Headless. Run with:
  PYTHONPATH=/Users/lbiewald/weird_objects \
    ./.venv/bin/python hexapod_walker/prototype_v1/sim/torque_analysis.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import mujoco

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, PROTO_DIR)

import mujoco_prototype as MP  # noqa: E402

G = 9.81
DS3225_STALL_6V = 2.45  # N*m  (~25 kg*cm at 6.0-6.8V)
DS3225_STALL_5V = 2.40  # N*m  (~24.5 kg*cm at 5.0V, per POWER_SYSTEM.md)
# Model is light: 18x DS3225 (~1.1kg) + 3S LiPo (~0.19kg) + prints + Pi/Mega/wiring.
REAL_MASS_EST = 2.5     # kg, best estimate of the built robot (see report)


def build():
    # Flat ground, no terrain, no obstacles -> clean statics.
    model, data, _ = MP.build_world(
        terrain_enabled=False, obstacle_count=0)
    return model, data


def joint_dof(model, name):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(model.jnt_dofadr[j]), int(model.jnt_qposadr[j])


def total_mass(model):
    return float(np.sum(model.body_mass))


def set_pose_by_footpos(model, data, r_planar, foot_z, yaw=0.0):
    """Set qpos + ctrl targets for a symmetric stance where every foot is at
    (r_planar, 0, foot_z) in its yaw frame. Returns (pitch, knee) or None."""
    ik = MP._leg_ik((r_planar, 0.0, foot_z))
    if ik is None:
        return None
    p, k = ik
    # place chassis so feet are ~on the ground; compute foot z below hip.
    # hip (yaw output) height above chassis origin = YAW_OUTPUT_HEIGHT.
    base_z = -MP.YAW_OUTPUT_HEIGHT - foot_z + MP.FOOT_R + 0.010
    data.qpos[0:3] = [0.0, 0.0, base_z]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(6):
        for kind, val in (("yaw", yaw), ("pitch", p), ("knee", k)):
            _, qadr = joint_dof(model, f"L{i}_{kind}")
            data.qpos[qadr] = val
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return p, k


def set_targets_uniform(data, yaw, pitch, knee):
    for i in range(6):
        base = i * 6
        data.ctrl[base + 0] = yaw
        data.ctrl[base + 1] = pitch
        data.ctrl[base + 2] = knee


def settle(model, data, yaw, pitch, knee, seconds=2.5):
    set_targets_uniform(data, yaw, pitch, knee)
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def measure(model, data):
    """After settling, read holding torque at hip-pitch & knee dofs (per leg)."""
    mujoco.mj_forward(model, data)
    hip = np.zeros(6)
    knee = np.zeros(6)
    for i in range(6):
        dof_p, _ = joint_dof(model, f"L{i}_pitch")
        dof_k, _ = joint_dof(model, f"L{i}_knee")
        hip[i] = data.qfrc_actuator[dof_p]
        knee[i] = data.qfrc_actuator[dof_k]
    return hip, knee


def foot_contacts(model, data):
    """Return per-leg (in_contact, foot_xyz, vertical_force_est)."""
    info = []
    for i in range(6):
        fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_tibia")
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"L{i}_foot_site")
        info.append(np.array(data.site_xpos[site]))
    return info


def leg_geometry(model, data):
    """Horizontal distance from hip-pitch axis and knee axis to foot, per leg."""
    hipdist = np.zeros(6)
    kneedist = np.zeros(6)
    footz = np.zeros(6)
    for i in range(6):
        femur = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_femur")
        tibia = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_tibia")
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"L{i}_foot_site")
        hip_p = np.array(data.xpos[femur])
        knee_p = np.array(data.xpos[tibia])
        foot_p = np.array(data.site_xpos[site])
        hipdist[i] = math.hypot(foot_p[0] - hip_p[0], foot_p[1] - hip_p[1])
        kneedist[i] = math.hypot(foot_p[0] - knee_p[0], foot_p[1] - knee_p[1])
        footz[i] = foot_p[2]
    return hipdist, kneedist, footz


def chassis_height(model, data):
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    return float(data.xpos[b][2])


def total_contact_fz(model, data):
    fz = 0.0
    for c in range(data.ncon):
        con = data.contact[c]
        f = np.zeros(6)
        mujoco.mj_contactForce(model, data, c, f)
        # contact frame: force[0] is normal along contact normal (world approx z for floor)
        # rotate into world
        frame = con.frame.reshape(3, 3)
        f_world = frame.T @ f[:3]
        fz += f_world[2]
    return fz


def achieved_angles(model, data):
    p = np.zeros(6)
    k = np.zeros(6)
    for i in range(6):
        _, qp = joint_dof(model, f"L{i}_pitch")
        _, qk = joint_dof(model, f"L{i}_knee")
        p[i] = data.qpos[qp]
        k[i] = data.qpos[qk]
    return p, k


def foot_radial_from_center(model, data):
    out = np.zeros(6)
    for i in range(6):
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"L{i}_foot_site")
        fp = np.array(data.site_xpos[site])
        out[i] = math.hypot(fp[0], fp[1])
    return out


def n_nonfoot_contacts(model, data):
    n = 0
    for c in range(data.ncon):
        con = data.contact[c]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or ""
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or ""
        if "foot" not in g1 and "foot" not in g2:
            n += 1
    return n


def report_case(name, model, data, yaw, pitch, knee, settle_s=2.5):
    settle(model, data, yaw, pitch, knee, seconds=settle_s)
    hip, kn = measure(model, data)
    hipdist, kneedist, footz = leg_geometry(model, data)
    ch = chassis_height(model, data)
    fz = total_contact_fz(model, data)
    ach_p, ach_k = achieved_angles(model, data)
    foot_rad = foot_radial_from_center(model, data)
    nnf = n_nonfoot_contacts(model, data)
    hip_abs = np.abs(hip)
    kn_abs = np.abs(kn)
    # Validity: body not resting on ground, achieved angles ~ target, feet bear weight.
    ang_err = max(abs(ach_p.mean() - pitch), abs(ach_k.mean() - knee))
    valid = (nnf == 0 and ch > 0.012 and ang_err < math.radians(5)
             and abs(fz - total_mass(model) * G) < 0.5)
    tag = "VALID" if valid else "*** INVALID (collapsed/clamped) ***"
    print(f"\n=== {name}   [{tag}] ===")
    print(f"  target pitch={math.degrees(pitch):.1f}deg knee={math.degrees(knee):.1f}deg  "
          f"| achieved pitch={math.degrees(ach_p.mean()):.1f} knee={math.degrees(ach_k.mean()):.1f}deg")
    print(f"  chassis height = {ch*1000:.1f} mm   total contact Fz = {fz:.2f} N "
          f"(weight = {total_mass(model)*G:.2f} N)   ncon={data.ncon} nonfoot_con={nnf}")
    print(f"  mean foot radial-from-center = {foot_rad.mean()*1000:.1f} mm")
    print(f"  mean foot horiz dist from hip axis  = {hipdist.mean()*1000:.1f} mm")
    print(f"  mean foot horiz dist from knee axis = {kneedist.mean()*1000:.1f} mm")
    print(f"  HIP-PITCH holding torque per leg (N*m): "
          + " ".join(f"{v:+.3f}" for v in hip))
    print(f"  KNEE      holding torque per leg (N*m): "
          + " ".join(f"{v:+.3f}" for v in kn))
    print(f"  HIP-PITCH |torque|: mean={hip_abs.mean():.3f}  max={hip_abs.max():.3f} N*m"
          f"  ({hip_abs.max()/DS3225_STALL_6V*100:.0f}% of DS3225 stall)")
    print(f"  KNEE      |torque|: mean={kn_abs.mean():.3f}  max={kn_abs.max():.3f} N*m"
          f"  ({kn_abs.max()/DS3225_STALL_6V*100:.0f}% of DS3225 stall)")
    ratio = hip_abs.max() / max(kn_abs.max(), 1e-9)
    print(f"  --> hip/knee max-torque ratio = {ratio:.2f}x")
    return {
        "name": name, "hip_max": hip_abs.max(), "knee_max": kn_abs.max(),
        "hip_mean": hip_abs.mean(), "knee_mean": kn_abs.mean(),
        "hipdist": hipdist.mean(), "kneedist": kneedist.mean(),
        "chassis_h": ch, "fz": fz, "valid": valid,
        "foot_rad": foot_rad.mean(),
    }


JLIM = {"pitch": (-1.40, 0.52), "knee": (-0.35, 1.40)}


def feasible(p, k):
    return (JLIM["pitch"][0] <= p <= JLIM["pitch"][1]
            and JLIM["knee"][0] <= k <= JLIM["knee"][1])


def main():
    model, data = build()
    tm = total_mass(model)
    print(f"Model: nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody}")
    print(f"TOTAL MODEL MASS = {tm:.3f} kg   (weight = {tm*G:.2f} N)")
    print("Body masses:")
    for b in range(model.nbody):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if model.body_mass[b] > 0:
            print(f"    {nm:14s} {model.body_mass[b]*1000:6.1f} g")

    # Nominal stance angles from the model.
    stance_p = MP.STANCE_FEMUR
    stance_k = MP.STANCE_TIBIA
    print(f"\nStance from model: femur(hip)={math.degrees(stance_p):.1f}deg "
          f"tibia(knee)={math.degrees(stance_k):.1f}deg")

    results = []

    # Firmware stand-up tuck schedule (mm, yaw-frame foot_x vs foot_z below hip).
    foot_neutral_x = (MP.COXA + MP.FEMUR * math.cos(math.radians(-25.0))
                      + MP.TIBIA * math.cos(math.radians(-25.0 + 60.0)))
    foot_neutral_z = (-MP.FEMUR * math.sin(math.radians(-25.0))
                      - MP.TIBIA * math.sin(math.radians(-25.0 + 60.0)))
    tuck_x = MP.STANCE_FOOT_RADIAL_MM * MP.M
    tuck_z = MP.STANCE_FOOT_Z_MM * MP.M

    def stance_foot_x(foot_z):
        if foot_z >= foot_neutral_z:
            return foot_neutral_x
        if foot_z <= tuck_z:
            return tuck_x
        t = (foot_z - foot_neutral_z) / (tuck_z - foot_neutral_z)
        return foot_neutral_x + t * (tuck_x - foot_neutral_x)

    def run(label, foot_x, foot_z):
        ik = MP._leg_ik((foot_x, 0.0, foot_z))
        if ik is None:
            print(f"\n[skip] {label}: IK unreachable (foot_x={foot_x*1000:.0f} z={foot_z*1000:.0f})")
            return
        p, k = ik
        if not feasible(p, k):
            print(f"\n[skip] {label}: pitch={math.degrees(p):.1f} knee={math.degrees(k):.1f}"
                  f" OUTSIDE joint limits (pitch>=-80.2, knee<=80.2)")
            return
        set_pose_by_footpos(model, data, foot_x, foot_z)
        results.append(report_case(label, model, data, 0.0, p, k))

    # (a) STATIC HOLD -- the real firmware planted stance (tucked, tall).
    run("STATIC stance (firmware)", tuck_x, tuck_z)

    # Foot-TUCK sweep at FIXED tall body depth (foot_z = -150) to isolate the
    # effect of pulling the feet in vs out at constant height.
    print("\n----- FOOT-TUCK SWEEP @ fixed tall body (foot_z = -150mm) -----")
    for fx in (0.115, 0.135, 0.150, 0.160):
        run(f"tuck foot_x={int(fx*1000)}mm @-150", fx, -0.150)

    # (b) STAND-UP STAGE sweep: replicate firmware stanceFootX() tuck schedule,
    # stepping body from low crouch (sprawled feet) up to tall stance (tucked).
    print("\n----- STAND-UP STAGES (firmware tuck schedule) low->tall -----")
    for fz_mm in (-60, -80, -100, -120, -150):
        fz = fz_mm * MP.M
        fx = stance_foot_x(fz)
        run(f"stand fz={fz_mm}mm fx={int(fx*1000)}mm", fx, fz)

    # First-principles sanity check for nominal stance.
    print("\n=== FIRST-PRINCIPLES SANITY CHECK (nominal stance) ===")
    per_foot = tm * G / 6.0
    r0 = results[0]
    print(f"  weight per foot (6-leg support) = {per_foot:.2f} N")
    print(f"  predicted hip torque  ~ {per_foot:.2f} N * {r0['hipdist']*1000:.0f} mm "
          f"= {per_foot*r0['hipdist']:.3f} N*m  (sim max {r0['hip_max']:.3f})")
    print(f"  predicted knee torque ~ {per_foot:.2f} N * {r0['kneedist']*1000:.0f} mm "
          f"= {per_foot*r0['kneedist']:.3f} N*m  (sim max {r0['knee_max']:.3f})")

    scale = REAL_MASS_EST / tm
    print(f"\n=== SUMMARY (max across legs; VALID feet-only stances) ===")
    print(f"    model mass {tm:.2f}kg -> real-mass scale x{scale:.2f} (real~{REAL_MASS_EST}kg)")
    print(f"  {'case':26s} {'hipNm':>6s} {'kneeNm':>6s} {'h/k':>5s} | "
          f"{'hipReal':>7s} {'hip%6V':>6s} {'kneeReal':>8s} {'knee%6V':>7s}")
    for r in results:
        hr = r['hip_max'] * scale
        kr = r['knee_max'] * scale
        print(f"  {r['name']:26s} {r['hip_max']:6.3f} {r['knee_max']:6.3f} "
              f"{r['hip_max']/max(r['knee_max'],1e-9):4.2f}x | "
              f"{hr:7.3f} {hr/DS3225_STALL_6V*100:5.0f}% {kr:8.3f} {kr/DS3225_STALL_6V*100:6.0f}%")
    print("\n  (hipReal/kneeReal = torque scaled to real ~2.5kg robot; %6V vs 2.45 N*m stall.)")
    print("  NB: single-leg support (tripod swing / uneven ground) ~2x these numbers.")


if __name__ == "__main__":
    main()
