#!/usr/bin/env python3
"""Full-robot BuildViz assembly with the bearing-sandwich joint architecture.

Design intent (Jun 2026 review): every LONG leg segment is supported on
BOTH ends, not cantilevered off the servo horn.

    YAW   joint -> cantilevered (coxa is 25 mm, < the ~50 mm rule -> no
                   passive bearing needed): servo in chassis cradle,
                   dia-20 disc horn drives the coxa.
    HIP   joint -> SANDWICH: servo at the coxa's outboard end drives the
                   femur disc horn; a 688 ball bearing on the far side
                   carries the femur yoke's stub.  Femur = dia-8 CF tube.
    KNEE  joint -> SANDWICH: servo at the femur's knee end drives the
                   tibia; 688 bearing on the far side.  Tibia = dia-8 CF
                   tube ending in the foot.

Kinematics are FROZEN (6 legs, COXA 25 / FEMUR 90 / TIBIA 130, stance
angles, CHASSIS_YAW_OUTPUT_Z pinned) so MuJoCo + the gait stay valid; only
the joint/segment CONSTRUCTION changes.

Joint hardware meshes come from the validated proof
(tools/sts3215_testfit.py + tools/joint_viz_build.py) in the joint-local
frame:  origin = servo back face, +Z = output, output axis at x=OUTPUT_X.

Run:
    python tools/full_robot_viz_build.py
    node /Users/lbiewald/buildviz/bin/buildviz.mjs full_robot_viz --port 5189
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import trimesh

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE))

import hexapod_prototype as HP  # noqa: E402
import sts3215_testfit as TF  # noqa: E402
import joint_viz_build as JV  # noqa: E402  (real servo + disc horn + bearing builders)

OUT_DIR = _HERE.parent / "full_robot_viz"
STL_DIR = OUT_DIR / "stl"

# Joint-local landmark: disc-horn TOP plane on the output axis.
Z_HORN_TOP = TF.BODY_H + TF.PLATE_T + TF.DISC_HORN_H   # 44 mm
OUTPUT_X = TF.OUTPUT_X                                  # 12.5 mm


# --------------------------------------------------------------------------
# Transform helpers
# --------------------------------------------------------------------------
def _Rz(a):
    return HP.rotation_matrix(a, [0, 0, 1])[:3, :3]


def _Ry(a):
    return HP.rotation_matrix(a, [0, 1, 0])[:3, :3]


def _place(mount_world, x_dir, z_dir):
    """4x4 mapping joint-local -> world so that:
        local +X -> x_dir, local +Z -> z_dir (orthonormalized),
        and the local disc-horn-top point (OUTPUT_X, 0, Z_HORN_TOP)
        lands on mount_world.
    """
    z = np.asarray(z_dir, float)
    z /= np.linalg.norm(z)
    x = np.asarray(x_dir, float)
    x = x - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.eye(4)
    R[:3, 0] = x
    R[:3, 1] = y
    R[:3, 2] = z
    anchor = R[:3, :3] @ np.array([OUTPUT_X, 0.0, Z_HORN_TOP])
    M = np.eye(4)
    M[:3, :3] = R[:3, :3]
    M[:3, 3] = np.asarray(mount_world, float) - anchor
    return M


def _cyl_between(p0, p1, radius):
    """A capsule-free cylinder mesh spanning world points p0->p1."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    v = p1 - p0
    length = float(np.linalg.norm(v))
    c = HP._cyl(radius, length)            # along +Z, centred at origin
    z = v / length
    # rotate +Z onto z
    zaxis = np.array([0, 0, 1.0])
    ax = np.cross(zaxis, z)
    if np.linalg.norm(ax) < 1e-9:
        R = np.eye(4) if z[2] > 0 else HP.rotation_matrix(np.pi, [1, 0, 0])
    else:
        ang = np.arccos(np.clip(np.dot(zaxis, z), -1, 1))
        R = HP.rotation_matrix(ang, ax)
    c.apply_transform(R)
    c.apply_translation(0.5 * (p0 + p1))
    return c


# --------------------------------------------------------------------------
# Part meshes (joint-local frame, from the validated proof)
# --------------------------------------------------------------------------
def _servo():
    return JV._servo_body()


def _front_plate_housing():
    """Yaw housing: front mount plate + side walls, NO back bearing."""
    return TF.make_testfit_cradle(back_bearing=False)


def _sandwich_housing():
    """Hip/knee housing: front plate + side walls + 688 back bearing pocket."""
    return TF.make_testfit_cradle(back_bearing=True)


def _disc_horn():
    return JV._disc_horn()


def _bearing():
    return JV._bearing()


def _yoke():
    return TF.make_testfit_yoke()


# --------------------------------------------------------------------------
# Leg assembly
# --------------------------------------------------------------------------
PALETTE = {
    "servo": "#2e2e33",
    "housing": "#ff7f0e",
    "disc_horn": "#b6b6bd",
    "bearing": "#5b6b7a",
    "yoke": "#2ca02c",
    "tube": "#1a1a1a",
    "coxa": "#9467bd",
    "foot": "#8c564b",
    "pin": "#d62728",
}


def _leg_instances(i: int):
    """Return list of (part_type, color, world_mesh) for one leg."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (i + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    outboard = np.array([np.cos(a), np.sin(a), 0.0])
    tangential = np.array([-np.sin(a), np.cos(a), 0.0])
    zhat = np.array([0.0, 0.0, 1.0])

    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)

    out = []

    # ---------------- YAW (cantilevered) ----------------
    yaw_mount = edge_mid + HP.CHASSIS_YAW_OUTPUT_Z * zhat
    M_yaw = _place(yaw_mount, x_dir=outboard, z_dir=zhat)
    for part, fn in (("servo", _servo), ("housing", _front_plate_housing),
                     ("disc_horn", _disc_horn)):
        mesh = fn().copy()
        mesh.apply_transform(M_yaw)
        out.append((part if part != "housing" else "housing",
                    PALETTE["housing" if part == "housing" else part], mesh))

    # ---------------- COXA (printed arm yaw_axis -> hip_axis) ----------------
    hip_local = np.array([HP.COXA_LENGTH, 0.0, HP.COXA_HIP_DROP])
    hip_axis_world = _Rz(a) @ hip_local + yaw_mount
    coxa = _cyl_between(yaw_mount, hip_axis_world, 6.0)
    out.append(("coxa", PALETTE["coxa"], coxa))

    # ---------------- HIP (sandwich) ----------------
    femur_dir = _Rz(a) @ (_Ry(p) @ np.array([1.0, 0.0, 0.0]))
    M_hip = _place(hip_axis_world, x_dir=femur_dir, z_dir=tangential)
    for part, fn in (("servo", _servo), ("housing", _sandwich_housing),
                     ("disc_horn", _disc_horn), ("bearing", _bearing),
                     ("yoke", _yoke)):
        mesh = fn().copy()
        mesh.apply_transform(M_hip)
        out.append((part if part != "housing" else "housing",
                    PALETTE["housing" if part == "housing" else part], mesh))

    # ---------------- FEMUR tube ----------------
    knee_axis_world = hip_axis_world + femur_dir * HP.FEMUR_LENGTH
    # tube offset from the joint axis line by the yoke socket offset (J:
    # socket centre at z=18 vs horn-top z=44 -> -26 along output axis).
    sock_off = -26.0
    f0 = hip_axis_world + tangential * sock_off + femur_dir * 16.0
    f1 = knee_axis_world + tangential * sock_off - femur_dir * 16.0
    out.append(("tube", PALETTE["tube"], _cyl_between(f0, f1, HP.LEG_TUBE_OD / 2.0)))
    # transverse retention pin at the hip-yoke socket mouth
    pf = f0 + femur_dir * HP.LEG_TUBE_PIN_INSET
    out.append(("pin", PALETTE["pin"], _cyl_between(
        pf - tangential * 9.0, pf + tangential * 9.0, HP.LEG_TUBE_PIN_OD / 2.0)))

    # ---------------- KNEE (sandwich) ----------------
    tibia_dir = _Rz(a) @ (_Ry(pt) @ np.array([1.0, 0.0, 0.0]))
    M_knee = _place(knee_axis_world, x_dir=tibia_dir, z_dir=tangential)
    for part, fn in (("servo", _servo), ("housing", _sandwich_housing),
                     ("disc_horn", _disc_horn), ("bearing", _bearing),
                     ("yoke", _yoke)):
        mesh = fn().copy()
        mesh.apply_transform(M_knee)
        out.append((part if part != "housing" else "housing",
                    PALETTE["housing" if part == "housing" else part], mesh))

    # ---------------- TIBIA tube + foot ----------------
    foot_world = knee_axis_world + tibia_dir * HP.TIBIA_LENGTH
    t0 = knee_axis_world + tangential * sock_off + tibia_dir * 16.0
    t1 = foot_world + tangential * sock_off
    out.append(("tube", PALETTE["tube"], _cyl_between(t0, t1, HP.LEG_TUBE_OD / 2.0)))
    # transverse retention pin at the knee-yoke socket mouth
    pt_ = t0 + tibia_dir * HP.LEG_TUBE_PIN_INSET
    out.append(("pin", PALETTE["pin"], _cyl_between(
        pt_ - tangential * 9.0, pt_ + tangential * 9.0, HP.LEG_TUBE_PIN_OD / 2.0)))
    foot = HP._cyl(HP.FOOT_PAD_OD / 2.0, 8.0)
    foot.apply_translation(t1)
    out.append(("foot", PALETTE["foot"], foot))

    return out


def main(single_leg: bool = False) -> None:
    STL_DIR.mkdir(parents=True, exist_ok=True)

    legs = [0] if single_leg else list(range(6))
    raw = []
    for i in legs:
        for part, color, mesh in _leg_instances(i):
            raw.append((f"L{i}", part, color, mesh))

    # chassis plates (existing STLs, OLD yaw cradle is fine for context)
    chassis = []
    for nm, dz in (("chassis_bottom.stl", 0.0),
                   ("chassis_top.stl", HP.CHASSIS_GAP + HP.CHASSIS_PLATE_T)):
        path = HP.STL_DIR + "/" + nm
        if Path(path).is_file():
            m = trimesh.load(path, process=False)
            if isinstance(m, trimesh.Scene):
                m = trimesh.util.concatenate(list(m.geometry.values()))
            m.apply_translation([0, 0, dz])
            chassis.append((nm.replace(".stl", ""), m))

    # lift so feet land near z=0
    zmin = min(m.bounds[0][2] for _, _, _, m in raw)
    lift = -zmin
    meshes, instances, allb = [], [], []
    idx = 0

    def emit(pid, fname, color, role, mesh, leg):
        nonlocal idx
        mm = mesh.copy()
        mm.apply_translation([0, 0, lift])
        mm.export(STL_DIR / fname)
        allb.append(mm.bounds)
        meshes.append({"id": f"stl:{fname[:-4]}", "name": fname, "url": f"stl/{fname}"})
        instances.append({
            "id": f"{idx:03d}-{fname[:-4]}", "meshId": f"stl:{fname[:-4]}",
            "name": f"{pid} {role}", "partType": pid, "role": role,
            "leg": leg, "joint": None, "color": color,
            "transform": [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0],
            "centroid": [float(v) for v in mm.centroid],
        })
        idx += 1

    for leg, part, color, mesh in raw:
        emit(part, f"{leg}_{part}_{idx}.stl", color, f"{leg} {part}", mesh, leg)
    for nm, m in chassis:
        m.apply_translation([0, 0, lift])
        m.export(STL_DIR / f"{nm}.stl")
        allb.append(m.bounds)
        meshes.append({"id": f"stl:{nm}", "name": f"{nm}.stl", "url": f"stl/{nm}.stl"})
        instances.append({
            "id": f"{idx:03d}-{nm}", "meshId": f"stl:{nm}", "name": nm,
            "partType": nm, "role": "chassis", "leg": None, "joint": None,
            "color": "#79b0e1",
            "transform": [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0],
            "centroid": [float(v) for v in m.centroid],
        })
        idx += 1

    b = np.array(allb)
    center = [float(v) for v in (b[:, 0, :].min(0) + b[:, 1, :].max(0)) / 2.0]
    scene = {
        "name": "Hexapod — bearing-sandwich full robot",
        "source": str(OUT_DIR), "designSpecUrl": "design_spec.yaml",
        "units": "mm", "center": center,
        "meshes": meshes, "instances": instances,
    }
    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    print(f"Wrote {OUT_DIR/'scene.json'}  ({len(instances)} instances, lift={lift:.1f})")


if __name__ == "__main__":
    main(single_leg="--single" in sys.argv)
