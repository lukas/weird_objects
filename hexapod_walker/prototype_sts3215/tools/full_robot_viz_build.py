#!/usr/bin/env python3
"""Full-robot BuildViz scene built from the VERIFIED standing-pose parts.

History: the original scene drew the coxa / femur / tibia as plain
cylinders and dressed each joint with generic test-fit housing/yoke
meshes.  Those proxies visually clipped each other (the straight coxa
cylinder ran through the hip hardware) even though the real printed parts
clear it.

This version instead reuses the EXACT placement the verifier checks for
self-collision -- ``_verify_prototype._build_standing_leg`` (the real
``make_coxa_link`` / ``make_femur_link`` / ``make_tibia_link`` meshes) and
``_verify_prototype._place_servo_bodies`` (the three STS3215 servo
envelopes in their cradles).  That configuration passes
``check_self_collision`` (6.8 mm^3) and ``check_servo_clearance`` (<=165
mm^3), so the viewer now shows a geometry that is provably free of
unexpected pass-throughs.

The verifier builds only leg 0 (apothem direction a = pi/6).  The six legs
are rotationally symmetric about the chassis Z axis, so legs 1-5 are leg 0
rotated by i*60 deg.  Body parts (chassis plates, battery holder,
electronics tray) are placed exactly as ``make_assembly_preview`` does.

Run:
    ./run.sh hexapod_walker/prototype_sts3215/tools/full_robot_viz_build.py
    npx buildviz full_robot_viz --port 5174
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import hexapod_prototype as HP  # noqa: E402
import _verify_prototype as V  # noqa: E402
import fastener_registry as FR  # noqa: E402

OUT_DIR = _HERE.parent / "full_robot_viz"
STL_DIR = OUT_DIR / "stl"
FASTENERS_DIR = _HERE.parent / "fasteners"

# Joint screws to render (the leg "connecting" fasteners).  Bright colours
# for the hip bolts so the coxa<->femur connection stands out.
FASTENER_JOINT_COLOR = {"yaw": "#bdbdbd", "hip": "#ffd000", "knee": "#9ad0ff"}

PALETTE = {
    "coxa_link": "#9467bd",
    "femur_link": "#2ca02c",
    "tibia_link": "#1f77b4",
    "yaw_servo": "#2e2e33", "hip_servo": "#3a3a40", "knee_servo": "#46464d",
    "chassis_bottom": "#79b0e1", "chassis_top": "#5b8fc7",
    "battery_holder": "#d62728", "electronics_tray": "#17becf",
}
ROLE = {
    "coxa_link": "frame", "femur_link": "frame", "tibia_link": "frame",
    "yaw_servo": "motor", "hip_servo": "motor", "knee_servo": "motor",
    "chassis_bottom": "chassis", "chassis_top": "chassis",
    "battery_holder": "electronics", "electronics_tray": "electronics",
}


def _leg0_parts() -> list[tuple[str, trimesh.Trimesh]]:
    """Leg-0 printed links + servo bodies, in the verifier's world frame."""
    links = V._build_standing_leg()          # coxa_link, femur_link, tibia_link
    servos = V._place_servo_bodies()         # yaw_servo, hip_servo, knee_servo
    out: list[tuple[str, trimesh.Trimesh]] = []
    for name, mesh in {**links, **servos}.items():
        out.append((name, mesh))
    return out


def _axis_to_transform(axis, origin) -> np.ndarray:
    """Map mesh-local +Z onto ``axis`` with the mesh origin at ``origin``
    (fastener cache convention: +Z = shaft, origin = head face)."""
    z = np.asarray(axis, float)
    n = float(np.linalg.norm(z))
    z = z / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])
    seed = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = seed - z * float(np.dot(seed, z))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, np.asarray(origin, float)
    return T


def _fastener_instances(chassis_lift: float):
    """Yield (name, leg, joint, color, mesh) for every leg-joint screw."""
    for fi in FR.build_all_fastener_instances():
        if getattr(fi, "is_virtual", False):
            continue
        if fi.joint not in FASTENER_JOINT_COLOR:
            continue
        cache = FASTENERS_DIR / fi.cache_stl
        if not cache.is_file():
            continue
        mesh = trimesh.load(cache, process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
        mesh.apply_transform(_axis_to_transform(fi.axis_world, fi.head_world_xyz))
        mesh.apply_translation([0, 0, chassis_lift])
        name = f"screw_{fi.joint}"
        yield name, fi.leg_index, fi.joint, FASTENER_JOINT_COLOR[fi.joint], mesh


def _body_parts(chassis_lift: float) -> list[tuple[str, trimesh.Trimesh]]:
    bot = HP.make_chassis_bottom()
    bot.apply_translation([0, 0, chassis_lift])
    top = HP.make_chassis_top()
    top.apply_translation([0, 0, chassis_lift + HP.CHASSIS_GAP + HP.CHASSIS_PLATE_T])
    bh = HP.make_battery_holder()
    bh.apply_translation([HP.BATTERY_HOLDER_CENTRE_X, 0,
                          chassis_lift + HP.CHASSIS_PLATE_T / 2.0])
    et = HP.make_electronics_tray()
    et.apply_translation([35.0, 0, chassis_lift + HP.CHASSIS_PLATE_T / 2.0 + 1.0])
    return [("chassis_bottom", bot), ("chassis_top", top),
            ("battery_holder", bh), ("electronics_tray", et)]


def main(single_leg: bool = False) -> None:
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    leg0 = _leg0_parts()

    # Lift so the lowest leg point sits on z = 0.
    z_min = min(float(m.bounds[0][2]) for _n, m in leg0)
    chassis_lift = -z_min

    legs = [0] if single_leg else list(range(6))

    tagged: list[tuple[str, int | None, trimesh.Trimesh]] = []
    for name, mesh in _body_parts(chassis_lift):
        tagged.append((name, None, mesh))

    for i in legs:
        R = rotation_matrix(i * np.pi / 3.0, [0, 0, 1])
        for name, mesh in leg0:
            m = mesh.copy()
            m.apply_transform(R)
            m.apply_translation([0, 0, chassis_lift])
            tagged.append((name, i, m))

    # Leg-joint screws (placed independently by the fastener registry in
    # the SAME per-leg frame as the verifier parts).
    fastener_color: dict[str, str] = {}
    for name, leg, joint, color, mesh in _fastener_instances(chassis_lift):
        if single_leg and leg != 0:
            continue
        fastener_color[name] = color
        tagged.append((name, leg, mesh))

    meshes_json: list[dict] = []
    instances_json: list[dict] = []
    allb: list = []
    for idx, (name, leg, mesh) in enumerate(tagged):
        leg_tag = f"L{leg}_" if leg is not None else "body_"
        fname = f"{leg_tag}{name}_{idx}.stl"
        mesh.export(STL_DIR / fname)
        allb.append(mesh.bounds)
        mid = f"stl:{fname[:-4]}"
        meshes_json.append({"id": mid, "name": fname, "url": f"stl/{fname}"})
        instances_json.append({
            "id": f"{idx:03d}-{name}",
            "meshId": mid,
            "name": f"L{leg} {name}" if leg is not None else name,
            "partType": name,
            "role": "fastener" if name.startswith("screw_") else ROLE.get(name, "part"),
            "leg": leg,
            "joint": name[len("screw_"):] if name.startswith("screw_") else None,
            "color": fastener_color.get(name) or PALETTE.get(name, "#888888"),
            "transform": [1.0, 0, 0, 0, 0, 1.0, 0, 0,
                          0, 0, 1.0, 0, 0, 0, 0, 1.0],
            "centroid": [float(v) for v in mesh.centroid],
        })

    b = np.array(allb)
    center = [float(v) for v in (b[:, 0, :].min(0) + b[:, 1, :].max(0)) / 2.0]
    scene = {
        "name": "Hexapod STS3215 — full robot (verified standing pose)",
        "source": str(OUT_DIR),
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": center,
        "meshes": meshes_json,
        "instances": instances_json,
    }
    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    print(f"Wrote {OUT_DIR/'scene.json'}  ({len(instances_json)} instances, "
          f"lift={chassis_lift:.1f})")


if __name__ == "__main__":
    main(single_leg="--single" in sys.argv)
