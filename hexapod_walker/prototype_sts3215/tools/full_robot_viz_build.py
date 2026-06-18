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

# BuildViz serves each build under ``/builds/<id>/``.  The viewer's STLLoader
# does NOT prepend the build base to mesh URLs, so mesh URLs must be ABSOLUTE
# (``/builds/<id>/stl/...``) -- a relative ``stl/...`` URL resolves against the
# page origin, 404s to the SPA's index.html, and the loader then misparses HTML
# as a binary STL ("Invalid typed array length").  This must match the build id
# the build is registered/served under (public/builds/prototype_sts3215).
SCENE_BUILD_ID = "prototype_sts3215"
SCENE_ASSET_BASE = f"/builds/{SCENE_BUILD_ID}/stl"

# Joint screws to render (the leg "connecting" fasteners).  Bright colours
# for the hip bolts so the coxa<->femur connection stands out.
FASTENER_JOINT_COLOR = {"yaw": "#bdbdbd", "hip": "#ffd000", "knee": "#9ad0ff"}

PALETTE = {
    # Each leg is now called out as its INDIVIDUAL printed parts (BOM-correct
    # sandwich: yoke + dia-8 CF tube + bracket/fitting) instead of the merged
    # femur_link / tibia_link proxies.
    "coxa_link": "#9467bd",
    "coxa_yaw_hub": "#9467bd", "coxa_hip_bracket": "#b08fd6",
    "yaw_bearing_lower": "#d4af37", "yaw_bearing_upper": "#ffd966",
    "femur_hip_yoke": "#2ca02c", "femur_knee_bracket": "#7fce5a",
    "tibia_knee_yoke": "#1f77b4", "tibia_foot_fitting": "#17becf",
    "femur_tube": "#2b2b2b", "tibia_tube": "#2b2b2b",   # carbon fibre
    "foot_pad": "#3a3a3a",                               # TPU pad
    "yaw_servo": "#2e2e33", "hip_servo": "#3a3a40", "knee_servo": "#46464d",
    "chassis_bottom": "#79b0e1", "chassis_top": "#5b8fc7",
    "uno_q_tray": "#9467bd", "buck_tray": "#bcbd22",
    "uno_q": "#1b7a3d", "buck_converter": "#b5651d",
    "lipo_battery": "#d62728",
    "hip_clamp_cap": "#4a90d9", "knee_clamp_cap": "#4a90d9",
}
ROLE = {
    "coxa_link": "frame",
    "coxa_yaw_hub": "frame", "coxa_hip_bracket": "frame",
    "yaw_bearing_lower": "bearing", "yaw_bearing_upper": "bearing",
    "femur_hip_yoke": "frame", "femur_knee_bracket": "frame",
    "tibia_knee_yoke": "frame", "tibia_foot_fitting": "frame",
    "femur_tube": "spar", "tibia_tube": "spar", "foot_pad": "frame",
    "yaw_servo": "motor", "hip_servo": "motor", "knee_servo": "motor",
    "chassis_bottom": "chassis", "chassis_top": "chassis",
    "uno_q_tray": "electronics", "buck_tray": "electronics",
    "uno_q": "electronics", "buck_converter": "electronics",
    "lipo_battery": "electronics",
    "hip_clamp_cap": "frame", "knee_clamp_cap": "frame",
}


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


def _leg0_individual_link_parts() -> list[tuple[str, trimesh.Trimesh]]:
    """Leg-0 femur/tibia decomposed into their REAL printed parts (yoke +
    dia-8 CF tube + bracket/fitting) + coxa_link + foot_pad, each placed
    with the verifier's EXACT per-link world transform.

    The femur/tibia sub-parts are built in each link's LOCAL frame exactly
    as ``make_femur_link`` / ``make_tibia_link`` do, then the verifier's
    link transform (``_build_standing_leg``) is applied to each piece.  So
    the union of the pieces occupies the identical space the verifier
    checks for collision -- they stay aligned with the servos/caps (which
    are taken from the same verifier world frame) and provably clear."""
    a = 0.5 * np.pi / 3.0
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    edge = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0]) \
        + HP.CHASSIS_YAW_OUTPUT_Z * np.array([0.0, 0.0, 1.0])
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    hip_local = np.array(HP.COXA_HIP_ANCHOR)
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([HP.FEMUR_LENGTH, 0.0, 0.0])

    # World transform of each link's local frame (mirrors _build_standing_leg).
    Rz = rotation_matrix(a, [0, 0, 1])
    T_coxa = _trans(edge) @ Rz
    T_femur = _trans(edge) @ Rz @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = _trans(edge) @ Rz @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])

    def placed(mesh, T):
        m = mesh.copy()
        m.apply_transform(T)
        return m

    xz = (1, 0, 0), HP.LEG_PITCH_AXIS
    out: list[tuple[str, trimesh.Trimesh]] = []
    # Coxa called out as its TWO printed parts (yaw turntable hub + hip
    # bracket) plus the SPACED 6706 bearing PAIR (visual, NOT printed) so the
    # bearing-supported yaw joint is visible.
    out.append(("coxa_yaw_hub", placed(HP.make_coxa_yaw_hub(), T_coxa)))
    out.append(("coxa_hip_bracket", placed(HP.make_coxa_hip_bracket(), T_coxa)))
    out.append(("yaw_bearing_lower", placed(HP.make_yaw_bearing_lower(), T_coxa)))
    out.append(("yaw_bearing_upper", placed(HP.make_yaw_bearing_upper(), T_coxa)))

    # Femur sub-parts in femur-link-local frame (== make_femur_link).
    Mh = HP._joint_place((0.0, 0.0, 0.0), *xz)
    Mk = HP._joint_place((HP.FEMUR_LENGTH, 0.0, 0.0), *xz)
    hy = HP.make_femur_hip_yoke();     hy.apply_transform(Mh)
    kb = HP.make_femur_knee_bracket(); kb.apply_transform(Mk)
    fa = (Mh @ np.array([HP._YOKE_SPINE_X1, 0.0, HP.JOINT_SOCKET_Z, 1.0]))[:3]
    fb = (Mk @ np.array([-HP.WELL_W / 2.0, 0.0, HP.JOINT_SOCKET_Z, 1.0]))[:3]
    ftube = HP._tube_between(fa, fb, HP.LEG_TUBE_OD / 2.0)
    out.append(("femur_hip_yoke", placed(hy, T_femur)))
    out.append(("femur_knee_bracket", placed(kb, T_femur)))
    out.append(("femur_tube", placed(ftube, T_femur)))

    # Tibia sub-parts in tibia-link-local frame (== make_tibia_link).
    Mk0 = HP._joint_place((0.0, 0.0, 0.0), *xz)
    ky = HP.make_tibia_knee_yoke(); ky.apply_transform(Mk0)
    ta = (Mk0 @ np.array([HP._YOKE_SPINE_X1, 0.0, HP.JOINT_SOCKET_Z, 1.0]))[:3]
    foot_sock = ta + np.array([HP.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    foot_frame = HP._frame(foot_sock, (1, 0, 0), (0, 0, 1))
    ff = HP.make_tibia_foot_fitting(); ff.apply_transform(foot_frame)
    ttube = HP._tube_between(ta, foot_sock, HP.LEG_TUBE_OD / 2.0)
    out.append(("tibia_knee_yoke", placed(ky, T_tibia)))
    out.append(("tibia_tube", placed(ttube, T_tibia)))
    out.append(("tibia_foot_fitting", placed(ff, T_tibia)))

    # Foot pad: hinge at local x=16 on the foot fitting; the foot stays flat
    # on the ground (yaw-only orientation), dropped by FOOT_HINGE_FOOT_Z.
    hinge_w = (T_tibia @ foot_frame @ np.array([16.0, 0.0, 0.0, 1.0]))[:3]
    foot = HP.make_foot_pad()
    foot.apply_transform(rotation_matrix(a, [0, 0, 1]))
    foot.apply_translation([hinge_w[0], hinge_w[1],
                            hinge_w[2] - HP.FOOT_HINGE_FOOT_Z])
    out.append(("foot_pad", foot))
    return out


def _leg0_parts() -> list[tuple[str, trimesh.Trimesh]]:
    """Leg-0 INDIVIDUAL printed parts + servo bodies + clamp caps, all in
    the verifier's leg-0 world frame (apothem a = pi/6).  The femur/tibia
    are decomposed into their real printed sandwich parts so the assembly
    calls out every part instead of the merged link proxies."""
    links = _leg0_individual_link_parts()
    servos = V._place_servo_bodies()         # yaw_servo, hip_servo, knee_servo
    caps = V._place_servo_clamp_caps()       # hip_clamp_cap, knee_clamp_cap
    out: list[tuple[str, trimesh.Trimesh]] = list(links)
    for name, mesh in {**servos, **caps}.items():
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

    # LiPo velcro-strapped to chassis_bottom's top face (no holder).
    from trimesh.creation import box as _box_mesh
    lipo = _box_mesh(extents=(105.0, 35.0, 25.0))
    lipo.apply_translation([HP.BATTERY_HOLDER_CENTRE_X, 0,
                            chassis_lift + HP.CHASSIS_PLATE_T / 2.0 + 25.0 / 2.0])

    # Stacked electronics decks on 4 columns above chassis_top.
    deck_z0 = chassis_lift + HP.CHASSIS_GAP + 1.5 * HP.CHASSIS_PLATE_T
    uno_tray_z = deck_z0 + HP.DECK_LEVEL_1_STANDOFF_H
    buck_tray_z = deck_z0 + HP.DECK_LEVEL_1_STANDOFF_H + HP.DECK_LEVEL_2_STANDOFF_H
    uno_tray = HP.make_uno_q_tray()
    uno_tray.apply_translation([0, 0, uno_tray_z])
    buck_tray = HP.make_buck_tray()
    buck_tray.apply_translation([0, 0, buck_tray_z])
    uno = HP.make_uno_q_visual()
    uno.apply_translation([0, 0, uno_tray_z + HP.DECK_TRAY_T + HP.DECK_STANDOFF_BOSS_H])
    buck = HP.make_buck_converter_visual()
    buck.apply_translation([0, 0, buck_tray_z + HP.DECK_TRAY_T + HP.DECK_STANDOFF_BOSS_H])

    return [("chassis_bottom", bot), ("chassis_top", top),
            ("lipo_battery", lipo),
            ("uno_q_tray", uno_tray), ("buck_tray", buck_tray),
            ("uno_q", uno), ("buck_converter", buck)]


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
        meshes_json.append({"id": mid, "name": fname,
                            "url": f"{SCENE_ASSET_BASE}/{fname}"})
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
