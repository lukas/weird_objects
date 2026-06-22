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
# for the hip bolts so the coxa<->femur connection stands out.  Chassis-level
# / foot fasteners (registry joint == None) fall back to FASTENER_CHASSIS_COLOR.
FASTENER_JOINT_COLOR = {"yaw": "#bdbdbd", "hip": "#ffd000", "knee": "#9ad0ff"}
FASTENER_CHASSIS_COLOR = "#9a9a9a"          # steel hardware (standoffs/foot/deck)
DISC_HORN_COLOR = "#b8b8c0"                  # silver-anodised aluminium disc horn

# Non-printed COTS roles: every instance with one of these roles is hardware we
# BUY (servos, bearings, disc horns, fasteners) and tag ``cots: true`` so the
# viewer can distinguish them from printed bodies.  They live ONLY in this scene
# and are never registered in the printable PART_REGISTRY / trays / BOM.
COTS_ROLES = {"motor", "bearing", "horn", "fastener", "electronics"}

PALETTE = {
    # Each leg is now called out as its INDIVIDUAL printed parts (BOM-correct
    # sandwich: yoke + dia-8 CF tube + bracket/fitting) instead of the merged
    # femur_link / tibia_link proxies.
    "coxa_link": "#9467bd",
    "coxa_yaw_hub": "#9467bd", "coxa_hip_bracket": "#b08fd6",
    "yaw_bearing_cap": "#6b4fa0",
    "yaw_bearing_lower": "#d4af37", "yaw_bearing_upper": "#ffd966",
    "femur_hip_yoke": "#2ca02c", "femur_knee_bracket": "#7fce5a",
    "tibia_knee_yoke": "#1f77b4", "tibia_foot_fitting": "#17becf",
    "femur_tube": "#2b2b2b", "tibia_tube": "#2b2b2b",   # carbon fibre
    "foot_pad": "#3a3a3a",                               # TPU pad
    "disc_horn_yaw": DISC_HORN_COLOR, "disc_horn_hip": DISC_HORN_COLOR,
    "disc_horn_knee": DISC_HORN_COLOR,
    "yaw_servo": "#2e2e33", "hip_servo": "#3a3a40", "knee_servo": "#46464d",
    "chassis_bottom": "#79b0e1", "chassis_bottom_lower": "#5fa0d8",
    "chassis_top": "#5b8fc7",
    "uno_q_tray": "#9467bd", "buck_tray": "#bcbd22",
    "spider_carapace": "#23232a",
    "uno_q": "#1b7a3d", "buck_converter": "#b5651d",
    "lipo_battery": "#d62728",
    "hip_clamp_cap": "#4a90d9", "knee_clamp_cap": "#4a90d9",
}
ROLE = {
    "coxa_link": "frame",
    "coxa_yaw_hub": "frame", "coxa_hip_bracket": "frame",
    "yaw_bearing_cap": "chassis",
    "yaw_bearing_lower": "bearing", "yaw_bearing_upper": "bearing",
    "femur_hip_yoke": "frame", "femur_knee_bracket": "frame",
    "tibia_knee_yoke": "frame", "tibia_foot_fitting": "frame",
    "femur_tube": "spar", "tibia_tube": "spar", "foot_pad": "frame",
    "disc_horn_yaw": "horn", "disc_horn_hip": "horn", "disc_horn_knee": "horn",
    "yaw_servo": "motor", "hip_servo": "motor", "knee_servo": "motor",
    "chassis_bottom": "chassis", "chassis_bottom_lower": "chassis",
    "chassis_top": "chassis",
    "uno_q_tray": "electronics", "buck_tray": "electronics",
    "spider_carapace": "chassis",
    "uno_q": "electronics", "buck_converter": "electronics",
    "lipo_battery": "electronics",
    "hip_clamp_cap": "frame", "knee_clamp_cap": "frame",
}


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


# ---------------------------------------------------------------------------
# BuildViz "checks" sidecar (precomputed, read on scene load)
# ---------------------------------------------------------------------------
#
# BuildViz auto-loads a ``buildviz_checks.json`` sidecar next to ``scene.json``
# into its clickable Checks panel (see /Users/lbiewald/buildviz
# BUILDVIZ_INTEGRATION.md "scene.json Contract" + the ``buildviz check --emit``
# sidecar: generatedAt / build / checksConfig / summary / checks[] / highlights).
# We can't run the (concurrently-edited) buildviz CLI, so we PRECOMPUTE the
# generic geometric ``mesh_overlap`` class here -- the same printed-part
# interpenetration class the offline verifier guards with
# ``check_clamp_cap_interference`` -- and write the sidecar in BuildViz's exact
# schema so the panel surfaces it on load (the publish path is a symlink, so it
# is picked up).
#
# SceneCheck = { id, kind, status: 'pass'|'warn'|'fail', label,
#                instances?: string[], point?: [x,y,z], region?: {min,max} }
# Status colours (CHECK_STATUS_COLOR): fail #ff3b30, warn #f59e0b, pass #22c55e.

CHECK_STATUS_COLOR = {"fail": "#ff3b30", "warn": "#f59e0b", "pass": "#22c55e"}

# Overlap volume (mm^3) at/above which a non-intended printed-part pair is a
# FAIL.  A correct flush mate (clamp cap on its cradle, stacked hub/bracket)
# leaves only voxel-boundary noise well under this; the clamp-cap lip bug this
# guards against was ~560 mm^3.
CHECK_OVERLAP_MM3 = 80.0
CHECK_PITCH_MM = 2.0

# Intended part-type matings (overlap by design): servo bodies seated in their
# cradles, the disc horn clamped on its servo output, the carbon tube epoxied
# into its sockets, and the bolt-together printed stacks that share a flush or
# bonded interface.  These are reported ``pass (allowed)`` rather than failing,
# exactly as BuildViz's ``checksConfig.ignoreOverlapPairs`` intends.  The
# sandwich-joint clamp cap <-> its bracket is DELIBERATELY *not* listed: it must
# mate flush (~0 mm^3), so a real interpenetration there still surfaces as FAIL.
INTENDED_OVERLAP_PAIRS = frozenset(
    frozenset(p) for p in [
        # Servo body fills its cradle / bracket.
        ("chassis_bottom", "yaw_servo"),
        ("chassis_bottom_lower", "yaw_servo"),
        ("coxa_hip_bracket", "hip_servo"),
        ("femur_knee_bracket", "knee_servo"),
        # Disc horn seats on its servo output boss.
        ("yaw_servo", "disc_horn_yaw"),
        ("hip_servo", "disc_horn_hip"),
        ("knee_servo", "disc_horn_knee"),
        # Carbon-fibre spar epoxied into the yoke / fitting sockets.
        ("femur_hip_yoke", "femur_tube"),
        ("femur_knee_bracket", "femur_tube"),
        ("tibia_knee_yoke", "tibia_tube"),
        ("tibia_foot_fitting", "tibia_tube"),
        # Bolt-together printed stacks sharing a flush/bonded interface.
        ("coxa_yaw_hub", "coxa_hip_bracket"),
        ("coxa_yaw_hub", "yaw_bearing_cap"),
        ("coxa_hip_bracket", "yaw_bearing_cap"),
        # LiPo velcro-strapped onto the chassis bottom plate.
        ("chassis_bottom", "lipo_battery"),
    ]
)


def _pair_overlap(a_mesh, b_mesh, *, pitch, skip_below):
    """Voxel-sample the AABB intersection of two WORLD-space meshes.

    Returns ``(volume_mm3, centroid_xyz, region_min, region_max)`` or
    ``(0.0, None, None, None)`` when they do not interpenetrate.  Mirrors the
    generic estimator in tools/buildviz_checks.py / the verifier's
    ``_pair_overlap_volume`` so the numbers line up with the offline gate."""
    lo = np.maximum(a_mesh.bounds[0], b_mesh.bounds[0])
    hi = np.minimum(a_mesh.bounds[1], b_mesh.bounds[1])
    if np.any(hi <= lo):
        return 0.0, None, None, None
    span = hi - lo
    if float(np.prod(span)) <= skip_below:
        return 0.0, None, None, None
    n = np.maximum(2, np.ceil(span / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    try:
        in_a = a_mesh.contains(pts)
    except Exception:
        return 0.0, None, None, None
    if not in_a.any():
        return 0.0, None, None, None
    pts_a = pts[in_a]
    try:
        in_b = b_mesh.contains(pts_a)
    except Exception:
        return 0.0, None, None, None
    n_overlap = int(in_b.sum())
    if n_overlap == 0:
        return 0.0, None, None, None
    voxel_vol = float(np.prod(span / n.astype(float)))
    inside = pts_a[in_b]
    centroid = [float(c) for c in inside.mean(axis=0)]
    rmin = [float(c) for c in inside.min(axis=0)]
    rmax = [float(c) for c in inside.max(axis=0)]
    return n_overlap * voxel_vol, centroid, rmin, rmax


def _build_checks_sidecar(tagged, instances_json):
    """Compute the generic ``mesh_overlap`` + ``scene_meta`` checks over the
    placed scene and return the BuildViz sidecar dict (its exact schema)."""
    # tagged[idx] world mesh is aligned 1:1 with instances_json[idx].
    items = []
    for inst, (name, _leg, mesh) in zip(instances_json, tagged):
        # Fasteners occupy their holes by design (BuildViz suppresses them by
        # default); skip them from the interpenetration scan.
        if inst.get("role") == "fastener":
            continue
        items.append((inst["id"], inst["partType"], mesh))

    checks: list[dict] = []

    # scene_meta: every instance resolves to a mesh + ids are unique (the
    # builder guarantees this, so it is a green hygiene line in the panel).
    ids = [i["id"] for i in instances_json]
    dupes = sorted({x for x in ids if ids.count(x) > 1})
    checks.append({
        "id": "scene_meta-unique-ids",
        "kind": "scene_meta",
        "status": "pass" if not dupes else "fail",
        "label": (f"{len(ids)} instances, ids unique"
                  if not dupes else f"duplicate ids: {dupes}"),
    })

    # mesh_overlap: pairwise interpenetration with an AABB pre-filter.
    overlaps = []
    for i in range(len(items)):
        ida, pta, ma = items[i]
        amin, amax = ma.bounds
        for j in range(i + 1, len(items)):
            idb, ptb, mb = items[j]
            bmin, bmax = mb.bounds
            if np.any(amax < bmin) or np.any(bmax < amin):
                continue
            vol, centroid, rmin, rmax = _pair_overlap(
                ma, mb, pitch=CHECK_PITCH_MM, skip_below=CHECK_OVERLAP_MM3)
            if vol <= CHECK_OVERLAP_MM3:
                continue
            overlaps.append((vol, ida, pta, idb, ptb, centroid, rmin, rmax))

    overlaps.sort(key=lambda r: -r[0])
    for k, (vol, ida, pta, idb, ptb, centroid, rmin, rmax) in enumerate(overlaps):
        allowed = frozenset((pta, ptb)) in INTENDED_OVERLAP_PAIRS
        status = "pass" if allowed else "fail"
        suffix = " (allowed)" if allowed else ""
        rec = {
            "id": f"mesh_overlap-{k}",
            "kind": "mesh_overlap",
            "status": status,
            "label": f"{pta} \u2229 {ptb} = {vol:.0f} mm\u00b3 interpenetration{suffix}",
            "instances": [ida, idb],
        }
        if centroid is not None:
            rec["point"] = centroid
        if rmin is not None and rmax is not None:
            rec["region"] = {"min": rmin, "max": rmax}
        checks.append(rec)

    # summary + highlights, mirroring buildvizChecks.summarizeChecks /
    # checksToHighlights so the panel badge + overlays match a CLI --emit run.
    summary = {"total": len(checks), "pass": 0, "warn": 0, "fail": 0, "byKind": {}}
    for c in checks:
        summary[c["status"]] += 1
        bk = summary["byKind"].setdefault(c["kind"], {"pass": 0, "warn": 0, "fail": 0})
        bk[c["status"]] += 1

    parts: dict[str, dict] = {}
    points: list[dict] = []
    regions: list[dict] = []
    for c in checks:
        if c["status"] == "pass":
            continue
        color = CHECK_STATUS_COLOR[c["status"]]
        for iid in c.get("instances", []):
            parts.setdefault(iid, {"instanceId": iid, "color": color,
                                   "annotation": c["label"]})
        if "region" in c:
            regions.append({"min": c["region"]["min"], "max": c["region"]["max"],
                            "color": color, "annotation": c["label"]})
        if "point" in c:
            pt = {"point": c["point"], "color": color, "annotation": c["label"]}
            if "region" not in c and c.get("instances"):
                pt["instanceId"] = c["instances"][0]
            points.append(pt)

    return {
        "generatedAt": __import__("datetime").datetime.now(
            __import__("datetime").timezone.utc).isoformat(),
        "build": {"id": SCENE_BUILD_ID, "name": "Hexapod STS3215 — full robot"},
        "generatedBy": "full_robot_viz_build.py (precomputed offline)",
        "checksConfig": {
            "overlapMm3": CHECK_OVERLAP_MM3,
            "pitchMm": CHECK_PITCH_MM,
            "ignoreOverlapPairs": sorted(sorted(p) for p in INTENDED_OVERLAP_PAIRS),
        },
        "summary": summary,
        "checks": checks,
        "highlights": {"parts": list(parts.values()), "points": points,
                       "regions": regions},
    }


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
    # yaw_bearing_cap: TOP half of the SPLIT bearing tower, coaxial on the yaw
    # axis (built in coxa-local, same frame as coxa_yaw_hub).
    out.append(("yaw_bearing_cap", placed(HP.make_yaw_bearing_cap(), T_coxa)))
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


def _primitive_bolt(fi) -> trimesh.Trimesh:
    """Fallback capped-cylinder bolt for any fastener lacking a cache STL.

    Mesh-local +Z = shaft, origin = head face (matches the cache convention
    consumed by ``_axis_to_transform``): a flat head disc at z in [0, head_t]
    and a shank running down -Z for ``length_mm``."""
    from trimesh.creation import cylinder as _cyl_mesh
    dia = float(getattr(fi, "nominal_dia_mm", 3.0) or 3.0)
    length = float(getattr(fi, "length_mm", 8.0) or 8.0)
    head_d, head_t = dia * 1.8, max(0.6 * dia, 1.0)
    head = _cyl_mesh(radius=head_d / 2.0, height=head_t)
    head.apply_translation([0, 0, -head_t / 2.0])      # head top at z=0
    shank = _cyl_mesh(radius=dia / 2.0, height=length)
    shank.apply_translation([0, 0, -head_t - length / 2.0])
    bolt = trimesh.util.concatenate([head, shank])
    # Flip so +Z points along the shaft (down into the part) like the caches.
    bolt.apply_transform(rotation_matrix(np.pi, [1, 0, 0]))
    return bolt


def _fastener_instances(chassis_lift: float):
    """Yield (name, leg, joint, color, mesh) for EVERY non-virtual fastener
    in the registry -- leg-joint screws (yaw/hip/knee) AND chassis-level /
    foot / deck hardware (registry ``joint is None``).  COTS, scene-only."""
    for fi in FR.build_all_fastener_instances():
        if getattr(fi, "is_virtual", False):
            continue
        cache = FASTENERS_DIR / fi.cache_stl if fi.cache_stl else None
        if cache is not None and cache.is_file():
            mesh = trimesh.load(cache, process=False)
            if isinstance(mesh, trimesh.Scene):
                mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
        else:
            mesh = _primitive_bolt(fi)
        mesh.apply_transform(_axis_to_transform(fi.axis_world, fi.head_world_xyz))
        mesh.apply_translation([0, 0, chassis_lift])
        joint_key = fi.joint if fi.joint in FASTENER_JOINT_COLOR else "chassis"
        color = FASTENER_JOINT_COLOR.get(fi.joint, FASTENER_CHASSIS_COLOR)
        yield f"screw_{joint_key}", fi.leg_index, joint_key, color, mesh


def _disc_horn_instances(chassis_lift: float, legs: list[int]):
    """Yield (name, leg, joint, color, mesh) for the Phi20 aluminium disc
    horn seated on EACH servo output (yaw/hip/knee) of every leg.

    Authoritative pose: ``_verify_prototype._horn_world_transform`` maps the
    horn-local frame of ``make_disc_horn`` (origin = spline/servo face, +Z =
    output-shaft axis) into the per-leg world frame -- the SAME transform the
    verifier's disc-horn-fit / fastener-engagement checks use, so the horns
    land exactly on the servo output bosses the link bolts clamp to."""
    for joint in V._HORN_JOINTS:
        for leg in legs:
            mesh = HP.make_disc_horn()
            mesh.apply_transform(V._horn_world_transform(joint, leg))
            mesh.apply_translation([0, 0, chassis_lift])
            yield f"disc_horn_{joint}", leg, joint, DISC_HORN_COLOR, mesh


def _body_parts(chassis_lift: float) -> list[tuple[str, trimesh.Trimesh]]:
    bot = HP.make_chassis_bottom()
    bot.apply_translation([0, 0, chassis_lift])
    # Jun 2026 print split: bolt-on LOW half (yaw-servo cradle plate).
    bot_lower = HP.make_chassis_bottom_lower()
    bot_lower.apply_translation([0, 0, chassis_lift])
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

    # Spider carapace dome bolts on as a 3rd deck level above the buck tray.
    carapace_z = (deck_z0 + HP.DECK_LEVEL_1_STANDOFF_H
                  + HP.DECK_LEVEL_2_STANDOFF_H
                  + HP.DECK_LEVEL_3_STANDOFF_H)
    carapace = HP.make_spider_carapace()
    carapace.apply_translation([0, 0, carapace_z])

    return [("chassis_bottom", bot), ("chassis_bottom_lower", bot_lower),
            ("chassis_top", top),
            ("lipo_battery", lipo),
            ("uno_q_tray", uno_tray), ("buck_tray", buck_tray),
            ("uno_q", uno), ("buck_converter", buck),
            ("spider_carapace", carapace)]


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

    # COTS hardware placed independently in the SAME per-leg frame as the
    # verifier parts: every screw/nut/insert (registry) + the disc horns on
    # each servo output.  These are scene-only and NOT printable parts.
    cots_color: dict[str, str] = {}
    for name, leg, joint, color, mesh in _fastener_instances(chassis_lift):
        if single_leg and leg not in (0, None):
            continue
        cots_color[name] = color
        tagged.append((name, leg, mesh))
    for name, leg, joint, color, mesh in _disc_horn_instances(chassis_lift, legs):
        if single_leg and leg != 0:
            continue
        cots_color[name] = color
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
        if name.startswith("screw_"):
            role, joint = "fastener", name[len("screw_"):]
        elif name.startswith("disc_horn_"):
            role, joint = "horn", name[len("disc_horn_"):]
        else:
            role, joint = ROLE.get(name, "part"), None
        instances_json.append({
            "id": f"{idx:03d}-{name}",
            "meshId": mid,
            "name": f"L{leg} {name}" if leg is not None else name,
            "partType": name,
            "role": role,
            "leg": leg,
            "joint": joint,
            "cots": role in COTS_ROLES,
            "color": cots_color.get(name) or PALETTE.get(name, "#888888"),
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
        # Project intent as DATA (BuildViz checksConfig): the intended part-type
        # matings that overlap by design, so the viewer's live checks mark them
        # pass(allowed) instead of flagging them.
        "checksConfig": {
            "overlapMm3": CHECK_OVERLAP_MM3,
            "pitchMm": CHECK_PITCH_MM,
            "ignoreOverlapPairs": sorted(sorted(p) for p in INTENDED_OVERLAP_PAIRS),
        },
        "meshes": meshes_json,
        "instances": instances_json,
    }
    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    print(f"Wrote {OUT_DIR/'scene.json'}  ({len(instances_json)} instances, "
          f"lift={chassis_lift:.1f})")

    # Precompute the BuildViz checks sidecar (read on scene load).
    sidecar = _build_checks_sidecar(tagged, instances_json)
    (OUT_DIR / "buildviz_checks.json").write_text(json.dumps(sidecar, indent=2))
    s = sidecar["summary"]
    print(f"Wrote {OUT_DIR/'buildviz_checks.json'}  "
          f"({s['fail']} fail / {s['warn']} warn / {s['pass']} pass "
          f"of {s['total']} checks)")


if __name__ == "__main__":
    main(single_leg="--single" in sys.argv)
