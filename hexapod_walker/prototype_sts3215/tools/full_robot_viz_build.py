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

Motion is now baked into this ONE scene: the additive BuildViz ``joints[]`` +
``poses[]`` + ``animations[]`` blocks (per-joint sliders, clickable poses, a
looping tripod-walk clip) live directly in ``full_robot_viz/scene.json`` -- there
is no longer a separate ``scene_motion.json`` file or a distinct
``prototype_sts3215_motion`` build id.  Generating the motion blocks is
essentially free (~0.01 s of joint/pose math on the ALREADY-placed meshes vs the
~4 s STL rebuild), so motion is ALWAYS generated.  The only expensive motion step
is the swept self-overlap check (``buildviz sweep`` ~13 s), which is therefore
OPT-IN on push (see the Makefile ``SWEEP=1``), while the cheap manifest
validation runs every push.

Run:
    ./run.sh hexapod_walker/prototype_sts3215/tools/full_robot_viz_build.py

Then PUBLISH into the ONE machine-wide BuildViz hub (default port 5183) and view
it there -- never start a per-project dev server on 5173/etc:
    make -C hexapod_walker/prototype_sts3215 verify-buildviz   # buildviz push --bump
    open http://127.0.0.1:5183/?build=prototype_sts3215
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
from pi_control import wire_harness_plan as WHP  # noqa: E402

OUT_DIR = _HERE.parent / "full_robot_viz"
STL_DIR = OUT_DIR / "stl"
FASTENERS_DIR = _HERE.parent / "fasteners"

# Mesh URLs are RELATIVE (``stl/<file>.stl``) so this build is self-contained
# and portable across every BuildViz entry point: ``buildviz push
# --upload-assets`` ships the referenced STL bytes (it only uploads relative,
# locally-resolvable URLs) and the hub rewrites them to its content-addressed
# ``/builds/_assets/<sha>`` store; the local ``buildviz .`` debug server rewrites
# relative ``stl/...`` URLs to the served build; and ``tools/buildviz_checks.py``
# resolves them next to ``scene.json``.  (Absolute ``/builds/<id>/stl/...`` URLs
# only resolved against a hand-made static ``public/builds/<id>`` mirror, which
# this project no longer relies on -- versioned publishing now goes through the
# hub cache via ``push --bump``.)
SCENE_BUILD_ID = "prototype_sts3215"
SCENE_ASSET_BASE = "stl"

# Joint screws to render (the leg "connecting" fasteners).  Bright colours
# for the hip bolts so the coxa<->femur connection stands out.  Chassis-level
# / foot fasteners (registry joint == None) fall back to FASTENER_CHASSIS_COLOR.
FASTENER_JOINT_COLOR = {"yaw": "#bdbdbd", "hip": "#ffd000", "knee": "#9ad0ff"}
FASTENER_CHASSIS_COLOR = "#9a9a9a"          # steel hardware (standoffs/foot/deck)
DISC_HORN_COLOR = "#b8b8c0"                  # silver-anodised aluminium disc horn
PASSIVE_ADAPTER_COLOR = "#caa46a"            # printed rear-boss centering adapter

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
    "femur_strut": "#5a9e3a",                            # printed PA12 strut
    "tibia_tube": "#2b2b2b",                             # carbon fibre
    "foot_pad": "#3a3a3a",                               # TPU pad
    "disc_horn_yaw": DISC_HORN_COLOR, "disc_horn_hip": DISC_HORN_COLOR,
    "disc_horn_knee": DISC_HORN_COLOR,
    "passive_horn_hip": DISC_HORN_COLOR, "passive_horn_knee": DISC_HORN_COLOR,
    "passive_adapter_hip": PASSIVE_ADAPTER_COLOR,
    "passive_adapter_knee": PASSIVE_ADAPTER_COLOR,
    "yaw_servo": "#2e2e33", "hip_servo": "#3a3a40", "knee_servo": "#46464d",
    "chassis_bottom": "#79b0e1",
    "chassis_top": "#5b8fc7",
    "uno_q_tray": "#9467bd", "buck_tray": "#bcbd22",
    "spider_carapace": "#23232a",
    "uno_q": "#1b7a3d", "buck_converter": "#b5651d",
    "lipo_battery": "#d62728",
    "hip_clamp_cap": "#4a90d9", "knee_clamp_cap": "#4a90d9",
    "yaw_servo_retainer": "#e377c2",
    "switch_holster": "#c46a1f", "imu_pad": "#7a5cc4",
}
ROLE = {
    "coxa_link": "frame",
    "coxa_yaw_hub": "frame", "coxa_hip_bracket": "frame",
    "yaw_bearing_cap": "chassis",
    "yaw_bearing_lower": "bearing", "yaw_bearing_upper": "bearing",
    "femur_hip_yoke": "frame", "femur_knee_bracket": "frame",
    "tibia_knee_yoke": "frame", "tibia_foot_fitting": "frame",
    "femur_strut": "spar", "tibia_tube": "spar", "foot_pad": "frame",
    "disc_horn_yaw": "horn", "disc_horn_hip": "horn", "disc_horn_knee": "horn",
    "passive_horn_hip": "horn", "passive_horn_knee": "horn",
    "passive_adapter_hip": "frame", "passive_adapter_knee": "frame",
    "yaw_servo": "motor", "hip_servo": "motor", "knee_servo": "motor",
    "chassis_bottom": "chassis",
    "chassis_top": "chassis",
    "uno_q_tray": "electronics", "buck_tray": "electronics",
    "spider_carapace": "chassis",
    "uno_q": "electronics", "buck_converter": "electronics",
    "lipo_battery": "electronics",
    "hip_clamp_cap": "frame", "knee_clamp_cap": "frame",
    "yaw_servo_retainer": "frame",
    "switch_holster": "electronics", "imu_pad": "electronics",
}

# Per-partType design intent / rationale.  This is the semantic source of truth
# that `tools/full_robot_viz_build.py` writes into full_robot_viz/design_spec.yaml
# so the file ALWAYS covers every partType the scene emits (BuildViz `compat`
# fails on any scene part with no entry).  Kinematics are FROZEN: 6 legs, COXA 25
# / FEMUR 90 / TIBIA 130 mm; only joint/segment CONSTRUCTION changes.  Joint
# architecture: YAW = cantilevered (coxa 25 mm < ~50 mm rule, no passive
# bearing); HIP + KNEE = bearing sandwich (driven Ø20 25T disc horn on one side,
# 688 passive bearing on the other).  When you add/rename a partType in PALETTE,
# add its rationale here too — a missing key makes the build non-compat.
DESCRIPTIONS = {
    "coxa_link": "Short (25 mm) coxa link driven by the yaw disc horn; carries the hip joint at its outboard end. Cantilevered (no passive bearing) because 25 mm < the ~50 mm span where a passive bearing earns its complexity.",
    "coxa_yaw_hub": "Printed coxa inboard hub that bolts to the driven yaw disc horn (Ø14 / 4x M3 circle) and takes the yaw servo's output.",
    "coxa_hip_bracket": "Printed coxa outboard bracket that carries the hip joint (hip servo cradle + disc-horn / passive-bearing sandwich).",
    "yaw_bearing_cap": "Printed cap that closes the top of each chassis yaw-bearing tower, capturing the upper yaw bearing.",
    "yaw_bearing_lower": "Lower ball bearing of the yaw-axis bearing pair in the chassis tower (COTS).",
    "yaw_bearing_upper": "Upper ball bearing of the yaw-axis bearing pair in the chassis tower (COTS).",
    "femur_hip_yoke": "Printed femur hip-end yoke: straddles the hip servo, top arm bolts to the driven disc horn, bottom stub rides the 688 passive bearing; sockets the Ø8 femur strut.",
    "femur_knee_bracket": "Printed femur knee-end bracket that mounts the knee servo and sockets the femur strut.",
    "tibia_knee_yoke": "Printed tibia knee-end yoke driven off the knee disc horn (bearing-sandwich passive side on the opposite face); sockets the Ø8 tibia tube.",
    "tibia_foot_fitting": "Printed tibia foot-end fitting that sockets the tibia tube and carries the compliant foot pad.",
    "femur_strut": "Printed PA12 femur strut (90 mm hip->knee segment). Printed (not CF tube) here so the hip yoke/knee bracket integrate as one stiff member.",
    "tibia_tube": "Ø8 carbon-fibre tibia segment (130 mm knee->foot). Retained by epoxy bond + a transverse pin. CF for stiffness/weight at the longest, most-loaded segment.",
    "foot_pad": "Compliant TPU foot pad at each tibia tip for ground grip + shock absorption.",
    "disc_horn_yaw": "Driven Ø20 25T aluminium disc horn on the yaw servo output; the coxa hub bolts to it (Ø14 / 4x M3). COTS.",
    "disc_horn_hip": "Driven Ø20 25T aluminium disc horn on the hip servo output; the femur hip yoke bolts to it. COTS.",
    "disc_horn_knee": "Driven Ø20 25T aluminium disc horn on the knee servo output; the tibia knee yoke bolts to it. COTS.",
    "passive_horn_hip": "Passive-side disc horn of the hip bearing sandwich (mirrors the driven horn so the yoke is symmetric across the servo).",
    "passive_horn_knee": "Passive-side disc horn of the knee bearing sandwich.",
    "passive_adapter_hip": "Printed rear-boss centering adapter that seats the hip 688 passive bearing to the yoke's passive stub.",
    "passive_adapter_knee": "Printed rear-boss centering adapter that seats the knee 688 passive bearing.",
    "yaw_servo": "FEETECH STS3215 serial-bus servo driving the hip-yaw axis (real FEETECH envelope). COTS.",
    "hip_servo": "FEETECH STS3215 serial-bus servo driving the hip-pitch axis. COTS.",
    "knee_servo": "FEETECH STS3215 serial-bus servo driving the knee axis. COTS.",
    "chassis_bottom": "Structural 200 mm flat-to-flat hex deck (single merged print) with 6 integrated STS3215 front-face-mount yaw cradles + upward yaw-bearing towers, one per leg at each hex-edge midpoint. Each STS3215 inserts from BELOW (output UP), bolts via 4x M2.5 through the cradle front plate, body hangs DOWN through a body cutout; the bolt-on yaw_servo_retainer stirrup captures it. A folded 4 mm floor makes the printed bottom one flush flat face so it prints flat, tower-up, no supports (Jun 2026 flush-bottom fix; check_flat_bottom overhang = 0.00 mm).",
    "chassis_top": "Hex top plate that closes the chassis over the electronics stack.",
    "uno_q_tray": "Printed tray that mounts the Arduino Uno Q compute board on the chassis.",
    "buck_tray": "Printed tray that mounts the buck converter on the chassis.",
    "spider_carapace": "Cosmetic domed carapace shell over the chassis (jumping-spider styling); vented, lifts off as one piece.",
    "uno_q": "Arduino Uno Q compute board (high-level control host). COTS.",
    "buck_converter": "DC-DC buck converter stepping the LiPo down to the servo-bus/logic rail. COTS.",
    "lipo_battery": "LiPo battery pack, mounted low + central for stance stability. COTS.",
    "hip_clamp_cap": "Printed clamp cap that closes the hip yoke's tube socket, clamping the Ø8 femur strut.",
    "knee_clamp_cap": "Printed clamp cap that closes the knee yoke's tube socket, clamping the Ø8 tibia tube.",
    "yaw_servo_retainer": "Printed stirrup that bolts under each chassis yaw cradle to capture the STS3215 servo body from below.",
    "switch_holster": "Printed holster for the anti-spark XT60 on/off switch, bolted to 2 bosses on chassis_top's +X edge (2x M3x10 SHCS into heat-set inserts).",
    "imu_pad": "Printed vibration-isolated MPU-6050 mounting pad, foam-taped to chassis_top at the chassis CG; the IMU bolts to 4 heat-set inserts in its bosses.",
    "screw_yaw": "Fasteners around the yaw joint / servo mount (rendered set; count is the summed per-leg joint hardware).",
    "screw_hip": "Fasteners around the hip joint / bearing sandwich (rendered set).",
    "screw_knee": "Fasteners around the knee joint / bearing sandwich (rendered set).",
    "screw_chassis": "Chassis-level / foot / deck fasteners (rendered set).",
}


def _write_design_spec(instances_json: list[dict]) -> None:
    """Write full_robot_viz/design_spec.yaml keyed by every scene partType.

    BuildViz's compatibility contract (see ~/buildviz/BUILDVIZ_COMPATIBILITY.md)
    requires the design_spec's ``parts:`` mapping to cover every ``partType`` the
    scene emits.  We derive it from the instances we just placed so the spec can
    never silently drift out of coverage: qty + role + cots come from the scene,
    the rationale from DESCRIPTIONS above.
    """
    counts: dict[str, int] = {}
    role_of: dict[str, str] = {}
    cots_of: dict[str, bool] = {}
    for inst in instances_json:
        pt = inst["partType"]
        counts[pt] = counts.get(pt, 0) + 1
        role_of[pt] = inst.get("role", "")
        cots_of[pt] = bool(inst.get("cots", False))

    lines = [
        "# Hexapod STS3215 — full robot (viz).  design intent + rationale, and",
        "# BuildViz's semantic source of truth (one entry per scene partType).",
        "#",
        "# GENERATED by tools/full_robot_viz_build.py from the placed instances so",
        "# it always covers every scene partType (edit DESCRIPTIONS in that script,",
        "# not this file).  Kinematics FROZEN: 6 legs, COXA 25 / FEMUR 90 / TIBIA",
        "# 130 mm.  YAW = cantilevered; HIP + KNEE = bearing sandwich (driven Ø20",
        "# 25T disc horn + 688 passive bearing).",
        "",
        "units: mm",
        "",
        "parts:",
    ]
    for pt in sorted(counts):
        label = pt.replace("_", " ")
        desc = DESCRIPTIONS.get(pt, f"{label} (see full_robot_viz_build.py).")
        lines.append(f"  {pt}:")
        lines.append(f"    label: {json.dumps(label)}")
        lines.append(f"    role: {json.dumps(role_of[pt])}")
        lines.append(f"    cots: {'true' if cots_of[pt] else 'false'}")
        lines.append(f"    qty: {counts[pt]}")
        lines.append(f"    description: {json.dumps(desc)}")
    (OUT_DIR / "design_spec.yaml").write_text("\n".join(lines) + "\n")
    print(f"Wrote {OUT_DIR/'design_spec.yaml'}  ({len(counts)} part types)")


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
        ("coxa_hip_bracket", "hip_servo"),
        ("femur_knee_bracket", "knee_servo"),
        # Clamp cap PRESS-FIT onto its own servo body: the centre tongue
        # reaches CLAMP_TONGUE_INTERF = 1 mm PAST the seated body +Y face for a
        # snug press fit (user request, Jun 2026; ~1.5 k mm^3 designed
        # interference).  NB: this is the servo<->cap press; the cap<->BRACKET
        # mate below stays strict (must be flush ~0 mm^3).
        ("hip_servo", "hip_clamp_cap"),
        ("knee_servo", "knee_clamp_cap"),
        # Disc horn seats on its servo output boss.
        ("yaw_servo", "disc_horn_yaw"),
        ("hip_servo", "disc_horn_hip"),
        ("knee_servo", "disc_horn_knee"),
        # Femur: printed strut pinned into the yoke / bracket sockets.
        # Tibia: carbon-fibre spar epoxied into the yoke / fitting sockets.
        ("femur_hip_yoke", "femur_strut"),
        ("femur_knee_bracket", "femur_strut"),
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


# ---------------------------------------------------------------------------
# LOCAL-FRAME part producers (so BuildViz reports CAD-local coordinates)
# ---------------------------------------------------------------------------
#
# BuildViz's pick read-out shows ``local XYZ`` via ``mesh.worldToLocal`` -- the
# inverse of each instance's ``transform``.  If we bake the world placement into
# the STL and ship an identity transform (the old behaviour), that read-out
# equals WORLD mm, which does NOT match any single part's design frame and made
# it painful to talk to an LLM about feature positions.
#
# Instead every producer below returns ``(name, local_mesh, M0)`` where:
#   * ``local_mesh`` is the RAW ``make_*()`` part in its OWN design frame -- the
#     identical frame as ``stl_prototype/<part>.stl`` and the ``make_*`` source
#     code -- so picking a point reports coordinates an LLM can act on directly.
#   * ``M0`` is the 4x4 that maps that local frame into the leg-0 world frame
#     (apothem a=pi/6), BEFORE the per-leg yaw rotation and standing-pose lift
#     (both applied in ``main`` and folded into the per-instance ``transform``).
# World placement is ``M0 @ local`` and reproduces EXACTLY the poses the
# verifier checks (asserted in ``main`` against ``_verify_prototype``).

# Multiple instances share one identical local STL (all six legs print the same
# coxa hub; every servo well holds the same body envelope; etc).  This map
# collapses those onto a single exported mesh; unlisted names key on themselves.
_SCENE_MESH_KEY = {
    "yaw_servo": "servo_body",
    "hip_servo": "servo_body",
    "knee_servo": "servo_body",
    "hip_clamp_cap": "servo_clamp_cap",
    "knee_clamp_cap": "servo_clamp_cap",
}


def _scene_mesh_key(name: str) -> str:
    return _SCENE_MESH_KEY.get(name, name)


def _leg0_local_link_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for every leg-0 printed link part.

    The femur/tibia are decomposed into their REAL printed sandwich parts
    (yoke + dia-8 CF tube + bracket/fitting) so the assembly calls out every
    printable part instead of the merged link proxies.  ``M0`` mirrors the
    per-link world transform ``_build_standing_leg`` uses, so ``M0 @ local``
    occupies the identical space the verifier checks for self-collision."""
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

    xz = (1, 0, 0), HP.LEG_PITCH_AXIS
    # Femur sub-part frames inside the femur-link-local frame (== make_femur_link).
    Mh = HP._joint_place((0.0, 0.0, 0.0), *xz)
    Mk = HP._joint_place((HP.FEMUR_LENGTH, 0.0, 0.0), *xz)
    # Jun 2026: printed SOLID femur strut replaces the femur CF tube; place it
    # in the hip-yoke joint-local frame (== make_femur_link) so it spans + pins
    # to both sockets exactly where the tube did.
    fstrut = HP._femur_strut_joint_local()
    fstrut.apply_transform(Mh)

    # Tibia sub-part frames inside the tibia-link-local frame (== make_tibia_link).
    Mk0 = HP._joint_place((0.0, 0.0, 0.0), *xz)
    ta = (Mk0 @ np.array([HP._YOKE_SOCKET_X, 0.0, HP.JOINT_SOCKET_Z, 1.0]))[:3]
    foot_sock = ta + np.array([HP.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    foot_frame = HP._frame(foot_sock, (1, 0, 0), (0, 0, 1))
    ttube = HP._tube_between(ta, foot_sock, HP.LEG_TUBE_OD / 2.0)

    # Foot pad: hinge at local x=16 on the foot fitting; the foot stays flat
    # on the ground (yaw-only orientation), dropped by FOOT_HINGE_FOOT_Z.
    hinge_w = (T_tibia @ foot_frame @ np.array([16.0, 0.0, 0.0, 1.0]))[:3]
    foot_M0 = _trans([hinge_w[0], hinge_w[1], hinge_w[2] - HP.FOOT_HINGE_FOOT_Z]) \
        @ rotation_matrix(a, [0, 0, 1])

    return [
        # Coxa: yaw turntable hub + hip bracket, plus the SPACED 688 bearing
        # pair (visual, NOT printed) so the bearing-supported yaw joint shows.
        ("coxa_yaw_hub", HP.make_coxa_yaw_hub(), T_coxa),
        ("coxa_hip_bracket", HP.make_coxa_hip_bracket(), T_coxa),
        ("yaw_bearing_cap", HP.make_yaw_bearing_cap(), T_coxa),
        ("yaw_bearing_lower", HP.make_yaw_bearing_lower(), T_coxa),
        ("yaw_bearing_upper", HP.make_yaw_bearing_upper(), T_coxa),
        # Femur sandwich (yoke + CF spar + knee bracket).  The CF tube has no
        # make_* raw frame, so its local frame IS the femur-link frame.
        ("femur_hip_yoke", HP.make_femur_hip_yoke(), T_femur @ Mh),
        ("femur_knee_bracket", HP.make_femur_knee_bracket(), T_femur @ Mk),
        ("femur_strut", fstrut, T_femur),
        # Tibia sandwich (knee yoke + CF spar + foot fitting).
        ("tibia_knee_yoke", HP.make_tibia_knee_yoke(), T_tibia @ Mk0),
        ("tibia_tube", ttube, T_tibia),
        ("tibia_foot_fitting", HP.make_tibia_foot_fitting(), T_tibia @ foot_frame),
        ("foot_pad", HP.make_foot_pad(), foot_M0),
    ]


def _servo_local_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for the leg-0 servo bodies, sandwich clamp
    caps, and the yaw capture stirrup.

    Mirrors the authoritative placement chains in
    ``_verify_prototype._place_servo_bodies`` / ``_place_servo_clamp_caps`` /
    ``_place_yaw_retainers`` but keeps the local-mesh + matrix split.  ``main``
    asserts ``M0 @ local`` matches those functions exactly, so any drift in the
    duplicated math is caught at build time."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    R_a = rotation_matrix(a, [0, 0, 1])
    yaw_output_world = edge_mid + HP.CHASSIS_YAW_OUTPUT_Z * np.array([0.0, 0.0, 1.0])
    hip_local = np.array(HP.COXA_HIP_ANCHOR)
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)

    M0_yaw = _trans(edge_mid) @ R_a @ _trans(
        [-HP.SERVO_OUTPUT_X, 0.0,
         HP.CHASSIS_YAW_OUTPUT_Z - HP.HORN_STACK_H - HP.WELL_RIM_Z])
    M_hip = HP._joint_place(HP.COXA_HIP_ANCHOR, (1, 0, 0), HP.LEG_PITCH_AXIS)
    M0_hip = _trans(yaw_output_world) @ R_a @ M_hip
    M_knee = HP._joint_place((HP.FEMUR_LENGTH, 0.0, 0.0), (1, 0, 0), HP.LEG_PITCH_AXIS)
    M0_knee = _trans(yaw_output_world) @ R_a @ _trans(hip_local) \
        @ rotation_matrix(p, [0, 1, 0]) @ M_knee
    M0_ret = _trans(edge_mid) @ R_a

    return [
        ("yaw_servo", V._load_mesh("servo_body"), M0_yaw),
        ("hip_servo", V._load_mesh("servo_body"), M0_hip),
        ("knee_servo", V._load_mesh("servo_body"), M0_knee),
        ("hip_clamp_cap", HP.make_servo_clamp_cap(), M0_hip),
        ("knee_clamp_cap", HP.make_servo_clamp_cap(), M0_knee),
        ("yaw_servo_retainer", HP.make_yaw_servo_retainer(), M0_ret),
    ]


def _leg0_local_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """All leg-0 parts as ``(name, local_mesh, M0)`` (links + servos/caps/strap)."""
    return _leg0_local_link_parts() + _servo_local_parts()


def _assert_servo_placement(tol: float = 1e-4) -> None:
    """Guard: the duplicated servo/cap/strap placement math reproduces the
    authoritative ``_verify_prototype`` poses (bounds match to ``tol`` mm)."""
    ref: dict[str, trimesh.Trimesh] = {}
    ref.update(V._place_servo_bodies())
    ref.update(V._place_servo_clamp_caps())
    ref.update(V._place_yaw_retainers())
    for name, local, M0 in _servo_local_parts():
        placed = local.copy()
        placed.apply_transform(M0)
        drift = float(np.abs(np.asarray(placed.bounds)
                             - np.asarray(ref[name].bounds)).max())
        if drift > tol:
            raise AssertionError(
                f"{name}: local-frame placement drifts {drift:.3e} mm from "
                f"_verify_prototype authoritative pose")


def _leg0_parts() -> list[tuple[str, trimesh.Trimesh]]:
    """(verifier-facing) leg-0 parts as PLACED ``(name, world_mesh)`` in the
    leg-0 world frame -- a thin wrapper over ``_leg0_local_parts`` so the
    verifier and the local-coordinate scene share one source of truth."""
    out: list[tuple[str, trimesh.Trimesh]] = []
    for name, local, M0 in _leg0_local_parts():
        m = local.copy()
        m.apply_transform(M0)
        out.append((name, m))
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


def _fastener_instances():
    """Yield ``(name, leg, joint, color, mesh_key, local_mesh, M)`` for EVERY
    non-virtual fastener in the registry -- leg-joint screws (yaw/hip/knee) AND
    chassis-level / foot / deck hardware (registry ``joint is None``).  COTS,
    scene-only.  ``local_mesh`` is the cached bolt in its native frame (+Z =
    shaft, origin = head face) and ``M`` maps it into the per-leg WORLD frame
    (registry-supplied, pre-lift); the standing-pose lift is folded in by
    ``main``."""
    for fi in FR.build_all_fastener_instances():
        if getattr(fi, "is_virtual", False):
            continue
        cache = FASTENERS_DIR / fi.cache_stl if fi.cache_stl else None
        if cache is not None and cache.is_file():
            mesh = trimesh.load(cache, process=False)
            if isinstance(mesh, trimesh.Scene):
                mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
            mesh_key = f"fastener_{cache.stem}"
        else:
            mesh = _primitive_bolt(fi)
            mesh_key = (f"fastener_prim_{getattr(fi, 'nominal_dia_mm', 3.0)}"
                        f"_{getattr(fi, 'length_mm', 8.0)}")
        M = _axis_to_transform(fi.axis_world, fi.head_world_xyz)
        joint_key = fi.joint if fi.joint in FASTENER_JOINT_COLOR else "chassis"
        color = FASTENER_JOINT_COLOR.get(fi.joint, FASTENER_CHASSIS_COLOR)
        yield f"screw_{joint_key}", fi.leg_index, joint_key, color, mesh_key, mesh, M


def _disc_horn_instances(legs: list[int]):
    """Yield ``(name, leg, joint, color, mesh_key, local_mesh, M)`` for the
    Phi20 aluminium disc horn seated on EACH servo output (yaw/hip/knee) of
    every leg.

    Authoritative pose: ``_verify_prototype._horn_world_transform`` maps the
    horn-local frame of ``make_disc_horn`` (origin = spline/servo face, +Z =
    output-shaft axis) into the per-leg world frame -- the SAME transform the
    verifier's disc-horn-fit / fastener-engagement checks use, so the horns
    land exactly on the servo output bosses the link bolts clamp to.  The
    standing-pose lift is folded in by ``main``."""
    for joint in V._HORN_JOINTS:
        for leg in legs:
            M = V._horn_world_transform(joint, leg)
            yield (f"disc_horn_{joint}", leg, joint, DISC_HORN_COLOR,
                   "disc_horn", HP.make_disc_horn(), M)


def _passive_horn_instances(legs: list[int]):
    """Yield the PASSIVE rear-boss disc horn + its printed centering adapter
    for each sandwich joint (hip/knee) of every leg -- the symmetric-yoke
    refit.  Poses come from the SAME verifier transforms the passive-horn
    fastener-engagement + adapter checks use, so the reused Phi20 disc horn
    and the printed adapter land coaxial on the servo's rear idler boss."""
    for joint in V._PASSIVE_HORN_JOINTS:
        for leg in legs:
            yield (f"passive_horn_{joint}", leg, joint, DISC_HORN_COLOR,
                   "disc_horn", HP.make_disc_horn(),
                   V._passive_horn_world_transform(joint, leg))
            yield (f"passive_adapter_{joint}", leg, joint,
                   PASSIVE_ADAPTER_COLOR, "passive_horn_adapter",
                   HP.make_passive_horn_adapter(),
                   V._passive_adapter_world_transform(joint, leg))


def _body_local_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for the chassis / electronics body parts.

    Each ``local_mesh`` is the raw ``make_*()`` part (matching its
    ``stl_prototype`` STL) and ``M0`` is its PRE-LIFT stack offset along Z (the
    standing-pose lift is added in ``main``).  Mirrors ``make_assembly_preview``
    so the body sits exactly as the printed stack assembles."""
    from trimesh.creation import box as _box_mesh
    gap = HP.CHASSIS_GAP
    plate = HP.CHASSIS_PLATE_T
    deck0 = gap + 1.5 * plate
    l1 = HP.DECK_LEVEL_1_STANDOFF_H
    l2 = HP.DECK_LEVEL_2_STANDOFF_H
    l3 = HP.DECK_LEVEL_3_STANDOFF_H
    boss = HP.DECK_TRAY_T + HP.DECK_STANDOFF_BOSS_H
    uno_tray_z = deck0 + l1
    buck_tray_z = deck0 + l1 + l2

    # LiPo velcro-strapped to chassis_bottom's top face (no holder); modelled as
    # a box centred on the origin so its local frame is its own centroid.
    lipo = _box_mesh(extents=(105.0, 35.0, 25.0))

    return [
        ("chassis_bottom", HP.make_chassis_bottom(), np.eye(4)),
        ("chassis_top", HP.make_chassis_top(), _trans([0, 0, gap + plate])),
        ("lipo_battery", lipo,
         _trans([HP.BATTERY_HOLDER_CENTRE_X, 0.0, plate / 2.0 + 25.0 / 2.0])),
        ("uno_q_tray", HP.make_uno_q_tray(), _trans([0, 0, uno_tray_z])),
        ("buck_tray", HP.make_buck_tray(), _trans([0, 0, buck_tray_z])),
        ("uno_q", HP.make_uno_q_visual(), _trans([0, 0, uno_tray_z + boss])),
        ("buck_converter", HP.make_buck_converter_visual(),
         _trans([0, 0, buck_tray_z + boss])),
        ("spider_carapace", HP.make_spider_carapace(),
         _trans([0, 0, deck0 + l1 + l2 + l3])),
        # May 2026 "essentials" printed parts on chassis_top's TOP face.
        # These were in the verifier's world assembly + fastener registry all
        # along but MISSING from this scene list, so their screw_chassis
        # fasteners rendered floating in air (the Jul 2026 "floating
        # screw_chassis" bug).  Placement mirrors
        # _verify_prototype._build_world_assembly_parts exactly.
        ("switch_holster", HP.make_switch_holster(),
         _trans([HP.SWITCH_HOLSTER_CENTRE_X, HP.SWITCH_HOLSTER_CENTRE_Y,
                 gap + 1.5 * plate + HP.SWITCH_HOLSTER_BOSS_H])),
        ("imu_pad", HP.make_imu_pad(),
         _trans([HP.IMU_PAD_CENTRE_X, HP.IMU_PAD_CENTRE_Y,
                 gap + 1.5 * plate + HP.IMU_PAD_TAPE_T])),
    ]


def _body_parts(chassis_lift: float) -> list[tuple[str, trimesh.Trimesh]]:
    """(verifier-facing) body parts as PLACED ``(name, world_mesh)`` lifted by
    ``chassis_lift`` -- a thin wrapper over ``_body_local_parts``."""
    Tlift = _trans([0.0, 0.0, chassis_lift])
    out: list[tuple[str, trimesh.Trimesh]] = []
    for name, local, M0 in _body_local_parts():
        m = local.copy()
        m.apply_transform(Tlift @ M0)
        out.append((name, m))
    return out


def _col_major(M: np.ndarray) -> list[float]:
    """Flatten a 4x4 row-major numpy matrix into the column-major 16-array
    BuildViz feeds to ``THREE.Matrix4().fromArray`` (translation at 12..14)."""
    return [float(v) for v in np.asarray(M, float).flatten("F")]


# ---------------------------------------------------------------------------
# Motion model (BuildViz ``joints[]`` + ``poses[]`` + ``animations[]``)
# ---------------------------------------------------------------------------
#
# These blocks are ALWAYS baked into the single ``scene.json`` (mirroring the
# prototype_v1 ``hexapod-prototype`` build): generating them is essentially free
# (joint/pose math on the already-placed meshes), so there is no reason to gate
# GENERATION or split them into a second build.  Only the swept self-overlap
# VALIDATION (``buildviz sweep``) is expensive and is opt-in on push.
#
# BuildViz drives motion from three OPTIONAL top-level scene keys (see
# /Users/lbiewald/buildviz BUILDVIZ_INTEGRATION.md "Motion / kinematics" and
# src/buildvizKinematics.ts).  Old viewers ignore them; a viewer that
# understands them gets per-joint sliders, an "Animate sweep" button (each DOF
# swept min->max->min), clickable named poses, and a play/scrub timeline for the
# keyframed ``animations[]`` clip, plus swept-pose overlap validation.  The
# forward-kinematics the viewer computes is EXACTLY:
#
#   L(joint, v) = T(origin) * R(axis, v-home) * T(-origin)     [revolute, deg]
#   C(j)        = C(parent) * L(j)                             [chain compose]
#   world(inst) = C(j) * inst.transform                       [home = static]
#
# so ``axis``/``origin`` are in the SCENE (world home) frame and a joint's
# ``instances`` list holds ONLY its own distal link (ancestors live on the
# parent joints).  The static scene IS the home pose (every joint value 0), so
# we can reuse the verified standing-pose ``transform``s untouched and only emit
# the joint axes/pivots + which instances each joint carries.
#
# The yaw/hip/knee axes and pivots below are derived from the SAME authoritative
# leg-0 kinematic chain ``_leg0_local_link_parts`` / ``_servo_local_parts`` use
# (apothem direction a=pi/6; the static ``T_coxa``/``T_femur``/``T_tibia``
# placement), then mapped into each leg's world frame by the per-leg yaw
# rotation + standing-pose lift -- so the rotation a slider applies lands on the
# real joint axis through the real pivot, not a centroid guess.

# partType (instance ``name``) -> the kinematic LINK it rigidly belongs to.
# An instance moves with the DEEPEST joint listing it; with the yaw->hip->knee
# parent chain, listing each part on its own link composes correctly:
#   * yaw  link = coxa turntable + the hip servo BODY bolted to it (the hip
#                 servo's body rides the coxa; its output drives the femur).
#   * hip  link = femur sandwich + hip horns + the knee servo BODY on the femur.
#   * knee link = tibia sandwich + knee horns + foot.
# The yaw servo body / its chassis retainer stay FIXED (chassis), as do all
# body/electronics parts -- they map to None and never move.
_MOTION_LINK_OF_PARTTYPE = {
    # --- yaw link (rotates the whole leg about vertical Z) ---
    "coxa_yaw_hub": "yaw", "coxa_hip_bracket": "yaw",
    "yaw_bearing_cap": "yaw", "yaw_bearing_lower": "yaw",
    "yaw_bearing_upper": "yaw", "disc_horn_yaw": "yaw",
    "hip_servo": "yaw", "hip_clamp_cap": "yaw", "screw_yaw": "yaw",
    # --- hip link (femur swings in its vertical plane) ---
    "femur_hip_yoke": "hip", "femur_strut": "hip", "femur_knee_bracket": "hip",
    "disc_horn_hip": "hip", "passive_horn_hip": "hip",
    "passive_adapter_hip": "hip", "knee_servo": "hip",
    "knee_clamp_cap": "hip", "screw_hip": "hip",
    # --- knee link (tibia + foot) ---
    "tibia_knee_yoke": "knee", "tibia_tube": "knee",
    "tibia_foot_fitting": "knee", "foot_pad": "knee",
    "disc_horn_knee": "knee", "passive_horn_knee": "knee",
    "passive_adapter_knee": "knee", "screw_knee": "knee",
}

# Slider clamps in DEGREES of offset from the home stance (home = 0).  Kept
# inside the real STS3215 leg envelope the MuJoCo model / workspace sweep use
# (yaw +/-35; femur absolute in [-80, 30] deg about the home -25; tibia relative
# in [0, 80] deg about the home 60), so an "Animate sweep" stays in a reachable,
# physical range.
_MOTION_LIMITS = {"yaw": (-35.0, 35.0), "hip": (-45.0, 30.0), "knee": (-30.0, 20.0)}


def _leg0_joint_frames() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """``{joint: (axis, origin)}`` in the LEG-0 world frame (pre per-leg yaw and
    pre standing-pose lift), matching the static placement chain exactly.

    * yaw  spins about world +Z through the yaw output (``edge``).
    * hip  swings about the coxa-rotated tangent ``Rz @ +Y`` through the hip
      anchor ``edge + Rz @ COXA_HIP_ANCHOR`` (the pivot of ``T_femur``).
    * knee swings about the same tangent through the knee anchor
      ``edge + Rz @ knee_local`` (the pivot of ``T_tibia``).
    """
    a = 0.5 * np.pi / 3.0
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    edge = np.array([apothem * np.cos(a), apothem * np.sin(a),
                     HP.CHASSIS_YAW_OUTPUT_Z])
    Rz = rotation_matrix(a, [0, 0, 1])[:3, :3]
    hip_local = np.array(HP.COXA_HIP_ANCHOR, float)
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([HP.FEMUR_LENGTH, 0.0, 0.0])
    tangent = Rz @ np.array([0.0, 1.0, 0.0])
    return {
        "yaw": (np.array([0.0, 0.0, 1.0]), edge.copy()),
        "hip": (tangent, edge + Rz @ hip_local),
        "knee": (tangent, edge + Rz @ knee_local),
    }


def _build_motion(instances_json, chassis_lift, legs):
    """Build the additive ``(joints, poses, animations)`` motion blocks for the
    placed scene.  ``joints`` carry real per-leg yaw/hip/knee axes + pivots in
    the final world frame and the exact instance ids each drives; ``poses`` are a
    handful of named whole-body configurations (a tripod-gait march + stance
    variants) expressed as degree offsets from the home stance; ``animations``
    is a single looping tripod-walk clip so the assembly can be seen WALKING."""
    frames = _leg0_joint_frames()

    # Group placed instance ids by (leg, link) using the authoritative ids the
    # builder already assigned (idx-prefixed, e.g. "045-hip_servo").
    ids_by: dict[tuple[int, str], list[str]] = {}
    for inst in instances_json:
        leg = inst.get("leg")
        if leg is None:
            continue
        link = _MOTION_LINK_OF_PARTTYPE.get(inst["partType"])
        if link is None:
            continue
        ids_by.setdefault((leg, link), []).append(inst["id"])

    joints: list[dict] = []
    parent_of = {"yaw": None, "hip": "yaw", "knee": "hip"}
    for i in legs:
        Ri = rotation_matrix(i * np.pi / 3.0, [0, 0, 1])[:3, :3]
        lift = np.array([0.0, 0.0, chassis_lift])
        for jname in ("yaw", "hip", "knee"):
            ids = ids_by.get((i, jname))
            if not ids:
                continue
            axis0, origin0 = frames[jname]
            axis = Ri @ axis0
            origin = Ri @ origin0 + lift
            lo, hi = _MOTION_LIMITS[jname]
            joint = {
                "id": f"L{i}-{jname}",
                "type": "revolute",
                "axis": [float(v) for v in axis],
                "origin": [float(v) for v in origin],
                "instances": ids,
                "limits": {"min": lo, "max": hi},
                "home": 0,
                "label": f"L{i} {jname}",
            }
            parent = parent_of[jname]
            if parent is not None:
                joint["parent"] = f"L{i}-{parent}"
            joints.append(joint)

    poses = _build_motion_poses(legs)
    animations = _build_motion_animations(legs)
    return joints, poses, animations


def _build_motion_poses(legs):
    """Named whole-body poses (degree offsets from the home stance).

    Sign convention is tied to the static ``R(+Y, phi)`` femur/tibia placement,
    where leg-local foot height is ``-FEMUR*sin(phi_femur) - TIBIA*sin(phi_tibia)``:
    a NEGATIVE hip offset rotates the femur tip UP (lifts the foot), so the
    tripod-swing poses lift legs {0,2,4} / {1,3,5} on alternate phases.  These
    are demonstrative keyframes; the continuous "Animate sweep" drives every DOF
    directly from the joint axes above."""
    A = [i for i in legs if i % 2 == 0]
    B = [i for i in legs if i % 2 == 1]

    def fill(which, *, yaw=0.0, hip=0.0, knee=0.0):
        out: dict[str, float] = {}
        for i in which:
            out[f"L{i}-yaw"] = yaw
            out[f"L{i}-hip"] = hip
            out[f"L{i}-knee"] = knee
        return out

    return [
        {"id": "home", "name": "Home (verified stance)", "jointValues": {}},
        {"id": "tripodA", "name": "Tripod A swing (legs 0·2·4 lift)",
         "jointValues": fill(A, hip=-18.0)},
        {"id": "tripodB", "name": "Tripod B swing (legs 1·3·5 lift)",
         "jointValues": fill(B, hip=-18.0)},
        {"id": "stepA", "name": "Tripod A step forward",
         "jointValues": {**fill(A, hip=-18.0, yaw=14.0),
                         **fill(B, yaw=-10.0)}},
        {"id": "stepB", "name": "Tripod B step forward",
         "jointValues": {**fill(B, hip=-18.0, yaw=14.0),
                         **fill(A, yaw=-10.0)}},
        {"id": "crouch", "name": "Crouch (body low)",
         "jointValues": fill(legs, hip=-12.0, knee=-12.0)},
        {"id": "tall", "name": "Stand tall",
         "jointValues": fill(legs, hip=10.0, knee=18.0)},
        {"id": "splay", "name": "Yaw splay",
         "jointValues": fill(legs, yaw=28.0)},
    ]


# Tripod-walk clip tuning (degree offsets from the home stance; matches the
# _MOTION_LIMITS envelope so the clip never drives a joint out of range).
_WALK_YAW_AMP = 14.0      # +/- fore/aft stride yaw
_WALK_LIFT_DEG = 18.0     # swing-phase hip lift (femur tip up = foot up)
_WALK_FRAMES = 24         # keyframes per cycle (inclusive end closes the loop)
_WALK_SECONDS = 2.4       # clip playback length for one gait cycle


def _walk_leg_offset(phase: float) -> tuple[float, float]:
    """``(yaw, hip)`` degree offset for one leg at gait ``phase`` in [0, 1).

    Stance half (phase 0..0.5): foot planted, yaw sweeps fore->aft as the body
    drives forward (hip level).  Swing half (0.5..1): foot lifts (smooth
    ``hip = -LIFT*sin``, negative = femur tip up) and recovers aft->fore."""
    if phase < 0.5:
        s = phase / 0.5
        return _WALK_YAW_AMP - 2.0 * _WALK_YAW_AMP * s, 0.0
    s = (phase - 0.5) / 0.5
    return -_WALK_YAW_AMP + 2.0 * _WALK_YAW_AMP * s, -_WALK_LIFT_DEG * np.sin(np.pi * s)


def _build_motion_animations(legs):
    """A single looping tripod-walk clip built from the same per-joint DOFs.

    Tripod A (legs 0·2·4) and B (legs 1·3·5) run half a cycle out of phase, so
    one tripod plants + drives the body forward while the other lifts + swings.
    Keyframes are degree offsets from the home stance (knee held at home), so the
    clip plays inside the verified joint envelope.  Mirrors the prototype_v1
    ``hexapod-prototype`` build's keyframed ``animations[]`` walk clip."""
    keyframes = []
    for k in range(_WALK_FRAMES + 1):      # inclusive end closes the loop
        frac = k / _WALK_FRAMES
        values: dict[str, float] = {}
        for i in legs:
            phase = (frac + (0.0 if i % 2 == 0 else 0.5)) % 1.0
            yaw, hip = _walk_leg_offset(phase)
            values[f"L{i}-yaw"] = round(float(yaw), 3)
            values[f"L{i}-hip"] = round(float(hip), 3)
            values[f"L{i}-knee"] = 0.0
        keyframes.append({"t": round(frac * _WALK_SECONDS, 4),
                          "jointValues": values})
    return [{
        "id": "walk",
        "name": "Walk (tripod gait)",
        "loop": True,
        "duration": _WALK_SECONDS,
        "keyframes": keyframes,
    }]


# ---------------------------------------------------------------------------
# Cable/harness routes (BuildViz ``routes[]`` -> the routing_reach gate)
# ---------------------------------------------------------------------------
#
# One route per joint, straight from the SOURCE OF TRUTH for cable reach --
# ``pi_control/wire_harness_plan.py`` (WIRE_HARNESS_PLAN).  Each route is the
# polyline the joint's 3-pin bus lead follows in CHASSIS frame: cradle wire-exit
# -> the leg's chassis_bottom drop slot -> (inter-plate dogleg, below) -> the
# leg's bus-bar landing post.  The scene builder only lifts those chassis-frame
# waypoints by the standing-pose ``chassis_lift`` -- no new geometry is derived
# here, so the routes[] can never drift from the harness plan.
#
# The length BUDGET per route is the plan's cable build: the 300 mm stock lead
# plus any 30 cm extensions its ``extension_required`` string calls for
# (currently none -- all 18 joints fit the stock lead).  BuildViz's
# ``routing_reach`` check then gates BOTH polyline length <= budget AND
# "no segment passes through a solid part".

_ROUTE_COLOR_DATA = "#eab308"   # amber: these are the serial-bus DATA leads

def _route_budget_mm(extension_required: str) -> float:
    """Stock lead + n x 30 cm extensions, parsed from the plan's BOM string
    ("STS3215 stock bus lead" / "+ 30 cm extension" / "+ N x 30 cm ...")."""
    import re
    if not extension_required.startswith("+"):
        return WHP.STOCK_PIGTAIL_MM
    m = re.match(r"\+ (\d+) x", extension_required)
    n_ext = int(m.group(1)) if m else 1
    return WHP.STOCK_PIGTAIL_MM + n_ext * WHP.EXTENSION_LENGTH_MM

# Per-leg intermediate waypoints (chassis frame) inserted between the drop
# slot and the bus landing where a STRAIGHT drop->landing segment would clip a
# solid in the routing_reach ray test.  Filled from the `buildviz check
# --checks routing_reach` obstruction report (Jul 2026): legs 2 + 3 (the -X
# pair) aim straight through the LiPo (a 105 x 35 x 25 box at x centre -25,
# spanning x in [-77.5, +27.5], |y| <= 17.5, z in [2, 27]), so their harness
# instead hugs the battery's +/-Y flank (|y| ~ 24 > 17.5) along the
# chassis_bottom top face until past the battery's +X face (x = 30 > 27.5),
# then cuts over to the bus landing.  Legs whose straight segment is already
# clear stay out of this table.
#
# The corridor: rise off the drop slot to z ~ 6.5 (probed clear band between
# the plate-top bosses, z <= 5, and the cradle overhangs, z >= 8.5) while
# turning radially inward, run along the battery's flank at that height, then
# cut over to the landing once past the battery's +X face.
_ROUTE_LEG_DOGLEGS: dict[int, list[tuple[float, float, float]]] = {
    2: [(-38.0, 24.5, 6.5), (30.0, 24.0, 6.5)],
    3: [(-38.0, -24.5, 6.5), (30.0, -24.0, 6.5)],
}

def _route_dogleg_points(entry: WHP.HarnessEntry) -> list[tuple[float, float, float]]:
    """Intermediate inter-plate waypoints for one plan entry (may be [])."""
    return _ROUTE_LEG_DOGLEGS.get(entry["leg_idx"], [])

def _build_routes(chassis_lift: float, legs: list[int]) -> list[dict]:
    """BuildViz ``routes[]`` for every WIRE_HARNESS_PLAN joint on ``legs``.

    Waypoints are the plan's chassis-frame source / drop-slot / bus-landing
    nodes (plus the inter-plate dogleg of ``_route_dogleg_points``), lifted to
    the scene's standing-pose world frame; the budget is the plan's cable
    build (stock lead + extensions)."""
    routes: list[dict] = []
    for entry in WHP.WIRE_HARNESS_PLAN:
        if entry["leg_idx"] not in legs:
            continue
        axis_label = {"hip_pitch": "hip_pitch"}.get(entry["axis"], entry["axis"])
        chassis_pts = [
            entry["source_xyz_chassis"],
            entry["via_chassis_drop_xyz"],
            *_route_dogleg_points(entry),
            entry["destination_xyz_chassis"],
        ]
        routes.append({
            "id": f"route-j{entry['joint_idx']:02d}",
            "points": [[float(x), float(y), float(z) + chassis_lift]
                       for x, y, z in chassis_pts],
            "maxLengthMm": _route_budget_mm(entry["extension_required"]),
            "label": (f"L{entry['leg_idx']} {axis_label} bus lead "
                      f"(servo ID {entry['servo_id']}, data)"),
            "color": _ROUTE_COLOR_DATA,
        })
    return routes

def main(single_leg: bool = False, motion: bool = True) -> None:
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    # Guard: the local-frame split reproduces the authoritative verifier poses.
    _assert_servo_placement()

    leg0 = _leg0_local_parts()        # (name, local_mesh, M0)  leg-0 frame
    body = _body_local_parts()        # (name, local_mesh, M0)  pre-lift

    # Lift so the lowest leg point sits on z = 0 (on the PLACED leg-0 parts).
    z_min = min(float(local.copy().apply_transform(M0).bounds[0][2])
                for _n, local, M0 in leg0)
    chassis_lift = -z_min
    Tlift = _trans([0.0, 0.0, chassis_lift])

    legs = [0] if single_leg else list(range(6))

    # Each entry: (name, leg, mesh_key, local_mesh, M_world, role, joint, color).
    # M_world places local_mesh into the final standing-pose WORLD frame; it is
    # shipped as the instance ``transform`` so BuildViz's pick read-out reports
    # CAD-local coordinates (worldToLocal = inverse(M_world)).
    plan: list[tuple] = []

    for name, local, M0 in body:
        plan.append((name, None, _scene_mesh_key(name), local, Tlift @ M0,
                     ROLE.get(name, "part"), None,
                     PALETTE.get(name, "#888888")))

    for i in legs:
        R = rotation_matrix(i * np.pi / 3.0, [0, 0, 1])
        for name, local, M0 in leg0:
            plan.append((name, i, _scene_mesh_key(name), local, Tlift @ R @ M0,
                         ROLE.get(name, "part"), None,
                         PALETTE.get(name, "#888888")))

    # COTS hardware placed independently in the SAME per-leg frame as the
    # verifier parts: every screw/nut/insert (registry) + the disc horns on
    # each servo output.  These are scene-only and NOT printable parts.
    for name, leg, joint, color, key, local, M in _fastener_instances():
        if single_leg and leg not in (0, None):
            continue
        plan.append((name, leg, key, local, Tlift @ M, "fastener", joint, color))
    for name, leg, joint, color, key, local, M in _disc_horn_instances(legs):
        if single_leg and leg != 0:
            continue
        plan.append((name, leg, key, local, Tlift @ M, "horn", joint, color))
    for name, leg, joint, color, key, local, M in _passive_horn_instances(legs):
        if single_leg and leg != 0:
            continue
        role = "horn" if key == "disc_horn" else "frame"
        plan.append((name, leg, key, local, Tlift @ M, role, joint, color))

    # Export each UNIQUE local mesh once (identical parts across legs share an
    # STL) and reference it by id; the per-instance transform does the placing.
    mesh_id_for: dict[str, str] = {}
    meshes_json: list[dict] = []
    instances_json: list[dict] = []
    tagged: list[tuple[str, int | None, trimesh.Trimesh]] = []
    allb: list = []
    for idx, (name, leg, key, local, M_world, role, joint, color) in enumerate(plan):
        if key not in mesh_id_for:
            fname = f"{key}.stl"
            local.export(STL_DIR / fname)
            mesh_id_for[key] = f"stl:{key}"
            meshes_json.append({"id": f"stl:{key}", "name": fname,
                                "url": f"{SCENE_ASSET_BASE}/{fname}"})
        # World mesh for the (world-space) overlap sidecar + camera framing.
        world = local.copy()
        world.apply_transform(M_world)
        allb.append(world.bounds)
        tagged.append((name, leg, world))
        instances_json.append({
            "id": f"{idx:03d}-{name}",
            "meshId": mesh_id_for[key],
            "name": f"L{leg} {name}" if leg is not None else name,
            "partType": name,
            "role": role,
            "leg": leg,
            "joint": joint,
            "cots": role in COTS_ROLES,
            "color": color,
            "transform": _col_major(M_world),
            "centroid": [float(v) for v in world.centroid],
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

    # ADDITIVE motion, baked into this ONE scene (there is no separate
    # scene_motion.json / prototype_sts3215_motion build any more).  Generating
    # the joints/poses/animations is essentially free (~0.01 s of joint/pose math
    # on the already-placed meshes), so we ALWAYS bake them in -- mirroring the
    # prototype_v1 hexapod-prototype build.  Old viewers ignore the extra keys and
    # render the identical static home pose.
    if motion:
        joints, poses, animations = _build_motion(
            instances_json, chassis_lift, legs)
        scene["joints"] = joints
        scene["poses"] = poses
        scene["animations"] = animations

    # ADDITIVE cable/harness routes (BuildViz routing_reach gate + tube render):
    # one per WIRE_HARNESS_PLAN joint, waypoints straight from the plan.
    scene["routes"] = _build_routes(chassis_lift, legs)

    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    msg = f"Wrote {OUT_DIR/'scene.json'}  ({len(instances_json)} instances, "
    if motion:
        moved = sum(len(j["instances"]) for j in scene["joints"])
        msg += (f"lift={chassis_lift:.1f}; motion: {len(scene['joints'])} joints "
                f"driving {moved} instances, {len(scene['poses'])} poses, "
                f"{len(scene['animations'])} walk clip; "
                f"{len(scene['routes'])} harness routes)")
    else:
        msg += (f"lift={chassis_lift:.1f}; motion OFF; "
                f"{len(scene['routes'])} harness routes)")
    print(msg)

    # BuildViz-compatibility contract (see ~/buildviz/BUILDVIZ_COMPATIBILITY.md):
    # write a design_spec.yaml that covers every scene partType, and copy the
    # project-authored assembly guide + BOM in under the contract's expected
    # names so full_robot_viz/ is a self-contained, compat-clean build dir.
    _write_design_spec(instances_json)
    project_dir = _HERE.parent
    for source_name, dest_name in (
        ("PROTOTYPE.md", "ASSEMBLY.md"),
        ("PROTOTYPE_BOM.md", "BOM.md"),
    ):
        source = project_dir / source_name
        if source.exists():
            shutil.copy2(source, OUT_DIR / dest_name)

    # Precompute the BuildViz checks sidecar (read on scene load).
    sidecar = _build_checks_sidecar(tagged, instances_json)
    (OUT_DIR / "buildviz_checks.json").write_text(json.dumps(sidecar, indent=2))
    s = sidecar["summary"]
    print(f"Wrote {OUT_DIR/'buildviz_checks.json'}  "
          f"({s['fail']} fail / {s['warn']} warn / {s['pass']} pass "
          f"of {s['total']} checks)")


if __name__ == "__main__":
    # Motion is ALWAYS baked into scene.json (generation is free).  ``--no-motion``
    # is a debug escape hatch for a static-only scene; there is no ``--motion``
    # flag any more because motion is the default.
    main(single_leg="--single" in sys.argv,
         motion="--no-motion" not in sys.argv)
