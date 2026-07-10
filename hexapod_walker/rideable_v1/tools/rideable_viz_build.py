#!/usr/bin/env python3
"""Build a BuildViz scene for the RIDEABLE hexapod (`rideable_v1`).

Unlike ``prototype_sts3215`` (which exports verified parametric CAD), the
rideable design is a mechanical-design DRAFT captured only in markdown
(README.md / DRIVETRAIN.md / STRUCTURE.md / POWER_SYSTEM.md / BOM.md).  So this
tool GENERATES primitive stand-in geometry — triangulated 4130 space-frame legs
(3-chord trusses), cylindrical COTS actuators / secondary-reduction pulleys /
fail-safe brakes, and a welded rider chassis (hex tube frame + deck + saddle +
battery) — placed at rideable scale directly from the numbers in
``design_spec.yaml`` (which distils the decided parameters from the docs).

It writes a self-contained BuildViz build under ``rideable_v1/full_robot_viz/``
(mirroring the sts3215 layout):

    full_robot_viz/scene.json      BuildViz manifest (meshes[] + instances[])
    full_robot_viz/stl/*.stl       one world-baked STL per instance

Each instance mesh is baked in WORLD coordinates and shipped with an identity
transform, so every consumer agrees on placement: the web viewer, the PyVista
renderer (tools/render_rideable.py), and the generic geometry verifier
(../prototype_sts3215/tools/buildviz_checks.py).

Run:
    ./run.sh hexapod_walker/rideable_v1/tools/rideable_viz_build.py

Then VIEW it in the ONE machine-wide BuildViz hub (default port 5183) -- never
start a per-project dev server on 5175/etc:
    make -C hexapod_walker/rideable_v1 view-buildviz   # register into the hub + open
    # http://127.0.0.1:5183/?build=rideable_v1
"""

from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
from trimesh.creation import box as _box
from trimesh.creation import cylinder as _cyl

_HERE = Path(__file__).resolve().parent
OUT_DIR = _HERE.parent / "full_robot_viz"
STL_DIR = OUT_DIR / "stl"

# ---------------------------------------------------------------------------
# Design parameters (mm / deg) — mirror design_spec.yaml, sourced from the docs.
# ---------------------------------------------------------------------------
N_LEGS = 6
HIP_YAW_RING_R = 520.0          # hip-yaw axis radius from body centre (assumption)
COXA_LEN = 150.0                # hip-yaw -> hip-pitch  (brief)
FEMUR_LEN = 600.0              # hip-pitch -> knee     (brief)
TIBIA_LEN = 800.0             # knee -> foot          (brief)
FOOT_OUT = 280.0               # tucked foot, outboard of the hip-pitch joint
FOOT_DOWN = 560.0              # tucked foot, below the hip-pitch joint

FRAME_DEPTH = 150.0            # 4130 truss depth h (STRUCTURE.md)
CHORD_OD = 38.0               # main chord tube (STRUCTURE.md member schedule)
WEB_OD = 25.0                 # diagonal / web member

# COTS actuator envelopes (DRIVETRAIN.md / BOM.md; dia's are assumptions).
X15_DIA, X15_T = 100.0, 60.0   # RMD-X15-450 (hip-pitch, knee)
X8_DIA, X8_T = 80.0, 45.0      # RMD-X8-120 (hip-yaw)
SEC_DIA, SEC_T = 300.0, 45.0   # 6:1 HTD-14M driven pulley (large)
BRAKE_DIA, BRAKE_T = 90.0, 50.0
FOOT_DIA, FOOT_T = 120.0, 40.0

# Colours.
C_X8 = "#3a5a80"          # hip-yaw actuator (blue-grey)
C_X15 = "#2e2e33"         # RMD-X15 (dark)
C_SEC = "#95a5a6"         # belt-reduction pulley housing (grey)
C_BRAKE = "#f1c40f"       # fail-safe brake — bright yellow (safety call-out)
C_STEEL = "#8b9bab"       # 4130 chromoly space-frame
C_STEEL_HI = "#7a8a9a"    # tibia (slightly darker)
C_FOOT = "#2b2b2b"        # urethane pad
C_CHASSIS = "#6b7c8c"     # welded chassis tube
C_DECK = "#c8ccd0"        # aluminium deck plate
C_SADDLE = "#191919"      # saddle leather
C_POST = "#4a4a4a"        # seatpost
C_CTRL = "#3a3a3a"        # handlebar
C_PEG = "#555555"         # footrest
C_BATT = "#2e7d32"        # battery (green)
C_EBAY = "#6a4fb0"        # electronics (purple)

COTS_ROLES = {"motor", "brake", "bearing", "electronics", "drivetrain", "rider"}

# Intended part-type matings that overlap by design (BuildViz
# checksConfig.ignoreOverlapPairs).  Anything NOT listed that interpenetrates
# is a real clash the verifier will flag.
INTENDED_OVERLAP_PAIRS = [
    ("chassis_frame", "hip_yaw_actuator"),
    ("hip_yaw_actuator", "coxa_frame"),
    ("hip_yaw_actuator", "hip_secondary"),   # large 6:1 pulley sits just above the yaw actuator
    ("coxa_frame", "hip_pitch_actuator"),
    ("coxa_frame", "hip_secondary"),         # 6:1 driven pulley shares the hip node with the coxa
    ("coxa_frame", "femur_frame"),           # coxa + femur trusses meet at the hip-pitch node
    ("hip_pitch_actuator", "hip_secondary"),
    ("hip_pitch_actuator", "femur_frame"),
    ("hip_secondary", "femur_frame"),
    ("femur_frame", "tibia_frame"),          # folded-knee trusses meet at the knee node
    ("femur_frame", "knee_actuator"),
    ("femur_frame", "knee_secondary"),
    ("knee_actuator", "knee_secondary"),
    ("knee_actuator", "knee_brake"),
    ("knee_actuator", "tibia_frame"),
    ("knee_secondary", "tibia_frame"),
    ("tibia_frame", "foot"),
    ("chassis_frame", "coxa_frame"),
    ("chassis_frame", "deck_plate"),
    ("chassis_frame", "battery"),
    ("chassis_frame", "e_bay"),
    ("chassis_frame", "saddle_post"),
    ("chassis_frame", "footpeg"),
    ("deck_plate", "saddle_post"),
    ("deck_plate", "battery"),
    ("deck_plate", "e_bay"),
    ("deck_plate", "controls"),
    ("deck_plate", "footpeg"),
    ("saddle_post", "saddle"),
    ("battery", "e_bay"),
]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tube(p0, p1, radius) -> trimesh.Trimesh:
    """Solid cylinder spanning p0->p1."""
    return _cyl(radius=radius, segment=np.array([p0, p1], dtype=float))


def _actuator(center, axis, dia, thick) -> trimesh.Trimesh:
    """Disc/cylinder actuator body of ``thick`` along unit ``axis``."""
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c = np.asarray(center, float)
    return _tube(c - axis * thick / 2.0, c + axis * thick / 2.0, dia / 2.0)


def _frame_vectors(axis):
    """Return (u, v, w): u along axis, v/w perpendicular."""
    u = np.asarray(axis, float)
    u = u / np.linalg.norm(u)
    ref = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    v = np.cross(u, ref)
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    return u, v, w


def _truss(p0, p1, depth, chord_r, web_r, min_bays=1) -> trimesh.Trimesh:
    """A triangulated 3-chord space-frame truss from p0 to p1.

    Three chord tubes run the full length at the corners of an equilateral
    triangle (radius ~depth/2 about the centreline); per bay we add a corner
    ring plus a zig-zag diagonal on each of the 3 faces.  Everything is merged
    into ONE mesh so intra-truss member overlaps don't count as clashes.
    """
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    u, v, w = _frame_vectors(p1 - p0)
    r_corner = depth / 2.0
    angs = np.deg2rad([90.0, 210.0, 330.0])
    corners = [np.cos(a) * v * r_corner + np.sin(a) * w * r_corner for a in angs]

    n_bays = max(min_bays, int(round(L / 160.0)))
    stations = [p0 + (p1 - p0) * (k / n_bays) for k in range(n_bays + 1)]

    parts: list[trimesh.Trimesh] = []
    # 3 full-length chords
    for c in corners:
        parts.append(_tube(p0 + c, p1 + c, chord_r))
    # rings at every station + diagonals per bay
    for k, s in enumerate(stations):
        for j in range(3):
            parts.append(_tube(s + corners[j], s + corners[(j + 1) % 3], web_r))
        if k < n_bays:
            s_next = stations[k + 1]
            for j in range(3):
                parts.append(_tube(s + corners[j], s_next + corners[(j + 1) % 3], web_r))
    return trimesh.util.concatenate(parts)


def _hex_chassis(z_bot, z_top, r_vertex, hub_r, tube_r) -> trimesh.Trimesh:
    """Welded hex tube frame: two hexagonal rings (bottom/top) at the 6 leg
    bearings + verticals + face diagonals + radial spokes to a centre hub."""
    verts = []
    for i in range(N_LEGS):
        a = i * 2 * np.pi / N_LEGS
        p = np.array([r_vertex * np.cos(a), r_vertex * np.sin(a), 0.0])
        verts.append(p)
    parts: list[trimesh.Trimesh] = []
    hub_bot = np.array([0.0, 0.0, z_bot])
    hub_top = np.array([0.0, 0.0, z_top])
    for i in range(N_LEGS):
        vb = verts[i] + [0, 0, z_bot]
        vt = verts[i] + [0, 0, z_top]
        vb_n = verts[(i + 1) % N_LEGS] + [0, 0, z_bot]
        vt_n = verts[(i + 1) % N_LEGS] + [0, 0, z_top]
        parts.append(_tube(vb, vb_n, tube_r))     # bottom ring
        parts.append(_tube(vt, vt_n, tube_r))     # top ring
        parts.append(_tube(vb, vt, tube_r))       # vertical
        parts.append(_tube(vb, vt_n, tube_r))     # face diagonal
        parts.append(_tube(hub_top, vt, tube_r))  # top radial spoke
        parts.append(_tube(hub_bot, vb, tube_r))  # bottom radial spoke
    parts.append(_tube(hub_bot, hub_top, tube_r))  # centre column
    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
# Leg kinematics (2-link IK, knee-up branch) — see design_spec.yaml.
# ---------------------------------------------------------------------------
def _solve_leg():
    """Return leg-0 joint points in the leg frame (x out, z up, hip-yaw z=0).

    Places the foot TUCKED at (FOOT_OUT, -FOOT_DOWN) relative to the hip-pitch
    joint using femur/tibia lengths exactly (knee-up branch)."""
    yaw = np.array([HIP_YAW_RING_R, 0.0, 0.0])
    hip = yaw + np.array([COXA_LEN, 0.0, 0.0])
    tx, tz = FOOT_OUT, -FOOT_DOWN
    d = float(np.hypot(tx, tz))
    L1, L2 = FEMUR_LEN, TIBIA_LEN
    if not (abs(L1 - L2) <= d <= L1 + L2):
        raise ValueError(f"foot target unreachable: d={d:.1f}, L1={L1}, L2={L2}")
    phi = np.arctan2(tz, tx)
    a = np.arccos(np.clip((d * d + L1 * L1 - L2 * L2) / (2 * d * L1), -1, 1))
    femur_ang = phi + a                         # knee-up: femur points up-out
    knee = hip + L1 * np.array([np.cos(femur_ang), 0.0, np.sin(femur_ang)])
    foot = hip + np.array([tx, 0.0, tz])
    knee_int = np.degrees(np.arccos(np.clip(
        (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2), -1, 1)))
    return {"yaw": yaw, "hip": hip, "knee": knee, "foot": foot,
            "femur_deg": np.degrees(femur_ang), "knee_int_deg": knee_int, "d": d}


def _leg0_instances(k):
    """(name, role, color, mesh) for every leg-0 part, in the leg-0 frame."""
    yaw, hip, knee, foot = k["yaw"], k["hip"], k["knee"], k["foot"]
    pitch_axis = np.array([0.0, 1.0, 0.0])       # hip-pitch / knee axis (tangential)

    out = []
    # hip-yaw actuator (vertical axis), sits at the ring, output up into coxa.
    out.append(("hip_yaw_actuator", "motor", C_X8,
                _actuator(yaw + [0, 0, X8_T / 2], [0, 0, 1], X8_DIA, X8_T)))
    # coxa truss: yaw output -> hip-pitch joint
    out.append(("coxa_frame", "frame", C_STEEL,
                _truss(yaw + [0, 0, X8_T], hip, 0.75 * FRAME_DEPTH, CHORD_OD / 2, WEB_OD / 2)))
    # hip-pitch actuator + 6:1 driven pulley on the joint axis
    out.append(("hip_pitch_actuator", "motor", C_X15,
                _actuator(hip - pitch_axis * (SEC_T / 2 + X15_T / 2), pitch_axis, X15_DIA, X15_T)))
    out.append(("hip_secondary", "drivetrain", C_SEC,
                _actuator(hip, pitch_axis, SEC_DIA, SEC_T)))
    # femur truss: hip -> knee
    out.append(("femur_frame", "frame", C_STEEL,
                _truss(hip, knee, FRAME_DEPTH, CHORD_OD / 2, WEB_OD / 2)))
    # knee actuator + 6:1 driven pulley + fail-safe brake on the fast shaft
    out.append(("knee_actuator", "motor", C_X15,
                _actuator(knee - pitch_axis * (SEC_T / 2 + X15_T / 2), pitch_axis, X15_DIA, X15_T)))
    out.append(("knee_secondary", "drivetrain", C_SEC,
                _actuator(knee, pitch_axis, SEC_DIA, SEC_T)))
    out.append(("knee_brake", "brake", C_BRAKE,
                _actuator(knee - pitch_axis * (SEC_T / 2 + X15_T + BRAKE_T / 2),
                          pitch_axis, BRAKE_DIA, BRAKE_T)))
    # tibia truss: knee -> foot
    out.append(("tibia_frame", "frame", C_STEEL_HI,
                _truss(knee, foot, 0.85 * FRAME_DEPTH, CHORD_OD / 2, WEB_OD / 2)))
    # foot pad (top at the ankle/foot point)
    out.append(("foot", "frame", C_FOOT,
                _actuator(foot + [0, 0, -FOOT_T / 2], [0, 0, 1], FOOT_DIA, FOOT_T)))
    return out


def _body_instances():
    """(name, role, color, mesh) for the chassis + rider interface, baseline
    frame (hip-yaw axis at z=0)."""
    z_bot, z_top = 40.0, 190.0
    deck_t = 25.0
    deck_bot = z_top
    deck_ctr_z = deck_bot + deck_t / 2.0
    deck_top = deck_bot + deck_t

    out = []
    out.append(("chassis_frame", "chassis", C_CHASSIS,
                _hex_chassis(z_bot, z_top, r_vertex=HIP_YAW_RING_R, hub_r=60, tube_r=20)))
    out.append(("deck_plate", "chassis", C_DECK,
                _box(extents=(780, 560, deck_t),
                     transform=_trans([0, 0, deck_ctr_z]))))
    # battery low + central
    out.append(("battery", "electronics", C_BATT,
                _box(extents=(420, 300, 150), transform=_trans([40, 0, 115]))))
    # electronics bay on the deck, rear
    out.append(("e_bay", "electronics", C_EBAY,
                _box(extents=(240, 200, 120), transform=_trans([-230, 0, deck_top + 60]))))
    # saddle on a suspension post (slightly rear)
    post_top = deck_top + 190.0
    out.append(("saddle_post", "rider", C_POST,
                _tube([-40, 0, deck_top], [-40, 0, post_top], 26)))
    out.append(("saddle", "rider", C_SADDLE,
                _box(extents=(360, 300, 90), transform=_trans([-40, 0, post_top + 45]))))
    # handlebar (post + crossbar) at the front
    bar_z = deck_top + 250.0
    ctrl = trimesh.util.concatenate([
        _tube([300, 0, deck_top], [300, 0, bar_z], 20),
        _tube([300, -190, bar_z], [300, 190, bar_z], 18),
    ])
    out.append(("controls", "rider", C_CTRL, ctrl))
    # footrests either side
    out.append(("footpeg", "rider", C_PEG,
                _box(extents=(150, 90, 30), transform=_trans([-40, 330, deck_top - 10]))))
    out.append(("footpeg", "rider", C_PEG,
                _box(extents=(150, 90, 30), transform=_trans([-40, -330, deck_top - 10]))))
    return out


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


def _rot_z(theta) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


# ---------------------------------------------------------------------------
# Scene assembly
# ---------------------------------------------------------------------------
def main() -> None:
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    k = _solve_leg()
    leg0 = _leg0_instances(k)
    body = _body_instances()

    # Build every world-baked mesh (baseline frame), collect for lift.
    baked: list[tuple[str, str, str, trimesh.Trimesh]] = []
    for name, role, color, mesh in body:
        baked.append((name, role, color, mesh))
    for i in range(N_LEGS):
        R = _rot_z(i * 2 * np.pi / N_LEGS)
        for name, role, color, mesh in leg0:
            m = mesh.copy()
            m.apply_transform(R)
            baked.append((name, role, color, m))

    # Lift so the lowest point sits on z = 0 (ground).
    z_min = min(float(m.bounds[0][2]) for _n, _r, _c, m in baked)
    Tlift = _trans([0.0, 0.0, -z_min])

    instances_json: list[dict] = []
    meshes_json: list[dict] = []
    all_bounds: list[np.ndarray] = []
    identity = [float(v) for v in np.eye(4).flatten("F")]

    for idx, (name, role, color, mesh) in enumerate(baked):
        m = mesh.copy()
        m.apply_transform(Tlift)
        all_bounds.append(m.bounds)
        fname = f"{idx:03d}_{name}.stl"
        m.export(STL_DIR / fname)
        mesh_id = f"stl:{idx:03d}_{name}"
        meshes_json.append({"id": mesh_id, "name": fname, "url": f"stl/{fname}"})
        instances_json.append({
            "id": f"{idx:03d}-{name}",
            "meshId": mesh_id,
            "name": name.replace("_", " "),
            "partType": name,
            "role": role,
            "cots": role in COTS_ROLES,
            "color": color,
            "transform": identity,
            "centroid": [float(v) for v in m.centroid],
        })

    b = np.array(all_bounds)
    center = [float(v) for v in (b[:, 0, :].min(0) + b[:, 1, :].max(0)) / 2.0]

    scene = {
        "name": "Rideable hexapod (rideable_v1) — design-draft visualization",
        "source": str(OUT_DIR),
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": center,
        "checksConfig": {
            "overlapMm3": 3000.0,
            "pitchMm": 12.0,
            "ignoreOverlapPairs": sorted(sorted(p) for p in INTENDED_OVERLAP_PAIRS),
        },
        "meshes": meshes_json,
        "instances": instances_json,
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))

    # Make full_robot_viz/ a self-contained, BuildViz-COMPATIBLE build dir
    # (see ~/buildviz/BUILDVIZ_COMPATIBILITY.md): copy the project's
    # design_spec.yaml (semantic source of truth) plus the project-authored
    # ASSEMBLY.md + BOM.md in next to scene.json/stl so `buildviz compat`
    # and `register` see all five required files in one directory.  The
    # project docs at rideable_v1/ stay the single source; these are copies.
    project_dir = _HERE.parent
    for doc_name in ("design_spec.yaml", "ASSEMBLY.md", "BOM.md"):
        source = project_dir / doc_name
        if source.exists():
            dest = OUT_DIR / doc_name
            shutil.copy2(source, dest)
            # copy2 preserves the source mtime; stamp design_spec.yaml to "now"
            # so `buildviz compat`'s staleness heuristic doesn't flag it as
            # older than the scene.json/STLs we just regenerated in this pass.
            if doc_name == "design_spec.yaml":
                os.utime(dest, None)

    dims = b[:, 1, :].max(0) - b[:, 0, :].min(0)
    print(f"Wrote {OUT_DIR/'scene.json'}")
    print(f"  {len(instances_json)} instances, {len(meshes_json)} meshes")
    print(f"  leg IK: femur {k['femur_deg']:.1f}deg above horiz, "
          f"knee interior {k['knee_int_deg']:.1f}deg, hip->foot {k['d']:.0f} mm")
    print(f"  footprint bbox (mm): {dims[0]:.0f} x {dims[1]:.0f} wide, "
          f"{dims[2]:.0f} tall; ground-lift {-z_min:.0f} mm")


if __name__ == "__main__":
    main()
