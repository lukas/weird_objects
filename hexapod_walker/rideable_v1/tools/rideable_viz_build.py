#!/usr/bin/env python3
"""Build a BuildViz scene for the RIDEABLE hexapod (`rideable_v1`).

Unlike ``prototype_sts3215`` (which exports verified parametric CAD), the
rideable design is a mechanical-design DRAFT captured only in markdown
(README.md / DRIVETRAIN.md / STRUCTURE.md / POWER_SYSTEM.md / BOM.md).  So this
tool GENERATES primitive stand-in geometry — triangulated 4130 space-frame legs
(3-chord trusses), cylindrical COTS actuators / secondary-reduction sprockets /
fail-safe brakes, and a welded rider chassis (hex tube frame + deck + saddle +
battery) — placed at rideable scale directly from the numbers in
``design_spec.yaml`` (which distils the decided parameters from the docs).

The joint layout is physically articulable (and verified by the BuildViz
joint-articulation checks):

  * every revolute DOF is declared in an additive ``joints[]`` block
    (6 hip-yaw + 6 hip-pitch + 6 knee, world axis + origin + distal links);
  * the driven sprocket of each 6:1 secondary sits ON the joint axis and is
    bolted to the MOVING (distal) link; the actuator + fail-safe brake are
    offset-mounted on the PARENT link (chain/belt run not modelled);
  * parent and child links meet only through a coaxial pivot pin (a
    ``fasteners:``-namespaced instance, exempt as the joint's own axle) with
    real clearance everywhere else — no link is fused or bolted across a DOF;
  * intended mounting interfaces are flat-on-flat with a nominal 0.1 mm
    seat so the ``mating_contact`` gate reads them as in-contact.

It writes a self-contained BuildViz build under ``rideable_v1/full_robot_viz/``
(mirroring the sts3215 layout):

    full_robot_viz/scene.json      BuildViz manifest (meshes[] + instances[] + joints[])
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
HIP_DROP = 80.0                # hip-pitch axis sits this far below the yaw plane
                               # (drops the Ø300 sprocket clear of the chassis)

FRAME_DEPTH = 150.0            # 4130 truss depth h (STRUCTURE.md)
CHORD_OD = 38.0               # main chord tube (STRUCTURE.md member schedule)
WEB_OD = 25.0                 # diagonal / web member

# COTS actuator envelopes (DRIVETRAIN.md / BOM.md; dia's are assumptions).
X15_DIA, X15_T = 100.0, 60.0   # RMD-X15-450 (hip-pitch, knee)
X8_DIA, X8_T = 80.0, 60.0      # RMD-X8-120 (hip-yaw)
SEC_DIA, SEC_T = 300.0, 50.0   # 6:1 driven sprocket (#40 72T, PCD ~291) + carrier
BRAKE_DIA, BRAKE_T = 90.0, 50.0
FOOT_DIA, FOOT_T = 120.0, 40.0

# Joint-node geometry (leg frame; the pitch/knee axis is the local +Y).
HUB_R = 45.0                   # moving-link pivot hub radius
HUB_Y = (-45.0, 40.0)          # hub extent along the joint axis (asymmetric:
                               # the -Y face carries the driven sprocket)
PIN_R = 15.0                   # pivot pin (coaxial fastener) radius
PIN_Y = (-40.0, 75.0)          # pin extent: grips the hub AND the +Y clevis plate
SPROCKET_Y = (-95.0, -44.9)    # sprocket: flush-seated 0.1 mm onto the hub -Y face
CLEVIS_PLATE_Y = (45.0, 70.0)  # parent-side bearing plate (single-sided clevis)
CROSSBAR_S = (200.0, 240.0)    # parent crossbar span, measured back from the joint
PLATE_S = (-35.0, 240.0)       # clevis plate span, measured back from the joint
DRIVE_STANDOFF = 350.0         # knee actuator centre, back from the knee along the femur
HIP_DRIVE_X = 400.0            # hip actuator centre (leg frame x; ~270 mm inboard of the hip)
DRIVE_Y = (-160.0, -100.0)     # actuator body extent along the joint axis
BRAKE_Y = (-210.0, -159.9)     # brake, flush-seated on the actuator outboard face
SEAT = 0.1                     # nominal interference at intended flat seats (mm)

# Truss setbacks so nothing structural crosses a declared joint: each truss
# stops well clear of the Ø300 sprocket disc and of the opposing link across
# the folded knee, and a Ø70 collar (part of the same link) bridges the gap.
FEMUR_TRUSS_S0, FEMUR_TRUSS_S1 = 185.0, 230.0   # from hip / short of knee
TIBIA_TRUSS_S0 = 230.0                           # from knee
TIBIA_TRUSS_S1 = 100.0                           # short of the foot point
COLLAR_R = 35.0

# Chassis / rider interface (leg frame z: 0 = yaw-actuator mount plane).
YAW_ACT_Z = (0.0, 60.0 + SEAT)     # RMD-X8, output DOWN, flush under the vertex plate
VERTEX_PLATE_Z = (60.0, 95.0)      # chassis vertex mounting plate
COXA_HUB_Z = (-60.0, -5.0)         # coxa yaw hub (5 mm below the actuator)
YAW_PIN_Z = (-40.0, 30.0)          # yaw output shaft (coaxial fastener)
COXA_BEAM = dict(x=(330.0, 530.0), y=40.0, z=(-110.0, -50.0))
RING_Z = (110.0, 260.0)            # chassis hex ring centrelines (tube r=20)
RING_TUBE_R = 20.0
DECK = dict(x=900.0, y=660.0, t=25.0)

# Colours.
C_X8 = "#3a5a80"          # hip-yaw actuator (blue-grey)
C_X15 = "#2e2e33"         # RMD-X15 (dark)
C_SEC = "#95a5a6"         # driven sprocket / secondary (grey)
C_BRAKE = "#f1c40f"       # fail-safe brake — bright yellow (safety call-out)
C_PIN = "#b8860b"         # pivot pins / output shafts
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

# Intended MOUNTING interfaces (flat seats, nominally 0.1 mm interference).
# These are BuildViz checksConfig.ignoreOverlapPairs entries: the
# mating_contact gate verifies each pair really is seated (|gap| and
# penetration within tolerance), and the repo-side voxel overlap check skips
# them.  The pivot-pin pairs are listed only for the repo-side checker
# (BuildViz already exempts fasteners); the pins intentionally pass through
# the links they join.
INTENDED_OVERLAP_PAIRS = [
    ("chassis_frame", "hip_yaw_actuator"),      # X8 flange under the vertex plate
    ("femur_frame", "hip_secondary"),           # hip driven sprocket on the femur hub
    ("coxa_frame", "hip_pitch_actuator"),       # hip X15 on the coxa mount plate
    ("hip_pitch_actuator", "hip_brake"),        # hip brake on the X15 tail shaft
    ("tibia_frame", "knee_secondary"),          # knee driven sprocket on the tibia hub
    ("femur_frame", "knee_actuator"),           # knee X15 on the femur mount plate
    ("knee_actuator", "knee_brake"),            # knee brake on the X15 tail shaft
    ("tibia_frame", "foot"),                    # foot disc under the ankle post
    ("chassis_frame", "deck_plate"),
    ("chassis_frame", "battery"),               # battery tray on the lower spokes
    ("deck_plate", "e_bay"),
    ("deck_plate", "saddle_post"),
    ("deck_plate", "controls"),
    ("deck_plate", "footpeg"),
    ("saddle_post", "saddle"),
    # pivot pins (fasteners) — repo-side checker only:
    ("hip_yaw_actuator", "yaw_pivot_pin"),
    ("coxa_frame", "yaw_pivot_pin"),
    ("coxa_frame", "hip_pivot_pin"),
    ("femur_frame", "hip_pivot_pin"),
    ("femur_frame", "knee_pivot_pin"),
    ("tibia_frame", "knee_pivot_pin"),
]

# Declared joint travel (deg, about the home/static pose).  The docs leave the
# exact ranges open; these are assumptions consistent with the tucked stance
# (README.md §3) and the ±0.3 m stride excursion.
YAW_LIMITS = (-35.0, 35.0)
HIP_LIMITS = (-45.0, 30.0)
KNEE_LIMITS = (-40.0, 40.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tube(p0, p1, radius, sections=32) -> trimesh.Trimesh:
    """Solid cylinder spanning p0->p1."""
    return _cyl(radius=radius, segment=np.array([p0, p1], dtype=float),
                sections=sections)


def _disc(center, axis, dia, thick, lo=None, hi=None, sections=48) -> trimesh.Trimesh:
    """Disc/cylinder of ``thick`` along unit ``axis`` centred at ``center``,
    or spanning [lo, hi] along the axis when given."""
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c = np.asarray(center, float)
    if lo is not None and hi is not None:
        return _tube(c + axis * lo, c + axis * hi, dia / 2.0, sections)
    return _tube(c - axis * thick / 2.0, c + axis * thick / 2.0, dia / 2.0, sections)


def _obox(center, half_extents, axes=None) -> trimesh.Trimesh:
    """Box with half-extents, optionally oriented by a 3x3 column-axes matrix."""
    T = np.eye(4)
    if axes is not None:
        T[:3, :3] = np.asarray(axes, float)
    T[:3, 3] = np.asarray(center, float)
    return _box(extents=2.0 * np.asarray(half_extents, float), transform=T)


def _union(parts: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    """Boolean-union a list of primitive solids into ONE watertight body (so a
    truss/chassis ships as a manifold mesh with no internal self-intersections).

    Uses manifold3d NATIVELY (build Manifolds, batch-union, take the output
    mesh): the Manifold output is guaranteed 2-manifold, and empirically it
    survives the float32 STL round-trip with 0 open / 0 non-manifold edges,
    unlike trimesh's post-boolean scrubbing.  Falls back to a raw concatenate
    if the engine is unavailable (checks would then flag the overlap debris).
    """
    if len(parts) == 1:
        return parts[0]
    try:
        import manifold3d as _m3d

        solids = [
            _m3d.Manifold(_m3d.Mesh(
                vert_properties=np.asarray(p.vertices, dtype=np.float32),
                tri_verts=np.asarray(p.faces, dtype=np.uint32),
            ))
            for p in parts
        ]
        merged = _m3d.Manifold.batch_boolean(solids, _m3d.OpType.Add)
        # Collapse the near-degenerate sliver triangles booleans leave along
        # coplanar seams (they round to zero-area / self-touching after the
        # float32 STL export); 0.01 mm is far below any real feature here.
        merged = merged.simplify(0.01)
        out = merged.to_mesh()
        return trimesh.Trimesh(
            vertices=np.asarray(out.vert_properties)[:, :3],
            faces=np.asarray(out.tri_verts),
        )
    except BaseException:  # noqa: BLE001 - engine availability differs per env
        return trimesh.util.concatenate(parts)


def _frame_vectors(axis):
    """Return (u, v, w): u along axis, v/w perpendicular."""
    u = np.asarray(axis, float)
    u = u / np.linalg.norm(u)
    ref = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    v = np.cross(u, ref)
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    return u, v, w


def _truss_parts(p0, p1, depth, chord_r, web_r, min_bays=1) -> list[trimesh.Trimesh]:
    """Members of a triangulated 3-chord space-frame truss from p0 to p1.

    Three chord tubes run the full length at the corners of an equilateral
    triangle (radius ~depth/2 about the centreline); per bay we add a corner
    ring plus a zig-zag diagonal on each of the 3 faces.
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
    for c in corners:
        parts.append(_tube(p0 + c, p1 + c, chord_r))
    for k, s in enumerate(stations):
        for j in range(3):
            parts.append(_tube(s + corners[j], s + corners[(j + 1) % 3], web_r))
        if k < n_bays:
            s_next = stations[k + 1]
            for j in range(3):
                parts.append(_tube(s + corners[j], s_next + corners[(j + 1) % 3], web_r))
    return parts


def _hex_chassis() -> trimesh.Trimesh:
    """Welded hex tube frame: two hexagonal rings at the 6 leg vertices +
    verticals + face diagonals + radial spokes to a centre hub column, plus a
    flat vertex mounting plate under each hip-yaw actuator."""
    z_bot, z_top = RING_Z
    r_vertex, tube_r = HIP_YAW_RING_R, RING_TUBE_R
    verts = []
    for i in range(N_LEGS):
        a = i * 2 * np.pi / N_LEGS
        verts.append(np.array([r_vertex * np.cos(a), r_vertex * np.sin(a), 0.0]))
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
        # Flat mounting plate under the vertex: the hip-yaw actuator flange
        # seats on its underside (reaches up past the ring tube bottom so the
        # union stays one body).
        parts.append(_obox(verts[i] + [0, 0, (VERTEX_PLATE_Z[0] + VERTEX_PLATE_Z[1]) / 2],
                           [55.0, 55.0, (VERTEX_PLATE_Z[1] - VERTEX_PLATE_Z[0]) / 2]))
    parts.append(_tube(hub_bot, hub_top, tube_r))  # centre column
    return _union(parts)


# ---------------------------------------------------------------------------
# Leg kinematics (2-link IK, knee-up branch) — see design_spec.yaml.
# ---------------------------------------------------------------------------
def _solve_leg():
    """Return leg-0 joint points in the leg frame (x out, z up, hip-yaw z=0).

    Places the foot TUCKED at (FOOT_OUT, -FOOT_DOWN) relative to the hip-pitch
    joint using femur/tibia lengths exactly (knee-up branch)."""
    yaw = np.array([HIP_YAW_RING_R, 0.0, 0.0])
    hip = yaw + np.array([COXA_LEN, 0.0, -HIP_DROP])
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


def _pitch_node_parent(J, back_dir):
    """Parent-side clevis pieces at a pitch joint ``J``: a crossbar well clear
    of the sprocket disc + a single +Y bearing plate hosting the pivot pin.
    Returned as raw solids to merge into the parent link's mesh."""
    J = np.asarray(J, float)
    a = np.asarray(back_dir, float)
    a = a / np.linalg.norm(a)
    y = np.array([0.0, 1.0, 0.0])
    w = np.cross(a, y)
    axes = np.column_stack([a, y, w])
    s0, s1 = CROSSBAR_S
    crossbar = _obox(J + a * (s0 + s1) / 2, [(s1 - s0) / 2, 69.0, 30.0], axes)
    p0, p1 = PLATE_S
    py0, py1 = CLEVIS_PLATE_Y
    plate = _obox(J + a * (p0 + p1) / 2 + y * (py0 + py1) / 2,
                  [(p1 - p0) / 2, (py1 - py0) / 2, 30.0], axes)
    return [crossbar, plate]


def _pitch_node_child(J, out_dir):
    """Child-side pivot hub + collar at a pitch joint ``J`` (merged into the
    moving link's mesh): a hub on the axis (its -Y face seats the sprocket)
    and a collar reaching out toward the link's truss."""
    J = np.asarray(J, float)
    c = np.asarray(out_dir, float)
    c = c / np.linalg.norm(c)
    y = np.array([0.0, 1.0, 0.0])
    hub = _disc(J, y, 2 * HUB_R, None, lo=HUB_Y[0], hi=HUB_Y[1])
    collar = _tube(J, J + c * (TIBIA_TRUSS_S0 + 40.0), COLLAR_R)
    return [hub, collar]


def _drive_stack(P, part_prefix):
    """Actuator + brake bodies on the joint axis direction at x-z point ``P``
    (leg frame), offset along -Y per DRIVE_Y/BRAKE_Y.  The mount plate that
    seats them belongs to the PARENT link mesh (built by the caller)."""
    y = np.array([0.0, 1.0, 0.0])
    act = _disc(P, y, X15_DIA, None, lo=DRIVE_Y[0], hi=DRIVE_Y[1])
    brake = _disc(P, y, BRAKE_DIA, None, lo=BRAKE_Y[0], hi=BRAKE_Y[1])
    return [(f"{part_prefix}_actuator", "motor", C_X15, act),
            (f"{part_prefix}_brake", "brake", C_BRAKE, brake)]


def _leg0_instances(k):
    """(name, role, color, mesh) for every leg-0 part, in the leg-0 frame."""
    yaw, hip, knee, foot = k["yaw"], k["hip"], k["knee"], k["foot"]
    y = np.array([0.0, 1.0, 0.0])
    f_hat = knee - hip
    f_hat = f_hat / np.linalg.norm(f_hat)
    t_hat = foot - knee
    t_hat = t_hat / np.linalg.norm(t_hat)

    out = []

    # --- hip-yaw node: X8 hangs under the chassis vertex plate; the coxa hub
    # hangs 5 mm below it, joined only by the coaxial yaw output shaft. ---
    out.append(("hip_yaw_actuator", "motor", C_X8,
                _disc(yaw, [0, 0, 1], X8_DIA, None, lo=YAW_ACT_Z[0], hi=YAW_ACT_Z[1])))
    out.append(("yaw_pivot_pin", "bearing", C_PIN,
                _disc(yaw, [0, 0, 1], 2 * 14.0, None, lo=YAW_PIN_Z[0], hi=YAW_PIN_Z[1])))

    # --- coxa: yaw hub + box beam (reaches inboard to carry the hip drive) +
    # hip clevis (crossbar + bearing plate) + hip-drive mount plate.  The yaw
    # hub radius (40) stays inside the driven sprocket's -Y face plane. ---
    bx, by, bz = COXA_BEAM["x"], COXA_BEAM["y"], COXA_BEAM["z"]
    mount_y = ((DRIVE_Y[1] - SEAT) + (-30.0)) / 2         # plate spans y in
    mount_hy = ((-30.0) - (DRIVE_Y[1] - SEAT)) / 2        # [DRIVE_Y[1]-SEAT, -30]
    coxa_parts = [
        _disc(yaw, [0, 0, 1], 2 * 40.0, None, lo=COXA_HUB_Z[0], hi=COXA_HUB_Z[1]),
        _obox([(bx[0] + bx[1]) / 2, 0.0, (bz[0] + bz[1]) / 2],
              [(bx[1] - bx[0]) / 2, by, (bz[1] - bz[0]) / 2]),
        _obox([HIP_DRIVE_X, mount_y, (bz[0] + bz[1]) / 2],
              [45.0, mount_hy, (bz[1] - bz[0]) / 2]),
    ]
    coxa_parts += _pitch_node_parent(hip, np.array([-1.0, 0.0, 0.0]))
    out.append(("coxa_frame", "frame", C_STEEL, _union(coxa_parts)))

    # --- hip drive: X15 + fail-safe brake on the coxa mount plate, inboard,
    # chain run (not modelled) to the driven sprocket on the hip axis. ---
    hip_drive_xz = np.array([HIP_DRIVE_X, 0.0, hip[2]])
    out += _drive_stack(hip_drive_xz, "hip_pitch")
    # rename hip_pitch_brake -> hip_brake to match the docs/spec
    name, role, color, mesh = out[-1]
    out[-1] = ("hip_brake", role, color, mesh)

    # --- hip joint: pivot pin + driven sprocket (bolted to the femur hub). ---
    out.append(("hip_pivot_pin", "bearing", C_PIN,
                _disc(hip, y, 2 * PIN_R, None, lo=PIN_Y[0], hi=PIN_Y[1])))
    out.append(("hip_secondary", "drivetrain", C_SEC,
                _disc(hip, y, SEC_DIA, None, lo=SPROCKET_Y[0], hi=SPROCKET_Y[1])))

    # --- femur: pivot hub + collar + truss + knee clevis + knee-drive mount ---
    femur_parts = [
        _disc(hip, y, 2 * HUB_R, None, lo=HUB_Y[0], hi=HUB_Y[1]),
        _tube(hip, hip + f_hat * (FEMUR_TRUSS_S0 + 40.0), COLLAR_R),
    ]
    femur_parts += _truss_parts(hip + f_hat * FEMUR_TRUSS_S0,
                                knee - f_hat * FEMUR_TRUSS_S1,
                                FRAME_DEPTH, CHORD_OD / 2, WEB_OD / 2)
    femur_parts += _pitch_node_parent(knee, -f_hat)
    # knee-drive mount plate: reaches from the truss chords down to the
    # actuator seat plane (flat -Y face at DRIVE_Y[1] - SEAT).
    knee_drive_xz = knee - f_hat * DRIVE_STANDOFF
    w_f = np.cross(f_hat, y)
    axes_f = np.column_stack([f_hat, y, w_f])
    femur_parts.append(_obox(
        knee_drive_xz + y * ((DRIVE_Y[1] - SEAT) + (-50.0)) / 2 + w_f * (-28.0),
        [45.0, ((-50.0) - (DRIVE_Y[1] - SEAT)) / 2, 28.0], axes_f))
    out.append(("femur_frame", "frame", C_STEEL, _union(femur_parts)))

    # --- knee drive: X15 + MANDATORY fail-safe brake, mid-femur. ---
    out += _drive_stack(knee_drive_xz, "knee")

    # --- knee joint: pivot pin + driven sprocket (bolted to the tibia hub) ---
    out.append(("knee_pivot_pin", "bearing", C_PIN,
                _disc(knee, y, 2 * PIN_R, None, lo=PIN_Y[0], hi=PIN_Y[1])))
    out.append(("knee_secondary", "drivetrain", C_SEC,
                _disc(knee, y, SEC_DIA, None, lo=SPROCKET_Y[0], hi=SPROCKET_Y[1])))

    # --- tibia: pivot hub + collar + truss + an AXIAL ankle strut down to the
    # foot (along the tibia axis so it crosses the truss end rings at a clean
    # angle — a vertical post grazes the near-parallel chords and leaves
    # float32 self-intersection slivers). ---
    tibia_parts = _pitch_node_child(knee, t_hat)
    tibia_parts += _truss_parts(knee + t_hat * TIBIA_TRUSS_S0,
                                foot - t_hat * TIBIA_TRUSS_S1,
                                0.85 * FRAME_DEPTH, CHORD_OD / 2, WEB_OD / 2)
    tibia_parts.append(_tube(foot - t_hat * (TIBIA_TRUSS_S1 + 60.0),
                             foot - t_hat * 40.0, 30.0))
    # short vertical ankle stub: flat-on-flat 0.1 mm seat onto the foot disc
    tibia_parts.append(_tube([foot[0], foot[1], foot[2] + 65.0],
                             [foot[0], foot[1], foot[2] - SEAT], 30.0))
    out.append(("tibia_frame", "frame", C_STEEL_HI, _union(tibia_parts)))

    # --- foot pad (top at the ankle/foot point) ---
    out.append(("foot", "frame", C_FOOT,
                _disc(foot, [0, 0, 1], FOOT_DIA, None, lo=-FOOT_T, hi=0.0)))
    return out


def _body_instances():
    """(name, role, color, mesh) for the chassis + rider interface, baseline
    frame (hip-yaw actuator mount plane at z=0)."""
    deck_bot = RING_Z[1] + RING_TUBE_R - SEAT
    deck_top = deck_bot + DECK["t"]

    out = []
    out.append(("chassis_frame", "chassis", C_CHASSIS, _hex_chassis()))
    out.append(("deck_plate", "chassis", C_DECK,
                _obox([0, 0, (deck_bot + deck_top) / 2],
                      [DECK["x"] / 2, DECK["y"] / 2, DECK["t"] / 2])))
    # battery tray: low + central, seated on the lower radial spokes (footprint
    # kept inside the chassis face diagonals)
    batt_bot = RING_Z[0] + RING_TUBE_R - SEAT
    out.append(("battery", "electronics", C_BATT,
                _obox([240, 0, batt_bot + 50], [150, 130, 50])))
    # electronics bay on the deck, rear
    out.append(("e_bay", "electronics", C_EBAY,
                _obox([-230, 0, deck_top - SEAT + 60], [120, 100, 60])))
    # saddle on a suspension post (slightly rear)
    post_top = deck_top + 190.0
    out.append(("saddle_post", "rider", C_POST,
                _tube([-40, 0, deck_top - SEAT], [-40, 0, post_top], 26)))
    out.append(("saddle", "rider", C_SADDLE,
                _obox([-40, 0, post_top - SEAT + 45], [180, 150, 45])))
    # handlebar (post + crossbar) at the front — one welded body
    bar_z = deck_top + 250.0
    out.append(("controls", "rider", C_CTRL, _union([
        _tube([300, 0, deck_top - SEAT], [300, 0, bar_z], 20),
        _tube([300, -190, bar_z], [300, 190, bar_z], 18),
    ])))
    # footrests either side, on the deck
    for sy in (+1, -1):
        out.append(("footpeg", "rider", C_PEG,
                    _obox([-40, sy * 300, deck_top - SEAT + 15], [75, 45, 15])))
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
# Distal (moving) link membership per declared joint, by leg-0 part name.
# The driven sprocket moves with the joint it drives; the actuator + brake
# ride on the PARENT link (so e.g. the knee drive belongs to the HIP set).
YAW_DISTAL = ["coxa_frame", "yaw_pivot_pin", "hip_pitch_actuator", "hip_brake"]
HIP_DISTAL = ["femur_frame", "hip_secondary", "hip_pivot_pin",
              "knee_actuator", "knee_brake"]
KNEE_DISTAL = ["tibia_frame", "knee_secondary", "knee_pivot_pin", "foot"]

FASTENER_PARTS = {"yaw_pivot_pin", "hip_pivot_pin", "knee_pivot_pin"}


def main() -> None:
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    k = _solve_leg()
    leg0 = _leg0_instances(k)
    body = _body_instances()

    # Build every world-baked mesh (baseline frame), collect for lift.
    baked: list[tuple[str, str, str, trimesh.Trimesh, int | None]] = []
    for name, role, color, mesh in body:
        baked.append((name, role, color, mesh, None))
    for i in range(N_LEGS):
        R = _rot_z(i * 2 * np.pi / N_LEGS)
        for name, role, color, mesh in leg0:
            m = mesh.copy()
            m.apply_transform(R)
            baked.append((name, role, color, m, i))

    # Lift so the lowest point sits on z = 0 (ground).
    z_min = min(float(m.bounds[0][2]) for _n, _r, _c, m, _l in baked)
    Tlift = _trans([0.0, 0.0, -z_min])

    # Pivot-pin shaft axes in the BODY frame (before lift), per leg: the yaw
    # pin is vertical, the pitch/knee pins lie along the leg's pitch axis.
    # Fastener meshes are exported in a LOCAL frame with the shaft along +Z and
    # placed by a real instance transform, so any consumer that reasons about a
    # fastener's axis (e.g. the joint-check coaxial-pivot exemption) sees the
    # true shaft direction rather than a world-AABB guess.
    pin_axis_leg0 = {
        "yaw_pivot_pin": np.array([0.0, 0.0, 1.0]),
        "hip_pivot_pin": np.array([0.0, 1.0, 0.0]),
        "knee_pivot_pin": np.array([0.0, 1.0, 0.0]),
    }

    def _axis_to_transform(axis, origin) -> np.ndarray:
        """Rigid transform mapping local +Z to ``axis`` at ``origin``."""
        u = np.asarray(axis, float)
        u = u / np.linalg.norm(u)
        ref = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        v = np.cross(u, ref)
        v /= np.linalg.norm(v)
        w = np.cross(u, v)
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = v, w, u
        T[:3, 3] = np.asarray(origin, float)
        return T

    instances_json: list[dict] = []
    meshes_json: list[dict] = []
    all_bounds: list[np.ndarray] = []
    identity = [float(v) for v in np.eye(4).flatten("F")]
    ids_by_leg: dict[int, dict[str, str]] = {i: {} for i in range(N_LEGS)}

    for idx, (name, role, color, mesh, leg) in enumerate(baked):
        m = mesh.copy()
        m.apply_transform(Tlift)
        all_bounds.append(m.bounds)
        fname = f"{idx:03d}_{name}.stl"
        if name in FASTENER_PARTS:
            # Export the pin in its local shaft frame; ship a real transform.
            R = _rot_z(leg * 2 * np.pi / N_LEGS)[:3, :3]
            axis = R @ pin_axis_leg0[name]
            T = _axis_to_transform(axis, m.centroid)
            local = m.copy()
            local.apply_transform(np.linalg.inv(T))
            local.export(STL_DIR / fname)
            transform = [float(v) for v in T.flatten("F")]
        else:
            m.export(STL_DIR / fname)
            transform = identity
        # Pivot pins are fasteners: namespace their mesh id so BuildViz treats
        # them as hardware (exempt from interference, used as joint axles).
        ns = "fasteners" if name in FASTENER_PARTS else "stl"
        mesh_id = f"{ns}:{idx:03d}_{name}"
        inst_id = f"{idx:03d}-{name}"
        meshes_json.append({"id": mesh_id, "name": fname, "url": f"stl/{fname}"})
        instances_json.append({
            "id": inst_id,
            "meshId": mesh_id,
            "name": name.replace("_", " ") + ("" if leg is None else f" L{leg}"),
            "partType": name,
            "role": role,
            "cots": role in COTS_ROLES,
            "color": color,
            "transform": transform,
            "centroid": [float(v) for v in m.centroid],
            "leg": None if leg is None else f"L{leg}",
        })
        if leg is not None:
            ids_by_leg[leg][name] = inst_id

    # --- joints[] : the additive kinematics block (see buildScene.ts). One
    # revolute joint per articulating DOF, world axis + origin at the HOME
    # (static) pose, distal instances only (ancestors compose via parent). ---
    lift = np.array([0.0, 0.0, -z_min])
    joints_json: list[dict] = []
    for i in range(N_LEGS):
        th = i * 2 * np.pi / N_LEGS
        R = _rot_z(th)[:3, :3]
        yaw_o = R @ np.array([HIP_YAW_RING_R, 0.0, (COXA_HUB_Z[0] + YAW_ACT_Z[1]) / 2]) + lift
        hip_o = R @ k["hip"] + lift
        knee_o = R @ k["knee"] + lift
        pitch_axis = R @ np.array([0.0, 1.0, 0.0])
        ids = ids_by_leg[i]
        joints_json.append({
            "id": f"L{i}-yaw", "type": "revolute",
            "axis": [0.0, 0.0, 1.0],
            "origin": [float(v) for v in yaw_o],
            "instances": [ids[n] for n in YAW_DISTAL],
            "limits": {"min": YAW_LIMITS[0], "max": YAW_LIMITS[1]},
            "home": 0, "label": f"L{i} hip yaw",
        })
        joints_json.append({
            "id": f"L{i}-hip", "type": "revolute",
            "axis": [float(v) for v in pitch_axis],
            "origin": [float(v) for v in hip_o],
            "parent": f"L{i}-yaw",
            "instances": [ids[n] for n in HIP_DISTAL],
            "limits": {"min": HIP_LIMITS[0], "max": HIP_LIMITS[1]},
            "home": 0, "label": f"L{i} hip pitch",
        })
        joints_json.append({
            "id": f"L{i}-knee", "type": "revolute",
            "axis": [float(v) for v in pitch_axis],
            "origin": [float(v) for v in knee_o],
            "parent": f"L{i}-hip",
            "instances": [ids[n] for n in KNEE_DISTAL],
            "limits": {"min": KNEE_LIMITS[0], "max": KNEE_LIMITS[1]},
            "home": 0, "label": f"L{i} knee",
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
        "joints": joints_json,
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
    print(f"  {len(instances_json)} instances, {len(meshes_json)} meshes, "
          f"{len(joints_json)} joints")
    print(f"  leg IK: femur {k['femur_deg']:.1f}deg above horiz, "
          f"knee interior {k['knee_int_deg']:.1f}deg, hip->foot {k['d']:.0f} mm")
    print(f"  footprint bbox (mm): {dims[0]:.0f} x {dims[1]:.0f} wide, "
          f"{dims[2]:.0f} tall; ground-lift {-z_min:.0f} mm")


if __name__ == "__main__":
    main()
