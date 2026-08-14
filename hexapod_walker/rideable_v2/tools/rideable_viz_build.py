#!/usr/bin/env python3
"""Build a BuildViz scene for the RIDEABLE hexapod, iteration 2 (`rideable_v2`).

Like `rideable_v1`, this design is a mechanical-design DRAFT captured in
markdown (README.md / DRIVETRAIN.md / STRUCTURE.md / POWER_SYSTEM.md / BOM.md /
PARTS.md), so this tool GENERATES primitive stand-in geometry — 6061 box-beam
femurs / tube tibias (v2 is aluminum, not v1's steel truss), cylindrical
AK80-64s, the three per-joint belt-stage driven pulleys, the joint-side
parking-pin locks, and a welded rider chassis — placed at rideable scale
directly from the numbers in ``design_spec.yaml``.

What is DIFFERENT from the v1 scene (the v2 design moves):

  * the KNEE actuator sits at the HIP end of the femur (~140 mm from the hip
    axis), with its 3:1 belt running ~212 mm down the femur — on the OPPOSITE
    femur face from the hip's Ø255 driven pulley so the two stages package;
  * hip-yaw is belt-driven too (2:1): the yaw AK80-64 hangs off-axis under
    the chassis and a Ø183 driven pulley sits on the yaw shaft;
  * the load-holding device is a JOINT-SIDE spring-applied parking pin
    (yellow) seated against each hip/knee driven pulley web — drawn ENGAGED,
    which is truthful for the parked home pose;
  * belt runs themselves are NOT modelled (same convention as v1's chain):
    a belt spans a declared joint, so solid belt geometry would fail the
    articulation checks it is physically exempt from.

The joint layout is physically articulable (verified by the BuildViz
joint-articulation checks): every revolute DOF is declared in an additive
``joints[]`` block (6 yaw + 6 hip + 6 knee), the driven pulley of each stage
sits ON the joint axis bolted to the MOVING link, the actuator rides the
PARENT link, and parent/child meet only through a coaxial pivot pin
(``fasteners:``-namespaced, exempt as the joint's own axle).  Intended mounts
are flat-on-flat with a nominal 0.1 mm seat for the ``mating_contact`` gate.

Writes a self-contained BuildViz build under ``rideable_v2/full_robot_viz/``:

    full_robot_viz/scene.json      BuildViz manifest (meshes[] + instances[] + joints[])
    full_robot_viz/stl/*.stl       one world-baked STL per instance

Run:
    ./run.sh hexapod_walker/rideable_v2/tools/rideable_viz_build.py

Then VIEW it in the ONE machine-wide BuildViz hub (port 5183) — never start a
per-project dev server:
    make -C hexapod_walker/rideable_v2 view-buildviz
    # http://127.0.0.1:5183/?build=rideable_v2
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
# Design parameters (mm / deg) — mirror design_spec.yaml.
# ---------------------------------------------------------------------------
N_LEGS = 6
HIP_YAW_RING_R = 440.0         # body ~880 mm across the yaw axes
COXA_LEN = 120.0               # hip-yaw -> hip-pitch
FEMUR_LEN = 350.0              # hip-pitch -> knee
TIBIA_LEN = 450.0              # knee -> foot
FOOT_OUT = 300.0               # tucked foot, outboard of the hip-pitch joint
FOOT_DOWN = 550.0              # tucked foot, below the hip-pitch joint
HIP_DROP = 165.0               # hip-pitch axis below the yaw mount plane —
                               # drops the Ø255 hip pulley's top rim clear of
                               # the yaw-stage pulley plane (v1 did the same
                               # for its Ø300 sprocket)

# COTS actuator envelope (AK80-64 ~ Ø98 x 60).
ACT_DIA, ACT_T = 98.0, 60.0

# Belt-stage driven pulleys (PD from design_spec.yaml; drawn as solid discs).
PULLEY_T = 25.0
HIP_PULLEY_DIA = 254.6         # 100T
KNEE_PULLEY_DIA = 213.9        # 84T
YAW_PULLEY_DIA = 183.3         # 72T

# Drive placement (center distances from PARTS.md §2).
YAW_DRIVE_C = 136.0            # yaw AK80-64, inboard of the yaw axis
HIP_DRIVE_C = 208.0            # hip AK80-64 on the coxa, inboard of the hip
KNEE_DRIVE_C = 212.0           # knee AK80-64 on the femur, back from the knee

# Leg member sections (STRUCTURE.md).
FEMUR_H, FEMUR_W = 60.0, 40.0  # box, long axis in the load plane
TIBIA_R = 25.0                 # Ø50 tube
FOOT_DIA, FOOT_T = 120.0, 40.0

# Joint-node geometry (leg frame; pitch/knee axes are local +Y).
# ``side`` = +1/-1: which Y side the driven pulley (and lock) live on.
# Hip: pulley on -Y / clevis plate on +Y.  Knee: mirrored (pulley +Y) so the
# knee drive clears the big hip pulley disc (the v2 packaging move).
HUB_R = 45.0
HUB_NEAR, HUB_FAR = 45.0, 40.0     # hub extent: NEAR = pulley side, FAR = plate side
PIN_R = 15.0
PIN_NEAR, PIN_FAR = 40.0, 75.0     # pin extent (grips hub + clevis plate)
PULLEY_NEAR = 45.0                 # pulley inner face |y| (0.1 mm seat onto hub)
CLEVIS_Y0, CLEVIS_Y1 = 45.0, 70.0  # parent-side bearing plate (plate side)
CROSSBAR_S = (135.0, 175.0)        # hip parent crossbar span back from the joint
KNEE_CROSSBAR_S = (115.0, 155.0)   # knee crossbar (clears the Ø214 pulley)
PLATE_HIP_S = (-35.0, 175.0)
PLATE_KNEE_S = (-35.0, 155.0)
DRIVE_NEAR, DRIVE_FAR = 100.0, 160.0   # actuator body |y| extent on its side
LOCK_LEN = 27.0                # parking-pin lock body length along the axis
LOCK_R = 90.0                  # engage radius on the pulley web (spec: r=90,
                               # inside the knee 84T tooth root at r~102)
SEAT = 0.1                     # nominal interference at intended flat seats

FEMUR_S0, FEMUR_S1 = 60.0, 90.0    # femur beam: from hip / short of knee
TIBIA_S0, TIBIA_S1 = 120.0, 70.0   # tibia tube: from knee / short of the foot
COLLAR_R = 28.0                    # < FEMUR_H/2: an exactly-tangent collar
                                   # leaves self-intersecting slivers

# Chassis / rider interface (leg frame z: 0 = yaw mount plane).
VERTEX_PLATE = dict(x=(250.0, 500.0), hy=60.0, z=(60.0, 95.0))
YAW_ACT_Z = (0.0, 60.0 + SEAT)     # yaw AK80-64, output DOWN, under the plate
COXA_HUB_Z = (-60.0, -31.0)        # coxa yaw hub (pulley stacks on its top)
YAW_PULLEY_Z = (COXA_HUB_Z[1] - SEAT,
                COXA_HUB_Z[1] - SEAT + PULLEY_T)  # seated onto the hub top
YAW_PIN_Z = (-40.0, 75.0)          # yaw shaft: grips hub, pulley AND plate
# Coxa gooseneck beam: x stops at 510 so the femur's r45 hip hub (reaching
# back to x=515) never touches it; z reaches from the yaw hub down to the
# dropped hip clevis.
COXA_BEAM = dict(x=(330.0, 510.0), y=40.0, z=(-195.0, -55.0))
RING_Z = (110.0, 260.0)
RING_TUBE_R = 20.0
DECK = dict(x=800.0, y=600.0, t=25.0)

# Colours.
C_ACT = "#2e2e33"          # AK80-64 (dark)
C_PULLEY = "#95a5a6"       # driven pulleys (grey)
C_LOCK = "#f1c40f"         # parking-pin locks — bright yellow (safety call-out)
C_PIN = "#b8860b"          # pivot pins / joint shafts
C_ALU = "#aab4be"          # 6061 legs (lighter than v1's steel)
C_ALU_HI = "#98a4b0"       # tibia
C_FOOT = "#2b2b2b"
C_CHASSIS = "#6b7c8c"
C_DECK = "#c8ccd0"
C_SADDLE = "#191919"
C_POST = "#4a4a4a"
C_CTRL = "#3a3a3a"
C_PEG = "#555555"
C_BATT = "#2e7d32"
C_EBAY = "#6a4fb0"

COTS_ROLES = {"motor", "brake", "bearing", "electronics", "drivetrain", "rider"}

# Intended MOUNTING interfaces (flat seats, nominally 0.1 mm interference).
# The lock/pulley pairs are the parking pins drawn ENGAGED against the pulley
# web (truthful in the parked home pose); rotation about the pulley axis is
# geometry-invariant, so the articulation sweeps stay clean.
INTENDED_OVERLAP_PAIRS = [
    ("chassis_frame", "yaw_actuator"),      # AK80-64 flange under the vertex plate
    ("coxa_frame", "yaw_pulley"),           # yaw driven pulley on the coxa hub
    ("coxa_frame", "hip_actuator"),         # hip AK80-64 on the coxa mount plate
    ("femur_frame", "hip_pulley"),          # hip driven pulley on the femur hub
    ("hip_pulley", "hip_lock"),             # parking pin engaged in the web
    ("femur_frame", "knee_actuator"),       # knee AK80-64 on the femur mount plate
    ("tibia_frame", "knee_pulley"),         # knee driven pulley on the tibia hub
    ("knee_pulley", "knee_lock"),           # parking pin engaged in the web
    ("tibia_frame", "foot"),
    ("chassis_frame", "deck_plate"),
    ("chassis_frame", "battery"),
    ("deck_plate", "e_bay"),
    ("deck_plate", "saddle_post"),
    ("deck_plate", "controls"),
    ("deck_plate", "footpeg"),
    ("saddle_post", "saddle"),
    # pivot pins (fasteners) — repo-side checker only:
    ("chassis_frame", "yaw_pivot_pin"),
    ("coxa_frame", "yaw_pivot_pin"),
    ("yaw_pulley", "yaw_pivot_pin"),
    ("coxa_frame", "hip_pivot_pin"),
    ("femur_frame", "hip_pivot_pin"),
    ("femur_frame", "knee_pivot_pin"),
    ("tibia_frame", "knee_pivot_pin"),
]

# Declared joint travel (deg about home) — design_spec.yaml joint_limits_deg.
YAW_LIMITS = (-35.0, 35.0)
HIP_LIMITS = (-50.0, 35.0)
KNEE_LIMITS = (-45.0, 45.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tube(p0, p1, radius, sections=32) -> trimesh.Trimesh:
    return _cyl(radius=radius, segment=np.array([p0, p1], dtype=float),
                sections=sections)


def _disc(center, axis, dia, lo, hi, sections=48) -> trimesh.Trimesh:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    c = np.asarray(center, float)
    return _tube(c + axis * lo, c + axis * hi, dia / 2.0, sections)


def _obox(center, half_extents, axes=None) -> trimesh.Trimesh:
    T = np.eye(4)
    if axes is not None:
        T[:3, :3] = np.asarray(axes, float)
    T[:3, 3] = np.asarray(center, float)
    return _box(extents=2.0 * np.asarray(half_extents, float), transform=T)


def _union(parts: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    """Boolean-union primitives into ONE watertight body via manifold3d
    (guaranteed 2-manifold; survives the float32 STL round-trip)."""
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
        # NO simplify(): on float32-quantized coplanar seams it emits sliver
        # self-intersections that fail buildviz's STL checks. The un-
        # simplified booleans are small; keep them exact.
        out = merged.to_mesh()
        return trimesh.Trimesh(
            vertices=np.asarray(out.vert_properties)[:, :3],
            faces=np.asarray(out.tri_verts),
        )
    except BaseException:  # noqa: BLE001 - engine availability differs per env
        return trimesh.util.concatenate(parts)


def _hex_chassis() -> trimesh.Trimesh:
    """Welded hex tube frame + per-vertex mounting plates.  The plate at each
    vertex extends INBOARD to carry the off-axis yaw actuator (v2's 2:1 yaw
    belt stage — v1's yaw was direct-mounted on the axis)."""
    z_bot, z_top = RING_Z
    r_vertex, tube_r = HIP_YAW_RING_R, RING_TUBE_R
    verts = []
    for i in range(N_LEGS):
        a = i * 2 * np.pi / N_LEGS
        verts.append(np.array([r_vertex * np.cos(a), r_vertex * np.sin(a), 0.0]))
    parts: list[trimesh.Trimesh] = []
    hub_bot = np.array([0.0, 0.0, z_bot])
    hub_top = np.array([0.0, 0.0, z_top])
    px0, px1 = VERTEX_PLATE["x"]
    pz0, pz1 = VERTEX_PLATE["z"]
    for i in range(N_LEGS):
        a = i * 2 * np.pi / N_LEGS
        u = np.array([np.cos(a), np.sin(a), 0.0])
        v = np.array([-np.sin(a), np.cos(a), 0.0])
        vb = verts[i] + [0, 0, z_bot]
        vt = verts[i] + [0, 0, z_top]
        vb_n = verts[(i + 1) % N_LEGS] + [0, 0, z_bot]
        vt_n = verts[(i + 1) % N_LEGS] + [0, 0, z_top]
        parts.append(_tube(vb, vb_n, tube_r))
        parts.append(_tube(vt, vt_n, tube_r))
        parts.append(_tube(vb, vt, tube_r))
        parts.append(_tube(vb, vt_n, tube_r))
        parts.append(_tube(hub_top, vt, tube_r))
        parts.append(_tube(hub_bot, vb, tube_r))
        # Vertex mount plate (yaw actuator seats under it; the yaw shaft
        # passes through it at the vertex) + a drop post tying it to the ring.
        c = u * (px0 + px1) / 2 + np.array([0, 0, (pz0 + pz1) / 2])
        axes = np.column_stack([u, v, np.array([0.0, 0.0, 1.0])])
        parts.append(_obox(c, [(px1 - px0) / 2, VERTEX_PLATE["hy"],
                               (pz1 - pz0) / 2], axes))
        parts.append(_tube(u * ((px0 + px1) / 2) + [0, 0, pz0],
                           u * ((px0 + px1) / 2) + [0, 0, z_bot + tube_r], 18.0))
    parts.append(_tube(hub_bot, hub_top, tube_r))
    return _union(parts)


# ---------------------------------------------------------------------------
# Leg kinematics (2-link IK, knee-up branch) — see design_spec.yaml.
# ---------------------------------------------------------------------------
def _solve_leg():
    yaw = np.array([HIP_YAW_RING_R, 0.0, 0.0])
    hip = yaw + np.array([COXA_LEN, 0.0, -HIP_DROP])
    tx, tz = FOOT_OUT, -FOOT_DOWN
    d = float(np.hypot(tx, tz))
    L1, L2 = FEMUR_LEN, TIBIA_LEN
    if not (abs(L1 - L2) <= d <= L1 + L2):
        raise ValueError(f"foot target unreachable: d={d:.1f}")
    phi = np.arctan2(tz, tx)
    a = np.arccos(np.clip((d * d + L1 * L1 - L2 * L2) / (2 * d * L1), -1, 1))
    femur_ang = phi + a
    knee = hip + L1 * np.array([np.cos(femur_ang), 0.0, np.sin(femur_ang)])
    foot = hip + np.array([tx, 0.0, tz])
    knee_int = np.degrees(np.arccos(np.clip(
        (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2), -1, 1)))
    return {"yaw": yaw, "hip": hip, "knee": knee, "foot": foot,
            "femur_deg": np.degrees(femur_ang), "knee_int_deg": knee_int, "d": d}


def _pitch_node_parent(J, back_dir, side, crossbar_s, plate_s):
    """Parent-side clevis at a pitch joint: crossbar clear of the pulley disc
    + a single bearing plate on the NON-pulley side (``-side``)."""
    J = np.asarray(J, float)
    a = np.asarray(back_dir, float)
    a = a / np.linalg.norm(a)
    y = np.array([0.0, 1.0, 0.0])
    w = np.cross(a, y)
    axes = np.column_stack([a, y, w])
    s0, s1 = crossbar_s
    # z offsets are CHUNKY (>= 3 mm), never sub-mm: exactly-coplanar or
    # nearly-coplanar union faces leave sliver triangles that the float32
    # STL write flips into self-intersections (buildviz fails the mesh).
    crossbar = _obox(J + a * (s0 + s1) / 2, [(s1 - s0) / 2, 69.0, 33.0], axes)
    p0, p1 = plate_s
    p1 += 5.0   # well past the crossbar end face — no coplanar end seam
    py = -side * (CLEVIS_Y0 + CLEVIS_Y1) / 2
    plate = _obox(J + a * (p0 + p1) / 2 + y * py,
                  [(p1 - p0) / 2, (CLEVIS_Y1 - CLEVIS_Y0) / 2, 27.0], axes)
    return [crossbar, plate]


def _hub(J, side):
    """Child-side pivot hub; its ``side`` face seats the driven pulley."""
    lo = -HUB_FAR if side > 0 else -HUB_NEAR
    hi = HUB_NEAR if side > 0 else HUB_FAR
    return _disc(J, [0, 1, 0], 2 * HUB_R, lo, hi)


def _pitch_pin(J, side):
    lo = -PIN_FAR if side > 0 else -PIN_NEAR
    hi = PIN_NEAR if side > 0 else PIN_FAR
    return _disc(J, [0, 1, 0], 2 * PIN_R, lo, hi)


def _pulley(J, dia, side):
    lo = PULLEY_NEAR - SEAT if side > 0 else -(PULLEY_NEAR + PULLEY_T)
    hi = PULLEY_NEAR + PULLEY_T if side > 0 else -(PULLEY_NEAR - SEAT)
    return _disc(J, [0, 1, 0], dia, lo, hi)


def _actuator_body(P, side):
    lo = DRIVE_NEAR if side > 0 else -DRIVE_FAR
    hi = DRIVE_FAR if side > 0 else -DRIVE_NEAR
    return _disc(P, [0, 1, 0], ACT_DIA, lo, hi)


def _lock_body(J, side):
    """Parking-pin lock: a compact block at the engage radius (LOCK_R, above
    the joint), seated 0.1 mm against the pulley's outer web face — drawn
    ENGAGED (truthful in the parked home pose)."""
    c = np.asarray(J, float) + np.array([0.0, 0.0, LOCK_R])
    y = np.array([0.0, 1.0, 0.0])
    face = PULLEY_NEAR + PULLEY_T          # pulley outer face |y|
    if side > 0:
        lo, hi = face - SEAT, face - SEAT + LOCK_LEN
    else:
        lo, hi = -(face - SEAT + LOCK_LEN), -(face - SEAT)
    mid = c + y * (lo + hi) / 2
    return _obox(mid, [18.0, (hi - lo) / 2, 18.0])


def _mount_plate(P, along, side, beam_z_half=30.0):
    """Actuator mount plate: from INSIDE the parent member (y0=15 overlaps
    both the coxa beam ±40 and the femur box ±20) out to the actuator seat
    plane (flat face at |y| = DRIVE_NEAR - SEAT on the drive side)."""
    y = np.array([0.0, 1.0, 0.0])
    a = np.asarray(along, float)
    a = a / np.linalg.norm(a)
    w = np.cross(a, y)
    axes = np.column_stack([a, y, w])
    y0 = 15.0
    y1 = DRIVE_NEAR - SEAT
    py = side * (y0 + y1) / 2
    # 3 mm z inset: strictly inside the parent beam's faces, chunky enough
    # that no sliver triangles appear at the seam (see _pitch_node_parent).
    return _obox(np.asarray(P, float) + y * py,
                 [35.0, (y1 - y0) / 2, beam_z_half - 3.0], axes)


def _leg0_instances(k):
    """(name, role, color, mesh) for every leg-0 part, in the leg-0 frame."""
    yaw, hip, knee, foot = k["yaw"], k["hip"], k["knee"], k["foot"]
    f_hat = knee - hip
    f_hat = f_hat / np.linalg.norm(f_hat)
    t_hat = foot - knee
    t_hat = t_hat / np.linalg.norm(t_hat)

    out = []

    # --- yaw drive: AK80-64 hangs under the chassis vertex plate, INBOARD of
    # the yaw axis by the 2:1 stage's center distance (belt not modelled). ---
    yaw_drive = np.array([HIP_YAW_RING_R - YAW_DRIVE_C, 0.0, 0.0])
    out.append(("yaw_actuator", "motor", C_ACT,
                _disc(yaw_drive, [0, 0, 1], ACT_DIA, YAW_ACT_Z[0], YAW_ACT_Z[1])))
    # yaw joint shaft: grips the coxa hub AND the chassis vertex plate
    out.append(("yaw_pivot_pin", "bearing", C_PIN,
                _disc(yaw, [0, 0, 1], 2 * 14.0, YAW_PIN_Z[0], YAW_PIN_Z[1])))
    # yaw driven pulley (72T): ON the axis, seated onto the coxa hub top
    out.append(("yaw_pulley", "drivetrain", C_PULLEY,
                _disc(yaw, [0, 0, 1], YAW_PULLEY_DIA,
                      YAW_PULLEY_Z[0], YAW_PULLEY_Z[1])))

    # --- coxa: yaw hub + box beam + hip clevis + hip-drive mount plate. ---
    bx, by, bz = COXA_BEAM["x"], COXA_BEAM["y"], COXA_BEAM["z"]
    hip_drive = np.array([hip[0] - HIP_DRIVE_C, 0.0, hip[2]])
    coxa_parts = [
        # r46, NOT r40: the beam is +-40 wide, and an exactly-tangent hub
        # (48-gon vertex line ON the beam face) makes the boolean emit sub-
        # float32 slivers that collapse into self-intersections in the STL.
        _disc(yaw, [0, 0, 1], 2 * 46.0, COXA_HUB_Z[0], COXA_HUB_Z[1]),
        _obox([(bx[0] + bx[1]) / 2, 0.0, (bz[0] + bz[1]) / 2],
              [(bx[1] - bx[0]) / 2, by, (bz[1] - bz[0]) / 2]),
        _mount_plate(hip_drive, [1.0, 0.0, 0.0], -1),
    ]
    coxa_parts += _pitch_node_parent(hip, [-1.0, 0.0, 0.0], -1,
                                     CROSSBAR_S, PLATE_HIP_S)
    # Frame parts stay as PRIMITIVE LISTS: main() rotates the primitives into
    # each leg's final frame and unions THERE, so every STL is exact manifold
    # output (rotating a unioned mesh afterwards lets float32 quantization
    # flip sliver triangles at box junctions into self-intersections).
    out.append(("coxa_frame", "frame", C_ALU, coxa_parts))

    # --- hip drive: AK80-64 on the coxa (pulley side, -Y), 4:1 belt (not
    # modelled) to the 100T pulley on the hip axis. ---
    out.append(("hip_actuator", "motor", C_ACT, _actuator_body(hip_drive, -1)))

    # --- hip joint: pin + 100T driven pulley (bolted to the femur hub) +
    # the coxa-mounted parking-pin lock, drawn engaged in the pulley web. ---
    out.append(("hip_pivot_pin", "bearing", C_PIN, _pitch_pin(hip, -1)))
    out.append(("hip_pulley", "drivetrain", C_PULLEY,
                _pulley(hip, HIP_PULLEY_DIA, -1)))
    out.append(("hip_lock", "brake", C_LOCK, _lock_body(hip, -1)))

    # --- femur: hip hub + collar + 60x40 box beam + knee clevis (mirrored:
    # plate on -Y) + the knee-drive mount ~140 mm from the hip on +Y. ---
    knee_drive = hip + f_hat * (FEMUR_LEN - KNEE_DRIVE_C)
    y = np.array([0.0, 1.0, 0.0])
    w_f = np.cross(f_hat, y)
    axes_f = np.column_stack([f_hat, y, w_f])
    beam_c = (hip + f_hat * FEMUR_S0 + knee - f_hat * FEMUR_S1) / 2
    beam_len = FEMUR_LEN - FEMUR_S0 - FEMUR_S1
    femur_parts = [
        _hub(hip, -1),
        _tube(hip, hip + f_hat * (FEMUR_S0 + 30.0), COLLAR_R),
        _obox(beam_c, [beam_len / 2, FEMUR_W / 2, FEMUR_H / 2], axes_f),
        _mount_plate(knee_drive, f_hat, +1, beam_z_half=FEMUR_H / 2),
    ]
    femur_parts += _pitch_node_parent(knee, -f_hat, +1,
                                      KNEE_CROSSBAR_S, PLATE_KNEE_S)
    out.append(("femur_frame", "frame", C_ALU, femur_parts))

    # --- knee drive: AK80-64 at the hip end of the femur, +Y side (clears
    # the hip pulley disc); 3:1 belt runs down the femur (not modelled). ---
    out.append(("knee_actuator", "motor", C_ACT, _actuator_body(knee_drive, +1)))

    # --- knee joint: pin + 84T pulley (bolted to the tibia hub, +Y side) +
    # the femur-mounted parking-pin lock. ---
    out.append(("knee_pivot_pin", "bearing", C_PIN, _pitch_pin(knee, +1)))
    out.append(("knee_pulley", "drivetrain", C_PULLEY,
                _pulley(knee, KNEE_PULLEY_DIA, +1)))
    out.append(("knee_lock", "brake", C_LOCK, _lock_body(knee, +1)))

    # --- tibia: knee hub + collar + Ø50 tube + ankle stub to the foot. ---
    tibia_parts = [
        _hub(knee, +1),
        _tube(knee, knee + t_hat * (TIBIA_S0 + 30.0), COLLAR_R),
        _tube(knee + t_hat * TIBIA_S0, foot - t_hat * TIBIA_S1, TIBIA_R),
        _tube(foot - t_hat * (TIBIA_S1 + 40.0), foot - t_hat * 35.0, 28.0),
        _tube([foot[0], foot[1], foot[2] + 60.0],
              [foot[0], foot[1], foot[2] - SEAT], 28.0),
    ]
    out.append(("tibia_frame", "frame", C_ALU_HI, tibia_parts))

    # --- foot pad ---
    out.append(("foot", "frame", C_FOOT,
                _disc(foot, [0, 0, 1], FOOT_DIA, -FOOT_T, 0.0)))
    return out


def _body_instances():
    deck_bot = RING_Z[1] + RING_TUBE_R - SEAT
    deck_top = deck_bot + DECK["t"]

    out = []
    out.append(("chassis_frame", "chassis", C_CHASSIS, _hex_chassis()))
    out.append(("deck_plate", "chassis", C_DECK,
                _obox([0, 0, (deck_bot + deck_top) / 2],
                      [DECK["x"] / 2, DECK["y"] / 2, DECK["t"] / 2])))
    batt_bot = RING_Z[0] + RING_TUBE_R - SEAT
    out.append(("battery", "electronics", C_BATT,
                _obox([210, 0, batt_bot + 50], [140, 120, 50])))
    out.append(("e_bay", "electronics", C_EBAY,
                _obox([-210, 0, deck_top - SEAT + 55], [110, 95, 55])))
    post_top = deck_top + 190.0
    out.append(("saddle_post", "rider", C_POST,
                _tube([-40, 0, deck_top - SEAT], [-40, 0, post_top], 26)))
    out.append(("saddle", "rider", C_SADDLE,
                _obox([-40, 0, post_top - SEAT + 45], [170, 140, 45])))
    bar_z = deck_top + 250.0
    out.append(("controls", "rider", C_CTRL, _union([
        _tube([270, 0, deck_top - SEAT], [270, 0, bar_z], 20),
        _tube([270, -180, bar_z], [270, 180, bar_z], 18),
    ])))
    for sy in (+1, -1):
        out.append(("footpeg", "rider", C_PEG,
                    _obox([-40, sy * 270, deck_top - SEAT + 15], [70, 40, 15])))
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
# Driven pulleys move with the joint they drive; actuators and parking locks
# ride the PARENT link (so the hip's drive+lock belong to the YAW set).
YAW_DISTAL = ["coxa_frame", "yaw_pulley", "yaw_pivot_pin",
              "hip_actuator", "hip_lock"]
HIP_DISTAL = ["femur_frame", "hip_pulley", "hip_pivot_pin",
              "knee_actuator", "knee_lock"]
KNEE_DISTAL = ["tibia_frame", "knee_pulley", "knee_pivot_pin", "foot"]

FASTENER_PARTS = {"yaw_pivot_pin", "hip_pivot_pin", "knee_pivot_pin"}


def main() -> None:
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    k = _solve_leg()
    leg0 = _leg0_instances(k)
    body = _body_instances()

    baked: list[tuple[str, str, str, trimesh.Trimesh, int | None]] = []
    for name, role, color, mesh in body:
        baked.append((name, role, color, mesh, None))
    for i in range(N_LEGS):
        R = _rot_z(i * 2 * np.pi / N_LEGS)
        for name, role, color, mesh in leg0:
            if isinstance(mesh, list):
                # union in the FINAL frame (see coxa_frame comment)
                parts = []
                for p in mesh:
                    q = p.copy()
                    q.apply_transform(R)
                    parts.append(q)
                m = _union(parts)
            else:
                m = mesh.copy()
                m.apply_transform(R)
            baked.append((name, role, color, m, i))

    z_min = min(float(m.bounds[0][2]) for _n, _r, _c, m, _l in baked)
    Tlift = _trans([0.0, 0.0, -z_min])

    pin_axis_leg0 = {
        "yaw_pivot_pin": np.array([0.0, 0.0, 1.0]),
        "hip_pivot_pin": np.array([0.0, 1.0, 0.0]),
        "knee_pivot_pin": np.array([0.0, 1.0, 0.0]),
    }

    def _axis_to_transform(axis, origin) -> np.ndarray:
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

    # --- joints[]: 18 revolute DOFs at the HOME (parked) pose. ---
    lift = np.array([0.0, 0.0, -z_min])
    joints_json: list[dict] = []
    for i in range(N_LEGS):
        th = i * 2 * np.pi / N_LEGS
        R = _rot_z(th)[:3, :3]
        yaw_o = R @ np.array([HIP_YAW_RING_R, 0.0,
                              (COXA_HUB_Z[0] + YAW_PIN_Z[1]) / 2]) + lift
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
        "name": "Rideable hexapod v2 (rideable_v2) — design-draft visualization",
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

    # BuildViz-compatible build dir: copy design_spec.yaml + ASSEMBLY.md +
    # BOM.md next to scene.json (the project docs remain the source).
    project_dir = _HERE.parent
    for doc_name in ("design_spec.yaml", "ASSEMBLY.md", "BOM.md"):
        source = project_dir / doc_name
        if source.exists():
            dest = OUT_DIR / doc_name
            shutil.copy2(source, dest)
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
