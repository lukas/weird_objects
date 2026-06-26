#!/usr/bin/env python3
"""Procedural BuildViz model of a cat-like quadruped ROBOT (rev 10).

Builds a clearly feline, ACTUALLY-BUILDABLE quadruped with ``trimesh`` and
exports it as a BuildViz "build": one STL per part under ``cat_viz/stl/`` plus a
``cat_viz/scene.json`` manifest (identity transforms; every mesh baked into world
coords -- same convention as the hexapod's full_robot_viz build).

Rev-8 theme: **CUTE.**  Revs 6-7 were a full-size adult cat on big STS3215
servos; reviewers found it menacing -- a hunched teardrop body, a small ducked
head, spindly legs with clunky external knee boxes, exposed electronics.  Rev 8
throws out "anatomically full-size" and chases NEOTENY (the "cute"/baby-schema
recipe) on a Petoi-Nybble-class micro platform:

* **Smaller hardware.**  The legs/neck now use ~10 g **micro metal-gear serial
  servos** (~23x12x24 mm, ~0.29 N-m) instead of the 60 g, 45x34 mm STS3215, and
  the brain is a compact **NyBoard/BiBoard-class ESP32 controller** (the 68 mm
  Arduino Uno Q does NOT fit a cute compact body) on a small 2S LiPo.  At a light
  ~Nybble-size cat the micro servo still has big torque margin (see design_spec).
* **Neotenic proportions.**  A BIG round head held UP with large forward eyes
  (with iris + pupil + catch-light), plump cheeks, small pink nose, rounded ears;
  a compact, chubby, rounded body (head:body ~1:2.6); short stubby rounded legs
  whose knee/elbow actuators hide INSIDE the chubby leg (no external boxes);
  rounded paws; a cute upright tail curl; a little collar + bell that also hides
  the neck joint and wiring.  Electronics tuck low inside the belly, out of sight.

Rev-9 theme: **MANUFACTURABLE.**  Rev 8 nailed the cute look but the body was a
single solid shell -- impossible to print, open, or mount hardware in.  Rev 9
re-architects it as a real screw-together printed assembly without touching the
cute silhouette:

* **Split body shell.**  The chubby torso is split on the SAGITTAL plane into a
  ``shell_left`` + ``shell_right`` (seam hidden along the dorsal spine + ventral
  midline, ~0.2 mm print clearance), plus a removable ``belly_hatch`` underneath
  for battery/board service.  The head prints as its own piece that mounts on the
  neck-pan servo.
* **Assembly features.**  Screw bosses along the seam (M2 self-tap), a hatch
  frame the belly panel screws into, and an alignment lip so the halves register.
* **Internal hardware mounts.**  A printed servo-mount bracket anchors each of
  the 5 body/neck servos (the 4 elbow/knee servos are captured by their FUR knee
  housings); ESP32 board standoffs + a battery cradle/retainer hold the
  electronics; wiring routes through the open belly cavity.
* **Legibility.**  Every part now carries a ``role`` marking it MOVING (head,
  leg segments, horns) vs STATIC structure (shells, hatch, mounts, bosses,
  servo bodies, electronics); a ``kinematics`` block in design_spec.yaml plus a
  ``cat_kinematics.png`` render call out the 9 articulated joints.

Rev-10 theme: **BABY KITTEN.**  Rev 8/9 read as a small adult cat; rev 10 pushes
the proportions to peak neoteny so it's unmistakably a *kitten*: an even bigger,
rounder, domed head (head:body ~1:2) with bigger/rounder low-set eyes; a shorter,
rounder pot-belly body on shorter, stubbier legs with oversized paws; softer
rounder ears; and a shorter, stubbier, softly-curled tail.  The rev-10 tail is
also a real BUGFIX: the old 7-point sharp-hook loft self-intersected at the tip
(it looked detached); it's now a Catmull-Rom-smoothed, densely-ringed continuous
curl solidly rooted in the rump (consecutive-ring gap ~0).

Carried over from rev 9: the cute neotenic look; the manufacturable split shell
(L/R halves + belly hatch + separate head), M2 screw bosses + alignment lip,
integral servo/board/battery mounts, and the 9-DOF kinematics tagging; real
servos at every joint with real mounting; no unexpected interpenetration; one
connected assembly with nothing hovering; CoM inside the support polygon.

Coordinate frame (world, mm):  +X forward (nose), +Y the cat's LEFT, +Z up.
Lifted so the lowest paw point sits on z = 0.

Run:
    ./run.sh robot_cat/build_cat.py
    ./run.sh robot_cat/check_overlaps.py
    ./run.sh robot_cat/check_assembly.py
    ./run.sh robot_cat/render_cat.py
"""

from __future__ import annotations

import argparse
import datetime
import json
import shutil
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
OUT_DIR = HERE / "cat_viz"
STL_DIR = OUT_DIR / "stl"

# ----------------------------------------------------------------------------
# VERSIONING -- this build uses BuildViz's OWN project/build/named-version model.
# ----------------------------------------------------------------------------
# The robot cat is a SINGLE BuildViz build (hub id ``robot-cat``) with several
# NAMED VERSIONS.  The live working tip is the build-root ``cat_viz/scene.json``
# (the DEFAULT version, ``main``); each frozen revision is an immutable named
# version on disk at ``cat_viz/versions/<name>/`` and ``cat_viz/meta.json``
# records the default + the version list.  ``build_cat.py`` (no args) rewrites
# the live tip; ``--freeze v<REV>`` snapshots it.  No per-revision hub ids.
REV = 10
VERSION = f"v{REV}"
SCENE_NAME = (
    f"Robot Cat — BABY KITTEN proportions, manufacturable split-shell build (rev {REV})"
)

BUILD_ID = "robot-cat"                 # the single BuildViz build id (one build)
BUILD_NAME = "Robot Cat (cat-like quadruped)"
DEFAULT_VERSION = "main"               # BuildViz default-version convention
META_PATH = OUT_DIR / "meta.json"
VERSIONS_DIR = OUT_DIR / "versions"    # cat_viz/versions/<name>/  (native layout)

# ----------------------------------------------------------------------------
# GLOBAL SCALE
# ----------------------------------------------------------------------------
# Structural geometry is authored in "design mm" and multiplied by SCALE at
# export; the real servos / electronics are placed at the scaled joint
# coordinates but kept at TRUE physical size.  Rev 8: SCALE = 1.0 (design-mm ARE
# real mm) -- a compact ~Nybble-class kitten, NOT a full-size cat.
SCALE = 1.0

# ----------------------------------------------------------------------------
# Real hardware (TRUE size) -- a Petoi-Nybble-class micro platform.
# ----------------------------------------------------------------------------
# Micro metal-gear serial-bus servo (Feetech P1S / SCS-class, ~MG90 footprint):
#   body ~23 x 12 x 24 mm, ~10 g, ~0.29 N-m stall @ 8.4 V.  Small enough that the
#   knee/elbow actuator HIDES inside a chubby stubby leg (no external box).
MS_W = 23.0     # body length (servo-local +X)
MS_D = 12.0     # body depth  (servo-local +Y)
MS_H = 24.0     # body height (servo-local +Z), output shaft on the +Z face
MS_OX = 5.5     # output-shaft offset from body centre (+X)
MS_OUTH = 2.0   # output hub height above the top face
HORN_TOP = MS_H / 2.0 + MS_OUTH + 1.6   # local z where the driven bone seats
SERVO_MASS_G = 11.0
SERVO_STALL_NM = 0.29

# Compact ESP32 controller (NyBoard / BiBoard-class).  ~46 x 30 x 9 mm, ~16 g.
# (The 68.58 x 53.34 mm Arduino Uno Q used in revs 6-7 is too big for a cute
#  compact body, so rev 8 drops it for this Nybble-class board.)
CTRL_W, CTRL_D, CTRL_H = 46.0, 30.0, 9.0
CONTROLLER_MASS_G = 16.0

# Battery: a small 2S LiPo (7.4 V ~450 mAh), ~46 x 28 x 12 mm, ~32 g.
BATTERY_L, BATTERY_W, BATTERY_H = 46.0, 28.0, 12.0
BATTERY_MASS_G = 32.0

# Packaging placement (DESIGN mm; SCALE = 1.0 so these are world mm) -- tucked
# low in the rounded belly, fully inside the shell, out of sight.
BATTERY_CENTER = (-8.0, 0.0, 37.0)
CONTROLLER_CENTER = (24.0, 0.0, 62.0)

# ----------------------------------------------------------------------------
# Palette + role per part type.  Cute scheme: soft light "fur", cream muzzle/
# paws, pink nose/ears, big green eyes, a coral collar + gold bell.
# ----------------------------------------------------------------------------
FUR = "#b9bdc7"         # soft light blue-grey fur
CREAM = "#efe7d6"       # muzzle, paws, chin
PINK = "#f0a6b6"        # nose + inner ear + toe accents
EYE_WHITE = "#f6f8fb"
IRIS = "#46b48a"        # big friendly green eyes
PUPIL = "#16181d"
GLEAM = "#ffffff"
COLLAR_C = "#e8556d"
BELL_C = "#f1c247"
INNER = "#6f737b"       # printed interior structure (mounts, bosses, frames)
STEEL = "#aeb2ba"       # fasteners / screws

PALETTE = {
    # split outer shell (all FUR so the assembled cat still reads as one body)
    "shell_left": FUR, "shell_right": FUR, "belly_hatch": FUR,
    "torso": FUR, "neck": FUR, "head": FUR, "cheek": FUR,
    "ear": FUR, "ear_inner": PINK,
    "muzzle": CREAM, "nose": PINK,
    "eye_white": EYE_WHITE, "iris": IRIS, "pupil": PUPIL, "eye_gleam": GLEAM,
    "tail_segment": FUR,
    "upper_arm": FUR, "forearm": FUR, "front_paw": CREAM, "knee": FUR,
    "femur": FUR, "shank": FUR, "hind_paw": CREAM,
    "collar": COLLAR_C, "bell": BELL_C,
    "micro_servo": "#2f3338", "servo_horn": "#ccced3",
    "battery": "#6f2530", "controller": "#1f8a4d",
    "controller_tray": "#41464e", "battery_cradle": "#41464e",
    # rev-9 manufacturability features
    "seam_bosses": INNER, "hatch_frame": INNER, "servo_mount": INNER,
    "board_standoffs": INNER, "battery_strap": INNER, "head_mount": INNER,
    "shell_screws": STEEL, "hatch_screws": STEEL,
}
# role taxonomy.  STATIC structure: shell, body, bracket, boss, fastener, motor,
# controller, battery.  MOVING (articulated): head + features, limb, paw, horn.
ROLE = {
    "shell_left": "shell", "shell_right": "shell", "belly_hatch": "shell",
    "torso": "body", "neck": "shell",
    "head": "head", "cheek": "head", "ear": "head", "ear_inner": "head",
    "muzzle": "head", "nose": "head",
    "eye_white": "head", "iris": "head", "pupil": "head", "eye_gleam": "head",
    "tail_segment": "tail",
    "upper_arm": "limb", "forearm": "limb", "front_paw": "paw", "knee": "limb",
    "femur": "limb", "shank": "limb", "hind_paw": "paw",
    "collar": "trim", "bell": "trim",
    "micro_servo": "motor", "servo_horn": "horn",
    "battery": "battery", "controller": "controller",
    "controller_tray": "bracket", "battery_cradle": "bracket",
    "seam_bosses": "boss", "hatch_frame": "boss", "servo_mount": "bracket",
    "board_standoffs": "fastener", "battery_strap": "bracket",
    "head_mount": "bracket",
    "shell_screws": "fastener", "hatch_screws": "fastener",
}

# Articulated (MOVING) part types -- used by the kinematics render + scene tags.
MOVING_TYPES = {
    "head", "cheek", "ear", "ear_inner", "muzzle", "nose",
    "eye_white", "iris", "pupil", "eye_gleam", "head_mount",
    "upper_arm", "forearm", "front_paw", "knee",
    "femur", "shank", "hind_paw", "servo_horn",
}

# ----------------------------------------------------------------------------
# Geometry primitives.
# ----------------------------------------------------------------------------


def _norm(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def _circle_pts(center, axis, radius, n):
    axis = _norm(axis)
    ref = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, ref)
    u /= np.linalg.norm(u) + 1e-12
    v = np.cross(axis, u)
    ang = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return (np.asarray(center, float)
            + np.outer(np.cos(ang), u) * radius
            + np.outer(np.sin(ang), v) * radius)


def frustum(p0, p1, r0, r1, n=28):
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    pts = np.vstack([_circle_pts(p0, p1 - p0, r0, n),
                     _circle_pts(p1, p1 - p0, r1, n)])
    return trimesh.Trimesh(vertices=pts).convex_hull


def ellipsoid(center, rx, ry, rz, subdiv=3):
    m = trimesh.creation.icosphere(subdivisions=subdiv, radius=1.0)
    m.apply_scale([rx, ry, rz])
    m.apply_translation(np.asarray(center, float))
    return m


def sphere(center, r, subdiv=2):
    m = trimesh.creation.icosphere(subdivisions=subdiv, radius=r)
    m.apply_translation(np.asarray(center, float))
    return m


def box(center, extents):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(np.asarray(center, float))
    return m


def _trim(p0, p1, back0, back1):
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    u = (p1 - p0) / (np.linalg.norm(p1 - p0) + 1e-12)
    return p0 + u * back0, p1 - u * back1


def _pt(xz, y):
    return np.array([xz[0], y, xz[1]], float)


def _ellipse_ring(center, axis, inplane, ru, rv, n):
    """A ring of ``n`` points in the plane perpendicular to ``axis``."""
    axis = _norm(axis)
    u = np.asarray(inplane, float)
    u = u - axis * float(np.dot(u, axis))
    u = _norm(u)
    v = np.cross(axis, u)
    ang = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return (np.asarray(center, float)
            + np.outer(np.cos(ang), u) * ru
            + np.outer(np.sin(ang), v) * rv)


def loft_rings(rings, cap=True):
    """Stitch consecutive rings (each (N,3), consistent ordering) into a smooth
    tube and cap the ends.  Preserves concavity (unlike a convex hull)."""
    n_rings = len(rings)
    N = len(rings[0])
    V = np.vstack([np.asarray(r, float) for r in rings])
    faces = []
    for i in range(n_rings - 1):
        a, b = i * N, (i + 1) * N
        for j in range(N):
            k = (j + 1) % N
            faces.append([a + j, b + j, b + k])
            faces.append([a + j, b + k, a + k])
    if cap:
        c0 = V[:N].mean(0)
        c1 = V[(n_rings - 1) * N:].mean(0)
        i0 = len(V)
        V = np.vstack([V, c0, c1])
        i1 = i0 + 1
        for j in range(N):
            k = (j + 1) % N
            faces.append([i0, k, j])
            faces.append([i1, (n_rings - 1) * N + j, (n_rings - 1) * N + k])
    m = trimesh.Trimesh(vertices=V, faces=np.asarray(faces), process=True)
    m.fix_normals()
    return m


def catmull(points, seg=8):
    """Catmull-Rom spline through ``points`` rows, ``seg`` samples per segment."""
    pts = np.asarray(points, float)
    P = np.vstack([pts[0], pts, pts[-1]])
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        for t in np.linspace(0.0, 1.0, seg, endpoint=False):
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                              + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(P[-2])
    return np.asarray(out)


def _body_ring(x, zc, hy, hz, n, belly=1.0):
    """Rounded body cross-section at station x (chubby, nearly circular)."""
    ang = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    y = hy * np.cos(ang)
    s = np.sin(ang)
    z = zc + np.where(s >= 0.0, hz * s, belly * hz * s)
    return np.column_stack([np.full(n, x), y, z])


def smooth_bone(p0, p1, prof, n=30, flat=0.92):
    """Tapered limb segment: an ellipse tube along p0->p1 whose semi-axis follows
    ``prof`` = list of (t, radius); slightly flattened laterally (``flat``)."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    axis = p1 - p0
    inplane = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(_norm(axis), inplane))) > 0.9:
        inplane = np.array([0.0, 1.0, 0.0])
    rings = [_ellipse_ring(p0 + axis * t, axis, inplane, r, r * flat, n)
             for t, r in prof]
    return loft_rings(rings)


def smooth_paw(center, fwd=1.0):
    """A plump, rounded paw: a chubby ellipsoid, a touch longer fore (toes),
    bottom sitting at center z (so it lands on the ground)."""
    cx, cy, cz = center
    m = trimesh.creation.icosphere(subdivisions=4, radius=1.0)
    v = m.vertices.copy()
    # rev 10: OVERSIZED baby paws (kittens have big mitten paws)
    v[:, 0] *= 16.5
    v[:, 1] *= 15.0
    v[:, 2] *= 13.0
    m = trimesh.Trimesh(vertices=v, faces=m.faces, process=False)
    m.apply_translation([cx + 2.0 * fwd, cy, cz + 13.0])
    return m


def _rounded_box(extents, transform, radius=5.0, margin=3.5, subdiv=2):
    """A box with filleted edges/corners (convex hull of 8 corner spheres),
    sized ``extents`` + ``margin`` and placed by ``transform``.  Fully encloses a
    box of the same size, so it cleanly swallows the micro-servo case into a
    smooth rounded 'knee' bulge."""
    he = np.asarray(extents, float) / 2.0 + margin
    radius = float(min(radius, he.min() * 0.85))
    inner = he - radius
    base = trimesh.creation.icosphere(subdivisions=subdiv, radius=radius).vertices
    pts = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                pts.append(base + inner * np.array([sx, sy, sz]))
    hull = trimesh.Trimesh(vertices=np.vstack(pts)).convex_hull
    hull.apply_transform(transform)
    return hull


# ----------------------------------------------------------------------------
# Real micro-servo cluster (servo body + tiny disc horn), built ONCE in the
# servo-local frame and copied onto each joint at TRUE size.
# ----------------------------------------------------------------------------


def _make_micro_servo():
    """A compact micro metal-gear servo: body + 2 mounting tabs + output boss."""
    parts = [box((0.0, 0.0, 0.0), (MS_W, MS_D, MS_H))]
    for sx in (1.0, -1.0):
        parts.append(box((sx * (MS_W / 2.0 + 3.5), 0.0, MS_H / 2.0 - 5.0),
                         (7.0, MS_D, 2.4)))
    parts.append(frustum((MS_OX, 0.0, MS_H / 2.0 - 0.5),
                         (MS_OX, 0.0, MS_H / 2.0 + MS_OUTH), 5.2, 4.4, n=18))
    return trimesh.util.concatenate(parts)


def _make_horn():
    """A small printed disc horn the driven bone bolts to."""
    return frustum((MS_OX, 0.0, MS_H / 2.0 + MS_OUTH),
                   (MS_OX, 0.0, HORN_TOP), 6.0, 5.4, n=20)


_BASE_SERVO = _make_micro_servo()
_BASE_HORN = _make_horn()


def servo_frame(P, axis, long):
    """4x4 transform mapping servo-local coords to world so the horn TOP (local
    (MS_OX, 0, HORN_TOP)) lands at joint point ``P`` with the output axis along
    ``axis`` and the body length (local +X) along ``long``."""
    z = _norm(axis)
    x = np.asarray(long, float)
    x = x - z * float(np.dot(x, z))
    x = _norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(P, float) - R @ np.array([MS_OX, 0.0, HORN_TOP])
    return T


def add_servo(fixed, name, P, axis, long):
    """Place a real micro servo cluster at the SCALED joint location ``P``
    (design mm; scaled here, servo kept TRUE size).  Returns ``(servo_mesh, T)``
    -- the world transform ``T`` lets callers bolt a printed mount to the case."""
    Pw = np.asarray(P, float) * SCALE
    T = servo_frame(Pw, axis, long)
    s = _BASE_SERVO.copy(); s.apply_transform(T)
    fixed.append(("micro_servo", f"{name}_servo", s))
    h = _BASE_HORN.copy(); h.apply_transform(T)
    fixed.append(("servo_horn", f"{name}_horn", h))
    return s, T


def build_servo_mount(struct, name, T):
    """A printed bracket that captures a body/neck micro servo: a backing rib on
    the servo's mounting (-Z) face + two screw bosses under its mounting tabs
    (which carry the M2 self-tap screws).  Built in servo-local mm, placed by the
    servo's world transform ``T`` so it hugs the real case."""
    tabx = MS_W / 2.0 + 3.5            # mounting-tab centre offset (servo-local X)
    tabz = MS_H / 2.0 - 5.0            # tab height (servo-local Z)
    feats = [box((0.0, 0.0, -MS_H / 2.0 - 2.5), (MS_W + 2.0, MS_D + 6.0, 5.0))]
    for sx in (1.0, -1.0):
        feats.append(frustum((sx * tabx, 0.0, tabz),
                             (sx * tabx, 0.0, -MS_H / 2.0 - 2.5), 3.2, 3.2, n=16))
    m = trimesh.util.concatenate(feats)
    m.apply_transform(T)
    struct.append(("servo_mount", f"{name}_mount", m))


# ----------------------------------------------------------------------------
# Dimensional design (DESIGN mm) -- a compact, chubby, neotenic kitten.
# ----------------------------------------------------------------------------
# ONE continuous rounded body (x, z_centre, half_y, half_z) front -> rear:
# short, plump, nearly circular sections -> a chubby barrel, not a long teardrop.
# Rev 10: SHORT + ROUND pot-belly kitten body -- compressed in X, fatter
# cross-sections (half up to ~53), lower z so the big head dominates.
TORSO_CTRL = [
    (58.0, 80.0, 24.0, 27.0),    # chest front (meets the neck)
    (46.0, 78.0, 42.0, 42.0),    # wide chest -> shoulder servos tuck inside
    (28.0, 77.0, 49.0, 48.0),    # deep round barrel
    (4.0, 77.0, 52.0, 49.0),     # roundest pot belly (belly a touch shallower than wide)
    (-22.0, 78.0, 50.0, 48.0),
    (-44.0, 82.0, 45.0, 43.0),   # round haunches
    (-58.0, 88.0, 30.0, 32.0),
    (-68.0, 94.0, 15.0, 18.0),   # tail base
]

# Short, STUBBY kitten legs -- 2 DOF each in the sagittal plane.  Roots tuck
# INBOARD of the body wall so the hip/shoulder servos hide inside the torso; paws
# a touch forward of the pivots for a friendly, stable, slightly splayed stance.
# Rev 10: lower pivots + shorter reach -> stubbier baby legs; the legs sit further
# OUTBOARD so they emerge from the lower sides of the round belly (which has
# clearance there) instead of piercing the deepest belly under the centreline.
FRONT_Y = 36.0
FRONT = {"shoulder": (42.0, 64.0), "elbow": (45.0, 34.0), "paw": (48.0, 0.0)}
HIND_Y = 40.0
HIND = {"hip": (-40.0, 68.0), "knee": (-34.0, 36.0), "paw": (-44.0, 0.0)}

NECK_BASE = (50.0, 96.0)
HEAD_CENTER = (82.0, 132.0)            # BIG round head, held high + forward
HEAD_RXYZ = (46.0, 42.0, 47.0)         # bigger + domed (taller than wide)

# Joint-end trim (DESIGN mm): bones shortened just enough not to overlap the
# OTHER bone at a shared joint while still reaching the servo/horn (no hovering).
JT = 6.5

# ----------------------------------------------------------------------------
# Manufacturability (rev 9): how the body splits into printable, screw-together
# parts.  DESIGN mm.
# ----------------------------------------------------------------------------
SEAM_CLEAR = 0.2        # printable clearance at a mating seam (each half shifts SEAM_CLEAR/2)
HATCH_X = (-32.0, 28.0)   # belly-hatch footprint along X (under the electronics)
HATCH_DEPTH = 0.55        # fraction of the lower half-height the hatch wraps up
HATCH_CLEAR = 0.3         # in-plane shrink of the hatch panel for a print fit

# Seam screw bosses along the sagittal parting line (DESIGN mm: x, z).  Top
# (dorsal) row + a couple low on the belly fore/aft of the hatch.  M2 self-tap.
SEAM_BOSSES = [
    (46.0, "top"), (22.0, "top"), (-4.0, "top"), (-32.0, "top"), (-56.0, "top"),
    (40.0, "bot"), (-52.0, "bot"),
]
M2_TAP_BOSS_R = 2.6     # outer radius of an M2 self-tap boss (2 mm pilot inside)


# Reverse-sorted-by-x interpolation tables for the torso silhouette so we can
# query (z_centre, half_y, half_z) at any station X (used to seat seam bosses,
# the hatch frame, and to size the split).
_TX = np.array([c[0] for c in TORSO_CTRL])[::-1]
_TZC = np.array([c[1] for c in TORSO_CTRL])[::-1]
_THY = np.array([c[2] for c in TORSO_CTRL])[::-1]
_THZ = np.array([c[3] for c in TORSO_CTRL])[::-1]


def _torso_profile(x):
    """(z_centre, half_y, half_z) of the torso silhouette at station ``x``."""
    return (float(np.interp(x, _TX, _TZC)),
            float(np.interp(x, _TX, _THY)),
            float(np.interp(x, _TX, _THZ)))


# ----------------------------------------------------------------------------
# Builders.  Structural meshes -> ``struct`` (scaled later); real hardware ->
# ``fixed`` (placed at scaled joints, true size).
# ----------------------------------------------------------------------------


def _torso_mesh():
    """The ONE continuous chubby cat-body loft (the rev-8 silhouette)."""
    secs = catmull(TORSO_CTRL, seg=7)
    rings = [_body_ring(x, zc, hy, hz, n=72) for (x, zc, hy, hz) in secs]
    return loft_rings(rings)


def build_body_shells(struct):
    """Split the chubby body into PRINTABLE, screw-together pieces (rev 9):

    * ``shell_left`` / ``shell_right`` -- the torso halved on the sagittal
      (Y=0) plane.  The seam runs along the hidden dorsal spine + ventral
      midline; each half is shifted out by SEAM_CLEAR/2 to leave a real ~0.2 mm
      print-fit gap (still well within the 2 mm connectivity tolerance).
    * ``belly_hatch`` -- a removable underside panel over the electronics bay so
      the board + LiPo install/service without splitting the shell; shrunk
      slightly in-plane for a drop-in fit.

    Implemented by partitioning the loft's FACES (left / right / hatch) so the
    three printed parts exactly tile the cute silhouette."""
    mesh = _torso_mesh()
    fc = mesh.triangles_center
    hx0, hx1 = HATCH_X

    # hatch = the lower-belly band inside the X window (the underside panel)
    zc = np.interp(fc[:, 0], _TX, _TZC)
    hz = np.interp(fc[:, 0], _TX, _THZ)
    hatch_mask = ((fc[:, 0] > hx0) & (fc[:, 0] < hx1)
                  & (fc[:, 2] < zc - HATCH_DEPTH * hz))
    left_mask = (fc[:, 1] > 0.0) & ~hatch_mask
    right_mask = (~left_mask) & ~hatch_mask

    def _sub(mask, dy, name, ptype):
        idx = np.nonzero(mask)[0]
        sm = mesh.submesh([idx], append=True, repair=False)
        if dy:
            sm.apply_translation([0.0, dy, 0.0])
        struct.append((ptype, name, sm))

    _sub(left_mask, +SEAM_CLEAR / 2.0, "shell_left", "shell_left")
    _sub(right_mask, -SEAM_CLEAR / 2.0, "shell_right", "shell_right")

    # the hatch panel: same belly faces, shrunk in-plane about its own centroid
    idx = np.nonzero(hatch_mask)[0]
    hatch = mesh.submesh([idx], append=True, repair=False)
    c = hatch.centroid
    s = 1.0 - HATCH_CLEAR / max(20.0, float(hatch.extents.max()))
    hatch.apply_translation(-c); hatch.apply_scale(s); hatch.apply_translation(c)
    struct.append(("belly_hatch", "belly_hatch", hatch))


def build_seam_features(struct):
    """Screw bosses + an alignment lip along the sagittal parting line so the two
    shell halves register and bolt together (M2 self-tap into the bosses)."""
    bosses = []
    for x, where in SEAM_BOSSES:
        zc, _hy, hz = _torso_profile(x)
        z = zc + (hz - 7.0) if where == "top" else zc - (hz - 7.0)
        # boss is a short cylinder straddling Y=0 (screw axis along Y joins halves)
        bosses.append(frustum((x, -7.0, z), (x, 7.0, z),
                              M2_TAP_BOSS_R, M2_TAP_BOSS_R, n=16))
    struct.append(("seam_bosses", "seam_bosses", trimesh.util.concatenate(bosses)))

    # alignment lip: a thin tongue running along the dorsal seam on the inner
    # wall so the halves can't shear — a registration feature, not a fastener.
    lip_rings = []
    for x in np.linspace(54.0, -64.0, 26):
        zc, _hy, hz = _torso_profile(x)
        lip_rings.append(_ellipse_ring((x, 0.0, zc + hz - 4.0),
                                       (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                                       2.6, 3.4, 10))
    struct.append(("hatch_frame", "seam_lip", loft_rings(lip_rings, cap=True)))


def build_hatch_frame(struct):
    """A recessed rim + 4 corner bosses the belly hatch screws down into."""
    hx0, hx1 = HATCH_X
    feats = []
    for x in (hx0 + 5.0, hx1 - 5.0):
        zc, hy, hz = _torso_profile(x)
        zb = zc - HATCH_DEPTH * hz       # the hatch opening edge height at this x
        for sy in (1.0, -1.0):
            feats.append(frustum((x, sy * (hy * 0.45), zb + 4.0),
                                 (x, sy * (hy * 0.45), zb - 5.0),
                                 M2_TAP_BOSS_R, M2_TAP_BOSS_R, n=14))
    # a thin ledge spanning the opening for the panel to seat on
    feats.append(box((sum(HATCH_X) / 2.0, 0.0,
                      _torso_profile(sum(HATCH_X) / 2.0)[0]
                      - HATCH_DEPTH * _torso_profile(sum(HATCH_X) / 2.0)[2] + 1.0),
                     (hx1 - hx0 - 8.0, 8.0, 2.5)))
    struct.append(("hatch_frame", "hatch_frame", trimesh.util.concatenate(feats)))


def build_head_mount(struct):
    """A printed socket at the head base that grips the neck column / neck-servo
    horn, so the separately-printed head bolts onto the driven neck."""
    hc = (HEAD_CENTER[0], 0.0, HEAD_CENTER[1])
    socket = frustum((hc[0] - 24.0, 0.0, hc[2] - 30.0),
                     (hc[0] - 12.0, 0.0, hc[2] - 14.0), 11.0, 9.0, n=20)
    struct.append(("head_mount", "head_mount", socket))


def _eye(struct, hc, s):
    """A big, friendly forward-facing eye: white + green iris + pupil + a tiny
    catch-light (the sparkle that sells 'cute')."""
    tag = "L" if s > 0 else "R"
    # rev 10: BIGGER, ROUNDER eyes set lower (~12 mm below skull centre) and wider
    # apart -- the strongest baby-kitten cue.  Iris fills more of the eye.
    ey = s * 19.0
    struct.append(("eye_white", f"{tag}_eye_white",
                   ellipsoid((hc[0] + 30.0, ey, hc[2] - 11.0), 16.5, 15.0, 16.5, subdiv=4)))
    struct.append(("iris", f"{tag}_iris",
                   ellipsoid((hc[0] + 41.5, ey + s * 1.0, hc[2] - 11.0), 8.5, 11.5, 11.5, subdiv=3)))
    struct.append(("pupil", f"{tag}_pupil",
                   ellipsoid((hc[0] + 47.0, ey + s * 1.0, hc[2] - 11.0), 5.2, 7.0, 7.6, subdiv=3)))
    struct.append(("eye_gleam", f"{tag}_eye_gleam",
                   sphere((hc[0] + 49.5, ey + s * 4.0, hc[2] - 4.0), 2.8, subdiv=2)))


def _ear(hc, s):
    """A soft, ROUND baby-kitten ear: a plump near-round ellipsoid (low aspect,
    not a sharp triangle), tilted slightly outward on top of the big head."""
    m = ellipsoid((hc[0] - 8.0, s * 25.0, hc[2] + 34.0), 12.0, 9.5, 13.0, subdiv=3)
    c = m.centroid
    th = np.radians(s * -13.0)            # gentle outward tilt
    R = trimesh.transformations.rotation_matrix(th, [1.0, 0.0, 0.0], point=c)
    m.apply_transform(R)
    return m


def _ear_inner(hc, s):
    m = ellipsoid((hc[0] - 6.0, s * 25.5, hc[2] + 33.0), 7.0, 5.5, 8.5, subdiv=3)
    c = m.centroid
    th = np.radians(s * -13.0)
    R = trimesh.transformations.rotation_matrix(th, [1.0, 0.0, 0.0], point=c)
    m.apply_transform(R)
    return m


def build_neck_head(struct, fixed):
    nb = (NECK_BASE[0], 0.0, NECK_BASE[1])
    hc = (HEAD_CENTER[0], 0.0, HEAD_CENTER[1])
    _sv, Tneck = add_servo(fixed, "neck_pan", (NECK_BASE[0], 0.0, NECK_BASE[1]),
                           (0, 0, 1), (1, 0, 0))
    build_servo_mount(struct, "neck_pan", Tneck)
    # short, thick neck blending chest -> the big skull
    struct.append(("neck", "neck",
                   smooth_bone(nb, (hc[0] - 8.0, 0.0, hc[2] - 12.0),
                               [(0.0, 23.0), (0.5, 21.0), (1.0, 19.0)],
                               n=44, flat=0.96)))
    struct.append(("head", "head", ellipsoid(hc, *HEAD_RXYZ, subdiv=4)))
    # extra-plump chubby cheeks low on the face -- a key baby-schema cue
    for s, tag in ((1.0, "L"), (-1.0, "R")):
        struct.append(("cheek", f"{tag}_cheek",
                       ellipsoid((hc[0] + 26.0, s * 27.0, hc[2] - 16.0),
                                 18.0, 16.0, 17.0, subdiv=3)))
    # small soft muzzle + tiny pink nose, set in the lower face
    struct.append(("muzzle", "muzzle",
                   ellipsoid((hc[0] + 37.0, 0.0, hc[2] - 19.0), 15.0, 16.0, 12.0, subdiv=4)))
    struct.append(("nose", "nose", sphere((hc[0] + 50.0, 0.0, hc[2] - 16.0), 4.6, subdiv=3)))
    for s, tag in ((1.0, "L"), (-1.0, "R")):
        _eye(struct, hc, s)
        struct.append(("ear", f"{tag}_ear", _ear(hc, s)))
        struct.append(("ear_inner", f"{tag}_ear_inner", _ear_inner(hc, s)))


def build_collar(struct, fixed):
    """A little coral collar + gold bell around the lower neck -- cute, and it
    tidily hides the neck joint and the servo-bus wiring at the chest."""
    rings = []
    for x in (44.0, 48.0, 52.0):
        # follow the neck taper; the collar sits proud of the neck surface
        r = 23.0 - (x - 44.0) * 0.4
        rings.append(_ellipse_ring((x, 0.0, 100.0 + (x - 48.0) * 0.3),
                                   (1.0, 0.0, 0.25), (0.0, 0.0, 1.0), r, r, 40))
    struct.append(("collar", "collar", loft_rings(rings)))
    struct.append(("bell", "bell", sphere((56.0, 0.0, 80.0), 5.4, subdiv=3)))


def build_tail(struct):
    """One short, stubby, softly-curled BABY-kitten tail -- ONE continuous mesh.

    Rev-10 bugfix: the old tail lofted only 7 control points through a sharp hook,
    so the tip segment turned ~90 deg between rings -- the loft self-intersected
    and the tip read as DETACHED.  Here the centreline is first Catmull-Rom
    smoothed into ~50 densely-spaced points (so each ring turns only a few
    degrees, no self-intersection), the radii taper smoothly to a STUBBY tip
    (~6.5 mm, not a thin point), and the thick base (~15 mm) overlaps deep into
    the rump so the tail is solidly attached."""
    ctrl = [(-62.0, 94.0), (-72.0, 109.0), (-79.0, 127.0), (-76.0, 144.0),
            (-64.0, 153.0), (-51.0, 154.0)]            # soft upward curl, forward tip
    rad_ctrl = [15.0, 13.5, 11.5, 9.8, 8.2, 7.0]       # stubby: never tapers to a spike
    C = np.array([[x, 0.0, z] for x, z in ctrl], float)
    P = catmull(C, seg=10)                              # ~50 closely-spaced points
    t = np.linspace(0.0, len(rad_ctrl) - 1, len(P))
    radii = np.interp(t, np.arange(len(rad_ctrl)), rad_ctrl)
    rings = []
    for i, (p, r) in enumerate(zip(P, radii)):
        if i == 0:
            ax = P[1] - P[0]
        elif i == len(P) - 1:
            ax = P[-1] - P[-2]
        else:
            ax = P[i + 1] - P[i - 1]
        rings.append(_ellipse_ring(p, ax, np.array([1.0, 0.0, 0.0]), r, r, 24))
    struct.append(("tail_segment", "tail", loft_rings(rings)))


def build_knee(struct, name, servo_mesh):
    """A small, smooth FUR-colored 'knee' bulge that swallows the micro-servo at
    the elbow/knee so it reads as a chubby joint, not a black box."""
    obb = servo_mesh.bounding_box_oriented
    pad = _rounded_box(obb.primitive.extents, obb.primitive.transform,
                       radius=5.0, margin=2.8)
    struct.append(("knee", f"{name}_knee", pad))


def build_front_leg(struct, fixed, tag, s):
    """2-DOF front leg: shoulder-pitch + elbow-pitch, hinging about Y.  Short,
    stubby and chubby, with both micro servos hidden inside the body/leg."""
    y = s * FRONT_Y
    P = FRONT
    sh, el = _pt(P["shoulder"], y), _pt(P["elbow"], y)
    paw_foot = np.array([P["paw"][0] + 2.0, y, 5.0])

    # shoulder servo tucks INBOARD inside the torso (hidden by the body shell)
    _sv, Tsh = add_servo(fixed, f"{tag}_shoulder", sh, (0, s, 0), (0.2, 0, 1))
    build_servo_mount(struct, f"{tag}_shoulder", Tsh)

    a, b = _trim(sh, el, JT, JT)
    arm = smooth_bone(a, b, [(0.0, 15.0), (0.5, 15.5), (1.0, 15.5)], flat=0.95)
    struct.append(("upper_arm", f"{tag}_upper_arm", arm))

    # elbow servo hides inside the chubby knee bulge (no external box)
    ev, _Tel = add_servo(fixed, f"{tag}_elbow", el, (0, -s, 0), sh - el)
    build_knee(struct, f"{tag}_elbow", ev)

    # trim the forearm well below the elbow so its root tucks under the (whitelisted)
    # fur knee bulge instead of the bare bone poking into the chubby belly shell
    a, b = _trim(el, paw_foot, 13.0, 0.0)
    fore = smooth_bone(a, b, [(0.0, 12.0), (0.45, 11.0), (1.0, 10.5)], flat=0.95)
    struct.append(("forearm", f"{tag}_forearm", fore))
    struct.append(("front_paw", f"{tag}_front_paw", smooth_paw((P["paw"][0], y, 0.0))))


def build_hind_leg(struct, fixed, tag, s):
    """2-DOF hind leg: hip-pitch + knee-pitch, hinging about Y."""
    y = s * HIND_Y
    P = HIND
    hip, kn = _pt(P["hip"], y), _pt(P["knee"], y)
    paw_foot = np.array([P["paw"][0] + 2.0, y, 5.0])

    _sv, Thip = add_servo(fixed, f"{tag}_hip", hip, (0, s, 0), (-0.15, 0, 1))
    build_servo_mount(struct, f"{tag}_hip", Thip)

    a, b = _trim(hip, kn, JT, JT)
    fem = smooth_bone(a, b, [(0.0, 17.0), (0.5, 16.5), (1.0, 16.0)], flat=0.95)
    struct.append(("femur", f"{tag}_femur", fem))

    ev, _Tkn = add_servo(fixed, f"{tag}_knee", kn, (0, -s, 0), hip - kn)
    build_knee(struct, f"{tag}_knee", ev)

    # trim the shank well below the knee so its root tucks under the (whitelisted)
    # fur knee bulge instead of the bare bone poking into the chubby belly shell
    a, b = _trim(kn, paw_foot, 13.0, 0.0)
    shank = smooth_bone(a, b, [(0.0, 12.0), (0.45, 11.0), (1.0, 10.5)], flat=0.95)
    struct.append(("shank", f"{tag}_shank", shank))
    struct.append(("hind_paw", f"{tag}_hind_paw", smooth_paw((P["paw"][0], y, 0.0))))


def build_electronics(fixed):
    """Compact ESP32 controller + small 2S LiPo + printed mounts, packaged LOW
    inside the rounded belly so nothing shows from outside.  The cradle touches
    the inner belly wall (so the cluster is part of the ONE connected chassis)."""
    bx, by, bz = (c * SCALE for c in BATTERY_CENTER)
    fixed.append(("battery", "battery",
                  box((bx, by, bz), (BATTERY_L, BATTERY_W, BATTERY_H))))

    cx, cy, cz = (c * SCALE for c in CONTROLLER_CENTER)

    # cradle floor BRIDGES the inner belly wall up to the battery underside (so it
    # touches the shell -> one connected chassis even as the belly gets deeper),
    # plus side rails and a riser carrying the controller tray.
    belly_floor = _torso_profile(BATTERY_CENTER[0])[0] - _torso_profile(BATTERY_CENTER[0])[2]
    bat_bot = bz - BATTERY_H / 2.0
    floor_top = bat_bot + 0.5
    floor_h = max(3.0, floor_top - (belly_floor + 1.0))
    cradle = [box((bx, by, floor_top - floor_h / 2.0),
                  (BATTERY_L + 8, BATTERY_W + 8, floor_h))]
    for sy in (1.0, -1.0):
        cradle.append(box((bx, sy * (BATTERY_W / 2.0 + 3.0), bz),
                          (BATTERY_L + 8, 2.4, BATTERY_H + 3)))
    # riser up to the controller tray, tying the two together
    riser_z0 = bz
    riser_z1 = cz - CTRL_H / 2.0 - 2.0
    cradle.append(box(((bx + cx) / 2.0, 0.0, (riser_z0 + riser_z1) / 2.0),
                      (10.0, 12.0, abs(riser_z1 - riser_z0) + 6.0)))
    fixed.append(("battery_cradle", "battery_cradle",
                  trimesh.util.concatenate(cradle)))

    tray = box((cx - 4.0, cy, cz - CTRL_H / 2.0 - 2.0), (CTRL_W + 8, CTRL_D + 8, 3.0))
    fixed.append(("controller_tray", "controller_tray", tray))
    fixed.append(("controller", "controller",
                  box((cx, cy, cz), (CTRL_W, CTRL_D, CTRL_H))))

    # 4 printed standoffs lift the ESP32 board off its tray (M2 self-tap posts)
    posts = []
    for sx in (CTRL_W / 2.0 - 4.0, -(CTRL_W / 2.0 - 4.0)):
        for sy in (CTRL_D / 2.0 - 4.0, -(CTRL_D / 2.0 - 4.0)):
            posts.append(frustum((cx + sx, cy + sy, cz - CTRL_H / 2.0 - 2.0),
                                 (cx + sx, cy + sy, cz - CTRL_H / 2.0 + 1.0),
                                 2.4, 2.4, n=12))
    fixed.append(("board_standoffs", "board_standoffs",
                  trimesh.util.concatenate(posts)))

    # a thin retainer strap arching over the LiPo to hold it in the cradle
    strap = box((bx, by, bz + BATTERY_H / 2.0 + 1.5), (8.0, BATTERY_W + 8, 3.0))
    fixed.append(("battery_strap", "battery_strap", strap))


def build_fasteners(fixed):
    """Representative M2 screws (so the exploded view + BOM are concrete): the
    seam screws joining the shell halves, and the belly-hatch screws."""
    seam = []
    for x, where in SEAM_BOSSES:
        zc, _hy, hz = _torso_profile(x)
        z = zc + (hz - 7.0) if where == "top" else zc - (hz - 7.0)
        seam.append(frustum((x, 5.0, z), (x, 11.0, z), 1.0, 1.0, n=10))   # shank
        seam.append(frustum((x, 11.0, z), (x, 13.0, z), 2.0, 2.0, n=10))  # head
    fixed.append(("shell_screws", "shell_screws", trimesh.util.concatenate(seam)))

    hatch = []
    for x in (HATCH_X[0] + 5.0, HATCH_X[1] - 5.0):
        zc, hy, hz = _torso_profile(x)
        zb = zc - HATCH_DEPTH * hz
        for sy in (1.0, -1.0):
            y = sy * (hy * 0.45)
            hatch.append(frustum((x, y, zb - 1.0), (x, y, zb - 7.0), 1.0, 1.0, n=10))
            hatch.append(frustum((x, y, zb - 7.0), (x, y, zb - 9.0), 2.0, 2.0, n=10))
    fixed.append(("hatch_screws", "hatch_screws", trimesh.util.concatenate(hatch)))


def build_all():
    struct, fixed = [], []
    build_body_shells(struct)
    build_seam_features(struct)
    build_hatch_frame(struct)
    build_head_mount(struct)
    build_neck_head(struct, fixed)
    build_collar(struct, fixed)
    build_tail(struct)
    build_electronics(fixed)
    build_fasteners(fixed)
    for s, tag in ((1.0, "L"), (-1.0, "R")):
        build_front_leg(struct, fixed, tag, s)
        build_hind_leg(struct, fixed, tag, s)
    return struct, fixed


# ----------------------------------------------------------------------------
# Static-stability / gait feasibility + torque check
# ----------------------------------------------------------------------------
PLA_DENS = 0.00124      # g/mm^3 solid PLA
SHELL_MM = 2.0          # solid-equivalent wall thickness for hollow shells
_PLA_EFF = 0.00055      # near-solid small printed parts (legs/tail/face)


def _part_mass_g(role, mesh):
    if role == "motor":
        return SERVO_MASS_G
    if role == "horn":
        return 1.5
    if role == "battery":
        return BATTERY_MASS_G
    if role == "controller":
        return CONTROLLER_MASS_G
    if role in ("body", "head", "shell"):
        # shell halves + hatch are printed ~2 mm walls -> mass from surface area
        return abs(float(mesh.area)) * SHELL_MM * PLA_DENS
    if role in ("bracket", "fastener", "boss"):
        return max(1.0, abs(float(mesh.volume)) * PLA_DENS * 0.5)
    return abs(float(mesh.volume)) * _PLA_EFF


def stability_report(parts):
    tot = 0.0
    com = np.zeros(3)
    for ptype, _name, mesh in parts:
        m = _part_mass_g(ROLE.get(ptype, "part"), mesh)
        com += m * np.asarray(mesh.centroid, float)
        tot += m
    com /= tot

    feet = np.array([m.centroid[:2] for pt, _n, m in parts if ROLE.get(pt) == "paw"])
    c2 = feet.mean(0)
    order = np.argsort(np.arctan2(feet[:, 1] - c2[1], feet[:, 0] - c2[0]))
    poly = feet[order]
    p = com[:2]
    inside = True
    margin = 1e9
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        nrm = np.array([-e[1], e[0]])
        nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
        d = float(np.dot(p - a, nrm))
        if d < 0:
            inside = False
        margin = min(margin, abs(d))
    return tot, com, poly, inside, margin


def torque_report(parts, total_g):
    g = 9.81
    W = total_g / 1000.0 * g
    kn = _pt(HIND["knee"], 0.0) * SCALE
    paw = np.array([HIND["paw"][0], 0.0, 0.0]) * SCALE
    arm = float(np.linalg.norm((paw - kn))) / 1000.0
    tau_stand = (W / 4.0) * arm
    tau_trot = (W / 2.0) * arm
    return arm, tau_stand, tau_trot


# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------


def _now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%S.000Z")


def _write_meta(versions: list[str]) -> None:
    """Write/refresh ``cat_viz/meta.json`` in BuildViz's project/build/version
    schema.  ``main`` (the live tip) is always the default version; every frozen
    named version under ``versions/<name>/`` is listed too.  Existing
    ``pushedAt`` timestamps are preserved."""
    prior = {}
    created_at = _now_iso()
    if META_PATH.exists():
        try:
            old = json.loads(META_PATH.read_text())
            created_at = old.get("createdAt", created_at)
            for entry in old.get("versions", []):
                if isinstance(entry, dict) and entry.get("name"):
                    prior[entry["name"]] = entry.get("pushedAt")
        except (json.JSONDecodeError, OSError):
            pass

    now = _now_iso()
    names = [DEFAULT_VERSION] + [v for v in versions if v != DEFAULT_VERSION]
    meta = {
        "schema": 2,
        "buildId": BUILD_ID,
        "project": BUILD_ID,
        "build": BUILD_ID,
        "name": BUILD_NAME,
        "createdAt": created_at,
        "updatedAt": now,
        "defaultVersion": DEFAULT_VERSION,
        "latestVersion": DEFAULT_VERSION,
        "versions": [
            {"name": n, "pushedAt": (now if n == DEFAULT_VERSION
                                     else prior.get(n) or now)}
            for n in names
        ],
    }
    META_PATH.write_text(json.dumps(meta, indent=2) + "\n")


def _frozen_version_names() -> list[str]:
    if not VERSIONS_DIR.exists():
        return []
    return sorted(
        d.name for d in VERSIONS_DIR.iterdir()
        if d.is_dir() and (d / "scene.json").exists())


def freeze_version(name: str | None = None, force: bool = False) -> None:
    """Freeze the current live ``cat_viz/`` as an immutable NAMED VERSION at
    ``cat_viz/versions/<name>/`` -- BuildViz's native per-build version layout.
    The one build ``robot-cat`` simply gains another version (no separate hub
    build id).  Refuses to clobber an existing frozen version unless force."""
    name = name or VERSION
    if name == DEFAULT_VERSION:
        raise SystemExit(
            f"'{DEFAULT_VERSION}' is the live tip (build root), not a frozen "
            f"version. Pass a name like 'v{REV}'.")

    scene = OUT_DIR / "scene.json"
    if not scene.exists():
        raise SystemExit(
            f"Nothing to freeze: {scene} does not exist. "
            f"Run `./run.sh robot_cat/build_cat.py` first.")

    dest = VERSIONS_DIR / name
    if dest.exists():
        if not force:
            raise SystemExit(
                f"Version already frozen: {dest}\n"
                f"Frozen versions are immutable. Re-run with --force to replace "
                f"it, or pick a new name / bump REV.")
        print(f"--force: replacing existing frozen version {dest}")
        shutil.rmtree(dest)

    dest.mkdir(parents=True)
    shutil.copy2(scene, dest / "scene.json")
    spec = OUT_DIR / "design_spec.yaml"
    if spec.exists():
        shutil.copy2(spec, dest / "design_spec.yaml")
    shutil.copytree(STL_DIR, dest / "stl")
    print(f"Froze live cat_viz/  ->  versions/{name}/  (scene + design_spec + stl)")

    _write_meta(_frozen_version_names())
    print(f"Updated {META_PATH.name}: default '{DEFAULT_VERSION}' (live tip) + "
          f"versions {_frozen_version_names()}")
    print(f"\nThe single hub build '{BUILD_ID}' now serves version '{name}':")
    print(f"  http://127.0.0.1:5183/?build={BUILD_ID}&version={name}")
    print("(If the hub doesn't show it yet: "
          "`node /Users/lbiewald/buildviz/bin/buildviz.mjs hub restart`.)")


def build():
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    struct, fixed = build_all()
    for _pt, _n, m in struct:
        m.apply_scale(SCALE)
    parts = struct + fixed

    z_min = min(float(m.bounds[0][2]) for _pt, _n, m in parts)
    lift = -z_min
    for _pt, _n, m in parts:
        m.apply_translation([0.0, 0.0, lift])

    meshes_json, instances_json, allb = [], [], []
    role_counts, type_counts = {}, {}
    for idx, (ptype, name, mesh) in enumerate(parts):
        fname = f"{idx:02d}_{name}.stl"
        mesh.export(STL_DIR / fname)
        allb.append(mesh.bounds)
        mid = f"stl:{name}"
        role = ROLE.get(ptype, "part")
        role_counts[role] = role_counts.get(role, 0) + 1
        type_counts[ptype] = type_counts.get(ptype, 0) + 1
        meshes_json.append({"id": mid, "name": fname, "url": f"stl/{fname}"})
        instances_json.append({
            "id": f"{idx:02d}-{name}",
            "meshId": mid,
            "name": name.replace("_", " "),
            "partType": ptype,
            "role": role,
            "moving": ptype in MOVING_TYPES,
            "color": PALETTE.get(ptype, "#888888"),
            "transform": [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0],
            "centroid": [float(v) for v in mesh.centroid],
        })

    b = np.array(allb)
    center = [float(v) for v in (b[:, 0, :].min(0) + b[:, 1, :].max(0)) / 2.0]
    scene = {
        "name": SCENE_NAME,
        "source": str(OUT_DIR),
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": center,
        "meshes": meshes_json,
        "instances": instances_json,
    }
    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    _write_meta(_frozen_version_names())

    n_servo = type_counts.get("micro_servo", 0)
    size = b.reshape(-1, 3).max(0) - b.reshape(-1, 3).min(0)
    print(f"Wrote {OUT_DIR / 'scene.json'}  ({len(instances_json)} instances, "
          f"SCALE={SCALE}, lift={lift:.1f} mm)")
    print(f"Overall bbox (mm): L(x)={size[0]:.0f}  W(y)={size[1]:.0f}  H(z)={size[2]:.0f}")
    print(f"Micro servos: {n_servo}  ->  servo mass {n_servo * SERVO_MASS_G:.0f} g")
    print("Parts by role: " + ", ".join(f"{k}={v}" for k, v in sorted(role_counts.items())))

    tot_g, com, poly, inside, margin = stability_report(parts)
    print(f"\nEstimated total mass (incl. battery {BATTERY_MASS_G:.0f} g + controller "
          f"{CONTROLLER_MASS_G:.0f} g): {tot_g:.0f} g")
    print(f"CoM (mm): x={com[0]:.1f} y={com[1]:.1f} z={com[2]:.1f}")
    print("Support polygon (paw XY): " + ", ".join(f"({x:.0f},{y:.0f})" for x, y in poly))
    verdict = "INSIDE" if inside else "OUTSIDE"
    print(f"CoM ground projection is {verdict} support polygon; edge margin = {margin:.0f} mm")

    arm, tau_s, tau_t = torque_report(parts, tot_g)
    print(f"\nKnee moment arm (scaled): {arm*1000:.0f} mm; micro-servo stall = "
          f"{SERVO_STALL_NM:.2f} N-m")
    print(f"Standing knee torque ~ {tau_s:.3f} N-m  ->  margin {SERVO_STALL_NM/tau_s:.1f}x")
    print(f"Trot knee torque     ~ {tau_t:.3f} N-m  ->  margin {SERVO_STALL_NM/tau_t:.1f}x")

    fx = sorted(m.centroid[0] for pt, _n, m in parts if ROLE.get(pt) == "paw")
    print(f"Foot fore/aft span: {fx[-1]-fx[0]:.0f} mm (front {fx[-1]:.0f} .. hind {fx[0]:.0f}); "
          f"lateral track ±{max(abs(m.centroid[1]) for pt,_n,m in parts if ROLE.get(pt)=='paw'):.0f} mm")


def main():
    parser = argparse.ArgumentParser(
        description="Build the robot-cat BuildViz model (rev %d) into the live "
                    "build root cat_viz/ (the 'main' version), or freeze the "
                    "current build as an immutable named BuildViz version." % REV)
    parser.add_argument(
        "--freeze", nargs="?", const=VERSION, default=None, metavar="NAME",
        help="Freeze the current cat_viz/ as the immutable named version "
             "cat_viz/versions/NAME/ (default NAME=%s) and record it in "
             "meta.json. Does not rebuild." % VERSION)
    parser.add_argument(
        "--force", action="store_true",
        help="With --freeze, replace an existing (immutable) frozen version.")
    args = parser.parse_args()

    if args.freeze is not None:
        freeze_version(name=args.freeze, force=args.force)
    else:
        build()


if __name__ == "__main__":
    main()
