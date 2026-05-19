"""Parametric fastener geometry generators.

These functions produce *visualization* meshes that look approximately
like the real McMaster-Carr fasteners.  They are NOT load-bearing
parts; the verifier probes them as bounding cylinders anyway.

Coordinate frame for every fastener mesh:

    Origin at the centre of the HEAD's mating face (the underside of a
    SHCS cap, or the visible outer face of a captive nyloc nut).
    +Z = body axis -- the body / shank extends UP (+Z) from the head's
         mating face, NOT down.  Note: when ``inspect_build`` places a
         fastener, it builds a transform that maps the mesh's +Z to
         ``-axis_world`` (because ``axis_world`` points INTO the
         material; the body of the screw sits between the head and the
         tip, i.e. on the side opposite to the driver approach).
    +X is arbitrary but consistent across nuts so the flats land on
       the X axis.

For a SHCS this means the cap head is at z in [-CAP_H, 0] and the
shank at z in [0, length].  For a nut this means the captive nut body
is at z in [-NUT_H, 0] (= INTO the hex pocket) and the shank passes
through z >= 0 freely.

All units are millimetres.
"""

from __future__ import annotations

import math

import numpy as np
import trimesh
from shapely.geometry import Polygon as _ShapelyPolygon


# ---------------------------------------------------------------------------
# Stock dimensions (DIN 912 / DIN 985 / etc.)
# ---------------------------------------------------------------------------

# M3 socket-head cap screw (DIN 912 / ISO 4762):
M3_THREAD_D     = 3.0
M3_CAP_D        = 5.5         # cap head outer diameter
M3_CAP_H        = 3.0         # cap head height
M3_HEX_SOCKET_AF = 2.5        # 2.5 mm hex key across-flats
M3_HEX_SOCKET_DEPTH = 1.5     # depth of the hex pocket inside the cap

# M2.5 socket-head cap screw (DIN 912):
M25_THREAD_D    = 2.5
M25_CAP_D       = 4.5
M25_CAP_H       = 2.5
M25_HEX_SOCKET_AF = 2.0
M25_HEX_SOCKET_DEPTH = 1.25

# M3 pan-head Phillips (DIN 7985 / ISO 7045):
M3_PAN_D        = 6.0         # pan head outer diameter
M3_PAN_H        = 2.4         # pan head height
M3_PAN_PHILLIPS_W = 0.9       # Phillips cross slot width
M3_PAN_PHILLIPS_DEPTH = 1.0

# M3 nylon-insert lock nut (DIN 985):
M3_NUT_AF       = 5.5         # hex across-flats (matches the hex pocket
                              #   in _servo_cradle_nut_traps)
M3_NUT_H        = 4.0         # overall body height (steel + nylon ring)
M3_NUT_STEEL_H  = 2.4         # steel portion (bottom)
M3_NUT_NYLON_H  = M3_NUT_H - M3_NUT_STEEL_H  # nylon ring on top


# ---------------------------------------------------------------------------
# Low-level primitive builders
# ---------------------------------------------------------------------------


def _hex_prism(across_flats: float, height: float) -> trimesh.Trimesh:
    """Build a regular hexagonal prism (axis = +Z), with the given
    across-flats width and given height.  Centroid sits at the origin.
    """
    half_af = across_flats / 2.0
    # The corner-to-corner distance is 2 * half_af / cos(30 deg).
    corner_r = half_af / math.cos(math.pi / 6.0)
    pts = []
    for k in range(6):
        # Vertex angle: start at 30 deg so two flats land on +/-X.
        ang = math.pi / 6.0 + k * math.pi / 3.0
        pts.append((corner_r * math.cos(ang), corner_r * math.sin(ang)))
    poly = _ShapelyPolygon(pts)
    mesh = trimesh.creation.extrude_polygon(poly, height=height)
    # extrude_polygon extrudes in +Z from z=0; centre about the origin
    # so the bottom face sits at z=-H/2 and the top at z=+H/2.
    mesh.apply_translation([0.0, 0.0, -height / 2.0])
    return mesh


def _cyl(radius: float, height: float, sections: int = 32) -> trimesh.Trimesh:
    """Cylinder centred on the origin, axis = +Z."""
    return trimesh.creation.cylinder(radius=radius, height=height, sections=sections)


def _box(extents, center=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    """Axis-aligned box with the given (X, Y, Z) extents centred at ``center``."""
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m


# ---------------------------------------------------------------------------
# Public fastener builders
# ---------------------------------------------------------------------------


def make_m3_shcs(length_mm: float) -> trimesh.Trimesh:
    """M3 socket-head cap screw, ``length_mm`` shank length, head at
    z=[-CAP_H, 0], shank at z=[0, length_mm]."""
    # Cap body
    cap = _cyl(M3_CAP_D / 2.0, M3_CAP_H)
    cap.apply_translation([0.0, 0.0, -M3_CAP_H / 2.0])
    # Hex socket carved into the cap top (sitting flush with the cap
    # top face).  Top face is at z = 0.  Socket depth pushes DOWN
    # into the cap.  We just SUBTRACT a hex prism; boolean-diff is
    # slow but only runs once per spec, then we cache.
    socket = _hex_prism(M3_HEX_SOCKET_AF, M3_HEX_SOCKET_DEPTH + 0.4)
    socket.apply_translation([0.0, 0.0, -M3_HEX_SOCKET_DEPTH / 2.0 + 0.2])
    cap = cap.difference(socket)
    # Shank
    shank = _cyl(M3_THREAD_D / 2.0, length_mm)
    shank.apply_translation([0.0, 0.0, length_mm / 2.0])
    # Tiny chamfer at the shank tip would be nice, but the bounding-
    # cylinder verifier doesn't care.
    return trimesh.util.concatenate([cap, shank])


def make_m25_shcs(length_mm: float) -> trimesh.Trimesh:
    """M2.5 socket-head cap screw (e.g. the servo spline center screw)."""
    cap = _cyl(M25_CAP_D / 2.0, M25_CAP_H)
    cap.apply_translation([0.0, 0.0, -M25_CAP_H / 2.0])
    socket = _hex_prism(M25_HEX_SOCKET_AF, M25_HEX_SOCKET_DEPTH + 0.4)
    socket.apply_translation([0.0, 0.0, -M25_HEX_SOCKET_DEPTH / 2.0 + 0.2])
    cap = cap.difference(socket)
    shank = _cyl(M25_THREAD_D / 2.0, length_mm)
    shank.apply_translation([0.0, 0.0, length_mm / 2.0])
    return trimesh.util.concatenate([cap, shank])


def make_m3_pan_head(length_mm: float) -> trimesh.Trimesh:
    """M3 pan-head Phillips screw -- used as the foot hinge pin.

    Approximates the rounded pan head as a flat cylinder with a small
    domed cap on top, plus a Phillips cross slot carved into the top
    face.  Visually adequate for the inspector; the verifier treats
    the head as a bounding cylinder anyway.
    """
    # Pan body
    head = _cyl(M3_PAN_D / 2.0, M3_PAN_H)
    head.apply_translation([0.0, 0.0, -M3_PAN_H / 2.0])
    # Small rounded dome on top (sphere segment).  Place its centre at
    # z=-M3_PAN_H * 0.8 with radius ~ M3_PAN_D so only a sliver pokes
    # above z=0.  Skipped for parametric fallback to keep mesh small.
    # Carve Phillips cross slot into the top face.
    slot_a = _box(
        (M3_PAN_D * 0.85, M3_PAN_PHILLIPS_W, M3_PAN_PHILLIPS_DEPTH + 0.4),
        center=(0.0, 0.0, -M3_PAN_PHILLIPS_DEPTH / 2.0 + 0.2),
    )
    slot_b = _box(
        (M3_PAN_PHILLIPS_W, M3_PAN_D * 0.85, M3_PAN_PHILLIPS_DEPTH + 0.4),
        center=(0.0, 0.0, -M3_PAN_PHILLIPS_DEPTH / 2.0 + 0.2),
    )
    head = head.difference(slot_a)
    head = head.difference(slot_b)
    shank = _cyl(M3_THREAD_D / 2.0, length_mm)
    shank.apply_translation([0.0, 0.0, length_mm / 2.0])
    return trimesh.util.concatenate([head, shank])


def make_m3_nyloc_nut() -> trimesh.Trimesh:
    """M3 nylon-insert lock nut (DIN 985).

    Steel hex body sits at z in [-NUT_STEEL_H, 0] = the OUTBOARD face
    of the nut (the visible side).  Nylon ring at z in
    [-(NUT_STEEL_H + NUT_NYLON_H), -NUT_STEEL_H] sits AGAINST the
    wall.  +Z is the "through" direction for the bolt's shank.
    """
    # Steel hex body
    steel = _hex_prism(M3_NUT_AF, M3_NUT_STEEL_H)
    # Centre extruded prism so its top face = z=0, bottom = z=-STEEL_H
    steel.apply_translation([0.0, 0.0, -M3_NUT_STEEL_H / 2.0])
    # Nylon ring -- slightly inset so it visually reads as a different
    # band on the nut.  Diameter = NUT_AF * 0.85 to keep it tucked
    # inside the hex's inscribed circle.
    ring_d = M3_NUT_AF * 0.85
    nylon = _cyl(ring_d / 2.0, M3_NUT_NYLON_H)
    nylon.apply_translation([
        0.0, 0.0,
        -M3_NUT_STEEL_H - M3_NUT_NYLON_H / 2.0,
    ])
    # Central thread hole.  Carve through both pieces.
    bore = _cyl(M3_THREAD_D / 2.0, M3_NUT_H + 1.0)
    bore.apply_translation([0.0, 0.0, -M3_NUT_H / 2.0])
    steel = steel.difference(bore)
    nylon = nylon.difference(bore)
    return trimesh.util.concatenate([steel, nylon])


# ---------------------------------------------------------------------------
# Dispatcher (used by make_fastener_meshes.py)
# ---------------------------------------------------------------------------


def build_for_spec(spec: str) -> trimesh.Trimesh:
    """Return the parametric mesh for a given fastener spec label.

    Spec labels match the ``spec`` field on ``FastenerInstance``:
        "M3x14 SHCS"
        "M3x8  SHCS"
        "M3x32 SHCS"
        "M3x16 pan-head"
        "M2.5x8 spline screw"
        "M3 nyloc nut"
    """
    s = spec.replace(" ", "").lower()
    if s == "m3x14shcs":
        return make_m3_shcs(14.0)
    if s == "m3x8shcs":
        return make_m3_shcs(8.0)
    if s == "m3x32shcs":
        return make_m3_shcs(32.0)
    if s == "m3x16pan-head":
        return make_m3_pan_head(16.0)
    if s == "m2.5x8splinescrew":
        return make_m25_shcs(8.0)
    if s == "m3nylocnut":
        return make_m3_nyloc_nut()
    raise ValueError(f"unknown fastener spec: {spec!r}")
