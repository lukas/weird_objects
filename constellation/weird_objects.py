"""14-node polyhedral constellation — all distinct convex solids of the
icosahedral family (2 Platonic + 6 Archimedean + 6 Catalan).

This script is a thin layout/rendering wrapper.  Every polyhedron, its
edges, and its dual all come from :mod:`polyhedra`, which is exhaustively
unit-tested in ``test_polyhedra.py``.
"""

from __future__ import annotations

import numpy as np
import trimesh

import polyhedra as P


# ---------------------------------------------------------------------------
# Render parameters
# ---------------------------------------------------------------------------

OUTFILE = "polyhedron_constellation_14_node.stl"

EDGE_RADIUS = 1.0   # mm — wireframe rod (>= 1 mm is safe for most metal printing)
NODE_SCALE  = 18.0  # outer radius of each polyhedron (mm)


# ---------------------------------------------------------------------------
# Wireframe helpers
# ---------------------------------------------------------------------------


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v if n == 0 else v / n


def _cylinder_between(p1, p2, radius, sections=16):
    p1, p2 = np.asarray(p1), np.asarray(p2)
    direction = p2 - p1
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return None
    cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    cyl.apply_translation([0, 0, length / 2.0])
    z = np.array([0.0, 0.0, 1.0])
    T = trimesh.geometry.align_vectors(z, _unit(direction))
    cyl.apply_transform(T)
    cyl.apply_translation(p1)
    return cyl


def make_wire_solid(solid_name, center, scale=NODE_SCALE,
                    edge_radius=EDGE_RADIUS):
    """Build a wireframe of the named solid centered at ``center`` with
    outer radius ``scale``."""
    verts, edges = P.generate_wireframe_data(solid_name)
    verts = np.asarray(verts, dtype=float)
    verts = verts - verts.mean(axis=0)
    radius = np.linalg.norm(verts, axis=1).max()
    verts = verts * (scale / radius) + np.asarray(center, dtype=float)

    parts = []
    for i, j in edges:
        cyl = _cylinder_between(verts[i], verts[j], edge_radius)
        if cyl is not None:
            parts.append(cyl)
    for v in verts:
        s = trimesh.creation.icosphere(subdivisions=1, radius=edge_radius * 1.35)
        s.apply_translation(v)
        parts.append(s)
    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
# Constellation layout — all 14 distinct icosahedral-family convex solids
# (2 Platonic + 6 Archimedean + 6 Catalan).
# ---------------------------------------------------------------------------

# (display name, layout center mm, per-solid scale multiplier)
LAYOUT = [
    # Centerpiece: the icosidodecahedron is the natural "hub" of the family.
    ("Icosidodecahedron",          [  0,   0,   0], 1.25),
    # Y-axis: the two Platonic icosahedral solids.
    ("Icosahedron",                [  0,  72,   0], 1.00),
    ("Dodecahedron",               [  0, -72,   0], 1.00),
    # X-axis: the two truncated Platonics.
    ("Truncated Icosahedron",      [ 72,   0,   0], 1.00),
    ("Truncated Dodecahedron",     [-72,   0,   0], 1.00),
    # Upper diagonals: the remaining Archimedeans + one Catalan.
    ("Snub Dodecahedron",          [ 48,  48,  34], 0.95),
    ("Rhombicosidodecahedron",     [-48,  48,  34], 0.95),
    ("Truncated Icosidodecahedron",[ 48, -48,  34], 1.05),
    ("Rhombic Triacontahedron",    [-48, -48,  34], 0.90),
    # Lower diagonals: four Catalan duals.
    ("Pentakis Dodecahedron",      [ 48,  48, -34], 0.90),
    ("Triakis Icosahedron",        [-48,  48, -34], 0.90),
    ("Deltoidal Hexecontahedron",  [ 48, -48, -34], 0.90),
    ("Disdyakis Triacontahedron",  [-48, -48, -34], 1.00),
    # Top spire: the chiral Catalan dual.
    ("Pentagonal Hexecontahedron", [  0,   0,  68], 0.95),
]


def main():
    parts = []
    for name, center, scale_factor in LAYOUT:
        print(f"Adding {name}")
        parts.append(make_wire_solid(name, center, scale=NODE_SCALE * scale_factor))

    combined = trimesh.util.concatenate(parts)
    print(f"Faces: {len(combined.faces)}")
    print(f"Watertight: {combined.is_watertight}")
    print(f"Bounds mm: {combined.bounds}")
    combined.export(OUTFILE)
    print(f"Saved {OUTFILE}")


if __name__ == "__main__":
    main()
