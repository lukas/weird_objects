"""Library for the 5 Platonic, 13 Archimedean, and 13 Catalan solids.

Every Platonic & Archimedean solid is generated from explicit canonical
Cartesian coordinates (no fitting, no sampling).  Catalan solids are
computed as the *polar reciprocation* of their Archimedean parent: for
each polygon face with outward unit normal ``n`` at signed distance
``d`` from the origin, the dual vertex is ``n / d``.  This produces
exact canonical Catalan coordinates on a common reciprocation surface,
so dual faces are perfectly planar.

Public surface:

    - ``ALL_SOLIDS`` — list of :class:`Solid` (5 + 13 + 13 = 31)
    - ``PLATONIC``, ``ARCHIMEDEAN``, ``CATALAN`` — sub-lists
    - ``generate_mesh(solid)`` — :class:`trimesh.Trimesh` convex hull
    - ``generate_wireframe_data(solid)`` — ``(vertices, edges)``
    - ``polar_dual(mesh)`` — exact polar dual as a Trimesh
    - ``real_polyhedron_edges(mesh)`` — edges with triangulation diagonals filtered out
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
import trimesh


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PHI = (1.0 + np.sqrt(5.0)) / 2.0  # golden ratio


def _real_root(coeffs):
    """Return the unique real root of a cubic with one real root."""
    rs = np.roots(coeffs)
    return float(np.real(rs[np.argmin(np.abs(np.imag(rs)))]))


# tribonacci constant — real root of x^3 - x^2 - x - 1 = 0  (snub cube)
TAU = _real_root([1.0, -1.0, -1.0, -1.0])

# snub-dodecahedron constant — real root of x^3 - 2x - phi = 0
XI = _real_root([1.0, 0.0, -2.0, -PHI])


# ---------------------------------------------------------------------------
# Permutation / sign helpers
# ---------------------------------------------------------------------------


def _cyclic_perms(t):
    a, b, c = t
    return [(a, b, c), (b, c, a), (c, a, b)]


def _antic_perms(t):
    a, b, c = t
    return [(a, c, b), (b, a, c), (c, b, a)]


def _all_perms(t):
    a, b, c = t
    return [(a, b, c), (a, c, b), (b, a, c), (b, c, a), (c, a, b), (c, b, a)]


def _signed_variants(t):
    """All sign combos, deduping when a coordinate is zero."""
    out = set()
    for s0 in (-1, 1):
        if s0 == -1 and abs(t[0]) < 1e-9:
            continue
        for s1 in (-1, 1):
            if s1 == -1 and abs(t[1]) < 1e-9:
                continue
            for s2 in (-1, 1):
                if s2 == -1 and abs(t[2]) < 1e-9:
                    continue
                out.add((s0 * t[0], s1 * t[1], s2 * t[2]))
    return list(out)


def _vset(templates, perm_fn=_cyclic_perms):
    """Build a vertex set from coordinate templates with all-sign × given perms."""
    pts = set()
    for t in templates:
        for signed in _signed_variants(t):
            for perm in perm_fn(signed):
                pts.add(tuple(round(c, 9) for c in perm))
    return np.array([list(p) for p in pts])


def _count_minus(perm, signs):
    n = 0
    for s, p in zip(signs, perm):
        if s == -1 and abs(p) > 1e-9:
            n += 1
    return n


def _vset_with_sign_predicate(templates, perm_fn, sign_predicate):
    """Like ``_vset`` but only emit vertices whose sign pattern passes the predicate.

    ``sign_predicate(n_minus, perm)`` -> bool.
    """
    pts = set()
    for t in templates:
        for perm in perm_fn(t):
            for s0 in (-1, 1):
                if s0 == -1 and abs(perm[0]) < 1e-9:
                    continue
                for s1 in (-1, 1):
                    if s1 == -1 and abs(perm[1]) < 1e-9:
                        continue
                    for s2 in (-1, 1):
                        if s2 == -1 and abs(perm[2]) < 1e-9:
                            continue
                        nm = _count_minus(perm, (s0, s1, s2))
                        if not sign_predicate(nm, perm):
                            continue
                        pts.add(tuple(round(s * p, 9) for s, p in zip((s0, s1, s2), perm)))
    return np.array([list(p) for p in pts])


# ---------------------------------------------------------------------------
# Platonic solids
# ---------------------------------------------------------------------------


def tetrahedron():
    """4 V, 6 E, 4 F."""
    return np.array(
        [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]],
        dtype=float,
    )


def cube():
    """8 V, 12 E, 6 F."""
    return _vset([(1, 1, 1)], perm_fn=lambda t: [t])


def octahedron():
    """6 V, 12 E, 8 F."""
    return _vset([(1, 0, 0)], perm_fn=_all_perms)


def dodecahedron():
    """20 V, 30 E, 12 F.

    Standard form: cube vertices (±1, ±1, ±1) plus three rectangles
    (0, ±1/φ, ±φ) (cyclic).
    """
    cube_v = _vset([(1, 1, 1)], perm_fn=lambda t: [t])
    rects = _vset([(0, 1.0 / PHI, PHI)], perm_fn=_cyclic_perms)
    return np.vstack([cube_v, rects])


def icosahedron():
    """12 V, 30 E, 20 F.  Three orthogonal golden rectangles (0, ±1, ±φ)."""
    return _vset([(0, 1.0, PHI)], perm_fn=_cyclic_perms)


# ---------------------------------------------------------------------------
# Archimedean solids — tetrahedral family
# ---------------------------------------------------------------------------


def truncated_tetrahedron():
    """12 V, 18 E, 8 F (4 triangles + 4 hexagons).

    Cyclic permutations of (±3, ±1, ±1) with an even number of minus
    signs.  These are the 12 truncation points obtained by clipping each
    vertex of the tetrahedron at (±1, ±1, ±1) (with even-parity sign
    pattern) one third of the way along each edge.
    """
    return _vset_with_sign_predicate(
        [(3.0, 1.0, 1.0)],
        perm_fn=_cyclic_perms,
        sign_predicate=lambda nm, _perm: nm % 2 == 0,
    )


# ---------------------------------------------------------------------------
# Archimedean solids — octahedral / cubic family
# ---------------------------------------------------------------------------


def cuboctahedron():
    """12 V, 24 E, 14 F.  All permutations of (±1, ±1, 0)."""
    return _vset([(1.0, 1.0, 0.0)], perm_fn=_all_perms)


def truncated_cube():
    """24 V, 36 E, 14 F.  All permutations of (±ξ, ±1, ±1) with ξ = √2 − 1."""
    xi = np.sqrt(2.0) - 1.0
    return _vset([(xi, 1.0, 1.0)], perm_fn=_all_perms)


def truncated_octahedron():
    """24 V, 36 E, 14 F.  All permutations of (0, ±1, ±2)."""
    return _vset([(0.0, 1.0, 2.0)], perm_fn=_all_perms)


def rhombicuboctahedron():
    """24 V, 48 E, 26 F.  All permutations of (±1, ±1, ±(1+√2))."""
    return _vset([(1.0, 1.0, 1.0 + np.sqrt(2.0))], perm_fn=_all_perms)


def truncated_cuboctahedron():
    """48 V, 72 E, 26 F.  All permutations of (±1, ±(1+√2), ±(1+2√2))."""
    return _vset(
        [(1.0, 1.0 + np.sqrt(2.0), 1.0 + 2.0 * np.sqrt(2.0))],
        perm_fn=_all_perms,
    )


def snub_cube():
    """24 V, 60 E, 38 F (chiral).

    Even (cyclic) permutations of (±1, ±1/τ, ±τ) with even # plus signs,
    plus odd (anticyclic) permutations with odd # plus signs, where τ is
    the tribonacci constant.  Picks one chirality.
    """
    template = (1.0, 1.0 / TAU, TAU)
    pts = set()
    for s0 in (-1, 1):
        for s1 in (-1, 1):
            for s2 in (-1, 1):
                signed = (s0 * template[0], s1 * template[1], s2 * template[2])
                # ``s0*s1*s2 < 0`` <=> even # plus signs (out of 3)
                if s0 * s1 * s2 < 0:
                    for cyc in _cyclic_perms(signed):
                        pts.add(tuple(round(c, 9) for c in cyc))
                else:
                    for ac in _antic_perms(signed):
                        pts.add(tuple(round(c, 9) for c in ac))
    return np.array([list(p) for p in pts])


# ---------------------------------------------------------------------------
# Archimedean solids — icosahedral family
# ---------------------------------------------------------------------------


def icosidodecahedron():
    """30 V, 60 E, 32 F.  Edge midpoints of the icosahedron.

    Equivalent canonical form: cyclic permutations of (0, 0, ±φ) and
    (±1/2, ±φ/2, ±φ²/2).
    """
    a = _vset([(0.0, 0.0, PHI)], perm_fn=_cyclic_perms)
    b = _vset([(0.5, PHI / 2.0, PHI ** 2 / 2.0)], perm_fn=_cyclic_perms)
    return np.vstack([a, b])


def truncated_dodecahedron():
    """60 V, 90 E, 32 F (12 decagons + 20 triangles)."""
    return _vset(
        [
            (0.0, 1.0 / PHI, 2.0 + PHI),
            (1.0 / PHI, PHI, 2.0 * PHI),
            (PHI, 2.0, PHI + 1.0),
        ],
        perm_fn=_cyclic_perms,
    )


def truncated_icosahedron():
    """60 V, 90 E, 32 F (12 pentagons + 20 hexagons — soccer ball)."""
    return _vset(
        [
            (0.0, 1.0, 3.0 * PHI),
            (1.0, 2.0 + PHI, 2.0 * PHI),
            (PHI, 2.0, 2.0 * PHI + 1.0),
        ],
        perm_fn=_cyclic_perms,
    )


def rhombicosidodecahedron():
    """60 V, 120 E, 62 F."""
    return _vset(
        [
            (1.0, 1.0, PHI ** 3),
            (PHI ** 2, PHI, 2.0 * PHI),
            (PHI + 2.0, 0.0, PHI ** 2),
        ],
        perm_fn=_cyclic_perms,
    )


def truncated_icosidodecahedron():
    """120 V, 180 E, 62 F."""
    return _vset(
        [
            (1.0 / PHI, 1.0 / PHI, 3.0 + PHI),
            (2.0 / PHI, PHI, 1.0 + 2.0 * PHI),
            (1.0 / PHI, PHI ** 2, 3.0 * PHI - 1.0),
            (2.0 * PHI - 1.0, 2.0, 2.0 + PHI),
            (PHI, 3.0, 2.0 * PHI),
        ],
        perm_fn=_cyclic_perms,
    )


def snub_dodecahedron():
    """60 V, 150 E, 92 F (chiral).

    Even (cyclic) permutations of 5 templates with even # plus signs,
    where ``α = ξ - 1/ξ`` and ``β = ξφ + φ² + φ/ξ``.
    Picks one chirality.
    """
    a = XI - 1.0 / XI
    b = XI * PHI + PHI ** 2 + PHI / XI
    templates = [
        (2.0 * a, 2.0, 2.0 * b),
        (a + b / PHI + PHI, -a * PHI + b + 1.0 / PHI, a / PHI + b * PHI - 1.0),
        (a + b / PHI - PHI, a * PHI - b + 1.0 / PHI, a / PHI + b * PHI + 1.0),
        (-a / PHI + b * PHI + 1.0, -a + b / PHI - PHI, a * PHI + b - 1.0 / PHI),
        (-a / PHI + b * PHI - 1.0, a - b / PHI - PHI, a * PHI + b + 1.0 / PHI),
    ]
    pts = set()
    for t in templates:
        for s0 in (-1, 1):
            for s1 in (-1, 1):
                for s2 in (-1, 1):
                    if s0 * s1 * s2 < 0:  # even # plus signs
                        signed = (s0 * t[0], s1 * t[1], s2 * t[2])
                        for cyc in _cyclic_perms(signed):
                            pts.add(tuple(round(c, 9) for c in cyc))
    return np.array([list(p) for p in pts])


# ---------------------------------------------------------------------------
# Edge filtering & polar duality
# ---------------------------------------------------------------------------


def real_polyhedron_edges(mesh, planarity_tol=1e-3):
    """Edges of a (triangulated) convex polyhedron, with triangulation
    diagonals filtered out.

    A diagonal across a single planar polygon face shows up as a shared
    triangle edge whose two adjacent triangles are coplanar.  We detect
    that via the dot of their normals being close to 1.
    """
    edges = []
    for (f1, f2), edge_verts in zip(mesh.face_adjacency, mesh.face_adjacency_edges):
        n1 = mesh.face_normals[f1]
        n2 = mesh.face_normals[f2]
        if float(np.dot(n1, n2)) < 1.0 - planarity_tol:
            a, b = sorted((int(edge_verts[0]), int(edge_verts[1])))
            edges.append((a, b))
    return sorted(set(edges))


def polar_dual(mesh):
    """Exact polar dual of a convex polyhedron centered at the origin.

    For each polygon face with outward unit normal ``n`` at signed
    distance ``d`` from the origin, the dual vertex is ``n / d``.
    Coplanar adjacent triangles are grouped via :pyattr:`mesh.facets`
    so each true polygon face contributes exactly one dual vertex.
    """
    centroid = mesh.vertices.mean(axis=0)
    verts = mesh.vertices - centroid
    centers = []
    used = set()
    for facet in mesh.facets:
        idx = np.asarray(facet)
        n = mesh.face_normals[idx[0]]
        v0 = verts[mesh.faces[idx[0]][0]]
        d = float(np.dot(n, v0))
        if abs(d) < 1e-12:
            continue
        centers.append(n / d)
        used.update(int(i) for i in idx)
    for tri_idx in range(len(mesh.faces)):
        if tri_idx in used:
            continue
        n = mesh.face_normals[tri_idx]
        v0 = verts[mesh.faces[tri_idx][0]]
        d = float(np.dot(n, v0))
        if abs(d) < 1e-12:
            continue
        centers.append(n / d)
    return trimesh.convex.convex_hull(np.array(centers))


# ---------------------------------------------------------------------------
# Solid registry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Solid:
    name: str
    category: str  # "platonic" | "archimedean" | "catalan"
    V: int
    E: int
    F: int
    parent: Optional[str] = None  # for catalans: name of dual's archimedean parent
    chiral: bool = False


PLATONIC = [
    Solid("Tetrahedron",  "platonic",  4,  6,  4),
    Solid("Cube",         "platonic",  8, 12,  6),
    Solid("Octahedron",   "platonic",  6, 12,  8),
    Solid("Dodecahedron", "platonic", 20, 30, 12),
    Solid("Icosahedron",  "platonic", 12, 30, 20),
]

ARCHIMEDEAN = [
    Solid("Truncated Tetrahedron",     "archimedean",  12,  18,  8),
    Solid("Cuboctahedron",             "archimedean",  12,  24, 14),
    Solid("Truncated Cube",            "archimedean",  24,  36, 14),
    Solid("Truncated Octahedron",      "archimedean",  24,  36, 14),
    Solid("Rhombicuboctahedron",       "archimedean",  24,  48, 26),
    Solid("Truncated Cuboctahedron",   "archimedean",  48,  72, 26),
    Solid("Snub Cube",                 "archimedean",  24,  60, 38, chiral=True),
    Solid("Icosidodecahedron",         "archimedean",  30,  60, 32),
    Solid("Truncated Dodecahedron",    "archimedean",  60,  90, 32),
    Solid("Truncated Icosahedron",     "archimedean",  60,  90, 32),
    Solid("Rhombicosidodecahedron",    "archimedean",  60, 120, 62),
    Solid("Truncated Icosidodecahedron","archimedean",120, 180, 62),
    Solid("Snub Dodecahedron",         "archimedean",  60, 150, 92, chiral=True),
]

CATALAN = [
    Solid("Triakis Tetrahedron",         "catalan",  8,  18, 12, parent="Truncated Tetrahedron"),
    Solid("Rhombic Dodecahedron",        "catalan", 14,  24, 12, parent="Cuboctahedron"),
    Solid("Triakis Octahedron",          "catalan", 14,  36, 24, parent="Truncated Cube"),
    Solid("Tetrakis Hexahedron",         "catalan", 14,  36, 24, parent="Truncated Octahedron"),
    Solid("Deltoidal Icositetrahedron",  "catalan", 26,  48, 24, parent="Rhombicuboctahedron"),
    Solid("Disdyakis Dodecahedron",      "catalan", 26,  72, 48, parent="Truncated Cuboctahedron"),
    Solid("Pentagonal Icositetrahedron", "catalan", 38,  60, 24, parent="Snub Cube", chiral=True),
    Solid("Rhombic Triacontahedron",     "catalan", 32,  60, 30, parent="Icosidodecahedron"),
    Solid("Triakis Icosahedron",         "catalan", 32,  90, 60, parent="Truncated Dodecahedron"),
    Solid("Pentakis Dodecahedron",       "catalan", 32,  90, 60, parent="Truncated Icosahedron"),
    Solid("Deltoidal Hexecontahedron",   "catalan", 62, 120, 60, parent="Rhombicosidodecahedron"),
    Solid("Disdyakis Triacontahedron",   "catalan", 62, 180, 120, parent="Truncated Icosidodecahedron"),
    Solid("Pentagonal Hexecontahedron",  "catalan", 92, 150, 60, parent="Snub Dodecahedron", chiral=True),
]

ALL_SOLIDS = PLATONIC + ARCHIMEDEAN + CATALAN

_SOLID_BY_NAME = {s.name: s for s in ALL_SOLIDS}


_GENERATORS = {
    "Tetrahedron":                tetrahedron,
    "Cube":                       cube,
    "Octahedron":                 octahedron,
    "Dodecahedron":               dodecahedron,
    "Icosahedron":                icosahedron,
    "Truncated Tetrahedron":      truncated_tetrahedron,
    "Cuboctahedron":              cuboctahedron,
    "Truncated Cube":             truncated_cube,
    "Truncated Octahedron":       truncated_octahedron,
    "Rhombicuboctahedron":        rhombicuboctahedron,
    "Truncated Cuboctahedron":    truncated_cuboctahedron,
    "Snub Cube":                  snub_cube,
    "Icosidodecahedron":          icosidodecahedron,
    "Truncated Dodecahedron":     truncated_dodecahedron,
    "Truncated Icosahedron":      truncated_icosahedron,
    "Rhombicosidodecahedron":     rhombicosidodecahedron,
    "Truncated Icosidodecahedron":truncated_icosidodecahedron,
    "Snub Dodecahedron":          snub_dodecahedron,
}


# ---------------------------------------------------------------------------
# Mesh & wireframe construction
# ---------------------------------------------------------------------------


def _mesh_cache_clear():
    _mesh_cache.clear()


_mesh_cache: dict = {}


def generate_mesh(solid):
    """Return the convex-hull :class:`trimesh.Trimesh` for the given solid.

    Catalans are computed by polar reciprocation of their Archimedean parent.
    Results are memoised.
    """
    if isinstance(solid, str):
        solid = _SOLID_BY_NAME[solid]
    if solid.name in _mesh_cache:
        return _mesh_cache[solid.name]
    if solid.category == "catalan":
        parent = _SOLID_BY_NAME[solid.parent]
        parent_mesh = generate_mesh(parent)
        mesh = polar_dual(parent_mesh)
    else:
        verts = _GENERATORS[solid.name]()
        mesh = trimesh.convex.convex_hull(verts)
    _mesh_cache[solid.name] = mesh
    return mesh


def generate_wireframe_data(solid):
    """Return ``(vertices, edges)`` for use with cylinder-wireframe rendering."""
    mesh = generate_mesh(solid)
    return np.asarray(mesh.vertices), real_polyhedron_edges(mesh)


def face_count(mesh):
    """Number of polygon (non-triangulated) faces of a triangulated convex mesh."""
    n_facets = len(mesh.facets)
    used = set()
    for facet in mesh.facets:
        for tri in facet:
            used.add(int(tri))
    n_unfaceted_tris = len(mesh.faces) - len(used)
    return n_facets + n_unfaceted_tris
