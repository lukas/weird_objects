"""Pytest tests verifying every polyhedron generator in :mod:`polyhedra`.

Run with::

    pytest -q test_polyhedra.py
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import polyhedra as P


# ---------------------------------------------------------------------------
# Test fixtures: parametrise across every solid in the registry
# ---------------------------------------------------------------------------


def _all_solid_ids():
    return [s.name for s in P.ALL_SOLIDS]


def _solid(name: str) -> P.Solid:
    return next(s for s in P.ALL_SOLIDS if s.name == name)


# ---------------------------------------------------------------------------
# Registry-level invariants
# ---------------------------------------------------------------------------


def test_registry_counts():
    assert len(P.PLATONIC) == 5
    assert len(P.ARCHIMEDEAN) == 13
    assert len(P.CATALAN) == 13
    assert len(P.ALL_SOLIDS) == 31


def test_registry_unique_names():
    names = [s.name for s in P.ALL_SOLIDS]
    assert len(set(names)) == len(names), "duplicate solid name in registry"


def test_registry_categories():
    for s in P.PLATONIC:
        assert s.category == "platonic"
    for s in P.ARCHIMEDEAN:
        assert s.category == "archimedean"
    for s in P.CATALAN:
        assert s.category == "catalan"


def test_catalan_parents_are_archimedean():
    """Every Catalan must list a real Archimedean as its dual parent."""
    arch_names = {s.name for s in P.ARCHIMEDEAN}
    for s in P.CATALAN:
        assert s.parent is not None, f"{s.name} missing parent"
        assert s.parent in arch_names, f"{s.name} parent {s.parent!r} not an Archimedean"


def test_catalan_parent_pairing_unique():
    """Each Archimedean appears as the parent of *exactly one* Catalan."""
    parents = [s.parent for s in P.CATALAN]
    assert len(parents) == len(set(parents)), "two Catalans share a parent"
    assert set(parents) == {s.name for s in P.ARCHIMEDEAN}


def test_chiral_pairing():
    """The chiral Archimedeans (snub cube/dodecahedron) must have chiral Catalan duals."""
    chiral_arch = {s.name for s in P.ARCHIMEDEAN if s.chiral}
    assert chiral_arch == {"Snub Cube", "Snub Dodecahedron"}
    for s in P.CATALAN:
        if s.parent in chiral_arch:
            assert s.chiral, f"{s.name} should be chiral (dual of {s.parent})"
        else:
            assert not s.chiral, f"{s.name} should NOT be chiral"


def test_dual_VE_F_swap():
    """For dual pairs V_a = F_b and F_a = V_b and E_a = E_b."""
    for c in P.CATALAN:
        a = _solid(c.parent)
        assert c.V == a.F, f"{c.name}.V != {a.name}.F"
        assert c.F == a.V, f"{c.name}.F != {a.name}.V"
        assert c.E == a.E, f"{c.name}.E != {a.name}.E"


# ---------------------------------------------------------------------------
# Per-solid geometric invariants
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", _all_solid_ids())
def test_vertex_count(name):
    solid = _solid(name)
    verts, _edges = P.generate_wireframe_data(solid)
    assert len(verts) == solid.V, (
        f"{name}: got {len(verts)} vertices, expected {solid.V}"
    )


@pytest.mark.parametrize("name", _all_solid_ids())
def test_edge_count(name):
    solid = _solid(name)
    _verts, edges = P.generate_wireframe_data(solid)
    assert len(edges) == solid.E, (
        f"{name}: got {len(edges)} edges, expected {solid.E}"
    )


@pytest.mark.parametrize("name", _all_solid_ids())
def test_face_count(name):
    solid = _solid(name)
    mesh = P.generate_mesh(solid)
    fc = P.face_count(mesh)
    assert fc == solid.F, (
        f"{name}: got {fc} faces, expected {solid.F}"
    )


@pytest.mark.parametrize("name", _all_solid_ids())
def test_no_floating_vertices(name):
    solid = _solid(name)
    verts, edges = P.generate_wireframe_data(solid)
    used = set()
    for i, j in edges:
        used.add(int(i))
        used.add(int(j))
    floating = len(verts) - len(used)
    assert floating == 0, f"{name}: {floating} floating vertices"


@pytest.mark.parametrize("name", _all_solid_ids())
def test_euler_characteristic(name):
    """Every convex polyhedron must satisfy V - E + F = 2."""
    solid = _solid(name)
    verts, edges = P.generate_wireframe_data(solid)
    mesh = P.generate_mesh(solid)
    f = P.face_count(mesh)
    v, e = len(verts), len(edges)
    assert v - e + f == 2, (
        f"{name}: V-E+F = {v}-{e}+{f} = {v - e + f}, expected 2"
    )


@pytest.mark.parametrize("name", _all_solid_ids())
def test_mesh_is_convex(name):
    """Convex hull of the vertex set should equal the mesh itself (idempotent)."""
    solid = _solid(name)
    import trimesh

    mesh = P.generate_mesh(solid)
    rehull = trimesh.convex.convex_hull(np.asarray(mesh.vertices))
    assert len(rehull.vertices) == len(mesh.vertices), (
        f"{name}: vertex set is not the convex hull (interior points present)"
    )


@pytest.mark.parametrize("name", _all_solid_ids())
def test_mesh_is_watertight(name):
    """All convex polyhedra are closed surfaces."""
    solid = _solid(name)
    mesh = P.generate_mesh(solid)
    assert mesh.is_watertight, f"{name}: mesh is not watertight"


@pytest.mark.parametrize("name", _all_solid_ids())
def test_mesh_centered_at_origin(name):
    """Canonical Cartesian forms put the centroid at the origin."""
    solid = _solid(name)
    mesh = P.generate_mesh(solid)
    c = np.asarray(mesh.vertices).mean(axis=0)
    assert np.linalg.norm(c) < 1e-6, f"{name}: centroid offset {c}"


# ---------------------------------------------------------------------------
# Vertex-transitivity for Platonic & Archimedean (uniform vertex figure)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name", [s.name for s in P.PLATONIC] + [s.name for s in P.ARCHIMEDEAN]
)
def test_vertex_transitive(name):
    """Platonic & Archimedean solids are vertex-transitive: all vertices the
    same distance from the origin."""
    solid = _solid(name)
    verts, _ = P.generate_wireframe_data(solid)
    radii = np.linalg.norm(verts, axis=1)
    assert radii.max() - radii.min() < 1e-6, (
        f"{name}: vertex radii vary by {radii.max() - radii.min():g}"
    )


# ---------------------------------------------------------------------------
# Face-transitivity for Catalan (uniform inscribed-sphere)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", [s.name for s in P.CATALAN])
def test_catalan_face_transitive(name):
    """Catalan solids are face-transitive: all polygon-face planes the same
    distance from the origin (so the inscribed sphere touches every face)."""
    solid = _solid(name)
    mesh = P.generate_mesh(solid)
    centroid = np.asarray(mesh.vertices).mean(axis=0)
    verts = np.asarray(mesh.vertices) - centroid
    distances = []
    used = set()
    for facet in mesh.facets:
        idx = np.asarray(facet)
        n = mesh.face_normals[idx[0]]
        v0 = verts[mesh.faces[idx[0]][0]]
        distances.append(abs(float(np.dot(n, v0))))
        used.update(int(i) for i in idx)
    for tri_idx in range(len(mesh.faces)):
        if tri_idx in used:
            continue
        n = mesh.face_normals[tri_idx]
        v0 = verts[mesh.faces[tri_idx][0]]
        distances.append(abs(float(np.dot(n, v0))))
    distances = np.asarray(distances)
    assert distances.max() - distances.min() < 1e-6, (
        f"{name}: face-plane distances vary by {distances.max() - distances.min():g}"
    )


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


def test_phi_value():
    assert math.isclose(P.PHI, (1 + math.sqrt(5)) / 2, abs_tol=1e-12)


def test_tau_value():
    """Tribonacci constant: x^3 - x^2 - x - 1 = 0, ≈ 1.83929."""
    assert math.isclose(P.TAU, 1.8392867552141612, abs_tol=1e-9)
    assert math.isclose(P.TAU ** 3 - P.TAU ** 2 - P.TAU - 1.0, 0.0, abs_tol=1e-9)


def test_xi_value():
    """Snub-dodecahedron constant: x^3 - 2x - phi = 0, ≈ 1.71556."""
    assert math.isclose(P.XI, 1.7155614996973291, abs_tol=1e-9)
    assert math.isclose(P.XI ** 3 - 2.0 * P.XI - P.PHI, 0.0, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# Face-shape tests — catch wrong-geometry-but-right-counts bugs.
#
# A truncated tetrahedron and a hexagonal prism both have V=12, E=18, F=8
# (and even uniform vertex-radius), so V/E/F counts alone don't prove a
# generator is correct.  These tests assert the *face shapes* themselves.
# ---------------------------------------------------------------------------


def _face_polygons(mesh):
    """Reconstruct the polygon face vertex-loops from a triangulated convex
    mesh by stitching together triangles in each :pyattr:`mesh.facets` group
    and emitting un-faceted triangles as 3-cycles."""
    used = set()
    polys = []
    for facet in mesh.facets:
        edge_count = {}
        for ti in facet:
            tri = mesh.faces[ti]
            for a, b in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]:
                k = tuple(sorted((int(a), int(b))))
                edge_count[k] = edge_count.get(k, 0) + 1
            used.add(int(ti))
        boundary = [k for k, c in edge_count.items() if c == 1]
        if not boundary:
            continue
        loop = list(boundary[0])
        edges = {k: list(k) for k in boundary[1:]}
        while edges:
            tail = loop[-1]
            for k, ab in list(edges.items()):
                if tail in ab:
                    nxt = ab[1] if ab[0] == tail else ab[0]
                    loop.append(nxt)
                    del edges[k]
                    break
            else:
                break
        if loop and loop[0] == loop[-1]:
            loop = loop[:-1]
        polys.append(loop)
    for ti in range(len(mesh.faces)):
        if ti in used:
            continue
        polys.append([int(v) for v in mesh.faces[ti]])
    return polys


def _face_side_count_histogram(polys):
    hist = {}
    for poly in polys:
        n = len(poly)
        hist[n] = hist.get(n, 0) + 1
    return hist


# Canonical face-shape histograms.  Source: Wikipedia / Wolfram MathWorld.
_ARCHIMEDEAN_FACES = {
    "Truncated Tetrahedron":      {3: 4, 6: 4},
    "Cuboctahedron":              {3: 8, 4: 6},
    "Truncated Cube":             {3: 8, 8: 6},
    "Truncated Octahedron":       {4: 6, 6: 8},
    "Rhombicuboctahedron":        {3: 8, 4: 18},
    "Truncated Cuboctahedron":    {4: 12, 6: 8, 8: 6},
    "Snub Cube":                  {3: 32, 4: 6},
    "Icosidodecahedron":          {3: 20, 5: 12},
    "Truncated Dodecahedron":     {3: 20, 10: 12},
    "Truncated Icosahedron":      {5: 12, 6: 20},
    "Rhombicosidodecahedron":     {3: 20, 4: 30, 5: 12},
    "Truncated Icosidodecahedron":{4: 30, 6: 20, 10: 12},
    "Snub Dodecahedron":          {3: 80, 5: 12},
}

# Platonic face shapes are all-one-kind.
_PLATONIC_FACES = {
    "Tetrahedron":  {3: 4},
    "Cube":         {4: 6},
    "Octahedron":   {3: 8},
    "Dodecahedron": {5: 12},
    "Icosahedron":  {3: 20},
}


@pytest.mark.parametrize("name", list(_PLATONIC_FACES.keys()))
def test_platonic_face_histogram(name):
    """Each Platonic solid has the canonical face shape histogram."""
    mesh = P.generate_mesh(name)
    polys = _face_polygons(mesh)
    assert _face_side_count_histogram(polys) == _PLATONIC_FACES[name]


@pytest.mark.parametrize("name", list(_ARCHIMEDEAN_FACES.keys()))
def test_archimedean_face_histogram(name):
    """Each Archimedean solid has the canonical mix of regular polygon faces.

    This catches imposters with the same V/E/F counts but different face
    shapes (e.g. a hexagonal prism masquerading as a truncated tetrahedron)."""
    mesh = P.generate_mesh(name)
    polys = _face_polygons(mesh)
    assert _face_side_count_histogram(polys) == _ARCHIMEDEAN_FACES[name], (
        f"{name}: got {_face_side_count_histogram(polys)}, "
        f"expected {_ARCHIMEDEAN_FACES[name]}"
    )


@pytest.mark.parametrize("name", [s.name for s in P.PLATONIC] + list(_ARCHIMEDEAN_FACES.keys()))
def test_all_edges_uniform_length(name):
    """Platonic + Archimedean solids must have all edges the same length —
    a necessary condition for all faces to be regular polygons."""
    solid = _solid(name)
    verts, edges = P.generate_wireframe_data(solid)
    lens = [float(np.linalg.norm(verts[i] - verts[j])) for i, j in edges]
    lens = np.asarray(lens)
    spread = lens.max() - lens.min()
    rel = spread / lens.mean()
    assert rel < 1e-6, (
        f"{name}: edge lengths span {lens.min():.6f} — {lens.max():.6f} "
        f"(Δ={spread:.6g})"
    )


@pytest.mark.parametrize("name", [s.name for s in P.PLATONIC] + list(_ARCHIMEDEAN_FACES.keys()))
def test_all_faces_regular(name):
    """Each face is a *regular* polygon — equal-length edges and all
    vertices equidistant from the face centroid (cyclic equilateral =
    regular for n ≥ 3)."""
    mesh = P.generate_mesh(name)
    polys = _face_polygons(mesh)
    for poly in polys:
        verts = mesh.vertices[poly]
        # Equilateral: all sides same length.
        sides = np.linalg.norm(verts - np.roll(verts, -1, axis=0), axis=1)
        assert sides.max() - sides.min() < 1e-6, (
            f"{name}: face {poly} not equilateral, sides {sides}"
        )
        # Cyclic: all vertices equidistant from face centroid.
        center = verts.mean(axis=0)
        radii = np.linalg.norm(verts - center, axis=1)
        assert radii.max() - radii.min() < 1e-6, (
            f"{name}: face {poly} not cyclic, radii {radii}"
        )


# ---------------------------------------------------------------------------
# Face congruence for Catalan solids (face-transitive).
# ---------------------------------------------------------------------------


def _sorted_side_lengths(verts):
    sides = np.linalg.norm(verts - np.roll(verts, -1, axis=0), axis=1)
    return tuple(sorted(round(float(s), 6) for s in sides))


@pytest.mark.parametrize("name", [s.name for s in P.CATALAN])
def test_catalan_faces_congruent(name):
    """All faces of a Catalan solid are congruent to each other (face-
    transitive).  We compare sorted side-length signatures."""
    mesh = P.generate_mesh(name)
    polys = _face_polygons(mesh)
    sigs = {_sorted_side_lengths(mesh.vertices[poly]) for poly in polys}
    assert len(sigs) == 1, (
        f"{name}: faces are not all congruent — found {len(sigs)} distinct "
        f"side-length signatures"
    )
