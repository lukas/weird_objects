"""Generate a free-standing rhombic-triacontahedron of cut-and-project
icosahedral quasicrystal lattice — no enclosing cube, no edges chopped
mid-strut.

The rhombic triacontahedron is the canonical icosahedral envelope:

* It is the projection of the unit 6-cube ``[0, 1]⁶`` through the same
  ``P‖`` matrix that we use to project the 6-D lattice to physical space —
  so it is literally the physical-space "shape" of a single 6-D unit cell.
* Its 30 rhombic faces are golden rhombi (diagonals in ratio φ) — every
  face is congruent and identical to the faces of the prolate / oblate
  golden rhombohedra of 3-D Penrose tiling.
* Its symmetry group is full icosahedral ``Iₕ`` — the same as the
  quasicrystal lattice we drop inside it.
* Real icosahedral quasicrystals (Cd-Yb, Cd-Ho, Al-Pd-Mn, …) cleave and
  grow into rhombic-triacontahedral grains in the lab.

Two of the 30 face-normals are aligned with ±Z in our convention (the
2-fold direction we use for printing), so the part sits flat on a
rhombic face for FDM printing.
"""

import argparse
from pathlib import Path

import numpy as np
import trimesh
from skimage import measure

from make_lattice_cubes import (
    _PHI,
    _draw_cylinder_into_sdf,
    _draw_sphere_into_sdf,
    _report_lattice_printability,
    icosahedral_quasi_lattice,
)


# ---------------------------------------------------------------------------
# Rhombic triacontahedron geometry
# ---------------------------------------------------------------------------

def _icosahedron_vertices():
    """The 12 canonical icosahedron vertices ``(0, ±1, ±φ)`` and cyclic
    permutations. Edge length is exactly 2."""
    phi = _PHI
    verts = []
    for s1 in (1.0, -1.0):
        for s2 in (1.0, -1.0):
            verts.append((0.0, s1, s2 * phi))
            verts.append((s1, s2 * phi, 0.0))
            verts.append((s2 * phi, 0.0, s1))
    return np.array(verts, dtype=np.float64)


def _triacontahedron_face_normals():
    """The 30 unit face-normals of the rhombic triacontahedron — equivalently,
    the 30 normalised edge midpoints of the icosahedron (a.k.a. the vertices
    of the icosidodecahedron)."""
    verts = _icosahedron_vertices()
    n = len(verts)
    edge_length_sq = 4.0  # canonical icosahedron edge length is 2
    mids = []
    for i in range(n):
        for j in range(i + 1, n):
            d2 = float(np.sum((verts[i] - verts[j]) ** 2))
            if abs(d2 - edge_length_sq) < 1e-6:
                mids.append((verts[i] + verts[j]) / 2.0)
    mids = np.asarray(mids, dtype=np.float64)
    assert len(mids) == 30, f"expected 30 face normals, got {len(mids)}"
    return mids / np.linalg.norm(mids, axis=1, keepdims=True)


_TRIACO_NORMALS = _triacontahedron_face_normals()

# Ratios between the rhombic-triacontahedron's three characteristic radii
# and its in-radius (perpendicular origin → face distance):
#   5-fold vertex distance = in_radius · √(2 + φ) / φ ≈ 1.1756 · in_radius
#   3-fold vertex distance = in_radius · √3 / φ        ≈ 1.0705 · in_radius
# (derived by checking when a vertex along (0, 1, φ) or (1, 1, 1) touches
# its adjacent unit-length face normals — see derivation in the README.)
_TRIACO_CIRCUM_OVER_IN = float(np.sqrt(2.0 + _PHI) / _PHI)


def triacontahedron_sdf(points, in_radius):
    """Vectorised signed-distance estimate to the rhombic triacontahedron
    of given in-radius. Negative inside, positive outside. Uses the
    half-space-intersection bound — exact on faces, conservative near
    edges/vertices (good enough for filtering)."""
    return np.max(points @ _TRIACO_NORMALS.T, axis=-1) - in_radius


def inside_triacontahedron(points, in_radius):
    return triacontahedron_sdf(points, in_radius) <= 0.0


# ---------------------------------------------------------------------------
# Lattice generation
# ---------------------------------------------------------------------------

def triacontahedron_lattice_mesh(
    in_radius_mm=22.0,
    edge_mm=6.0,
    vertex_radius_mm=1.1,
    edge_radius_mm=0.6,
    acceptance_radius=1.5,
    resolution=200,
    layer_height_mm=0.2,
):
    # 1. Generate enough lattice to fill the triacontahedron. The
    #    circumradius is in_radius * √(2 + φ); enumerate over a cube of
    #    slightly larger half-side so every accepted vertex is reachable.
    circum = in_radius_mm * _TRIACO_CIRCUM_OVER_IN
    enum_cube_side = 2.0 * (circum + 2.0 * edge_mm)

    vertices, edges = icosahedral_quasi_lattice(
        enum_cube_side, edge_mm, acceptance_radius=acceptance_radius,
    )

    # 2. Filter to the triacontahedral envelope.
    inside = inside_triacontahedron(vertices, in_radius_mm)
    n_in = int(inside.sum())
    print(f"  vertices: {n_in} of {len(vertices)} inside the triacontahedron")
    if n_in == 0:
        raise RuntimeError("no lattice vertices fall inside the chosen envelope")

    keep_idx = np.where(inside)[0]
    idx_map = -np.ones(len(vertices), dtype=np.int64)
    idx_map[keep_idx] = np.arange(n_in)
    vertices = vertices[inside]

    new_edges = []
    for i, j in edges:
        ni = idx_map[i]
        nj = idx_map[j]
        if ni >= 0 and nj >= 0:
            new_edges.append((int(ni), int(nj)))
    edges = new_edges
    print(f"  edges   : {len(edges)} (both endpoints inside; no chopped struts)")

    _report_lattice_printability(
        vertices, edges, layer_height_mm, edge_radius_mm, max_bridge_mm=10.0,
    )

    # 3. Build the SDF on a cubical grid that wraps the triacontahedron
    #    plus a couple of vertex-radii of slack.
    extent = float(circum + 1.5 * vertex_radius_mm)
    coords = np.linspace(-extent, extent, resolution).astype(np.float32)
    spacing = float(coords[1] - coords[0])
    print(f"  grid    : {resolution}³ voxels, spacing = {spacing:.3f} mm, "
          f"extent ±{extent:.2f} mm")

    sdf = np.full((resolution, resolution, resolution), 1.0e6, dtype=np.float32)
    for v in vertices:
        _draw_sphere_into_sdf(sdf, coords, v, vertex_radius_mm, spacing)
    for i, j in edges:
        _draw_cylinder_into_sdf(
            sdf, coords, vertices[i], vertices[j], edge_radius_mm, spacing,
        )

    # 4. No outer-shape clip — the lattice draws its own boundary.
    verts, faces, _, _ = measure.marching_cubes(
        sdf, level=0.0, spacing=(spacing, spacing, spacing),
    )
    verts -= extent
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)

    components = mesh.split(only_watertight=False)
    if len(components) > 1:
        components = sorted(components, key=lambda m: -len(m.faces))
        dropped = sum(len(c.faces) for c in components[1:])
        print(f"  dropped {len(components) - 1} disconnected fragment(s) "
              f"({dropped} face(s)) — keeping main lattice")
        mesh = components[0]

    return mesh


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--in-radius-mm", type=float, default=22.0,
        help="Triacontahedron in-radius — perpendicular distance from origin "
             "to any face (default 22 mm → ~42 mm tip-to-tip)")
    parser.add_argument("--edge-mm", type=float, default=6.0,
                        help="Quasicrystal edge length")
    parser.add_argument("--vertex-radius-mm", type=float, default=1.1)
    parser.add_argument("--edge-radius-mm", type=float, default=0.6)
    parser.add_argument("--acceptance-radius", type=float, default=1.5,
                        help="Internal-space acceptance window radius "
                             "(in unscaled 6-D units)")
    parser.add_argument("--resolution", type=int, default=200,
                        help="Voxels per axis for marching cubes (default 200)")
    parser.add_argument(
        "--out", type=str,
        default=str(Path(__file__).parent / "stl"
                    / "quasicrystal_triacontahedron.stl"),
    )
    args = parser.parse_args()

    print("=== quasicrystal_triacontahedron  —  Icosahedral quasicrystal lattice in its natural envelope ===")
    print(f"  in-radius          = {args.in_radius_mm} mm")
    print(f"  circumradius (5-fold tip) = "
          f"{args.in_radius_mm * _TRIACO_CIRCUM_OVER_IN:.2f} mm")
    print(f"  edge length        = {args.edge_mm} mm")
    print(f"  vertex / edge r    = {args.vertex_radius_mm} / "
          f"{args.edge_radius_mm} mm")

    mesh = triacontahedron_lattice_mesh(
        in_radius_mm=args.in_radius_mm,
        edge_mm=args.edge_mm,
        vertex_radius_mm=args.vertex_radius_mm,
        edge_radius_mm=args.edge_radius_mm,
        acceptance_radius=args.acceptance_radius,
        resolution=args.resolution,
    )

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(out))
    print(f"  faces       : {len(mesh.faces):,}")
    lo, hi = mesh.bounds
    print(f"  bounds (mm) : [{lo[0]:.2f}, {lo[1]:.2f}, {lo[2]:.2f}]  →  "
          f"[{hi[0]:.2f}, {hi[1]:.2f}, {hi[2]:.2f}]")
    print(f"  watertight  : {mesh.is_watertight}")
    if mesh.is_watertight:
        print(f"  volume mm^3 : {mesh.volume:.1f}")
    print(f"  → {out}")


if __name__ == "__main__":
    main()
