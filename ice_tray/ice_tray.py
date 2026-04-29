"""Two-part ice mold for all 31 Platonic, Archimedean, and Catalan solids.

Each ice-cube cavity is fully enclosed by a clamshell mold (bottom + top
halves), so the resulting ice cubes take the COMPLETE 3D shape of each
polyhedron — not just the bottom half with a flat water-line top.

Outputs:
    ice_tray_31_bottom.stl   bottom half (cavities open upward, alignment pegs)
    ice_tray_31_top.stl      top    half (cavities open downward, fill holes,
                                          alignment peg holes)

How it works
------------
1. Each polyhedron is oriented with its largest face up and uniformly scaled
   to a 30 mm circumscribed sphere.
2. For every polyhedron we find the z value where the horizontal cross-section
   has the maximum area (the "widest cross-section").  This is the natural
   split plane for a two-part mold of a convex shape: both halves taper
   outward from this plane, so the ice releases cleanly when the mold opens.
3. Every polyhedron is positioned so its widest cross-section sits at a
   common SPLIT_Z plane on the tray, and centered in its grid cell.
4. The bottom block is carved by the cavities (lower halves) and grows four
   alignment pegs at the corners.
5. The top block is carved by the cavities (upper halves), four matching
   peg-holes, and a small fill hole over every cavity for pouring water.

Symmetric solids (cube, octahedron, dodeca/icosa, all the snubs, every
Catalan) split right at the equator.  Asymmetric solids (tetrahedron,
truncated tetrahedron) end up as a deep cup with a thin flat lid — still
fully enclosed, ice still releases cleanly.

Layout
------
Same 7-column × 5-row grid as the prior single-piece tray:

    Row 0:  5 Platonic    (cols 0..4)
    Row 1:  7 Archimedean (cols 0..6)
    Row 2:  6 Archimedean (cols 0..5)
    Row 3:  7 Catalan     (cols 0..6)  -- duals of row 1
    Row 4:  6 Catalan     (cols 0..5)  -- duals of row 2
"""

from __future__ import annotations

import numpy as np
import trimesh

import polyhedra as P


# ---------------------------------------------------------------------------
# Tray dimensions (all in mm)
# ---------------------------------------------------------------------------

COLS                = 7
ROWS                = 5
CELL_SIZE           = 40.0      # pitch between adjacent cell centers
TRAY_BORDER         = 10.0      # rim outside the grid (room for alignment pegs)
BOTTOM_THICKNESS    = 4.0       # solid floor below the cavities
TOP_THICKNESS       = 4.0       # solid ceiling above the cavities

POLY_DIAMETER       = 30.0      # circumscribed-sphere diameter for every solid

FILL_HOLE_RADIUS    = 2.0       # mm — pour spout through the top piece
FILL_HOLE_OVERSHOOT = 1.0       # mm above the top surface for clean cut

PEG_RADIUS          = 2.5       # mm — alignment pin radius
PEG_HEIGHT          = 6.0       # mm above the split plane
PEG_HOLE_TOLERANCE  = 0.30      # mm — extra radius for slip-fit hole
PEG_INSET           = 5.0       # mm from tray edges

OUTFILE_BOTTOM = "ice_tray_31_bottom.stl"
OUTFILE_TOP    = "ice_tray_31_top.stl"

MM_PER_INCH = 25.4


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------


def build_layout():
    """Return ``[(solid, col, row)]`` for all 31 solids."""
    layout = []
    for i, s in enumerate(P.PLATONIC):
        layout.append((s, i, 0))
    for i in range(7):
        layout.append((P.ARCHIMEDEAN[i], i, 1))
    for i in range(6):
        layout.append((P.ARCHIMEDEAN[7 + i], i, 2))
    for i in range(7):
        layout.append((P.CATALAN[i], i, 3))
    for i in range(6):
        layout.append((P.CATALAN[7 + i], i, 4))
    return layout


# ---------------------------------------------------------------------------
# Polyhedron orientation & sizing
# ---------------------------------------------------------------------------


def _polygon_faces(mesh):
    """Yield ``(triangle_indices, total_area, outward_normal)`` for every
    polygon face of a triangulated convex mesh.  Coplanar adjacent triangles
    are grouped into a single facet; isolated triangles count as their own
    face."""
    in_facet: set[int] = set()
    for f in mesh.facets:
        for t in f:
            in_facet.add(int(t))
        area = float(mesh.area_faces[f].sum())
        normal = np.asarray(mesh.face_normals[int(f[0])])
        yield np.asarray(f, dtype=int), area, normal
    for i in range(len(mesh.faces)):
        if i not in in_facet:
            yield (np.array([i], dtype=int),
                   float(mesh.area_faces[i]),
                   np.asarray(mesh.face_normals[i]))


def _orient_largest_face_up(mesh):
    """Rotate ``mesh`` so its largest polygon face's outward normal points +Z."""
    best_area = -1.0
    best_normal = np.array([0.0, 0.0, 1.0])
    for _tris, area, normal in _polygon_faces(mesh):
        if area > best_area:
            best_area = area
            best_normal = normal
    target = np.array([0.0, 0.0, 1.0])
    if np.allclose(best_normal, target, atol=1e-6):
        return mesh.copy()
    if np.allclose(best_normal, -target, atol=1e-6):
        T = trimesh.transformations.rotation_matrix(np.pi, [1.0, 0.0, 0.0])
    else:
        T = trimesh.geometry.align_vectors(best_normal, target)
    out = mesh.copy()
    out.apply_transform(T)
    return out


def _build_cavity_solid(solid, target_diameter):
    """Return an oriented and uniformly scaled solid mesh, centered at origin."""
    mesh = P.generate_mesh(solid).copy()
    mesh.apply_translation(-mesh.bounds.mean(axis=0))
    radius = float(np.linalg.norm(mesh.vertices, axis=1).max())
    mesh.apply_scale(target_diameter / 2.0 / radius)
    mesh = _orient_largest_face_up(mesh)
    return mesh


def find_widest_z(mesh, n_samples=160, tie_tol=0.005):
    """Return ``(z, area)`` where the horizontal cross-section of ``mesh`` is
    widest.  On ties (within ``tie_tol`` fractional area) the z closest to
    the geometric center is preferred — this keeps cube-like shapes
    (constant cross-section) splitting at the equator."""
    z_lo, z_hi = float(mesh.bounds[0][2]), float(mesh.bounds[1][2])
    z_mid = (z_lo + z_hi) / 2.0
    eps = (z_hi - z_lo) * 0.005
    zs = np.linspace(z_lo + eps, z_hi - eps, n_samples)
    areas = np.zeros(len(zs))
    for i, z in enumerate(zs):
        try:
            section = mesh.section(plane_origin=[0.0, 0.0, z],
                                   plane_normal=[0.0, 0.0, 1.0])
            if section is None:
                continue
            planar, _ = section.to_2D()
            areas[i] = float(planar.area)
        except Exception:
            continue
    if areas.max() <= 0:
        return z_mid, 0.0
    near_max = areas >= areas.max() * (1.0 - tie_tol)
    candidates = zs[near_max]
    best_z = float(candidates[np.argmin(np.abs(candidates - z_mid))])
    return best_z, float(areas.max())


# ---------------------------------------------------------------------------
# Two-part mold construction
# ---------------------------------------------------------------------------


def build_cavities():
    """Build oriented + scaled meshes for each polyhedron and measure how far
    they extend below / above their widest cross-section."""
    layout = build_layout()
    print(f"Computing widest cross-section for {len(layout)} polyhedra...")
    cavities = []
    for solid, col, row in layout:
        poly = _build_cavity_solid(solid, POLY_DIAMETER)
        widest_z, area = find_widest_z(poly)
        h_below = widest_z - poly.bounds[0][2]
        h_above = poly.bounds[1][2] - widest_z
        print(f"  {solid.category:11s}  {solid.name:32s}"
              f"  h_below={h_below:5.2f}  h_above={h_above:5.2f}"
              f"  split_area={area:6.2f} mm²")
        cavities.append({
            'solid': solid, 'col': col, 'row': row, 'poly': poly,
            'widest_z': widest_z, 'h_below': h_below, 'h_above': h_above,
        })
    return cavities


def build_mold():
    cavities = build_cavities()

    max_h_below = max(c['h_below'] for c in cavities)
    max_h_above = max(c['h_above'] for c in cavities)

    SPLIT_Z       = BOTTOM_THICKNESS + max_h_below
    TRAY_TOP_Z    = SPLIT_Z + max_h_above + TOP_THICKNESS
    BOTTOM_PIECE_H = SPLIT_Z
    TOP_PIECE_H    = TRAY_TOP_Z - SPLIT_Z

    tray_w = COLS * CELL_SIZE + 2 * TRAY_BORDER
    tray_d = ROWS * CELL_SIZE + 2 * TRAY_BORDER

    print(f"\nMold layout:")
    print(f"  Footprint:    {tray_w:.1f} x {tray_d:.1f} mm "
          f"({tray_w/MM_PER_INCH:.2f} x {tray_d/MM_PER_INCH:.2f} in)")
    print(f"  Total height: {TRAY_TOP_Z:.1f} mm  (assembled)")
    print(f"  Split at z =  {SPLIT_Z:.1f} mm")
    print(f"  Bottom piece: {BOTTOM_PIECE_H:.1f} mm tall")
    print(f"  Top piece:    {TOP_PIECE_H:.1f} mm tall")
    print(f"  max h_below = {max_h_below:.2f}, max h_above = {max_h_above:.2f}")

    # -- 1. Position cavities so widest cross-section is at z = SPLIT_Z --
    positioned = []
    for c in cavities:
        cell_x = TRAY_BORDER + (c['col'] + 0.5) * CELL_SIZE
        cell_y = TRAY_BORDER + (c['row'] + 0.5) * CELL_SIZE
        z_off  = SPLIT_Z - c['widest_z']
        c['poly'].apply_translation([cell_x, cell_y, z_off])
        positioned.append(c['poly'])

    # -- 2. Base blocks --
    bottom_box = trimesh.creation.box(extents=[tray_w, tray_d, BOTTOM_PIECE_H])
    bottom_box.apply_translation([tray_w / 2, tray_d / 2, BOTTOM_PIECE_H / 2])

    top_box = trimesh.creation.box(extents=[tray_w, tray_d, TOP_PIECE_H])
    top_box.apply_translation([tray_w / 2, tray_d / 2, SPLIT_Z + TOP_PIECE_H / 2])

    # -- 3. Alignment pegs (cylinders at the four corners) --
    peg_xy = [
        (PEG_INSET,           PEG_INSET),
        (tray_w - PEG_INSET,  PEG_INSET),
        (PEG_INSET,           tray_d - PEG_INSET),
        (tray_w - PEG_INSET,  tray_d - PEG_INSET),
    ]
    pegs, peg_holes = [], []
    for px, py in peg_xy:
        peg = trimesh.creation.cylinder(radius=PEG_RADIUS, height=PEG_HEIGHT)
        peg.apply_translation([px, py, SPLIT_Z + PEG_HEIGHT / 2.0])
        pegs.append(peg)
        h_hole = PEG_HEIGHT + 1.0  # slightly taller for clean cut
        hole = trimesh.creation.cylinder(
            radius=PEG_RADIUS + PEG_HOLE_TOLERANCE, height=h_hole
        )
        hole.apply_translation([px, py, SPLIT_Z + h_hole / 2.0])
        peg_holes.append(hole)

    # -- 4. Fill holes (one cylinder per cavity, through the top piece) --
    fill_holes = []
    for poly in positioned:
        cavity_top_z = poly.bounds[1][2]
        cx = (poly.bounds[0][0] + poly.bounds[1][0]) / 2.0
        cy = (poly.bounds[0][1] + poly.bounds[1][1]) / 2.0
        z_lo = cavity_top_z - 1.0          # overlap a hair into the cavity
        z_hi = TRAY_TOP_Z + FILL_HOLE_OVERSHOOT
        h    = z_hi - z_lo
        hole = trimesh.creation.cylinder(radius=FILL_HOLE_RADIUS, height=h)
        hole.apply_translation([cx, cy, (z_lo + z_hi) / 2.0])
        fill_holes.append(hole)

    # -- 5. Boolean operations --
    print(f"\nBooleans:")
    print(f"  Union of {len(positioned)} cavities...")
    cavities_union = trimesh.boolean.union(positioned, engine="manifold")
    print(f"    cavities union: {len(cavities_union.faces):,} faces, "
          f"watertight={cavities_union.is_watertight}")

    print(f"  Bottom piece = (box ∪ {len(pegs)} pegs) − cavities ...")
    bottom_with_pegs = trimesh.boolean.union([bottom_box, *pegs], engine="manifold")
    bottom_piece = trimesh.boolean.difference(
        [bottom_with_pegs, cavities_union], engine="manifold"
    )

    print(f"  Top piece    = box − cavities − {len(fill_holes)} fill holes "
          f"− {len(peg_holes)} peg holes ...")
    top_holes_union = trimesh.boolean.union(fill_holes + peg_holes, engine="manifold")
    top_subs        = trimesh.boolean.union(
        [cavities_union, top_holes_union], engine="manifold"
    )
    top_piece = trimesh.boolean.difference([top_box, top_subs], engine="manifold")

    return bottom_piece, top_piece, cavities_union


def main():
    bottom_piece, top_piece, _cavities = build_mold()

    for label, mesh, fname in [
        ("Bottom", bottom_piece, OUTFILE_BOTTOM),
        ("Top",    top_piece,    OUTFILE_TOP),
    ]:
        ext = mesh.bounds[1] - mesh.bounds[0]
        print(f"\n{label} piece:")
        print(f"  faces:        {len(mesh.faces):,}")
        print(f"  watertight:   {mesh.is_watertight}")
        print(f"  is_volume:    {mesh.is_volume}")
        print(f"  bounds (mm):  {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f}")
        print(f"  bounds (in):  {ext[0]/MM_PER_INCH:.2f} x "
              f"{ext[1]/MM_PER_INCH:.2f} x {ext[2]/MM_PER_INCH:.2f}")
        print(f"  volume cm³:   {mesh.volume / 1.0e3:.1f}")
        mesh.export(fname)
        print(f"  saved -> {fname}")


if __name__ == "__main__":
    main()
