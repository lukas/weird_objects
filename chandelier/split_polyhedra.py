"""Split each chandelier polyhedron into its own per-polyhedron STL,
oriented for FDM printing.

Each output STL contains exactly one polyhedron, sized to its post-cast
diameter (~130 mm for the reference 42" chandelier), rotated so the
largest face points down toward the build plate, translated so the
lowest point sits at Z = 0 (printer build-plate convention), and with an
M3 heat-set insert pocket subtracted from the top vertex node so a brass
insert can be melted in for the cable + eye-bolt attachment per
ASSEMBLY.md §7.

This is the "minimalist build" workflow: print these 31 plastic
polyhedra in PETG, paint them metallic, and bolt them onto the cast
metal star + cable suspension described in ASSEMBLY.md.

Usage
-----

    python split_polyhedra.py                     # writes ./polyhedra_stl/*.stl
    python split_polyhedra.py --out-dir parts/    # alternative output dir
    python split_polyhedra.py --skip-pocket       # skip the heat-set pocket
                                                  # (drill it post-print instead)
    python split_polyhedra.py --skip-fiber-hole   # skip the fiber pass-through

Output filenames are prefixed by category so the 31 STLs sort cleanly
in a slicer file browser:

    polyhedra_stl/
        platonic_01_tetrahedron.stl
        platonic_02_cube.stl
        ...
        archimedean_01_truncated_tetrahedron.stl
        ...
        catalan_01_triakis_tetrahedron.stl
        ...
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import trimesh

import polyhedra as P
from all_polyhedra import (
    NODE_DIAMETER_MM,
    NODE_RADIUS,
    TARGET_OUTER_DIAMETER_MM,
    ARCHIMEDEAN_RING_R,
    make_wire_solid,
    PLATONIC_ORDER,
    ARCHIMEDEAN_ORDER,
)


# ---------------------------------------------------------------------------
# Heat-set insert pocket (M3 brass insert, ~ 5 mm OD × 5 mm long)
# ---------------------------------------------------------------------------
#
# Sized for a snug press-fit on a standard M3 brass heat-set insert.
# Common sizes:
#   - "Voron-style" 5 mm OD × 4 mm long inserts -> use 4.0 mm hole Ø
#   - Short 4.2 mm OD × 3 mm long inserts       -> use 3.6 mm hole Ø
# The default below targets the more common 5 mm OD insert.  Override
# with --pocket-radius / --pocket-depth if you have different inserts.
INSERT_HOLE_DIAMETER_MM = 4.2     # post-cast mm; for 5 mm OD inserts
INSERT_HOLE_DEPTH_MM    = 5.5     # post-cast mm; goes through the node

# Single loose fiber pass-through next to the suspension insert.  A 2.0 mm
# printed hole is intentionally generous for common 1.0-1.5 mm PMMA end-glow
# fiber without consuming too much of the ~10 mm top node.
FIBER_HOLE_DIAMETER_MM = 2.0
FIBER_HOLE_DEPTH_MM = 10.5
FIBER_HOLE_OFFSET_MM = 3.25


# ---------------------------------------------------------------------------
# Chandelier scale factor — matches the SCALE_FACTOR used by
# all_polyhedra.py and make_panel_outlines.py.  Recompute from the same
# geometry so this stays in sync if dimensions change.
# ---------------------------------------------------------------------------

def _chandelier_scale():
    horizontal_extent = 2.0 * (ARCHIMEDEAN_RING_R + NODE_DIAMETER_MM / 2.0)
    return TARGET_OUTER_DIAMETER_MM / horizontal_extent


SCALE_FACTOR = _chandelier_scale()  # ≈ 3.598


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _safe_filename(name: str) -> str:
    return (name.lower()
                .replace(" ", "_")
                .replace("/", "_")
                .replace("(", "")
                .replace(")", ""))


def _build_top_node_hole(top_vertex_post, hole_diameter_mm, hole_depth_mm,
                         radial_offset_mm=0.0):
    """Return a cylindrical cutter aligned along +Y, entering through the
    top surface of the top vertex node and extending down into the part.
    Coordinates are post-cast mm."""
    overshoot = 1.0  # mm, ensures clean boolean past the node surface
    height = hole_depth_mm + overshoot
    node_radius_post = NODE_RADIUS * SCALE_FACTOR
    top_surface_y = top_vertex_post[1] + node_radius_post
    cyl = trimesh.creation.cylinder(
        radius=hole_diameter_mm / 2.0,
        height=height,
        sections=48,
    )
    cyl.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1.0, 0.0, 0.0]))
    cyl.apply_translation([
        top_vertex_post[0] + radial_offset_mm,
        top_surface_y + overshoot / 2.0 - hole_depth_mm / 2.0,
        top_vertex_post[2],
    ])
    return cyl


def _split_index_for_filename(category: str, name: str) -> int:
    """Return a stable 1-based index per category so filenames sort by
    the same order used in the chandelier (matches the visual layout)."""
    if category == "platonic":
        return PLATONIC_ORDER.index(name) + 1
    if category == "archimedean":
        return ARCHIMEDEAN_ORDER.index(name) + 1
    if category == "catalan":
        # Catalans ordered by their parent Archimedean's position.
        for i, arch_name in enumerate(ARCHIMEDEAN_ORDER):
            cat = next((s for s in P.CATALAN if s.parent == arch_name), None)
            if cat is not None and cat.name == name:
                return i + 1
    return 0


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v if n < 1e-12 else v / n


def _oriented_solid_vertices(solid):
    """Return the source polyhedron vertices in the same +Y-up orientation
    used by ``all_polyhedra.make_wire_solid``."""
    verts, _edges = P.generate_wireframe_data(solid)
    verts = np.asarray(verts, dtype=float)
    verts = verts - verts.mean(axis=0)
    top_idx = int(np.argmax(verts[:, 1]))
    R = trimesh.geometry.align_vectors(
        _unit(verts[top_idx]),
        np.array([0.0, 1.0, 0.0]),
    )[:3, :3]
    return verts @ R.T


def _polygon_faces(mesh):
    """Yield vertex-index arrays for real polygon faces of a triangulated
    convex mesh, with coplanar triangle facets grouped."""
    faceted_tris = set()
    for facet in mesh.facets:
        idx = np.asarray(facet, dtype=int)
        faceted_tris.update(int(i) for i in idx)
        yield np.unique(mesh.faces[idx].reshape(-1))

    for tri_idx, tri in enumerate(mesh.faces):
        if int(tri_idx) not in faceted_tris:
            yield np.asarray(tri, dtype=int)


def _face_area(face_verts):
    """Area of a convex polygon face in 3-D."""
    centre = face_verts.mean(axis=0)
    area = 0.0
    for i in range(len(face_verts)):
        a = face_verts[i] - centre
        b = face_verts[(i + 1) % len(face_verts)] - centre
        area += 0.5 * np.linalg.norm(np.cross(a, b))
    return area


def _print_face_down_transform(solid):
    """Return a transform that points the largest face toward -Z.

    The hanging/open face is not always the best print base: for example,
    the rhombicuboctahedron's lowest hanging face is a small triangle.  Use
    the largest polygon face instead so the slicer gets the broadest, most
    stable face-down orientation available for that solid.
    """
    source_mesh = P.generate_mesh(solid)
    verts = _oriented_solid_vertices(solid)

    print_face = max(
        _polygon_faces(source_mesh),
        key=lambda face: _face_area(verts[face]),
    )
    face_centroid = verts[print_face].mean(axis=0)
    outward_normal = _unit(face_centroid - verts.mean(axis=0))
    return trimesh.geometry.align_vectors(
        outward_normal,
        np.array([0.0, 0.0, -1.0]),
    )


def split_one(solid, *, with_pocket: bool, with_fiber_hole: bool,
              hole_diameter_mm: float, hole_depth_mm: float,
              fiber_hole_diameter_mm: float, fiber_hole_depth_mm: float,
              fiber_hole_offset_mm: float):
    """Build one print-ready per-polyhedron mesh."""
    mesh, world_verts = make_wire_solid(solid, center=(0.0, 0.0, 0.0))

    # Scale up to post-cast mm so pocket dimensions are in real world units.
    mesh.apply_scale(SCALE_FACTOR)
    world_verts_post = world_verts * SCALE_FACTOR

    if with_pocket:
        top_v = world_verts_post[int(np.argmax(world_verts_post[:, 1]))]
        pocket = _build_top_node_hole(top_v, hole_diameter_mm, hole_depth_mm)
        try:
            mesh = trimesh.boolean.difference([mesh, pocket], engine="manifold")
        except Exception as e:  # pragma: no cover — defensive fallback
            print(f"    pocket subtraction failed for {solid.name}: {e};"
                  " skipping pocket on this part")

    if with_fiber_hole:
        top_v = world_verts_post[int(np.argmax(world_verts_post[:, 1]))]
        fiber_hole = _build_top_node_hole(
            top_v,
            fiber_hole_diameter_mm,
            fiber_hole_depth_mm,
            radial_offset_mm=fiber_hole_offset_mm,
        )
        try:
            mesh = trimesh.boolean.difference([mesh, fiber_hole],
                                              engine="manifold")
        except Exception as e:  # pragma: no cover — defensive fallback
            print(f"    fiber hole subtraction failed for {solid.name}: {e};"
                  " skipping fiber hole on this part")

    # Rotate for printing: put the largest face down toward the build plate,
    # rather than leaving the hanging top vertex pointed upward.
    mesh.apply_transform(_print_face_down_transform(solid))

    # Translate so the lowest point sits at Z = 0 (build-plate datum).
    mesh.apply_translation([0.0, 0.0, -mesh.bounds[0][2]])

    return mesh


def split_all(out_dir: str, *, with_pocket: bool, with_fiber_hole: bool,
              hole_diameter_mm: float, hole_depth_mm: float,
              fiber_hole_diameter_mm: float, fiber_hole_depth_mm: float,
              fiber_hole_offset_mm: float):
    os.makedirs(out_dir, exist_ok=True)

    print(f"Per-polyhedron STL split (scale {SCALE_FACTOR:.3f}x)")
    print(f"  Each polyhedron: ~{NODE_DIAMETER_MM * SCALE_FACTOR:.0f} mm OD post-cast")
    if with_pocket:
        print(f"  Heat-set insert pocket: {hole_diameter_mm:.1f} mm Ø × "
              f"{hole_depth_mm:.1f} mm deep at top vertex")
    else:
        print("  Heat-set insert pocket: SKIPPED (drill post-print)")
    if with_fiber_hole:
        print(f"  Fiber pass-through: {fiber_hole_diameter_mm:.1f} mm Ø × "
              f"{fiber_hole_depth_mm:.1f} mm deep, "
              f"{fiber_hole_offset_mm:.1f} mm beside insert")
    else:
        print("  Fiber pass-through: SKIPPED")
    print(f"  Output dir: {out_dir}")
    print()

    n_ok = 0
    for solid in P.ALL_SOLIDS:
        idx = _split_index_for_filename(solid.category, solid.name)
        fname = (f"{solid.category}_"
                 f"{idx:02d}_{_safe_filename(solid.name)}.stl")
        out_path = os.path.join(out_dir, fname)

        mesh = split_one(solid, with_pocket=with_pocket,
                         with_fiber_hole=with_fiber_hole,
                         hole_diameter_mm=hole_diameter_mm,
                         hole_depth_mm=hole_depth_mm,
                         fiber_hole_diameter_mm=fiber_hole_diameter_mm,
                         fiber_hole_depth_mm=fiber_hole_depth_mm,
                         fiber_hole_offset_mm=fiber_hole_offset_mm)
        mesh.export(out_path)

        ext = mesh.bounds[1] - mesh.bounds[0]
        wt = "OK" if mesh.is_watertight else "open"
        print(f"  [{solid.category:11s}] {solid.name:32s}  "
              f"size={ext[0]:5.1f} x {ext[1]:5.1f} x {ext[2]:5.1f} mm  "
              f"({wt})  -> {fname}")
        n_ok += 1

    print()
    print(f"Wrote {n_ok} STL files to {out_dir}/")
    print("Each polyhedron prints in PETG at ~ 0.3 mm layers, 15% gyroid")
    print("infill, no supports.  See ASSEMBLY.md §6.1 for full print settings.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="polyhedra_stl",
                    help="output directory (default: polyhedra_stl)")
    ap.add_argument("--skip-pocket", action="store_true",
                    help="don't subtract the heat-set insert pocket "
                         "(drill it post-print instead)")
    ap.add_argument("--skip-fiber-hole", action="store_true",
                    help="don't subtract the top-node fiber pass-through")
    ap.add_argument("--pocket-diameter", type=float, default=INSERT_HOLE_DIAMETER_MM,
                    help=f"insert pocket Ø in mm (default: {INSERT_HOLE_DIAMETER_MM})")
    ap.add_argument("--pocket-depth", type=float, default=INSERT_HOLE_DEPTH_MM,
                    help=f"insert pocket depth in mm (default: {INSERT_HOLE_DEPTH_MM})")
    ap.add_argument("--fiber-hole-diameter", type=float,
                    default=FIBER_HOLE_DIAMETER_MM,
                    help=f"fiber pass-through Ø in mm "
                         f"(default: {FIBER_HOLE_DIAMETER_MM})")
    ap.add_argument("--fiber-hole-depth", type=float,
                    default=FIBER_HOLE_DEPTH_MM,
                    help=f"fiber pass-through depth in mm "
                         f"(default: {FIBER_HOLE_DEPTH_MM})")
    ap.add_argument("--fiber-hole-offset", type=float,
                    default=FIBER_HOLE_OFFSET_MM,
                    help=f"fiber pass-through offset from insert centre in mm "
                         f"(default: {FIBER_HOLE_OFFSET_MM})")
    args = ap.parse_args()

    split_all(args.out_dir,
              with_pocket=not args.skip_pocket,
              with_fiber_hole=not args.skip_fiber_hole,
              hole_diameter_mm=args.pocket_diameter,
              hole_depth_mm=args.pocket_depth,
              fiber_hole_diameter_mm=args.fiber_hole_diameter,
              fiber_hole_depth_mm=args.fiber_hole_depth,
              fiber_hole_offset_mm=args.fiber_hole_offset)


if __name__ == "__main__":
    main()
