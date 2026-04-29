"""Build the visualization STLs used to render the *minimalist* chandelier
build (cast metal star + cable suspension + plastic polyhedra) — see
ASSEMBLY.md.

This is the rendering counterpart to the engineering exports in
``all_polyhedra.py``.  The metal-only subset is already exported as
``chandelier_metal_minimal.stl``; the missing pieces for a complete
visual render are:

  * ``chandelier_minimalist_polyhedra.stl`` — the 31 wireframe polyhedra
    in their final chandelier positions (these are 3D-printed plastic
    in the real build and rendered with a painted-metallic shader, so
    they ship as a separate STL from the cast-metal subset).

  * ``chandelier_minimalist_cables.stl`` — the 33 stainless aircraft
    cable segments that suspend the polyhedra:
        18 from star tips down to the Platonic / Archimedean polyhedra
            (5 from the inner pentagram + 13 from the outer 13-star),
        13 from each Archimedean's bottom vertex down to its matched
            Catalan, and
         2 short structural cables from the canopy plate down to the
            centres of the inner pentagram and outer 13-star.

Run after ``all_polyhedra.py`` so the chandelier scale factor and
geometry constants are in sync.

Usage:
    python build_minimalist_render.py
"""

from __future__ import annotations

import os

import numpy as np
import trimesh

from all_polyhedra import (
    # Geometry constants
    Y_STAR,
    Y_CANOPY_DISK_BOTTOM,
    STAR_INNER_R,
    STAR_OUTER_R,
    RING_PHASE,
    NODE_DIAMETER_MM,
    TARGET_OUTER_DIAMETER_MM,
    ARCHIMEDEAN_RING_R,
    MINIMAL_CANOPY_DISK_HEIGHT,
    # Builders
    make_wire_solid,
    _layout_positions,
    _star_tip_positions,
    _top_vertex,
    _bottom_vertex,
    _cylinder_between,
    _watertight_union,
)


POLYHEDRA_OUT = "chandelier_minimalist_polyhedra.stl"
CABLES_OUT    = "chandelier_minimalist_cables.stl"


# Cable diameters in pre-scale mm.  The real stainless 7×7 aircraft
# cable is 0.8 mm OD (1/32"), but rendered at the chandelier's full
# scale that's ~ 1 px wide in a typical render and effectively invisible.
# We bump it to 2 mm post-cast (≈ 0.55 mm pre-scale) so the cables read
# clearly on screen without dominating the silhouette.
CABLE_RADIUS_PRE       = 0.30   # post ≈ 2.2 mm OD
STRUCTURAL_RADIUS_PRE  = 0.40   # post ≈ 2.9 mm OD (heavier 2 mm cable + visual margin)


def _chandelier_scale() -> float:
    horizontal_extent = 2.0 * (ARCHIMEDEAN_RING_R + NODE_DIAMETER_MM / 2.0)
    return TARGET_OUTER_DIAMETER_MM / horizontal_extent


SCALE = _chandelier_scale()  # ≈ 3.604


def build_polyhedra():
    """Build the 31 polyhedra at their final chandelier positions and
    return per-polyhedron lists of world-space vertex arrays so the
    cable builder can attach to the right anchor points."""
    items = _layout_positions()
    meshes = []
    arch_verts = []
    plat_verts = []
    cat_verts  = []

    print(f"Building {len(items)} polyhedra at chandelier positions...")
    for solid, center in items:
        m, world_verts = make_wire_solid(solid, center)
        meshes.append(m)
        if solid.category == "archimedean":
            arch_verts.append(world_verts)
        elif solid.category == "platonic":
            plat_verts.append(world_verts)
        elif solid.category == "catalan":
            cat_verts.append(world_verts)

    return meshes, arch_verts, plat_verts, cat_verts


def build_cables(arch_verts, plat_verts, cat_verts):
    """Build cylinder meshes for all 49 cables in the minimalist build.

    Cable layout (matches ASSEMBLY.md §5 + §7):

        18 hanger drops from star tips:
             5 inner-pentagram tip → top vertex of each Platonic
            13 outer-13-star tip   → top vertex of each Archimedean

        13 secondary drops:
            13 Archimedean bottom vertex → top vertex of matched Catalan

         2 structural drops:
            canopy plate centre line → centre of inner pentagram
            canopy plate centre line → centre of outer 13-star
    """
    parts = []

    inner_tips = _star_tip_positions(5,  STAR_INNER_R, Y_STAR)
    outer_tips = _star_tip_positions(13, STAR_OUTER_R, Y_STAR)

    print("  5 inner-pentagram → Platonic cables")
    for tip, verts in zip(inner_tips, plat_verts):
        c = _cylinder_between(tip, _top_vertex(verts), CABLE_RADIUS_PRE)
        if c is not None:
            parts.append(c)

    print("  13 outer-13-star → Archimedean cables")
    for tip, verts in zip(outer_tips, arch_verts):
        c = _cylinder_between(tip, _top_vertex(verts), CABLE_RADIUS_PRE)
        if c is not None:
            parts.append(c)

    print("  13 Archimedean → Catalan cables")
    for arch_v, cat_v in zip(arch_verts, cat_verts):
        c = _cylinder_between(_bottom_vertex(arch_v),
                              _top_vertex(cat_v),
                              CABLE_RADIUS_PRE)
        if c is not None:
            parts.append(c)

    print("  2 structural cables (canopy → star centres)")
    canopy_anchor_y = Y_CANOPY_DISK_BOTTOM + MINIMAL_CANOPY_DISK_HEIGHT / 2.0
    # Slight horizontal offset so the two structural cables don't sit on
    # exactly the same X-axis line and visually merge into one.  The
    # offset is in the same plane as the cables (perpendicular to the
    # camera's typical view direction) so they read as two distinct lines.
    OFFSET = 4.0  # mm pre-scale
    structural_drops = [
        ((+OFFSET, canopy_anchor_y, 0.0), (+OFFSET, Y_STAR, 0.0)),  # 13-star
        ((-OFFSET, canopy_anchor_y, 0.0), (-OFFSET, Y_STAR, 0.0)),  # pentagram
    ]
    for p1, p2 in structural_drops:
        c = _cylinder_between(p1, p2, STRUCTURAL_RADIUS_PRE)
        if c is not None:
            parts.append(c)

    print(f"  -> {len(parts)} cable cylinders")
    return parts


def main():
    print(f"Minimalist visualization STL export (scale {SCALE:.3f}x)\n")

    polyhedra_meshes, arch_v, plat_v, cat_v = build_polyhedra()
    cable_meshes = build_cables(arch_v, plat_v, cat_v)

    print()
    print("Concatenating + scaling + exporting...")

    poly_combined = trimesh.util.concatenate(polyhedra_meshes)
    poly_combined.apply_scale(SCALE)
    poly_combined.export(POLYHEDRA_OUT)
    poly_ext = poly_combined.bounds[1] - poly_combined.bounds[0]
    print(f"  {POLYHEDRA_OUT}")
    print(f"    {len(poly_combined.faces)} faces, "
          f"bounds {poly_ext[0]:.0f} x {poly_ext[1]:.0f} x {poly_ext[2]:.0f} mm")

    cable_combined = trimesh.util.concatenate(cable_meshes)
    cable_combined.apply_scale(SCALE)
    cable_combined.export(CABLES_OUT)
    cab_ext = cable_combined.bounds[1] - cable_combined.bounds[0]
    print(f"  {CABLES_OUT}")
    print(f"    {len(cable_combined.faces)} faces, "
          f"bounds {cab_ext[0]:.0f} x {cab_ext[1]:.0f} x {cab_ext[2]:.0f} mm")

    print()
    print("Render with:")
    print("  ./render_blender.sh --minimalist")


if __name__ == "__main__":
    main()
