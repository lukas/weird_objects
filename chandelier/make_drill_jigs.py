"""Generate printable drill jigs for the post-cast machining steps in
ASSEMBLY.md §6.4.

Outputs (in --out-dir, default ``jigs/``):

    canopy_drill_jig.stl       Slip-on cap that drops over the cast canopy
                               plate from above.  Through-holes in the
                               cap's top face locate the 4 canopy holes:
                                 * 1 × 15 mm Ø (fiber bundle exit)
                                 * 2 × 3   mm Ø (structural cables)
                                 * 1 × 9.5 mm Ø (wall-wart cord)

    star_tip_drill_jig.stl     Cup with a hemispherical pocket that drops
                               over a single star tip node sphere from
                               above.  Through-hole guides a 2 mm bit
                               vertically through the centre of the node.
                               Print 2–4 copies; reuse for all 18 tips
                               (13 outer 13-star + 5 inner pentagram).

Print recommendation: PETG, 3 perimeters, 25 % infill.  Optionally press
2 mm and 9.5 mm Slip-Renewable steel bushings into the canopy jig holes
for a long-life jig.

Usage:
    python make_drill_jigs.py
    python make_drill_jigs.py --out-dir jigs
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import trimesh

from all_polyhedra import (
    MINIMAL_CANOPY_DISK_RADIUS,
    MINIMAL_CANOPY_DISK_HEIGHT,
    STAR_NODE_RADIUS,
    NODE_DIAMETER_MM,
    TARGET_OUTER_DIAMETER_MM,
    ARCHIMEDEAN_RING_R,
)


# ---------------------------------------------------------------------------
# Same scale factor as the rest of the chandelier (≈ 3.598x for the
# reference 42" build).  Recompute from the same geometry so the jigs
# stay in sync if dimensions change.
# ---------------------------------------------------------------------------

def _chandelier_scale() -> float:
    horizontal_extent = 2.0 * (ARCHIMEDEAN_RING_R + NODE_DIAMETER_MM / 2.0)
    return TARGET_OUTER_DIAMETER_MM / horizontal_extent


SCALE = _chandelier_scale()

# Post-scale cast-part dimensions the jigs must register against.
CANOPY_OD_MM    = MINIMAL_CANOPY_DISK_RADIUS * 2.0 * SCALE   # ≈ 201 mm
CANOPY_THICK_MM = MINIMAL_CANOPY_DISK_HEIGHT * SCALE         # ≈ 18 mm
STAR_NODE_R_MM  = STAR_NODE_RADIUS * SCALE                   # ≈ 10.8 mm

# Drill-bit diameters (slightly oversized for clean bit slip).
HOLE_BUNDLE_MM     = 15.5    # fiber bundle exit
HOLE_STRUCTURAL_MM = 3.2     # 2 mm structural cables → 3.2 mm guide
HOLE_CORD_MM       = 9.5     # wall-wart cord
HOLE_FIBER_MM      = 2.2     # 2 mm bit through star tip node

# Canopy jig geometry.
CANOPY_JIG_WALL      = 4.0    # mm side-wall thickness
CANOPY_JIG_TOP       = 6.0    # mm top-wall thickness (where bit slips)
CANOPY_JIG_SKIRT     = 8.0    # mm skirt depth (drops below jig's top)
CANOPY_JIG_FIT_CLEAR = 0.6    # mm slip-fit clearance on plate OD
STRUCTURAL_HOLE_OFFSET_MM = 18.0   # ± offset of the 2 structural-cable holes
CORD_HOLE_OFFSET_MM       = -25.0  # offset from rim toward centre for cord hole

# Star-tip jig geometry.
STAR_JIG_OD       = 26.0      # mm outer diameter
STAR_JIG_HEIGHT   = 14.0      # mm total height
STAR_JIG_DIMPLE_C = 0.5       # mm clearance on hemispherical dimple
# Dimple depth is constrained by rod-clearance.  Rod top sits at
# ``STAR_ROD_RADIUS * SCALE`` (~ 4.7 mm) above the node equator; the jig's
# bottom face has to sit clear of that band.  A 2 mm dimple gives ~ 1.5 mm
# rod clearance — small registration cup but no scraping.
STAR_JIG_DIMPLE_D = 2.0       # mm dimple depth into the jig's bottom face


# ---------------------------------------------------------------------------
# Builders
# ---------------------------------------------------------------------------

def _y_axis_cylinder(radius: float, height: float, sections: int = 32):
    """Cylinder oriented along the Y axis (default trimesh cylinder is
    along Z; the canopy jig is built with Z up, so this isn't used —
    kept for clarity in case we add features later)."""
    cyl = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)
    cyl.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1.0, 0.0, 0.0]))
    return cyl


def build_canopy_drill_jig() -> trimesh.Trimesh:
    """Slip-on cap that drops over the cast canopy plate.  Built with Z up
    (printer convention)."""
    plate_od = CANOPY_OD_MM
    plate_h  = CANOPY_THICK_MM

    # Cup outer / inner diameters.
    cup_od = plate_od + 2.0 * (CANOPY_JIG_FIT_CLEAR + CANOPY_JIG_WALL)
    cup_id = plate_od + 2.0 * CANOPY_JIG_FIT_CLEAR
    skirt  = CANOPY_JIG_SKIRT
    top    = CANOPY_JIG_TOP
    cup_h  = skirt + top

    # Outer body (closed cylinder).
    outer = trimesh.creation.cylinder(radius=cup_od / 2.0, height=cup_h, sections=96)

    # Inner cavity (open at bottom) — subtract a cylinder that occupies
    # the lower ``skirt`` mm of the cup, plus a small overshoot below.
    cavity_h = skirt + 1.0  # 1 mm overshoot past the bottom for clean boolean
    cavity = trimesh.creation.cylinder(radius=cup_id / 2.0, height=cavity_h, sections=96)
    # Position so cavity bottom sits 0.5 mm below the cup bottom (overshoot)
    # and the cavity top sits ``top`` above the cup bottom.
    cavity.apply_translation([0.0, 0.0, -cup_h / 2.0 + cavity_h / 2.0 - 0.5])

    body = trimesh.boolean.difference([outer, cavity], engine="manifold")

    # Through-holes in the top wall.  Each hole is a Z-axis cylinder
    # tall enough to punch all the way through the jig.
    holes = [
        ("fiber bundle (15 mm)",            0.0,                          0.0, HOLE_BUNDLE_MM),
        ("structural cable to 13-star",     STRUCTURAL_HOLE_OFFSET_MM,    0.0, HOLE_STRUCTURAL_MM),
        ("structural cable to pentagram",  -STRUCTURAL_HOLE_OFFSET_MM,    0.0, HOLE_STRUCTURAL_MM),
        ("wall-wart cord",                  plate_od / 2.0 + CORD_HOLE_OFFSET_MM,
                                                                          0.0, HOLE_CORD_MM),
    ]
    for label, hx, hy, hd in holes:
        cutter = trimesh.creation.cylinder(
            radius=hd / 2.0,
            height=cup_h * 1.5,
            sections=32,
        )
        cutter.apply_translation([hx, hy, 0.0])
        body = trimesh.boolean.difference([body, cutter], engine="manifold")

    # Translate so the lowest face sits at Z = 0 (build-plate datum).
    body.apply_translation([0.0, 0.0, -body.bounds[0][2]])

    return body


def build_star_tip_drill_jig() -> trimesh.Trimesh:
    """Small cup that drops over a star tip node sphere from above.

    The cup's bottom face sits clear of the rod band entering the node
    (rods are ~ 14 mm OD post-cast and exit horizontally from the node
    equator).  A small hemispherical dimple in the top of the bottom-
    cavity self-centers the jig on the upper portion of the node sphere.
    Through-hole guides a 2 mm bit vertically through the node centre.
    """
    od = STAR_JIG_OD
    h  = STAR_JIG_HEIGHT

    # Solid body.
    body = trimesh.creation.cylinder(radius=od / 2.0, height=h, sections=64)

    # Hemispherical dimple — registers on the very top of the node sphere
    # so the jig sits clear of the horizontal rods at the node equator.
    dimple_r = STAR_NODE_R_MM + STAR_JIG_DIMPLE_C
    dimple = trimesh.creation.icosphere(subdivisions=3, radius=dimple_r)
    # Position so the dimple just dips ``STAR_JIG_DIMPLE_D`` mm into the
    # jig's bottom face (small registration cup, big rod clearance).
    dimple.apply_translation([0.0, 0.0,
                              -h / 2.0 - dimple_r + STAR_JIG_DIMPLE_D])
    body = trimesh.boolean.difference([body, dimple], engine="manifold")

    # 2 mm guide hole, full-height through-hole at centre.
    hole = trimesh.creation.cylinder(
        radius=HOLE_FIBER_MM / 2.0,
        height=h * 1.5,
        sections=24,
    )
    body = trimesh.boolean.difference([body, hole], engine="manifold")

    # Z = 0 at lowest face for printing.
    body.apply_translation([0.0, 0.0, -body.bounds[0][2]])

    return body


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--out-dir", default="jigs",
                    help="output directory (default: jigs/)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    from all_polyhedra import STAR_ROD_RADIUS
    rod_top_above_eq = STAR_ROD_RADIUS * SCALE
    dimple_r = STAR_NODE_R_MM + STAR_JIG_DIMPLE_C
    cavity_z = -STAR_JIG_HEIGHT / 2.0 - dimple_r + STAR_JIG_DIMPLE_D
    node_z   = cavity_z - STAR_JIG_DIMPLE_C
    jig_bottom_z = -STAR_JIG_HEIGHT / 2.0
    rod_clearance = jig_bottom_z - (node_z + rod_top_above_eq)

    print(f"Drill-jig export (chandelier scale {SCALE:.3f}x)")
    print(f"  Canopy plate OD post-cast:  {CANOPY_OD_MM:.1f} mm")
    print(f"  Canopy plate thickness:     {CANOPY_THICK_MM:.1f} mm")
    print(f"  Star tip node OD post-cast: {STAR_NODE_R_MM * 2:.1f} mm")
    print(f"  Star rod OD post-cast:      {rod_top_above_eq * 2:.1f} mm")
    print(f"  Star-tip jig rod clearance: {rod_clearance:.2f} mm")
    print(f"  Output dir:                 {args.out_dir}/")
    print()

    print("Building canopy_drill_jig.stl ...")
    cj = build_canopy_drill_jig()
    cj_ext = cj.bounds[1] - cj.bounds[0]
    cj_path = os.path.join(args.out_dir, "canopy_drill_jig.stl")
    cj.export(cj_path)
    cj_wt = "OK" if cj.is_watertight else "open"
    print(f"  envelope:    {cj_ext[0]:.1f} x {cj_ext[1]:.1f} x {cj_ext[2]:.1f} mm  ({cj_wt})")
    print(f"  faces:       {len(cj.faces)}")
    print(f"  Saved {cj_path}")
    print()

    print("Building star_tip_drill_jig.stl ...")
    sj = build_star_tip_drill_jig()
    sj_ext = sj.bounds[1] - sj.bounds[0]
    sj_path = os.path.join(args.out_dir, "star_tip_drill_jig.stl")
    sj.export(sj_path)
    sj_wt = "OK" if sj.is_watertight else "open"
    print(f"  envelope:    {sj_ext[0]:.1f} x {sj_ext[1]:.1f} x {sj_ext[2]:.1f} mm  ({sj_wt})")
    print(f"  faces:       {len(sj.faces)}")
    print(f"  Saved {sj_path}")
    print()

    print("Print recommendation: PETG, 3 perimeters, 25% infill.  Print")
    print("the star_tip jig 2-4 times (one per drill operation in flight).")
    print("See ASSEMBLY.md §6.5 for the full machining workflow.")


if __name__ == "__main__":
    main()
