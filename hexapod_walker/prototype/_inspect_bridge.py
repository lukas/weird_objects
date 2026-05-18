"""Quantify the cross-sectional area along the spar/bridge between the
horn-mating yoke (hub) and the servo cradle (well) for each leg link.

For each link we slice the mesh at many X positions (X = the spar's long
axis in link-local coords) and report the area of the slice's YZ cross
section, plus its YZ extents. The minimum area along the "bridge" region
(between the yoke and the cradle) tells us how thin the load path is.
"""

from __future__ import annotations
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import hexapod_prototype as hp


def scan_x_cross_sections(mesh, x_min: float, x_max: float, n: int = 121):
    """For each x in linspace(x_min, x_max, n), slice the mesh with the plane
    x = x_i (normal = +X) and compute (area, y_extent, z_extent) of the
    resulting 2-D polygon."""
    xs = np.linspace(x_min, x_max, n)
    rows = []
    for x in xs:
        section = mesh.section(plane_origin=[float(x), 0.0, 0.0],
                               plane_normal=[1.0, 0.0, 0.0])
        if section is None:
            rows.append((float(x), 0.0, 0.0, 0.0))
            continue
        planar, _ = section.to_planar()
        # area is signed sum across polygons inside planar
        try:
            area = float(planar.area)
        except Exception:
            area = 0.0
        if planar.vertices.size == 0:
            rows.append((float(x), 0.0, 0.0, 0.0))
            continue
        ymin, ymax = planar.vertices[:, 0].min(), planar.vertices[:, 0].max()
        zmin, zmax = planar.vertices[:, 1].min(), planar.vertices[:, 1].max()
        y_ext = float(ymax - ymin)
        z_ext = float(zmax - zmin)
        rows.append((float(x), area, y_ext, z_ext))
    return rows


def summarise(name: str, rows, x_window=None):
    print(f"\n=== {name}: cross-section vs X ===")
    if x_window is not None:
        rows = [r for r in rows if x_window[0] <= r[0] <= x_window[1]]
    print(f"  {'x [mm]':>8s}  {'area [mm^2]':>11s}  "
          f"{'y_ext [mm]':>10s}  {'z_ext [mm]':>10s}")
    # Print every 4th row to keep output legible, plus min row.
    for i, r in enumerate(rows):
        x, a, y, z = r
        if i % 4 == 0:
            print(f"  {x:8.2f}  {a:11.2f}  {y:10.2f}  {z:10.2f}")
    # Find min area among nonzero rows (skip x positions where the mesh
    # has no cross-section).
    nonzero = [r for r in rows if r[1] > 1.0]
    if not nonzero:
        print("  (no slices with area > 1 mm^2)")
        return None
    rmin = min(nonzero, key=lambda r: r[1])
    print(f"  --- MIN: x = {rmin[0]:6.2f}  area = {rmin[1]:7.2f} mm^2  "
          f"y_ext = {rmin[2]:5.2f}  z_ext = {rmin[3]:5.2f} ---")
    return rmin


def main():
    print("Loading meshes ...")
    cl = hp.make_coxa_link()
    fl = hp.make_femur_link()
    cb = hp.make_coxa_bracket()
    print(f"  coxa_link    bounds = {cl.bounds.tolist()}")
    print(f"  femur_link   bounds = {fl.bounds.tolist()}")
    print(f"  coxa_bracket bounds = {cb.bounds.tolist()}")

    # ---------- coxa_link ----------
    # Hub spans x in [-17, 17].  Well outer envelope is at link-x in [-14, 44].
    # Inboard of x=-17 nothing exists; outboard of x=41 the arm ends.  The
    # "bridge" between the yoke and the cradle effectively lives in
    # x in [17, 41] (just outboard of the hub, before the well outer wall).
    cl_rows = scan_x_cross_sections(cl, -20.0, 45.0, n=131)
    summarise("coxa_link (full scan)", cl_rows)
    summarise("coxa_link (BRIDGE x in [17, 30])", cl_rows,
              x_window=(17.0, 30.0))
    # x in [30, 35] is the spar's previously-thinnest region (arm only,
    # no bridge member).  With the new gusset extending out to x = 41,
    # this is where the gusset's contribution to the bridge stiffness
    # is most visible.
    summarise("coxa_link (PREV-WEAK x in [30, 35])", cl_rows,
              x_window=(30.0, 35.0))
    summarise("coxa_link (TAIL x in [36, 41])", cl_rows,
              x_window=(36.0, 41.0))

    # ---------- femur_link ----------
    fl_rows = scan_x_cross_sections(fl, -2.0, 92.0, n=189)
    summarise("femur_link (full scan)", fl_rows)
    # Femur "bridge" -- between the hip pad (square at x=0, extends to
    # x=HIP_PAD_R=17) and the knee cradle insertion-slot region (which
    # starts at body_x_min = FEMUR_LENGTH - SERVO_OUTPUT_X - SERVO_BODY_W/2 - 1
    # = 90 - 10 - 20 - 1 = 59).
    summarise("femur_link (BRIDGE x in [17, 59])", fl_rows,
              x_window=(17.0, 59.0))

    # ---------- coxa_bracket ----------
    # Bracket flange is at x in [-30, 0], well below it.  No analogous
    # bridge -- but let's print the cross-sections anyway for ref.
    cb_rows = scan_x_cross_sections(cb, -35.0, 25.0, n=121)
    summarise("coxa_bracket (full scan)", cb_rows)


if __name__ == "__main__":
    main()
