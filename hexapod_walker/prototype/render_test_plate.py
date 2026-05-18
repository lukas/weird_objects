"""Quick OpenGL screenshot of the 3-part test print plate."""

from __future__ import annotations

import math
import os

import pyvista as pv
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
STL_PATH = os.path.join(HERE, "test_print_plate", "test_print_plate.stl")
OUT_PNG = os.path.join(HERE, "renders", "test_print_plate.png")


def main() -> None:
    if not os.path.isfile(STL_PATH):
        raise SystemExit(
            f"{STL_PATH} not found. Run make_test_print_plate.py first.")

    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)

    mesh = trimesh.load(STL_PATH)
    extents = mesh.extents
    cx, cy, cz = mesh.centroid

    plotter = pv.Plotter(off_screen=True, window_size=(1600, 1000),
                         lighting="three lights")
    plotter.set_background("#9eaab7")

    bed_w = 256.0
    bed = pv.Plane(center=(0, 0, -0.5),
                   direction=(0, 0, 1),
                   i_size=bed_w, j_size=bed_w)
    plotter.add_mesh(bed, color=(0.30, 0.32, 0.36), opacity=1.0)
    plotter.add_mesh(pv.wrap(mesh), color=(0.78, 0.85, 0.92),
                     smooth_shading=False, specular=0.2)

    span = max(extents[0], extents[1], extents[2])
    dist = max(span * 1.4, 220.0)
    az = math.radians(40.0)
    el = math.radians(35.0)
    fx, fy, fz = cx, cy, cz * 0.4
    pos = (
        fx + dist * math.cos(el) * math.cos(az),
        fy + dist * math.cos(el) * math.sin(az),
        fz + dist * math.sin(el),
    )
    plotter.camera_position = [pos, (fx, fy, fz), (0, 0, 1)]
    plotter.screenshot(OUT_PNG)
    print(f"Wrote {OUT_PNG}")


if __name__ == "__main__":
    main()
