"""Static preview render of the prototype walker (no Blender).

Uses the category STLs in prototype_assembly/ (build with
build_prototype_assembly.py) and PyVista for an off-screen OpenGL shot.

  ./run.sh hexapod_walker/prototype/render_prototype_preview.py

For a photoreal Cycles image, install Blender and run render_prototype.sh.
"""

from __future__ import annotations

import math
import os
import sys

import pyvista as pv
import trimesh
from trimesh.transformations import rotation_matrix

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(THIS_DIR, "prototype_assembly")
OUT_DIR = os.path.join(THIS_DIR, "renders")
OUT_PATH = os.path.join(OUT_DIR, "prototype_preview.png")

# Approximate render_prototype.sh material colours (hex)
COLORS = {
    "frame":   "#dfe4e7",
    "motors":  "#2a2c30",
    "battery": "#262638",
    "soft":    "#181818",
}


def _hex_to_rgb1(s: str) -> tuple:
    s = s.lstrip("#")
    return tuple(int(s[i : i + 2], 16) / 255.0 for i in (0, 2, 4))


def _load_category(name: str) -> trimesh.Trimesh:
    path = os.path.join(ASSETS, f"{name}.stl")
    if not os.path.isfile(path):
        sys.exit(f"Missing {path} — run build_prototype_assembly.py first.")
    m = trimesh.load(path)
    if not isinstance(m, trimesh.Trimesh):
        m = trimesh.util.concatenate(
            tuple(g for g in m.geometry.values() if len(g.faces) > 0)
        )
    m.apply_scale(0.001)
    m.apply_transform(rotation_matrix(math.pi / 2.0, [1.0, 0.0, 0.0]))
    return m


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    plotter = pv.Plotter(
        off_screen=True,
        window_size=(1920, 1200),
        lighting="three lights",
    )
    plotter.set_background("#b8c5d4")

    xmin = ymin = zmin = float("inf")
    xmax = ymax = zmax = float("-inf")
    for key in ("frame", "motors", "battery", "soft"):
        tm = _load_category(key)
        bb = tm.bounds
        lo, hi = bb[0], bb[1]
        xmin = min(xmin, lo[0])
        xmax = max(xmax, hi[0])
        ymin = min(ymin, lo[1])
        ymax = max(ymax, hi[1])
        zmin = min(zmin, lo[2])
        zmax = max(zmax, hi[2])
        pv_mesh = pv.wrap(tm)
        plotter.add_mesh(
            pv_mesh,
            color=_hex_to_rgb1(COLORS[key]),
            smooth_shading=False,
            specular=0.35 if key == "frame" else 0.15,
            specular_power=30.0,
        )

    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    span = max(xmax - xmin, ymax - ymin, zmax - zmin, 0.05)
    floor = pv.Plane(
        center=(cx, cy, zmin - 0.002),
        direction=(0, 0, 1),
        i_size=span * 3.5,
        j_size=span * 3.5,
    )
    plotter.add_mesh(floor, color=_hex_to_rgb1("#cfcdc6"), opacity=1.0)

    # Camera: 3/4 view, similar defaults to render_prototype.sh (metres, Z up)
    dist = max(span * 1.25, 0.55)
    az = math.radians(35.0)
    el = math.radians(12.0)
    fx, fy, fz = cx, cy, zmin + 0.06 + (zmax - zmin) * 0.28

    pos = (
        fx + dist * math.cos(el) * math.cos(az),
        fy + dist * math.cos(el) * math.sin(az),
        fz + dist * math.sin(el) + 0.10,
    )
    plotter.camera_position = [pos, (fx, fy, fz), (0, 0, 1)]
    plotter.screenshot(OUT_PATH)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
