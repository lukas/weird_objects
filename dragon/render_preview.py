"""Render PNG previews of the generated articulated dragon STL.

Writes:
    preview.png         - side-on view (the long axis of the dragon visible)
    preview_iso.png     - 3/4 isometric perspective
    preview_joint.png   - real cross-section through one ball-and-socket joint
"""
from pathlib import Path

import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

HERE = Path(__file__).resolve().parent
STL = HERE / "stl" / "extra_long_dragon.stl"

mesh = pv.read(str(STL))
print("loaded:", mesh.n_points, "points,", mesh.n_cells, "cells")
print("bounds:", mesh.bounds)
center = np.array(mesh.center)
length = mesh.bounds[1] - mesh.bounds[0]
width = mesh.bounds[3] - mesh.bounds[2]
height = mesh.bounds[5] - mesh.bounds[4]


def make_plotter(window_size, light_floor=True) -> pv.Plotter:
    p = pv.Plotter(off_screen=True, window_size=window_size)
    p.add_mesh(
        mesh,
        color="#7fa05a",
        smooth_shading=True,
        specular=0.4,
        specular_power=15,
    )
    if light_floor:
        p.add_floor("-z", lighting=True, color="#cccccc", pad=0.2)
        p.enable_shadows()
    p.set_background("#222b33")
    return p


# ---------- top-down ----------
p = make_plotter(window_size=(2200, 800))
p.camera_position = [
    (center[0], center[1], center[2] + 1.6 * max(length, width)),
    tuple(center),
    (0, 1, 0),
]
p.camera.zoom(1.05)
p.screenshot(str(HERE / "preview.png"))
print("wrote", HERE / "preview.png")

# ---------- 3/4 isometric ----------
p = make_plotter(window_size=(1800, 800))
cam_pos = (
    center[0] + 0.2 * length,
    center[1] - 0.7 * length,
    center[2] + 0.45 * length,
)
p.camera_position = [cam_pos, tuple(center), (0, 0, 1)]
p.camera.zoom(1.05)
p.screenshot(str(HERE / "preview_iso.png"))
print("wrote", HERE / "preview_iso.png")

# ---------- joint cross-section ----------
# Take a thin slab in Y around the spine center, then look at it from -Y.
# Use slice_orthogonal -> pick a Y plane, then thicken with a clip box for solidity.
joint_x = mesh.bounds[0] + 0.30 * length

# Slice in Y plane to get a true cross-section curve
slice_mesh = mesh.slice(normal=(0, 1, 0), origin=(joint_x, center[1], center[2]))
print("slice n_cells:", slice_mesh.n_cells)

p = pv.Plotter(off_screen=True, window_size=(1600, 700))
# Show slice as thick lines
p.add_mesh(
    slice_mesh,
    color="#3a5a2c",
    line_width=3.0,
)
# Plus a ghost of the full mesh in light grey for context
p.add_mesh(
    mesh.clip_box(
        bounds=(
            joint_x - 70.0,
            joint_x + 70.0,
            mesh.bounds[2] - 1,
            mesh.bounds[3] + 1,
            mesh.bounds[4] - 1,
            mesh.bounds[5] + 1,
        ),
        invert=False,
    ),
    color="#cfd8c4",
    opacity=0.25,
    smooth_shading=True,
)
p.set_background("#f4f4ee")
p.camera_position = [
    (joint_x, center[1] - 220.0, center[2] + 5.0),
    (joint_x, center[1], center[2] + 5.0),
    (0, 0, 1),
]
p.camera.zoom(2.4)
p.screenshot(str(HERE / "preview_joint.png"))
print("wrote", HERE / "preview_joint.png")
