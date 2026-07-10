#!/usr/bin/env python3
"""Headless PyVista preview PNGs of the rideable_v1 BuildViz scene.

Loads every instance mesh from full_robot_viz/stl/, applies its (column-major)
scene transform, colours it, and writes iso / side / front / top PNGs next to
the scene so the design can be looked at without the web viewer.

Run:
    ./run.sh hexapod_walker/rideable_v1/tools/render_rideable.py
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pyvista as pv

_HERE = Path(__file__).resolve().parent
SCENE_DIR = _HERE.parent / "full_robot_viz"


def _matrix(flat) -> np.ndarray:
    if not flat or len(flat) != 16:
        return np.eye(4)
    return np.array(flat, dtype=float).reshape(4, 4).T   # column-major -> row-major


def _load_scene():
    scene = json.loads((SCENE_DIR / "scene.json").read_text())
    url_by_id = {m["id"]: m["url"] for m in scene["meshes"]}
    actors = []
    for inst in scene["instances"]:
        mesh = pv.read(SCENE_DIR / url_by_id[inst["meshId"]])
        M = _matrix(inst.get("transform"))
        if not np.allclose(M, np.eye(4)):
            mesh = mesh.transform(M, inplace=False)
        actors.append((mesh, inst.get("color", "#888888")))
    return scene, actors


def _plot(actors, *, position, name, azimuth=0.0, elevation=0.0, parallel=True):
    pl = pv.Plotter(off_screen=True, window_size=(1700, 1200))
    pl.set_background("white", top="#cfd8e6")
    for mesh, color in actors:
        pl.add_mesh(mesh, color=color, smooth_shading=True, specular=0.25)
    pl.add_axes()
    pl.camera_position = position
    if azimuth:
        pl.camera.azimuth = azimuth
    if elevation:
        pl.camera.elevation = elevation
    if parallel:
        pl.enable_parallel_projection()
    pl.reset_camera()
    out = SCENE_DIR / name
    pl.screenshot(str(out))
    pl.close()
    print(f"Wrote {out}")


def main() -> None:
    _scene, actors = _load_scene()
    _plot(actors, position="iso", azimuth=25, elevation=12, name="rideable_iso.png")
    _plot(actors, position="yz", name="rideable_side.png")   # look along +X (side)
    _plot(actors, position="xz", name="rideable_front.png")  # look along +Y (front)
    _plot(actors, position="xy", name="rideable_top.png")    # look down -Z (top)


if __name__ == "__main__":
    main()
