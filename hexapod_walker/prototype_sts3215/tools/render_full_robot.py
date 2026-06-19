#!/usr/bin/env python3
"""Render whole-robot preview PNGs of the full_robot_viz scene (spider +
carapace dome).  Loads STLs by FILENAME from full_robot_viz/stl/ (the
scene.json mesh URLs are absolute /builds/... paths for the web hub, so we
use the mesh 'name' field instead).  Writes iso / front / side / top PNGs.
"""
from __future__ import annotations

import json
from pathlib import Path

import pyvista as pv

_HERE = Path(__file__).resolve().parent
SCENE_DIR = _HERE.parent / "full_robot_viz"
OUT_DIR = SCENE_DIR


def _plot(scene, meshes_by_id, *, az, el, name):
    pl = pv.Plotter(off_screen=True, window_size=(1600, 1200))
    pl.set_background("white", top="#cfd8e6")
    for inst in scene["instances"]:
        fn = meshes_by_id[inst["meshId"]]
        m = pv.read(SCENE_DIR / "stl" / fn)
        pl.add_mesh(m, color=inst.get("color", "#888888"),
                    smooth_shading=True, specular=0.2)
    pl.enable_shadows() if False else None
    pl.camera_position = "iso"
    pl.camera.azimuth = az
    pl.camera.elevation = el
    pl.reset_camera()
    out = OUT_DIR / name
    pl.screenshot(str(out))
    print(f"Wrote {out}")
    pl.close()


def _plot_cam(scene, meshes_by_id, *, cam, focal, up, name, parallel=False):
    pl = pv.Plotter(off_screen=True, window_size=(1600, 1200))
    pl.set_background("white", top="#cfd8e6")
    for inst in scene["instances"]:
        fn = meshes_by_id[inst["meshId"]]
        m = pv.read(SCENE_DIR / "stl" / fn)
        pl.add_mesh(m, color=inst.get("color", "#888888"),
                    smooth_shading=True, specular=0.25)
    pl.camera_position = [cam, focal, up]
    if parallel:
        pl.enable_parallel_projection()
    out = OUT_DIR / name
    pl.screenshot(str(out))
    print(f"Wrote {out}")
    pl.close()


def main() -> None:
    scene = json.loads((SCENE_DIR / "scene.json").read_text())
    meshes_by_id = {m["id"]: m["name"] for m in scene["meshes"]}
    # Iso overview (3/4 from the front-right so the eye face shows).
    _plot(scene, meshes_by_id, az=-125, el=18, name="robot_spider_iso.png")
    _plot(scene, meshes_by_id, az=90, el=8, name="robot_spider_side.png")
    _plot(scene, meshes_by_id, az=0, el=89, name="robot_spider_top.png")
    # Explicit anterior view: camera out on +X (forward), slightly above,
    # looking back at the dome so the 8-eye spider face reads clearly.
    _plot_cam(scene, meshes_by_id,
              cam=(360.0, 0.0, 175.0), focal=(0.0, 0.0, 135.0),
              up=(0.0, 0.0, 1.0), name="robot_spider_face.png")


if __name__ == "__main__":
    main()
