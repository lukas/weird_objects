#!/usr/bin/env python3
"""Headless PyVista preview of the joint_viz BuildViz assembly.

Loads the STLs that ``joint_viz_build.py`` emitted and renders an iso
screenshot so the bearing-sandwich joint can be eyeballed without opening
the interactive BuildViz web viewer.
"""
from __future__ import annotations

import json
from pathlib import Path

import pyvista as pv

_HERE = Path(__file__).resolve().parent
OUT_DIR = _HERE.parent / "joint_viz"


def main() -> None:
    scene = json.loads((OUT_DIR / "scene.json").read_text())
    mesh_url = {m["id"]: m["url"] for m in scene["meshes"]}

    pl = pv.Plotter(off_screen=True, window_size=(1500, 1000))
    pl.set_background("white", top="lightgray")
    for inst in scene["instances"]:
        url = mesh_url[inst["meshId"]]
        mesh = pv.read(OUT_DIR / url)
        translucent = {"yoke": 0.45, "housing": 0.35}
        pl.add_mesh(mesh, color=inst["color"], smooth_shading=True,
                    opacity=translucent.get(inst["partType"], 1.0),
                    show_edges=False)
    pl.add_axes()
    pl.camera_position = "iso"
    pl.camera.azimuth = 35
    pl.camera.elevation = 12
    pl.reset_camera()
    out = OUT_DIR / "joint_viz_preview.png"
    pl.screenshot(str(out))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
