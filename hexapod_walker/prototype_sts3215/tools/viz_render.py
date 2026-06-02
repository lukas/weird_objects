#!/usr/bin/env python3
"""Headless PyVista preview of any BuildViz scene dir in this project.

Usage: python tools/viz_render.py <scene_dir> [out.png] [--azimuth A] [--elev E]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pyvista as pv

_HERE = Path(__file__).resolve().parent


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    scene_dir = (_HERE.parent / args[0]) if args else (_HERE.parent / "joint_viz")
    out = Path(args[1]) if len(args) > 1 else (scene_dir / "preview.png")
    az = 35.0
    el = 12.0
    for a in sys.argv[1:]:
        if a.startswith("--azimuth"):
            az = float(a.split("=")[1])
        if a.startswith("--elev"):
            el = float(a.split("=")[1])

    scene = json.loads((scene_dir / "scene.json").read_text())
    mesh_url = {m["id"]: m["url"] for m in scene["meshes"]}
    pl = pv.Plotter(off_screen=True, window_size=(1600, 1100))
    pl.set_background("white", top="lightgray")
    translucent = {"yoke": 0.4, "housing": 0.4}
    for inst in scene["instances"]:
        mesh = pv.read(scene_dir / mesh_url[inst["meshId"]])
        pl.add_mesh(mesh, color=inst["color"], smooth_shading=True,
                    opacity=translucent.get(inst["partType"], 1.0))
    pl.add_axes()
    pl.camera_position = "iso"
    pl.camera.azimuth = az
    pl.camera.elevation = el
    pl.reset_camera()
    pl.screenshot(str(out))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
