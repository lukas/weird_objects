"""Render the coxa_link from a side-isometric angle matching the slicer
view the user posted, so we can confirm visually where the thin sheet
sits and verify a fix after geometry edits.

Saves to ``renders/coxa_link_flimsy_view.png``.
"""

from __future__ import annotations

import math
import os
import sys

import pyvista as pv

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as hp


OUT_DIR = os.path.join(THIS_DIR, "renders")
OUT_PATH = os.path.join(OUT_DIR, "coxa_link_flimsy_view.png")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    cl = hp.make_coxa_link()
    pv_mesh = pv.wrap(cl)

    plotter = pv.Plotter(
        off_screen=True,
        window_size=(1024, 640),
        lighting="three lights",
    )
    plotter.set_background("#dadada")
    plotter.add_mesh(
        pv_mesh,
        color=(0.0, 0.55, 0.55),
        smooth_shading=True,
        specular=0.25,
        specular_power=20.0,
    )

    bb = cl.bounds
    cx = 0.5 * (bb[0][0] + bb[1][0])
    cy = 0.5 * (bb[0][1] + bb[1][1])
    cz = 0.5 * (bb[0][2] + bb[1][2])
    span = max(
        bb[1][0] - bb[0][0],
        bb[1][1] - bb[0][1],
        bb[1][2] - bb[0][2],
    )

    # Side-iso angle: look from +X +Y -Z so we see the long side of the
    # arm slab, hub on the left, well on the right -- close to the user's
    # slicer screenshot.
    az = math.radians(-65.0)
    el = math.radians(8.0)
    dist = span * 2.2

    fx, fy, fz = cx, cy, cz
    pos = (
        fx + dist * math.cos(el) * math.cos(az),
        fy + dist * math.cos(el) * math.sin(az),
        fz + dist * math.sin(el),
    )
    plotter.camera_position = [pos, (fx, fy, fz), (0, 0, 1)]
    plotter.screenshot(OUT_PATH)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
