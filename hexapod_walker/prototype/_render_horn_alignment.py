"""Render the servo-horn / adapter / coxa-link-hub stack from above so the
4 M3 bolt holes can be visually confirmed to line up.

Writes:
    renders/horn_alignment_top.png   -- top-down, all 3 layers overlaid
    renders/horn_alignment_iso.png   -- slight tilt so you can see the
                                         stack profile

Use this after any change to HORN_BOLT_PCD / HORN_BOLT_ANGLES_RAD to
verify the rotational alignment by eye.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pyvista as pv

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.join(THIS_DIR, "renders")
os.makedirs(OUT_DIR, exist_ok=True)


def main(argv=None):
    horn = hp.make_servo_horn()                            # bottom (Z 0..2 mm)
    adapter = hp.make_servo_horn_adapter()                 # middle (Z 2..6 mm)
    adapter.apply_translation([0, 0, 2.0])

    # Just the hub disc of the coxa link -- isolating it keeps the
    # stack render tight in XY.
    coxa = hp.make_coxa_link()
    # Move the coxa link down so its hub bolt-mating face sits on the
    # adapter top face (Z = 6 mm).  COXA_LIFT is the link's pillar
    # height above the hub bolt face.
    coxa.apply_translation([0, 0, 6.0 - hp.COXA_LIFT])

    bolt_markers = []
    for a in hp.HORN_BOLT_ANGLES_RAD:
        marker = pv.Cylinder(
            center=(hp.HORN_BOLT_PCD / 2.0 * np.cos(a),
                     hp.HORN_BOLT_PCD / 2.0 * np.sin(a),
                     6.0 + hp.COXA_LIFT / 2.0),
            direction=(0, 0, 1),
            radius=hp.HORN_BOLT_OD / 2.0 + 0.2,
            height=6.0 + hp.COXA_LIFT,
        )
        bolt_markers.append(marker)

    plotter = pv.Plotter(off_screen=True, window_size=(1280, 960),
                          lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(pv.wrap(horn),    color=(0.1, 0.1, 0.12),
                       smooth_shading=False)
    plotter.add_mesh(pv.wrap(adapter), color=(0.2, 0.55, 0.85),
                       opacity=0.55, smooth_shading=False)
    plotter.add_mesh(pv.wrap(coxa),    color=(0.0, 0.55, 0.55),
                       opacity=0.40, smooth_shading=False)
    for marker in bolt_markers:
        plotter.add_mesh(marker, color=(0.95, 0.35, 0.10),
                          opacity=0.55, smooth_shading=False)
    plotter.add_text(
        "Horn (black) + adapter (blue) + coxa-link hub (teal): "
        "M3 bolt threads (orange) at HORN_BOLT_PCD = 20.8 mm, "
        "aligned with X-horn arms",
        position="upper_left", font_size=10, color="black")

    centre = (0.0, 0.0, 6.0)
    span = max(hp.HORN_ADAPTER_OD, 36.0)
    plotter.camera_position = [(0, 0, span * 4), centre, (0, 1, 0)]
    out_top = os.path.join(OUT_DIR, "horn_alignment_top.png")
    plotter.show(screenshot=out_top)
    plotter.close()
    print(f"  wrote {os.path.relpath(out_top, THIS_DIR)}")

    plotter = pv.Plotter(off_screen=True, window_size=(1280, 960),
                          lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(pv.wrap(horn),    color=(0.1, 0.1, 0.12),
                       smooth_shading=False)
    plotter.add_mesh(pv.wrap(adapter), color=(0.2, 0.55, 0.85),
                       opacity=0.6, smooth_shading=False)
    plotter.add_mesh(pv.wrap(coxa),    color=(0.0, 0.55, 0.55),
                       opacity=0.4, smooth_shading=False)
    for marker in bolt_markers:
        plotter.add_mesh(marker, color=(0.95, 0.35, 0.10),
                          opacity=0.55, smooth_shading=False)
    plotter.add_text(
        "Stack ISO: M3 bolts pass straight through all 3 layers",
        position="upper_left", font_size=11, color="black")
    plotter.camera_position = [(span * 2.5, -span * 2.5, span * 2.0),
                                  centre, (0, 0, 1)]
    out_iso = os.path.join(OUT_DIR, "horn_alignment_iso.png")
    plotter.show(screenshot=out_iso)
    plotter.close()
    print(f"  wrote {os.path.relpath(out_iso, THIS_DIR)}")


if __name__ == "__main__":
    main()
