"""Render the servo cradles with the SERVO BODY DROPPED IN so the
RECTANGULAR +X WIRE-EXIT BOOT is visible inside the cradle's matching
slot.  Saves three PNGs under ``renders/wire_exit_*.png``:

    wire_exit_servo_body_iso.png    -- the servo body alone (shows the
                                       new rectangular +X wire boot)
    wire_exit_coxa_bracket.png      -- bracket cradle + servo
    wire_exit_coxa_link.png         -- coxa_link cradle + servo

Use these renders to confirm the wire-exit orientation BEFORE printing
the parts.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pyvista as pv
from trimesh.transformations import rotation_matrix

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.join(THIS_DIR, "renders")
os.makedirs(OUT_DIR, exist_ok=True)


def _servo_body_with_highlighted_boot():
    """Return ``(body_without_boot, boot)`` so the boot can be drawn in
    a contrasting colour.
    """
    body = hp._box((hp.SERVO_BODY_W, hp.SERVO_BODY_D, hp.SERVO_BODY_H),
                    center=(0, 0, hp.SERVO_BODY_H / 2.0))
    flange = (hp.SERVO_TAB_W - hp.SERVO_BODY_W) / 2.0
    tab_l = hp._box((flange, hp.SERVO_BODY_D, hp.SERVO_TAB_T),
                     center=(-(hp.SERVO_BODY_W + flange) / 2.0, 0.0,
                             hp.SERVO_TAB_Z))
    tab_r = hp._box((flange, hp.SERVO_BODY_D, hp.SERVO_TAB_T),
                     center=(+(hp.SERVO_BODY_W + flange) / 2.0, 0.0,
                             hp.SERVO_TAB_Z))
    body = hp._union(body, tab_l, tab_r)
    hub = hp._cyl(hp.SERVO_OUTPUT_OD / 2.0, hp.SERVO_OUTPUT_H)
    hub.apply_translation([hp.SERVO_OUTPUT_X, 0,
                             hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H / 2.0])
    spline = hp._cyl(hp.SERVO_SPLINE_OD / 2.0, hp.SERVO_OUTPUT_H + 1.5)
    spline.apply_translation([hp.SERVO_OUTPUT_X, 0,
                                hp.SERVO_BODY_H + (hp.SERVO_OUTPUT_H + 1.5) / 2.0])
    body = hp._union(body, hub, spline)

    boot_x_centre = hp.SERVO_BODY_W / 2.0 + hp.WIRE_BOOT_PROTRUSION / 2.0
    boot_z_centre = hp.WIRE_BOOT_Z_BASE + hp.WIRE_BOOT_H / 2.0
    boot = hp._box((hp.WIRE_BOOT_PROTRUSION, hp.WIRE_BOOT_W, hp.WIRE_BOOT_H),
                    center=(boot_x_centre, 0.0, boot_z_centre))
    return body, boot


def _place(R4, t):
    body, boot = _servo_body_with_highlighted_boot()
    if R4 is not None:
        body.apply_transform(R4)
        boot.apply_transform(R4)
    body.apply_translation(t)
    boot.apply_translation(t)
    return body, boot


def _render(part, body, boot, out_path, cam_offset_dirs, title):
    pv_part = pv.wrap(part)
    pv_body = pv.wrap(body)
    pv_boot = pv.wrap(boot)

    plotter = pv.Plotter(off_screen=True, window_size=(1280, 960),
                           lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(pv_part, color=(0.0, 0.55, 0.55), opacity=0.55,
                      smooth_shading=False)
    plotter.add_mesh(pv_body, color=(0.10, 0.10, 0.12), smooth_shading=False)
    plotter.add_mesh(pv_boot, color=(0.95, 0.45, 0.10),
                      smooth_shading=False)

    pb = np.asarray(part.bounds)
    bb = np.asarray(body.bounds)
    lo = np.minimum(pb[0], bb[0])
    hi = np.maximum(pb[1], bb[1])
    centre = 0.5 * (lo + hi)
    span = float(np.max(hi - lo)) * 1.5

    direction = np.array(cam_offset_dirs, dtype=float)
    direction = direction / np.linalg.norm(direction)
    cam_pos = centre + direction * span

    plotter.camera_position = [tuple(cam_pos), tuple(centre), (0, 0, 1)]
    plotter.camera.zoom(1.0)
    plotter.add_text(title, position="upper_left", font_size=12, color="black")
    plotter.add_text("ORANGE = wire boot on body's +X face",
                      position="lower_left", font_size=10, color="black")
    plotter.show(screenshot=out_path)
    plotter.close()
    print(f"  wrote {os.path.relpath(out_path, THIS_DIR)}")


def main(argv=None):
    body_alone, boot_alone = _servo_body_with_highlighted_boot()
    plotter = pv.Plotter(off_screen=True, window_size=(1024, 768),
                           lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(pv.wrap(body_alone), color=(0.10, 0.10, 0.12),
                       smooth_shading=False)
    plotter.add_mesh(pv.wrap(boot_alone), color=(0.95, 0.45, 0.10),
                       smooth_shading=False)
    plotter.add_text("Servo body: output gear is at +X (top); "
                       "rectangular wire boot exits the +X SHORT face",
                       position="upper_left", font_size=11, color="black")
    bb = body_alone.bounds
    centre = 0.5 * (bb[0] + bb[1])
    span = float(np.max(bb[1] - bb[0])) * 1.5
    direction = np.array([0.6, -1.0, 0.6])
    direction /= np.linalg.norm(direction)
    plotter.camera_position = [tuple(centre + direction * span),
                                  tuple(centre), (0, 0, 1)]
    out = os.path.join(OUT_DIR, "wire_exit_servo_body_iso.png")
    plotter.show(screenshot=out)
    plotter.close()
    print(f"  wrote {os.path.relpath(out, THIS_DIR)}")

    # Coxa bracket: well is at bracket-local x = -SERVO_OUTPUT_X.
    bracket = hp.make_coxa_bracket()
    bracket_body, bracket_boot = _place(
        None,
        np.array([-hp.SERVO_OUTPUT_X, 0.0, 0.0]),
    )
    _render(bracket, bracket_body, bracket_boot,
              os.path.join(OUT_DIR, "wire_exit_coxa_bracket.png"),
              cam_offset_dirs=(1.0, -0.8, 0.6),
              title="coxa_bracket + servo: boot exits OUTBOARD (+X)")

    # Coxa link: same R + t as the verification cradle.
    coxa = hp.make_coxa_link()
    R_link = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    t_link = np.array([
        hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
        hp.COXA_HIP_DROP,
    ])
    coxa_body, coxa_boot = _place(R_link, t_link)
    _render(coxa, coxa_body, coxa_boot,
              os.path.join(OUT_DIR, "wire_exit_coxa_link.png"),
              cam_offset_dirs=(0.8, -1.2, 0.6),
              title="coxa_link + servo: boot exits +X (toward femur joint)")

    print()
    print("Done.  Three wire-exit verification renders are in renders/.")


if __name__ == "__main__":
    main()
