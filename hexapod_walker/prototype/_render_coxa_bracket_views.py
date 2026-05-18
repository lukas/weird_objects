"""Render the coxa_bracket from top + iso views (solo), and an assembled
view that places the coxa_link "arm" on top of the bracket where the
printed horn adapter would mate, so we can visually verify clearance
between the bracket's new closed-top cap and the lifted arm.

Saves to:
  renders/coxa_bracket_new_top.png
  renders/coxa_bracket_new_iso.png
  renders/coxa_bracket_new_side.png
  renders/coxa_bracket_with_arm_iso.png
  renders/coxa_bracket_with_arm_side.png
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pyvista as pv
import trimesh

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as hp


OUT_DIR = os.path.join(THIS_DIR, "renders")

# Bracket-local Z of the printed horn-adapter TOP face = where the
# coxa_link's hub origin lands in the assembly.  Mirrors the stack
# math used by build_prototype_assembly._build_leg_assembly:
#   chassis-plate top -> well rim                z = 0
#   well rim          -> body top                + (SERVO_BODY_H - WELL_RIM_Z)
#   body top          -> gear-stack top          + SERVO_OUTPUT_H
#   gear-stack top    -> plastic-horn top        + PLASTIC_HORN_H
#   plastic-horn top  -> printed-adapter top     + HORN_ADAPTER_T
PLASTIC_HORN_H = 5.0
YAW_OUTPUT_Z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                + hp.SERVO_OUTPUT_H
                + PLASTIC_HORN_H
                + hp.HORN_ADAPTER_T)


def _bracket_mesh() -> trimesh.Trimesh:
    return hp.make_coxa_bracket()


def _coxa_link_in_bracket_frame() -> trimesh.Trimesh:
    """coxa_link mesh placed in the BRACKET's local frame: hub origin
    at (0, 0, YAW_OUTPUT_Z) on the yaw axis, arm extending in +X
    outboard, hip-pitch axis along +Y -- i.e. the standing-pose
    yaw=0 placement used in the full leg assembly."""
    cl = hp.make_coxa_link()
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, YAW_OUTPUT_Z]
    cl.apply_transform(T)
    return cl


def _horn_adapter_in_bracket_frame() -> trimesh.Trimesh:
    """Printed horn adapter, sits between the plastic horn (below)
    and the coxa_link's hub (above).  Adapter bottom face at
    bracket-z = YAW_OUTPUT_Z - HORN_ADAPTER_T."""
    ad = hp.make_servo_horn_adapter()
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, YAW_OUTPUT_Z - hp.HORN_ADAPTER_T]
    ad.apply_transform(T)
    return ad


def _plastic_horn_in_bracket_frame() -> trimesh.Trimesh:
    """Plastic 4-arm output horn (the part that ships with the servo),
    sits between the servo spline tip (below) and the printed horn
    adapter (above).  Bottom at bracket-z = YAW_OUTPUT_Z -
    HORN_ADAPTER_T - PLASTIC_HORN_H."""
    horn = hp.make_servo_horn()
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0,
                YAW_OUTPUT_Z - hp.HORN_ADAPTER_T - PLASTIC_HORN_H]
    horn.apply_transform(T)
    return horn


def _set_camera(plotter: pv.Plotter, view: str) -> None:
    if view == "top":
        plotter.camera_position = "xy"
        plotter.camera.zoom(1.2)
    elif view == "iso":
        plotter.camera_position = [
            (120.0, -100.0, 90.0),
            (0.0, 0.0, -5.0),
            (0.0, 0.0, 1.0),
        ]
        plotter.camera.zoom(1.05)
    elif view == "side":
        plotter.camera_position = "xz"
        plotter.camera.zoom(1.2)
    elif view == "with_arm_iso":
        plotter.camera_position = [
            (140.0, -160.0, 60.0),
            (15.0, 0.0, 30.0),
            (0.0, 0.0, 1.0),
        ]
        plotter.camera.zoom(0.85)
    elif view == "with_arm_side":
        plotter.camera_position = [
            (0.0, -220.0, 25.0),
            (15.0, 0.0, 25.0),
            (0.0, 0.0, 1.0),
        ]
        plotter.camera.zoom(0.85)
    elif view == "with_arm_front":
        plotter.camera_position = [
            (-220.0, 0.0, 25.0),
            (0.0, 0.0, 25.0),
            (0.0, 0.0, 1.0),
        ]
        plotter.camera.zoom(0.85)
    else:
        raise ValueError(view)


def _render_solo(view: str, out_path: str,
                  show_edges: bool = False) -> None:
    """Render the bracket on its own.

    smooth_shading=False so PyVista does NOT interpolate normals across
    the boolean mesh's triangle edges.  Smooth shading on the boolean
    output makes the flat top slab look like it has fake diagonal
    pyramidal creases between the chassis bolt holes -- visually
    suggesting the top is "open" or "warped" when in fact it is a
    completely solid 2.5 mm flange cap (the only true hole through
    the cap is the Phi 11 mm gear clearance at the yaw axis).
    """
    cb = _bracket_mesh()
    plotter = pv.Plotter(off_screen=True, window_size=(1024, 768),
                         lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(pv.wrap(cb),
                     color=(0.55, 0.62, 0.70),
                     smooth_shading=False,
                     show_edges=show_edges,
                     edge_color=(0.20, 0.22, 0.25),
                     line_width=0.5,
                     specular=0.25, specular_power=20.0)
    _set_camera(plotter, view)
    plotter.show(screenshot=out_path)
    print(f"wrote {out_path}")


def _render_top_cutaway(out_path: str) -> None:
    """Render a side-on cross-section of the bracket cut at y = 0 (the
    yaw axis plane) so the closed 2.5 mm flange cap above the body
    pocket is unambiguously visible, with the Phi 11 mm gear hole
    through it as the only opening."""
    cb = _bracket_mesh()
    pv_mesh = pv.wrap(cb)
    cut = pv_mesh.clip(normal=(0, 1, 0), origin=(0.0, 0.5, 0.0),
                       invert=True)

    plotter = pv.Plotter(off_screen=True, window_size=(1024, 768),
                         lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(cut,
                     color=(0.55, 0.62, 0.70),
                     smooth_shading=False,
                     show_edges=True,
                     edge_color=(0.20, 0.22, 0.25),
                     line_width=0.5,
                     specular=0.25, specular_power=20.0)
    plotter.camera_position = [
        (0.0, -180.0, -5.0),
        (-10.0, 0.0, -5.0),
        (0.0, 0.0, 1.0),
    ]
    plotter.camera.zoom(1.1)
    # Annotate the closed-cap region
    plotter.add_text(
        "TOP CAP (closed; 2.5 mm of flange material above the body pocket)\n"
        "Only the Phi 11 mm gear-clearance hole pierces it (at yaw axis).",
        position="upper_left", font_size=10, color="black")
    plotter.show(screenshot=out_path)
    print(f"wrote {out_path}")


def _render_with_arm(view: str, out_path: str) -> None:
    cb = _bracket_mesh()
    arm = _coxa_link_in_bracket_frame()
    adapter = _horn_adapter_in_bracket_frame()
    plastic = _plastic_horn_in_bracket_frame()

    plotter = pv.Plotter(off_screen=True, window_size=(1280, 800),
                         lighting="three lights")
    plotter.set_background("#dadada")
    plotter.add_mesh(pv.wrap(cb),
                     color=(0.55, 0.62, 0.70),
                     smooth_shading=False,
                     specular=0.25, specular_power=20.0,
                     name="bracket")
    plotter.add_mesh(pv.wrap(plastic),
                     color=(0.15, 0.15, 0.18),
                     smooth_shading=False,
                     specular=0.15, specular_power=18.0,
                     name="plastic_horn")
    plotter.add_mesh(pv.wrap(adapter),
                     color=(0.95, 0.78, 0.30),
                     smooth_shading=False,
                     specular=0.3, specular_power=20.0,
                     name="horn_adapter")
    plotter.add_mesh(pv.wrap(arm),
                     color=(0.78, 0.48, 0.38),
                     smooth_shading=False,
                     specular=0.25, specular_power=20.0,
                     name="coxa_link")
    _set_camera(plotter, view)
    plotter.show(screenshot=out_path)
    print(f"wrote {out_path}")


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    for v in ("top", "iso", "side"):
        _render_solo(v, os.path.join(OUT_DIR, f"coxa_bracket_new_{v}.png"))
    _render_solo("iso",
                 os.path.join(OUT_DIR, "coxa_bracket_new_iso_edges.png"),
                 show_edges=True)
    _render_top_cutaway(os.path.join(OUT_DIR,
                                     "coxa_bracket_top_cutaway.png"))
    for v in ("with_arm_iso", "with_arm_side", "with_arm_front"):
        _render_with_arm(v, os.path.join(OUT_DIR, f"coxa_bracket_{v}.png"))


if __name__ == "__main__":
    main()
