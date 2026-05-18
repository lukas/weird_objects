"""Labeled assembly-guide render of ONE prototype hexapod leg.

Produces a 3/4 isometric (plus top and side) PNG showing every printed
part in a distinct colour, the three DS3225 servo bodies in their wells,
the printed horn adapters at every joint, and a thin M3 x 16 hinge pin
at the foot clevis.  Labels are drawn on top via matplotlib with thin
leader lines pointing to each part's projected centroid.

Re-run after geometry edits:

    ./run.sh hexapod_walker/prototype/render_leg_assembly.py
"""

from __future__ import annotations

import math
import os
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pyvista as pv
import trimesh
import vtk
from trimesh.transformations import rotation_matrix

import hexapod_prototype as HP


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(THIS_DIR, "renders")
os.makedirs(OUT_DIR, exist_ok=True)

WINDOW = (1920, 1200)
BG = "#f1f3f5"

PLASTIC_HORN_H = 5.0

PART_COLORS: dict[str, str] = {
    "coxa_bracket":      "#2563eb",
    "coxa_link":         "#16a34a",
    "femur_link":        "#ea580c",
    "tibia_link":        "#9333ea",
    "foot_pad":          "#0f172a",
    "yaw_horn_adapter":  "#facc15",
    "hip_horn_adapter":  "#facc15",
    "knee_horn_adapter": "#facc15",
    "yaw_servo":         "#1f2937",
    "hip_servo":         "#1f2937",
    "knee_servo":        "#1f2937",
    "hinge_bolt":        "#a3a3a3",
}


def _hex_to_rgb1(s: str) -> tuple[float, float, float]:
    s = s.lstrip("#")
    return tuple(int(s[i : i + 2], 16) / 255.0 for i in (0, 2, 4))


# ---------------------------------------------------------------------------
# Geometry: build ONE leg in chassis frame, keyed by part name.
# Mirrors hexapod_prototype._leg_in_body_frame /
# build_prototype_assembly._build_leg but keeps each labelled part as a
# separate trimesh so the renderer can colour and label them individually.
# ---------------------------------------------------------------------------


def _align_z_to(axis: np.ndarray) -> np.ndarray:
    """4x4 rotation that maps +Z onto `axis` (unit vector)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    z = np.array([0.0, 0.0, 1.0])
    dot = float(np.clip(z @ axis, -1.0, 1.0))
    if dot > 1.0 - 1e-9:
        return np.eye(4)
    if dot < -1.0 + 1e-9:
        return rotation_matrix(math.pi, [1.0, 0.0, 0.0])
    n = np.cross(z, axis)
    n = n / np.linalg.norm(n)
    return rotation_matrix(math.acos(dot), n)


def build_leg_parts(leg_index: int = 0) -> tuple[dict[str, trimesh.Trimesh],
                                                  dict[str, np.ndarray]]:
    """Return (parts, landmarks) for ONE leg in chassis frame.

    Uses the same kinematic chain as build_prototype_assembly._build_leg
    (which itself mirrors hexapod_prototype._leg_in_body_frame).  We just
    don't union the per-part meshes -- each labelled part stays separate.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * math.pi / 3.0
    edge_mid = np.array([apothem * math.cos(a),
                         apothem * math.sin(a),
                         0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    yaw_output_z = ((HP.SERVO_BODY_H - HP.WELL_RIM_Z)
                    + HP.SERVO_OUTPUT_H
                    + PLASTIC_HORN_H
                    + HP.HORN_ADAPTER_T)
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    p_femur = math.radians(HP.STANCE_FEMUR_DEG)
    p_tibia = math.radians(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)

    R_a = rotation_matrix(a, [0, 0, 1])
    R_a_3 = R_a[:3, :3]

    arm_t = 6.0
    hip_drop = HP.COXA_HIP_DROP
    hip_joint_local = np.array([HP.COXA_LENGTH, 0.0, hip_drop])

    Ry_p_3  = rotation_matrix(p_femur, [0, 1, 0])[:3, :3]
    Ry_pt_3 = rotation_matrix(p_tibia, [0, 1, 0])[:3, :3]
    knee_joint_local = (hip_joint_local
                        + Ry_p_3 @ np.array([HP.FEMUR_LENGTH, 0.0, 0.0]))

    def to_world(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        mesh.apply_transform(R_a)
        mesh.apply_translation(yaw_output_world)
        return mesh

    parts: dict[str, trimesh.Trimesh] = {}

    # ---- coxa_bracket (chassis-fixed; bracket-local origin at edge_mid)
    cb = HP.make_coxa_bracket()
    cb.apply_transform(R_a)
    cb.apply_translation(edge_mid)
    parts["coxa_bracket"] = cb

    # ---- yaw servo body (hangs in the bracket well; body bottom at -WELL_RIM_Z)
    yaw_servo = HP.make_servo_body()
    yaw_servo.apply_translation([-HP.SERVO_OUTPUT_X, 0, -HP.WELL_RIM_Z])
    yaw_servo.apply_transform(R_a)
    yaw_servo.apply_translation(edge_mid)
    parts["yaw_servo"] = yaw_servo

    # ---- yaw horn adapter (above plastic horn above the spline tip)
    yaw_adapter = HP.make_servo_horn_adapter()
    yaw_adapter.apply_translation([
        0, 0,
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
            + HP.SERVO_OUTPUT_H
            + PLASTIC_HORN_H,
    ])
    yaw_adapter.apply_transform(R_a)
    yaw_adapter.apply_translation(edge_mid)
    parts["yaw_horn_adapter"] = yaw_adapter

    # ---- coxa_link (rotates with yaw; standing pose -> yaw = 0)
    cl = HP.make_coxa_link()
    cl.apply_transform(R_a)
    cl.apply_translation(yaw_output_world)
    parts["coxa_link"] = cl

    # ---- hip-pitch servo body (output along leg +Y)
    R_hip = rotation_matrix(-math.pi / 2.0, [1, 0, 0])
    hip_servo = HP.make_servo_body()
    hip_servo.apply_transform(R_hip)
    hip_servo.apply_translation([
        HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
        -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
        hip_drop,
    ])
    to_world(hip_servo)
    parts["hip_servo"] = hip_servo

    # ---- hip horn adapter (disc plane facing leg +Y)
    hip_adapter = HP.make_servo_horn_adapter()
    hip_adapter.apply_transform(R_hip)
    hip_adapter.apply_translation([HP.COXA_LENGTH, PLASTIC_HORN_H, hip_drop])
    to_world(hip_adapter)
    parts["hip_horn_adapter"] = hip_adapter

    # ---- femur (pitched about leg-local +Y by STANCE_FEMUR_DEG)
    fl = HP.make_femur_link()
    fl.apply_transform(rotation_matrix(p_femur, [0, 1, 0]))
    fl.apply_translation(hip_joint_local)
    to_world(fl)
    parts["femur_link"] = fl

    # ---- knee servo body (lives at femur's far end)
    knee_servo = HP.make_servo_body()
    knee_servo.apply_transform(R_hip)
    knee_servo.apply_translation([
        HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
        -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
        0,
    ])
    knee_servo.apply_transform(rotation_matrix(p_femur, [0, 1, 0]))
    knee_servo.apply_translation(hip_joint_local)
    to_world(knee_servo)
    parts["knee_servo"] = knee_servo

    # ---- knee horn adapter
    knee_adapter = HP.make_servo_horn_adapter()
    knee_adapter.apply_transform(R_hip)
    knee_adapter.apply_translation([HP.FEMUR_LENGTH, PLASTIC_HORN_H, 0])
    knee_adapter.apply_transform(rotation_matrix(p_femur, [0, 1, 0]))
    knee_adapter.apply_translation(hip_joint_local)
    to_world(knee_adapter)
    parts["knee_horn_adapter"] = knee_adapter

    # ---- tibia (rotates with knee-pitch)
    tl = HP.make_tibia_link()
    tl.apply_transform(rotation_matrix(p_tibia, [0, 1, 0]))
    tl.apply_translation(knee_joint_local)
    to_world(tl)
    parts["tibia_link"] = tl

    # ---- foot pad (hangs on the clevis pin; NOT pitched with tibia)
    hinge_local = knee_joint_local + Ry_pt_3 @ np.array(
        [HP.TIBIA_LENGTH, 0.0, HP.FOOT_HINGE_TIBIA_Z]
    )
    hinge_world = R_a_3 @ hinge_local + yaw_output_world

    foot = HP.make_foot_pad()
    foot.apply_transform(rotation_matrix(a, [0, 0, 1]))
    foot.apply_translation([hinge_world[0], hinge_world[1],
                            hinge_world[2] - HP.FOOT_HINGE_FOOT_Z])
    parts["foot_pad"] = foot

    # ---- M3 x 16 hinge bolt at the foot-tibia clevis (the visible pin)
    bolt_axis_world = R_a_3 @ np.array([0.0, 1.0, 0.0])
    bolt = trimesh.creation.cylinder(radius=1.6, height=18.0, sections=28)
    bolt.apply_transform(_align_z_to(bolt_axis_world))
    bolt.apply_translation(hinge_world)
    parts["hinge_bolt"] = bolt

    # ---------- Landmarks for label leader anchors --------------------
    landmarks: dict[str, np.ndarray] = {}

    flange_local = np.array([
        -(HP.BRACKET_FLANGE_INSET + HP.BRACKET_BOLT_PCD_X / 2.0),
        0.0,
        HP.BRACKET_FLANGE_T,
    ])
    landmarks["coxa_bracket_flange"] = R_a_3 @ flange_local + edge_mid

    # Wire harness exits on the body's +X SHORT face (same X-end as the
    # output gear).  In bracket-local coords the well origin is at
    # -SERVO_OUTPUT_X, so the body's +X face is at -SERVO_OUTPUT_X +
    # SERVO_BODY_W/2 and the slot center sits one wall-half farther out.
    wire_local = np.array([
        -HP.SERVO_OUTPUT_X
            + HP.SERVO_BODY_W / 2.0
            + HP.WELL_WALL_X / 2.0,
        0.0,
        HP.WIRE_SLOT_DEPTH / 2.0,
    ])
    landmarks["wire_exit"] = R_a_3 @ wire_local + edge_mid

    landmarks["yaw_joint"] = edge_mid + np.array([
        0, 0,
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z) + HP.SERVO_OUTPUT_H,
    ])
    landmarks["hip_joint"] = (R_a_3 @ np.array([HP.COXA_LENGTH, 0.0, hip_drop])
                              + yaw_output_world)
    landmarks["knee_joint"] = R_a_3 @ knee_joint_local + yaw_output_world
    landmarks["hinge_pin"] = hinge_world

    # "tibia clearance" callout anchor: the gap between the femur's knee
    # well rim (+Y) and the tibia hip pad.  In leg-local that gap lives
    # just outboard of the knee joint along +Y.
    clearance_local = (knee_joint_local
                       + np.array([0.0, HP.WELL_D / 2.0 + 2.0, 0.0]))
    landmarks["tibia_clearance"] = R_a_3 @ clearance_local + yaw_output_world

    return parts, landmarks


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def _aabb_center(mesh: trimesh.Trimesh) -> np.ndarray:
    lo, hi = mesh.bounds
    return 0.5 * (lo + hi)


def _project_to_pixel(plotter: pv.Plotter,
                       world_xyz: np.ndarray) -> tuple[float, float]:
    """Project a world point to display pixel coords (origin bottom-left)."""
    coord = vtk.vtkCoordinate()
    coord.SetCoordinateSystemToWorld()
    coord.SetValue(float(world_xyz[0]),
                   float(world_xyz[1]),
                   float(world_xyz[2]))
    px, py = coord.GetComputedDisplayValue(plotter.renderer)
    return float(px), float(py)


def _camera_for_view(view: str, focal: np.ndarray, span: float) -> list:
    """Return [pos, focal, up] for a named view direction."""
    dist = max(span * 5.0, 500.0)
    if view == "iso":
        # az=-65, el=30 -> world +X projects mostly to screen-right, world
        # +Z projects to screen-up; gives an iso 3/4 feel without making
        # the leg's primary axis collapse along the camera viewing direction.
        az = math.radians(-65.0)
        el = math.radians(30.0)
        pos = np.array([
            focal[0] + dist * math.cos(el) * math.cos(az),
            focal[1] + dist * math.cos(el) * math.sin(az),
            focal[2] + dist * math.sin(el),
        ])
        return [pos.tolist(), focal.tolist(), (0, 0, 1)]
    if view == "side":
        # looking along +Y at the leg.  +X right, +Z up.
        pos = focal + np.array([0.0, -dist, 0.0])
        return [pos.tolist(), focal.tolist(), (0, 0, 1)]
    if view == "top":
        # looking straight down.  +X right, +Y up.
        pos = focal + np.array([0.0, 0.0, dist])
        return [pos.tolist(), focal.tolist(), (0, 1, 0)]
    raise ValueError(view)


def _fit_parallel_scale(plotter: pv.Plotter,
                         parts: dict[str, trimesh.Trimesh],
                         image_size: tuple[int, int],
                         margin_frac: float = 0.18) -> None:
    """Adjust parallel_scale so the leg fits the central (1 - 2*margin_frac)
    of the image.  Also nudges the focal point so the leg's projected
    centre lands at the image centre."""
    W, H = image_size

    pts = np.vstack([np.asarray(m.vertices) for m in parts.values()])

    # Subsample for speed
    if len(pts) > 4000:
        idx = np.random.default_rng(seed=1).choice(len(pts), 4000,
                                                    replace=False)
        sample = pts[idx]
    else:
        sample = pts

    # First pass: project leg vertices using current parallel_scale.
    plotter.render()
    proj = np.array([_project_to_pixel(plotter, p) for p in sample])
    sx_min, sy_min = proj.min(axis=0)
    sx_max, sy_max = proj.max(axis=0)
    width = max(sx_max - sx_min, 1.0)
    height = max(sy_max - sy_min, 1.0)

    target_w = (1 - 2 * margin_frac) * W
    target_h = (1 - 2 * margin_frac) * H
    scale = min(target_w / width, target_h / height)

    plotter.camera.parallel_scale = float(plotter.camera.parallel_scale / scale)

    # Second pass: nudge focal point so screen-centre of leg matches image
    # centre.
    plotter.render()
    proj2 = np.array([_project_to_pixel(plotter, p) for p in sample])
    cx_world = float(np.mean(sample[:, 0]))
    cy_world = float(np.mean(sample[:, 1]))
    cz_world = float(np.mean(sample[:, 2]))
    leg_center_world = np.array([cx_world, cy_world, cz_world])
    cx_px, cy_px = _project_to_pixel(plotter, leg_center_world)
    dx_px = (W / 2.0) - cx_px
    dy_px = (H / 2.0) - cy_px

    # Compute camera right + up unit vectors in world coords so we can
    # convert pixel offsets to focal-point translation.
    cam = plotter.camera
    pos = np.array(cam.position)
    focal = np.array(cam.focal_point)
    up = np.array(cam.up)
    forward = focal - pos
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, up)
    right = right / np.linalg.norm(right)
    up_perp = np.cross(right, forward)
    up_perp = up_perp / np.linalg.norm(up_perp)

    world_per_pixel = 2.0 * cam.parallel_scale / H

    delta = (-dx_px * world_per_pixel) * right + (-dy_px * world_per_pixel) * up_perp
    # ^ minus signs: to move the projected leg-centre RIGHT on screen, we
    #   move the camera focal LEFT (in screen-right direction).  Similarly
    #   vertical: vtk display y is from bottom, but our dx_px / dy_px are
    #   in display-pixel coords (bottom-left origin) so positive dy_px
    #   means "move UP on screen", i.e. focal should move DOWN.  Test: if
    #   leg-centre is below image-centre (cy_px < H/2), dy_px > 0; we want
    #   to nudge focal DOWN so leg appears UP -> camera focal moves in
    #   -up_perp direction.

    cam.position = (pos + delta).tolist()
    cam.focal_point = (focal + delta).tolist()


def render_view(
    parts: dict[str, trimesh.Trimesh],
    label_specs: list[dict[str, Any]],
    view: str,
    out_png: str,
    *,
    floor_z: float | None = None,
    side_assignment: dict[str, str] | None = None,
) -> None:
    plotter = pv.Plotter(off_screen=True, window_size=WINDOW,
                         lighting="three lights")
    plotter.set_background(BG)

    all_pts = np.vstack([np.asarray(m.vertices) for m in parts.values()])
    lo = all_pts.min(axis=0)
    hi = all_pts.max(axis=0)
    center = 0.5 * (lo + hi)
    span = float(np.linalg.norm(hi - lo))

    if floor_z is not None:
        floor_span = max(hi[0] - lo[0], hi[1] - lo[1], 60.0) * 3.0
        floor = pv.Plane(
            center=(center[0], center[1], floor_z - 0.5),
            direction=(0, 0, 1),
            i_size=floor_span,
            j_size=floor_span,
        )
        plotter.add_mesh(floor, color=_hex_to_rgb1("#dde2e7"), opacity=1.0)

    for name, mesh in parts.items():
        pv_mesh = pv.wrap(mesh)
        plotter.add_mesh(
            pv_mesh,
            color=_hex_to_rgb1(PART_COLORS[name]),
            smooth_shading=False,
            specular=0.30,
            specular_power=22.0,
        )

    plotter.camera_position = _camera_for_view(view, center, span)
    plotter.enable_parallel_projection()
    plotter.camera.parallel_scale = span * 0.55
    _fit_parallel_scale(plotter, parts, WINDOW, margin_frac=0.20)

    img = plotter.screenshot(filename=None, return_img=True)
    H, W = img.shape[0], img.shape[1]

    placed = []
    for spec in label_specs:
        anchor = np.asarray(spec["anchor"])
        px, py = _project_to_pixel(plotter, anchor)
        py_top = H - py
        s = {**spec, "anchor_px": (px, py_top)}
        if side_assignment is not None and spec.get("key") in side_assignment:
            s["side"] = side_assignment[spec["key"]]
        placed.append(s)

    plotter.close()

    # Perimeter label layout: each label is pushed to either the left or
    # right margin of the image; labels on a side are stacked vertically
    # so leader lines don't overlap.  Anchors near image-y are honoured.
    left_x = 140
    right_x = W - 140
    min_v_spacing = 55

    def assign_side(spec):
        if "side" in spec:
            return spec["side"]
        return "left" if spec["anchor_px"][0] < W / 2 else "right"

    for s in placed:
        s["side"] = assign_side(s)

    def distribute(specs, x_pos, ha):
        if not specs:
            return
        specs.sort(key=lambda s: s["anchor_px"][1])
        ys = [float(s["anchor_px"][1]) for s in specs]
        for i in range(1, len(ys)):
            if ys[i] - ys[i - 1] < min_v_spacing:
                ys[i] = ys[i - 1] + min_v_spacing
        if ys[-1] > H - 50:
            shift = ys[-1] - (H - 50)
            ys = [y - shift for y in ys]
        if ys[0] < 50:
            shift = 50 - ys[0]
            ys = [y + shift for y in ys]
        for s, y in zip(specs, ys):
            s["text_xy"] = (x_pos, y)
            s["ha"] = ha
            s["va"] = "center"

    left_specs = [s for s in placed if s["side"] == "left"]
    right_specs = [s for s in placed if s["side"] == "right"]
    distribute(left_specs, left_x, "left")
    distribute(right_specs, right_x, "right")

    # ----- matplotlib overlay -----
    fig = plt.figure(figsize=(W / 100.0, H / 100.0), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img, interpolation="bilinear")
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.set_axis_off()

    for spec in placed:
        ax_px, ay_px = spec["anchor_px"]
        text = spec["text"]
        tx, ty = spec["text_xy"]
        ha = spec["ha"]
        va = spec.get("va", "center")
        fc = spec.get("box_fc", "white")
        ec = spec.get("box_ec", "#0f172a")
        fontsize = spec.get("fontsize", 14)
        color = spec.get("text_color", "#0f172a")
        ax.annotate(
            text,
            xy=(ax_px, ay_px),
            xytext=(tx, ty),
            ha=ha,
            va=va,
            fontsize=fontsize,
            fontweight="bold",
            color=color,
            arrowprops=dict(
                arrowstyle="-",
                color="#0f172a",
                lw=1.3,
                shrinkA=3,
                shrinkB=5,
            ),
            bbox=dict(
                boxstyle="round,pad=0.32",
                fc=fc,
                ec=ec,
                lw=1.1,
            ),
        )

    fig.savefig(out_png, dpi=100)
    plt.close(fig)
    print(f"  wrote {out_png}  ({W} x {H})")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    print("Building one prototype leg in chassis frame ...")
    parts, landmarks = build_leg_parts(leg_index=0)

    # Rotate the whole leg so it points along world +X (clean iso framing).
    a0 = (0 + 0.5) * math.pi / 3.0
    Rz = rotation_matrix(-a0, [0, 0, 1])
    Rz3 = Rz[:3, :3]
    for m in parts.values():
        m.apply_transform(Rz)
    for k in list(landmarks.keys()):
        landmarks[k] = Rz3 @ landmarks[k]

    # Lift so the foot sits on z = 0.
    all_pts = np.vstack([np.asarray(m.vertices) for m in parts.values()])
    z_lift = -float(all_pts[:, 2].min())
    for m in parts.values():
        m.apply_translation([0, 0, z_lift])
    for k in list(landmarks.keys()):
        landmarks[k] = landmarks[k] + np.array([0, 0, z_lift])

    centers = {name: _aabb_center(m) for name, m in parts.items()}

    # ---- ISO view ---------------------------------------------------
    # Bracket lives at world (low X) -> LEFT on screen.
    # Foot lives at world (high X) -> RIGHT on screen.
    # Auto-side based on each anchor's projected x picks left/right
    # naturally; we only override for the colour-coded callouts that
    # want a specific column.
    iso_labels: list[dict[str, Any]] = [
        dict(key="bracket", text="coxa_bracket",
             anchor=landmarks["coxa_bracket_flange"]),
        dict(key="wire", text="wire exit (L-slot)",
             anchor=landmarks["wire_exit"],
             box_fc="#fef9c3", box_ec="#a16207"),
        dict(key="yaw_servo", text="yaw servo (DS3225)",
             anchor=centers["yaw_servo"]),
        dict(key="yaw_horn", text="horn adapter (yaw)",
             anchor=landmarks["yaw_joint"],
             box_fc="#fef9c3", box_ec="#a16207", fontsize=12),
        dict(key="coxa_link", text="coxa_link",
             anchor=centers["coxa_link"]),
        dict(key="hip_servo", text="hip-pitch servo (DS3225)",
             anchor=centers["hip_servo"]),
        dict(key="hip_horn", text="horn adapter (hip)",
             anchor=landmarks["hip_joint"],
             box_fc="#fef9c3", box_ec="#a16207", fontsize=12),
        dict(key="femur", text="femur_link",
             anchor=centers["femur_link"]),
        dict(key="knee_servo", text="knee servo (DS3225)",
             anchor=centers["knee_servo"]),
        dict(key="knee_horn", text="horn adapter (knee)",
             anchor=landmarks["knee_joint"],
             box_fc="#fef9c3", box_ec="#a16207", fontsize=12),
        dict(key="tibia_clear", text="tibia clearance \u2713",
             anchor=landmarks["tibia_clearance"], side="right",
             box_fc="#dcfce7", box_ec="#15803d",
             text_color="#14532d", fontsize=12),
        dict(key="tibia", text="tibia_link",
             anchor=centers["tibia_link"]),
        dict(key="foot", text="foot_pad (TPU)",
             anchor=centers["foot_pad"]),
        dict(key="bolt", text="M3\u00d716 hinge bolt",
             anchor=landmarks["hinge_pin"]),
    ]
    iso_out = os.path.join(OUT_DIR, "leg_assembly_labeled_iso.png")
    render_view(parts, iso_labels, "iso", iso_out, floor_z=0.0)

    # ---- TOP view ---------------------------------------------------
    # Looking straight down.  +X right, +Y up.  Bracket on the LEFT
    # (low X), foot on the RIGHT (high X).
    top_labels: list[dict[str, Any]] = [
        dict(key="bracket", text="coxa_bracket",
             anchor=landmarks["coxa_bracket_flange"]),
        dict(key="wire", text="wire exit (L-slot)",
             anchor=landmarks["wire_exit"],
             box_fc="#fef9c3", box_ec="#a16207"),
        dict(key="yaw_servo", text="yaw servo",
             anchor=centers["yaw_servo"]),
        dict(key="coxa_link", text="coxa_link",
             anchor=centers["coxa_link"]),
        dict(key="hip_servo", text="hip-pitch servo",
             anchor=centers["hip_servo"]),
        dict(key="femur", text="femur_link",
             anchor=centers["femur_link"]),
        dict(key="knee_servo", text="knee servo",
             anchor=centers["knee_servo"]),
        dict(key="tibia", text="tibia_link",
             anchor=centers["tibia_link"]),
        dict(key="foot", text="foot_pad",
             anchor=centers["foot_pad"]),
    ]
    top_out = os.path.join(OUT_DIR, "leg_assembly_labeled_top.png")
    render_view(parts, top_labels, "top", top_out, floor_z=None)

    # ---- SIDE view --------------------------------------------------
    # Looking along +Y (camera at -Y).  +X right, +Z up.  Bracket lower-
    # LEFT, foot lower-RIGHT.
    side_labels: list[dict[str, Any]] = [
        dict(key="bracket", text="coxa_bracket",
             anchor=landmarks["coxa_bracket_flange"]),
        dict(key="wire", text="wire exit (L-slot)",
             anchor=landmarks["wire_exit"],
             box_fc="#fef9c3", box_ec="#a16207"),
        dict(key="yaw_servo", text="yaw servo",
             anchor=centers["yaw_servo"]),
        dict(key="coxa_link", text="coxa_link",
             anchor=centers["coxa_link"]),
        dict(key="hip_servo", text="hip-pitch servo",
             anchor=centers["hip_servo"]),
        dict(key="femur", text="femur_link",
             anchor=centers["femur_link"]),
        dict(key="knee_servo", text="knee servo",
             anchor=centers["knee_servo"]),
        dict(key="tibia", text="tibia_link",
             anchor=centers["tibia_link"]),
        dict(key="foot", text="foot_pad",
             anchor=centers["foot_pad"]),
        dict(key="bolt", text="M3\u00d716 hinge bolt",
             anchor=landmarks["hinge_pin"]),
        dict(key="tibia_clear", text="tibia clearance \u2713",
             anchor=landmarks["tibia_clearance"], side="right",
             box_fc="#dcfce7", box_ec="#15803d",
             text_color="#14532d", fontsize=12),
    ]
    side_out = os.path.join(OUT_DIR, "leg_assembly_labeled_side.png")
    render_view(parts, side_labels, "side", side_out, floor_z=0.0)

    print()
    print("Wrote labeled assembly renders:")
    print(f"   {iso_out}")
    print(f"   {top_out}")
    print(f"   {side_out}")


if __name__ == "__main__":
    main()
