"""Interactive PyVista build inspector for the hexapod walker prototype.

This is the third "view" target alongside the two MuJoCo viewers.  It
loads every per-part STL emitted by ``hexapod_prototype.main`` into a
PyVista scene, places each instance at the assembled-pose transform
used by ``build_prototype_assembly._build_leg`` / ``make_assembly_preview``,
colors it by part type (from ``part_palette.PART_COLORS``), labels it
with its role, and gives you an exploded-view slider so you can pull
the legs out and inspect the joints.

Design
------
* Transforms are recomputed in this file (mirroring the math in
  ``build_prototype_assembly._build_leg`` and ``make_assembly_preview``)
  so the inspector uses the STLs as the source of truth and applies
  numpy 4x4 matrices, instead of carrying around already-baked
  trimesh objects.
* The explode slider does NOT mutate the cached world transforms; it
  only sets each actor's ``user_matrix`` to a translation offset
  proportional to ``(instance_centroid - chassis_centroid)``.  Going
  back to 0.0 returns to the exact assembled pose.
* The per-part-type hide/show checkboxes flip ``actor.visibility``
  on every actor of a given type.  Labels respect visibility too.
* The status overlay at the bottom-left is a real PyVista legend
  built from ``PART_COLORS`` so the user never has to memorize what
  "the teal one" is.

Keyboard shortcuts (also printed at launch)
-------------------------------------------
* ``L`` -- toggle the floating per-instance labels
* ``E`` -- toggle exploded view between 0.0 and 1.5
* ``R`` -- reset the camera view
* ``S`` -- save a screenshot to ``artifacts/views/build_inspect.png``
* ``Q`` -- quit (also bound by PyVista's default)

CLI flags
---------
* ``--screenshot PATH`` -- render once headless and save a PNG, then exit.
* ``--explode FLOAT``   -- initial explode amount in ``[0, 2]``
                          (default 0.0 = exact assembled pose).

Extending
---------
To add another part type:
    1. Add a color to ``part_palette.PART_COLORS``.
    2. Add a label rule in ``part_palette.instance_role``.
    3. Append an instance with the right 4x4 transform in
       :func:`_build_assembly_instances` below.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

import numpy as np

# pyvista + trimesh are heavy; defer import where convenient so the
# argparse --help path stays snappy.
import pyvista as pv
import trimesh

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

import hexapod_prototype as HP  # noqa: E402
import part_palette as palette  # noqa: E402


STL_DIR = HP.STL_DIR
ARTIFACTS_DIR = os.path.join(THIS_DIR, "artifacts", "views")


# Plastic-horn height stack (matches build_prototype_assembly.py).  The
# value is repeated locally instead of imported because it lives as a
# bare ``PLASTIC_HORN_H = 5.0`` inside ``_build_leg`` rather than as a
# module-level constant.
PLASTIC_HORN_H = 5.0


# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------


def _trans(x: float, y: float, z: float) -> np.ndarray:
    """4x4 pure-translation matrix."""
    m = np.eye(4)
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def _rot_x(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1],
    ])


def _rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ])


def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


# ---------------------------------------------------------------------------
# Per-instance world transforms
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Instance:
    """One placed copy of one STL inside the assembled hexapod."""
    part_type: str
    stl_name: str       # filename inside stl_prototype/
    leg_index: int | None
    joint: str | None   # 'yaw' / 'hip' / 'knee' for joint hardware, else None
    transform: np.ndarray  # 4x4 world transform (pre-chassis-lift)


def _build_assembly_instances() -> list[Instance]:
    """Return every part instance in the assembled robot.

    The transforms returned here do NOT yet include ``chassis_lift``;
    the caller adds a single +Z translation to every instance after
    computing chassis_lift from the bounding boxes of the loaded STLs.
    Matches the structure of ``build_prototype_assembly._build_leg``
    and ``make_assembly_preview`` so a part type appears in the same
    spatial location as in a Cycles render.
    """
    instances: list[Instance] = []

    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    plate_t = HP.CHASSIS_PLATE_T
    gap = HP.CHASSIS_GAP

    # ---------- chassis-level (one each) ----------
    instances.append(Instance(
        "chassis_bottom", "chassis_bottom.stl", None, None, _trans(0, 0, 0),
    ))
    instances.append(Instance(
        "chassis_top", "chassis_top.stl", None, None,
        _trans(0, 0, gap + plate_t),
    ))
    instances.append(Instance(
        "battery_holder", "battery_holder.stl", None, None,
        _trans(-25.0, 0, plate_t),
    ))
    instances.append(Instance(
        "electronics_tray", "electronics_tray.stl", None, None,
        _trans(35.0, 0, plate_t + 1.0),
    ))

    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + PLASTIC_HORN_H
        + HP.HORN_ADAPTER_T
    )
    p_femur = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    hip_drop = HP.COXA_HIP_DROP

    R_hip = _rot_x(-np.pi / 2.0)

    for i in range(6):
        a = (i + 0.5) * np.pi / 3.0
        edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
        R_a = _rot_z(a)

        yaw_output_world = edge_mid + np.array([0.0, 0.0, yaw_output_z])

        # Frame helpers: T_edge places a mesh in chassis-edge coords
        # (bracket origin); T_yaw_out places a mesh in coxa-link / leg
        # local coords (yaw output at the origin).
        T_edge = _trans(*edge_mid) @ R_a
        T_yaw_out = _trans(*yaw_output_world) @ R_a

        # ----- coxa_bracket (chassis-fixed)
        instances.append(Instance(
            "coxa_bracket", "coxa_bracket.stl", i, None, T_edge,
        ))

        # ----- yaw servo body (hangs below the bracket)
        T_yaw_body = T_edge @ _trans(-HP.SERVO_OUTPUT_X, 0, -HP.WELL_RIM_Z)
        instances.append(Instance(
            "servo_body", "servo_body.stl", i, "yaw", T_yaw_body,
        ))

        # ----- yaw plastic horn (above the bracket flange)
        yaw_horn_z = (HP.SERVO_BODY_H - HP.WELL_RIM_Z) + HP.SERVO_OUTPUT_H
        T_yaw_horn = T_edge @ _trans(0, 0, yaw_horn_z)
        instances.append(Instance(
            "servo_horn", "servo_horn.stl", i, "yaw", T_yaw_horn,
        ))

        # ----- yaw printed adapter (on top of plastic horn)
        yaw_adapter_z = yaw_horn_z + PLASTIC_HORN_H
        T_yaw_adapter = T_edge @ _trans(0, 0, yaw_adapter_z)
        instances.append(Instance(
            "servo_horn_adapter", "servo_horn_adapter.stl", i, "yaw",
            T_yaw_adapter,
        ))

        # ----- coxa link (rotates with yaw output -- in standing pose, yaw=0)
        instances.append(Instance(
            "coxa_link", "coxa_link.stl", i, None, T_yaw_out,
        ))

        # ----- hip-pitch servo body (in the coxa-link cradle)
        delta_hip = np.array([
            HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
            hip_drop,
        ])
        T_hip_body = T_yaw_out @ _trans(*delta_hip) @ R_hip
        instances.append(Instance(
            "servo_body", "servo_body.stl", i, "hip", T_hip_body,
        ))

        # ----- hip plastic horn (on the hip-pitch output axis)
        T_hip_horn = T_yaw_out @ _trans(HP.COXA_LENGTH, 0, hip_drop) @ R_hip
        instances.append(Instance(
            "servo_horn", "servo_horn.stl", i, "hip", T_hip_horn,
        ))

        # ----- hip printed adapter (stacked above the plastic horn)
        # The adapter sits PLASTIC_HORN_H above the horn base along the
        # horn's local +Z; R_hip then aligns that to the leg-local +Y
        # output direction.  Order matters: rotate the adapter's local
        # frame first, then add the offset, then move to the joint.
        T_hip_adapter = (
            T_yaw_out
            @ _trans(HP.COXA_LENGTH, 0, hip_drop)
            @ R_hip
            @ _trans(0, 0, PLASTIC_HORN_H)
        )
        instances.append(Instance(
            "servo_horn_adapter", "servo_horn_adapter.stl", i, "hip",
            T_hip_adapter,
        ))

        # ----- femur link (rotated by hip-pitch stance angle)
        hip_joint_local = np.array([HP.COXA_LENGTH, 0, hip_drop])
        T_femur = T_yaw_out @ _trans(*hip_joint_local) @ _rot_y(p_femur)
        instances.append(Instance(
            "femur_link", "femur_link.stl", i, None, T_femur,
        ))

        # ----- knee-pitch servo body (in the femur cradle)
        delta_knee = np.array([
            HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
            0.0,
        ])
        T_knee_body = T_femur @ _trans(*delta_knee) @ R_hip
        instances.append(Instance(
            "servo_body", "servo_body.stl", i, "knee", T_knee_body,
        ))

        # ----- knee plastic horn
        T_knee_horn = T_femur @ _trans(HP.FEMUR_LENGTH, 0, 0) @ R_hip
        instances.append(Instance(
            "servo_horn", "servo_horn.stl", i, "knee", T_knee_horn,
        ))

        # ----- knee printed adapter
        T_knee_adapter = (
            T_femur
            @ _trans(HP.FEMUR_LENGTH, 0, 0)
            @ R_hip
            @ _trans(0, 0, PLASTIC_HORN_H)
        )
        instances.append(Instance(
            "servo_horn_adapter", "servo_horn_adapter.stl", i, "knee",
            T_knee_adapter,
        ))

        # ----- tibia link
        Ry_p_3 = _rot_y(p_femur)[:3, :3]
        knee_joint_local = (
            hip_joint_local + Ry_p_3 @ np.array([HP.FEMUR_LENGTH, 0, 0])
        )
        T_tibia = T_yaw_out @ _trans(*knee_joint_local) @ _rot_y(pt)
        instances.append(Instance(
            "tibia_link", "tibia_link.stl", i, None, T_tibia,
        ))

        # ----- foot pad (passive hinge -- only the leg azimuth is applied)
        Ry_pt_3 = _rot_y(pt)[:3, :3]
        hinge_local = knee_joint_local + Ry_pt_3 @ np.array(
            [HP.TIBIA_LENGTH, 0.0, HP.FOOT_HINGE_TIBIA_Z]
        )
        R_a_3 = R_a[:3, :3]
        hinge_world = R_a_3 @ hinge_local + yaw_output_world
        T_foot = (
            _trans(hinge_world[0], hinge_world[1],
                   hinge_world[2] - HP.FOOT_HINGE_FOOT_Z)
            @ R_a
        )
        instances.append(Instance(
            "foot_pad", "foot_pad.stl", i, None, T_foot,
        ))

    return instances


# ---------------------------------------------------------------------------
# STL loading
# ---------------------------------------------------------------------------


def _missing_stl_message(missing: list[str]) -> str:
    listing = "\n  ".join(missing)
    return (
        f"inspect_build: missing {len(missing)} STL(s) under {STL_DIR}:\n"
        f"  {listing}\n"
        "Run `make build` (or `./run.sh hexapod_walker/prototype/build_all.py`) "
        "first to regenerate the per-part STLs."
    )


def _load_stl_cache(stl_names: set[str]) -> dict[str, pv.PolyData]:
    """Load each unique STL once.  Returns part-local PolyData meshes."""
    cache: dict[str, pv.PolyData] = {}
    missing: list[str] = []
    for name in sorted(stl_names):
        path = os.path.join(STL_DIR, name)
        if not os.path.isfile(path):
            missing.append(name)
            continue
        # trimesh handles the STL parse robustly; convert to pyvista
        # PolyData by feeding the verts + faces in pv's flat triangle
        # connectivity layout.
        tm = trimesh.load_mesh(path, process=False)
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(list(tm.geometry.values()))
        cache[name] = _trimesh_to_pv(tm)
    if missing:
        print(_missing_stl_message(missing), file=sys.stderr)
        sys.exit(2)
    return cache


def _trimesh_to_pv(tm: trimesh.Trimesh) -> pv.PolyData:
    """Cheap conversion: PolyData with triangle connectivity."""
    faces = np.hstack([
        np.full((len(tm.faces), 1), 3, dtype=np.int64),
        tm.faces.astype(np.int64),
    ]).ravel()
    return pv.PolyData(tm.vertices.astype(np.float64), faces)


def _apply_transform(mesh: pv.PolyData, T: np.ndarray) -> pv.PolyData:
    """Return a transformed copy (does not modify the cached source)."""
    return mesh.copy().transform(T, inplace=False)


# ---------------------------------------------------------------------------
# Scene assembly
# ---------------------------------------------------------------------------


@dataclass
class PlacedInstance:
    """An Instance after STL loading + chassis_lift + add_mesh."""
    instance: Instance
    actor: pv.Actor
    centroid: np.ndarray   # world-space centroid of the placed mesh
    label_pos: np.ndarray  # where to anchor the floating label


def _compute_chassis_lift(
    instances: list[Instance],
    stl_cache: dict[str, pv.PolyData],
) -> float:
    """Mirror build_prototype_assembly._build_leg's chassis_lift logic.

    The lowest z point across leg 0's parts (foot pad, tibia, knee
    horn, etc.) gives ``z_min``; the chassis lift is ``-z_min`` so
    the foot pads land on z = 0.
    """
    z_min = np.inf
    for inst in instances:
        if inst.leg_index != 0:
            continue
        mesh = _apply_transform(stl_cache[inst.stl_name], inst.transform)
        zlow = float(mesh.bounds[4])  # bounds = (xmin,xmax,ymin,ymax,zmin,zmax)
        if zlow < z_min:
            z_min = zlow
    if not np.isfinite(z_min):
        return 0.0
    return -z_min


# ---------------------------------------------------------------------------
# Plotter app
# ---------------------------------------------------------------------------


SHORTCUTS_TEXT = (
    "Keyboard shortcuts:\n"
    "  L  toggle labels\n"
    "  E  toggle exploded view (0 / 1.5)\n"
    "  R  reset camera\n"
    "  S  save screenshot\n"
    "  Q  quit"
)


def _print_shortcuts() -> None:
    print(SHORTCUTS_TEXT)


def _add_instances_to_plotter(
    plotter_factory_args: dict,
    instances: list[Instance],
    stl_cache: dict[str, pv.PolyData],
    chassis_lift: float,
) -> tuple[pv.Plotter, list[PlacedInstance], np.ndarray]:
    """Create the plotter, add every instance as a colored actor.

    Returns (plotter, placed_instances, chassis_centroid).
    """
    plotter = pv.Plotter(**plotter_factory_args)
    plotter.set_background("white", top="lightgray")

    placed: list[PlacedInstance] = []
    chassis_centroids: list[np.ndarray] = []

    lift = _trans(0, 0, chassis_lift)

    for inst in instances:
        world_T = lift @ inst.transform
        mesh = _apply_transform(stl_cache[inst.stl_name], world_T)
        r, g, b = palette.PART_COLORS.get(
            inst.part_type, (0.7, 0.7, 0.7),
        )
        # Servo proxies stay muted to keep the printed parts visually
        # dominant.  Slightly thinner edges + lower opacity.
        is_servo = inst.part_type in ("servo_body", "servo_horn")
        actor = plotter.add_mesh(
            mesh,
            color=(r, g, b),
            show_edges=True,
            edge_color=(0.10, 0.10, 0.10),
            line_width=0.5 if is_servo else 0.7,
            opacity=0.85 if is_servo else 1.0,
            ambient=0.18,
            diffuse=0.85,
            specular=0.15,
            specular_power=18.0,
            smooth_shading=True,
            name=f"{inst.part_type}_L{inst.leg_index}_{inst.joint}",
        )
        cen = np.asarray(mesh.center, dtype=np.float64)
        # For long, narrow parts like the femur and tibia, anchoring
        # the label at the centroid drops it into the middle of the
        # mesh.  Push the label up by a few mm so it sits above the
        # geometry instead of inside it.
        zoff = float(mesh.bounds[5] - cen[2])
        label_anchor = cen + np.array([0.0, 0.0, 0.6 * max(zoff, 8.0)])
        placed.append(PlacedInstance(
            instance=inst,
            actor=actor,
            centroid=cen,
            label_pos=label_anchor,
        ))
        if inst.part_type in ("chassis_top", "chassis_bottom",
                              "chassis_plate_a", "chassis_plate_b"):
            chassis_centroids.append(cen)

    if chassis_centroids:
        chassis_centroid = np.mean(np.vstack(chassis_centroids), axis=0)
    else:
        # Fallback: average all instance centroids
        chassis_centroid = np.mean(
            np.vstack([p.centroid for p in placed]), axis=0,
        )

    return plotter, placed, chassis_centroid


def run(
    *,
    screenshot: str | None,
    explode: float,
    window_size: tuple[int, int] = (1600, 1000),
) -> None:
    instances = _build_assembly_instances()
    stl_names = {inst.stl_name for inst in instances}
    stl_cache = _load_stl_cache(stl_names)
    chassis_lift = _compute_chassis_lift(instances, stl_cache)

    headless = screenshot is not None
    # We initially build a placeholder plotter for actor creation, then
    # discard it and rebuild a "full" plotter with all the widgets and
    # callbacks attached.  Simpler approach: just construct one plotter
    # up-front with the right off_screen mode.
    plotter_kwargs = {"off_screen": headless, "window_size": window_size}
    plotter, placed, chassis_centroid = _add_instances_to_plotter(
        plotter_kwargs, instances, stl_cache, chassis_lift,
    )

    # Re-decorate this plotter with the legend, labels, slider, and
    # widgets.  We pass a constructor function so _build_plotter can
    # use a single plotter (we already have one populated with actors).
    # Re-implement the "decoration" inline here to avoid double-creation:
    _decorate_plotter(plotter, placed, chassis_centroid, explode)

    plotter.camera_position = "iso"
    plotter.reset_camera()

    if headless:
        _print_shortcuts()
        os.makedirs(os.path.dirname(os.path.abspath(screenshot)), exist_ok=True)
        plotter.screenshot(screenshot)
        size = os.path.getsize(screenshot)
        print(
            f"inspect_build: headless screenshot -> {screenshot} "
            f"({size / 1024:.1f} KB)"
        )
        plotter.close()
        return

    _print_shortcuts()
    plotter.show(title="hexapod build inspector")


def _decorate_plotter(
    plotter: pv.Plotter,
    placed: list[PlacedInstance],
    chassis_centroid: np.ndarray,
    initial_explode: float,
) -> None:
    """Attach legend, labels, slider, checkboxes, key events to plotter."""
    present_types: list[str] = []
    for p in placed:
        if p.instance.part_type not in present_types:
            present_types.append(p.instance.part_type)

    legend_entries = [
        (pt, palette.PART_COLORS[pt]) for pt in present_types
        if pt in palette.PART_COLORS
    ]
    if legend_entries:
        plotter.add_legend(
            labels=legend_entries,
            bcolor="white",
            face="rectangle",
            size=(0.16, max(0.02 * len(legend_entries), 0.05)),
            loc="lower right",
            border=True,
        )

    plotter.add_text(
        SHORTCUTS_TEXT,
        position="upper_left",
        font_size=9,
        color="black",
        shadow=False,
    )

    label_positions = np.array([p.label_pos for p in placed])
    label_texts = [
        palette.instance_label(
            p.instance.part_type,
            p.instance.leg_index,
            p.instance.joint,
        )
        for p in placed
    ]

    label_actor_holder = {"actor": plotter.add_point_labels(
        label_positions,
        label_texts,
        point_size=0,
        show_points=False,
        font_size=14,
        bold=True,
        always_visible=True,
        shape_color="white",
        shape_opacity=0.7,
        text_color="black",
        name="instance_labels",
    )}
    labels_state = {"visible": True}

    def _toggle_labels() -> None:
        labels_state["visible"] = not labels_state["visible"]
        label_actor_holder["actor"].SetVisibility(labels_state["visible"])
        plotter.render()

    state = {"explode": float(initial_explode)}

    def _apply_explode(factor: float) -> None:
        for p in placed:
            offset = p.centroid - chassis_centroid
            shift = factor * offset
            m = np.eye(4)
            m[0, 3] = shift[0]
            m[1, 3] = shift[1]
            m[2, 3] = shift[2]
            p.actor.user_matrix = m
        new_pos = np.array([
            p.label_pos + factor * (p.centroid - chassis_centroid)
            for p in placed
        ])
        plotter.remove_actor("instance_labels")
        new_actor = plotter.add_point_labels(
            new_pos,
            label_texts,
            point_size=0,
            show_points=False,
            font_size=14,
            bold=True,
            always_visible=True,
            shape_color="white",
            shape_opacity=0.7,
            text_color="black",
            name="instance_labels",
        )
        new_actor.SetVisibility(labels_state["visible"])
        label_actor_holder["actor"] = new_actor

    def _slider_callback(value: float) -> None:
        state["explode"] = float(value)
        _apply_explode(state["explode"])

    plotter.add_slider_widget(
        _slider_callback,
        rng=(0.0, 2.0),
        value=state["explode"],
        title="explode",
        pointa=(0.62, 0.06),
        pointb=(0.96, 0.06),
        title_height=0.020,
        slider_width=0.02,
        tube_width=0.005,
        style="modern",
        fmt="%.2f",
    )

    if state["explode"] != 0.0:
        _apply_explode(state["explode"])

    # checkbox column
    actors_by_type: dict[str, list[pv.Actor]] = {}
    for p in placed:
        actors_by_type.setdefault(p.instance.part_type, []).append(p.actor)

    def _make_toggle(part_type: str):
        def _cb(value: bool) -> None:
            for actor in actors_by_type[part_type]:
                actor.SetVisibility(bool(value))
            plotter.render()
        return _cb

    cb_size = 18
    cb_x = 12
    cb_y0 = 12
    for idx, part_type in enumerate(present_types):
        pos_y = cb_y0 + idx * (cb_size + 6)
        r, g, b = palette.PART_COLORS.get(part_type, (0.5, 0.5, 0.5))
        plotter.add_checkbox_button_widget(
            _make_toggle(part_type),
            value=True,
            position=(cb_x, pos_y),
            size=cb_size,
            border_size=1,
            color_on=(r, g, b),
            color_off=(0.85, 0.85, 0.85),
            background_color="white",
        )
        plotter.add_text(
            part_type,
            position=(cb_x + cb_size + 4, pos_y + 2),
            font_size=8,
            color="black",
        )

    explode_toggle_state = {"on": initial_explode > 0.0}

    def _toggle_explode() -> None:
        explode_toggle_state["on"] = not explode_toggle_state["on"]
        state["explode"] = 1.5 if explode_toggle_state["on"] else 0.0
        try:
            for sw in plotter.slider_widgets:
                sw.GetSliderRepresentation().SetValue(state["explode"])
        except Exception:
            pass
        _apply_explode(state["explode"])
        plotter.render()

    def _reset_camera() -> None:
        plotter.reset_camera()
        plotter.render()

    def _save_screenshot() -> None:
        os.makedirs(ARTIFACTS_DIR, exist_ok=True)
        out = os.path.join(ARTIFACTS_DIR, "build_inspect.png")
        plotter.screenshot(out)
        print(f"inspect_build: saved screenshot -> {out}")

    for k in ("l", "L"):
        plotter.add_key_event(k, _toggle_labels)
    for k in ("e", "E"):
        plotter.add_key_event(k, _toggle_explode)
    for k in ("r", "R"):
        plotter.add_key_event(k, _reset_camera)
    for k in ("s", "S"):
        plotter.add_key_event(k, _save_screenshot)
    for k in ("q", "Q"):
        plotter.add_key_event(k, plotter.close)

    try:
        plotter.show_axes()
    except Exception:
        pass
    try:
        plotter.enable_anti_aliasing("msaa")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Interactive PyVista build inspector for the hexapod "
            "prototype.  Colored, labeled parts placed at the assembled "
            "pose; exploded-view slider + per-part-type hide toggles."
        ),
    )
    parser.add_argument(
        "--screenshot",
        type=str,
        default=None,
        help=(
            "Render once headless to PATH and exit.  When set, no "
            "interactive window is opened."
        ),
    )
    parser.add_argument(
        "--explode",
        type=float,
        default=0.0,
        help=(
            "Initial explode amount in [0, 2].  0 = exact assembled "
            "pose; 1 = pulled out enough to inspect joints; 2 = fully "
            "exploded."
        ),
    )
    parser.add_argument(
        "--window-w", type=int, default=1600, help="Window width (pixels).",
    )
    parser.add_argument(
        "--window-h", type=int, default=1000, help="Window height (pixels).",
    )
    args = parser.parse_args(argv)

    if not 0.0 <= args.explode <= 2.0:
        print(
            f"inspect_build: --explode must be in [0, 2], got {args.explode}",
            file=sys.stderr,
        )
        sys.exit(2)

    run(
        screenshot=args.screenshot,
        explode=args.explode,
        window_size=(args.window_w, args.window_h),
    )


if __name__ == "__main__":
    main()
