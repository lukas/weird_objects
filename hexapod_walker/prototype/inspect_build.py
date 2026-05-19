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
import fastener_registry  # noqa: E402


STL_DIR = HP.STL_DIR
FASTENERS_DIR = os.path.join(THIS_DIR, "fasteners")
ARTIFACTS_DIR = os.path.join(THIS_DIR, "artifacts", "views")


# Plastic-horn height stack (matches build_prototype_assembly.py and
# mujoco_prototype.py via HP.HORN_STACK_H).  Design B (May 2026): with
# the printed servo_horn_adapter retired, the link's mating face sits
# directly on top of the plastic horn, so the stack collapses to
# HP.HORN_STACK_H = PLASTIC_HORN_H = 5 mm above the spline tip (was
# PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm).
PLASTIC_HORN_H = HP.HORN_STACK_H   # 5 mm


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
    stl_name: str       # filename inside stl_prototype/ (or fasteners/ if
                        # ``stl_dir`` is set).
    leg_index: int | None
    joint: str | None   # 'yaw' / 'hip' / 'knee' for joint hardware, else None
    transform: np.ndarray  # 4x4 world transform (pre-chassis-lift)
    # Fasteners use ``fasteners/`` instead of ``stl_prototype/``, and
    # carry their own role string built by the registry (e.g.
    # ``"coxa_link L0 hip cradle -X top SHCS"``).  Empty / None for
    # printed + servo parts so nothing changes for them.
    stl_dir: str | None = None
    fastener_role: str | None = None


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
        + PLASTIC_HORN_H   # = HP.HORN_STACK_H (5 mm); see PLASTIC_HORN_H above
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

        # ----- yaw plastic horn (above the bracket flange).  Design B
        # (May 2026): the link's pad now bolts DIRECTLY onto this
        # plastic horn -- no printed servo_horn_adapter disc in the
        # stack any more.  The coxa_link's pedestal bottom mating face
        # therefore lands at z = yaw_horn_z + PLASTIC_HORN_H (= the
        # plastic horn's top face) rather than yaw_horn_z +
        # PLASTIC_HORN_H + HORN_ADAPTER_T.  yaw_output_z above already
        # reflects this.
        yaw_horn_z = (HP.SERVO_BODY_H - HP.WELL_RIM_Z) + HP.SERVO_OUTPUT_H
        T_yaw_horn = T_edge @ _trans(0, 0, yaw_horn_z)
        instances.append(Instance(
            "servo_horn", "servo_horn.stl", i, "yaw", T_yaw_horn,
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

        # ----- hip plastic horn (on the hip-pitch output axis).
        # Design B (May 2026): femur's hip pad bolts DIRECTLY onto this
        # plastic horn; the printed servo_horn_adapter is gone.
        T_hip_horn = T_yaw_out @ _trans(HP.COXA_LENGTH, 0, hip_drop) @ R_hip
        instances.append(Instance(
            "servo_horn", "servo_horn.stl", i, "hip", T_hip_horn,
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

        # ----- knee plastic horn.  Design B (May 2026): tibia's knee
        # pad bolts DIRECTLY onto this plastic horn; the printed
        # servo_horn_adapter is gone.
        T_knee_horn = T_femur @ _trans(HP.FEMUR_LENGTH, 0, 0) @ R_hip
        instances.append(Instance(
            "servo_horn", "servo_horn.stl", i, "knee", T_knee_horn,
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

    instances.extend(_build_fastener_instances())
    return instances


def _axis_to_transform(axis: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """Build a 4x4 that maps mesh-local +Z to ``axis`` and puts the
    mesh origin at ``origin``.  Used to place parametric fastener
    meshes (whose +Z is along the screw shaft) into the world.
    """
    z_new = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(z_new))
    if n < 1e-12:
        z_new = np.array([0.0, 0.0, 1.0])
    else:
        z_new = z_new / n
    # Pick the world axis least parallel to z_new as a seed for the new X.
    seed = (
        np.array([1.0, 0.0, 0.0])
        if abs(z_new[0]) < 0.9
        else np.array([0.0, 1.0, 0.0])
    )
    x_new = seed - z_new * float(np.dot(seed, z_new))
    x_new /= float(np.linalg.norm(x_new))
    y_new = np.cross(z_new, x_new)
    T = np.eye(4)
    T[:3, 0] = x_new
    T[:3, 1] = y_new
    T[:3, 2] = z_new
    T[:3, 3] = np.asarray(origin, dtype=float)
    return T


def _build_fastener_instances() -> list[Instance]:
    """Convert every entry in ``fastener_registry.build_all_fastener_instances()``
    into an inspector ``Instance``.

    The parametric fastener meshes in ``fasteners/`` use the convention:
        origin = head mating face, +Z = body axis (into material).
    So the world transform that places mesh-local +Z along
    ``fi.axis_world`` and the mesh origin at ``fi.head_world_xyz``
    correctly drops the head onto the printed part.
    """
    out: list[Instance] = []
    for fi in fastener_registry.build_all_fastener_instances():
        T = _axis_to_transform(fi.axis_world, fi.head_world_xyz)
        out.append(Instance(
            part_type=fi.spec,
            stl_name=fi.cache_stl,
            leg_index=fi.leg_index,
            joint=fi.joint,
            transform=T,
            stl_dir=FASTENERS_DIR,
            fastener_role=fi.role,
        ))
    return out


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


def _load_stl_cache(stl_keys: set[tuple[str, str]]) -> dict[tuple[str, str], pv.PolyData]:
    """Load each unique STL once.  Returns a (stl_dir, stl_name) -> PolyData
    cache, so fasteners (loaded out of ``fasteners/``) and printed parts
    (loaded out of ``stl_prototype/``) share the same lookup path.
    """
    cache: dict[tuple[str, str], pv.PolyData] = {}
    missing: list[str] = []
    for stl_dir, name in sorted(stl_keys):
        path = os.path.join(stl_dir, name)
        if not os.path.isfile(path):
            missing.append(os.path.join(os.path.basename(stl_dir.rstrip("/")), name))
            continue
        tm = trimesh.load_mesh(path, process=False)
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(list(tm.geometry.values()))
        cache[(stl_dir, name)] = _trimesh_to_pv(tm)
    if missing:
        print(_missing_stl_message(missing), file=sys.stderr)
        sys.exit(2)
    return cache


def _instance_stl_key(inst: Instance) -> tuple[str, str]:
    return (inst.stl_dir or STL_DIR, inst.stl_name)


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
        # Skip fasteners when computing chassis lift -- their cache
        # mesh has the head at (0,0,0) and the shank at (0,0,+L), so
        # they never extend the floor; counting them would lift the
        # foot from the ground.
        if palette.is_fastener(inst.part_type):
            continue
        mesh = _apply_transform(stl_cache[_instance_stl_key(inst)], inst.transform)
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
    "Hover -> label.  Left-click -> isolate part.\n"
    "Keyboard shortcuts:\n"
    "  L     toggle ALL labels on at once\n"
    "  E     toggle exploded view (0 / 1.5)\n"
    "  I/Esc clear isolation\n"
    "  R     reset camera\n"
    "  S     save screenshot\n"
    "  Q     quit"
)

# Opacity applied to every non-isolated actor while one part is
# isolated.  The isolated part stays at full opacity in its original
# assembled-pose position -- we do NOT translate it on click, because
# the whole point of isolation is seeing how the part sits relative to
# its neighbours.  Pulling it outward defeats that.
ISOLATE_DIM_OPACITY = 0.15


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

    fastener_counter = 0
    for inst in instances:
        world_T = lift @ inst.transform
        mesh = _apply_transform(stl_cache[_instance_stl_key(inst)], world_T)
        r, g, b = palette.PART_COLORS.get(
            inst.part_type, (0.7, 0.7, 0.7),
        )
        is_servo = inst.part_type in ("servo_body", "servo_horn")
        is_fastener = palette.is_fastener(inst.part_type)
        # Fasteners are small + numerous; turn off edge rendering so
        # the inspector stays responsive at full count (~300 actors
        # added by fastener_registry).
        if is_fastener:
            fastener_counter += 1
            actor_name = f"fastener_{inst.part_type}_{fastener_counter}"
        else:
            actor_name = f"{inst.part_type}_L{inst.leg_index}_{inst.joint}"
        actor = plotter.add_mesh(
            mesh,
            color=(r, g, b),
            show_edges=False if is_fastener else True,
            edge_color=(0.10, 0.10, 0.10),
            line_width=0.5 if is_servo else 0.7,
            opacity=0.85 if is_servo else 1.0,
            ambient=0.22 if is_fastener else 0.18,
            diffuse=0.85,
            specular=0.35 if is_fastener else 0.15,
            specular_power=24.0 if is_fastener else 18.0,
            smooth_shading=True,
            name=actor_name,
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
    stl_keys = {_instance_stl_key(inst) for inst in instances}
    stl_cache = _load_stl_cache(stl_keys)
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
        if pt in palette.PART_COLORS and not palette.is_fastener(pt)
    ]
    if any(palette.is_fastener(pt) for pt in present_types):
        legend_entries.append(
            ("fasteners (M3/M2.5)", palette.PART_COLORS["M3x14 SHCS"]),
        )
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
            fastener_role=p.instance.fastener_role,
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
    # Persistent all-labels-at-once is OFF by default now -- hover gives
    # you the same info one part at a time without the clutter.  Press
    # `L` to bring the full cloud back when you want overview context.
    labels_state = {"visible": False}
    label_actor_holder["actor"].SetVisibility(False)

    def _toggle_labels() -> None:
        labels_state["visible"] = not labels_state["visible"]
        label_actor_holder["actor"].SetVisibility(labels_state["visible"])
        plotter.render()

    state = {"explode": float(initial_explode)}

    # --- isolation state ----------------------------------------------
    # When the user left-clicks on a part, every OTHER actor's opacity
    # drops to ISOLATE_DIM_OPACITY so the picked part stands out.  We
    # deliberately do not move the picked part -- it stays where it
    # belongs in the assembly so the user can see how it fits.
    iso_state: dict[str, object | None] = {"placed": None}
    saved_opacity: dict[int, float] = {
        id(p.actor): float(p.actor.prop.opacity) for p in placed
    }

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
    fastener_actors: list[pv.Actor] = []
    for p in placed:
        actors_by_type.setdefault(p.instance.part_type, []).append(p.actor)
        if palette.is_fastener(p.instance.part_type):
            fastener_actors.append(p.actor)

    def _make_toggle(part_type: str):
        def _cb(value: bool) -> None:
            for actor in actors_by_type[part_type]:
                actor.SetVisibility(bool(value))
            plotter.render()
        return _cb

    cb_size = 18
    cb_x = 12
    cb_y0 = 12

    # Filter the per-part-type column to NON-fastener types -- the
    # fasteners share a single master toggle (below) so we don't end
    # up with six near-identical SHCS rows competing for column space.
    non_fastener_types = [pt for pt in present_types if not palette.is_fastener(pt)]
    has_fasteners = bool(fastener_actors)

    column_rows = list(non_fastener_types)
    if has_fasteners:
        # Master "fasteners" toggle sits at the TOP of the column (idx
        # 0) so the user always finds it in the same spot.  Render it
        # black to make it visually distinct from the per-printed-part
        # color swatches below.
        column_rows = ["__fasteners__"] + column_rows

    for idx, row_key in enumerate(column_rows):
        pos_y = cb_y0 + idx * (cb_size + 6)
        if row_key == "__fasteners__":
            def _toggle_fasteners(value: bool) -> None:
                for actor in fastener_actors:
                    actor.SetVisibility(bool(value))
                plotter.render()
            plotter.add_checkbox_button_widget(
                _toggle_fasteners,
                value=True,
                position=(cb_x, pos_y),
                size=cb_size,
                border_size=1,
                color_on=(0.10, 0.10, 0.10),       # black, per spec
                color_off=(0.85, 0.85, 0.85),
                background_color="white",
            )
            plotter.add_text(
                "fasteners",
                position=(cb_x + cb_size + 4, pos_y + 2),
                font_size=8,
                color="black",
            )
            continue
        part_type = row_key
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

    # ------------------------------------------------------------------
    # Hover label + click-to-isolate (mouse interaction)
    # ------------------------------------------------------------------
    # 2D overlay text that shows whichever part is under the cursor (or
    # the currently isolated part if any).  Sits near the top center so
    # it's visible regardless of camera angle without crowding the
    # bottom slider or the upper-left shortcuts.
    ww, wh = plotter.window_size
    hover_text_actor = plotter.add_text(
        "(hover a part)",
        position=(int(ww * 0.30), wh - 36),
        font_size=14,
        color="black",
        shadow=False,
        name="hover_label",
    )

    def _set_hover_text(s: str) -> None:
        try:
            hover_text_actor.SetInput(s)
        except Exception:
            # vtkTextActor in some PyVista builds uses SetText(0, s).
            try:
                hover_text_actor.SetText(0, s)
            except Exception:
                pass

    # Map actor identity -> placed instance for picker lookups.  Using
    # id() lets us key without paying for repeated isinstance checks.
    actor_to_placed: dict[int, PlacedInstance] = {
        id(p.actor): p for p in placed
    }

    # vtk cell picker: cheap, ray-casts against renderer actors and
    # returns the closest hit.  We reuse a single picker instance.
    try:
        import vtk  # type: ignore
        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.001)
    except Exception:
        picker = None  # type: ignore

    def _placed_under_cursor() -> "PlacedInstance | None":
        if picker is None:
            return None
        try:
            x, y = plotter.iren.interactor.GetEventPosition()
        except Exception:
            return None
        try:
            picker.Pick(x, y, 0, plotter.renderer)
        except Exception:
            return None
        actor = picker.GetActor()
        if actor is None:
            return None
        return actor_to_placed.get(id(actor))

    def _label_for(p: PlacedInstance) -> str:
        return palette.instance_label(
            p.instance.part_type,
            p.instance.leg_index,
            p.instance.joint,
            fastener_role=p.instance.fastener_role,
        )

    def _on_mouse_move(_obj, _evt) -> None:
        # Don't overwrite the isolation banner with hover noise.
        if iso_state["placed"] is not None:
            return
        p = _placed_under_cursor()
        _set_hover_text(_label_for(p) if p is not None else "(hover a part)")
        plotter.render()

    def _apply_isolation_visuals() -> None:
        """Reflect the current iso_state on every actor's opacity."""
        iso = iso_state["placed"]
        for p in placed:
            if iso is None:
                p.actor.prop.opacity = saved_opacity[id(p.actor)]
            elif p is iso:
                # Keep isolated part at full opacity so it pops.
                p.actor.prop.opacity = max(
                    saved_opacity[id(p.actor)], 0.95,
                )
            else:
                p.actor.prop.opacity = ISOLATE_DIM_OPACITY

    def _set_isolation(p: "PlacedInstance | None") -> None:
        iso_state["placed"] = p
        _apply_isolation_visuals()
        if p is None:
            _set_hover_text("(hover a part)")
        else:
            _set_hover_text(f"Isolated: {_label_for(p)}  [press I / Esc to clear]")
        plotter.render()

    def _on_left_click(point) -> None:
        # ``point`` is (x_pixel, y_pixel).
        if picker is None:
            return
        try:
            picker.Pick(point[0], point[1], 0, plotter.renderer)
        except Exception:
            return
        actor = picker.GetActor()
        placed_hit = actor_to_placed.get(id(actor)) if actor is not None else None
        if placed_hit is None:
            # Empty space -> drop isolation.
            if iso_state["placed"] is not None:
                _set_isolation(None)
        else:
            _set_isolation(placed_hit)

    def _clear_isolation() -> None:
        if iso_state["placed"] is not None:
            _set_isolation(None)

    try:
        plotter.iren.add_observer("MouseMoveEvent", _on_mouse_move)
    except Exception:
        # Older PyVistas expose this via plotter.iren.interactor.
        try:
            plotter.iren.interactor.AddObserver(
                "MouseMoveEvent", _on_mouse_move,
            )
        except Exception:
            pass

    try:
        plotter.track_click_position(
            callback=_on_left_click, side="left", viewport=True,
        )
    except Exception:
        pass

    for k in ("l", "L"):
        plotter.add_key_event(k, _toggle_labels)
    for k in ("e", "E"):
        plotter.add_key_event(k, _toggle_explode)
    for k in ("r", "R"):
        plotter.add_key_event(k, _reset_camera)
    for k in ("s", "S"):
        plotter.add_key_event(k, _save_screenshot)
    for k in ("i", "I", "Escape"):
        plotter.add_key_event(k, _clear_isolation)
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
