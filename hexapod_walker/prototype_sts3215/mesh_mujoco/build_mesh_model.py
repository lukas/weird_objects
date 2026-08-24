#!/usr/bin/env python3
"""Mesh-accurate MuJoCo model of the as-built STS3215 hexapod (EXPERIMENTAL).

Unlike ``mujoco_prototype.py`` (primitive capsule/box physics with optional,
known-misaligned visual STLs), this builder places the ACTUAL CAD part meshes
-- every printed part plus the bought servos / horns / bearings / CF tubes --
at the verified assembly transforms, and uses them for BOTH rendering and
collision (MuJoCo collides per-part convex hulls).

Placement source of truth (never re-derived here):
  * ``tools/full_robot_viz_build.py`` link/servo frames -- the same math the
    BuildViz scene uses, guarded by ``_assert_servo_placement`` against
    ``_verify_prototype``'s authoritative poses.
  * ``_verify_prototype._horn_world_transform`` / ``_passive_horn_world_transform``
    for the disc horns.

Joint conventions match the legacy sim EXACTLY (names ``L{i}_yaw/pitch/knee``,
axes, ranges, actuator layout, ``L{i}_pad``/``L{i}_foot``/``L{i}_foot_site``),
so qpos semantics carry over.  What is MORE accurate than the legacy sim:

  * hip-pitch axis at the REAL coxa anchor ``COXA_HIP_ANCHOR`` =
    (12.5, -25.65, +38.4) mm -- the legacy sim dropped the +38.4 mm rise;
  * the tibia/foot line sits at the real sandwich mid-plane (+24.15 mm along
    the knee axis), which cancels the coxa anchor's -25.65 mm tangential
    offset -- the legacy sim left the foot ~24 mm off the leg radial;
  * per-part mesh collision instead of two capsules + three boxes.

The CF tibia tube is CUT to length here so the boot apex lands exactly at the
bench-measured ``TIBIA_LENGTH`` (150 mm knee->tip; legs 0/4 use the 4 mm-short
tube + ``foot_boot_plus4``, same apex).  The CAD's merged-tibia formula would
land it at ~179.5 mm; the tube is a cut-to-length bought part, so the printed
parts stay exact.

Outputs (both gitignored -- regenerate with this script):
  assets/*.stl       part meshes in mm (MJCF loads them with scale 0.001)
  hexapod_mesh.xml   the model

Run (repo venv):  python build_mesh_model.py [--no-render]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PROTO = HERE.parent
sys.path[0:0] = [str(PROTO), str(PROTO / "tools")]

import trimesh  # noqa: E402
from trimesh.transformations import (  # noqa: E402
    quaternion_from_matrix,
    rotation_matrix,
)

import hexapod_prototype as HP  # noqa: E402
import _verify_prototype as V  # noqa: E402
import full_robot_viz_build as VIZ  # noqa: E402
from part_palette import PART_COLORS  # noqa: E402

MM = 0.001  # mm -> m

ASSET_DIR = HERE / "assets"
XML_PATH = HERE / "hexapod_mesh.xml"
PREVIEW_DIR = HERE / "previews"

# --- legacy-sim parity constants (mujoco_prototype.py) ---------------------
TORQUE_LIMIT = 2.2
KP = {"yaw": 18.0, "pitch": 26.0, "knee": 24.0}
KV = {"yaw": 0.35, "pitch": 0.45, "knee": 0.40}
ARMATURE = 0.0004
JOINT_RANGE = {"yaw": (-0.61, 0.61), "pitch": (-1.40, 0.52), "knee": (-0.35, 2.62)}
BODY_MASS_BUDGET = {  # calibrated per-body masses from the legacy sim (kg)
    "chassis": 0.55, "coxa": 0.110, "femur": 0.105, "tibia": 0.040, "pad": 0.004,
}
SERVO_MASS = 0.060  # one STS3215, pinned exactly wherever a servo body sits

# CAD display stance (spider pose).  NOTE: with the REAL hip height this pose
# rests the robot on its under-belly yaw-servo retainers, feet unloaded --
# the legacy sim only stood on it because its hip axis was ~46 mm too high.
STANCE = (0.0, math.radians(HP.STANCE_FEMUR_DEG), math.radians(HP.STANCE_TIBIA_DEG))
# canonical standing plant (rl_move sim_env._default_plant_deg: +20/+80 rel)
PLANT = (0.0, math.radians(20.0), math.radians(80.0))
FOOT_R = HP.FOOT_BOOT_OD / 2.0 * MM

# Parts that take part in contact (everything else is render-only).  Horns,
# bearings and the electronics stack stay visual: they are enclosed by /
# sandwiched between collidable parts, and their convex hulls would only add
# noise.  LiPo packs DO collide -- they are the under-belly rest surface.
COLLIDE_PARTS = {
    "coxa_link", "femur_link", "tibia_knee_yoke", "tibia_tube",
    "yaw_servo", "hip_servo", "knee_servo",
    "hip_clamp_cap", "knee_clamp_cap", "yaw_servo_retainer",
    "chassis_top", "chassis_bottom", "switch_holster", "lipo_battery",
}

# part types that ARE an STS3215 body (mass pinned to SERVO_MASS)
SERVO_PARTS = {"yaw_servo", "hip_servo", "knee_servo"}

# viz partType -> MuJoCo body ("yaw" link = coxa body etc.)
LINK_TO_BODY = {"yaw": "coxa", "hip": "femur", "knee": "tibia"}


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


def link_frames_leg0():
    """Chassis-frame transforms of the coxa/femur/tibia link frames for leg 0
    at the CAD stance pose (mirrors ``_leg0_local_link_parts``)."""
    a = 0.5 * math.pi / 3.0
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    edge = np.array([apothem * math.cos(a), apothem * math.sin(a),
                     HP.CHASSIS_YAW_OUTPUT_Z])
    p = math.radians(HP.STANCE_FEMUR_DEG)
    pt = math.radians(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    hip_local = np.array(HP.COXA_HIP_ANCHOR, float)
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] @ \
        np.array([HP.FEMUR_LENGTH, 0.0, 0.0])
    Rz = rotation_matrix(a, [0, 0, 1])
    T_coxa = _trans(edge) @ Rz
    T_femur = _trans(edge) @ Rz @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = _trans(edge) @ Rz @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])
    return T_coxa, T_femur, T_tibia


def tibia_tube_geometry():
    """As-built tibia tube runs.  The CAD merged-tibia formula measures the
    tube from the yoke socket, overshooting the bench-measured knee->apex span
    (TIBIA_LENGTH).  Reality: the tube is CUT so the apex lands at
    TIBIA_LENGTH.  Returns (socket_point_ta, std_run_mm, short_run_mm)."""
    xz = (1, 0, 0), HP.LEG_PITCH_AXIS
    Mk0 = HP._joint_place((0.0, 0.0, 0.0), *xz)
    ta = (Mk0 @ np.array([HP._YOKE_SOCKET_X, 0.0, HP.JOINT_SOCKET_Z, 1.0]))[:3]
    std_run = HP.TIBIA_LENGTH - HP.FOOT_BOOT_TIP_L - ta[0]
    short_run = std_run - HP.FOOT_BOOT_SHORT_EXTRA
    # apex = ta.x + run + tip_l must land exactly at TIBIA_LENGTH
    assert abs(ta[0] + std_run + HP.FOOT_BOOT_TIP_L - HP.TIBIA_LENGTH) < 1e-9
    assert abs(ta[0] + short_run
               + HP.FOOT_BOOT_TIP_L + HP.FOOT_BOOT_SHORT_EXTRA
               - HP.TIBIA_LENGTH) < 1e-9
    return ta, std_run, short_run


class Part:
    """One geom: a mesh asset + local 4x4 (mm) inside its MuJoCo body."""

    def __init__(self, part_type: str, asset: str, mesh: trimesh.Trimesh,
                 local: np.ndarray, collide: bool):
        self.part_type = part_type
        self.asset = asset
        self.mesh = mesh
        self.local = np.asarray(local, float)
        self.collide = collide
        self.mass = 0.0  # filled by distribute_masses

    @property
    def volume(self) -> float:
        m = self.mesh
        try:
            if m.is_watertight and m.volume > 0:
                return float(m.volume)
        except Exception:
            pass
        return float(m.convex_hull.volume)


def collect_parts():
    """Gather every part with its body assignment and body-local transform.

    Returns (bodies, assets) where bodies maps body name ->
    list[Part] and assets maps asset name -> mesh (mm)."""
    VIZ._assert_servo_placement()  # guard: duplicated math == verifier poses

    T_coxa, T_femur, T_tibia = link_frames_leg0()
    inv = np.linalg.inv
    T_by_body = {"chassis": np.eye(4), "coxa": T_coxa,
                 "femur": T_femur, "tibia": T_tibia}

    assets: dict[str, trimesh.Trimesh] = {}
    bodies: dict[str, list[Part]] = {
        "chassis": [], "coxa": [], "femur": [], "tibia": [], "pad": [],
        "pad_short": [],
    }

    def add(body: str, part_type: str, mesh, world_or_local, *,
            asset: str | None = None, in_world: bool = True):
        asset = asset or part_type
        if asset not in assets:
            assets[asset] = mesh
        local = (inv(T_by_body[body]) @ world_or_local if in_world
                 else np.asarray(world_or_local, float))
        bodies[body].append(
            Part(part_type, asset, assets[asset], local,
                 part_type in COLLIDE_PARTS))

    # -- leg link parts (viz local-frame list; tube/boot rebuilt below) -----
    for name, mesh, M0 in VIZ._leg0_local_link_parts():
        if name in ("tibia_tube", "foot_boot"):
            continue
        body = LINK_TO_BODY[VIZ._MOTION_LINK_OF_PARTTYPE[name]]
        add(body, name, mesh, M0)

    # -- servo bodies / clamp caps / yaw retainer ---------------------------
    servo_body_map = {
        "yaw_servo": "chassis", "hip_servo": "coxa", "knee_servo": "femur",
        "hip_clamp_cap": "coxa", "knee_clamp_cap": "femur",
        "yaw_servo_retainer": "chassis",
    }
    for name, mesh, M0 in VIZ._servo_local_parts():
        add(servo_body_map[name], name, mesh, M0)

    # -- disc horns (driven + passive), verifier-authoritative poses --------
    for joint, body in (("yaw", "coxa"), ("hip", "femur"), ("knee", "tibia")):
        add(body, f"disc_horn_{joint}", HP.make_disc_horn(),
            V._horn_world_transform(joint, 0), asset="disc_horn")
    for joint, body in (("hip", "femur"), ("knee", "tibia")):
        add(body, f"passive_horn_{joint}", HP.make_disc_horn(),
            V._passive_horn_world_transform(joint, 0), asset="disc_horn")

    # -- tibia tube, cut to the measured span; boot on the pad body ---------
    ta, std_run, short_run = tibia_tube_geometry()
    r = HP.LEG_TUBE_OD / 2.0
    add("tibia", "tibia_tube",
        HP._tube_between(ta, ta + np.array([std_run, 0, 0]), r),
        np.eye(4), asset="tibia_tube_std", in_world=False)
    assets["tibia_tube_short"] = HP._tube_between(
        ta, ta + np.array([short_run, 0, 0]), r)
    # boots live on the L{i}_pad body whose origin is the tube end
    add("pad", "foot_boot", HP.make_foot_boot(), np.eye(4), in_world=False)
    bodies["pad_short"].append(
        Part("foot_boot", "foot_boot_plus4", HP.make_foot_boot_plus4(),
             np.eye(4), collide=True))
    assets["foot_boot_plus4"] = bodies["pad_short"][0].mesh

    # -- chassis: printed plates + as-built electronics ---------------------
    try:
        body_parts = VIZ._body_local_parts()
    except Exception as exc:
        # the electronics stack loads two extra_stl meshes; regenerate them
        # with tools/make_xtool_hex_mount_plate.py + _hex_raised_platform.py
        print(f"  [warn] electronics stack unavailable ({exc}); bare plates only")
        body_parts = [
            ("chassis_bottom", HP.make_chassis_bottom(), np.eye(4)),
            ("chassis_top", HP.make_chassis_top(),
             _trans([0, 0, HP.CHASSIS_TOP_CENTRE_Z])),
            ("switch_holster", HP.make_switch_holster(),
             _trans([HP.SWITCH_HOLSTER_CENTRE_X, HP.SWITCH_HOLSTER_CENTRE_Y,
                     HP.CHASSIS_TOP_TOP_Z])),
        ]
    for name, mesh, M0 in body_parts:
        add("chassis", name, mesh, M0)

    return bodies, assets, (ta, std_run, short_run)


def distribute_masses(bodies: dict[str, list[Part]]):
    """Match each MuJoCo body's total mass to the calibrated legacy-sim
    budget: STS3215 bodies pinned at 60 g, the remainder spread over the
    body's other parts proportional to actual mesh volume.  Chassis parts in
    leg frames (yaw servo + retainer) replicate 6x and are budgeted as such."""
    for body, budget in BODY_MASS_BUDGET.items():
        if body == "pad":
            for grp in ("pad", "pad_short"):
                for p in bodies[grp]:
                    p.mass = budget
            continue
        parts = bodies[body]
        mult = (lambda p: 6 if p.part_type in
                ("yaw_servo", "yaw_servo_retainer") else 1) \
            if body == "chassis" else (lambda p: 1)
        pinned = sum(SERVO_MASS * mult(p) for p in parts
                     if p.part_type in SERVO_PARTS)
        rest = [p for p in parts if p.part_type not in SERVO_PARTS]
        vol = sum(p.volume * mult(p) for p in rest)
        remaining = budget - pinned
        if remaining <= 0:
            raise RuntimeError(f"{body}: servo mass exceeds budget {budget}")
        for p in parts:
            if p.part_type in SERVO_PARTS:
                p.mass = SERVO_MASS
            else:
                p.mass = remaining * p.volume / vol
    # the short-tube asset shares the std tube's mass (4 mm shorter is noise)


def _color_for(part_type: str) -> tuple[float, float, float]:
    if part_type in PART_COLORS:
        return tuple(PART_COLORS[part_type])
    pal = VIZ.PALETTE.get(part_type)
    if pal is None and part_type.startswith("disc_horn"):
        pal = VIZ.DISC_HORN_COLOR
    if pal is None and part_type.startswith("passive_horn"):
        pal = VIZ.DISC_HORN_COLOR
    if isinstance(pal, str) and pal.startswith("#"):
        return tuple(int(pal[k:k + 2], 16) / 255.0 for k in (1, 3, 5))
    return (0.62, 0.64, 0.68)


def _fmt_local(local: np.ndarray) -> str:
    pos = local[:3, 3] * MM
    q = quaternion_from_matrix(local)  # (w, x, y, z)
    q = q / np.linalg.norm(q)
    s = f'pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}"'
    if abs(q[0] - 1.0) > 1e-9:
        s += f' quat="{q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}"'
    return s


def _geom_xml(name: str, p: Part, *, asset: str | None = None) -> str:
    cls = "part" if p.collide else "visual"
    return (f'<geom class="{cls}" name="{name}" mesh="{asset or p.asset}" '
            f'{_fmt_local(p.local)} material="mat_{p.part_type}" '
            f'mass="{p.mass:.6f}"/>')


def build_xml(bodies, assets, tube_info, base_z: dict) -> str:
    ta, std_run, short_run = tube_info
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0 * MM
    hip = np.array(HP.COXA_HIP_ANCHOR) * MM
    femur = HP.FEMUR_LENGTH * MM

    mesh_xml = "\n    ".join(
        f'<mesh name="{n}" file="{n}.stl" scale="0.001 0.001 0.001"/>'
        for n in sorted(assets))
    part_types = sorted({p.part_type for parts in bodies.values() for p in parts})
    mat_xml = "\n    ".join(
        '<material name="mat_{}" rgba="{:.4f} {:.4f} {:.4f} 1" '
        'specular="0.3" shininess="0.3"/>'.format(t, *_color_for(t))
        for t in part_types)

    # chassis-frame geoms: body parts once + leg-frame parts (yaw servo,
    # retainer) rotated per leg
    chassis_geoms = []
    counts: dict[str, int] = {}
    for p in bodies["chassis"]:
        if p.part_type in ("yaw_servo", "yaw_servo_retainer"):
            for i in range(6):
                Rz = rotation_matrix(i * math.pi / 3.0, [0, 0, 1])
                q = Part(p.part_type, p.asset, p.mesh, Rz @ p.local, p.collide)
                q.mass = p.mass
                chassis_geoms.append(_geom_xml(f"L{i}_{p.part_type}", q))
        else:
            k = counts[p.part_type] = counts.get(p.part_type, 0) + 1
            chassis_geoms.append(_geom_xml(f"{p.part_type}_{k}", p))
    chassis_xml = "\n      ".join(chassis_geoms)

    # IMU site at the real MPU-6050 spot on the top deck
    imu = (getattr(HP, "MPU_ASBUILT_CENTRE", (0.0, 0.0)))
    imu_z = getattr(HP, "CHASSIS_TOP_TOP_Z", 36.0) + \
        getattr(HP, "IMU_PCB_T", 1.6) / 2.0
    imu_pos = (imu[0] * MM, imu[1] * MM, imu_z * MM)

    leg_blocks, act_blocks, sens_blocks = [], [], []
    for i in range(6):
        a = (i + 0.5) * math.pi / 3.0
        px, py = apothem * math.cos(a), apothem * math.sin(a)
        pz = HP.CHASSIS_YAW_OUTPUT_Z * MM
        qw, qz = math.cos(a / 2.0), math.sin(a / 2.0)
        short = i in HP.SHORT_CF_LEG_INDICES
        tube_asset = "tibia_tube_short" if short else "tibia_tube_std"
        pad_parts = bodies["pad_short" if short else "pad"]
        tube_end_x = (ta[0] + (short_run if short else std_run)) * MM
        tip_l = HP.FOOT_BOOT_TIP_L + (HP.FOOT_BOOT_SHORT_EXTRA if short else 0)
        site_x = (tip_l - HP.FOOT_BOOT_OD / 2.0) * MM  # dome centre

        def geoms(body: str, indent: str, leg=i) -> str:
            out = []
            cnt: dict[str, int] = {}
            for p in bodies[body]:
                if p.part_type == "tibia_tube":
                    out.append(_geom_xml(f"L{leg}_tibia_tube", p,
                                         asset=tube_asset))
                    continue
                k = cnt[p.part_type] = cnt.get(p.part_type, 0) + 1
                name = f"L{leg}_{p.part_type}" + (f"_{k}" if k > 1 else "")
                out.append(_geom_xml(name, p))
            return ("\n" + indent).join(out)

        boot = pad_parts[0]
        rng = JOINT_RANGE
        leg_blocks.append(f"""      <body name="L{i}_yaw" pos="{px:.6f} {py:.6f} {pz:.6f}" quat="{qw:.8f} 0 0 {qz:.8f}">
        <joint name="L{i}_yaw" type="hinge" axis="0 0 1" range="{rng['yaw'][0]} {rng['yaw'][1]}"/>
        {geoms('coxa', '        ')}
        <body name="L{i}_femur" pos="{hip[0]:.6f} {hip[1]:.6f} {hip[2]:.6f}">
          <joint name="L{i}_pitch" type="hinge" axis="0 1 0" range="{rng['pitch'][0]} {rng['pitch'][1]}"/>
          {geoms('femur', '          ')}
          <body name="L{i}_tibia" pos="{femur:.6f} 0 0">
            <joint name="L{i}_knee" type="hinge" axis="0 1 0" range="{rng['knee'][0]} {rng['knee'][1]}"/>
            {geoms('tibia', '            ')}
            <body name="L{i}_pad" pos="{tube_end_x:.6f} {ta[1] * MM:.6f} {ta[2] * MM:.6f}">
              <geom class="foot" name="L{i}_foot" mesh="{boot.asset}" material="mat_foot_boot" mass="{boot.mass:.6f}"/>
              <site name="L{i}_foot_site" pos="{site_x:.6f} 0 0" size="0.018" group="4" rgba="1 1 1 0.05"/>
            </body>
          </body>
        </body>
      </body>
""")
        act_blocks.append(f"""    <position name="L{i}_yaw" joint="L{i}_yaw" kp="{KP['yaw']}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <position name="L{i}_pitch" joint="L{i}_pitch" kp="{KP['pitch']}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <position name="L{i}_knee" joint="L{i}_knee" kp="{KP['knee']}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_yaw_d" joint="L{i}_yaw" kv="{KV['yaw']}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_pitch_d" joint="L{i}_pitch" kv="{KV['pitch']}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_knee_d" joint="L{i}_knee" kv="{KV['knee']}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
""")
        sens_blocks.append(f"""    <jointpos name="L{i}_yaw_p" joint="L{i}_yaw"/>
    <jointpos name="L{i}_pitch_p" joint="L{i}_pitch"/>
    <jointpos name="L{i}_knee_p" joint="L{i}_knee"/>
    <touch name="L{i}_foot_t" site="L{i}_foot_site"/>
""")

    def _key(name: str, pose, z: float) -> str:
        q = " ".join(f"{v:.6f}" for v in pose)
        qpos = f"0 0 {z:.6f} 1 0 0 0 " + " ".join([q] * 6)
        ctrl = " ".join([q + " 0 0 0"] * 6)
        return f'<key name="{name}" qpos="{qpos}" ctrl="{ctrl}"/>'

    keys_xml = "\n    ".join(
        _key(name, pose, base_z[name])
        for name, pose in (("plant", PLANT), ("stance", STANCE)))

    return f"""<mujoco model="hexapod_sts3215_mesh">
  <compiler angle="radian" coordinate="local" autolimits="true" meshdir="assets"/>
  <option gravity="0 0 -9.81" timestep="0.002" iterations="50" solver="Newton" cone="elliptic"/>
  <visual>
    <global offwidth="1920" offheight="1200"/>
    <quality shadowsize="4096"/>
  </visual>

  <default>
    <joint armature="{ARMATURE}" damping="0.01" limited="true"/>
    <geom solref="0.006 1" solimp="0.95 0.99 0.001"/>
    <default class="part">
      <geom type="mesh" group="1" condim="3" friction="1.0 0.02 0.0001"/>
    </default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/>
    </default>
    <default class="foot">
      <geom type="mesh" group="1" condim="6" friction="2.0 0.1 0.001"
            solref="0.01 1" solimp="0.95 0.99 0.001"/>
    </default>
  </default>

  <asset>
    <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.7 0.85 1.0" rgb2="0.4 0.5 0.7" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.25 0.3 0.35" rgb2="0.18 0.22 0.27" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.15"/>
    {mat_xml}
    {mesh_xml}
  </asset>

  <worldbody>
    <light name="sun" pos="2 -1.5 2.2" dir="-0.6 0.4 -1" castshadow="true"/>
    <geom name="floor" type="plane" size="8 8 0.05" material="grid" friction="1.5 0.05 0.0001"/>

    <body name="chassis" pos="0 0 {base_z['plant']:.6f}">
      <freejoint name="root"/>
      <site name="chassis_imu" pos="{imu_pos[0]:.6f} {imu_pos[1]:.6f} {imu_pos[2]:.6f}" size="0.008"/>
      {chassis_xml}

{''.join(leg_blocks)}    </body>
  </worldbody>

  <actuator>
{''.join(act_blocks)}  </actuator>

  <sensor>
{''.join(sens_blocks)}    <accelerometer name="chassis_acc" site="chassis_imu"/>
    <gyro name="chassis_gyro" site="chassis_imu"/>
    <framepos name="chassis_pos" objtype="body" objname="chassis"/>
    <framezaxis name="chassis_up" objtype="body" objname="chassis"/>
  </sensor>

  <keyframe>
    {keys_xml}
  </keyframe>
</mujoco>
"""


def export_assets(assets):
    ASSET_DIR.mkdir(exist_ok=True)
    for name, mesh in assets.items():
        mesh.export(ASSET_DIR / f"{name}.stl")
    print(f"  exported {len(assets)} mesh assets -> {ASSET_DIR.relative_to(HERE)}/")


def _lowest_collidable_z(model, data) -> float:
    """Lowest world-z point over every collidable geom (mesh verts exact)."""
    import mujoco
    lows = []
    for g in range(model.ngeom):
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        pos = data.geom_xpos[g]
        R = data.geom_xmat[g].reshape(3, 3)
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = int(model.geom_dataid[g])
            adr, num = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
            v = model.mesh_vert[adr:adr + num]
            lows.append(float((v @ R.T + pos)[:, 2].min()))
        else:
            lows.append(float(pos[2]) - float(np.max(model.geom_size[g])))
    return min(lows)


def compute_base_z(xml: str) -> dict:
    """Per-keyframe chassis height so the lowest COLLIDABLE point (feet at
    the plant; the under-belly hardware at the CAD display stance) rests on
    the floor."""
    import mujoco
    model = mujoco.MjModel.from_xml_string(xml, assets=_asset_bytes())
    data = mujoco.MjData(model)
    out = {}
    for k in range(model.nkey):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, k)
        mujoco.mj_resetDataKeyframe(model, data, k)
        mujoco.mj_forward(model, data)
        out[name] = float(data.qpos[2]) - _lowest_collidable_z(model, data) \
            + 0.0005
    return out


def _asset_bytes():
    return {f"{p.name}": p.read_bytes() for p in ASSET_DIR.glob("*.stl")}


def _up_z(data) -> float:
    """World-z component of the chassis +Z axis (1 = upright)."""
    w, x, y, z = data.qpos[3:7]
    return 1.0 - 2.0 * (x * x + y * y)


def check_model(render: bool = True):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    print(f"  model: nq={model.nq} nv={model.nv} nu={model.nu} "
          f"nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh}")
    total = float(np.sum(model.body_mass))
    print(f"  total mass = {total:.3f} kg (legacy sim: 2.104)")

    for kname in ("plant", "stance"):
        k = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, kname)
        mujoco.mj_resetDataKeyframe(model, data, k)
        mujoco.mj_forward(model, data)
        z0 = data.qpos[2]
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "L1_foot_site")
        p = data.site_xpos[sid]
        print(f"  [{kname}] base z = {z0 * 1000:.1f} mm, foot dome centre "
              f"r={math.hypot(p[0], p[1]) * 1000:.1f} mm z={p[2] * 1000:.1f} mm")

        # non-floor penetration report at the keyframe pose
        bad = []
        for c in data.contact:
            g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or "?"
            g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or "?"
            if "floor" in (g1, g2):
                continue
            if c.dist < -1e-4:
                bad.append((g1, g2, c.dist))
        if bad:
            print(f"  [{kname}] NON-FLOOR penetrating contacts:")
            for g1, g2, d in sorted(bad, key=lambda t: t[2])[:20]:
                print(f"    {g1} <-> {g2}: {d * 1000:.2f} mm")
        else:
            print(f"  [{kname}] no non-floor penetrations")

        # settle 3 s holding the keyframe's servo targets
        for _ in range(int(3.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        floor_geoms = sorted({
            (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
             if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM,
                                  c.geom1) == "floor"
             else mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1))
            for c in data.contact
            if "floor" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM,
                                             c.geom1),
                           mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM,
                                             c.geom2))})
        print(f"  [{kname}] after 3 s: base z = {data.qpos[2] * 1000:.1f} mm "
              f"(drop {(z0 - data.qpos[2]) * 1000:+.1f}), upright = "
              f"{_up_z(data):.4f}, max |qvel| = "
              f"{np.max(np.abs(data.qvel)):.4f}")
        print(f"  [{kname}] resting on: {', '.join(floor_geoms)}")
        if not np.isfinite(data.qpos).all():
            raise RuntimeError("simulation diverged (NaN in qpos)")

    if render:
        render_previews(model, data)
    return model, data


def render_previews(model, data):
    import mujoco
    PREVIEW_DIR.mkdir(exist_ok=True)
    try:
        import imageio
        save = lambda path, img: imageio.imwrite(path, img)
    except ImportError:
        import cv2
        save = lambda path, img: cv2.imwrite(str(path), img[:, :, ::-1])
    renderer = mujoco.Renderer(model, height=900, width=1400)
    cam = mujoco.MjvCamera()
    shots = {  # (keyframe, azimuth, elevation, distance, lookat_z)
        "plant_iso": ("plant", 135, -18, 0.80, 0.10),
        "plant_side": ("plant", 90, -6, 0.75, 0.10),
        "stance_iso": ("stance", 135, -22, 0.80, 0.07),
        "stance_top": ("stance", 90, -78, 0.90, 0.05),
    }
    for name, (kname, az, el, dist, lz) in shots.items():
        k = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, kname)
        mujoco.mj_resetDataKeyframe(model, data, k)
        mujoco.mj_forward(model, data)
        cam.lookat[:] = [0.0, 0.0, lz]
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        renderer.update_scene(data, camera=cam)
        save(PREVIEW_DIR / f"{name}.png", renderer.render())
    renderer.close()
    print(f"  previews -> {PREVIEW_DIR.relative_to(HERE)}/")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()

    print("[1/4] building part meshes from CAD factories ...")
    bodies, assets, tube_info = collect_parts()
    distribute_masses(bodies)
    export_assets(assets)

    print("[2/4] emitting MJCF ...")
    xml = build_xml(bodies, assets, tube_info,
                    base_z={"plant": 0.5, "stance": 0.5})
    print("[3/4] solving keyframe base heights ...")
    base_z = compute_base_z(xml)
    xml = build_xml(bodies, assets, tube_info, base_z=base_z)
    XML_PATH.write_text(xml)
    print(f"  wrote {XML_PATH.relative_to(HERE)} "
          f"(base z: plant {base_z['plant'] * 1000:.1f} mm, "
          f"stance {base_z['stance'] * 1000:.1f} mm)")

    print("[4/4] checking model ...")
    check_model(render=not args.no_render)
    print("done.")


if __name__ == "__main__":
    main()
