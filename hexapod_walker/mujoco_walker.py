"""MuJoCo simulation of the hexapod walker.

Builds an MJCF model from the kinematic constants in `hexapod_walker.py`,
attaches the existing STL parts from ``stl/`` as visual meshes, adds simple
primitive collision geometry, hangs 18 hinge joints with PD position
actuators, and drives them with an alternating-tripod gait controller.

Usage
-----

    # interactive 3D viewer (macOS: launch from main thread, this script
    # uses ``mujoco.viewer.launch_passive`` so it works on macOS too)
    ./run.sh hexapod_walker/mujoco_walker.py

    # headless: simulate for N seconds, print body trajectory + save MJCF
    ./run.sh hexapod_walker/mujoco_walker.py --headless --duration 6.0

    # just dump the generated XML and exit
    ./run.sh hexapod_walker/mujoco_walker.py --dump-xml hexapod_walker/walker.xml

    # play with the gait
    ./run.sh hexapod_walker/mujoco_walker.py --stride 0.45 --period 1.6 --lift 0.10

The model uses the same hexagonal chassis with 6 × 3 = 18 actuators that
the build guide describes.  Geometry is mirrored from
``hexapod_walker.py`` (apothem, leg lengths, motor envelope, stance
angles) so the simulation matches the CAD.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import mujoco

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)
import hexapod_walker as HW  # noqa: E402


# ---------------------------------------------------------------------------
# Geometry constants (mm -> m)
# ---------------------------------------------------------------------------

M = 0.001  # mm -> m

APOTHEM = HW.CHASSIS_FLAT_TO_FLAT / 2.0 * M     # 0.60 m  (apothem of hex)
COXA = HW.COXA_LENGTH * M                       # 0.15 m
FEMUR = HW.FEMUR_LENGTH * M                     # 0.60 m
TIBIA = HW.TIBIA_LENGTH * M                     # 0.80 m
TUBE = HW.CHASSIS_TUBE * M                      # 0.08 m
MOTOR_OD = HW.MOTOR_OD * M                      # 0.17 m
MOTOR_LENGTH = HW.MOTOR_LENGTH * M              # 0.13 m
YAW_OUTPUT_HEIGHT = (HW.MOTOR_LENGTH + HW.LINK_END_CAP_T) / 2.0 * M  # 0.075
COXA_RADIAL = (HW.CHASSIS_TUBE / 2.0
               + HW.MOTOR_OD / 2.0 + 30.0) * M   # 0.155
LEG_RADIAL = APOTHEM + COXA_RADIAL              # 0.755 m  (yaw axis -> centre)

CHASSIS_Z = HW.CHASSIS_HEIGHT * M               # 0.22 m  (overall depth)
FEMUR_TUBE_W = HW.FEMUR_TUBE_W * M              # 0.09
FEMUR_TUBE_H = HW.FEMUR_TUBE_H * M              # 0.12
TIBIA_TUBE_W = HW.TIBIA_TUBE_W * M              # 0.07
TIBIA_TUBE_H = HW.TIBIA_TUBE_H * M              # 0.09

STANCE_FEMUR = math.radians(HW.STANCE_FEMUR_DEG)   # -25 deg
STANCE_TIBIA = math.radians(HW.STANCE_TIBIA_DEG)   # +60 deg

# Foot-collision sphere radius (the actual cast urethane foot pad is wider
# but flatter; a sphere is the simplest, most stable contact primitive).
FOOT_R = 0.05  # m

# Mass distribution (kg).  Total ~ 390 kg, matching ASSEMBLY.md §5.3
# walker-with-rider weight.  Chassis lumps deck + battery + electronics +
# rider + saddle into one rigid body so we can focus on leg dynamics.
CHASSIS_MASS = 195.0
COXA_MASS = 7.0
FEMUR_MASS = 14.0
TIBIA_MASS = 12.0

# Joint actuator gains (industrial harmonic-drive servo, ~ FHA-40C-100).
# These translate to: a 1 deg position error costs ~ 90 N·m at the hip.
KP_YAW   = 6_000.0
KP_PITCH = 35_000.0
KP_KNEE  = 25_000.0
DAMP_YAW   = 400.0
DAMP_PITCH = 1500.0
DAMP_KNEE  = 1000.0
TORQUE_LIMIT = 1700.0   # N·m, per-joint clamp (matches §5.2 budget)

# Joint armature: reflected rotor inertia of a 1.5 kW BLDC * (100:1)^2 ~
# 0.5 kg·m² at the output shaft.  Helps numerical stability with high kp.
ARMATURE = 0.5

STL_DIR = os.path.join(THIS_DIR, "stl")

# Heightfield terrain: nrow x ncol grid covering 2*HFIELD_SIZE on each side
# of the spawn point.  ``HFIELD_MAX_Z`` is the elevation that maps to a
# normalized hfield value of 1.0; ``HFIELD_BASE`` is the slab thickness
# beneath the data plane that prevents the walker from falling through if
# the data ever goes to 0 at the boundary.
HFIELD_NROW = 96
HFIELD_NCOL = 96
HFIELD_SIZE = 18.0      # half-extent in metres (hfield spans 36 m × 36 m)
HFIELD_MAX_Z = 0.20
HFIELD_BASE = 0.5
HFIELD_SPAWN_FLAT_R = 2.5  # radius (m) of flat spawn pad at origin

# A stylized 1.75 m seated rider (~85 kg) for visual scale.  The mass is
# already lumped into CHASSIS_MASS, so the rider rides as a fixed visual
# attachment to the chassis -- no additional dynamics.
RIDER_STL = os.path.join(STL_DIR, "rider_seated.stl")


def make_terrain_heightmap(seed: int = 0) -> np.ndarray:
    """Procedural rolling-hill heightmap normalized to [0, 1].

    Produces a smooth blend of low-frequency sinusoids plus a few isolated
    larger bumps, then fades the elevation to zero inside a circular flat
    pad of radius ``HFIELD_SPAWN_FLAT_R`` around the origin so the walker
    can settle on level ground before venturing onto the terrain.
    """
    rng = np.random.default_rng(seed)
    xs = np.linspace(-HFIELD_SIZE, HFIELD_SIZE, HFIELD_NCOL)
    ys = np.linspace(-HFIELD_SIZE, HFIELD_SIZE, HFIELD_NROW)
    X, Y = np.meshgrid(xs, ys, indexing="xy")

    # Three octaves of smooth sinusoidal hills.
    h = (
        0.55 * np.sin(0.18 * X + 0.13 * Y + 0.7)
        + 0.40 * np.cos(0.12 * X - 0.21 * Y + 1.6)
        + 0.20 * np.sin(0.45 * X) * np.cos(0.42 * Y)
        + 0.15 * np.cos(0.31 * (X - 4.0)) * np.sin(0.27 * (Y + 3.0))
    )
    # A few isolated taller bumps to walk over.
    for _ in range(8):
        cx, cy = rng.uniform(-HFIELD_SIZE * 0.7, HFIELD_SIZE * 0.7, size=2)
        sigma = rng.uniform(1.5, 3.5)
        amp = rng.uniform(0.4, 0.85)
        h += amp * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * sigma ** 2))

    h -= h.min()
    if h.max() > 0:
        h /= h.max()

    # Fade to zero in a smooth ramp around the origin so the spawn area is
    # flat regardless of what the noise produced there.
    R = np.hypot(X, Y)
    fade = np.clip((R - HFIELD_SPAWN_FLAT_R) /
                   max(1e-3, HFIELD_SPAWN_FLAT_R), 0.0, 1.0)
    fade = fade ** 1.5
    h = h * fade

    # Renormalize so peaks reach 1.0 (max elevation = HFIELD_MAX_Z metres).
    if h.max() > 0:
        h = h / h.max()
    return h.astype(np.float32)


def sample_terrain_height(heights: np.ndarray, x: float, y: float) -> float:
    """Look up the terrain elevation (metres) at world XY by bilinear-
    interpolating the hfield."""
    nrow, ncol = heights.shape
    fx = (x + HFIELD_SIZE) / (2 * HFIELD_SIZE) * (ncol - 1)
    fy = (y + HFIELD_SIZE) / (2 * HFIELD_SIZE) * (nrow - 1)
    fx = max(0.0, min(ncol - 1.0001, fx))
    fy = max(0.0, min(nrow - 1.0001, fy))
    c0 = int(fx); r0 = int(fy)
    dx = fx - c0; dy = fy - r0
    h00 = heights[r0,     c0]
    h01 = heights[r0,     c0 + 1]
    h10 = heights[r0 + 1, c0]
    h11 = heights[r0 + 1, c0 + 1]
    h = (h00 * (1 - dx) * (1 - dy) + h01 * dx * (1 - dy)
         + h10 * (1 - dx) * dy     + h11 * dx * dy)
    return float(h) * HFIELD_MAX_Z


def make_obstacles_xml(heights: np.ndarray, *, count: int = 14,
                       seed: int = 0,
                       inner_radius: float = None,
                       outer_radius: float = None) -> str:
    """Procedurally place static obstacles (boxes, pillars, curbs, ramps)
    around the walker, avoiding the flat spawn pad.  Each obstacle's
    bottom is rested on the local terrain elevation."""
    if count <= 0:
        return ""
    if inner_radius is None:
        inner_radius = HFIELD_SPAWN_FLAT_R + 1.0
    if outer_radius is None:
        outer_radius = HFIELD_SIZE * 0.85

    rng = np.random.default_rng(seed * 31 + 17)
    placed = []   # list of (x, y, exclusion_radius)
    snippets = []

    PALETTES = {
        "crate":    "0.55 0.40 0.22 1",
        "pillar":   "0.55 0.55 0.58 1",
        "curb":     "0.85 0.55 0.18 1",
        "block":    "0.70 0.25 0.22 1",
        "ramp":     "0.55 0.55 0.58 1",
        "bollard":  "0.95 0.85 0.10 1",
    }

    def _nonoverlap(x, y, r):
        if math.hypot(x, y) - r < inner_radius:
            return False
        for px, py, pr in placed:
            if math.hypot(px - x, py - y) < r + pr + 0.30:
                return False
        return True

    def _pick_xy(r_excl):
        for _ in range(50):
            r = rng.uniform(inner_radius, outer_radius)
            theta = rng.uniform(0, 2 * math.pi)
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            if _nonoverlap(x, y, r_excl):
                return x, y
        return None

    types = (["crate"]   * 5 + ["pillar"] * 3 + ["curb"]    * 3
             + ["block"] * 2 + ["ramp"]   * 2 + ["bollard"] * 2)
    rng.shuffle(types)

    idx = 0
    for n in range(count):
        kind = types[n % len(types)]

        if kind == "crate":
            sx = rng.uniform(0.25, 0.45)
            sy = rng.uniform(0.25, 0.45)
            sz = rng.uniform(0.20, 0.45)
            yaw = rng.uniform(0, 2 * math.pi)
            r_excl = math.hypot(sx, sy) + 0.1
            xy = _pick_xy(r_excl)
            if xy is None: continue
            x, y = xy
            gz = sample_terrain_height(heights, x, y) + sz
            qw, qz = math.cos(yaw / 2), math.sin(yaw / 2)
            snippets.append(
                f'<geom name="obs_{idx}" type="box" '
                f'size="{sx:.3f} {sy:.3f} {sz:.3f}" '
                f'pos="{x:.3f} {y:.3f} {gz:.3f}" '
                f'quat="{qw:.4f} 0 0 {qz:.4f}" '
                f'rgba="{PALETTES["crate"]}" friction="1.5 0.05 0.0001"/>')
            placed.append((x, y, r_excl))

        elif kind == "pillar":
            r = rng.uniform(0.18, 0.35)
            h = rng.uniform(0.8, 2.4)
            r_excl = r + 0.2
            xy = _pick_xy(r_excl)
            if xy is None: continue
            x, y = xy
            gz = sample_terrain_height(heights, x, y) + h / 2.0
            snippets.append(
                f'<geom name="obs_{idx}" type="cylinder" '
                f'size="{r:.3f} {h/2:.3f}" '
                f'pos="{x:.3f} {y:.3f} {gz:.3f}" '
                f'rgba="{PALETTES["pillar"]}"/>')
            placed.append((x, y, r_excl))

        elif kind == "curb":
            sx = rng.uniform(0.8, 2.2)   # length
            sy = rng.uniform(0.10, 0.20)  # width (thin)
            sz = rng.uniform(0.06, 0.13)  # low so walker can step on it
            yaw = rng.uniform(0, 2 * math.pi)
            r_excl = math.hypot(sx, sy) + 0.1
            xy = _pick_xy(r_excl)
            if xy is None: continue
            x, y = xy
            gz = sample_terrain_height(heights, x, y) + sz
            qw, qz = math.cos(yaw / 2), math.sin(yaw / 2)
            snippets.append(
                f'<geom name="obs_{idx}" type="box" '
                f'size="{sx:.3f} {sy:.3f} {sz:.3f}" '
                f'pos="{x:.3f} {y:.3f} {gz:.3f}" '
                f'quat="{qw:.4f} 0 0 {qz:.4f}" '
                f'rgba="{PALETTES["curb"]}" friction="1.5 0.05 0.0001"/>')
            placed.append((x, y, r_excl))

        elif kind == "block":
            sx = rng.uniform(0.6, 1.2)
            sy = rng.uniform(0.6, 1.2)
            sz = rng.uniform(0.5, 1.0)
            yaw = rng.uniform(0, 2 * math.pi)
            r_excl = math.hypot(sx, sy) + 0.2
            xy = _pick_xy(r_excl)
            if xy is None: continue
            x, y = xy
            gz = sample_terrain_height(heights, x, y) + sz
            qw, qz = math.cos(yaw / 2), math.sin(yaw / 2)
            snippets.append(
                f'<geom name="obs_{idx}" type="box" '
                f'size="{sx:.3f} {sy:.3f} {sz:.3f}" '
                f'pos="{x:.3f} {y:.3f} {gz:.3f}" '
                f'quat="{qw:.4f} 0 0 {qz:.4f}" '
                f'rgba="{PALETTES["block"]}"/>')
            placed.append((x, y, r_excl))

        elif kind == "ramp":
            length = rng.uniform(2.0, 3.5)
            width = rng.uniform(1.0, 1.5)
            slope = rng.uniform(math.radians(8), math.radians(20))
            half_h = (length / 2.0) * math.sin(slope)
            half_x = (length / 2.0) * math.cos(slope)
            yaw = rng.uniform(0, 2 * math.pi)
            r_excl = length / 2 + 0.3
            xy = _pick_xy(r_excl)
            if xy is None: continue
            x, y = xy
            gz_base = sample_terrain_height(heights, x, y)
            # Box rotated by ``slope`` about its body Y so its long axis is
            # angled and its midpoint sits on the ground at one edge.
            # Place midpoint at height = gz_base + half_h so the lower edge
            # (after rotation) touches z = gz_base.
            cy = math.cos(yaw / 2) * math.cos(slope / 2)
            sy_q = math.cos(yaw / 2) * math.sin(slope / 2)
            sz_q = math.sin(yaw / 2) * math.cos(slope / 2)
            # Compose yaw-then-pitch quaternion (intrinsic):
            #   q = q_yaw * q_pitch
            # q_yaw = (cos(yaw/2), 0, 0, sin(yaw/2))
            # q_pitch = (cos(s/2), 0, sin(s/2), 0)
            cy_h, sy_h = math.cos(yaw / 2), math.sin(yaw / 2)
            cs_h, ss_h = math.cos(slope / 2), math.sin(slope / 2)
            qw = cy_h * cs_h
            qx = -sy_h * ss_h
            qy = cy_h * ss_h
            qz = sy_h * cs_h
            snippets.append(
                f'<geom name="obs_{idx}" type="box" '
                f'size="{length/2:.3f} {width/2:.3f} 0.05" '
                f'pos="{x:.3f} {y:.3f} {gz_base + half_h + 0.05:.3f}" '
                f'quat="{qw:.4f} {qx:.4f} {qy:.4f} {qz:.4f}" '
                f'rgba="{PALETTES["ramp"]}" friction="1.8 0.05 0.0001"/>')
            placed.append((x, y, r_excl))

        elif kind == "bollard":
            r = rng.uniform(0.10, 0.20)
            h = rng.uniform(0.4, 0.9)
            r_excl = r + 0.15
            xy = _pick_xy(r_excl)
            if xy is None: continue
            x, y = xy
            gz = sample_terrain_height(heights, x, y) + h / 2.0
            snippets.append(
                f'<geom name="obs_{idx}" type="cylinder" '
                f'size="{r:.3f} {h/2:.3f}" '
                f'pos="{x:.3f} {y:.3f} {gz:.3f}" '
                f'rgba="{PALETTES["bollard"]}"/>')
            placed.append((x, y, r_excl))
        idx += 1

    return "\n      ".join(snippets)


def _ensure_rider_stl(force: bool = False) -> str:
    """Build a Z-up, chassis-relative STL of the seated rider on demand.

    ``build_full_assembly._build_rider`` returns the rider in mm, Z-up, in
    a frame where Z=0 coincides with the chassis-hex centre (i.e. matches
    chassis-body origin in our MJCF).  We re-export that mesh, skipping
    the Y-up rotation that build_full_assembly does for Blender export.
    """
    if os.path.exists(RIDER_STL) and not force:
        return RIDER_STL
    import importlib
    bfa = importlib.import_module("build_full_assembly")
    rider_mesh = bfa._build_rider(chassis_lift=0.0)
    rider_mesh.export(RIDER_STL)
    return RIDER_STL


# ---------------------------------------------------------------------------
# Stance kinematics
# ---------------------------------------------------------------------------

def stance_foot_z_relative_to_hip() -> float:
    """Z of the foot tip in the chassis frame minus hip joint Z, at stance."""
    p = STANCE_FEMUR
    pt = STANCE_FEMUR + STANCE_TIBIA
    # Rotation about +Y_local: x'=x cos+z sin, z'=-x sin+z cos
    knee_z = -FEMUR * math.sin(p)             # +0.254 m (knee above hip)
    foot_z = knee_z - TIBIA * math.sin(pt)    # -0.205 m (foot below hip)
    return foot_z


def stance_chassis_height() -> float:
    """Height (m) at which the chassis-body origin should sit at rest.

    Foot collision sphere centre is placed at (TIBIA, 0, 0) in the tibia
    frame -- i.e. exactly at the tibia tip.  The sphere extends FOOT_R
    below the tip, so for the sphere bottom to sit on z=0 we need
    chassis_origin_z = -YAW_OUTPUT_HEIGHT - foot_z_rel + FOOT_R.
    """
    foot_z_rel = stance_foot_z_relative_to_hip()
    return -YAW_OUTPUT_HEIGHT - foot_z_rel + FOOT_R + 0.010  # 10 mm buffer


def foot_horizontal_reach() -> float:
    """Distance from the leg's yaw axis to the foot tip (in horizontal plane,
    at neutral stance pose).  This is the lever arm that converts a yaw
    rotation into foot tangential displacement."""
    p = STANCE_FEMUR
    pt = STANCE_FEMUR + STANCE_TIBIA
    return COXA + FEMUR * math.cos(p) + TIBIA * math.cos(pt)


D_FOOT = foot_horizontal_reach()


# ---------------------------------------------------------------------------
# MJCF builder
# ---------------------------------------------------------------------------

def build_xml(obstacles_xml: str = "") -> str:
    """Return the MuJoCo MJCF XML for the hexapod walker.

    ``obstacles_xml`` is an XML fragment containing one or more <geom>
    elements that will be inserted as static world obstacles (boxes,
    pillars, ramps...).  Empty string means no obstacles.
    """
    base_z = stance_chassis_height()

    # Six legs at angles a_i = (i + 0.5) * π/3 -- on hex *edges*, not vertices
    leg_data = []
    for i in range(6):
        a = (i + 0.5) * math.pi / 3.0
        # Body-attached coxa pivot in chassis frame (yaw axis location)
        x = LEG_RADIAL * math.cos(a)
        y = LEG_RADIAL * math.sin(a)
        z = YAW_OUTPUT_HEIGHT
        # Yaw frame: +X = outboard, +Y = tangential, +Z = up
        qw = math.cos(a / 2.0)
        qz = math.sin(a / 2.0)
        leg_data.append((i, a, x, y, z, qw, qz))

    leg_blocks = []
    actuator_blocks = []
    sensor_blocks = []
    for (i, a, x, y, z, qw, qz) in leg_data:
        leg_blocks.append(_leg_xml(i, x, y, z, qw, qz))
        actuator_blocks.append(_leg_actuators(i))
        sensor_blocks.append(_leg_sensors(i))

    legs_xml = "\n".join(leg_blocks)
    acts_xml = "\n".join(actuator_blocks)
    sens_xml = "\n".join(sensor_blocks)

    # MJCF
    return f"""<mujoco model="hexapod_walker">
  <compiler angle="radian" coordinate="local" autolimits="true" meshdir="{STL_DIR}"/>
  <option gravity="0 0 -9.81" timestep="0.002" iterations="50" solver="Newton" cone="elliptic"/>

  <default>
    <joint armature="{ARMATURE}" damping="2" limited="true"/>
    <geom solref="0.005 1" solimp="0.95 0.99 0.001"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="2" density="0"/>
    </default>
    <default class="collision">
      <geom group="3" rgba="1 0 0 0.25" condim="4" friction="1.5 0.05 0.0001"/>
    </default>
    <default class="foot">
      <geom group="3" rgba="0.15 0.15 0.15 1" condim="6" friction="2.0 0.1 0.001"
            solref="0.01 1" solimp="0.95 0.99 0.001"/>
    </default>
  </default>

  <asset>
    <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.7 0.85 1.0" rgb2="0.4 0.5 0.7" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.25 0.3 0.35"
             rgb2="0.18 0.22 0.27" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.15"/>
    <material name="alu"   rgba="0.78 0.80 0.83 1" specular="0.5" shininess="0.4"/>
    <material name="motor" rgba="0.2  0.2  0.22 1" specular="0.6" shininess="0.6"/>
    <material name="frame" rgba="0.55 0.58 0.62 1" specular="0.4" shininess="0.4"/>
    <material name="rubber" rgba="0.12 0.12 0.12 1"/>

    <mesh name="chassis_hex"      file="chassis_hex.stl"      scale="0.001 0.001 0.001"/>
    <mesh name="chassis_top_deck" file="chassis_top_deck.stl" scale="0.001 0.001 0.001"/>
    <mesh name="saddle_mount"     file="saddle_mount.stl"     scale="0.001 0.001 0.001"/>
    <mesh name="battery_box"      file="battery_box.stl"      scale="0.001 0.001 0.001"/>
    <mesh name="electronics_bay"  file="electronics_bay.stl"  scale="0.001 0.001 0.001"/>
    <mesh name="coxa_bracket"     file="coxa_bracket.stl"     scale="0.001 0.001 0.001"/>
    <mesh name="coxa_link"        file="coxa_link.stl"        scale="0.001 0.001 0.001"/>
    <mesh name="femur_link"       file="femur_link.stl"       scale="0.001 0.001 0.001"/>
    <mesh name="tibia_link"       file="tibia_link.stl"       scale="0.001 0.001 0.001"/>
    <mesh name="foot_pad"         file="foot_pad.stl"         scale="0.001 0.001 0.001"/>
    <mesh name="rider_seated"     file="rider_seated.stl"     scale="0.001 0.001 0.001"/>

    <material name="denim"  rgba="0.30 0.36 0.50 1" specular="0.1" shininess="0.05"/>
    <material name="skin"   rgba="0.86 0.70 0.55 1" specular="0.05" shininess="0.05"/>
    <texture name="terrain_tex" type="2d" builtin="checker"
             rgb1="0.34 0.45 0.30" rgb2="0.26 0.36 0.22"
             width="320" height="320"/>
    <material name="terrain_mat" texture="terrain_tex"
              texrepeat="20 20" reflectance="0.05" shininess="0.0"/>
    <hfield name="terrain"
            nrow="{HFIELD_NROW}" ncol="{HFIELD_NCOL}"
            size="{HFIELD_SIZE} {HFIELD_SIZE} {HFIELD_MAX_Z} {HFIELD_BASE}"/>
  </asset>

  <worldbody>
    <light name="sun" pos="6 -4 8" dir="-0.6 0.4 -1" castshadow="true"
           diffuse="0.7 0.7 0.7" specular="0.3 0.3 0.3"/>
    <light name="fill" pos="-4 4 6" dir="0.5 -0.5 -1" diffuse="0.3 0.3 0.35"/>
    <!-- Out-of-bounds backup plane (in case the walker leaves the hfield) -->
    <geom name="floor" type="plane" size="200 200 0.05"
          pos="0 0 -0.001" material="grid"
          friction="1.5 0.05 0.0001"/>
    <!-- Procedural rolling-hills terrain.  Heights are populated from
         Python after the model is loaded (see _populate_hfield). -->
    <geom name="terrain" type="hfield" hfield="terrain"
          material="terrain_mat" friction="1.5 0.05 0.0001"
          condim="4"/>

    <!-- Procedural static obstacles (placed by make_obstacles_xml). -->
      {obstacles_xml}

    <body name="chassis" pos="0 0 {base_z:.5f}">
      <freejoint name="root"/>
      <inertial pos="0 0 0.05" mass="{CHASSIS_MASS}"
                diaginertia="32 32 60"/>

      <!-- visual chassis tubes + deck + saddle/battery/elec stack -->
      <geom class="visual" type="mesh" mesh="chassis_hex"      material="frame"/>
      <geom class="visual" type="mesh" mesh="chassis_top_deck" material="alu"
            pos="0 0 0.072"/>
      <geom class="visual" type="mesh" mesh="saddle_mount"     material="alu"
            pos="0 0 0.078"/>
      <geom class="visual" type="mesh" mesh="battery_box"      material="frame"
            pos="-0.26 0 0.078"/>
      <geom class="visual" type="mesh" mesh="electronics_bay"  material="frame"
            pos="0.26 0 0.078"/>

      <!-- collision: a hex-prism approximated by a single oriented box -->
      <geom class="collision" name="chassis_box" type="box"
            pos="0 0 0.05" size="0.7 0.65 0.07"/>

      <!-- seated rider (visual only; mass is already in chassis inertia) -->
      <geom class="visual" type="mesh" mesh="rider_seated" material="denim"/>

{legs_xml}
    </body>
  </worldbody>

  <actuator>
{acts_xml}
  </actuator>

  <sensor>
{sens_xml}
    <accelerometer name="chassis_acc" site="chassis_imu"/>
    <gyro          name="chassis_gyro" site="chassis_imu"/>
    <framepos      name="chassis_pos"  objtype="body" objname="chassis"/>
    <framezaxis    name="chassis_up"   objtype="body" objname="chassis"/>
  </sensor>
</mujoco>
"""


def _leg_xml(i: int, x: float, y: float, z: float,
             qw: float, qz: float) -> str:
    """Hierarchical XML for one leg.  Local +X = outboard."""
    # Foot collision sphere lives at (TIBIA, 0, 0) in tibia frame -- its centre
    # is at the foot tip; the sphere extends FOOT_R below.
    # Coxa bracket is bolted to the chassis (it doesn't rotate with yaw); we
    # render it on the chassis side, at edge_mid + (TUBE/2)*outboard, in the
    # leg's rotated frame -- i.e. at position (TUBE/2 + APOTHEM - LEG_RADIAL)
    # along +X relative to the yaw axis.  But for simplicity we attach
    # cosmetic chassis-side meshes at the chassis level where they belong.
    return f"""      <body name="L{i}_yaw" pos="{x:.5f} {y:.5f} {z:.5f}" quat="{qw:.6f} 0 0 {qz:.6f}">
        <inertial pos="0.05 0 0" mass="{COXA_MASS}"
                  diaginertia="0.05 0.05 0.05"/>
        <joint name="L{i}_yaw" type="hinge" axis="0 0 1"
               range="-0.7854 0.7854"/>
        <geom class="visual" type="mesh" mesh="coxa_link"  material="alu"/>
        <!-- yaw motor housing (decorative) -->
        <geom class="visual" type="cylinder" material="motor"
              pos="0 0 -0.07" size="{MOTOR_OD/2:.4f} 0.05"/>

        <body name="L{i}_femur" pos="{COXA:.5f} 0 0">
          <inertial pos="{FEMUR/2:.4f} 0 0" mass="{FEMUR_MASS}"
                    diaginertia="0.05 0.45 0.45"/>
          <joint name="L{i}_pitch" type="hinge" axis="0 1 0"
                 range="-1.4 0.7"/>
          <geom class="visual" type="mesh" mesh="femur_link" material="alu"/>
          <!-- femur capsule kept for inertia approximation only (no
               collision: ground contact happens at the foot sphere) -->
          <geom class="visual" type="capsule" rgba="0 0 0 0"
                fromto="0 0 0 {FEMUR:.5f} 0 0" size="0.06"/>

          <body name="L{i}_tibia" pos="{FEMUR:.5f} 0 0">
            <inertial pos="{TIBIA/2:.4f} 0 0" mass="{TIBIA_MASS}"
                      diaginertia="0.04 0.65 0.65"/>
            <joint name="L{i}_knee" type="hinge" axis="0 1 0"
                   range="-0.3 1.7"/>
            <geom class="visual" type="mesh" mesh="tibia_link" material="alu"/>
            <geom class="visual" type="capsule" rgba="0 0 0 0"
                  fromto="0 0 0 {TIBIA - FOOT_R:.5f} 0 0" size="0.05"/>
            <!-- foot pad (visual) -->
            <geom class="visual" type="mesh" mesh="foot_pad" material="rubber"
                  pos="{TIBIA:.5f} 0 -0.072" euler="0 0 0"/>
            <!-- foot collision sphere centred at the tibia tip; the
                 sphere bottom is what makes ground contact -->
            <geom class="foot" name="L{i}_foot" type="sphere"
                  pos="{TIBIA:.5f} 0 0" size="{FOOT_R}"/>
            <site name="L{i}_foot_site" pos="{TIBIA:.5f} 0 0"
                  size="0.01"/>
          </body>
        </body>
      </body>"""


def _leg_actuators(i: int) -> str:
    return f"""    <position name="L{i}_yaw"   joint="L{i}_yaw"   kp="{KP_YAW}"
              forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <position name="L{i}_pitch" joint="L{i}_pitch" kp="{KP_PITCH}"
              forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <position name="L{i}_knee"  joint="L{i}_knee"  kp="{KP_KNEE}"
              forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_yaw_d"   joint="L{i}_yaw"   kv="{DAMP_YAW}"
              forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_pitch_d" joint="L{i}_pitch" kv="{DAMP_PITCH}"
              forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_knee_d"  joint="L{i}_knee"  kv="{DAMP_KNEE}"
              forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>"""


def _leg_sensors(i: int) -> str:
    return f"""    <jointpos name="L{i}_yaw_p"    joint="L{i}_yaw"/>
    <jointpos name="L{i}_pitch_p"  joint="L{i}_pitch"/>
    <jointpos name="L{i}_knee_p"   joint="L{i}_knee"/>
    <touch    name="L{i}_foot_t"   site="L{i}_foot_site"/>"""


# A fake site in the chassis is needed for the IMU sensor; add it once after
# the fact (we can't easily put it in the f-string above without losing
# clarity).  Just patch the XML string.
def _patch_imu_site(xml: str) -> str:
    return xml.replace(
        '<freejoint name="root"/>',
        '<freejoint name="root"/>\n      '
        '<site name="chassis_imu" pos="0 0 0.06" size="0.02"/>'
    )


# ---------------------------------------------------------------------------
# Reusable model construction (used by main() and by HexapodWalkerEnv)
# ---------------------------------------------------------------------------

def build_world(*, terrain_seed: int = 0, terrain_enabled: bool = True,
                obstacle_count: int = 14, obstacle_seed: int = 0,
                ensure_rider: bool = True):
    """Generate the MJCF XML, terrain heightmap and a loaded MjModel.

    Returns ``(model, data, heights)`` where ``heights`` is the (nrow, ncol)
    terrain heightmap (already populated into ``model.hfield_data``).

    Used by both the CLI runner and the Gymnasium environment so they
    share exactly the same terrain / obstacle / model code.
    """
    if ensure_rider:
        _ensure_rider_stl()

    if terrain_enabled:
        heights = make_terrain_heightmap(seed=terrain_seed)
    else:
        heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)

    obstacles_xml = make_obstacles_xml(heights, count=obstacle_count,
                                        seed=obstacle_seed)
    xml = _patch_imu_site(build_xml(obstacles_xml=obstacles_xml))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    _populate_hfield(model, heights)
    return model, data, heights


# ---------------------------------------------------------------------------
# Tripod-gait controller
# ---------------------------------------------------------------------------

def _leg_ik(target_xyz_in_yaw_frame):
    """Solve (yaw_yaw_joint=0 because target is already in yaw frame, pitch, knee)
    for the 3-DOF leg so the foot tip lands at ``target_xyz_in_yaw_frame``.

    The leg's kinematic chain in the yaw frame:
        hip joint  : (COXA, 0, 0)
        knee joint : hip + R_y(p) @ (FEMUR, 0, 0)
        foot tip   : knee + R_y(p+pt) @ (TIBIA, 0, 0)

    Y-coordinate of the target is ignored (the leg is planar in YAW-XZ).
    The yaw joint angle should be set independently to point the leg's +X
    axis at the foot's azimuth.

    Returns (pitch, knee) in radians, or None if the target is unreachable.
    """
    # Convention:
    #   x_foot = COXA + FEMUR cos(p) + TIBIA cos(p + pt)
    #   z_foot = -FEMUR sin(p) - TIBIA sin(p + pt)
    # Let u = x_foot - COXA, w = -z_foot (so target is in standard 2R "up"
    # convention with both links measured CCW from +X).  Then:
    #   u = F cos(p) + T cos(p + pt)
    #   w = F sin(p) + T sin(p + pt)
    # Standard 2-link IK:
    u =  float(target_xyz_in_yaw_frame[0]) - COXA
    w = -float(target_xyz_in_yaw_frame[2])
    L = math.hypot(u, w)
    if L > FEMUR + TIBIA - 1e-6 or L < abs(FEMUR - TIBIA) + 1e-6:
        return None
    cos_pt = (L * L - FEMUR * FEMUR - TIBIA * TIBIA) / (2 * FEMUR * TIBIA)
    cos_pt = max(-1.0, min(1.0, cos_pt))
    pt = math.acos(cos_pt)              # elbow-down solution (pt > 0)
    p = math.atan2(w, u) - math.atan2(TIBIA * math.sin(pt),
                                       FEMUR + TIBIA * math.cos(pt))
    return p, pt


class TripodGait:
    """Alternating-tripod gait commanded by a body-frame twist (vx, vy, ω).

    Body axes: +X = body-forward, +Y = body-left, +Z = up.  Set ``vx``,
    ``vy`` and ``omega`` (rad/s, +CCW about Z) on the gait object at any
    time and the next call to ``desired()`` will plan accordingly --
    these are the live-tunable inputs the keyboard controller writes to.

    For each foot at body-frame angle a_i and radius R, the rigid-body
    velocity of the foot's neutral position induced by (vx, vy, ω) is
        V_at_foot = (vx - ω R sin a, vy + ω R cos a).
    Since the foot stays planted in the world during stance, in the body
    frame it must move at -V_at_foot.  Symmetric placement around the
    neutral pose gives the swept body-frame displacement
        Δp_body = prog · V_at_foot · T/2,
    where ``prog`` is +0.5 at the start of stance and -0.5 at the end
    (and runs the opposite way during swing).
    """

    MAX_VX = 1.5      # hard caps for keyboard control safety
    MAX_VY = 1.5
    MAX_OMEGA = 0.6

    def __init__(self, *, period: float = 1.0, lift: float = 0.07,
                 ramp: float = 0.4, vx: float = 0.0, vy: float = 0.0,
                 omega: float = 0.0):
        self.period = period
        self.lift = lift
        self.ramp = max(ramp, 1e-3)

        self.vx = vx
        self.vy = vy
        self.omega = omega

        self.leg_angles = [(i + 0.5) * math.pi / 3.0 for i in range(6)]
        p, pt = STANCE_FEMUR, STANCE_FEMUR + STANCE_TIBIA
        self.foot_neutral_x = COXA + FEMUR * math.cos(p) + TIBIA * math.cos(pt)
        self.foot_neutral_z = -FEMUR * math.sin(p) - TIBIA * math.sin(pt)
        self._foot_radius = LEG_RADIAL + self.foot_neutral_x
        self._phase_offset = math.pi / 2.0
        self._fallback = (0.0, STANCE_FEMUR, STANCE_TIBIA)
        # First-order low-pass on the velocity command so keyboard step
        # changes don't shock the leg trajectories.
        self._vx_smooth = vx
        self._vy_smooth = vy
        self._om_smooth = omega
        self._last_t = None

    def set_velocity(self, *, vx=None, vy=None, omega=None):
        if vx is not None:
            self.vx = max(-self.MAX_VX, min(self.MAX_VX, float(vx)))
        if vy is not None:
            self.vy = max(-self.MAX_VY, min(self.MAX_VY, float(vy)))
        if omega is not None:
            self.omega = max(-self.MAX_OMEGA, min(self.MAX_OMEGA, float(omega)))

    def stop(self):
        self.set_velocity(vx=0.0, vy=0.0, omega=0.0)

    def _smoothed_command(self, t: float):
        """Critically-damped 1st-order low-pass toward (vx, vy, omega)."""
        if self._last_t is None:
            self._last_t = t
            return self.vx, self.vy, self.omega
        dt = max(1e-4, t - self._last_t)
        self._last_t = t
        tau = 0.20  # 200 ms time constant
        a = 1.0 - math.exp(-dt / tau)
        self._vx_smooth += a * (self.vx - self._vx_smooth)
        self._vy_smooth += a * (self.vy - self._vy_smooth)
        self._om_smooth += a * (self.omega - self._om_smooth)
        return self._vx_smooth, self._vy_smooth, self._om_smooth

    def _foot_target_in_body(self, i: int, t: float, vx, vy, omega):
        """Return (Δx_body, Δy_body, Δz_lift) for leg ``i`` at time ``t``."""
        T = self.period
        phase = (2 * math.pi * t / T + self._phase_offset) % (2 * math.pi)
        ramp_amp = min(t / self.ramp, 1.0)

        tripod = 0 if i % 2 == 0 else 1
        phi = (phase + tripod * math.pi) % (2 * math.pi)
        if phi < math.pi:                          # swing (foot in air)
            s = phi / math.pi
            prog = -0.5 + s                        # -0.5 -> +0.5
            dz = self.lift * ramp_amp * math.sin(math.pi * s)
        else:                                      # stance (foot planted)
            s = (phi - math.pi) / math.pi
            prog = 0.5 - s                         # +0.5 -> -0.5
            dz = 0.0

        a_i = self.leg_angles[i]
        sa, ca = math.sin(a_i), math.cos(a_i)
        # Foot's neutral body-frame velocity due to body twist:
        v_x_at = vx - omega * self._foot_radius * sa
        v_y_at = vy + omega * self._foot_radius * ca
        # Symmetric body-frame displacement during stance/swing:
        dx = prog * v_x_at * T / 2.0 * ramp_amp
        dy = prog * v_y_at * T / 2.0 * ramp_amp
        return dx, dy, dz

    def desired(self, t: float):
        """Return (yaws, pitches, knees) target arrays of length 6 at time t."""
        vx, vy, omega = self._smoothed_command(t)

        yaws = np.zeros(6)
        pitches = np.full(6, STANCE_FEMUR)
        knees = np.full(6, STANCE_TIBIA)

        for i, a in enumerate(self.leg_angles):
            dx_b, dy_b, dz_b = self._foot_target_in_body(i, t, vx, vy, omega)
            fx_b_neutral = self._foot_radius * math.cos(a)
            fy_b_neutral = self._foot_radius * math.sin(a)
            fx_b = fx_b_neutral + dx_b
            fy_b = fy_b_neutral + dy_b
            yaw_origin_x = LEG_RADIAL * math.cos(a)
            yaw_origin_y = LEG_RADIAL * math.sin(a)
            rx = fx_b - yaw_origin_x
            ry = fy_b - yaw_origin_y
            ca, sa = math.cos(a), math.sin(a)
            x_yaw =  ca * rx + sa * ry
            y_yaw = -sa * rx + ca * ry
            yaw_angle = math.atan2(y_yaw, x_yaw)
            r_planar = math.hypot(x_yaw, y_yaw)
            target_yz = (r_planar, 0.0, self.foot_neutral_z + dz_b)
            ik = _leg_ik(target_yz)
            if ik is None:
                yaws[i] = self._fallback[0]
                pitches[i] = self._fallback[1]
                knees[i]   = self._fallback[2]
            else:
                p, k = ik
                yaws[i]    = yaw_angle
                pitches[i] = p
                knees[i]   = k
        return yaws, pitches, knees


# ---------------------------------------------------------------------------
# Simulation drivers
# ---------------------------------------------------------------------------

def _set_targets(model: mujoco.MjModel, data: mujoco.MjData,
                 yaws, pitches, knees):
    for i in range(6):
        # Position actuators come first (yaw, pitch, knee) -- order set up
        # in build_xml.  Indices: leg i offset = i*6; first 3 are position,
        # next 3 are velocity (which we leave at zero target = pure damping).
        base = i * 6
        data.ctrl[base + 0] = yaws[i]
        data.ctrl[base + 1] = pitches[i]
        data.ctrl[base + 2] = knees[i]
        # velocity actuators stay at 0 -> they act as joint dampers.


def _initial_pose(data: mujoco.MjData):
    for i in range(6):
        base = i * 6
        data.ctrl[base + 0] = 0.0
        data.ctrl[base + 1] = STANCE_FEMUR
        data.ctrl[base + 2] = STANCE_TIBIA


def _populate_hfield(model: mujoco.MjModel, heights: np.ndarray):
    """Copy a (nrow, ncol) heights array (values in [0, 1]) into the model.

    MuJoCo stores all heightfield samples in a single flat buffer
    ``model.hfield_data``; for a single hfield it is just ``data.flatten()``.
    """
    hf_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
    nrow = int(model.hfield_nrow[hf_id])
    ncol = int(model.hfield_ncol[hf_id])
    expected = nrow * ncol
    flat = np.asarray(heights, dtype=np.float32).reshape(nrow, ncol).flatten()
    if flat.size != expected:
        raise ValueError(f"hfield expects {expected} samples, got {flat.size}")
    start = int(model.hfield_adr[hf_id])
    model.hfield_data[start:start + expected] = flat


def _set_stance_qpos(model: mujoco.MjModel, data: mujoco.MjData):
    """Set free-joint + 18 hinge positions so the walker is born standing."""
    base_z = stance_chassis_height()
    data.qpos[0:3] = [0.0, 0.0, base_z]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # identity quaternion (w x y z)
    for i in range(6):
        # qpos layout after free-joint: 6 legs × (yaw, pitch, knee).
        # Joint indexing follows body order in MJCF (L0_yaw, L0_pitch,
        # L0_knee, L1_yaw, ...).
        q_yaw   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"L{i}_yaw")
        q_pitch = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"L{i}_pitch")
        q_knee  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    f"L{i}_knee")
        data.qpos[model.jnt_qposadr[q_yaw]]   = 0.0
        data.qpos[model.jnt_qposadr[q_pitch]] = STANCE_FEMUR
        data.qpos[model.jnt_qposadr[q_knee]]  = STANCE_TIBIA
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


_HELP_TEXT = """
hexapod walker controls (focus the MuJoCo window first)
-------------------------------------------------------
  Arrow Up / Down     forward / back   (body-frame +X / -X), ±0.15 m/s per tap
  Arrow Left / Right  strafe L / R     (body-frame +Y / -Y), ±0.15 m/s per tap
  Page Up / Page Down turn left / right (CCW / CW),         ±0.10 rad/s per tap
  Home                full stop (zero vx, vy, omega)
  End                 reset chassis to spawn pad + zero twist
  Insert / Delete     +/- 50% to all current twist components (slow / boost)

  These keys are deliberately chosen to NOT collide with MuJoCo's built-in
  viewer keys (Space=pause, W=wireframe, C=contacts, F=forces, T=transparent,
  R=record video, 0..5=geom group toggles, etc.).  Numeric keys and most
  letters will toggle MuJoCo's own visualization flags -- avoid them.

  Mouse + MuJoCo built-ins
    Left-drag       rotate camera                 Right-drag   pan
    Scroll          zoom                          Double-click focus on body
    Ctrl+drag       apply force to a body         Tab          hide UI panel
    Space           pause / resume the simulation
    Backspace       reset to MJCF keyframe
"""


# GLFW key constants used by MuJoCo's viewer (mirrors glfw.* constants).
_GK_SPACE = 32
_GK_BACKTICK = 96
_GK_RIGHT = 262
_GK_LEFT = 263
_GK_DOWN = 264
_GK_UP = 265
_GK_PAGE_UP = 266
_GK_PAGE_DOWN = 267
_GK_HOME = 268
_GK_END = 269
_GK_INSERT = 260
_GK_DELETE = 261


def _make_key_callback(gait, on_reset):
    """Build a key_callback fn for mujoco.viewer.launch_passive that uses
    only keys MuJoCo doesn't bind to anything else."""
    DV = 0.15
    DOM = 0.10

    def status():
        print(f"  twist now: vx={gait.vx:+.2f} m/s  vy={gait.vy:+.2f} m/s  "
              f"omega={gait.omega:+.2f} rad/s", flush=True)

    def cb(keycode):
        kc = int(keycode)
        if   kc == _GK_UP:        gait.set_velocity(vx=gait.vx + DV)
        elif kc == _GK_DOWN:      gait.set_velocity(vx=gait.vx - DV)
        elif kc == _GK_LEFT:      gait.set_velocity(vy=gait.vy + DV)
        elif kc == _GK_RIGHT:     gait.set_velocity(vy=gait.vy - DV)
        elif kc == _GK_PAGE_UP:   gait.set_velocity(omega=gait.omega + DOM)
        elif kc == _GK_PAGE_DOWN: gait.set_velocity(omega=gait.omega - DOM)
        elif kc == _GK_HOME:      gait.stop()
        elif kc == _GK_END:
            on_reset()
            gait.stop()
        elif kc == _GK_INSERT:
            gait.set_velocity(vx=gait.vx * 1.5, vy=gait.vy * 1.5,
                               omega=gait.omega * 1.5)
        elif kc == _GK_DELETE:
            gait.set_velocity(vx=gait.vx * 0.5, vy=gait.vy * 0.5,
                               omega=gait.omega * 0.5)
        elif kc == _GK_BACKTICK:
            print(_HELP_TEXT)
        else:
            return  # not one of ours; let MuJoCo handle it
        status()
    return cb


def _print_status_line(t, gait, body_pos):
    print(f"  t={t:6.2f}s  pos=({body_pos[0]:+6.2f},{body_pos[1]:+6.2f},"
          f"{body_pos[2]:+5.2f})m  cmd=(vx={gait.vx:+.2f},vy={gait.vy:+.2f},"
          f"ω={gait.omega:+.2f})", flush=True)


def run_viewer(model, data, gait: TripodGait, *, run_gait: bool = True,
               settle: float = 0.8):
    import mujoco.viewer as viewer  # type: ignore
    print(_HELP_TEXT)
    print("Launching MuJoCo viewer.  Close the window to exit.")

    pos_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    def _do_reset():
        _set_stance_qpos(model, data)

    _do_reset()
    settle_pose_until = data.time + settle
    settle_started = data.time

    cb = _make_key_callback(gait, on_reset=_do_reset) if run_gait else None

    last_status = -1.0
    with viewer.launch_passive(model, data, key_callback=cb) as v:
        t_origin = data.time
        while v.is_running():
            t_local = data.time - t_origin
            in_settle = data.time < settle_pose_until
            if in_settle or not run_gait:
                _initial_pose(data)
            else:
                yaws, pitches, knees = gait.desired(t_local - settle)
                _set_targets(model, data, yaws, pitches, knees)
            mujoco.mj_step(model, data)
            v.sync()
            if data.time - last_status > 1.0:
                _print_status_line(data.time, gait,
                                   np.array(data.body(pos_idx).xpos))
                last_status = data.time
            time.sleep(max(0.0, model.opt.timestep - 1e-4))


def run_headless(model, data, gait: TripodGait, *, duration: float,
                 settle: float = 0.8, run_gait: bool = True):
    n_settle = int(settle / model.opt.timestep)
    n_walk = int(duration / model.opt.timestep)

    _set_stance_qpos(model, data)
    pos_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    print(f"Settling for {settle:.1f} s ...")
    for _ in range(n_settle):
        _initial_pose(data)
        mujoco.mj_step(model, data)
    p0 = np.array(data.body(pos_idx).xpos)
    print(f"  chassis after settle: pos = {p0} m")

    label = "Walking" if run_gait else "Holding stance"
    print(f"{label} for {duration:.1f} s ...")
    log = []
    for k in range(n_walk):
        t = k * model.opt.timestep
        if run_gait:
            yaws, pitches, knees = gait.desired(t)
            _set_targets(model, data, yaws, pitches, knees)
        else:
            _initial_pose(data)
        mujoco.mj_step(model, data)
        if k % 50 == 0:  # 10 Hz log
            p = np.array(data.body(pos_idx).xpos)
            log.append((t, *p))
    p1 = np.array(data.body(pos_idx).xpos)
    qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
    yaw = math.atan2(2 * (qw * qz + qx * qy),
                     1 - 2 * (qy * qy + qz * qz))
    delta = p1 - p0
    horiz = math.hypot(delta[0], delta[1])
    print(f"  chassis after walk:   pos = {p1} m")
    print(f"  Δposition = ({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f}) m")
    print(f"  horizontal travel    = {horiz:.3f} m  in {duration:.2f} s "
          f"-> avg speed {horiz/duration:.3f} m/s")
    print(f"  chassis yaw          = {math.degrees(yaw):+.2f}°  "
          f"({yaw/duration:+.3f} rad/s)")
    return log


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--headless", action="store_true",
                    help="Run without launching the GUI viewer")
    ap.add_argument("--duration", type=float, default=6.0,
                    help="Headless walk duration in seconds (default: 6.0)")
    ap.add_argument("--period", type=float, default=1.0,
                    help="Gait cycle period in seconds (smaller = faster)")
    ap.add_argument("--lift", type=float, default=0.07,
                    help="Foot lift height during swing in metres")
    ap.add_argument("--ramp", type=float, default=0.4,
                    help="Time (s) for stride/lift to ramp from 0 -> full")
    # New body-twist command surface
    ap.add_argument("--vx", type=float, default=None,
                    help="Body-frame forward velocity (m/s)")
    ap.add_argument("--vy", type=float, default=None,
                    help="Body-frame leftward velocity (m/s)")
    ap.add_argument("--omega", type=float, default=None,
                    help="Body angular velocity (rad/s, +CCW)")
    # Legacy command surface (still supported, mapped to vx/vy/omega)
    ap.add_argument("--stride", type=float, default=None,
                    help="[Legacy] stride length in metres; mapped to "
                         "vx/vy via --fwd-dir")
    ap.add_argument("--fwd-dir", type=float, default=0.0,
                    help="[Legacy] walking direction (rad, 0=+X) for --stride")
    ap.add_argument("--turn", type=float, default=None,
                    help="[Legacy] turn rate (rad/s, +CCW); maps to --omega")
    ap.add_argument("--no-gait", action="store_true",
                    help="Hold stance pose only, do not walk")
    ap.add_argument("--no-terrain", action="store_true",
                    help="Skip the procedural terrain (flat floor only)")
    ap.add_argument("--terrain-seed", type=int, default=0,
                    help="RNG seed for the procedural terrain")
    ap.add_argument("--obstacles", type=int, default=14,
                    help="Number of procedural obstacles to spawn "
                         "(0 to disable)")
    ap.add_argument("--obstacle-seed", type=int, default=0,
                    help="RNG seed for obstacle placement")
    ap.add_argument("--dump-xml", default=None,
                    help="Write the generated MJCF XML to this path and exit")
    args = ap.parse_args()

    # Resolve gait command: explicit --vx/--vy/--omega win over the legacy
    # --stride/--fwd-dir/--turn knobs.  If neither is given, walker stands.
    vx = args.vx
    vy = args.vy
    omega = args.omega
    if args.stride is not None:
        v_lin = 2.0 * args.stride / args.period
        if vx is None:
            vx = v_lin * math.cos(args.fwd_dir)
        if vy is None:
            vy = v_lin * math.sin(args.fwd_dir)
    if args.turn is not None and omega is None:
        omega = args.turn
    vx = 0.0 if vx is None else vx
    vy = 0.0 if vy is None else vy
    omega = 0.0 if omega is None else omega

    print(f"Stance chassis height = {stance_chassis_height():.3f} m")
    print(f"Foot reach (radius from chassis centre) ≈ "
          f"{LEG_RADIAL + COXA + FEMUR*math.cos(STANCE_FEMUR) + TIBIA*math.cos(STANCE_FEMUR+STANCE_TIBIA):.3f} m")

    rider_path = _ensure_rider_stl()
    print(f"Rider mesh: {rider_path}")

    # Generate terrain BEFORE building the XML so obstacle placement can
    # sample the local elevation and rest each obstacle on the ground.
    if args.no_terrain:
        heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)
    else:
        heights = make_terrain_heightmap(seed=args.terrain_seed)

    obstacles_xml = make_obstacles_xml(heights, count=args.obstacles,
                                        seed=args.obstacle_seed)
    n_obstacles = obstacles_xml.count("<geom ")

    xml = build_xml(obstacles_xml=obstacles_xml)
    xml = _patch_imu_site(xml)

    if args.dump_xml:
        with open(args.dump_xml, "w") as f:
            f.write(xml)
        print(f"Wrote {args.dump_xml} ({len(xml)} bytes)")
        return

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    _populate_hfield(model, heights)
    print(f"Terrain: {HFIELD_NROW}×{HFIELD_NCOL} hfield "
          f"({2*HFIELD_SIZE:.0f} m × {2*HFIELD_SIZE:.0f} m, peak "
          f"{heights.max()*HFIELD_MAX_Z:.2f} m)  "
          f"+ {n_obstacles} obstacle(s)")

    print(f"Loaded MuJoCo model: nq={model.nq} nv={model.nv} "
          f"nu={model.nu} nbody={model.nbody}")

    gait = TripodGait(period=args.period, lift=args.lift, ramp=args.ramp,
                      vx=vx, vy=vy, omega=omega)
    print(f"Gait: T={args.period:.2f}s lift={args.lift:.2f}m  "
          f"twist=(vx={vx:+.2f}, vy={vy:+.2f}, ω={omega:+.2f})")

    if args.headless:
        run_headless(model, data, gait, duration=args.duration,
                     run_gait=not args.no_gait)
    else:
        run_viewer(model, data, gait, run_gait=not args.no_gait)


if __name__ == "__main__":
    main()
