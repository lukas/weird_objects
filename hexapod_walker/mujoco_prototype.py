"""MuJoCo simulation of the tabletop hexapod prototype.

This is the prototype-scale counterpart to ``mujoco_walker.py``.  It keeps
the same public API used by the RL stack:

    build_world(...)
    TripodGait
    _set_stance_qpos(...)

so ``hexapod_env.py``, ``train_walker.py`` and ``rollout_walker.py`` can be
reused for the prototype with only module aliasing.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import mujoco
import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)
import hexapod_prototype as HP  # noqa: E402


M = 0.001  # mm -> m

APOTHEM = HP.CHASSIS_FLAT_TO_FLAT / 2.0 * M
COXA = HP.COXA_LENGTH * M
FEMUR = HP.FEMUR_LENGTH * M
TIBIA = HP.TIBIA_LENGTH * M
STANCE_FEMUR = math.radians(HP.STANCE_FEMUR_DEG)
STANCE_TIBIA = math.radians(HP.STANCE_TIBIA_DEG)

PLASTIC_HORN_H = 5.0
YAW_OUTPUT_HEIGHT = (
    (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
    + HP.SERVO_OUTPUT_H
    + PLASTIC_HORN_H
    + HP.HORN_ADAPTER_T
) * M
LEG_RADIAL = APOTHEM

CHASSIS_MASS = 0.55
COXA_MASS = 0.055
FEMUR_MASS = 0.075
TIBIA_MASS = 0.055

# DS3225-class hobby servos are around 25 kg-cm ~= 2.45 N*m.  Keep a little
# below that so the sim does not assume impossible torque.
TORQUE_LIMIT = 2.2
KP_YAW = 18.0
KP_PITCH = 26.0
KP_KNEE = 24.0
DAMP_YAW = 0.35
DAMP_PITCH = 0.45
DAMP_KNEE = 0.40
ARMATURE = 0.0004

FOOT_R = HP.FOOT_PAD_OD / 2.0 * M

HFIELD_NROW = 128
HFIELD_NCOL = 128
HFIELD_SIZE = 2.5
HFIELD_MAX_Z = 0.018
HFIELD_BASE = 0.08
HFIELD_SPAWN_FLAT_R = 0.32


def make_terrain_heightmap(seed: int = 0) -> np.ndarray:
    """Small indoor-scale terrain normalized to [0, 1]."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(-HFIELD_SIZE, HFIELD_SIZE, HFIELD_NCOL)
    ys = np.linspace(-HFIELD_SIZE, HFIELD_SIZE, HFIELD_NROW)
    X, Y = np.meshgrid(xs, ys, indexing="xy")

    h = (
        0.40 * np.sin(4.0 * X + 0.7)
        + 0.30 * np.cos(3.1 * Y - 0.2)
        + 0.20 * np.sin(8.0 * X + 5.0 * Y)
    )
    for _ in range(5):
        cx, cy = rng.uniform(-1.8, 1.8, size=2)
        sigma = rng.uniform(0.10, 0.28)
        amp = rng.uniform(0.15, 0.45)
        h += amp * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * sigma ** 2))

    h -= h.min()
    if h.max() > 0:
        h /= h.max()

    R = np.hypot(X, Y)
    fade = np.clip((R - HFIELD_SPAWN_FLAT_R) / (HFIELD_SPAWN_FLAT_R * 0.8), 0, 1)
    h *= fade ** 1.2
    if h.max() > 0:
        h /= h.max()
    return h.astype(np.float32)


def sample_terrain_height(heights: np.ndarray, x: float, y: float) -> float:
    nrow, ncol = heights.shape
    fx = (x + HFIELD_SIZE) / (2 * HFIELD_SIZE) * (ncol - 1)
    fy = (y + HFIELD_SIZE) / (2 * HFIELD_SIZE) * (nrow - 1)
    fx = max(0.0, min(ncol - 1.0001, fx))
    fy = max(0.0, min(nrow - 1.0001, fy))
    c0 = int(fx)
    r0 = int(fy)
    dx = fx - c0
    dy = fy - r0
    h00 = heights[r0, c0]
    h01 = heights[r0, c0 + 1]
    h10 = heights[r0 + 1, c0]
    h11 = heights[r0 + 1, c0 + 1]
    h = h00 * (1 - dx) * (1 - dy) + h01 * dx * (1 - dy) + h10 * (1 - dx) * dy + h11 * dx * dy
    return float(h) * HFIELD_MAX_Z


def make_obstacles_xml(
    heights: np.ndarray,
    *,
    count: int = 8,
    seed: int = 0,
    inner_radius: float | None = None,
    outer_radius: float | None = None,
) -> str:
    if count <= 0:
        return ""
    rng = np.random.default_rng(seed * 31 + 17)
    if inner_radius is None:
        inner_radius = HFIELD_SPAWN_FLAT_R + 0.18
    if outer_radius is None:
        outer_radius = HFIELD_SIZE * 0.75

    placed: list[tuple[float, float, float]] = []
    snippets = []

    def nonoverlap(x, y, r):
        if math.hypot(x, y) - r < inner_radius:
            return False
        return all(math.hypot(px - x, py - y) > r + pr + 0.08 for px, py, pr in placed)

    for idx in range(count):
        sx = float(rng.uniform(0.035, 0.10))
        sy = float(rng.uniform(0.025, 0.08))
        sz = float(rng.uniform(0.010, 0.035))
        yaw = float(rng.uniform(0, 2 * math.pi))
        r_excl = math.hypot(sx, sy) + 0.04
        xy = None
        for _ in range(60):
            rr = rng.uniform(inner_radius, outer_radius)
            th = rng.uniform(0, 2 * math.pi)
            x, y = rr * math.cos(th), rr * math.sin(th)
            if nonoverlap(x, y, r_excl):
                xy = (x, y)
                break
        if xy is None:
            continue
        x, y = xy
        z = sample_terrain_height(heights, x, y) + sz
        qw, qz = math.cos(yaw / 2), math.sin(yaw / 2)
        snippets.append(
            f'<geom name="obs_{idx}" type="box" size="{sx:.4f} {sy:.4f} {sz:.4f}" '
            f'pos="{x:.4f} {y:.4f} {z:.4f}" quat="{qw:.5f} 0 0 {qz:.5f}" '
            f'rgba="0.52 0.45 0.35 1" friction="1.5 0.05 0.0001"/>'
        )
        placed.append((x, y, r_excl))
    return "\n      ".join(snippets)


def stance_foot_z_relative_to_hip() -> float:
    p = STANCE_FEMUR
    pt = STANCE_FEMUR + STANCE_TIBIA
    knee_z = -FEMUR * math.sin(p)
    foot_z = knee_z - TIBIA * math.sin(pt)
    return foot_z


def stance_chassis_height() -> float:
    return -YAW_OUTPUT_HEIGHT - stance_foot_z_relative_to_hip() + FOOT_R + 0.008


def foot_horizontal_reach() -> float:
    p = STANCE_FEMUR
    pt = STANCE_FEMUR + STANCE_TIBIA
    return COXA + FEMUR * math.cos(p) + TIBIA * math.cos(pt)


D_FOOT = foot_horizontal_reach()


def build_xml(obstacles_xml: str = "") -> str:
    base_z = stance_chassis_height()
    leg_blocks = []
    actuator_blocks = []
    sensor_blocks = []
    for i in range(6):
        a = (i + 0.5) * math.pi / 3.0
        x = LEG_RADIAL * math.cos(a)
        y = LEG_RADIAL * math.sin(a)
        qw = math.cos(a / 2.0)
        qz = math.sin(a / 2.0)
        leg_blocks.append(_leg_xml(i, x, y, YAW_OUTPUT_HEIGHT, qw, qz))
        actuator_blocks.append(_leg_actuators(i))
        sensor_blocks.append(_leg_sensors(i))

    return f"""<mujoco model="hexapod_prototype">
  <compiler angle="radian" coordinate="local" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" iterations="50" solver="Newton" cone="elliptic"/>

  <default>
    <joint armature="{ARMATURE}" damping="0.01" limited="true"/>
    <geom solref="0.006 1" solimp="0.95 0.99 0.001"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="2" density="0"/>
    </default>
    <default class="collision">
      <geom group="3" rgba="1 0 0 0.18" condim="4" friction="1.3 0.04 0.0001"/>
    </default>
    <default class="foot">
      <geom group="3" rgba="0.02 0.02 0.02 1" condim="6" friction="2.0 0.1 0.001"
            solref="0.01 1" solimp="0.95 0.99 0.001"/>
    </default>
  </default>

  <asset>
    <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.7 0.85 1.0" rgb2="0.4 0.5 0.7" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.25 0.3 0.35" rgb2="0.18 0.22 0.27" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.15"/>
    <material name="frame" rgba="0.78 0.82 0.85 1" specular="0.25" shininess="0.25"/>
    <material name="motor" rgba="0.05 0.05 0.06 1" specular="0.2" shininess="0.2"/>
    <material name="battery" rgba="0.16 0.16 0.24 1"/>
    <material name="rubber" rgba="0.02 0.02 0.02 1"/>
    <texture name="terrain_tex" type="2d" builtin="checker" rgb1="0.34 0.45 0.30" rgb2="0.26 0.36 0.22" width="320" height="320"/>
    <material name="terrain_mat" texture="terrain_tex" texrepeat="12 12" reflectance="0.05" shininess="0.0"/>
    <hfield name="terrain" nrow="{HFIELD_NROW}" ncol="{HFIELD_NCOL}" size="{HFIELD_SIZE} {HFIELD_SIZE} {HFIELD_MAX_Z} {HFIELD_BASE}"/>
  </asset>

  <worldbody>
    <light name="sun" pos="2 -1.5 2.2" dir="-0.6 0.4 -1" castshadow="true"/>
    <geom name="floor" type="plane" size="8 8 0.05" pos="0 0 -0.001" material="grid" friction="1.5 0.05 0.0001"/>
    <geom name="terrain" type="hfield" hfield="terrain" material="terrain_mat" friction="1.5 0.05 0.0001" condim="4"/>
      {obstacles_xml}

    <body name="chassis" pos="0 0 {base_z:.5f}">
      <freejoint name="root"/>
      <site name="chassis_imu" pos="0 0 0.035" size="0.008"/>
      <inertial pos="0 0 0.018" mass="{CHASSIS_MASS}" diaginertia="0.006 0.006 0.010"/>
      <geom class="visual" type="cylinder" size="{APOTHEM / math.cos(math.pi / 6):.5f} 0.004" euler="0 0 {math.pi / 6:.5f}" material="frame"/>
      <geom class="visual" type="box" size="0.055 0.019 0.014" pos="-0.025 0 0.018" material="battery"/>
      <geom class="visual" type="box" size="0.050 0.035 0.003" pos="0.035 0 0.021" material="frame"/>
      <geom class="collision" name="chassis_box" type="box" pos="0 0 0.014" size="0.115 0.105 0.020"/>

{''.join(leg_blocks)}
    </body>
  </worldbody>

  <actuator>
{''.join(actuator_blocks)}
  </actuator>

  <sensor>
{''.join(sensor_blocks)}
    <accelerometer name="chassis_acc" site="chassis_imu"/>
    <gyro name="chassis_gyro" site="chassis_imu"/>
    <framepos name="chassis_pos" objtype="body" objname="chassis"/>
    <framezaxis name="chassis_up" objtype="body" objname="chassis"/>
  </sensor>
</mujoco>
"""


def _leg_xml(i: int, x: float, y: float, z: float, qw: float, qz: float) -> str:
    return f"""      <body name="L{i}_yaw" pos="{x:.5f} {y:.5f} {z:.5f}" quat="{qw:.6f} 0 0 {qz:.6f}">
        <inertial pos="{COXA / 2:.5f} 0 0" mass="{COXA_MASS}" diaginertia="0.00005 0.00006 0.00006"/>
        <joint name="L{i}_yaw" type="hinge" axis="0 0 1" range="-0.90 0.90"/>
        <geom class="visual" type="box" pos="{COXA / 2:.5f} 0 0" size="{COXA / 2:.5f} 0.010 0.004" material="frame"/>
        <geom class="visual" type="box" pos="0 -0.026 -0.017" size="0.020 0.010 0.019" material="motor"/>

        <body name="L{i}_femur" pos="{COXA:.5f} 0 0">
          <inertial pos="{FEMUR / 2:.5f} 0 0" mass="{FEMUR_MASS}" diaginertia="0.00008 0.00022 0.00022"/>
          <joint name="L{i}_pitch" type="hinge" axis="0 1 0" range="-1.40 0.85"/>
          <geom class="visual" type="capsule" fromto="0 0 0 {FEMUR:.5f} 0 0" size="0.009" material="frame"/>
          <geom class="visual" type="box" pos="{FEMUR - 0.010:.5f} -0.026 0" size="0.020 0.010 0.019" material="motor"/>

          <body name="L{i}_tibia" pos="{FEMUR:.5f} 0 0">
            <inertial pos="{TIBIA / 2:.5f} 0 0" mass="{TIBIA_MASS}" diaginertia="0.00006 0.00022 0.00022"/>
            <joint name="L{i}_knee" type="hinge" axis="0 1 0" range="-0.35 1.85"/>
            <geom class="visual" type="capsule" fromto="0 0 0 {TIBIA - FOOT_R:.5f} 0 0" size="0.007" material="frame"/>
            <geom class="visual" type="cylinder" pos="{TIBIA:.5f} 0 -0.006" size="{FOOT_R:.5f} 0.006" material="rubber"/>
            <geom class="foot" name="L{i}_foot" type="sphere" pos="{TIBIA:.5f} 0 0" size="{FOOT_R:.5f}"/>
            <site name="L{i}_foot_site" pos="{TIBIA:.5f} 0 0" size="0.004"/>
          </body>
        </body>
      </body>
"""


def _leg_actuators(i: int) -> str:
    return f"""    <position name="L{i}_yaw" joint="L{i}_yaw" kp="{KP_YAW}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <position name="L{i}_pitch" joint="L{i}_pitch" kp="{KP_PITCH}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <position name="L{i}_knee" joint="L{i}_knee" kp="{KP_KNEE}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_yaw_d" joint="L{i}_yaw" kv="{DAMP_YAW}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_pitch_d" joint="L{i}_pitch" kv="{DAMP_PITCH}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
    <velocity name="L{i}_knee_d" joint="L{i}_knee" kv="{DAMP_KNEE}" forcerange="-{TORQUE_LIMIT} {TORQUE_LIMIT}"/>
"""


def _leg_sensors(i: int) -> str:
    return f"""    <jointpos name="L{i}_yaw_p" joint="L{i}_yaw"/>
    <jointpos name="L{i}_pitch_p" joint="L{i}_pitch"/>
    <jointpos name="L{i}_knee_p" joint="L{i}_knee"/>
    <touch name="L{i}_foot_t" site="L{i}_foot_site"/>
"""


def build_world(
    *,
    terrain_seed: int = 0,
    terrain_enabled: bool = True,
    obstacle_count: int = 8,
    obstacle_seed: int = 0,
    ensure_rider: bool = False,
):
    del ensure_rider
    if terrain_enabled:
        heights = make_terrain_heightmap(seed=terrain_seed)
    else:
        heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)
    obstacles_xml = make_obstacles_xml(heights, count=obstacle_count, seed=obstacle_seed)
    model = mujoco.MjModel.from_xml_string(build_xml(obstacles_xml=obstacles_xml))
    data = mujoco.MjData(model)
    _populate_hfield(model, heights)
    return model, data, heights


def _leg_ik(target_xyz_in_yaw_frame):
    u = float(target_xyz_in_yaw_frame[0]) - COXA
    w = -float(target_xyz_in_yaw_frame[2])
    L = math.hypot(u, w)
    if L > FEMUR + TIBIA - 1e-6 or L < abs(FEMUR - TIBIA) + 1e-6:
        return None
    cos_pt = (L * L - FEMUR * FEMUR - TIBIA * TIBIA) / (2 * FEMUR * TIBIA)
    cos_pt = max(-1.0, min(1.0, cos_pt))
    pt = math.acos(cos_pt)
    p = math.atan2(w, u) - math.atan2(TIBIA * math.sin(pt), FEMUR + TIBIA * math.cos(pt))
    return p, pt


class TripodGait:
    MAX_VX = 0.28
    MAX_VY = 0.20
    MAX_OMEGA = 1.0
    SCALE_PERIOD_MIN = 0.40
    SCALE_PERIOD_MAX = 2.00
    SCALE_LIFT_MIN = 0.30
    SCALE_LIFT_MAX = 2.50
    SCALE_STRIDE_MIN = 0.30
    SCALE_STRIDE_MAX = 1.80

    def __init__(
        self,
        *,
        period: float = 0.65,
        lift: float = 0.025,
        ramp: float = 0.35,
        vx: float = 0.0,
        vy: float = 0.0,
        omega: float = 0.0,
        period_scale: float = 1.0,
        lift_scale=1.0,
        stride_scale: float = 1.0,
    ):
        self.period = period
        self.lift = lift
        self.ramp = max(ramp, 1e-3)
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.period_scale = period_scale
        self.lift_scale = self._broadcast_lift(lift_scale)
        self.stride_scale = stride_scale
        self.leg_angles = [(i + 0.5) * math.pi / 3.0 for i in range(6)]
        p, pt = STANCE_FEMUR, STANCE_FEMUR + STANCE_TIBIA
        self.foot_neutral_x = COXA + FEMUR * math.cos(p) + TIBIA * math.cos(pt)
        self.foot_neutral_z = -FEMUR * math.sin(p) - TIBIA * math.sin(pt)
        self._foot_radius = LEG_RADIAL + self.foot_neutral_x
        self._phase_offset = math.pi / 2.0
        self._fallback = (0.0, STANCE_FEMUR, STANCE_TIBIA)
        self._vx_smooth = vx
        self._vy_smooth = vy
        self._om_smooth = omega
        self._last_t = None
        self._phase = 0.0
        self._elapsed = 0.0

    def set_velocity(self, *, vx=None, vy=None, omega=None):
        if vx is not None:
            self.vx = max(-self.MAX_VX, min(self.MAX_VX, float(vx)))
        if vy is not None:
            self.vy = max(-self.MAX_VY, min(self.MAX_VY, float(vy)))
        if omega is not None:
            self.omega = max(-self.MAX_OMEGA, min(self.MAX_OMEGA, float(omega)))

    def _broadcast_lift(self, lift_scale) -> np.ndarray:
        arr = np.asarray(lift_scale, dtype=np.float64)
        if arr.ndim == 0:
            arr = np.full(6, float(arr))
        elif arr.shape != (6,):
            raise ValueError(f"lift_scale must be scalar or length-6, got {arr.shape}")
        return np.clip(arr, self.SCALE_LIFT_MIN, self.SCALE_LIFT_MAX)

    def set_scales(self, *, period_scale=None, lift_scale=None, stride_scale=None):
        if period_scale is not None:
            self.period_scale = float(np.clip(period_scale, self.SCALE_PERIOD_MIN, self.SCALE_PERIOD_MAX))
        if lift_scale is not None:
            self.lift_scale = self._broadcast_lift(lift_scale)
        if stride_scale is not None:
            self.stride_scale = float(np.clip(stride_scale, self.SCALE_STRIDE_MIN, self.SCALE_STRIDE_MAX))

    def swing_mask(self) -> np.ndarray:
        out = np.zeros(6, dtype=np.float64)
        for i in range(6):
            tripod = 0 if i % 2 == 0 else 1
            phi = (self._phase + self._phase_offset + tripod * math.pi) % (2 * math.pi)
            out[i] = 1.0 if phi < math.pi else 0.0
        return out

    def reset_phase(self, *, phase: float = 0.0, t: float = 0.0):
        self._phase = phase % (2 * math.pi)
        self._elapsed = 0.0
        self._last_t = t
        self._vx_smooth = self.vx
        self._vy_smooth = self.vy
        self._om_smooth = self.omega

    def stop(self):
        self.set_velocity(vx=0.0, vy=0.0, omega=0.0)

    def _advance(self, t: float):
        if self._last_t is None:
            self._last_t = t
            return 0.0
        dt = max(0.0, t - self._last_t)
        self._last_t = t
        self._elapsed += dt
        self._phase = (self._phase + 2 * math.pi * dt / max(self.period * self.period_scale, 0.05)) % (2 * math.pi)
        return dt

    def _smoothed_command(self, dt: float):
        if dt <= 0.0:
            return self._vx_smooth, self._vy_smooth, self._om_smooth
        tau = 0.15
        a = 1.0 - math.exp(-dt / tau)
        self._vx_smooth += a * (self.vx - self._vx_smooth)
        self._vy_smooth += a * (self.vy - self._vy_smooth)
        self._om_smooth += a * (self.omega - self._om_smooth)
        return self._vx_smooth, self._vy_smooth, self._om_smooth

    def _foot_target_in_body(self, i: int, vx, vy, omega):
        t_eff = max(self.period * self.period_scale, 0.05)
        ramp_amp = min(self._elapsed / self.ramp, 1.0)
        tripod = 0 if i % 2 == 0 else 1
        phi = (self._phase + self._phase_offset + tripod * math.pi) % (2 * math.pi)
        if phi < math.pi:
            s = phi / math.pi
            prog = -0.5 + s
            dz = self.lift * float(self.lift_scale[i]) * ramp_amp * math.sin(math.pi * s)
        else:
            s = (phi - math.pi) / math.pi
            prog = 0.5 - s
            dz = 0.0
        a_i = self.leg_angles[i]
        sa, ca = math.sin(a_i), math.cos(a_i)
        v_x_at = vx - omega * self._foot_radius * sa
        v_y_at = vy + omega * self._foot_radius * ca
        dx = prog * v_x_at * t_eff / 2.0 * ramp_amp * self.stride_scale
        dy = prog * v_y_at * t_eff / 2.0 * ramp_amp * self.stride_scale
        return dx, dy, dz

    def desired(self, t: float):
        dt = self._advance(t)
        vx, vy, omega = self._smoothed_command(dt)
        yaws = np.zeros(6)
        pitches = np.full(6, STANCE_FEMUR)
        knees = np.full(6, STANCE_TIBIA)
        for i, a in enumerate(self.leg_angles):
            dx_b, dy_b, dz_b = self._foot_target_in_body(i, vx, vy, omega)
            fx_b_neutral = self._foot_radius * math.cos(a)
            fy_b_neutral = self._foot_radius * math.sin(a)
            fx_b = fx_b_neutral + dx_b
            fy_b = fy_b_neutral + dy_b
            yaw_origin_x = LEG_RADIAL * math.cos(a)
            yaw_origin_y = LEG_RADIAL * math.sin(a)
            rx = fx_b - yaw_origin_x
            ry = fy_b - yaw_origin_y
            ca, sa = math.cos(a), math.sin(a)
            x_yaw = ca * rx + sa * ry
            y_yaw = -sa * rx + ca * ry
            yaw_angle = math.atan2(y_yaw, x_yaw)
            r_planar = math.hypot(x_yaw, y_yaw)
            ik = _leg_ik((r_planar, 0.0, self.foot_neutral_z + dz_b))
            if ik is None:
                yaws[i], pitches[i], knees[i] = self._fallback
            else:
                p, k = ik
                yaws[i], pitches[i], knees[i] = yaw_angle, p, k
        return yaws, pitches, knees


def _set_targets(model: mujoco.MjModel, data: mujoco.MjData, yaws, pitches, knees):
    for i in range(6):
        base = i * 6
        data.ctrl[base + 0] = yaws[i]
        data.ctrl[base + 1] = pitches[i]
        data.ctrl[base + 2] = knees[i]


def _initial_pose(data: mujoco.MjData):
    for i in range(6):
        base = i * 6
        data.ctrl[base + 0] = 0.0
        data.ctrl[base + 1] = STANCE_FEMUR
        data.ctrl[base + 2] = STANCE_TIBIA


def _populate_hfield(model: mujoco.MjModel, heights: np.ndarray):
    hf_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
    nrow = int(model.hfield_nrow[hf_id])
    ncol = int(model.hfield_ncol[hf_id])
    start = int(model.hfield_adr[hf_id])
    model.hfield_data[start:start + nrow * ncol] = np.asarray(heights, dtype=np.float32).reshape(nrow, ncol).flatten()


def _set_stance_qpos(model: mujoco.MjModel, data: mujoco.MjData):
    data.qpos[0:3] = [0.0, 0.0, stance_chassis_height()]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(6):
        for kind, val in (("yaw", 0.0), ("pitch", STANCE_FEMUR), ("knee", STANCE_TIBIA)):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{kind}")
            data.qpos[model.jnt_qposadr[j]] = val
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def run_headless(model, data, gait: TripodGait, *, duration: float, settle: float = 0.8, run_gait: bool = True):
    _set_stance_qpos(model, data)
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    for _ in range(int(settle / model.opt.timestep)):
        _initial_pose(data)
        mujoco.mj_step(model, data)
    p0 = np.array(data.body(chassis).xpos)
    for k in range(int(duration / model.opt.timestep)):
        if run_gait:
            y, p, kn = gait.desired(k * model.opt.timestep)
            _set_targets(model, data, y, p, kn)
        else:
            _initial_pose(data)
        mujoco.mj_step(model, data)
    p1 = np.array(data.body(chassis).xpos)
    d = p1 - p0
    print(f"  chassis after settle: pos = {p0} m")
    print(f"  chassis after walk:   pos = {p1} m")
    print(f"  Δposition = ({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}) m")
    print(f"  horizontal travel = {math.hypot(d[0], d[1]):.3f} m in {duration:.2f}s")


def run_viewer(model, data, gait: TripodGait, *, run_gait: bool = True, settle: float = 0.8):
    import mujoco.viewer as viewer

    def reset():
        _set_stance_qpos(model, data)

    reset()
    with viewer.launch_passive(model, data) as v:
        t0 = data.time
        while v.is_running():
            t = data.time - t0
            if data.time < settle or not run_gait:
                _initial_pose(data)
            else:
                y, p, kn = gait.desired(t - settle)
                _set_targets(model, data, y, p, kn)
            mujoco.mj_step(model, data)
            v.sync()
            time.sleep(max(0.0, model.opt.timestep - 1e-4))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--period", type=float, default=0.65)
    ap.add_argument("--lift", type=float, default=0.025)
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--omega", type=float, default=0.0)
    ap.add_argument("--no-gait", action="store_true")
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--terrain-seed", type=int, default=0)
    ap.add_argument("--obstacles", type=int, default=6)
    ap.add_argument("--obstacle-seed", type=int, default=0)
    ap.add_argument("--dump-xml", default=None)
    args = ap.parse_args()

    heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32) if args.no_terrain else make_terrain_heightmap(args.terrain_seed)
    obstacles_xml = make_obstacles_xml(heights, count=args.obstacles, seed=args.obstacle_seed)
    xml = build_xml(obstacles_xml=obstacles_xml)
    if args.dump_xml:
        with open(args.dump_xml, "w") as f:
            f.write(xml)
        print(f"Wrote {args.dump_xml}")
        return
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    _populate_hfield(model, heights)
    print(f"Prototype stance height = {stance_chassis_height():.3f} m")
    print(f"Loaded MuJoCo model: nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody}")
    gait = TripodGait(period=args.period, lift=args.lift, vx=args.vx, vy=args.vy, omega=args.omega)
    if args.headless:
        run_headless(model, data, gait, duration=args.duration, run_gait=not args.no_gait)
    else:
        run_viewer(model, data, gait, run_gait=not args.no_gait)


if __name__ == "__main__":
    main()
