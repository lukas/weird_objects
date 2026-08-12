"""STS3215 servo model for the MuJoCo twin.

Two halves:

- ``SimServoParams`` — per-axis actuator parameters. Ships with defaults;
  ``fit_motor_model.py`` overwrites them from hardware measurements and
  saves ``sim_model.json`` next to this file.
- ``ServoProfile`` — reproduces how the real bus drives the joints: each
  commanded target is delayed by the measured latency, then the servo's
  internal profile slews the position setpoint toward it at the commanded
  speed, ignoring errors smaller than the deadband. MuJoCo's position
  actuator (kp) + velocity actuator (kv) then track that profile target.

The MuJoCo model itself comes from ``mujoco_prototype.build_xml()``;
``apply_params_to_model`` pushes kp / kv / frictionloss into a loaded
model in place (no XML rebuild), which is what the fitting loop and the
domain-randomization reset both need.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

_PROTO = Path(__file__).resolve().parents[2]
if str(_PROTO) not in sys.path:
    sys.path.insert(0, str(_PROTO))

N_JOINTS = 18
AXES = ("yaw", "hip", "knee")
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# Feetech position counts per degree (4096 counts / 360°); the bus commands
# profile speed in counts/s.
COUNTS_PER_DEG = 4096.0 / 360.0

SIM_MODEL_PATH = Path(__file__).resolve().parent / "sim_model.json"
# Loaded-actuator fit (fit_loaded_actuator.py, 08-10 bench session):
# selected with --cfg-set bus.servo_params=loaded; the default stays the
# air fit so every existing lineage is untouched.
LOADED_MODEL_PATH = Path(__file__).resolve().parent / "sim_model_loaded.json"


@dataclass
class AxisParams:
    """Fitted per-axis actuator parameters."""
    kp: float                     # MuJoCo position-actuator gain
    kv: float                     # MuJoCo velocity-actuator damping gain
    frictionloss: float           # joint dof frictionloss (N·m)
    latency_ms: float             # command → first motion (measured delay)
    vel_max_deg_s: float          # servo velocity ceiling at test speed
    deadband_deg: float           # profile ignores errors below this
    torque_limit_nm: float = 2.2


@dataclass
class SimServoParams:
    axes: dict[str, AxisParams] = field(default_factory=dict)
    # Per-axis relative spread across joints (for DR ranges), e.g.
    # {"yaw": {"rise_ms_pct": 0.15, "delay_ms_pct": 0.2}, ...}
    spread: dict[str, dict] = field(default_factory=dict)
    source: str = "defaults"
    timestamp: str = ""
    speed_counts_s: float = 350.0

    # -- vectorized (18,) views used by the env ---------------------------
    def per_joint(self, attr: str) -> np.ndarray:
        out = np.zeros(N_JOINTS, dtype=float)
        for j in range(N_JOINTS):
            out[j] = getattr(self.axes[AXES[j % 3]], attr)
        return out

    @classmethod
    def defaults(cls) -> "SimServoParams":
        """Pre-hardware guesses (mujoco_prototype constants + STS datasheet)."""
        import mujoco_prototype as MP
        mk = lambda kp, kv: AxisParams(  # noqa: E731
            kp=kp, kv=kv, frictionloss=0.02,
            latency_ms=60.0,
            vel_max_deg_s=350.0 / COUNTS_PER_DEG,   # profile speed 350 counts/s
            deadband_deg=0.35,
            torque_limit_nm=MP.TORQUE_LIMIT,
        )
        return cls(axes={
            "yaw": mk(MP.KP_YAW, MP.DAMP_YAW),
            "hip": mk(MP.KP_PITCH, MP.DAMP_PITCH),
            "knee": mk(MP.KP_KNEE, MP.DAMP_KNEE),
        }, source="defaults", timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"))

    def save(self, path: Path | str = SIM_MODEL_PATH) -> Path:
        path = Path(path)
        blob = {
            "source": self.source,
            "timestamp": self.timestamp,
            "speed_counts_s": self.speed_counts_s,
            "axes": {ax: asdict(p) for ax, p in self.axes.items()},
            "spread": self.spread,
        }
        path.write_text(json.dumps(blob, indent=2))
        return path

    @classmethod
    def load(cls, path: Path | str = SIM_MODEL_PATH) -> "SimServoParams":
        """Load fitted params; fall back to defaults if never fitted."""
        path = Path(path)
        if not path.is_file():
            return cls.defaults()
        blob = json.loads(path.read_text())
        return cls(
            axes={ax: AxisParams(**p) for ax, p in blob["axes"].items()},
            spread=blob.get("spread", {}),
            source=blob.get("source", str(path)),
            timestamp=blob.get("timestamp", ""),
            speed_counts_s=float(blob.get("speed_counts_s", 350.0)),
        )

    @classmethod
    def from_cfg(cls, cfg: dict | None) -> "SimServoParams":
        """Resolve the params set from cfg key ``bus.servo_params``.

        "" / absent (default) -> the air fit (``sim_model.json``) —
        legacy byte-exact; "loaded" -> ``sim_model_loaded.json`` (the
        08-10 loaded bench fit); any other value -> explicit json path.
        A missing file for an EXPLICIT selection raises instead of
        silently falling back (a dropped reward/cfg package voided a
        verdict once — gotcha 3; same failure class).
        """
        sel = ""
        if cfg is not None:
            from rl_move.config import cfg_get
            sel = str(cfg_get(cfg, "bus", "servo_params", default="") or "")
        if not sel:
            return cls.load()
        path = LOADED_MODEL_PATH if sel == "loaded" else Path(sel)
        if not path.is_file():
            raise FileNotFoundError(
                f"bus.servo_params={sel!r} -> {path} does not exist; "
                "run fit_loaded_actuator.py or fix the path")
        return cls.load(path)


# ---------------------------------------------------------------------------
# MuJoCo model plumbing
# ---------------------------------------------------------------------------

def build_model(*, fixed_base: bool = False, flat_terrain: bool = True,
                mesh_visuals: bool = True, mjx_compat: bool = False,
                terrain_amp: float = 1.0, terrain_seed: int = 0,
                leg_chassis_collision: bool = False):
    """Load the hexapod MJCF. ``fixed_base`` welds the chassis (bench/air
    tests); ``flat_terrain`` zeroes the random hfield so the floor is flat.

    With ``flat_terrain=False`` the hfield is POPULATED with
    ``make_terrain_heightmap(terrain_seed) * terrain_amp`` (amp 1.0 =
    the full 18 mm indoor bumps; the map is flat within 0.32 m of the
    spawn and fades in from there). Cfg knobs ``env.terrain_amp`` /
    ``env.terrain_seed`` reach here from every training/eval env.

    ``mesh_visuals=False`` renders the primitive geometry (capsules /
    spheres built from the same kinematic constants the physics uses)
    instead of the decorative STL meshes. The June 2026 STL re-export
    broke the mesh placement offsets in mujoco_prototype (tibia mesh is
    ~44 mm longer than the kinematic link), so primitives are the
    truthful choice for RL rollout videos. Physics is identical either
    way — visual geoms are contype=0, density=0.

    ``mjx_compat=True`` prepares the model for ``mjx.put_model``: with
    ``flat_terrain=True`` (the training default) the hfield terrain is
    swapped for an equivalent flat plane (same z=0 surface, friction and
    condim) — cheapest ground contact on GPU; with ``flat_terrain=False``
    the hfield is KEPT: MuJoCo-Warp collides with height fields
    (verified on H200, mujoco-mjx 3.11 — the ccd_hfield kernel), so
    rough-terrain experiments can run under ``impl="warp"``. The XLA
    impl still has no hfield collisions. Either way the backup floor
    plane is removed (redundant contact work on GPU).
    """
    import mujoco
    import mujoco_prototype as MP
    saved = (MP.USE_PART_MESHES, MP.USE_SERVO_MESHES)
    try:
        if not mesh_visuals:
            MP.USE_PART_MESHES = MP.USE_SERVO_MESHES = False
        xml = MP.build_xml()
    finally:
        MP.USE_PART_MESHES, MP.USE_SERVO_MESHES = saved
    if fixed_base:
        xml = xml.replace('<freejoint name="root"/>', '')
    if leg_chassis_collision:
        # cfg env.leg_chassis_collision=1 — model the belly knife-edge
        # (SIM.md known-gap 4, added 08-12). replay_trace.py's diagnosis
        # of the 10/10 hardware stand-up failures: the belly-curl support
        # set is a knife edge the sim never forms — hardware tips
        # pivoting on one foot pad while the tucked shanks / knee servos
        # bear on the chassis underside; sim sweeps the leg segments
        # straight through the chassis because legcol geoms pair ONLY
        # with the floor. MuJoCo precomputes the collidable pair set at
        # COMPILE time (runtime contype/conaffinity edits never register
        # — verified 08-12 on 3.11), so the masks are rewritten in the
        # XML here, pre-compile, and the MJX/Warp pair set inherits them
        # via put_model. Bit plan (floor=conaffinity 5, legcol contype 4,
        # everything else default 1/1):
        #   bit 8:  tucked-shank group (tibia + knee-servo box) <->
        #           chassis underside (chassis_box + 6 yaw-servo boxes)
        #   bit 16: femur root <-> yaw-servo boxes ONLY. The femur
        #           capsule permanently overlaps the coarse chassis_box
        #           by ~15 mm at the hip anchor in EVERY pose (a box
        #           modeling artifact), so femur-chassis_box stays OFF
        #           or nominal stance would carry a constant bogus
        #           contact force. Femur-to-own-bracket gap is ~4.6 mm
        #           at nominal — inert in plant stance, live in the curl.
        # Leg-leg stays OFF (legcol conaffinity stays 0); floor / foot
        # pairings are bit-identical. DEFAULT OFF — bit-exact when off.
        rewritten = xml
        rewrites = [('<geom class="collision" name="chassis_box" ',
                     '<geom class="collision" conaffinity="9" '
                     'name="chassis_box" ')]
        for i in range(6):
            rewrites += [
                (f'<geom class="collision" name="L{i}_yaw_servo_col" ',
                 f'<geom class="collision" conaffinity="25" '
                 f'name="L{i}_yaw_servo_col" '),
                (f'<geom class="legcol" name="L{i}_tibia_col" ',
                 f'<geom class="legcol" contype="12" '
                 f'name="L{i}_tibia_col" '),
                (f'<geom class="legcol" name="L{i}_knee_servo_col" ',
                 f'<geom class="legcol" contype="12" '
                 f'name="L{i}_knee_servo_col" '),
                (f'<geom class="legcol" name="L{i}_femur_col" ',
                 f'<geom class="legcol" contype="20" '
                 f'name="L{i}_femur_col" '),
            ]
        for old, new in rewrites:
            if rewritten.count(old) != 1:
                raise RuntimeError(
                    f"leg_chassis_collision rewrite failed on {old!r} — "
                    "mujoco_prototype geom XML changed?")
            rewritten = rewritten.replace(old, new)
        xml = rewritten
    if mjx_compat:
        import re
        if flat_terrain:
            xml = re.sub(r'<hfield name="terrain"[^/]*/>', "", xml)
            xml, n = re.subn(
                r'<geom name="terrain" type="hfield" hfield="terrain" '
                r'([^/]*)/>',
                r'<geom name="terrain" type="plane" size="8 8 0.05" \1/>',
                xml)
            if n != 1 or "hfield" in xml:
                raise RuntimeError("mjx_compat terrain rewrite failed — "
                                   "mujoco_prototype terrain XML changed?")
        # Drop the backup grid plane 1 mm under the terrain: it can never
        # win a contact in C MuJoCo, but MJX evaluates every plane×geom
        # pair every substep, so the duplicate doubles ground-contact
        # work on the GPU.
        xml, n = re.subn(r'<geom name="floor" type="plane"[^/]*/>', "", xml)
        if n != 1:
            raise RuntimeError("mjx_compat floor removal failed — "
                               "mujoco_prototype floor XML changed?")
    model = mujoco.MjModel.from_xml_string(xml)
    if flat_terrain and model.hfield_data.size:
        model.hfield_data[:] = 0.0
    elif not flat_terrain and model.hfield_data.size:
        heights = MP.make_terrain_heightmap(seed=int(terrain_seed))
        # hfield data must stay normalized to [0, 1]; physical bump height
        # is data * hfield_size[2] (HFIELD_MAX_Z, 36 mm at amp 1.0). Amps
        # <=1 scale the data; amps >1 scale the z-extent instead — the old
        # np.clip(amp, 0, 1) here silently capped every terrain at 36 mm
        # (found 08-11 when the champion probe returned bit-identical
        # results at amp 1.5/2.0/3.0).
        amp = max(0.0, float(terrain_amp))
        heights = heights * min(amp, 1.0)
        if amp > 1.0:
            hf_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
            model.hfield_size[hf_id][2] *= amp
        MP._populate_hfield(model, heights)
    return model


def _act_id(model, name: str) -> int:
    import mujoco
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise KeyError(f"actuator {name!r} not in model")
    return aid


def joint_names() -> list[str]:
    return [f"L{j // 3}_{('yaw', 'pitch', 'knee')[j % 3]}"
            for j in range(N_JOINTS)]


def apply_params_to_model(model, params: SimServoParams,
                          *, kp_scale: np.ndarray | None = None,
                          kv_scale: np.ndarray | None = None,
                          torque_scale: float = 1.0) -> None:
    """Push kp/kv/frictionloss/torque limits into a loaded model in place.

    ``kp_scale`` / ``kv_scale`` are optional per-joint (18,) DR multipliers;
    ``torque_scale`` models battery sag / servo unit spread.
    """
    import mujoco
    for j, jname in enumerate(joint_names()):
        ax = params.axes[AXES[j % 3]]
        kp = ax.kp * (1.0 if kp_scale is None else float(kp_scale[j]))
        kv = ax.kv * (1.0 if kv_scale is None else float(kv_scale[j]))
        tl = ax.torque_limit_nm * torque_scale

        pa = _act_id(model, jname)
        model.actuator_gainprm[pa, 0] = kp
        model.actuator_biasprm[pa, 1] = -kp
        model.actuator_forcerange[pa] = (-tl, tl)

        # kv lives in DOF damping, NOT the velocity actuator: actuator
        # damping is integrated EXPLICITLY and with the torque clamp it
        # goes bang-bang at the 2 ms timestep (Δv per step = tl·h/I ≈
        # 3 rad/s for the tibia) — belly-rest poses vibrated at ±4 rad/s
        # forever and read as phantom 2.2 A knee current. DOF damping is
        # implicit and unconditionally stable. The old velocity actuator
        # is zeroed but kept in the XML for compatibility.
        va = _act_id(model, jname + "_d")
        model.actuator_gainprm[va, 0] = 0.0
        model.actuator_biasprm[va, 2] = 0.0
        model.actuator_forcerange[va] = (-tl, tl)

        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        dadr = model.jnt_dofadr[jid]
        model.dof_damping[dadr] = kv
        model.dof_frictionloss[dadr] = ax.frictionloss


def joint_qpos_addrs(model) -> np.ndarray:
    import mujoco
    return np.array([
        model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
        for n in joint_names()], dtype=int)


def joint_qvel_addrs(model) -> np.ndarray:
    import mujoco
    return np.array([
        model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
        for n in joint_names()], dtype=int)


def position_actuator_ids(model) -> np.ndarray:
    return np.array([_act_id(model, n) for n in joint_names()], dtype=int)


# ---------------------------------------------------------------------------
# Profiled target — the servo's internal motion profile + bus latency
# ---------------------------------------------------------------------------

# Feetech acc register unit: 100 counts/s² = 8.789 °/s².
ACC_UNIT_DEG_S2 = 100.0 * 360.0 / 4096.0


class ServoProfile:
    """Latency + trapezoidal profile + deadband for all 18 joints.

    The real loop SyncWrites goal positions with a profile speed AND
    acceleration; the servo slews its internal setpoint toward the goal
    along a trapezoid (accelerate at ``acc`` up to ``speed``, cruise,
    decelerate to stop at the goal). MuJoCo's position actuator should
    track this *profile target*, not the raw goal — the 2026-08-07
    hardware battery showed the acceleration ramp alone stretches a 12°
    step's rise time by ~60 ms (ACC=15 ≈ 132 °/s² takes ~0.23 s to reach
    the 31 °/s cruise speed).
    """

    def __init__(self, params: SimServoParams, q0_rad: np.ndarray, *,
                 latency_scale: float = 1.0,
                 deadband_scale: float = 1.0,
                 vel_scale: float = 1.0):
        self._latency_s = params.per_joint("latency_ms") / 1000.0 * latency_scale
        self._deadband = params.per_joint("deadband_deg") * DEG2RAD * deadband_scale
        # Exposed so the env can apply the same dead-zone at the physics
        # level (real firmware outputs no torque inside the deadband).
        self.deadband_rad = self._deadband
        self._vel_default = params.per_joint("vel_max_deg_s") * DEG2RAD * vel_scale
        self._acc_default = np.full(N_JOINTS, 15.0 * ACC_UNIT_DEG_S2 * DEG2RAD)
        self.reset(q0_rad)

    def reset(self, q0_rad: np.ndarray) -> None:
        q0 = np.asarray(q0_rad, dtype=float).reshape(N_JOINTS)
        self.goal = q0.copy()
        self.target = q0.copy()
        self._vel_now = self._vel_default.copy()
        self._acc_now = self._acc_default.copy()
        self._v = np.zeros(N_JOINTS, dtype=float)  # profile velocity
        self._queue: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []
        self._t = 0.0

    def command(self, q_rad: np.ndarray, *,
                speed_deg_s: float | np.ndarray | None = None,
                acc_units: float | np.ndarray | None = None) -> None:
        """Queue a goal write (arrives after per-joint latency).

        ``acc_units`` is in Feetech register units (×100 counts/s²), the
        same number the real bus writes (e.g. write_acc / ACC).
        """
        q = np.asarray(q_rad, dtype=float).reshape(N_JOINTS).copy()
        if speed_deg_s is None:
            vel = self._vel_default.copy()
        else:
            vel = np.minimum(
                np.broadcast_to(
                    np.asarray(speed_deg_s, dtype=float) * DEG2RAD,
                    (N_JOINTS,)).astype(float),
                self._vel_default)
        if acc_units is None:
            acc = self._acc_default.copy()
        else:
            acc = np.broadcast_to(
                np.asarray(acc_units, dtype=float) * ACC_UNIT_DEG_S2
                * DEG2RAD, (N_JOINTS,)).astype(float)
        self._queue.append((self._t, q, vel, acc))

    def tick(self, dt: float) -> np.ndarray:
        """Advance one physics step; returns the (18,) profile target."""
        self._t += dt
        # Apply, oldest → newest, every write whose per-joint latency has
        # matured; the newest matured write wins per joint. Entries are
        # dropped only once matured for ALL joints (idempotent re-apply).
        keep_from = 0
        for i, (t_w, q, vel, acc) in enumerate(self._queue):
            matured = (self._t - t_w) >= self._latency_s
            if not np.any(matured):
                break
            self.goal = np.where(matured, q, self.goal)
            self._vel_now = np.where(matured, vel, self._vel_now)
            self._acc_now = np.where(matured, acc, self._acc_now)
            if np.all(matured):
                keep_from = i + 1
        if keep_from:
            self._queue = self._queue[keep_from:]

        err = self.goal - self.target
        active = np.abs(err) > self._deadband
        acc = np.maximum(self._acc_now, 1e-6)

        # Trapezoid: decelerate when the stopping distance reaches the
        # remaining error, else accelerate toward the cruise speed (in the
        # direction of the error).
        direction = np.sign(err)
        stop_dist = self._v ** 2 / (2.0 * acc)
        toward = self._v * direction > 0  # moving toward the goal
        decel = toward & (stop_dist >= np.abs(err))
        dv = np.where(decel, -np.sign(self._v) * acc, direction * acc) * dt
        v_new = np.clip(self._v + dv, -self._vel_now, self._vel_now)

        step = v_new * dt
        # Never step past the goal; arriving (or inside deadband) stops.
        arrive = np.abs(step) >= np.abs(err)
        move = active & ~arrive
        self.target = np.where(move, self.target + step,
                               np.where(active, self.goal, self.target))
        self._v = np.where(move, v_new, 0.0)
        return self.target
