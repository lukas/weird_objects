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
import os
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
            params = cls.load()
        else:
            path = LOADED_MODEL_PATH if sel == "loaded" else Path(sel)
            if not path.is_file():
                raise FileNotFoundError(
                    f"bus.servo_params={sel!r} -> {path} does not exist; "
                    "run fit_loaded_actuator.py or fix the path")
            params = cls.load(path)

        # Opt-in profile-speed ceiling override (operator order 08-19,
        # fast-walker headroom): every fitted set carries the ~350
        # counts/s sys-ID test speed as vel_max_deg_s, so raising
        # bus.write_speed alone does nothing — ServoProfile.command and
        # the MJX TickParams clamp the commanded speed right back to the
        # old fit. ``bus.servo_vel_max_counts_s`` lifts that ceiling on
        # all 18 joints:
        #   absent / "" (default)  -> OFF, bit-exact legacy behavior;
        #   "write_speed"          -> mirror bus.write_speed, so the
        #                             profile ceiling always matches the
        #                             STS write profile;
        #   a number               -> that many counts/s.
        # Anything unparsable or <= 0 raises — fail-closed, same class
        # as the servo_params handling above (a silently dropped cfg
        # override voided a verdict once; never fall back quietly).
        raw = None
        if cfg is not None:
            raw = cfg_get(cfg, "bus", "servo_vel_max_counts_s",
                          default=None)
        if raw is not None and str(raw).strip() != "":
            if str(raw).strip() == "write_speed":
                counts = float(cfg_get(cfg, "bus", "write_speed",
                                       default=400))
            else:
                try:
                    counts = float(raw)
                except (TypeError, ValueError):
                    raise ValueError(
                        f"bus.servo_vel_max_counts_s={raw!r} — expected "
                        "a positive counts/s number or 'write_speed'")
            if not counts > 0:
                raise ValueError(
                    f"bus.servo_vel_max_counts_s={raw!r} resolved to "
                    f"{counts} counts/s — must be > 0")
            deg_s = counts / COUNTS_PER_DEG
            for ax in params.axes.values():
                ax.vel_max_deg_s = deg_s
            params.source += f"+vel_max={counts:g}cps"
        return params


def motor_contract(cfg: dict | None = None,
                   params: "SimServoParams | None" = None,
                   backend: str = "servo_profile_np") -> dict:
    """The resolved actuator/command contract this process runs under.

    Operator follow-up fb_20260820T000059 (after the steer5-fastprof1
    profile-headroom canary): every train/eval path must RECORD which
    servo profile it actually used — bus.write_speed / write_acc, the
    opt-in vel-ceiling override, the RESOLVED per-joint velocity
    ceiling, the safety slew clamp, the control rate, and which profile
    backend enforces it — so "what motor contract did this checkpoint
    train under?" is answerable from the run page / report.json alone,
    never reverse-engineered from launch args.

    ``cfg=None`` reports the stock config.yaml contract. ``params``
    defaults to ``SimServoParams.from_cfg(cfg)`` — the single
    resolution point both backends feed from (ServoProfile on CPU,
    TickParams.vel_max on MJX), so the reported ceiling is the enforced
    one by construction.
    """
    from rl_move.config import cfg_get, load_config

    def _merge(base: dict, override: dict | None) -> dict:
        if not override:
            return dict(base)
        out = dict(base)
        for key, val in override.items():
            if isinstance(val, dict) and isinstance(out.get(key), dict):
                out[key] = _merge(out[key], val)
            else:
                out[key] = val
        return out

    resolved = _merge(load_config(), cfg)
    if params is None:
        params = SimServoParams.from_cfg(resolved)
    vel_deg = params.per_joint("vel_max_deg_s")
    hz = float(cfg_get(resolved, "control", "hz", default=25.0))
    max_dq = float(cfg_get(resolved, "safety", "max_delta_q_deg",
                           default=1.5))
    return {
        "bus.write_speed": float(cfg_get(resolved, "bus", "write_speed",
                                         default=400.0)),
        "bus.write_acc": float(cfg_get(resolved, "bus", "write_acc",
                                       default=20.0)),
        "bus.servo_vel_max_counts_s": str(
            cfg_get(resolved, "bus", "servo_vel_max_counts_s",
                    default="") or ""),
        "bus.servo_params": str(
            cfg_get(resolved, "bus", "servo_params", default="") or ""),
        "resolved_vel_max_deg_s_min": float(vel_deg.min()),
        "resolved_vel_max_deg_s_max": float(vel_deg.max()),
        "resolved_vel_max_counts_s_max": float(
            vel_deg.max() * COUNTS_PER_DEG),
        "safety.max_delta_q_deg": max_dq,
        "control.hz": hz,
        "slew_limit_deg_s": max_dq * hz,
        # Profile ramp-in (08-20, fast anti-skate option (b)): 0 = no
        # ramp; > 0 = train_ppo_mjx anneals the live write profile from
        # bus.profile_ramp_start_* to the target keys above over this
        # many global env steps (the values above are the TARGET dose).
        "bus.profile_ramp_steps": float(cfg_get(
            resolved, "bus", "profile_ramp_steps", default=0) or 0),
        "servo_params_source": params.source,
        "backend_profile": backend,
    }


def motor_contract_line(contract: dict) -> str:
    """One greppable log line for the pod stdout ([motor-contract] ...)."""
    c = contract
    return ("[motor-contract] write_speed=%g acc=%g vel_override=%r "
            "resolved_vel_max=%.1f deg/s (%.0f counts/s) "
            "slew=%g deg/tick @ %g Hz (= %g deg/s) src=%s backend=%s"
            % (c["bus.write_speed"], c["bus.write_acc"],
               c["bus.servo_vel_max_counts_s"],
               c["resolved_vel_max_deg_s_max"],
               c["resolved_vel_max_counts_s_max"],
               c["safety.max_delta_q_deg"], c["control.hz"],
               c["slew_limit_deg_s"], c["servo_params_source"],
               c["backend_profile"]))


# ---------------------------------------------------------------------------
# MuJoCo model plumbing
# ---------------------------------------------------------------------------

MESH_DIR = _PROTO / "mesh_mujoco"
MESH_XML = MESH_DIR / "hexapod_mesh.xml"           # generated, gitignored
MESH_MJX_XML = MESH_DIR / "hexapod_mesh_mjx.xml"   # self-contained, checked in
MODEL_SOURCES = ("mesh", "mesh_mjx", "primitive")


def resolve_model_source(cfg=None) -> str:
    """cfg ``env.model_source`` — which MJCF family ``build_model`` loads.

    - ``mesh`` (default since 2026-08-24): the mesh-accurate model from
      ``mesh_mujoco/build_mesh_model.py`` — real CAD kinematics (hip-pitch
      axis at the true coxa anchor +38.4 mm rise; tibia/foot line on the
      sandwich mid-plane, cancelling the legacy ~24 mm off-radial foot;
      exact 150 mm knee->boot-apex) with per-part convex-hull collision on
      CPU. Where the generated hull assets are absent (CoreWeave pods, fresh
      worktrees) it automatically uses the checked-in primitive-collision
      twin ``hexapod_mesh_mjx.xml`` — SAME kinematics, masses and inertia,
      cheap fitted primitives for contact.
    - ``mesh_mjx``: force the primitive-collision twin everywhere, making
      Mac CPU rollouts bit-consistent with pod training/eval contact-wise.
    - ``primitive``: the legacy ``mujoco_prototype`` model, bit-identical to
      pre-08-24 behavior. REQUIRED for continuity whenever you resume,
      warm-start (``respec --from``) or evaluate a checkpoint whose lineage
      predates the switch — the two kinematic families do NOT transfer.

    The ``HEXAPOD_MODEL_SOURCE`` environment variable overrides the cfg.
    The calibrated behavior test suite pins itself to ``primitive`` with it
    (``tests/conftest.py``) — those tests encode dynamics measured on the
    legacy robot; mesh-family coverage lives in ``test_model_source.py``.
    """
    src = os.environ.get("HEXAPOD_MODEL_SOURCE", "").strip()
    if not src:
        from rl_move.config import cfg_get, load_config
        if cfg is None:
            cfg = load_config()
        src = str(cfg_get(cfg, "env", "model_source", default="mesh")).strip()
    if src not in MODEL_SOURCES:
        raise ValueError(f"env.model_source={src!r} — expected one of "
                         f"{MODEL_SOURCES}")
    return src


def _mesh_xml_text(want_full: bool):
    """(xml, path, assets|None) for the mesh family. The full model needs
    the gitignored STL assets (generated on the Mac); everywhere else fall
    back to the checked-in primitive-collision twin."""
    if want_full:
        adir = MESH_DIR / "assets"
        if MESH_XML.exists() and adir.is_dir():
            assets = {p.name: p.read_bytes()
                      for p in sorted(adir.glob("*.stl"))}
            if assets:
                return MESH_XML.read_text(), MESH_XML, assets
        print("[servo_model] full STL mesh assets not available (run "
              "mesh_mujoco/build_mesh_model.py for CPU full-mesh eval); "
              "using the checked-in mesh-family MJX primitive-collision "
              "twin hexapod_mesh_mjx.xml",
              file=sys.stderr)
    if not MESH_MJX_XML.exists():
        raise FileNotFoundError(
            f"{MESH_MJX_XML} missing — regenerate with "
            "mesh_mujoco/build_mesh_model.py, or set "
            "env.model_source=primitive")
    return MESH_MJX_XML.read_text(), MESH_MJX_XML, None


def _apply_leg_chassis_rewrites(xml: str, what: str) -> str:
    """The belly knife-edge contact bit plan (see build_model docstring).
    Geom names are shared between mujoco_prototype and the mesh_mjx twin,
    so one rewrite list serves both."""
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
        if xml.count(old) != 1:
            raise RuntimeError(
                f"leg_chassis_collision rewrite failed on {old!r} — "
                f"{what} geom XML changed?")
        xml = xml.replace(old, new)
    return xml


def _populate_terrain(model, flat_terrain: bool, terrain_amp: float,
                      terrain_seed: int) -> None:
    """Zero (flat) or populate the hfield, shared by every model source."""
    import mujoco
    import mujoco_prototype as MP
    if not model.hfield_data.size:
        return
    if flat_terrain:
        model.hfield_data[:] = 0.0
        return
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


def _build_mesh_model(*, source: str, fixed_base: bool, flat_terrain: bool,
                      mjx_compat: bool, terrain_amp: float,
                      terrain_seed: int, leg_chassis_collision: bool):
    """build_model backend for the mesh family (source mesh / mesh_mjx)."""
    import mujoco
    want_full = source == "mesh" and not mjx_compat
    xml, path, assets = _mesh_xml_text(want_full)
    is_full = path == MESH_XML
    if not flat_terrain and not is_full:
        raise RuntimeError(
            "rough terrain (env.terrain_amp > 0) needs the hfield: use the "
            "full mesh model on CPU or env.model_source=primitive — the "
            "mesh_mjx twin ships a flat plane only")
    if fixed_base:
        xml = xml.replace('<freejoint name="root"/>', '')
    if leg_chassis_collision:
        if is_full:
            # the full mesh model has REAL chassis/leg hulls; enabling the
            # knife-edge axis means opening full self-collision (a superset
            # of the legacy bit plan: leg-leg included) by rewriting the
            # legpart class default from floor-only to open pairing.
            old = ('<geom type="mesh" contype="4" conaffinity="0" '
                   'group="1" condim="3" friction="1.0 0.02 0.0001"/>')
            if xml.count(old) != 1:
                raise RuntimeError(
                    "leg_chassis_collision legpart rewrite failed — "
                    "mesh_mujoco legpart class XML changed?")
            xml = xml.replace(
                old, '<geom type="mesh" contype="1" conaffinity="1" '
                     'group="1" condim="3" friction="1.0 0.02 0.0001"/>')
        else:
            xml = _apply_leg_chassis_rewrites(xml, path.name)
    model = (mujoco.MjModel.from_xml_string(xml, assets=assets)
             if assets else mujoco.MjModel.from_xml_string(xml))
    _populate_terrain(model, flat_terrain, terrain_amp, terrain_seed)
    return model


def lowest_collidable_z(model, data) -> float:
    """Lowest world-z over every collidable robot geom at the current
    ``data`` pose (mesh vertices exact; sphere/capsule/box exact). Used for
    model-driven base placement on the mesh-family models, where the legacy
    analytic ``YAW_OUTPUT_HEIGHT``-based estimate is ~60 mm off."""
    import mujoco
    G = mujoco.mjtGeom
    lows = []
    for g in range(model.ngeom):
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        t = model.geom_type[g]
        if t in (G.mjGEOM_PLANE, G.mjGEOM_HFIELD):
            continue  # ground, not robot
        pos = data.geom_xpos[g]
        R = data.geom_xmat[g].reshape(3, 3)
        size = model.geom_size[g]
        if t == G.mjGEOM_MESH:
            mid = int(model.geom_dataid[g])
            adr = int(model.mesh_vertadr[mid])
            num = int(model.mesh_vertnum[mid])
            v = model.mesh_vert[adr:adr + num]
            lows.append(float((v @ R.T + pos)[:, 2].min()))
        elif t == G.mjGEOM_SPHERE:
            lows.append(float(pos[2]) - float(size[0]))
        elif t == G.mjGEOM_CAPSULE:
            lows.append(float(pos[2])
                        - (abs(R[2, 2]) * float(size[1]) + float(size[0])))
        elif t == G.mjGEOM_BOX:
            lows.append(float(pos[2]) - float(np.abs(R[2, :]) @ size))
        else:
            lows.append(float(pos[2]) - float(np.max(size)))
    return min(lows)


def build_model(*, fixed_base: bool = False, flat_terrain: bool = True,
                mesh_visuals: bool = True, mjx_compat: bool = False,
                terrain_amp: float = 1.0, terrain_seed: int = 0,
                leg_chassis_collision: bool = False,
                source: str | None = None):
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

    ``source`` (default: cfg ``env.model_source``, see
    ``resolve_model_source``) selects the MJCF family: ``mesh`` /
    ``mesh_mjx`` load the mesh-accurate kinematics from ``mesh_mujoco/``
    (``mjx_compat=True`` always uses the primitive-collision twin there;
    ``mesh_visuals`` is a no-op — the meshes ARE the model); ``primitive``
    is the legacy path below, bit-identical to pre-08-24 behavior.
    """
    import mujoco
    if source is None:
        source = resolve_model_source(None)
    if source != "primitive":
        return _build_mesh_model(
            source=source, fixed_base=fixed_base, flat_terrain=flat_terrain,
            mjx_compat=mjx_compat, terrain_amp=terrain_amp,
            terrain_seed=terrain_seed,
            leg_chassis_collision=leg_chassis_collision)
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
        xml = _apply_leg_chassis_rewrites(xml, "mujoco_prototype")
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
    _populate_terrain(model, flat_terrain, terrain_amp, terrain_seed)
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

    def command_robot_abs(self, q_robot_abs_rad: np.ndarray, *,
                          speed_deg_s: float | np.ndarray | None = None,
                          acc_units: float | np.ndarray | None = None) -> None:
        """Queue a real-robot logical-joint command into the MuJoCo model.

        Robot scripts and the web API command the knee as an absolute tibia
        angle.  MuJoCo's knee qpos is the physical hinge angle relative to
        the femur.  This is the command-frame boundary: policy training may
        still use native model qpos, but robot-authored command streams should
        enter through this method.
        """
        from rl_move.joint_frame import robot_abs_rad_to_model_rel_rad
        self.command(robot_abs_rad_to_model_rel_rad(q_robot_abs_rad),
                     speed_deg_s=speed_deg_s, acc_units=acc_units)

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
