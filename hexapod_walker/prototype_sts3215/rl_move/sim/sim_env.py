"""SimHexapodBalanceEnv — MuJoCo twin of the hardware ``HexapodBalanceEnv``.

Same 47-dim observation, 6-dim body-offset action, reward terms, safety
layer and fixed-foot body IK as the real env — only the "robot" is a
MuJoCo model driven through the fitted ``ServoProfile`` (latency +
profile speed + deadband) so a policy trained here sees hardware-like
actuation, not ideal position control.

Usage
-----
    from rl_move.sim.sim_env import SimHexapodBalanceEnv
    env = SimHexapodBalanceEnv(randomize=True, seed=0)
    obs, info = env.reset()
    obs, r, term, trunc, info = env.step(env.action_space.sample())
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.body_ik import FixedFootBodyIK, N_ACT, fk_all_feet  # noqa: E402
from rl_move.config import cfg_get, load_config  # noqa: E402
from rl_move.env import build_obs, compute_reward  # noqa: E402
from rl_move.robot_state import (  # noqa: E402
    DEG2RAD, N_JOINTS, RAD2DEG, RobotState,
)
from rl_move.safety import SafetyLayer, action_to_body_offset  # noqa: E402

from .domain_rand import DomainRandomizer, EpisodeRandomization  # noqa: E402
from .servo_model import (  # noqa: E402
    ServoProfile, SimServoParams, apply_params_to_model, build_model,
    joint_qpos_addrs, joint_qvel_addrs, position_actuator_ids,
)

G0 = 9.80665
N_OBS = 47

# Rise-reference cache (reward.rise_ref_path): one load per process —
# the MJX vec envs build thousands of shims that share this module.
_RISE_REF_CACHE: dict[str, dict] = {}


def load_rise_ref(path: str) -> dict:
    """npz with ``q_rad`` (T,18) joint trajectory of a known-good rise,
    ``dt`` (s/tick at recording) and ``ramp_i0`` (tick where the height
    ref leaves zero). Built by ``extract_rise_ref.py`` from a champion
    rollout (Stage-II reference, HumanUP/HoST style)."""
    ref = _RISE_REF_CACHE.get(path)
    if ref is None:
        z = np.load(path)
        q = np.asarray(z["q_rad"], dtype=float)
        if q.ndim != 2 or q.shape[1] != 18 or len(q) == 0:
            raise ValueError(
                f"rise_ref {path}: expected (T,18) q_rad, got {q.shape}")
        ref = {"q": q, "dt": float(z["dt"]), "ramp_i0": int(z["ramp_i0"])}
        # Per-tick chassis height above the start pose (newer extracts;
        # RSI needs it to command the REMAINING rise from a mid-path
        # spawn). Older npz lack it — RSI refuses, tracking still works.
        if "h_rel_m" in z.files:
            ref["h"] = np.asarray(z["h_rel_m"], dtype=float)
        _RISE_REF_CACHE[path] = ref
    return ref

try:  # gymnasium is optional for pure scripted use
    import gymnasium as _gym
    _GymBase = _gym.Env
except Exception:  # pragma: no cover
    _gym = None
    _GymBase = object


def support_margin_m(feet_xy: np.ndarray, com_xy: np.ndarray) -> float:
    """Signed distance (m) from com_xy to the support-polygon boundary.

    Positive = inside (min distance to any edge), negative = outside.
    feet_xy: (N, 2) contact-foot positions. Needs N >= 3 non-collinear
    points; degenerate inputs return 0.0 (caller gates on contact count).
    Small-N convex hull via Andrew's monotone chain — no scipy.
    """
    pts = np.unique(np.round(np.asarray(feet_xy, dtype=float), 6), axis=0)
    if len(pts) < 3:
        return 0.0
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    cross = lambda o, a, b: ((a[0] - o[0]) * (b[1] - o[1])  # noqa: E731
                             - (a[1] - o[1]) * (b[0] - o[0]))
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(tuple(p))
    for p in pts[::-1]:
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(tuple(p))
    hull = lo[:-1] + up[:-1]          # CCW
    if len(hull) < 3:
        return 0.0                    # collinear feet: no polygon
    d = np.inf
    inside = True
    for i in range(len(hull)):
        a, b = np.array(hull[i]), np.array(hull[(i + 1) % len(hull)])
        e = b - a
        n = np.linalg.norm(e)
        if n < 1e-9:
            continue
        s = cross(a, b, com_xy) / n   # >0 = left of edge = inside (CCW)
        if s < 0:
            inside = False
        d = min(d, abs(s))
    return float(d if inside else -d)


# --------------------------------------------------------------------------
# Valid-plant specification (operator, 2026-08-10). "Standing" is a
# GEOMETRIC condition, not a torso height: every rise arm before this
# lost to a height-only cheat (flag-leg/tripod at height, b2p1; stilt
# pop, rfix-fresh1). A stand is VALID iff, at episode end:
#
#   height     |height_err| <= 15 mm of the commanded target
#   attitude   |roll| and |pitch| <= 10 deg (a stand is level;
#              the 25 deg envelope is for WALKING dynamics)
#   feet down  >= 5 of 6 pads within 20 mm of their grounded z
#   no flags   NO pad above 60 mm (a flag leg is never a stand)
#   support    robot CoM XY inside the down-feet support polygon
#              with >= 20 mm margin
#   footprint  mean body-frame foot XY within 40 mm of the walkable
#              plant footprint (the stance the walk champion expects;
#              rejects the stilt/splay family that passes height +
#              level + feet-down)
#   effort     max per-servo current <= 2.0 A (precarious poses fight;
#              informative once the holding-current model lands —
#              sim hold currents are ~5x under real today, SIM.md)
#
# One function, three consumers: the training reward gate, the eval
# harness (report + optional success gate), and the MDP_PREFLIGHT
# rise bank in test_task_semantics.py. Never let these drift apart.

PLANT_SPEC = {
    "height_err_mm": 15.0,
    "attitude_deg": 10.0,
    "foot_down_mm": 20.0,
    "min_feet_down": 5,
    "flag_leg_mm": 60.0,
    "com_margin_mm": 20.0,
    "footprint_err_mm": 40.0,
    "max_current_a": 2.0,
}


def valid_plant(*, pad_clear_m, feet_xy, com_xy,
                roll_rad, pitch_rad, height_err_m=None,
                footprint_err_m=None, max_current_a=None,
                spec: dict | None = None) -> tuple[bool, dict]:
    """PLANT_SPEC as a predicate. Returns (ok, detail); detail holds
    every sub-check so failures name themselves. None inputs skip
    their check (e.g. height when no target is commanded).
    ``feet_xy`` (6, 2) must be in the same frame as ``com_xy``; only
    the DOWN feet (clearance <= foot_down_mm) form the polygon."""
    s = dict(PLANT_SPEC)
    if spec:
        s.update(spec)
    clear_mm = np.asarray(pad_clear_m, dtype=float) * 1000.0
    down = clear_mm <= s["foot_down_mm"]
    n_down = int(np.sum(down))
    feet_down = np.asarray(feet_xy, dtype=float).reshape(-1, 2)[down]
    margin_mm = support_margin_m(
        feet_down, np.asarray(com_xy, dtype=float)) * 1000.0 \
        if n_down >= 3 else -1e9
    detail = {
        "height_ok": (True if height_err_m is None else
                      abs(height_err_m) * 1000.0 <= s["height_err_mm"]),
        "attitude_ok": (abs(roll_rad) <= s["attitude_deg"] * DEG2RAD
                        and abs(pitch_rad) <= s["attitude_deg"] * DEG2RAD),
        "feet_down_ok": n_down >= s["min_feet_down"],
        "no_flag_ok": float(np.max(clear_mm)) <= s["flag_leg_mm"],
        "support_ok": margin_mm >= s["com_margin_mm"],
        "footprint_ok": (True if footprint_err_m is None else
                         footprint_err_m * 1000.0
                         <= s["footprint_err_mm"]),
        "current_ok": (True if max_current_a is None else
                       max_current_a <= s["max_current_a"]),
        "n_feet_down": n_down,
        "com_margin_mm": round(float(margin_mm), 1),
        "max_clear_mm": round(float(np.max(clear_mm)), 1),
    }
    ok = all(v for k, v in detail.items() if k.endswith("_ok"))
    return bool(ok), detail


def soften_contacts(model) -> None:
    """3x-softer foot/pad/belly solref (see the __init__ comment).

    Module-level so the batched MJX vec env can prepare its SHARED model
    with exactly the same contact softening the C env applies.
    """
    import mujoco
    for i in range(6):
        for gname in (f"L{i}_foot", f"L{i}_pad_col",
                      f"L{i}_yaw_servo_col"):
            fid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            if fid >= 0:
                model.geom_solref[fid, 0] *= 3.0


def set_foot_ground_friction(model, mu_slide: float) -> None:
    """Set the foot–ground SLIDE friction to a calibrated value.

    cfg ``env.foot_friction_slide`` (0 = keep the XML default, foot
    μ=2.0 / floor μ=1.5 → pair μ=2.0). MuJoCo combines a contact
    pair's friction as the element-wise MAX of the two geoms, so the
    floor/terrain AND the foot/pad geoms must all move together —
    changing only the feet would leave the pair pinned at the floor's
    1.5. Calibrate with ``rl_move/sim/calibrate_slip.py`` against the
    tape-measured travel ratio (0.50–0.51, hardware_traces/
    tape_20260810_summary.json). DR's ``friction_scale`` still
    multiplies around this recentered value. Module-level so the MJX
    shared-model prep applies the identical mutation."""
    import mujoco
    names = ["floor", "terrain"]
    for i in range(6):
        names += [f"L{i}_foot", f"L{i}_pad_col"]
    for gname in names:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
        if gid >= 0:
            model.geom_friction[gid, 0] = float(mu_slide)


def _default_plant_deg() -> np.ndarray:
    """Standing plant in hardware convention (learned plant or +20/+80)."""
    try:
        from feetech_bus import standing_pose_degrees
        return np.asarray(standing_pose_degrees(), dtype=float)
    except Exception:
        return np.array([0.0, 20.0, 80.0] * 6, dtype=float)


class SimHexapodBalanceEnv(_GymBase):
    """Gymnasium env; obs/action/reward identical to the hardware env."""

    metadata = {"render_modes": ["rgb_array"]}
    # Extra per-episode attributes a task subclass needs included in the
    # batched MJX vec env's pooled reset-state snapshots (see
    # mjx_vec_env.py). Base env: none.
    MJX_SNAPSHOT_EXTRA: tuple = ()
    # Sliding friction during the reset slip-settle (see reset()): low
    # enough to relieve tangential preload from placement/geometry error,
    # high enough that the plant stance doesn't splay outward under load.
    SLIP_MU = 0.15

    def __init__(self, cfg: dict | None = None, *,
                 params: SimServoParams | None = None,
                 randomize: bool = False,
                 randomizer: DomainRandomizer | None = None,
                 dr_scale: float = 1.0,
                 plant_deg: np.ndarray | list[float] | None = None,
                 episode_seconds: float | None = None,
                 seed: int | None = None,
                 render_mode: str | None = None,
                 mesh_visuals: bool = True,
                 model=None):
        import mujoco
        self._mujoco = mujoco
        self.cfg = cfg if cfg is not None else load_config()
        # bus.servo_params selects the fitted actuator set ("" = air fit,
        # "loaded" = 08-10 loaded bench fit); explicit params win.
        self.params = (params if params is not None
                       else SimServoParams.from_cfg(self.cfg))
        self.render_mode = render_mode
        self.rng = np.random.default_rng(seed)

        # Temporal actor (plan §Architecture): obs.history_frames > 1
        # stacks the last K single-tick observations NEWEST-FIRST, so a
        # parent trained on width W transplants via --obs-pad-transplant
        # (its weights read frame 0 = the current tick; frames 1..K-1
        # start as zero columns). History is built ENV-SIDE so trainer,
        # eval harness, and the hardware bridge see the identical obs.
        self._hist_n = max(1, int(cfg_get(self.cfg, "obs",
                                          "history_frames", default=1)))
        self._hist_buf: list | None = None

        self.dt = 1.0 / float(cfg_get(self.cfg, "control", "hz", default=25))
        ep_s = (episode_seconds if episode_seconds is not None
                else float(cfg_get(self.cfg, "episode", "seconds", default=5)))
        self.episode_steps = int(round(ep_s / self.dt))
        self.write_speed_deg_s = (
            float(cfg_get(self.cfg, "bus", "write_speed", default=400))
            * 360.0 / 4096.0)
        self.write_acc_units = float(
            cfg_get(self.cfg, "bus", "write_acc", default=20))

        # ``model``: a pre-built, fully PREPARED (contact-softened) MjModel
        # shared with other envs — the batched MJX vec env owns physics
        # and passes one model to all its per-env shims. A shared model
        # must never be mutated per episode, so the shim path runs with
        # model DR disabled. Default (None): private model, as always.
        self._owns_model = model is None
        if model is not None:
            self.model = model
        else:
            # Rough terrain (cfg env.terrain_amp > 0) reaches the private
            # C-env model here, so eval-harness / local-viewer episodes run
            # on the same ground the batched trainer used.
            _t_amp = float(cfg_get(self.cfg, "env", "terrain_amp",
                                   default=0.0))
            _t_seed = int(cfg_get(self.cfg, "env", "terrain_seed",
                                  default=0))
            self.model = build_model(fixed_base=False,
                                     flat_terrain=_t_amp <= 0.0,
                                     terrain_amp=_t_amp,
                                     terrain_seed=_t_seed,
                                     mesh_visuals=mesh_visuals)
        self.data = mujoco.MjData(self.model)
        self._substeps = max(1, int(round(self.dt / self.model.opt.timestep)))
        self._qadr = joint_qpos_addrs(self.model)
        self._vadr = joint_qvel_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)
        self._chassis_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        gid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self._gyro_adr = self.model.sensor_adr[gid]
        # Foot touch sensors — the unload task's ground-truth leg load.
        self._touch_adr = []
        for i in range(6):
            sid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, f"L{i}_foot_t")
            self._touch_adr.append(
                self.model.sensor_adr[sid] if sid >= 0 else -1)
        # Foot pad bodies + per-episode grounded-z reference, for the
        # stance-clearance penalty (see step()).
        self._pad_bids = [mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_pad")
            for i in range(6)]
        self._pad_z_ref: np.ndarray | None = None
        self._end_posture_from: int | None = None

        # Soften the foot contacts: the CAD model's solref (0.01 s) is
        # near-rigid, so mm-scale randomized leg-length differences make
        # 2-3 "long" legs carry the whole robot (three-legged-stool) and
        # the stiff servos read 2-3 A standing still. Real rubber feet +
        # PLA leg flex compress ~1-2 mm and spread the load; ~3x softer
        # timeconst gives that. DR's contact_stiff_scale still varies it.
        # (A shared model arrives already softened — soften ONCE, or
        # every env would multiply solref by another 3x.)
        if self._owns_model:
            soften_contacts(self.model)
            # Calibrated foot–ground slide μ (0 = XML default). Applied
            # BEFORE the pristine copies so DR restores + rescales around
            # the calibrated value. Shared models arrive already prepared.
            _mu = float(cfg_get(self.cfg, "env", "foot_friction_slide",
                                default=0.0))
            if _mu > 0.0:
                set_foot_ground_friction(self.model, _mu)

        # Pristine copies for DR restore at every reset.
        self._base_body_mass = self.model.body_mass.copy()
        self._base_body_inertia = self.model.body_inertia.copy()
        self._base_body_ipos = self.model.body_ipos.copy()
        self._base_body_pos = self.model.body_pos.copy()
        self._base_geom_pos = self.model.geom_pos.copy()
        self._base_site_pos = self.model.site_pos.copy()
        self._base_geom_friction = self.model.geom_friction.copy()
        self._base_geom_solref = self.model.geom_solref.copy()
        self._base_gravity = self.model.opt.gravity.copy()

        if randomizer is not None:
            self.randomizer = randomizer
        elif randomize:
            self.randomizer = DomainRandomizer.from_params(
                self.params, scale=dr_scale)
        else:
            self.randomizer = None
        # cfg-driven DR range overrides: --cfg-set dr.<field>=v or "lo,hi".
        # ABSOLUTE values applied AFTER dr_scale scaling (an override is the
        # experiment's exact range, e.g. payload dr.mass_scale=1.0,1.5).
        # Unknown fields raise — a typo must fail the launch, not silently
        # train the default DR. Covers CPU and MJX stacks (the MJX host
        # applies this env's _ep_rand per world).
        if self.randomizer is not None:
            for _k, _v in (self.cfg.get("dr") or {}).items():
                if not hasattr(self.randomizer.ranges, _k):
                    raise ValueError(f"unknown DR override dr.{_k}")
                if isinstance(_v, str):
                    _parts = tuple(float(x) for x in _v.split(","))
                    _v = _parts[0] if len(_parts) == 1 else _parts
                setattr(self.randomizer.ranges, _k, _v)
        self._ep_rand: EpisodeRandomization | None = None

        self._plant_deg = (np.asarray(plant_deg, dtype=float).reshape(N_JOINTS)
                           if plant_deg is not None else _default_plant_deg())

        self.ik = FixedFootBodyIK()
        self.safety = SafetyLayer(self.cfg)
        # Subclasses with a different action space (e.g. raw joint targets)
        # override n_act and _act_to_q; everything else is shared.
        self.n_act = N_ACT
        self._q_nom = np.zeros(N_JOINTS, dtype=float)
        self._prev_action = np.zeros(self.n_act, dtype=float)
        self._cmd = np.zeros(N_JOINTS, dtype=float)
        self._profile: ServoProfile | None = None
        self._step_i = 0
        self._episode = 0
        self._state: RobotState | None = None
        self._renderer = None
        self._goal_traj = None            # set by goal-conditioned subclass
        self._imu_prev_v: np.ndarray | None = None
        self._imu_f_accum = np.zeros(3)
        self._imu_f_n = 0
        self._gyro_accum = np.zeros(3)
        self._gyro_n = 0
        self._att_rp: np.ndarray | None = None
        self._tilt_ref0 = (0.0, 0.0)
        self._z0 = 0.0

        if _gym is not None:
            self.observation_space = self._obs_space_box(N_OBS)
            self.action_space = _gym.spaces.Box(
                -1.0, 1.0, shape=(self.n_act,), dtype=np.float32)

    # ------------------------------------------------------------------
    # observation finalization: subclass augmentation + history stacking
    # ------------------------------------------------------------------

    def _obs_space_box(self, width: int):
        """Box obs space for a single-frame width, times history depth."""
        return _gym.spaces.Box(-np.inf, np.inf,
                               shape=(width * self._hist_n,),
                               dtype=np.float32)

    def _augment_obs(self, obs: np.ndarray, *,
                     reset: bool = False) -> np.ndarray:
        """Subclass hook: append extra per-tick dims (walk vel/phase).

        Runs BEFORE history stacking so appended dims are part of every
        stacked frame. Base env: identity.
        """
        return obs

    def _final_obs(self, obs: np.ndarray, *, reset: bool) -> np.ndarray:
        """Apply the augment hook, then the obs-history stack."""
        obs = self._augment_obs(obs, reset=reset).astype(np.float32)
        if self._hist_n <= 1:
            return obs
        if reset or self._hist_buf is None:
            self._hist_buf = [obs.copy() for _ in range(self._hist_n)]
        else:
            self._hist_buf.pop()
            self._hist_buf.insert(0, obs.copy())
        # newest first: frame 0 is the current tick (transplant prefix).
        return np.concatenate(self._hist_buf).astype(np.float32)

    # ------------------------------------------------------------------
    # state readout (sim → RobotState, with DR sensor corruption)
    # ------------------------------------------------------------------

    def _read_state(self) -> RobotState:
        mujoco = self._mujoco
        q = self.data.qpos[self._qadr].copy()
        qd = self.data.qvel[self._vadr].copy()

        # Attitude the way the hardware computes it: from the accelerometer
        # specific force f = a - g at the IMU's mounting point. Gravity may
        # be tilted (ground slope DR), the IMU frame rotated (mount
        # misalignment DR), and — because the IMU could be bolted anywhere
        # on the robot — the mounting point offset from the chassis center.
        # An off-center IMU picks up lever-arm acceleration whenever the
        # body rotates, corrupting the tilt estimate exactly during leans;
        # that corruption is the point of modeling it.
        er = self._ep_rand
        R = self.data.xmat[self._chassis_bid].reshape(3, 3).copy()
        mount = np.eye(3) if er is None else er.imu_mount_rot
        # Specific force averaged over the tick's physics substeps (see
        # _advance) — like a 1 kHz-sampled, low-passed MEMS accel read at
        # 25 Hz. FD across whole control ticks aliased servo dither into
        # ±10° phantom tilt spikes.
        if self._imu_f_n > 0:
            f_world = self._imu_f_accum / self._imu_f_n
        else:
            f_world = -np.asarray(self.model.opt.gravity, dtype=float)
        self._imu_f_accum[:] = 0.0
        self._imu_f_n = 0
        f_imu = (R @ mount).T @ f_world
        ax, ay, az = f_imu  # measures +g when level and static
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))
        if self._gyro_n > 0:
            gyro = self._gyro_accum / self._gyro_n
        else:
            gyro = self.data.sensordata[
                self._gyro_adr:self._gyro_adr + 3].copy()
        self._gyro_accum[:] = 0.0
        self._gyro_n = 0

        if er is not None:
            q = q + er.joint_zero_bias_rad
            q = q + self.rng.normal(0.0, er.encoder_noise_rad, N_JOINTS)
            roll_acc += (er.imu_bias_rad[0]
                         + self.rng.normal(0.0, er.tilt_noise_rad))
            pitch_acc += (er.imu_bias_rad[1]
                          + self.rng.normal(0.0, er.tilt_noise_rad))
            gyro = er.imu_mount_rot.T @ gyro  # gyro axes rotate with mount
            gyro = (gyro + er.gyro_bias_rad_s
                    + self.rng.normal(0.0, er.gyro_noise_rad_s, 3))

        # Complementary filter, same as the hardware estimator
        # (ComplementaryAttitude alpha=0.98): integrate gyro, drift-correct
        # slowly toward the accel tilt. Without it the raw accel tilt sees
        # the full lever-arm spikes of an off-center IMU (±20° for one
        # tick) — real firmware filters those out, so the sim must too.
        alpha = 0.98
        if self._att_rp is None:
            self._att_rp = np.array([roll_acc, pitch_acc])
        else:
            self._att_rp = np.array([
                alpha * (self._att_rp[0] + gyro[0] * self.dt)
                + (1.0 - alpha) * roll_acc,
                alpha * (self._att_rp[1] + gyro[1] * self.dt)
                + (1.0 - alpha) * pitch_acc])
        roll, pitch = float(self._att_rp[0]), float(self._att_rp[1])

        # Servo current ≈ |net actuator torque| × A/N·m. Feeds the effort
        # reward penalty AND the SafetyLayer over-current trip (2.5 A), so
        # stall-fighting a bad pose terminates in sim like it must on
        # hardware. 1.2 A/N·m puts torque saturation (2.2 N·m) just past
        # the trip — SUSTAINED saturation = episode over. Low-pass with a
        # ~0.1 s time constant so a millisecond torque spike doesn't trip:
        # hardware reads current at ~10 Hz and never sees such transients.
        torque = self.data.qfrc_actuator[self._vadr]
        # Cap at ~stall current: the same motor is on every joint, so the
        # estimate can't exceed what the winding physically draws at 12 V
        # (fitted per-axis torque limits would otherwise let some axes
        # report 4+ A).
        raw_current = np.minimum(np.abs(torque) * 1.2, 3.0)
        if getattr(self, "_cur_filt", None) is None:
            self._cur_filt = raw_current
        else:
            alpha = self.dt / (self.dt + 0.1)
            self._cur_filt = (1.0 - alpha) * self._cur_filt + alpha * raw_current
        servo_current = self._cur_filt.copy()

        del mujoco
        return RobotState(
            timestamp=self.data.time,
            joint_position=q,
            joint_velocity=qd,
            imu_roll=float(roll),
            imu_pitch=float(pitch),
            imu_yaw=0.0,
            imu_gyro=gyro,
            imu_accel=f_imu,
            commanded_position=self._cmd.copy(),
            servo_current=servo_current,
            bus_ok=True,
            imu_ok=True,
            dt=self.dt,
        )

    # ------------------------------------------------------------------
    # physics
    # ------------------------------------------------------------------

    def _advance(self, *, limp: bool = False) -> None:
        assert self._profile is not None
        mujoco = self._mujoco
        h = self.model.opt.timestep
        r_off = (np.zeros(3) if self._ep_rand is None
                 else self._ep_rand.imu_pos_m)
        vel = np.zeros(6)
        for _ in range(self._substeps):
            target = self._profile.tick(h)
            q = self.data.qpos[self._qadr]
            if limp:
                # Torque-off settling (reset only): the actuator reference
                # follows q, leaving pure kv damping — how an operator
                # lays the robot down before enabling hold.
                eff = q
            else:
                # Firmware dead-zone at the PHYSICS level: inside the
                # deadband the real controller outputs nothing, so a servo
                # holding a settled pose applies ~zero torque. Without
                # this, the stiff fitted kp (≈1000 Nm/rad) turns a 0.2°
                # captured-pose offset into a 3 Nm isometric fight against
                # the ground — the robot chatters on its contacts and
                # "draws" 3 A lying still (belly-rest episodes tripped
                # over_current doing nothing). Soft dead-zone: torque
                # grows smoothly from zero past the band.
                err = target - q
                db = self._profile.deadband_rad
                eff = q + np.sign(err) * np.maximum(np.abs(err) - db, 0.0)
            self.data.ctrl[self._pos_act] = eff
            mujoco.mj_step(self.model, self.data)
            # Accumulate the IMU-point specific force at the physics rate
            # (exact velocities, one FD) — includes the lever-arm
            # acceleration of an off-center IMU without tick-rate
            # aliasing. Averaged per control tick in _read_state.
            mujoco.mj_objectVelocity(
                self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                self._chassis_bid, vel, 0)
            R = self.data.xmat[self._chassis_bid].reshape(3, 3)
            v_pt = vel[3:] + np.cross(vel[:3], R @ r_off)
            if self._imu_prev_v is not None:
                a_pt = (v_pt - self._imu_prev_v) / h
                self._imu_f_accum += a_pt - self.model.opt.gravity
                self._imu_f_n += 1
            self._imu_prev_v = v_pt.copy()
            # Gyro too: the chassis micro-dithers at tens of Hz (stiff
            # contacts + stiff servos); sampling the instantaneous rate
            # once per control tick aliases that into phantom rotation
            # which the attitude filter then integrates. The real MPU
            # integrates at 1 kHz where zero-mean dither cancels.
            self._gyro_accum += self.data.sensordata[
                self._gyro_adr:self._gyro_adr + 3]
            self._gyro_n += 1

    def _settle(self, seconds: float, *, limp: bool = False) -> None:
        n = int(round(seconds / self.dt))
        for _ in range(n):
            self._advance(limp=limp)

    @staticmethod
    def _clip_to_joint_limits(q: np.ndarray) -> np.ndarray:
        from rl_move.safety import AXIS_LIMITS_DEG
        q = q.copy()
        for j in range(N_JOINTS):
            lo, hi = AXIS_LIMITS_DEG[j % 3]
            q[j] = float(np.clip(q[j], lo * DEG2RAD, hi * DEG2RAD))
        return q

    def _start_pose_rad(self) -> np.ndarray:
        """Plant pose plus this episode's placement noise / bad-start
        offsets, clipped to the hardware joint limits."""
        q = self._plant_deg * DEG2RAD
        if self._ep_rand is not None:
            q = q + self._ep_rand.start_offset_rad
        return self._clip_to_joint_limits(q)

    # Tipped-start hip-fold gain: settled body roll per degree of hip
    # fold, measured on the CPU twin at the plant stance (probe 08-10:
    # settled roll ≈ 0.36 × fold, near-linear over 6-18° targets; see
    # the dr.tipped_start_* axis in domain_rand.py). The inverse maps
    # the sampled target roll to the fold the pattern commands.
    TIP_ROLL_PER_FOLD = 0.36

    def _apply_tipped_start(self, q_start: np.ndarray) -> np.ndarray:
        """Add the tipped-start (roll recovery) pattern, if this episode
        drew one (dr.tipped_start_*; plant/park starts only — the
        caller decides, belly-rise starts never tip).

        The sampled target body roll becomes an asymmetric leg fold:
        folding one side's hips drops the body on that side once the
        feet load (the same hip sign the park start uses to LIFT feet —
        with the foot planted the body moves instead); the folded
        side's knees extend a little to keep the feet under the hips.
        The target is capped at 70% of the run's own tilt-trip envelope
        so the episode spawns with recovery headroom, never mid-trip
        (stance runs at 10° see ≤7° tips; the 25° deployment contract
        sees ≤17.5°). Sets ``_tipped_applied`` so _reset_finalize keeps
        the tilt reference LEVEL instead of re-anchoring at the lean.
        """
        er = self._ep_rand
        if er is None or abs(er.tipped_roll_deg) < 1e-9:
            return q_start
        cap = 0.7 * self.safety.max_roll * RAD2DEG
        roll = float(np.clip(er.tipped_roll_deg, -cap, cap))
        fold = abs(roll) / self.TIP_ROLL_PER_FOLD * DEG2RAD
        # Legs 0-2 mount on the +y (left) side (azimuths 30/90/150°),
        # legs 3-5 on the right; positive roll leans the body right
        # (IMU convention: roll = atan2(ay, az), +y side up).
        legs = (3, 4, 5) if roll > 0 else (0, 1, 2)
        q = q_start.copy()
        for leg in legs:
            q[3 * leg + 1] -= fold
            q[3 * leg + 2] += 0.5 * fold
        self._tipped_applied = True
        return q

    def _true_roll_pitch(self) -> tuple[float, float]:
        """Ground-truth chassis attitude in the IMU's roll/pitch
        convention (privileged; reset-time only). Uses the episode's
        own gravity so slope DR stays inside the tilt reference,
        exactly like the legacy start-attitude anchoring."""
        R = np.asarray(self.data.xmat[self._chassis_bid],
                       dtype=float).reshape(3, 3)
        g = (self._ep_rand.gravity_vec if self._ep_rand is not None
             else np.array([0.0, 0.0, -9.80665]))
        f = R.T @ (-g)   # static specific force in the body frame
        return (math.atan2(f[1], f[2]),
                math.atan2(-f[0], math.hypot(f[1], f[2])))

    def _walk_park_bank(self) -> np.ndarray | None:
        """Harvested own-park poses (cycle 27). Lazy-loads the npz named
        by cfg goal.walk_park_bank (key ``q_rad``, shape (K,18)); caches
        None when unset so the legacy path costs one attribute check."""
        if hasattr(self, "_park_bank_cache"):
            return self._park_bank_cache
        path = cfg_get(self.cfg, "goal", "walk_park_bank", default=None)
        bank = None
        if path:
            arr = np.load(str(path))["q_rad"]
            arr = np.asarray(arr, dtype=float)
            if arr.ndim != 2 or arr.shape[1] != N_JOINTS or len(arr) == 0:
                raise ValueError(
                    f"walk_park_bank {path}: expected (K,{N_JOINTS}) "
                    f"q_rad, got {arr.shape}")
            bank = arr
        self._park_bank_cache = bank
        return bank

    def _place_at_plant(self, q_rad: np.ndarray) -> None:
        """Set qpos to ``q_rad`` with the chassis at foot-contact height."""
        import mujoco_prototype as MP
        mujoco = self._mujoco
        feet = fk_all_feet(q_rad)              # body frame, z at yaw plane
        foot_drop = float(np.min(feet[:, 2]))  # most negative = lowest foot
        base_z = MP.YAW_OUTPUT_HEIGHT - foot_drop + MP.FOOT_R + 0.002

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, base_z)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self._qadr] = q_rad
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self._pos_act] = q_rad
        mujoco.mj_forward(self.model, self.data)
        # Foot-height placement assumes feet are the lowest points. At the
        # zero pose (legs straight out) the yaw-servo belly boxes are lower
        # than the feet and would start inside the floor — lift until
        # nothing penetrates, then the settle drops it onto the belly.
        for _ in range(40):
            worst = 0.0
            for ci in range(self.data.ncon):
                worst = min(worst, float(self.data.contact[ci].dist))
            if worst > -1e-4:
                break
            self.data.qpos[2] += -worst + 0.001
            mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    # gym API
    # ------------------------------------------------------------------

    def _reset_begin(self, seed: int | None = None) -> np.ndarray:
        """Pre-physics half of reset: bookkeeping, this episode's DR
        sample, goal sample, and the start pose. Touches NO model or
        physics state, so the batched MJX vec env can drive it for a
        shim env and run the settle choreography itself. Returns
        q_start (rad, 18). RNG draw order (DR sample, then goal) is
        identical to the historical inline code.
        """
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._episode += 1
        self._step_i = 0
        self._prev_action[:] = 0.0
        self.safety.clear_estop()
        self._tipped_applied = False

        self._ep_rand = (self.randomizer.sample(self.rng)
                         if self.randomizer is not None else None)

        # Goal first: it decides the reset pose. Rise episodes start at
        # the ZERO pose — legs straight out, belly resting on the yaw
        # servos, exactly how the operator places the robot — and must
        # curl the legs in and stand. Everything else starts at the plant.
        self._goal_traj = self._sample_goal()
        start_at = ("plant" if self._goal_traj is None
                    else getattr(self._goal_traj, "start_at", "plant"))
        # Reference state initialization (RSI, DeepMimic-style; operator
        # 08-10 late). The 08-10 forensic ladder (score1 -> scoreref1 ->
        # -dr0 -> -dr0-lowlr -> -dr0-riseonly) proved the rise reward
        # orders correctly even under full exploration noise (noisy
        # replay +357 vs cheats < 0) yet training NEVER visits the paid
        # states: pricing, DR, LR, mode interference and noise were each
        # exonerated by a controlled run, leaving pure exploration - the
        # gradient cannot cross from "lying at the ref start" to "the
        # full rise" because nothing between pays. RSI closes it by
        # SPAWNING rise episodes on the demonstrated path at a random
        # phase (belly curl through ~90% of the ramp), so rollouts
        # experience the paid states directly and learning can proceed
        # backward along the path. cfg goal.rise_rsi_frac in [0,1]
        # (default 0 = legacy exact; no rng draw when off), needs
        # reward.rise_ref_path. The settle choreography (incl. the limp
        # sag) runs unchanged; _reset_finalize re-aligns the reference
        # clock to the nearest path point of the pose that actually
        # settled, so the mechanism is robust to sag on every impl.
        self._rsi_pending = False
        self._rsi_ref_tick0: int | None = None
        if (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "rise"):
            rsi_f = float(cfg_get(self.cfg, "goal", "rise_rsi_frac",
                                  default=0.0))
            rsi_ref = cfg_get(self.cfg, "reward", "rise_ref_path",
                              default=None)
            if rsi_f > 0.0 and rsi_ref:
                ref = load_rise_ref(str(rsi_ref))
                # "h" (per-tick height) only exists in newer extracts;
                # without it the remaining-rise schedule can't be built.
                if "h" in ref and self.rng.random() < rsi_f:
                    n, i0 = len(ref["q"]), int(ref["ramp_i0"])
                    j = int(self.rng.integers(
                        0, i0 + int(0.9 * (n - 1 - i0))))
                    q_rsi = ref["q"][j] + self.rng.uniform(
                        -2.0, 2.0, N_JOINTS) * DEG2RAD
                    self._rsi_pending = True
                    return self._clip_to_joint_limits(q_rsi)
        if start_at == "zero":
            q_start = np.zeros(N_JOINTS, dtype=float)
            # Bridge start (rise reverse-curriculum): blend the start
            # joints toward the crouch pose. Zero pose is exactly q=0,
            # so the blend is a plain scale of the crouch solution.
            f = float(getattr(self._goal_traj, "start_curl", 0.0))
            if f > 0.0:
                from rl_move.body_ik import BodyOffset
                bridge_ik = FixedFootBodyIK()
                bridge_ik.reset(self._plant_deg * DEG2RAD)
                res = bridge_ik.solve(BodyOffset(
                    height=-float(self._goal_traj.crouch_dz)))
                if res.ok:
                    q_start = f * res.q_rad
            if self._ep_rand is not None:
                q_start = self._clip_to_joint_limits(
                    q_start + self._ep_rand.start_offset_rad)
        elif start_at == "crouch":
            # Feet at the plant footprint, body crouch_dz lower: solve
            # the same fixed-foot IK the policy uses.
            from rl_move.body_ik import BodyOffset
            crouch_ik = FixedFootBodyIK()
            crouch_ik.reset(self._plant_deg * DEG2RAD)
            res = crouch_ik.solve(
                BodyOffset(height=-float(self._goal_traj.crouch_dz)))
            q_start = (self._clip_to_joint_limits(res.q_rad) if res.ok
                       else self._start_pose_rad())
            if self._ep_rand is not None:
                q_start = self._clip_to_joint_limits(
                    q_start + self._ep_rand.start_offset_rad)
        elif start_at == "park":
            # Tripod-park start (walk reset diversity, cycle 24): plant
            # pose with one alternating tripod's hips lifted 10-25 deg
            # (feet hover ~15-45 mm — the park attractor observed on
            # camera, duty ~[0.9,0.1,0.9,0.1,0.9,0.1]) plus small knee
            # jitter. The policy must step OUT of the park to earn; see
            # walk_task._sample_walk for the rationale.
            # HARVESTED bank (cycle 27): synthetic tripods taught exits
            # from synthetic parks while the policy's OWN park survived
            # (dose refuted at update parity). cfg goal.walk_park_bank
            # (npz path with q_rad (K,18), built by harvest_park_states)
            # + goal.walk_park_bank_frac f: with prob f a park start is
            # drawn from the bank (+-2 deg jitter) instead of synthetic.
            # Bank checks are short-circuited so the legacy rng stream
            # is untouched when no bank is configured.
            bank = self._walk_park_bank()
            bank_frac = float(cfg_get(self.cfg, "goal",
                                      "walk_park_bank_frac", default=0.5))
            if bank is not None and self.rng.random() < bank_frac:
                q_start = bank[int(self.rng.integers(len(bank)))].copy()
                q_start += self.rng.uniform(
                    -2.0, 2.0, N_JOINTS) * DEG2RAD
            else:
                q_start = (self._plant_deg * DEG2RAD).copy()
                tripod = (1, 3, 5) if self.rng.random() < 0.5 else (0, 2, 4)
                for leg in tripod:
                    q_start[3 * leg + 1] -= float(
                        self.rng.uniform(10.0, 25.0)) * DEG2RAD
                    q_start[3 * leg + 2] += float(
                        self.rng.uniform(-5.0, 10.0)) * DEG2RAD
            if self._ep_rand is not None:
                q_start = q_start + self._ep_rand.start_offset_rad
            q_start = self._apply_tipped_start(q_start)
            q_start = self._clip_to_joint_limits(q_start)
        else:
            q_start = self._clip_to_joint_limits(
                self._apply_tipped_start(self._start_pose_rad()))
        return q_start

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        del options
        q_start = self._reset_begin(seed)

        # Restore pristine model, then apply this episode's randomization.
        self.model.body_mass[:] = self._base_body_mass
        self.model.body_inertia[:] = self._base_body_inertia
        self.model.body_ipos[:] = self._base_body_ipos
        self.model.body_pos[:] = self._base_body_pos
        self.model.geom_pos[:] = self._base_geom_pos
        self.model.site_pos[:] = self._base_site_pos
        self.model.geom_friction[:] = self._base_geom_friction
        self.model.geom_solref[:] = self._base_geom_solref
        self.model.opt.gravity[:] = self._base_gravity
        if self._ep_rand is not None:
            self._ep_rand.apply_to_model(
                self.model, chassis_bid=self._chassis_bid)
            apply_params_to_model(
                self.model, self.params,
                kp_scale=self._ep_rand.kp_scale,
                kv_scale=self._ep_rand.kv_scale,
                torque_scale=self._ep_rand.torque_scale)
        else:
            apply_params_to_model(self.model, self.params)

        self._place_at_plant(q_start)
        er = self._ep_rand
        self._profile = ServoProfile(
            self.params, q_start,
            latency_scale=1.0 if er is None else er.latency_scale,
            deadband_scale=1.0 if er is None else er.deadband_scale,
            vel_scale=1.0 if er is None else er.vel_scale,
        )
        self._cmd = q_start.copy()
        # Settle with slippery feet AND limp servos first: when a human
        # sets the robot down (torque off), feet micro-slip and joints
        # sag until the structure reaches a passive equilibrium —
        # otherwise randomized geometry + pinned contacts leave the legs
        # isometrically preloaded at 2-3 A from step 0.
        fr = self.model.geom_friction[:, 0].copy()
        self.model.geom_friction[:, 0] = self.SLIP_MU
        self._settle(0.4)          # stiff: reach the commanded pose
        self._settle(0.5, limp=True)   # limp: bleed off contact preload
        self.model.geom_friction[:, 0] = fr

        # Hold-current semantics, same as the hardware env: nominal is the
        # pose the robot actually SETTLED at (however badly it was placed),
        # not the ideal plant. Capturing the PASSIVE equilibrium means
        # "hold this pose" needs ~zero torque — like hardware, where
        # q_nom is read from encoders while the servos are unloaded.
        self._q_nom = self.data.qpos[self._qadr].copy()
        self._profile.reset(self._q_nom)
        self._cmd = self._q_nom.copy()
        self._settle(0.3)
        return self._reset_finalize()

    def _reset_finalize(self):
        """Post-settle half of reset: episode references, filter resets,
        first state read, first obs. Reads physics only through
        ``self.data`` (the vec env feeds a shim env a batched-tick data
        view), with ``self._q_nom`` already captured by the caller.
        Returns the (obs, info) reset tuple.
        """
        # Curl channel target: the ideal plant footprint (foot anchors can
        # slide from wherever they started toward it — required to stand
        # up from the zero pose, useful to fix a badly-placed leg).
        self.ik.reset(self._q_nom, plant_q_rad=self._plant_deg * DEG2RAD)
        self.safety.set_nominal(self._q_nom)
        self._cur_filt = None
        self._imu_prev_v = None
        self._imu_f_accum[:] = 0.0
        self._imu_f_n = 0
        self._gyro_accum[:] = 0.0
        self._gyro_n = 0
        self._att_rp = None
        # Height anchor: goal height refs (rise) are relative to wherever
        # the body actually settled, same convention as the tilt refs.
        self._z0 = float(self.data.xpos[self._chassis_bid, 2])
        # Grounded pad heights at episode start. Stance episodes begin at
        # the plant with all six feet loaded (verified by zero-action
        # probe), so this is the "foot down" z for each pad.
        self._pad_z_ref = np.array(
            [float(self.data.xpos[b, 2]) if b >= 0 else 0.0
             for b in self._pad_bids])
        # Per-episode cache: first charged tick of the terminal
        # end-posture window (computed lazily from the goal schedule).
        self._end_posture_from = None
        # RSI episodes: align the reference clock to the path point the
        # robot ACTUALLY settled at (the limp-settle stage sags mid-rise
        # poses; nearest-neighbor re-alignment makes the mechanism
        # sag-robust on every impl), then rewrite the height schedule to
        # command the REMAINING rise from the settled spawn — heights
        # stay relative to _z0 like every other episode's.
        if getattr(self, "_rsi_pending", False) and self._goal_traj is not None:
            ref = load_rise_ref(str(cfg_get(
                self.cfg, "reward", "rise_ref_path", default="")))
            q_set = np.asarray(self.data.qpos[self._qadr], dtype=float)
            rms = np.sqrt(((ref["q"] - q_set[None, :]) ** 2).mean(axis=1))
            j0 = int(np.argmin(rms))
            self._rsi_ref_tick0 = j0
            h_end = float(ref["h"][-1])
            h_left = max(h_end - float(ref["h"][j0]), 0.002)
            ramp_s = float(cfg_get(self.cfg, "goal", "rise_ramp_s",
                                   default=6.0))
            n_ramp = max(int(round(ramp_s * min(h_left / max(h_end, 1e-3),
                                                1.0) / self.dt)), 3)
            n_ep = len(np.asarray(self._goal_traj.height))
            self._goal_traj.height = h_left * np.clip(
                np.arange(n_ep, dtype=float) / n_ramp, 0.0, 1.0)
        # Staged height scores (rise/raise/lower): potential-based
        # progress on |height_err| plus one-time milestone bonuses at
        # fractions of the episode's height target. Sim-only (privileged
        # body height). SIGNED: lower episodes have a negative target and
        # milestones fire on the way down.
        if self._goal_traj is not None:
            h = np.asarray(self._goal_traj.height)
            self._h_target = float(h[int(np.argmax(np.abs(h)))])
        else:
            self._h_target = 0.0
        self._h_milestones: set[float] = set()
        self._prev_h_err_abs = 0.0
        # Stand-score ratchet baseline (reward.rise_score_income): seeded
        # with the episode's FIRST score so crouch/near-plant starts don't
        # collect their starting posture as free income.
        self._score_best: float | None = None
        # Feet-under-body ("curl") scores, rise episodes only: mean XY
        # distance from each foot to its plant-footprint anchor. Curling
        # changes NO height term (belly stays down), so without this the
        # one step that makes standing possible has zero gradient.
        self._is_rise = (self._goal_traj is not None
                         and getattr(self._goal_traj, "mode", "") == "rise")
        # First ramp tick of a rise schedule (hold window ends here) —
        # the alignment anchor for the rise-reference tracking term:
        # references are recorded ramp-relative so episodes with
        # jittered hold lengths all join the same trajectory.
        self._rise_ramp_i0 = 0
        if self._is_rise:
            nz = np.nonzero(
                np.abs(np.asarray(self._goal_traj.height)) > 1e-12)[0]
            self._rise_ramp_i0 = int(nz[0]) if len(nz) else 0
        self._plant_feet_xy = fk_all_feet(
            self._plant_deg * DEG2RAD)[:, :2]
        self._curl_dist_prev = self._curl_dist()
        self._curl_milestones: set[float] = set()
        self._state = self._read_state()
        # Anchor trip, obs, and reward to the start attitude — mount bias
        # / slope isn't tipping, and goals mean "lean from here".
        self._tilt_ref0 = (self._state.imu_roll, self._state.imu_pitch)
        if getattr(self, "_tipped_applied", False):
            # Tipped-start episodes (dr.tipped_start_*) keep the
            # reference LEVEL: subtract the privileged true attitude so
            # only the IMU bias/mount part stays in the ref. The policy
            # then SEES the lean in obs and the attitude terms pay it
            # to level out — re-anchoring at the tip would train it to
            # HOLD the lean, the opposite of the point.
            t_roll, t_pitch = self._true_roll_pitch()
            self._tilt_ref0 = (self._state.imu_roll - t_roll,
                               self._state.imu_pitch - t_pitch)
        self.safety.set_tilt_reference(*self._tilt_ref0)
        er = self._ep_rand
        info = {
            "episode": self._episode,
            "q_nominal_deg": (self._q_nom * RAD2DEG).tolist(),
            "roll_deg": self._state.imu_roll * RAD2DEG,
            "pitch_deg": self._state.imu_pitch * RAD2DEG,
            # Attitude relative to the episode's tilt reference — the
            # tipped-start recovery metric reads this (≈ true lean for
            # tipped episodes, where the ref stays level).
            "roll_rel_deg": (self._state.imu_roll
                             - self._tilt_ref0[0]) * RAD2DEG,
            "randomization": None if er is None else er.summary(),
        }
        goal = self._current_goal()
        if goal is not None:
            info["goal_mode"] = self._goal_traj.mode
        return self._final_obs(
            build_obs(self.cfg, self._state, self._q_nom,
                      self._prev_action, goal=goal,
                      tilt_ref=self._tilt_ref0), reset=True), info

    def _curl_dist(self) -> float:
        """Mean XY distance (m) from each foot to its plant anchor,
        computed in the body frame from true joint angles."""
        feet = fk_all_feet(self.data.qpos[self._qadr])[:, :2]
        return float(np.mean(
            np.linalg.norm(feet - self._plant_feet_xy, axis=1)))

    def plant_report(self,
                     height_err_m: float | None = None
                     ) -> tuple[bool, dict]:
        """Live PLANT_SPEC check of the CURRENT tick (see valid_plant
        at module level — the shared stand criterion). The footprint
        term reuses _curl_dist (body-frame FK vs the plant anchors),
        so the identical criterion is available to the reward path,
        the eval harness, and the semantics bank."""
        if self._pad_z_ref is None or any(b < 0 for b in self._pad_bids):
            return False, {"error": "no pad clearance reference"}
        clear = [float(self.data.xpos[b, 2]) - self._pad_z_ref[i]
                 for i, b in enumerate(self._pad_bids)]
        feet_xy = np.array([self.data.xpos[b, :2]
                            for b in self._pad_bids])
        cur = self._state.servo_current
        return valid_plant(
            pad_clear_m=clear, feet_xy=feet_xy,
            com_xy=self.data.subtree_com[0, :2],
            roll_rad=self._state.imu_roll,
            pitch_rad=self._state.imu_pitch,
            height_err_m=height_err_m,
            footprint_err_m=self._curl_dist(),
            max_current_a=(float(np.max(np.abs(cur)))
                           if cur is not None else None))

    # Goal-conditioned subclass hooks (base env: no goal, 47-dim obs).
    def _sample_goal(self):
        return None

    def _current_goal(self):
        if self._goal_traj is None:
            return None
        return self._goal_traj.at(self._step_i)

    def _act_to_q(self, clipped: np.ndarray):
        """Map a clipped action to joint targets: (q_rad, ok, reason).

        Base env: body-offset action through the fixed-foot IK. The raw
        joint-space subclass overrides this and nothing else.
        """
        offset = action_to_body_offset(clipped, self.cfg)
        ik = self.ik.solve(offset)
        return ik.q_rad, ik.ok, ik.reason

    def _step_begin(self, action):
        """Pre-physics half of step: action validation, IK, safety
        filter, and the servo command. Returns ``(early, ctx)`` —
        ``early`` is a full step tuple when the action was rejected
        outright (no physics runs in that case), else None with ``ctx``
        for :meth:`_step_finish` after physics has advanced one tick.
        Split so the batched MJX vec env can run all envs' pre-physics
        halves, one batched tick, then all post-physics halves.
        """
        assert self._state is not None and self._profile is not None
        clipped, bad = self.safety.validate_action(action, n_act=self.n_act)
        pen = float(cfg_get(self.cfg, "reward",
                            "safety_termination_penalty", default=10))
        if clipped is None:
            self._step_i += 1
            parts = {"reward_termination": -pen}
            return (self._final_obs(
                        build_obs(self.cfg, self._state, self._q_nom,
                                  self._prev_action,
                                  goal=self._current_goal(),
                                  tilt_ref=self._tilt_ref0), reset=False),
                    -pen, True, False,
                    {"termination_reason": bad, **parts}), None

        if self._ep_rand is not None and self._ep_rand.action_noise > 0:
            clipped = np.clip(
                clipped + self.rng.normal(0.0, self._ep_rand.action_noise,
                                          self.n_act), -1.0, 1.0)

        q_prop, q_ok, q_reason = self._act_to_q(clipped)
        q_safe, status = self.safety.filter(
            q_prop, self._state, ik_ok=q_ok, ik_reason=q_reason,
            action=clipped)

        terminated = bool(status.terminate)
        if not terminated:
            self._cmd = q_safe.copy()
            # DR: occasionally a SyncWrite is lost on the bus — the servos
            # keep chasing the previous goal for one tick.
            dropped = (self._ep_rand is not None
                       and self.rng.random() < self._ep_rand.cmd_drop_prob)
            if not dropped:
                # Zero-drift FRAME mode (dr.zero_drift_cmd_frame=1): a
                # drifted set_zero shifts reads AND commands together on
                # hardware — logical target C drives physical C - bias
                # (obs adds +bias, so the read converges back to C and
                # the loop is self-consistent; only physics sees the
                # offset). Legacy mode biased reads only, leaving a
                # cmd-vs-read residual hardware never shows.
                cmd_phys = q_safe
                if (self._ep_rand is not None
                        and self._ep_rand.zero_drift_cmd_frame):
                    cmd_phys = q_safe - self._ep_rand.joint_zero_bias_rad
                self._profile.command(
                    cmd_phys, speed_deg_s=self.write_speed_deg_s,
                    acc_units=self.write_acc_units)
        return None, (clipped, terminated, status, pen)

    def step(self, action):
        early, ctx = self._step_begin(action)
        if early is not None:
            return self._post_step(early)
        self._advance()
        return self._post_step(self._step_finish(ctx))

    def _post_step(self, result):
        """Subclass hook applied to EVERY completed step tuple (both the
        normal path and the rejected-action early return) — walk-mode
        shaping lives here so the batched vec env inherits it."""
        return result

    def _rise_ref_clock(self, ref: dict) -> tuple[int, bool]:
        """Reference tick for the CURRENT (post-step) state + is_rsi.

        RSI episodes joined the reference at the settled tick j0
        (_reset_finalize nearest-neighbor alignment); legacy episodes
        time-align at the ramp start. Clamped to the path end — the
        reference's final plant is the hold target thereafter. Shared
        by the rise-ref tracking reward and the BC-anchor target
        emission so the two can never disagree about the clock."""
        if self._rsi_ref_tick0 is not None:
            j = self._rsi_ref_tick0 + int(round(
                self._step_i * self.dt / ref["dt"]))
            is_rsi = True
        else:
            t_rel = (self._step_i - self._rise_ramp_i0) * self.dt
            j = ref["ramp_i0"] + int(round(t_rel / ref["dt"]))
            is_rsi = False
        return min(max(j, 0), len(ref["q"]) - 1), is_rsi

    def _step_finish(self, ctx):
        """Post-physics half of step: state read, reward, obs."""
        clipped, terminated, status, pen = ctx
        self._state = self._read_state()
        self._step_i += 1
        goal = self._current_goal()
        h_err = None
        h_rel = float(self.data.xpos[self._chassis_bid, 2]) - self._z0
        unload_f = None
        if goal is not None:
            h_err = h_rel - goal.height_ref
            if goal.unload_leg is not None:
                adr = self._touch_adr[int(goal.unload_leg)]
                if adr >= 0:
                    unload_f = float(self.data.sensordata[adr])
        # Quiet-stance gate: the reference is stationary when this tick's
        # refs match last tick's (holds, and the flat top of every ramp).
        ref_quiet = True
        if goal is not None and self._goal_traj is not None:
            prev_g = self._goal_traj.at(self._step_i - 1)
            ref_quiet = (
                abs(goal.roll_ref - prev_g.roll_ref) < 1e-9
                and abs(goal.pitch_ref - prev_g.pitch_ref) < 1e-9
                and abs(goal.height_ref - prev_g.height_ref) < 1e-9)
        reward, parts = compute_reward(self.cfg, self._state, clipped,
                                       self._prev_action, goal=goal,
                                       tilt_ref=self._tilt_ref0,
                                       height_err=h_err,
                                       unload_force_n=unload_f,
                                       ref_quiet=ref_quiet)
        # HOLD/TRACK stillness+feet pricing (2026-08-11, cfg
        # reward.hold_still_gate in [0,1], default 0 = legacy exact).
        # cw-stand-bc1-hard1's dig-in showed hold/track are not quiet
        # stands under the stand-line stack: the tracking kernel pays
        # torso attitude/height with NO opinion on the legs, so a policy
        # that cycles legs continuously (duty 0.85/0.09, 6-19 swings per
        # 15 s episode, feet ending 100-161 mm up) or parks two legs
        # splayed in the air collects near-full income; k_still is a
        # BONUS (default 0) and charges nothing. This gate scales the
        # kernel income on hold/track ticks by
        #   feet_factor * still_factor, where
        #   feet_factor = (fraction of feet down)^2 x HARD zero when any
        #     foot exceeds PLANT_SPEC.flag_leg_mm (60 mm: honest
        #     recovery/adjustment swings stay far below it, the observed
        #     splay sits at 100-160 mm),
        #   still_factor = Gaussian on mean qd^2 (still_sigma_rad_s),
        #     applied only while the reference is stationary so TRACK's
        #     commanded attitude motion is never charged.
        # Blend: f = (1-g) + g*feet*still. Scoped strictly to
        # hold/track (quad lifts legs on purpose, unload opens a
        # contact on purpose, rise/lower/raise have their own stacks).
        # HOLD bank in test_task_semantics.py pins the orderings.
        g_hold = float(cfg_get(self.cfg, "reward", "hold_still_gate",
                               default=0.0))
        if (g_hold > 0.0 and goal is not None
                and self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") in ("hold",
                                                             "track")
                and self._pad_z_ref is not None):
            clear_h = np.array(
                [float(self.data.xpos[b, 2]) - self._pad_z_ref[i]
                 for i, b in enumerate(self._pad_bids)])
            n_down_h = float(np.sum(
                clear_h <= PLANT_SPEC["foot_down_mm"] * 0.001))
            # No-flag factor: hard zero by default. cw-stand-holdstill1
            # (08-11) showed the hard zero is a zero-gradient plateau:
            # a leg parked at ~110 mm earns 0 income, but so does every
            # nearby behavior, so PPO gets no slope pointing the leg
            # back down and the park persists. reward.hold_flag_fade=1
            # swaps a linear fade over [flag_leg_mm, 2*flag_leg_mm]
            # (60->120 mm): compliant poses keep exactly 1.0, the
            # observed 110 mm park earns scraps WITH a downhill slope,
            # and the 190 mm class still earns 0.
            worst_h = float(np.max(clear_h))
            flag_m = PLANT_SPEC["flag_leg_mm"] * 0.001
            if float(cfg_get(self.cfg, "reward", "hold_flag_fade",
                             default=0.0)) > 0.0:
                noflag_h = min(max((2.0 * flag_m - worst_h) / flag_m,
                                   0.0), 1.0)
            else:
                noflag_h = 1.0 if worst_h <= flag_m else 0.0
            feet_h = (n_down_h / max(float(len(clear_h)), 1.0)) ** 2 \
                * noflag_h
            still_h = 1.0
            if ref_quiet:
                sig_qd_h = float(cfg_get(
                    self.cfg, "reward", "still_sigma_rad_s",
                    default=0.3))
                qd2_h = float(np.mean(np.square(
                    self._state.joint_velocity)))
                still_h = math.exp(-qd2_h / (2.0 * sig_qd_h ** 2))
            f_hold = (1.0 - g_hold) + g_hold * feet_h * still_h
            r_task_h = parts.get("reward_task", 0.0)
            if r_task_h > 0.0:
                reward += r_task_h * (f_hold - 1.0)
                parts["reward_task"] = r_task_h * f_hold
            parts["hold_feet_factor"] = feet_h
            parts["hold_still_factor"] = still_h
        # Rise decomposed into scored steps (rise/raise episodes only —
        # the ones with a real height target). Progress is potential-
        # based (telescoping: total = k * (start_err − end_err)) so it
        # steers exploration toward the target without changing what the
        # optimal policy is; freezing while the ref ramps away CHARGES.
        # Milestones pay once when the body first reaches a fraction of
        # the target — belly-off, half-way, nearly-there.
        if h_err is not None and abs(self._h_target) > 1e-3:
            kpg = float(cfg_get(self.cfg, "reward", "k_rise_progress",
                                default=100.0))
            kms = float(cfg_get(self.cfg, "reward", "k_rise_milestone",
                                default=2.0))
            e_abs = abs(h_err)
            r_prog = kpg * (self._prev_h_err_abs - e_abs)
            self._prev_h_err_abs = e_abs
            # Posture gate on rise/lower INCOME (2026-08-10, cfg
            # reward.rise_posture_gate in [0,1], default 0 = legacy
            # exact). The rfix pair (cw-uni-rfix-warm1/fresh1) showed
            # the height terms alone are gameable: fresh1 saturated the
            # in-training height criterion (rise "6/6") while the
            # posture-strict harness scored it 0/6 with a foot 273-313
            # mm in the air — torso-at-height via bridge/flail, not
            # standing. Height income (milestones, finish bonus, and
            # the post-ramp tracking kernel) is scaled by the fraction
            # of pads within end_posture_allow_m of their grounded z
            # (GEOMETRIC clearance, matching the eval harness's
            # end_posture_ok — NOT touch force: the champions' known
            # load concentration leaves grounded feet under 0.5 N, and
            # lightly-loaded is not the exploit; airborne is). "At
            # height, on your feet" pays full; "at height, feet
            # flying" earns ~(1-g). Progress and penalties are never
            # scaled (same construction as rise_income_prog_gate).
            g_pf = float(cfg_get(self.cfg, "reward",
                                 "rise_posture_gate", default=0.0))
            pf = 1.0
            if g_pf > 0.0 and self._pad_z_ref is not None:
                # Mode-correct allowance (bug found 2026-08-10,
                # cw-uni-rfix-postgate1 dig-in): the belly-ending lower
                # pose legitimately leaves pads 20-45 mm up (measured on
                # cw-uni-rfix-warm1's 6/6 harness-PASSING lowers:
                # 16.9-43.4 mm), so gating lower at the 20 mm stand
                # allowance made an HONEST lower earn pf 0.67-0.83 —
                # indistinguishable from the legs-aloft outrigger cheat
                # (pf 0.67), which then won on faster post-ramp income
                # and eroded warm1's clean lower to 0/12. Use the same
                # 60 mm lower allowance the harness end_posture_ok and
                # the reward_end_posture penalty already use
                # (self._h_target < 0 == lower episode).
                if self._h_target < 0.0:
                    allow_pf = float(cfg_get(
                        self.cfg, "reward", "end_posture_allow_lower_m",
                        default=0.06))
                else:
                    allow_pf = float(cfg_get(
                        self.cfg, "reward", "end_posture_allow_m",
                        default=0.02))
                n_on, n_tot = 0, 0
                for i in range(6):
                    if self._pad_bids[i] < 0:
                        continue
                    n_tot += 1
                    if (float(self.data.xpos[self._pad_bids[i], 2])
                            - self._pad_z_ref[i]) <= allow_pf:
                        n_on += 1
                if n_tot:
                    pf = (1.0 - g_pf) + g_pf * (n_on / n_tot)
                parts["rise_posture_factor"] = pf
            # Valid-plant geometric gate (operator spec 2026-08-10; cfg
            # reward.rise_plant_polygon_gate in [0,1], default 0 =
            # legacy exact). The clearance gate above counts feet near
            # the ground but is blind to WHERE they are: a stilt/splay
            # stand or a CoM-on-the-polygon-edge pose passes it. This
            # gate scales the same income terms by a continuous
            # PLANT_SPEC factor: CoM depth inside the down-feet support
            # polygon (full pay at the 20 mm spec margin), level
            # attitude (fades 10->20 deg), and body-frame footprint
            # near the walkable plant anchors (fades 40->80 mm — the
            # stilt family sits ~50 mm out). Rise only; lower ends on
            # the belly where a support polygon is meaningless.
            g_poly = float(cfg_get(self.cfg, "reward",
                                   "rise_plant_polygon_gate",
                                   default=0.0))
            # Stand-score income routing (operator, 08-10 evening, after
            # cw-stand-plantgate1 STOP): multiplicative gates LEAK — the
            # flag-leg cheat still collected ~60% of the height income
            # (5/6 feet down -> pf .83; footprint fade at ~50 mm -> .75)
            # and out-earned the harder honest stand, even warm-started
            # FROM the honest champion. Detection is not enough; the
            # income itself must come from standing correctly. Under
            # reward.rise_score_income=1 (RISE episodes only; lower keeps
            # its solved legacy stack) all height income is zeroed and
            # replaced by two terms on a single stand-score S — see the
            # block after the milestone code below.
            score_mode = (self._h_target > 0.0
                          and self._pad_z_ref is not None
                          and float(cfg_get(self.cfg, "reward",
                                            "rise_score_income",
                                            default=0.0)) == 1.0)
            clear = down = plant_f = None
            if ((g_poly > 0.0 or score_mode) and self._h_target > 0.0
                    and self._pad_z_ref is not None):
                clear = np.array(
                    [float(self.data.xpos[b, 2]) - self._pad_z_ref[i]
                     for i, b in enumerate(self._pad_bids)])
                down = clear <= PLANT_SPEC["foot_down_mm"] * 0.001
                feet_dn = np.array(
                    [self.data.xpos[b, :2] for b in self._pad_bids]
                )[down]
                margin_mm = (support_margin_m(
                    feet_dn, self.data.subtree_com[0, :2]) * 1000.0
                    if int(down.sum()) >= 3 else -1e9)
                margin_f = min(max(
                    margin_mm / PLANT_SPEC["com_margin_mm"], 0.0), 1.0)
                att_deg = max(abs(self._state.imu_roll),
                              abs(self._state.imu_pitch)) * RAD2DEG
                att_f = min(max((2.0 * PLANT_SPEC["attitude_deg"]
                                 - att_deg)
                                / PLANT_SPEC["attitude_deg"], 0.0), 1.0)
                fp_mm = self._curl_dist() * 1000.0
                fp_f = min(max((2.0 * PLANT_SPEC["footprint_err_mm"]
                                - fp_mm)
                               / PLANT_SPEC["footprint_err_mm"],
                               0.0), 1.0)
                plant_f = margin_f * att_f * fp_f
                if g_poly > 0.0:
                    pf *= (1.0 - g_poly) + g_poly * plant_f
                parts["rise_plant_factor"] = plant_f
            r_mile = 0.0
            for frac in (0.25, 0.50, 0.75, 0.90):
                # Fraction of the SIGNED target covered — works for rise
                # (positive) and lower (negative) alike.
                if (frac not in self._h_milestones
                        and h_rel / self._h_target >= frac):
                    self._h_milestones.add(frac)
                    r_mile += kms
            r_mile *= pf
            if score_mode:
                # Height progress + milestones are exactly the streams
                # that bankrolled every flag-leg/tripod cheat — zeroed
                # here; the stand-score below is the only rise income.
                r_prog, r_mile = 0.0, 0.0
            parts["reward_rise_progress"] = r_prog
            parts["reward_rise_milestone"] = r_mile
            reward += r_prog + r_mile
            if score_mode and clear is not None:
                # The tracking kernel pays torso-at-ref-height with no
                # posture opinion — the stream every cheat lived on.
                # Strip it for the whole rise episode (the curl-window
                # repricing below re-installs its own curl-priced kernel
                # during the pre-ramp hold, which is honest shaping).
                # 08-11: strip the NEGATIVE side too (opt-in,
                # reward.rise_score_strip_pen=1). The k_height=100
                # quadratic PENALTY was left live by the original strip
                # and it FUNDS the flag-leg cheat: lying honestly on
                # the belly under a +111mm command costs -1.2/tick
                # while torso-up-feet-flagged costs only the -0.5/tick
                # posture rent, so among behaviors a mediocre policy
                # can actually reach, the cheat is the paid optimum
                # (measured: rsi2 collapsed there even with correct
                # pool restore + RSI state coverage). With the penalty
                # stripped, height gradient comes only from score/ref
                # income — belly rest is free, the cheat pays pure
                # rent, honesty is the only positive slope.
                r_task = parts.get("reward_task", 0.0)
                strip_pen = float(cfg_get(self.cfg, "reward",
                                          "rise_score_strip_pen",
                                          default=0.0)) == 1.0
                if r_task > 0.0 or (strip_pen and r_task != 0.0):
                    reward -= r_task
                    parts["reward_task"] = 0.0
                # Stand-score S in [0,1]: height kernel x (feet-down
                # fraction)^2 x HARD no-flag x plant factor (attitude,
                # CoM-in-polygon, footprint at the walkable anchors).
                # Conjunction of the full PLANT_SPEC — anything scoring
                # high on all factors at once IS the stand. The no-flag
                # factor is a hard zero (not a fade): a flag-leg pose
                # earns nothing, not a 60% consolation.
                sig_s = float(cfg_get(self.cfg, "reward",
                                      "rise_score_sigma_mm",
                                      default=15.0)) * 0.001
                err_t = h_rel - self._h_target
                h_f = math.exp(-0.5 * (err_t / max(sig_s, 1e-6)) ** 2)
                n_down = float(down.sum()) / max(float(len(clear)), 1.0)
                noflag = (1.0 if float(np.max(clear))
                          <= PLANT_SPEC["flag_leg_mm"] * 0.001 else 0.0)
                p_now = n_down ** 2 * noflag * float(plant_f)
                s_now = h_f * p_now
                parts["rise_score"] = s_now
                # Exported for the ref-track block below: under score
                # mode ALL rise income is grounded-feet-only, including
                # the exploration crutch (bank, 08-10 late: at k=2 the
                # ref kernel alone re-funded flag-leg +419/ep — 15 of
                # 18 joints track the reference just fine with one leg
                # flagged).
                parts["rise_feet_factor"] = n_down ** 2 * noflag
                if self._score_best is None:
                    self._score_best = s_now
                ksp = float(cfg_get(self.cfg, "reward",
                                    "k_rise_score_prog", default=30.0))
                r_sp = ksp * max(0.0, s_now - self._score_best)
                self._score_best = max(self._score_best, s_now)
                parts["reward_rise_score_prog"] = r_sp
                reward += r_sp
                # Hold pay: only once the commanded ramp has arrived —
                # holding the true plant quietly is the only way to keep
                # earning. S^2 sharpens the top (5/6 feet at height with
                # perfect geometry otherwise caps at ~0.48).
                if goal is not None \
                        and goal.height_ref >= self._h_target - 1e-9:
                    ksh = float(cfg_get(self.cfg, "reward",
                                        "k_rise_score_hold",
                                        default=1.0))
                    r_sh = ksh * s_now ** 2
                    parts["reward_rise_score_hold"] = r_sh
                    reward += r_sh
                # Airborne-feet rent, ramp-weighted (bank finding,
                # 08-10: with the income fixed, the CHEATS won on the
                # penalty side — reward_height charges "torso not at
                # ref", so flag-leg DODGED ~120/ep of it by getting the
                # torso up on 5 legs while the honest partial rise paid
                # in full). Once the commanded ramp is up you owe rent
                # on FEET IN THE AIR (feet-down^2 x no-flag), exactly
                # as you already owe it on missing height. Deliberately
                # NOT the geometric plant factor: the honest reference
                # itself moves through wide-footprint poses mid-rise
                # (measured: charging plant_f rents the demonstration
                # ~100/ep and prices honest-but-parked below the
                # flag-leg cheat). Grounded-but-imperfect = unfinished,
                # charged via height + zero income; airborne = cheat.
                kpp = float(cfg_get(self.cfg, "reward",
                                    "k_rise_posture_pen", default=1.0))
                if kpp > 0.0 and goal is not None:
                    w = min(max(goal.height_ref / self._h_target,
                                0.0), 1.0)
                    r_pp = -kpp * w * (1.0 - n_down ** 2 * noflag)
                    parts["reward_rise_posture_pen"] = r_pp
                    reward += r_pp
            # Income prog-gate (2026-08-10 rise/lower freeze audit; cfg
            # reward.rise_income_prog_gate in [0,1], default 0 = legacy
            # exact). Measured: in lower episodes a robot that FREEZES at
            # the start height banks ~+74/ep (kernel + finish income
            # front-loaded during hold + early ramp) while every imperfect
            # attempt scores below it — a paid freeze plateau, the same
            # "worth less by construction" violation walk_kernel_prog_gate
            # closed for the park basin. Once the ramp has left zero,
            # multiply the INCOME terms (task kernel here, finish bonus
            # below) by the fraction of the signed target covered:
            # freeze earns ~(1-g), tracking earns full pay, penalties are
            # never scaled. The pre-ramp hold window is deliberately
            # ungated (holding IS the commanded behavior there).
            g_inc = float(cfg_get(self.cfg, "reward",
                                  "rise_income_prog_gate", default=0.0))
            inc_f = 1.0
            if g_inc > 0.0 and not score_mode and goal is not None \
                    and abs(goal.height_ref) > 1e-9:
                covered = min(max(h_rel / self._h_target, 0.0), 1.0)
                inc_f = (1.0 - g_inc) + g_inc * covered
                r_task = parts.get("reward_task", 0.0)
                if r_task > 0.0:
                    reward += r_task * (inc_f - 1.0)
                    parts["reward_task"] = r_task * inc_f
                parts["rise_income_factor"] = inc_f
            # Finish bonus (run 08): the tracking kernel is 20 mm wide,
            # so parking 20 mm below target still collects 61% of full
            # pay — run 07 drifted into exactly that discount (banked
            # the same curl as run 06 but stopped 43-62 mm short). Once
            # the ref has fully ramped to the target, a narrow second
            # kernel pays ONLY for actually arriving.
            # BUG (found 2026-08-10, cfg reward.rise_finish_gate_signed=1
            # to fix, default 0 = legacy exact): the legacy `ref >=
            # target` gate is correct for rise (ref climbs UP to the
            # target) but always-open for lower (negative target — the
            # ref starts ABOVE it), so the "arrival" kernel paid a robot
            # frozen at the START height through the hold + early ramp
            # (~+57 of the freeze plateau's +74). Signed mode requires
            # the ref to have reached the target from its own side.
            ramp_done = (goal is not None
                         and goal.height_ref >= self._h_target - 1e-9)
            if (goal is not None and self._h_target < 0.0
                    and float(cfg_get(
                        self.cfg, "reward", "rise_finish_gate_signed",
                        default=0.0)) == 1.0):
                ramp_done = goal.height_ref <= self._h_target + 1e-9
            if ramp_done and not score_mode:
                kfin = float(cfg_get(self.cfg, "reward", "k_rise_finish",
                                     default=1.0))
                sfin = float(cfg_get(
                    self.cfg, "reward", "rise_finish_sigma_mm",
                    default=8.0)) * 0.001
                r_fin = kfin * math.exp(
                    -0.5 * (h_err / max(sfin, 1e-6)) ** 2)
                if inc_f != 1.0:
                    r_fin *= inc_f
                r_fin *= pf
                parts["reward_rise_finish"] = r_fin
                reward += r_fin
                # Post-ramp tracking kernel is also height-only income
                # — a torso parked at the target with feet flying would
                # still collect ~1/tick for the rest of the episode.
                # Gate it by the same loaded-feet factor, but only once
                # the ramp is done: mid-rise transients stay untaxed.
                if pf != 1.0:
                    r_task = parts.get("reward_task", 0.0)
                    if r_task > 0.0:
                        reward += r_task * (pf - 1.0)
                        parts["reward_task"] = r_task * pf
        # Curl scores (rise only): pay pulling the feet in toward the
        # plant footprint. Potential-based, so crouch starts (dist ~0)
        # and foot-parking exploits earn nothing net.
        if self._is_rise:
            kcp = float(cfg_get(self.cfg, "reward", "k_curl_progress",
                                default=50.0))
            kms = float(cfg_get(self.cfg, "reward", "k_rise_milestone",
                                default=2.0))
            th_mm = cfg_get(self.cfg, "reward", "curl_milestone_mm",
                            default=[40.0, 15.0])
            dist = self._curl_dist()
            r_cprog = kcp * (self._curl_dist_prev - dist)
            self._curl_dist_prev = dist
            r_cmile = 0.0
            for th in th_mm:
                th = float(th)
                if th not in self._curl_milestones and dist <= th * 0.001:
                    self._curl_milestones.add(th)
                    r_cmile += kms
            parts["reward_curl_progress"] = r_cprog
            parts["reward_curl_milestone"] = r_cmile
            reward += r_cprog + r_cmile
            # Hold-phase repricing (run 06): while the height ref still
            # sits at 0 (the curl window), the tracking kernel pays for
            # CURL DISTANCE, not stillness. Before this, lying frozen
            # and level earned ~1/tick from the tilt/height kernel while
            # the entire curl bonus summed to ~2.5 — so preparation was
            # priced as a loss and the policy (correctly, by that math)
            # pinned curl negative through runs 03-05. Crouch starts
            # have dist ~0 and earn full pay unchanged.
            if goal is not None and goal.height_ref <= 1e-4:
                sig_c = float(cfg_get(
                    self.cfg, "reward", "rise_hold_curl_sigma_mm",
                    default=20.0)) * 0.001
                k_tr = float(cfg_get(self.cfg, "reward", "k_track",
                                     default=1.0))
                curl_kernel = k_tr * math.exp(
                    -0.5 * (dist / max(sig_c, 1e-6)) ** 2)
                reward += curl_kernel - parts.get("reward_task", 0.0)
                parts["reward_task"] = curl_kernel
                # Reprice the quiet-stance bonus with the swapped kernel
                # too: the plain kernel is ~1 lying level on the belly,
                # which would pay k_still for frozen belly-rest — the
                # exact freeze shortcut this branch exists to prevent.
                if parts.get("reward_still", 0.0) != 0.0:
                    ksl = float(cfg_get(self.cfg, "reward", "k_still",
                                        default=0.0))
                    r_still = (ksl * (curl_kernel / max(k_tr, 1e-9))
                               * parts.get("still_factor", 0.0))
                    reward += r_still - parts["reward_still"]
                    parts["reward_still"] = r_still
        # Rise-reference tracking (default OFF; operator 08-10, the
        # Stage-II route from the stand-up literature — HumanUP / HoST:
        # discover the motion once, then train the deployable policy to
        # TRACK it instead of rediscovering from a height reward). Our
        # "discovery stage" already exists: the stance champion's
        # learned belly-rise. This term pays a joint-space kernel on
        # RMS error against that recorded trajectory, time-aligned at
        # the RAMP START tick (episodes with jittered holds and
        # crouch/bridge starts all join the same reference — pre-ramp
        # ticks track the reference's own curl phase, clamped at its
        # start). A scaffold, not the objective: run it at full weight
        # to seed the skill, then anneal k to 0 across warm-started
        # arms so the final policy is not trajectory-locked.
        # Enable: --cfg-set reward.k_rise_ref_track=<k>
        #         --cfg-set reward.rise_ref_path=<npz>.
        k_ref = float(cfg_get(self.cfg, "reward", "k_rise_ref_track",
                              default=0.0))
        if k_ref > 0.0 and self._is_rise:
            ref_path = cfg_get(self.cfg, "reward", "rise_ref_path",
                               default=None)
            if ref_path:
                ref = load_rise_ref(str(ref_path))
                j, _is_rsi = self._rise_ref_clock(ref)
                parts["rise_rsi"] = 1.0 if _is_rsi else 0.0
                err = self.data.qpos[self._qadr] - ref["q"][j]
                sig = float(cfg_get(
                    self.cfg, "reward", "rise_ref_sigma_deg",
                    default=12.0)) * DEG2RAD
                rms = float(np.sqrt(np.mean(err ** 2)))
                r_ref = k_ref * math.exp(-0.5 * (rms / max(sig, 1e-6)) ** 2)
                # Score-income mode: the crutch is income too, and
                # income only pays on grounded feet (see the
                # rise_feet_factor export above). Legacy stacks
                # (factor absent) are exactly unchanged.
                r_ref *= parts.get("rise_feet_factor", 1.0)
                parts["reward_rise_ref"] = r_ref
                reward += r_ref
        # Per-servo hot-current penalty (archive/RL_PLAN_NEXT.md §4, default OFF).
        # The aggregate current penalty lets the policy park all load on
        # one knee; visual eval of the cw champions found tripod stances
        # with one servo above 1.5 A for most of the episode. Charge
        # concentration directly: quadratic above a soft per-servo
        # threshold, so 6 legs at 0.4 A cost nothing while one at 1.8 A
        # hurts. Enable with --cfg-set reward.k_current_hot=<k>.
        k_hot = float(cfg_get(self.cfg, "reward", "k_current_hot",
                              default=0.0))
        if k_hot > 0.0 and self._state.servo_current is not None:
            hot_a = float(cfg_get(self.cfg, "reward", "current_hot_a",
                                  default=1.0))
            over = np.maximum(self._state.servo_current - hot_a, 0.0)
            r_hot = -k_hot * float(np.sum(over ** 2))
            parts["reward_current_hot"] = r_hot
            reward += r_hot
        # --- First-principles posture terms (operator 08-08 ~20:45Z,
        # default OFF). WHY a waving leg is bad: smaller support polygon
        # (tips), load concentration (hot knees), wasted hold torque.
        # Static-hold pricing was measured CORRECT (hip 0.149 Nm -> 0.179 A,
        # actuator carries full gravity torque; diagnosis 08-08 cycle 12):
        # the defect is that the LINEAR current charge is invariant to load
        # distribution, so concentration is free. These two GLOBAL terms
        # price the physics directly, in every mode (declared routing:
        # GLOBAL - they encode "don't tip / don't concentrate heat", not
        # gait morphology). The unload target leg is skipped like the
        # other stance terms.
        k_margin = float(cfg_get(self.cfg, "reward", "k_support_margin",
                                 default=0.0))
        k_even = float(cfg_get(self.cfg, "reward", "k_load_even",
                               default=0.0))
        if (k_margin > 0.0 or k_even > 0.0):
            skip = int(goal.unload_leg) if (
                goal is not None and goal.unload_leg is not None) else -1
            forces, feet_xy = [], []
            for i in range(6):
                if i == skip or self._touch_adr[i] < 0:
                    continue
                f = max(float(self.data.sensordata[self._touch_adr[i]]), 0.0)
                forces.append(f)
                if f > 0.5 and self._pad_bids[i] >= 0:
                    feet_xy.append(self.data.xpos[self._pad_bids[i], :2])
            if k_margin > 0.0 and len(feet_xy) >= 3:
                # Reward CoM depth inside the support polygon, saturating
                # at 40 mm: centered stances earn the cap, near-edge or
                # outside-CoM poses earn ~0/negative. Belly rest (<3 foot
                # contacts, chassis supported) is exempt by the gate.
                m = support_margin_m(np.asarray(feet_xy),
                                     self.data.subtree_com[0, :2])
                r_margin = k_margin * float(np.clip(m, -0.04, 0.04)) / 0.04
                parts["reward_support_margin"] = r_margin
                reward += r_margin
            ftot = float(np.sum(forces))
            if k_even > 0.0 and ftot > 1.0:
                # Load concentration: Herfindahl index of foot normal
                # forces. Even 6-leg load = 1/6 (charge 0); everything on
                # one foot = 1 (max charge). Dense, mode-independent.
                fr = np.asarray(forces) / ftot
                hhi = float(np.sum(fr ** 2))
                r_even = -k_even * (hhi - 1.0 / len(forces))
                parts["reward_load_even"] = r_even
                reward += r_even
        # Stance-contact shaping (default OFF): during stance modes the
        # kernel is blind to how many feet carry the body, so a 3-leg
        # tripod scores like a 6-leg stance (and cooks servos). Pay a
        # small bonus per loaded foot; for unload episodes the target
        # leg is excluded (it is SUPPOSED to be in the air).
        k_stance = float(cfg_get(self.cfg, "reward", "k_stance_contact",
                                 default=0.0))
        mode_now = self._goal_traj.mode if self._goal_traj else ""
        if k_stance > 0.0 and mode_now in ("hold", "lean", "track",
                                           "unload", "raise"):
            skip = int(goal.unload_leg) if (
                goal is not None and goal.unload_leg is not None) else -1
            feet = [i for i in range(6) if i != skip]
            n_on = sum(1 for i in feet
                       if self._touch_adr[i] >= 0
                       and float(self.data.sensordata[self._touch_adr[i]])
                       > 0.5)
            r_stance = k_stance * n_on / len(feet)
            parts["reward_stance"] = r_stance
            reward += r_stance
        # Stance-clearance penalty (default OFF): the contact bonus above
        # failed to break the learned tripod in cw-stance-even — a foot
        # held in the air earns nothing for moving DOWN until it actually
        # touches, so PPO never feels a gradient toward ground. Charging
        # for height above the episode-start (grounded) pad z is dense:
        # every millimeter a hovering foot descends pays immediately.
        # "raise" is exempt: cw-stance-clear collapsed raise to 0/6
        # (parked 13-17 mm short) while hold/rise/lower stayed perfect —
        # lifting the body requires transient foot repositioning that a
        # z-referenced clearance charge punishes.
        k_clear = float(cfg_get(self.cfg, "reward", "k_stance_clearance",
                                default=0.0))
        if k_clear > 0.0 and self._pad_z_ref is not None \
                and mode_now in ("hold", "lean", "track", "unload"):
            skip = int(goal.unload_leg) if (
                goal is not None and goal.unload_leg is not None) else -1
            clear = 0.0
            for i in range(6):
                if i == skip or self._pad_bids[i] < 0:
                    continue
                clear += max(float(self.data.xpos[self._pad_bids[i], 2])
                             - self._pad_z_ref[i], 0.0)
            r_clear = -k_clear * clear
            parts["reward_clearance"] = r_clear
            reward += r_clear
        # Flag-leg penalty (default OFF): the 08-08 video review found
        # every walk-lineage policy (and the stance line's lower endings)
        # parking one leg straight up in the air — modes exempt from the
        # stance-clearance penalty (walk/rise/lower/raise) have no
        # gradient against it. Charge only clearance ABOVE a generous
        # allowance (default 50 mm over the episode-start pad z), so
        # normal swing (~10-20 mm) and rise/lower repositioning stay
        # free while a vertical flag leg (~150 mm) pays every step.
        # Default: every mode; the unload target leg is skipped.
        # cw-walk-flag (08-08) refuted the all-modes routing: rise needs
        # >50 mm transient swings from belly starts, and the global
        # charge collapsed rise/raise while only making the walk flag
        # leg transient. reward.flag_leg_walk_only=1 routes the charge
        # to walk mode alone (declared routing per RL_PLAN.md).
        k_flag = float(cfg_get(self.cfg, "reward", "k_flag_leg",
                               default=0.0))
        if k_flag > 0.0 and float(cfg_get(
                self.cfg, "reward", "flag_leg_walk_only",
                default=0.0)) > 0.0 and mode_now != "walk":
            k_flag = 0.0
        if k_flag > 0.0 and self._pad_z_ref is not None:
            allow = float(cfg_get(self.cfg, "reward", "flag_leg_allow_m",
                                  default=0.05))
            skip = int(goal.unload_leg) if (
                goal is not None and goal.unload_leg is not None) else -1
            over = 0.0
            for i in range(6):
                if i == skip or self._pad_bids[i] < 0:
                    continue
                over += max(float(self.data.xpos[self._pad_bids[i], 2])
                            - self._pad_z_ref[i] - allow, 0.0)
            r_flag = -k_flag * over
            parts["reward_flag_leg"] = r_flag
            reward += r_flag
        # Terminal end-posture pricing (default OFF; cycle 14). Root
        # cause chain: flag-leg endings <- airborne legs are free at
        # episode end <- load_even/support_margin have ZERO gradient on
        # an unloaded airborne leg, stance_clearance excludes
        # rise/lower/raise (their transients need freedom), and the
        # all-modes flag_leg charge was refuted for taxing exactly those
        # transients <- the deepest link (current-model dead zone
        # underpricing static holds) needs hardware recalibration, not
        # reachable in sim alone. This term charges per-foot clearance
        # above the grounded pad reference ONLY AFTER the goal height
        # reference has settled to its final value (plus a grace
        # window): the charge window is SCHEDULE-based, so the policy
        # cannot dodge it by avoiding the target, and the motion phase
        # is untaxed. Routed to the modes stance_clearance excludes.
        # Enable: --cfg-set reward.k_end_posture=<k>.
        k_endp = float(cfg_get(self.cfg, "reward", "k_end_posture",
                               default=0.0))
        if k_endp > 0.0 and self._pad_z_ref is not None \
                and self._goal_traj is not None \
                and mode_now in ("rise", "lower", "raise"):
            if self._end_posture_from is None:
                # The lower/rise ramps run to the last scheduled step
                # (no settled plateau exists), so "terminal" means: the
                # height REFERENCE is within end_posture_ref_mm of its
                # final value from here to the end — still a pure
                # function of the pre-sampled schedule.
                h = np.asarray(self._goal_traj.height)
                ref_m = float(cfg_get(
                    self.cfg, "reward", "end_posture_ref_mm",
                    default=15.0)) * 0.001
                far = np.nonzero(np.abs(h - h[-1]) > ref_m)[0]
                start = (int(far[-1]) + 1) if len(far) else 0
                grace_s = float(cfg_get(
                    self.cfg, "reward", "end_posture_grace_s",
                    default=0.25))
                # Also clamp to the last end_posture_window_s of the
                # episode: small-amplitude rise refs sit near final
                # almost immediately, and charging the early curl
                # transient is the exact mistake that refuted the
                # all-modes flag_leg charge.
                win_s = float(cfg_get(
                    self.cfg, "reward", "end_posture_window_s",
                    default=1.5))
                self._end_posture_from = max(
                    start + int(round(grace_s / self.dt)),
                    self.episode_steps - int(round(win_s / self.dt)))
                # Dense variant (cycle 25, lower only): a proper lower
                # keeps all six feet planted THROUGHOUT the descent —
                # there is no legitimate leg-lift transient to protect
                # (the transient exemption exists for rise curls). With
                # this flag the clearance charge covers the whole lower
                # episode, pricing the spear-leg tilt-guard where it is
                # used instead of only at the end. Same term, same k,
                # same per-tick magnitude — only the window changes.
                # Enable: --cfg-set reward.end_posture_lower_dense=1.
                if mode_now == "lower" and float(cfg_get(
                        self.cfg, "reward", "end_posture_lower_dense",
                        default=0.0)) > 0.0:
                    self._end_posture_from = 0
            if self._step_i >= self._end_posture_from:
                # Mirror the eval gate's allowances: 20 mm for
                # stand-ending modes, 60 mm for belly-ending lower.
                allow = float(cfg_get(
                    self.cfg, "reward",
                    "end_posture_allow_lower_m", default=0.06)) \
                    if mode_now == "lower" else float(cfg_get(
                        self.cfg, "reward", "end_posture_allow_m",
                        default=0.02))
                skip = int(goal.unload_leg) if (
                    goal is not None and goal.unload_leg is not None) \
                    else -1
                over_e = 0.0
                for i in range(6):
                    if i == skip or self._pad_bids[i] < 0:
                        continue
                    c = (float(self.data.xpos[self._pad_bids[i], 2])
                         - self._pad_z_ref[i] - allow)
                    over_e += min(max(c, 0.0), 0.30)
                r_endp = -k_endp * over_e
                parts["reward_end_posture"] = r_endp
                reward += r_endp
        if terminated:
            parts["reward_termination"] = -pen
            reward -= pen
        truncated = self._step_i >= self.episode_steps
        self._prev_action = clipped.copy()
        info = {"termination_reason": status.reason, **parts,
                "safety_ok": status.ok,
                "roll_deg": self._state.imu_roll * RAD2DEG,
                "pitch_deg": self._state.imu_pitch * RAD2DEG,
                "roll_rel_deg": (self._state.imu_roll
                                 - self._tilt_ref0[0]) * RAD2DEG}
        # BC-anchor target (rl_move/sim/bc_anchor.py; RL_PLAN queue 2a,
        # operator 08-11 — the rise lever AFTER all reward-side levers
        # closed). For every rise tick with a live reference clock,
        # emit the normalized action whose joint target is the
        # reference pose one ref-tick ahead; the trainer's aux loss
        # pulls pi_mean(obs) toward it. NOT a reward term — reward
        # above is untouched and the rise semantics bank is unaffected.
        # Emitted only when the trainer asked (train.bc_anchor_coef
        # rides into the env cfg, same pattern as mirror_loss_coef)
        # and only on raw-18-joint tasks (the inverse action map is
        # joint_task's per-axis affine). Uses only per-episode attrs
        # already in mjx_host.SNAP_ATTRS (_rsi_ref_tick0,
        # _rise_ramp_i0, _step_i) — pool-restore safe by construction.
        if (self._is_rise and getattr(self, "n_act", 0) == N_JOINTS
                and float(cfg_get(self.cfg, "train", "bc_anchor_coef",
                                  default=0.0)) > 0.0):
            _bc_ref_path = cfg_get(self.cfg, "reward", "rise_ref_path",
                                   default=None)
            if _bc_ref_path:
                from .joint_task import q_rad_to_action
                _bc_ref = load_rise_ref(str(_bc_ref_path))
                _bc_j, _ = self._rise_ref_clock(_bc_ref)
                _bc_jn = min(
                    _bc_j + max(int(round(self.dt / _bc_ref["dt"])), 1),
                    len(_bc_ref["q"]) - 1)
                info["bc_target"] = q_rad_to_action(
                    _bc_ref["q"][_bc_jn]).astype(np.float32)
        if self._state.servo_current is not None:
            info["mean_current_a"] = float(
                np.mean(np.abs(self._state.servo_current)))
            info["max_current_a"] = float(
                np.max(np.abs(self._state.servo_current)))
        if goal is not None:
            info["goal_mode"] = self._goal_traj.mode
            info["roll_ref_deg"] = goal.roll_ref * RAD2DEG
            info["pitch_ref_deg"] = goal.pitch_ref * RAD2DEG
            info["track_err_deg"] = math.hypot(
                self._state.imu_roll - self._tilt_ref0[0] - goal.roll_ref,
                self._state.imu_pitch - self._tilt_ref0[1] - goal.pitch_ref
            ) * RAD2DEG
            info["height_mm"] = h_rel * 1000.0
            info["height_ref_mm"] = goal.height_ref * 1000.0
            info["height_err_mm"] = h_err * 1000.0
            if unload_f is not None:
                info["unload_force_n"] = unload_f
        return (self._final_obs(
                    build_obs(self.cfg, self._state, self._q_nom,
                              self._prev_action, goal=goal,
                              tilt_ref=self._tilt_ref0), reset=False),
                float(reward), terminated, truncated, info)

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        mujoco = self._mujoco
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)
            self._cam = mujoco.MjvCamera()
            self._cam.distance = 0.7
            self._cam.elevation = -25.0
            self._cam.azimuth = 130.0
        self._cam.lookat[:] = self.data.xpos[self._chassis_bid]
        self._renderer.update_scene(self.data, camera=self._cam)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            # mujoco.Renderer only gained close() in 3.x; fall back to GC.
            if hasattr(self._renderer, "close"):
                self._renderer.close()
            self._renderer = None


def make_env(**kwargs):
    """Factory for SB3 ``make_vec_env``."""
    def _thunk():
        return SimHexapodBalanceEnv(**kwargs)
    return _thunk


if __name__ == "__main__":
    env = SimHexapodBalanceEnv(randomize=True, seed=0)
    obs, info = env.reset()
    print(f"obs {obs.shape} roll={info['roll_deg']:+.2f}° "
          f"pitch={info['pitch_deg']:+.2f}°")
    t0 = time.monotonic()
    ret = 0.0
    for i in range(env.episode_steps):
        obs, r, term, trunc, info = env.step(np.zeros(N_ACT))
        ret += r
        if term or trunc:
            break
    print(f"zero-action episode: steps={i + 1} return={ret:.3f} "
          f"roll={info['roll_deg']:+.2f}° pitch={info['pitch_deg']:+.2f}° "
          f"({time.monotonic() - t0:.2f}s wall)")
