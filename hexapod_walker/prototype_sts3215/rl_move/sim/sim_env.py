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
    joint_qpos_addrs, joint_qvel_addrs, lowest_collidable_z,
    position_actuator_ids, resolve_model_source,
)
from .struct_compliance import StructCompliance  # noqa: E402

G0 = 9.80665
N_OBS = 47

# QUADWALK "quadstance" spawn (08-13, quad track): per-lift-leg
# (yaw, hip, knee) rad — the "tuck" claw from
# quadruped_feasibility.FRONT_POSES (kept literal here so env workers
# don't import that mujoco-loading probe module; c57 static sweep GO,
# within joint limits yaw ±0.61 / hip −1.40..0.52 / knee −0.35..2.62).
_QUAD_TUCK_RAD = (0.0, -1.10, 2.40)

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


def leg_chassis_collision_from_cfg(cfg) -> bool:
    """cfg ``env.leg_chassis_collision`` (0 = off, the default) — the
    belly knife-edge contact axis (SIM.md known-gap 4, added 08-12).
    The masks must be rewritten in the XML BEFORE compile (MuJoCo
    precomputes the collidable pair set; runtime contype/conaffinity
    edits never register — verified 08-12 on 3.11), so this is a
    ``build_model(leg_chassis_collision=...)`` kwarg, not a model
    mutation. See servo_model.build_model for the bit plan."""
    if cfg is None:
        from rl_move.config import load_config
        cfg = load_config()
    return bool(int(cfg_get(cfg, "env", "leg_chassis_collision",
                            default=0)))


def _default_plant_deg() -> np.ndarray:
    """Standing plant in SIM joint convention (knee relative to femur).

    A genuinely CAPTURED plant (plant_pose.json) is authoritative: it is
    stored in the measured robot's absolute-tibia convention since
    30660b51 and sim_gait_compat converts it at the boundary. The
    hardware DEFAULT stand home (+19/+28 absolute = +19/+9 relative) is
    deliberately NOT adopted: it sits on the leg-extension boundary
    (hip->foot ~239.9 of 240 mm), where the fixed-foot body IK is
    singular, and the sim's canonical training/eval stance has always
    been +20/+80 relative."""
    try:
        from feetech_bus import load_plant_pose
        if load_plant_pose().get("learned"):
            from sim_gait_compat import standing_pose_degrees
            return np.asarray(standing_pose_degrees(), dtype=float)
    except Exception:
        pass
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

        # In-run coefficient scheduler (2026-08-13, nobc gait line —
        # GAIT.md P3 lever 2 "annealed-up charge", the last unbuilt
        # lever after every fixed-coefficient / warm-start form closed).
        # Linearly ramps ONE cfg coefficient DURING a training run, by
        # GLOBAL env steps. Default OFF (sched.key unset) = bit-exact
        # legacy behavior: nothing is tracked, no cfg value is written.
        #   sched.key       dotted cfg path to drive, e.g.
        #                   "reward.k_drag_stance"
        #   sched.v0 / v1   value before t0 / after t1 (linear between)
        #   sched.t0_steps / t1_steps   GLOBAL env-step boundaries
        #   sched.n_envs    total parallel envs in the run — each env
        #                   converts its own tick count to global steps
        #                   as ticks * n_envs (exact for the synchronous
        #                   vec envs, which step every env every batch
        #                   tick). REQUIRED — no silent default, a
        #                   mis-clocked schedule is worse than a crash.
        # The clock is a per-process monotone tick counter. It is NOT
        # in mjx_host.SNAP_ATTRS on purpose: episode pool-restores must
        # never rewind it (the commit-65edba7 bug class). It restarts
        # at 0 on resume-from-checkpoint — scheduled runs should be
        # fresh single-process runs, note it in the spec. Eval-harness
        # envs built from the same cfg sit at tick ~0 and so read ~v0
        # for the scheduled key: judge scheduled runs on measured
        # behavior metrics (slip/gait/travel), not eval reward panels.
        self._sched_key = str(cfg_get(self.cfg, "sched", "key",
                                      default="") or "")
        self._sched_ticks = 0
        self._sched_value: float | None = None
        if self._sched_key:
            self._sched_path = tuple(self._sched_key.split("."))
            if len(self._sched_path) < 2:
                raise ValueError(
                    "sched.key must be a dotted cfg path "
                    f"(section.leaf), got {self._sched_key!r}")

            def _sched_req(leaf: str) -> float:
                v = cfg_get(self.cfg, "sched", leaf, default=None)
                if v is None:
                    raise ValueError(
                        f"sched.key is set but sched.{leaf} is missing "
                        "— the scheduler has no silent defaults")
                return float(v)

            self._sched_v0 = _sched_req("v0")
            self._sched_v1 = _sched_req("v1")
            self._sched_t0 = _sched_req("t0_steps")
            self._sched_t1 = _sched_req("t1_steps")
            self._sched_n = _sched_req("n_envs")
            if not (self._sched_t1 > self._sched_t0 >= 0.0):
                raise ValueError(
                    "sched requires t1_steps > t0_steps >= 0")
            if self._sched_n < 1.0:
                raise ValueError("sched.n_envs must be >= 1")

        # Physics easing (2026-08-13, GAIT.md P3 lever 3, nobc track):
        # ease.gravity_scale / ease.vel_ceiling_scale multiply THIS
        # EPISODE's gravity magnitude and servo velocity ceiling. Both
        # are read from cfg at EVERY reset (see _reset_begin) so the
        # sched.* engine above — which writes its target cfg path each
        # tick — can anneal them across a run (eased physics early,
        # nominal by the end); within an episode physics never changes.
        # Default (keys unset / 1.0) is bit-exact legacy: no draw, no
        # mutation, no extra code path. Application point is the
        # episode's DR draw (_ep_rand) — the one object BOTH trainer
        # stacks consume (private model: EpisodeRand.apply_to_model;
        # batched MJX: ModelDrScratch.rows_for + tp_rows) — so easing
        # composes with DR (slope direction kept, |g| scaled) with NO
        # DomainRandomizer or per-world plumbing changes. These two
        # fields hold the randomize=False PRIVATE-model fallback used
        # by reset(); shared-model shims without DR raise instead
        # (per-world model fields are the only route to eased gravity
        # in the batched path).
        self._ease_g = 1.0
        self._ease_v = 1.0

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

        # Servo-profile RAMP-IN (2026-08-20, fast anti-skate option (b),
        # q_20260820T0830Z: the bcgait1_hard1 transplant dies zero-shot
        # under the raised 1500/80 profile at the V5 B0 precert, dose-
        # graded — the PROFILE DOSE itself destabilizes the walker
        # before any curriculum/penalty engages). When armed, the
        # TRAINER anneals the commanded write profile from a gentle
        # start (default = the fitted regime: 350 counts/s effective
        # cruise, acc 20, 1.5 deg/tick slew) to the cfg TARGET
        # (bus.write_speed / bus.write_acc / safety.max_delta_q_deg)
        # linearly over ``bus.profile_ramp_steps`` GLOBAL env steps.
        #   - Default (key absent/0) = OFF: no state, no new code path,
        #     bit-exact legacy behavior.
        #   - Armed but never applied = TARGET profile: construction
        #     never moves the dials, so eval_checkpoint / play / the
        #     periodic C-env evals judge checkpoints at the FULL dose
        #     even when the training cfg carries ramp keys. Only an
        #     explicit apply_profile_ramp_frac() call (train_ppo_mjx:
        #     frac 0 before the pre-PPO cert, then per rollout) moves
        #     the profile below target.
        #   - Fail-closed: a ramp whose target write_speed exceeds the
        #     resolved actuator velocity ceiling would be silently
        #     clamped (the exact silent-no-op class the 08-19
        #     servo_vel_max_counts_s override exists for) — raise at
        #     construction instead.
        self._profile_ramp: dict | None = None
        self._profile_ramp_dq_rad: float | None = None
        _ramp_steps = int(float(cfg_get(
            self.cfg, "bus", "profile_ramp_steps", default=0) or 0))
        if _ramp_steps > 0:
            _r_start_ws = float(cfg_get(
                self.cfg, "bus", "profile_ramp_start_write_speed",
                default=350.0))
            _r_start_acc = float(cfg_get(
                self.cfg, "bus", "profile_ramp_start_write_acc",
                default=20.0))
            _r_start_dq = float(cfg_get(
                self.cfg, "bus", "profile_ramp_start_max_delta_q_deg",
                default=1.5))
            _r_tgt_ws = float(cfg_get(self.cfg, "bus", "write_speed",
                                      default=400))
            _r_tgt_acc = float(cfg_get(self.cfg, "bus", "write_acc",
                                       default=20))
            _r_tgt_dq = float(cfg_get(self.cfg, "safety",
                                      "max_delta_q_deg", default=2.0))
            if min(_r_start_ws, _r_start_acc, _r_start_dq) <= 0.0:
                raise ValueError(
                    "bus.profile_ramp_start_* must all be > 0 (got "
                    f"write_speed={_r_start_ws}, acc={_r_start_acc}, "
                    f"max_delta_q_deg={_r_start_dq})")
            _ceil_counts = (float(self.params.per_joint(
                "vel_max_deg_s").min()) * 4096.0 / 360.0)
            if _r_tgt_ws > _ceil_counts + 1e-6:
                raise ValueError(
                    f"bus.profile_ramp_steps={_ramp_steps} targets "
                    f"write_speed={_r_tgt_ws:g} counts/s but the "
                    "resolved actuator velocity ceiling is "
                    f"{_ceil_counts:.0f} counts/s — the ramp would be "
                    "silently clamped; set bus.servo_vel_max_counts_s "
                    "(e.g. 'write_speed') so the profile ceiling "
                    "matches the target dose")
            self._profile_ramp = {
                "steps": _ramp_steps, "frac": 1.0,
                "start": (_r_start_ws, _r_start_acc, _r_start_dq),
                "target": (_r_tgt_ws, _r_tgt_acc, _r_tgt_dq),
            }

        # ``model``: a pre-built, fully PREPARED (contact-softened) MjModel
        # shared with other envs — the batched MJX vec env owns physics
        # and passes one model to all its per-env shims. A shared model
        # must never be mutated per episode, so the shim path runs with
        # model DR disabled. Default (None): private model, as always.
        self._owns_model = model is None
        # cfg env.model_source: mesh-family (corrected kinematics) or the
        # legacy primitive model — see servo_model.resolve_model_source.
        # Shared models arrive pre-built from the same cfg, so the resolved
        # source still describes them.
        self._model_source = resolve_model_source(self.cfg)
        if model is not None:
            # dr.walk_push_*: private-model envs apply the xfrc in
            # their own _advance loop; shared-model shims delegate to
            # the MJX stepper, whose tick takes the per-env push_nm
            # (the vec envs read _walk_push_torque_nm() per tick and
            # hand it over — plumbed 08-12 in mjx_backend/mjx_vec_env/
            # mjx_sharded_vec_env).
            self.model = model
        else:
            # Rough terrain (cfg env.terrain_amp > 0) reaches the private
            # C-env model here, so eval-harness / local-viewer episodes run
            # on the same ground the batched trainer used.
            _t_amp = float(cfg_get(self.cfg, "env", "terrain_amp",
                                   default=0.0))
            _t_seed = int(cfg_get(self.cfg, "env", "terrain_seed",
                                  default=0))
            self.model = build_model(
                fixed_base=False,
                flat_terrain=_t_amp <= 0.0,
                terrain_amp=_t_amp,
                terrain_seed=_t_seed,
                mesh_visuals=mesh_visuals,
                leg_chassis_collision=leg_chassis_collision_from_cfg(
                    self.cfg),
                source=self._model_source)
        self.data = mujoco.MjData(self.model)
        self._substeps = max(1, int(round(self.dt / self.model.opt.timestep)))
        self._qadr = joint_qpos_addrs(self.model)
        self._vadr = joint_qvel_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)
        self._struct_comp = StructCompliance.from_cfg(self.cfg)
        self._struct_comp_k: np.ndarray | None = None
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
        # Mode-sequencing state (goal.mode_seq) — populated per episode
        # in _reset_begin/_sample_mode_seq; None = feature off.
        self._seq_plan = None
        self._seq_idx = 0
        self._seq_stand_z = None
        self._seq_seg_end = None
        self._seq_pose_anchor = None
        # Canonical per-family segment frames (goal.mode_seq): the
        # settled plant / belly reference frames a FRESH episode of each
        # segment family would derive at reset (q_nom, _z0, pad-z ref).
        # Captured by a settle probe inside reset() (see
        # _seq_capture_frames) and installed at every mid-episode
        # switch — the trans-dagger2 kill (08-14) proved that carrying
        # the episode-reset q_nom across switches puts every later
        # segment's obs frame up to ~79 deg (knee, belly-vs-plant) off
        # the teachers' training distribution.
        self._seq_frames: dict | None = None
        self._imu_prev_v: np.ndarray | None = None
        self._imu_f_accum = np.zeros(3)
        self._imu_f_n = 0
        self._gyro_accum = np.zeros(3)
        self._gyro_n = 0
        self._att_rp: np.ndarray | None = None
        self._tilt_ref0 = (0.0, 0.0)
        self._settle_lean = (0.0, 0.0)
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

    def _final_obs(self, obs: np.ndarray, *, reset: bool,
                   augment_reset: bool | None = None) -> np.ndarray:
        """Apply the augment hook, then the obs-history stack."""
        aug_reset = reset if augment_reset is None else bool(augment_reset)
        obs = self._augment_obs(obs, reset=aug_reset).astype(np.float32)
        if self._hist_n <= 1:
            return obs
        if reset or self._hist_buf is None:
            self._hist_buf = [obs.copy() for _ in range(self._hist_n)]
        else:
            self._hist_buf.pop()
            self._hist_buf.insert(0, obs.copy())
        # newest first: frame 0 is the current tick (transplant prefix).
        return np.concatenate(self._hist_buf).astype(np.float32)

    def _reset_history_probe_steps(self) -> int:
        """Controlled hold ticks used to seed a real observation history.

        The legacy reset repeats one final frame K times.  That erases the
        only dynamics available before the first policy action, precisely
        where a recovery policy needs to infer support/contact.  This
        opt-in probe keeps commanding the captured passive equilibrium and
        records K-1 additional sensor frames without advancing the episode
        clock or paying reward.  C MuJoCo and both MJX reset paths call the
        same two helpers.
        """
        enabled = float(cfg_get(self.cfg, "obs", "reset_history_probe",
                                default=0.0)) > 0.0
        return self._hist_n - 1 if enabled and self._hist_n > 1 else 0

    def _reset_history_probe_obs(self) -> np.ndarray:
        """Read one controlled reset-probe tick into the history stack."""
        self._state = self._read_state()
        goal = self._current_goal()
        return self._final_obs(
            build_obs(self.cfg, self._state, self._q_nom,
                      self._prev_action, goal=goal,
                      tilt_ref=self._tilt_ref0),
            reset=False, augment_reset=True)

    # ------------------------------------------------------------------
    # state readout (sim → RobotState, with DR sensor corruption)
    # ------------------------------------------------------------------

    def _read_state(self) -> RobotState:
        mujoco = self._mujoco
        q = self.data.qpos[self._qadr].copy()
        qd = self.data.qvel[self._vadr].copy()
        torque = self.data.qfrc_actuator[self._vadr].copy()
        if self._struct_comp is not None and self._struct_comp_k is not None:
            q = self._struct_comp.reported_q(q, torque,
                                             k=self._struct_comp_k)

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

    def _apply_struct_compliance_to_model(self, model) -> None:
        if self._struct_comp is None or self._struct_comp_k is None:
            return
        self._struct_comp.apply_effective_kp(
            model, self._pos_act, k=self._struct_comp_k)

    def _walk_push_torque_nm(self) -> float:
        """dr.walk_push_* (08-12, the takeoff mechanism the command-side
        kick could not deliver): signed half-sine roll TORQUE on the
        chassis over the first ~second of walk-mode episodes, applied
        via xfrc_applied about the chassis's own x-axis. The 08-12
        replay_trace calibration measured the fold-pulse kick
        saturating at 5-10° peak / ~10 °/s at ANY dose (planted
        opposite feet + write-profile rate limit), far below the
        hardware takeoff regime (13-27° peaks, 11-46 °/s) — a base
        torque bypasses the actuator path and reaches it. Stateless
        per tick (pure function of _ep_rand + _step_i; the substep
        loop overwrites xfrc every step, zero outside the window →
        pool-restore safe). Applied by _advance on private-model envs;
        shared-model (MJX shim) envs expose it to their vec env, which
        hands the per-env value to the batched stepper's xfrc row."""
        er = self._ep_rand
        if (er is None or er.walk_push_peak_nm == 0.0
                or er.walk_push_dur_s <= 0.0
                or self._goal_traj is None
                or getattr(self._goal_traj, "mode", "") != "walk"):
            return 0.0
        t = self._step_i * self.dt
        if t >= er.walk_push_dur_s:
            return 0.0
        return er.walk_push_peak_nm * math.sin(
            math.pi * t / er.walk_push_dur_s)

    def _ext_push_force_n(self) -> tuple[float, float]:
        """dr.ext_push_* (AMP brief §7.4/§9.3, M3 push-recovery
        curriculum): a mid-episode horizontal FORCE pulse (fx, fy),
        world-frame, half-sine ramped like every pulse in this file.
        Unlike ``_walk_push_torque_nm`` (a fixed roll TORQUE confined to
        the first ~1.5s that reproduces the hardware TAKEOFF wobble),
        this fires once (or, with dr.ext_push_repeat_max>1, several
        times -- see EpisodeRandomization.ext_push_extra) at random
        point(s) LATER in a walk-mode episode on a policy that is
        already walking -- the actual "shove it mid-stride and see if
        it recovers" test. Stateless per tick (pure function of
        _ep_rand + _step_i); zero outside every pulse's window ->
        pool-restore safe, same as its sibling. The pulses are sampled
        non-overlapping (see domain_rand.sample), so at most one term
        is ever nonzero -- summing is just the simplest way to combine
        them without a branch per pulse."""
        er = self._ep_rand
        if (er is None or self._goal_traj is None
                or getattr(self._goal_traj, "mode", "") != "walk"):
            return (0.0, 0.0)
        t = self._step_i * self.dt
        fx = fy = 0.0
        for peak, dur, t0, ang in (
                (er.ext_push_peak_n, er.ext_push_dur_s,
                 er.ext_push_start_s, er.ext_push_dir_rad),
                *er.ext_push_extra):
            if peak == 0.0 or dur <= 0.0 or t < t0 or t >= t0 + dur:
                continue
            mag = peak * math.sin(math.pi * (t - t0) / dur)
            fx += mag * math.cos(ang)
            fy += mag * math.sin(ang)
        return (fx, fy)

    def _advance(self, *, limp: bool = False) -> None:
        assert self._profile is not None
        mujoco = self._mujoco
        h = self.model.opt.timestep
        r_off = (np.zeros(3) if self._ep_rand is None
                 else self._ep_rand.imu_pos_m)
        vel = np.zeros(6)
        push_nm = 0.0 if limp else self._walk_push_torque_nm()
        push_fx, push_fy = (0.0, 0.0) if limp else self._ext_push_force_n()
        # Only claim xfrc_applied[chassis, 0:3] for episodes that actually
        # drew an ext_push this episode (dr.ext_push_prob > 0 somewhere
        # upstream) -- indices 0:3 are also used by unrelated interactive
        # tools (web_session.py's manual push slider, quad probes) that
        # never touch dr.ext_push_*; when this episode never drew a push
        # (the default, and always true for those tools since they don't
        # set dr.ext_push_prob), skip the write entirely so this axis
        # stays a complete no-op and cannot clobber their state.
        ext_push_owns_row = (not limp and self._ep_rand is not None
                             and self._ep_rand.ext_push_peak_n != 0.0)
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
            # Takeoff push torque about the chassis's CURRENT x-axis
            # (world-frame xfrc row). Overwritten every substep, zeroed
            # outside the pulse window — no state survives the window.
            Rp = self.data.xmat[self._chassis_bid].reshape(3, 3)
            self.data.xfrc_applied[self._chassis_bid, 3:6] = (
                Rp[:, 0] * push_nm)
            # Mid-episode external push (dr.ext_push_*): world-frame
            # horizontal force, same overwrite-every-substep /
            # zero-outside-window convention as the takeoff torque
            # above (no state survives the pulse window) -- but ONLY
            # for episodes that own this row (see ext_push_owns_row).
            if ext_push_owns_row:
                self.data.xfrc_applied[self._chassis_bid, 0:3] = (
                    push_fx, push_fy, 0.0)
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

    def _rise_rock_offset(self) -> np.ndarray | None:
        """dr.rise_rock_* (08-11, hardware belly-curl rocking gap):
        one-side hip/knee fold bias added to the PHYSICAL servo
        command on rise-mode episodes that drew it, RAMP-GATED by the
        rise goal's height-ramp progress. Uses the tipped-start
        fold→roll mapping. The logical loop never sees the bias (like
        zero_drift_cmd_frame); encoders read the true drooped angles
        and the tilt reference stays level, so leveling and honest
        ref-tracking are paid only when the policy closes the
        command-vs-read loop. Stateless per tick (pure function of
        _ep_rand + _goal_traj + _step_i → pool-restore safe by
        construction).

        CALIBRATION (08-11/12, replay_trace open-loop replay of the
        10 recorded rl_stand failures): the hardware trip is NOT a
        curl-long rock — the tapes are FLAT through the curl, then
        ramp 0→10.6° in the last ~1.2 s as the belly unloads onto a
        near-diagonal foot pair (support knife-edge; sim and hardware
        sit on opposite branches — sim gets caught at ~2° by the
        planted opposite feet, hardware tips through the trip).
        A PERSISTENT fold rocks the flat curl too (unlike every tape)
        and one sign saturates at 3-5°; gating the fold by the
        height-ramp fraction reproduces the recorded signature:
        flat curl, then an accelerating ramp that crosses the 10°
        trip band near ramp end at ~18° target dose on the branch
        that removes the catching foot (the other branch saturates —
        sign is sampled ±, so training visits both)."""
        er = self._ep_rand
        if (er is None or er.rise_rock_roll_deg == 0.0
                or self._goal_traj is None
                or getattr(self._goal_traj, "mode", "") != "rise"):
            return None
        h = np.asarray(self._goal_traj.height, dtype=float)
        h_tgt = float(np.max(h)) if h.size else 0.0
        if h_tgt <= 1e-9:
            frac = 1.0
        else:
            i = min(max(self._step_i, 0), len(h) - 1)
            frac = float(np.clip(h[i] / h_tgt, 0.0, 1.0))
        if frac <= 1e-6:
            return None
        roll = er.rise_rock_roll_deg
        fold = abs(roll) * frac / self.TIP_ROLL_PER_FOLD * DEG2RAD
        legs = (3, 4, 5) if roll > 0 else (0, 1, 2)
        dq = np.zeros(N_JOINTS, dtype=float)
        for leg in legs:
            dq[3 * leg + 1] -= fold
            dq[3 * leg + 2] += 0.5 * fold
        return dq

    def _walk_kick_offset(self) -> np.ndarray | None:
        """dr.walk_kick_* (08-11, hardware takeoff-transient gap):
        TRANSIENT one-side fold pulse on the PHYSICAL servo command
        over the first ~second of walk-mode episodes. bench_report
        over 18 hardware walks: every one crosses 5° roll within
        0.6-1.5 s of gait start at 11-46 °/s roll rates, and static
        leans do not reproduce it (takeoff25-r1 child==parent) — the
        gap is the roll RATE, so the injection must move. Half-sine
        envelope: ramps in and out with net-zero terminal offset, so
        only the dynamic excursion remains to be survived. Same
        fold→roll mapping and command-side wiring as tipped/rise-rock
        (logical loop blind, encoders read true angles, tilt ref
        level). Stateless per tick (pure function of _ep_rand +
        _step_i → pool-restore safe by construction)."""
        er = self._ep_rand
        if (er is None or er.walk_kick_roll_deg == 0.0
                or er.walk_kick_dur_s <= 0.0
                or self._goal_traj is None
                or getattr(self._goal_traj, "mode", "") != "walk"):
            return None
        t = self._step_i * self.dt
        if t >= er.walk_kick_dur_s:
            return None
        roll = er.walk_kick_roll_deg * math.sin(
            math.pi * t / er.walk_kick_dur_s)
        if abs(roll) < 0.5:
            return None
        fold = abs(roll) / self.TIP_ROLL_PER_FOLD * DEG2RAD
        legs = (3, 4, 5) if roll > 0 else (0, 1, 2)
        dq = np.zeros(N_JOINTS, dtype=float)
        for leg in legs:
            dq[3 * leg + 1] -= fold
            dq[3 * leg + 2] += 0.5 * fold
        return dq

    def _walk_stop_freeze_override(self, q_safe):
        """goal.walk_stop_freeze_s (2026-08-24, joyfullcurr10-
        stopsettle-probe dig-in): STRUCTURAL stop-hold, not a reward
        charge. The stopsettle diagnostic measured the V6 b1 cert's
        residual creep as a genuine POST-grace floor (excluding the
        exact 0.4s reward.walk_stop_grace_s window a checkpoint was
        trained under barely moved the metric: stop_speed_settled_m_s
        0.03107 vs raw stop_speed_m_s 0.03264, ~5%) -- refuting the
        entire stop-speed/stop-current REWARD-PRICING lever both by
        dose (joyfullcurr9/10) and by measurement methodology (this
        run), per that run's own pre-registered gate text. This hook
        is the named next lever: instead of pricing sustained motion
        and hoping the policy learns true stillness, it FORCES the
        physical command to hold -- once a walk/quadwalk-mode stop
        segment has been commanded for more than walk_stop_freeze_s
        seconds (same grace convention as the stop reward charges),
        the tick's own q_safe is discarded and replaced with the
        PREVIOUS tick's own safe command (self._cmd, read before it is
        overwritten below) -- i.e. the policy's action this tick is
        never issued to the plant. This is why the override must
        return the previous q_safe rather than re-deriving a target
        from a zeroed action: the action space is an offset from the
        nominal pose, not from the previous command, so a zeroed
        action would NOT generally hold the current pose. Turn-in-
        place commands (wz_ref != 0) are exempted exactly like the
        stop reward charges (a nonzero turn IS the commanded motion).
        The elapsed-stop timer still advances during an exempted turn
        (matching the reward charges' own timer semantics) so a turn
        that stops mid-episode does not get a fresh grace window.
        Default 0.0 = off, bit-exact (returns q_safe unchanged, no
        state touched)."""
        thr = float(cfg_get(self.cfg, "goal", "walk_stop_freeze_s",
                             default=0.0))
        if thr <= 0.0:
            return q_safe
        if (self._goal_traj is None
                or getattr(self._goal_traj, "mode", "")
                not in ("walk", "quadwalk")):
            return q_safe
        goal = self._current_goal()
        s_ref = float(np.hypot(
            float(getattr(goal, "vx_ref", 0.0) or 0.0),
            float(getattr(goal, "vy_ref", 0.0) or 0.0)))
        if s_ref > 1e-3:
            self._walk_stop_freeze_cmd_s = 0.0
            return q_safe
        t = getattr(self, "_walk_stop_freeze_cmd_s", 0.0) + self.dt
        self._walk_stop_freeze_cmd_s = t
        wz = abs(float(getattr(goal, "wz_ref", 0.0) or 0.0))
        if wz > 1e-3 or t < thr:
            return q_safe
        return self._cmd.copy()

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

    def _recover_start_bank(self) -> np.ndarray | None:
        """Harvested recover-mode start poses (08-15, recover_to_plant
        family 2). Lazy-loads the npz named by cfg
        goal.recover_start_bank (key ``q_rad``, shape (K,18)); caches
        None when unset. Same contract as _walk_park_bank."""
        if hasattr(self, "_rec_bank_cache"):
            return self._rec_bank_cache
        path = cfg_get(self.cfg, "goal", "recover_start_bank",
                       default=None)
        bank = None
        if path:
            arr = np.asarray(np.load(str(path))["q_rad"], dtype=float)
            if arr.ndim != 2 or arr.shape[1] != N_JOINTS or len(arr) == 0:
                raise ValueError(
                    f"recover_start_bank {path}: expected "
                    f"(K,{N_JOINTS}) q_rad, got {arr.shape}")
            bank = arr
        self._rec_bank_cache = bank
        return bank

    def _recover_rsi_bank(self) -> np.ndarray | None:
        """Harvested ON-PATH recover-mode poses for the harvested-bank
        RSI variant (08-16, tangle-wall mechanism fix; see the
        sim_env spawn branch and walk_task._sample_recover). Lazy-
        loads the npz named by cfg goal.recover_rsi_bank_path (key
        ``q_rad``, shape (K,18)); caches None when unset. Same
        contract as _recover_start_bank / _walk_park_bank."""
        if hasattr(self, "_rec_rsi_bank_cache"):
            return self._rec_rsi_bank_cache
        path = cfg_get(self.cfg, "goal", "recover_rsi_bank_path",
                       default=None)
        bank = None
        if path:
            arr = np.asarray(np.load(str(path))["q_rad"], dtype=float)
            if arr.ndim != 2 or arr.shape[1] != N_JOINTS or len(arr) == 0:
                raise ValueError(
                    f"recover_rsi_bank_path {path}: expected "
                    f"(K,{N_JOINTS}) q_rad, got {arr.shape}")
            bank = arr
        self._rec_rsi_bank_cache = bank
        return bank

    def _rise_start_bank(self) -> np.ndarray | None:
        """Harvested settled lower-endpoint poses (08-14, post-lower
        rise exposure — SESSION_BULK_GATE's named boundary). Lazy-loads
        the npz named by cfg goal.rise_start_bank (key ``q_rad``, shape
        (K,18)); caches None when unset so the legacy path costs one
        attribute check. Same contract as _walk_park_bank."""
        if hasattr(self, "_rise_bank_cache"):
            return self._rise_bank_cache
        path = cfg_get(self.cfg, "goal", "rise_start_bank", default=None)
        bank = None
        self._rise_bank_full = None
        self._rise_bank_zstand = None
        if path:
            npz = np.load(str(path))
            arr = np.asarray(npz["q_rad"], dtype=float)
            if arr.ndim != 2 or arr.shape[1] != N_JOINTS or len(arr) == 0:
                raise ValueError(
                    f"rise_start_bank {path}: expected (K,{N_JOINTS}) "
                    f"q_rad, got {arr.shape}")
            bank = arr
            # Full-state twin (08-14 postlower2 dig-in): newer banks also
            # carry the exact settled qpos/qvel; goal.rise_start_bank_exact
            # (default OFF) restores them verbatim instead of the
            # joints-only _place_at_plant reconstruction, which proved
            # off-distribution (parent 0/12 from reconstruction vs
            # 0.801/0.967 from real in-session post-lower states).
            if "qpos_full" in npz.files and "qvel_full" in npz.files:
                qp = np.asarray(npz["qpos_full"], dtype=float)
                qv = np.asarray(npz["qvel_full"], dtype=float)
                if len(qp) == len(arr) and len(qv) == len(arr):
                    self._rise_bank_full = (qp, qv)
            # Standing anchor per row (08-14, the postlower1/2 root
            # cause): rise heights are z0-relative and belly-calibrated;
            # a bank spawn settles ~50mm above the belly, so anchoring
            # the band at the spawn commands an IMPOSSIBLE target.
            # z_stand = the harvest lower-episode's own standing z0 —
            # what this endpoint should rise back to.
            self._rise_bank_zstand = (
                np.asarray(npz["z_stand"], dtype=float)
                if "z_stand" in npz.files
                and len(npz["z_stand"]) == len(arr) else None)
        self._rise_bank_cache = bank
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
        if self._model_source != "primitive":
            # The analytic base_z above uses the LEGACY kinematic constants
            # (body_ik FK + YAW_OUTPUT_HEIGHT); on the mesh-family models
            # (real hip rise, mid-plane foot line) it lands ~60 mm high and
            # the spawn would free-fall through the settle. Re-place from
            # the model's own geometry: lowest collidable point 2 mm above
            # the ground plane (legacy path untouched — bit-exact).
            low = lowest_collidable_z(self.model, self.data)
            self.data.qpos[2] += 0.002 - low
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
        # RECOVER "flip" spawn (08-15): consume-once pending base
        # orientation — rotate, lift clear of the floor, and let the
        # caller's settle choreography drop it however it lands
        # (side/back/upside-down). Both reset paths (C reset() and the
        # MJX batched choreography's place_env) run through here, so
        # one hook covers both. None everywhere outside the recover
        # mode's flip kind — every legacy placement is bit-exact.
        flip = getattr(self, "_flip_spawn_pending", None)
        if flip is not None:
            self._flip_spawn_pending = None
            self.data.qpos[3:7] = flip
            self.data.qpos[2] += 0.03
            mujoco.mj_forward(self.model, self.data)
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
        self._rise_bank_zstand_pending = None
        self._flip_spawn_pending = None

        self._ep_rand = (self.randomizer.sample(self.rng)
                         if self.randomizer is not None else None)
        if self._struct_comp is not None:
            dr = (getattr(self.randomizer, "scale", 1.0)
                  if self.randomizer is not None else 0.0)
            self._struct_comp_k = self._struct_comp.sample(
                self.rng, scale=dr)
        else:
            self._struct_comp_k = None

        # Physics easing (see __init__): scale this episode's gravity /
        # servo velocity ceiling by the CURRENT cfg values, so an
        # active sched.* ramp moves the physics episode-by-episode.
        # Batched-pool note: pooled resets restore entries minted at
        # choreography time, so under an active schedule an episode's
        # eased physics can lag the schedule by up to the pool depth
        # (a few episodes) — end easing schedules at v1=1.0 (nominal)
        # and judge scheduled runs on measured behavior metrics.
        self._ease_g = 1.0
        self._ease_v = 1.0
        _e_g = float(cfg_get(self.cfg, "ease", "gravity_scale",
                             default=1.0))
        _e_v = float(cfg_get(self.cfg, "ease", "vel_ceiling_scale",
                             default=1.0))
        if _e_g != 1.0 or _e_v != 1.0:
            if _e_g <= 0.0 or _e_v <= 0.0:
                raise ValueError("ease.* scales must be > 0, got "
                                 f"gravity={_e_g} vel_ceiling={_e_v}")
            if self._ep_rand is not None:
                if _e_g != 1.0:
                    self._ep_rand.gravity_vec = (
                        np.asarray(self._ep_rand.gravity_vec, float)
                        * _e_g)
                if _e_v != 1.0:
                    self._ep_rand.vel_scale = (
                        float(self._ep_rand.vel_scale) * _e_v)
            elif self._owns_model:
                # randomize=False private-model env (eval harness at
                # DR-0, viewers): reset() applies the same scales
                # directly to the model / servo profile.
                self._ease_g, self._ease_v = _e_g, _e_v
            else:
                raise ValueError(
                    "ease.* on a shared-model shim env needs "
                    "randomize=True (dr_scale may be 0): the batched "
                    "path can only ease gravity through per-world "
                    "model-DR fields")

        # Mode-sequencing episode state (goal.mode_seq, TRANSITIONS_
        # DIRECTIVE CODE item 1). Cleared BEFORE _sample_goal so the
        # planner (walk_task._sample_mode_seq) can repopulate it; all
        # five ride mjx_host.SNAP_ATTRS (pool-restore lesson). None/0
        # defaults = legacy bit-exact (the switch hook is a single
        # attr check per tick and no rng is ever drawn).
        self._seq_plan = None          # [{mode, tick, blend}, ...]
        self._seq_idx = 0              # index of the ACTIVE segment
        self._seq_stand_z = None       # abs z of the last commanded stand
        self._seq_seg_end = None       # active segment's end tick
        self._seq_pose_anchor = None   # hold/lower BC base pose mid-seq

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
                and getattr(self._goal_traj, "mode", "") == "rise"
                # Bank episodes ARE the post-lower exposure — RSI must
                # not override them (rise_bank never occurs unless
                # goal.rise_start_bank is configured, so the legacy
                # rng stream is untouched when the feature is off).
                and getattr(self._goal_traj, "start_at", "")
                != "rise_bank"):
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
        # RECOVER RSI (08-16, zero-family mechanism fix): spawn a
        # flagged recover episode ON the demonstrated belly->plant
        # path instead of its family pose. Root cause (cw-recover-any8
        # /any9, hw track): the recovery ladder's partial_* rungs are
        # LINEAR curls (f * q_crouch), not states on the executable
        # rise trajectory, so a policy stuck at the zero (belly-flat)
        # family never PRACTICES from mid-rise states — the exact
        # exploration gap the rise-mode RSI above closed for the rise
        # task (row-range formula reused: belly curl through ~90% of
        # the ramp, never the free-success top). The flag is set ONLY
        # by walk_task._sample_recover on NATURALLY drawn kinds
        # (goal.recover_rsi_frac/_kinds, default off); forced CERT/
        # eval kinds never carry it, so certification stays pure by
        # construction. The settle choreography (incl. limp sag) runs
        # unchanged — same sag-robustness contract as rise RSI. Level
        # tilt anchoring matches the recover "any" branch below.
        if (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "recover"
                and getattr(self._goal_traj, "recover_rsi", False)):
            rsi_ref = cfg_get(self.cfg, "reward", "rise_ref_path",
                              default=None)
            if not rsi_ref:
                raise ValueError("goal.recover_rsi_frac needs "
                                 "reward.rise_ref_path")
            ref = load_rise_ref(str(rsi_ref))
            if "h" not in ref:
                raise ValueError("goal.recover_rsi_frac needs a rise "
                                 "reference with per-tick heights "
                                 "(re-extract with extract_rise_ref)")
            n, i0 = len(ref["q"]), int(ref["ramp_i0"])
            j = int(self.rng.integers(0, i0 + int(0.9 * (n - 1 - i0))))
            q_rsi = ref["q"][j] + self.rng.uniform(
                -2.0, 2.0, N_JOINTS) * DEG2RAD
            if self._ep_rand is not None:
                q_rsi = q_rsi + self._ep_rand.start_offset_rad
            self._tipped_applied = True
            return self._clip_to_joint_limits(q_rsi)
        # RECOVER RSI, HARVESTED-BANK variant (08-16, tangle-wall
        # mechanism fix): spawn on a pose harvested from a checkpoint's
        # OWN successful rollouts of the target kind
        # (goal.recover_rsi_bank_path, built by
        # harvest_recover_rsi_bank.py) instead of the belly->plant
        # reference above, which has no equivalent for non-monotonic
        # untangling motion. Flag set only by
        # walk_task._sample_recover (goal.recover_rsi_bank_frac/
        # _kinds, default off, forced CERT/eval kinds never carry it).
        if (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "recover"
                and getattr(self._goal_traj, "recover_rsi_bank", False)):
            bank = self._recover_rsi_bank()
            if bank is None:
                raise ValueError("goal.recover_rsi_bank_frac needs "
                                 "goal.recover_rsi_bank_path")
            idx = int(self.rng.integers(0, len(bank)))
            q_rsi = bank[idx] + self.rng.uniform(
                -2.0, 2.0, N_JOINTS) * DEG2RAD
            if self._ep_rand is not None:
                q_rsi = q_rsi + self._ep_rand.start_offset_rad
            self._tipped_applied = True
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
        elif start_at == "rise_bank":
            # Post-lower rise start (08-14): a harvested settled
            # lower-endpoint pose of the policy's OWN lower skill
            # (goal.rise_start_bank, built by harvest_lower_endpoints).
            # SESSION_BULK_GATE named this the single trainable
            # boundary: ALL 10 det session failures + the weakest sto
            # stratum (0.801, over_current-dominated) were post-lower
            # rises, while synthetic-start first rises were 300/300.
            # +-2 deg jitter, same as the walk park bank.
            bank = self._rise_start_bank()
            if bank is None:
                raise RuntimeError(
                    "start_at='rise_bank' requires goal.rise_start_bank")
            bi = int(self.rng.integers(len(bank)))
            q_start = bank[bi].copy()
            anchor = float(cfg_get(self.cfg, "goal",
                                   "rise_start_bank_anchor_stand",
                                   default=0.0)) > 0.0
            if anchor:
                zs = getattr(self, "_rise_bank_zstand", None)
                if zs is None:
                    raise ValueError(
                        "goal.rise_start_bank_anchor_stand needs a bank "
                        "with the z_stand array (re-harvest with the "
                        "08-14 harvest_lower_endpoints) — the legacy "
                        "bank has no standing anchor to rewrite the "
                        "height schedule against.")
                self._rise_bank_zstand_pending = float(zs[bi])
            exact = (float(cfg_get(self.cfg, "goal",
                                   "rise_start_bank_exact",
                                   default=0.0)) > 0.0
                     and getattr(self, "_rise_bank_full", None) is not None)
            if exact:
                # Exact-restore mode (08-14, default OFF): spawn IS the
                # harvested settled state, verbatim — no jitter, no
                # start-offset, no re-plant/settle choreography. The
                # joints-only reconstruction proved off-distribution
                # (see _rise_start_bank docstring).
                qp, qv = self._rise_bank_full
                self._exact_start_pending = (qp[bi].copy(), qv[bi].copy())
                q_start = self._clip_to_joint_limits(q_start)
            else:
                q_start += self.rng.uniform(-2.0, 2.0, N_JOINTS) * DEG2RAD
                if self._ep_rand is not None:
                    q_start = q_start + self._ep_rand.start_offset_rad
                q_start = self._clip_to_joint_limits(q_start)
        elif start_at == "gait":
            # Mid-stride TALL spawn (TALL LADDER T6: RSI-for-walk, see
            # walk_task._sample_walk). Scripted tripod-gait pose at a
            # random phase, generated at the episode's own commanded
            # velocity so swing/stance geometry matches the command the
            # policy wakes up under. The gait is rolled forward ~1 s
            # plus a uniform slice of one period so its internal
            # command smoothing is engaged and every phase is sampled.
            from sim_gait_compat import TripodGait
            traj = self._goal_traj
            i_ss = min(int(round(0.5 / self.dt)), len(traj.vx) - 1)
            g = TripodGait()
            g.sync_plant_stance(float(self._plant_deg[1]),
                                float(self._plant_deg[2]))
            g.set_velocity(vx=float(traj.vx[i_ss]),
                           vy=float(traj.vy[i_ss]))
            # Turn-state reset densification (08-23, turnlib3 FAIL
            # branch): goal.walk_gait_spawn_wz (default 0 = off,
            # bit-exact: omega is simply never passed) additionally
            # feeds the episode's own commanded yaw rate into the
            # scripted-gait pose generator, so turn / turn-in-place
            # episodes SPAWN mid-rotation instead of always entering
            # the turn from a standstill. Pricing (k_yaw_prog 1-3x),
            # demo range (teacher_v3) and style ablation (-noamp1)
            # were all measured unable to move tip tracking; the
            # policy never VISITS fast-turning states — same
            # densify-at-reset shape as park_start/gait_start.
            spawn_wz = float(cfg_get(self.cfg, "goal",
                                     "walk_gait_spawn_wz", default=0.0))
            wz_arr = getattr(traj, "wz", None)
            if spawn_wz > 0.0 and wz_arr is not None:
                g.set_velocity(omega=float(wz_arr[i_ss]) * spawn_wz)
            g.reset_phase()
            warm = 1.0 + float(self.rng.uniform(0.0, g.period))
            t, q_deg = 0.0, g.neutral_pose_deg()
            while t < warm:
                q_deg = g.desired_deg(t)
                t += self.dt
            q_start = np.asarray(q_deg, dtype=float) * DEG2RAD
            q_start += self.rng.uniform(-2.0, 2.0, N_JOINTS) * DEG2RAD
            if self._ep_rand is not None:
                q_start = q_start + self._ep_rand.start_offset_rad
            q_start = self._clip_to_joint_limits(q_start)
        elif start_at == "any":
            # GETUP-mode start diversity (operator 08-11: "from any
            # position: recover -> stand -> walk"). The recovery task's
            # curriculum IS its start distribution — episodes spawn all
            # along the pipeline (random legal tangle that settles
            # however it lands incl. tipped, belly-zero, partial curl,
            # crouch, plant, tripod park) so backward-chaining needs no
            # reference trajectory or RSI. The KIND was drawn by
            # _sample_getup (goal side, where the force hook lives) and
            # rides on the trajectory.
            kind = getattr(self._goal_traj, "start_kind", "tangle")
            tangle_blends = {
                "tangle_mild": 0.25,
                "tangle_mid": 0.50,
                "tangle_60": 0.60,
                "tangle_70": 0.70,
                # Legacy forced-eval alias retained for old probes.
                "tangle_deep": 0.75,
                "tangle_80": 0.80,
                "tangle_90": 0.90,
                "tangle": 1.0,
            }
            if kind in tangle_blends:
                from rl_move.safety import AXIS_LIMITS_DEG
                q_random = np.array(
                    [self.rng.uniform(*AXIS_LIMITS_DEG[j % 3])
                     for j in range(N_JOINTS)], dtype=float) * DEG2RAD
                q_start = tangle_blends[kind] * q_random
            elif kind == "zero":
                q_start = self.rng.uniform(
                    -2.0, 2.0, N_JOINTS) * DEG2RAD
            elif kind in ("partial", "crouch", "crouch_shallow",
                          "crouch_mid", "crouch_deep", "partial_high",
                          "partial_mid", "partial_low"):
                from rl_move.body_ik import BodyOffset
                crouch_ranges = {
                    "crouch_shallow": (0.010, 0.025),
                    "crouch_mid": (0.025, 0.045),
                    "crouch_deep": (0.045, 0.070),
                }
                depth = (float(self.rng.uniform(*crouch_ranges[kind]))
                         if kind in crouch_ranges
                         else float(self.rng.uniform(0.03, 0.07)))
                any_ik = FixedFootBodyIK()
                any_ik.reset(self._plant_deg * DEG2RAD)
                res = any_ik.solve(BodyOffset(
                    height=-depth))
                q_c = (res.q_rad if res.ok
                       else self._plant_deg * DEG2RAD)
                partial_ranges = {
                    "partial_high": (0.70, 0.95),
                    "partial_mid": (0.40, 0.70),
                    "partial_low": (0.15, 0.40),
                }
                f = (1.0 if kind in ("crouch", "crouch_shallow",
                                     "crouch_mid", "crouch_deep")
                     else float(self.rng.uniform(
                         *partial_ranges.get(kind, (0.10, 0.90)))))
                # Zero pose is exactly q=0, so the belly->crouch blend
                # is a plain scale (same construction as rise bridge).
                q_start = f * np.asarray(q_c, dtype=float)
            elif kind == "plant_catch":
                # First backward-curriculum rung: already at the goal
                # neighborhood, but the controller must catch and hold
                # plant for the full success dwell instead of receiving
                # a free terminal reward at reset.
                q_start = (self._plant_deg * DEG2RAD).copy()
                q_start += self.rng.uniform(
                    -2.0, 2.0, N_JOINTS) * DEG2RAD
            elif kind == "park":
                q_start = (self._plant_deg * DEG2RAD).copy()
                tripod = ((1, 3, 5) if self.rng.random() < 0.5
                          else (0, 2, 4))
                for leg in tripod:
                    q_start[3 * leg + 1] -= float(
                        self.rng.uniform(10.0, 25.0)) * DEG2RAD
                    q_start[3 * leg + 2] += float(
                        self.rng.uniform(-5.0, 10.0)) * DEG2RAD
            elif kind in ("onefoot_micro", "onefoot_mid", "onefoot"):
                # Progressive one-foot correction rungs.  They use the
                # same construction and differ only in disturbance
                # magnitude, so promotion measures a real expansion of
                # the solved basin instead of a task-definition switch.
                q_start = (self._plant_deg * DEG2RAD).copy()
                leg = int(self.rng.integers(6))
                if kind == "onefoot_micro":
                    hip_deg = self.rng.uniform(3.0, 8.0)
                    knee_deg = self.rng.uniform(-1.0, 3.0)
                elif kind == "onefoot_mid":
                    hip_deg = self.rng.uniform(8.0, 15.0)
                    knee_deg = self.rng.uniform(-3.0, 6.0)
                else:
                    hip_deg = self.rng.uniform(15.0, 30.0)
                    knee_deg = self.rng.uniform(-5.0, 12.0)
                q_start[3 * leg + 1] -= float(
                    hip_deg) * DEG2RAD
                q_start[3 * leg + 2] += float(knee_deg) * DEG2RAD
            elif kind in ("repair_one", "repair_two"):
                # Terminal contact-repair rungs. Keep the chassis on a
                # plant support polygon while one/two legs begin folded
                # and laterally misplaced. Unlike the early one-foot
                # rungs, yaw is wrong too: merely lowering the hip cannot
                # satisfy footprint + six-load success, so the policy must
                # identify and deliberately re-place the missing foot.
                q_start = (self._plant_deg * DEG2RAD).copy()
                n_bad = 1 if kind == "repair_one" else 2
                first = int(self.rng.integers(6))
                if n_bad == 1:
                    legs = (first,)
                else:
                    # Adjacent lifted pairs put the four remaining feet
                    # on one side and collapse the chassis during limp
                    # settle. Non-adjacent pairs retain a true four-foot
                    # support polygon, matching the quiet B14 failures.
                    candidates = [leg for leg in range(6)
                                  if leg != first
                                  and (leg - first) % 6 not in (1, 5)]
                    legs = (first, int(self.rng.choice(candidates)))
                for leg in np.asarray(legs, dtype=int):
                    sign = -1.0 if self.rng.random() < 0.5 else 1.0
                    q_start[3 * leg] += sign * float(
                        self.rng.uniform(15.0, 35.0)) * DEG2RAD
                    # The quadstance feasibility sweep's tucked claw is
                    # known to stay clear while the other four feet form
                    # a support polygon. Small jitter keeps this a family,
                    # not one memorized target.
                    q_start[3 * leg + 1] = (_QUAD_TUCK_RAD[1]
                                             + self.rng.uniform(-3.0, 3.0)
                                             * DEG2RAD)
                    q_start[3 * leg + 2] = (_QUAD_TUCK_RAD[2]
                                             + self.rng.uniform(-4.0, 4.0)
                                             * DEG2RAD)
            elif kind == "bank":
                # RECOVER family 2: harvested post-lower/interrupted
                # poses (goal.recover_start_bank npz, key q_rad
                # (K,18)). Placement + slip/limp settle produce a
                # physically consistent start; the exact-qvel restore
                # is CPU-only (family-5 falling velocities are the
                # pre-registered next rung).
                bank = self._recover_start_bank()
                if bank is None:
                    raise ValueError("start_kind 'bank' requires "
                                     "goal.recover_start_bank")
                q_start = bank[int(self.rng.integers(len(bank)))].copy()
                q_start += self.rng.uniform(
                    -2.0, 2.0, N_JOINTS) * DEG2RAD
            elif kind == "flip":
                # Final recovery rung: random legal joints plus a base
                # rotation about a
                # random horizontal axis, applied by _place_at_plant
                # (consume-once pending quat, both C and MJX paths go
                # through place_env -> _place_at_plant), then the
                # slip/limp settle drops it however it lands. Runs
                # enabling this kind must widen safety.max_roll/
                # pitch_deg to ~179 (a fall is a recoverable state).
                from rl_move.safety import AXIS_LIMITS_DEG
                q_start = np.array(
                    [self.rng.uniform(*AXIS_LIMITS_DEG[j % 3])
                     for j in range(N_JOINTS)], dtype=float) * DEG2RAD
                ax_ang = float(self.rng.uniform(0.0, 2.0 * math.pi))
                ang = float(self.rng.uniform(90.0, 180.0)) * DEG2RAD
                ax = (math.cos(ax_ang), math.sin(ax_ang), 0.0)
                half = ang / 2.0
                s = math.sin(half)
                self._flip_spawn_pending = (
                    math.cos(half), ax[0] * s, ax[1] * s, ax[2] * s)
            else:  # "plant"
                q_start = (self._plant_deg * DEG2RAD).copy()
            if self._ep_rand is not None:
                q_start = q_start + self._ep_rand.start_offset_rad
            q_start = self._clip_to_joint_limits(q_start)
            # Recovery episodes anchor tilt obs/trip/reward to LEVEL
            # (gravity truth), like tipped starts: the task is to
            # level out from wherever the spawn settled, never to hold
            # the spawn lean. Runs enabling this mode must widen
            # safety.max_roll/pitch_deg — a fall is a recoverable
            # state here, not a termination.
            self._tipped_applied = True
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
        elif start_at == "quadstance":
            # QUADWALK four-leg spawn (08-13, quad track; opt-in via
            # cfg goal.quadwalk_start="quad", see
            # walk_task._sample_quadwalk). Mid feet splayed forward
            # (goal.quadwalk_mid_splay_m, default 0.06 — the bare
            # plant+tuck stance pitch-trips in <1 s, CoM ahead of the
            # 4-foot polygon front edge; the splayed form is the
            # QUADWALK bank's own statically-surviving freeze stance),
            # commanded lift legs pre-folded into the feasibility
            # sweep's "tuck" claw. Episodes begin INSIDE the fronts-up
            # stance the policy already knows from quad-hold, so
            # rear-four stepping is the reachable behavior and six-leg
            # walking requires actively planting the charged fronts.
            # +-2 deg jitter matches the gait/park spawn convention.
            # Reached only from quadwalk trajectories, so no legacy
            # rng stream can be perturbed.
            from sim_gait_compat import TripodGait
            splay = float(cfg_get(self.cfg, "goal",
                                  "quadwalk_mid_splay_m", default=0.06))
            g = TripodGait()
            g.sync_plant_stance(float(self._plant_deg[1]),
                                float(self._plant_deg[2]))
            _orig = g._foot_target_in_body

            def _splayed(i, vx, vy, om, _o=_orig, _s=splay):
                dx, dy, dz = _o(i, vx, vy, om)
                if i in (1, 4):
                    dx += _s
                return (dx, dy, dz)
            g._foot_target_in_body = _splayed
            g.set_velocity(vx=0.0, vy=0.0)
            g.reset_phase()
            q_start = np.asarray(g.desired_deg(0.0),
                                 dtype=float) * DEG2RAD
            lift = tuple(getattr(self._goal_traj, "lift_legs", None)
                         or (0, 5))
            for leg in lift:
                q_start[3 * leg: 3 * leg + 3] = _QUAD_TUCK_RAD
            q_start += self.rng.uniform(-2.0, 2.0, N_JOINTS) * DEG2RAD
            if self._ep_rand is not None:
                q_start = q_start + self._ep_rand.start_offset_rad
            q_start = self._apply_tipped_start(q_start)
            q_start = self._clip_to_joint_limits(q_start)
            # The limp-settle stage passively pitches this front-back-
            # asymmetric stance nose-down onto the tucked claws (~15-17
            # deg) before the servos engage; anchoring the tilt ref at
            # that sagged attitude would (a) train the policy to HOLD
            # the sag and (b) trip tilt_pitch the moment it LEVELS by
            # more than the envelope (measured: recovery to 6 deg
            # tripped at |6-16.6|>10). Keep the reference LEVEL like
            # tipped starts / "any" recovery spawns, so the attitude
            # terms pay leveling out. Runs enabling this spawn must
            # widen safety.max_roll/pitch_deg past the sag transient
            # (the deployment-contract 25 deg envelope covers it),
            # same contract as the getup "any" starts.
            self._tipped_applied = True
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
            # dr.fault_*: no-op unless this episode drew a fault (must
            # run AFTER apply_params_to_model, which SETS these rows).
            self._ep_rand.apply_fault_to_model(self.model)
        else:
            apply_params_to_model(self.model, self.params)
        self._apply_struct_compliance_to_model(self.model)
        if self._ease_g != 1.0:
            # Physics-easing fallback for randomize=False private-model
            # envs (_reset_begin); with DR on the scale already lives in
            # _ep_rand.gravity_vec and _ease_g stays 1.0.
            self.model.opt.gravity[:] = (
                self.model.opt.gravity * self._ease_g)

        # Mode-sequencing canonical frames (goal.mode_seq): capture the
        # settled plant/belly reference frames on THIS episode's model
        # (DR applied above) before the episode's own placement — the
        # probe physics is wiped by the placement + settle below, so the
        # episode start is untouched. Cached across resets when the
        # model cannot change (no DR, no easing); recomputed per episode
        # otherwise.
        if ((float(cfg_get(self.cfg, "goal", "mode_seq",
                           default=0.0)) > 0.0
             or float(cfg_get(self.cfg, "goal", "mode_seq_stance",
                              default=0.0)) > 0.0)
                and (self._seq_frames is None
                     or self._ep_rand is not None
                     or self._ease_g != 1.0 or self._ease_v != 1.0)):
            self._seq_capture_frames()

        exact_start = getattr(self, "_exact_start_pending", None)
        self._exact_start_pending = None
        if exact_start is not None:
            # Exact-restore spawn (08-14, rise_start_bank_exact): the
            # harvested settled state IS the episode start — restore it
            # verbatim (servos holding, contacts as-settled) instead of
            # re-planting at foot height and re-settling, which distorts
            # deep post-lower poses into an unreal family.
            qp, qv = exact_start
            self._mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[:] = qp
            # Recenter horizontally: harvest episodes drift in x/y and
            # dynamics are translation-invariant; keeps eval odometry
            # (forward_dist) comparable with every other spawn.
            self.data.qpos[0:2] = 0.0
            self.data.qvel[:] = qv
            self.data.ctrl[:] = 0.0
            self.data.ctrl[self._pos_act] = q_start
            self._mujoco.mj_forward(self.model, self.data)
        else:
            self._place_at_plant(q_start)
        er = self._ep_rand
        self._profile = ServoProfile(
            self.params, q_start,
            latency_scale=1.0 if er is None else er.latency_scale,
            deadband_scale=1.0 if er is None else er.deadband_scale,
            vel_scale=((1.0 if er is None else er.vel_scale)
                       * self._ease_v),
        )
        self._cmd = q_start.copy()
        if exact_start is None:
            # Settle with slippery feet AND limp servos first: when a
            # human sets the robot down (torque off), feet micro-slip and
            # joints sag until the structure reaches a passive
            # equilibrium — otherwise randomized geometry + pinned
            # contacts leave the legs isometrically preloaded at 2-3 A
            # from step 0.
            fr = self.model.geom_friction[:, 0].copy()
            self.model.geom_friction[:, 0] = self.SLIP_MU
            self._settle(0.4)      # stiff: reach the commanded pose
            self._settle(0.5, limp=True)  # limp: bleed contact preload
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
        obs, info = self._reset_finalize()
        probe_n = self._reset_history_probe_steps()
        for _ in range(probe_n):
            self._advance()
            obs = self._reset_history_probe_obs()
        if probe_n:
            info["reset_history_probe_ticks"] = probe_n
            info["reset_history_probe_s"] = probe_n * self.dt
        return obs, info

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
        # Transition-drag bookkeeping (operator 08-11 night: the robot
        # scrapes its feet across the floor during stand/sit; nothing
        # outside walk mode priced that). Per-foot previous contact +
        # XY for the loaded-slide charge in _step_finish; snapshot via
        # mjx_host.SNAP_ATTRS (pool-restore lesson, commit 65edba7).
        self._tdrag_prev_xy = [None] * 6
        self._tdrag_prev_on = [False] * 6
        self._tdrag_acc = 0.0
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
            # STALE-REFERENCE-HEIGHT FIX (08-22, root-caused from
            # cw-stand-footlow2-plant150-2c-heightfix's RSI-start
            # height_err pinned at 22-29mm after 10M extra steps of
            # training, zero movement — the pre-registered "stays
            # pinned regardless of budget" FAIL branch). ref["h"] is
            # the npz's OWN recorded per-tick chassis height, extracted
            # before the tibia-150 geometry change; replaying the SAME
            # q_rad trajectory on the CURRENT sim settles ~21mm higher
            # (measured h_rel=131.94mm vs the npz's stored h_rel_end_m
            # =110.96mm — CURRENT_TRUTHS rise_valid_plant finding).
            # The old code anchored the RSI episode's ABSOLUTE height
            # target to this stale h_end, silently training every RSI
            # spawn to a target ~21mm below the corrected
            # goal.rise_height_mm window — a genuine reward<->eval
            # misalignment, not undertraining. Fix: anchor the
            # ABSOLUTE target to this episode's own already-sampled,
            # CURRENT-cfg height (self._goal_traj.height[-1], drawn
            # from goal.rise_height_mm before this block runs) and use
            # the reference array ONLY for the FRACTIONAL progress at
            # the spawn point (robust to a uniform/stale h-scale
            # mismatch; the q_rad geometry, and hence the progress
            # ordering along the path, is unaffected by tibia length).
            h_end_ref = float(ref["h"][-1])
            h_target = float(np.asarray(self._goal_traj.height)[-1])
            frac_done = float(ref["h"][j0]) / max(h_end_ref, 1e-6)
            frac_done = min(max(frac_done, 0.0), 1.0)
            h_left = max(h_target * (1.0 - frac_done), 0.002)
            ramp_s = float(cfg_get(self.cfg, "goal", "rise_ramp_s",
                                   default=6.0))
            n_ramp = max(int(round(ramp_s * min(h_left / max(h_target, 1e-3),
                                                1.0) / self.dt)), 3)
            n_ep = len(np.asarray(self._goal_traj.height))
            self._goal_traj.height = h_left * np.clip(
                np.arange(n_ep, dtype=float) / n_ramp, 0.0, 1.0)
        # Bank-episode standing re-anchor (08-14, the postlower1/2 root
        # cause): rise heights are relative to _z0 (= wherever the body
        # settled) with a BELLY-calibrated band, but a post-lower bank
        # spawn settles ~50mm above the belly — the unmodified schedule
        # commands an impossible ~190-213mm chassis height (measured;
        # the champion parent scores 0/12 on it while rising from REAL
        # in-session post-lower states 80-97% of the time). Rewrite the
        # schedule to the REMAINING rise back to the harvested standing
        # height, RSI-style ramp scaling. Opt-in
        # (goal.rise_start_bank_anchor_stand, default OFF => bit-exact).
        z_stand = getattr(self, "_rise_bank_zstand_pending", None)
        self._rise_bank_zstand_pending = None
        if z_stand is not None and self._goal_traj is not None:
            h_left = max(z_stand - self._z0, 0.002)
            hs = np.asarray(self._goal_traj.height)
            h_end = max(float(hs[-1]), 1e-3)
            ramp_s = float(cfg_get(self.cfg, "goal", "rise_ramp_s",
                                   default=6.0))
            n_ramp = max(int(round(ramp_s * min(h_left / h_end, 1.0)
                                   / self.dt)), 3)
            n_ep = len(hs)
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
        # GETUP (recover→stand→walk, 08-11) episode state: mode flag +
        # the staged-progress ratchet baseline. The baseline is seeded
        # on the FIRST post-settle tick (walk_task._post_step) so the
        # spawn posture is never income — same convention as
        # _score_best. Both attrs ride mjx_host.SNAP_ATTRS.
        self._is_getup = (self._goal_traj is not None
                          and getattr(self._goal_traj, "mode", "")
                          == "getup")
        self._getup_best = None
        # RECOVER (recover_to_plant, 08-15 directive) episode state:
        # mode flag, the PBRS previous-potential (seeded on the first
        # post-settle tick — spawn posture is never income), and the
        # continuous success-hold counter. All three ride
        # mjx_host.SNAP_ATTRS (pool-restored episodes must not inherit
        # another episode's potential baseline or hold streak).
        self._is_recover = (self._goal_traj is not None
                            and getattr(self._goal_traj, "mode", "")
                            == "recover")
        self._rec_phi_prev = None
        self._rec_hold_n = 0
        # HOLD/TRACK BC-anchor eligibility (RL_PLAN queue 2.3, 08-11:
        # the rise lever repeated after both hold pricing levers — hard
        # zero, then the fade — moved the pricing but never reached a
        # quiet plant). Unlike rise, hold/track have no moving reference
        # to chase; the natural supervised target is simply the pose the
        # episode actually settled at (self._q_nom, captured post-settle
        # in reset() — "trivially available", RISE.md). Emission happens
        # in _step_finish once self._q_nom exists.
        self._is_hold_bc = (self._goal_traj is not None
                            and getattr(self._goal_traj, "mode", "")
                            in ("hold", "track"))
        # LOWER BC-anchor eligibility (08-11, anchorstate2 follow-up):
        # the lower bank's strict xfail documents that one-leg-aloft
        # keeps ~85% of the honest lower return (rise_posture_gate
        # prices a lifted leg at pf=5/6) — the incentive behind the
        # deployed specialist's 62-99 mm dangling foot AND the prime
        # suspect for the six-run leg-1 hold park (pricing changes
        # moved nothing all campaign; anchor supervision is the only
        # lever that moved the park fingerprint). Target emission in
        # _step_finish; gated by train.bc_anchor_lower.
        self._is_lower_bc = (self._goal_traj is not None
                             and getattr(self._goal_traj, "mode", "")
                             == "lower")
        # WALK BC-anchor reference (RL_PLAN queue 2.1 follow-up, 08-11:
        # probe_walk_income exonerated the trans1 stack's pricing —
        # honest gait out-earns every degenerate 2-4x in all four
        # directions at DR 0 AND 0.5, and the collapsed trans1/mirror2
        # checkpoints earn BELOW a freeze — so the paddle/sacrifice
        # attractors are optimization failures, not paid basins. Same
        # signature as rise/hold: nothing tells a churning leg WHICH
        # WAY to move. Third application of the proven lever: supervise
        # walk-tick actions toward the command-conditioned scripted
        # TripodGait — the exact open-loop gait that walks/crabs/turns
        # the REAL robot (tape-measured 08-10). Per-episode instance,
        # phase state lives on it, so it MUST ride SNAP_ATTRS
        # (pool-restore lesson, commit 65edba7).
        # 08-12 follow-up (cw-arch-gru-anchor1/scratch-anchor1): both
        # arms reproduce the twice-closed walk-freeze/paddle failure
        # when walk ticks are anchored ALONGSIDE rise/hold/lower on a
        # GRU, while rise/hold/lower gains held clean — the anchor
        # protects stance skills but still fights locomotion. Gated by
        # train.bc_anchor_walk so a future arm can anchor stance only.
        # Default 1.0 (on) preserves every existing config bit-exact;
        # only an explicit bc_anchor_walk=0 disables this block.
        self._walk_bc_gait = None
        self._walk_bc_t = 0.0
        if (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "walk"
                and float(cfg_get(self.cfg, "train", "bc_anchor_coef",
                                  default=0.0)) > 0.0
                and float(cfg_get(self.cfg, "train", "bc_anchor_walk",
                                  default=1.0)) > 0.0):
            self._walk_bc_gait = self._make_walk_bc_gait()
        # First ramp tick of a rise schedule (hold window ends here) —
        # the alignment anchor for the rise-reference tracking term:
        # references are recorded ramp-relative so episodes with
        # jittered hold lengths all join the same trajectory.
        self._rise_ramp_i0 = 0
        if self._is_rise:
            nz = np.nonzero(
                np.abs(np.asarray(self._goal_traj.height)) > 1e-12)[0]
            self._rise_ramp_i0 = int(nz[0]) if len(nz) else 0
        # Mode-seq stand anchor (goal.mode_seq): the absolute chassis z
        # of the last COMMANDED standing height. A mid-sequence rise
        # aims back at this (re-anchored per switch — lesson 5 of the
        # transitions directive: start-relative refs are the #1 hidden-
        # state trap). Standing starts anchor at the settled height;
        # rise starts at the commanded top; a belly-start lower has no
        # known stand height until its first rise completes.
        if getattr(self, "_seq_plan", None) is not None:
            _m0 = getattr(self._goal_traj, "mode", "")
            if _m0 == "rise":
                self._seq_stand_z = self._z0 + self._h_target
            elif getattr(self._goal_traj, "start_at", "plant") != "zero":
                self._seq_stand_z = self._z0
            else:
                self._seq_stand_z = None
        self._plant_feet_xy = fk_all_feet(
            self._plant_deg * DEG2RAD)[:, :2]
        self._curl_dist_prev = self._curl_dist()
        self._curl_milestones: set[float] = set()
        self._state = self._read_state()
        self._rec_reset_height_mm = 0.0
        self._rec_reset_tilt_deg = 0.0
        self._rec_reset_min_load_n = 0.0
        self._rec_reset_pad_spread_mm = 0.0
        if self._is_recover:
            _rr, _rp = self._true_roll_pitch()
            _loads = [max(float(self.data.sensordata[a]), 0.0)
                      if a >= 0 else 0.0 for a in self._touch_adr]
            _pad_z = [float(self.data.xpos[b, 2]) for b in self._pad_bids]
            self._rec_reset_height_mm = float(
                self.data.xpos[self._chassis_bid, 2]) * 1000.0
            self._rec_reset_tilt_deg = max(abs(_rr), abs(_rp)) * RAD2DEG
            self._rec_reset_min_load_n = min(_loads)
            self._rec_reset_pad_spread_mm = (
                max(_pad_z) - min(_pad_z)) * 1000.0
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
        # Settled lean relative to the episode tilt reference, captured
        # once post-settle (same tick _q_nom was captured). The
        # tilt-comp teacher's settle-lean source reads this
        # (train.bc_anchor_tilt_from_settle in _step_finish): a
        # per-episode CONSTANT, so the commanded counter-rotation does
        # not shrink as the student levels. Probe-measured 08-13
        # (probe_tilt_teacher): the current-lean proportional source
        # has a closed-loop fixed point at (L0+deadband)/2 (~4deg for
        # the ~6.5deg tipped spawns) — the teacher itself can never
        # demonstrate a <=3deg settle. In SNAP_ATTRS (pool-restore).
        self._settle_lean = (self._state.imu_roll - self._tilt_ref0[0],
                             self._state.imu_pitch - self._tilt_ref0[1])
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
        if self._struct_comp is not None and self._struct_comp_k is not None:
            info["struct_compliance"] = self._struct_comp.summary(
                self._struct_comp_k)
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

    def _active_episode_steps(self) -> int:
        """Current trajectory horizon, bounded by the env's allocation."""
        limit = getattr(self._goal_traj, "duration_steps", None)
        if limit is None:
            return int(self.episode_steps)
        return min(int(self.episode_steps), max(1, int(limit)))

    def apply_profile_ramp_frac(self, frac: float) -> dict:
        """Move the live write profile to ``frac`` of the ramp
        (0 = gentle start, 1 = the cfg target dose); trainer-driven —
        see the ``bus.profile_ramp_steps`` block in ``__init__``.
        Returns the applied values (counts/s, acc units, deg/tick) so
        the trainer can print/log the active profile. Raises when the
        ramp is not armed: a broadcast that silently no-ops is the
        dropped-cfg failure class (gotcha 3), never fall back quietly.
        """
        if self._profile_ramp is None:
            raise RuntimeError(
                "apply_profile_ramp_frac called but bus."
                "profile_ramp_steps is not set (>0) in this env's cfg "
                "— the profile ramp is not armed")
        f = min(max(float(frac), 0.0), 1.0)
        s = self._profile_ramp["start"]
        t = self._profile_ramp["target"]
        ws, acc, dq = (s[i] + f * (t[i] - s[i]) for i in range(3))
        self.write_speed_deg_s = ws * 360.0 / 4096.0
        self.write_acc_units = float(acc)
        # safety is deep-copied into MJX pool-restore snapshots
        # (mjx_host.SNAP_ATTRS), so a restored episode would revive a
        # stale max_dq — _step_begin re-asserts this value every tick
        # while the ramp is armed (the commit-65edba7 bug class).
        self._profile_ramp_dq_rad = math.radians(dq)
        self.safety.max_dq = self._profile_ramp_dq_rad
        self._profile_ramp["frac"] = f
        return {"frac": f, "write_speed_counts_s": ws,
                "write_acc": float(acc), "max_delta_q_deg": dq}

    def _step_begin(self, action):
        """Pre-physics half of step: action validation, IK, safety
        filter, and the servo command. Returns ``(early, ctx)`` —
        ``early`` is a full step tuple when the action was rejected
        outright (no physics runs in that case), else None with ``ctx``
        for :meth:`_step_finish` after physics has advanced one tick.
        Split so the batched MJX vec env can run all envs' pre-physics
        halves, one batched tick, then all post-physics halves.
        """
        # In-run coefficient scheduler (see __init__): both stacks call
        # _step_begin every tick (sim_env.step and the MJX vec envs'
        # step_wait), so this is the one hook that clocks identically
        # everywhere. Runs BEFORE this tick's reward is computed.
        if self._sched_key:
            self._sched_ticks += 1
            t = self._sched_ticks * self._sched_n
            if t <= self._sched_t0:
                v = self._sched_v0
            elif t >= self._sched_t1:
                v = self._sched_v1
            else:
                f = ((t - self._sched_t0)
                     / (self._sched_t1 - self._sched_t0))
                v = self._sched_v0 + f * (self._sched_v1 - self._sched_v0)
            self._sched_value = v
            node = self.cfg
            for k in self._sched_path[:-1]:
                node = node.setdefault(k, {})
            node[self._sched_path[-1]] = v
        assert self._state is not None and self._profile is not None
        clipped, bad = self.safety.validate_action(action, n_act=self.n_act)
        pen = float(cfg_get(self.cfg, "reward",
                            "safety_termination_penalty", default=10))
        if clipped is None:
            self._step_i += 1
            # Early-fall horizon cost (08-15, operator directive
            # fb_20260815T114414): reward.term_cost_per_remaining_s
            # charges k * REMAINING episode seconds on top of the flat
            # penalty for ANY safety termination, so a drag-then-fall
            # cannot bank income a survivor would have kept earning
            # (cw-mt-c2's ~6 s drag-then-fall retained positive return
            # at the flat -10). Truncation is never charged. Default
            # 0.0 = legacy bit-exact.
            k_rem = float(cfg_get(self.cfg, "reward",
                                  "term_cost_per_remaining_s",
                                  default=0.0))
            if k_rem > 0.0:
                rem_cost = k_rem * max(self._active_episode_steps()
                                       - self._step_i,
                                       0) * self.dt
                # Bounded terminal cost (08-17, operator-approved
                # fb_20260817T005114 item 5): the uncapped horizon
                # charge reached ~-730 on an early 60 s fall and the
                # critic never learned to predict that rare cliff
                # (explained variance ~0 through 40M on
                # cw-arch-joystick-long-scratch3). term_cost_max caps
                # the ADDED horizon component only (flat penalty is
                # untouched); falls stay decisively bad via the dense
                # roll/pitch shaping + this bounded charge. Default
                # 0 = off, legacy uncapped bit-exact.
                cap = float(cfg_get(self.cfg, "reward",
                                    "term_cost_max", default=0.0))
                if cap > 0.0:
                    rem_cost = min(rem_cost, cap)
                pen += rem_cost
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
        if self._profile_ramp_dq_rad is not None:
            # Profile ramp armed: pool-restores revive a deep-copied
            # SafetyLayer minted under an older ramp value — re-assert
            # the live slew clamp every tick (see apply_profile_ramp_frac).
            self.safety.max_dq = self._profile_ramp_dq_rad
        q_safe, status = self.safety.filter(
            q_prop, self._state, ik_ok=q_ok, ik_reason=q_reason,
            action=clipped)
        # Structural stop-hold override (goal.walk_stop_freeze_s,
        # default 0.0 = off, bit-exact identity) -- see
        # _walk_stop_freeze_override for why this runs here (after
        # the safety filter, before self._cmd is latched below).
        q_safe = self._walk_stop_freeze_override(q_safe)

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
                rock = self._rise_rock_offset()
                if rock is not None:
                    cmd_phys = self._clip_to_joint_limits(cmd_phys + rock)
                kick = self._walk_kick_offset()
                if kick is not None:
                    cmd_phys = self._clip_to_joint_limits(cmd_phys + kick)
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
        # AMP track (08-22, M1 reward-loop wiring): when
        # goal.amp_style_obs=1 (default 0 = bit-exact legacy: no key,
        # no compute), emit the 60-dim AMP discriminator feature
        # vector (rl_docs/AMP_LOCOMOTION.md §3.6) into info each tick.
        # RAW joint angles in dims 0..17 (neutral=0): the trainer-side
        # AMPStyleVecWrapper subtracts the motion library's OWN neutral
        # pose so there is exactly one authoritative neutral convention
        # (the library's — verified identical across all teacher_v1
        # clips). Works on both physics backends: obs_style_from_data
        # only touches qpos/qvel/xpos/xmat/sensordata, present on real
        # MjData and on mjx_host.FakeData alike. On the rejected-action
        # early return the mirror is one tick stale — that episode
        # terminates immediately and the wrapper's done-masking drops
        # the follow-up pairing, so at most one near-duplicate
        # transition per (rare) rejected action reaches the
        # discriminator replay.
        amp_on = getattr(self, "_amp_style_on", None)
        if amp_on is None:
            amp_on = float(cfg_get(self.cfg, "goal", "amp_style_obs",
                                   default=0.0)) > 0.0
            self._amp_style_on = amp_on
            # goal.amp_style_cmd_cond=1 (default 0 = bit-exact legacy,
            # only reachable when amp_style_obs is also on): appends
            # the CURRENT commanded (vx_ref, vy_ref, wz_ref) to the
            # emitted obs_style vector (60 -> 63 dims). This is the
            # command-conditioning fix for the 08-23 yaw-authority
            # root-cause finding (rl_docs/tracks/amp/STATUS.md ~12:4x):
            # the discriminator previously saw raw base_angular_velocity
            # with no idea what rotation rate was COMMANDED, so any
            # policy turning faster than the teacher's own demos (which
            # embody ~0.13-0.18 rad/s regardless of label) read as
            # "unlike the teacher" and got docked — pricing, wider-
            # ceiling demos, style ablation and reset densification
            # were all measured unable to move this. Only meaningful
            # paired with a motion library built with
            # ``build_motion_library.py --cmd-cond`` (matching 63-dim
            # obs_style) — a mismatched dim is a loud shape error from
            # AMPStyleVecWrapper/AMPDiscriminator, never a silent
            # misread.
            self._amp_style_cmd_cond = float(cfg_get(
                self.cfg, "goal", "amp_style_cmd_cond", default=0.0)) > 0.0
            if amp_on:
                from .amp_features import chassis_pad_gyro_ids
                self._amp_style_ids = chassis_pad_gyro_ids(self)
                self._amp_style_neutral = np.zeros(
                    int(self._amp_style_ids.qadr.shape[0]))
        if amp_on:
            from .amp_features import obs_style_from_data
            cmd = None
            if self._amp_style_cmd_cond:
                goal = self._current_goal()
                cmd = (float(getattr(goal, "vx_ref", 0.0)),
                       float(getattr(goal, "vy_ref", 0.0)),
                       float(getattr(goal, "wz_ref", 0.0)))
            result[4]["amp_obs_style"] = obs_style_from_data(
                self.data, self._amp_style_ids, self._amp_style_neutral,
                cmd=cmd)
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

    def _make_walk_bc_gait(self):
        """Per-episode TripodGait instance for the walk BC anchor
        (shared by _reset_finalize and the mode-seq switch path).

        train.bc_anchor_knee_abs=1 (default 0) selects the RAW
        hardware-module TripodGait, whose post-30660b51 desired_deg
        knees are ABSOLUTE-tibia angles fed unconverted into the sim
        knee joints. That IS the joint-space dialect of the 08-22
        phase-BC-clone lineage (ppo_goal_cw_bcgait_init_fullprof_
        phase1 was minted from the raw module before the
        sim_gait_compat boundary existed, and it walks clean at the
        measured plant) — anchoring that lineage to the CONVERTED
        gait would pull the clone off its own proven gait. Default 0
        keeps the convention-corrected sim gait, bit-exact."""
        if float(cfg_get(self.cfg, "train", "bc_anchor_knee_abs",
                         default=0.0)) > 0.0:
            try:
                from tripod_gait import TripodGait
            except ImportError:
                import sys as _sys
                _lc = str(Path(__file__).resolve().parents[2]
                          / "linux_control")
                if _lc not in _sys.path:
                    _sys.path.insert(0, _lc)
                from tripod_gait import TripodGait
            _g = TripodGait(vx=0.0)
            _g.sync_plant_stance(20.0, 80.0)
            _g.reset_phase()
            return _g
        try:
            from sim_gait_compat import TripodGait
        except ImportError:
            import sys as _sys
            _lc = str(Path(__file__).resolve().parents[2]
                      / "linux_control")
            if _lc not in _sys.path:
                _sys.path.insert(0, _lc)
            from sim_gait_compat import TripodGait
        _g = TripodGait(vx=0.0)
        # Canonical sim plant stance (same source as _default_plant
        # fallback and the WALK semantics bank): +20/+80.
        _g.sync_plant_stance(20.0, 80.0)
        _g.reset_phase()
        return _g

    # ---- mode sequencing (goal.mode_seq; TRANSITIONS_DIRECTIVE item 1)

    def _seq_segment_traj(self, mode: str, tick: int):
        """Build one mid-episode segment's reference schedule. Only the
        goal tasks support mode sequencing: goal_task (rise/hold/lower,
        goal.mode_seq_stance) and walk_task (adds walk, goal.mode_seq)."""
        raise NotImplementedError(
            "mode_seq segments require a goal task (joint_goal for "
            "stance-only sequences, joint_walk for walk grammars)")

    # Segment family -> canonical start pose the frame probe settles at.
    # walk/hold/track/lower episodes all reset at the plant; rise resets
    # belly-flat (the instrument's post-lower rise uses
    # force_rise_start="flat" — eval_modeseq.reanchor_to).
    SEQ_FRAME_FAMILY = {"rise": "belly", "walk": "plant", "hold": "plant",
                        "track": "plant", "lower": "plant"}

    def _seq_capture_frames(self) -> None:
        """Settle-probe the canonical segment frames for this episode's
        model (called from reset() BEFORE the episode's own placement,
        which wipes the probe physics). Each probe replays the exact
        reset choreography (place -> slip stiff settle -> slip limp
        settle -> capture nominal -> hold settle) at the family's
        canonical start pose, and records the reference frame a FRESH
        episode of that family would get: q_nom, _z0, pad-z ref. These
        are the eval_handoff/reanchor_to() mechanics — the composition-
        proven switch context both eval instruments derive via a full
        env.reset() — reproduced in-env so mid-episode switches see the
        identical frame. No rng draws (legacy streams bit-exact)."""
        frames: dict = {}
        for fam, q_probe in (
                ("plant", self._clip_to_joint_limits(
                    self._plant_deg * DEG2RAD)),
                ("belly", np.zeros(N_JOINTS, dtype=float))):
            self._place_at_plant(q_probe)
            er = self._ep_rand
            self._profile = ServoProfile(
                self.params, q_probe,
                latency_scale=1.0 if er is None else er.latency_scale,
                deadband_scale=1.0 if er is None else er.deadband_scale,
                vel_scale=((1.0 if er is None else er.vel_scale)
                           * self._ease_v),
            )
            self._cmd = q_probe.copy()
            fr = self.model.geom_friction[:, 0].copy()
            self.model.geom_friction[:, 0] = self.SLIP_MU
            self._settle(0.4)
            self._settle(0.5, limp=True)
            self.model.geom_friction[:, 0] = fr
            q_nom = self.data.qpos[self._qadr].copy()
            self._profile.reset(q_nom)
            self._cmd = q_nom.copy()
            self._settle(0.3)
            frames[fam] = {
                "q_nom": q_nom,
                "z0": float(self.data.xpos[self._chassis_bid, 2]),
                "pad_z_ref": np.array(
                    [float(self.data.xpos[b, 2]) if b >= 0 else 0.0
                     for b in self._pad_bids]),
            }
        self._seq_frames = frames

    def _seq_maybe_switch(self) -> None:
        """Mid-episode mode switch (called once per tick, immediately
        after _step_i advances and BEFORE the goal is read). At each
        planned boundary: install the new segment family's CANONICAL
        reference frame (q_nom / _z0 / pad-z ref from the settle probe
        — exactly what eval_handoff/reanchor_to() derive via a fresh
        reset of the target mode), regenerate the refs with a blend
        window, and re-derive the goal-derived episode bookkeeping.
        Physics, servo/profile state, tilt frame and the safety layer's
        slew memory all carry over — on hardware a mode command changes
        no physical state, and the proven reanchor mechanics explicitly
        restore them across the switch.

        HISTORY (trans-dagger2 kill, 08-14): v1 of this switch kept the
        EPISODE-reset q_nom and re-based _z0 on the instantaneous
        chassis height. Since obs joints are (q - q_nom), a rise-start
        sequence fed every later plant-family segment a belly frame
        (~79 deg off at the knees) — footlow2_hard1 fell 99/225 demo
        sequences in-env (lower 73) while scoring 11/12 zero-fall on
        the instrument, whose frames come from reanchor_to(). The
        canonical-frame install below is the fix."""
        nxt = self._seq_idx + 1
        if self._seq_plan is None or nxt >= len(self._seq_plan):
            return
        seg = self._seq_plan[nxt]
        if self._step_i < int(seg["tick"]):
            return
        i0 = int(seg["tick"])
        frame = (self._seq_frames or {}).get(
            self.SEQ_FRAME_FAMILY[str(seg["mode"])])
        if frame is None:
            raise RuntimeError(
                "goal.mode_seq: canonical segment frames missing — "
                "neither reset() (_seq_capture_frames) nor the MJX "
                "choreography (MjxVecEnv._mint_seq_frames, the batched "
                "twin landed 08-14) minted them before the first "
                "switch. This is an invariant violation, not a "
                "missing-feature guard; see TRANSITIONS_DIRECTIVE.")
        # Old ABSOLUTE refs at the boundary (blend origin).
        g_old = self._goal_traj.at(self._step_i)
        old_abs_h = self._z0 + g_old.height_ref
        old_r, old_p = g_old.roll_ref, g_old.pitch_ref
        # Install the new segment family's canonical frame (lesson 5:
        # rise-after-lower must NOT aim at a stale frame — and the
        # trans-dagger2 lesson above: the frame must be the one the
        # specialists trained in, not the episode's start frame).
        self._z0 = float(frame["z0"])
        self._q_nom = frame["q_nom"].copy()
        self._pad_z_ref = frame["pad_z_ref"].copy()
        traj, h_target, ramp_i0 = self._seq_segment_traj(
            str(seg["mode"]), i0)
        # Blend window: refs continuous in ABSOLUTE terms across the
        # switch (engagement-snap lesson 6 — never hand the policy a
        # step-change reference at a control handoff).
        b = int(seg.get("blend", 0))
        if b > 0:
            n = len(traj.height)
            s = np.clip((np.arange(n) - i0 + 1.0) / float(b), 0.0, 1.0)
            s[:i0] = 0.0   # pre-switch region, never read again
            dh = old_abs_h - self._z0
            traj.height = (1.0 - s) * dh + s * np.asarray(traj.height,
                                                          dtype=float)
            traj.roll = (1.0 - s) * old_r + s * np.asarray(traj.roll,
                                                           dtype=float)
            traj.pitch = (1.0 - s) * old_p + s * np.asarray(traj.pitch,
                                                            dtype=float)
        self._goal_traj = traj
        self._seq_idx = nxt
        self._seq_seg_end = (int(self._seq_plan[nxt + 1]["tick"])
                             if nxt + 1 < len(self._seq_plan)
                             else int(self.episode_steps))
        self._seq_reset_mode_state(str(seg["mode"]), ramp_i0, h_target)

    def _seq_reset_mode_state(self, mode: str, ramp_i0: int,
                              h_target: float) -> None:
        """Re-derive the goal-derived per-episode bookkeeping for a new
        segment (the exact set _reset_finalize derives from the goal —
        milestones/ratchets restart so a segment can never inherit
        another segment's income baseline; SNAP_ATTRS lesson)."""
        self._h_target = float(h_target)
        self._h_milestones = set()
        self._prev_h_err_abs = 0.0
        self._score_best = None
        self._is_rise = mode == "rise"
        self._is_getup = False
        self._getup_best = None
        self._is_recover = False   # recover never occurs mid-sequence
        self._rec_phi_prev = None
        self._rec_hold_n = 0
        self._is_hold_bc = mode in ("hold", "track")
        self._is_lower_bc = mode == "lower"
        self._rise_ramp_i0 = int(ramp_i0)
        self._rsi_pending = False
        self._rsi_ref_tick0 = None
        self._end_posture_from = None
        self._curl_dist_prev = self._curl_dist()
        self._curl_milestones = set()
        # Hold/lower BC anchors mid-sequence use q_nom directly — which
        # the switch just re-based to the CANONICAL plant frame, i.e.
        # exactly the settled-plant base a fresh single-mode hold/lower
        # episode anchors at (and the frame the stance teachers were
        # trained against). The v1 "pose carried INTO the segment"
        # anchor existed to dodge the belly episode-q_nom trap; with
        # per-switch canonical frames that trap is gone and the carried
        # pose (mid-stride after a walk segment) would anchor WORSE
        # than the teachers' own base. Kept as an attr (always None)
        # for SNAP_ATTRS/pool compatibility.
        self._seq_pose_anchor = None
        self._walk_bc_gait = None
        self._walk_bc_t = 0.0
        if (mode == "walk"
                and float(cfg_get(self.cfg, "train", "bc_anchor_coef",
                                  default=0.0)) > 0.0
                and float(cfg_get(self.cfg, "train", "bc_anchor_walk",
                                  default=1.0)) > 0.0):
            self._walk_bc_gait = self._make_walk_bc_gait()

    def _step_finish(self, ctx):
        """Post-physics half of step: state read, reward, obs."""
        clipped, terminated, status, pen = ctx
        self._state = self._read_state()
        self._step_i += 1
        if getattr(self, "_seq_plan", None) is not None:
            self._seq_maybe_switch()
        goal = self._current_goal()
        h_err = None
        h_rel = float(self.data.xpos[self._chassis_bid, 2]) - self._z0
        # Optional walk-only collapse termination.  Tilt alone does not
        # catch a level chassis resting on its belly, which lets a seated
        # scoot survive and collect locomotion income for the full episode.
        # Keep this opt-in so every existing task/config remains bit-exact.
        walk_max_drop_mm = float(cfg_get(
            self.cfg, "safety", "walk_max_height_drop_mm", default=0.0))
        walk_height_grace_s = float(cfg_get(
            self.cfg, "safety", "walk_height_grace_s", default=0.0))
        if (not terminated and walk_max_drop_mm > 0.0
                and self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "walk"
                and self._step_i * self.dt >= walk_height_grace_s
                and h_rel < -walk_max_drop_mm * 0.001):
            terminated = True
            status.ok = False
            status.terminate = True
            status.reason = "walk_low_height"
        # Sustained-idle termination (2026-08-24, walkcurr park_duty-
        # class closure dig-in): every anti-park PRICE tried so far
        # (idle charge, park_duty, up to bank-legal 1.5x dose) left a
        # clean, non-colliding static stand as PPO's cheapest optimum
        # for the entire episode -- an absorbing state the stagea-
        # slip1 lesson says a soft price alone cannot evict (op ruling
        # 08-24: "absorbing states beat prices; must come WITH a
        # termination, never instead of one"). This is that boundary
        # for the FROZEN-POLICY-OUTPUT absorbing state, mirroring
        # walk_max_height_drop_mm's structure exactly: if mean
        # |joint velocity| across the 18 actuated joints (own EMA,
        # _walk_qvel_ema) stays below
        # safety.walk_idle_terminate_qvel_deg_s for
        # safety.walk_idle_terminate_s consecutive seconds (after an
        # initial grace window), the episode ends with
        # reward.walk_idle_terminate_penalty (falls back to the usual
        # reward.term_penalty if unset). Joint velocity, not body
        # speed, is the deliberate choice: a body-speed version also
        # flagged a real dragging/skating gait (near-zero BODY speed,
        # legs cycling hard) and genuine wrong-direction travel
        # (reverse/sideways -- near-zero ALONG-command speed, real
        # total speed) as "idle", cutting both short and robbing them
        # of the full-episode loadslip/heading charges the
        # WALKCURR_PF bank relies on to rank them below a plain park
        # -- measured to invert that ranking in bank probes. Mean
        # |qvel| cleanly separates a literally FROZEN policy output
        # (~5e-5 rad/s, pure settle jitter) from every other scripted
        # behavior (>=0.1 rad/s, ~35x higher) regardless of whether
        # the legs' motion translates into body progress, wrong-way
        # progress, or slip -- see the WALKCURR_PF_IDLE_TERM bank.
        # Default walk_idle_terminate_s=0.0 = off, bit-exact legacy
        # (no new state read, no behavior change) -- every existing
        # task/cfg is unaffected until a walkcurr rung explicitly arms
        # this.
        walk_idle_term_s = float(cfg_get(
            self.cfg, "safety", "walk_idle_terminate_s", default=0.0))
        if (not terminated and walk_idle_term_s > 0.0
                and self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "walk"):
            idle_grace_s = float(cfg_get(
                self.cfg, "safety", "walk_idle_terminate_grace_s",
                default=0.0))
            idle_floor = max(float(cfg_get(
                self.cfg, "safety", "walk_idle_terminate_qvel_deg_s",
                default=2.0)) * DEG2RAD, 1e-9)
            idle_tau = max(float(cfg_get(
                self.cfg, "safety", "walk_idle_terminate_tau_s",
                default=0.25)), self.dt)
            qvel_now = float(np.mean(np.abs(self.data.qvel[self._vadr])))
            self._walk_qvel_ema += (self.dt / idle_tau) * (
                qvel_now - self._walk_qvel_ema)
            if self._step_i * self.dt < idle_grace_s:
                self._walk_idle_low_s = 0.0
            else:
                if self._walk_qvel_ema < idle_floor:
                    self._walk_idle_low_s += self.dt
                else:
                    self._walk_idle_low_s = 0.0
                if self._walk_idle_low_s >= walk_idle_term_s:
                    terminated = True
                    status.ok = False
                    status.terminate = True
                    status.reason = "walk_idle_terminate"
        unload_f = None
        if goal is not None:
            # GETUP mode has no height reference at all: its staged
            # stand score (walk_task._post_step) replaces the height
            # kernel/shaping. Feeding h_err = h_rel here would CHARGE
            # standing up away from the settled spawn height — the
            # exact opposite of the task. RECOVER (08-15) has the
            # identical shape: its potential Phi prices height, and
            # the spawn-anchored h_err would charge every honest rise
            # (measured -58/ep on the reference rise replay).
            if not (getattr(self, "_is_getup", False)
                    or getattr(self, "_is_recover", False)):
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
            # Measured-load gate on hold income (2026-08-11, cfg
            # reward.hold_feet_load in [0,1], default 0 = legacy
            # exact). The crouchrise1/2/3 trio all converged on the
            # SAME hold cheat under this stack: two legs hover 1-19 mm
            # up — below foot_down_mm (20 mm), so the clearance count
            # above prices them as "down", and far below flag_leg_mm,
            # so the no-flag fade never fires — while the eval's
            # contact-duty clause (touch force > 0.5 N) reads them at
            # 0.01-0.04 duty. Clearance is the wrong proxy at the
            # bottom of its range; the gate must price MEASURED LOAD,
            # the same signal the gate metric uses. Per-foot
            #   s_i = max(clip(touch_N / hold_load_ref_n, 0, 1), floor)
            # multiplied over the six feet: an all-loaded stance keeps
            # exactly 1.0 (per-foot force >> 1 N at this robot's
            # weight), each unloaded foot costs a factor of
            # hold_load_floor (0.5 -> the two-leg hover earns 0.25, the
            # fade bank's "scraps, not a living" band), and the linear
            # ramp below ref gives partially-loaded feet a slope. The
            # holdstill1 zero-gradient lesson doesn't apply: this
            # plateau is only the ~2 mm to contact (crossed constantly
            # by DR + action noise), not a 50 mm climb, and the floor
            # keeps paid slope alive everywhere else.
            l_load = float(cfg_get(self.cfg, "reward", "hold_feet_load",
                                   default=0.0))
            if l_load > 0.0:
                f_ref = float(cfg_get(self.cfg, "reward",
                                      "hold_load_ref_n", default=1.0))
                floor_l = float(cfg_get(self.cfg, "reward",
                                        "hold_load_floor", default=0.5))
                s_feet = []
                for i in range(6):
                    if self._touch_adr[i] >= 0:
                        f_n = max(float(
                            self.data.sensordata[self._touch_adr[i]]),
                            0.0)
                        s_i = min(f_n / max(f_ref, 1e-6), 1.0)
                    else:   # no sensor: fall back to the clearance test
                        s_i = (1.0 if clear_h[i]
                               <= PLANT_SPEC["foot_down_mm"] * 0.001
                               else 0.0)
                    s_feet.append(s_i)
                # MIN-over-feet variant (2026-08-12, the pre-registered
                # anchormix1-r1 reopen lever — CURRENT_TRUTHS: "price
                # the min-over-feet load, not the product"). The
                # product with floor 0.5 caps ONE unloaded foot's tax
                # at x0.5 — and six straight stand runs (crouchrise1/2/
                # 3, holdload1, anchorstate1/2, anchormix1-r1) show the
                # habit sheds EXACTLY ONE foot: a five-foot stance at
                # half pay is "sufficient and cheaper", and supervision
                # only moves WHICH foot parks. With
                # reward.hold_feet_load_min=1 the WORST foot is the
                # whole factor: load = max(min_i s_i, min_floor), so a
                # single fully-unloaded foot cuts gated hold income to
                # hold_load_min_floor (default 0.1 — scraps), with the
                # same linear on-ramp below ref for slope. An
                # all-loaded stance keeps exactly 1.0 either way.
                # Default 0 = the legacy product path, bit-exact.
                min_w = float(cfg_get(self.cfg, "reward",
                                      "hold_feet_load_min", default=0.0))
                if min_w > 0.0:
                    floor_min = float(cfg_get(
                        self.cfg, "reward", "hold_load_min_floor",
                        default=0.1))
                    load_h = max(min(s_feet), floor_min)
                else:
                    load_h = 1.0
                    for s_i in s_feet:
                        load_h *= max(s_i, floor_l)
                feet_h *= (1.0 - l_load) + l_load * load_h
                parts["hold_load_factor"] = load_h
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
        # Transition foot-drag charge (operator 08-11 night: stand/sit
        # scrape their feet across the floor and NOTHING outside walk
        # mode priced it — k_drag_loaded/k_drag_stance live in the
        # walk-tick block only). reward.k_drag_trans charges loaded
        # foot-XY translation (−k per meter, per-foot 0.5 mm/tick
        # deadband, walk's k_drag_loaded convention) on every NON-walk
        # tick: rise, lower, raise, hold, track, lean, unload, quad.
        # A pivoting/sliding loaded foot pays; a foot that LIFTS and
        # steps is never charged — the honest fix is stepping, exactly
        # what the hardware needs. Charged incrementally beyond a
        # per-EPISODE allowance (k_drag_stance's telescoping form: a
        # foot that never lifts cannot defer payment). Allowances are
        # measured, not guessed (probe 08-11): the demonstrated
        # belly->plant rise inherently slides its pads 463 mm during
        # the curl at the old 128 mm tibia (0.55 default rise
        # allowance kept the honest reference free). RE-MEASURED
        # 2026-08-22 at the corrected 150 mm tibia geometry (the
        # longer leg drags its pads further through the same curl):
        # ~656 mm, so the allowance is raised to 0.75 with the same
        # proportional headroom the original measurement carried.
        # The honest lower is anchored-feet by
        # construction and the quiet stand measures 0.0 — both charge
        # from the first excess millimeter. Default k 0 = byte-exact
        # legacy. The trans_drag_mm metric is emitted whenever the
        # axis is measured (charge on or off) so evals and W&B can
        # watch the dragging without coupling metric to price.
        mode_td = (getattr(self._goal_traj, "mode", "")
                   if self._goal_traj is not None else "")
        if mode_td and mode_td != "walk":
            k_td = float(cfg_get(self.cfg, "reward", "k_drag_trans",
                                 default=0.0))
            drag_td = 0.0
            for f_td in range(6):
                adr_td = self._touch_adr[f_td]
                on_td = (adr_td >= 0 and
                         float(self.data.sensordata[adr_td]) > 0.5)
                xy_td = self.data.xpos[self._pad_bids[f_td], :2]
                if (on_td and self._tdrag_prev_on[f_td]
                        and self._tdrag_prev_xy[f_td] is not None):
                    slip_td = float(np.linalg.norm(
                        xy_td - self._tdrag_prev_xy[f_td]))
                    if slip_td > 0.0005:
                        drag_td += slip_td
                self._tdrag_prev_xy[f_td] = xy_td.copy()
                self._tdrag_prev_on[f_td] = on_td
            parts["trans_drag_mm"] = drag_td * 1000.0
            if k_td > 0.0 and drag_td > 0.0:
                if mode_td in ("rise", "raise"):
                    allow_td = float(cfg_get(
                        self.cfg, "reward", "drag_trans_allow_rise_m",
                        default=0.75))
                else:
                    allow_td = float(cfg_get(
                        self.cfg, "reward", "drag_trans_allow_m",
                        default=0.0))
                acc0_td = self._tdrag_acc
                self._tdrag_acc = acc0_td + drag_td
                r_td = -k_td * (max(self._tdrag_acc - allow_td, 0.0)
                                - max(acc0_td - allow_td, 0.0))
                if r_td != 0.0:
                    reward += r_td
                parts["reward_drag_trans"] = r_td
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
                # Mode-seq segments end at the next switch, not the
                # episode end — clamp the charge window to the ACTIVE
                # segment (None outside mode_seq = legacy exact).
                _ep_end = int(getattr(self, "_seq_seg_end", None)
                              or self.episode_steps)
                self._end_posture_from = max(
                    start + int(round(grace_s / self.dt)),
                    _ep_end - int(round(win_s / self.dt)))
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
            # Early-fall horizon cost — same key/semantics as the
            # _step_begin site (reward.term_cost_per_remaining_s,
            # default 0.0 = legacy bit-exact); only safety
            # terminations are charged, never time-limit truncation.
            k_rem = float(cfg_get(self.cfg, "reward",
                                  "term_cost_per_remaining_s",
                                  default=0.0))
            if k_rem > 0.0:
                rem_cost = k_rem * max(self._active_episode_steps()
                                       - self._step_i,
                                       0) * self.dt
                # reward.term_cost_max: same bounded-terminal-cost
                # semantics as the _step_begin site above (08-17,
                # fb_20260817T005114 item 5); default 0 = off.
                cap = float(cfg_get(self.cfg, "reward",
                                    "term_cost_max", default=0.0))
                if cap > 0.0:
                    rem_cost = min(rem_cost, cap)
                pen += rem_cost
            parts["reward_termination"] = -pen
            reward -= pen
        truncated = self._step_i >= self._active_episode_steps()
        self._prev_action = clipped.copy()
        info = {"termination_reason": status.reason, **parts,
                "safety_ok": status.ok,
                "roll_deg": self._state.imu_roll * RAD2DEG,
                "pitch_deg": self._state.imu_pitch * RAD2DEG,
                "roll_rel_deg": (self._state.imu_roll
                                 - self._tilt_ref0[0]) * RAD2DEG}
        if self._sched_value is not None:
            # Live scheduled-coefficient value — auto-logged to W&B as
            # env/sched_value by the trainers' info-scalar sweep, so
            # triage can see WHERE on the ramp a behavior change lands.
            info["sched_value"] = float(self._sched_value)
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
        _bc_coef = float(cfg_get(self.cfg, "train", "bc_anchor_coef",
                                 default=0.0))
        if (self._is_rise and getattr(self, "n_act", 0) == N_JOINTS
                and _bc_coef > 0.0):
            _bc_ref_path = cfg_get(self.cfg, "reward", "rise_ref_path",
                                   default=None)
            if _bc_ref_path:
                from .joint_task import q_rad_to_action
                _bc_ref = load_rise_ref(str(_bc_ref_path))
                # STATE-ALIGNED indexing (2026-08-11, cfg
                # train.bc_anchor_state_aligned, default 0 = legacy
                # clock-exact). The crouchrise1/2/3 + holdload1 chain
                # isolated the anchor as the sole remaining suspect for
                # the legs-1+4 hover-park: holdload1 kept the park at a
                # measured 4x hold-income loss, so the pose is TAUGHT,
                # not paid for. Mechanism: non-RSI crouch starts
                # time-align the clock at the BELLY ramp start
                # (_rise_ref_clock), so a robot sitting plant-adjacent
                # is supervised toward early-path lifted-leg poses —
                # and pi generalizes that obs->action association into
                # hold. State-aligned mode re-indexes every tick by
                # nearest reference pose to the CURRENT joints (the
                # identical RMS nearest-neighbor _reset_finalize
                # already trusts for RSI spawns): plant-adjacent
                # states can then only ever anchor toward the path's
                # planted tail, sagged states get "climb from where
                # you ARE", and the target stays one ref-tick ahead as
                # before. Reads only current qpos + the cached ref —
                # no per-episode state, pool-restore safe. The
                # rise-ref TRACKING REWARD keeps the shared clock
                # (semantics banks untouched; its income is
                # grounded-feet-gated so it cannot fund flag poses).
                if float(cfg_get(self.cfg, "train",
                                 "bc_anchor_state_aligned",
                                 default=0.0)) > 0.0:
                    _bc_qnow = np.asarray(
                        self.data.qpos[self._qadr], dtype=float)
                    _bc_j = int(np.argmin(
                        ((_bc_ref["q"] - _bc_qnow[None, :]) ** 2)
                        .mean(axis=1)))
                    # Pursuit lookahead: "one tick ahead of where you
                    # ARE" stalls (the servo never fully converges
                    # within a tick, so the matched index crawls —
                    # measured 1 ref tick per 60 chained steps); the
                    # lookahead must exceed the tracking lag. The
                    # legacy clock never needed this because it
                    # advances on its own.
                    _bc_ahead = max(int(round(float(cfg_get(
                        self.cfg, "train", "bc_anchor_lookahead_s",
                        default=0.25)) / _bc_ref["dt"])), 1)
                    # HEIGHT-FLOOR pursuit (08-12, cw-stand-footlow1
                    # dig-in / probe_anchor_align): a TIME lookahead
                    # degenerates to a near-zero POSE lookahead inside
                    # the reference's low prep segment — this ref
                    # spends 5+ s (ticks ~126-250) crawling 0->25 mm,
                    # so at a stalled ~7 mm belly state the +0.5 s
                    # target commands a pose only 1-5 mm higher and
                    # the loaded-servo tracking sag (~0.3 s settle)
                    # cancels it: the matched index PINS (measured:
                    # j=128-133, 0 ticks advance over 3 s) while the
                    # anchor loss reads low/converged — the anchor
                    # actively supervises the stall. When
                    # train.bc_anchor_min_h_ahead_mm > 0, additionally
                    # require the target tick to command at least that
                    # many mm above the chassis's CURRENT height
                    # (first such tick at/after the match; path end if
                    # none) — in flat segments the pursuit skips ahead
                    # to where the reference genuinely climbs, in
                    # steep segments the time lookahead already
                    # satisfies it and nothing changes. Default 0 =
                    # off, bit-exact. Needs ref["h"] (newer extracts;
                    # absent -> floor is a no-op, time lookahead
                    # unchanged). Stateless — pool-restore safe.
                    _bc_min_h = float(cfg_get(
                        self.cfg, "train", "bc_anchor_min_h_ahead_mm",
                        default=0.0))
                    if _bc_min_h > 0.0 and "h" in _bc_ref:
                        _bc_hnow = float(self.data.xpos[
                            self._chassis_bid, 2]) - self._z0
                        _bc_ks = np.flatnonzero(
                            _bc_ref["h"][_bc_j:]
                            >= _bc_hnow + _bc_min_h * 1e-3)
                        _bc_floor = (_bc_j + int(_bc_ks[0])
                                     if len(_bc_ks)
                                     else len(_bc_ref["q"]) - 1)
                        _bc_ahead = max(_bc_ahead, _bc_floor - _bc_j)
                else:
                    _bc_j, _ = self._rise_ref_clock(_bc_ref)
                    _bc_ahead = max(
                        int(round(self.dt / _bc_ref["dt"])), 1)
                _bc_jn = min(_bc_j + _bc_ahead, len(_bc_ref["q"]) - 1)
                info["bc_target"] = q_rad_to_action(
                    _bc_ref["q"][_bc_jn]).astype(np.float32)
                info["bc_mode"] = 0    # rise (stratified sampling tag)
        # GETUP BC-anchor target (08-12, cw-getup2-r1 follow-up —
        # RL_PLAN queue; see bc_anchor.py header for the full story).
        # Warm-starting getup from the rise+hold specialist was not
        # enough: env/getup_S declined over training back toward the
        # from-scratch collapse, so the specialist's skill needs an
        # explicit pull, same as rise once needed. Cfg-gated by
        # train.bc_anchor_getup (default 0 = off, bit-exact). Reuses
        # the rise reference demo but ALWAYS state-aligned (nearest
        # reference pose to CURRENT joints) — getup starts are
        # arbitrary (belly/tangle/crouch/park/...), so there is no
        # live clock to time-align a fixed-index target to; a
        # clock-exact target here would repeat the exact
        # plant-adjacent-supervised-toward-early-path mistake the rise
        # lever's state-aligned mode was built to fix.
        elif (getattr(self, "_is_getup", False)
                and getattr(self, "n_act", 0) == N_JOINTS
                and _bc_coef > 0.0
                and float(cfg_get(self.cfg, "train", "bc_anchor_getup",
                                  default=0.0)) > 0.0):
            _bc_ref_path = cfg_get(self.cfg, "reward", "rise_ref_path",
                                   default=None)
            if _bc_ref_path:
                from .joint_task import q_rad_to_action
                _bc_ref = load_rise_ref(str(_bc_ref_path))
                _bc_qnow = np.asarray(
                    self.data.qpos[self._qadr], dtype=float)
                _bc_j = int(np.argmin(
                    ((_bc_ref["q"] - _bc_qnow[None, :]) ** 2)
                    .mean(axis=1)))
                _bc_ahead = max(int(round(float(cfg_get(
                    self.cfg, "train", "bc_anchor_lookahead_s",
                    default=0.25)) / _bc_ref["dt"])), 1)
                _bc_jn = min(_bc_j + _bc_ahead, len(_bc_ref["q"]) - 1)
                info["bc_target"] = q_rad_to_action(
                    _bc_ref["q"][_bc_jn]).astype(np.float32)
                info["bc_mode"] = 4    # getup
        # RECOVER BC-anchor target (08-15 directive: "preserve the
        # explicit state-aligned getup BC anchor that made cw-getup3
        # work; apply it on MASTERED rise/plant states so the
        # inherited skill cannot decay", with matching conditioned on
        # orientation/height/contact — not nearest-q alone). Same
        # nearest-q + lookahead emit as getup, but ELIGIBILITY-GATED:
        # the target only fires when the body is upright-ish (true
        # tilt <= train.bc_anchor_recover_tilt_deg), at/below plant
        # height (no stilt supervision), and with real ground reaction
        # through the feet — a side/back/flipped robot is never pulled
        # toward rise poses it cannot reach from there. Cfg-gated by
        # train.bc_anchor_recover (default 0 = off, bit-exact).
        elif (getattr(self, "_is_recover", False)
                and getattr(self, "n_act", 0) == N_JOINTS
                and _bc_coef > 0.0
                and float(cfg_get(self.cfg, "train",
                                  "bc_anchor_recover",
                                  default=0.0)) > 0.0):
            info["recover_bc_eligible"] = 0.0
            _bc_ref_path = cfg_get(self.cfg, "reward", "rise_ref_path",
                                   default=None)
            if _bc_ref_path:
                _r, _p = self._true_roll_pitch()
                _tilt = max(abs(_r), abs(_p)) * 180.0 / math.pi
                _tilt_max = float(cfg_get(
                    self.cfg, "train", "bc_anchor_recover_tilt_deg",
                    default=25.0))
                _touch_n = 0.0
                for _f in range(6):
                    _adr = self._touch_adr[_f]
                    if _adr >= 0:
                        _touch_n += max(
                            float(self.data.sensordata[_adr]), 0.0)
                _z_now = float(self.data.xpos[self._chassis_bid, 2])
                _z_pl, _ = self._getup_geom()
                if (_tilt <= _tilt_max and _touch_n >= 0.5
                        and _z_now <= _z_pl + 0.02):
                    info["recover_bc_eligible"] = 1.0
                    from .joint_task import q_rad_to_action
                    _bc_ref = load_rise_ref(str(_bc_ref_path))
                    _bc_qnow = np.asarray(
                        self.data.qpos[self._qadr], dtype=float)
                    _bc_dist = ((_bc_ref["q"] - _bc_qnow[None, :]) ** 2
                                ).mean(axis=1)
                    # Recover starts span belly to plant height.  Nearest-q
                    # alone was known to match a parked near-plant leg to a
                    # low, slow part of the rise reference.  Restrict the
                    # pose match to reference frames near the current
                    # absolute belly->plant height whenever h_rel_m exists;
                    # upright/contact eligibility above supplies the other
                    # two state dimensions from the directive.
                    _bc_hnow = None
                    if "h" in _bc_ref:
                        _z_belly = float(cfg_get(
                            self.cfg, "reward", "getup_z_belly_mm",
                            default=38.0)) * 1e-3
                        _bc_hnow = max(_z_now - _z_belly, 0.0)
                        _h_tol = float(cfg_get(
                            self.cfg, "train",
                            "bc_anchor_recover_height_match_mm",
                            default=25.0)) * 1e-3
                        _height_rows = np.flatnonzero(
                            np.abs(_bc_ref["h"] - _bc_hnow) <= _h_tol)
                    else:
                        _height_rows = np.empty(0, dtype=int)
                    if len(_height_rows):
                        _bc_j = int(_height_rows[
                            np.argmin(_bc_dist[_height_rows])])
                    else:
                        _bc_j = int(np.argmin(_bc_dist))
                    _bc_ahead = max(int(round(float(cfg_get(
                        self.cfg, "train", "bc_anchor_lookahead_s",
                        default=0.25)) / _bc_ref["dt"])), 1)
                    # Carry the proven footlow2 height-floor pursuit into
                    # recovery.  Use absolute height above the belly datum,
                    # not height above this episode's spawn (_z0): a
                    # near-goal recovery starts are already near standing.
                    _bc_min_h = float(cfg_get(
                        self.cfg, "train", "bc_anchor_min_h_ahead_mm",
                        default=0.0))
                    if (_bc_min_h > 0.0 and "h" in _bc_ref
                            and _bc_hnow is not None):
                        _bc_ks = np.flatnonzero(
                            _bc_ref["h"][_bc_j:]
                            >= _bc_hnow + _bc_min_h * 1e-3)
                        _bc_floor = (_bc_j + int(_bc_ks[0])
                                     if len(_bc_ks)
                                     else len(_bc_ref["q"]) - 1)
                        _bc_ahead = max(_bc_ahead, _bc_floor - _bc_j)
                    _bc_jn = min(_bc_j + _bc_ahead,
                                 len(_bc_ref["q"]) - 1)
                    info["bc_target"] = q_rad_to_action(
                        _bc_ref["q"][_bc_jn]).astype(np.float32)
                    info["bc_mode"] = 6    # recover
                    info["recover_bc_ref_index"] = float(_bc_j)
                    info["recover_bc_target_index"] = float(_bc_jn)
        # HOLD/TRACK BC-anchor target (RL_PLAN queue 2.3, 08-11): the
        # rise lever repeated after two hold pricing misses (hard zero,
        # then a linear fade) neither reached a quiet plant. Hold/track
        # have no moving reference to chase, so the target is simply
        # the pose the episode actually settled at — self._q_nom,
        # already captured post-settle in _reset_finalize for the
        # hold-current reward term ("trivially available", RISE.md)
        # and already in mjx_host.SNAP_ATTRS. Constant for the whole
        # episode: this literally IS "stand still right here".
        elif (self._is_hold_bc and getattr(self, "n_act", 0) == N_JOINTS
                and _bc_coef > 0.0):
            from .joint_task import q_rad_to_action
            # Mode-seq hold segments anchor at the pose carried INTO
            # the segment (_seq_pose_anchor, captured at the switch);
            # None outside mode_seq = the legacy settled q_nom.
            _q_hold_base = (self._q_nom
                            if getattr(self, "_seq_pose_anchor", None)
                            is None else self._seq_pose_anchor)
            _q_tgt = _q_hold_base
            # TIP-AWARE HOLD REFERENCE (08-13, train.bc_anchor_tilt_comp,
            # default 0 = legacy constant-q_nom target, bit-exact).
            # cw-stand-footlow2-tip1's gate consequence: tipped-start DR
            # with a tilt-BLIND anchor taught the policy to HOLD the
            # lean (target = q_nom regardless of attitude gives zero
            # leveling gradient; joint-space MSE is attitude-blind), and
            # the hardware candidate stands with a persistent ~8deg
            # lean. When enabled, HOLD episodes (not track — track
            # commands attitude goals the compensation would fight)
            # anchor toward the pose that COUNTER-ROTATES the measured
            # lean: FixedFootBodyIK from q_nom with
            # BodyOffset(roll/pitch = -comp * rel_attitude), i.e. a
            # proportional posture-feedback teacher. rel attitude is
            # measured against the episode tilt reference exactly like
            # the tipped-start recovery metric (tipped episodes keep the
            # ref LEVEL, so rel ~= true lean; mount-bias/slope stays
            # inside the ref). Soft deadband keeps the target continuous
            # and leaves settled-level ticks anchored at q_nom; the
            # commanded correction is clipped for IK safety. Solved
            # fresh per tick from SNAP_ATTRS state only (_q_nom,
            # _tilt_ref0, _state) — pool-restore safe, same pattern as
            # the lower anchor. imu roll/pitch and BodyOffset roll/pitch
            # share the same axis convention (rot_x/rot_y; verified in
            # test_bc_anchor.py::test_tilt_comp_counter_rotates).
            _tc = float(cfg_get(self.cfg, "train", "bc_anchor_tilt_comp",
                                default=0.0))
            if (_tc > 0.0 and self._goal_traj is not None
                    and self._goal_traj.mode == "hold"
                    and self._q_nom is not None):
                _dead = float(cfg_get(
                    self.cfg, "train", "bc_anchor_tilt_deadband_deg",
                    default=1.5)) * DEG2RAD
                # Cap default 6.0: measured expressibility boundary —
                # the counter-rotated pose from a settled hold stance
                # round-trips the [-1,1] action space EXACTLY up to 6
                # deg and saturates a joint bound from 7 deg (the
                # target must be a pose the policy can actually
                # command; a clipped target supervises garbage on the
                # saturated joints).
                _maxc = float(cfg_get(
                    self.cfg, "train", "bc_anchor_tilt_max_deg",
                    default=6.0)) * DEG2RAD

                def _soft(x: float) -> float:
                    return math.copysign(max(abs(x) - _dead, 0.0), x)

                # Comp source (train.bc_anchor_tilt_from_settle,
                # default 0 = the original current-lean proportional
                # source, bit-exact). The proportional source is a
                # P-controller with a closed-loop fixed point at
                # (L0+deadband)/2 — as the student levels, the
                # commanded correction SHRINKS below what leveling
                # needs (probe_tilt_teacher, 08-13: a perfect student
                # settles 3.95deg from 6.5deg spawns vs the 3deg gate
                # bar; predicted 3.98). The settle-lean source uses the
                # episode's post-settle lean (_settle_lean, a
                # per-episode constant in SNAP_ATTRS): the ideal
                # student levels to the deadband (or the cap-limited
                # residual), where the income Gaussian regains
                # gradient and RL can finish the job.
                if float(cfg_get(self.cfg, "train",
                                 "bc_anchor_tilt_from_settle",
                                 default=0.0)) > 0.0:
                    _er = _soft(self._settle_lean[0])
                    _ep_ = _soft(self._settle_lean[1])
                else:
                    _er = _soft(self._state.imu_roll
                                - self._tilt_ref0[0])
                    _ep_ = _soft(self._state.imu_pitch
                                 - self._tilt_ref0[1])
                if _er != 0.0 or _ep_ != 0.0:
                    from rl_move.body_ik import BodyOffset, FixedFootBodyIK
                    _cr = float(np.clip(-_tc * _er, -_maxc, _maxc))
                    _cp = float(np.clip(-_tc * _ep_, -_maxc, _maxc))
                    _ik = FixedFootBodyIK()
                    _ik.reset(_q_hold_base)
                    # Halving retry: a correction the stance geometry
                    # can't reach degrades to a smaller one instead of
                    # silently reverting to the tilt-blind target (the
                    # 12deg default cap was IK-infeasible from the
                    # settled hold stance — caught by the clip test).
                    for _s in (1.0, 0.5, 0.25):
                        _res = _ik.solve(BodyOffset(
                            roll=_cr * _s, pitch=_cp * _s))
                        if _res.ok:
                            _q_tgt = _res.q_rad
                            break
            info["bc_target"] = q_rad_to_action(
                _q_tgt).astype(np.float32)
            info["bc_mode"] = 1        # hold/track
        # LOWER BC-anchor target (08-11, cfg train.bc_anchor_lower,
        # default 0 = legacy no-emission). The lower bank's strict
        # xfail pins the pricing gap (one-leg-aloft keeps ~85% of
        # honest income) and prescribes "strengthen the pricing (or
        # BC-anchor lower ticks)"; six runs showed pricing never moves
        # the behavior while anchor supervision does. The target is
        # the bank's own honest demonstration: the FixedFootBodyIK
        # descent — all six feet anchored at the SETTLED stance
        # (_q_nom, post-settle, already in SNAP_ATTRS), body tracking
        # the commanded height one tick ahead. Solved fresh per tick
        # (~5 us, analytic) from snapped state only — pool-restore
        # safe by construction, no per-episode IK object to snapshot.
        elif (getattr(self, "_is_lower_bc", False)
                and getattr(self, "n_act", 0) == N_JOINTS
                and _bc_coef > 0.0
                and float(cfg_get(self.cfg, "train", "bc_anchor_lower",
                                  default=0.0)) > 0.0
                and self._q_nom is not None
                and self._goal_traj is not None):
            from rl_move.body_ik import BodyOffset, FixedFootBodyIK
            from .joint_task import q_rad_to_action
            # _step_i was already incremented at the top of
            # _step_finish; the next commanded tick is _step_i + 1
            # (same one-tick-ahead convention as the rise clock).
            _g_next = self._goal_traj.at(self._step_i + 1)
            _ik = FixedFootBodyIK()
            # Mode-seq lower segments descend from the stance carried
            # INTO the segment; None outside mode_seq = legacy q_nom.
            _ik.reset(self._q_nom
                      if getattr(self, "_seq_pose_anchor", None) is None
                      else self._seq_pose_anchor)
            _res = _ik.solve(BodyOffset(
                height=float(_g_next.height_ref)))
            if _res.ok:
                info["bc_target"] = q_rad_to_action(
                    _res.q_rad).astype(np.float32)
                info["bc_mode"] = 2    # lower
        # WALK BC-anchor target (08-11, probe_walk_income follow-up):
        # the scripted TripodGait's joint pose one control tick ahead,
        # driven by the LIVE blended command (vx/vy/wz ref) — so the
        # target is command-conditioned in every direction, points at
        # a plant-hold on genuine stop segments (set_velocity(0,0)
        # holds the stance), and supplies the per-leg "step this way"
        # gradient the reward provably cannot (degenerates earn below
        # freeze yet PPO still found them). Reward stack untouched.
        elif (getattr(self, "_walk_bc_gait", None) is not None
                and getattr(self, "n_act", 0) == N_JOINTS
                and _bc_coef > 0.0):
            _bc_goal = self._current_goal()
            # ONLY on commanded ticks: TripodGait at zero velocity
            # marches in place (the bank's "stall" policy), so a stop
            # segment must get NO gait supervision — standing still is
            # the commanded behavior there and the kernel already pays
            # it (walk_kernel freeze income at s_ref=0 is legitimate).
            if _bc_goal is not None:
                _bc_wz = float(getattr(_bc_goal, "wz_ref", 0.0) or 0.0)
                _bc_cmd = (math.hypot(_bc_goal.vx_ref, _bc_goal.vy_ref)
                           > 1e-3 or abs(_bc_wz) > 1e-3)
                if _bc_cmd:
                    from .joint_task import q_rad_to_action
                    _g = self._walk_bc_gait
                    _g.set_velocity(vx=float(_bc_goal.vx_ref),
                                    vy=float(_bc_goal.vy_ref),
                                    omega=_bc_wz)
                    # _step_i was already incremented at the top of
                    # _step_finish: it IS the next pre-step tick index,
                    # so the next scripted action is desired_deg at
                    # _step_i * dt (the bank rollouts command
                    # desired_deg(step*dt) at pre-step tick `step`).
                    _t_bc = self._step_i * self.dt
                    if float(cfg_get(self.cfg, "train",
                                     "bc_anchor_phase_lock",
                                     default=0.0)) > 0.0:
                        # PHASE-LOCKED anchor clock (08-22, operator
                        # reward-alignment order fb_20260822T032514,
                        # phasedir2 line): with goal.walk_phase_obs=1
                        # the POLICY's clock advances only while a
                        # linear velocity is commanded
                        # (walk_task._augment_obs), and the phase BC
                        # clone was distilled against exactly that
                        # clock (bc_init_gait unwraps the phase obs to
                        # drive the teacher). The legacy wall-clock
                        # time above jumps the gait phase across every
                        # settle hold / stop segment (~0.33 cycle for
                        # the standard 1 s spawn hold), so the anchor
                        # would pull TOWARD A DIFFERENTLY-PHASED GAIT
                        # than the clock the policy sees. This
                        # accumulator advances by dt on exactly the
                        # ticks the obs clock advances (s_ref > 1e-3;
                        # a wz-only commanded tick keeps the clock —
                        # and the gait phase — frozen, matching the
                        # obs clock's linear-command gate). Default 0
                        # = legacy wall-clock, bit-exact.
                        if math.hypot(_bc_goal.vx_ref,
                                      _bc_goal.vy_ref) > 1e-3:
                            _dt_bc = self.dt
                            # Speed-coupled clock (08-22, amp M2
                            # speedrange root cause): when
                            # goal.walk_phase_speed_scale>0 the obs
                            # clock in walk_task._augment_obs runs at
                            # hz_eff, not hz_base — scale the anchor
                            # accumulator by the same ratio so the
                            # anchor gait stays phase-locked to the
                            # clock the policy sees. Default 0 =
                            # legacy, bit-exact.
                            _k_coup = float(cfg_get(
                                self.cfg, "goal",
                                "walk_phase_speed_scale", default=0.0))
                            if _k_coup > 0.0:
                                from rl_move.sim.walk_task import (
                                    phase_hz_effective,
                                    PHASE_HZ_DEFAULT,
                                    PHASE_SPEED_NOM_DEFAULT)
                                _hz0 = float(cfg_get(
                                    self.cfg, "goal", "walk_phase_hz",
                                    default=PHASE_HZ_DEFAULT))
                                _hz_eff = phase_hz_effective(
                                    _hz0,
                                    math.hypot(_bc_goal.vx_ref,
                                               _bc_goal.vy_ref),
                                    _k_coup,
                                    s_nom=float(cfg_get(
                                        self.cfg, "goal",
                                        "walk_phase_speed_nom",
                                        default=PHASE_SPEED_NOM_DEFAULT)),
                                    hz_max=float(cfg_get(
                                        self.cfg, "goal",
                                        "walk_phase_hz_max",
                                        default=0.0)))
                                if _hz0 > 0.0:
                                    _dt_bc = self.dt * (_hz_eff / _hz0)
                            self._walk_bc_t += _dt_bc
                        _t_bc = self._walk_bc_t
                    _q_bc = np.asarray(_g.desired_deg(_t_bc)) * DEG2RAD
                    info["bc_target"] = q_rad_to_action(
                        _q_bc).astype(np.float32)
                    info["bc_mode"] = 3    # walk
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
            if h_err is not None:   # getup mode has no height ref
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
