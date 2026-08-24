"""Host-side pieces of the MJX vec envs, importable WITHOUT torch/jax.

Shared by ``mjx_vec_env.MjxVecEnv`` (in-process) and the worker
processes of ``mjx_sharded_vec_env.MjxShardedVecEnv`` — workers must
run the per-env Python halves (safety/IK/reward/obs) without paying a
torch or jax import, so everything here depends only on numpy + mujoco
+ the sim env code.
"""
from __future__ import annotations

import copy

import numpy as np

from .mjx_backend import MODEL_DR_FIELDS
from .servo_model import (
    SimServoParams, apply_params_to_model, build_model,
    position_actuator_ids, resolve_model_source,
)
from .sim_env import (leg_chassis_collision_from_cfg,
                      set_foot_ground_friction, soften_contacts)
from .struct_compliance import StructCompliance

DEG2RAD = np.pi / 180.0

# Host attributes that constitute an env's per-episode state — everything
# reset() establishes that step() later reads. Pooled resets snapshot /
# restore exactly these (plus the task class's MJX_SNAPSHOT_EXTRA).
# ``safety`` and ``ik`` are stateful objects (estop latches, tilt refs,
# foot anchors) and are deep-copied whole.
SNAP_ATTRS = (
    "_ep_rand", "_goal_traj", "_q_nom", "_cmd", "_prev_action", "_step_i",
    "_state", "_att_rp", "_cur_filt", "_tilt_ref0", "_settle_lean",
    "_z0", "_pad_z_ref",
    "_h_target", "_h_milestones", "_prev_h_err_abs", "_is_rise",
    "_plant_feet_xy", "_curl_dist_prev", "_curl_milestones",
    "_imu_prev_v", "_imu_f_accum", "_imu_f_n", "_gyro_accum", "_gyro_n",
    "_hist_buf", "safety", "ik",
    # Score-stack + RSI episode state (08-11). These were MISSING from
    # the list for every warp run of the 08-10/11 stand campaign:
    # pool-restored episodes inherited another episode's ratchet
    # high-water mark (_score_best) and ramp anchor, so score income
    # silently stopped paying as pooled episodes took over — measured
    # as the "warm start pays, then decays over ~20-30 updates"
    # erosion in score1/scoreref1/-dr0/-lowlr/-riseonly/rsi1. Any new
    # per-episode attr set in _reset_begin/_reset_finalize and read in
    # the step path MUST be added here.
    "_score_best", "_rise_ramp_i0", "_end_posture_from",
    "_rsi_pending", "_rsi_ref_tick0",
    # HOLD/TRACK BC-anchor eligibility (08-11, RL_PLAN queue 2.3): set
    # in _reset_finalize, read every step-path tick — the exact class
    # of attr this list exists to catch (see comment above).
    "_is_hold_bc",
    "_is_lower_bc",
    # WALK BC-anchor gait reference (08-11, queue 2.1 follow-up): a
    # per-episode TripodGait instance carrying phase/velocity state,
    # created in the reset path, read every walk tick.
    "_walk_bc_gait",
    # Command-gated anchor clock (08-22, train.bc_anchor_phase_lock):
    # accumulated commanded-tick seconds driving the anchor gait so it
    # stays locked to the goal.walk_phase_obs clock the policy sees.
    "_walk_bc_t",
    # Transition-drag bookkeeping (08-11 night, reward.k_drag_trans):
    # per-foot prev contact + XY plus the episode drag accumulator,
    # read every non-walk tick.
    "_tdrag_prev_xy", "_tdrag_prev_on", "_tdrag_acc",
    # GETUP (recover→stand→walk) episode state (08-11): mode flag +
    # the staged-progress ratchet baseline, set in _reset_finalize and
    # read every getup tick in walk_task._post_step. A pool-restored
    # episode inheriting another episode's ratchet is exactly the
    # commit-65edba7 bug class this list exists to prevent.
    "_is_getup", "_getup_best",
    # RECOVER (recover_to_plant, 08-15) episode state: mode flag, the
    # PBRS previous-potential baseline, and the continuous success-hold
    # counter. A pool-restored episode inheriting another episode's
    # potential baseline or hold streak is exactly the commit-65edba7
    # bug class this list exists to prevent. (The adaptive-curriculum
    # stats dict `_rec_stats` is deliberately NOT here — it is
    # cross-episode learning state, like _lp_weights.)
    "_is_recover", "_rec_phi_prev", "_rec_hold_n",
    "_rec_reset_height_mm", "_rec_reset_tilt_deg",
    "_rec_reset_min_load_n", "_rec_reset_pad_spread_mm",
    # Mode-sequencing episode state (goal.mode_seq, TRANSITIONS_
    # DIRECTIVE item 1, 08-13): the segment plan/index, the stand-
    # height anchor mid-sequence rises aim at, the active segment's
    # end tick (end-posture window clamp) and the hold/lower BC base
    # pose captured at each switch. Set in the reset path AND at
    # mid-episode switches, read every tick — the exact attr class
    # this list exists for.
    "_seq_plan", "_seq_idx", "_seq_stand_z", "_seq_seg_end",
    "_seq_pose_anchor",
    # Canonical segment frames (trans-dagger2 fix, 08-14): the settled
    # plant/belly q_nom/_z0/pad-z refs the switch installs per segment.
    # Under DR they are minted per episode — a pool-restored episode
    # must carry ITS model's frames, not a later draw's.
    "_seq_frames",
    # Physics easing (08-13): the episode's baked ease multipliers,
    # set in _reset_begin next to the _ep_rand they scaled and emitted
    # in step info — a pool-restored episode must report the ease
    # values its own DR rows were minted with, not a later episode's.
    "_ease_g", "_ease_v",
    # Structural compliance (08-21): sampled per-episode stiffness vector.
    # The model rows and host-side encoder correction must agree after a
    # pooled restore.
    "_struct_comp_k",
)


def snap_attrs_for(task_cls) -> tuple:
    return SNAP_ATTRS + tuple(getattr(task_cls, "MJX_SNAPSHOT_EXTRA", ()))


def snapshot_env(env, attrs) -> dict:
    return {n: copy.deepcopy(getattr(env, n)) for n in attrs}


def restore_env(env, snap: dict) -> None:
    for n, v in snap.items():
        setattr(env, n, v)


def terrain_from_cfg(cfg) -> tuple[float, int]:
    """(amp, seed) from cfg env.terrain_amp / env.terrain_seed; amp 0 =
    flat (legacy exact). Shared by both vec envs so parent and worker
    processes build the identical terrain."""
    from rl_move.config import cfg_get, load_config
    if cfg is None:
        cfg = load_config()
    return (float(cfg_get(cfg, "env", "terrain_amp", default=0.0)),
            int(cfg_get(cfg, "env", "terrain_seed", default=0)))


def foot_mu_from_cfg(cfg) -> float:
    """cfg env.foot_friction_slide (0 = XML default; see sim_env)."""
    from rl_move.config import cfg_get, load_config
    if cfg is None:
        cfg = load_config()
    return float(cfg_get(cfg, "env", "foot_friction_slide", default=0.0))


def leg_chassis_from_cfg(cfg) -> bool:
    """cfg env.leg_chassis_collision (0 = off, the default; see sim_env)."""
    return leg_chassis_collision_from_cfg(cfg)


def _apply_nominal_struct_compliance(model, cfg) -> None:
    if cfg is None:
        from rl_move.config import load_config
        cfg = load_config()
    comp = StructCompliance.from_cfg(cfg)
    if comp is not None:
        comp.apply_effective_kp(model, position_actuator_ids(model),
                                k=comp.nominal_k)


def prepare_shared_model(params: SimServoParams, *, iterations: int,
                         ls_iterations: int, terrain_amp: float = 0.0,
                         terrain_seed: int = 0, foot_mu: float = 0.0,
                         leg_chassis: bool = False, cfg=None):
    """The ONE model every shim (and the device stepper) uses: MJX-compat
    terrain, the C env's contact softening, fitted servo params, reduced
    solver iterations. Deterministic, so parent and worker processes
    build identical copies independently.

    ``terrain_amp > 0`` keeps and populates the hfield (rough-terrain
    experiments; Warp impl only — see build_model). ``foot_mu > 0``
    applies the calibrated foot–ground slide friction, mirroring the C
    env's cfg env.foot_friction_slide hook."""
    # leg_chassis: cfg env.leg_chassis_collision — belly knife-edge
    # contact axis (SIM.md gap 4). Must be an XML/compile-time rewrite
    # (build_model kwarg) so put_model's device pair set includes the
    # chassis-underside contacts; runtime mask edits never register.
    model = build_model(fixed_base=False, flat_terrain=terrain_amp <= 0.0,
                        terrain_amp=terrain_amp, terrain_seed=terrain_seed,
                        mesh_visuals=False, mjx_compat=True,
                        leg_chassis_collision=leg_chassis,
                        source=resolve_model_source(cfg))
    soften_contacts(model)
    if foot_mu > 0.0:
        set_foot_ground_friction(model, foot_mu)
    apply_params_to_model(model, params)
    _apply_nominal_struct_compliance(model, cfg)
    model.opt.iterations = int(iterations)
    model.opt.ls_iterations = int(ls_iterations)
    return model


class ModelDrScratch:
    """Host twin of the C env's per-episode model prep, for model DR.

    ``sim_env.reset`` restores base model fields, applies
    ``ep_rand.apply_to_model`` and ``apply_params_to_model`` (kp/kv/
    torque DR) to ITS model. The batched vec envs keep ONE pristine
    shared model instead, so this scratch replays that exact prep on a
    private copy and hands out the resulting ``MODEL_DR_FIELDS`` rows
    for the device (plus the DR'd scratch model itself for placement,
    where leg-length scales change the plant pose).
    """

    # Fields apply_to_model MULTIPLIES in place — restore before reuse.
    _RESTORE = ("body_mass", "body_inertia", "body_ipos", "body_pos",
                "geom_friction", "geom_solref")

    def __init__(self, prepared_model, params: SimServoParams):
        self.model = copy.deepcopy(prepared_model)
        self.params = params
        self._base = {f: getattr(prepared_model, f).copy()
                      for f in self._RESTORE}
        self._base_gravity = prepared_model.opt.gravity.copy()

    def rows_for(self, env) -> dict:
        """Apply ``env._ep_rand`` (may be None) to the scratch model and
        return {field: row} for every MODEL_DR_FIELDS entry. Leaves
        ``self.model`` in the DR'd state so the caller can run the
        placement on it right after."""
        m = self.model
        for f in self._RESTORE:
            getattr(m, f)[:] = self._base[f]
        m.opt.gravity[:] = self._base_gravity
        er = env._ep_rand
        if er is not None:
            er.apply_to_model(m, chassis_bid=env._chassis_bid)
            apply_params_to_model(m, self.params,
                                  kp_scale=er.kp_scale,
                                  kv_scale=er.kv_scale,
                                  torque_scale=er.torque_scale)
            # dr.fault_*: per-episode fault rows (no-op when healthy).
            # Every touched field is in MODEL_DR_FIELDS, so the
            # per-world upload below carries the fault as-is.
            er.apply_fault_to_model(m)
        else:
            apply_params_to_model(m, self.params)
        env._apply_struct_compliance_to_model(m)
        rows = {}
        for name in MODEL_DR_FIELDS:
            v = (m.opt.gravity if name == "opt.gravity"
                 else getattr(m, name))
            rows[name] = np.array(v, dtype=np.float64, copy=True)
        return rows


def tp_rows(env) -> dict:
    """This env's TickParams row from its freshly sampled ep_rand
    (mirrors what reset() passes to ServoProfile / _advance)."""
    p = env.params
    er = env._ep_rand
    lat = p.per_joint("latency_ms") / 1000.0
    dbd = p.per_joint("deadband_deg") * DEG2RAD
    vel = p.per_joint("vel_max_deg_s") * DEG2RAD
    if er is not None:
        lat = lat * er.latency_scale
        dbd = dbd * er.deadband_scale
        vel = vel * er.vel_scale
    off = np.zeros(3) if er is None else np.asarray(er.imu_pos_m, float)
    return dict(latency_s=lat, deadband=dbd, vel_max=vel, imu_off=off)


class FakeData:
    """Per-env numpy mirror of the MjData fields the host halves read."""

    def __init__(self, model):
        self.qpos = np.zeros(model.nq)
        self.qvel = np.zeros(model.nv)
        self.qfrc_actuator = np.zeros(model.nv)
        self.xpos = np.zeros((model.nbody, 3))
        self.xmat = np.zeros((model.nbody, 9))
        self.subtree_com = np.zeros((model.nbody, 3))
        self.sensordata = np.zeros(model.nsensordata)
        self.time = 0.0


class CommandStub:
    """Stands in for ServoProfile on a shim env: records the command
    that _step_begin issues so the vec env can batch it to the device
    profile. reset() is never called on it (the choreography re-inits
    device profiles directly)."""

    __slots__ = ("pending",)

    def __init__(self):
        self.pending = None   # (q_rad, speed_deg_s, acc_units) | None

    def command(self, q_rad, *, speed_deg_s, acc_units):
        self.pending = (np.asarray(q_rad, float).copy(),
                        float(speed_deg_s), float(acc_units))

    def reset(self, q0):  # pragma: no cover — choreography owns resets
        raise RuntimeError("shim profile reset must go through the vec env")


def make_shim_class(task_cls):
    """Subclass ``task_cls`` so physics is owned by the vec env."""

    class _Shim(task_cls):
        def __init__(self, shared_model, **kwargs):
            super().__init__(model=shared_model, **kwargs)
            # Replace the MjData super() built with the batched-tick
            # mirror; placement temporarily lends a real scratch MjData.
            self.data = FakeData(shared_model)

        def _advance(self, *, limp=False):
            raise RuntimeError(
                "shim envs never step physics — the MJX vec env drives "
                "the batched tick; use the vec env's step()/reset()")

        def reset(self, **kwargs):
            raise RuntimeError("call the vec env's reset(), not env.reset()")

        def step(self, action):
            raise RuntimeError("call the vec env's step(), not env.step()")

        def render(self):
            raise RuntimeError("MJX vec envs do not render; build a C env")

    _Shim.__name__ = f"Mjx{task_cls.__name__}"
    return _Shim


def push_output_row(env, pad_bids, outs, i: int) -> None:
    """Copy env i's slice of a (host-fetched) TickOutput into its
    FakeData mirror + IMU accumulators, so _read_state and the reward
    stack read exactly what C code would."""
    fd = env.data
    fd.qpos[:] = outs.qpos_all[i]
    fd.qvel[:] = outs.qvel_all[i]
    fd.qfrc_actuator[env._vadr] = outs.qfrc_actuator[i]
    fd.xpos[env._chassis_bid] = outs.chassis_xpos[i]
    for j, b in enumerate(pad_bids):
        if b >= 0:
            fd.xpos[b] = outs.pad_xpos[i, j]
    fd.xmat[env._chassis_bid] = np.asarray(outs.chassis_xmat[i]).ravel()
    # Row 0 = world subtree CoM. Static geoms are massless, so the
    # robot subtree CoM the stepper returns is the same quantity.
    fd.subtree_com[0] = outs.subtree_com[i]
    fd.subtree_com[env._chassis_bid] = outs.subtree_com[i]
    fd.sensordata[:] = outs.sensordata[i]
    fd.time = float(outs.time[i])
    # One accumulated sample per tick: _read_state averages accum/n,
    # and the device already averaged over the tick's substeps.
    env._imu_f_accum = np.asarray(outs.f_imu[i], dtype=float).copy()
    env._imu_f_n = 1
    env._gyro_accum = np.asarray(outs.gyro[i], dtype=float).copy()
    env._gyro_n = 1


def place_env(env, scratch_data, q_start, model=None) -> np.ndarray:
    """Run _place_at_plant on a lent real MjData; returns full qpos.

    ``model``: lend a DR'd scratch model too (model DR — leg-length /
    mass draws change the placement); default keeps the env's shared
    pristine model.
    """
    fake = env.data
    shared = env.model
    env.data = scratch_data
    if model is not None:
        env.model = model
    try:
        env._place_at_plant(q_start)
        return np.asarray(scratch_data.qpos, dtype=float).copy()
    finally:
        env.data = fake
        env.model = shared
