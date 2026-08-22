"""Optional MJX (MuJoCo-XLA / JAX) physics backend — phase 1 of the port.

Goal: run the *physics half* of ``SimHexapodBalanceEnv`` — the
``_advance()`` inner loop — as one jitted, vmapped JAX call so a single
GPU steps hundreds/thousands of robots per control tick, instead of one
C-MuJoCo ``mj_step`` per env per process. Observation building, reward,
safety, IK and reset placement stay host-side numpy (they read the small
per-tick arrays this module returns), so all reward semantics are
byte-for-byte the existing code. See ``MJX_PORT.md`` for the phase plan.

What this module provides today:

- ``mjx_is_available()`` — import probe.
- ``build_model(mjx_compat=True)`` (in ``servo_model``) — the only model
  change MJX needs is hfield→plane; Newton solver, elliptic cone and
  condim 4/6 all convert as-is (verified against mujoco-mjx 3.9).
- ``ServoProfile`` port: the Feetech latency queue + trapezoid profile +
  deadband from ``servo_model.ServoProfile``, as a JAX pytree
  (``ProfileState``) that lives inside jit. Semantics match the numpy
  implementation (see ``tests/test_mjx_parity.py``).
- ``MjxTickStepper`` — holds a batch of ``mjx.Data`` plus profile/IMU
  state and advances everything one 25 Hz control tick per call,
  replicating ``SimHexapodBalanceEnv._advance``: profile tick →
  firmware dead-zone → ``mjx.step`` → IMU specific-force and gyro
  accumulation at the physics rate.

Phase 2 lives in ``mjx_vec_env.py``: the SB3 ``MjxVecEnv`` drives this
stepper (batched reset choreography via the in-tick slip override,
pooled resets via ``snapshot_state``/``inject_env_states``). Per-env
MODEL-field DR (mass/geometry/friction/gravity/actuator gains) is
supported via ``model_dr=True`` + ``set_model_fields`` — the
``MODEL_DR_FIELDS`` get a leading (B,) world dim, which both the warp
and XLA impls batch correctly (probed on H200, mujoco-mjx 3.11).
Nothing in the default training path imports this module — it is
opt-in.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import partial
from typing import Any, NamedTuple

import numpy as np

from .servo_model import (
    ACC_UNIT_DEG_S2, N_JOINTS, SimServoParams, build_model,
    joint_qpos_addrs, joint_qvel_addrs, position_actuator_ids,
)

DEG2RAD = math.pi / 180.0

# Pending-command ring slots. Commands arrive once per control tick;
# a slot may only be overwritten after its goal has matured (applied),
# so K * dt_ctrl must comfortably exceed the largest latency
# (air fit ~60 ms; the LOADED fit (sim_model_loaded.json, 08-10) carries
# ~85-106 ms per axis × DR latency_scale ≤ 1.8 ⇒ ~190 ms; 12 × 40 ms =
# 480 ms keeps the upload check's 2× margin. Ring scan cost is a
# fori over slots inside jit — 12 vs 8 is noise).
PENDING_SLOTS = 12

# The MjModel fields per-episode model DR writes (domain_rand
# EpisodeRandomization.apply_to_model + servo_model.apply_params_to_model
# with kp/kv/torque scales). Under ``model_dr`` these become PER-WORLD
# device arrays with a leading (B,) dim — verified supported by both the
# warp and XLA impls on mujoco-mjx 3.11 (see MJX_PORT.md).
MODEL_DR_FIELDS = (
    "body_mass", "body_inertia", "body_ipos", "body_pos",
    "geom_friction", "geom_solref",
    "actuator_gainprm", "actuator_biasprm", "actuator_forcerange",
    "dof_damping", "opt.gravity",
)


def mjx_is_available() -> bool:
    try:
        import jax  # noqa: F401
        from mujoco import mjx  # noqa: F401
        return True
    except Exception:
        return False


def _require_mjx():
    import jax
    import jax.numpy as jnp
    from mujoco import mjx
    return jax, jnp, mjx


def tune_for_mjx(model, *, iterations: int = 8, ls_iterations: int = 8):
    """GPU performance knobs, applied in place. Opt-in.

    The XML's ``iterations=50`` Newton setting is tuned for C MuJoCo;
    on MJX the solver cost is the whole bill and the official guidance
    is a handful of iterations plus a few line-search steps. Physics at
    reduced iterations must be re-validated against the C twin (the
    phase-2 behavioral A/B in MJX_PORT.md) before training on it.
    """
    model.opt.iterations = int(iterations)
    model.opt.ls_iterations = int(ls_iterations)
    return model


# ---------------------------------------------------------------------------
# Servo profile as a JAX pytree (port of servo_model.ServoProfile)
# ---------------------------------------------------------------------------


class ProfileState(NamedTuple):
    """Per-env servo-profile state. All arrays float32 on device."""
    t: Any            # ()      profile clock (s)
    goal: Any         # (18,)   latest matured goal per joint
    target: Any       # (18,)   profile target the actuators track
    v: Any            # (18,)   profile velocity
    vel_now: Any      # (18,)   active cruise-speed limit
    acc_now: Any      # (18,)   active profile acceleration
    pend_q: Any       # (K,18)  queued goals
    pend_vel: Any     # (K,18)  queued cruise speeds
    pend_acc: Any     # (K,18)  queued accelerations
    pend_t: Any       # (K,)    write times (+inf = empty slot)
    head: Any         # ()      int32 ring index of the oldest slot


class TickParams(NamedTuple):
    """Per-env constants (change only at episode reset, e.g. under DR)."""
    latency_s: Any    # (18,)  command → first motion
    deadband: Any     # (18,)  rad; profile AND firmware dead-zone
    vel_max: Any      # (18,)  rad/s ceiling (commanded speed is clamped)
    imu_off: Any      # (3,)   IMU mounting point offset, chassis frame


class ImuState(NamedTuple):
    """Physics-rate IMU accumulators (mirrors _advance/_read_state)."""
    prev_v: Any       # (3,)  IMU-point velocity at the previous substep
    has_prev: Any     # ()    bool
    f_accum: Any      # (3,)  Σ specific force samples
    f_n: Any          # ()    int32
    gyro_accum: Any   # (3,)  Σ gyro samples (site frame)
    gyro_n: Any       # ()    int32


class Command(NamedTuple):
    """One control-tick goal write (what the bus SyncWrite carries)."""
    q: Any            # (18,)  goal positions (rad)
    vel: Any          # (18,)  profile speed (rad/s), clamped to vel_max
    acc: Any          # (18,)  profile acceleration (rad/s²)
    valid: Any        # ()     bool — False models a dropped SyncWrite /
                      #        no-write ticks (limp settling)


class TickOutput(NamedTuple):
    """Everything host-side obs/reward code needs, one control tick."""
    q: Any            # (18,)   joint positions
    qd: Any           # (18,)   joint velocities
    qpos_all: Any     # (nq,)   full qpos incl. root free joint (resets)
    qvel_all: Any     # (nv,)   full qvel (root twist + joints)
    qfrc_actuator: Any  # (18,) net actuator torque (current estimate)
    chassis_xpos: Any   # (3,)
    chassis_xmat: Any   # (3,3)
    subtree_com: Any    # (3,)  whole-robot CoM (root subtree)
    pad_xpos: Any       # (6,3) foot-pad body positions (walk shaping)
    pad_z: Any          # (6,)  foot-pad body heights
    f_imu: Any          # (3,)  mean specific force at the IMU point (world)
    gyro: Any           # (3,)  mean gyro (site frame)
    sensordata: Any     # (nsensordata,) raw MJX sensors (touch/gyro/…)
    time: Any           # ()


def init_profile_state(jnp, q0, vel0, acc0) -> ProfileState:
    """Mirror ServoProfile.reset(q0)."""
    q0 = jnp.asarray(q0, dtype=jnp.float32)
    K = PENDING_SLOTS
    return ProfileState(
        t=jnp.float32(0.0),
        goal=q0, target=q0,
        v=jnp.zeros(N_JOINTS, jnp.float32),
        vel_now=jnp.asarray(vel0, jnp.float32),
        acc_now=jnp.asarray(acc0, jnp.float32),
        pend_q=jnp.zeros((K, N_JOINTS), jnp.float32),
        pend_vel=jnp.zeros((K, N_JOINTS), jnp.float32),
        pend_acc=jnp.zeros((K, N_JOINTS), jnp.float32),
        pend_t=jnp.full((K,), jnp.inf, jnp.float32),
        head=jnp.int32(0),
    )


def init_imu_state(jnp) -> ImuState:
    return ImuState(
        prev_v=jnp.zeros(3, jnp.float32), has_prev=jnp.bool_(False),
        f_accum=jnp.zeros(3, jnp.float32), f_n=jnp.int32(0),
        gyro_accum=jnp.zeros(3, jnp.float32), gyro_n=jnp.int32(0),
    )


def _profile_enqueue(jnp, prof: ProfileState, params: TickParams,
                     cmd: Command) -> ProfileState:
    """ServoProfile.command(): write into the oldest ring slot."""
    vel = jnp.minimum(cmd.vel, params.vel_max)
    i = prof.head
    new = prof._replace(
        pend_q=prof.pend_q.at[i].set(cmd.q),
        pend_vel=prof.pend_vel.at[i].set(vel),
        pend_acc=prof.pend_acc.at[i].set(cmd.acc),
        pend_t=prof.pend_t.at[i].set(prof.t),
        head=(i + 1) % PENDING_SLOTS,
    )
    # Dropped SyncWrite / no-write tick: state unchanged.
    import jax
    return jax.tree_util.tree_map(
        lambda a, b: jnp.where(cmd.valid, a, b), new, prof)


def _profile_tick(jnp, prof: ProfileState, params: TickParams,
                  h: float) -> tuple[ProfileState, Any]:
    """ServoProfile.tick(h): mature queued writes, slew the trapezoid."""
    t = prof.t + h
    goal, vel_now, acc_now = prof.goal, prof.vel_now, prof.acc_now
    # Oldest → newest so the newest matured write wins per joint. Slots
    # are re-applied idempotently until overwritten (same semantics as
    # the numpy queue, which keeps entries until matured for all joints;
    # per-joint latency is constant within an episode, so an older entry
    # always matures no later than a newer one).
    for k in range(PENDING_SLOTS):
        idx = (prof.head + k) % PENDING_SLOTS
        matured = (t - prof.pend_t[idx]) >= params.latency_s   # (18,)
        goal = jnp.where(matured, prof.pend_q[idx], goal)
        vel_now = jnp.where(matured, prof.pend_vel[idx], vel_now)
        acc_now = jnp.where(matured, prof.pend_acc[idx], acc_now)

    err = goal - prof.target
    active = jnp.abs(err) > params.deadband
    acc = jnp.maximum(acc_now, 1e-6)

    direction = jnp.sign(err)
    stop_dist = prof.v ** 2 / (2.0 * acc)
    toward = prof.v * direction > 0
    decel = toward & (stop_dist >= jnp.abs(err))
    dv = jnp.where(decel, -jnp.sign(prof.v) * acc, direction * acc) * h
    v_new = jnp.clip(prof.v + dv, -vel_now, vel_now)

    step = v_new * h
    arrive = jnp.abs(step) >= jnp.abs(err)
    move = active & ~arrive
    target = jnp.where(move, prof.target + step,
                       jnp.where(active, goal, prof.target))
    v = jnp.where(move, v_new, 0.0)
    return prof._replace(t=t, goal=goal, target=target, v=v,
                         vel_now=vel_now, acc_now=acc_now), target


# ---------------------------------------------------------------------------
# Batched control-tick stepper
# ---------------------------------------------------------------------------


@dataclass
class _Addrs:
    qadr: np.ndarray
    vadr: np.ndarray
    pos_act: np.ndarray
    chassis_bid: int
    chassis_rootid: int
    gyro_site: int
    pad_bids: np.ndarray


def _model_addrs(mj_model) -> _Addrs:
    import mujoco
    chassis_bid = mujoco.mj_name2id(
        mj_model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    gid = mujoco.mj_name2id(
        mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
    if mj_model.sensor_type[gid] != mujoco.mjtSensor.mjSENS_GYRO:
        raise RuntimeError("chassis_gyro is not a gyro sensor")
    pad_bids = np.array([
        mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_pad")
        for i in range(6)], dtype=int)
    return _Addrs(
        qadr=joint_qpos_addrs(mj_model),
        vadr=joint_qvel_addrs(mj_model),
        pos_act=position_actuator_ids(mj_model),
        chassis_bid=chassis_bid,
        chassis_rootid=int(mj_model.body_rootid[chassis_bid]),
        gyro_site=int(mj_model.sensor_objid[gid]),
        pad_bids=pad_bids,
    )


def _imu_point_velocity(jnp, dx, adr: _Addrs, r_off):
    """World-frame velocity of the IMU mounting point.

    Equivalent to mj_objectVelocity(..., mjOBJ_BODY, chassis, flg_local=0)
    followed by v_pt = vel[3:] + ω × (R @ r_off), as in
    sim_env._advance. mjOBJ_BODY reports the velocity at the body's
    inertial-frame origin (xipos, the chassis CoM — NOT xpos); cvel is
    the com-based spatial velocity about the root subtree CoM, in world
    frame, so we shift its reference point to xipos + R @ r_off.
    """
    omega = dx.cvel[adr.chassis_bid, :3]
    v_com = dx.cvel[adr.chassis_bid, 3:]
    R = dx.xmat[adr.chassis_bid].reshape(3, 3)
    p = dx.xipos[adr.chassis_bid] + R @ r_off
    return omega, R, v_com + jnp.cross(
        omega, p - dx.subtree_com[adr.chassis_rootid])


def _make_tick_fn(base_model, adr: _Addrs, substeps: int,
                  dr_fields: tuple[str, ...] = ()):
    """Build the single-env control-tick function (to be vmapped+jitted).

    ``base_model`` is a CLOSURE CONSTANT (compile-time), which is what
    keeps Warp fast: passing the model (or any of its fields) as
    runtime data measurably slows the compiled step (whole model as
    argument: ~1.65x on H200). The slip-settle therefore stays a
    SECOND compiled variant closing over the slip-friction model, as
    before, never a runtime friction select.

    Model DR enters through the tick's ``dr_vals`` argument: per-world
    arrays for exactly ``dr_fields``, tree_replaced over the base
    inside the trace (empty = no model DR, dr_vals must be None).
    Gravity for the IMU specific force comes from ``model.opt.gravity``
    — the DR'd, possibly tilted vector, exactly like sim_env._advance.
    """
    jax, jnp, mjx = _require_mjx()
    h = float(base_model.opt.timestep)

    def substep(carry, _):
        model, dx, prof, imu, params, limp, push_nm, push_fxy = carry
        gravity = model.opt.gravity
        prof, target = _profile_tick(jnp, prof, params, h)
        q = dx.qpos[adr.qadr]
        # Firmware dead-zone at the physics level (see sim_env._advance);
        # limp = torque-off settling (actuator reference follows q).
        err = target - q
        eff = q + jnp.sign(err) * jnp.maximum(
            jnp.abs(err) - params.deadband, 0.0)
        eff = jnp.where(limp, q, eff)
        dx = dx.replace(ctrl=dx.ctrl.at[adr.pos_act].set(eff))
        # dr.walk_push_* takeoff torque about the chassis's CURRENT
        # x-axis (world-frame xfrc row, recomputed each substep —
        # mirrors sim_env._advance). Written unconditionally: push 0
        # clears the row, so no state survives the pulse window.
        x_axis = dx.xmat[adr.chassis_bid].reshape(3, 3)[:, 0]
        dx = dx.replace(xfrc_applied=dx.xfrc_applied.at[
            adr.chassis_bid, 3:6].set(x_axis * push_nm))
        # dr.ext_push_* mid-episode horizontal FORCE (M3 push-recovery
        # curriculum, sim_env._ext_push_force_n): already world-frame,
        # no rotation needed. Same overwrite-every-substep convention.
        force3 = jnp.concatenate(
            [push_fxy, jnp.zeros((1,), push_fxy.dtype)])
        dx = dx.replace(xfrc_applied=dx.xfrc_applied.at[
            adr.chassis_bid, 0:3].set(force3))
        dx = mjx.step(model, dx)

        omega, R, v_pt = _imu_point_velocity(jnp, dx, adr, params.imu_off)
        a_pt = (v_pt - imu.prev_v) / h
        f_new = imu.f_accum + (a_pt - gravity)
        imu = imu._replace(
            prev_v=v_pt,
            has_prev=jnp.bool_(True),
            f_accum=jnp.where(imu.has_prev, f_new, imu.f_accum),
            f_n=imu.f_n + jnp.where(imu.has_prev, 1, 0),
            # Gyro sensor = body angular velocity in the site frame,
            # accumulated at the physics rate (anti-aliasing, like the
            # 1 kHz MPU integration).
            gyro_accum=imu.gyro_accum
            + dx.site_xmat[adr.gyro_site].reshape(3, 3).T @ omega,
            gyro_n=imu.gyro_n + 1,
        )
        return (model, dx, prof, imu, params, limp, push_nm, push_fxy), None

    def tick(dr_vals, dx, prof, imu, params, cmd, limp, push_nm, push_fxy):
        model = base_model
        if dr_fields:
            model = model.tree_replace(dict(zip(dr_fields, dr_vals)))
        prof = _profile_enqueue(jnp, prof, params, cmd)
        (_, dx, prof, imu, _, _, _, _), _ = jax.lax.scan(
            substep,
            (model, dx, prof, imu, params, limp, push_nm, push_fxy), None,
            length=substeps)
        f_imu = jnp.where(imu.f_n > 0, imu.f_accum / jnp.maximum(imu.f_n, 1),
                          -model.opt.gravity)
        gyro = imu.gyro_accum / jnp.maximum(imu.gyro_n, 1)
        # Accumulators reset per tick (read), prev_v persists (_read_state).
        imu = imu._replace(
            f_accum=jnp.zeros(3, jnp.float32), f_n=jnp.int32(0),
            gyro_accum=jnp.zeros(3, jnp.float32), gyro_n=jnp.int32(0))
        out = TickOutput(
            q=dx.qpos[adr.qadr], qd=dx.qvel[adr.vadr],
            qpos_all=dx.qpos, qvel_all=dx.qvel,
            qfrc_actuator=dx.qfrc_actuator[adr.vadr],
            chassis_xpos=dx.xpos[adr.chassis_bid],
            chassis_xmat=dx.xmat[adr.chassis_bid].reshape(3, 3),
            subtree_com=dx.subtree_com[adr.chassis_rootid],
            pad_xpos=dx.xpos[adr.pad_bids],
            pad_z=dx.xpos[adr.pad_bids, 2],
            f_imu=f_imu, gyro=gyro,
            sensordata=dx.sensordata, time=dx.time)
        return dx, prof, imu, out

    return tick


class MjxTickStepper:
    """Batched MJX twin of ``SimHexapodBalanceEnv._advance``.

    One ``tick()`` advances all ``n_envs`` robots by one control tick
    (``substeps`` physics steps) in a single jitted call. The caller owns
    everything else: reset placement (compute qpos host-side with C
    MuJoCo, hand it to ``reset_envs``), commands (from the SafetyLayer /
    IK / policy), and obs/reward (from the returned ``TickOutput``).

    Per-env DR: actuation DR (latency/deadband/vel scales, IMU mount)
    via per-env ``TickParams``; MODEL-field DR (mass, geometry,
    friction, compliance, gravity, actuator gains) via ``model_dr=True``
    — the ``MODEL_DR_FIELDS`` become per-world device arrays written
    with ``set_model_fields``.
    """

    def __init__(self, mj_model, n_envs: int, *,
                 params: SimServoParams | None = None,
                 device=None, impl: str | None = None,
                 nacon_per_env: int = 64, njmax: int = 256,
                 slip_mu: float | None = None,
                 model_dr: bool = False):
        """``impl``: None = mjx default, "jax" = XLA, "warp" = MuJoCo-Warp
        (needs mujoco-warp installed; ~2 orders of magnitude faster on
        GPU for this model). ``nacon_per_env``/``njmax`` size Warp's
        fixed contact/constraint buffers — Warp WARNS on overflow and
        silently drops contacts, so keep headroom (the settled hexapod
        uses ~11 contacts/env).

        ``slip_mu``: also compile a tick variant closing over a model
        with every geom's sliding friction set to this value — the
        reset slip-settle (SimHexapodBalanceEnv.SLIP_MU), selected per
        tick with ``tick(..., slip=True)``. Two static models instead
        of a runtime friction write keeps friction a compile-time
        constant (a runtime select measurably slows Warp).

        ``model_dr``: give every world its own copy of the
        ``MODEL_DR_FIELDS`` (leading (B,) dim on device). Rows start at
        the base model's values; write per-episode draws with
        ``set_model_fields`` (the vec envs do this on reset). The slip
        variant applies every DR field EXCEPT ``geom_friction`` (the
        slip write overrides the DR'd sliding friction, exactly like
        the C env's temporary ``geom_friction[:, 0] = SLIP_MU``).
        """
        import copy as _copy
        jax, jnp, mjx = _require_mjx()
        self._jax, self._jnp, self._mjx = jax, jnp, mjx
        self.n_envs = int(n_envs)
        self.mj_model = mj_model
        self.impl = impl
        # Rough terrain requires Warp: the XLA impl has no hfield
        # collisions and would silently walk on the flat backup plane.
        if (impl != "warp" and mj_model.hfield_data.size
                and float(abs(mj_model.hfield_data).max()) > 0.0):
            raise RuntimeError(
                "rough terrain (env.terrain_amp > 0) needs --impl warp; "
                f"impl={impl!r} has no heightfield collisions")
        self._nacon_per_env = int(nacon_per_env)
        self._njmax = int(njmax)
        self._slip_mu = slip_mu
        self.model_dr = bool(model_dr)
        kw = {} if impl is None else {"impl": impl}
        self.model = mjx.put_model(mj_model, device=device, **kw)
        self.model_slip = None
        if slip_mu is not None:
            m_slip = _copy.deepcopy(mj_model)
            m_slip.geom_friction[:, 0] = float(slip_mu)
            self.model_slip = mjx.put_model(m_slip, device=device, **kw)
        if self.model_dr:
            B = self.n_envs
            # Per-world copies of the DR'able fields only — the rest of
            # the model stays a shared compile-time constant.
            self._dr_fields = {
                name: jnp.broadcast_to(
                    (v := self._get_model_field(self.model, name)),
                    (B,) + v.shape)
                for name in MODEL_DR_FIELDS}
        else:
            self._dr_fields = None
        self.adr = _model_addrs(mj_model)
        self.substeps = None  # set per dt at first reset
        self.params_src = params if params is not None else SimServoParams.load()
        self._tick_jit = None
        self._tick_jit_slip = None
        self._fwd_jit = None
        self._dx = None
        self._prof = None
        self._imu = None
        self._tick_params = None
        self._acc_units0 = 15.0

    @staticmethod
    def _get_model_field(model, name):
        if name.startswith("opt."):
            return getattr(model.opt, name[4:])
        return getattr(model, name)

    # Slip variant: friction comes from the slip model, not DR rows.
    _SLIP_DR_FIELDS = tuple(f for f in MODEL_DR_FIELDS
                            if f != "geom_friction")

    def _dr_vals(self, fields=MODEL_DR_FIELDS):
        """Tuple the compiled fns take for model DR (None when off)."""
        if not self.model_dr:
            return None
        return tuple(self._dr_fields[n] for n in fields)

    def set_model_fields(self, fields: dict, idx=None) -> None:
        """Write per-env MODEL_DR_FIELDS rows (host numpy) into the
        per-world device copies. ``fields`` maps field name → (k, ...)
        rows; ``idx`` selects the worlds (None = full batch, rows must
        then be (B, ...)). Requires ``model_dr=True``."""
        assert self.model_dr, "stepper built without model_dr"
        jnp = self._jnp
        for name, rows in fields.items():
            cur = self._dr_fields[name]
            rows = jnp.asarray(np.asarray(rows), dtype=cur.dtype)
            if idx is None:
                self._dr_fields[name] = jnp.broadcast_to(
                    rows, cur.shape)
            else:
                ii = jnp.asarray(np.asarray(idx, np.int32))
                self._dr_fields[name] = cur.at[ii].set(rows)

    # -- host-side helpers -------------------------------------------------

    def default_tick_params(self, *, latency_scale=1.0, deadband_scale=1.0,
                            vel_scale=1.0, imu_off=None) -> dict:
        """Numpy TickParams fields for one env (mirrors ServoProfile.__init__).

        Pass arrays of shape (n_envs,) / (n_envs, 3) to the scales to
        randomize per env; scalars broadcast.
        """
        p = self.params_src
        lat = p.per_joint("latency_ms") / 1000.0
        dbd = p.per_joint("deadband_deg") * DEG2RAD
        vel = p.per_joint("vel_max_deg_s") * DEG2RAD
        B = self.n_envs
        one = np.ones(B)
        lat = np.outer(one * latency_scale, lat)
        dbd = np.outer(one * deadband_scale, dbd)
        vel = np.outer(one * vel_scale, vel)
        off = (np.zeros((B, 3)) if imu_off is None
               else np.broadcast_to(np.asarray(imu_off, float), (B, 3)))
        return dict(latency_s=lat, deadband=dbd, vel_max=vel, imu_off=off)

    def make_command(self, q_rad: np.ndarray, *,
                     speed_deg_s: float | np.ndarray,
                     acc_units: float | np.ndarray,
                     valid: np.ndarray | bool = True) -> Command:
        """Batched bus write in the units train-time code uses
        (mirrors ServoProfile.command)."""
        jnp = self._jnp
        B = self.n_envs
        q = np.broadcast_to(np.asarray(q_rad, np.float32), (B, N_JOINTS))
        vel = np.broadcast_to(
            np.asarray(speed_deg_s, np.float32) * DEG2RAD, (B, N_JOINTS))
        acc = np.broadcast_to(
            np.asarray(acc_units, np.float32) * ACC_UNIT_DEG_S2 * DEG2RAD,
            (B, N_JOINTS))
        val = np.broadcast_to(np.asarray(valid, bool), (B,))
        return Command(q=jnp.asarray(q), vel=jnp.asarray(vel),
                       acc=jnp.asarray(acc), valid=jnp.asarray(val))

    # -- lifecycle -----------------------------------------------------------

    def reset_envs(self, qpos: np.ndarray, qvel: np.ndarray,
                   q0_profile: np.ndarray, *, dt_ctrl: float,
                   tick_params: dict | None = None,
                   acc_units0: float = 15.0) -> None:
        """(Re)initialize all envs from host-computed states.

        qpos (B, nq), qvel (B, nv): full MuJoCo states (place the robot
        with the existing C-MuJoCo logic, then copy the arrays here).
        q0_profile (B, 18): the profile's initial goal/target.
        """
        jax, jnp, mjx = self._jax, self._jnp, self._mjx
        B = self.n_envs
        qpos = np.broadcast_to(np.asarray(qpos, np.float32),
                               (B, self.mj_model.nq))
        qvel = np.broadcast_to(np.asarray(qvel, np.float32),
                               (B, self.mj_model.nv))
        q0 = np.broadcast_to(np.asarray(q0_profile, np.float32),
                             (B, N_JOINTS))

        substeps = max(1, int(round(dt_ctrl / self.mj_model.opt.timestep)))
        self._acc_units0 = float(acc_units0)
        tp = tick_params or self.default_tick_params()
        self.set_tick_params(tp, dt_ctrl=dt_ctrl)

        if self.substeps != substeps or self._tick_jit is None:
            self.substeps = substeps
            dr_ax = 0 if self.model_dr else None
            dr_f = MODEL_DR_FIELDS if self.model_dr else ()
            tick = _make_tick_fn(self.model, self.adr, substeps,
                                 dr_fields=dr_f)
            self._tick_jit = jax.jit(jax.vmap(
                tick, in_axes=(dr_ax, 0, 0, 0, 0, 0, 0, 0, 0)))
            self._tick_jit_slip = None
            if self.model_slip is not None:
                dr_fs = self._SLIP_DR_FIELDS if self.model_dr else ()
                tick_s = _make_tick_fn(self.model_slip, self.adr,
                                       substeps, dr_fields=dr_fs)
                self._tick_jit_slip = jax.jit(jax.vmap(
                    tick_s, in_axes=(dr_ax, 0, 0, 0, 0, 0, 0, 0, 0)))

            def fwd(dr_vals, dx):
                m = self.model
                if self.model_dr:
                    m = m.tree_replace(dict(zip(MODEL_DR_FIELDS, dr_vals)))
                return self._mjx.forward(m, dx)
            self._fwd_jit = jax.jit(jax.vmap(fwd, in_axes=(dr_ax, 0)))

        if self.impl == "warp":
            # Warp builds Data from the raw MjModel and uses fixed-size
            # batch-total contact / per-env constraint-row buffers.
            dx0 = self._mjx.make_data(
                self.mj_model, impl="warp",
                naconmax=self._nacon_per_env * B, njmax=self._njmax)
        else:
            dx0 = self._mjx.make_data(self.model)
        dx = jax.vmap(lambda q, v: dx0.replace(
            qpos=q, qvel=v, ctrl=dx0.ctrl.at[self.adr.pos_act].set(
                q[self.adr.qadr])))(jnp.asarray(qpos), jnp.asarray(qvel))
        self._dx = self._fwd_jit(self._dr_vals(), dx)

        self.reset_profiles(q0)
        self._imu = jax.vmap(lambda _: init_imu_state(jnp))(jnp.arange(B))

    def set_tick_params(self, tp: dict, *, dt_ctrl: float) -> None:
        """Upload per-env TickParams (full batch; small arrays)."""
        jnp = self._jnp
        max_lat = float(np.max(tp["latency_s"]))
        if PENDING_SLOTS * dt_ctrl < 2.0 * max_lat:
            raise ValueError(
                f"latency {max_lat * 1e3:.0f} ms too large for "
                f"{PENDING_SLOTS} pending-command slots at dt={dt_ctrl}")
        self._tick_params = TickParams(
            latency_s=jnp.asarray(tp["latency_s"], jnp.float32),
            deadband=jnp.asarray(tp["deadband"], jnp.float32),
            vel_max=jnp.asarray(tp["vel_max"], jnp.float32),
            imu_off=jnp.asarray(tp["imu_off"], jnp.float32))

    def reset_profiles(self, q0: np.ndarray) -> None:
        """Re-initialize EVERY env's servo profile at q0 (B, 18) — the
        reset choreography's capture-nominal stage (ServoProfile.reset).
        """
        jax, jnp = self._jax, self._jnp
        q0 = np.broadcast_to(np.asarray(q0, np.float32),
                             (self.n_envs, N_JOINTS))
        acc0 = jnp.full((self.n_envs, N_JOINTS),
                        self._acc_units0 * ACC_UNIT_DEG_S2 * DEG2RAD,
                        jnp.float32)
        self._prof = jax.vmap(partial(init_profile_state, jnp))(
            jnp.asarray(q0), self._tick_params.vel_max, acc0)

    def tick(self, cmd: Command, *, limp: np.ndarray | bool = False,
             slip: bool = False,
             push_nm: np.ndarray | None = None,
             push_fxy: np.ndarray | None = None) -> TickOutput:
        """Advance every env one control tick; returns device arrays
        (np.asarray() them host-side). ``slip=True`` uses the low-
        friction model variant (reset slip-settle; needs slip_mu).
        ``push_nm``: per-env (B,) chassis roll torque (N·m) applied as
        xfrc about each chassis's own x-axis for this tick — the
        dr.walk_push_* takeoff disturbance (None/0 = no push).
        ``push_fxy``: per-env (B, 2) world-frame (fx, fy) horizontal
        force applied as xfrc at the chassis for this tick — the
        dr.ext_push_* mid-episode push-recovery disturbance
        (None/0 = no push)."""
        assert self._dx is not None, "call reset_envs() first"
        jnp = self._jnp
        if slip:
            fn = self._tick_jit_slip
            if fn is None:
                raise RuntimeError("slip tick requested but slip_mu not set")
            dr = self._dr_vals(self._SLIP_DR_FIELDS)
        else:
            fn = self._tick_jit
            dr = self._dr_vals()
        limp_b = jnp.asarray(
            np.broadcast_to(np.asarray(limp, bool), (self.n_envs,)))
        push = (np.zeros(self.n_envs, np.float32) if push_nm is None
                else np.broadcast_to(
                    np.asarray(push_nm, np.float32), (self.n_envs,)))
        push_xy = (np.zeros((self.n_envs, 2), np.float32)
                   if push_fxy is None
                   else np.broadcast_to(
                       np.asarray(push_fxy, np.float32),
                       (self.n_envs, 2)))
        self._dx, self._prof, self._imu, out = fn(
            dr, self._dx, self._prof, self._imu, self._tick_params, cmd,
            limp_b, jnp.asarray(push), jnp.asarray(push_xy))
        return out

    # -- state surgery (pooled resets) ---------------------------------------

    def snapshot_state(self):
        """Reference-snapshot of all device state (jax arrays are
        immutable, so no copies are needed). Includes the per-world DR
        fields under model_dr — a pool-refill choreography overwrites
        them and must not leak new draws into the live episodes."""
        return (self._dx, self._prof, self._imu, self._tick_params,
                dict(self._dr_fields) if self.model_dr else None)

    def restore_state(self, snap) -> None:
        (self._dx, self._prof, self._imu, self._tick_params, dr) = snap
        if self.model_dr:
            self._dr_fields = dict(dr)

    def inject_env_states(self, idx, qpos, qvel, q_nom) -> None:
        """Overwrite envs ``idx`` with settled reset states (pooled
        resets): full qpos/qvel, servo profile re-initialized at q_nom,
        IMU accumulators cleared — then one batched forward to refresh
        derived quantities (pure function of state, so live envs are
        unchanged by it). Update tick params rows via set_tick_params
        BEFORE calling if the pooled episode carries different DR."""
        jax, jnp = self._jax, self._jnp
        k = len(idx)
        idx = jnp.asarray(np.asarray(idx, np.int32))
        qpos = jnp.asarray(np.asarray(qpos, np.float32).reshape(
            k, self.mj_model.nq))
        qvel = jnp.asarray(np.asarray(qvel, np.float32).reshape(
            k, self.mj_model.nv))
        q0 = jnp.asarray(np.asarray(q_nom, np.float32).reshape(k, N_JOINTS))

        dx = self._dx
        ctrl = dx.ctrl.at[idx[:, None],
                          jnp.asarray(self.adr.pos_act)[None, :]].set(q0)
        dx = dx.replace(qpos=dx.qpos.at[idx].set(qpos),
                        qvel=dx.qvel.at[idx].set(qvel), ctrl=ctrl)
        self._dx = self._fwd_jit(self._dr_vals(), dx)

        acc0 = jnp.full((k, N_JOINTS),
                        self._acc_units0 * ACC_UNIT_DEG_S2 * DEG2RAD,
                        jnp.float32)
        prof_new = jax.vmap(partial(init_profile_state, jnp))(
            q0, self._tick_params.vel_max[idx], acc0)
        self._prof = jax.tree_util.tree_map(
            lambda full, new: full.at[idx].set(new), self._prof, prof_new)
        imu_new = jax.vmap(lambda _: init_imu_state(jnp))(jnp.arange(k))
        self._imu = jax.tree_util.tree_map(
            lambda full, new: full.at[idx].set(new), self._imu, imu_new)

    @property
    def data(self):
        """The batched mjx.Data (advanced state), for custom readouts."""
        return self._dx
