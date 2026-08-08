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

Not here yet (phase 2+): batched reset/settle, per-env DR model fields
(MJX supports vmapping model arrays), touch-sensor reward inputs, and a
VecEnv that plugs into train_ppo_sim. Nothing in the default training
path imports this module — it is opt-in.
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
# (fitted ~60 ms × DR latency_scale ≤ 1.8 ⇒ ~110 ms; 8 × 40 ms = 320 ms).
PENDING_SLOTS = 8


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
    qfrc_actuator: Any  # (18,) net actuator torque (current estimate)
    chassis_xpos: Any   # (3,)
    chassis_xmat: Any   # (3,3)
    subtree_com: Any    # (3,)  whole-robot CoM (root subtree)
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


def _make_tick_fn(mjx_model, adr: _Addrs, substeps: int, gravity):
    """Build the single-env control-tick function (to be vmapped+jitted)."""
    jax, jnp, mjx = _require_mjx()
    h = float(mjx_model.opt.timestep)
    gravity = jnp.asarray(gravity, jnp.float32)

    def substep(carry, _):
        dx, prof, imu, params, limp = carry
        prof, target = _profile_tick(jnp, prof, params, h)
        q = dx.qpos[adr.qadr]
        # Firmware dead-zone at the physics level (see sim_env._advance);
        # limp = torque-off settling (actuator reference follows q).
        err = target - q
        eff = q + jnp.sign(err) * jnp.maximum(
            jnp.abs(err) - params.deadband, 0.0)
        eff = jnp.where(limp, q, eff)
        dx = dx.replace(ctrl=dx.ctrl.at[adr.pos_act].set(eff))
        dx = mjx.step(mjx_model, dx)

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
        return (dx, prof, imu, params, limp), None

    def tick(dx, prof, imu, params, cmd, limp):
        prof = _profile_enqueue(jnp, prof, params, cmd)
        (dx, prof, imu, _, _), _ = jax.lax.scan(
            substep, (dx, prof, imu, params, limp), None, length=substeps)
        f_imu = jnp.where(imu.f_n > 0, imu.f_accum / jnp.maximum(imu.f_n, 1),
                          -gravity)
        gyro = imu.gyro_accum / jnp.maximum(imu.gyro_n, 1)
        # Accumulators reset per tick (read), prev_v persists (_read_state).
        imu = imu._replace(
            f_accum=jnp.zeros(3, jnp.float32), f_n=jnp.int32(0),
            gyro_accum=jnp.zeros(3, jnp.float32), gyro_n=jnp.int32(0))
        out = TickOutput(
            q=dx.qpos[adr.qadr], qd=dx.qvel[adr.vadr],
            qfrc_actuator=dx.qfrc_actuator[adr.vadr],
            chassis_xpos=dx.xpos[adr.chassis_bid],
            chassis_xmat=dx.xmat[adr.chassis_bid].reshape(3, 3),
            subtree_com=dx.subtree_com[adr.chassis_rootid],
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

    Phase-1 limits: one shared ``mjx.Model`` — per-env DR of model
    fields (mass, friction, gravity …) is NOT applied yet; per-env
    actuation DR (latency/deadband/vel scales, IMU mount) IS supported
    via per-env ``TickParams``.
    """

    def __init__(self, mj_model, n_envs: int, *,
                 params: SimServoParams | None = None,
                 device=None):
        jax, jnp, mjx = _require_mjx()
        self._jax, self._jnp, self._mjx = jax, jnp, mjx
        self.n_envs = int(n_envs)
        self.mj_model = mj_model
        self.model = mjx.put_model(mj_model, device=device)
        self.adr = _model_addrs(mj_model)
        self.substeps = None  # set per dt at first reset
        self.params_src = params if params is not None else SimServoParams.load()
        self._tick_jit = None
        self._dx = None
        self._prof = None
        self._imu = None
        self._tick_params = None

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
        tp = tick_params or self.default_tick_params()
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

        if self.substeps != substeps or self._tick_jit is None:
            self.substeps = substeps
            tick = _make_tick_fn(self.model, self.adr, substeps,
                                 self.mj_model.opt.gravity)
            self._tick_jit = jax.jit(jax.vmap(tick, in_axes=0))

        dx0 = self._mjx.make_data(self.model)
        dx = jax.vmap(lambda q, v: dx0.replace(
            qpos=q, qvel=v, ctrl=dx0.ctrl.at[self.adr.pos_act].set(
                q[self.adr.qadr])))(jnp.asarray(qpos), jnp.asarray(qvel))
        self._dx = jax.jit(jax.vmap(
            partial(self._mjx.forward, self.model)))(dx)

        vel0 = self._tick_params.vel_max
        acc0 = jnp.full((B, N_JOINTS),
                        acc_units0 * ACC_UNIT_DEG_S2 * DEG2RAD, jnp.float32)
        self._prof = jax.vmap(partial(init_profile_state, jnp))(
            jnp.asarray(q0), vel0, acc0)
        self._imu = jax.vmap(lambda _: init_imu_state(jnp))(jnp.arange(B))

    def tick(self, cmd: Command, *,
             limp: np.ndarray | bool = False) -> TickOutput:
        """Advance every env one control tick; returns device arrays
        (np.asarray() them host-side)."""
        assert self._dx is not None, "call reset_envs() first"
        jnp = self._jnp
        limp_b = jnp.asarray(
            np.broadcast_to(np.asarray(limp, bool), (self.n_envs,)))
        self._dx, self._prof, self._imu, out = self._tick_jit(
            self._dx, self._prof, self._imu, self._tick_params, cmd, limp_b)
        return out

    @property
    def data(self):
        """The batched mjx.Data (advanced state), for custom readouts."""
        return self._dx
