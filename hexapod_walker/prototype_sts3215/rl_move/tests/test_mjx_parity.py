"""Parity tests: MJX backend vs the C-MuJoCo path it mirrors.

Skipped entirely when jax / mujoco-mjx are not installed (they are
optional deps — see rl_move/sim/requirements-mjx.txt).

What "parity" means per test:
- Servo profile: the JAX port must match numpy ServoProfile to float32
  round-off — same latency queue, trapezoid and deadband semantics.
- IMU point velocity: the cvel-based formula must match
  mj_objectVelocity at an identical dynamic state.
- Air trajectory (fixed base, no contacts): MJX float32 vs C float64
  drift over 1 s of a commanded step stays well under a degree.
- Contact settle: loose bounds only — Newton/elliptic contact solves
  differ in float32; this guards gross divergence (wrong pose, tipping),
  not exactness. Trusting MJX for training needs the phase-2 behavioral
  comparison described in MJX_PORT.md.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

_PROTO = Path(__file__).resolve().parents[2]
if str(_PROTO) not in sys.path:
    sys.path.insert(0, str(_PROTO))

from rl_move.sim.mjx_backend import (  # noqa: E402
    MjxTickStepper, mjx_is_available,
)
from rl_move.sim.servo_model import (  # noqa: E402
    ServoProfile, SimServoParams, apply_params_to_model, build_model,
)

pytestmark = pytest.mark.skipif(
    not mjx_is_available(), reason="jax / mujoco-mjx not installed")

DEG2RAD = math.pi / 180.0
DT_CTRL = 0.04
SPEED_DEG_S = 400 * 360.0 / 4096.0
ACC_UNITS = 20.0


def _params() -> SimServoParams:
    return SimServoParams.load()


# ---------------------------------------------------------------------------
# 1. servo profile: JAX pytree vs numpy ServoProfile
# ---------------------------------------------------------------------------


def test_profile_matches_numpy():
    import jax
    import jax.numpy as jnp

    from rl_move.sim import mjx_backend as mb

    params = _params()
    rng = np.random.default_rng(0)
    q0 = rng.uniform(-0.5, 0.5, 18)

    ref = ServoProfile(params, q0, latency_scale=1.3, deadband_scale=0.8,
                       vel_scale=1.1)

    lat = params.per_joint("latency_ms") / 1000.0 * 1.3
    dbd = params.per_joint("deadband_deg") * DEG2RAD * 0.8
    vel = params.per_joint("vel_max_deg_s") * DEG2RAD * 1.1
    tp = mb.TickParams(latency_s=jnp.asarray(lat, jnp.float32),
                       deadband=jnp.asarray(dbd, jnp.float32),
                       vel_max=jnp.asarray(vel, jnp.float32),
                       imu_off=jnp.zeros(3, jnp.float32))
    acc0 = np.full(18, 15.0 * mb.ACC_UNIT_DEG_S2 * DEG2RAD)
    st = mb.init_profile_state(jnp, q0, vel, acc0)

    h = 0.002
    substeps = int(round(DT_CTRL / h))

    @jax.jit
    def run_tick(st, cmd):
        st = mb._profile_enqueue(jnp, st, tp, cmd)
        targets = []
        for _ in range(substeps):
            st, tgt = mb._profile_tick(jnp, st, tp, h)
            targets.append(tgt)
        return st, jnp.stack(targets)

    worst = 0.0
    for i in range(40):
        goal = rng.uniform(-0.8, 0.8, 18)
        valid = rng.random() > 0.15          # some dropped SyncWrites
        if valid:
            ref.command(goal, speed_deg_s=SPEED_DEG_S, acc_units=ACC_UNITS)
        ref_targets = np.stack([ref.tick(h) for _ in range(substeps)])

        cmd = mb.Command(
            q=jnp.asarray(goal, jnp.float32),
            vel=jnp.full(18, SPEED_DEG_S * DEG2RAD, jnp.float32),
            acc=jnp.full(18, ACC_UNITS * mb.ACC_UNIT_DEG_S2 * DEG2RAD,
                         jnp.float32),
            valid=jnp.bool_(valid))
        st, jax_targets = run_tick(st, cmd)
        worst = max(worst, float(np.max(np.abs(
            np.asarray(jax_targets) - ref_targets))))
    # float32 accumulation over 40 ticks; ~6e-3 deg is round-off, not
    # a semantic difference.
    assert worst < 2e-4, f"profile diverged: {worst:.2e} rad"


# ---------------------------------------------------------------------------
# 2. IMU point velocity: cvel formula vs mj_objectVelocity
# ---------------------------------------------------------------------------


def test_imu_point_velocity_matches_mujoco():
    import jax.numpy as jnp
    import mujoco
    from mujoco import mjx

    from rl_move.sim import mjx_backend as mb

    model = build_model(fixed_base=False, flat_terrain=True,
                        mesh_visuals=False, mjx_compat=True)
    apply_params_to_model(model, _params())
    adr = mb._model_addrs(model)

    rng = np.random.default_rng(1)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.4                       # airborne, tumbling state
    data.qpos[3:7] = rng.normal(size=4)
    data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
    data.qpos[adr.qadr] = rng.uniform(-0.5, 0.5, 18)
    data.qvel[:] = rng.uniform(-1.0, 1.0, model.nv)
    mujoco.mj_forward(model, data)

    r_off = np.array([0.05, -0.03, 0.08])
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                             adr.chassis_bid, vel, 0)
    R = data.xmat[adr.chassis_bid].reshape(3, 3)
    v_ref = vel[3:] + np.cross(vel[:3], R @ r_off)

    mx = mjx.put_model(model)
    dx = mjx.put_data(model, data)
    omega, _, v_pt = mb._imu_point_velocity(
        jnp, dx, adr, jnp.asarray(r_off, jnp.float32))
    assert np.allclose(np.asarray(omega), vel[:3], atol=1e-5)
    assert np.allclose(np.asarray(v_pt), v_ref, atol=1e-5), (
        np.asarray(v_pt), v_ref)


# ---------------------------------------------------------------------------
# 3/4. physics trajectories
# ---------------------------------------------------------------------------


def _c_advance(model, data, profile, q_target, substeps):
    """Replica of SimHexapodBalanceEnv._advance's actuation path."""
    import mujoco
    from rl_move.sim.servo_model import position_actuator_ids
    from rl_move.sim.servo_model import joint_qpos_addrs
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    h = model.opt.timestep
    for _ in range(substeps):
        target = profile.tick(h)
        q = data.qpos[qadr]
        err = target - q
        db = profile.deadband_rad
        eff = q + np.sign(err) * np.maximum(np.abs(err) - db, 0.0)
        data.ctrl[pos_act] = eff
        mujoco.mj_step(model, data)


def test_air_trajectory_parity():
    import mujoco

    from rl_move.sim import mjx_backend as mb

    params = _params()
    model = build_model(fixed_base=True, flat_terrain=True,
                        mesh_visuals=False, mjx_compat=True)
    apply_params_to_model(model, params)
    # Lift the welded chassis so nothing can touch the floor.
    cb = mb._model_addrs(model).chassis_bid
    model.body_pos[cb, 2] += 0.5
    adr = mb._model_addrs(model)

    q0 = np.zeros(18)
    goal = np.tile([0.15, 0.35, -0.6], 6)     # a big multi-joint step

    # C-MuJoCo reference.
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0, "air test touched something"
    prof = ServoProfile(params, q0)
    substeps = int(round(DT_CTRL / model.opt.timestep))
    prof.command(goal, speed_deg_s=SPEED_DEG_S, acc_units=ACC_UNITS)
    ref_traj = []
    for _ in range(25):                        # 1 s
        _c_advance(model, data, prof, goal, substeps)
        ref_traj.append(data.qpos[adr.qadr].copy())
    ref_traj = np.stack(ref_traj)

    # MJX.
    st = MjxTickStepper(model, 2, params=params)
    data2 = mujoco.MjData(model)
    mujoco.mj_resetData(model, data2)
    st.reset_envs(np.tile(data2.qpos, (2, 1)), np.zeros((2, model.nv)),
                  np.tile(q0, (2, 1)), dt_ctrl=DT_CTRL)
    cmd = st.make_command(np.tile(goal, (2, 1)), speed_deg_s=SPEED_DEG_S,
                          acc_units=ACC_UNITS)
    hold = st.make_command(np.tile(goal, (2, 1)), speed_deg_s=SPEED_DEG_S,
                           acc_units=ACC_UNITS, valid=False)
    mjx_traj = []
    for i in range(25):
        out = st.tick(cmd if i == 0 else hold)
        mjx_traj.append(np.asarray(out.q)[0])
    mjx_traj = np.stack(mjx_traj)

    err_deg = np.abs(mjx_traj - ref_traj) / DEG2RAD
    assert float(err_deg.max()) < 0.5, (
        f"air trajectory diverged: max {err_deg.max():.3f} deg")
    # Both envs in the batch must be identical (determinism).
    st2_q = np.asarray(out.q)
    assert np.array_equal(st2_q[0], st2_q[1])


def test_contact_settle_parity_loose():
    import mujoco

    from rl_move.sim import mjx_backend as mb

    params = _params()
    model = build_model(fixed_base=False, flat_terrain=True,
                        mesh_visuals=False, mjx_compat=True)
    apply_params_to_model(model, params)
    adr = mb._model_addrs(model)

    q0 = np.zeros(18)

    def place(data):
        mujoco.mj_resetData(model, data)
        data.qpos[2] = 0.08
        mujoco.mj_forward(model, data)

    # C reference: hold zero pose, settle 1.2 s onto the belly.
    data = mujoco.MjData(model)
    place(data)
    prof = ServoProfile(params, q0)
    substeps = int(round(DT_CTRL / model.opt.timestep))
    for _ in range(30):
        _c_advance(model, data, prof, q0, substeps)
    ref_q = data.qpos[adr.qadr].copy()
    ref_z = float(data.xpos[adr.chassis_bid, 2])

    st = MjxTickStepper(model, 1, params=params)
    data2 = mujoco.MjData(model)
    place(data2)
    st.reset_envs(data2.qpos[None], data2.qvel[None], q0[None],
                  dt_ctrl=DT_CTRL)
    hold = st.make_command(q0[None], speed_deg_s=SPEED_DEG_S,
                           acc_units=ACC_UNITS, valid=False)
    for _ in range(30):
        out = st.tick(hold)

    z = float(np.asarray(out.chassis_xpos)[0, 2])
    dq_deg = np.abs(np.asarray(out.q)[0] - ref_q) / DEG2RAD
    assert abs(z - ref_z) < 0.01, f"chassis z {z:.4f} vs C {ref_z:.4f}"
    assert float(dq_deg.max()) < 3.0, (
        f"settled joints diverged: max {dq_deg.max():.2f} deg")
    # Static and level: the IMU must read ~+1 g.
    f = np.asarray(out.f_imu)[0]
    assert abs(np.linalg.norm(f) - 9.80665) < 0.3
