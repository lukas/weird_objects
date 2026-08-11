"""Locks rl_move/sim/rot60.py — the rot-60 canonicalization maps AND
the physics-level claim behind them: the compiled MuJoCo model is
EXACTLY invariant under rotate-60-degrees + relabel-legs. If someone
breaks the hexagonal symmetry (per-leg params, chassis inertia,
collision geoms) the dynamics test here fails before a wrapped policy
walks sideways into a wall.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from rl_move.sim import rot60


def _frame(rng):
    x = rng.normal(0, 0.5, rot60.FRAME_WALK).astype(np.float32)
    x[36:38] = rng.normal(0, 0.25, 2) / 0.2   # physical tilt, scaled
    x[59:61] = rng.normal(0, 0.1, 2) / 0.2
    return x


def test_roundtrip_composition_identity():
    rng = np.random.default_rng(0)
    x = _frame(rng)
    assert np.array_equal(rot60.frame_transform(x, 0), x)
    for k in range(6):
        y = rot60.frame_transform(x, k)
        z = rot60.frame_transform(y, (6 - k) % 6)
        np.testing.assert_allclose(z, x, atol=1e-5)
    a = rot60.frame_transform(rot60.frame_transform(x, 1), 1)
    np.testing.assert_allclose(a, rot60.frame_transform(x, 2), atol=1e-5)


def test_leg_perm_and_action_inverse():
    rng = np.random.default_rng(1)
    act = rng.normal(0, 1, 18)
    for k in range(-2, 4):
        canon = act[rot60.leg_perm(k)]
        np.testing.assert_array_equal(
            rot60.action_from_canonical(canon, k), act)
    # canonical leg 0 must be real leg k
    for k in range(6):
        jp = rot60.leg_perm(k)
        assert list(jp[:3]) == [3 * (k % 6), 3 * (k % 6) + 1,
                                3 * (k % 6) + 2]


def test_tilt_rotation_matches_rotation_matrices():
    """tilt_rotate == recompute roll/pitch from R @ Rz(alpha)."""
    rng = np.random.default_rng(2)
    for _ in range(20):
        roll, pitch = rng.normal(0, 0.3, 2)
        yaw = rng.uniform(-math.pi, math.pi)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx          # body->world, ZYX convention
        for k in range(6):
            alpha = k * rot60.SECTOR_RAD
            ca, sa = math.cos(alpha), math.sin(alpha)
            Rza = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
            RV = R @ Rza          # canonical body frame in world
            a = RV.T @ np.array([0.0, 0.0, 1.0])   # static accel
            roll_v = math.atan2(a[1], a[2])
            pitch_v = math.atan2(-a[0], math.hypot(a[1], a[2]))
            got = rot60.tilt_rotate(roll, pitch, alpha)
            assert got[0] == pytest.approx(roll_v, abs=1e-9)
            assert got[1] == pytest.approx(pitch_v, abs=1e-9)


def test_sector_selection_and_hysteresis():
    s = rot60.sector_from_cmd
    assert s(0.05, 0.0) == 0
    assert s(-0.05, 0.0) == 3            # backward
    assert s(0.025, 0.0433) == 1         # +60
    assert s(0.025, -0.0433) == -1       # -60
    assert s(-0.025, 0.0433) == 2        # +120
    assert s(0.0, 0.0, last_k=2) == 2    # zero cmd holds sector
    # hysteresis: 33 deg heading stays in sector 0 if we were there,
    # but snaps to 1 if we come from sector 1's side
    vx, vy = 0.05 * math.cos(math.radians(33)), \
        0.05 * math.sin(math.radians(33))
    assert s(vx, vy, last_k=0) == 0
    assert s(vx, vy, last_k=1) == 1
    # far past the band it must switch regardless
    vx, vy = 0.05 * math.cos(math.radians(45)), \
        0.05 * math.sin(math.radians(45))
    assert s(vx, vy, last_k=0) == 1


def _quat_mul(q, p):
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = p
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


@pytest.fixture(scope="module")
def env():
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    return SimHexapodJointWalkEnv(params=SimServoParams.from_cfg(None),
                                  randomize=False, episode_seconds=5.0,
                                  seed=0)


def test_mujoco_dynamics_equivariance(env):
    """THE claim: rotate+relabel is a symmetry of the compiled model.

    The same physical scene described in the relabeled coordinates
    (root quat right-multiplied by Rz(60), joints/ctrl cyclically
    permuted) must produce the IDENTICAL world trajectory: equal root
    position, equal permuted joint trajectories, quat staying exactly
    the Rz(60) relabel of the original.

    Tolerances (measured 08-11): the relabeled start differs only by
    quaternion-rotation rounding (~2e-8); through 30 contact steps the
    error stays <1e-6 — an EXACT symmetry at float precision. Beyond
    that, contact-solver Lyapunov divergence amplifies the rounding
    exponentially (3e-3 by step 200), which is chaos, not asymmetry —
    a real per-leg model difference shows ~1e-3 within a few steps.
    So: tight bound early, loose chaos allowance later.
    """
    import mujoco
    from rl_move.sim.servo_model import (joint_qpos_addrs,
                                         joint_qvel_addrs, joint_names)

    model = env.model
    qadr = joint_qpos_addrs(model)
    vadr = joint_qvel_addrs(model)
    aadr = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                       n) for n in joint_names()])
    k = 1
    jp = rot60.leg_perm(k)
    alpha = k * rot60.SECTOR_RAD
    qz = np.array([math.cos(alpha / 2), 0.0, 0.0, math.sin(alpha / 2)])

    rng = np.random.default_rng(3)
    q0 = rng.uniform(-0.25, 0.25, 18)          # asymmetric leg pose
    tilt_q = np.array([math.cos(0.06), 0.05, 0.06, 0.02])
    tilt_q /= np.linalg.norm(tilt_q)
    ctrl0 = rng.uniform(-0.2, 0.2, 18)

    def rollout(joints, quat, ctrl, steps=200):
        d = mujoco.MjData(model)
        d.qpos[:] = 0
        d.qpos[2] = 0.09
        d.qpos[3:7] = quat
        d.qpos[qadr] = joints
        d.qvel[:] = 0
        d.ctrl[:] = 0
        d.ctrl[aadr] = ctrl
        mujoco.mj_forward(model, d)
        traj_j, traj_root = [], []
        for _ in range(steps):
            mujoco.mj_step(model, d)
            traj_j.append(d.qpos[qadr].copy())
            traj_root.append(d.qpos[:7].copy())
        return np.array(traj_j), np.array(traj_root)

    j1, r1 = rollout(q0, tilt_q, ctrl0, steps=100)
    j2, r2 = rollout(q0[jp], _quat_mul(tilt_q, qz), ctrl0[jp], steps=100)

    # same physical scene: identical world position, joints permuted,
    # quat exactly the relabel of the original — exact early, chaos
    # allowance later (see docstring)
    np.testing.assert_allclose(j2[:30], j1[:30, jp], atol=1e-6)
    np.testing.assert_allclose(r2[:30, :3], r1[:30, :3], atol=1e-6)
    expect_quat = np.array([_quat_mul(q, qz) for q in r1[:30, 3:7]])
    sign = np.sign(np.sum(expect_quat * r2[:30, 3:7], axis=1))[:, None]
    np.testing.assert_allclose(r2[:30, 3:7] * sign, expect_quat,
                               atol=1e-6)
    np.testing.assert_allclose(j2, j1[:, jp], atol=2e-2)


def test_wrapper_reduces_rotated_command_to_forward(env):
    """Rot60Policy: a +120deg command must reach the inner model as a
    wedge command with legs relabeled; the returned action must be the
    inverse relabel of what the inner model produced."""

    class Probe:
        def __init__(self):
            self.seen = None

        def predict(self, obs, deterministic=True, **kw):
            self.seen = np.asarray(obs, dtype=np.float64).copy()
            return np.arange(18, dtype=np.float64), None

    rng = np.random.default_rng(4)
    frames = 16
    obs = np.concatenate([_frame(rng) for _ in range(frames)])
    # command +120 deg at 0.05 m/s in EVERY frame (steady command)
    vx, vy = 0.05 * math.cos(math.radians(120)), \
        0.05 * math.sin(math.radians(120))
    for f in range(frames):
        obs[f * 72 + 68:f * 72 + 70] = [vx / 0.05, vy / 0.05]
        obs[f * 72 + 70:f * 72 + 72] = [vx / 0.05, vy / 0.05]
    probe = Probe()
    pol = rot60.Rot60Policy(probe)
    act, _ = pol.predict(obs)
    assert pol.k == 2
    # inner model saw the command rotated into the wedge (heading 0)
    seen = probe.seen.reshape(frames, 72)
    for f in range(frames):
        h = math.degrees(math.atan2(seen[f, 69], seen[f, 68]))
        assert abs(h) <= 30.0 + 1e-6
    np.testing.assert_array_equal(
        act, np.arange(18)[rot60.leg_perm(-2)])
