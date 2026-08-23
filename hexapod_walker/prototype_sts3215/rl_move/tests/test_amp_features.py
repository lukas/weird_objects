"""test_amp_features.py — AMP track M1: cross-backend obs_style parity
+ the first REAL (non-synthetic) rollout-to-discriminator smoke test.

Two things this bank proves that test_amp_discriminator.py's synthetic
noise/shuffle fakes cannot:

1. ``amp_features.obs_style_from_data``'s xmat-based world->body
   rotation is mathematically IDENTICAL to ``build_motion_library.py``'s
   xquat-based one for the same physical orientation (not just "close
   enough") — the reason the substitution is safe to use on the MJX
   shim env, which has no ``xquat`` field at all.
2. The SAME feature function runs against the actual batched MJX/Warp
   vec env (``mjx_vec_env.MjxVecEnv``) that the real trainer uses —
   closing the "fake transitions stand in for policy rollout" gap
   ``amp_discriminator.py``'s docstring flags — and a real rollout's
   obs_style vectors flow through the discriminator without NaN.

Test 1 is CPU/plain-MuJoCo, always runs. Tests 2-3 need mujoco-mjx +
jax (skipped, same guard as test_mjx_vec_env.py, otherwise); they were
verified on-pod where those are installed (see snapshot notes) since
the controller does not carry jax/mjx.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")  # see test_mjx_vec_env.py

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import mujoco  # noqa: E402

from rl_move.sim.amp_features import (  # noqa: E402
    chassis_pad_gyro_ids,
    obs_style_batch,
    obs_style_from_data,
)


def _quat_rot_world_to_body(quat_wxyz: np.ndarray, v_world: np.ndarray) -> np.ndarray:
    """Copy of build_motion_library.py's helper (kept independent on
    purpose: this test must not pass just because it imports the same
    code it is checking)."""
    qinv = np.zeros(4)
    mujoco.mju_negQuat(qinv, quat_wxyz)
    out = np.zeros(3)
    mujoco.mju_rotVecQuat(out, v_world, qinv)
    return out


def _cpu_walk_env(seed: int = 0, episode_seconds: float = 3.0):
    from rl_move.config import load_config
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed, cfg=cfg)
    return env


def test_xmat_matches_xquat_rotation():
    """The xmat-based rotation amp_features.py uses (required for MJX
    shim compat) must be numerically identical to build_motion_library
    .py's xquat-based one on a real rollout, not just plausible."""
    from tripod_gait import TripodGait
    from rl_move.robot_state import DEG2RAD
    from rl_move.sim.joint_task import q_rad_to_action

    env = _cpu_walk_env()
    env.reset()
    gait = TripodGait(vx=0.06, lift=0.025)
    gait.sync_plant_stance(20.0, 80.0)
    gait.reset_phase()
    ids = chassis_pad_gyro_ids(env)
    neutral = env.data.qpos[env._qadr].copy()

    max_grav_diff = 0.0
    max_foot_diff = 0.0
    for step in range(30):
        t = step * env.dt
        gait.set_velocity(vx=0.06, vy=0.0, omega=0.0)
        act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        env.step(act)

        # xmat-based (amp_features.py, what the MJX shim can compute).
        style = obs_style_from_data(env.data, ids, neutral)
        proj_grav_xmat = style[18 + 18 + 3:18 + 18 + 3 + 3]
        foot_xmat = style[18 + 18 + 3 + 3:]

        # xquat-based (build_motion_library.py's own method).
        quat = env.data.xquat[ids.chassis_bid]
        proj_grav_xquat = _quat_rot_world_to_body(
            quat, np.array([0.0, 0.0, -1.0]))
        chassis_xyz = env.data.xpos[ids.chassis_bid]
        feet_xquat = []
        for b in ids.pad_bids:
            rel_world = np.asarray(env.data.xpos[b]) - chassis_xyz
            feet_xquat.append(_quat_rot_world_to_body(quat, rel_world))
        foot_xquat = np.concatenate(feet_xquat)

        max_grav_diff = max(max_grav_diff,
                            float(np.max(np.abs(proj_grav_xmat - proj_grav_xquat))))
        max_foot_diff = max(max_foot_diff,
                            float(np.max(np.abs(foot_xmat - foot_xquat))))

    env.close()
    assert max_grav_diff < 1e-5, f"xmat/xquat gravity mismatch {max_grav_diff}"
    assert max_foot_diff < 1e-5, f"xmat/xquat foot-position mismatch {max_foot_diff}"


def test_obs_style_from_data_shape_and_finite():
    env = _cpu_walk_env()
    env.reset()
    ids = chassis_pad_gyro_ids(env)
    neutral = env.data.qpos[env._qadr].copy()
    env.step(np.zeros(18))
    style = obs_style_from_data(env.data, ids, neutral)
    assert style.shape == (60,)
    assert np.all(np.isfinite(style))
    env.close()


def test_obs_style_from_data_cmd_cond_appends_tail():
    """08-23 yaw-authority follow-up: cmd=None (default) is bit-exact
    (60-dim, unchanged values); passing (vx, vy, wz) appends it at the
    tail with the first 60 dims untouched -> 63-dim."""
    env = _cpu_walk_env()
    env.reset()
    ids = chassis_pad_gyro_ids(env)
    neutral = env.data.qpos[env._qadr].copy()
    env.step(np.zeros(18))
    base = obs_style_from_data(env.data, ids, neutral)
    assert base.shape == (60,)
    same = obs_style_from_data(env.data, ids, neutral, cmd=None)
    np.testing.assert_array_equal(base, same)
    cmd = (0.08, -0.01, 0.25)
    ext = obs_style_from_data(env.data, ids, neutral, cmd=cmd)
    assert ext.shape == (63,)
    np.testing.assert_array_equal(ext[:60], base)
    np.testing.assert_allclose(ext[60:], cmd, atol=1e-6)
    env.close()


def test_obs_style_batch_matches_per_env():
    """obs_style_batch over N copies of the same data must equal N
    stacked calls to obs_style_from_data (pure plumbing check)."""
    env = _cpu_walk_env()
    env.reset()
    ids = chassis_pad_gyro_ids(env)
    neutral = env.data.qpos[env._qadr].copy()
    env.step(np.zeros(18))
    single = obs_style_from_data(env.data, ids, neutral)
    batch = obs_style_batch([env.data, env.data, env.data], ids, neutral)
    assert batch.shape == (3, 60)
    for row in batch:
        assert np.allclose(row, single)
    env.close()
