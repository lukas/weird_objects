"""amp_features.py — backend-agnostic AMP discriminator feature vector
(rl_docs/AMP_LOCOMOTION.md §3.6), shared between the offline
motion-library builder (real MuJoCo ``MjData``,
``build_motion_library.py``) and the live batched trainer
(``mjx_vec_env.MjxVecEnv``'s per-env ``mjx_host.FakeData`` shim).

WHY THIS FILE EXISTS (08-22 M1 gap found while wiring the discriminator
into the live trainer): ``build_motion_library.py`` computes the
world->body rotation from ``data.xquat`` (a real ``MjData`` field). The
batched MJX/Warp vec env's per-env host mirror (``mjx_host.FakeData``,
the ONLY per-env state the live trainer's shim envs — and therefore any
in-training discriminator reward — can read) has NO ``xquat`` field at
all; it only carries ``qpos, qvel, qfrc_actuator, xpos, xmat,
subtree_com, sensordata`` (see ``mjx_host.FakeData.__init__``). A
discriminator feature function written against ``xquat`` would work
standalone against the offline npz forever and then hard-fail (missing
attribute) the moment someone wires it into the real training loop —
exactly the kind of gap the AMP brief's M1/M0 "reuse before building"
audit is supposed to catch before a wave burns a training budget on it.

FIX: rotate with the body's ``xmat`` (row-major 3x3 body->world
rotation matrix — a real ``MjData`` field too, already used elsewhere
in this file's neighborhood for the identical purpose, e.g.
``walk_task.py``'s ``_body_vel_xy``/``_body_wz``: ``R = xmat.reshape(3,3);
R.T @ v_world`` is world->body). This is mathematically IDENTICAL to
the quaternion rotation for the same physical orientation (both encode
the same body->world rotation) — proven by
``test_amp_features.test_xmat_matches_xquat_rotation`` on a real
teacher rollout, not asserted by inspection. ``build_motion_library.py``
is NOT touched by this fix (its shipped ``teacher_v1.npz`` stays as-is,
no re-generation, no re-validation churn for zero behavior change) —
this module is the ONE place both backends can share going forward.

Public API:

- ``chassis_pad_gyro_ids(env)``: pulls the four model-derived ids/adrs
  (``chassis_bid``, ``pad_bids``, ``gyro_adr``, ``qadr``, ``vadr``) any
  ``SimHexapodBalanceEnv`` subclass already sets in ``__init__``
  (``sim_env.py``) — identical on the CPU env and the MJX shim env
  (same model, same layout; verified by
  ``test_mjx_vecenv_obs_style_batched``), so callers never touch
  private attributes directly.
- ``obs_style_from_data(data, ids, neutral_qpos)``: the 60-dim
  (joint_pos_rel_neutral[18] + joint_vel[18] + base_angular_velocity[3]
  + projected_gravity[3] + foot_positions_rel_body[18]) vector for ONE
  env at the CURRENT tick, from any object exposing
  ``qpos/qvel/xpos/xmat/sensordata`` (real ``MjData`` or
  ``mjx_host.FakeData`` both qualify).
- ``obs_style_batch(datas, ids, neutral_qpos_batch)``: stacks
  ``obs_style_from_data`` over a list of per-env data objects (e.g.
  ``[e.data for e in vecenv.envs]``) -> ``(n_envs, 60)``.

NOT YET WIRED into the live reward loop (that is a separate,
larger change to ``train_ppo_mjx.py``'s reward computation + an online
discriminator-update step — tracked in ``rl_docs/tracks/amp/STATUS.md``).
This module and its tests close the specific gap above: the SAME
feature function now runs against both physics backends, and
``test_amp_features.py``'s MJX-batched test feeds ACTUAL rollout
transitions (not synthetic noise/shuffle) through the discriminator
for the first time.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ObsStyleIds:
    chassis_bid: int
    pad_bids: tuple
    gyro_adr: int
    qadr: np.ndarray
    vadr: np.ndarray


def chassis_pad_gyro_ids(env) -> ObsStyleIds:
    """Pull the model-derived ids ``sim_env.py`` sets on ANY
    ``SimHexapodBalanceEnv`` subclass in ``__init__`` — identical
    across the CPU env and the MJX shim (same shared model)."""
    return ObsStyleIds(
        chassis_bid=int(env._chassis_bid),
        pad_bids=tuple(int(b) for b in env._pad_bids),
        gyro_adr=int(env._gyro_adr),
        qadr=np.asarray(env._qadr),
        vadr=np.asarray(env._vadr),
    )


def obs_style_from_data(data, ids: ObsStyleIds,
                         neutral_qpos: np.ndarray) -> np.ndarray:
    """60-dim AMP discriminator feature vector for one env at the
    current tick of ``data`` (real ``MjData`` or ``mjx_host.FakeData``).

    World->body rotation via ``xmat`` (present on BOTH backends;
    ``xquat`` is NOT present on the MJX shim's ``FakeData`` — see
    module docstring). ``R = xmat[chassis_bid].reshape(3,3)`` is the
    body->world rotation matrix (MuJoCo convention), so ``R.T @ v``
    rotates a world-frame vector into the body frame — the same
    operation ``walk_task.py``'s ``_body_vel_xy``/``_body_wz`` already
    use for velocity, applied here to gravity and foot offsets.
    """
    qpos = np.asarray(data.qpos, dtype=np.float64)[ids.qadr]
    qvel = np.asarray(data.qvel, dtype=np.float64)[ids.vadr]
    gyro = np.asarray(data.sensordata, dtype=np.float64)[
        ids.gyro_adr:ids.gyro_adr + 3]
    R = np.asarray(data.xmat[ids.chassis_bid], dtype=np.float64).reshape(3, 3)
    proj_grav = R.T @ np.array([0.0, 0.0, -1.0])
    chassis_xyz = np.asarray(data.xpos[ids.chassis_bid], dtype=np.float64)
    feet = []
    for b in ids.pad_bids:
        rel_world = np.asarray(data.xpos[b], dtype=np.float64) - chassis_xyz
        feet.append(R.T @ rel_world)
    foot_pos_body = np.concatenate(feet)
    return np.concatenate([
        qpos - np.asarray(neutral_qpos, dtype=np.float64),
        qvel, gyro, proj_grav, foot_pos_body,
    ]).astype(np.float32)


def obs_style_batch(datas, ids: ObsStyleIds,
                     neutral_qpos_batch: np.ndarray) -> np.ndarray:
    """Stack ``obs_style_from_data`` over a batch of per-env data
    objects. ``neutral_qpos_batch``: (n_envs, n_qpos) or a single
    (n_qpos,) row broadcast to every env."""
    neutral_qpos_batch = np.asarray(neutral_qpos_batch, dtype=np.float64)
    if neutral_qpos_batch.ndim == 1:
        neutral_qpos_batch = np.broadcast_to(
            neutral_qpos_batch, (len(datas), neutral_qpos_batch.shape[0]))
    return np.stack([
        obs_style_from_data(d, ids, neutral_qpos_batch[i])
        for i, d in enumerate(datas)
    ], axis=0)
