"""frames.py — the canonical per-timestep feature frame for dynrep.

One FRAME = everything the robot could plausibly sense at one control
tick (25 Hz), extracted from the sim env AFTER the env has applied its
own sensor corruption (DR joint-zero bias, encoder noise, IMU mount
rotation, complementary-filter attitude...). The dynamics model never
sees privileged ground truth as input — only what a real robot would.

Layout (float32, FRAME_DIM = 86), version "v2":

    [ 0:18]  q          joint positions RELATIVE to the episode's
                        settled start pose q_nom, rad. This matches the
                        policy observation contract exactly (build_obs
                        emits (q - q_nom)/q_scale and q_nom is captured
                        per episode at reset), so an encoder pretrained
                        on these frames can be fed from the obs at
                        deploy time. Absolute q is NOT recoverable from
                        the obs; q_nom is kept as episode metadata so
                        analysis can reconstruct it. (v1 stored
                        absolute q — incompatible with PPO wiring.)
    [18:36]  qd         joint velocities, rad/s
    [36:38]  tilt       roll, pitch relative to episode-start attitude, rad
                        (same frame the policy obs uses — removes IMU
                        mount / floor-slope bias)
    [38:41]  gyro       body angular velocity, rad/s
    [41:44]  accel      IMU specific force, m/s^2 (substep-averaged)
    [44:62]  current    per-servo current estimate, A
    [62:68]  contact    per-foot touch force, N (L0..L5)
    [68:86]  prev_action  previous [-1,1]^18 joint action

STATE = frame[0:44] (q, qd, tilt, gyro, accel) — the raw physical state
the short-horizon heads predict directly. Contacts are predicted as
binary (force > CONTACT_THRESH_N) via BCE.

PRIV = privileged simulator truths stored as targets only. They are
never model inputs, even for the "full" input set:

    [ 0:3]  body_vel     true chassis linear velocity in body frame, m/s
    [ 3:4]  body_wz      true chassis yaw rate about body z, rad/s
    [ 4:5]  chassis_z    world chassis height, m
    [ 5:7]  yaw_rel      sin/cos of chassis yaw relative to episode start
    [ 7:10] cmd          current walk command vx, vy, wz
    [10:12] cmd_vel      achieved velocity along/across commanded xy dir
    [12:14] cmd_heading  sin/cos of commanded xy direction (0/1 if stop)

Input sets for the encoder (targets always use the full frame):

    "full"  all 86 dims — the strongest v1 representation.
    "obs"   the 59 dims present in the deployed policy observation
            contract (q, qd, tilt, gyro, prev_action). REQUIRED before
            any PPO wiring (conditions B/C in rl_docs/DYNREP.md): the
            encoder can only feed a policy from channels the policy
            actually receives at deploy time.

Alongside each frame the collector stores the EXECUTED action a_t
(frames F = T+1 states s_0..s_T, actions length T) plus the privileged
sidecar above for supervised targets / diagnostics only — never model
input.
"""
from __future__ import annotations

import math

import numpy as np

LAYOUT_VERSION = "v2"

N_JOINTS = 18
FRAME_DIM = 86
ACTION_DIM = 18
PRIV_DIM = 14
LEGACY_PRIV_DIM = 4  # old shards: body-frame vx, vy, wz, chassis z
PRIV_NAMES = (
    "vx_body", "vy_body", "vz_body", "wz_body", "chassis_z",
    "sin_yaw_rel", "cos_yaw_rel",
    "vx_ref", "vy_ref", "wz_ref",
    "v_along_cmd", "v_cross_cmd",
    "sin_cmd_heading", "cos_cmd_heading",
)

# (name, start, stop) — contiguous channel groups, in frame order.
GROUPS = (
    ("q", 0, 18),
    ("qd", 18, 36),
    ("tilt", 36, 38),
    ("gyro", 38, 41),
    ("accel", 41, 44),
    ("current", 44, 62),
    ("contact", 62, 68),
    ("prev_action", 68, 86),
)

STATE_SLICE = slice(0, 44)       # q + qd + tilt + gyro + accel
STATE_DIM = 44
CONTACT_SLICE = slice(62, 68)
N_FEET = 6
CONTACT_THRESH_N = 0.5           # same "foot is on" threshold the env uses

# Encoder input column indices per input set.
_OBS_IDX = np.concatenate([np.arange(0, 41), np.arange(68, 86)])
INPUT_SETS = {
    "full": np.arange(FRAME_DIM),
    "obs": _OBS_IDX,             # 59 dims = deployed policy proprio
}


def group_slices() -> dict[str, slice]:
    return {name: slice(a, b) for name, a, b in GROUPS}


def extract_frame(env, prev_action: np.ndarray) -> np.ndarray:
    """Build one frame from a live sim env (after reset or step).

    Reads the env's own corrupted RobotState (``env._state``) plus the
    per-foot touch sensors, so the frame matches what a policy /
    hardware estimator would see. q is stored relative to the
    episode's ``env._q_nom`` (the obs contract).
    """
    st = env._state
    tr = getattr(env, "_tilt_ref0", (0.0, 0.0))
    contact = np.zeros(N_FEET)
    for i, adr in enumerate(env._touch_adr):
        if adr >= 0:
            contact[i] = max(float(env.data.sensordata[adr]), 0.0)
    current = (st.servo_current if st.servo_current is not None
               else np.zeros(N_JOINTS))
    frame = np.concatenate([
        st.joint_position - env._q_nom,
        st.joint_velocity,
        [st.imu_roll - tr[0], st.imu_pitch - tr[1]],
        st.imu_gyro,
        st.imu_accel,
        current,
        contact,
        prev_action,
    ]).astype(np.float32)
    assert frame.shape == (FRAME_DIM,)
    return frame


def _body_heading(env) -> float:
    R = env.data.xmat[env._chassis_bid].reshape(3, 3)
    return float(math.atan2(R[1, 0], R[0, 0]))


def _wrap_pi(x: float) -> float:
    return float((x + math.pi) % (2.0 * math.pi) - math.pi)


def reset_priv_episode(env) -> None:
    """Reset episode-relative privileged-target references."""
    env._dynrep_yaw0 = _body_heading(env)


def extract_priv(env) -> np.ndarray:
    """Privileged simulator truths (targets only; never model input)."""
    R = env.data.xmat[env._chassis_bid].reshape(3, 3)
    v_body = (R.T @ env.data.qvel[:3]).astype(np.float32)
    if hasattr(env, "_body_vel_xy"):
        v_body[:2] = np.asarray(env._body_vel_xy(), dtype=np.float32)
    wz = 0.0
    if hasattr(env, "_body_wz"):
        wz = float(env._body_wz())
    else:
        wz = float((R.T @ env.data.qvel[3:6])[2])
    z = float(env.data.xpos[env._chassis_bid, 2])
    yaw = _body_heading(env)
    if not hasattr(env, "_dynrep_yaw0"):
        reset_priv_episode(env)
    yaw_rel = _wrap_pi(yaw - float(env._dynrep_yaw0))

    vx_ref = vy_ref = wz_ref = 0.0
    goal = None
    if hasattr(env, "_current_goal"):
        try:
            goal = env._current_goal()
        except Exception:
            goal = None
    if goal is not None:
        vx_ref = float(getattr(goal, "vx_ref", 0.0))
        vy_ref = float(getattr(goal, "vy_ref", 0.0))
        wz_ref = float(getattr(goal, "wz_ref", 0.0))
    s_ref = math.hypot(vx_ref, vy_ref)
    if s_ref > 1e-6:
        ux, uy = vx_ref / s_ref, vy_ref / s_ref
        v_along = float(v_body[0] * ux + v_body[1] * uy)
        v_cross = float(-v_body[0] * uy + v_body[1] * ux)
        sin_cmd, cos_cmd = uy, ux
    else:
        v_along = v_cross = 0.0
        sin_cmd, cos_cmd = 0.0, 1.0
    priv = np.array([
        float(v_body[0]), float(v_body[1]), float(v_body[2]), wz, z,
        math.sin(yaw_rel), math.cos(yaw_rel),
        vx_ref, vy_ref, wz_ref,
        v_along, v_cross, sin_cmd, cos_cmd,
    ], dtype=np.float32)
    assert priv.shape == (PRIV_DIM,)
    return priv


def upgrade_priv(priv: np.ndarray) -> np.ndarray:
    """Pad legacy sidecars to the current privileged-target layout."""
    priv = np.asarray(priv, dtype=np.float32)
    if priv.shape[-1] == PRIV_DIM:
        return priv
    if priv.shape[-1] != LEGACY_PRIV_DIM:
        raise ValueError(
            f"priv sidecar has dim {priv.shape[-1]}, expected "
            f"{PRIV_DIM} or legacy {LEGACY_PRIV_DIM}")
    out = np.zeros((*priv.shape[:-1], PRIV_DIM), dtype=np.float32)
    out[..., 0] = priv[..., 0]          # vx_body
    out[..., 1] = priv[..., 1]          # vy_body
    out[..., 3] = priv[..., 2]          # wz_body
    out[..., 4] = priv[..., 3]          # chassis_z
    out[..., 6] = 1.0                   # cos_yaw_rel unknown -> 0 rad
    out[..., 13] = 1.0                  # cos_cmd_heading stop/default
    return out
