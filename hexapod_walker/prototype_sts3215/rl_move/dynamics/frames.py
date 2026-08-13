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

Input sets for the encoder (targets always use the full frame):

    "full"  all 86 dims — the strongest v1 representation.
    "obs"   the 59 dims present in the deployed policy observation
            contract (q, qd, tilt, gyro, prev_action). REQUIRED before
            any PPO wiring (conditions B/C in rl_docs/DYNREP.md): the
            encoder can only feed a policy from channels the policy
            actually receives at deploy time.

Alongside each frame the collector stores the EXECUTED action a_t
(frames F = T+1 states s_0..s_T, actions length T) plus a small
privileged sidecar (body-frame vx, vy, wz, chassis z) for analysis
only — never model input.
"""
from __future__ import annotations

import numpy as np

LAYOUT_VERSION = "v2"

N_JOINTS = 18
FRAME_DIM = 86
ACTION_DIM = 18
PRIV_DIM = 4          # body-frame vx, vy, wz, chassis z (analysis only)

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


def extract_priv(env) -> np.ndarray:
    """Privileged sidecar for later analysis (never a model input)."""
    vx = vy = wz = 0.0
    if hasattr(env, "_body_vel_xy"):
        vx, vy = (float(v) for v in env._body_vel_xy())
    if hasattr(env, "_body_wz"):
        wz = float(env._body_wz())
    z = float(env.data.xpos[env._chassis_bid, 2])
    return np.array([vx, vy, wz, z], dtype=np.float32)
