"""Rot-60 exact-equivariance canonicalization for hexapod walk policies.

Why this exists (TURN.md / RL_PLAN queue 2.1, 08-11): omni-directional
translation ("walk where the joystick points") collapsed four times on
the trans1 stack — freeze, leg-sacrifice, paddle-stall, march-in-place
— despite the income probe proving the honest gait out-earns every
degenerate 2-4x in every direction. PPO cannot DISCOVER the rotated
gait each heading needs. But it does not have to: the robot is a
regular hexagon — six IDENTICAL leg templates mounted at exactly
(i+0.5)*60 deg (mujoco_prototype._leg_xml, one quat=Rz per leg), an
AXISYMMETRIC chassis inertia (diaginertia 0.006 0.006 0.010), and
identical per-leg actuators. Rotating the world by 60 deg and
relabeling legs is an EXACT symmetry of the model (unlike the mirror,
which the COXA_HIP_ANCHOR_Y pinwheel only approximates — mirror.py).
So walking at heading theta is EXACTLY walking at heading theta-60
with legs relabeled: a policy that tracks the +/-30 deg wedge tracks
the full circle through this wrapper, zero training required.

Canonicalization contract (all maps in this file):

  sector    k = nearest multiple of 60 deg to the commanded heading
            (hysteresis + hold-on-zero-command so k never chatters)
  alpha     = k * 60 deg. Canonical body frame = real body frame
            rotated +alpha about body z (so the canonical-frame
            command heading lands in [-30, 30] deg).
  vectors   v_canon = Rz(-alpha) @ v_real for every body-frame xy
            pair: gyro xy, velocity refs, velocity meas. gyro z
            unchanged.
  tilt      (roll, pitch) converted to the gravity direction via the
            IMU convention (roll = atan2(ay, az)), gravity xy rotated
            by Rz(-alpha), converted back. Exact for absolute tilt;
            obs carry tilt RELATIVE to the episode-start attitude, so
            the map is exact to second order in (tilt * tilt_ref) —
            tilt_ref is the settled level stance (~0).
  legs      canonical leg j = real leg (j+k) mod 6 for q_rel, qd,
            prev_action and the unload/lift one-hot (leg i sits at
            azimuth (i+0.5)*60, so a +alpha frame rotation advances
            indices by k). No sign flips anywhere — rotation
            preserves handedness (the mirror's yaw negation does not
            apply).
  actions   real action leg i = canonical action leg (i-k) mod 6
            (inverse permutation; the [-1,1]->rad map is per-AXIS
            affine and identical across legs — joint_task).

Obs layout canonicalized here = mirror.py's walk frame (width 72,
stacked newest-first): q_rel 18 | qd 18 | tilt 2 | gyro 3 |
prev_action 18 | roll/pitch/height ref 3 | leg one-hot 6 |
vx,vy ref 2 | vx,vy meas 2. Phase-clock / yaw-command tails are NOT
supported (assert) — the omni-translation stack trains with neither.

tests/test_rot60.py locks: map round-trips, k=0 identity, the tilt
convention against rotation matrices, AND raw-MuJoCo dynamics
equivariance (rotate+relabel the state, permute the ctrl targets,
step 200x, trajectories match) — the physics-level proof that the
compiled model really is rot-60 exact.
"""
from __future__ import annotations

import math

import numpy as np

N_LEGS = 6
N_JOINTS = 18
FRAME_WALK = 72          # matches mirror.py FRAME_WALK
SECTOR_RAD = math.pi / 3.0

# per-frame slices (walk frame)
_SL_Q = slice(0, 18)
_SL_QD = slice(18, 36)
_SL_TILT = slice(36, 38)      # roll, pitch (scaled by 1/tilt_scale)
_SL_GYRO = slice(38, 41)      # gx, gy, gz
_SL_PREV = slice(41, 59)
_SL_RPREF = slice(59, 61)     # roll_ref, pitch_ref (same tilt scaling)
# 61 height_ref (invariant)
_SL_ONEHOT = slice(62, 68)    # per-leg unload/lift one-hot
_SL_VREF = slice(68, 70)      # vx_ref, vy_ref
_SL_VMEAS = slice(70, 72)     # vx_meas, vy_meas
OBS_VREF_X = 68               # newest-frame command indices (frame 0)
OBS_VREF_Y = 69


def leg_perm(k: int) -> np.ndarray:
    """18-wide index map: canonical joint vector = real[leg_perm(k)].

    Canonical leg j takes real leg (j+k) mod 6; yaw/pitch/knee order
    preserved, no sign flips.
    """
    legs = (np.arange(N_LEGS) + k) % N_LEGS
    return (3 * legs[:, None] + np.arange(3)[None, :]).reshape(-1)


def one_hot_perm(k: int) -> np.ndarray:
    return (np.arange(N_LEGS) + k) % N_LEGS


def rot2(alpha: float) -> np.ndarray:
    c, s = math.cos(alpha), math.sin(alpha)
    return np.array([[c, -s], [s, c]], dtype=float)


def tilt_rotate(roll: float, pitch: float, alpha: float
                ) -> tuple[float, float]:
    """Exact (roll, pitch) under a +alpha body-z frame relabel.

    IMU convention (sim_env: roll = atan2(ay, az)): static accel
    a = (-sin p, cos p sin r, cos p cos r). Canonical-frame accel is
    Rz(-alpha) @ a; convert back.
    """
    ax = -math.sin(pitch)
    ay = math.cos(pitch) * math.sin(roll)
    az = math.cos(pitch) * math.cos(roll)
    c, s = math.cos(-alpha), math.sin(-alpha)
    axc = c * ax - s * ay
    ayc = s * ax + c * ay
    roll_c = math.atan2(ayc, az)
    pitch_c = math.atan2(-axc, math.hypot(ayc, az))
    return roll_c, pitch_c


def frame_transform(frame: np.ndarray, k: int, *,
                    tilt_scale: float = 0.2) -> np.ndarray:
    """Canonicalize ONE walk obs frame (width 72) by sector k."""
    if frame.shape[-1] != FRAME_WALK:
        raise ValueError(
            f"rot60 supports the walk frame width {FRAME_WALK} only "
            f"(no phase/yaw tails); got {frame.shape[-1]}")
    k = int(k) % N_LEGS
    if k == 0:
        return frame.copy()
    alpha = k * SECTOR_RAD
    R = rot2(-alpha)
    jp = leg_perm(k)
    out = frame.copy()
    out[_SL_Q] = frame[_SL_Q][jp]
    out[_SL_QD] = frame[_SL_QD][jp]
    r, p = tilt_rotate(frame[36] * tilt_scale, frame[37] * tilt_scale,
                       alpha)
    out[36] = r / tilt_scale
    out[37] = p / tilt_scale
    out[38:40] = R @ frame[38:40]
    # gyro z (40) unchanged
    out[_SL_PREV] = frame[_SL_PREV][jp]
    rr, pr = tilt_rotate(frame[59] * tilt_scale, frame[60] * tilt_scale,
                         alpha)
    out[59] = rr / tilt_scale
    out[60] = pr / tilt_scale
    # 61 height_ref unchanged
    out[_SL_ONEHOT] = frame[_SL_ONEHOT][one_hot_perm(k)]
    out[_SL_VREF] = R @ frame[_SL_VREF]
    out[_SL_VMEAS] = R @ frame[_SL_VMEAS]
    return out


def obs_transform(obs: np.ndarray, k: int, *,
                  tilt_scale: float = 0.2) -> np.ndarray:
    """Canonicalize a stacked obs (frames * 72, newest-first)."""
    obs = np.asarray(obs, dtype=np.float32)
    if obs.ndim != 1 or obs.size % FRAME_WALK:
        raise ValueError(f"obs size {obs.shape} not a multiple of "
                         f"{FRAME_WALK}")
    if int(k) % N_LEGS == 0:
        return obs.copy()
    frames = obs.reshape(-1, FRAME_WALK)
    return np.concatenate(
        [frame_transform(f, k, tilt_scale=tilt_scale) for f in frames]
    ).astype(np.float32)


def action_from_canonical(act: np.ndarray, k: int) -> np.ndarray:
    """Map the policy's canonical-frame action back to real legs."""
    if int(k) % N_LEGS == 0:
        return np.asarray(act).copy()
    # canonical[j] = real[(j+k)]  =>  real = canonical[leg_perm(-k)]
    return np.asarray(act)[leg_perm(-k)]


def sector_from_cmd(vx: float, vy: float, last_k: int = 0, *,
                    hysteresis_deg: float = 6.0,
                    min_speed: float = 1e-3) -> int:
    """Sector index k in [-2..3] from the commanded heading.

    Holds the previous sector on ~zero commands (heading undefined)
    and inside a +/-hysteresis band past the 30 deg boundary so k
    never chatters while an operator sweeps the stick.
    """
    if math.hypot(vx, vy) < min_speed:
        return last_k
    theta = math.atan2(vy, vx)
    # hysteresis: keep last_k while inside its widened wedge
    rel = (theta - last_k * SECTOR_RAD + math.pi) % (2 * math.pi) - math.pi
    if abs(rel) <= SECTOR_RAD / 2 + math.radians(hysteresis_deg):
        return last_k
    k = int(round(theta / SECTOR_RAD))
    return ((k + 2) % N_LEGS) - 2


class Rot60Policy:
    """SB3-predict-compatible wrapper: canonicalize obs, restore action.

    Reads the commanded (vx_ref, vy_ref) straight from the newest obs
    frame (indices 68:70 — scale-invariant for the heading), so it
    needs no side channel and deploys anywhere the obs contract holds.
    """

    def __init__(self, model, *, tilt_scale: float = 0.2):
        self.model = model
        self.tilt_scale = float(tilt_scale)
        self.k = 0

    def reset(self):
        self.k = 0

    def predict(self, obs, deterministic: bool = True, **kw):
        obs = np.asarray(obs)
        squeeze = obs.ndim == 1
        rows = obs[None, :] if squeeze else obs
        acts = []
        for row in rows:
            self.k = sector_from_cmd(float(row[OBS_VREF_X]),
                                     float(row[OBS_VREF_Y]), self.k)
            canon = obs_transform(row, self.k, tilt_scale=self.tilt_scale)
            a, _ = self.model.predict(canon, deterministic=deterministic,
                                      **kw)
            acts.append(action_from_canonical(a, self.k))
        out = np.asarray(acts)
        return (out[0] if squeeze else out), None
