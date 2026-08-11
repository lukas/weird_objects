"""Leg-odometry body-velocity estimator (numpy only, board-safe).

THE ESTIMATOR RUNG (operator directive 2026-08-11 — WISHLIST item 22
un-deferred). Why it exists: every policy trained with the deployable
meas := ref contract (walk_obs_body_vel=2, the cw-dep line) is BLIND —
two of its four velocity obs dims carry zero information by
construction, so it can never observe slip, stall, or incipient
tipping. Measured consequence: those policies crouch 50-77 mm below
the plant stance and creep (the open-loop-safe posture), while
privileged-velocity policies (mode 1, e.g. the sim champion
longdist_r2) stand tall and track. The velocity signal, not the
reward, buys the upright gait.

The robot has no velocity sensor, but it does not need one: on a
walking hexapod some feet are always planted, and a planted foot is
stationary in the world. For stance foot i at body-frame position
r_i(q):

    0 = v_body + omega x r_i + dr_i/dt   =>   v_body = -(omega x r_i
                                                          + dr_i/dt)

with r_i from forward kinematics (body_ik.fk_all_feet — the same leg
geometry the IK/gait code uses), dr_i/dt from differencing FK across
control ticks, and omega from the gyro. Average over the stance set,
low-pass, done. This is standard legged-robot leg odometry.

Deployment contract: this module is plain numpy and consumes ONLY
signals the board produces at 25 Hz (encoder positions, IMU gyro +
tilt). The sim feeds it the DR-corrupted observed state
(sim_env._read_state applies encoder noise / zero bias / IMU mount
error), so training sees estimator output with hardware-realistic
error — no separate noise model needed, and the trained policy's
velocity obs is bit-honest at deploy time.

Stance inference has no contact sensor to lean on: feet whose
gravity-aligned height is within STANCE_Z_TOL_M of the lowest foot
count as stance. Wrong picks (a foot chosen during early swing) show
up as outlier per-foot velocities; the mean over >=2-3 stance feet
plus the low-pass keeps the estimate serviceable (quantified in
tests/test_estimator.py and the sim-rollout validation).

State is two small arrays; instances deep-copy cleanly, which the MJX
pooled-reset snapshot machinery requires (mjx_host.SNAP_ATTRS lesson:
per-episode state MUST snapshot/restore, or pooled episodes inherit a
stale estimate).
"""
from __future__ import annotations

import math

import numpy as np

from .body_ik import fk_all_feet

N_LEGS = 6

# Feet within this height of the lowest (gravity-aligned) foot count as
# stance. Wide enough to keep 3 tripod feet through weight transfer,
# tight enough to reject mid-swing feet (swing lift is 25-40 mm).
STANCE_Z_TOL_M = 0.012
# One-pole low-pass on the output. alpha 0.3 at 25 Hz ~ 0.13 s time
# constant — cuts FK-differencing encoder noise ~3x while lagging a
# 0.05 m/s command ramp by only a few ticks.
DEFAULT_ALPHA = 0.3


class LegOdometryVelocity:
    """Stateful 25 Hz leg-odometry estimator -> body-frame (vx, vy) m/s.

    Call ``update(q_rad, gyro_rad_s, roll_rad, pitch_rad)`` once per
    control tick; call ``reset()`` at episode start. First tick after
    reset returns zeros (no previous FK frame to difference).
    """

    def __init__(self, dt: float, *, alpha: float = DEFAULT_ALPHA,
                 stance_z_tol_m: float = STANCE_Z_TOL_M):
        self.dt = float(dt)
        self.alpha = float(alpha)
        self.stance_z_tol_m = float(stance_z_tol_m)
        self._prev_feet: np.ndarray | None = None
        self._prev_stance: np.ndarray | None = None
        self._v_filt = np.zeros(2, dtype=float)

    def reset(self) -> None:
        self._prev_feet = None
        self._prev_stance = None
        self._v_filt = np.zeros(2, dtype=float)

    def _stance_mask(self, feet: np.ndarray, roll: float, pitch: float
                     ) -> np.ndarray:
        # Gravity-aligned height of each foot: z component of the foot
        # position rotated by the body attitude (yaw irrelevant).
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        z_g = (-sp * feet[:, 0] + sr * cp * feet[:, 1]
               + cr * cp * feet[:, 2])
        return z_g <= (float(np.min(z_g)) + self.stance_z_tol_m)

    def update(self, q_rad: np.ndarray, gyro_rad_s: np.ndarray,
               roll_rad: float, pitch_rad: float) -> np.ndarray:
        """One tick. Returns filtered body-frame (vx, vy) in m/s."""
        feet = fk_all_feet(np.asarray(q_rad, dtype=float))
        stance = self._stance_mask(feet, float(roll_rad),
                                   float(pitch_rad))
        if self._prev_feet is None:
            self._prev_feet = feet
            self._prev_stance = stance
            return self._v_filt.copy()

        # A foot only informs odometry if it was planted across the
        # whole differencing interval.
        both = stance & self._prev_stance
        if np.count_nonzero(both) >= 2:
            dr = (feet - self._prev_feet) / self.dt          # (6,3)
            w = np.asarray(gyro_rad_s, dtype=float).reshape(3)
            wxr = np.cross(np.broadcast_to(w, (N_LEGS, 3)), feet)
            v_per_foot = -(dr + wxr)[:, :2]                  # (6,2)
            v = v_per_foot[both].mean(axis=0)
            self._v_filt += self.alpha * (v - self._v_filt)
        # <2 persistent stance feet (brief flight/degenerate read):
        # hold the previous filtered estimate rather than injecting 0.

        self._prev_feet = feet
        self._prev_stance = stance
        return self._v_filt.copy()


def integrate_track(q_deg_rows: np.ndarray, t_s: np.ndarray, *,
                    alpha: float = 1.0) -> dict:
    """Offline leg-odometry over a logged trace -> travel summary.

    ``q_deg_rows`` (T, 18) joint DEGREES, ``t_s`` (T,) seconds
    (irregular ok — dt is taken per sample pair). No gyro/tilt columns
    in the hardware telemetry CSVs, so omega := 0 and level attitude
    are assumed; on a straight-line tape run yaw rate is near zero and
    the induced error is second-order. Returns integrated body-frame
    displacement, path length, and per-sample speeds — built to
    validate the estimator against the 08-10 tape-measure session
    (distance estimate vs the operator's tape reading).

    ``alpha`` defaults to 1.0 (no low-pass) here, unlike the live
    25 Hz estimator: at the telemetry logger's ~3 Hz each sample pair
    already averages a third of a second, and measured on the 08-10
    traces the 25 Hz filter constant mixes canceling stance/swing
    velocities across gait phases (estimate collapsed ~3x).
    """
    q = np.asarray(q_deg_rows, dtype=float) * math.pi / 180.0
    t = np.asarray(t_s, dtype=float)
    assert q.ndim == 2 and q.shape[1] == 18 and len(t) == len(q)
    est = LegOdometryVelocity(dt=1.0, alpha=alpha)
    disp = np.zeros(2)
    path = 0.0
    speeds = []
    zeros3 = np.zeros(3)
    for k in range(len(q)):
        if k > 0:
            dt = max(float(t[k] - t[k - 1]), 1e-3)
            est.dt = dt
        v = est.update(q[k], zeros3, 0.0, 0.0)
        if k > 0:
            step = v * (t[k] - t[k - 1])
            disp += step
            path += float(np.hypot(*step))
            speeds.append(float(np.hypot(*v)))
    return {
        "disp_xy_m": disp.tolist(),
        "disp_along_m": float(np.hypot(*disp)),
        "path_m": path,
        "speed_mean_mps": float(np.mean(speeds)) if speeds else 0.0,
        "n_samples": int(len(q)),
        "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
    }
