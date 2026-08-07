"""Synchronized RobotState acquisition for Phase-1 balance."""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .attitude import ComplementaryAttitude, G0
from .config import cfg_get

_LINUX = Path(__file__).resolve().parents[1] / "linux_control"
if str(_LINUX) not in sys.path:
    sys.path.insert(0, str(_LINUX))
_URT2 = _LINUX / "urt2_setup"
if str(_URT2) not in sys.path:
    sys.path.insert(0, str(_URT2))

N_JOINTS = 18
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


@dataclass
class RobotState:
    timestamp: float
    joint_position: np.ndarray       # (18,) rad
    joint_velocity: np.ndarray       # (18,) rad/s
    imu_roll: float
    imu_pitch: float
    imu_yaw: float
    imu_gyro: np.ndarray             # (3,) rad/s
    imu_accel: np.ndarray            # (3,) m/s²
    commanded_position: np.ndarray   # (18,) rad
    servo_load: np.ndarray | None = None
    servo_current: np.ndarray | None = None
    servo_temperature: np.ndarray | None = None
    bus_ok: bool = True
    imu_ok: bool = True
    dt: float = 0.0
    timing: dict = field(default_factory=dict)


@dataclass
class AcquisitionTiming:
    t_pos: float = 0.0
    t_imu: float = 0.0
    t_fb: float = 0.0
    t_total: float = 0.0
    did_full_feedback: bool = False


class JointVelocityFilter:
    def __init__(self, *, alpha: float = 0.3, max_jump_rad: float = 0.5,
                 n: int = N_JOINTS):
        self.alpha = float(alpha)
        self.max_jump = float(max_jump_rad)
        self.n = int(n)
        self._q_prev: np.ndarray | None = None
        self._qd = np.zeros(self.n, dtype=float)
        self._t_prev: float | None = None

    def reset(self) -> None:
        self._q_prev = None
        self._qd[:] = 0.0
        self._t_prev = None

    def update(self, q: np.ndarray, t: float) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.n)
        if self._q_prev is None or self._t_prev is None:
            self._q_prev = q.copy()
            self._t_prev = float(t)
            self._qd[:] = 0.0
            return self._qd.copy()
        dt = float(t) - self._t_prev
        if dt < 1e-4 or dt > 0.25:
            # Missed / bad sample — hold velocity, refresh pose stamp carefully.
            self._q_prev = q.copy()
            self._t_prev = float(t)
            return self._qd.copy()
        dq = q - self._q_prev
        # Reject absurd jumps (bus glitch); hold qd.
        if np.any(np.abs(dq) > self.max_jump):
            self._q_prev = q.copy()
            self._t_prev = float(t)
            return self._qd.copy()
        raw = dq / dt
        a = self.alpha
        self._qd = a * raw + (1.0 - a) * self._qd
        self._q_prev = q.copy()
        self._t_prev = float(t)
        return self._qd.copy()


class RobotStateEstimator:
    """One coherent snapshot per ``update()`` call."""

    def __init__(self, bus: Any, cfg: dict | None = None):
        self.bus = bus
        cfg = cfg or {}
        self._alpha = float(cfg_get(cfg, "velocity_filter", "alpha", default=0.3))
        self._max_jump = float(
            cfg_get(cfg, "velocity_filter", "max_jump_rad", default=0.5))
        fb_hz = float(cfg_get(cfg, "sensing", "full_feedback_hz", default=10))
        self._fb_period = (1.0 / fb_hz) if fb_hz > 0 else 1e9
        self._qd_filter = JointVelocityFilter(
            alpha=self._alpha, max_jump_rad=self._max_jump)
        self._att = ComplementaryAttitude(alpha=0.98)
        self._cmd = np.zeros(N_JOINTS, dtype=float)
        self._last_fb_t = -1e9
        self._load = np.zeros(N_JOINTS, dtype=float)
        self._current = np.zeros(N_JOINTS, dtype=float)
        self._temp = np.zeros(N_JOINTS, dtype=float)
        self._have_fb = False
        self._imu_stale_s = float(
            cfg_get(cfg, "safety", "imu_stale_ms", default=100)) / 1000.0
        self.last_timing = AcquisitionTiming()
        self._t_prev_state: float | None = None

    def set_commanded(self, q_rad: np.ndarray | list[float]) -> None:
        self._cmd = np.asarray(q_rad, dtype=float).reshape(N_JOINTS).copy()

    def reset_episode_filters(self) -> None:
        """Clear qd history + attitude transients; keep physical level."""
        self._qd_filter.reset()
        self._att.reset_transients()
        self._t_prev_state = None

    def update(self, *, want_full_feedback: bool | None = None) -> RobotState:
        t0 = time.monotonic()
        timing = AcquisitionTiming()

        # --- positions ---
        t_a = time.monotonic()
        pos_deg = self.bus.read_all_positions()
        timing.t_pos = time.monotonic() - t_a
        bus_ok = isinstance(pos_deg, dict) and len(pos_deg) >= N_JOINTS
        q = np.zeros(N_JOINTS, dtype=float)
        if bus_ok:
            for j in range(N_JOINTS):
                if j not in pos_deg:
                    bus_ok = False
                    break
                q[j] = float(pos_deg[j]) * DEG2RAD
        elif isinstance(pos_deg, dict):
            for j, deg in pos_deg.items():
                if 0 <= int(j) < N_JOINTS:
                    q[int(j)] = float(deg) * DEG2RAD

        t_now = time.monotonic()
        qd = self._qd_filter.update(q, t_now)

        # --- IMU ---
        t_b = time.monotonic()
        imu = None
        try:
            imu = self.bus.read_imu(apply_calib=True)
        except Exception:
            imu = None
        timing.t_imu = time.monotonic() - t_b
        imu_ok = isinstance(imu, dict) and "ax_g" in imu
        if imu_ok:
            accel_g = (imu["ax_g"], imu["ay_g"], imu["az_g"])
            gyro_rps = (imu["gx_dps"] * DEG2RAD,
                        imu["gy_dps"] * DEG2RAD,
                        imu["gz_dps"] * DEG2RAD)
            accel_mps2 = np.array(
                [accel_g[0] * G0, accel_g[1] * G0, accel_g[2] * G0],
                dtype=float)
            gyro = np.array(gyro_rps, dtype=float)
            dt_att = 0.0 if self._t_prev_state is None \
                else max(0.0, t_now - self._t_prev_state)
            att = self._att.update(accel_g, gyro_rps, dt_att)
            roll, pitch, yaw = att.roll, att.pitch, att.yaw
        else:
            accel_mps2 = np.zeros(3, dtype=float)
            gyro = np.zeros(3, dtype=float)
            roll = pitch = yaw = 0.0

        # --- opportunistic full feedback ---
        did_fb = False
        if want_full_feedback is None:
            want_full_feedback = (t_now - self._last_fb_t) >= self._fb_period
        if want_full_feedback:
            t_c = time.monotonic()
            try:
                fb = self.bus.read_all_feedback()
            except Exception:
                fb = {}
            timing.t_fb = time.monotonic() - t_c
            if isinstance(fb, dict) and fb:
                did_fb = True
                self._last_fb_t = t_now
                self._have_fb = True
                for j, rec in fb.items():
                    jj = int(j)
                    if 0 <= jj < N_JOINTS:
                        self._load[jj] = float(rec.get("load_pct") or 0.0)
                        self._current[jj] = float(rec.get("current_a") or 0.0)
                        self._temp[jj] = float(rec.get("temp_c") or 0.0)
        timing.did_full_feedback = did_fb

        dt = 0.0 if self._t_prev_state is None else (t_now - self._t_prev_state)
        self._t_prev_state = t_now
        timing.t_total = time.monotonic() - t0
        self.last_timing = timing

        return RobotState(
            timestamp=t_now,
            joint_position=q,
            joint_velocity=qd,
            imu_roll=float(roll),
            imu_pitch=float(pitch),
            imu_yaw=float(yaw),
            imu_gyro=gyro,
            imu_accel=accel_mps2,
            commanded_position=self._cmd.copy(),
            servo_load=self._load.copy() if self._have_fb else None,
            servo_current=self._current.copy() if self._have_fb else None,
            servo_temperature=self._temp.copy() if self._have_fb else None,
            bus_ok=bus_ok,
            imu_ok=imu_ok,
            dt=float(dt),
            timing={
                "t_pos": timing.t_pos,
                "t_imu": timing.t_imu,
                "t_fb": timing.t_fb,
                "t_total": timing.t_total,
                "full_feedback": did_fb,
            },
        )
