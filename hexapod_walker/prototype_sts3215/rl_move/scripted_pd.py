"""Scripted roll/pitch PD before PPO (Milestone 2)."""
from __future__ import annotations

import numpy as np

from .config import cfg_get
from .robot_state import RobotState


class ScriptedBalancePD:
    """Maps tilt error → normalized body roll/pitch actions (xy/h = 0)."""

    def __init__(self, cfg: dict):
        self.kp_roll = float(cfg_get(cfg, "pd", "kp_roll", default=1.5) or 1.5)
        self.kd_roll = float(cfg_get(cfg, "pd", "kd_roll", default=0.15) or 0.15)
        self.kp_pitch = float(cfg_get(cfg, "pd", "kp_pitch", default=1.5) or 1.5)
        self.kd_pitch = float(cfg_get(cfg, "pd", "kd_pitch", default=0.15) or 0.15)
        self.target_roll = 0.0
        self.target_pitch = 0.0

    def set_target(self, roll: float = 0.0, pitch: float = 0.0) -> None:
        self.target_roll = float(roll)
        self.target_pitch = float(pitch)

    def act(self, state: RobotState) -> np.ndarray:
        # Gyro[0]≈roll rate, gyro[1]≈pitch rate (sensor frame; tune if swapped).
        e_r = self.target_roll - state.imu_roll
        e_p = self.target_pitch - state.imu_pitch
        roll_cmd = self.kp_roll * e_r - self.kd_roll * float(state.imu_gyro[0])
        pitch_cmd = self.kp_pitch * e_p - self.kd_pitch * float(state.imu_gyro[1])
        a = np.array([roll_cmd, pitch_cmd, 0.0, 0.0, 0.0], dtype=float)
        return np.clip(a, -1.0, 1.0)
