from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from rl_move.env import HexapodBalanceEnv  # noqa: E402
from rl_move.robot_state import N_JOINTS, RobotState  # noqa: E402


def _state() -> RobotState:
    z = np.zeros(N_JOINTS, dtype=float)
    return RobotState(
        timestamp=0.0,
        joint_position=z.copy(),
        joint_velocity=z.copy(),
        imu_roll=0.0,
        imu_pitch=0.0,
        imu_yaw=0.0,
        imu_gyro=np.zeros(3, dtype=float),
        imu_accel=np.zeros(3, dtype=float),
        commanded_position=z.copy(),
        bus_ok=True,
        imu_ok=True,
        timing={"source": "fake"},
    )


class _FakeEstimator:
    def __init__(self):
        self.commanded = []
        self.snapshots = []
        self.updates = 0

    def set_commanded(self, q):
        self.commanded.append(np.asarray(q, dtype=float).copy())

    def update_from_snapshot(self, snap):
        self.snapshots.append(dict(snap))
        return _state()

    def update(self):
        self.updates += 1
        return _state()


class _StepBus:
    def __init__(self, snap):
        self.snap = snap
        self.steps = 0
        self.writes = 0

    def step_all(self, _deg, *, speed, acc):
        self.steps += 1
        return self.snap

    def write_all(self, _deg, *, speed, acc):
        self.writes += 1


def _env(bus, estimator):
    env = HexapodBalanceEnv.__new__(HexapodBalanceEnv)
    env.bus = bus
    env.estimator = estimator
    env.enable_motion = True
    env.write_speed = 400
    env.write_acc = 20
    return env


def test_command_and_update_uses_step_all_snapshot():
    snap = {
        "seq": 7,
        "pos_age_ms": 1,
        "imu_age_ms": 1,
        "pos_deg": {j: 0.0 for j in range(N_JOINTS)},
        "imu": {
            "ax_g": 0.0, "ay_g": 0.0, "az_g": 1.0,
            "gx_dps": 0.0, "gy_dps": 0.0, "gz_dps": 0.0,
        },
    }
    bus = _StepBus(snap)
    est = _FakeEstimator()

    state = HexapodBalanceEnv._command_and_update(
        _env(bus, est), np.zeros(N_JOINTS, dtype=float))

    assert state.bus_ok
    assert bus.steps == 1
    assert bus.writes == 0
    assert est.updates == 0
    assert est.snapshots == [snap]


def test_command_and_update_falls_back_when_step_all_has_no_snapshot():
    bus = _StepBus(None)
    est = _FakeEstimator()

    state = HexapodBalanceEnv._command_and_update(
        _env(bus, est), np.zeros(N_JOINTS, dtype=float))

    assert state.bus_ok
    assert bus.steps == 1
    assert bus.writes == 1
    assert est.updates == 1
    assert est.snapshots == []
