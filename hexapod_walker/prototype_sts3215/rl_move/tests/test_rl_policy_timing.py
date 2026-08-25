from types import SimpleNamespace
from pathlib import Path
import sys

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "linux_control") not in sys.path:
    sys.path.insert(0, str(_ROOT / "linux_control"))

import rl_policy  # noqa: E402
from rl_move.robot_state import RobotState  # noqa: E402


def _policy(meta):
    return SimpleNamespace(meta=dict(meta))


def _state(*, bus_ok=True):
    z = np.zeros(rl_policy.N_JOINTS, dtype=float)
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
        bus_ok=bus_ok,
        imu_ok=True,
        timing={"source": "fake_state"},
    )


class _FakeBus:
    def __init__(self):
        self.writes = 0

    def write_all(self, _deg, *, speed, acc):
        self.writes += 1


class _FakeStepBus(_FakeBus):
    has_stream = True

    def __init__(self, snaps=None):
        super().__init__()
        self.steps = 0
        self.snaps = list(snaps or [])

    def step_all(self, _deg, *, speed, acc):
        self.steps += 1
        if self.snaps:
            return self.snaps.pop(0)
        return {
            "seq": self.steps,
            "pos_age_ms": 1,
            "imu_age_ms": 1,
            "pos_deg": {j: 0.0 for j in range(rl_policy.N_JOINTS)},
            "imu": {
                "ax_g": 0.0, "ay_g": 0.0, "az_g": 1.0,
                "gx_dps": 0.0, "gy_dps": 0.0, "gz_dps": 0.0,
            },
        }


class _PreflightBus:
    def __init__(self, q_deg):
        self.q_deg = list(q_deg)

    def read_all_positions(self):
        return {j: float(v) for j, v in enumerate(self.q_deg)}

    def read_imu(self, *, apply_calib=True):
        return {
            "ax_g": 0.0, "ay_g": 0.0, "az_g": 1.0,
            "gx_dps": 0.0, "gy_dps": 0.0, "gz_dps": 0.0,
        }


class _FakeEstimator:
    def __init__(self, states):
        self._states = list(states)
        self.commanded = []
        self.snapshots = []

    def set_commanded(self, q):
        self.commanded.append(np.asarray(q, dtype=float).copy())

    def update(self):
        return self._states.pop(0) if self._states else _state()

    def update_from_snapshot(self, snap):
        self.snapshots.append(dict(snap))
        return self.update()


def test_policy_bus_profile_prefers_metadata():
    cfg = {"bus": {"write_speed": 400, "write_acc": 20}}

    assert rl_policy._policy_bus_profile(  # noqa: SLF001
        _policy({"bus_write_speed": 1500, "bus_write_acc": 80}), cfg
    ) == (1500, 80)


def test_policy_bus_profile_falls_back_to_config():
    cfg = {"bus": {"write_speed": 500, "write_acc": 30}}

    assert rl_policy._policy_bus_profile(_policy({}), cfg) == (500, 30)  # noqa: SLF001


def test_inner_stream_plan_noops_at_100hz_default():
    cfg = {"control": {"inner_hz": 100}}

    steps, actual_hz, inner_dt = rl_policy._inner_stream_plan(  # noqa: SLF001
        _policy({"control_hz": 100}), cfg, policy_hz=100
    )

    assert steps == 1
    assert actual_hz == pytest.approx(100.0)
    assert inner_dt == pytest.approx(0.01)


def test_legacy_policy_streams_25hz_decisions_at_100hz_inner_rate():
    cfg = {"control": {"inner_hz": 100}}

    steps, actual_hz, inner_dt = rl_policy._inner_stream_plan(  # noqa: SLF001
        _policy({}), cfg, policy_hz=25
    )

    assert steps == 4
    assert actual_hz == pytest.approx(100.0)
    assert inner_dt == pytest.approx(0.01)


def test_inner_stream_plan_can_be_disabled():
    cfg = {"control": {"inner_hz": 25}}

    steps, actual_hz, inner_dt = rl_policy._inner_stream_plan(  # noqa: SLF001
        _policy({"inner_hz": 25}), cfg, policy_hz=25
    )

    assert steps == 1
    assert actual_hz == pytest.approx(25.0)
    assert inner_dt == pytest.approx(0.04)


def test_policy_timing_adapts_legacy_and_explicit_rates():
    legacy = rl_policy._policy_timing(_policy({}))  # noqa: SLF001
    explicit = rl_policy._policy_timing(  # noqa: SLF001
        _policy({"policy_hz": 100})
    )

    assert legacy.policy_hz == pytest.approx(25.0)
    assert legacy.policy_dt == pytest.approx(0.04)
    assert legacy.adapted is True
    assert explicit.policy_hz == pytest.approx(100.0)
    assert explicit.adapted is False
    assert rl_policy._check_policy_control_hz(  # noqa: SLF001
        _policy({"policy_hz": 25}), "walk"
    ) is None


def test_policy_safety_slew_preserves_deg_per_second_for_legacy():
    cfg = {"control": {"hz": 100}, "safety": {"max_delta_q_deg": 0.375}}

    dq, explicit = rl_policy._policy_safety_max_delta_q_deg(  # noqa: SLF001
        _policy({}), cfg, policy_hz=25
    )

    assert explicit is False
    assert dq == pytest.approx(1.5)


def test_policy_safety_slew_prefers_trained_metadata():
    cfg = {"control": {"hz": 100}, "safety": {"max_delta_q_deg": 0.375}}

    dq, explicit = rl_policy._policy_safety_max_delta_q_deg(  # noqa: SLF001
        _policy({"max_delta_q_deg": 5.0}), cfg, policy_hz=25
    )

    assert explicit is True
    assert dq == pytest.approx(5.0)


def test_walk_start_options_use_sim_start_only():
    options, err = rl_policy._expected_start_options_deg("walk")  # noqa: SLF001

    assert err == ""
    assert options is not None
    names = [name for name, _pose, _tol in options]
    assert "sim_walk_start" in names
    assert names == ["sim_walk_start"]
    pose = dict((name, pose) for name, pose, _tol in options)["sim_walk_start"]
    assert pose.tolist() == pytest.approx([0.0, 20.0, 80.0] * 6)


def test_walk_preflight_reports_sim_walk_start():
    ok, reason, details = rl_policy.preflight(
        _PreflightBus([0.0, 20.0, 80.0] * 6), "walk")

    assert ok, reason
    assert details["start_pose"] == "sim_walk_start"
    assert details["max_pose_delta_deg"] == pytest.approx(0.0)


def test_neutral_drive_command_stays_in_hold():
    assert not rl_policy._drive_command_is_moving(  # noqa: SLF001
        0.0, 0.0, 0.0
    )


def test_drive_command_engages_walk_on_translation_or_yaw():
    assert rl_policy._drive_command_is_moving(  # noqa: SLF001
        0.001, 0.0, 0.0
    )
    assert rl_policy._drive_command_is_moving(  # noqa: SLF001
        0.0, 0.0, 0.01, walk_obs=93
    )
    assert not rl_policy._drive_command_is_moving(  # noqa: SLF001
        0.0, 0.0, 0.01, walk_obs=72
    )


def test_drive_translation_clamps_to_hardware_trained_band():
    vx, vy = rl_policy._drive_clamp_translation(0.002, 0.0)  # noqa: SLF001

    assert vx == pytest.approx(rl_policy.WALK_SPEED_MIN)
    assert vy == pytest.approx(0.0)

    vx, vy = rl_policy._drive_clamp_translation(0.2, 0.0)  # noqa: SLF001

    assert vx == pytest.approx(rl_policy.WALK_SPEED_MAX)
    assert vy == pytest.approx(0.0)


def test_joint_hold_fallback_does_not_use_learned_policy():
    assert not rl_policy._drive_uses_learned_policy(  # noqa: SLF001
        "hold", None
    )
    assert rl_policy._drive_uses_learned_policy(  # noqa: SLF001
        "walk", None
    )
    assert rl_policy._drive_uses_learned_policy(  # noqa: SLF001
        "hold", _policy({"obs_dim": 68})
    )


def test_stream_target_tolerates_short_feedback_dropouts():
    bus = _FakeBus()
    good0 = _state()
    good1 = _state()
    est = _FakeEstimator([
        _state(bus_ok=False),
        _state(bus_ok=False),
        good1,
        _state(),
    ])

    out = rl_policy._stream_target(  # noqa: SLF001
        bus,
        est,
        np.zeros(rl_policy.N_JOINTS),
        np.ones(rl_policy.N_JOINTS) * 0.1,
        t_next=0.0,
        inner_steps=4,
        inner_dt=0.0,
        write_speed=100,
        write_acc=20,
        abort_check=lambda: False,
        last_good_state=good0,
        stale_ticks=0,
        max_stale_ticks=3,
    )

    state, _t_next, _overruns, err, stale_ticks, stale_samples = out
    assert err == ""
    assert state.bus_ok is True
    assert not rl_policy._stream_state_is_stale(state)  # noqa: SLF001
    assert stale_ticks == 0
    assert stale_samples == 2
    assert bus.writes == 4


def test_stream_target_prefers_combined_step_all_snapshot():
    bus = _FakeStepBus()
    est = _FakeEstimator([_state() for _ in range(4)])

    out = rl_policy._stream_target(  # noqa: SLF001
        bus,
        est,
        np.zeros(rl_policy.N_JOINTS),
        np.ones(rl_policy.N_JOINTS) * 0.1,
        t_next=0.0,
        inner_steps=4,
        inner_dt=0.0,
        write_speed=100,
        write_acc=20,
        abort_check=lambda: False,
        last_good_state=_state(),
        stale_ticks=0,
        max_stale_ticks=3,
    )

    state, _t_next, _overruns, err, stale_ticks, stale_samples = out
    assert err == ""
    assert state.bus_ok is True
    assert stale_ticks == 0
    assert stale_samples == 0
    assert bus.steps == 4
    assert bus.writes == 0
    assert len(est.snapshots) == 4


def test_stream_target_treats_stream_step_all_miss_as_stale_sample():
    bus = _FakeStepBus(snaps=[None, None, None, None])
    good0 = _state()
    est = _FakeEstimator([])

    out = rl_policy._stream_target(  # noqa: SLF001
        bus,
        est,
        np.zeros(rl_policy.N_JOINTS),
        np.ones(rl_policy.N_JOINTS) * 0.1,
        t_next=0.0,
        inner_steps=4,
        inner_dt=0.0,
        write_speed=100,
        write_acc=20,
        abort_check=lambda: False,
        last_good_state=good0,
        stale_ticks=0,
        max_stale_ticks=3,
    )

    state, _t_next, _overruns, err, stale_ticks, stale_samples = out
    assert err == "feedback stale during stream"
    assert rl_policy._stream_state_is_stale(state)  # noqa: SLF001
    diag = state.timing["stale_diag"]
    assert diag["transport"] == "step_all"
    assert diag["step_all_none"] is True
    assert diag["fallback_suppressed"] == "stream_firmware"
    assert diag["stale_ticks_after"] == 4
    assert diag["max_stale_ticks"] == 3
    assert diag["last_good_source"] == "fake_state"
    assert stale_ticks == 4
    assert stale_samples == 4
    assert bus.steps == 4
    assert bus.writes == 0


def test_stream_target_stops_after_stale_feedback_limit():
    bus = _FakeBus()
    good0 = _state()
    est = _FakeEstimator([_state(bus_ok=False) for _ in range(4)])

    out = rl_policy._stream_target(  # noqa: SLF001
        bus,
        est,
        np.zeros(rl_policy.N_JOINTS),
        np.ones(rl_policy.N_JOINTS) * 0.1,
        t_next=0.0,
        inner_steps=4,
        inner_dt=0.0,
        write_speed=100,
        write_acc=20,
        abort_check=lambda: False,
        last_good_state=good0,
        stale_ticks=0,
        max_stale_ticks=3,
    )

    _state_out, _t_next, _overruns, err, stale_ticks, stale_samples = out
    assert err == "feedback stale during stream"
    assert stale_ticks == 4
    assert stale_samples == 4
    assert bus.writes == 4
