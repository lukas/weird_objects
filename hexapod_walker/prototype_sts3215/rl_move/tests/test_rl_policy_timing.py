from types import SimpleNamespace
from pathlib import Path
import sys

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "linux_control") not in sys.path:
    sys.path.insert(0, str(_ROOT / "linux_control"))

import rl_policy  # noqa: E402


def _policy(meta):
    return SimpleNamespace(meta=dict(meta))


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
