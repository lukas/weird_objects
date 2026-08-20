"""bus.servo_vel_max_counts_s — opt-in ServoProfile speed-ceiling override.

Operator order 08-19 (fast-walker headroom): raising bus.write_speed
alone is a no-op because every fitted actuator set carries the ~350
counts/s sys-ID test speed as vel_max_deg_s and both backends clamp the
commanded profile speed to it (ServoProfile.command on CPU,
TickParams.vel_max via per_joint("vel_max_deg_s") on MJX). The cfg key
lifts the ceiling at the single resolution point every training/eval
path uses (SimServoParams.from_cfg). Default OFF must stay bit-exact.
"""
import math

import numpy as np
import pytest

from rl_move.sim.servo_model import (
    ACC_UNIT_DEG_S2, COUNTS_PER_DEG, DEG2RAD, N_JOINTS, ServoProfile,
    SimServoParams)


def _vels(p: SimServoParams) -> np.ndarray:
    return p.per_joint("vel_max_deg_s")


def test_default_off_is_bit_exact():
    base = _vels(SimServoParams.load())
    for cfg in (None, {}, {"bus": {}}, {"bus": {"servo_vel_max_counts_s": ""}},
                {"bus": {"write_speed": 1500}}):
        got = _vels(SimServoParams.from_cfg(cfg))
        assert np.array_equal(got, base), f"cfg={cfg!r} changed vel_max"


def test_numeric_override_sets_all_axes():
    p = SimServoParams.from_cfg({"bus": {"servo_vel_max_counts_s": 1500}})
    want = 1500.0 / COUNTS_PER_DEG
    assert np.allclose(_vels(p), want)
    assert "+vel_max=1500cps" in p.source


def test_numeric_override_accepts_string_number():
    p = SimServoParams.from_cfg({"bus": {"servo_vel_max_counts_s": "900"}})
    assert np.allclose(_vels(p), 900.0 / COUNTS_PER_DEG)


def test_write_speed_sentinel_mirrors_bus_write_speed():
    p = SimServoParams.from_cfg(
        {"bus": {"servo_vel_max_counts_s": "write_speed",
                 "write_speed": 1500}})
    assert np.allclose(_vels(p), 1500.0 / COUNTS_PER_DEG)


def test_write_speed_sentinel_default_400():
    # No explicit bus.write_speed -> the config default (400 counts/s).
    p = SimServoParams.from_cfg(
        {"bus": {"servo_vel_max_counts_s": "write_speed"}})
    assert np.allclose(_vels(p), 400.0 / COUNTS_PER_DEG)


@pytest.mark.parametrize("bad", ["fast", -5, 0, "0"])
def test_bad_values_fail_closed(bad):
    with pytest.raises(ValueError):
        SimServoParams.from_cfg({"bus": {"servo_vel_max_counts_s": bad}})


def test_other_axis_params_untouched():
    base = SimServoParams.load()
    p = SimServoParams.from_cfg({"bus": {"servo_vel_max_counts_s": 1500}})
    for attr in ("kp", "kv", "frictionloss", "latency_ms",
                 "deadband_deg", "torque_limit_nm"):
        assert np.array_equal(p.per_joint(attr), base.per_joint(attr)), attr


def _settle_time(params: SimServoParams, speed_counts_s: float,
                 acc_units: float, step_deg: float = 40.0) -> float:
    """Ticks a ServoProfile through a step command; returns arrival time."""
    q0 = np.zeros(N_JOINTS)
    prof = ServoProfile(params, q0)
    goal = np.full(N_JOINTS, step_deg * DEG2RAD)
    prof.command(goal, speed_deg_s=speed_counts_s / COUNTS_PER_DEG,
                 acc_units=acc_units)
    dt = 1.0 / 250.0
    # The profile parks inside the deadband of the goal — arrival =
    # within deadband + a hair, not exact equality.
    tol = params.per_joint("deadband_deg").max() * DEG2RAD + 1e-6
    for i in range(int(8.0 / dt)):
        tgt = prof.tick(dt)
        if np.all(np.abs(tgt - goal) <= tol):
            return (i + 1) * dt
    return math.inf


def test_profile_actually_goes_faster_with_override():
    """The whole point: write_speed=1500 is clamped to ~350 counts/s
    without the override, and genuinely ~4x faster with it."""
    stock = SimServoParams.from_cfg(None)
    fast = SimServoParams.from_cfg(
        {"bus": {"servo_vel_max_counts_s": "write_speed",
                 "write_speed": 1500}})
    t_stock = _settle_time(stock, 1500, 80)
    t_fast = _settle_time(fast, 1500, 80)
    assert t_fast < t_stock / 2.5, (t_stock, t_fast)
    # Clamped case ~= commanding the old ceiling explicitly.
    t_ceiling = _settle_time(stock, 350, 80)
    assert abs(t_stock - t_ceiling) < 0.02, (t_stock, t_ceiling)


def test_mjx_reads_the_same_field():
    """MJX TickParams derive vel_max from per_joint('vel_max_deg_s')
    (mjx_backend.default_tick_params / mjx_host.tp_rows) — assert the
    override lands in exactly that vector, rad/s."""
    p = SimServoParams.from_cfg({"bus": {"servo_vel_max_counts_s": 1500}})
    vel = p.per_joint("vel_max_deg_s") * DEG2RAD
    assert np.allclose(vel, 1500.0 / COUNTS_PER_DEG * DEG2RAD)
