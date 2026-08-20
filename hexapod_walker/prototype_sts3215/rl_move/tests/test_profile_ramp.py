"""bus.profile_ramp_steps — trainer-driven servo-profile ramp-in.

08-20, fast anti-skate option (b), q_20260820T0830Z: both V5 canaries
died at the pre-PPO B0 bridge cert because the bcgait1_hard1 transplant
cannot survive the raised write profile ZERO-SHOT (dose-graded:
1500/80 falls 6/8, 750/40 falls 2/8). The ramp lets a run START at the
fitted regime (350 counts/s effective cruise, acc 20, 1.5 deg/tick
slew — where the transplant is stable) and anneal linearly to the cfg
target dose (bus.write_speed / bus.write_acc / safety.max_delta_q_deg)
over ``bus.profile_ramp_steps`` global env steps, driven by
train_ppo_mjx broadcasts of ``apply_profile_ramp_frac``.

Contract under test:
  - default (key absent/0) is bit-exact OFF: no ramp state, apply raises;
  - ARMED env CONSTRUCTS at the TARGET dose (eval_checkpoint / play /
    periodic C evals judge full dose without any broadcast);
  - frac 0 -> start trio, 0.5 -> midpoint, >=1 -> target, clamped;
  - fail-closed: target write_speed above the resolved actuator
    velocity ceiling raises at construction (silent-clamp trap);
  - the slew clamp survives an MJX pool-restore of a stale SafetyLayer
    (safety is in SNAP_ATTRS; _step_begin re-asserts it every tick).
"""
from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.servo_model import COUNTS_PER_DEG, SimServoParams  # noqa: E402

RAMP_KEYS = {
    ("bus", "write_speed"): 1500,
    ("bus", "write_acc"): 80,
    ("bus", "servo_vel_max_counts_s"): "write_speed",
    ("safety", "max_delta_q_deg"): 5.0,
    ("bus", "profile_ramp_steps"): 2_000_000,
}


def _cfg(extra=None):
    cfg = load_config()
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    return cfg


def _env(extra=None, params=None, seed=0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = _cfg(extra)
    if params is None:
        params = SimServoParams.from_cfg(cfg)
    return SimHexapodJointWalkEnv(
        params=params, randomize=False, dr_scale=0.0,
        episode_seconds=2.0, seed=seed, cfg=cfg)


def _counts(deg_s: float) -> float:
    return deg_s * COUNTS_PER_DEG


def test_default_off_bit_exact_and_apply_raises():
    env = _env()
    assert env._profile_ramp is None
    assert env._profile_ramp_dq_rad is None
    assert _counts(env.write_speed_deg_s) == pytest.approx(400.0)
    assert env.write_acc_units == pytest.approx(20.0)
    with pytest.raises(RuntimeError, match="not armed"):
        env.apply_profile_ramp_frac(0.5)
    env.close()


def test_armed_env_constructs_at_target_dose():
    env = _env(RAMP_KEYS)
    assert env._profile_ramp is not None
    # No broadcast yet -> full target dose (the eval contract).
    assert _counts(env.write_speed_deg_s) == pytest.approx(1500.0)
    assert env.write_acc_units == pytest.approx(80.0)
    assert env.safety.max_dq == pytest.approx(math.radians(5.0))
    env.close()


def test_frac_endpoints_midpoint_and_clamp():
    env = _env(RAMP_KEYS)
    v0 = env.apply_profile_ramp_frac(0.0)
    assert v0["write_speed_counts_s"] == pytest.approx(350.0)
    assert v0["write_acc"] == pytest.approx(20.0)
    assert v0["max_delta_q_deg"] == pytest.approx(1.5)
    assert _counts(env.write_speed_deg_s) == pytest.approx(350.0)
    assert env.safety.max_dq == pytest.approx(math.radians(1.5))

    vm = env.apply_profile_ramp_frac(0.5)
    assert vm["write_speed_counts_s"] == pytest.approx((350 + 1500) / 2)
    assert vm["write_acc"] == pytest.approx(50.0)
    assert vm["max_delta_q_deg"] == pytest.approx(3.25)

    v1 = env.apply_profile_ramp_frac(1.0)
    assert v1["write_speed_counts_s"] == pytest.approx(1500.0)
    assert v1["max_delta_q_deg"] == pytest.approx(5.0)

    # Clamped both ways.
    assert env.apply_profile_ramp_frac(7.0)["frac"] == 1.0
    assert env.apply_profile_ramp_frac(-3.0)["frac"] == 0.0
    env.close()


def test_custom_start_keys():
    env = _env({**RAMP_KEYS,
                ("bus", "profile_ramp_start_write_speed"): 750,
                ("bus", "profile_ramp_start_write_acc"): 40,
                ("bus", "profile_ramp_start_max_delta_q_deg"): 3.0})
    v0 = env.apply_profile_ramp_frac(0.0)
    assert v0["write_speed_counts_s"] == pytest.approx(750.0)
    assert v0["write_acc"] == pytest.approx(40.0)
    assert v0["max_delta_q_deg"] == pytest.approx(3.0)
    env.close()


def test_target_above_ceiling_fails_closed():
    # params WITHOUT the vel-ceiling override (fitted ~350 counts/s)
    # but a ramp targeting 1500 -> the ramp would be silently clamped.
    with pytest.raises(ValueError, match="silently clamped"):
        _env(RAMP_KEYS, params=SimServoParams.from_cfg(None))


def test_bad_start_values_fail_closed():
    with pytest.raises(ValueError, match="profile_ramp_start"):
        _env({**RAMP_KEYS,
              ("bus", "profile_ramp_start_write_acc"): 0})


def test_slew_clamp_survives_pool_restore():
    """MJX pool-restores revive a deep-copied SafetyLayer minted under
    an older ramp value; _step_begin must re-assert the live clamp."""
    env = _env(RAMP_KEYS)
    stale_safety = copy.deepcopy(env.safety)   # minted at target (5 deg)
    env.apply_profile_ramp_frac(0.0)           # ramp now at 1.5 deg
    env.reset()
    env.safety = copy.deepcopy(stale_safety)   # simulate restore_env
    env.safety._estop = False
    assert env.safety.max_dq == pytest.approx(math.radians(5.0))
    env.step(np.zeros(env.action_space.shape, dtype=np.float32))
    assert env.safety.max_dq == pytest.approx(math.radians(1.5))
    env.close()


def test_motor_contract_reports_ramp():
    from rl_move.sim.servo_model import motor_contract
    cfg = _cfg(RAMP_KEYS)
    c = motor_contract(cfg)
    assert c["bus.profile_ramp_steps"] == pytest.approx(2_000_000)
    off = motor_contract(_cfg())
    assert off["bus.profile_ramp_steps"] == 0.0
