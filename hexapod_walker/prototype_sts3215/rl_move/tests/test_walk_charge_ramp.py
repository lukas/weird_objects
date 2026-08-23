"""reward.walk_charge_ramp_steps — trainer-driven dense-walk-charge
ramp-in (walkcurr track, 08-23).

Three straight rung-1 from-scratch arms (cw-walkcurr-pf-fwd1,
-fwd2-swing, -fwd2-swingterm800, all FAIL 08-23) froze into a
tilt-safe splayed crouch for their whole 2M budgets: the dense
per-step walk charge flow (~-4.7/step, loadslip-dominated via the
0.03 m travel floor) makes freezing the best REACHABLE policy and
instantly punishes exploratory flailing, while every income channel
(step events, freeprog, even the direction-free swing bonus at its
bank-capped dose) pays orders of magnitude less than the charges cost
(env/reward_walk_prog identically 0.0 across all three runs). This
ramp starts the four dense walk charges (k_loadslip_excess,
k_park_duty, k_walk_idle_charge, k_walk_heading) at
walk_charge_ramp_min_frac of their bank-proven dose and anneals them
linearly UP to full over reward.walk_charge_ramp_steps global env
steps, mirroring the term-penalty ramp's construction exactly
(cfg-armed, trainer-driven via apply_walk_charge_frac, default OFF =
bit-exact legacy, armed-but-unbroadcast sits at the FULL charges so
evals always judge the bank-proven pricing).

Contract under test:
  - default (key absent/0) is bit-exact OFF: no ramp state, apply
    raises, scale is 1.0, and stepped rewards match a keyless env;
  - ARMED env sits at the FULL charges until broadcast (scale 1.0);
  - frac 0 -> min_frac, 0.5 -> midpoint, >=1 -> 1.0, clamped;
  - fail-closed: min_frac outside [0, 1] raises at construction;
  - the live scale actually changes what gets charged (the heading
    charge on an identical trajectory scales by exactly the ratio).
"""
from __future__ import annotations

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
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

RAMP_KEYS = {
    ("reward", "walk_charge_ramp_steps"): 1_000_000,
    ("reward", "k_walk_heading"): 0.5,
    ("reward", "k_loadslip_excess"): 4.5,
}


def _cfg(extra=None):
    cfg = load_config()
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    return cfg


def _env(extra=None, seed=0, episode_seconds=2.0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = _cfg(extra)
    params = SimServoParams.from_cfg(cfg)
    return SimHexapodJointWalkEnv(
        params=params, randomize=False, dr_scale=0.0,
        episode_seconds=episode_seconds, seed=seed, cfg=cfg)


def test_default_off_bit_exact_and_apply_raises():
    env = _env()
    assert env._walk_charge_ramp is None
    assert env._walk_charge_override is None
    assert env._walk_charge_scale() == 1.0
    with pytest.raises(RuntimeError, match="not armed"):
        env.apply_walk_charge_frac(0.5)
    # armed=0 is the same OFF path
    env0 = _env({("reward", "walk_charge_ramp_steps"): 0})
    assert env0._walk_charge_ramp is None


def test_default_off_rewards_bit_exact():
    """An env whose cfg carries steps=0 must produce byte-identical
    rewards to a keyless env on the same seed/action sequence."""
    env_a = _env(seed=3)
    env_b = _env({("reward", "walk_charge_ramp_steps"): 0}, seed=3)
    env_a.reset(seed=3)
    env_b.reset(seed=3)
    rng = np.random.default_rng(0)
    for _ in range(25):
        act = rng.uniform(-1, 1, env_a.action_space.shape).astype(
            np.float32)
        _, ra, term_a, trunc_a, _ = env_a.step(act)
        _, rb, term_b, trunc_b, _ = env_b.step(act)
        assert ra == rb
        assert (term_a, trunc_a) == (term_b, trunc_b)
        if term_a or trunc_a:
            break


def test_armed_unbroadcast_sits_at_full_charges():
    env = _env(RAMP_KEYS)
    assert env._walk_charge_ramp is not None
    assert env._walk_charge_override is None
    assert env._walk_charge_scale() == 1.0


def test_frac_mapping_and_clamping():
    keys = dict(RAMP_KEYS)
    keys[("reward", "walk_charge_ramp_min_frac")] = 0.2
    env = _env(keys)
    out = env.apply_walk_charge_frac(0.0)
    assert out["charge_scale"] == pytest.approx(0.2)
    out = env.apply_walk_charge_frac(0.5)
    assert out["charge_scale"] == pytest.approx(0.6)
    out = env.apply_walk_charge_frac(2.0)   # clamps
    assert out["charge_scale"] == pytest.approx(1.0)
    out = env.apply_walk_charge_frac(-1.0)  # clamps
    assert out["charge_scale"] == pytest.approx(0.2)
    assert env._walk_charge_scale() == pytest.approx(0.2)
    # default min_frac when the key is absent
    env2 = _env(RAMP_KEYS)
    out2 = env2.apply_walk_charge_frac(0.0)
    assert out2["charge_scale"] == pytest.approx(0.15)


def test_bad_min_frac_fails_closed():
    keys = dict(RAMP_KEYS)
    keys[("reward", "walk_charge_ramp_min_frac")] = 1.5
    with pytest.raises(ValueError, match="must be in"):
        _env(keys)
    keys[("reward", "walk_charge_ramp_min_frac")] = -0.1
    with pytest.raises(ValueError, match="must be in"):
        _env(keys)


def test_live_scale_changes_the_charge():
    """Identical seed + action sequence; reward does not feed back
    into dynamics, so the heading charge must scale by exactly the
    broadcast ratio between two armed envs."""
    keys = dict(RAMP_KEYS)
    keys[("reward", "walk_charge_ramp_min_frac")] = 0.1
    # walkcurr rung-1 shaped context so the charges actually fire on
    # random flail: fixed forward command, loadslip stack armed.
    keys[("goal", "walk_pure")] = 1
    keys[("goal", "walk_speed_min_m_s")] = 0.05
    keys[("goal", "walk_speed_max_m_s")] = 0.05
    keys[("goal", "walk_heading_max_rad")] = 0.0
    keys[("reward", "walk_loadslip_gate")] = 0.75
    keys[("reward", "loadslip_ok")] = 1.2
    keys[("reward", "loadslip_max")] = 3.0
    keys[("reward", "loadslip_floor_m")] = 0.03
    env_full = _env(keys, seed=5, episode_seconds=6.0)
    env_min = _env(keys, seed=5, episode_seconds=6.0)
    env_full.apply_walk_charge_frac(1.0)   # scale 1.0
    env_min.apply_walk_charge_frac(0.0)    # scale 0.1
    env_full.reset(seed=5)
    env_min.reset(seed=5)
    rng = np.random.default_rng(1)
    got = 0
    for _ in range(40):
        act = rng.uniform(-1, 1, env_full.action_space.shape).astype(
            np.float32)
        *_a, info_f = env_full.step(act)
        *_b, info_m = env_min.step(act)
        for key in ("reward_walk_heading", "reward_loadslip_excess"):
            rf = info_f.get(key)
            rm = info_m.get(key)
            if rf is not None and rf != 0.0:
                assert rm == pytest.approx(0.1 * rf, rel=1e-6), (
                    f"{key} did not scale: full={rf} min={rm}")
                got += 1
        if _a[2] or _a[3]:   # term / trunc
            break
    assert got > 0, "no charge ever fired; probe is not exercising the ramp"
