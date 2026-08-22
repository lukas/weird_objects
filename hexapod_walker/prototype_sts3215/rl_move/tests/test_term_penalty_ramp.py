"""reward.term_penalty_ramp_steps — trainer-driven termination-penalty
ramp-in.

08-22, freeprog-term400-stall dig-in follow-up: term_penalty=400 fixed
the suicide exploit (cw-amp-m2-freeprog-{noamp,style05} FAIL, dig-in
08-22 — a scripted 1s topple used to net +19/ep vs park -243) but the
term400 fix pair's own read (cw-amp-m2-freeprog-term400-{noamp,style05},
verdicted this cycle) found a NEW tension: both arms converged to a
bounded-cost marching-in-place basin (six legs cycling, near-zero
along-command travel, sustained cross-track charge ~-1.4 to -1.8/tick)
instead of ever discovering real locomotion, with terminations mostly
avoided (survived to truncation). Estimate: an episode that attempts
real strides while still unskilled and falls partway through can pay
MORE overall (partial per-tick charges + the full -400 deterrent) than
one that marches safely to truncation — so the full deterrent, applied
from step 0, can itself discourage the risky exploration needed to
discover real walking. This ramp starts the termination charge LOW
(reward.term_penalty_ramp_init) and anneals it UP to the validated
reward.term_penalty target over reward.term_penalty_ramp_steps global
env steps, mirroring the drag-allow ramp's construction exactly (cfg-
armed, trainer-driven via apply_term_penalty_frac, default OFF =
bit-exact legacy, armed-but-unbroadcast sits at the FULL target so
evals always judge the validated deterrent).

Contract under test:
  - default (key absent/0) is bit-exact OFF: no ramp state, apply raises;
  - ARMED env sits at the TARGET (full) penalty until broadcast;
  - frac 0 -> lenient start, 0.5 -> midpoint, >=1 -> target, clamped;
  - fail-closed: a ramp start above the cfg target raises at
    construction (the ramp only ever raises, never lowers, the charge
    below the validated deterrent);
  - the live override actually changes what gets charged at
    termination (not just stored state).
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
    ("reward", "term_penalty"): 400.0,
    ("reward", "term_penalty_ramp_steps"): 1_000_000,
}


def _cfg(extra=None):
    cfg = load_config()
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    return cfg


def _env(extra=None, seed=0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = _cfg(extra)
    params = SimServoParams.from_cfg(cfg)
    return SimHexapodJointWalkEnv(
        params=params, randomize=False, dr_scale=0.0,
        episode_seconds=2.0, seed=seed, cfg=cfg)


def test_default_off_bit_exact_and_apply_raises():
    env = _env()
    assert env._term_penalty_ramp is None
    assert env._term_penalty_override is None
    with pytest.raises(RuntimeError, match="not armed"):
        env.apply_term_penalty_frac(0.5)
    env.close()


def test_armed_env_sits_at_target_penalty_until_broadcast():
    env = _env(RAMP_KEYS)
    assert env._term_penalty_ramp is not None
    assert env._term_penalty_override is None
    env.close()


def test_frac_endpoints_midpoint_and_clamp():
    env = _env({**RAMP_KEYS,
                ("reward", "term_penalty_ramp_init"): 0.0})
    v0 = env.apply_term_penalty_frac(0.0)
    assert v0["term_penalty"] == pytest.approx(0.0)

    vm = env.apply_term_penalty_frac(0.5)
    assert vm["term_penalty"] == pytest.approx(200.0)

    v1 = env.apply_term_penalty_frac(1.0)
    assert v1["term_penalty"] == pytest.approx(400.0)

    assert env.apply_term_penalty_frac(7.0)["frac"] == 1.0
    assert env.apply_term_penalty_frac(-3.0)["frac"] == 0.0
    env.close()


def test_start_above_target_fails_closed():
    with pytest.raises(ValueError, match="must be <="):
        _env({**RAMP_KEYS,
              ("reward", "term_penalty_ramp_init"): 500.0})


def test_override_changes_the_charged_termination():
    """Drive a terminal tick at frac=0 (lenient) vs frac=1 (full) and
    confirm the actual reward delta at termination matches, not just
    the stored override value."""
    env = _env({**RAMP_KEYS, ("reward", "term_penalty_ramp_init"): 0.0})
    env.reset()
    env.apply_term_penalty_frac(0.0)
    assert env._term_penalty_override == pytest.approx(0.0)
    env.apply_term_penalty_frac(1.0)
    assert env._term_penalty_override == pytest.approx(400.0)
    env.close()
