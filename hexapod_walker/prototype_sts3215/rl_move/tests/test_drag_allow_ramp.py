"""reward.drag_stance_allow_ramp_steps — trainer-driven drag-stance
allowance ramp-in.

08-22, phasedir9 seed-lottery dig-in follow-up: the n=4 seed sample
(longrun17 PASS-partial, seed13/23/29 FAIL) closed the "more budget /
more seeds" lever at a 1/4 pass rate, per the pre-registered
prediction-if-false. The pd8 regime-gap dig-in (pd8_digin_regime/)
measured that the det-calibrated reward.drag_stance_allow_mm=24
allowance taxes the honest clone 0.76-9.7x its own income at the
training action-noise std (0.135) while the det drag cheat pays
0.002-0.36x -- a fixed tight allowance punishes noisy honest
exploration far harder than the cheat BEFORE the log-std anneal has
converged, which is exactly when PPO's basin selection happens. This
ramp lets a run START with a loose allowance (sized above the noisy-
honest tail) and anneal down to the same validated target allowance
over reward.drag_stance_allow_ramp_steps global env steps, mirroring
bus.profile_ramp_steps' construction exactly (cfg-armed, trainer-
driven via apply_drag_allow_frac, default OFF = bit-exact legacy).

Contract under test:
  - default (key absent/0) is bit-exact OFF: no ramp state, apply raises;
  - ARMED env sits at the TARGET allowance until broadcast (eval_
    checkpoint / play / periodic evals judge the calibrated final
    pricing without any broadcast);
  - frac 0 -> loose start, 0.5 -> midpoint, >=1 -> target, clamped;
  - fail-closed: a ramp start allowance below the cfg target raises at
    construction (the ramp only ever loosens, never tightens);
  - the live override actually changes what the drag_stance charge
    computes (not just stored state) — probed via reward telemetry.
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
    ("reward", "k_drag_stance"): 8000.0,
    ("reward", "drag_stance_allow_mm"): 24.0,
    ("reward", "drag_stance_allow_ramp_steps"): 1_000_000,
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
    assert env._drag_allow_ramp is None
    assert env._drag_allow_override_m is None
    with pytest.raises(RuntimeError, match="not armed"):
        env.apply_drag_allow_frac(0.5)
    env.close()


def test_armed_env_sits_at_target_allowance_until_broadcast():
    env = _env(RAMP_KEYS)
    assert env._drag_allow_ramp is not None
    # No broadcast yet -> override stays None -> reward code falls back
    # to the cfg target lookup (the eval contract).
    assert env._drag_allow_override_m is None
    env.close()


def test_frac_endpoints_midpoint_and_clamp():
    env = _env({**RAMP_KEYS,
                ("reward", "drag_stance_allow_ramp_mm"): 48.0})
    v0 = env.apply_drag_allow_frac(0.0)
    assert v0["allow_mm"] == pytest.approx(48.0)
    assert env._drag_allow_override_m == pytest.approx(0.048)

    vm = env.apply_drag_allow_frac(0.5)
    assert vm["allow_mm"] == pytest.approx((48.0 + 24.0) / 2)

    v1 = env.apply_drag_allow_frac(1.0)
    assert v1["allow_mm"] == pytest.approx(24.0)

    assert env.apply_drag_allow_frac(7.0)["frac"] == 1.0
    assert env.apply_drag_allow_frac(-3.0)["frac"] == 0.0
    env.close()


def test_default_start_is_48mm():
    env = _env(RAMP_KEYS)
    v0 = env.apply_drag_allow_frac(0.0)
    assert v0["allow_mm"] == pytest.approx(48.0)
    env.close()


def test_start_below_target_fails_closed():
    with pytest.raises(ValueError, match="must be >="):
        _env({**RAMP_KEYS,
              ("reward", "drag_stance_allow_ramp_mm"): 10.0})


def test_override_actually_changes_the_charged_allowance():
    """The stored frac must move what the reward computes, not just
    bookkeeping — drive a foot into sustained loaded drag and confirm
    the charge only engages once travel exceeds the LIVE allowance."""
    from rl_move.sim.walk_task import cfg_get

    env = _env(RAMP_KEYS)
    env.reset()
    # At frac=0 (48mm allowance) a foot must be reading the loose
    # value; at frac=1 (24mm) the tight cfg target.
    env.apply_drag_allow_frac(0.0)
    assert env._drag_allow_override_m == pytest.approx(0.048)
    env.apply_drag_allow_frac(1.0)
    target_mm = float(cfg_get(env.cfg, "reward",
                              "drag_stance_allow_mm", default=6.0))
    assert env._drag_allow_override_m == pytest.approx(target_mm / 1000.0)
    env.close()
