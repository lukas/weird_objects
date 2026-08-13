"""Physics easing (ease.* cfg keys, 2026-08-13 — GAIT.md P3 lever 3).

ease.gravity_scale / ease.vel_ceiling_scale multiply one EPISODE's
gravity magnitude and servo velocity ceiling, read from cfg at every
reset so the in-run sched.* engine can anneal them (eased physics
early, nominal by the end). Contract under test:

- default OFF (keys unset) and explicit 1.0 are bit-exact legacy:
  identical reset physics, no state mutation;
- gravity scale reaches the model: |g| scaled, direction preserved
  (slope DR stays a direction-only effect), both with DR on
  (via the _ep_rand draw) and with randomize=False private-model
  envs (reset() fallback);
- vel ceiling scale reaches the servo profile the same two ways;
- values are re-read at EVERY reset (a sched-driven cfg write between
  episodes changes the next episode's physics);
- a shared-model shim env without DR raises loudly instead of
  silently training on uneased physics;
- non-positive scales raise.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

mujoco = pytest.importorskip("mujoco")

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.sim_env import SimHexapodBalanceEnv  # noqa: E402



def _make_env(ease: dict | None, *, randomize: bool = False,
              seed: int = 0, **kw):
    cfg = load_config()
    if ease is not None:
        cfg["ease"] = dict(ease)
    return SimHexapodBalanceEnv(seed=seed, cfg=cfg, randomize=randomize,
                                episode_seconds=2.0, **kw)


def test_ease_off_is_bitexact():
    """Unset keys and explicit 1.0 give byte-identical reset physics."""
    obs_ref, _ = _make_env(None, seed=3).reset()
    env_unset = _make_env(None, seed=3)
    env_one = _make_env({"gravity_scale": 1.0, "vel_ceiling_scale": 1.0},
                        seed=3)
    o_unset, _ = env_unset.reset()
    o_one, _ = env_one.reset()
    assert np.array_equal(o_unset, obs_ref)
    assert np.array_equal(o_one, obs_ref)
    assert np.array_equal(env_unset.model.opt.gravity,
                          env_one.model.opt.gravity)
    assert env_unset._ease_g == env_one._ease_g == 1.0
    assert env_unset._ease_v == env_one._ease_v == 1.0


def test_gravity_ease_private_model_no_dr():
    """randomize=False fallback: model gravity scaled at reset."""
    env = _make_env({"gravity_scale": 0.5})
    env.reset()
    assert np.allclose(env.model.opt.gravity,
                       0.5 * np.asarray(env._base_gravity), rtol=1e-9)


def test_gravity_ease_with_dr_scales_draw_keeps_direction():
    env = _make_env({"gravity_scale": 0.5}, randomize=True, dr_scale=0.5)
    base = _make_env(None, randomize=True, dr_scale=0.5)
    env.reset()
    base.reset()  # same seed => identical DR draw before easing
    g_e = np.asarray(env._ep_rand.gravity_vec, float)
    g_b = np.asarray(base._ep_rand.gravity_vec, float)
    assert np.allclose(g_e, 0.5 * g_b, rtol=1e-9)   # magnitude halved
    # direction identical (slope DR preserved)
    assert np.allclose(g_e / np.linalg.norm(g_e),
                       g_b / np.linalg.norm(g_b), rtol=1e-9)
    # and the model got the eased vector
    assert np.allclose(env.model.opt.gravity, g_e)


def test_vel_ease_private_model_no_dr():
    env = _make_env({"vel_ceiling_scale": 1.5})
    base = _make_env(None)
    env.reset()
    base.reset()
    assert np.allclose(env._profile._vel_default,
                       1.5 * base._profile._vel_default, rtol=1e-9)


def test_vel_ease_with_dr_scales_draw():
    env = _make_env({"vel_ceiling_scale": 1.5}, randomize=True,
                    dr_scale=0.5)
    base = _make_env(None, randomize=True, dr_scale=0.5)
    env.reset()
    base.reset()
    assert np.isclose(env._ep_rand.vel_scale,
                      1.5 * base._ep_rand.vel_scale, rtol=1e-9)


def test_ease_reread_each_reset():
    """A cfg write between episodes (what sched.* does) moves the next
    episode's physics — the keys are live, not construction-frozen."""
    env = _make_env({"gravity_scale": 0.5})
    env.reset()
    base = np.asarray(env._base_gravity)
    assert np.allclose(env.model.opt.gravity, 0.5 * base, rtol=1e-9)
    env.cfg["ease"]["gravity_scale"] = 1.0   # schedule reached nominal
    env.reset()
    assert np.allclose(env.model.opt.gravity, base, rtol=1e-9)
    assert env._ease_g == 1.0


def test_ease_shared_model_shim_without_dr_raises():
    donor = _make_env(None)
    shim = _make_env({"gravity_scale": 0.5}, model=donor.model)
    with pytest.raises(ValueError, match="randomize=True"):
        shim.reset()


def test_ease_nonpositive_raises():
    env = _make_env({"gravity_scale": 0.0})
    with pytest.raises(ValueError, match="must be > 0"):
        env.reset()
    env2 = _make_env({"vel_ceiling_scale": -1.0})
    with pytest.raises(ValueError, match="must be > 0"):
        env2.reset()
