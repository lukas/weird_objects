"""Speed-coupled walk phase clock (goal.walk_phase_speed_scale).

Root cause this mechanism repairs (amp M2, 08-22): the walk_phase_obs
clock advanced at a FIXED hz whenever any linear velocity was
commanded, so the actor's cadence could not follow the commanded
speed — cw-amp-m2-bcinit-sec5-style05-speedrange compressed a
0.05-0.25 m/s command range into ~0.10+/-0.02 m/s realized, and the
fastphase / fastphase-nostyle probe pair showed neither a faster
CONSTANT clock nor removing AMP style widens it.

Contract under test:
- default OFF (scale <= 0): phase_hz_effective returns hz_base
  EXACTLY for any commanded speed (legacy bit-exact);
- anchor point: s_ref == s_nom returns hz_base exactly for any scale
  (the teacher's cadence at its natural speed is preserved);
- proportionality at scale=1: hz_eff scales linearly with s_ref
  (double the command -> double the clock rate);
- partial coupling 0<k<1 interpolates between fixed and proportional;
- hz_max clamps the top, never the bottom; result never negative;
- env-level: with the key OFF the per-tick phase advance of
  SimHexapodJointWalkEnv is independent of commanded speed; with
  scale=1 the advance tracks s_ref/s_nom.
"""
import math

import numpy as np
import pytest

from rl_move.sim.walk_task import (
    PHASE_HZ_DEFAULT, PHASE_SPEED_NOM_DEFAULT, phase_hz_effective)


def test_off_is_identity_for_any_speed():
    for s in (0.0, 0.01, 0.08, 0.16, 0.25, 3.0):
        assert phase_hz_effective(1.333333, s, 0.0) == 1.333333
        assert phase_hz_effective(1.333333, s, -1.0) == 1.333333


def test_anchor_speed_preserves_base_rate():
    for k in (0.25, 0.5, 1.0, 2.0):
        assert phase_hz_effective(1.333333, 0.08, k, s_nom=0.08) \
            == pytest.approx(1.333333)


def test_full_coupling_is_proportional():
    hz0 = 1.333333
    for mult in (0.5, 1.0, 2.0, 3.0):
        hz = phase_hz_effective(hz0, 0.08 * mult, 1.0, s_nom=0.08)
        assert hz == pytest.approx(hz0 * mult)


def test_partial_coupling_interpolates():
    hz0 = 2.0
    # k=0.5 at 2x nominal speed -> 1.5x base rate
    assert phase_hz_effective(hz0, 0.16, 0.5, s_nom=0.08) \
        == pytest.approx(hz0 * 1.5)
    # k=0.5 at 0.5x nominal -> 0.75x base
    assert phase_hz_effective(hz0, 0.04, 0.5, s_nom=0.08) \
        == pytest.approx(hz0 * 0.75)


def test_clamp_and_floor():
    # top clamp
    assert phase_hz_effective(1.333333, 0.25, 1.0, s_nom=0.08,
                              hz_max=3.0) == pytest.approx(3.0)
    # hz_max=0 means no clamp
    assert phase_hz_effective(1.333333, 0.25, 1.0, s_nom=0.08,
                              hz_max=0.0) > 3.0
    # never negative even for absurd k
    assert phase_hz_effective(1.0, 0.001, 50.0, s_nom=0.08) >= 0.0
    # zero / non-positive s_ref falls back to base (caller gates on
    # s_ref > 1e-3 anyway)
    assert phase_hz_effective(1.0, 0.0, 1.0) == 1.0


def _phase_advance_per_tick(env, vx):
    """Reset, force a straight-line command of speed vx, measure the
    obs-clock advance over one step from the sin/cos tail."""
    from rl_move.sim.walk_task import N_MODE_OBS
    env.reset(seed=0)
    goal = env._current_goal()
    # force the command the clock sees
    goal.vx_ref, goal.vy_ref = float(vx), 0.0
    p0 = float(env._phase)
    a = np.zeros(env.action_space.shape, dtype=np.float32)
    env.step(a)
    goal2 = env._current_goal()
    if goal2 is not None:
        goal2.vx_ref, goal2.vy_ref = float(vx), 0.0
    p1 = float(env._phase)
    d = (p1 - p0) % (2.0 * math.pi)
    return d


@pytest.mark.parametrize("scale,expect_ratio", [(0.0, 1.0), (1.0, 2.0)])
def test_env_phase_advance_tracks_command(scale, expect_ratio):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = {
        "goal": {
            "walk_phase_obs": 1,
            "walk_phase_hz": 1.333333,
            "walk_phase_speed_scale": scale,
            "walk_phase_speed_nom": 0.08,
            "mix": {"walk": 1.0},
        },
    }
    env = SimHexapodJointWalkEnv(cfg=cfg)
    try:
        d_lo = _phase_advance_per_tick(env, 0.08)
        d_hi = _phase_advance_per_tick(env, 0.16)
        assert d_lo > 0.0
        assert d_hi / d_lo == pytest.approx(expect_ratio, rel=0.05)
    finally:
        env.close()
