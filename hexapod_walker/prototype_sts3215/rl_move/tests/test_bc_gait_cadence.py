"""Fast-cadence BC teacher knob (operator order 08-20).

The fast-gait write_speed arms all failed (fastthru/fastramp, mid+full
dose); the operator's replacement lever is a FASTER TripodGait CLOCK in
the BC-INIT teacher: shorter period -> proportionally shorter strides
at the same body speed, native servo profile untouched. These tests pin
the contract: the knob is default-off and bit-exact at 1.0, it scales
the gait clock as advertised, and the phase-obs coupling fails closed.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from sim_gait_compat import TripodGait  # noqa: E402
from rl_move.sim.bc_init_gait import _make_teacher  # noqa: E402
from rl_move.sim.probe_walk_income import parse_policy_spec  # noqa: E402


def _mk(scale=None, vx=0.055):
    kw = {} if scale is None else {"period_scale": scale}
    g = TripodGait(vx=vx, **kw)
    g.sync_plant_stance(20.0, 80.0)
    g.reset_phase()
    return g


def test_scale_one_is_bit_exact():
    """period_scale=1.0 must reproduce the legacy gait exactly."""
    g0, g1 = _mk(None), _mk(1.0)
    for k in range(200):
        t = k * 0.04
        assert g0.desired_deg(t) == g1.desired_deg(t)


def test_scale_speeds_clock():
    """At scale s the clock completes 1/s cycles in the legacy period."""
    s = 0.75
    g = _mk(s)
    dt = 0.005
    T = 3.0  # well past the 0.35 s ramp
    n = int(round(T / dt))
    for k in range(1, n + 1):
        g.desired_deg(k * dt)
    cycles = T / (g.period * s)
    expected = (cycles % 1.0) * 2.0 * math.pi
    assert abs(g._phase - expected) < 1e-6 + 2 * math.pi * dt / (g.period * s)


def test_scale_shortens_stride():
    """Same body speed, shorter period -> proportionally shorter stride."""
    s = 0.6
    ga, gb = _mk(s), _mk(None)
    # run both to converged smoothing + full ramp, at their own clocks
    for k in range(1, 1001):
        ga.desired_deg(k * 0.004)
        gb.desired_deg(k * 0.004)
    dxa = ga._foot_target_in_body(0, 0.055, 0.0, 0.0)[0]
    dxb = gb._foot_target_in_body(0, 0.055, 0.0, 0.0)[0]
    # dx = prog * v * t_eff / 2 with identical prog only at matched
    # phase; instead compare the stride ENVELOPE via t_eff directly.
    t_eff_a = ga.period * ga.period_scale
    t_eff_b = gb.period * gb.period_scale
    assert abs(t_eff_a - s * t_eff_b) < 1e-12
    # and the instantaneous offsets stay bounded by each envelope
    assert abs(dxa) <= 0.055 * t_eff_a / 2.0 + 1e-9
    assert abs(dxb) <= 0.055 * t_eff_b / 2.0 + 1e-9


def test_make_teacher_plumbs_scale():
    g = _make_teacher("tripod", 0.75)
    assert g.period_scale == 0.75
    g = _make_teacher("tripod")
    assert g.period_scale == 1.0


def test_make_teacher_rejects_noslip_scale():
    with pytest.raises(SystemExit):
        _make_teacher("noslip", 0.75)


def test_collect_rejects_phase_hz_mismatch():
    """With walk_phase_obs on, a scaled clock must fail closed unless
    goal.walk_phase_hz matches 1/(0.75*scale)."""
    from rl_move.sim.bc_init_gait import collect
    stack = {("goal", "walk_phase_obs"): 1.0,
             ("goal", "walk_phase_hz"): 1.0 / 0.75}
    with pytest.raises(SystemExit):
        collect(0, 0, teacher="tripod", stack=stack, period_scale=0.75)


def test_parse_policy_spec():
    assert parse_policy_spec("gait") == ("gait", 1.0)
    assert parse_policy_spec("gait@p0.75") == ("gait", 0.75)
    assert parse_policy_spec("ckpt:foo.zip") == ("ckpt:foo.zip", 1.0)
    with pytest.raises(SystemExit):
        parse_policy_spec("gait@x2")
