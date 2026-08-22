"""bc_init_gait.py --drive-omega (BC-turn-clone, amp M2-yaw, 08-22).

Root cause this mechanism repairs: every BC clone lineage was
demonstrated with ``gait.set_velocity(vx=.., vy=..)`` only -- the
teacher's own TripodGait/NoSlipGait omega (turn-in-place) channel was
NEVER driven, even when the env's sampled goal carried a nonzero
wz_ref (goal.walk_yaw_cmd=1). Every downstream RL arm therefore had to
invent turning from a zero-omega BC prior and never did (tip50/tip90:
park at ~|wz_ref| regardless of exposure or the phase-clock fix).

Contract under test:
- default (drive_omega=False): bit-exact regardless of whether the
  stack happens to carry goal.walk_yaw_cmd=1 -- the omega channel is
  never touched, so the demonstrated ACTIONS are identical whether or
  not a yaw command is being sampled (only the recorded obs WIDTH
  differs, from the extra wz_ref tail feature).
- drive_omega=True requires goal.walk_yaw_cmd=1 in the stack (fails
  closed, not a silent no-op, if the env has no wz to read).
- drive_omega=True with a real yaw command actually changes the
  demonstrated actions relative to the omega=0 default on ticks where
  wz_ref != 0 (the dataset really is teaching a different motor
  pattern, not a no-op plumbing change).
- obs.mode_onehot is refused together with goal.walk_phase_obs (the
  tail-index arithmetic this collector relies on does not extend to
  it) -- fail closed rather than silently mis-index the phase.
"""
import numpy as np
import pytest

from rl_move.sim.bc_init_gait import collect
from rl_move.sim.probe_walk_income import STACKS

VREF1 = STACKS["vref1"]


def _yaw_stack(**extra):
    s = dict(VREF1)
    s[("goal", "walk_yaw_cmd")] = 1.0
    s[("goal", "walk_yaw_max_rad_s")] = 0.3
    s[("goal", "walk_yaw_zero_frac")] = 0.0  # every episode commands a turn
    s[("goal", "walk_phase_obs")] = 1.0
    s[("goal", "walk_phase_hz")] = 1.0 / 0.75
    s[("goal", "walk_phase_run_on_yaw")] = 1.0
    s.update(extra)
    return s


def test_drive_omega_requires_yaw_cmd():
    with pytest.raises(SystemExit):
        collect(1, seed0=0, stack=dict(VREF1), drive_omega=True)


def test_mode_onehot_refused_with_phase_obs():
    s = dict(VREF1)
    s[("goal", "walk_phase_obs")] = 1.0
    s[("goal", "walk_phase_hz")] = 1.0 / 0.75
    s[("obs", "mode_onehot")] = 1.0
    with pytest.raises(SystemExit):
        collect(1, seed0=0, stack=s, drive_omega=False)


def test_default_off_bit_exact_regardless_of_yaw_cmd():
    # No phase_obs (keeps the two stacks' obs WIDTH identical: the
    # tail-index shift being tested elsewhere only matters with
    # phase_obs on) -- isolates the omega no-op claim from the layout
    # question tested separately below.
    plain = dict(VREF1)
    yawed = dict(VREF1)
    yawed[("goal", "walk_yaw_cmd")] = 1.0
    yawed[("goal", "walk_yaw_max_rad_s")] = 0.3
    yawed[("goal", "walk_yaw_zero_frac")] = 0.0
    Xa, Ya, _ = collect(2, seed0=0, stack=plain, drive_omega=False)
    Xb, Yb, _ = collect(2, seed0=0, stack=yawed, drive_omega=False)
    # Actions (what the teacher DOES) must be identical: omega is
    # never read on this path no matter what the env samples.
    assert Ya.shape == Yb.shape
    np.testing.assert_array_equal(Ya, Yb)
    # Obs differ only by the extra wz_ref tail column.
    assert Xb.shape[1] == Xa.shape[1] + 1


def test_drive_omega_changes_actions_on_turn_ticks():
    s = _yaw_stack()
    _, Y_straight, _ = collect(2, seed0=0, stack=s, drive_omega=False)
    _, Y_turn, _ = collect(2, seed0=0, stack=s, drive_omega=True)
    assert Y_straight.shape == Y_turn.shape
    # Same command stream (same seed), only the teacher's omega input
    # differs -- if the mechanism is wired, most ticks (every walk
    # tick, since walk_yaw_zero_frac=0 above) must show a real action
    # delta, not a rounding-noise one.
    delta = np.abs(Y_turn - Y_straight)
    frac_changed = float((delta.max(axis=1) > 1e-4).mean())
    assert frac_changed > 0.5, (
        f"only {frac_changed:.2f} of ticks changed action under "
        "drive_omega=True; the omega channel does not look wired")


def test_phase_recovered_correctly_with_yaw_cmd_tail_shift():
    # With walk_yaw_cmd=1 the phase pair moves from obs[-2:] to
    # obs[-3:-1] (one extra wz_ref column at the true tail). If the
    # collector mis-indexed, t_gait would desync from the true env
    # clock and the teacher would produce a degenerate (non-cyclic)
    # action stream. A real gait must revisit similar actions roughly
    # every teacher period; check the action stream has near-repeats
    # (low nearest-neighbor distance a whole period apart), not a
    # generic assertion. Cheap proxy: action array is far from
    # constant (a frozen/mis-synced clock collapses to one action).
    s = _yaw_stack()
    _, Y, _ = collect(2, seed0=0, stack=s, drive_omega=True)
    per_dim_std = Y.std(axis=0)
    assert float(per_dim_std.max()) > 0.02, (
        "action stream nearly constant -- phase tail index is likely "
        "wrong for goal.walk_yaw_cmd=1 (mis-synced gait clock)")
