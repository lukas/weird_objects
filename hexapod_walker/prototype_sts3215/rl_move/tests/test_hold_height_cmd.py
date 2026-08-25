"""Commandable standing-height schedule generator (goal.hold_height_
cmd_*, 08-25, operator MCP request fb_20260825T195117_3dce6e).

Unit-level contract for GoalGenerator._hold_height_schedule (the RNG-
driven training-time schedule); the reward ORDERING question (track
beats stale/flagleg/hover/skate) is banked separately in
test_task_semantics.py's "HOLD bank, COMMANDABLE HEIGHT variant"
against a pinned, exact profile. This file covers the generator
itself:
  - default (hold_height_cmd_frac=0) is bit-exact legacy: zero extra
    rng draw, height stays flat 0 for every hold episode;
  - every generated schedule stays inside hold_height_cmd_range_mm and
    never steps faster than hold_height_cmd_rate_mm_s, at both a
    normal and a very short (truncating) episode length;
  - the force_hold_height_profile canary/bank hook reaches the exact
    pinned (kind, target) pair.
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

from rl_move.sim.goal_task import GoalGenerator              # noqa: E402

DT = 0.01
N_STEPS = 1500


def _gen(frac: float = 1.0, rate_mm_s: float = 15.0,
        range_mm=(-40.0, 20.0), max_height_mm: float = 88.0
        ) -> GoalGenerator:
    cfg = {"goal": {"p_hold": 1.0, "hold_height_cmd_frac": frac,
                    "hold_height_cmd_range_mm": list(range_mm),
                    "hold_height_cmd_rate_mm_s": rate_mm_s},
          "actions": {"max_height_mm": max_height_mm}}
    return GoalGenerator(cfg)


def test_default_off_is_bit_exact_flat():
    """hold_height_cmd_frac=0 (the default) must never draw the extra
    rng and must leave every hold episode's height flat at 0 —
    existing hold-including lineages' rng streams are untouched."""
    gen = _gen(frac=0.0)
    for seed in range(10):
        rng = np.random.default_rng(seed)
        traj = gen.sample(rng, N_STEPS, DT, force_mode="hold")
        assert np.all(traj.height == 0.0), (
            f"seed {seed}: frac=0 leaked a nonzero height schedule")


@pytest.mark.parametrize("n_steps", (N_STEPS, 90))
def test_schedule_respects_range_and_rate(n_steps):
    """Every random draw stays inside the configured range and never
    steps faster than the configured rate, including a short,
    transition-truncating episode (n_steps=90, just past the settle
    window — the generator must slow down, never speed up; shorter
    than the settle window is an unsupported episode length, same as
    every other goal kind's ramp helper)."""
    gen = _gen()
    for seed in range(40):
        rng = np.random.default_rng(seed)
        traj = gen.sample(rng, n_steps, DT, force_mode="hold")
        h = np.asarray(traj.height)
        assert h.min() >= -0.0400001 and h.max() <= 0.0200001, (
            f"seed {seed} n={n_steps}: height {h.min()*1000:.2f}/"
            f"{h.max()*1000:.2f}mm left the configured range")
        if len(h) > 1:
            dh_mm_s = np.diff(h) / DT * 1000.0
            assert np.max(np.abs(dh_mm_s)) <= 15.0001, (
                f"seed {seed} n={n_steps}: max slope "
                f"{np.max(np.abs(dh_mm_s)):.3f}mm/s exceeds the "
                f"15mm/s rate limit")


def test_schedule_starts_settled_at_zero():
    """No episode may start with a step: the settle window (goal.
    ramp_s) must hold height at exactly 0 before any command begins,
    matching every other goal kind's no-step-at-spawn convention."""
    gen = _gen()
    rng = np.random.default_rng(0)
    traj = gen.sample(rng, N_STEPS, DT, force_mode="hold")
    settle_n = max(1, int(round(gen.ramp_s / DT)))
    assert np.all(np.asarray(traj.height)[:settle_n] == 0.0)


@pytest.mark.parametrize("kind,target_mm", [
    ("ramp", -30.0), ("ramp", 15.0), ("pulse", 18.0), ("sine", 12.0),
])
def test_force_profile_hook_reaches_the_pinned_target(kind, target_mm):
    """The canary/bank hook must reach the exact pinned target and
    never exceed the rate limit, for every kind."""
    gen = _gen()
    gen.force_hold_height_profile = (kind, target_mm * 0.001)
    rng = np.random.default_rng(3)
    traj = gen.sample(rng, N_STEPS, DT, force_mode="hold")
    h = np.asarray(traj.height)
    dh_mm_s = np.diff(h) / DT * 1000.0
    assert np.max(np.abs(dh_mm_s)) <= 15.0001
    if kind == "ramp":
        assert abs(h[-1] * 1000.0 - target_mm) < 0.5, (
            f"pinned ramp to {target_mm}mm ended at {h[-1]*1000:.2f}mm")
    else:
        # pulse/sine both return toward 0 by construction; the peak
        # excursion must still reach close to the pinned target.
        peak = h.max() if target_mm > 0 else h.min()
        assert abs(peak * 1000.0 - target_mm) < 1.5, (
            f"pinned {kind} to {target_mm}mm peaked at "
            f"{peak*1000:.2f}mm")


def test_range_is_clipped_to_the_action_envelope():
    """hold_height_cmd_range_mm is clipped to +/- actions.
    max_height_mm — a run cannot command a height the body-IK/action
    envelope could never reach."""
    gen = _gen(range_mm=(-200.0, 200.0), max_height_mm=25.0)
    assert gen.hold_height_cmd_range_m == (-0.025, 0.025)
