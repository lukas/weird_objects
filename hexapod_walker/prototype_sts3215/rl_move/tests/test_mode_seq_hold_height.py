"""goal.mode_seq_hold_height_cmd tests (08-26, STAND_HEIGHT rung 5 prep).

Wires the existing standalone-hold height-command schedule
(`GoalGenerator._hold_height_schedule`, goal.hold_height_cmd_*) into a
mid-sequence "hold" segment of `goal.mode_seq_stance`
(`SimHexapodGoalEnv._seq_segment_traj`), which previously always left
mid-sequence hold segments at a flat zero height regardless of the
hold_height_cmd_* config. New key `goal.mode_seq_hold_height_cmd`
(default 0 = OFF, bit-exact) gates it; ON only takes effect when the
generator's own `goal.hold_height_cmd_frac` is also nonzero.

These tests lock:
1. default OFF is bit-exact (flat-zero hold segment), even with
   hold_height_cmd_frac configured on -- the new key is the switch;
2. ON (+ hold_height_cmd_frac>0) makes the hold segment vary, starting
   continuously at 0 (no seam discontinuity) at the segment's own
   local clock, and never leaving the configured range;
3. ON with hold_height_cmd_frac==0 (the default) still stays flat --
   both switches must be on;
4. unset vs explicit 0.0 produce an identical observation stream
   (paranoid parity, matching this file's siblings' convention).
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

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.joint_task import SimHexapodJointGoalEnv  # noqa: E402


def _make_env(seed: int = 0, *, hh_cmd: float | None = None,
              hh_frac: float = 1.0, episode_seconds: float = 8.0):
    cfg = load_config()
    g = cfg.setdefault("goal", {})
    g["mode_seq_stance"] = 1.0
    g["hold_height_cmd_frac"] = hh_frac
    g["hold_height_cmd_range_mm"] = [-40.0, 20.0]
    g["hold_height_cmd_rate_mm_s"] = 15.0
    if hh_cmd is not None:
        g["mode_seq_hold_height_cmd"] = hh_cmd
    env = SimHexapodJointGoalEnv(cfg, seed=seed,
                                 episode_seconds=episode_seconds)
    return env


def test_default_off_flat_hold_segment():
    env = _make_env(hh_cmd=None)  # key unset = default 0 = OFF
    env.reset()
    env._z0 = 0.083
    tick = 40
    traj, h_target, ramp_i0 = env._seq_segment_traj("hold", tick)
    assert float(np.abs(traj.height[tick:]).max()) == 0.0
    assert h_target == 0.0 and ramp_i0 == 0
    env.close()


def test_on_but_frac_zero_stays_flat():
    env = _make_env(hh_cmd=1.0, hh_frac=0.0)
    env.reset()
    env._z0 = 0.083
    tick = 40
    traj, _, _ = env._seq_segment_traj("hold", tick)
    assert float(np.abs(traj.height[tick:]).max()) == 0.0
    env.close()


def test_on_composes_varying_continuous_in_range_schedule():
    env = _make_env(hh_cmd=1.0, hh_frac=1.0)
    env.reset()
    env._z0 = 0.083
    tick = 30
    traj, _, _ = env._seq_segment_traj("hold", tick)
    seg = traj.height[tick:]
    # varies (the whole point of the fix)
    assert float(np.ptp(seg)) > 0.001
    # continuous at the seam: the settle window starts exactly at 0,
    # matching the frame the preceding rise segment just landed at.
    assert seg[0] == pytest.approx(0.0, abs=1e-9)
    # never asked outside the configured range
    lo_m, hi_m = -0.040, 0.020
    assert seg.min() >= lo_m - 1e-6 and seg.max() <= hi_m + 1e-6
    # head before the segment start is untouched (still zero)
    assert float(np.abs(traj.height[:tick]).max()) == 0.0
    env.close()


def test_unset_matches_explicit_zero_stream():
    a = _make_env(seed=5, hh_cmd=None, hh_frac=1.0)
    b = _make_env(seed=5, hh_cmd=0.0, hh_frac=1.0)
    oa, _ = a.reset()
    ob, _ = b.reset()
    np.testing.assert_array_equal(oa, ob)
    rng = np.random.default_rng(0)
    for _ in range(40):
        act = rng.uniform(-0.2, 0.2, a.action_space.shape[0])
        ra = a.step(act)
        rb = b.step(act)
        np.testing.assert_array_equal(ra[0], rb[0])
    a.close()
    b.close()
