"""eval_modeseq.py --rise-from-h -- the "remaining-rise" semantics
probe (WAITING-ON 08-17, hw: prices the postlower `[operator]` fork
before a product-contract change; SESSION_BULK_GATE.md Cohort c5rr).

Root cause this exists: `bulk_session_eval`/`eval_modeseq` composes
already-trained specialists by RE-ANCHORING at every segment boundary
(a fresh `env.reset()` + physical-state restore) -- the post-lower
rise segment's height goal comes from the LEGACY cold-start sampler
(`GoalGenerator.sample`), which never reads
`goal.mode_seq_rise_from_h` at all (only
`SimHexapodGoalEnv._seq_segment_traj`, reached via
`_sample_mode_seq_stance`, does -- a code path this external harness
never enters). A naive `--cfg-set goal.mode_seq_rise_from_h=1` is
therefore CONFIRMED a no-op through this harness; these tests lock:

1. `rise_from_h_traj()` calls the REAL trained generator
   (`_seq_segment_traj`) and produces a schedule that holds at the
   robot's CURRENT height (not belly=0) and ramps to the target.
2. cfg is restored exactly (key absent before -> absent after; key
   present with some other value before -> that value after) -- the
   toggle never leaks into any other segment/episode.
3. the legacy path (flag unset) is untouched: `GoalGenerator.sample`
   still drives the mid-sequence rise, cold-belly-hold schedule.
4. `_seq_stand_z` is left alone (None for a bare reanchored env, no
   in-context `_seq_plan`) -- rise_from_h_traj must not require or
   fabricate cross-segment stand-height tracking to do its job.

Run: python3 -m pytest rl_move/tests/test_eval_modeseq_rise_from_h.py -q
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.eval_modeseq import rise_from_h_traj  # noqa: E402
from rl_move.sim.walk_task import SimHexapodJointWalkEnv  # noqa: E402


def _make_env(seed: int = 0, episode_seconds: float = 12.0):
    cfg = load_config()
    env = SimHexapodJointWalkEnv(cfg, seed=seed,
                                 episode_seconds=episode_seconds)
    return env, cfg


def _cold_rise_reset(env, gen):
    """Mirror eval_modeseq's reanchor_to("rise", force_rise_start=
    "flat"): a fresh rise-mode reset (installs _z0 at the belly
    frame), no physical-state restore needed for these unit tests
    (we move the chassis ourselves below)."""
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_rise = 1.0
    gen.force_rise_start = "flat"
    env.reset()
    gen.force_rise_start = None


def _lift_chassis(env, dz: float) -> None:
    """Raise the robot's current physical height by dz (simulating a
    carried-over post-lower pose above the belly reset frame) and
    resettle mujoco's derived quantities."""
    env.data.qpos[2] += dz
    mujoco.mj_forward(env.model, env.data)


def test_rise_from_h_holds_at_current_height_then_ramps():
    env, cfg = _make_env(seed=0)
    gen = env._goal_gen
    _cold_rise_reset(env, gen)
    z0 = env._z0
    _lift_chassis(env, 0.04)   # robot now ~40mm above the belly frame
    h_start_expect = float(env.data.xpos[env._chassis_bid, 2]) - z0
    assert h_start_expect > 0.03   # sanity: the lift actually landed
    traj, h_target = rise_from_h_traj(env, cfg)
    assert traj.mode == "rise"
    h = np.asarray(traj.height)
    assert np.isclose(h[0], h_start_expect, atol=1e-6), (
       "segment must START at the robot's CURRENT height, not belly=0")
    assert np.isclose(h[-1], h_target, atol=1e-6)
    assert h_target > h_start_expect, "must still target a real stand"
    # monotonic non-decreasing hold->ramp->hold shape
    assert np.all(np.diff(h) >= -1e-9)


def test_rise_from_h_cfg_toggle_is_scoped_and_restored():
    env, cfg = _make_env(seed=1)
    gen = env._goal_gen
    _cold_rise_reset(env, gen)
    _lift_chassis(env, 0.02)
    assert "mode_seq_rise_from_h" not in cfg.get("goal", {})
    rise_from_h_traj(env, cfg)
    assert "mode_seq_rise_from_h" not in cfg["goal"], (
        "the key must not leak past the call when it was absent before")
    cfg["goal"]["mode_seq_rise_from_h"] = 0.0   # explicit legacy value
    rise_from_h_traj(env, cfg)
    assert cfg["goal"]["mode_seq_rise_from_h"] == 0.0, (
        "an explicit prior value must be restored exactly, not cleared")


def test_rise_from_h_does_not_require_seq_stand_z():
    env, cfg = _make_env(seed=2)
    gen = env._goal_gen
    _cold_rise_reset(env, gen)
    _lift_chassis(env, 0.03)
    assert env._seq_stand_z is None, (
        "a bare reanchored env (no in-context _seq_plan) never sets "
        "_seq_stand_z -- rise_from_h_traj must work without it")
    traj, h_target = rise_from_h_traj(env, cfg)
    assert h_target > 0.0   # falls back to the legacy rng.uniform draw


def test_legacy_path_unaffected_when_flag_never_called():
    # The regression this whole probe exists to fix: a naive cfg-set
    # WITHOUT calling rise_from_h_traj is a no-op through the legacy
    # sampler -- GoalGenerator.sample ignores mode_seq_rise_from_h
    # entirely (only _seq_segment_traj reads it).
    env, cfg = _make_env(seed=3)
    gen = env._goal_gen
    cfg.setdefault("goal", {})["mode_seq_rise_from_h"] = 1
    _cold_rise_reset(env, gen)
    _lift_chassis(env, 0.05)
    h = np.asarray(env._goal_traj.height)
    assert np.isclose(h[0], 0.0), (
       "legacy sampler must still hold at belly=0 even with the cfg "
       "key set -- proves the key alone is a no-op through this path")
