"""Unit tests for `goal.walk_stop_settle_s` / `stop_speed_settled_m_s`
(2026-08-24, joyfullcurr10-chg2/chg4 dig-in / cert-methodology audit).

The V6 walk-curriculum cert's `stop_speed_m_s` probe metric
(`walk_task._walk_probe_tick`) has always counted EVERY stop tick from
the very first one of a commanded-stop segment, uniformly averaged --
unlike the training reward's own stop charges
(`reward.k_walk_stop_charge` / `k_walk_stop_current`), which explicitly
ramp their multiplier 0->1 over `reward.walk_stop_grace_s` because the
unavoidable physical deceleration transient right after a stop command
is not creep and must not be priced like it is (the joyfullcurr7
finding). Dosing the reward charge (joyfullcurr9/10 ladder) can only
ever discipline post-grace ticks; a dose-insensitive `stop_speed_m_s`
plateau does not, by itself, prove the residual creep is unshapeable
-- it may just mean the cert is still counting the untouched transient
window every time.

`goal.walk_stop_settle_s` (default 0.0, purely additive metric, the
legacy `stop_speed_m_s` computation is completely untouched) lets an
offline read exclude the first N seconds of EACH stop segment from a
new `stop_speed_settled_m_s` field, so the two can be compared without
any new training.

Fast (~seconds): short episodes, scripted actions, no PPO.
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
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402
from sim_gait_compat import TripodGait  # noqa: E402

FC_GOAL = {
    ("goal", "walk_yaw_cmd"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.06,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_heading_max_rad"): 0.0,
    ("goal", "walk_stop_frac"): 0.0,
    ("goal", "walk_cmd_resample_s"): 0.0,
    ("goal", "walk_cmd_resample_jitter"): 0.0,
    ("goal", "walk_cmd_blend_s_min"): 0.0,
    ("goal", "walk_cmd_blend_s_max"): 0.0,
    ("goal", "walk_yaw_zero_frac"): 1.0,
    ("goal", "walk_probe"): 1.0,
}


def _walk_env(extra=None, episode_seconds=8.0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in {**FC_GOAL, **(extra or {})}.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=0, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _creep_action(t):
    gait = TripodGait(vx=0.04, lift=0.025)
    gait.sync_plant_stance(20.0, 80.0)
    return q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)


def _command_stop(env):
    traj = env._goal_traj
    traj.vx[:] = 0.0
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0


def _run_to_probe(extra=None, episode_seconds=8.0):
    env = _walk_env(extra=extra, episode_seconds=episode_seconds)
    env.reset(seed=0)
    _command_stop(env)
    probe = None
    for step in range(env.episode_steps + 2):
        act = _creep_action(step * env.dt)
        _o, _r, term, trunc, info = env.step(act)
        if "walk_probe" in info:
            probe = info["walk_probe"]
        if term or trunc:
            break
    env.close()
    assert probe is not None, "episode ended without a walk_probe payload"
    return probe


def test_default_settle_s_reproduces_legacy_stop_speed_exactly():
    """settle_s absent (default 0.0) must make the new
    `stop_speed_settled_m_s` bit-identical to the legacy
    `stop_speed_m_s` -- every stop tick counted, exactly as before."""
    probe = _run_to_probe()
    assert probe["stop_ticks_settled_frac"] == pytest.approx(1.0)
    assert probe["stop_speed_settled_m_s"] == pytest.approx(
        probe["stop_speed_m_s"], abs=1e-12)


def test_legacy_stop_speed_unaffected_by_settle_s():
    """Changing walk_stop_settle_s must NEVER change the legacy
    `stop_speed_m_s` field -- it is a wholly separate, additive
    accumulator (bit-exact-when-absent guarantee for the existing
    cert gate)."""
    p0 = _run_to_probe(extra={("goal", "walk_stop_settle_s"): 0.0})
    # settle_s must clear the probe's own 2.0s episode-start warmup
    # (head_ticks) to have any visible effect in this scenario (stop
    # is commanded from tick 0, so the stop segment and the episode
    # start together) -- 3.0s excludes roughly the 2.0-3.0s window on
    # top of that warmup.
    p1 = _run_to_probe(extra={("goal", "walk_stop_settle_s"): 3.0})
    assert p0["stop_speed_m_s"] == pytest.approx(
        p1["stop_speed_m_s"], abs=1e-12)
    assert p0["stop_ticks_settled_frac"] > p1["stop_ticks_settled_frac"]


def test_settle_window_longer_than_episode_yields_nan():
    """A settle window that never elapses within the single commanded
    stop segment must leave zero settled ticks -> nan, not a
    divide-by-zero or a silent 0.0."""
    probe = _run_to_probe(
        extra={("goal", "walk_stop_settle_s"): 100.0},
        episode_seconds=6.0)
    assert probe["stop_ticks_settled_frac"] == pytest.approx(0.0)
    assert probe["stop_speed_settled_m_s"] != probe["stop_speed_settled_m_s"]


def test_walking_segment_resets_the_settle_timer():
    """A resumed walk command mid-episode must start a FRESH stop
    segment (fresh settle clock) the next time a stop is commanded --
    mirrors _walk_stop_cmd_s's own reset rule, checked independently
    here since the settle timer is a separate accumulator."""
    env = _walk_env(episode_seconds=10.0)
    env.reset(seed=0)
    _command_stop(env)
    for step in range(20):
        env.step(_creep_action(step * env.dt))
    assert env._wp["stop_seg_s"] > 0.0
    traj = env._goal_traj
    traj.vx[:] = 0.06
    env.step(_creep_action(0.0))
    assert env._wp["stop_seg_s"] == 0.0
    env.close()
