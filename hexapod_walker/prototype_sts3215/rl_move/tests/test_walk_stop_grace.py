"""Unit tests for reward.walk_stop_grace_s (2026-08-24, joyfullcurr7
dig-in).

`cw-arch-hist16-dep1-c1-joyfullcurr7` (k_walk_stop_charge=1.0, no
grace) measured two things at once: (1) the charge DID cut the
trained b1 stop-tick creep from ~0.045 to ~0.026-0.031 m/s but
PLATEAUED there, short of the 0.015 cert bar; (2) EVERY fall in the
held-out randomized joygate session was `over_current` (an actuator
safety trip), not roll/tilt -- the per-tick charge prices the
UNAVOIDABLE physical deceleration transient right after a stop
command exactly as harshly as sustained creep, and the policy was
braking hard enough to spike current.

`reward.walk_stop_grace_s` ramps the charge's multiplier linearly
from 0.0 at the instant a stop segment begins to 1.0 at
`walk_stop_grace_s` seconds in (then holds at 1.0), tracked by the
new `_walk_stop_cmd_s` per-episode timer (seconds since s_ref last
exceeded 1e-3). Default 0.0 = off, bit-exact (multiplier stays
exactly 1.0, identical to the pre-grace formula).

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

WALK_PLANT = (20.0, 80.0)

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
    ("reward", "k_walk_stop_charge"): 1.0,
}


def _walk_env(extra=None, episode_seconds=6.0):
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
    gait.sync_plant_stance(*WALK_PLANT)
    return q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)


def _command_stop(env):
    """Flip the active goal trajectory to a commanded stop (s_ref ~
    0) starting THIS tick, mirroring test_task_semantics.py's own
    all-stop twin construction."""
    traj = env._goal_traj
    traj.vx[:] = 0.0
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0


def test_default_off_is_bit_exact():
    """grace_s absent (default 0.0) must reproduce the pre-grace
    formula exactly: multiplier pinned at 1.0 every stop tick, so the
    charged reward matches a rollout that never touches the new key."""
    for extra in (None, {("reward", "walk_stop_grace_s"): 0.0}):
        env = _walk_env(extra=extra)
        env.reset(seed=0)
        _command_stop(env)
        total = 0.0
        mults = []
        for step in range(env.episode_steps - 1):
            act = _creep_action(step * env.dt)
            _o, r, term, trunc, info = env.step(act)
            total += float(r)
            if "walk_stop_grace_mult" in info:
                mults.append(info["walk_stop_grace_mult"])
            if term or trunc:
                break
        env.close()
        assert mults, "stop charge never armed -- test is vacuous"
        assert all(m == 1.0 for m in mults), (
            f"grace_s=0 must never discount the charge: {mults}")


def test_grace_ramps_zero_to_one_then_holds():
    grace_s = 0.5
    env = _walk_env(extra={("reward", "walk_stop_grace_s"): grace_s})
    env.reset(seed=0)
    _command_stop(env)
    mults, cmd_s = [], []
    for step in range(env.episode_steps - 1):
        act = _creep_action(step * env.dt)
        _o, _r, term, trunc, info = env.step(act)
        mults.append(info["walk_stop_grace_mult"])
        cmd_s.append(env._walk_stop_cmd_s)
        if term or trunc:
            break
    env.close()
    # First tick: the stop segment has been active for one dt (reset
    # zeroed the timer, the charge site increments it before pricing),
    # so the multiplier must be small, not 1.0 -- the whole point.
    assert mults[0] < 0.3, f"first-tick multiplier not discounted: {mults[0]}"
    # Monotonically non-decreasing while ramping.
    ramp = mults[:int(grace_s / env.dt) + 2]
    assert all(b >= a - 1e-9 for a, b in zip(ramp, ramp[1:])), ramp
    # Fully saturated well past the grace window.
    tail = mults[int((grace_s + 1.0) / env.dt):]
    assert tail and all(m == 1.0 for m in tail), (
        f"multiplier never saturates to 1.0 past the grace window: {tail[:5]}")


def test_sustained_creep_charged_identically_with_or_without_grace():
    """Grace must not let a SUSTAINED stop-tick creep escape pricing
    -- only the transient. Compare the per-tick charge once both
    rollouts are well past the grace window: must match exactly."""
    grace_s = 0.3
    settle_ticks = int((grace_s + 1.0) / 0.04)  # dt~0.04 @ 25Hz
    for extra, tag in (
        ({("reward", "walk_stop_grace_s"): grace_s}, "grace"),
        ({("reward", "walk_stop_grace_s"): 0.0}, "plain"),
    ):
        env = _walk_env(extra=extra, episode_seconds=8.0)
        env.reset(seed=0)
        _command_stop(env)
        recorded = None
        for step in range(env.episode_steps - 1):
            act = _creep_action(step * env.dt)
            _o, _r, term, trunc, info = env.step(act)
            if step == settle_ticks:
                recorded = (info["reward_walk_stop"],
                            info["walk_stop_speed_m_s"])
            if term or trunc:
                break
        env.close()
        if tag == "grace":
            grace_pair = recorded
        else:
            plain_pair = recorded
    assert grace_pair is not None and plain_pair is not None
    assert grace_pair[1] == pytest.approx(plain_pair[1], abs=1e-6), (
        "test setup drifted -- speeds should match at the same tick")
    assert grace_pair[0] == pytest.approx(plain_pair[0], abs=1e-9), (
        f"sustained creep charged differently with grace on: "
        f"{grace_pair} vs {plain_pair}")


def test_walking_resets_the_timer():
    """A command that resumes translation (s_ref > 1e-3) must reset
    _walk_stop_cmd_s to 0 so a NEW stop segment gets the full grace
    again, never inheriting elapsed time from an earlier one."""
    env = _walk_env(extra={("reward", "walk_stop_grace_s"): 0.5})
    env.reset(seed=0)
    _command_stop(env)
    for step in range(30):
        env.step(_creep_action(step * env.dt))
    assert env._walk_stop_cmd_s > 0.0
    # Resume a real walk command.
    traj = env._goal_traj
    traj.vx[:] = 0.06
    env.step(_creep_action(0.0))
    assert env._walk_stop_cmd_s == 0.0
    env.close()
