"""Unit tests for safety.walk_idle_terminate_s (2026-08-24, walkcurr
park_duty-class closure dig-in).

Every anti-park PRICE tried on the walkcurr rung-1 diet (idle charge,
park_duty up to bank-legal 1.5x) left a clean, non-colliding static
stand as PPO's cheapest optimum for the whole episode -- an absorbing
state the stagea-slip1 lesson says a soft price alone cannot evict
("absorbing states beat prices; must come WITH a termination, never
instead of one"). This is that termination boundary for the
STANDING-STILL absorbing state:

  - safety.walk_idle_terminate_s: consecutive seconds the along-command
    speed EMA (reward.k_walk_idle_charge's own _walk_idle_ema) may
    stay below safety.walk_idle_terminate_speed_m_s before the episode
    ends (reward.term_penalty applies, same as any other termination).
  - safety.walk_idle_terminate_grace_s: initial window exempt from the
    check (settle time).
  - Default walk_idle_terminate_s=0.0 = OFF, bit-exact legacy: no new
    state read that changes behavior, no new info key emitted, no
    reward change.

Fast (~seconds): short episodes, scripted zero action, no PPO.
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

WALK_PLANT = (20.0, 80.0)

FC_GOAL = {
    ("goal", "walk_yaw_cmd"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.05,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_heading_max_rad"): 0.0,
    ("goal", "walk_stop_frac"): 0.0,
    ("goal", "walk_cmd_resample_s"): 0.0,
    ("goal", "walk_cmd_resample_jitter"): 0.0,
    ("goal", "walk_cmd_blend_s_min"): 0.0,
    ("goal", "walk_cmd_blend_s_max"): 0.0,
    ("goal", "walk_yaw_zero_frac"): 1.0,
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


def _hold_action(env):
    return np.zeros(env.action_space.shape, dtype=np.float32)


def _park_action():
    """A genuinely STATIC settled-plant-stance action (unlike raw
    action==0, which the actbias1 dig-in found maps to the hardware
    axis MIDRANGE, not the settled stand -- holding raw zeros is a
    dynamic collapse-in-progress, not a resting statue, and its qvel
    never actually settles near 0 within a short test episode)."""
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    return q_rad_to_action(plant_rad)


def test_default_off_never_terminates_on_a_static_stand():
    env = _walk_env(episode_seconds=6.0)
    env.reset()
    for step in range(env.episode_steps - 1):
        _o, _r, term, trunc, info = env.step(_hold_action(env))
        assert not term, (step, info.get("termination_reason"))
    env.close()


def test_armed_terminates_a_sustained_static_stand():
    env = _walk_env(extra={
        ("safety", "walk_idle_terminate_s"): 0.5,
        ("safety", "walk_idle_terminate_grace_s"): 0.5,
        ("safety", "walk_idle_terminate_qvel_deg_s"): 2.0,
        ("reward", "term_penalty"): 7.0,
    }, episode_seconds=6.0)
    env.reset()
    act = _park_action()
    fired = False
    for step in range(env.episode_steps - 1):
        _o, r, term, trunc, info = env.step(act)
        if term:
            fired = True
            assert info["termination_reason"] == "walk_idle_terminate"
            break
        assert not trunc
    assert fired, "idle termination never fired on a held plant stance"
    env.close()


def test_grace_window_is_respected():
    # grace covers the whole episode -> never fires even armed.
    env = _walk_env(extra={
        ("safety", "walk_idle_terminate_s"): 0.2,
        ("safety", "walk_idle_terminate_grace_s"): 100.0,
        ("safety", "walk_idle_terminate_qvel_deg_s"): 2.0,
    }, episode_seconds=4.0)
    env.reset()
    act = _park_action()
    for step in range(env.episode_steps - 1):
        _o, _r, term, trunc, info = env.step(act)
        assert not term, (step, info.get("termination_reason"))
    env.close()


def test_low_s_counter_resets_across_episodes():
    env = _walk_env(extra={
        ("safety", "walk_idle_terminate_s"): 10.0,  # long enough not
        ("safety", "walk_idle_terminate_grace_s"): 0.0,   # to fire in
        ("safety", "walk_idle_terminate_qvel_deg_s"): 2.0,  # 1 episode
    }, episode_seconds=2.0)
    env.reset()
    act = _park_action()
    for _ in range(env.episode_steps - 1):
        env.step(act)
    assert env._walk_idle_low_s > 0.0
    env.reset()
    assert env._walk_idle_low_s == 0.0
    env.close()


def test_off_by_default_matches_zero_key_absent():
    # walk_idle_terminate_s absent from cfg entirely (not just 0.0)
    # must be identical to the explicit-0.0 legacy path.
    a = _walk_env(episode_seconds=3.0)
    b = _walk_env(extra={("safety", "walk_idle_terminate_s"): 0.0},
                  episode_seconds=3.0)
    a.reset(seed=0)
    b.reset(seed=0)
    for _ in range(a.episode_steps - 1):
        oa = a.step(_hold_action(a))
        ob = b.step(_hold_action(b))
        assert oa[1] == pytest.approx(ob[1])
        assert oa[2] == ob[2] == False
    a.close()
    b.close()
