"""Unit tests for reward.k_tau_over — the per-motor torque hinge
(operator amendment 2026-08-25 to the 05:09 effort-priced walk arm,
from the local load-probe session).

Probe truth: honest plant-height stepping needs at most ~1 N*m per hip
(0.23 N*m static on six feet, ~0.5 in tripod, ~1 at dynamic peaks)
while the skater-crouch/locked-leg fight runs 1.1-1.5 N*m STATIC and
rails the 2.2 N*m actuator clamp. Motor heat is I^2*R, so sustained
above-threshold torque matters superlinearly and the mean-current
k_walk_effort underprices it; before this term nothing existed between
free and the over_current episode trip. The hinge prices ONLY the
waste and the rail events, never normal stepping:

    r -= k_tau_over * mean(relu(|qfrc_actuator| - tau_over_nm))

per walk tick, walk-routed exactly like k_walk_effort (ALL walk-mode
ticks, including commanded-stop ticks: a static crouch is the same
thermal waste whatever the command). tau_over_nm default 1.0 N*m.
Default k 0 = off, block skipped, no info keys, legacy bit-exact
(drag_stance pattern).

Measured on the mesh model (this bank's own probe, k=1, 3 s / 100 Hz):
honest tripod gait charge -0.24 (tau_max med 0.817, p95 1.08); static
six-foot stance charge 0.00 (tau med 0.234); knee-offset fight charge
-43.1 (tau railed at the 2.2 clamp every tick).

Fast (~tens of seconds): short episodes, scripted actions, no PPO.
Reuses the walk-move-current bank's env/action helpers.
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

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from sim_gait_compat import TripodGait  # noqa: E402

from test_walk_move_current import (  # noqa: E402
    JOYFULLCURR_STACK, PLANT_RAD, WALK_PLANT, _command_stop,
    _command_walk, _fight_action, _walk_env)

K_RUN = 2.0  # the launch rung under test


def _rollout(action_fn, extra=None, episode_seconds=3.0, stop=False):
    """Returns (total_reward, per-tick list of (charge, tau_max))."""
    env = _walk_env(extra=extra, episode_seconds=episode_seconds)
    env.reset(seed=0)
    if stop:
        _command_stop(env)
    else:
        _command_walk(env)
    total, ticks = 0.0, []
    for step in range(env.episode_steps - 1):
        _o, r, term, trunc, info = env.step(action_fn(step * env.dt))
        total += float(r)
        ticks.append((info.get("reward_tau_over"),
                      info.get("walk_tau_max_nm")))
        if term or trunc:
            break
    env.close()
    return total, ticks


def _gait_action(speed=0.06):
    g = TripodGait(vx=speed, lift=0.025)
    g.sync_plant_stance(*WALK_PLANT)
    return lambda t: q_rad_to_action(
        np.asarray(g.desired_deg(t)) * DEG2RAD)


FIGHT = (0.0, -40.0)  # knee offset that rails the 2.2 N*m clamp


def test_default_off_is_bit_exact():
    totals = []
    for extra in (None, {("reward", "k_tau_over"): 0.0}):
        tot, ticks = _rollout(lambda t: _fight_action(*FIGHT),
                              extra=extra)
        totals.append(tot)
        assert all(c is None and m is None for c, m in ticks), (
            "tau-over info keys emitted while the hinge is off")
    assert totals[0] == pytest.approx(totals[1], abs=1e-12), totals


def test_railed_fight_pays_hard():
    """The locked-leg fight (the skater-crouch signature: torque railed
    at the 2.2 N*m clamp) must accumulate a large charge."""
    extra = {("reward", "k_tau_over"): K_RUN}
    _tot, ticks = _rollout(lambda t: _fight_action(*FIGHT), extra=extra)
    taus = [m for _c, m in ticks if m is not None]
    assert taus, "hinge never armed on the fight rollout -- vacuous"
    assert float(np.median(taus)) > 1.5, (
        f"fight construction broke: median tau {np.median(taus)}")
    charge = sum(c for c, _m in ticks if c is not None)
    assert charge < -50.0, (
        f"railed fight barely charged at k={K_RUN}: {charge}")


def test_honest_gait_and_static_stance_near_free():
    """The hinge must price ONLY the waste: honest cycling (brief ~1
    N*m dynamic hip peaks) and the six-foot static stance (0.23 N*m)
    pay ~nothing."""
    extra = {("reward", "k_tau_over"): K_RUN}
    _tot, ticks_g = _rollout(_gait_action(), extra=extra)
    charge_g = sum(c for c, _m in ticks_g if c is not None)
    assert charge_g > -3.0, (
        f"honest cycling gait taxed beyond brief-peak floor: {charge_g}")
    _tot, ticks_s = _rollout(lambda t: q_rad_to_action(PLANT_RAD),
                             extra=extra)
    charge_s = sum(c for c, _m in ticks_s if c is not None)
    assert charge_s == pytest.approx(0.0, abs=1e-9), (
        f"static plant-height stance charged: {charge_s}")
    _tot, ticks_f = _rollout(lambda t: _fight_action(*FIGHT), extra=extra)
    charge_f = sum(c for c, _m in ticks_f if c is not None)
    assert charge_g > charge_f + 40.0, (charge_g, charge_f)


def test_fires_on_stop_ticks_too():
    """Routing is k_walk_effort-like (ALL walk-mode ticks), NOT
    s_ref-scoped like k_walk_move_current: a static crouch is the same
    thermal waste whatever the command."""
    extra = {("reward", "k_tau_over"): K_RUN}
    _tot, ticks = _rollout(lambda t: _fight_action(*FIGHT),
                           extra=extra, stop=True)
    charge = sum(c for c, _m in ticks if c is not None)
    assert charge < -50.0, (
        f"railed fight unpriced on commanded-stop walk ticks: {charge}")


def test_full_stack_honest_gait_beats_the_lock_and_a_stall():
    """Launch-rule proof under the exact movecur1+hinge walk stack:
    honest cycling at the commanded speed must out-earn both the
    rigid locked-leg fight and a stationary stall."""
    stack = {**JOYFULLCURR_STACK,
             ("reward", "k_walk_move_current"): 2.0,
             ("reward", "k_tau_over"): K_RUN}
    tot_walk, _ = _rollout(_gait_action(), extra=stack,
                           episode_seconds=6.0)
    tot_fight, _ = _rollout(lambda t: _fight_action(*FIGHT),
                            extra=stack, episode_seconds=6.0)
    tot_stall, _ = _rollout(lambda t: q_rad_to_action(PLANT_RAD),
                            extra=stack, episode_seconds=6.0)
    assert tot_walk > tot_fight + 10.0, (tot_walk, tot_fight)
    assert tot_walk > tot_stall + 10.0, (tot_walk, tot_stall)
