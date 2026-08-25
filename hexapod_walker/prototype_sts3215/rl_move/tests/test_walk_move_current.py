"""Unit tests for reward.k_walk_move_current (2026-08-25,
gaitgate-scratch1/tf64-mesh-acq1 dig-in).

The mesh model family (as-built 3.50 kg vs the legacy 2.104 kg
calibration) makes a rigid PARTIAL-TRIPOD HOLD (3 legs locked planted,
3 held aloft) the cheapest way to satisfy walk income under the
V7/100Hz joyfullcurr13 reward stack -- both the from-scratch MLP
(`cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-acq1`) and the
transformer (`cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-acq1`) die the
SAME way: `over_current` (servo current > 2.5 A sustained 0.8 s
through a 0.1 s low-pass), never `tilt_pitch`, with duty_cycle pinned
near 1.0 on the locked legs and near 0 on the sacrificed ones.
`reward.walk_gait_gate` (the min-across-legs step-completion gate, the
other tried anti-sacrifice lever) does NOT fix this: baking it in from
step 0 (`cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1`) made
the held-out fall rate WORSE than the ungated parent (39/48 vs
12/48) -- it is gameable via rare token swings that satisfy a rolling
window while the policy spends most ticks in the same rigid lock.

`reward.k_walk_move_current` is the WALKING-tick mirror of the
already-proven `reward.k_walk_stop_current` (test_walk_stop_current.py):
on commanded-TRANSLATING ticks only (s_ref > 1e-3), r -= k *
min(sum(max(|I_servo| - thr, 0)^2), cap), with thr =
reward.walk_move_current_a (default 2.2 A -- ABOVE the stop charge's
1.5 A, since honest cycling legitimately draws brief per-leg stance
current the stop charge's threshold would over-price; still 0.3 A of
headroom under the 2.5 A trip). A per-tick quadratic-over-threshold
charge already prices DURATION correctly with no extra timer state: an
episode-long lock pays every tick, a brief honest stance-loading spike
pays for only as many ticks as it lasts. Default 0 = off, bit-exact.

Fast (~tens of seconds): short episodes, scripted actions, no PPO.
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
PLANT_RAD = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD

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
}

JOYFULLCURR_STACK = {
    ("reward", "k_step_event"): 1.0,
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("reward", "k_loadslip_excess"): 0.8,
    ("reward", "loadslip_ok"): 1.2,
    ("reward", "loadslip_max"): 3.0,
    ("reward", "loadslip_floor_m"): 0.03,
    ("reward", "k_walk_course"): 1.0,
    ("reward", "walk_course_tau_s"): 0.75,
    ("reward", "walk_course_min_speed_m_s"): 0.01,
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


def _command_stop(env):
    traj = env._goal_traj
    traj.vx[:] = 0.0
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0


def _command_walk(env, vx=0.06):
    traj = env._goal_traj
    traj.vx[:] = vx
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0


def _fight_action(femur_off_deg, knee_off_deg):
    q = PLANT_RAD.copy()
    for leg in range(6):
        q[leg * 3 + 1] += femur_off_deg * DEG2RAD
        q[leg * 3 + 2] += knee_off_deg * DEG2RAD
    return q_rad_to_action(q)


def _rollout(action_fn, extra=None, episode_seconds=6.0, stop=False):
    """Returns (total_reward, per-tick list of (charge, max_a) info).
    Default stop=False: these tests are about the WALK-tick charge, so
    commanded translation is the default scope (mirrors the stop-
    current bank's default stop=True)."""
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
        ticks.append((info.get("reward_walk_move_current"),
                      info.get("walk_move_current_max_a")))
        if term or trunc:
            break
    env.close()
    return total, ticks


@pytest.fixture(scope="module")
def fight_offsets():
    """Same probe as test_walk_stop_current.py's fixture (the physical
    action fully determines current draw regardless of the commanded
    goal), re-checked under a WALK command so the fixture never goes
    vacuous if the walk-tick code path scopes current differently."""
    candidates = [(0.0, -40.0), (0.0, -45.0), (0.0, -35.0),
                  (-20.0, -40.0)]
    extra = {("reward", "k_walk_move_current"): 2.0}
    for fo, ko in candidates:
        _tot, ticks = _rollout(lambda t: _fight_action(fo, ko),
                               extra=extra, episode_seconds=3.0)
        cur = [m for _c, m in ticks if m is not None]
        tail = cur[len(cur) // 2:]
        if tail and float(np.median(tail)) > 2.0:
            return fo, ko
    pytest.fail("no probed pose offset draws sustained current > 2.0 A "
                "under a walk command -- fight construction is broken, "
                "bank is vacuous")


def test_default_off_is_bit_exact():
    totals = []
    for extra in (None, {("reward", "k_walk_move_current"): 0.0}):
        tot, ticks = _rollout(lambda t: _fight_action(0.0, 25.0),
                              extra=extra, episode_seconds=3.0)
        totals.append(tot)
        assert all(c is None and m is None for c, m in ticks), (
            "walk-move-current info keys emitted while the charge is off")
    assert totals[0] == pytest.approx(totals[1], abs=1e-12), totals


def test_locked_leg_fight_pays_hard_on_walk_ticks(fight_offsets):
    """The sustained rigid-lock fight (the leg-sacrifice exploit's
    physical signature: a subset of legs held at a fixed offset,
    drawing high sustained current) must be charged hard on commanded-
    walk ticks."""
    fo, ko = fight_offsets
    extra = {("reward", "k_walk_move_current"): 2.0}
    _tot_f, ticks_f = _rollout(lambda t: _fight_action(fo, ko),
                               extra=extra)
    charge_f = sum(c for c, _m in ticks_f if c is not None)
    cur_f = [m for _c, m in ticks_f if m is not None]
    assert cur_f, "charge never armed on the fight rollout -- vacuous"
    assert charge_f < -50.0, (
        f"sustained locked-leg fight barely charged: {charge_f}")


def test_honest_cycling_gait_pays_far_less_than_the_lock(fight_offsets):
    """Root claim of the whole mechanism: an honest six-leg cycling
    gait at normal speed must pay MUCH less than the sustained lock,
    even though the gait also legitimately touches high current in
    brief per-leg stance spikes (the scripted teacher itself peaks at
    2.627 A, per the mesh dig-in) -- duration, not peak, must decide
    the charge."""
    fo, ko = fight_offsets
    extra = {("reward", "k_walk_move_current"): 2.0}

    def gait_action(speed):
        g = TripodGait(vx=speed, lift=0.025)
        g.sync_plant_stance(*WALK_PLANT)
        return lambda t: q_rad_to_action(
            np.asarray(g.desired_deg(t)) * DEG2RAD)

    _tot_f, ticks_f = _rollout(lambda t: _fight_action(fo, ko),
                               extra=extra)
    _tot_g, ticks_g = _rollout(gait_action(0.06), extra=extra)
    charge_f = sum(c for c, _m in ticks_f if c is not None)
    charge_g = sum(c for c, _m in ticks_g if c is not None)
    assert charge_g > charge_f + 40.0, (
        f"honest gait ({charge_g}) not clearly cheaper than the locked "
        f"fight ({charge_f})")
    # And the honest gait must be cheap in absolute terms too (near
    # the floor), not just relatively better than the fight.
    assert charge_g > -15.0, (
        f"honest cycling gait taxed more than a brief-spike floor: "
        f"{charge_g}")


def test_stop_ticks_exempt(fight_offsets):
    """On commanded-STOP ticks (s_ref <= 1e-3) the walk-move charge
    must never fire, whatever the current draw -- that scope belongs
    to reward.k_walk_stop_current instead."""
    fo, ko = fight_offsets
    extra = {("reward", "k_walk_move_current"): 2.0}
    _tot, ticks = _rollout(lambda t: _fight_action(fo, ko),
                           extra=extra, episode_seconds=3.0, stop=True)
    assert all(c is None for c, _m in ticks), (
        "walk-move-current charge fired on a commanded-stop tick")


def test_full_stack_honest_gait_beats_the_lock_and_a_stall(fight_offsets):
    """Launch-rule proof under the exact joyfullcurr13 walk stack (no
    walk_gait_gate -- this is testing k_walk_move_current alone): under
    a commanded-walk schedule, honest cycling at the commanded speed
    must out-earn both the rigid locked-leg fight and a stationary
    stall (legs planted, not cycling, zero progress)."""
    fo, ko = fight_offsets
    stack = {**JOYFULLCURR_STACK, ("reward", "k_walk_move_current"): 2.0}

    def gait_action(speed):
        g = TripodGait(vx=speed, lift=0.025)
        g.sync_plant_stance(*WALK_PLANT)
        return lambda t: q_rad_to_action(
            np.asarray(g.desired_deg(t)) * DEG2RAD)

    tot_walk, _ = _rollout(gait_action(0.06), extra=stack)
    tot_fight, _ = _rollout(lambda t: _fight_action(fo, ko), extra=stack)
    tot_stall, _ = _rollout(lambda t: q_rad_to_action(PLANT_RAD),
                            extra=stack)
    assert tot_walk > tot_fight + 10.0, (tot_walk, tot_fight)
    assert tot_walk > tot_stall + 10.0, (tot_walk, tot_stall)
