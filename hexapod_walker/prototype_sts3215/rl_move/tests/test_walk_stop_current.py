"""Unit tests for reward.k_walk_stop_current (2026-08-24, joyfullcurr8
dig-in).

Two consecutive stop-speed-charge variants failed the same way:
`cw-arch-hist16-dep1-c1-joyfullcurr7` (k_walk_stop_charge=1.0, no
grace) and `...-joyfullcurr8-stopgrace` (+0.4 s grace ramp) both ended
with 100% of their held-out joygate falls terminating `over_current`
-- the grace ramp measurably improved slip and dir_err but moved
over_current not at all. Root cause: the SafetyLayer trips on servo
current > 2.5 A SUSTAINED for 0.8 s through a ~0.1 s low-pass, i.e. a
sustained isometric fight (stiff position targets pressing against
contact) held through the stop stance -- not a braking transient, so
re-timing a SPEED-based charge cannot touch it by construction.

`reward.k_walk_stop_current` prices the fight directly: on stop ticks
only (same s_ref <= 1e-3 / turn-in-place scoping as the speed charge)
r -= k * grace_mult * min(sum(max(|I_servo| - thr, 0)^2), cap), with
thr = reward.walk_stop_current_a (default 1.5 A, 1.0 A of headroom
under the trip) and the SAME walk_stop_grace_s timer as the speed
charge. A settled deadband stance draws ~0 A, so relaxed stillness
pays nothing. Default 0 = off, bit-exact.

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

# The exact joyfullcurr reward stack (mirrors the stopcharge bank in
# test_task_semantics.py) -- the launch-rule proof must run under the
# recipe the relaunch trains.
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
    ("reward", "k_walk_stop_charge"): 1.0,
    ("reward", "walk_stop_grace_s"): 0.4,
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


def _fight_action(femur_off_deg, knee_off_deg):
    q = PLANT_RAD.copy()
    for leg in range(6):
        q[leg * 3 + 1] += femur_off_deg * DEG2RAD
        q[leg * 3 + 2] += knee_off_deg * DEG2RAD
    return q_rad_to_action(q)


def _command_walk(env, vx=0.06):
    """Force a translation command on EVERY tick (the env's own drawn
    schedule can open with a stop head even at walk_stop_frac=0)."""
    traj = env._goal_traj
    traj.vx[:] = vx
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0


def _rollout(action_fn, extra=None, episode_seconds=6.0, stop=True):
    """Returns (total_reward, per-tick list of (charge, max_a) info)."""
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
        ticks.append((info.get("reward_walk_stop_current"),
                      info.get("walk_stop_current_max_a")))
        if term or trunc:
            break
    env.close()
    return total, ticks


@pytest.fixture(scope="module")
def fight_offsets():
    """Find a scripted pose offset that reliably draws sustained
    per-servo current above the 1.5 A threshold while parked on stop
    ticks (pressing femur/knee targets past the settled plant pose
    into ground contact). Probed, not assumed, so the test never
    silently goes vacuous if sign conventions change."""
    # (0, -40) measured 08-24: sustains 2.64 A median on the tail and
    # actually trips the SafetyLayer over_current termination ~2.9 s in
    # -- the bank's fight rollout reproduces the exact joygate failure
    # mode. Other probed offsets settle after a transient peak.
    candidates = [(0.0, -40.0), (0.0, -45.0), (0.0, -35.0),
                  (-20.0, -40.0)]
    extra = {("reward", "k_walk_stop_current"): 2.0}
    for fo, ko in candidates:
        _tot, ticks = _rollout(lambda t: _fight_action(fo, ko),
                               extra=extra, episode_seconds=3.0)
        cur = [m for _c, m in ticks if m is not None]
        # Sustained: the median of the second half must clear the
        # threshold, not just a transient peak.
        tail = cur[len(cur) // 2:]
        if tail and float(np.median(tail)) > 2.0:
            return fo, ko
    pytest.fail("no probed pose offset draws sustained current > 2.0 A "
                "-- fight construction is broken, bank is vacuous")


def test_default_off_is_bit_exact():
    """Key absent and key=0.0 must both charge nothing, emit no new
    info keys, and produce identical returns."""
    totals = []
    for extra in (None, {("reward", "k_walk_stop_current"): 0.0}):
        tot, ticks = _rollout(lambda t: _fight_action(0.0, 25.0),
                              extra=extra, episode_seconds=3.0)
        totals.append(tot)
        assert all(c is None and m is None for c, m in ticks), (
            "stop-current info keys emitted while the charge is off")
    assert totals[0] == pytest.approx(totals[1], abs=1e-12), totals


def test_fight_pays_relaxed_still_free(fight_offsets):
    """The sustained isometric fight must be charged hard; the relaxed
    plant stance (deadband -> ~0 A) must pay ~nothing."""
    fo, ko = fight_offsets
    extra = {("reward", "k_walk_stop_current"): 2.0}
    _tot_f, ticks_f = _rollout(lambda t: _fight_action(fo, ko),
                               extra=extra)
    _tot_r, ticks_r = _rollout(lambda t: q_rad_to_action(PLANT_RAD),
                               extra=extra)
    charge_f = sum(c for c, _m in ticks_f if c is not None)
    charge_r = sum(c for c, _m in ticks_r if c is not None)
    cur_r = [m for _c, m in ticks_r if m is not None]
    assert cur_r, "charge never armed on relaxed rollout -- vacuous"
    # Relaxed stance stays under the threshold, so it pays ~zero.
    assert charge_r > -1.0, (
        f"relaxed plant stance is being taxed: {charge_r}")
    # The fight pays, a lot.
    assert charge_f < -50.0, (
        f"sustained fight barely charged: {charge_f}")
    assert charge_f < charge_r - 50.0


def test_commanded_motion_exempt(fight_offsets):
    """On commanded-walk ticks (s_ref > 1e-3) the charge must never
    fire, whatever the current draw -- motion needs torque."""
    fo, ko = fight_offsets
    extra = {("reward", "k_walk_stop_current"): 2.0}
    _tot, ticks = _rollout(lambda t: _fight_action(fo, ko),
                           extra=extra, episode_seconds=3.0, stop=False)
    assert all(c is None for c, _m in ticks), (
        "stop-current charge fired on commanded-motion ticks")


def test_grace_discounts_exactly(fight_offsets):
    """With walk_stop_grace_s on, the per-tick charge must equal
    grace_mult * the ungraced charge (deterministic env, identical
    scripted actions -> identical currents tick-for-tick)."""
    fo, ko = fight_offsets
    grace_s = 0.5
    base = {("reward", "k_walk_stop_current"): 2.0}
    _t0, ticks_plain = _rollout(lambda t: _fight_action(fo, ko),
                                extra=base, episode_seconds=3.0)
    _t1, ticks_grace = _rollout(
        lambda t: _fight_action(fo, ko),
        extra={**base, ("reward", "walk_stop_grace_s"): grace_s},
        episode_seconds=3.0)
    env_dt = 0.04  # 25 Hz control
    checked = 0
    for k, ((cp, _mp), (cg, _mg)) in enumerate(
            zip(ticks_plain, ticks_grace)):
        if cp is None or cg is None:
            continue
        gm = min((k + 1) * env_dt / grace_s, 1.0)
        assert cg == pytest.approx(gm * cp, abs=1e-9), (
            f"tick {k}: grace charge {cg} != {gm} * plain {cp}")
        checked += 1
    assert checked > 30, "too few charged ticks compared -- vacuous"
    # And the discount is real early on.
    first = next((k for k, (c, _m) in enumerate(ticks_grace)
                  if c is not None), None)
    assert first is not None and first * env_dt < grace_s


def test_full_stack_relaxed_stillness_is_optimum(fight_offsets):
    """Launch-rule proof under the exact joyfullcurr9 recipe (full
    reward stack + speed charge + grace + current charge): on an
    all-stop schedule, RELAXED stillness must out-earn the stiff
    fight, the 0.04 m/s creep cheat, and walking through the stop."""
    fo, ko = fight_offsets
    stack = {**JOYFULLCURR_STACK,
             ("reward", "k_walk_stop_current"): 2.0}

    def gait_action(speed):
        g = TripodGait(vx=speed, lift=0.025)
        g.sync_plant_stance(*WALK_PLANT)
        return lambda t: q_rad_to_action(
            np.asarray(g.desired_deg(t)) * DEG2RAD)

    tot_relaxed, _ = _rollout(lambda t: q_rad_to_action(PLANT_RAD),
                              extra=stack)
    tot_fight, _ = _rollout(lambda t: _fight_action(fo, ko), extra=stack)
    tot_creep, _ = _rollout(gait_action(0.04), extra=stack)
    tot_walk, _ = _rollout(gait_action(0.06), extra=stack)
    assert tot_relaxed > tot_fight + 10.0, (tot_relaxed, tot_fight)
    assert tot_relaxed > tot_creep + 10.0, (tot_relaxed, tot_creep)
    assert tot_relaxed > tot_walk + 10.0, (tot_relaxed, tot_walk)
