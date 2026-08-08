"""Sim-twin sanity tests (no hardware; needs mujoco in the venv)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

mujoco = pytest.importorskip("mujoco")

from rl_move.sim.domain_rand import (  # noqa: E402
    DomainRandomizer, RandRanges,
)
from rl_move.sim.servo_model import (  # noqa: E402
    N_JOINTS, ServoProfile, SimServoParams,
)
from rl_move.sim.sim_env import N_ACT, N_OBS, SimHexapodBalanceEnv  # noqa: E402

DEG = math.pi / 180.0


def test_servo_profile_latency_and_slew():
    params = SimServoParams.defaults()
    ax = params.axes["hip"]
    prof = ServoProfile(params, np.zeros(N_JOINTS))
    goal = np.zeros(N_JOINTS)
    goal[1] = 10 * DEG  # a hip joint
    prof.command(goal, speed_deg_s=30.0)

    dt = 0.002
    # During the latency window nothing moves.
    steps_lat = int(ax.latency_ms / 1000.0 / dt) - 1
    for _ in range(steps_lat):
        t = prof.tick(dt)
    assert abs(t[1]) < 1e-9

    # After latency the target ramps up (trapezoid, ACC=15 ≈ 132 °/s²) —
    # ~0.66° in the first 0.1 s — instead of snapping.
    for _ in range(50):  # 0.1 s
        t = prof.tick(dt)
    moved = t[1] / DEG
    assert 0.2 < moved < 2.5, f"profile moved {moved:.2f}° in 0.1 s"

    # Eventually reaches the goal (within deadband).
    for _ in range(1000):
        t = prof.tick(dt)
    assert abs(t[1] - 10 * DEG) <= ax.deadband_deg * DEG + 1e-6


def test_zero_action_episode_holds_stand():
    env = SimHexapodBalanceEnv(seed=0)
    obs, info = env.reset()
    assert obs.shape == (N_OBS,)
    assert np.all(np.isfinite(obs))
    terminated = truncated = False
    steps = 0
    while not (terminated or truncated):
        obs, r, terminated, truncated, info = env.step(np.zeros(N_ACT))
        assert np.all(np.isfinite(obs))
        steps += 1
    assert truncated and not terminated, \
        f"fell at step {steps}: {info.get('termination_reason')}"
    assert abs(info["roll_deg"]) < 5 and abs(info["pitch_deg"]) < 5


def test_domain_randomization_varies_and_survives():
    env = SimHexapodBalanceEnv(randomize=True, seed=1)
    summaries = []
    for _ in range(3):
        obs, info = env.reset()
        summaries.append(info["randomization"])
        for _ in range(20):
            obs, r, term, trunc, _ = env.step(np.zeros(N_ACT))
            if term or trunc:
                break
        assert np.all(np.isfinite(obs))
    masses = {s["mass_scale"] for s in summaries}
    assert len(masses) > 1, "DR did not vary between episodes"
    tilts = {s["ground_tilt_deg"] for s in summaries}
    assert len(tilts) > 1, "ground slope did not vary"


def test_geometry_randomization_mutates_and_restores():
    env = SimHexapodBalanceEnv(randomize=True, seed=3)
    fem_bid = mujoco.mj_name2id(
        env.model, mujoco.mjtObj.mjOBJ_BODY, "L0_femur")
    nominal_coxa = env._base_body_pos[fem_bid, 0]

    coxa_lengths = []
    for _ in range(3):
        env.reset()
        coxa_lengths.append(float(env.model.body_pos[fem_bid, 0]))
    assert len(set(coxa_lengths)) == 3, "leg geometry did not vary"
    for v in coxa_lengths:
        assert v != nominal_coxa
        assert abs(v / nominal_coxa - 1.0) < 0.05  # within DR range

    # Without DR the model must come back to nominal geometry + gravity.
    env.randomizer = None
    env.reset()
    assert float(env.model.body_pos[fem_bid, 0]) == pytest.approx(
        nominal_coxa)
    assert float(env.model.opt.gravity[2]) == pytest.approx(-9.81)


def test_imu_mount_misalignment_shifts_tilt():
    # Same seed → same physics; mount misalignment must move the measured
    # roll/pitch (that is what the policy has to be robust to). The
    # complementary filter needs a few reads to converge on the accel
    # tilt, so read repeatedly and compare the converged estimates.
    def converged_tilt(mount_rot):
        env = SimHexapodBalanceEnv(randomize=True, seed=4)
        env.reset()
        er = env._ep_rand
        assert er is not None
        if mount_rot is not None:
            er.imu_mount_rot = mount_rot
        env._att_rp = None
        for _ in range(120):
            state = env._read_state()
        return state.imu_roll, state.imu_pitch

    meas = converged_tilt(None)          # sampled mount misalignment
    true = converged_tilt(np.eye(3))     # perfectly aligned
    shift = math.hypot(meas[0] - true[0], meas[1] - true[1])
    assert shift > 0.001, "IMU mount rotation had no effect on tilt"


def test_dr_ranges_widen_from_spread():
    params = SimServoParams.defaults()
    params.spread = {"hip": {"rise_ms_pct": 0.30, "delay_ms_pct": 0.25}}
    dr = DomainRandomizer.from_params(params)
    assert dr.ranges.kp_scale_pct >= 0.44
    assert dr.ranges.latency_scale[1] >= 1.5


def test_reward_includes_effort_penalty():
    env = SimHexapodBalanceEnv(seed=5)
    env.reset()
    _, _, _, _, info = env.step(np.zeros(N_ACT))
    assert "reward_current" in info
    assert info["reward_current"] <= 0.0
    state = env._read_state()
    assert state.servo_current is not None
    assert float(np.max(state.servo_current)) < 2.5, \
        "quiet stand should not be near the over-current trip"


def test_bad_start_adopts_settled_pose_and_does_not_crash():
    ranges = RandRanges()
    ranges.bad_start_prob = 1.0  # force a badly-placed episode every time
    env = SimHexapodBalanceEnv(
        randomizer=DomainRandomizer(ranges), seed=6)
    plant_rad = env._plant_deg * DEG
    saw_bad = False
    for _ in range(4):
        obs, info = env.reset()
        assert np.all(np.isfinite(obs))
        bad = info["randomization"]["bad_start_joints"]
        assert bad, "bad_start_prob=1.0 must flag joints"
        # Hold-current semantics: nominal follows the (badly) settled pose,
        # not the ideal plant.
        offs = np.abs(env._q_nom - plant_rad) / DEG
        if max(offs[j] for j in bad) > 5.0:
            saw_bad = True
        # Episode must run without exceptions; termination is allowed
        # (tilt / over-current on a truly broken stance is the point).
        for _ in range(30):
            obs, r, term, trunc, info = env.step(np.zeros(N_ACT))
            assert np.all(np.isfinite(obs))
            if term or trunc:
                break
    assert saw_bad, "no episode actually started with a way-off joint"


def test_action_moves_body_reward_worsens():
    env = SimHexapodBalanceEnv(seed=2)
    env.reset()
    # Constant max roll command should tilt the body measurably.
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    roll = 0.0
    for _ in range(60):
        _, r, term, trunc, info = env.step(a)
        roll = info["roll_deg"]
        if term or trunc:
            break
    assert abs(roll) > 0.8, f"roll only {roll:.2f}° after sustained command"


def test_imu_position_lever_arm_corrupts_tilt_during_motion():
    # The IMU can be bolted anywhere: static tilt is position-independent,
    # but during a body swing an off-center IMU feels lever-arm
    # acceleration and reads a different tilt. Same seed twice; only the
    # IMU position differs.
    def run(imu_pos):
        ranges = RandRanges()
        ranges.placement_noise_deg = 0.0
        ranges.bad_start_prob = 0.0
        ranges.ground_tilt_deg = 0.0
        env = SimHexapodBalanceEnv(
            randomizer=DomainRandomizer(ranges), seed=11)
        env.reset()
        env._ep_rand.imu_pos_m = np.asarray(imu_pos, dtype=float)
        env._ep_rand.imu_mount_rot = np.eye(3)   # isolate position
        env._imu_prev_v = None
        env._att_rp = None
        # Re-anchor the episode tilt reference to the overridden IMU —
        # otherwise the safety layer sees the removed mount bias as a
        # giant instantaneous "tilt change" and trips.
        st = env._read_state()
        env._tilt_ref0 = (st.imu_roll, st.imu_pitch)
        env.safety.set_tilt_reference(*env._tilt_ref0)
        rolls = []
        for i in range(50):
            # Gentle 1 Hz body-roll sine — sustained rotation without
            # physically walking the robot over.
            a = np.array([math.sin(2 * math.pi * i / 25),
                          0.0, 0.0, 0.0, 0.0, 0.0])
            _, _, term, trunc, info = env.step(a)
            rolls.append(info["roll_deg"])
            if term or trunc:
                break
        return np.array(rolls)

    r0 = run([0.0, 0.0, 0.0])
    r1 = run([0.07, 0.07, 0.10])
    n = min(len(r0), len(r1))
    assert n > 20, f"episode ended after {n} steps"
    diff = np.abs(r0[:n] - r1[:n])
    assert diff.max() > 0.03, \
        f"IMU position had no effect during motion (max Δ {diff.max():.4f}°)"


def test_goal_env_obs_dim_and_mode_variety():
    from rl_move.env import GOAL_DIM
    from rl_move.sim.goal_task import SimHexapodGoalEnv

    env = SimHexapodGoalEnv(randomize=True, seed=21)
    modes = set()
    for _ in range(8):
        obs, info = env.reset()
        assert obs.shape == (N_OBS + GOAL_DIM,)
        assert np.all(np.isfinite(obs))
        modes.add(info["goal_mode"])
        for _ in range(5):
            obs, r, term, trunc, info = env.step(np.zeros(N_ACT))
            assert np.all(np.isfinite(obs))
            if term or trunc:
                break
    assert len(modes) >= 2, f"goal modes did not vary: {modes}"


def test_goal_reward_tracks_reference_and_unload():
    from rl_move.env import TaskGoal, compute_reward

    env = SimHexapodBalanceEnv(seed=22)
    env.reset()
    st = env._read_state()
    zeros = np.zeros(N_ACT)

    # Zero goal must be exactly the plain balance reward.
    r_none, _ = compute_reward(env.cfg, st, zeros, zeros, goal=None)
    r_zero, p_zero = compute_reward(env.cfg, st, zeros, zeros,
                                    goal=TaskGoal())
    assert r_zero == pytest.approx(r_none)

    # An unmet lean reference must cost more than the met zero reference.
    _, p_lean = compute_reward(env.cfg, st, zeros, zeros,
                               goal=TaskGoal(roll_ref=0.05))
    assert p_lean["reward_roll"] < p_zero["reward_roll"]

    # Unload goal penalizes the loaded target leg's current.
    cur = np.abs(st.servo_current).reshape(6, 3)
    leg = int(cur.mean(axis=1).argmax())
    _, p_unload = compute_reward(env.cfg, st, zeros, zeros,
                                 goal=TaskGoal(unload_leg=leg))
    assert p_unload["reward_unload"] < 0.0


def test_kernel_reward_pays_tracking_not_freezing():
    """The 2026-08-07 redesign: ignoring the goal must earn ~nothing."""
    from rl_move.env import TaskGoal, compute_reward

    env = SimHexapodBalanceEnv(seed=23)
    env.reset()
    st = env._read_state()
    zeros = np.zeros(N_ACT)
    ref = 0.06  # ~3.4° — several kernel sigmas away

    # Frozen at level while commanded to lean: task reward ≈ 0.
    _, p_ignore = compute_reward(env.cfg, st, zeros, zeros,
                                 goal=TaskGoal(roll_ref=ref),
                                 tilt_ref=(st.imu_roll, st.imu_pitch))
    # Tracking the same reference: task reward ≈ k_track.
    _, p_track = compute_reward(env.cfg, st, zeros, zeros,
                                goal=TaskGoal(roll_ref=ref),
                                tilt_ref=(st.imu_roll - ref, st.imu_pitch))
    assert p_track["reward_task"] > 5 * max(p_ignore["reward_task"], 1e-9)
    assert p_ignore["reward_task"] < 0.1
    assert p_track["reward_task"] > 0.8


def test_curl_channel_enables_rise_from_zero_pose():
    """From the zero pose, height-only IK must fail (legs fully extended)
    but curl + height must succeed — that is the whole point of the 6th
    action channel."""
    from rl_move.body_ik import BodyOffset, FixedFootBodyIK

    ik = FixedFootBodyIK()
    q_zero = np.zeros(18)
    q_plant = np.array([0.0, 20.0, 80.0] * 6) * DEG
    ik.reset(q_zero, plant_q_rad=q_plant)

    up = ik.solve(BodyOffset(height=0.05, curl=0.0))
    assert not up.ok, "raising the body with fully extended pinned feet " \
                      "should be unreachable"
    # Curl is a RATE with a ratchet: hold curl=1 across ticks and the
    # anchors walk to the plant footprint (~2.5 s at 25 Hz), after which
    # the same height command is comfortably reachable.
    for _ in range(80):
        up_curl = ik.solve(BodyOffset(height=0.05, curl=1.0))
    assert ik.curl_frac == 1.0, f"ratchet did not saturate: {ik.curl_frac}"
    assert up_curl.ok, f"curl+rise IK failed: {up_curl.reason}"
    knees = up_curl.q_rad.reshape(6, 3)[:, 2] / DEG
    assert np.all(knees > 20.0), f"knees did not bend: {knees}"
    assert np.all(knees < 150.0), f"knees exceed the 150° cap: {knees}"
    # Ratchet: dropping curl back to 0 must NOT slide the anchors out.
    frac = ik.curl_frac
    ik.solve(BodyOffset(height=0.05, curl=0.0))
    assert ik.curl_frac == frac, "curl ratchet slid back"


def test_rise_episode_starts_on_belly():
    from rl_move.sim.goal_task import SimHexapodGoalEnv

    env = SimHexapodGoalEnv(seed=24)
    g = env._goal_gen
    g.p_hold = g.p_lean = g.p_track = g.p_unload = 0.0
    g.p_rise = 1.0
    obs, info = env.reset()
    assert info["goal_mode"] == "rise"
    assert np.all(np.isfinite(obs))
    # Belly rest is ~40 mm; the plant stance is ~160 mm.
    z = float(env.data.xpos[env._chassis_bid, 2])
    assert z < 0.09, f"rise episode started standing (chassis z={z:.3f} m)"
    for _ in range(10):
        obs, r, term, trunc, info = env.step(np.zeros(N_ACT))
        assert np.all(np.isfinite(obs))
        assert not term, info.get("termination_reason")


def test_flag_leg_penalty_walk_only_routing():
    """reward.flag_leg_walk_only=1 must gate the charge to walk mode.

    cw-walk-flag (08-08) refuted the all-modes routing: the global
    charge collapsed rise/raise. Fake a flagged leg by shifting the
    episode-start pad reference down 0.2 m and check the term fires
    per routing flag and mode.
    """
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    def make_env(walk_only: float, walk_mode: bool):
        cfg = load_config()
        cfg.setdefault("reward", {})["k_flag_leg"] = 5.0
        cfg["reward"]["flag_leg_walk_only"] = walk_only
        env = SimHexapodJointWalkEnv(cfg, seed=3)
        g = env._goal_gen
        for m in ("hold", "lean", "track", "unload", "raise",
                  "rise", "lower", "walk"):
            if hasattr(g, f"p_{m}"):
                setattr(g, f"p_{m}", 0.0)
        setattr(g, "p_walk" if walk_mode else "p_hold", 1.0)
        env.reset()
        assert env._goal_traj.mode == ("walk" if walk_mode else "hold")
        # Fake >allowance clearance on every pad.
        env._pad_z_ref = env._pad_z_ref - 0.2
        _, _, _, _, info = env.step(np.zeros(env.n_act))
        env.close()
        return info

    info = make_env(walk_only=1.0, walk_mode=False)
    assert "reward_flag_leg" not in info, \
        "walk-only routing must not charge hold mode"
    info = make_env(walk_only=0.0, walk_mode=False)
    assert info.get("reward_flag_leg", 0.0) < -0.5, \
        "legacy all-modes routing must still charge"
    info = make_env(walk_only=1.0, walk_mode=True)
    assert info.get("reward_flag_leg", 0.0) < -0.5, \
        "walk-only routing must charge walk mode"
