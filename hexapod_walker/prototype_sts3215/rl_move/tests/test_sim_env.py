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
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0])
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
        env._imu_prev_p = env._imu_prev_v = None
        env._att_rp = None
        rolls = []
        for i in range(50):
            # Gentle 1 Hz body-roll sine — sustained rotation without
            # physically walking the robot over.
            a = np.array([math.sin(2 * math.pi * i / 25),
                          0.0, 0.0, 0.0, 0.0])
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
