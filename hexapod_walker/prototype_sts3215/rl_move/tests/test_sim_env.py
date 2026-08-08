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


def test_asym_policy_actor_masked_critic_full():
    """AsymActorCriticPolicy: privileged dims invisible to the actor,
    visible to the critic; state_dict transplant-compatible with the
    stock MlpPolicy (same net_arch)."""
    torch = pytest.importorskip("torch")
    from gymnasium import spaces
    from stable_baselines3.common.policies import ActorCriticPolicy
    from rl_move.sim.asym_policy import AsymActorCriticPolicy

    obs_n = 10
    obs_space = spaces.Box(-np.inf, np.inf, (obs_n,), dtype=np.float32)
    act_space = spaces.Box(-1.0, 1.0, (3,), dtype=np.float32)
    pol = AsymActorCriticPolicy(obs_space, act_space, lambda _: 3e-4,
                                net_arch=[16, 16],
                                privileged_idx=(-2, -1))
    pol.set_training_mode(False)
    obs = torch.zeros(1, obs_n)
    obs_priv = obs.clone()
    obs_priv[0, -2:] = 5.0          # perturb privileged dims only
    obs_reg = obs.clone()
    obs_reg[0, 0] = 5.0             # perturb a hardware-visible dim
    with torch.no_grad():
        a0 = pol._predict(obs, deterministic=True)
        a_priv = pol._predict(obs_priv, deterministic=True)
        a_reg = pol._predict(obs_reg, deterministic=True)
        v0 = pol.predict_values(obs)
        v_priv = pol.predict_values(obs_priv)
    assert torch.allclose(a0, a_priv), \
        "actor must be blind to privileged dims"
    assert not torch.allclose(a0, a_reg), \
        "actor must still see hardware dims (mask too broad?)"
    assert not torch.allclose(v0, v_priv), \
        "critic must see privileged dims"
    stock = ActorCriticPolicy(obs_space, act_space, lambda _: 3e-4,
                              net_arch=[16, 16])
    assert set(pol.state_dict()) == set(stock.state_dict()), \
        "state_dict keys must match stock MlpPolicy for transplant"
    pol.load_state_dict(stock.state_dict(), strict=True)


def test_walk_lp_curriculum_bucket_sampling():
    """goal.walk_lp_curriculum=1: commands come from LP_BUCKETS per the
    broadcast weights; bucket id surfaces in step info; default off is
    untouched elsewhere (covered by legacy tests)."""
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv, LP_BUCKETS

    cfg = load_config()
    cfg.setdefault("goal", {})["walk_lp_curriculum"] = 1.0
    env = SimHexapodJointWalkEnv(cfg, seed=0)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise",
              "rise", "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_walk = 1.0
    one_hot = np.zeros(len(LP_BUCKETS))
    one_hot[5] = 1.0                      # bucket 5 = 0.07-0.08 m/s
    env.set_walk_bucket_weights(one_hot)
    for _ in range(5):
        env.reset()
        traj = env._goal_traj
        speed = float(np.hypot(traj.vx[-1], traj.vy[-1]))
        lo, hi = LP_BUCKETS[5]
        assert lo <= speed <= hi + 1e-9
        assert env._walk_bucket == 5
    _, _, _, _, info = env.step(np.zeros(env.n_act))
    assert info.get("walk_bucket") == 5
    env.set_walk_bucket_weights(np.zeros(len(LP_BUCKETS)))  # → uniform
    seen = set()
    for _ in range(40):
        env.reset()
        seen.add(env._walk_bucket)
    assert len(seen) >= 4, f"uniform sampling too narrow: {seen}"
    env.close()


def test_canary_force_rise_start_and_seed_determinism():
    """Fixed-seed canaries (review §5a): force_rise_start pins the rise
    start kind, and reset(seed=N) makes the episode draw repeatable."""
    from rl_move.sim.goal_task import SimHexapodGoalEnv

    env = SimHexapodGoalEnv(randomize=True, seed=7)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_rise = 1.0
    for force, check in (
            ("flat", lambda t: t.start_at == "zero" and t.start_curl == 0),
            ("bridge", lambda t: t.start_at == "zero" and t.start_curl > 0),
            ("crouch", lambda t: t.start_at == "crouch")):
        g.force_rise_start = force
        for seed in (1001, 2002):
            env.reset(seed=seed)
            assert check(env._goal_traj), \
                f"force={force} gave {env._goal_traj.start_at}" \
                f"/{env._goal_traj.start_curl}"
    # Same seed twice = identical episode (obs stream + goal heights).
    g.force_rise_start = "bridge"
    obs_a, _ = env.reset(seed=1001)
    h_a = env._goal_traj.height.copy()
    curl_a = env._goal_traj.start_curl
    obs_b, _ = env.reset(seed=1001)
    assert np.allclose(obs_a, obs_b)
    assert np.allclose(h_a, env._goal_traj.height)
    assert curl_a == env._goal_traj.start_curl
    # Hook off = normal random draw still works.
    g.force_rise_start = None
    env.reset()
    env.close()


def test_canary_runner_and_protected_groups():
    """_run_canaries covers every case on a rise+lower env; the
    protected set is exactly the groups the baseline passed 2/2."""
    from rl_move.sim.goal_task import SimHexapodGoalEnv
    from rl_move.sim.train_ppo_sim import (
        CANARY_CASES, _protected_groups, _run_canaries)

    # Full-length episodes: the goal generator's hold+ramp windows must
    # fit inside the episode (canaries always run at the training
    # episode length in production).
    env = SimHexapodGoalEnv(seed=9)
    res = _run_canaries(env, lambda obs: np.zeros(env.n_act))
    assert set(res) == {c[0] for c in CANARY_CASES}
    assert all(isinstance(v, bool) for v in res.values())
    assert env._goal_gen.force_rise_start is None  # hook disarmed after
    env.close()

    assert _protected_groups({c[0]: True for c in CANARY_CASES}) == \
        ["lower", "rise_bridge", "rise_crouch", "rise_flat"]
    partial = {c[0]: c[0] != "rise_flat_b" for c in CANARY_CASES}
    assert "rise_flat" not in _protected_groups(partial)  # 1/2 ≠ protected


def test_canary_stop_callback_streak_logic():
    """Auto-stop (review §5c): 3 consecutive FULL-group failures of a
    protected skill stop training; partial failures reset nothing."""
    from rl_move.sim.train_ppo_sim import _make_canary_stop_callback

    class FakeBg:
        def __init__(self):
            self.queue = []

        def pop_canaries(self):
            out, self.queue = self.queue, []
            return out

    def probe(flat_a, flat_b):
        return {"rise_flat_a": flat_a, "rise_flat_b": flat_b,
                "lower_a": True, "lower_b": True}

    bg = FakeBg()
    cb = _make_canary_stop_callback(bg, ["rise_flat", "lower"],
                                    stop_after=3)
    cb.num_timesteps = 0
    bg.queue = [probe(False, False), probe(False, False)]
    assert cb._on_step() is True          # streak 2 < 3
    bg.queue = [probe(False, True)]
    assert cb._on_step() is True          # 1/2 pass resets the streak
    assert cb.streak["rise_flat"] == 0
    bg.queue = [probe(False, False)] * 3
    assert cb._on_step() is False         # 3 consecutive full failures
    # monitor-only mode never stops
    bg2 = FakeBg()
    cb2 = _make_canary_stop_callback(bg2, ["rise_flat"], stop_after=0)
    cb2.num_timesteps = 0
    bg2.queue = [probe(False, False)] * 5
    assert cb2._on_step() is True


def test_walk_success_requires_gait_validity():
    """Gait-validity gate (review §5b): perfect tracking with a
    sacrificed leg is NOT a successful walk episode."""
    from rl_move.sim.eval_checkpoint import _success

    good = {"vel_err_mean": 0.01, "gait_valid": True}
    assert _success("walk", False, good)
    flagged = {"vel_err_mean": 0.01, "gait_valid": False,
               "sacrificed_legs": [4]}
    assert not _success("walk", False, flagged)


def test_walk_phase_obs_clock_and_routing():
    """goal.walk_phase_obs=1: obs +2 (sin/cos), clock advances at
    goal.walk_phase_hz only while a walk velocity is commanded; default
    off leaves the legacy width untouched."""
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    env0 = SimHexapodJointWalkEnv(cfg, seed=0)
    w_legacy = env0.observation_space.shape[0]
    obs, _ = env0.reset()
    assert obs.shape[0] == w_legacy
    env0.close()

    cfg = load_config()
    cfg.setdefault("goal", {})["walk_phase_obs"] = 1.0
    cfg["goal"]["walk_phase_hz"] = 1.0
    env = SimHexapodJointWalkEnv(cfg, seed=0)
    assert env.observation_space.shape[0] == w_legacy + 2
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_walk = 1.0
    obs, _ = env.reset()
    assert env._goal_traj.mode == "walk"
    assert obs.shape[0] == w_legacy + 2
    # settle hold: no command yet -> clock frozen at 0 -> [sin,cos]=[0,1]
    obs, _, _, _, _ = env.step(np.zeros(env.n_act))
    assert abs(obs[-2] - 0.0) < 1e-6 and abs(obs[-1] - 1.0) < 1e-6
    # run past the 1 s hold: clock must advance (2*pi*hz*dt per tick)
    for _ in range(int(2.0 / env.dt)):
        obs, _, _, _, info = env.step(np.zeros(env.n_act))
    assert env._phase > 0.0
    assert abs(obs[-2] - np.sin(env._phase)) < 1e-6
    assert abs(obs[-1] - np.cos(env._phase)) < 1e-6
    env.close()

    # hold mode: phase never advances, obs tail stays [0, 1]
    cfg = load_config()
    cfg.setdefault("goal", {})["walk_phase_obs"] = 1.0
    env = SimHexapodJointWalkEnv(cfg, seed=1)
    g = env._goal_gen
    for m in ("lean", "track", "unload", "raise", "rise", "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_hold, g.p_walk = 1.0, 0.0
    obs, _ = env.reset()
    assert env._goal_traj.mode == "hold"
    for _ in range(10):
        obs, _, _, _, info = env.step(np.zeros(env.n_act))
    assert env._phase == 0.0
    assert abs(obs[-2]) < 1e-6 and abs(obs[-1] - 1.0) < 1e-6
    assert "reward_phase_contact" not in info
    env.close()


def test_phase_contact_reward_pays_agreement():
    """reward.k_phase_contact: walk-only by construction, value equals
    k * (agreement - 0.5) * 2 recomputed from the touch sensors, and a
    static all-feet-down stance nets ~zero (agreement 3/6)."""
    import math as _math
    from rl_move.config import load_config
    from rl_move.sim.walk_task import (SimHexapodJointWalkEnv,
                                       PHASE_TRIPOD_A)

    cfg = load_config()
    cfg.setdefault("goal", {})["walk_phase_obs"] = 1.0
    cfg.setdefault("reward", {})["k_phase_contact"] = 1.0
    env = SimHexapodJointWalkEnv(cfg, seed=0)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_walk = 1.0
    env.reset()
    seen = False
    for _ in range(int(3.0 / env.dt)):
        _, _, term, trunc, info = env.step(np.zeros(env.n_act))
        if "reward_phase_contact" in info:
            seen = True
            stance_a = _math.sin(env._phase) >= 0.0
            agree = 0
            for f in range(6):
                adr = env._touch_adr[f]
                on = (adr >= 0
                      and float(env.data.sensordata[adr]) > 0.5)
                agree += int(on == ((f in PHASE_TRIPOD_A) == stance_a))
            expect = 1.0 * (agree / 6.0 - 0.5) * 2.0
            assert abs(info["reward_phase_contact"] - expect) < 1e-9
            assert abs(info["phase_agreement"] - agree / 6.0) < 1e-9
        if term or trunc:
            break
    assert seen, "phase reward never fired during a commanded walk"
    # zero-action = all feet planted -> agreement 3/6 -> zero net reward
    assert abs(info["reward_phase_contact"]) < 1e-9
    env.close()


def test_obs_pad_transplant_preserves_parent_behavior():
    """pad_obs_transplant: outputs bit-identical to the parent for any
    value of the appended dims (zero first-layer columns)."""
    torch = pytest.importorskip("torch")
    import gymnasium as gym
    from stable_baselines3 import PPO
    from rl_move.sim.train_ppo_sim import pad_obs_transplant

    class _Tiny(gym.Env):
        def __init__(self, n):
            self.observation_space = gym.spaces.Box(
                -np.inf, np.inf, (n,), dtype=np.float32)
            self.action_space = gym.spaces.Box(
                -1.0, 1.0, (3,), dtype=np.float32)
            self._n = n

        def reset(self, *, seed=None, options=None):
            return np.zeros(self._n, dtype=np.float32), {}

        def step(self, action):
            return (np.zeros(self._n, dtype=np.float32),
                    0.0, False, False, {})

    old = PPO("MlpPolicy", _Tiny(8), n_steps=32, seed=0, device="cpu",
              policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0))
    new = PPO("MlpPolicy", _Tiny(10), n_steps=32, seed=1, device="cpu",
              policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0))
    pad_obs_transplant(old, new, 2)
    obs8 = np.random.RandomState(3).randn(5, 8).astype(np.float32)
    pad = np.random.RandomState(4).randn(5, 2).astype(np.float32) * 10
    obs10 = np.concatenate([obs8, pad], axis=1)
    a_old, _ = old.predict(obs8, deterministic=True)
    a_new, _ = new.predict(obs10, deterministic=True)
    assert np.allclose(a_old, a_new, atol=0), \
        "padded dims must be invisible until trained"
    v_old = old.policy.predict_values(torch.as_tensor(obs8))
    v_new = new.policy.predict_values(torch.as_tensor(obs10))
    assert torch.allclose(v_old, v_new)
