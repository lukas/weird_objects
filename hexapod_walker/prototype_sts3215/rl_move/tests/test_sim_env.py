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


def test_struct_compliance_applies_model_and_reported_q():
    cfg = {
        "struct_comp": {
            "enabled": 1.0,
            "k_yaw_nm_rad": 300.0,
            "k_hip_nm_rad": 180.0,
            "k_knee_nm_rad": 120.0,
            "dr_scale_lo": 1.0,
            "dr_scale_hi": 1.0,
        },
    }
    env = SimHexapodBalanceEnv(cfg=cfg, seed=5)
    env.reset()

    base_kp = env.params.per_joint("kp")
    k = env._struct_comp_k
    assert k is not None
    expected_kp = base_kp * k / (base_kp + k)
    assert env.model.actuator_gainprm[env._pos_act, 0] == pytest.approx(
        expected_kp)

    j = 2  # L0 knee
    q_phys = env.data.qpos[env._qadr].copy()
    env.data.qfrc_actuator[env._vadr] = 0.0
    env.data.qfrc_actuator[env._vadr[j]] = 1.2
    env._att_rp = None
    st = env._read_state()
    assert st.joint_position[j] == pytest.approx(q_phys[j] - 1.2 / k[j])


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

    # Unload goal penalizes the loaded target leg's current.  At reset the
    # current filter can legitimately still be all zero, so inject a direct
    # loaded-leg state instead of relying on incidental settle current.
    st.servo_current = np.zeros(N_JOINTS)
    leg = 2
    st.servo_current[3 * leg:3 * leg + 3] = np.array([0.18, 0.20, 0.22])
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
    # With the measured 150 mm knee->boot-tip tibia, the old 20/80 plant
    # footprint is an extreme tuck for this fixed-foot rise test: solving
    # 50 mm of body lift from that footprint asks for ~156 deg of knee, past
    # the hardware cap.  20/60 has the same vertical foot drop but keeps the
    # curled footprint inside the physical knee envelope.
    q_plant = np.array([0.0, 20.0, 60.0] * 6) * DEG
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
    # Belly rest is ~40 mm; the plant stance is roughly 180 mm with the
    # measured 150 mm knee->boot-tip tibia.
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
    # Cases whose mode the env's generator lacks are skipped (the walk
    # cases on a stance-only goal env — RISE_WALK_NEXT_48H retention
    # set, 08-13); everything the generator supports must run.
    expected = {c[0] for c in CANARY_CASES
                if hasattr(env._goal_gen, f"p_{c[1]}")}
    assert set(res) == expected
    assert {"hold_a", "hold_b"} <= set(res)       # hold runs on goal env
    assert "walk_fwd_a" not in res                # no p_walk -> skipped
    assert all(isinstance(v, bool) for v in res.values())
    assert env._goal_gen.force_rise_start is None  # hook disarmed after
    env.close()

    assert _protected_groups({c[0]: True for c in CANARY_CASES}) == \
        ["hold", "lower", "rise_bridge", "rise_crouch", "rise_flat",
         "walk_fwd"]
    partial = {c[0]: c[0] != "rise_flat_b" for c in CANARY_CASES}
    assert "rise_flat" not in _protected_groups(partial)  # 1/2 ≠ protected


def test_walk_canary_smoke():
    """The pinned-forward walk canary (RISE_WALK_NEXT_48H retention
    set) runs end-to-end on the joint-walk env and fails a do-nothing
    policy — zero progress can never read as a pass."""
    from rl_move.sim.train_ppo_sim import _run_walk_canary
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    env = SimHexapodJointWalkEnv(seed=5, episode_seconds=6.0)
    gen = env._goal_gen
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_walk = 1.0
    obs, _ = env.reset(seed=6001)
    ok = _run_walk_canary(env, lambda o: np.zeros(env.n_act), obs)
    assert ok is False
    # command was pinned: zero head, then forward-only schedule
    traj = env._goal_traj
    assert float(np.max(traj.vx)) > 0.0 and np.all(traj.vy == 0.0)
    env.close()


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


def test_recover_success_is_the_named_terminal_and_keeps_start_kind():
    from types import SimpleNamespace

    from rl_move.sim.eval_checkpoint import _start_kind, _success

    ep = {"term_reason": "recover_success"}
    assert _success("recover", True, ep)
    assert not _success("recover", False, ep)
    assert not _success("recover", True, {"term_reason": "tilt"})
    traj = SimpleNamespace(start_at="any", start_kind="onefoot")
    assert _start_kind(traj) == "onefoot"


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


# ---------------------------------------------------------------------------
# First-principles posture terms (operator directive 2026-08-08 ~20:45Z)


def test_support_margin_geometry():
    from rl_move.sim.sim_env import support_margin_m
    import numpy as np
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
    # center of unit square: 0.5 from every edge
    assert abs(support_margin_m(sq, np.array([0.5, 0.5])) - 0.5) < 1e-9
    # near an edge
    assert abs(support_margin_m(sq, np.array([0.1, 0.5])) - 0.1) < 1e-9
    # outside: negative
    assert support_margin_m(sq, np.array([-0.2, 0.5])) < 0
    # degenerate: <3 points or collinear -> 0
    assert support_margin_m(sq[:2], np.array([0.5, 0.5])) == 0.0
    line = np.array([[0, 0], [1, 0], [2, 0]], dtype=float)
    assert support_margin_m(line, np.array([1.0, 0.5])) == 0.0


def test_posture_reward_terms_smoke():
    """k_support_margin / k_load_even produce finite, correctly-signed
    parts on a standing robot and stay absent when disabled (default)."""
    import numpy as np
    from rl_move.config import load_config
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.joint_task import SimHexapodJointGoalEnv, q_rad_to_action

    cfg = load_config()
    cfg.setdefault("reward", {})
    cfg["reward"]["k_support_margin"] = 1.0
    cfg["reward"]["k_load_even"] = 1.0
    env = SimHexapodJointGoalEnv(params=SimServoParams.load(), cfg=cfg,
                                 randomize=False, episode_seconds=2.0,
                                 seed=0)
    obs, _ = env.reset()
    a = q_rad_to_action(env._cmd.copy())
    parts_seen = {}
    for _ in range(10):
        obs, r, term, trunc, info = env.step(a)
        for k in ("reward_support_margin", "reward_load_even"):
            if k in info:
                parts_seen[k] = info[k]
    assert "reward_support_margin" in parts_seen, "margin term never fired"
    assert "reward_load_even" in parts_seen, "evenness term never fired"
    # standing plant: CoM well inside, load roughly even
    assert 0.0 < parts_seen["reward_support_margin"] <= 1.0
    assert -1.0 <= parts_seen["reward_load_even"] <= 0.0
    assert all(np.isfinite(v) for v in parts_seen.values())
    env.close()


# ---------------------------------------------------------------------------
# Temporal actor: env-side obs history (plan §Architecture, cycle 13)


def test_obs_history_stack_newest_first_and_parity():
    """obs.history_frames=K: width xK, frame 0 == the single-frame env's
    obs bit-exactly every tick (refactor parity), frame i is the obs
    from i ticks ago, reset seeds the buffer with the reset obs."""
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    ref = SimHexapodJointWalkEnv(load_config(), seed=5)
    w = ref.observation_space.shape[0]

    cfg = load_config()
    cfg.setdefault("obs", {})["history_frames"] = 4
    env = SimHexapodJointWalkEnv(cfg, seed=5)
    assert env.observation_space.shape[0] == 4 * w

    o_ref, _ = ref.reset()
    o, _ = env.reset()
    frames = o.reshape(4, w)
    for i in range(4):
        assert np.array_equal(frames[i], o_ref), "reset must seed buffer"

    prev_frame0 = frames[0].copy()
    for _ in range(6):
        a = np.zeros(env.n_act, dtype=np.float32)
        o_ref = ref.step(a)[0]
        o, _, term, trunc, _ = env.step(a)
        frames = o.reshape(4, w)
        assert np.array_equal(frames[0], o_ref), \
            "frame 0 must equal the history-free env's obs (parity)"
        assert np.array_equal(frames[1], prev_frame0), \
            "frame 1 must be last tick's frame 0 (newest first)"
        prev_frame0 = frames[0].copy()
        if term or trunc:
            break
    ref.close()
    env.close()


def test_obs_history_stance_env_width():
    """joint_goal (stance line) honors obs.history_frames too."""
    from rl_move.config import load_config
    from rl_move.sim.joint_task import SimHexapodJointGoalEnv

    cfg = load_config()
    cfg.setdefault("obs", {})["history_frames"] = 8
    env = SimHexapodJointGoalEnv(cfg, seed=0)
    obs, _ = env.reset()
    assert obs.shape == (8 * 68,)
    assert env.observation_space.shape == (8 * 68,)
    env.close()


def test_privileged_idx_history_frames():
    """asym-critic mask indices: (-2,-1) special case generalizes to one
    vel pair per stacked frame; phase obs shifts the pair by 2."""
    from types import SimpleNamespace
    from rl_move.sim.train_ppo_sim import _privileged_idx

    a = SimpleNamespace(cfg_set=None)
    assert _privileged_idx(a, 72) == (70, 71)
    a = SimpleNamespace(cfg_set=["obs.history_frames=4"])
    assert _privileged_idx(a, 288) == (70, 71, 142, 143, 214, 215,
                                       286, 287)
    a = SimpleNamespace(cfg_set=["obs.history_frames=2",
                                 "goal.walk_phase_obs=1"])
    assert _privileged_idx(a, 148) == (70, 71, 144, 145)


# ---------------------------------------------------------------------------
# Step-event reward package (operator queue item 0, cycle 13)


def _walk_only_env(seed=0, **rw):
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    cfg.setdefault("reward", {}).update(rw)
    env = SimHexapodJointWalkEnv(cfg, seed=seed)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_walk = 1.0
    return env


def test_park_duty_charges_pinned_legs_only():
    """k_park_duty: a standing (all-duty-1.0) stance during a commanded
    walk pays 6*0.1*k per tick once the 2 s window fills; charge fires
    only while a velocity is commanded."""
    env = _walk_only_env(seed=0, k_park_duty=1.0)
    env.reset()
    assert env._goal_traj.mode == "walk"
    n_win = int(round(2.0 / env.dt))
    charged = []
    for _ in range(int(4.0 / env.dt)):
        _, _, term, trunc, info = env.step(np.zeros(env.n_act))
        if "reward_park_duty" in info:
            charged.append(info["reward_park_duty"])
        if term or trunc:
            break
    assert charged, "park charge never evaluated during commanded walk"
    # zero-action = all six feet planted -> duty 1.0 each -> 6 * 0.1
    filled = [c for c in charged if c != 0.0]
    assert filled, "window never filled"
    assert abs(filled[-1] - (-0.6)) < 0.15, filled[-1]
    env.close()


def test_step_event_pays_forward_swing_not_park():
    """k_step_event: a synthetic completed swing along the command pays
    k*min(along/30mm, 1.5); a leg that never touches down pays 0."""
    env = _walk_only_env(seed=0, k_step_event=2.0)
    env.reset()
    # drive past the settle hold so a velocity is commanded
    for _ in range(int(1.5 / env.dt)):
        _, _, _, _, info = env.step(np.zeros(env.n_act))
    goal = env._current_goal()
    s = float(np.hypot(goal.vx_ref, goal.vy_ref))
    assert s > 1e-3
    # fake a liftoff record 40mm behind the current pad position along
    # the command, mark the foot airborne, then let the real contact
    # close the event on the next step (pick a foot that IS in contact
    # under zero action — the settled pose loads only some feet)
    loaded = [f for f in range(6)
              if float(env.data.sensordata[env._touch_adr[f]]) > 0.5]
    assert loaded, "no loaded foot to test with"
    f = loaded[0]
    xy = env.data.xpos[env._pad_bids[f], :2].copy()
    u = np.array([goal.vx_ref, goal.vy_ref]) / s
    env._foot_on[f] = False
    env._liftoff_xy[f] = xy - 0.040 * u
    env._liftoff_step[f] = env._step_i - 5
    _, _, _, _, info = env.step(np.zeros(env.n_act))
    r = info.get("reward_step_event", 0.0)
    # foot 0 is in contact under zero action -> touchdown event fires:
    # along >= ~40mm -> capped 1.5x -> 2.0*1.5 = 3.0 (allow contact
    # micro-motion tolerance)
    assert r > 2.0, r
    env.close()


def test_drag_charges_loaded_translation():
    """k_drag_loaded: per-tick charge equals k * loaded slip beyond the
    0.5mm deadband; a quiet stance pays ~nothing."""
    env = _walk_only_env(seed=0, k_drag_loaded=10.0)
    env.reset()
    tot = 0.0
    for _ in range(int(2.0 / env.dt)):
        _, _, term, trunc, info = env.step(np.zeros(env.n_act))
        tot += info.get("reward_drag", 0.0)
        if term or trunc:
            break
    # zero action = feet planted, no commanded drag -> tiny charge only
    assert tot > -0.5, tot
    env.close()


def test_drag_stance_allowance_and_floor_gate_the_charge():
    """k_drag_stance: sliding inside the per-stance allowance is free,
    and ticks below the jitter floor never accumulate — with either
    gate wide open the charge is exactly zero no matter how the feet
    move (zero action in this env actively drags toward joint center,
    ~20+ mm of above-floor slide in 2 s, so this is a real motion)."""
    for kw in ({"drag_stance_allow_mm": 1000.0},
               {"drag_stance_tick_floor_mm": 50.0}):
        env = _walk_only_env(seed=0, k_drag_stance=7000.0, **kw)
        env.reset()
        tot = 0.0
        for _ in range(int(2.0 / env.dt)):
            _, _, term, trunc, info = env.step(np.zeros(env.n_act))
            tot += info.get("reward_drag_stance", 0.0)
            if term or trunc:
                break
        assert tot == 0.0, (kw, tot)
        env.close()


def test_drag_stance_charges_over_allowance_stroke():
    """k_drag_stance: once a stance's accumulated travel exceeds the
    allowance, every further above-floor loaded millimetre is charged
    at k — and a touchdown resets the accumulator (fresh allowance)."""
    env = _walk_only_env(seed=0, k_drag_stance=7000.0)
    env.reset()
    assert env._goal_traj.mode == "walk"
    env.step(np.zeros(env.n_act))          # settle contact bookkeeping
    # Prime every foot as if its stance already dragged 20 mm (> 6 mm
    # allowance): the zero-action drag toward joint center now pays
    # k * slip on every above-floor loaded tick.
    env._stance_slip_acc = [0.020] * 6
    charged = 0.0
    for _ in range(int(2.0 / env.dt)):
        _, _, term, trunc, info = env.step(np.zeros(env.n_act))
        charged += info.get("reward_drag_stance", 0.0)
        if term or trunc:
            break
    assert charged < -0.5, charged
    # Touchdown reset: mark a loaded foot airborne, let real contact
    # re-plant it -> accumulator returns to zero (allowance renewed).
    loaded = [f for f in range(6)
              if float(env.data.sensordata[env._touch_adr[f]]) > 0.5]
    assert loaded, "no loaded foot to test with"
    f = loaded[0]
    env._foot_on[f] = False
    env._stance_slip_acc[f] = 0.050
    env.step(np.zeros(env.n_act))
    assert env._stance_slip_acc[f] < 0.010, env._stance_slip_acc[f]
    env.close()


def test_walk_gait_start_spawns_mid_stride_tall_with_live_command():
    """goal.walk_gait_start_frac (TALL LADDER T6: RSI-for-walk): with
    frac=1 every walk episode spawns MID-STRIDE in the scripted tripod
    gait's tall pose and the command is live almost immediately (0.3 s
    ramp, no 1 s hold). Default off = legacy rng streams untouched."""
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    cfg.setdefault("goal", {})["walk_gait_start_frac"] = 1.0
    env = SimHexapodJointWalkEnv(cfg, seed=0)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_walk = 1.0
    # Sample several spawns: every one must be TALL with a live
    # command; at least one must be strongly mid-stride (the random
    # phase legitimately hits tripod-crossover sometimes, where the
    # two tripods pass through symmetry).
    knee_asym = []
    for _ in range(5):
        env.reset()
        traj = env._goal_traj
        assert traj.mode == "walk" and traj.start_at == "gait"
        # Command live right after the fast ramp (legacy hold keeps
        # ~1 s of zeros; here 0.5 s in must be at full command).
        i = int(round(0.5 / env.dt))
        assert float(np.hypot(traj.vx[i], traj.vy[i])) > 0.02, (
            traj.vx[i], traj.vy[i])
        # TALL: mean hip pitch is much nearer the plant's +20 deg band
        # than the learned crouch (~-40 deg).  Structural compliance can
        # sag the physical qpos below 0 while still being a tall spawn.
        q = env.data.qpos[7:7 + 18].reshape(6, 3)
        pitch_deg = np.degrees(q[:, 1])
        assert float(np.mean(pitch_deg)) > -20.0, pitch_deg
        knee_deg = np.degrees(q[:, 2])
        knee_asym.append(abs(float(np.mean(knee_deg[[0, 2, 4]]))
                             - float(np.mean(knee_deg[[1, 3, 5]]))))
    assert max(knee_asym) > 5.0, knee_asym
    env.close()

    # Default off: same seed, no cfg key -> classic plant start.
    cfg2 = load_config()
    env2 = SimHexapodJointWalkEnv(cfg2, seed=0)
    g2 = env2._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        if hasattr(g2, f"p_{m}"):
            setattr(g2, f"p_{m}", 0.0)
    g2.p_walk = 1.0
    env2.reset()
    assert env2._goal_traj.start_at == "plant"
    env2.close()


def test_walk_gait_spawn_wz_turn_state_densification():
    """goal.walk_gait_spawn_wz (turn-state reset densification, 08-23,
    turnlib3 FAIL branch): on a turn-in-place episode drawn into the
    gait spawn, (1) the discarded linear command is NOT resurrected by
    the head replacement (vx/vy stay ~0), (2) the yaw command is live
    right after the 0.3 s fast ramp instead of the legacy 1 s zero
    hold, and (3) the spawn pose itself differs from the spawn_wz=0
    pose at the same seed (omega actually reaches the scripted-gait
    pose generator). Default off (key absent) = wz head untouched."""
    from rl_move.config import load_config
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    def make_env(spawn_wz):
        cfg = load_config()
        goal = cfg.setdefault("goal", {})
        goal["walk_yaw_cmd"] = 1
        goal["walk_yaw_max_rad_s"] = 0.3
        goal["walk_turn_in_place_frac"] = 1.0
        goal["walk_gait_start_frac"] = 1.0
        if spawn_wz:
            goal["walk_gait_spawn_wz"] = spawn_wz
        env = SimHexapodJointWalkEnv(cfg, seed=0)
        g = env._goal_gen
        for m in ("hold", "lean", "track", "unload", "raise", "rise",
                  "lower"):
            if hasattr(g, f"p_{m}"):
                setattr(g, f"p_{m}", 0.0)
        g.p_walk = 1.0
        env.reset()
        return env

    env_on = make_env(1.0)
    traj = env_on._goal_traj
    assert traj.mode == "walk" and traj.start_at == "gait"
    i = int(round(0.5 / env_on.dt))
    # (1) turn-in-place preserved: no linear command resurrection.
    assert float(np.hypot(traj.vx[i], traj.vy[i])) < 1e-9, (
        traj.vx[i], traj.vy[i])
    # (2) yaw command live at 0.5 s (tip draw guarantees |wz_t| >=
    # 0.5 * wz_max = 0.15).
    assert abs(float(traj.wz[i])) >= 0.15 - 1e-9, traj.wz[i]
    q_on = env_on.data.qpos[7:7 + 18].copy()
    env_on.close()

    # Key off, same seed: rng streams identical (no extra draws), but
    # the wz head keeps the legacy zero hold and the pose ignores wz.
    env_off = make_env(0.0)
    traj_off = env_off._goal_traj
    assert traj_off.start_at == "gait"
    assert abs(float(traj_off.wz[i])) < 1e-9, traj_off.wz[i]
    q_off = env_off.data.qpos[7:7 + 18].copy()
    env_off.close()
    # (3) omega reached the pose generator: spawn joints differ.
    assert float(np.max(np.abs(q_on - q_off))) > 1e-4, (
        np.max(np.abs(q_on - q_off)))


# ---------------------------------------------------------------------------
# Terminal end-posture pricing (cycle 14, pre-registered structural option)


def _goal_only_env(mode, **rw):
    from rl_move.config import load_config
    from rl_move.sim.joint_task import SimHexapodJointGoalEnv
    cfg = load_config()
    cfg.setdefault("reward", {}).update(rw)
    env = SimHexapodJointGoalEnv(cfg, seed=0)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    setattr(g, f"p_{mode}", 1.0)
    return env


def test_end_posture_charges_hoisted_leg_only_after_settle():
    """k_end_posture: a 150 mm hoisted leg pays k*(clear-20mm) per tick
    strictly AFTER the goal height schedule settles (+0.5 s grace);
    the motion phase pays nothing."""
    env = _goal_only_env("lower", k_end_posture=5.0)
    env.reset()
    assert env._goal_traj.mode == "lower"
    # Fake a hoisted leg: shift leg 4's grounded reference down 150 mm
    # so its (actually grounded) pad reads as 150 mm clearance.
    env._pad_z_ref[4] -= 0.150
    before, after = [], []
    from rl_move.sim.joint_task import q_rad_to_action
    a = q_rad_to_action(env._cmd.copy())
    for _ in range(env.episode_steps):
        _, _, term, trunc, info = env.step(a)
        part = info.get("reward_end_posture")
        assert env._end_posture_from is not None
        (after if env._step_i >= env._end_posture_from
         else before).append(part)
        if term or trunc:
            break
    assert all(p is None for p in before), \
        "charged during the motion phase"
    charged = [p for p in after if p is not None]
    assert charged, "terminal window never charged"
    # leg 4: clear 0.150 - lower allowance 0.060 = 0.090 -> -5*0.09 =
    # -0.45 (other feet may add real clearance while lowering)
    assert -1.4 < charged[-1] <= -0.40, charged[-1]
    env.close()


def test_end_posture_grounded_feet_and_routing_free():
    """Grounded feet pay ~nothing in a charged mode; hold mode (covered
    by stance_clearance instead) is never charged at all."""
    from rl_move.sim.joint_task import q_rad_to_action
    env = _goal_only_env("lower", k_end_posture=5.0)
    env.reset()
    a = q_rad_to_action(env._cmd.copy())
    tot = 0.0
    for _ in range(env.episode_steps):
        _, _, term, trunc, info = env.step(a)
        tot += info.get("reward_end_posture") or 0.0
        if term or trunc:
            break
    # holding the plant keeps feet grounded; only real lowering motion
    # may graze the 20 mm allowance briefly
    assert tot > -2.0, tot
    env.close()
    env = _goal_only_env("hold", k_end_posture=5.0)
    env.reset()
    env._pad_z_ref[4] -= 0.150
    a = q_rad_to_action(env._cmd.copy())
    for _ in range(int(3.0 / env.dt)):
        _, _, term, trunc, info = env.step(a)
        assert "reward_end_posture" not in info
        if term or trunc:
            break
    env.close()


# ---------------------------------------------------------------------------
# env.leg_chassis_collision — belly knife-edge contact axis (SIM.md gap 4)
# ---------------------------------------------------------------------------

def _legcol_chassis_pairs(model, data):
    """Active contact pairs between a legcol geom and a chassis-underside
    geom (either order)."""
    legcol = set()
    chassis = set()
    for i in range(6):
        for g in (f"L{i}_femur_col", f"L{i}_knee_servo_col",
                  f"L{i}_tibia_col"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if gid >= 0:
                legcol.add(gid)
        gid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"L{i}_yaw_servo_col")
        if gid >= 0:
            chassis.add(gid)
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "chassis_box")
    if gid >= 0:
        chassis.add(gid)
    pairs = []
    for c in data.contact[:data.ncon]:
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in legcol and g2 in chassis) or (g2 in legcol
                                                and g1 in chassis):
            pairs.append((g1, g2))
    return pairs


def test_leg_chassis_collision_default_off_bit_exact():
    """Without the cfg key the model's contact masks are untouched."""
    from rl_move.config import load_config
    cfg = load_config()
    env_off = SimHexapodBalanceEnv(cfg=cfg, seed=0)
    from rl_move.sim.servo_model import build_model
    raw = build_model(fixed_base=False, flat_terrain=True)
    assert np.array_equal(env_off.model.geom_contype, raw.geom_contype)
    assert np.array_equal(env_off.model.geom_conaffinity,
                          raw.geom_conaffinity)


def test_leg_chassis_collision_masks_and_scope():
    """Flag on: shank/knee-servo <-> chassis-underside and femur <->
    yaw-servo-box pairs become legal; femur <-> chassis_box (a permanent
    ~15 mm primitive overlap at the hip anchor) stays OFF, and leg-leg
    plus every ground pairing stay exactly as before."""
    from rl_move.config import load_config
    cfg = load_config()
    cfg.setdefault("env", {})["leg_chassis_collision"] = 1
    env = SimHexapodBalanceEnv(cfg=cfg, seed=0)
    m = env.model
    tib = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "L0_tibia_col")
    kns = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,
                            "L2_knee_servo_col")
    box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "chassis_box")
    ysc = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "L3_yaw_servo_col")
    fem = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "L1_femur_col")
    # Tucked-shank group pairs with the whole chassis underside.
    assert m.geom_contype[tib] & m.geom_conaffinity[box]
    assert m.geom_contype[tib] & m.geom_conaffinity[ysc]
    assert m.geom_contype[kns] & m.geom_conaffinity[box]
    # Femur pairs with the yaw-servo boxes ONLY, never chassis_box.
    assert m.geom_contype[fem] & m.geom_conaffinity[ysc]
    assert not (m.geom_contype[fem] & m.geom_conaffinity[box])
    assert not (m.geom_contype[box] & m.geom_conaffinity[fem])
    # Leg-leg is still off: legcol conaffinity stays 0.
    assert m.geom_conaffinity[tib] == 0 and m.geom_conaffinity[fem] == 0
    assert not (m.geom_contype[tib] & m.geom_conaffinity[fem])
    # Ground pairings unchanged: floor still pairs with legcol bit 4,
    # feet/chassis keep bit 1.
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "L0_foot")
    assert m.geom_conaffinity[floor] == 5
    assert m.geom_contype[tib] & m.geom_conaffinity[floor]
    assert m.geom_contype[fem] & m.geom_conaffinity[floor]
    assert m.geom_contype[foot] & m.geom_conaffinity[box]  # unchanged


def test_leg_chassis_collision_inert_in_plant_stance():
    """The axis must only act in tucked poses: a nominal plant-stance
    episode produces ZERO legcol<->chassis contacts (no new forces in
    ordinary standing/walking), while a full knee curl DOES bring the
    tucked shank into contact with the chassis underside."""
    from rl_move.config import load_config
    cfg = load_config()
    cfg.setdefault("env", {})["leg_chassis_collision"] = 1
    env = SimHexapodBalanceEnv(cfg=cfg, seed=0)
    env.reset(seed=0)
    hits = []
    for _ in range(25):  # 1 s of plant-stance hold
        env.step(np.zeros(env.n_act, dtype=np.float32))
        hits += _legcol_chassis_pairs(env.model, env.data)
    assert not hits, f"plant stance grew leg-chassis contacts: {hits}"

    # Yawed full tuck: knee at max flexion, hip raised, yaw at the stop
    # — geometry-verified (08-12 sweep) to overlap the tucked shank and
    # its yaw-servo bracket by ~5 mm. mj_forward on the posed model must
    # yield at least one legcol<->chassis contact — proof the compiled
    # pair set actually contains the new pairs (MuJoCo ignores runtime
    # mask edits; this guards against a regression to that approach).
    m, d = env.model, env.data
    qadr = env._qadr
    q = np.zeros(18)
    q[0::3] = -0.61   # yaw at the stop
    q[1::3] = 0.52    # hip raised
    q[2::3] = 2.62    # knee fully flexed (tucked shank)
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    d.qpos[2] = 0.2   # airborne — only leg-chassis pairs can touch
    for j, adr in enumerate(qadr):
        d.qpos[adr] = q[j]
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    tucked = _legcol_chassis_pairs(m, d)
    assert tucked, "yawed full tuck produced no leg-chassis contact"


def test_walk_slip_per_m_undefined_for_zero_command_episode():
    """eval_checkpoint bug fix (08-24, found via cw-amp-joy60-s29-ft1):
    slip_per_m used to divide by max(along_dist_m, 0.05) unconditionally,
    so a whole-episode zero-commanded-speed draw (a 'hold'/turn-in-place
    stress_mix archetype) turned ordinary marching-in-place foot travel
    into a slip_per_m of 100+ (vs a ~2.9 cap) -- exploding the n=24
    median even when every translating episode was healthy. slip_per_m
    must be undefined (None), matching the progress_ratio guard one
    line above it, whenever cmd_dist_m is ~0; the raw slip_m_total stays
    a real number either way so the 'does it stand still' question is
    still answerable."""
    from rl_move.config import load_config
    from rl_move.sim.eval_checkpoint import run_episode
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    class _ZeroModel:
        def predict(self, obs, deterministic=True):
            return np.zeros(N_ACT), None

    cfg = load_config()
    cfg.setdefault("goal", {})
    cfg["goal"]["walk_speed_min_m_s"] = 0.0
    cfg["goal"]["walk_speed_max_m_s"] = 0.0
    cfg["goal"]["walk_yaw_cmd"] = 0.0
    env = SimHexapodJointWalkEnv(cfg, seed=0)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 0.0)
    g.p_walk = 1.0
    env.reset(seed=0)
    assert env._goal_traj.mode == "walk"
    n_ticks = int(round(2.0 / env.dt))
    env._episode_steps = n_ticks  # force a short truncation
    ep, _frames = run_episode(env, _ZeroModel(), deterministic=True,
                              video=False, annotate=None)
    env.close()

    assert ep["cmd_dist_m"] == 0.0
    assert ep["slip_per_m"] is None
    assert isinstance(ep["slip_m_total"], float)
    assert ep["slip_m_total"] >= 0.0
