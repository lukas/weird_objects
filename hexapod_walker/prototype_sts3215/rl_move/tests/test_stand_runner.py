"""Contract lock for the DEPLOY-SIDE rise+hold specialist port
(linux_control/rl_policy.py stand/lower path, RL_PLAN critical-path
blocker "deploy-side port of the rise+hold handoff composition", 08-11).

The sim-proven learned stand-up + quiet hold
(ppo_goal_cw_stand_holdbc1_hard1, rl_docs/RISE.md: rise 12/12 handoffs
to the walk champion with zero falls) is now the robot's live stance
policy (linux_control/rl_policy_weights.json). What must never drift:

1. the LIVE stance weight file IS the specialist (same bytes as the
   picker copy, correct source checkpoint, obs 68 / act 18);
2. the runner feeds it the goal ramp it TRAINED with — the profile
   rides in the weight file's meta (hold 5 s / ramp 6 s / +111 mm, not
   the legacy stance_dr10 5/4/+50 shape) and reproduces the training
   GoalGenerator's rise height trajectory tick-for-tick;
3. legacy weight files without a meta profile keep the old constants
   (stance_dr10 rollback via the picker stays behavior-identical);
4. the runner's 68-wide stand obs layout matches the sim goal env's
   (build_obs + 9-dim TaskGoal block, height scaled by 0.05 m);
5. the deployed numpy forward matches SB3 predict on the source zip.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "linux_control") not in sys.path:
    sys.path.insert(0, str(_ROOT / "linux_control"))

import rl_policy  # noqa: E402  (linux_control runner)

from rl_move.config import load_config  # noqa: E402
from rl_move.env import TaskGoal, build_obs  # noqa: E402

LIVE = _ROOT / "linux_control" / "rl_policy_weights.json"
PICKER = _ROOT / "linux_control" / "policies" / "stand_holdbc1_hard1.json"
SOURCE_ZIP = (_ROOT / "rl_move" / "sim" / "policies"
              / "ppo_goal_cw_stand_holdbc1_hard1.zip")
CFG = load_config(str(_ROOT / "rl_move" / "config.yaml"))

# The specialist's trained goal overrides (ledger cw-stand-holdbc1-hard1).
TRAINED_RISE_MM = (108.0, 114.0)
TRAINED_RISE_RAMP_S = 6.0
TRAINING_HZ = 25.0


def _live_meta() -> dict:
    return json.loads(LIVE.read_text())["meta"]


def test_live_stance_weights_are_the_specialist():
    assert PICKER.is_file(), "picker copy missing"
    assert (hashlib.md5(LIVE.read_bytes()).hexdigest()
            == hashlib.md5(PICKER.read_bytes()).hexdigest()), (
        "live rl_policy_weights.json != picker stand_holdbc1_hard1.json")
    meta = _live_meta()
    assert meta["source"].endswith("ppo_goal_cw_stand_holdbc1_hard1.zip")
    assert meta["obs_dim"] == 68 and meta["act_dim"] == 18
    assert meta["training_hz"] == TRAINING_HZ
    prof = meta["profile"]["stand"]
    assert prof["hold_s"] == 5.0 and prof["ramp_s"] == TRAINED_RISE_RAMP_S
    assert (TRAINED_RISE_MM[0] <= prof["target_m"] * 1000.0
            <= TRAINED_RISE_MM[1])
    # total_s = the handoff eval's validated switch point (12.5 s),
    # inside the 15 s training horizon.
    assert prof["hold_s"] + prof["ramp_s"] < prof["total_s"] <= 15.0


def test_stand_profile_matches_trained_goal_generator():
    """The runner's height ramp == the training GoalGenerator's rise
    trajectory under the run's exact cfg overrides (flat start)."""
    from rl_move.sim.goal_task import GoalGenerator

    meta = _live_meta()
    hz = rl_policy.policy_training_hz(SimpleNamespace(meta=meta))
    prof = rl_policy.policy_profile(
        SimpleNamespace(meta=meta), "stand")
    target = prof["target_m"]
    cfg = load_config(str(_ROOT / "rl_move" / "config.yaml"))
    cfg.setdefault("actions", {})["max_height_mm"] = 115.0
    g = cfg.setdefault("goal", {})
    g["rise_height_mm"] = [target * 1000.0, target * 1000.0]  # pin band
    g["rise_ramp_s"] = TRAINED_RISE_RAMP_S
    gen = GoalGenerator(cfg)
    for a in [x for x in vars(gen) if x.startswith("p_")]:
        setattr(gen, a, 0.0)
    gen.p_rise = 1.0
    gen.force_rise_start = "flat"
    dt = 1.0 / hz
    n = int(round(15.0 * hz))
    traj = gen.sample(np.random.default_rng(0), n, dt)
    assert traj.mode == "rise" and traj.start_at == "zero"
    runner = np.array([rl_policy._height_ref(prof, i * dt)
                       for i in range(n)])
    # linspace-vs-analytic ramp edge differences are sub-millimetre;
    # anything larger means the shapes diverged.
    assert float(np.max(np.abs(runner - traj.height))) < 0.002, (
        "runner height ramp != trained GoalGenerator rise trajectory")


def test_legacy_fallback_without_meta_profile():
    stub = SimpleNamespace(meta={"obs_dim": 68})
    prof = rl_policy.policy_profile(stub, "stand")
    assert prof == {"hold_s": rl_policy.RISE_HOLD_S,
                    "ramp_s": rl_policy.RISE_RAMP_S,
                    "target_m": rl_policy.RISE_TARGET_M,
                    "total_s": rl_policy.RISE_TOTAL_S}
    prof = rl_policy.policy_profile(stub, "lower")
    assert prof == {"hold_s": rl_policy.LOWER_HOLD_S,
                    "ramp_s": rl_policy.LOWER_RAMP_S,
                    "target_m": rl_policy.LOWER_TARGET_M,
                    "total_s": rl_policy.LOWER_TOTAL_S}
    # Partial profiles override key-by-key, keeping legacy for the rest.
    stub = SimpleNamespace(meta={"profile": {"stand": {"ramp_s": 6.0}}})
    prof = rl_policy.policy_profile(stub, "stand")
    assert prof["ramp_s"] == 6.0
    assert prof["target_m"] == rl_policy.RISE_TARGET_M


def test_policy_training_hz_is_required():
    assert rl_policy.policy_training_hz(
        SimpleNamespace(meta={"training_hz": 100})) == 100.0
    with pytest.raises(ValueError, match="missing meta.training_hz"):
        rl_policy.policy_training_hz(SimpleNamespace(meta={"obs_dim": 68}))


def test_timing_trip_policy_tolerates_jitter_then_fails():
    hz = 100.0
    dt = 1.0 / hz
    grace = rl_policy._timing_late_grace(dt)
    assert grace == pytest.approx(0.002)
    assert rl_policy._timing_trip_reason(
        "walk", 1, hz, grace * 0.5, 0) is None
    assert "missed the 100 Hz deadline" in rl_policy._timing_trip_reason(
        "walk", 2, hz, dt * rl_policy.TIMING_HARD_LAG_FRAC, 1)
    assert "consecutive" in rl_policy._timing_trip_reason(
        "walk", 3, hz, grace * 1.1,
        rl_policy.TIMING_MAX_CONSECUTIVE_LATE)


def test_runner_stand_obs_layout():
    """68-wide stand obs: 59 proprio dims + the 9-dim TaskGoal block
    ([roll,pitch]/tilt_scale, height/height_scale, 6-wide one-hot)."""
    rng = np.random.default_rng(1)
    state = SimpleNamespace(
        joint_position=rng.normal(0, 0.3, 18),
        joint_velocity=rng.normal(0, 0.5, 18),
        imu_roll=0.02, imu_pitch=-0.01,
        imu_gyro=rng.normal(0, 0.3, 3),
    )
    q_nom = rng.normal(0, 0.2, 18)
    prev = rng.uniform(-1, 1, 18)
    meta = _live_meta()
    prof = rl_policy.policy_profile(SimpleNamespace(meta=meta), "stand")
    t = prof["hold_s"] + prof["ramp_s"]          # ramp done: full target
    goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0,
                    height_ref=rl_policy._height_ref(prof, t),
                    unload_leg=None)
    obs = build_obs(CFG, state, q_nom, prev, goal=goal,
                    tilt_ref=(0.0, 0.0))
    assert obs.shape == (68,) and meta["obs_dim"] == 68
    hs = float(CFG["obs"].get("height_scale_m", 0.05))
    np.testing.assert_allclose(obs[61], prof["target_m"] / hs, rtol=1e-5)
    np.testing.assert_array_equal(obs[62:68], np.zeros(6))  # no unload
    np.testing.assert_allclose(
        obs[41:59], prev.astype(np.float32))     # prev-action block


def test_numpy_parity_with_source_checkpoint():
    sb3 = pytest.importorskip("stable_baselines3")
    if not SOURCE_ZIP.is_file():
        pytest.skip("source checkpoint not on this machine")
    model = sb3.PPO.load(SOURCE_ZIP, device="cpu")
    pol = rl_policy.NumpyPolicy(LIVE)
    rng = np.random.default_rng(2)
    worst = 0.0
    for _ in range(50):
        obs = rng.normal(0, 1, 68).astype(np.float32)
        a_np = pol.act(obs)
        a_sb3, _ = model.predict(obs, deterministic=True)
        worst = max(worst, float(np.max(np.abs(a_np - a_sb3))))
    assert worst < 1e-5, f"deployed numpy forward drifted: {worst:.2e}"
