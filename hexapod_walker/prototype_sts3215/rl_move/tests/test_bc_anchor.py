"""BC-anchor spec tests (rise lever (a), RL_PLAN queue 2a, 08-11).

What must be true BEFORE any training run uses train.bc_anchor_coef:

1. default-off exactness — no cfg key, no bc_target in info, stock
   PPO class untouched;
2. the env emits an aligned target — bc_target is the normalized
   action for the reference pose one ref-tick ahead of the episode's
   live ref clock (RSI and legacy ramp-aligned episodes both);
3. the targets actually climb the path — chaining bc_target actions
   from an RSI spawn tracks the reference (small joint RMS), i.e. the
   anchor teaches the demonstrated rise, not garbage;
4. the trainer aux step pulls pi_mean toward the buffered targets and
   the collect callback never pairs a done step's info with the next
   episode's reset obs;
5. misconfiguration fails LOUD (bc coef without rise_ref_path refuses
   to build) — the pool-restore lesson: quiet no-ops are how three
   weeks of stand verdicts got confounded.

08-11 (RL_PLAN queue 2.3): hold/track ticks emit too, target = the
pose the episode actually settled at (``env._q_nom``, constant for
the whole episode — no moving reference to chase, unlike rise).
Covered here: emission + value on hold and track ticks, and that
rise/hold/track never emit for each other's mode.

The rise reward bank (test_task_semantics.py) is untouched by design:
the anchor is a trainer loss, not a reward term.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD, RAD2DEG  # noqa: E402
from rl_move.sim.joint_task import (  # noqa: E402
    SimHexapodJointGoalEnv, action_to_q_rad, q_rad_to_action)
from rl_move.sim.servo_model import SimServoParams  # noqa: E402
from rl_move.sim.sim_env import load_rise_ref  # noqa: E402

RISE_REF = "rl_move/sim/refs/rise_ref_belly2plant.npz"
BASE_OVERRIDES = {
    ("actions", "max_height_mm"): 115.0,
    ("goal", "rise_height_mm"): [108.0, 114.0],
    ("goal", "rise_ramp_s"): 6.0,
    ("reward", "k_rise_ref_track"): 2.0,
    ("reward", "rise_ref_path"): RISE_REF,
    ("reward", "rise_posture_gate"): 1.0,
    ("reward", "rise_income_prog_gate"): 1.0,
    ("reward", "rise_finish_gate_signed"): 1.0,
}


def _make_env(seed: int, extra=None,
             only_mode: str = "rise") -> SimHexapodJointGoalEnv:
    cfg = load_config()
    ov = dict(BASE_OVERRIDES)
    ov.update(extra or {})
    for (sec, leaf), val in ov.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == only_mode else 0.0)
    gen.force_rise_start = "flat"
    return env


def test_inverse_action_map_roundtrip():
    rng = np.random.default_rng(0)
    a = rng.uniform(-1.0, 1.0, size=(64, 18))
    back = np.array([q_rad_to_action(action_to_q_rad(x)) for x in a])
    assert np.allclose(back, a, atol=1e-9)


def test_default_off_no_target():
    env = _make_env(0)
    env.reset()
    for _ in range(5):
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info


def test_emission_and_alignment_rsi():
    env = _make_env(1, {("train", "bc_anchor_coef"): 1.0,
                        ("goal", "rise_rsi_frac"): 1.0})
    env.reset()
    assert env._rsi_ref_tick0 is not None, "RSI spawn did not engage"
    ref = load_rise_ref(str(ROOT / RISE_REF))
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    for _ in range(3):
        _o, _r, _t, _tr, info = env.step(hold)
        t = info.get("bc_target")
        assert t is not None and t.shape == (18,) and t.dtype == np.float32
        j, is_rsi = env._rise_ref_clock(ref)
        assert is_rsi
        jn = min(j + max(int(round(env.dt / ref["dt"])), 1),
                 len(ref["q"]) - 1)
        assert np.allclose(t, q_rad_to_action(ref["q"][jn]), atol=1e-6)


def test_emission_legacy_ramp_aligned():
    env = _make_env(2, {("train", "bc_anchor_coef"): 1.0})
    env.reset()
    assert env._rsi_ref_tick0 is None
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    _o, _r, _t, _tr, info = env.step(hold)
    assert "bc_target" in info


def test_bc_targets_track_the_reference():
    """Chaining emitted targets from an RSI spawn stays ON the path."""
    env = _make_env(3, {("train", "bc_anchor_coef"): 1.0,
                        ("goal", "rise_rsi_frac"): 1.0})
    env.reset()
    ref = load_rise_ref(str(ROOT / RISE_REF))
    act = q_rad_to_action(ref["q"][env._rsi_ref_tick0])
    worst = 0.0
    for _ in range(60):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc:
            break
        j, _ = env._rise_ref_clock(ref)
        rms = float(np.sqrt(np.mean(
            (env.data.qpos[env._qadr] - ref["q"][j]) ** 2))) * RAD2DEG
        worst = max(worst, rms)
        act = info["bc_target"]
    assert worst < 8.0, f"target chain drifted off the path ({worst:.1f}deg)"


def test_state_aligned_crouch_start_fixes_the_bleed():
    """The crouchrise1/2/3 + holdload1 mechanism, pinned: a NON-RSI
    crouch start time-aligns the legacy anchor clock at the BELLY ramp
    start (_rise_ref_clock), so a robot sitting plant-adjacent is
    supervised toward early-path lifted-leg poses — the obs->action
    association that bleeds into hold (holdload1 kept the legs-1+4
    park at a measured 4x hold-income loss, so the pose is taught,
    not paid for). The crouch pose is far off-path EVERYWHERE (the
    reference is a belly rise; probe: nearest point 63deg RMS, and it
    is the path END), so the correct supervision is the planted tail:
    train.bc_anchor_state_aligned=1 must emit it, legacy must not."""
    def target_rms_to_plant_deg(extra) -> float:
        env = _make_env(8, {("train", "bc_anchor_coef"): 1.0, **extra})
        env._goal_gen.force_rise_start = "crouch"
        env.reset()
        assert env._rsi_ref_tick0 is None, "want the non-RSI clock path"
        hold = q_rad_to_action(env.data.qpos[env._qadr])
        _o, _r, _t, _tr, info = env.step(hold)
        ref = load_rise_ref(str(ROOT / RISE_REF))
        d = action_to_q_rad(info["bc_target"]) - ref["q"][-1]
        env.close()
        return float(np.sqrt(np.mean(d ** 2))) * RAD2DEG
    legacy = target_rms_to_plant_deg({})
    aligned = target_rms_to_plant_deg(
        {("train", "bc_anchor_state_aligned"): 1.0})
    assert legacy > 20.0, (
        f"crouch-start legacy target is already the planted tail "
        f"({legacy:.1f}deg away) — the documented bleed is gone; "
        f"re-justify the state-aligned mode")
    assert aligned < 3.0, (
        f"state-aligned crouch target is {aligned:.1f}deg from the "
        f"planted tail — not anchoring plant-adjacent states to the "
        f"plant")


def test_state_aligned_chain_climbs_to_the_plant():
    """From an RSI mid-path spawn (poses there are distinct, unlike
    the near-identical belly-curl prefix where nearest-neighbor
    indices are ambiguous), chaining state-aligned targets must walk
    the reference FORWARD while staying on-path — supervision points
    up the path from wherever the robot actually is."""
    env = _make_env(9, {("train", "bc_anchor_coef"): 1.0,
                        ("train", "bc_anchor_state_aligned"): 1.0,
                        ("goal", "rise_rsi_frac"): 1.0})
    env.reset()
    assert env._rsi_ref_tick0 is not None, "RSI spawn did not engage"
    ref = load_rise_ref(str(ROOT / RISE_REF))

    def j_state() -> int:
        q_now = np.asarray(env.data.qpos[env._qadr], dtype=float)
        return int(np.argmin(
            ((ref["q"] - q_now[None, :]) ** 2).mean(axis=1)))

    j0 = j_state()
    worst_on_path = 0.0
    act = q_rad_to_action(env.data.qpos[env._qadr])
    for _ in range(60):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc or "bc_target" not in info:
            break
        act = info["bc_target"]
        j = j_state()
        q_now = np.asarray(env.data.qpos[env._qadr], dtype=float)
        rms = float(np.sqrt(np.mean(
            (q_now - ref["q"][j]) ** 2))) * RAD2DEG
        worst_on_path = max(worst_on_path, rms)
    j_end = j_state()
    env.close()
    assert j_end > j0 + 20, (
        f"chain barely advanced along the path ({j0} -> {j_end})")
    # Looser than the legacy chain's 8deg: lookahead pursuit cuts
    # corners by up to the lookahead distance (measured 8.4deg peak).
    assert worst_on_path < 12.0, (
        f"chain drifted off the path ({worst_on_path:.1f}deg)")


def test_emission_hold_mode():
    """Hold ticks emit a CONSTANT target = the settled episode start
    pose (env._q_nom) — no moving reference to chase, unlike rise."""
    env = _make_env(4, {("train", "bc_anchor_coef"): 1.0},
                    only_mode="hold")
    env.reset()
    assert env._is_hold_bc and not env._is_rise
    expected = q_rad_to_action(env._q_nom)
    for _ in range(3):
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
        t = info.get("bc_target")
        assert t is not None and t.shape == (18,) and t.dtype == np.float32
        assert np.allclose(t, expected, atol=1e-6)


def test_emission_track_mode():
    """Track ticks emit the same way as hold (attitude tracking moves
    the reference torso pose, not the leg-hold target)."""
    env = _make_env(5, {("train", "bc_anchor_coef"): 1.0},
                    only_mode="track")
    env.reset()
    assert env._is_hold_bc and not env._is_rise
    expected = q_rad_to_action(env._q_nom)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert np.allclose(info["bc_target"], expected, atol=1e-6)


def test_no_hold_target_when_coef_zero():
    env = _make_env(6, only_mode="hold")   # bc_anchor_coef defaults 0
    env.reset()
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info


def test_emission_carries_mode_tags():
    """Every emitted target carries its mode tag (rise 0, hold/track 1,
    lower 2) — the stratified sampler's stratum key. Untagged legacy
    pairs default to 0 in the callback, so the tag is load-bearing
    only when stratified sampling is on."""
    cases = (("rise", 0), ("hold", 1), ("track", 1), ("lower", 2))
    for mode, want in cases:
        extra = {("train", "bc_anchor_coef"): 1.0}
        if mode == "lower":
            extra[("train", "bc_anchor_lower")] = 1.0
        env = _make_env(13, extra, only_mode=mode)
        env.reset()
        hold = q_rad_to_action(env.data.qpos[env._qadr])
        _o, _r, _t, _tr, info = env.step(hold)
        assert "bc_target" in info, f"{mode} tick emitted no target"
        assert info.get("bc_mode") == want, (
            f"{mode} tick tagged bc_mode={info.get('bc_mode')}, "
            f"want {want}")
        env.close()


def test_stratified_sampling_balances_modes():
    """The loweranchor1 dilution mechanism, pinned: with uniform
    sampling a mode's gradient share tracks its emission share (a
    50:1 buffer imbalance gives the rare mode ~2% of the minibatch);
    with bc_stratified each mode present draws an equal quota."""
    model = _tiny_model(coef=1.0)
    rng_obs = np.random.default_rng(0)
    for _ in range(1000):
        model._bc_push(rng_obs.uniform(size=12).astype(np.float32),
                       np.zeros(18, dtype=np.float32), mode=1)
    for _ in range(20):
        model._bc_push(rng_obs.uniform(size=12).astype(np.float32),
                       np.ones(18, dtype=np.float32), mode=2)
    rng = np.random.default_rng(7)
    model.bc_stratified = False
    idx = model._bc_sample_idx(rng, model._bc_n, 512)
    rare_uniform = float(np.mean(model._bc_mode[idx] == 2))
    assert rare_uniform < 0.10, (
        f"uniform sampling gives the rare mode {rare_uniform:.0%} — "
        f"the dilution premise itself is broken")
    model.bc_stratified = True
    idx = model._bc_sample_idx(rng, model._bc_n, 512)
    rare_strat = float(np.mean(model._bc_mode[idx] == 2))
    assert 0.40 <= rare_strat <= 0.60, (
        f"stratified sampling gives the rare mode {rare_strat:.0%}, "
        f"want ~50%")
    assert len(idx) == 512


def test_lower_anchor_default_off():
    """Lower ticks emit NOTHING unless train.bc_anchor_lower opts in —
    coef alone must not change any existing run's data stream."""
    env = _make_env(10, {("train", "bc_anchor_coef"): 1.0},
                    only_mode="lower")
    env.reset()
    assert env._is_lower_bc
    for _ in range(3):
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info
    env.close()


def test_lower_anchor_emits_the_ik_descent():
    """With train.bc_anchor_lower=1 every lower tick's target is the
    honest demonstration the LOWER bank scripts: FixedFootBodyIK
    anchored at the settled stance, body at the NEXT commanded height
    — the fix the lower-bank strict xfail prescribes ('strengthen the
    pricing (or BC-anchor lower ticks)') for the one-leg-aloft gap."""
    from rl_move.body_ik import BodyOffset, FixedFootBodyIK
    env = _make_env(11, {("train", "bc_anchor_coef"): 1.0,
                         ("train", "bc_anchor_lower"): 1.0},
                    only_mode="lower")
    env.reset()
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    for _ in range(5):
        _o, _r, _t, _tr, info = env.step(hold)
        t = info.get("bc_target")
        assert t is not None and t.shape == (18,) and t.dtype == np.float32
        ik = FixedFootBodyIK()
        ik.reset(env._q_nom)
        res = ik.solve(BodyOffset(
            height=float(env._goal_traj.at(env._step_i + 1).height_ref)))
        assert res.ok
        assert np.allclose(t, q_rad_to_action(res.q_rad), atol=1e-6)
    env.close()


def test_lower_anchor_chain_descends_with_feet_planted():
    """Following the lower targets must produce the honest behavior
    class: body tracks the commanded descent while every foot stays
    at its grounded reference — no aloft/outrigger residue."""
    env = _make_env(12, {("train", "bc_anchor_coef"): 1.0,
                         ("train", "bc_anchor_lower"): 1.0},
                    only_mode="lower")
    env.reset()
    z_start = float(env.data.xpos[env._chassis_bid, 2])
    target_m = float(env._h_target)
    assert target_m < 0.0, "lower episode must command a descent"
    act = q_rad_to_action(env.data.qpos[env._qadr])
    worst_clear_mm = 0.0
    for _ in range(300):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc:
            assert not term, "the anchored descent fell over"
            break
        if "bc_target" in info:
            act = info["bc_target"]
        clear = max(float(env.data.xpos[b, 2]) - env._pad_z_ref[i]
                    for i, b in enumerate(env._pad_bids))
        worst_clear_mm = max(worst_clear_mm, clear * 1000.0)
    z_end = float(env.data.xpos[env._chassis_bid, 2])
    dropped = z_start - z_end
    env.close()
    assert dropped > 0.6 * abs(target_m), (
        f"chain only descended {dropped*1000:.0f}mm of the commanded "
        f"{abs(target_m)*1000:.0f}mm")
    assert worst_clear_mm < 25.0, (
        f"a foot came {worst_clear_mm:.0f}mm off the ground during the "
        f"anchored descent — not the honest feet-planted class")


def test_rise_and_hold_flags_mutually_exclusive():
    for mode in ("rise", "hold", "track", "lower"):
        env = _make_env(7, only_mode=mode)
        env.reset()
        assert env._is_rise == (mode == "rise")
        assert env._is_hold_bc == (mode in ("hold", "track"))


import gymnasium as _gym  # noqa: E402


class _DummyEnv(_gym.Env):
    """Minimal gym env: obs 12, act 18 (matches N_ACT)."""

    def __init__(self):
        super().__init__()
        self.observation_space = _gym.spaces.Box(-1, 1, (12,), np.float32)
        self.action_space = _gym.spaces.Box(-1, 1, (18,), np.float32)
        self._rng = np.random.default_rng(0)

    def reset(self, *, seed=None, options=None):
        return self._rng.uniform(-1, 1, 12).astype(np.float32), {}

    def step(self, action):
        return (self._rng.uniform(-1, 1, 12).astype(np.float32),
                0.0, False, False, {})


def _tiny_model(coef=5.0):
    from rl_move.sim.bc_anchor import make_bc_anchor_ppo_class
    cls = make_bc_anchor_ppo_class()
    model = cls("MlpPolicy", _DummyEnv(), n_steps=16, batch_size=16,
                n_epochs=1, learning_rate=1e-3, seed=0, device="cpu",
                policy_kwargs=dict(net_arch=[32]))
    model.bc_coef = coef
    model.bc_minibatches = 8
    model.bc_batch_size = 64
    model.bc_buffer_cap = 256
    return model


def test_trainer_aux_step_moves_policy_toward_targets():
    import torch
    model = _tiny_model()
    model.learn(total_timesteps=32)      # fills the rollout buffer
    rng = np.random.default_rng(1)
    obs = rng.uniform(-1, 1, (128, 12)).astype(np.float32)
    tgt = np.tile(np.linspace(-0.5, 0.5, 18, dtype=np.float32), (128, 1))
    for o, t in zip(obs, tgt):
        model._bc_push(o, t)

    def mse():
        with torch.no_grad():
            m = model.policy.get_distribution(
                torch.as_tensor(obs)).distribution.mean.numpy()
        return float(np.mean((m - tgt) ** 2))

    before = mse()
    for _ in range(5):
        model.train()
    assert mse() < before, "aux step did not move pi_mean toward targets"


def test_collect_callback_skips_done_pairs():
    from rl_move.sim.bc_anchor import make_bc_collect_callback
    model = _tiny_model()
    cb = make_bc_collect_callback()
    cb.model = model
    t = np.ones(18, dtype=np.float32)
    cb.locals = {
        "infos": [{"bc_target": t}, {"bc_target": t}, {}],
        "new_obs": np.zeros((3, 12), dtype=np.float32),
        "dones": np.array([False, True, False]),
    }
    cb._on_step()
    assert model._bc_n == 1      # done idx1 skipped, idx2 has no target


def test_buffer_wraparound():
    model = _tiny_model()
    model.bc_buffer_cap = 32
    for k in range(100):
        model._bc_push(np.full(12, k, np.float32), np.zeros(18, np.float32))
    assert model._bc_n == 32 and model._bc_obs.shape[0] == 32


def test_attach_refuses_without_ref_path():
    from rl_move.sim.bc_anchor import attach_bc_anchor
    model = _tiny_model()
    with pytest.raises(SystemExit):
        attach_bc_anchor(model, coef=1.0, cfg={}, task="joint_goal")
    with pytest.raises(SystemExit):
        attach_bc_anchor(
            model, coef=1.0,
            cfg={"reward": {"rise_ref_path": RISE_REF}}, task="joint_lower")


def test_attach_accepts_walk_task_without_ref_path():
    """The walk anchor's reference is the scripted TripodGait — code,
    not a recorded file; joint_walk must attach with no rise_ref_path
    (08-11, probe_walk_income follow-up)."""
    from rl_move.sim.bc_anchor import attach_bc_anchor
    model = _tiny_model()
    attach_bc_anchor(model, coef=1.0, cfg={}, task="joint_walk")
    assert model.bc_coef == 1.0


def test_composes_with_mirror_class():
    from rl_move.sim.bc_anchor import make_bc_anchor_ppo_class
    from rl_move.sim.mirror import make_mirror_ppo_class
    cls = make_bc_anchor_ppo_class(make_mirror_ppo_class())
    assert cls.__name__ == "BCAnchorPPO"


# ---------------------------------------------------------------------------
# WALK BC-anchor (08-11, RL_PLAN queue 2.1 / probe_walk_income follow-up):
# commanded walk ticks emit the command-conditioned scripted TripodGait
# pose one tick ahead; zero-command (stop) ticks emit NOTHING (the gait
# marches in place at v=0 — standing still is the commanded behavior).

WALK_OVERRIDES = {
    ("goal", "walk_speed_min_m_s"): 0.05,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_heading_max_rad"): 3.14159,
}


def _make_walk_env(seed: int, extra=None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    ov = dict(WALK_OVERRIDES)
    ov.update(extra or {})
    for (sec, leaf), val in ov.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _pin_walk_cmd(env, vx: float, vy: float) -> None:
    traj = env._goal_traj
    traj.vx[:] = vx
    traj.vy[:] = vy
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0


def test_walk_emission_matches_scripted_gait():
    """Commanded walk ticks emit the TripodGait pose one tick ahead —
    verified against an independent gait instance driven identically."""
    from tripod_gait import TripodGait
    env = _make_walk_env(0, {("train", "bc_anchor_coef"): 1.0})
    env.reset()
    assert env._walk_bc_gait is not None
    _pin_walk_cmd(env, 0.055, 0.0)
    ref = TripodGait(vx=0.0)
    ref.sync_plant_stance(20.0, 80.0)
    ref.reset_phase()
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    for _ in range(10):
        step_i = env._step_i
        _o, _r, term, trunc, info = env.step(hold)
        if term or trunc:
            break
        t = info.get("bc_target")
        assert t is not None and t.shape == (18,) and t.dtype == np.float32
        g = env._current_goal()
        ref.set_velocity(vx=float(g.vx_ref), vy=float(g.vy_ref), omega=0.0)
        expect = q_rad_to_action(
            np.asarray(ref.desired_deg((step_i + 1) * env.dt)) * DEG2RAD)
        assert np.allclose(t, expect, atol=1e-6)


def test_walk_target_is_a_gait_not_a_pose():
    """Across a second of commanded ticks the target must CYCLE (this
    is the whole point — a constant target cannot teach stepping)."""
    env = _make_walk_env(1, {("train", "bc_anchor_coef"): 1.0})
    env.reset()
    _pin_walk_cmd(env, 0.055, 0.0)
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    targets = []
    for _ in range(25):
        _o, _r, term, trunc, info = env.step(hold)
        if term or trunc:
            break
        if "bc_target" in info:
            targets.append(info["bc_target"])
    targets = np.asarray(targets)
    assert len(targets) >= 20
    spread = np.ptp(targets, axis=0).max()
    assert spread > 0.05, f"walk bc_target barely moves ({spread:.4f})"


def test_walk_direction_conditions_the_target():
    """Opposite commands must produce different gait targets at the
    same tick — the anchor is command-conditioned by construction."""
    outs = []
    for vx in (0.055, -0.055):
        env = _make_walk_env(2, {("train", "bc_anchor_coef"): 1.0})
        env.reset()
        _pin_walk_cmd(env, vx, 0.0)
        hold = q_rad_to_action(env.data.qpos[env._qadr])
        seq = []
        for _ in range(12):
            _o, _r, _t, _tr, info = env.step(hold)
            if "bc_target" in info:
                seq.append(info["bc_target"])
        outs.append(np.asarray(seq))
    n = min(len(outs[0]), len(outs[1]))
    assert n >= 8
    diff = np.abs(outs[0][:n] - outs[1][:n]).max()
    assert diff > 0.02, "forward and backward targets are identical"


def test_walk_no_target_on_stop_ticks():
    """Zero-command ticks emit NOTHING: TripodGait at v=0 marches in
    place, but the commanded behavior on a stop segment is standing
    still — anchoring a march there would fight the task."""
    env = _make_walk_env(3, {("train", "bc_anchor_coef"): 1.0})
    env.reset()
    _pin_walk_cmd(env, 0.0, 0.0)
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    for _ in range(5):
        _o, _r, _t, _tr, info = env.step(hold)
        assert "bc_target" not in info


def test_walk_no_gait_instance_when_coef_zero():
    env = _make_walk_env(4)   # bc_anchor_coef defaults 0
    env.reset()
    assert env._walk_bc_gait is None
    _pin_walk_cmd(env, 0.055, 0.0)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info


def test_walk_gait_attr_rides_snapshot_list():
    """Pool-restore lesson (commit 65edba7): the per-episode gait
    instance carries phase state read on every walk tick — it MUST be
    in mjx_host.SNAP_ATTRS or pooled episodes silently lose the
    anchor."""
    from rl_move.sim.mjx_host import SNAP_ATTRS
    assert "_walk_bc_gait" in SNAP_ATTRS
