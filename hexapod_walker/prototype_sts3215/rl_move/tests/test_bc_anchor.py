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
    # 2.4 s of chaining (60 ticks at the legacy 25 Hz). Chain progress
    # is slew-limited at a rate-invariant 37.5 deg/s (safety.
    # max_delta_q_deg scales with control.hz), so the budget must be
    # expressed in SECONDS: at the 100 Hz default (08-24 flip) a fixed
    # 60-tick loop is only 0.6 s and the chain "barely advances" (+6
    # rows, exactly 4x short — the 08-25 red-test root cause).
    for _ in range(int(round(2.4 / env.dt))):
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


def test_push_backfills_mode_on_legacy_warm_start():
    """Checkpoints trained before the mode tag pickle _bc_obs/_bc_act
    into the zip, so on warm-start hasattr(_bc_obs) is True and the
    lazy init never runs — _bc_mode must be backfilled on the first
    push (bit cw-stand-anchormix1's first launch: AttributeError on
    tick one)."""
    model = _tiny_model(coef=1.0)
    model._bc_push(np.zeros(12, dtype=np.float32),
                   np.zeros(18, dtype=np.float32))
    del model._bc_mode                     # emulate the legacy pickle
    model._bc_push(np.zeros(12, dtype=np.float32),
                   np.ones(18, dtype=np.float32), mode=2)
    assert model._bc_mode.shape[0] == model._bc_obs.shape[0]
    assert model._bc_mode[model._bc_i - 1] == 2


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
    # 12 s of descent (300 ticks at the legacy 25 Hz) — seconds, not
    # ticks: the command slew is rate-invariant deg/s (see the
    # state-aligned chain test's note on the 08-24 control.hz flip).
    for _ in range(int(round(12.0 / env.dt))):
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


def test_per_mode_anchor_loss_logged():
    """Per-mode diagnostic loss (08-12, the pre-registered
    observability gate before any further stand arm): after train(),
    every mode present in the ring gets train/bc_anchor_loss_<mode> +
    train/bc_anchor_fill_<mode>, and absent modes get nothing."""
    model = _tiny_model()
    model.learn(total_timesteps=32)
    rng = np.random.default_rng(2)
    for _ in range(64):
        model._bc_push(rng.uniform(-1, 1, 12).astype(np.float32),
                       np.zeros(18, dtype=np.float32), mode=0)
    for _ in range(16):
        model._bc_push(rng.uniform(-1, 1, 12).astype(np.float32),
                       np.ones(18, dtype=np.float32), mode=1)
    model.train()
    vals = model.logger.name_to_value
    assert "train/bc_anchor_loss_rise" in vals
    assert "train/bc_anchor_loss_hold" in vals
    assert vals["train/bc_anchor_fill_rise"] == 64
    assert vals["train/bc_anchor_fill_hold"] == 16
    assert "train/bc_anchor_loss_lower" not in vals
    assert "train/bc_anchor_loss_walk" not in vals
    # hold targets are all-ones vs rise all-zeros: the two per-mode
    # losses must actually discriminate (not read the same buffer).
    assert vals["train/bc_anchor_loss_hold"] != \
        vals["train/bc_anchor_loss_rise"]


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
    from sim_gait_compat import TripodGait
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


def test_walk_anchor_default_on_bit_exact():
    """bc_anchor_walk defaults to 1.0 (on) — an existing config that
    never mentions the new key keeps emitting exactly as before."""
    env = _make_walk_env(5, {("train", "bc_anchor_coef"): 1.0})
    env.reset()
    assert env._walk_bc_gait is not None
    _pin_walk_cmd(env, 0.055, 0.0)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" in info


def test_walk_anchor_opt_out():
    """train.bc_anchor_walk=0 with coef>0 anchors rise/hold/lower but
    NOT walk (08-12, cw-arch-gru-anchor1 follow-up: walk-tick anchoring
    freezes locomotion even though it protects stance skills)."""
    env = _make_walk_env(6, {("train", "bc_anchor_coef"): 1.0,
                             ("train", "bc_anchor_walk"): 0.0})
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


# --- GETUP lever (08-12, cw-getup2-r1 follow-up) ---------------------
# cw-getup2-r1 warm-started the getup task from the rise+hold
# specialist and showed the skill does NOT survive contact with the
# getup reward (env/getup_S declined over 2M steps back toward the
# from-scratch cw-getup1 collapse). This reuses the rise reference
# demo, ALWAYS state-aligned (getup starts are arbitrary), cfg-gated
# by train.bc_anchor_getup (default 0 = off, bit-exact when off).

GETUP_OVERRIDES = {
    ("safety", "max_roll_deg"): 60.0,
    ("safety", "max_pitch_deg"): 60.0,
    ("reward", "rise_ref_path"): RISE_REF,
}


def _make_getup_env(seed: int, extra=None, start: str = "crouch"):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    ov = dict(GETUP_OVERRIDES)
    ov.update(extra or {})
    for (sec, leaf), val in ov.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_getup = 1.0
    env.force_getup_start = start
    env.force_getup_cmd = (0.0, 0.0)
    return env


def test_getup_anchor_default_off_no_target():
    """coef alone (no bc_anchor_getup) must emit nothing on getup
    ticks — the existing rise/hold/lower/walk data streams are
    untouched by this lever."""
    env = _make_getup_env(20, {("train", "bc_anchor_coef"): 1.0})
    env.reset()
    assert env._is_getup
    for _ in range(5):
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info
    env.close()


def test_getup_anchor_no_target_when_coef_zero():
    """bc_anchor_getup alone (coef=0, the trainer's own kill switch)
    must also emit nothing."""
    env = _make_getup_env(21, {("train", "bc_anchor_getup"): 1.0})
    env.reset()
    for _ in range(5):
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info
    env.close()


def test_getup_anchor_emits_state_aligned_target():
    """With both knobs set, every getup tick's target is the reference
    pose nearest the CURRENT joints, one lookahead ahead — the exact
    state-aligned indexing the rise lever's anchorstate1/2 proved,
    never a fixed-time clock index (no live clock exists for an
    arbitrary getup start)."""
    env = _make_getup_env(22, {("train", "bc_anchor_coef"): 1.0,
                               ("train", "bc_anchor_getup"): 1.0},
                          start="crouch")
    env.reset()
    ref = load_rise_ref(str(ROOT / RISE_REF))
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    for _ in range(3):
        _o, _r, _t, _tr, info = env.step(hold)
        t = info.get("bc_target")
        assert t is not None and t.shape == (18,) and t.dtype == np.float32
        assert info.get("bc_mode") == 4
        qnow = np.asarray(env.data.qpos[env._qadr], dtype=float)
        j = int(np.argmin(((ref["q"] - qnow[None, :]) ** 2).mean(axis=1)))
        ahead = max(int(round(0.25 / ref["dt"])), 1)
        jn = min(j + ahead, len(ref["q"]) - 1)
        assert np.allclose(t, q_rad_to_action(ref["q"][jn]), atol=1e-6)
    env.close()


def test_getup_anchor_mode_tag_distinct_from_rise():
    """The getup tag (4) must never collide with rise/hold/lower/walk
    (0/1/2/3) — stratified sampling and the per-mode loss logging both
    key off this integer."""
    env = _make_getup_env(23, {("train", "bc_anchor_coef"): 1.0,
                               ("train", "bc_anchor_getup"): 1.0})
    env.reset()
    hold = q_rad_to_action(env.data.qpos[env._qadr])
    _o, _r, _t, _tr, info = env.step(hold)
    assert info.get("bc_mode") == 4
    env.close()


def test_getup_anchor_refuses_without_ref_path():
    """Misconfiguration fails LOUD (the pool-restore lesson): setting
    bc_anchor_getup with no rise_ref_path must never silently no-op."""
    from rl_move.sim.bc_anchor import attach_bc_anchor
    model = _tiny_model()
    with pytest.raises(SystemExit):
        attach_bc_anchor(
            model, coef=1.0,
            cfg={"train": {"bc_anchor_getup": 1.0}}, task="joint_walk")


def test_getup_anchor_accepts_with_ref_path():
    from rl_move.sim.bc_anchor import attach_bc_anchor
    model = _tiny_model()
    attach_bc_anchor(
        model, coef=1.0,
        cfg={"train": {"bc_anchor_getup": 1.0},
             "reward": {"rise_ref_path": RISE_REF}},
        task="joint_walk")
    assert model.bc_coef == 1.0


def test_recover_anchor_refuses_without_ref_path():
    from rl_move.sim.bc_anchor import attach_bc_anchor

    model = _tiny_model()
    with pytest.raises(SystemExit):
        attach_bc_anchor(
            model, coef=1.0,
            cfg={"train": {"bc_anchor_recover": 1.0}},
            task="joint_walk")


def test_recover_anchor_accepts_with_ref_path():
    from rl_move.sim.bc_anchor import attach_bc_anchor

    model = _tiny_model()
    attach_bc_anchor(
        model, coef=1.0,
        cfg={"train": {"bc_anchor_recover": 1.0},
             "reward": {"rise_ref_path": RISE_REF}},
        task="joint_walk")
    assert model.bc_coef == 1.0


# ---------------------------------------------------------------------------
# FOOT-HEIGHT anchor term (train.bc_anchor_foot_z, 08-12 park audit:
# the one-parked-foot hold habit is invisible to joint-space MSE — the
# parked policy's per-leg anchor loss matches the clean parent's on the
# same leg. These tests pin (1) the torch FK against body_ik's numpy
# FK, (2) default-off bit-exactness of the update, (3) that the term
# actually SEES a mm-scale park the joint MSE dilutes away.)

def test_foot_z_torch_fk_matches_body_ik():
    import torch
    from rl_move.body_ik import fk_all_feet
    from rl_move.sim.bc_anchor import _bc_foot_z
    from rl_move.sim.joint_task import action_to_q_rad
    rng = np.random.default_rng(7)
    acts = rng.uniform(-1, 1, (32, 18)).astype(np.float32)
    th_z = _bc_foot_z(torch.as_tensor(acts)).numpy()
    for k in range(32):
        np_z = fk_all_feet(action_to_q_rad(acts[k]))[:, 2]
        assert np.allclose(th_z[k], np_z, atol=1e-6), \
            f"torch FK z diverges from body_ik at row {k}"


def test_foot_z_default_off_is_bit_exact():
    """bc_anchor_foot_z=0 (default) must leave the update sequence
    byte-identical to a model that has no foot-z attribute at all."""
    import torch
    models = []
    for set_attr in (False, True):
        model = _tiny_model()
        if set_attr:
            model.bc_foot_z_coef = 0.0   # explicit off == absent
            model.bc_foot_z_mm = 10.0
        model.learn(total_timesteps=32)
        rng = np.random.default_rng(3)
        for _ in range(64):
            model._bc_push(rng.uniform(-1, 1, 12).astype(np.float32),
                           rng.uniform(-1, 1, 18).astype(np.float32))
        model.train()
        models.append(model)
    for p0, p1 in zip(models[0].policy.parameters(),
                      models[1].policy.parameters()):
        assert torch.equal(p0, p1), \
            "explicit foot_z=0 changed the update — must be bit-exact"


def test_foot_z_sees_the_park_joint_mse_misses():
    """A commanded one-leg hover of ~10 mm must cost the foot-z term
    >=50x more (relative to its clean-pose scale) than it costs the
    joint MSE — the measured blindness that motivated the term."""
    import torch
    from rl_move.body_ik import fk_all_feet
    from rl_move.sim.bc_anchor import _bc_foot_z
    from rl_move.sim.joint_task import action_to_q_rad, q_rad_to_action
    # A plant-like pose: yaw 0, hip 20 deg, knee 80 deg on all legs.
    q_plant = np.array([0.0, 20.0, 80.0] * 6) * DEG2RAD
    a_plant = q_rad_to_action(q_plant).astype(np.float32)
    # Park leg 1: lift hip a fraction of a degree at a time until the
    # commanded foot z rises ~10 mm above the plant reference.
    a_park = a_plant.copy()
    z_ref = fk_all_feet(q_plant)[1, 2]
    dq = 0.0
    while True:
        dq += 0.1 * DEG2RAD
        q = q_plant.copy()
        q[4] -= dq          # hip lift (hip negative = up per limits)
        z = fk_all_feet(q)[1, 2]
        if z - z_ref >= 0.010:
            break
        assert dq < 30 * DEG2RAD, "never reached a 10mm hover?"
    a_park = q_rad_to_action(q).astype(np.float32)
    th_plant = torch.as_tensor(a_plant[None])
    th_park = torch.as_tensor(a_park[None])
    joint_mse = float(((th_park - th_plant) ** 2).mean())
    dz = (_bc_foot_z(th_park) - _bc_foot_z(th_plant)) / 0.010  # 10mm scale
    fz = float(dz.pow(2).mean())
    assert fz > 0.1, f"foot-z term blind to a 10mm hover ({fz})"
    assert fz / max(joint_mse, 1e-12) >= 50, \
        f"foot-z/joint ratio only {fz / joint_mse:.1f} — term too weak"


# ---------------------------------------------------------------------------
# HEIGHT-FLOOR pursuit (train.bc_anchor_min_h_ahead_mm, 08-12
# cw-stand-footlow1 dig-in). Measured defect (probe_anchor_align on the
# live stalled policy): the reference crawls 0->25 mm over 5+ s
# (ticks ~126-250), so at a stalled ~7 mm belly state the +0.5 s TIME
# lookahead commands a pose only 1-5 mm higher; loaded-servo sag
# cancels it, the matched index pins (0 ticks advance over 3 s), and
# the anchor supervises the stall with a LOW converged loss. The floor
# requires the target tick to command >= min_h_ahead_mm above the
# chassis's current height.


def _floor_env(min_h_mm, seed=3, extra=None):
    ov = {("train", "bc_anchor_coef"): 1.0,
          ("train", "bc_anchor_state_aligned"): 1.0,
          ("train", "bc_anchor_lookahead_s"): 0.5,
          ("goal", "rise_rsi_frac"): 0.0}
    if min_h_mm is not None:
        ov[("train", "bc_anchor_min_h_ahead_mm")] = float(min_h_mm)
    ov.update(extra or {})
    return _make_env(seed, ov)


def test_min_h_ahead_default_off_bit_exact():
    """min_h_ahead absent and explicitly 0.0 emit byte-identical
    targets (the floor branch must not perturb the legacy pursuit)."""
    def targets(min_h):
        env = _floor_env(min_h)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        out = []
        for _ in range(40):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" in info:
                out.append(info["bc_target"].copy())
                act = info["bc_target"]
        env.close()
        return np.asarray(out)
    a = targets(None)
    b = targets(0.0)
    assert a.shape == b.shape and np.array_equal(a, b)


def test_min_h_ahead_targets_command_height_progress():
    """At a low state the legacy pursuit target commands almost no
    height gain (the measured stall supervision); with the floor every
    emitted target decodes to a reference tick at least min_h above
    the chassis's current height (or the path end)."""
    ref = load_rise_ref(str(ROOT / RISE_REF))
    T = len(ref["q"])

    def tick_of(target) -> int:
        q_t = action_to_q_rad(np.asarray(target, dtype=float))
        return int(np.argmin(
            ((ref["q"] - q_t[None, :]) ** 2).mean(axis=1)))

    for min_h, want_floor in ((0.0, False), (15.0, True)):
        env = _floor_env(min_h)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        low_gains = []
        for _ in range(80):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" not in info:
                continue
            h_now = float(env.data.xpos[env._chassis_bid, 2]) - env._z0
            k = tick_of(info["bc_target"])
            if h_now < 0.04:   # the plateau region under audit
                low_gains.append(
                    (float(ref["h"][k]) - h_now, k == T - 1))
            act = info["bc_target"]
        env.close()
        assert low_gains, "chain never visited a low state"
        if want_floor:
            bad = [g for g, is_end in low_gains
                   if g < 0.015 - 1e-6 and not is_end]
            assert not bad, (
                f"floor=15mm target(s) command only {min(bad)*1e3:.1f}mm "
                "above the current chassis height")
        else:
            # the defect this floor fixes: legacy pursuit at low
            # states commands <6mm of height gain at least once
            assert min(g for g, _ in low_gains) < 0.006, (
                "legacy pursuit no longer commands near-zero height "
                "gain at low states — re-justify the floor")


def test_min_h_ahead_unpins_the_plateau_traversal():
    """Chaining targets under LOADED servo params (the stalled run's
    own physics): the floor must traverse the low prep plateau several
    times faster — by tick 120 (4.8 s) the floored chain is high while
    the legacy chain is still in the plateau (measured 82.5 vs ~9 mm
    at t=50/100; policy-level the legacy pursuit pins outright)."""
    def h_at(min_h, seconds=4.8):
        env = _floor_env(min_h,
                         extra={("bus", "servo_params"): "loaded"})
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        h = 0.0
        # "tick 120" of the docstring = 4.8 s at the legacy 25 Hz;
        # seconds, not ticks (rate-invariant slew — see the
        # state-aligned chain test's note on the 08-24 control.hz flip).
        for _ in range(int(round(seconds / env.dt))):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" in info:
                act = info["bc_target"]
            h = float(env.data.xpos[env._chassis_bid, 2]) - env._z0
        env.close()
        return h * 1e3
    slow = h_at(0.0)
    fast = h_at(15.0)
    assert fast > 60.0, f"floored chain only reached {fast:.1f}mm"
    assert fast > slow + 30.0, (
        f"no traversal separation: floor {fast:.1f}mm vs "
        f"legacy {slow:.1f}mm")


# TUCK-EXEMPT floor (train.bc_anchor_min_h_tuck_exempt_i0, 08-25,
# tuckfloor0/-s1 FAIL-MECH follow-up). Removing the floor everywhere
# (tuckfloor0) collapsed BOTH the flat tuck AND the previously-clean
# press-phase starts into a new all-six-leg freeze — the floor's
# press-phase anti-stall role was load-bearing. This gate makes the
# floor a no-op while the matched index is still inside the
# reference's own tuck segment (< ramp_i0) and restores it unchanged
# at/after ramp_i0.


def test_tuck_exempt_default_off_bit_exact():
    """Absent/0.0 tuck-exempt must not perturb the existing floor
    pursuit at all, at any matched index — regression pin for the new
    branch's default path."""
    def targets(exempt):
        extra = {("train", "bc_anchor_min_h_ahead_mm"): 15.0}
        if exempt is not None:
            extra[("train", "bc_anchor_min_h_tuck_exempt_i0")] = exempt
        env = _floor_env(None, extra=extra)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        out = []
        for _ in range(80):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" in info:
                out.append(info["bc_target"].copy())
                act = info["bc_target"]
        env.close()
        return np.asarray(out)
    a = targets(None)
    b = targets(0.0)
    assert a.shape == b.shape and np.array_equal(a, b)


def test_tuck_exempt_skips_the_floor_inside_the_tuck_segment():
    """With the floor exempted before ramp_i0, chasing from a flat
    (tuck-segment) spawn must NOT jump straight to the press phase —
    the emitted target's matched reference tick should stay close to
    the current one (pure time-lookahead), unlike the legacy
    always-on floor which is proven elsewhere to jump the whole
    plateau in one tick."""
    ref = load_rise_ref(str(ROOT / RISE_REF))
    i0 = int(ref["ramp_i0"])

    def tick_of(target) -> int:
        q_t = action_to_q_rad(np.asarray(target, dtype=float))
        return int(np.argmin(
            ((ref["q"] - q_t[None, :]) ** 2).mean(axis=1)))

    env = _floor_env(15.0, extra={
        ("train", "bc_anchor_min_h_tuck_exempt_i0"): 1.0})
    env.reset()
    act = q_rad_to_action(env.data.qpos[env._qadr])
    first_tick = None
    for _ in range(5):
        _o, _r, term, trunc, info = env.step(act)
        assert not (term or trunc)
        if "bc_target" in info:
            first_tick = tick_of(info["bc_target"])
            break
    assert first_tick is not None
    assert first_tick < i0, (
        f"tuck-exempt floor still jumped to tick {first_tick} "
        f">= ramp_i0={i0} from a flat spawn")


def test_tuck_exempt_restores_the_floor_after_ramp_i0():
    """Once the matched index reaches/passes ramp_i0, tuck-exempt must
    behave exactly like the always-on floor (same anti-freeze
    traversal speed proven in test_min_h_ahead_unpins_the_plateau_
    traversal) — the exemption is tuck-only, not a global weakening."""
    def h_at(exempt, seconds=8.0):
        extra = {("bus", "servo_params"): "loaded"}
        if exempt is not None:
            extra[("train", "bc_anchor_min_h_tuck_exempt_i0")] = exempt
        env = _floor_env(15.0, extra=extra)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        h = 0.0
        for _ in range(int(round(seconds / env.dt))):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" in info:
                act = info["bc_target"]
            h = float(env.data.xpos[env._chassis_bid, 2]) - env._z0
        env.close()
        return h * 1e3
    # 8s clears the tuck (ramp_i0=126 ticks @ dt=0.04s = 5.04s of ref
    # time) with margin for both chains to reach the plateau.
    legacy = h_at(None)
    exempt = h_at(1.0)
    assert exempt > 60.0, (
        f"tuck-exempt floor only reached {exempt:.1f}mm past the tuck "
        "— the press-phase floor restore regressed")
    assert abs(exempt - legacy) < 20.0, (
        f"tuck-exempt ({exempt:.1f}mm) diverges from the always-on "
        f"floor ({legacy:.1f}mm) past the tuck segment")


# TUCK SCRIPT-INDEX floor (train.bc_anchor_tuck_lookahead_s, 08-25,
# tuckrise campaign dig-in). tuck_exempt turns the ACHIEVED-HEIGHT
# floor off inside the tuck, but plain time-lookahead alone through a
# height-flat, low-pose-delta tuck measured (tuckfloor0/tuckexempt0,
# 4/4 seeds) as a total freeze. This lever widens the lookahead itself
# while inside the tuck to a SCRIPT-INDEX offset from the current
# match (never from achieved height, so it cannot stick the way the
# height floor did).


def test_tuck_lookahead_default_off_bit_exact():
    """Absent/0.0 must not perturb the existing pursuit at all, at any
    matched index — regression pin for the new branch's default
    path."""
    def targets(tuck_ahead_s):
        extra = {("train", "bc_anchor_min_h_ahead_mm"): 15.0,
                 ("train", "bc_anchor_min_h_tuck_exempt_i0"): 1.0}
        if tuck_ahead_s is not None:
            extra[("train", "bc_anchor_tuck_lookahead_s")] = tuck_ahead_s
        env = _floor_env(None, extra=extra)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        out = []
        for _ in range(80):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" in info:
                out.append(info["bc_target"].copy())
                act = info["bc_target"]
        env.close()
        return np.asarray(out)
    a = targets(None)
    b = targets(0.0)
    assert a.shape == b.shape and np.array_equal(a, b)


def test_tuck_lookahead_widens_the_in_tuck_target_by_script_index():
    """With tuck_exempt=1 (floor off in-tuck) and a flat/tuck spawn,
    a non-zero tuck_lookahead_s must land the emitted target FURTHER
    ahead (by script index) than the bare lookahead does, and strictly
    inside the tuck for a dose small enough to stay there — a genuine
    script-progress jump, not the achieved-height floor's jump
    straight to ramp_i0."""
    ref = load_rise_ref(str(ROOT / RISE_REF))
    i0 = int(ref["ramp_i0"])

    def tick_of(target) -> int:
        q_t = action_to_q_rad(np.asarray(target, dtype=float))
        return int(np.argmin(
            ((ref["q"] - q_t[None, :]) ** 2).mean(axis=1)))

    def first_target_tick(tuck_ahead_s):
        extra = {("train", "bc_anchor_min_h_tuck_exempt_i0"): 1.0}
        if tuck_ahead_s is not None:
            extra[("train", "bc_anchor_tuck_lookahead_s")] = tuck_ahead_s
        env = _floor_env(15.0, extra=extra)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        tick = None
        for _ in range(5):
            _o, _r, term, trunc, info = env.step(act)
            assert not (term or trunc)
            if "bc_target" in info:
                tick = tick_of(info["bc_target"])
                break
        env.close()
        assert tick is not None
        return tick
    bare = first_target_tick(None)
    widened = first_target_tick(1.0)
    assert bare < i0, f"bare-lookahead tick {bare} already >= ramp_i0={i0}"
    assert widened < i0, (
        f"tuck_lookahead_s jumped clear of the tuck to tick {widened} "
        f">= ramp_i0={i0} — should stay a script-index offset, not "
        "an achieved-height floor jump")
    assert widened > bare, (
        f"tuck_lookahead_s={1.0}s target tick {widened} is not further "
        f"ahead than the bare-lookahead tick {bare}")


def test_tuck_lookahead_noop_past_the_tuck():
    """Once the matched index is at/after ramp_i0, tuck_lookahead_s
    must be a pure no-op (the branch is gated on `_bc_j < ramp_i0`) —
    same behavior as the always-on floor restored after the tuck."""
    def h_at(tuck_ahead_s, seconds=8.0):
        extra = {("bus", "servo_params"): "loaded",
                 ("train", "bc_anchor_min_h_tuck_exempt_i0"): 1.0}
        if tuck_ahead_s is not None:
            extra[("train", "bc_anchor_tuck_lookahead_s")] = tuck_ahead_s
        env = _floor_env(15.0, extra=extra)
        env.reset()
        act = q_rad_to_action(env.data.qpos[env._qadr])
        h = 0.0
        for _ in range(int(round(seconds / env.dt))):
            _o, _r, term, trunc, info = env.step(act)
            if term or trunc:
                break
            if "bc_target" in info:
                act = info["bc_target"]
            h = float(env.data.xpos[env._chassis_bid, 2]) - env._z0
        env.close()
        return h * 1e3
    bare = h_at(None)
    widened = h_at(1.0)
    assert widened > 60.0, (
        f"tuck_lookahead_s chain only reached {widened:.1f}mm past "
        "the tuck — the press-phase floor restore regressed")
    assert abs(widened - bare) < 20.0, (
        f"tuck_lookahead_s ({widened:.1f}mm) diverges from the bare "
        f"tuck-exempt chain ({bare:.1f}mm) past the tuck segment — "
        "the branch should be a no-op once past ramp_i0")


# ---------------------------------------------------------------------------
# TIP-AWARE HOLD REFERENCE (train.bc_anchor_tilt_comp, 08-13).
# cw-stand-footlow2-tip1's gate consequence: tipped-start DR under a
# tilt-BLIND anchor (constant q_nom target) teaches the policy to HOLD
# the lean — the anchor gives zero leveling gradient and joint-space
# MSE is attitude-blind. The hardware stance candidate stands with a
# persistent ~8deg lean. The tip-aware reference counter-rotates the
# measured relative attitude via FixedFootBodyIK so the anchor TEACHES
# proportional posture feedback. These tests pin: default-off
# bit-exactness, level ticks unchanged, the counter-rotation semantics
# (sign + magnitude, via foot_world_error against the trusted IK
# transform), the per-axis clip, track-mode exclusion, and end-to-end
# integration with the dr.tipped_start_* machinery.
# ---------------------------------------------------------------------------

def _tilt_env(seed, tilt_comp=None, only_mode="hold", extra=None):
    ov = {("train", "bc_anchor_coef"): 1.0}
    if tilt_comp is not None:
        ov[("train", "bc_anchor_tilt_comp")] = tilt_comp
    ov.update(extra or {})
    return _make_env(seed, ov, only_mode=only_mode)


def _shift_ref(env, roll_deg=0.0, pitch_deg=0.0):
    """Shift the episode tilt reference so the CURRENT (level) pose
    reads as a relative lean of (roll_deg, pitch_deg)."""
    env._tilt_ref0 = (env._tilt_ref0[0] - roll_deg * DEG2RAD,
                      env._tilt_ref0[1] - pitch_deg * DEG2RAD)
    env.safety.set_tilt_reference(*env._tilt_ref0)


def _soft(x, dead_deg=1.5):
    import math
    d = dead_deg * DEG2RAD
    return math.copysign(max(abs(x) - d, 0.0), x)


def _expected_corr(rel, cap_deg=6.0):
    cap = cap_deg * DEG2RAD
    return float(np.clip(-_soft(rel), -cap, cap))


def test_tilt_comp_default_off_is_bit_exact():
    """No tilt_comp key: an 8deg relative lean still anchors to the
    exact constant q_nom target (legacy behavior byte-for-byte)."""
    env = _tilt_env(30)
    env.reset()
    _shift_ref(env, roll_deg=8.0)
    expected = q_rad_to_action(env._q_nom)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert np.array_equal(info["bc_target"],
                          expected.astype(np.float32))


def test_tilt_comp_level_keeps_qnom():
    """tilt_comp on, chassis level (inside the 1.5deg soft deadband):
    target stays the constant q_nom pose."""
    env = _tilt_env(31, tilt_comp=1.0)
    env.reset()
    expected = q_rad_to_action(env._q_nom)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    r = abs(env._state.imu_roll - env._tilt_ref0[0]) * RAD2DEG
    p = abs(env._state.imu_pitch - env._tilt_ref0[1]) * RAD2DEG
    assert r < 1.5 and p < 1.5, "fixture not level enough for the test"
    assert np.allclose(info["bc_target"], expected, atol=1e-6)


def test_tilt_comp_counter_rotates():
    """A 5.5deg relative roll lean (soft-deadbanded to a 4deg
    correction — inside the cap, no action-bound saturation) anchors
    toward the pose that puts the body at MINUS the correction with
    the feet kept at the level q_nom anchors — checked with
    foot_world_error against the trusted IK transform (the wrong sign
    fails by an order of magnitude) — and the leaned-toward side's
    feet extend (body-frame z more negative)."""
    from rl_move.body_ik import (
        BodyOffset, fk_all_feet, foot_world_error)
    env = _tilt_env(32, tilt_comp=1.0)
    env.reset()
    _shift_ref(env, roll_deg=5.5)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    q_tgt = action_to_q_rad(info["bc_target"])
    assert not np.allclose(q_tgt, env._q_nom, atol=1e-4), \
        "tilt_comp emitted the legacy constant target"
    rel_r = env._state.imu_roll - env._tilt_ref0[0]
    rel_p = env._state.imu_pitch - env._tilt_ref0[1]
    feet_ref = fk_all_feet(env._q_nom)
    off_good = BodyOffset(roll=_expected_corr(rel_r),
                          pitch=_expected_corr(rel_p))
    off_bad = BodyOffset(roll=-_expected_corr(rel_r),
                         pitch=-_expected_corr(rel_p))
    e_good = foot_world_error(q_tgt, feet_ref, off_good)
    e_bad = foot_world_error(q_tgt, feet_ref, off_bad)
    assert e_good < 0.005, f"counter-rotation error {e_good*1e3:.1f}mm"
    assert e_bad > 3.0 * e_good, (
        f"sign not discriminated: good {e_good*1e3:.1f}mm vs "
        f"wrong-sign {e_bad*1e3:.1f}mm")
    # +roll = +y (left) side up = leaning right: right legs (3-5)
    # extend (z more negative), left legs (0-2) flex.
    z_tgt = fk_all_feet(q_tgt)[:, 2]
    z_nom = fk_all_feet(env._q_nom)[:, 2]
    assert np.mean(z_tgt[3:6]) < np.mean(z_nom[3:6]) - 1e-3
    assert np.mean(z_tgt[0:3]) > np.mean(z_nom[0:3]) + 1e-3


def test_tilt_comp_clips_the_correction():
    """A 30deg relative lean commands at most bc_anchor_tilt_max_deg
    (default 6 — the action-space expressibility boundary) of
    counter-rotation, not 28.5. The emitted action target equals the
    capped IK solve byte-for-byte (no saturation loss at the cap)."""
    from rl_move.body_ik import (
        BodyOffset, FixedFootBodyIK, fk_all_feet, foot_world_error)
    env = _tilt_env(33, tilt_comp=1.0)
    env.reset()
    _shift_ref(env, roll_deg=30.0)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    q_tgt = action_to_q_rad(info["bc_target"])
    rel_r = env._state.imu_roll - env._tilt_ref0[0]
    rel_p = env._state.imu_pitch - env._tilt_ref0[1]
    feet_ref = fk_all_feet(env._q_nom)
    off_cap = BodyOffset(roll=_expected_corr(rel_r),
                         pitch=_expected_corr(rel_p))
    off_full = BodyOffset(roll=-_soft(rel_r), pitch=-_soft(rel_p))
    e_cap = foot_world_error(q_tgt, feet_ref, off_cap)
    e_full = foot_world_error(q_tgt, feet_ref, off_full)
    assert e_cap < 0.005, f"capped-offset error {e_cap*1e3:.1f}mm"
    assert e_full > 3.0 * e_cap, "correction was not clipped"
    ik = FixedFootBodyIK()
    ik.reset(env._q_nom)
    res = ik.solve(off_cap)
    assert res.ok
    assert np.array_equal(
        info["bc_target"],
        q_rad_to_action(res.q_rad).astype(np.float32))


def test_tilt_comp_track_mode_excluded():
    """Track episodes command attitude goals the compensation would
    fight: tilt_comp must leave their target at the constant q_nom."""
    env = _tilt_env(34, tilt_comp=1.0, only_mode="track")
    env.reset()
    assert env._goal_traj.mode == "track"
    _shift_ref(env, roll_deg=8.0)
    expected = q_rad_to_action(env._q_nom)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    assert np.allclose(info["bc_target"], expected, atol=1e-6)


def test_tilt_comp_tipped_start_end_to_end():
    """Integration with dr.tipped_start_*: a forced 8deg tipped hold
    spawn (level tilt reference, the recovery-metric convention) emits
    a counter-rotating target — the same foot_world_error check driven
    entirely by the env's own measured relative attitude."""
    from rl_move.body_ik import (
        BodyOffset, fk_all_feet, foot_world_error)
    cfg = load_config()
    ov = dict(BASE_OVERRIDES)
    ov.update({("train", "bc_anchor_coef"): 1.0,
               ("train", "bc_anchor_tilt_comp"): 1.0,
               ("dr", "tipped_start_prob"): 1.0,
               ("dr", "tipped_start_deg"): [8.0, 8.0]})
    for (sec, leaf), val in ov.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=0.0, episode_seconds=16.0, seed=35, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    env.reset()
    assert getattr(env, "_tipped_applied", False), \
        "tipped start did not engage"
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    rel_r = env._state.imu_roll - env._tilt_ref0[0]
    rel_p = env._state.imu_pitch - env._tilt_ref0[1]
    if max(abs(rel_r), abs(rel_p)) * RAD2DEG <= 2.0:
        pytest.skip("tip settled level before the first tick")
    q_tgt = action_to_q_rad(info["bc_target"])
    assert not np.allclose(q_tgt, env._q_nom, atol=1e-4)
    feet_ref = fk_all_feet(env._q_nom)
    off_good = BodyOffset(roll=_expected_corr(rel_r),
                          pitch=_expected_corr(rel_p))
    e_good = foot_world_error(q_tgt, feet_ref, off_good)
    assert e_good < 0.005, f"counter-rotation error {e_good*1e3:.1f}mm"
    env.close()


# ---------------------------------------------------------------------------
# SETTLE-LEAN COMP SOURCE (train.bc_anchor_tilt_from_settle, 08-13).
# probe_tilt_teacher measured that the current-lean proportional source
# is a P-controller with a closed-loop fixed point at (L0+deadband)/2:
# a PERFECT student of the tilt-comp teacher settles at 3.95deg from
# 6.5deg tipped spawns (prediction 3.98) — above the 3deg leveling bar,
# because as the student levels, the measured lean (and hence the
# commanded counter-rotation) shrinks below what leveling needs. The
# settle-lean source freezes the comp at the episode's post-settle lean
# (a per-episode constant, SNAP_ATTRS pool-safe): the ideal student
# levels to the deadband / cap-limited residual. These tests pin:
# default-off bit-exactness vs the current-lean source, the source
# actually being the settled lean (not the live one), target constancy
# while the student levels, and the end-to-end tipped-reset capture.
# ---------------------------------------------------------------------------

def test_tilt_from_settle_off_matches_current_lean_source():
    """from_settle=0.0 emits byte-identical targets to the key being
    absent (default-off bit-exactness of the new source switch)."""
    outs = []
    for extra in ({}, {("train", "bc_anchor_tilt_from_settle"): 0.0}):
        env = _tilt_env(40, tilt_comp=1.0, extra=extra)
        env.reset()
        _shift_ref(env, roll_deg=5.5)
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
        outs.append(np.asarray(info["bc_target"]))
        env.close()
    assert np.array_equal(outs[0], outs[1])


def test_tilt_from_settle_ignores_current_lean():
    """from_settle=1.0: with the CURRENT pose level (proportional
    source would keep q_nom), a recorded 5.5deg settled lean still
    commands the counter-rotated target — the comp reads the episode
    constant, not the live attitude."""
    from rl_move.body_ik import (
        BodyOffset, fk_all_feet, foot_world_error)
    env = _tilt_env(41, tilt_comp=1.0,
                    extra={("train",
                            "bc_anchor_tilt_from_settle"): 1.0})
    env.reset()
    r = abs(env._state.imu_roll - env._tilt_ref0[0]) * RAD2DEG
    assert r < 1.5, "fixture not level enough for the test"
    env._settle_lean = (5.5 * DEG2RAD, 0.0)
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    q_tgt = action_to_q_rad(info["bc_target"])
    assert not np.allclose(q_tgt, env._q_nom, atol=1e-4), \
        "settle-lean source ignored the recorded settled lean"
    feet_ref = fk_all_feet(env._q_nom)
    off_good = BodyOffset(roll=_expected_corr(5.5 * DEG2RAD), pitch=0.0)
    off_bad = BodyOffset(roll=-_expected_corr(5.5 * DEG2RAD), pitch=0.0)
    e_good = foot_world_error(q_tgt, feet_ref, off_good)
    e_bad = foot_world_error(q_tgt, feet_ref, off_bad)
    assert e_good < 0.005, f"counter-rotation error {e_good*1e3:.1f}mm"
    assert e_bad > 3.0 * e_good


def test_tilt_from_settle_target_constant_while_leveling():
    """from_settle=1.0: the target does NOT backslide as the student
    levels — byte-identical across ticks while the live relative lean
    swings 5.5deg -> 0 -> -3deg (the proportional source would emit
    three different targets)."""
    env = _tilt_env(42, tilt_comp=1.0,
                    extra={("train",
                            "bc_anchor_tilt_from_settle"): 1.0})
    env.reset()
    env._settle_lean = (5.5 * DEG2RAD, 0.0)
    targets = []
    for shift in (5.5, -5.5, -3.0):
        _shift_ref(env, roll_deg=shift)
        _o, _r, _t, _tr, info = env.step(np.zeros(18))
        targets.append(np.asarray(info["bc_target"]))
    assert np.array_equal(targets[0], targets[1])
    assert np.array_equal(targets[0], targets[2])
    q_tgt = action_to_q_rad(targets[0])
    assert not np.allclose(q_tgt, env._q_nom, atol=1e-4)
    env.close()


def test_tilt_from_settle_captured_at_tipped_reset():
    """End-to-end with dr.tipped_start_*: a forced 8deg tipped hold
    spawn records its post-settle lean in _settle_lean (matching the
    reset-tick relative attitude) and the teacher counter-rotates THAT
    constant."""
    from rl_move.body_ik import (
        BodyOffset, fk_all_feet, foot_world_error)
    cfg = load_config()
    ov = dict(BASE_OVERRIDES)
    ov.update({("train", "bc_anchor_coef"): 1.0,
               ("train", "bc_anchor_tilt_comp"): 1.0,
               ("train", "bc_anchor_tilt_from_settle"): 1.0,
               ("dr", "tipped_start_prob"): 1.0,
               ("dr", "tipped_start_deg"): [8.0, 8.0]})
    for (sec, leaf), val in ov.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=0.0, episode_seconds=16.0, seed=43, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    env.reset()
    assert getattr(env, "_tipped_applied", False), \
        "tipped start did not engage"
    rel_r0 = env._state.imu_roll - env._tilt_ref0[0]
    rel_p0 = env._state.imu_pitch - env._tilt_ref0[1]
    assert abs(env._settle_lean[0] - rel_r0) < 1e-9
    assert abs(env._settle_lean[1] - rel_p0) < 1e-9
    if max(abs(rel_r0), abs(rel_p0)) * RAD2DEG <= 2.0:
        pytest.skip("tip settled level before capture")
    _o, _r, _t, _tr, info = env.step(np.zeros(18))
    q_tgt = action_to_q_rad(info["bc_target"])
    feet_ref = fk_all_feet(env._q_nom)
    off_good = BodyOffset(roll=_expected_corr(rel_r0),
                          pitch=_expected_corr(rel_p0))
    e_good = foot_world_error(q_tgt, feet_ref, off_good)
    assert e_good < 0.005, f"counter-rotation error {e_good*1e3:.1f}mm"
    env.close()


# ---------------------------------------------------------------------------
# train.bc_anchor_flat_time_indexed (08-25, tucklook1 dig-in): absolute
# script-clock anchor targets for pure FLAT rise starts — the measured
# +2021 replay_script optimum (probe_stance_pricing) that no pursuit
# variant ever taught. Non-flat/RSI episodes keep state-aligned pursuit.
# ---------------------------------------------------------------------------

def _flat_clock_env(flat_clock, seed=3, force="flat", extra=None):
    ov = {("train", "bc_anchor_coef"): 1.0,
          ("train", "bc_anchor_state_aligned"): 1.0,
          ("train", "bc_anchor_lookahead_s"): 0.5,
          ("goal", "rise_rsi_frac"): 0.0}
    if flat_clock is not None:
        ov[("train", "bc_anchor_flat_time_indexed")] = float(flat_clock)
    ov.update(extra or {})
    env = _make_env(seed, ov)
    env._goal_gen.force_rise_start = force
    return env


def _collect_targets(env, n=60, freeze=True):
    env.reset()
    act = q_rad_to_action(env.data.qpos[env._qadr])
    out = []
    for _ in range(n):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc:
            break
        if "bc_target" in info:
            out.append(info["bc_target"].copy())
            if not freeze:
                act = info["bc_target"]
    env.close()
    return np.asarray(out)


def test_flat_time_indexed_default_off_bit_exact():
    """Absent and explicitly 0.0 must emit byte-identical targets —
    the new branch's default path is the legacy state-aligned
    pursuit."""
    a = _collect_targets(_flat_clock_env(None))
    b = _collect_targets(_flat_clock_env(0.0))
    assert a.shape == b.shape and np.array_equal(a, b)


def test_flat_time_indexed_targets_follow_the_script_clock():
    """On a flat start the emitted target must be the ref row at the
    episode's own absolute clock (+1 env tick), advancing tick for
    tick even while the robot holds still — the anti-freeze property
    the state-aligned pursuit lacks (its matched index stalls with the
    robot)."""
    ref = load_rise_ref(str(ROOT / RISE_REF))

    def tick_of(target) -> int:
        q_t = action_to_q_rad(np.asarray(target, dtype=float))
        return int(np.argmin(
            ((ref["q"] - q_t[None, :]) ** 2).mean(axis=1)))

    env_probe = _flat_clock_env(1.0)
    dt_ratio = env_probe.dt / float(ref["dt"])
    env_probe.close()
    out = _collect_targets(_flat_clock_env(1.0), n=40, freeze=True)
    assert len(out) >= 30
    ticks = np.array([tick_of(t) for t in out])
    # Row i of `out` was emitted at _step_i = i+1 -> clock row
    # round((i+1)*dt/ref_dt), + max(round(dt/ref_dt),1) lookahead.
    # tick_of round-trips through the action map so allow +-1 row.
    ahead = max(int(round(dt_ratio)), 1)
    expect = np.array([int(round((i + 1) * dt_ratio)) + ahead
                       for i in range(len(out))])
    assert np.max(np.abs(ticks - expect)) <= 1, (
        f"flat-clock targets do not track the script clock: "
        f"got {ticks[:8]}... expected {expect[:8]}... "
        f"(dt_ratio {dt_ratio:.3f})")
    # Strictly advancing on a FROZEN robot (allowing flat spots from
    # action-map quantization, but net motion over any 10-tick span).
    assert all(ticks[i + 10] > ticks[i] for i in range(len(ticks) - 10))


def test_flat_time_indexed_nonflat_starts_unchanged():
    """Crouch starts must keep the state-aligned pursuit byte-for-byte
    — the absolute clock is only honest from the script's own start
    state (pure flat)."""
    a = _collect_targets(_flat_clock_env(None, seed=5, force="crouch"))
    b = _collect_targets(_flat_clock_env(1.0, seed=5, force="crouch"))
    assert a.shape == b.shape and np.array_equal(a, b)


# ---------------------------------------------------------------------------
# HEIGHT-AWARE HOLD REFERENCE (train.bc_anchor_hold_height_aware, 08-25).
# holdheight-rung1/-s1 (the goal.hold_height_cmd_frac mechanism canary)
# trained WITHOUT any hold pose anchor at all (the existing anchor's
# target, q_nom, is height-BLIND and would fight a moving command) and
# lost the champion's clean quiet-stand current/load profile even on
# the STATIC height_ref=0 DR-0 gate (cur_max 2.0-2.63A vs the
# champion's 0.67-0.71A; 5/6 det episodes tripped hold_min_load) —
# dropping the anchor threw out its general pose regularization along
# with its height-blindness. Fix: re-target the anchor at the pose
# that reaches the NEXT commanded height via FixedFootBodyIK, the
# same one-tick-ahead convention `bc_anchor_lower` already uses, so
# the anchor keeps supervising pose quality while tracking a moving
# target instead of fighting it.
# ---------------------------------------------------------------------------

def _height_cmd_env(seed, hha=None, cmd_frac=1.0, kind_target=None,
                    only_mode="hold"):
    ov = {("train", "bc_anchor_coef"): 1.0,
         ("goal", "hold_height_cmd_frac"): cmd_frac,
         ("goal", "hold_height_cmd_range_mm"): [-40.0, 20.0],
         ("goal", "hold_height_cmd_rate_mm_s"): 15.0}
    if hha is not None:
        ov[("train", "bc_anchor_hold_height_aware")] = hha
    env = _make_env(seed, ov, only_mode=only_mode)
    if kind_target is not None:
        env._goal_gen.force_hold_height_profile = kind_target
    return env


def test_hold_height_aware_default_off_bit_exact():
    """No bc_anchor_hold_height_aware key: a hold episode ramped to
    -30mm still anchors to the exact constant q_nom target (legacy
    height-blind behavior byte-for-byte), even though the goal itself
    is moving."""
    env = _height_cmd_env(40, hha=None, kind_target=("ramp", -0.030))
    env.reset()
    expected = q_rad_to_action(env._q_nom)
    act = expected
    for _ in range(int(round(6.0 / env.dt))):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc:
            break
        assert np.array_equal(info["bc_target"],
                              expected.astype(np.float32))
        act = info["bc_target"]
    env.close()


def test_hold_height_aware_zero_height_stays_qnom():
    """hha=1 but the commanded height is exactly 0 (the settle window,
    or a plain flat hold episode when cmd_frac<1): target is
    bit-identical to the legacy constant q_nom, same convention as
    tilt_comp's level-deadband no-op."""
    env = _height_cmd_env(41, hha=1.0, cmd_frac=0.0)
    env.reset()
    expected = q_rad_to_action(env._q_nom)
    _o, _r, _t, _tr, info = env.step(expected)
    assert np.array_equal(info["bc_target"], expected.astype(np.float32))
    env.close()


def test_hold_height_aware_targets_the_commanded_height():
    """With bc_anchor_hold_height_aware=1, once a ramp has reached its
    -30mm target every subsequent tick's bc_target is EXACTLY the
    FixedFootBodyIK descent from q_nom to the next commanded height —
    the honest demonstration, not the height-blind pose."""
    from rl_move.body_ik import BodyOffset, FixedFootBodyIK
    env = _height_cmd_env(42, hha=1.0, kind_target=("ramp", -0.030))
    env.reset()
    act = q_rad_to_action(env.data.qpos[env._qadr])
    seen_offset_target = False
    for _ in range(int(round(8.0 / env.dt))):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc:
            break
        act = info["bc_target"]
        g_next = env._goal_traj.at(env._step_i + 1)
        if abs(float(g_next.height_ref) + 0.030) < 1e-4:
            ik = FixedFootBodyIK()
            ik.reset(env._q_nom)
            res = ik.solve(BodyOffset(height=float(g_next.height_ref)))
            assert res.ok
            assert np.allclose(act, q_rad_to_action(res.q_rad),
                               atol=1e-6)
            seen_offset_target = True
    env.close()
    assert seen_offset_target, "ramp never settled at -30mm in 8s"


def test_hold_height_aware_track_mode_excluded():
    """track episodes never carry a moving height command, but the
    gate is explicit (mode == 'hold') — a track episode's target must
    stay the legacy constant q_nom regardless of the flag."""
    env = _height_cmd_env(43, hha=1.0, only_mode="track")
    env.reset()
    assert env._goal_traj.mode == "track"
    expected = q_rad_to_action(env._q_nom)
    _o, _r, _t, _tr, info = env.step(expected)
    assert np.array_equal(info["bc_target"], expected.astype(np.float32))
    env.close()


def test_hold_height_aware_chain_tracks_height_with_feet_planted():
    """Following the height-aware anchor's targets must produce the
    honest behavior class: body height tracks the commanded -30mm
    descent while every foot stays grounded (no aloft/outrigger
    residue) — the same class of check `bc_anchor_lower`'s own chain
    test uses."""
    env = _height_cmd_env(44, hha=1.0, kind_target=("ramp", -0.030))
    env.reset()
    z_start = float(env.data.xpos[env._chassis_bid, 2])
    act = q_rad_to_action(env.data.qpos[env._qadr])
    worst_clear_mm = 0.0
    for _ in range(int(round(8.0 / env.dt))):
        _o, _r, term, trunc, info = env.step(act)
        if term or trunc:
            assert not term, "the height-aware anchored hold fell over"
            break
        act = info["bc_target"]
        clear = max(float(env.data.xpos[b, 2]) - env._pad_z_ref[i]
                    for i, b in enumerate(env._pad_bids))
        worst_clear_mm = max(worst_clear_mm, clear * 1000.0)
    z_end = float(env.data.xpos[env._chassis_bid, 2])
    dropped = z_start - z_end
    env.close()
    assert dropped > 0.6 * 0.030, (
        f"chain only descended {dropped*1000:.0f}mm of the commanded 30mm")
    assert worst_clear_mm < 25.0, (
        f"a foot came {worst_clear_mm:.0f}mm off the ground during the "
        f"height-aware anchored hold — not the honest feet-planted class")
