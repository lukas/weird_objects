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
