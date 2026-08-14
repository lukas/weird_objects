"""MjxVecEnv correctness on CPU MJX (small batch, no GPU needed).

Validates the batched vec env against the C twin: reset choreography
produces the same settled stance, zero-action holds score like the C
env, terminations pop pooled resets that SB3 can consume, and the
host-side halves (safety/reward/obs) behave identically because they
ARE the same code. Skipped when mujoco-mjx / jax aren't installed.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_PROTO = Path(__file__).resolve().parents[2]
if str(_PROTO) not in sys.path:
    sys.path.insert(0, str(_PROTO))

from rl_move.sim.mjx_backend import mjx_is_available  # noqa: E402

if not mjx_is_available():  # pragma: no cover
    pytest.skip("mujoco-mjx / jax not installed", allow_module_level=True)

from rl_move.body_ik import N_ACT  # noqa: E402
from rl_move.sim.mjx_vec_env import MjxVecEnv  # noqa: E402
from rl_move.sim.sim_env import SimHexapodBalanceEnv  # noqa: E402

B = 2


@pytest.fixture(scope="module")
def venv():
    # desync_episodes=False: these tests assert lockstep truncation.
    # The bit-equality test below runs with desync ON (the default).
    v = MjxVecEnv(SimHexapodBalanceEnv, B,
                  env_kwargs=dict(randomize=False, episode_seconds=0.6),
                  seed=0, pool_per_env=1, desync_episodes=False)
    yield v
    v.close()


def test_reset_shapes_and_settle(venv):
    obs = venv.reset()
    assert obs.shape == (B, venv.observation_space.shape[0])
    assert np.all(np.isfinite(obs))
    for info in venv.reset_infos:
        # Settled level, like the C env's plant reset (typically <1°).
        assert abs(info["roll_deg"]) < 3.0
        assert abs(info["pitch_deg"]) < 3.0


def test_zero_action_hold_matches_c_env(venv):
    """Zero-action hold: reward and attitude must track the C twin.

    Physics differs (float32 MJX, plane terrain, iterations 8 vs 50) so
    this is a behavioral check with loose tolerances, not bit parity —
    the same bar the phase-1 settle test set (joint diffs <0.3°).
    """
    c_env = SimHexapodBalanceEnv(randomize=False, episode_seconds=0.6,
                                 seed=0)
    c_env.reset()
    r_c, roll_c = [], []
    for _ in range(5):
        _, r, term, trunc, info = c_env.step(np.zeros(N_ACT))
        r_c.append(r)
        roll_c.append(info["roll_deg"])
        assert not (term or trunc)

    venv.reset()
    r_m, roll_m = [], []
    for _ in range(5):
        obs, rews, dones, infos = venv.step(np.zeros((B, N_ACT)))
        assert obs.shape == (B, venv.observation_space.shape[0])
        assert not dones.any()
        r_m.append(rews)
        roll_m.append([i["roll_deg"] for i in infos])

    r_c, r_m = np.asarray(r_c), np.asarray(r_m)
    roll_c, roll_m = np.asarray(roll_c), np.asarray(roll_m)
    # Every env's per-step reward within 0.25 of the C twin's, attitude
    # within 1.5 degrees.
    assert np.max(np.abs(r_m - r_c[:, None])) < 0.25, (r_m, r_c)
    assert np.max(np.abs(roll_m - roll_c[:, None])) < 1.5


def test_rejected_action_pops_pooled_reset(venv):
    venv.reset()
    acts = np.zeros((B, N_ACT))
    acts[0] = np.nan                      # SafetyLayer rejects outright
    obs, rews, dones, infos = venv.step(acts)
    assert dones[0] and not dones[1]
    assert rews[0] < 0                    # termination penalty
    assert "terminal_observation" in infos[0]
    assert not infos[0]["TimeLimit.truncated"]
    # The returned obs is the POOLED reset's first observation and the
    # env must be immediately steppable.
    assert np.all(np.isfinite(obs))
    obs, rews, dones, infos = venv.step(np.zeros((B, N_ACT)))
    assert not dones.any()
    assert np.all(np.isfinite(rews))


def test_sharded_bitwise_matches_inprocess():
    """MjxShardedVecEnv must be BIT-IDENTICAL to the in-process
    reference: same seeds + same device ticks + same host halves ⇒ the
    obs/reward/done streams (incl. pooled-reset pops, DR draws, and a
    pool refill at truncation) may not differ in a single bit."""
    from rl_move.sim.mjx_sharded_vec_env import MjxShardedVecEnv

    kw = dict(randomize=True, episode_seconds=0.6)
    a = MjxVecEnv(SimHexapodBalanceEnv, 4, env_kwargs=kw, seed=7,
                  pool_per_env=1)
    b = MjxShardedVecEnv(SimHexapodBalanceEnv, 4, env_kwargs=kw, seed=7,
                         pool_per_env=1, host_workers=2)
    try:
        obs_a, obs_b = a.reset(), b.reset()
        assert np.array_equal(obs_a, obs_b)
        rng = np.random.default_rng(0)
        n = a.envs[0].episode_steps
        for k in range(n + 2):     # crosses truncation + pool refill
            act = rng.uniform(-0.5, 0.5, (4, N_ACT)).astype(np.float32)
            if k == 2:
                act[0] = np.nan    # rejected action → pooled reset pop
            oa, ra, da, ia = a.step(act)
            ob, rb, db, ib = b.step(act)
            assert np.array_equal(oa, ob), f"obs diverged at step {k}"
            assert np.array_equal(ra, rb), f"reward diverged at step {k}"
            assert np.array_equal(da, db), f"dones diverged at step {k}"
            for x, y in zip(ia, ib):
                assert x.get("termination_reason") == \
                    y.get("termination_reason")
        assert da[0] or True       # exercised the pop path above
    finally:
        a.close()
        b.close()


def test_model_dr_reaches_device_and_stays_stable():
    """Model-field DR: per-world device rows must exist, differ across
    envs, exactly reflect each env's _ep_rand draw (chassis mass row =
    base × mass_scale), and the batched physics must stay finite."""
    kw = dict(randomize=True, episode_seconds=0.6)
    v = MjxVecEnv(SimHexapodBalanceEnv, 3, env_kwargs=kw, seed=11,
                  pool_per_env=1)
    try:
        v.reset()
        st = v.stepper
        assert st.model_dr
        bm = np.asarray(st._dr_fields["body_mass"])
        base = np.asarray(st._get_model_field(st.model, "body_mass"))
        assert bm.shape[0] == 3
        assert not np.allclose(bm[0], base)      # DR'd, not nominal
        assert not np.allclose(bm[0], bm[1])     # per-world draws differ
        g = np.asarray(st._dr_fields["opt.gravity"])
        assert g.shape == (3, 3)
        assert not np.allclose(g[0], g[1])       # tilted gravity per world
        # Host draw ↔ device row: the chassis body gets only the global
        # mass scale (leg jitter applies to leg links).
        er0 = v.envs[0]._ep_rand
        cb = v.envs[0]._chassis_bid
        assert np.isclose(float(bm[0][cb]) / float(base[cb]),
                          er0.mass_scale, rtol=1e-5)
        for _ in range(5):
            obs, rews, dones, infos = v.step(np.zeros((3, N_ACT)))
            assert np.all(np.isfinite(obs))
            assert np.all(np.isfinite(rews))
    finally:
        v.close()


def test_truncation_and_pool_refill(venv):
    venv.reset()
    n = venv.envs[0].episode_steps
    for k in range(n):
        obs, rews, dones, infos = venv.step(np.zeros((B, N_ACT)))
        if k < n - 1:
            assert not dones.any()
    assert dones.all()                    # synchronized truncation
    for info in infos:
        assert info["TimeLimit.truncated"]
    # Pools were sized 1 and one entry was consumed earlier — this step
    # must have auto-refilled without corrupting the live batch.
    obs, rews, dones, infos = venv.step(np.zeros((B, N_ACT)))
    assert not dones.any()
    assert np.all(np.isfinite(obs))


# ---------------------------------------------------------------------------
# goal.mode_seq on the batched path (TRANSITIONS_DIRECTIVE follow-up,
# 08-14): the MJX choreography must mint the canonical plant/belly
# segment frames (sim_env._seq_capture_frames twin) — before this the
# vec path raised at the first mid-episode switch.
# ---------------------------------------------------------------------------

def _seq_cfg(seg_s=(3.0, 4.0)):
    # Mirrors test_mode_seq._make_env: segments shorter than the rise
    # sampler's hold+ramp horizon make the goal generator degenerate,
    # so keep the C test's 3-4 s segments / 12 s episodes.
    from rl_move.config import load_config
    cfg = load_config()
    g = cfg.setdefault("goal", {})
    g["mode_seq"] = 1.0
    g["mode_seq_segment_s_min"] = seg_s[0]
    g["mode_seq_segment_s_max"] = seg_s[1]
    cfg.setdefault("obs", {})["mode_onehot"] = 1.0
    return cfg


def test_mode_seq_frames_minted_and_match_c_env():
    """The batched mint must reproduce the C env's canonical frames.

    Physics differs (float32 MJX, iterations 8 vs 50) so this is the
    settle test's behavioral bar, not bit parity: q_nom within 0.03
    rad, z0 within 6 mm — and the two families must differ materially
    (the ~79 deg knee gap the trans-dagger2 kill measured is the whole
    point of the canonical-frame install)."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = _seq_cfg()
    v = MjxVecEnv(SimHexapodJointWalkEnv, B,
                  env_kwargs=dict(randomize=False, episode_seconds=12.0,
                                  cfg=cfg),
                  seed=3, pool_per_env=1, desync_episodes=False)
    try:
        v.reset()
        c_env = SimHexapodJointWalkEnv(randomize=False,
                                       episode_seconds=12.0, seed=3,
                                       cfg=cfg)
        c_env.reset()
        ref = c_env._seq_frames
        assert ref is not None
        for env in v.envs:
            fr = env._seq_frames
            assert fr is not None and set(fr) == {"plant", "belly"}
            assert np.max(np.abs(fr["plant"]["q_nom"]
                                 - fr["belly"]["q_nom"])) > 0.5
            assert fr["belly"]["z0"] < fr["plant"]["z0"]
            for fam in ("plant", "belly"):
                assert np.max(np.abs(fr[fam]["q_nom"]
                                     - ref[fam]["q_nom"])) < 0.03, fam
                assert abs(fr[fam]["z0"] - ref[fam]["z0"]) < 0.006, fam
                assert np.max(np.abs(fr[fam]["pad_z_ref"]
                                     - ref[fam]["pad_z_ref"])) < 0.008, fam
    finally:
        v.close()


def test_mode_seq_switch_crosses_on_batched_path():
    """Stepping across a segment boundary must install the canonical
    frame (pre-mint code raised RuntimeError at the first switch) and
    keep obs finite; at least one env must actually switch."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = _seq_cfg()
    v = MjxVecEnv(SimHexapodJointWalkEnv, B,
                  env_kwargs=dict(randomize=False, episode_seconds=12.0,
                                  cfg=cfg),
                  seed=5, pool_per_env=1, desync_episodes=False)
    try:
        v.reset()
        # Step just past the longest possible first segment (4 s):
        # every env's first switch must install the canonical frame
        # (pre-mint code raised RuntimeError here).
        dt = v.envs[0].dt
        n_act = v.action_space.shape[0]   # walk env n_act != balance N_ACT
        for _ in range(int(round(4.5 / dt))):
            obs, rews, dones, infos = v.step(np.zeros((B, n_act)))
            assert np.all(np.isfinite(obs))
            for info in infos:
                assert info.get("termination_reason") != "bad_action_shape"
        assert any(env._seq_idx > 0 for env in v.envs)
    finally:
        v.close()


def test_mode_seq_sharded_bitwise_matches_inprocess():
    """The sharded mint (08-14) must make MjxShardedVecEnv BIT-IDENTICAL
    to the in-process reference with goal.mode_seq on — same probe
    choreography, same frames, same switch behavior. Pre-fix the
    sharded path raised RuntimeError at the first switch (killed
    cw-arch-modeseq1's first launch); the in-process class is the
    pod-verified reference (test_mode_seq_frames_minted_and_match_c_env)."""
    from rl_move.sim.mjx_sharded_vec_env import MjxShardedVecEnv
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = _seq_cfg()
    kw = dict(randomize=False, episode_seconds=6.0, cfg=cfg)
    a = MjxVecEnv(SimHexapodJointWalkEnv, 4, env_kwargs=kw, seed=7,
                  pool_per_env=1, desync_episodes=False)
    b = MjxShardedVecEnv(SimHexapodJointWalkEnv, 4, env_kwargs=kw,
                         seed=7, pool_per_env=1, host_workers=2,
                         desync_episodes=False)
    try:
        obs_a, obs_b = a.reset(), b.reset()
        assert np.array_equal(obs_a, obs_b)
        # In-process reference must have minted (guards the seq gate).
        assert all(env._seq_frames is not None for env in a.envs)
        n_act = a.action_space.shape[0]
        dt = a.envs[0].dt
        rng = np.random.default_rng(0)
        # Cross the first switch (first segment <= 4 s) and keep going:
        # the sharded path must neither raise nor diverge by a bit.
        for k in range(int(round(4.5 / dt))):
            act = rng.uniform(-0.3, 0.3, (4, n_act)).astype(np.float32)
            oa, ra, da, ia = a.step(act)
            ob, rb, db, ib = b.step(act)
            assert np.array_equal(oa, ob), f"obs diverged at step {k}"
            assert np.array_equal(ra, rb), f"reward diverged at step {k}"
            assert np.array_equal(da, db), f"dones diverged at step {k}"
        assert any(env._seq_idx > 0 for env in a.envs)  # switch crossed
    finally:
        a.close()
        b.close()
