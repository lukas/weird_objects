"""Replay-parity lock for the DEPLOY-SIDE rot-60 port
(linux_control/rl_policy.py, RL_PLAN queue 2.1, 08-11).

The robot runner does not port the canonicalizer — it wraps
rl_move.sim.rot60.Rot60Policy itself through a NumpyPolicy shim
(make_walk_canonicalizer), so there is no second implementation to
drift. What CAN still break silently is the integration contract, and
that is what this file locks, using the REAL deployed weights file
(linux_control/rl_walk_weights.json = ppo_goal_cw_dep_vref1_r1):

1. the runner's walk obs (build_obs + vel tail) is exactly the 72-wide
   frame rot60.py's slices assume, block by block;
2. forward-wedge commands are a bit-exact no-op (k=0) — the proven
   hardware forward contract is untouched by default-ON;
3. full-circle command sequences (incl. zero-cmd holds and sector-
   boundary dither) produce the same sector trace and bit-identical
   actions as the manually composed rot60 primitives — the replay-
   parity check: logged REAL-frame obs replayed through the runner's
   canonicalizer must reproduce the logged actions;
4. the wedge fallback used when the module is missing on-board.
"""
from __future__ import annotations

import math
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
from rl_move.sim import rot60  # noqa: E402

CFG = load_config(str(_ROOT / "rl_move" / "config.yaml"))
WALK_VEL_SCALE = rl_policy.WALK_VEL_SCALE


def _state(rng):
    return SimpleNamespace(
        joint_position=rng.normal(0, 0.3, 18),
        joint_velocity=rng.normal(0, 0.5, 18),
        imu_roll=float(rng.normal(0, 0.05)),
        imu_pitch=float(rng.normal(0, 0.05)),
        imu_gyro=rng.normal(0, 0.3, 3),
    )


def _walk_obs(rng, vx_r, vy_r, prev_action=None, state=None):
    """Build the walk obs EXACTLY as rl_policy.run_policy_move does."""
    state = state or _state(rng)
    prev = prev_action if prev_action is not None else rng.uniform(-1, 1, 18)
    goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0, height_ref=0.0,
                    unload_leg=None)
    obs = build_obs(CFG, state, np.zeros(18), prev, goal=goal,
                    tilt_ref=(0.0, 0.0))
    return np.concatenate(
        [obs, np.array([vx_r, vy_r, vx_r, vy_r]) / WALK_VEL_SCALE]
    ).astype(np.float32)


def _policy():
    return rl_policy.NumpyPolicy(rl_policy.WALK_WEIGHTS_PATH)


def test_runner_obs_layout_matches_rot60_slices():
    """Every rot60 slice picks the runner quantity it thinks it does."""
    rng = np.random.default_rng(0)
    st = _state(rng)
    prev = rng.uniform(-1, 1, 18)
    obs = _walk_obs(rng, 0.04, -0.02, prev_action=prev, state=st)
    assert obs.shape == (rot60.FRAME_WALK,)
    ts = 0.2  # obs.tilt_scale (config.yaml)
    np.testing.assert_allclose(obs[0:18], st.joint_position, atol=1e-6)
    np.testing.assert_allclose(obs[18:36], st.joint_velocity / 2.0,
                               atol=1e-6)
    np.testing.assert_allclose(
        obs[36:38], [st.imu_roll / ts, st.imu_pitch / ts], atol=1e-6)
    np.testing.assert_allclose(obs[38:41], st.imu_gyro, atol=1e-6)
    np.testing.assert_allclose(obs[41:59], prev, atol=1e-6)
    np.testing.assert_allclose(obs[59:62], 0.0, atol=1e-9)  # level refs
    np.testing.assert_allclose(obs[62:68], 0.0, atol=1e-9)  # no unload
    np.testing.assert_allclose(
        obs[68:72],
        np.array([0.04, -0.02, 0.04, -0.02]) / WALK_VEL_SCALE, atol=1e-6)
    # one-hot really lands where rot60 permutes it
    goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0, height_ref=0.0,
                    unload_leg=3)
    obs2 = build_obs(CFG, st, np.zeros(18), prev, goal=goal,
                     tilt_ref=(0.0, 0.0))
    assert obs2[62 + 3] == 1.0 and obs2[62:68].sum() == 1.0


def test_forward_wedge_is_bit_exact_noop():
    """Default-ON changes NOTHING for the proven forward contract."""
    rng = np.random.default_rng(1)
    policy = _policy()
    canon = rl_policy.make_walk_canonicalizer(policy, CFG)
    assert canon is not None
    for heading_deg in (0.0, 12.0, -25.0, 29.0, -29.0):
        th = math.radians(heading_deg)
        obs = _walk_obs(rng, 0.05 * math.cos(th), 0.05 * math.sin(th))
        act_wrapped, _ = canon.predict(obs)
        assert canon.k == 0
        np.testing.assert_array_equal(act_wrapped, policy.act(obs))


def test_full_circle_replay_parity():
    """Runner canonicalizer == manually composed rot60 primitives,
    bit-exact, over a full-circle sweep with zero-cmd holds and
    boundary dither (the hysteresis/hold contract)."""
    rng = np.random.default_rng(2)
    policy = _policy()
    canon = rl_policy.make_walk_canonicalizer(policy, CFG)

    headings = list(range(0, 360, 20)) + [31, 29, 33, 27, 35]  # dither
    seq = []
    for i, h in enumerate(headings):
        th = math.radians(h)
        seq.append((0.05 * math.cos(th), 0.05 * math.sin(th)))
        if i % 5 == 4:
            seq.append((0.0, 0.0))       # stick released: hold sector
    prev = np.zeros(18)
    k_ref = 0
    for vx, vy in seq:
        obs = _walk_obs(rng, vx, vy, prev_action=prev)
        act_runner, _ = canon.predict(obs)
        # reference: rot60 primitives composed by hand
        k_ref = rot60.sector_from_cmd(
            float(obs[rot60.OBS_VREF_X]), float(obs[rot60.OBS_VREF_Y]),
            k_ref)
        assert canon.k == k_ref
        obs_c = rot60.obs_transform(obs, k_ref, tilt_scale=0.2)
        act_ref = rot60.action_from_canonical(policy.act(obs_c), k_ref)
        np.testing.assert_array_equal(act_runner, act_ref)
        prev = np.clip(act_runner, -1, 1)  # REAL-frame prev_action
    # the sweep must actually have exercised every sector
    # (full circle => all 6 wedges)
    ks = set()
    k = 0
    for vx, vy in seq:
        k = rot60.sector_from_cmd(vx, vy, k)
        ks.add(k)
    assert ks == {-2, -1, 0, 1, 2, 3}


def test_backward_command_moves_off_k0_and_relabels():
    """A backward command must select the opposite sector and produce a
    DIFFERENT real-frame action than the naked (frozen) policy."""
    rng = np.random.default_rng(3)
    policy = _policy()
    canon = rl_policy.make_walk_canonicalizer(policy, CFG)
    obs = _walk_obs(rng, -0.05, 0.0)
    act_wrapped, _ = canon.predict(obs)
    assert canon.k == 3
    assert not np.allclose(act_wrapped, policy.act(obs), atol=1e-4)


def test_wedge_fallback_heading_check():
    assert rl_policy.heading_in_trained_wedge(0.05, 0.0)
    assert rl_policy.heading_in_trained_wedge(0.0, 0.0)   # zero cmd ok
    assert rl_policy.heading_in_trained_wedge(0.05, 0.02)  # ~22 deg
    assert not rl_policy.heading_in_trained_wedge(0.0, 0.05)   # +90
    assert not rl_policy.heading_in_trained_wedge(-0.05, 0.0)  # back
    assert not rl_policy.heading_in_trained_wedge(0.03, -0.04)  # -53 deg


def test_rot60_module_is_numpy_only():
    """The board has no torch/mujoco/sb3: the canonicalizer import chain
    must stay numpy-only or deploy breaks silently."""
    import ast

    def _imports(path: Path) -> set[str]:
        mods = set()
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                mods |= {a.name.split(".")[0] for a in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module:
                mods.add(node.module.split(".")[0])
        return mods

    allowed = {"__future__", "math", "numpy"}
    for f in ("rot60.py", "__init__.py"):
        extra = _imports(Path(rot60.__file__).parent / f) - allowed
        assert not extra, f"sim/{f} imports non-deployable modules: {extra}"
    assert rl_policy._ROT60_OK
