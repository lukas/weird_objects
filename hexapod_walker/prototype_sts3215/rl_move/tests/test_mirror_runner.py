"""Deploy-side lock for the MirrorPolicy turn port
(linux_control/rl_policy.py turn=, TURN.md deploy port, 08-12).

Like the rot60 port, the runner does not re-implement the mirror — it
wraps rl_move.sim.mirror.MirrorPolicy itself (make_walk_mirror) so
there is no second implementation to drift. This file locks the
integration contract with the REAL deployed weights file
(linux_control/rl_walk_weights.json):

1. turn=None stays bit-identical: the naked/rot60 tick path is
   untouched (the selector only exists when turn is requested);
2. the mirrored stack == manually composed mirror primitives, on both
   the naked-policy and the rot60-composed paths, bit-exact — the
   replay contract for the CSV "mirror" column;
3. mirror OUTSIDE rot60: each chirality owns its rot60 sector state,
   and a mirrored off-wedge command selects the mirrored sector;
4. ChiralitySelector semantics: left/right constant chirality keyed
   off NAKED_DRIFT_SIGN, heading-hold bang-bang with the sim probe's
   4 deg hysteresis (never chatters inside the band);
5. mirror.py's import chain stays numpy-only (the board has no torch).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "linux_control") not in sys.path:
    sys.path.insert(0, str(_ROOT / "linux_control"))

import rl_policy  # noqa: E402  (linux_control runner)

from rl_move.config import load_config  # noqa: E402
from rl_move.env import TaskGoal, build_obs  # noqa: E402
from rl_move.sim import mirror, rot60  # noqa: E402

CFG = load_config(str(_ROOT / "rl_move" / "config.yaml"))
WALK_VEL_SCALE = rl_policy.WALK_VEL_SCALE
WALK_POLICY = rl_policy.NumpyPolicy(rl_policy.WALK_WEIGHTS_PATH)
DT = 1.0 / rl_policy.policy_training_hz(WALK_POLICY)


def _state(rng):
    return SimpleNamespace(
        joint_position=rng.normal(0, 0.3, 18),
        joint_velocity=rng.normal(0, 0.5, 18),
        imu_roll=float(rng.normal(0, 0.05)),
        imu_pitch=float(rng.normal(0, 0.05)),
        imu_gyro=rng.normal(0, 0.3, 3),
    )


def _walk_obs(rng, vx_r, vy_r, prev_action=None, state=None):
    """The walk obs EXACTLY as rl_policy.run_policy_move builds it."""
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
    return WALK_POLICY


def _manual_mirror_act(policy_like, obs):
    """mirror primitives composed by hand: reflect obs, run the stack,
    reflect the action back — the reference for make_walk_mirror."""
    operm, osign = mirror.obs_perm_sign(walk=True)
    aperm, asign = mirror.joint_perm_sign()
    m_obs = (np.asarray(obs, dtype=np.float32)[operm] * osign)
    act, _ = policy_like.predict(m_obs)
    return np.asarray(act)[aperm] * asign


def test_mirror_module_is_numpy_only():
    """mirror.py must stay importable on the board (no torch/sb3 at
    import time; the training helpers import lazily inside functions)."""
    import ast

    def _top_level_imports(path: Path) -> set[str]:
        mods = set()
        for node in ast.parse(path.read_text()).body:
            if isinstance(node, ast.Import):
                mods |= {a.name.split(".")[0] for a in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module:
                mods.add(node.module.split(".")[0])
        return mods

    allowed = {"__future__", "numpy"}
    extra = _top_level_imports(Path(mirror.__file__)) - allowed
    assert not extra, f"mirror.py top-level imports: {extra}"
    assert rl_policy._MIRROR_OK


def test_mirror_wrapper_matches_manual_primitives_naked():
    """make_walk_mirror(rot60=False) == hand-composed mirror around the
    naked policy, bit-exact."""
    rng = np.random.default_rng(0)
    policy = _policy()
    wrapper = rl_policy.make_walk_mirror(policy, CFG, rot60=False)
    shim = rl_policy._Rot60ModelShim(policy)
    for _ in range(10):
        obs = _walk_obs(rng, 0.05, 0.0)
        act_port, _ = wrapper.predict(obs)
        np.testing.assert_array_equal(
            act_port, _manual_mirror_act(shim, obs))


def test_mirror_wrapper_matches_manual_primitives_rot60():
    """make_walk_mirror(rot60=True) == mirror composed OUTSIDE a fresh
    canonicalizer, sector state and all, over a heading sweep."""
    rng = np.random.default_rng(1)
    policy = _policy()
    wrapper = rl_policy.make_walk_mirror(policy, CFG, rot60=True)
    ref_canon = rl_policy.make_walk_canonicalizer(policy, CFG)
    for h in (0.0, 20.0, 100.0, 200.0, -140.0, 0.0):
        th = math.radians(h)
        obs = _walk_obs(rng, 0.05 * math.cos(th), 0.05 * math.sin(th))
        act_port, _ = wrapper.predict(obs)
        np.testing.assert_array_equal(
            act_port, _manual_mirror_act(ref_canon, obs))


def test_mirror_flips_action_chirality():
    """The mirrored stack must differ from the naked one on a forward
    command (it IS a different gait chirality), and mirroring twice
    must return the naked action exactly (involution)."""
    rng = np.random.default_rng(2)
    policy = _policy()
    shim = rl_policy._Rot60ModelShim(policy)
    obs = _walk_obs(rng, 0.05, 0.0)
    act_naked = policy.act(obs)
    wrapper = rl_policy.make_walk_mirror(policy, CFG, rot60=False)
    act_mir, _ = wrapper.predict(obs)
    assert not np.allclose(act_mir, act_naked, atol=1e-4)
    # involution: mirror(mirror(pi))(obs) == pi(obs)
    operm, osign = mirror.obs_perm_sign(walk=True)
    aperm, asign = mirror.joint_perm_sign()
    m_obs = obs[operm] * osign
    act_inner, _ = wrapper.predict(m_obs)
    np.testing.assert_array_equal(
        act_inner[aperm] * asign, np.asarray(act_naked, dtype=act_inner.dtype))


def test_mirror_own_rot60_state_selects_mirrored_sector():
    """A left-of-wedge command must drive the mirror's INTERNAL
    canonicalizer to the reflected (right-of-wedge) sector — proof the
    composition reflects the command before sector selection and that
    the chirality owns its sector state."""
    rng = np.random.default_rng(3)
    policy = _policy()
    wrapper = rl_policy.make_walk_mirror(policy, CFG, rot60=True)
    obs = _walk_obs(rng, 0.05 * math.cos(math.radians(60)),
                    0.05 * math.sin(math.radians(60)))   # +60 deg heading
    wrapper.predict(obs)
    # mirrored command heading is -60 deg -> sector -1 (not +1)
    assert wrapper.model.k == rot60.sector_from_cmd(
        0.05 * math.cos(math.radians(-60)),
        0.05 * math.sin(math.radians(-60)), 0) == -1


def test_selector_left_right_constant():
    assert rl_policy.NAKED_DRIFT_SIGN == +1   # champion drifts LEFT
    left = rl_policy.ChiralitySelector("left")
    right = rl_policy.ChiralitySelector("right")
    for _ in range(100):
        assert left.update(0.5, DT) == "naked"    # left turn = the drift
        assert right.update(0.5, DT) == "mirror"  # right = reflected
    assert left.switches == 0 and right.switches == 0
    # heading integrates regardless (for the episode report)
    assert left.heading > 0


def test_selector_heading_hold_bang_bang():
    sel = rl_policy.ChiralitySelector("hold")
    assert sel.active == "naked"
    hyst = rl_policy.TURN_HYST_RAD
    # inside the band: never switches (no chatter)
    for _ in range(10):
        assert sel.update(0.0, DT) == "naked"
    assert sel.switches == 0
    # drift LEFT past +hysteresis -> mirror (drives back right)
    n = int(1.5 * hyst / (0.5 * DT))
    for _ in range(n):
        sel.update(0.5, DT)
    assert sel.active == "mirror" and sel.switches == 1
    # mirrored drift brings heading back below -hysteresis -> naked
    m = int(3.0 * hyst / (0.5 * DT))
    for _ in range(m):
        sel.update(-0.5, DT)
    assert sel.active == "naked" and sel.switches == 2
    # small wiggle inside the band never switches again
    for g in (0.05, -0.05) * 20:
        sel.update(g, DT)
    assert sel.switches == 2


def test_selector_respects_drift_sign():
    """A future right-drifting champion flips one constant, and the
    left/right mapping plus heading-hold direction follow."""
    left = rl_policy.ChiralitySelector("left", drift_sign=-1)
    assert left.update(0.0, DT) == "mirror"
    hold = rl_policy.ChiralitySelector("hold", drift_sign=-1)
    n = int(2.0 * rl_policy.TURN_HYST_RAD / (0.5 * DT))
    for _ in range(n):
        hold.update(-0.5, DT)     # veers RIGHT (negative heading)
    assert hold.active == "mirror"


def test_run_policy_move_refuses_bad_turn():
    drive = SimpleNamespace(bus=None, dry_run=True)
    r = rl_policy.run_policy_move(drive, "stand", turn="left")
    assert not r["ok"] and "walk-only" in r["error"]
    r = rl_policy.run_policy_move(drive, "walk", turn="sideways")
    assert not r["ok"] and "bad turn" in r["error"]
