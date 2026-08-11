"""Explicit mode/command one-hot tests (obs.mode_onehot; RL_PLAN 2.4).

The flagship unified-policy MDP gives the policy the commanded skill
FAMILY as a direct input: a 6-wide one-hot (hold/rise/lower/walk/
turn/quad) appended at the tail of every obs frame. These tests lock:

1. default OFF = the obs contract of every existing checkpoint is
   untouched (width bit-exact);
2. ON = +6 dims, the right family bit for every goal-mix mode,
   constant across the episode, present in every history frame;
3. the mirror maps treat the one-hot as mirror-invariant and stay
   involutions at the widened layout.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.mirror import (  # noqa: E402
    FRAME_WALK, frame_perm_sign, obs_perm_sign,
)
from rl_move.sim.walk_task import (  # noqa: E402
    MODE_ONEHOT_ORDER, N_MODE_OBS, SimHexapodJointWalkEnv, mode_onehot,
)

ALL_MODES = ("hold", "lean", "track", "unload", "raise",
             "rise", "lower", "walk", "quad")


def _make_env(mode: str | None = None, *, onehot: bool = True,
              hist: int = 1, seed: int = 0) -> SimHexapodJointWalkEnv:
    cfg = load_config()
    obs_cfg = cfg.setdefault("obs", {})
    if onehot:
        obs_cfg["mode_onehot"] = 1.0
    if hist != 1:
        obs_cfg["history_frames"] = hist
    env = SimHexapodJointWalkEnv(cfg, seed=seed)
    if mode is not None:
        g = env._goal_gen
        for m in ALL_MODES:
            if hasattr(g, f"p_{m}"):
                setattr(g, f"p_{m}", 0.0)
        setattr(g, f"p_{mode}", 1.0)
    return env


# ---------------------------------------------------------------------------
# The helper itself
# ---------------------------------------------------------------------------

def test_mode_onehot_families():
    exp = {"hold": "hold", "lean": "hold", "track": "hold",
           "unload": "hold", "raise": "rise", "rise": "rise",
           "lower": "lower", "walk": "walk", "quad": "quad"}
    for mode, fam in exp.items():
        v = mode_onehot(mode)
        assert v.sum() == 1.0
        assert v[MODE_ONEHOT_ORDER.index(fam)] == 1.0
    # unknown / missing -> hold (never lights a motion bit)
    assert mode_onehot("no_such_mode")[0] == 1.0
    # "turn" slot is reserved: no goal-mix mode maps onto it today
    turn = MODE_ONEHOT_ORDER.index("turn")
    for mode in ALL_MODES:
        assert mode_onehot(mode)[turn] == 0.0


# ---------------------------------------------------------------------------
# Env obs contract
# ---------------------------------------------------------------------------

def test_default_off_width_unchanged():
    env = _make_env(onehot=False)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (FRAME_WALK,)
    assert env.observation_space.shape == (FRAME_WALK,)


def test_on_width_plus_six():
    env = _make_env("walk")
    obs, _ = env.reset(seed=0)
    assert obs.shape == (FRAME_WALK + N_MODE_OBS,)
    assert env.observation_space.shape == (FRAME_WALK + N_MODE_OBS,)


@pytest.mark.parametrize("mode,fam", [
    ("hold", "hold"), ("track", "hold"), ("unload", "hold"),
    ("raise", "rise"), ("rise", "rise"), ("lower", "lower"),
    ("walk", "walk"), ("quad", "quad"),
])
def test_mode_bit_matches_episode_mode(mode, fam):
    env = _make_env(mode, seed=1)
    obs, _ = env.reset(seed=1)
    want = np.zeros(N_MODE_OBS)
    want[MODE_ONEHOT_ORDER.index(fam)] = 1.0
    assert np.array_equal(obs[-N_MODE_OBS:], want), \
        f"reset obs tail wrong for mode {mode}"
    assert env._goal_traj.mode == mode
    # constant across the episode (the command, not a measurement)
    a = np.zeros(env.action_space.shape, dtype=np.float32)
    for _ in range(5):
        obs, _, term, trunc, _ = env.step(a)
        assert np.array_equal(obs[-N_MODE_OBS:], want)
        if term or trunc:
            break


def test_history_frames_carry_mode_in_every_frame():
    env = _make_env("walk", hist=3, seed=2)
    obs, _ = env.reset(seed=2)
    w = FRAME_WALK + N_MODE_OBS
    assert obs.shape == (3 * w,)
    want = np.zeros(N_MODE_OBS)
    want[MODE_ONEHOT_ORDER.index("walk")] = 1.0
    for k in range(3):
        frame = obs[k * w:(k + 1) * w]
        assert np.array_equal(frame[-N_MODE_OBS:], want)


# ---------------------------------------------------------------------------
# Mirror maps at the widened layout
# ---------------------------------------------------------------------------

def test_mirror_maps_mode_onehot():
    perm, sign = frame_perm_sign(walk=True, mode_onehot=True)
    assert len(perm) == FRAME_WALK + N_MODE_OBS
    # tail block: identity perm, +1 sign (mirror-invariant command)
    tail = np.arange(FRAME_WALK, FRAME_WALK + N_MODE_OBS)
    assert np.array_equal(perm[FRAME_WALK:], tail)
    assert np.all(sign[FRAME_WALK:] == 1.0)
    # still an involution, incl. with yaw/phase/history composed
    for kw in (dict(), dict(yaw_cmd=True, phase_obs=True)):
        p, s = obs_perm_sign(walk=True, mode_onehot=True,
                             history_frames=4, **kw)
        x = np.random.default_rng(3).normal(size=len(p))
        y = s * x[p]
        assert np.allclose(s * y[p], x)
