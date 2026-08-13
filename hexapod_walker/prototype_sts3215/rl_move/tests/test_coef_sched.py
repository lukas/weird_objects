"""In-run coefficient scheduler (sched.* cfg keys, 2026-08-13).

The scheduler linearly ramps ONE cfg coefficient during training by
GLOBAL env steps (ticks * sched.n_envs). Contract under test:

- default OFF is bit-exact: no cfg mutation, no tracking, no info key;
- ramp math: v0 before t0_steps, linear between, v1 at/after t1_steps;
- the n_envs clock conversion (each env tick advances n_envs steps);
- a partial sched spec is a loud ValueError at env construction, never
  a silently mis-clocked run.
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

mujoco = pytest.importorskip("mujoco")

from rl_move.config import cfg_get, load_config  # noqa: E402
from rl_move.sim.sim_env import N_ACT, SimHexapodBalanceEnv  # noqa: E402

KEY = "reward.k_drag_stance"


def _make_env(sched: dict | None, seed: int = 0):
    cfg = load_config()
    if sched is not None:
        cfg["sched"] = dict(sched)
    return SimHexapodBalanceEnv(seed=seed, cfg=cfg, randomize=False,
                                episode_seconds=2.0)


def test_sched_off_is_bitexact_inert():
    env = _make_env(None)
    before = copy.deepcopy(env.cfg)
    env.reset()
    for _ in range(5):
        _, _, term, trunc, info = env.step(np.zeros(N_ACT))
        assert "sched_value" not in info
        if term or trunc:
            break
    assert env.cfg == before          # no cfg key ever written
    assert env._sched_ticks == 0      # clock never advances
    assert env._sched_value is None


def test_sched_ramp_values_and_info():
    env = _make_env({"key": KEY, "v0": 0.0, "v1": 8000.0,
                     "t0_steps": 4, "t1_steps": 8, "n_envs": 1})
    env.reset()
    seen = []
    for _ in range(10):
        _, _, term, trunc, info = env.step(np.zeros(N_ACT))
        got = cfg_get(env.cfg, "reward", "k_drag_stance", default=None)
        assert got == pytest.approx(info["sched_value"])
        seen.append(info["sched_value"])
        if term or trunc:
            break
    assert len(seen) >= 9
    # ticks 1..4 -> t = 1..4 <= t0 -> v0
    assert seen[0] == pytest.approx(0.0)
    assert seen[3] == pytest.approx(0.0)
    # tick 6 -> t = 6, halfway through [4, 8] -> midpoint
    assert seen[5] == pytest.approx(4000.0)
    # tick 8+ -> v1, and it stays there
    assert seen[7] == pytest.approx(8000.0)
    assert seen[8] == pytest.approx(8000.0)


def test_sched_n_envs_clock_conversion():
    # n_envs=4: each env tick advances the global clock by 4 steps, so
    # the [8, 16] ramp completes after 4 env ticks.
    env = _make_env({"key": KEY, "v0": 100.0, "v1": 500.0,
                     "t0_steps": 8, "t1_steps": 16, "n_envs": 4})
    env.reset()
    seen = []
    for _ in range(5):
        _, _, term, trunc, info = env.step(np.zeros(N_ACT))
        seen.append(info["sched_value"])
        if term or trunc:
            break
    assert seen[0] == pytest.approx(100.0)   # t=4  <= 8
    assert seen[1] == pytest.approx(100.0)   # t=8  == t0
    assert seen[2] == pytest.approx(300.0)   # t=12, midpoint
    assert seen[3] == pytest.approx(500.0)   # t=16 == t1
    assert seen[4] == pytest.approx(500.0)   # holds v1


def test_sched_clock_survives_reset():
    # The clock is per-process and monotone: an episode reset must NOT
    # rewind the schedule (pool-restore bug class).
    env = _make_env({"key": KEY, "v0": 0.0, "v1": 10.0,
                     "t0_steps": 0, "t1_steps": 10, "n_envs": 1})
    env.reset()
    for _ in range(4):
        env.step(np.zeros(N_ACT))
    ticks = env._sched_ticks
    env.reset()
    assert env._sched_ticks == ticks
    _, _, _, _, info = env.step(np.zeros(N_ACT))
    assert info["sched_value"] == pytest.approx(
        min(ticks + 1, 10) / 10.0 * 10.0)


@pytest.mark.parametrize("missing", ["v0", "v1", "t0_steps",
                                     "t1_steps", "n_envs"])
def test_sched_partial_spec_raises(missing):
    spec = {"key": KEY, "v0": 0.0, "v1": 1.0,
            "t0_steps": 0, "t1_steps": 10, "n_envs": 1}
    del spec[missing]
    with pytest.raises(ValueError, match="sched"):
        _make_env(spec)


def test_sched_bad_bounds_raise():
    with pytest.raises(ValueError, match="t1_steps > t0_steps"):
        _make_env({"key": KEY, "v0": 0.0, "v1": 1.0,
                   "t0_steps": 10, "t1_steps": 10, "n_envs": 1})
    with pytest.raises(ValueError, match="n_envs"):
        _make_env({"key": KEY, "v0": 0.0, "v1": 1.0,
                   "t0_steps": 0, "t1_steps": 10, "n_envs": 0})
    with pytest.raises(ValueError, match="dotted"):
        _make_env({"key": "k_drag_stance", "v0": 0.0, "v1": 1.0,
                   "t0_steps": 0, "t1_steps": 10, "n_envs": 1})
