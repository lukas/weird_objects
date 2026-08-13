"""distill_gru --transitions tests (TRANSITIONS_DIRECTIVE CODE item 2).

Locks the sequence-demo collector that rides goal.mode_seq (item 1):

1. default OFF: --transitions defaults to 0 and the flag requires
   --dual (failure-ledger lesson 1) before any heavy work;
2. per-tick teacher ROUTING: in one continuous sequence episode the
   label at every tick comes from the ACTIVE segment's teacher (walk
   teacher on walk ticks, stance teacher everywhere else) and the obs
   mode one-hot agrees with the recorded per-tick mode;
3. the demo stream is continuous (one episode array, mode flips
   mid-array, no reset between segments);
4. the in-context teacher verification aborts loudly when the verify
   window budget is exceeded;
5. collect_transitions refuses an env without goal.mode_seq=1.
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
from rl_move.sim.distill_gru import (  # noqa: E402
    collect_transitions, main as distill_main,
)
from rl_move.sim.walk_task import (  # noqa: E402
    MODE_ONEHOT_ORDER, SimHexapodJointWalkEnv,
)

N_ONEHOT = len(MODE_ONEHOT_ORDER)
WALK_CONST, STANCE_CONST = 0.05, -0.05


class _StubTeacher:
    """Constant-action teacher; enough API for collect_transitions."""

    class _Policy:
        def predict_values(self, t):
            return np.float32(0.0)

    def __init__(self, const: float):
        self.const = float(const)
        self.policy = self._Policy()

    def predict(self, obs, deterministic=True):
        return np.full(18, self.const, dtype=np.float32), None


def _make_seq_env(seed: int = 1, episode_seconds: float = 10.0,
                  mix: dict | None = None, seq: bool = True):
    cfg = load_config()
    g = cfg.setdefault("goal", {})
    if seq:
        g["mode_seq"] = 1.0
        g["mode_seq_segment_s_min"] = 3.0
        g["mode_seq_segment_s_max"] = 4.0
    cfg.setdefault("obs", {})["mode_onehot"] = 1.0
    env = SimHexapodJointWalkEnv(cfg, seed=seed,
                                 episode_seconds=episode_seconds)
    if mix is not None:
        gen = env._goal_gen
        for a in [a for a in vars(gen) if a.startswith("p_")]:
            setattr(gen, a, 0.0)
        gen.p_walk = 0.0
        for m, p in mix.items():
            setattr(gen, f"p_{m}", p)
    return env


def _teachers(env):
    n_env = int(env.observation_space.shape[0])
    return {"walk": (_StubTeacher(WALK_CONST), n_env - N_ONEHOT),
            "stance": (_StubTeacher(STANCE_CONST), 68)}


# ---------------------------------------------------------------------------
# 1. CLI defaults / guards
# ---------------------------------------------------------------------------

def test_transitions_defaults_off_and_requires_dual():
    # --transitions without --dual must exit before any teacher load.
    with pytest.raises(SystemExit, match="--dual"):
        distill_main(["--transitions", "1"])


# ---------------------------------------------------------------------------
# 2/3. routing + continuity on a real mode_seq env
# ---------------------------------------------------------------------------

def test_routing_and_onehot_agree():
    env = _make_seq_env(mix={"lower": 1.0})   # lower -> rise: a switch
    teachers = _teachers(env)
    rng = np.random.default_rng(0)
    episodes, stats = collect_transitions(
        env, teachers, n_ep=1, stochastic_frac=0.0, rng=rng,
        verify_n=0, verify_max_falls=99)
    env.close()
    assert len(episodes) == 1
    tag, obs, act, val = episodes[0]
    assert tag == "seq"
    modes = stats["eps"][0]["modes"]
    assert len(modes) == act.shape[0] == obs.shape[0] == val.shape[0]
    # one continuous stream with >= 2 distinct modes (a real switch)
    assert len(set(modes)) >= 2
    assert stats["eps"][0]["switches"] >= 1
    for t, mode in enumerate(modes):
        want = WALK_CONST if mode == "walk" else STANCE_CONST
        assert float(act[t, 0]) == pytest.approx(want), (t, mode)
        onehot = obs[t, -N_ONEHOT:]
        assert onehot[MODE_ONEHOT_ORDER.index(mode)] == 1.0, (t, mode)
        assert onehot.sum() == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# 4. verification abort path
# ---------------------------------------------------------------------------

def test_verify_abort_fires():
    env = _make_seq_env(mix={"lower": 1.0})
    teachers = _teachers(env)
    rng = np.random.default_rng(0)
    # max_falls=-1 makes any verify window (even 0 falls) exceed budget.
    with pytest.raises(SystemExit, match="SEQUENCE-COMPETENT"):
        collect_transitions(env, teachers, n_ep=2, stochastic_frac=0.0,
                            rng=rng, verify_n=1, verify_max_falls=-1)
    env.close()


# ---------------------------------------------------------------------------
# 5. refuse a non-mode_seq env
# ---------------------------------------------------------------------------

def test_refuses_env_without_mode_seq():
    env = _make_seq_env(seq=False)
    teachers = _teachers(env)
    with pytest.raises(SystemExit, match="mode_seq"):
        collect_transitions(env, teachers, n_ep=1, stochastic_frac=0.0,
                            rng=np.random.default_rng(0))
    env.close()
