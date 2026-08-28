"""RecurrentPredictor / wrap_recurrent_predictor (2026-08-28 runner-bug
audit, operator kick 20260828T153954Z).

Found live: manual_drive_session.py / drive_policy.py / view.py all ran
GRU (RecurrentPPO) checkpoints through the stateless ``model.predict``
path, which SB3 implements as "state=None -> fresh ZERO hidden state" on
EVERY call — the whole 08-28 manual-drive feel report evaluated a
memory-less lobotomy of the policy, not the policy. eval_checkpoint.py
always had the correct shim (_RecurrentPredictor); it now lives in
gru_policy as the shared canonical implementation. These tests lock:

1. wrap_recurrent_predictor passes a plain (non-recurrent) PPO through
   unchanged;
2. for a RecurrentPPO, the wrapper's action sequence matches manual
   hidden-state threading through policy.predict exactly;
3. the wrapper's actions DIVERGE from the stateless model.predict path
   after the first step (i.e. hidden state actually matters and is
   actually carried), and .reset() restores the step-0 action.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import gymnasium as gym  # noqa: E402


def _tiny_env():
    return gym.make("Pendulum-v1")


def _tiny_recurrent(seed=0):
    from sb3_contrib import RecurrentPPO
    return RecurrentPPO(
        "MlpLstmPolicy", _tiny_env(), seed=seed, n_steps=8, batch_size=8,
        policy_kwargs=dict(lstm_hidden_size=16, net_arch=[16]),
        device="cpu")


def test_plain_ppo_passthrough():
    from stable_baselines3 import PPO
    from rl_move.sim.gru_policy import wrap_recurrent_predictor
    m = PPO("MlpPolicy", _tiny_env(), seed=0, n_steps=8, batch_size=8,
            policy_kwargs=dict(net_arch=[16]), device="cpu")
    assert wrap_recurrent_predictor(m) is m


def test_wrapper_matches_manual_state_threading():
    from rl_move.sim.gru_policy import wrap_recurrent_predictor
    m = _tiny_recurrent()
    wrapped = wrap_recurrent_predictor(m)
    assert wrapped is not m
    rng = np.random.default_rng(0)
    obs_seq = [rng.normal(size=3).astype(np.float32) for _ in range(6)]
    got = [wrapped.predict(o, deterministic=True)[0].copy()
           for o in obs_seq]
    # manual threading reference
    state = None
    starts = np.ones((1,), dtype=bool)
    want = []
    for o in obs_seq:
        a, state = m.policy.predict(o, state=state, episode_start=starts,
                                    deterministic=True)
        starts = np.zeros((1,), dtype=bool)
        want.append(a.copy())
    for g, w in zip(got, want):
        np.testing.assert_array_equal(g, w)


def test_wrapper_diverges_from_stateless_and_resets():
    from rl_move.sim.gru_policy import wrap_recurrent_predictor
    m = _tiny_recurrent()
    wrapped = wrap_recurrent_predictor(m)
    rng = np.random.default_rng(1)
    obs_seq = [rng.normal(size=3).astype(np.float32) for _ in range(8)]
    threaded = [wrapped.predict(o, deterministic=True)[0].copy()
                for o in obs_seq]
    stateless = [m.predict(o, deterministic=True)[0].copy()
                 for o in obs_seq]
    # step 0 agrees (both start from a zero state)...
    np.testing.assert_allclose(threaded[0], stateless[0], atol=1e-7)
    # ...but the trajectories must diverge once memory accumulates
    # (an untrained GRU still has nonzero recurrent weights).
    diffs = [float(np.max(np.abs(t - s)))
             for t, s in zip(threaded[1:], stateless[1:])]
    assert max(diffs) > 1e-6, (
        "state threading had no effect — wrapper is not carrying the "
        f"hidden state (max action diff {max(diffs):.2e})")
    # reset() clears the state: first action repeats exactly
    wrapped.reset()
    a0 = wrapped.predict(obs_seq[0], deterministic=True)[0]
    np.testing.assert_array_equal(a0, threaded[0])
