from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from rl_move.sim.train_ppo_mjx import (
    _env_kwargs, _recover_cert_bucket_plan, _recover_episode_outcome,
    _recover_score_payload, _run_recover_cert_kind)


class _FakeCertEnv:
    num_envs = 3
    _episode_steps = 4
    _dt = 0.04

    def __init__(self):
        self.tick = 0
        self.forced = None

    def set_attr(self, name, value):
        assert name == "force_recover_start"
        self.forced = value

    def reset(self):
        self.tick = 0
        return np.zeros((self.num_envs, 2), dtype=np.float32)

    def step(self, actions):
        assert np.asarray(actions).shape == (self.num_envs, 1)
        self.tick += 1
        done_at = np.array([1, 2, 4])
        dones = self.tick >= done_at
        infos = [{}, {}, {}]
        if dones[0]:
            infos[0]["termination_reason"] = "recover_success"
        if dones[1]:
            infos[1]["termination_reason"] = "timeout"
        if dones[2]:
            infos[2]["recover_success"] = 1.0
        return (np.zeros((self.num_envs, 2), dtype=np.float32),
                np.zeros(self.num_envs), dones, infos)


class _FakeModel:
    def __init__(self):
        self.calls = []

    def predict(self, obs, *, state, episode_start, deterministic):
        self.calls.append((np.asarray(episode_start).copy(), deterministic))
        return np.zeros((len(obs), 1), dtype=np.float32), state


def test_recover_cert_runner_uses_deterministic_first_episodes():
    env = _FakeCertEnv()
    model = _FakeModel()
    result = _run_recover_cert_kind(env, model, "plant_catch")

    assert env.forced == "plant_catch"
    assert result["outcomes"] == [True, False, True]
    assert result["successes"] == 2
    assert result["episodes"] == 3
    assert result["success"] == 2 / 3
    assert result["time_mean_s"] == (1 + 2 + 4) / 3 * env._dt
    assert all(deterministic for _starts, deterministic in model.calls)
    assert model.calls[0][0].tolist() == [True, True, True]


def test_recover_training_outcome_is_an_episode_fraction_observation():
    assert _recover_episode_outcome({
        "recover_episode_bucket_7": 1.0,
        "recover_start_bucket": 7.0,
        "recover_success": 1.0,
    }) == (7, True)
    assert _recover_episode_outcome({
        "recover_episode_bucket_7": 1.0,
        "recover_start_bucket": 7.0,
        "termination_reason": "timeout",
    }) == (7, False)
    assert _recover_episode_outcome({"recover_start_bucket": 7.0}) is None


def test_recover_cert_plan_keeps_frontier_weakest_and_rotates_history():
    buckets, cursor = _recover_cert_bucket_plan(
        frontier=5, retention_count=2, cursor=0, weak_bucket=3)
    assert buckets == [5, 3, 0, 1]
    assert cursor == 2

    buckets, cursor = _recover_cert_bucket_plan(
        frontier=5, retention_count=2, cursor=cursor, weak_bucket=3)
    assert buckets == [5, 3, 2, 4]
    assert cursor == 0


def test_recover_score_uses_fixed_difficulty_weighted_denominator():
    state = {
        "total_buckets": 4,
        "max_unlocked_bucket": 2,
        "focus_bucket": 2,
        "weakest_bucket": 1,
        "buckets": {
            "0": {"success_fraction": 1.0, "gate_fraction": 1.0,
                  "successes": 8, "episodes": 8},
            "1": {"success_fraction": 0.5, "gate_fraction": 0.5,
                  "successes": 4, "episodes": 8},
            "2": {"success_fraction": 0.25, "gate_fraction": 0.25,
                  "successes": 2, "episodes": 8},
        },
        "sample_probabilities": {"0": 0.1, "1": 0.4, "2": 0.5},
    }
    payload, best = _recover_score_payload(
        state, best_score=0.30, cert_ages={0: 2, 1: 1, 2: 0})

    # B0=1*1.0, B1=2*0.5, B2=3*0.25: 2.75 of the fixed 1+2+3+4=10.
    assert payload["RECOVER_SCORE/overall_points"] == 2.75
    assert payload["RECOVER_SCORE/overall_weighted_success"] == 0.275
    assert payload["RECOVER_SCORE/best_overall_weighted_success"] == 0.30
    assert payload["RECOVER_SCORE/certified_weight_fraction"] == 0.6
    assert payload["RECOVER_SCORE/bucket_02_success_fraction"] == 0.25
    assert payload["RECOVER_SCORE/bucket_02_points"] == 0.75
    assert payload["RECOVER_SCORE/bucket_00_cert_age_rounds"] == 2.0
    assert payload[
        "RECOVER_SCORE/bucket_01_sample_probability"] == 0.4
    assert best == 0.30


def test_mjx_recover_run_opts_into_external_certification():
    args = SimpleNamespace(
        no_dr=False, dr_scale=0.1, episode_seconds=16.0,
        cfg_set=None, recover_cert_every=1_000_000,
        recover_cert_envs=8,
        goal_mix="recover=1.0,walk=0.0")
    params = object()

    kw = _env_kwargs(args, params=params)

    assert kw["params"] is params
    assert kw["cfg"]["goal"]["recover_external_certification"] == 1.0


def test_zero_sized_cert_pool_does_not_freeze_curriculum():
    args = SimpleNamespace(
        no_dr=False, dr_scale=0.1, episode_seconds=16.0,
        cfg_set=None, recover_cert_every=1_000_000, recover_cert_envs=0,
        goal_mix="recover=1.0")

    kw = _env_kwargs(args, params=object())

    assert "cfg" not in kw
