from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from rl_move.sim.train_ppo_mjx import (
    _env_kwargs, _recover_episode_outcome, _run_recover_cert_kind)


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
