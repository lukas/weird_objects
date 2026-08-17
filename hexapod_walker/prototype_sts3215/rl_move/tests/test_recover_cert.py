from __future__ import annotations

import json
import sys
from types import SimpleNamespace

import numpy as np

import rl_move.sim.train_ppo_mjx as train_ppo_mjx
from rl_move.sim.train_ppo_mjx import (
    _RecoverPopulation,
    _env_kwargs, _recover_cert_bucket_plan, _recover_episode_outcome,
    _recover_episode_training_error, _recover_population_choose_candidate,
    _recover_population_all_acked, _recover_population_record,
    _recover_population_release, _recover_score_payload,
    _recover_update_admission_all, _recover_update_regression_timers,
    _run_recover_cert_kind)


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


def test_recover_training_error_uses_explicit_terminal_shortfall():
    assert _recover_episode_training_error({
        "recover_episode_bucket_7": 1.0,
        "recover_start_bucket": 7.0,
        "recover_training_error": 0.35,
    }) == (7, 0.35)
    assert _recover_episode_training_error({
        "recover_episode_bucket_7": 1.0,
        "recover_start_bucket": 7.0,
    }) is None


def test_recover_cert_plan_keeps_frontier_weakest_and_rotates_history():
    buckets, cursor = _recover_cert_bucket_plan(
        frontier=5, retention_count=2, cursor=0, weak_bucket=3)
    assert buckets == [5, 3, 0, 1]
    assert cursor == 2

    buckets, cursor = _recover_cert_bucket_plan(
        frontier=5, retention_count=2, cursor=cursor, weak_bucket=3)
    assert buckets == [5, 3, 2, 4]
    assert cursor == 0


def test_recover_regression_timer_requires_same_bucket_for_elapsed_window():
    failed_since = {}
    assert _recover_update_regression_timers(
        failed_since, {0: 0.5, 1: 1.0}, 10, 0.6, 20) == []
    assert failed_since == {0: 10}

    # A different bucket failing does not inherit B0's elapsed time, and B0
    # recovering clears its own timer.
    assert _recover_update_regression_timers(
        failed_since, {0: 1.0, 1: 0.5}, 20, 0.6, 20) == []
    assert failed_since == {1: 20}
    assert _recover_update_regression_timers(
        failed_since, {0: 1.0, 1: 0.5}, 39, 0.6, 20) == []
    assert _recover_update_regression_timers(
        failed_since, {0: 1.0, 1: 0.5}, 40, 0.6, 20) == [1]


class _FakeAdmissionVec:
    def __init__(self, admissions):
        self.admissions = admissions
        self.calls = []

    def env_method(self, method, *args, **kwargs):
        self.calls.append((method, args, kwargs))
        return self.admissions


def test_recover_admission_updates_every_training_env():
    row = {"active_before": 3, "active_after": 4, "promoted": True}
    env = _FakeAdmissionVec([dict(row) for _ in range(512)])

    admission, synchronized = _recover_update_admission_all(env, 7)

    assert admission == row
    assert synchronized == 512
    assert env.calls == [("_recover_update_admission", (7,), {})]


def test_recover_admission_fails_loudly_on_desynchronized_envs():
    env = _FakeAdmissionVec([
        {"active_before": 3, "active_after": 4, "promoted": True},
        {"active_before": 3, "active_after": 3, "promoted": False},
    ])

    with np.testing.assert_raises_regex(RuntimeError, "desynchronized"):
        _recover_update_admission_all(env, 7)


def test_recover_population_elects_first_candidate_on_common_parent():
    def summary(member, time_ns, parent="winner-B3"):
        row = {
            "population_id": "pop",
            "bucket": 4,
            "member": member,
            "run_id": f"run-{member}",
            "run_name": f"member-{member}",
            "step": 10_000 + member,
            "time_ns": time_ns,
            "parent_fingerprint": parent,
            "policy_file": f"recover_promotions/member-{member}.zip",
            "curriculum_file": (
                f"recover_promotions/member-{member}.curriculum.json"),
            "policy_sha256": f"policy-{member}",
            "curriculum_sha256": f"curriculum-{member}",
        }
        return {"recover_population/candidate_B04": json.dumps(row)}

    peers = [
        (0, "run-0", "member-0", summary(0, 200)),
        (1, "run-1", "member-1", summary(1, 100)),
        # A fast candidate from a stale pre-sync branch is ineligible.
        (2, "run-2", "member-2", summary(2, 50, parent="stale")),
    ]

    winner = _recover_population_choose_candidate(
        peers, bucket=4, parent_fingerprint="winner-B3",
        population_id="pop")

    assert winner is not None
    assert winner["member"] == 1


def test_recover_population_record_accepts_wandb_last_wrapper():
    row = {"bucket": 3, "member": 2}
    summary = {
        "recover_population/winner_B03": {
            "last": json.dumps(row),
        },
    }

    assert _recover_population_record(summary, "winner", 3) == row


def test_recover_population_requires_ack_from_every_matching_member():
    winner = {"bucket": 4, "policy_sha256": "winner-B4"}

    def peer(member, fingerprint="winner-B4", population="pop"):
        run_id = f"run-{member}"
        run_name = f"member-{member}"
        ack = {
            "population_id": population,
            "bucket": 4,
            "member": member,
            "run_id": run_id,
            "run_name": run_name,
            "policy_sha256": fingerprint,
        }
        summary = {
            "recover_population/ack_B04": json.dumps(ack),
        }
        return member, run_id, run_name, summary

    peers = [peer(0), peer(1), peer(2)]
    assert _recover_population_all_acked(peers, winner, "pop", 3)
    assert not _recover_population_all_acked(peers[:2], winner, "pop", 3)
    assert not _recover_population_all_acked(
        [peer(0), peer(1, fingerprint="stale"), peer(2)],
        winner, "pop", 3)
    assert not _recover_population_all_acked(
        [peer(0), peer(1, population="other"), peer(2)],
        winner, "pop", 3)


def test_recover_population_force_refreshes_cached_peer_summaries():
    class ApiRun:
        def __init__(self, summary):
            self.summary = summary
            self.loads = []

        def load(self, force=False):
            self.loads.append(force)

    runs = {
        "run-0": ApiRun({"remote": 0}),
        "run-1": ApiRun({"remote": 1}),
        "run-2": ApiRun({"remote": 2}),
    }

    class Api:
        def run(self, path):
            return runs[path.rsplit("/", 1)[-1]]

    population = object.__new__(_RecoverPopulation)
    population.peer_names = ("member-0", "member-1", "member-2")
    population._peer_ids = {
        f"member-{member}": f"run-{member}" for member in range(3)
    }
    population.project_path = "entity/project"
    population.api = Api()
    population.run = SimpleNamespace(id="run-0")
    population._local_summary = {"local": True}
    population._api_runs = {}

    rows = population._peer_rows()

    assert [row[:3] for row in rows] == [
        (0, "run-0", "member-0"),
        (1, "run-1", "member-1"),
        (2, "run-2", "member-2"),
    ]
    assert rows[0][3] == {"remote": 0, "local": True}
    assert all(run.loads == [True] for run in runs.values())


def test_recover_population_next_election_requires_leader_release(
        monkeypatch):
    winner_b1 = {
        "population_id": "pop",
        "bucket": 1,
        "member": 0,
        "run_id": "run-0",
        "run_name": "member-0",
        "policy_sha256": "winner-B1",
    }
    candidate_b2 = {
        "population_id": "pop",
        "bucket": 2,
        "member": 1,
        "run_id": "run-1",
        "run_name": "member-1",
        "step": 2_000,
        "time_ns": 10,
        "parent_fingerprint": "winner-B1",
        "policy_file": "recover_promotions/member-1.zip",
        "curriculum_file": "recover_promotions/member-1.curriculum.json",
        "policy_sha256": "winner-B2",
        "curriculum_sha256": "curriculum-B2",
    }
    leader_summary = {
        "recover_population/winner_B01": json.dumps(winner_b1),
    }
    peers = [
        (0, "run-0", "member-0", leader_summary),
        (1, "run-1", "member-1", {
            "recover_population/candidate_B02": json.dumps(candidate_b2),
        }),
        (2, "run-2", "member-2", {}),
    ]
    population = object.__new__(_RecoverPopulation)
    population.member = 0
    population.peer_names = ("member-0", "member-1", "member-2")
    population.initial_bucket = 0
    population.population_id = "pop"
    population.run = SimpleNamespace(summary={"global_step": 2_000})
    population._local_summary = {}
    population._summary_update = lambda values: (
        population._local_summary.update(values))
    monkeypatch.setitem(
        sys.modules, "wandb", SimpleNamespace(log=lambda _payload: None))

    population._elect(peers, leader_summary)
    assert "recover_population/winner_B02" not in leader_summary

    leader_summary["recover_population/release_B01"] = json.dumps({
        "population_id": "pop",
        "bucket": 1,
        "policy_sha256": "winner-B1",
    })
    population._elect(peers, leader_summary)
    assert _recover_population_record(
        leader_summary, "winner", 2)["policy_sha256"] == "winner-B2"


def test_recover_population_release_is_bound_to_winner_hash():
    winner = {"bucket": 2, "policy_sha256": "winner-B2"}
    summary = {
        "recover_population/release_B02": json.dumps({
            "population_id": "pop",
            "bucket": 2,
            "policy_sha256": "winner-B2",
        }),
    }
    assert _recover_population_release(summary, winner, "pop") is not None
    winner["policy_sha256"] = "other"
    assert _recover_population_release(summary, winner, "pop") is None


def test_recover_population_leader_releases_only_after_all_acks(monkeypatch):
    winner = {
        "population_id": "pop",
        "bucket": 1,
        "member": 2,
        "run_id": "run-2",
        "policy_sha256": "winner-B1",
    }
    leader_summary = {}

    def peer(member):
        run_id = f"run-{member}"
        run_name = f"member-{member}"
        ack = {
            "population_id": "pop",
            "bucket": 1,
            "member": member,
            "run_id": run_id,
            "run_name": run_name,
            "policy_sha256": "winner-B1",
        }
        summary = leader_summary if member == 0 else {}
        summary["recover_population/ack_B01"] = json.dumps(ack)
        return member, run_id, run_name, summary

    peers = [peer(0), peer(1), peer(2)]
    population = object.__new__(_RecoverPopulation)
    population.population_id = "pop"
    population.member = 0
    population.peer_names = ("member-0", "member-1", "member-2")
    population.poll_seconds = 0.01
    population.barrier_timeout = 1.0
    population._flush_acks = lambda: None
    population._peer_rows = lambda: peers
    population._local_summary = {}

    def summary_update(values):
        leader_summary.update(values)
        population._local_summary.update(values)

    population._summary_update = summary_update
    monkeypatch.setitem(
        sys.modules, "wandb", SimpleNamespace(log=lambda _payload: None))

    population.wait_for_release(
        {"bucket": 1, "population_record": winner}, global_step=2_000)

    release = _recover_population_record(leader_summary, "release", 1)
    assert release is not None
    assert release["policy_sha256"] == "winner-B1"
    assert release["member_count"] == 3


def test_recover_population_publishes_only_after_checkpoint_upload(
        tmp_path, monkeypatch):
    events = []

    class Summary(dict):
        def update(self, values):
            events.append("summary")
            super().update(values)

    class Run:
        id = "run-0"
        summary = Summary()
        fail_save = True

        def save(self, path, **_kwargs):
            events.append(f"save:{path}")
            if self.fail_save:
                raise RuntimeError("upload unavailable")

    policy_dir = tmp_path / "policies"
    promotion_dir = policy_dir / "recover_promotions"
    promotion_dir.mkdir(parents=True)
    policy = promotion_dir / "candidate.zip"
    curriculum = promotion_dir / "candidate.curriculum.json"
    policy.write_bytes(b"policy")
    curriculum.write_text("{}")
    monkeypatch.setattr(train_ppo_mjx, "POLICY_DIR", policy_dir)
    monkeypatch.setitem(
        sys.modules, "wandb",
        SimpleNamespace(log=lambda _payload: events.append("metric")))

    population = object.__new__(_RecoverPopulation)
    population.population_id = "pop"
    population.member = 0
    population.peer_names = ("member-0", "member-1", "member-2")
    population.run = Run()
    population.parent_fingerprint = "root:pop"
    population._local_summary = {}
    population._pending_candidates = {}

    checkpoint = {
        "path": policy,
        "metadata_path": curriculum,
        "bucket": 1,
        "step": 1_000,
    }
    population.publish_candidate(checkpoint)
    assert 1 in population._pending_candidates
    assert "recover_population/candidate_B01" not in population.run.summary

    events.clear()
    population.run.fail_save = False
    population._flush_candidates()

    assert 1 not in population._pending_candidates
    assert events[0].startswith("save:")
    assert events[1].startswith("save:")
    assert events[2:] == ["summary", "metric"]
    assert "recover_population/candidate_B01" in population.run.summary


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
        "training_errors": {
            "0": {"ema": 0.1, "episodes": 20},
            "1": {"ema": 0.6, "episodes": 30},
            "2": {"ema": 0.8, "episodes": 40},
        },
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
    assert payload["RECOVER_SCORE/bucket_02_training_error_ema"] == 0.8
    assert payload[
        "RECOVER_SCORE/bucket_02_training_error_episodes"] == 40.0
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
