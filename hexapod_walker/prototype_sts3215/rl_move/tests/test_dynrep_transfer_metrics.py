from collections import deque

import pytest

from rl_move.dynamics.train_ppo_transfer import rollout_metrics_payload


def test_rollout_metrics_include_episode_and_transition_scores():
    episodes = deque([{"r": 10.0, "l": 20}, {"r": 20.0, "l": 30}])

    payload = rollout_metrics_payload(
        episodes,
        reward_sum=12.0,
        reward_count=6,
        reward_sum_cumulative=30.0,
        reward_count_cumulative=10,
        reward_ema=2.5,
        episodes_completed=4,
        early_terminations=1,
        time_limit_truncations=3,
    )

    assert payload["rollout/ep_rew_mean"] == 15.0
    assert payload["rollout/ep_len_mean"] == 25.0
    assert payload["rollout/reward_per_transition"] == 2.0
    assert payload["rollout/reward_per_transition_cumulative"] == 3.0
    assert payload["rollout/reward_per_transition_ema"] == 2.5
    assert payload["rollout/episodes_completed"] == 4
    assert payload["rollout/early_terminations"] == 1
    assert payload["rollout/time_limit_truncations"] == 3
    assert payload["rollout/early_term_rate"] == pytest.approx(0.25)
    assert payload["rollout/time_limit_rate"] == pytest.approx(0.75)


def test_rollout_metrics_handle_no_completed_episodes():
    payload = rollout_metrics_payload(
        [],
        reward_sum=4.0,
        reward_count=2,
        reward_sum_cumulative=4.0,
        reward_count_cumulative=2,
        reward_ema=2.0,
        episodes_completed=0,
        early_terminations=0,
        time_limit_truncations=0,
    )

    assert payload["rollout/reward_per_transition"] == 2.0
    assert payload["rollout/episodes_completed"] == 0
    assert "rollout/ep_rew_mean" not in payload
    assert "rollout/early_term_rate" not in payload
