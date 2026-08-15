import numpy as np
import pytest

from rl_move.dynamics import data as dd
from rl_move.dynamics import frames as fr


def _episode(global_idx: int, n_frames: int) -> dd.Episode:
    return dd.Episode(
        frames=np.zeros((n_frames, fr.FRAME_DIM), dtype=np.float32),
        actions=np.zeros((n_frames - 1, fr.ACTION_DIM), dtype=np.float32),
        priv=np.zeros((n_frames, fr.PRIV_DIM), dtype=np.float32),
        priv_mask=np.ones(fr.PRIV_DIM, dtype=np.float32),
        actor="random", mode="walk", reason="trunc", dr=0.0,
        global_idx=global_idx,
    )


def test_valid_window_count_matches_sampler_bounds():
    assert dd.valid_window_count(101, 16, (1, 2, 5, 10, 25)) == 61
    assert dd.valid_window_count(40, 16, (25,)) == 0
    assert dd.valid_window_count(41, 16, (25,)) == 1


def test_window_budget_uses_episode_split():
    episodes = [_episode(i, 101 + i) for i in range(20)]
    budget = dd.window_budget(episodes, 16, (1, 5, 25))
    expected = {
        split: sum(
            dd.valid_window_count(len(ep.frames), 16, (1, 5, 25))
            for ep in episodes if ep.split == split)
        for split in dd.SPLITS
    }
    assert budget == expected
    assert sum(budget.values()) == sum(
        dd.valid_window_count(len(ep.frames), 16, (1, 5, 25))
        for ep in episodes)


def test_episode_split_is_stable_disjoint_and_near_80_10_10():
    first = [dd.split_for_episode(i) for i in range(10_000)]
    second = [dd.split_for_episode(i) for i in range(10_000)]
    assert first == second
    assert set(first) == set(dd.SPLITS)
    fractions = {split: first.count(split) / len(first)
                 for split in dd.SPLITS}
    assert fractions["train"] == pytest.approx(0.8, abs=0.02)
    assert fractions["val"] == pytest.approx(0.1, abs=0.02)
    assert fractions["test"] == pytest.approx(0.1, abs=0.02)
    assert all(dd.is_val_episode(i) == (first[i] == "val")
               for i in range(len(first)))
    assert all(dd.is_test_episode(i) == (first[i] == "test")
               for i in range(len(first)))


def test_normalization_uses_train_episodes_only():
    episodes = [_episode(i, 50) for i in range(100)]
    for ep in episodes:
        ep.frames.fill({"train": 2.0, "val": 100.0, "test": -100.0}[ep.split])
    stats = dd.compute_stats(episodes)
    assert stats.mean == pytest.approx(np.full(fr.FRAME_DIM, 2.0))


def test_split_diagnostics_and_coverage_include_all_splits():
    episodes = [_episode(i, 101) for i in range(1000)]
    diagnostics = dd.split_diagnostics(episodes, 16, (1, 5, 25))
    dd.validate_split_coverage(diagnostics)
    assert set(diagnostics) == set(dd.SPLITS)
    assert sum(row["episodes"] for row in diagnostics.values()) == 1000
    assert sum(row["windows"] for row in diagnostics.values()) == 61_000
