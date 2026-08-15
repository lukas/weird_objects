import numpy as np

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
    expected_train = sum(
        dd.valid_window_count(len(ep.frames), 16, (1, 5, 25))
        for ep in episodes if not dd.is_val_episode(ep.global_idx))
    expected_val = sum(
        dd.valid_window_count(len(ep.frames), 16, (1, 5, 25))
        for ep in episodes if dd.is_val_episode(ep.global_idx))
    assert budget == {"train": expected_train, "val": expected_val}
    assert budget["train"] + budget["val"] == sum(
        dd.valid_window_count(len(ep.frames), 16, (1, 5, 25))
        for ep in episodes)
