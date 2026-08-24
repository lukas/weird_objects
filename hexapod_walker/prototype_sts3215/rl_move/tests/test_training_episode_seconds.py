"""Tests for --training-episode-seconds (walkcurr rung-1 episode-length
curriculum lever, 08-24): lets the TRAINING rollout envs use a shorter
episode than --episode-seconds (which still governs eval/video) so a
fixed --steps budget buys more resets/exploration diversity, without
touching reward pricing, architecture, or reset-pose distribution —
the one axis the 13-mechanism rung-1 campaign never tried.

Default None resolves to --episode-seconds, bit-exact with the
pre-flag behavior (the training env kwargs' own getattr(...) fallback
always hit args.episode_seconds because the attribute did not exist
before this flag).
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]

from rl_move.sim.train_ppo_mjx import (  # noqa: E402
    _resolve_training_episode_seconds,
)


def test_help_text_wires_the_flag():
    out = subprocess.run(
        [sys.executable, "-m", "rl_move.sim.train_ppo_mjx", "--help"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=60,
    )
    assert out.returncode == 0, out.stderr
    assert "--training-episode-seconds" in out.stdout


def test_default_none_is_bit_exact_with_episode_seconds():
    assert _resolve_training_episode_seconds(None, 10.0) == 10.0
    assert _resolve_training_episode_seconds(None, 25.0) == 25.0


def test_explicit_override_wins():
    assert _resolve_training_episode_seconds(4.0, 25.0) == 4.0


def test_nonpositive_override_rejected():
    with pytest.raises(SystemExit, match="--training-episode-seconds"):
        _resolve_training_episode_seconds(0.0, 25.0)
    with pytest.raises(SystemExit, match="--training-episode-seconds"):
        _resolve_training_episode_seconds(-1.0, 25.0)
