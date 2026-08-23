"""Tests for --use-sde / --sde-sample-freq (walkcurr gSDE exploration
lever, 08-23): SB3 generalized State-Dependent Exploration samples ONE
noise matrix per rollout (temporally correlated) instead of fresh
i.i.d. per-timestep Gaussian noise. Motivated by the walkcurr rung-1
freeze diagnosis — train/clip_fraction collapses to exactly 0 early in
every one of 8 FAILed from-scratch arms (i.i.d.-noise-scale and
entropy-coefficient levers both refuted) regardless of reward pricing.

Default OFF, bit-exact (SB3's own use_sde=False default); only applies
to from-scratch/transplant builds, mirroring --activation-fn's own
restriction.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]

from rl_move.sim.train_ppo_mjx import (  # noqa: E402
    _validate_use_sde_scratch_only,
)


def test_help_text_wires_both_flags():
    # Regression: a stray literal "%" in a help string crashes
    # argparse's formatter at --help time (found + fixed while adding
    # this flag: "~15%% of" -> "%o format" TypeError). Running --help
    # for real is the only check that catches that class of bug.
    out = subprocess.run(
        [sys.executable, "-m", "rl_move.sim.train_ppo_mjx", "--help"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=60,
    )
    assert out.returncode == 0, out.stderr
    assert "--use-sde" in out.stdout
    assert "--sde-sample-freq" in out.stdout


def test_default_off_is_a_noop():
    # Bit-exact legacy: use_sde=False never raises, regardless of
    # init-from state.
    _validate_use_sde_scratch_only(False, None, False, False)
    _validate_use_sde_scratch_only(False, Path("ckpt.zip"), False, False)


def test_scratch_and_transplant_builds_allowed():
    _validate_use_sde_scratch_only(True, None, False, False)  # scratch
    _validate_use_sde_scratch_only(True, Path("ckpt.zip"), True, False)
    _validate_use_sde_scratch_only(True, Path("ckpt.zip"), False, True)


def test_plain_warm_start_refused():
    with pytest.raises(SystemExit, match="--use-sde"):
        _validate_use_sde_scratch_only(True, Path("ckpt.zip"), False, False)
