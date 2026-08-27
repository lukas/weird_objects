"""CLI-level tests for --gru-dual-log-std-split / --log-std-anneal-core
(08-27, anchor4-stdanneal/anchor5-stdmild dose-bracket dig-in). Mirrors
test_use_sde_flag.py's pattern: pull the validation into a pure
function so it's unit-testable without mujoco/GPU, plus one real
--help subprocess run (the only check that catches a stray literal
"%" or similar argparse-formatter crash).
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.sim.train_ppo_mjx import (  # noqa: E402
    _validate_gru_dual_log_std_split,
)


def test_help_text_wires_both_flags():
    out = subprocess.run(
        [sys.executable, "-m", "rl_move.sim.train_ppo_mjx", "--help"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=60,
    )
    assert out.returncode == 0, out.stderr
    assert "--gru-dual-log-std-split" in out.stdout
    assert "--log-std-anneal-core" in out.stdout


def test_default_off_is_a_noop():
    # Bit-exact legacy: log_std_split=False never raises, regardless
    # of --gru-dual.
    _validate_gru_dual_log_std_split(False, False)
    _validate_gru_dual_log_std_split(False, True)


def test_with_gru_dual_allowed():
    _validate_gru_dual_log_std_split(True, True)


def test_without_gru_dual_refused():
    with pytest.raises(SystemExit, match="--gru-dual-log-std-split"):
        _validate_gru_dual_log_std_split(True, False)
