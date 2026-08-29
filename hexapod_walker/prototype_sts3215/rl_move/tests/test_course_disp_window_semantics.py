"""Sub-stride window-dose bank for reward.k_walk_course_disp.

WHAT THIS BANK PROVES (standwalk coursedisp-c1 DIG-IN follow-up,
2026-08-29): the k_walk_course_disp pricing ORDERING invariants
(obedience out-earns skew/stall/park/wrongway) hold when the trailing
displacement window shrinks BELOW the tripod half-stride (~0.375 s at
walk_phase_hz=1.333), where the term stops integrating intra-stride
sway away and starts pricing the per-tick zigzag that the harness's
`direction_err_mean_deg` headline actually measures.

Why sub-stride windows are the next lever, measured (probe_dir_floor,
08-29 dig-in): the scripted teacher under the standwalk contract
(mesh, 100 Hz, 0.375 deg/tick slew, 0.08 m/s cmd, DR-0) reads per-tick
dir_err mean 13.5 / med 5.4 deg while the coursedisp-c1 policy reads
60.3 / 32.4 on its own real eval ticks -- the 25 Hz-era "~35 deg
structural sway floor" is cadence-specific (31.5 deg re-measured at
25 Hz) and does NOT protect the policy's excess at 100 Hz.  The 1.5 s
default window cannot see that excess (every window 0.75-6.0 s read
~6 deg on the failed lineage); a sub-stride window can.

Same harness as test_course_disp_semantics (PHASEDIR2 stack, course
EMA off, disp on at matched coefficients), windows parametrized.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_phasedir_semantics as pd                          # noqa: E402
from test_course_disp_semantics import DISP_STACK             # noqa: E402

HEADING_BINS = pd.HEADING_BINS
SEEDS = pd.SEEDS

WINDOWS_S = (0.35, 0.15)


def _stack_at_window(window_s: float) -> dict:
    stack = dict(DISP_STACK)
    stack[("reward", "walk_course_disp_window_s")] = float(window_s)
    return stack


def _mean_disp(drive: str, heading: float, window_s: float) -> float:
    stack = _stack_at_window(window_s)
    return float(np.mean([
        pd._phasedir_rollout(drive, s, heading, stack)
        for s in SEEDS]))


@pytest.fixture(scope="module")
def win_returns() -> dict[str, float]:
    out: dict[str, float] = {}
    for w in WINDOWS_S:
        for bin_name, h in HEADING_BINS.items():
            for drive in ("obey", "skew", "stall", "park"):
                out[f"w{w}_{bin_name}_{drive}"] = _mean_disp(
                    drive, h, w)
        for bin_name in ("left", "rear"):
            out[f"w{w}_{bin_name}_wrongway"] = _mean_disp(
                "wrongway", HEADING_BINS[bin_name], w)
    return out


@pytest.mark.parametrize("window_s", WINDOWS_S)
@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_win_obey_beats_offcourse_skew(win_returns, window_s, bin_name):
    r = win_returns
    assert (r[f"w{window_s}_{bin_name}_obey"]
            > r[f"w{window_s}_{bin_name}_skew"] + 50.0), {
        k: v for k, v in r.items()
        if k.startswith(f"w{window_s}_{bin_name}")}


@pytest.mark.parametrize("window_s", WINDOWS_S)
@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_win_obey_beats_refusal_stall(win_returns, window_s, bin_name):
    r = win_returns
    assert (r[f"w{window_s}_{bin_name}_obey"]
            > r[f"w{window_s}_{bin_name}_stall"] + 100.0), {
        k: v for k, v in r.items()
        if k.startswith(f"w{window_s}_{bin_name}")}


@pytest.mark.parametrize("window_s", WINDOWS_S)
@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_win_obey_beats_park(win_returns, window_s, bin_name):
    r = win_returns
    assert (r[f"w{window_s}_{bin_name}_obey"]
            > r[f"w{window_s}_{bin_name}_park"] + 100.0), {
        k: v for k, v in r.items()
        if k.startswith(f"w{window_s}_{bin_name}")}


@pytest.mark.parametrize("window_s", WINDOWS_S)
@pytest.mark.parametrize("bin_name", ["left", "rear"])
def test_win_obey_beats_wrongway(win_returns, window_s, bin_name):
    r = win_returns
    assert (r[f"w{window_s}_{bin_name}_obey"]
            > r[f"w{window_s}_{bin_name}_wrongway"] + 50.0), {
        k: v for k, v in r.items()
        if k.startswith(f"w{window_s}_{bin_name}")}
