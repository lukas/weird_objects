"""HEXAPOD_CONTROL_HZ override for `rl_move.config.load_config`
(2026-08-25, leg-sacrifice-fingerprint DIG-IN).

Built while chasing a 54-test `test_task_semantics.py` full-bank
regression (was 1 known-red as of 2026-08-22) discovered the same
cycle: config.yaml's `control.hz` default flipped 25 -> 100 on 08-24
and the calibrated bank calls `load_config()` with no override anywhere,
so its fixed per-tick reward-magnitude thresholds (measured at 25 Hz)
silently went stale. This mirrors the existing `HEXAPOD_MODEL_SOURCE`
env-var pattern (`servo_model.resolve_model_source` /
`tests/conftest.py`) so a future dig-in can pin/sweep the rate without
editing config.yaml.

NOT wired into `tests/conftest.py`'s defaults this cycle: a quick
sample check found hz=25 does NOT simply restore the bank (one gait
mechanism's return-hit measured WORSE at hz=25 than at the current
hz=100 default) -- the 08-24 rate flip is at most a partial explanation
for the 54 failures, not the root cause, and this cycle does not ship
an unvalidated pin. This test only pins the override MECHANISM itself:
unset = bit-exact untouched behavior; set = the one intended field
changes and nothing else.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parents[1]
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from rl_move.config import load_config  # noqa: E402


def test_unset_env_var_is_bit_exact(monkeypatch):
    monkeypatch.delenv("HEXAPOD_CONTROL_HZ", raising=False)
    a = load_config()
    b = load_config()
    assert a == b
    # Whatever config.yaml currently says (100 as of 2026-08-24) is
    # untouched by this module -- this pins "no accidental override",
    # not a specific numeric value that would break when the file
    # changes again.
    import yaml
    from rl_move.config import DEFAULT_CONFIG
    raw = yaml.safe_load(DEFAULT_CONFIG.read_text())
    assert a["control"]["hz"] == raw["control"]["hz"]


def test_set_env_var_overrides_only_control_hz(monkeypatch):
    monkeypatch.delenv("HEXAPOD_CONTROL_HZ", raising=False)
    baseline = load_config()
    monkeypatch.setenv("HEXAPOD_CONTROL_HZ", "25")
    overridden = load_config()
    assert overridden["control"]["hz"] == 25
    # Nothing else in the tree moves.
    baseline_no_hz = dict(baseline["control"])
    overridden_no_hz = dict(overridden["control"])
    baseline_no_hz.pop("hz")
    overridden_no_hz.pop("hz")
    assert baseline_no_hz == overridden_no_hz
    assert {k: v for k, v in baseline.items() if k != "control"} == \
        {k: v for k, v in overridden.items() if k != "control"}


def test_int_and_float_forms_both_parse(monkeypatch):
    monkeypatch.setenv("HEXAPOD_CONTROL_HZ", "25")
    assert load_config()["control"]["hz"] == 25
    assert isinstance(load_config()["control"]["hz"], int)
    monkeypatch.setenv("HEXAPOD_CONTROL_HZ", "50.0")
    assert load_config()["control"]["hz"] == 50.0
