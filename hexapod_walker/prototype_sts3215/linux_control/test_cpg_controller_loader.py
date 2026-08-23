"""Tests for cpg_controller_loader.py (cpg/STATUS.md Next item 4).

Pure file-I/O + parsing, no hardware, no sim.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from cpg_controller_loader import list_cpg_controllers, load_cpg_controller

HERE = Path(__file__).resolve().parent
SIM_POLICIES = HERE.parent / "rl_move" / "sim" / "policies"
REAL_ARTIFACT = SIM_POLICIES / "cpg_controller_robust120_yawtrim.json"


@pytest.fixture()
def real_artifact_present():
    if not REAL_ARTIFACT.is_file():
        pytest.skip(f"missing {REAL_ARTIFACT} (not built on this checkout)")
    return REAL_ARTIFACT


def test_list_finds_real_artifact(real_artifact_present):
    entries = list_cpg_controllers(dirs=[SIM_POLICIES])
    names = [e["file"] for e in entries]
    assert "cpg_controller_robust120_yawtrim.json" in names
    entry = next(e for e in entries
                 if e["file"] == "cpg_controller_robust120_yawtrim.json")
    assert "error" not in entry
    assert entry["gait"] == "tetrapod"
    assert entry["gate_pass_dr0"] is True


def test_load_real_artifact_matches_raw_params(real_artifact_present):
    raw = json.loads(real_artifact_present.read_text())
    result = load_cpg_controller("robust120_yawtrim", dirs=[SIM_POLICIES])
    assert result["gait"] == raw["params"]["gait"]
    assert result["gait_kw"]["period"] == pytest.approx(
        raw["params"]["period"])
    assert result["gait_kw"]["swing_frac"] == pytest.approx(
        raw["params"]["swing_frac"])
    assert result["gait_kw"]["lift"] == pytest.approx(
        raw["params"]["lift_m"])
    assert result["gait_kw"]["cmd_tau"] == pytest.approx(
        raw["params"]["cmd_tau"])
    assert result["gait_kw"]["workspace_margin"] == pytest.approx(
        raw["params"]["workspace_margin"])
    assert "gait" not in result["gait_kw"]  # gait is separate, not a kwarg dup
    assert result["plant_stance_deg"] == raw["plant_stance_deg"]


def test_load_by_bare_name_and_by_full_path(real_artifact_present):
    a = load_cpg_controller("cpg_controller_robust120_yawtrim",
                            dirs=[SIM_POLICIES])
    b = load_cpg_controller(str(real_artifact_present))
    assert a["gait_kw"] == b["gait_kw"]


def test_load_gait_kw_constructs_se2_foot_gait(real_artifact_present):
    # This is the point of the loader: the result must be directly
    # usable as SE2FootGait(**gait_kw).
    from se2_foot_gait import SE2FootGait

    result = load_cpg_controller("robust120_yawtrim", dirs=[SIM_POLICIES])
    gait = SE2FootGait(gait=result["gait"], **result["gait_kw"])
    assert gait.gait_name == result["gait"]
    assert gait.period == pytest.approx(result["gait_kw"]["period"])


def test_missing_artifact_raises():
    with pytest.raises(ValueError):
        load_cpg_controller("no_such_controller_xyz", dirs=[SIM_POLICIES])


def test_bad_kind_reported_not_raised(tmp_path):
    bad = tmp_path / "cpg_controller_bad.json"
    bad.write_text(json.dumps({"kind": "something_else"}))
    entries = list_cpg_controllers(dirs=[tmp_path])
    assert entries[0]["error"]


def test_bad_kind_raises_on_load(tmp_path):
    bad = tmp_path / "cpg_controller_bad.json"
    bad.write_text(json.dumps({"kind": "something_else"}))
    with pytest.raises(ValueError):
        load_cpg_controller(str(bad))


def test_missing_params_key_raises(tmp_path):
    bad = tmp_path / "cpg_controller_incomplete.json"
    bad.write_text(json.dumps({
        "kind": "cpg_se2_controller",
        "params": {"gait": "tetrapod", "period": 2.0},
    }))
    with pytest.raises(ValueError):
        load_cpg_controller(str(bad))


def test_unknown_gait_name_raises(tmp_path):
    bad = tmp_path / "cpg_controller_badgait.json"
    bad.write_text(json.dumps({
        "kind": "cpg_se2_controller",
        "params": {"gait": "hexapod-shuffle", "period": 2.0,
                   "swing_frac": 0.3, "lift_m": 0.03, "cmd_tau": 0.1,
                   "workspace_margin": 0.9},
    }))
    with pytest.raises(ValueError):
        load_cpg_controller(str(bad))


def test_list_dedupes_by_filename_first_dir_wins(tmp_path):
    d1 = tmp_path / "d1"
    d2 = tmp_path / "d2"
    d1.mkdir()
    d2.mkdir()
    payload = {
        "kind": "cpg_se2_controller", "name": "d1-copy",
        "params": {"gait": "tetrapod", "period": 2.0, "swing_frac": 0.3,
                   "lift_m": 0.03, "cmd_tau": 0.1, "workspace_margin": 0.9},
    }
    (d1 / "cpg_controller_dup.json").write_text(json.dumps(payload))
    payload2 = dict(payload, name="d2-copy")
    (d2 / "cpg_controller_dup.json").write_text(json.dumps(payload2))
    entries = list_cpg_controllers(dirs=[d1, d2])
    assert len(entries) == 1
    assert entries[0]["name"] == "d1-copy"
