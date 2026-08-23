"""Unit tests for eval_amp_m5's verdict-file merge fix (08-23).

A `--skip` re-run must not silently overwrite/lose earlier sections'
pass/fail state (and must not let m5_pass go true while an unreflected
FAIL from an earlier full run still stands). Pure-Python, no sim, no
subprocess -- exercises only the merge helper and the m5_pass
aggregation math a real `main()` invocation performs.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
import eval_amp_m5 as m  # noqa: E402


def test_prior_sections_missing_file(tmp_path):
    assert m._prior_sections(tmp_path / "nope.json") == {}


def test_prior_sections_reads_existing(tmp_path):
    vpath = tmp_path / "m5_verdict.json"
    vpath.write_text(json.dumps({"sections": {"walk": {"pass": True}}}))
    assert m._prior_sections(vpath) == {"walk": {"pass": True}}


def test_prior_sections_corrupt_file_is_empty(tmp_path):
    vpath = tmp_path / "m5_verdict.json"
    vpath.write_text("{not json")
    assert m._prior_sections(vpath) == {}


def test_skip_rerun_merges_not_clobbers(tmp_path):
    """The exact bug this cycle hit: a full run records yaw FAIL; a
    later --skip walk,yaw re-run (only push/fault recomputed) must
    still show yaw's FAIL and an overall m5_pass=False, not silently
    drop yaw and report m5_pass=True from push/fault alone."""
    vpath = tmp_path / "m5_verdict.json"
    full = {
        "sections": {
            "walk": {"pass": True},
            "yaw": {"pass": False, "tip_left_err": 0.21, "tip_right_err": 0.23},
            "push": {"pass": False},
            "fault": {"pass": False},
        }
    }
    vpath.write_text(json.dumps(full))

    # Simulate a --skip walk,yaw re-run: start from prior sections,
    # overwrite only push/fault (now passing after some isolation fix).
    sections = m._prior_sections(vpath)
    sections["push"] = {"pass": True}
    sections["fault"] = {"pass": True}

    ran = [k for k in ("walk", "yaw", "push", "fault") if k in sections]
    m5_pass = bool(ran) and all(sections[k].get("pass") for k in ran)

    assert ran == ["walk", "yaw", "push", "fault"]  # yaw/walk survived
    assert sections["yaw"]["pass"] is False  # not lost
    assert m5_pass is False  # yaw's FAIL still vetoes overall pass


def test_full_run_no_prior_file_unaffected(tmp_path):
    """No pre-existing file (the common, no --skip case): behaves
    exactly as before -- sections start empty, ran/m5_pass reflect
    only what this call computed."""
    vpath = tmp_path / "m5_verdict.json"
    sections = m._prior_sections(vpath)
    sections["walk"] = {"pass": True}
    sections["yaw"] = {"pass": True}
    sections["push"] = {"pass": True}
    sections["fault"] = {"pass": True}
    ran = [k for k in ("walk", "yaw", "push", "fault") if k in sections]
    m5_pass = bool(ran) and all(sections[k].get("pass") for k in ran)
    assert ran == ["walk", "yaw", "push", "fault"]
    assert m5_pass is True
