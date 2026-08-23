"""Unit tests for merge_motion_library.py (08-23, amp yaw fork
surgical-splice tool): pure numpy, tiny synthetic libraries, no sim."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
import merge_motion_library as mm  # noqa: E402

TICK_KEYS = mm.TICK_KEYS


def _make_lib(tmp_path, name, clip_names, clip_seeds, lens, tag,
              obs_dim=6, dt=0.04):
    """A tiny synthetic library: every tick's values are `tag`-
    identifiable (base=0.0-offset, override=100.0-offset) so a splice
    can be verified by inspecting the merged VALUES, and every clip
    shares one neutral pose (joint_position_rel_neutral == joint_position,
    i.e. neutral == 0 for every clip -- trivially satisfies the
    per-clip-neutral invariant)."""
    n = sum(lens)
    d = {}
    base_val = 100.0 if tag == "override" else 0.0
    jp = np.zeros((n, 4), dtype=np.float64)
    rel = np.zeros((n, 4), dtype=np.float64)  # neutral == 0 everywhere
    for k, dim in (("joint_velocity", 4), ("base_orientation", 4),
                   ("base_angular_velocity", 3), ("projected_gravity", 3),
                   ("command", 3)):
        d[k] = np.full((n, dim), base_val)
    d["foot_positions"] = np.full((n, 6, 3), base_val)
    d["phase_label"] = np.zeros((n,), dtype=np.int8)
    # obs_style ticks are (tag_value + running index) so a merged file's
    # per-clip tick VALUES can be traced back to their source.
    d["obs_style"] = (base_val + np.arange(n))[:, None] * np.ones(
        (1, obs_dim))
    d["joint_position"] = jp
    d["joint_position_rel_neutral"] = rel
    starts = np.cumsum([0] + lens[:-1])
    d["clip_starts"] = np.asarray(starts, dtype=np.int64)
    d["clip_lens"] = np.asarray(lens, dtype=np.int64)
    d["clip_names"] = np.asarray(clip_names)
    d["clip_seeds"] = np.asarray(clip_seeds, dtype=np.int64)
    d["dt"] = np.asarray(dt)
    p = tmp_path / name
    np.savez(p, **d)
    return p


@pytest.fixture
def libs(tmp_path):
    names = ["forward_0.08", "forward_0.08", "turn_ccw", "turn_ccw",
             "turn_cw"]
    seeds = [0, 1, 0, 1, 0]
    lens = [10, 10, 8, 8, 8]
    base = _make_lib(tmp_path, "base.npz", names, seeds, lens, "base")
    override = _make_lib(tmp_path, "override.npz", names, seeds, lens,
                         "override")
    return base, override


def test_splice_keeps_base_for_untouched_families(libs):
    base, override = libs
    out = mm.merge(base, override, ["turn_ccw", "turn_cw"])
    # clip 0/1 (forward_0.08) untouched -> obs_style values stay < 100
    assert (out["obs_style"][:20] < 100).all()


def test_splice_takes_override_for_named_families(libs):
    base, override = libs
    out = mm.merge(base, override, ["turn_ccw", "turn_cw"])
    # clips 2,3,4 (turn_ccw x2, turn_cw x1) come from override -> >=100
    assert (out["obs_style"][20:] >= 100).all()


def test_splice_preserves_clip_metadata(libs):
    base, override = libs
    out = mm.merge(base, override, ["turn_cw"])
    assert list(out["clip_names"]) == ["forward_0.08", "forward_0.08",
                                       "turn_ccw", "turn_ccw", "turn_cw"]
    assert list(out["clip_lens"]) == [10, 10, 8, 8, 8]
    assert list(out["clip_starts"]) == [0, 10, 20, 28, 36]
    assert out["obs_style"].shape[0] == 44


def test_splice_no_families_is_byte_identical_to_base(libs):
    base, override = libs
    out = mm.merge(base, override, [])
    b = np.load(base, allow_pickle=True)
    assert np.array_equal(out["obs_style"], b["obs_style"])
    assert list(out["clip_names"]) == list(b["clip_names"])


def test_mismatched_clip_names_raises(tmp_path):
    b = _make_lib(tmp_path, "b.npz", ["forward_0.08", "turn_ccw"], [0, 0],
                  [4, 4], "base")
    o = _make_lib(tmp_path, "o.npz", ["forward_0.08", "turn_cw"], [0, 0],
                  [4, 4], "override")
    with pytest.raises(ValueError, match="clip_names"):
        mm.merge(b, o, ["turn_ccw"])


def test_mismatched_seeds_raises(tmp_path):
    b = _make_lib(tmp_path, "b.npz", ["turn_ccw", "turn_ccw"], [0, 1],
                  [4, 4], "base")
    o = _make_lib(tmp_path, "o.npz", ["turn_ccw", "turn_ccw"], [1, 0],
                  [4, 4], "override")
    with pytest.raises(ValueError, match="clip_seeds"):
        mm.merge(b, o, ["turn_ccw"])


def test_unknown_family_raises(libs):
    base, override = libs
    with pytest.raises(ValueError, match="turn_ccw_backwards"):
        mm.merge(base, override, ["turn_ccw_backwards"])


def test_dt_mismatch_raises(tmp_path):
    b = _make_lib(tmp_path, "b.npz", ["turn_ccw"], [0], [4], "base", dt=0.04)
    o = _make_lib(tmp_path, "o.npz", ["turn_ccw"], [0], [4], "override",
                  dt=0.05)
    with pytest.raises(ValueError, match="dt mismatch"):
        mm.merge(b, o, ["turn_ccw"])


def test_obs_style_width_mismatch_raises(tmp_path):
    b = _make_lib(tmp_path, "b.npz", ["turn_ccw"], [0], [4], "base",
                  obs_dim=6)
    o = _make_lib(tmp_path, "o.npz", ["turn_ccw"], [0], [4], "override",
                  obs_dim=8)
    with pytest.raises(ValueError, match="obs_style width"):
        mm.merge(b, o, ["turn_ccw"])


def test_neutral_mismatch_raises(tmp_path):
    """A spliced clip whose neutral pose differs from the base file's
    own must be rejected -- this is the exact hard-fail MotionLibrary
    itself applies at load time (q_20260822T0900Z); catch it at merge
    time instead of at training-launch time."""
    names, seeds, lens = ["forward_0.08", "turn_ccw"], [0, 0], [4, 4]
    b = _make_lib(tmp_path, "b.npz", names, seeds, lens, "base")
    o = _make_lib(tmp_path, "o.npz", names, seeds, lens, "override")
    # Corrupt the override's turn_ccw neutral (nonzero rel-to-jp gap).
    od = dict(np.load(o, allow_pickle=True))
    od["joint_position"] = od["joint_position"].copy()
    od["joint_position"][4:8, 0] += 5.0  # clip 1 (turn_ccw) shifted
    np.savez(o, **od)
    with pytest.raises(ValueError, match="neutral"):
        mm.merge(b, o, ["turn_ccw"])


def test_cli_round_trip(tmp_path, libs):
    """Exercises merge()+savez (main() itself just parses sys.argv and
    calls these two) -- the round trip is what matters for a unit test."""
    base, override = libs
    out_path = tmp_path / "merged.npz"
    result = mm.merge(base, override, ["turn_ccw", "turn_cw"])
    np.savez(out_path, **result)
    reloaded = np.load(out_path, allow_pickle=True)
    assert list(reloaded["clip_names"]) == list(result["clip_names"])
    assert np.array_equal(reloaded["obs_style"], result["obs_style"])
