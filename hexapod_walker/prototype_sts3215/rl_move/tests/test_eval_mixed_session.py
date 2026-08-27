"""eval_mixed_session.aggregate_session -- pure aggregation math, no sim.

Locks the long mixed-control session scorecard arithmetic (operator
priority change 2026-08-27: 60/180 s realistic sessions mixing rise/
hold-height/walk/turn/lower with recovery starts). Uses synthetic
report.json-shaped dicts (the schema eval_checkpoint.py writes,
including the additive seq_* session fields) so this never touches
MuJoCo.

Run: uv run python -m pytest rl_move/tests/test_eval_mixed_session.py -q
"""
from __future__ import annotations

from rl_move.sim.eval_mixed_session import aggregate_session


def _ep(*, terminated=False, term_reason="", start_kind="flat",
        planned=5, reached=5, end_seg="lower", slip=2.0,
        dir_err=30.0, gait_valid=True, sacrificed=None, h_err=3.0,
        cur_max=2.2, seq=True):
    ep = {"terminated": terminated, "term_reason": term_reason,
          "start_kind": start_kind, "height_err_end_mm": h_err,
          "cur_max_a": cur_max, "cur_p95_a": 1.5,
          "cur_s_above_soft": 0.5, "slip_m_total": 1.0}
    if seq:
        ep["seq_n_segments_planned"] = planned
        ep["seq_n_segments_reached"] = reached
        ep["seq_end_seg_mode"] = end_seg
        ep["seq_completed"] = (not terminated) and reached == planned
        ep["seq_plan"] = [{"mode": "rise", "t_s": 0.0}]
    if dir_err is not None:
        ep["direction_err_mean_deg"] = dir_err
        ep["vel_err_mean"] = 0.01
        ep["slip_per_m"] = slip
        ep["progress_ratio"] = 0.8
        ep["gait_valid"] = gait_valid
        ep["sacrificed_legs"] = sacrificed or []
    return ep


def _report(eps_by_label, dr_scale=0.0):
    return {"dr_scale": dr_scale, "episodes": eps_by_label}


def test_clean_sessions_pass_hard_and_strict():
    eps = [_ep() for _ in range(6)]
    r = aggregate_session({"dr0": _report({"rise/det": eps,
                                           "rise/sto": eps})},
                          strict=True)
    assert r["zero_falls"] is True
    assert r["gate"]["pass"] is True
    assert r["n_episodes"] == 12
    assert r["session_complete_frac"] == 1.0
    assert r["segments_reached_frac"] == 1.0
    assert r["walk"]["gait_valid_frac"] == 1.0


def test_any_termination_fails_hard_gate_and_is_localized():
    good = [_ep() for _ in range(5)]
    bad = _ep(terminated=True, term_reason="hold_min_load",
              end_seg="hold", start_kind="tipped", reached=3)
    r = aggregate_session({"dr0": _report({"rise/det": good + [bad]})})
    assert r["zero_falls"] is False
    assert r["gate"]["pass"] is False
    assert r["term_reasons"] == {"hold_min_load": 1}
    assert r["terms_by_segment_mode"] == {"hold": 1}
    assert r["terms_by_start_kind"] == {"tipped": 1}
    assert r["session_complete_frac"] < 1.0


def test_strict_soft_axes_fire_independently():
    # zero terms but slipping + sacrificed leg -> hard passes,
    # strict fails on the named axes.
    eps = [_ep(slip=5.0, gait_valid=False, sacrificed=[2])
           for _ in range(6)]
    r = aggregate_session({"dr0": _report({"walk/det": eps})},
                          strict=True)
    assert r["zero_falls"] is True
    assert r["gate"]["soft"]["slip_ok"] is False
    assert r["gate"]["soft"]["gait_valid_ok"] is False
    assert r["gate"]["pass"] is False
    assert r["walk"]["sacrificed_legs_seen"] == [2]
    # non-strict: same numbers, hard gate carries the pass
    r2 = aggregate_session({"dr0": _report({"walk/det": eps})})
    assert r2["gate"]["pass"] is True


def test_none_guards_and_medians():
    # hold-only episodes: no walk metrics, slip_per_m None -> medians
    # None, soft axes read not-ok but nothing crashes.
    eps = [_ep(dir_err=None) for _ in range(4)]
    r = aggregate_session({"dr0": _report({"rise/det": eps})},
                          strict=True)
    assert r["walk"]["slip_per_m_med"] is None
    assert r["walk"]["direction_err_med_deg"] is None
    assert r["gate"]["soft"]["slip_ok"] is False
    assert r["zero_falls"] is True


def test_non_sequence_episodes_still_counted():
    # A legacy (non-seq) report mixes in: counted in n/terms but not
    # in completion math.
    eps = [_ep(seq=False) for _ in range(3)]
    r = aggregate_session({"dr0": _report({"walk/det": eps})})
    assert r["n_episodes"] == 3
    assert r["n_sequence_episodes"] == 0
    assert r["session_complete_frac"] is None


def test_per_pass_breakdown():
    good = [_ep() for _ in range(2)]
    bad = [_ep(terminated=True, term_reason="fall", end_seg="walk",
               reached=2)]
    r = aggregate_session({"dr0": _report({"rise/det": good}),
                           "owndr": _report({"rise/det": bad})})
    assert r["per_pass"]["dr0"] == {"n": 2, "terms": 0}
    assert r["per_pass"]["owndr"] == {"n": 1, "terms": 1}
