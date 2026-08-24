"""eval_joystick_gate.aggregate_gate -- pure aggregation math, no sim.

Locks the joystick-track DONE-gate arithmetic (CURRENT_TRUTHS.md /
STATUS.md 08-21): zero falls across every pass, slip/m median <=
cap, direction_err median within margin of the teacher floor. Uses
synthetic report.json-shaped dicts (the same schema eval_checkpoint.py
writes) so this never touches MuJoCo.

Run: uv run python -m pytest rl_move/tests/test_eval_joystick_gate.py -q
"""
from __future__ import annotations

from rl_move.sim.eval_joystick_gate import aggregate_gate


def _ep(*, terminated=False, slip=2.0, dir_err=34.0, gait_valid=True,
        duty_cycle=None, swing_count=None, sacrificed_legs=None):
    ep = {"terminated": terminated, "slip_per_m": slip,
          "direction_err_mean_deg": dir_err, "gait_valid": gait_valid}
    if duty_cycle is not None:
        ep["duty_cycle"] = duty_cycle
    if swing_count is not None:
        ep["swing_count"] = swing_count
    if sacrificed_legs is not None:
        ep["sacrificed_legs"] = sacrificed_legs
    return ep


def _report(eps_det, eps_sto, dr_scale=0.0):
    return {"dr_scale": dr_scale,
            "episodes": {"walk/det": eps_det, "walk/sto": eps_sto}}


def test_clean_panel_passes():
    good = [_ep(slip=1.8, dir_err=33.0) for _ in range(12)]
    r = aggregate_gate({"dr0": _report(good, good)})
    assert r["pass"] is True
    assert r["falls_total"] == 0
    assert r["n_total"] == 24


def test_any_fall_anywhere_fails_even_if_medians_are_fine():
    good = [_ep(slip=1.8, dir_err=33.0) for _ in range(11)]
    one_fall = good + [_ep(terminated=True, slip=1.8, dir_err=33.0)]
    r = aggregate_gate({"dr0": _report(one_fall, good)})
    assert r["pass"] is False
    assert r["checks"]["zero_falls"] is False
    assert r["falls_total"] == 1


def test_slip_over_cap_fails_slip_check_only():
    high_slip = [_ep(slip=4.0, dir_err=30.0) for _ in range(12)]
    r = aggregate_gate({"dr0": _report(high_slip, high_slip)},
                        slip_cap=2.9)
    assert r["pass"] is False
    assert r["checks"]["slip_ok"] is False
    assert r["checks"]["zero_falls"] is True
    assert r["slip_per_m_med"] == 4.0


def test_dir_err_within_margin_of_teacher_floor_passes():
    # teacher floor 35deg, margin 5deg -> allow <= 40deg
    eps = [_ep(slip=2.0, dir_err=39.0) for _ in range(12)]
    r = aggregate_gate({"dr0": _report(eps, eps)},
                        teacher_dir_err_deg=35.0, dir_err_margin_deg=5.0)
    assert r["direction_err_allow_deg"] == 40.0
    assert r["checks"]["dir_ok"] is True
    assert r["pass"] is True


def test_dir_err_past_margin_fails():
    eps = [_ep(slip=2.0, dir_err=55.0) for _ in range(12)]
    r = aggregate_gate({"dr0": _report(eps, eps)},
                        teacher_dir_err_deg=35.0, dir_err_margin_deg=5.0)
    assert r["checks"]["dir_ok"] is False
    assert r["pass"] is False


def test_combines_multiple_dr_scale_passes():
    good = [_ep(slip=1.9, dir_err=32.0) for _ in range(12)]
    bad = good[:-1] + [_ep(terminated=True, slip=1.9, dir_err=32.0)]
    r = aggregate_gate({"dr0": _report(good, good),
                        "dr0p35": _report(good, bad)})
    assert r["n_total"] == 48
    assert r["falls_total"] == 1
    assert r["pass"] is False
    assert set(r["per_pass"].keys()) == {"dr0", "dr0p35"}


def test_gait_valid_false_anywhere_fails_gait_check():
    eps = [_ep(slip=1.8, dir_err=30.0) for _ in range(11)]
    eps.append(_ep(slip=1.8, dir_err=30.0, gait_valid=False))
    r = aggregate_gate({"dr0": _report(eps, eps)})
    assert r["checks"]["gait_valid_all"] is False
    assert r["pass"] is False


def test_non_walk_modes_are_ignored():
    # eval_checkpoint reports can carry other modes (hold/rise/...);
    # the joystick gate only ever asked for --modes walk, but a
    # defensive report with extras must not corrupt the aggregate.
    good = [_ep(slip=1.8, dir_err=30.0) for _ in range(12)]
    report = _report(good, good)
    report["episodes"]["hold/det"] = [
        {"terminated": True, "slip_per_m": 999.0,
         "direction_err_mean_deg": 999.0, "gait_valid": False}]
    r = aggregate_gate({"dr0": report})
    assert r["pass"] is True
    assert r["n_total"] == 24


def test_per_leg_metrics_absent_when_no_leg_fields():
    good = [_ep(slip=1.8, dir_err=33.0) for _ in range(12)]
    r = aggregate_gate({"dr0": _report(good, good)})
    assert r["per_leg"] is None


def test_per_leg_metrics_median_and_sacrificed_frac():
    # a frozen-tripod pathology: legs 0,2,4 planted (duty~1, no
    # swings), legs 1,3,5 held airborne (duty~0.02) every episode --
    # this is exactly the AMP M2 pilot pathology (CURRENT_TRUTHS
    # 08-22) the aggregate should surface without a manual video read.
    frozen = [_ep(slip=9.0, dir_err=70.0, gait_valid=False,
                  duty_cycle=[0.98, 0.02, 0.98, 0.02, 0.96, 0.02],
                  swing_count=[4, 2, 5, 1, 10, 2],
                  sacrificed_legs=[1, 3, 5])
              for _ in range(12)]
    r = aggregate_gate({"dr0": _report(frozen, frozen)})
    assert r["per_leg"]["duty_median"] == [0.98, 0.02, 0.98, 0.02,
                                            0.96, 0.02]
    assert r["per_leg"]["sacrificed_episode_count"] == [0, 24, 0, 24,
                                                          0, 24]
    assert r["per_leg"]["sacrificed_frac"] == [0.0, 1.0, 0.0, 1.0,
                                                0.0, 1.0]
    assert r["pass"] is False  # gait_valid_all catches it independently


def test_per_leg_metrics_mixed_sacrifice_pattern():
    clean = _ep(slip=1.8, dir_err=33.0,
                duty_cycle=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
                swing_count=[20, 20, 20, 20, 20, 20],
                sacrificed_legs=[])
    one_bad = _ep(slip=1.8, dir_err=33.0,
                  duty_cycle=[0.5, 0.5, 0.5, 0.5, 0.5, 0.02],
                  swing_count=[20, 20, 20, 20, 20, 1],
                  sacrificed_legs=[5])
    eps = [clean] * 11 + [one_bad]
    r = aggregate_gate({"dr0": _report(eps, eps)})
    assert r["per_leg"]["sacrificed_episode_count"] == [0, 0, 0, 0, 0, 2]
    assert r["per_leg"]["sacrificed_frac"][5] == round(2 / 24, 3)
    assert r["per_leg"]["duty_median"] == [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
