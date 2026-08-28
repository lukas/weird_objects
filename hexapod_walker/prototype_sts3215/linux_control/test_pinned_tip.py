"""Off-robot tests for pinned_tip (classifier + low-torque untrap).

Run locally:  uv run python linux_control/test_pinned_tip.py
No hardware: reuses test_safe_zero's FakeBus, extended with a fake IMU
whose tilt "levels out" once the knees fold past a threshold —
simulating the body rolling flat off the trapped leg.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import pinned_tip
from feetech_bus import N_JOINTS, joint_to_servo_id
from pinned_tip import (FOLD_KNEE_DEG, TUCK_TORQUE, check_pinned_tip,
                        classify_pinned_tip, run_untrap_tuck)
from test_safe_zero import FakeBus, _pose

# Speed the executor way up for tests (constants are read at call time).
pinned_tip.REST_S = 0.05
pinned_tip.SETTLE_S = 0.05
pinned_tip.TUCK_S = 0.5
pinned_tip.TUCK_TIMEOUT_S = 3.0

PLANT = _pose(hip=18.0, knee=79.0)     # post-fall pose: fell from stance


class TipBus(FakeBus):
    """FakeBus + IMU. ``level_at_knee_deg``: once every non-stalled knee
    folds past it, the body 'rolls flat' and roll drops to 3°."""

    def __init__(self, start_deg, *, roll_deg=25.0,
                 level_at_knee_deg=None, stalled=(), **kw):
        super().__init__(start_deg, stalled=stalled, **kw)
        self.roll_deg = roll_deg
        self.level_at_knee_deg = level_at_knee_deg
        self.torque_limit_writes: list[tuple[int, int]] = []
        real_w2 = self.pkt.write2ByteTxRx

        def _w2(sid, addr, val):
            if addr == 48:  # ADDR_TORQUE_LIMIT
                self.torque_limit_writes.append((sid, int(val)))
            return real_w2(sid, addr, val)

        self.pkt.write2ByteTxRx = _w2

    def read_imu(self, **kw):
        if self.level_at_knee_deg is not None:
            knees = [self.pos[j] for j in range(2, N_JOINTS, 3)
                     if j not in self.stalled]
            if knees and min(knees) >= self.level_at_knee_deg:
                self.roll_deg = 3.0
        r = math.radians(self.roll_deg)
        return {"ax_g": 0.0, "ay_g": math.sin(r), "az_g": math.cos(r)}


# ---------------------------------------------------------------------------
# Classifier (pure)
# ---------------------------------------------------------------------------

def test_level_is_not_pinned():
    v = classify_pinned_tip(PLANT, 2.0, -1.5)
    assert not v["tipped"] and not v["pinned"]


def test_tipped_over_folded_knees_is_pinned():
    v = classify_pinned_tip(PLANT, 25.0, 3.0)
    assert v["tipped"] and v["pinned"]
    assert len(v["candidates"]) == 6           # fell from stance: all folded
    assert "pinned-leg tip" in v["why"]


def test_tipped_with_straight_legs_is_not_pinned():
    # Slope / hand-placed on its side: keep the operator-judgment path.
    v = classify_pinned_tip(_pose(), 25.0, 0.0)
    assert v["tipped"] and not v["pinned"]


def test_single_folded_knee_named():
    q = _pose()
    q[8] = 60.0                                # L2 knee only
    v = classify_pinned_tip(q, -22.0, 5.0)
    assert v["pinned"]
    assert [c["name"] for c in v["candidates"]] == ["L2 knee"]


def test_missing_knees_unclassifiable():
    v = classify_pinned_tip([None] * N_JOINTS, 25.0, 0.0)
    assert v["tipped"] and not v["pinned"]
    assert "missing" in v["why"]


def test_pitch_tips_too():
    v = classify_pinned_tip(PLANT, 1.0, -24.0)
    assert v["tipped"] and v["pinned"]


def test_shallow_propped_rest_is_pinned():
    # Live regression (08-11 21:05): first real pinned test propped at
    # only 13° tilt (L3 knee 109°, L4 knee 126°), and the 08-11 falls
    # settled at 8.7-16.7° — the original 20° threshold missed ALL of
    # them. 13° + folded knees must classify pinned.
    q = _pose()
    q[9:12] = [8.1, 58.1, 108.9]      # L3 as measured on the bench
    q[12:15] = [0.1, 25.8, 126.4]     # L4
    v = classify_pinned_tip(q, -7.4, 13.0)
    assert v["tipped"] and v["pinned"]
    assert {c["name"] for c in v["candidates"]} == {"L3 knee", "L4 knee"}


# ---------------------------------------------------------------------------
# Detector (read-only, FakeBus)
# ---------------------------------------------------------------------------

def test_check_level_fast_path():
    bus = TipBus(PLANT, roll_deg=3.0)
    v = check_pinned_tip(bus)
    assert not v["tipped"] and not v["pinned"]


def test_check_tipped_pinned():
    bus = TipBus(PLANT, roll_deg=25.0)
    v = check_pinned_tip(bus)
    assert v["tipped"] and v["pinned"]
    assert "settling_max_deg" in v
    assert not bus.torque_off, "detector must be read-only"


def test_check_transient_rock_not_pinned():
    bus = TipBus(PLANT, roll_deg=25.0)
    reads = {"n": 0}
    orig = bus.read_imu

    def _imu(**kw):
        reads["n"] += 1
        if reads["n"] > 1:
            bus.roll_deg = 4.0             # settled level by 2nd read
        return orig(**kw)

    bus.read_imu = _imu
    v = check_pinned_tip(bus)
    assert not v["pinned"] and not v["tipped"]


# ---------------------------------------------------------------------------
# Untrap executor (FakeBus)
# ---------------------------------------------------------------------------

def test_untrap_success_levels_and_holds_low_torque():
    bus = TipBus(PLANT, roll_deg=25.0, level_at_knee_deg=120.0)
    res = run_untrap_tuck(bus)
    assert res["ok"], res
    assert res["tilt_deg"] <= pinned_tip.LEVEL_DEG
    assert res["trapped_joints"] == []
    # every knee actually folded
    for j in range(2, N_JOINTS, 3):
        assert abs(bus.pos[j] - FOLD_KNEE_DEG) < 6.0
    # success holds (torque ON) at the LOW limit — never restored high
    assert not bus.torque_off
    lims = [v for _, v in bus.torque_limit_writes]
    assert TUCK_TORQUE in lims and lims[-1] == TUCK_TORQUE
    assert res["torque_limit"] == TUCK_TORQUE


def test_untrap_still_trapped_limps_and_names_joint():
    # L1 knee (joint 5) pinned QUIETLY (current bounded by the 20%
    # torque limit, as designed); body never levels.
    bus = TipBus(PLANT, roll_deg=25.0, stalled={5}, stall_a=1.0)
    res = run_untrap_tuck(bus)
    assert not res["ok"] and res.get("limp"), res
    assert "L1 knee" in res["trapped_names"]
    assert "reposition by hand" in res["error"]
    assert bus.torque_off == {joint_to_servo_id(j)
                              for j in range(N_JOINTS)}, "must limp ALL"


def test_untrap_surprise_force_limps_with_causes():
    # A stalled joint holding MORE current than the 20% torque limit
    # should allow (jam / wrong zero / limit write failed): limp at
    # once with a descriptive error — do not wait out the fold.
    bus = TipBus(PLANT, roll_deg=25.0, stalled={5}, stall_a=2.5)
    res = run_untrap_tuck(bus)
    assert not res["ok"] and res.get("limp"), res
    assert "surprise force" in res["error"]
    assert "L1 knee" in res["error"]
    assert "wrong zero" in res["error"] and "set_zero" in res["error"]
    assert bus.torque_off == {joint_to_servo_id(j)
                              for j in range(N_JOINTS)}, "must limp ALL"


def test_untrap_suspect_zero_refused_without_motion():
    # Encoder far outside its axis range: the logical zero frame is
    # untrustworthy — refuse BEFORE limping or commanding anything.
    # (FakeBus ReadPos encodes through deg_to_count, which clamps to
    # the axis limit — a real present-position read does not — so feed
    # the raw out-of-range count directly for the L2 knee. 210° is
    # 60° past the knee's +150° limit: beyond even the widened 40°
    # untrap slop, i.e. a genuinely wild zero frame, not the
    # weight-shoved-past-soft-limit case the slop exists to allow.)
    from feetech_bus import COUNTS_PER_DEG, JOINT_SIGN, STS_CENTRE_COUNT
    bus = TipBus(PLANT, roll_deg=25.0)
    sid_l2_knee = joint_to_servo_id(8)
    raw = int(round(STS_CENTRE_COUNT + JOINT_SIGN[8] * 210.0
                    * COUNTS_PER_DEG))
    orig_read = bus.pkt.ReadPos

    def _rp(sid):
        if sid == sid_l2_knee:
            return raw, 0, 0
        return orig_read(sid)

    bus.pkt.ReadPos = _rp
    before = list(bus.target)
    res = run_untrap_tuck(bus)
    assert not res["ok"] and res.get("code") == "suspect_zero", res
    assert "set_zero" in res["error"] and "L2 knee" in res["error"]
    assert bus.target == before, "must not command any motion"
    assert not bus.torque_off, "refusal is read-only (no limp needed)"


def test_untrap_accepts_weight_shoved_hip():
    # Live regression (08-11 21:05): body weight shoved the pinned L3
    # hip to +58° — 18° past its +40° soft limit. That is the trapped
    # state itself, NOT a wrong zero; the (wider) untrap slop must let
    # the fold run instead of refusing suspect_zero. FakeBus ReadPos
    # clamps through deg_to_count, so encode this joint's raw count
    # unclamped (as a real servo reports) throughout the run.
    from feetech_bus import COUNTS_PER_DEG, JOINT_SIGN, STS_CENTRE_COUNT
    q = list(PLANT)
    q[10] = 58.1                                # L3 hip
    bus = TipBus(q, roll_deg=13.0, level_at_knee_deg=120.0)
    sid_l3_hip = joint_to_servo_id(10)
    orig_read = bus.pkt.ReadPos

    def _rp(sid):
        if sid == sid_l3_hip:
            bus._advance()
            raw = int(round(STS_CENTRE_COUNT
                            + JOINT_SIGN[10] * bus.pos[10] * COUNTS_PER_DEG))
            return raw, 0, 0
        return orig_read(sid)

    bus.pkt.ReadPos = _rp
    res = run_untrap_tuck(bus)
    assert res.get("code") != "suspect_zero", res
    assert res["ok"], res


def test_untrap_never_levels_no_stall_still_limps():
    bus = TipBus(PLANT, roll_deg=25.0)     # folds fine, stays tipped
    res = run_untrap_tuck(bus)
    assert not res["ok"] and res.get("limp")
    assert "still tipped" in res["error"]


def test_untrap_abort_limps():
    bus = TipBus(PLANT, roll_deg=25.0, level_at_knee_deg=120.0)
    res = run_untrap_tuck(bus, abort_check=lambda: True)
    assert not res["ok"] and res.get("limp")
    assert "abort" in res["error"]
    assert bus.torque_off


if __name__ == "__main__":
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  ok    {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {name}: {e}")
    print(f"{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
