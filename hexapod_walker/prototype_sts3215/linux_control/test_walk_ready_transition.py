from __future__ import annotations

import pytest

from walk_ready_transition import (
    N_JOINTS, TRIPOD_GROUPS, _foot_rz_mm, build_tripod_plant_transition,
)


def _pose(hip: float, knee: float) -> list[float]:
    return [0.0, hip, knee] * 6


def test_tripod_transition_alternates_three_leg_groups():
    start = _pose(20.0, 82.0)
    target = _pose(16.0, 44.0)

    frames = build_tripod_plant_transition(start, target)

    assert len(frames) >= 4
    for i in range(0, len(frames) - 1, 8):
        assert frames[i].phase == "lift"
        assert frames[i].legs == TRIPOD_GROUPS[0]
        assert frames[i + 1].phase == "support"
        assert frames[i + 1].legs == TRIPOD_GROUPS[1]
        assert frames[i + 2].phase == "swing"
        assert frames[i + 2].legs == TRIPOD_GROUPS[0]
        assert frames[i + 3].phase == "place"
        assert frames[i + 3].legs == TRIPOD_GROUPS[0]
        assert frames[i + 4].phase == "lift"
        assert frames[i + 4].legs == TRIPOD_GROUPS[1]
        assert frames[i + 5].phase == "support"
        assert frames[i + 5].legs == TRIPOD_GROUPS[0]
        assert frames[i + 6].phase == "swing"
        assert frames[i + 6].legs == TRIPOD_GROUPS[1]
        assert frames[i + 7].phase == "place"
        assert frames[i + 7].legs == TRIPOD_GROUPS[1]


def test_tripod_transition_finishes_at_exact_target():
    start = _pose(20.0, 82.0)
    target = [
        0.1, 15.9, 43.2,
        0.3, 16.6, 43.9,
        -0.1, 16.3, 43.2,
        0.0, 16.7, 44.3,
        -0.1, 15.6, 43.7,
        0.1, 15.8, 43.4,
    ]

    frames = build_tripod_plant_transition(start, target)

    assert len(frames[-1].q_deg) == N_JOINTS
    assert frames[-1].phase == "settle"
    assert frames[-1].legs == ()
    assert frames[-1].q_deg == pytest.approx(target)


def test_tripod_lift_only_moves_the_active_tripod_to_lift_pose():
    start = _pose(20.0, 82.0)
    target = _pose(16.0, 44.0)

    first = build_tripod_plant_transition(start, target)[0]

    for leg in range(6):
        base = 3 * leg
        if leg in TRIPOD_GROUPS[0]:
            r0, z0 = _foot_rz_mm(start[base + 1], start[base + 2])
            r1, z1 = _foot_rz_mm(first.q_deg[base + 1],
                                  first.q_deg[base + 2])
            assert first.q_deg[base] == pytest.approx(0.0)
            assert r1 == pytest.approx(r0, abs=1.0)
            assert z1 > z0 + 15.0
        else:
            assert first.q_deg[base:base + 3] == pytest.approx(
                start[base:base + 3])
