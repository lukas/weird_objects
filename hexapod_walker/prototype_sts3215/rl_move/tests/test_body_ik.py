"""FK ↔ fixed-foot IK consistency (no hardware)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.body_ik import (  # noqa: E402
    BodyOffset, FixedFootBodyIK, fk_all_feet, foot_world_error,
)


def _nominal_q() -> np.ndarray:
    # Hardware plant defaults: yaw 0, hip +20°, knee +80°.
    q = []
    for _ in range(6):
        q.extend([0.0, math.radians(20.0), math.radians(80.0)])
    return np.array(q, dtype=float)


def test_fk_ik_identity():
    q0 = _nominal_q()
    ik = FixedFootBodyIK()
    ik.reset(q0)
    res = ik.solve(BodyOffset())
    assert res.ok, res.reason
    err = foot_world_error(res.q_rad, ik.feet_world, BodyOffset())
    assert err < 1e-3, f"identity foot err {err*1000:.3f} mm"


def test_small_transforms():
    q0 = _nominal_q()
    ik = FixedFootBodyIK()
    ik.reset(q0)
    cases = [
        BodyOffset(roll=math.radians(1)),
        BodyOffset(roll=math.radians(-1)),
        BodyOffset(pitch=math.radians(1)),
        BodyOffset(pitch=math.radians(-1)),
        BodyOffset(height=0.002),
        BodyOffset(height=-0.002),
        BodyOffset(x=0.002),
        BodyOffset(x=-0.002),
        BodyOffset(y=0.002),
        BodyOffset(y=-0.002),
        BodyOffset(roll=math.radians(1), pitch=math.radians(-1),
                   height=0.002, x=0.001, y=-0.001),
    ]
    for off in cases:
        res = ik.solve(off)
        assert res.ok, f"IK fail {off}: {res.reason}"
        err = foot_world_error(res.q_rad, ik.feet_world, off)
        assert err < 2e-3, f"foot drift {err*1000:.2f} mm for {off}"


def test_unreachable_terminates():
    q0 = _nominal_q()
    ik = FixedFootBodyIK()
    ik.reset(q0)
    # Huge height should fail IK.
    res = ik.solve(BodyOffset(height=0.08))
    assert not res.ok


if __name__ == "__main__":
    test_fk_ik_identity()
    test_small_transforms()
    test_unreachable_terminates()
    print("test_body_ik: OK")
