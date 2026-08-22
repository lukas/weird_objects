"""SE2FootGait kinematic guarantees (no hardware, no sim)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from se2_foot_gait import (  # noqa: E402
    SE2FootGait, evaluate_grid, se2_exp, se2_log, support_margin,
    swing_point,
)

PLANT = (20.0, 80.0)   # hardware plant defaults (hip, knee) deg


def _gait(gait: str = "tetrapod", **kw) -> SE2FootGait:
    g = SE2FootGait(gait=gait, **kw)
    g.sync_plant_stance(*PLANT)   # deterministic: never the learned plant
    return g


def _world_foot(g: SE2FootGait, i: int, q18: list[float]
                ) -> tuple[float, float]:
    yaw, hip, knee = (math.radians(q18[3 * i + k]) for k in range(3))
    bx, by, _bz = g.fk_foot_body(i, yaw, hip, knee)
    px, py, pyaw = g.commanded_pose()
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * bx - s * by, py + s * bx + c * by


def test_neutral_pose_matches_plant():
    g = _gait()
    q = g.desired_deg(0.0)
    for i in range(6):
        assert abs(q[3 * i + 0] - 0.0) < 1e-6
        assert abs(q[3 * i + 1] - PLANT[0]) < 1e-6
        assert abs(q[3 * i + 2] - PLANT[1]) < 1e-6


def test_ik_fixed_branch_roundtrip():
    from tripod_gait import COXA, FEMUR, TIBIA

    g = _gait()
    checked = 0
    for yaw_d in (-30.0, -15.0, 0.0, 15.0, 30.0):
        for hip_d in (-60.0, -30.0, 0.0, 20.0):
            for knee_d in (10.0, 60.0, 100.0, 140.0):
                q = (math.radians(yaw_d), math.radians(hip_d),
                     math.radians(knee_d))
                # Fixed-branch domain: foot in FRONT of the yaw axis
                # (positive planar radius); folded-behind configs belong
                # to the other arm branch and must not round-trip.
                r = (COXA + FEMUR * math.cos(q[1])
                     + TIBIA * math.cos(q[1] + q[2]))
                if r < 0.005:
                    continue
                fx, fy, fz = g.fk_foot_body(2, *q)
                sol = g.leg_ik_body(2, fx, fy, fz)
                assert sol is not None, (yaw_d, hip_d, knee_d)
                for a, b in zip(sol, q):
                    assert abs(a - b) < 1e-9, (yaw_d, hip_d, knee_d)
                checked += 1
    assert checked > 50
    # Negative-knee input maps to the SAME foot point but the solver must
    # return the fixed positive-knee branch, never flip.
    fx, fy, fz = g.fk_foot_body(0, 0.0, math.radians(30.0),
                                math.radians(-10.0))
    sol = g.leg_ik_body(0, fx, fy, fz)
    assert sol is not None and sol[2] >= 0.0
    # Unreachable: beyond the leg annulus.
    assert g.leg_ik_body(0, 1.0, 1.0, -0.1) is None


def test_swing_bezier_properties():
    ax, ay, tx, ty = 0.0, 0.0, 0.040, 0.010
    lift = 0.020
    x0, y0, z0 = swing_point(ax, ay, tx, ty, lift, 0.0)
    x1, y1, z1 = swing_point(ax, ay, tx, ty, lift, 1.0)
    assert (abs(x0 - ax) < 1e-12 and abs(y0 - ay) < 1e-12
            and abs(z0) < 1e-12)
    assert (abs(x1 - tx) < 1e-12 and abs(y1 - ty) < 1e-12
            and abs(z1) < 1e-12)
    _xm, _ym, zm = swing_point(ax, ay, tx, ty, lift, 0.5)
    assert abs(zm - lift) < 1e-9
    # Doubled end control points -> zero endpoint velocity (no scuff).
    eps = 1e-3
    xa, ya, za = swing_point(ax, ay, tx, ty, lift, eps)
    xb, yb, zb = swing_point(ax, ay, tx, ty, lift, 1.0 - eps)
    assert math.hypot(xa - ax, ya - ay) < 1e-6 and abs(za) < 1e-6
    assert math.hypot(xb - tx, yb - ty) < 1e-6 and abs(zb) < 1e-6
    # Monotone progress along the chord.
    prev = -1.0
    for j in range(21):
        x, _y, _z = swing_point(ax, ay, tx, ty, lift, j / 20.0)
        assert x >= prev - 1e-12
        prev = x


def test_se2_exp_log_roundtrip():
    for tw in ((0.02, -0.01, 0.0), (0.0, 0.0, 0.3), (0.015, 0.01, -0.2)):
        for t in (0.1, 1.7, 4.0):
            d = se2_exp(*tw, t)
            back = se2_log(*d)
            for a, b in zip(back, (tw[0] * t, tw[1] * t, tw[2] * t)):
                assert abs(a - b) < 1e-9


def test_support_margin_geometry():
    tri = [(1.0, 0.0), (-0.5, 0.866), (-0.5, -0.866)]
    m = support_margin(tri, (0.0, 0.0))
    assert abs(m - 0.5) < 1e-3          # equilateral inradius
    assert support_margin(tri[:2]) == float("-inf")
    assert support_margin(tri, (2.0, 0.0)) < 0.0


def test_scheduler_stance_counts():
    for gait, max_swing, min_stance in (("tetrapod", 2, 4), ("wave", 1, 5)):
        g = _gait(gait, vx=0.005)
        dt = g.period / 400.0
        n_warm = 100                 # quarter cycle
        for k in range(n_warm + 1):
            g.desired_deg(k * dt)
        swung: dict[int, int] = {i: 0 for i in range(6)}
        prev_swing = set(g._swing)   # ignore a swing already in flight
        # Observe exactly one full cycle: every leg enters swing once.
        for k in range(n_warm + 1, n_warm + 401):
            g.desired_deg(k * dt)
            now = set(g._swing)
            assert len(now) <= max_swing
            assert len(g.stance_legs()) >= min_stance
            for i in now - prev_swing:
                swung[i] += 1
            prev_swing = now
        assert all(c == 1 for c in swung.values()), (gait, swung)


def test_stance_feet_world_pinned():
    g = _gait(vx=0.006, vy=0.002, omega=0.02)
    dt = 0.04
    prev_world: dict[int, tuple[float, float]] = {}
    prev_stance: set[int] = set()
    checked = 0
    t = 0.0
    for _ in range(int(2.5 * g.period / dt)):
        q = g.desired_deg(t)
        stance = set(g.stance_legs())
        for i in stance:
            w = _world_foot(g, i, q)
            if i in prev_stance and i in prev_world:
                dx = w[0] - prev_world[i][0]
                dy = w[1] - prev_world[i][1]
                assert math.hypot(dx, dy) < 1e-5, (i, t)
                checked += 1
            prev_world[i] = w
        for i in set(prev_world) - stance:
            prev_world.pop(i)
        prev_stance = stance
        t += dt
    assert checked > 200   # actually exercised the pinning property


def test_command_scaling():
    small = _gait(vx=0.004)
    assert small.last_command_scale == 1.0
    big = _gait(vx=0.040)   # 0.32 m/cycle at period 8 — far beyond reach
    assert 0.05 < big.last_command_scale < 0.7
    # The scaled command must run a clean cycle: no IK misses, no clips.
    dt = 0.04
    t = 0.0
    for _ in range(int(2.5 * big.period / dt)):
        big.desired_deg(t)
        t += dt
    assert big.ik_failures == 0
    assert big.limit_clips == 0
    # Excursions actually stay inside every leg's safe workspace.
    for i in big.stance_legs():
        fx, fy, _fz = big._foot_body(i)
        nx, ny = big.neutral_body[i]
        assert math.hypot(fx - nx, fy - ny) <= big.workspace_r[i] + 1e-6


def test_support_margin_positive_while_walking():
    for gait, floor in (("tetrapod", 0.03), ("wave", 0.05)):
        g = _gait(gait, vx=0.005)
        dt = 0.04
        t = 0.0
        for _ in range(int(2.0 * g.period / dt)):
            g.desired_deg(t)
            t += dt
        assert g.min_support_margin > floor, (gait, g.min_support_margin)


def test_workspace_radii_sane():
    g = _gait()
    for r in g.workspace_r:
        assert 0.020 < r < 0.100   # tens of mm, not degenerate or absurd
    # Neutral is feasible; a point outside the disc is not.
    for i in range(6):
        nx, ny = g.neutral_body[i]
        assert g.foot_feasible(i, nx, ny, g.foot_neutral_z)
        assert not g.foot_feasible(i, nx + g.workspace_r[i] + 0.01, ny,
                                   g.foot_neutral_z)


def test_grid_eval_smoke():
    rows = evaluate_grid(gait="tetrapod", n=3, vx_max=0.02, vy_max=0.02,
                         wz_max=0.10, dt=0.1)
    assert len(rows) == 27
    keys = {"cmd", "vx", "vy", "wz", "scale", "vx_ach", "vy_ach",
            "wz_ach", "min_margin_m", "max_rate_dps", "max_excursion",
            "min_stance_feet", "ik_failures", "limit_clips", "feasible"}
    for r in rows:
        assert keys <= set(r)
        assert 0.0 < r["scale"] <= 1.0
        assert r["ik_failures"] == 0 and r["limit_clips"] == 0
        assert r["min_margin_m"] > 0.0
        # Achieved dead-reckoned twist matches the scaled command.
        assert abs(r["vx_ach"] - r["scale"] * r["vx"]) < 2e-3
        assert abs(r["vy_ach"] - r["scale"] * r["vy"]) < 2e-3
        assert abs(r["wz_ach"] - r["scale"] * r["wz"]) < 2e-2
    stop = next(r for r in rows if r["vx"] == 0.0 and r["vy"] == 0.0
                and r["wz"] == 0.0)
    assert stop["feasible"] and stop["scale"] == 1.0


if __name__ == "__main__":
    test_neutral_pose_matches_plant()
    test_ik_fixed_branch_roundtrip()
    test_swing_bezier_properties()
    test_se2_exp_log_roundtrip()
    test_support_margin_geometry()
    test_scheduler_stance_counts()
    test_stance_feet_world_pinned()
    test_command_scaling()
    test_support_margin_positive_while_walking()
    test_workspace_radii_sane()
    test_grid_eval_smoke()
    print("test_se2_foot_gait: OK")
