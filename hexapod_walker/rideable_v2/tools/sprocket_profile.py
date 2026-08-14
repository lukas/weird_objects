#!/usr/bin/env python3
"""ANSI (ASME B29.1) roller-chain sprocket tooth profile, mm.

Standard tooth form per Machinery's Handbook / ASME B29.1: seating curve
(R) -> working curve (E) -> straight flank -> topping curve (F) -> tip
flat at the OD.  The naive "OD circle minus seat circles" hobby profile is
NOT used: it leaves the gap mouth narrower than the roller (3.8 mm vs the
7.92 mm #40 roller on a 52T) and cannot mesh.

`profile_segments()` returns the closed outline as tagged polyline
segments (one per true arc/line, so CAD exporters can emit exact splines);
`validate()` runs geometric self-checks including a roller-articulation
entry simulation.  Formulas are inch-native per the standard, applied mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

IN = 25.4


@dataclass
class Chain:
    pitch: float          # mm
    roller_d: float       # mm

    @property
    def seat_r(self) -> float:
        # Ds = 1.005 Dr + 0.003 in ; R = Ds/2
        return (1.005 * self.roller_d + 0.003 * IN) / 2.0


ANSI40 = Chain(pitch=12.7, roller_d=7.92)


def _arc(center, radius, a0, a1, n) -> list[tuple[float, float]]:
    return [(center[0] + radius * math.cos(t), center[1] + radius * math.sin(t))
            for t in np.linspace(a0, a1, n)]


def _left_wall(n_teeth: int, chain: Chain, clearance: float):
    """LEFT wall of one tooth gap, seat-local frame (origin = roller seat
    center, +y outward radial, sprocket center at (0, -rp)).  Returns the
    ordered segments climbing from the seat bottom to the left tooth tip:
    seat(bottom->t), working(t->y), flank(y->z), topping(z->tip)."""
    P, Dr, N = chain.pitch, chain.roller_d, n_teeth
    R = chain.seat_r + clearance
    A = math.radians(35.0 + 60.0 / N)
    B = math.radians(18.0 - 56.0 / N)
    E = 1.3025 * Dr + 0.0015 * IN
    F = Dr * (0.8 * math.cos(math.radians(18.0 - 56.0 / N))
              + 1.4 * math.cos(math.radians(17.0 - 64.0 / N))
              - 1.3025) - 0.0015 * IN
    ht = math.pi / N                                     # half-tooth angle
    rp = P / (2.0 * math.sin(ht))
    ro = P * (0.6 + 1.0 / math.tan(ht)) / 2.0            # OD/2 (J = 0.3P)
    center = (0.0, -rp)                                  # sprocket center

    # Working-curve center c: 0.8 Dr from the seat, at angle A outward of
    # the RIGHT chord (chord dips inward by ht).  Its tangency point t with
    # the seat circle lands on the LOWER LEFT of the seat — it forms the
    # left wall (E - 0.8 Dr = R makes them exactly internally tangent).
    c_ang = -ht + A
    c = (0.8 * Dr * math.cos(c_ang), 0.8 * Dr * math.sin(c_ang))
    t_ang = c_ang + math.pi                              # t = seat pt opposite c

    # seat: from the bottom (-90 deg) CLOCKWISE to t (t_ang - 2pi region)
    a1 = t_ang
    while a1 > -math.pi / 2.0:
        a1 -= 2.0 * math.pi
    seat = _arc((0, 0), R, -math.pi / 2.0, a1, 24)

    # working curve: from t sweeping B about c, climbing outward
    w0 = a1 + 2.0 * math.pi if a1 + 2.0 * math.pi < 2 * math.pi else a1
    w0 = t_ang                                           # angle of t about c
    working = _arc(c, E, w0, w0 - B, 16)
    y = working[-1]

    # topping-curve center b: on the LEFT chord, 1.4 Dr from the seat
    b = (-1.4 * Dr * math.cos(ht), -1.4 * Dr * math.sin(ht))

    # straight flank y->z: tangent line from y to circle(b, F); take the
    # tangency point that lies farther from the sprocket center (climbing)
    dyb = (y[0] - b[0], y[1] - b[1])
    d = math.hypot(*dyb)
    base = math.atan2(dyb[1], dyb[0])
    alpha = math.acos(min(1.0, F / d))
    cands = [(b[0] + F * math.cos(base + s * alpha),
              b[1] + F * math.sin(base + s * alpha)) for s in (+1, -1)]
    z = max(cands, key=lambda p: math.hypot(p[0] - center[0],
                                            p[1] - center[1]))
    flank = [y, z]

    # topping arc: around b from z outward until it reaches the OD circle
    z_ang = math.atan2(z[1] - b[1], z[0] - b[0])

    def rad(t):
        p = (b[0] + F * math.cos(t), b[1] + F * math.sin(t))
        return math.hypot(p[0] - center[0], p[1] - center[1])

    # choose sweep direction that moves outward
    step = 1e-3
    direction = 1.0 if rad(z_ang + step) > rad(z_ang) else -1.0
    lo = z_ang
    hi = z_ang
    for _ in range(2000):
        hi += direction * math.radians(0.1)
        if rad(hi) >= ro:
            break
    else:
        raise RuntimeError("topping arc never reaches the OD")
    for _ in range(60):                                  # bisect to the OD
        mid = 0.5 * (lo + hi)
        if rad(mid) < ro:
            lo = mid
        else:
            hi = mid
    topping = _arc(b, F, z_ang, 0.5 * (lo + hi), 16)

    return {"seat": seat, "working": working, "flank": flank,
            "topping": topping, "rp": rp, "ro": ro, "R": R}


def profile_segments(n_teeth: int, chain: Chain = ANSI40,
                     clearance: float = 0.0):
    """Full closed outline, sprocket centered at the origin, traversed
    CLOCKWISE (gap left wall descends, right wall ascends, gaps advance
    clockwise).  Returns (segments, meta); consecutive segments share
    endpoints and the last closes onto the first."""
    w = _left_wall(n_teeth, chain, clearance)
    rp, ro = w["rp"], w["ro"]
    tooth = 2.0 * math.pi / n_teeth

    def mirror(pts):                                     # local x -> -x
        return [(-x, y) for (x, y) in pts]

    def place(pts, k):
        """Local seat frame -> global, seat k at angle (90deg - k*tooth)."""
        base = math.pi / 2.0 - k * tooth
        out = []
        for (x, y) in pts:
            # local: +y outward at the seat; global: seat direction = base
            vx, vy = x, y + rp
            c, s = math.cos(base - math.pi / 2.0), math.sin(base - math.pi / 2.0)
            out.append((c * vx - s * vy, s * vx + c * vy))
        return out

    segs = []
    for k in range(n_teeth):
        # descend the LEFT wall (tip -> seat bottom)
        segs.append((f"top_l_{k}", place(w["topping"][::-1], k)))
        segs.append((f"flank_l_{k}", place(w["flank"][::-1], k)))
        segs.append((f"work_l_{k}", place(w["working"][::-1], k)))
        # seat: t_left -> bottom -> t_right (right = mirror of left)
        seat_full = w["seat"][::-1] + mirror(w["seat"])[1:]
        segs.append((f"seat_{k}", place(seat_full, k)))
        # ascend the RIGHT wall (mirror of the left)
        segs.append((f"work_r_{k}", place(mirror(w["working"]), k)))
        segs.append((f"flank_r_{k}", place(mirror(w["flank"]), k)))
        segs.append((f"top_r_{k}", place(mirror(w["topping"]), k)))
        # tip flat on the OD to the next gap's left-tip (clockwise)
        p0 = place([mirror(w["topping"])[-1]], k)[0]
        p1 = place([w["topping"][-1]], k + 1)[0]
        a0 = math.atan2(p0[1], p0[0])
        a1 = math.atan2(p1[1], p1[0])
        while a1 >= a0:
            a1 -= 2.0 * math.pi
        segs.append((f"tip_{k}", _arc((0, 0), ro, a0, a1, 8)))
    meta = {"rp": rp, "ro": ro, "R": w["R"], "chain": chain}
    return segs, meta


def outline_points(n_teeth: int, chain: Chain = ANSI40,
                   clearance: float = 0.0):
    segs, meta = profile_segments(n_teeth, chain, clearance)
    pts: list[tuple[float, float]] = []
    for _, s in segs:
        for p in s:
            if not pts or math.hypot(p[0] - pts[-1][0],
                                     p[1] - pts[-1][1]) > 1e-6:
                pts.append(p)
    if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6:
        pts.pop()
    return pts, meta


def validate(n_teeth: int, chain: Chain = ANSI40,
             clearance: float = 0.0) -> dict:
    """Geometric self-checks; raises AssertionError on failure."""
    from shapely.geometry import Point, Polygon

    pts, meta = outline_points(n_teeth, chain, clearance)
    poly = Polygon(pts)
    assert poly.is_valid, "profile polygon self-intersects"
    rp, R, Dr, P = meta["rp"], meta["R"], chain.roller_d, chain.pitch
    tooth = 2.0 * math.pi / n_teeth

    # 1. roller seats at the pitch point, at seat-curve distance
    c0 = Point(0.0, rp)                                  # seat 0 is at +y
    assert not c0.within(poly), "roller center inside metal"
    d = poly.exterior.distance(c0)
    assert abs(d - R) < 0.02, f"seat distance {d:.3f} != R {R:.3f}"

    # 2. roller articulation entry: arriving roller swings about the seated
    # one on a radius-P arc from the chain tangent down into its own seat.
    seated = (0.0, rp)
    target_ang = math.pi / 2.0 - tooth                   # next seat, clockwise
    target = (rp * math.cos(target_ang), rp * math.sin(target_ang))
    a_seat = math.atan2(target[1] - seated[1], target[0] - seated[0])
    min_clr = 1e9
    for f in np.linspace(0.0, 1.0, 80):
        a = a_seat + (1.0 - f) * math.radians(75)        # approach sweep
        c = Point(seated[0] + P * math.cos(a), seated[1] + P * math.sin(a))
        assert not c.within(poly), "arriving roller center passes through metal"
        clr = poly.exterior.distance(c)
        if f < 0.97:
            min_clr = min(min_clr, clr)
            assert clr >= Dr / 2.0 - 0.15, \
                f"roller fouls tooth during entry: {clr:.2f} mm at f={f:.2f}"

    # 3. gap mouth at the OD passes the roller
    ro = meta["ro"]
    tips = [p for p in pts if math.hypot(*p) > ro - 1e-3]
    gap_tips = [p for p in tips
                if abs(math.atan2(p[0], p[1])) < tooth * 0.75]
    left = [p for p in gap_tips if p[0] < 0]
    right = [p for p in gap_tips if p[0] > 0]
    mouth = min(math.hypot(l[0] - r[0], l[1] - r[1])
                for l in left for r in right)
    assert mouth > Dr * 0.95, f"gap mouth {mouth:.2f} vs roller {Dr:.2f}"

    return {"n_teeth": n_teeth, "rp": rp, "ro": ro, "seat_R": R,
            "mouth_mm": mouth, "entry_min_clearance_mm": min_clr}


if __name__ == "__main__":
    for n in (36, 42, 52):
        info = validate(n)
        print(f"{n}T: PD {2*info['rp']:.1f}  OD {2*info['ro']:.1f}  "
              f"seat R {info['seat_R']:.3f}  mouth {info['mouth_mm']:.1f}  "
              f"entry clr {info['entry_min_clearance_mm']:.2f}  ok")
