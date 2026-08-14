#!/usr/bin/env python3
"""Export STLs of every CUSTOM part needed to geometry-test ONE leg of the
rideable_v2 CHAIN VARIANT (CHAIN_VARIANT.md), for test printing (Xometry
MJF/SLS or home FDM) before any 7075 is cut on the Carvera.

These are GEOMETRY/FIT articles, not load parts: printed teeth take tens
of N·m at the joint, never the 0.36 kN·m bench load (CHAIN_VARIANT.md §8).

Sprocket tooth form: the full ANSI / ASME B29.1 profile (seating curve,
working curve, straight flank, topping curve, OD tip flat) implemented and
self-checked in `sprocket_profile.py` — the exact contour the Carvera will
later mill, so the print validates the production geometry file.

Run:  .venv/bin/python tools/export_leg_test_prints.py
Out:  test_prints_leg/*.stl + PRINT_README.md
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon

from sprocket_profile import ANSI40, outline_points, validate

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "test_prints_leg"

# ---- ANSI #40 chain -----------------------------------------------------
PITCH = ANSI40.pitch  # 12.7 mm
ROLLER_D = ANSI40.roller_d  # 7.92 mm (#40 roller)
PRINT_OFFSET = 0.0    # extra profile offset; regenerate with ~0.1 for FDM
PLATE_T = 6.35        # tooth-plate thickness (1/4" 7075 in production)
SPACER_T = 8.03       # duplex row spacing 14.38 - 6.35
DISC_T = 12.0         # lock/hub disc thickness
STANDOFF_T = 5.0      # axial standoff row-2 plate -> disc (duplex side plates
                      # protrude ~2.3 mm beyond the tooth plate face)
DISC_R_LOCK = 105.0   # lock disc radius (>=5 mm rim over the Ø18 ring bores)
DISC_R_PLAIN = 80.0   # yaw hub disc (no lock ring)
HUB_DISC_R = 50.0     # hub disc: M8 head (Ø13 at r40) fully on the face
REG_BOSS_R = 30.95    # register boss on the hub face; stack bore Ø62 slips over
REG_BOSS_T = 5.0

# chain axial overhang: (#40 inner width 7.95 - plate)/2 + link plate ~1.5
CHAIN_SIDEPLATE_OVERHANG = (7.95 - PLATE_T) / 2.0 + 1.5

# ---- stack interface (shared by all three joints) -------------------------
BORE_CENTER = 62.0    # center clearance bore over the register boss
BOLT_R, BOLT_D, BOLT_N = 40.0, 8.5, 6       # M8 hub bolt circle
CUSH_R, CUSH_D, CUSH_N = 60.0, 16.0, 6      # cush-drive bushing holes
LOCK_R, LOCK_D, LOCK_N = 90.0, 18.0, 24     # ring-bushing bores, 15 deg pitch
DOWEL_R, DOWEL_D, DOWEL_N = 50.0, 6.1, 4    # tooth-plate clocking dowels


def spacer_outer_r(n_teeth: int) -> float:
    """Spacer/standoff OD: clear seated rollers AND duplex middle plates
    (plate lower edge ~ pitch_r - 6) by >= 2 mm."""
    rp = PITCH / (2.0 * math.sin(math.pi / n_teeth))
    return rp - 8.0


def ring_holes(n: int, r: float, d: float, phase_deg: float = 0.0):
    out = []
    for i in range(n):
        a = math.radians(phase_deg) + 2 * math.pi * i / n
        c = (r * math.cos(a), r * math.sin(a))
        out.append(circle(c, d / 2.0))
    return out


def circle(center, radius, seg=96):
    a = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    return [(center[0] + radius * np.cos(t), center[1] + radius * np.sin(t))
            for t in a]


def sprocket_outline(n_teeth: int):
    """Full ANSI B29.1 outline (validated in sprocket_profile.py)."""
    validate(n_teeth, ANSI40, PRINT_OFFSET)
    pts, _ = outline_points(n_teeth, ANSI40, PRINT_OFFSET)
    return pts


def extrude(outline, holes, thickness) -> trimesh.Trimesh:
    poly = Polygon(outline, holes)
    if not poly.is_valid:
        poly = poly.buffer(0)
    m = trimesh.creation.extrude_polygon(poly, thickness)
    assert m.is_watertight, "extrusion not watertight"
    return m


def tooth_plate(n_teeth: int) -> trimesh.Trimesh:
    holes = [circle((0, 0), BORE_CENTER / 2.0)]
    holes += ring_holes(BOLT_N, BOLT_R, BOLT_D)
    holes += ring_holes(CUSH_N, CUSH_R, CUSH_D, phase_deg=30.0)
    holes += ring_holes(DOWEL_N, DOWEL_R, DOWEL_D, phase_deg=45.0)
    return extrude(sprocket_outline(n_teeth), holes, PLATE_T)


def spacer_ring(n_teeth: int) -> trimesh.Trimesh:
    holes = [circle((0, 0), BORE_CENTER / 2.0)]
    holes += ring_holes(BOLT_N, BOLT_R, BOLT_D)
    holes += ring_holes(DOWEL_N, DOWEL_R, DOWEL_D, phase_deg=45.0)
    return extrude(circle((0, 0), spacer_outer_r(n_teeth), seg=256),
                   holes, SPACER_T)


def standoff_ring(n_teeth: int) -> trimesh.Trimesh:
    """Between the row-2 tooth plate and the disc: gives the duplex side
    plates axial clearance to the disc face (they protrude ~2.3 mm and the
    disc rim sits radially inside the chain envelope)."""
    holes = [circle((0, 0), BORE_CENTER / 2.0)]
    holes += ring_holes(BOLT_N, BOLT_R, BOLT_D)
    return extrude(circle((0, 0), spacer_outer_r(n_teeth), seg=256),
                   holes, STANDOFF_T)


def lock_hub_disc(with_lock_ring: bool) -> trimesh.Trimesh:
    r = DISC_R_LOCK if with_lock_ring else DISC_R_PLAIN
    holes = [circle((0, 0), BORE_CENTER / 2.0)]
    holes += ring_holes(BOLT_N, BOLT_R, BOLT_D)
    holes += ring_holes(CUSH_N, CUSH_R, CUSH_D, phase_deg=30.0)
    if with_lock_ring:
        holes += ring_holes(LOCK_N, LOCK_R, LOCK_D)
    return extrude(circle((0, 0), r, seg=256), holes, DISC_T)


def union(parts) -> trimesh.Trimesh:
    m = trimesh.boolean.union(parts, engine="manifold")
    assert m.is_watertight
    return m


def difference(a, b) -> trimesh.Trimesh:
    m = trimesh.boolean.difference([a] + b, engine="manifold")
    assert m.is_watertight
    return m


def cyl(r, h, axis="z", at=(0, 0, 0)) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=r, height=h, sections=128)
    if axis == "x":
        c.apply_transform(trimesh.transformations.rotation_matrix(
            math.pi / 2, (0, 1, 0)))
    elif axis == "y":
        c.apply_transform(trimesh.transformations.rotation_matrix(
            math.pi / 2, (1, 0, 0)))
    c.apply_translation(at)
    return c


def box(ext, at=(0, 0, 0)) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=ext)
    b.apply_translation(at)
    return b


def femur_hub() -> trimesh.Trimesh:
    """Hip-end hub: Ø100 disc (joint axis y) + Ø62 register boss for the
    sprocket stack + collar + internal PLUG that slips inside the 60x40x3
    box beam (bolted through, 4x M8) — the old '3 mm shell socket' could
    not mate with the beam at all."""
    body = union([
        cyl(HUB_DISC_R, 85.0, axis="y", at=(0, -2.5, 0)),    # hub disc Ø100
        cyl(REG_BOSS_R, REG_BOSS_T, axis="y",
            at=(0, -45.0 - REG_BOSS_T / 2.0, 0)),            # register boss
        cyl(28.0, 70.0, axis="x", at=(35.0, 0, 0)),          # collar
        box((40.0, 40.0, 60.0), at=(50.0, 0, 0)),            # beam-stop shoulder
        box((120.0, 33.9, 53.9), at=(130.0, 0, 0)),          # plug x 70..190
    ])
    cuts = [cyl(12.5, 130.0, axis="y", at=(0, -5.0, 0))]     # joint shaft bore
    cuts += [cyl(4.25, 60.0, axis="y", at=(x, 0, z))         # 4x M8 cross-bolts
             for x in (105.0, 165.0) for z in (-15.0, 15.0)]
    return difference(body, cuts)


def tibia_hub() -> trimesh.Trimesh:
    """Knee-end hub: same joint interface + register boss; Ø50 tube slips
    into the collar socket, retained by 2x M8 cross-bolts."""
    body = union([
        cyl(HUB_DISC_R, 70.0, axis="y"),
        cyl(REG_BOSS_R, REG_BOSS_T, axis="y",
            at=(0, -35.0 - REG_BOSS_T / 2.0, 0)),            # register boss
        cyl(30.0, 90.0, axis="x", at=(45.0, 0, 0)),
    ])
    cuts = [
        cyl(12.5, 110.0, axis="y", at=(0, -5.0, 0)),
        cyl(25.2, 70.0, axis="x", at=(60.0, 0, 0)),          # Ø50 tube socket
    ]
    cuts += [cyl(4.25, 70.0, axis="z", at=(x, 0, 0))         # 2x M8 cross-bolts
             for x in (55.0, 75.0)]
    return difference(body, cuts)


def coxa_clevis() -> trimesh.Trimesh:
    """The FEA'd single-shear clevis: boss + Ø52 bearing bore + wing plate
    (4x M10 to the coxa crossbar)."""
    body = union([
        box((90.0, 40.0, 90.0), at=(0, 65.0, 0)),
        box((140.0, 25.0, 54.0), at=(110.0, 57.5, 0)),
    ])
    cuts = [cyl(26.0, 50.0, axis="y", at=(0, 65.0, 0))]
    cuts += [cyl(5.25, 40.0, axis="y", at=(x, 57.5, z))      # 4x M10 wing bolts
             for x in (120.0, 160.0) for z in (-16.0, 16.0)]
    return difference(body, cuts)


def motor_mount_plate() -> trimesh.Trimesh:
    """Face plate: AK80-64 flange + pilot-bearing (6905) seat.
    MOTOR BOLT PATTERN IS A PLACEHOLDER — verify against the AK80-64 STEP."""
    plate = box((130.0, 130.0, 12.0))
    cuts = [cyl(19.0, 30.0)]                                 # through bore Ø38
    cuts.append(cyl(21.1, 9.1, at=(0, 0, 6.0 - 4.5)))       # 6905 counterbore
    # (Ø42.2 x 9 deep on the +z face; bearing shoulders on the Ø38 step)
    for i in range(8):                                       # PLACEHOLDER BCD
        a = 2 * math.pi * i / 8
        cuts.append(cyl(2.75, 30.0, at=(45.0 * math.cos(a),
                                        45.0 * math.sin(a), 0)))
    cuts += [cyl(4.25, 30.0, at=(x, y, 0))                   # 4x M8 to the frame
             for x in (-55.0, 55.0) for y in (-55.0, 55.0)]
    return difference(plate, cuts)


def lock_clevis() -> trimesh.Trimesh:
    """SINGLE-CHEEK pin-lock block (not a straddling clevis: the sprocket
    stack + chain occupy the far side of the disc, and the Ø210 rim runs
    15 mm past the pin ring — a two-cheek clevis physically cannot be
    installed). Sits on the HUB side of the disc, ~1 mm off the face; the
    Ø12 pin rides a 30 mm guide bore and engages the disc bushing 10 mm
    BLIND (travel 11 mm < 12 mm solenoid stroke; pin bending at rated hold
    ~212 MPa on hardened Ø12). Bolts to a coxa/femur lock tab (4x M8)."""
    body = box((70.0, 36.0, 56.0))                           # pin axis = y
    cuts = [
        cyl(6.1, 40.0, axis="y"),                            # pin guide bore
        cyl(10.0, 12.0, axis="y", at=(0, -12.5, 0)),        # solenoid nose seat
        cyl(9.0, 16.0, axis="y", at=(0, 10.5, 0)),          # die-spring pocket
    ]
    cuts += [cyl(4.25, 60.0, at=(x, y, 0))                   # 4x M8 mount bolts
             for x in (-25.0, 25.0) for y in (-10.0, 10.0)]
    return difference(body, cuts)


def tensioner_arm() -> trimesh.Trimesh:
    outline = circle((0, 0), 15.0) + []
    # stadium shape: two circles + hull via shapely buffer of a line
    from shapely.geometry import LineString
    stadium = LineString([(0, 0), (100.0, 0)]).buffer(15.0, resolution=48)
    poly = Polygon(stadium.exterior.coords,
                   [circle((0, 0), 5.0), circle((100.0, 0), 5.0)])
    m = trimesh.creation.extrude_polygon(poly, 6.0)
    assert m.is_watertight
    return m


PARTS = {
    # hip stage (52T) — lock ring on the disc
    "hip_tooth_plate_52T": (tooth_plate, (52,), 2),
    "hip_spacer_ring": (spacer_ring, (52,), 1),
    "hip_standoff_ring": (standoff_ring, (52,), 1),
    "hip_lock_hub_disc": (lock_hub_disc, (True,), 1),
    # knee stage (42T)
    "knee_tooth_plate_42T": (tooth_plate, (42,), 2),
    "knee_spacer_ring": (spacer_ring, (42,), 1),
    "knee_standoff_ring": (standoff_ring, (42,), 1),
    "knee_lock_hub_disc": (lock_hub_disc, (True,), 1),
    # yaw stage (36T) — no parking lock at the yaw
    "yaw_tooth_plate_36T": (tooth_plate, (36,), 2),
    "yaw_spacer_ring": (spacer_ring, (36,), 1),
    "yaw_standoff_ring": (standoff_ring, (36,), 1),
    "yaw_hub_disc": (lock_hub_disc, (False,), 1),
    # structural nodes (the FEA'd geometry, print = fit/assembly check)
    "femur_hub": (femur_hub, (), 1),
    "tibia_hub": (tibia_hub, (), 1),
    "coxa_clevis": (coxa_clevis, (), 1),
    # brackets
    "motor_mount_plate": (motor_mount_plate, (), 3),
    "lock_clevis": (lock_clevis, (), 2),
    "tensioner_arm": (tensioner_arm, (), 3),
}


def main() -> None:
    OUT.mkdir(exist_ok=True)
    rows = []
    for name, (fn, args, qty) in PARTS.items():
        mesh = fn(*args)
        path = OUT / f"{name}.stl"
        mesh.export(path)
        dims = mesh.bounds[1] - mesh.bounds[0]
        rows.append((name, qty, dims, mesh.is_watertight))
        print(f"  {name}.stl  x{qty}  "
              f"{dims[0]:.0f} x {dims[1]:.0f} x {dims[2]:.0f} mm  "
              f"watertight={mesh.is_watertight}")
    write_readme(rows)
    print(f"\nwrote {len(rows)} STLs + PRINT_README.md -> {OUT}")


def write_readme(rows) -> None:
    lines = [
        "# rideable_v2 — one-leg test-print kit (chain variant)",
        "",
        "Generated by `tools/export_leg_test_prints.py` — regenerate, don't",
        "hand-edit. GEOMETRY/FIT articles only: printed sprockets take tens",
        "of N·m (moving an unloaded leg), never the bench-rig load",
        "([`CHAIN_VARIANT.md`](../CHAIN_VARIANT.md) §8).",
        "",
        "**Xometry settings:** MJF PA12 or SLS Nylon 12, no finish needed.",
        "Sprocket teeth use the full ANSI B29.1 form (built-in seat",
        "clearance). For home FDM regenerate with PRINT_OFFSET ~0.1 mm.",
        "Print tooth plates flat.",
        "",
        "**What the print validates:** ANSI tooth mesh with real #40-2 chain",
        "(same contour the Carvera later mills), duplex row spacing + dowel",
        "clocking, lock-pin ring + clevis + solenoid throw, hub bolt-up,",
        "chain routing/tensioner/guard clearance on the leg.",
        "",
        "**Known placeholder:** the motor-mount bolt pattern (8x M5 on",
        "Ø90 BCD) — verify against the AK80-64 STEP before trusting it.",
        "",
        "| STL | Qty (one leg) | Envelope (mm) |",
        "|---|---:|---|",
    ]
    for name, qty, dims, _ in rows:
        lines.append(f"| `{name}.stl` | {qty} | "
                     f"{dims[0]:.0f} x {dims[1]:.0f} x {dims[2]:.0f} |")
    lines += [
        "",
        "Buy-not-print for the same test: #40-2 chain loop + master links,",
        "stock 13T/14T/18T drivers, 15T idlers, Ø12 dowel pin, die spring,",
        "solenoid, M8 hardware ([`CHAIN_VARIANT.md`](../CHAIN_VARIANT.md) §7).",
    ]
    (OUT / "PRINT_README.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
