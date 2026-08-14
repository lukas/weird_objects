#!/usr/bin/env python3
"""Export STEP (+ DXF for the flat plates) of the one-leg CNC parts for the
rideable_v2 chain variant — the parts the Carvera would cut, packaged for a
Xometry CNC order while the machine is in transit.

Same geometry as `export_leg_test_prints.py` (shared constants imported
from it) but built as exact OCC solids: sprocket teeth are per-arc splines
of the validated ANSI B29.1 profile (`sprocket_profile.py`), not faceted
STL — what a machine shop actually needs.

EXCLUDED on purpose: `motor_mount_plate` (its AK80-64 bolt pattern is a
placeholder — print-check it first, never machine it), and the structural
hubs/clevis are exported but flagged HOLD in the notes until the printed
kit assembly check passes.

Run:  .venv/bin/python tools/export_leg_cnc.py
Out:  cnc_leg/*.step, cnc_leg/dxf/*.dxf, cnc_leg/MANUFACTURING_NOTES.md
"""
from __future__ import annotations

import math
from pathlib import Path

import gmsh

from export_leg_test_prints import (BOLT_D, BOLT_N, BOLT_R, BORE_CENTER,
                                    CUSH_D, CUSH_N, CUSH_R, DISC_R_LOCK,
                                    DISC_R_PLAIN, DISC_T, DOWEL_D, DOWEL_N,
                                    DOWEL_R, HUB_DISC_R, LOCK_D, LOCK_N,
                                    LOCK_R, PLATE_T, REG_BOSS_R, REG_BOSS_T,
                                    ROLLER_D, SPACER_T, STANDOFF_T,
                                    spacer_outer_r)
from sprocket_profile import ANSI40, profile_segments, validate

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "cnc_leg"


# --------------------------------------------------------------------------
# OCC helpers
# --------------------------------------------------------------------------
def ring(n, r, phase_deg=0.0):
    for i in range(n):
        a = math.radians(phase_deg) + 2.0 * math.pi * i / n
        yield r * math.cos(a), r * math.sin(a)


def occ_extruded_plate(outline_segs, holes, thickness):
    """outline_segs: list of (tag, pts) closed chain; holes: (cx, cy, r)."""
    occ = gmsh.model.occ
    # shared endpoint points
    pt_cache: dict[tuple, int] = {}

    def pt(p):
        key = (round(p[0], 6), round(p[1], 6))
        if key not in pt_cache:
            pt_cache[key] = occ.addPoint(p[0], p[1], 0.0)
        return pt_cache[key]

    curves = []
    for _, seg in outline_segs:
        tags = [pt(p) for p in seg]
        # collapse consecutive duplicates
        uniq = [tags[0]]
        for t in tags[1:]:
            if t != uniq[-1]:
                uniq.append(t)
        if len(uniq) < 2:
            continue
        if len(uniq) == 2:
            curves.append(occ.addLine(uniq[0], uniq[1]))
        else:
            curves.append(occ.addSpline(uniq))
    outer = occ.addWire(curves)
    loops = [outer]
    for cx, cy, r in holes:
        c = occ.addCircle(cx, cy, 0.0, r)
        loops.append(occ.addWire([c]))
    surf = occ.addPlaneSurface(loops)
    out = occ.extrude([(2, surf)], 0.0, 0.0, thickness)
    return [t for (d, t) in out if d == 3]


def write_part(name, build3d):
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add(name)
    build3d()
    gmsh.model.occ.synchronize()
    path = OUT / f"{name}.step"
    gmsh.write(str(path))
    gmsh.finalize()
    size = path.stat().st_size
    assert size > 1000, f"{name}.step suspiciously small"
    print(f"  {name}.step  ({size // 1024} KB)")


# --------------------------------------------------------------------------
# flat plates
# --------------------------------------------------------------------------
def plate_holes(kind, n_teeth=None):
    holes = [(0.0, 0.0, BORE_CENTER / 2.0)]
    holes += [(x, y, BOLT_D / 2.0) for x, y in ring(BOLT_N, BOLT_R)]
    if kind in ("tooth", "disc", "lockdisc"):
        holes += [(x, y, CUSH_D / 2.0) for x, y in ring(CUSH_N, CUSH_R, 30.0)]
    if kind in ("tooth", "spacer"):
        holes += [(x, y, DOWEL_D / 2.0)
                  for x, y in ring(DOWEL_N, DOWEL_R, 45.0)]
    if kind == "lockdisc":
        holes += [(x, y, LOCK_D / 2.0) for x, y in ring(LOCK_N, LOCK_R)]
    return holes  # "standoff" kind: bore + bolts only


def tooth_plate_build(n_teeth):
    validate(n_teeth, ANSI40)
    segs, _ = profile_segments(n_teeth, ANSI40)

    def build():
        occ_extruded_plate(segs, plate_holes("tooth", n_teeth), PLATE_T)
    return build


def circle_segs(radius, n_arcs=8, samples=24):
    segs = []
    for k in range(n_arcs):
        a0 = 2.0 * math.pi * k / n_arcs
        a1 = 2.0 * math.pi * (k + 1) / n_arcs
        import numpy as np
        pts = [(radius * math.cos(t), radius * math.sin(t))
               for t in np.linspace(a0, a1, samples)]
        segs.append((f"arc_{k}", pts))
    return segs


def spacer_build(n_teeth):
    def build():
        occ_extruded_plate(circle_segs(spacer_outer_r(n_teeth)),
                           plate_holes("spacer"), SPACER_T)
    return build


def standoff_build(n_teeth):
    def build():
        occ_extruded_plate(circle_segs(spacer_outer_r(n_teeth)),
                           plate_holes("standoff"), STANDOFF_T)
    return build


def disc_build(with_lock):
    def build():
        occ_extruded_plate(circle_segs(DISC_R_LOCK if with_lock
                                       else DISC_R_PLAIN),
                           plate_holes("lockdisc" if with_lock else "disc"),
                           DISC_T)
    return build


def tensioner_build():
    def build():
        occ = gmsh.model.occ
        b = occ.addBox(0, -15.0, 0, 100.0, 30.0, 6.0)
        c1 = occ.addCylinder(0, 0, 0, 0, 0, 6.0, 15.0)
        c2 = occ.addCylinder(100.0, 0, 0, 0, 0, 6.0, 15.0)
        body, _ = occ.fuse([(3, b)], [(3, c1), (3, c2)])
        h1 = occ.addCylinder(0, 0, -1, 0, 0, 8.0, 5.0)
        h2 = occ.addCylinder(100.0, 0, -1, 0, 0, 8.0, 5.0)
        occ.cut(body, [(3, h1), (3, h2)])
    return build


# --------------------------------------------------------------------------
# prismatic parts (same dimensions as the print exporter)
# --------------------------------------------------------------------------
def femur_hub_build():
    occ = gmsh.model.occ
    disc = occ.addCylinder(0, -45.0, 0, 0, 85.0, 0, HUB_DISC_R)
    boss = occ.addCylinder(0, -45.0 - REG_BOSS_T, 0, 0, REG_BOSS_T, 0,
                           REG_BOSS_R)
    collar = occ.addCylinder(0, 0, 0, 70.0, 0, 0, 28.0)
    shoulder = occ.addBox(30.0, -20.0, -30.0, 40.0, 40.0, 60.0)
    plug = occ.addBox(70.0, -16.95, -26.95, 120.0, 33.9, 53.9)
    body, _ = occ.fuse([(3, disc)], [(3, boss), (3, collar),
                                     (3, shoulder), (3, plug)])
    cuts = [(3, occ.addCylinder(0, -60.0, 0, 0, 120.0, 0, 12.5))]
    for x in (105.0, 165.0):
        for z in (-15.0, 15.0):
            cuts.append((3, occ.addCylinder(x, -30.0, z, 0, 60.0, 0, 4.25)))
    occ.cut(body, cuts)


def tibia_hub_build():
    occ = gmsh.model.occ
    disc = occ.addCylinder(0, -35.0, 0, 0, 70.0, 0, HUB_DISC_R)
    boss = occ.addCylinder(0, -35.0 - REG_BOSS_T, 0, 0, REG_BOSS_T, 0,
                           REG_BOSS_R)
    collar = occ.addCylinder(0, 0, 0, 90.0, 0, 0, 30.0)
    body, _ = occ.fuse([(3, disc)], [(3, boss), (3, collar)])
    cuts = [(3, occ.addCylinder(0, -55.0, 0, 0, 110.0, 0, 12.5)),
            (3, occ.addCylinder(25.0, 0, 0, 70.0, 0, 0, 25.2))]
    for x in (55.0, 75.0):
        cuts.append((3, occ.addCylinder(x, 0, -35.0, 0, 0, 70.0, 4.25)))
    occ.cut(body, cuts)


def coxa_clevis_build():
    occ = gmsh.model.occ
    boss = occ.addBox(-45.0, 45.0, -45.0, 90.0, 40.0, 90.0)
    wing = occ.addBox(40.0, 45.0, -27.0, 140.0, 25.0, 54.0)
    body, _ = occ.fuse([(3, boss)], [(3, wing)])
    cuts = [(3, occ.addCylinder(0, 40.0, 0, 0, 50.0, 0, 26.0))]
    for x in (120.0, 160.0):
        for z in (-16.0, 16.0):
            cuts.append((3, occ.addCylinder(x, 40.0, z, 0, 40.0, 0, 5.25)))
    occ.cut(body, cuts)


def lock_clevis_build():
    # single-cheek pin-lock block (see export_leg_test_prints.lock_clevis)
    occ = gmsh.model.occ
    body = occ.addBox(-35.0, -18.0, -28.0, 70.0, 36.0, 56.0)
    pin = occ.addCylinder(0, -20.0, 0, 0, 40.0, 0, 6.1)
    nose = occ.addCylinder(0, -18.5, 0, 0, 12.0, 0, 10.0)
    spring = occ.addCylinder(0, 2.5, 0, 0, 16.0, 0, 9.0)
    cuts = [(3, pin), (3, nose), (3, spring)]
    for x in (-25.0, 25.0):
        for y in (-10.0, 10.0):
            cuts.append((3, occ.addCylinder(x, y, -30.0, 0, 0, 60.0, 4.25)))
    occ.cut([(3, body)], cuts)


PARTS = {
    "hip_tooth_plate_52T": (tooth_plate_build(52), 2, "7075-T6", "plate"),
    "hip_spacer_ring": (spacer_build(52), 1, "7075-T6", "plate"),
    "hip_standoff_ring": (standoff_build(52), 1, "7075-T6", "plate"),
    "hip_lock_hub_disc": (disc_build(True), 1, "7075-T6", "plate"),
    "knee_tooth_plate_42T": (tooth_plate_build(42), 2, "7075-T6", "plate"),
    "knee_spacer_ring": (spacer_build(42), 1, "7075-T6", "plate"),
    "knee_standoff_ring": (standoff_build(42), 1, "7075-T6", "plate"),
    "knee_lock_hub_disc": (disc_build(True), 1, "7075-T6", "plate"),
    "yaw_tooth_plate_36T": (tooth_plate_build(36), 2, "7075-T6", "plate"),
    "yaw_spacer_ring": (spacer_build(36), 1, "7075-T6", "plate"),
    "yaw_standoff_ring": (standoff_build(36), 1, "7075-T6", "plate"),
    "yaw_hub_disc": (disc_build(False), 1, "7075-T6", "plate"),
    "tensioner_arm": (tensioner_build(), 3, "6061-T6", "plate"),
    "femur_hub": (femur_hub_build, 1, "6061-T6", "hold"),
    "tibia_hub": (tibia_hub_build, 1, "6061-T6", "hold"),
    "coxa_clevis": (coxa_clevis_build, 1, "6061-T6", "hold"),
    "lock_clevis": (lock_clevis_build, 2, "6061-T6", "hold"),
}


def export_dxf():
    """2D DXF of every flat plate (for the Carvera CAM later / waterjet)."""
    import trimesh
    from shapely.geometry import Polygon

    from sprocket_profile import outline_points

    dxf_dir = OUT / "dxf"
    dxf_dir.mkdir(exist_ok=True)

    def circle_pts(cx, cy, r, n=128):
        import numpy as np
        return [(cx + r * math.cos(t), cy + r * math.sin(t))
                for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]

    flats = {}
    for name, teeth, kind in (("hip_tooth_plate_52T", 52, "tooth"),
                              ("knee_tooth_plate_42T", 42, "tooth"),
                              ("yaw_tooth_plate_36T", 36, "tooth")):
        outline, _ = outline_points(teeth, ANSI40)
        flats[name] = (outline, plate_holes(kind, teeth))
    for name, teeth in (("hip_spacer_ring", 52), ("knee_spacer_ring", 42),
                        ("yaw_spacer_ring", 36)):
        flats[name] = (circle_pts(0, 0, spacer_outer_r(teeth)),
                       plate_holes("spacer"))
    for name, teeth in (("hip_standoff_ring", 52), ("knee_standoff_ring", 42),
                        ("yaw_standoff_ring", 36)):
        flats[name] = (circle_pts(0, 0, spacer_outer_r(teeth)),
                       plate_holes("standoff"))
    flats["hip_lock_hub_disc"] = (circle_pts(0, 0, DISC_R_LOCK),
                                  plate_holes("lockdisc"))
    flats["knee_lock_hub_disc"] = flats["hip_lock_hub_disc"]
    flats["yaw_hub_disc"] = (circle_pts(0, 0, DISC_R_PLAIN),
                             plate_holes("disc"))

    for name, (outline, holes) in flats.items():
        poly = Polygon(outline,
                       [circle_pts(cx, cy, r) for cx, cy, r in holes])
        path = trimesh.load_path(poly)
        path.export(str(dxf_dir / f"{name}.dxf"))
    print(f"  wrote {len(flats)} DXFs -> {dxf_dir}")


def write_notes():
    lines = [
        "# rideable_v2 — one-leg CNC order (chain variant)",
        "",
        "Generated by `tools/export_leg_cnc.py` — regenerate, don't hand-edit.",
        "STEP solids of every custom machined part for ONE leg",
        "([`CHAIN_VARIANT.md`](../CHAIN_VARIANT.md)); intended for a Xometry",
        "CNC order while the Carvera is in transit. DXFs of the flat plates",
        "are in `dxf/` for later Carvera CAM (or a waterjet fallback).",
        "",
        "## Order table",
        "",
        "| STEP | Qty | Material | Status |",
        "|---|---:|---|---|",
    ]
    for name, (_, qty, mat, kind) in PARTS.items():
        status = ("**HOLD** until print-kit assembly check passes"
                  if kind == "hold" else "release now")
        lines.append(f"| `{name}.step` | {qty} | {mat} | {status} |")
    lines += [
        "",
        "`motor_mount_plate` is deliberately NOT here: its AK80-64 bolt",
        "pattern is a placeholder. Verify against the CubeMars STEP, update",
        "`export_leg_test_prints.py`, print-check, then machine.",
        "",
        "## Critical callouts (attach to the Xometry quote)",
        "",
        "- Default tolerance ISO 2768-m is fine everywhere EXCEPT:",
        "- Lock/hub discs: 24x Ø18 lock-ring bores — press fit for hardened",
        "  bushings: Ø18.00 +0.00/-0.03 (S7-class).",
        "- Coxa clevis: Ø52 bearing bore — press seat for 30205 cups:",
        "  Ø52.00 +0.00/-0.03, both cups from the same bore, back-to-back.",
        "- Femur/tibia hubs: Ø25 shaft bore — H7 (+0.021/0), ream or bore.",
        "- Tooth plates: profile is the exact ANSI B29.1 form — machine the",
        "  outline as-modeled, no added offset; deburr both faces.",
        "- Dowel holes Ø6.1: as-machined is fine (slip fit over Ø6 dowels).",
        "- Tooth plates + spacers + discs of one joint are a matched stack:",
        "  ask for them deburred flat (no protruding burrs on mating faces).",
        "",
        "## Material notes",
        "",
        "- 7075-T6 for tooth plates (wear), spacers and discs may be quoted",
        "  6061-T6 to save cost with no structural issue.",
        "- Hubs/clevises 6061-T6 per the FEA basis (`tools/fea_leg_nodes.py`).",
        "",
        "## Sanity numbers (pinned by tools/sprocket_profile.py)",
        "",
        "- 52T: PD 210.3 / OD 217.6; 42T: PD 169.9 / OD 177.1;",
        "  36T: PD 145.7 / OD 152.8; seat R 4.018 (#40, Dr 7.92).",
    ]
    (OUT / "MANUFACTURING_NOTES.md").write_text("\n".join(lines) + "\n")
    print(f"  wrote {OUT / 'MANUFACTURING_NOTES.md'}")


def main():
    OUT.mkdir(exist_ok=True)
    for name, (build, _, _, _) in PARTS.items():
        write_part(name, build)
    export_dxf()
    write_notes()
    print(f"\ndone -> {OUT}")


if __name__ == "__main__":
    main()
