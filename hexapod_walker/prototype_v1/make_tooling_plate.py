#!/usr/bin/env python3
"""Reconfigurable HEXAGONAL tooling / experiment plate for the bench test.

A flat, 3D-printable hex plate that tucks INSIDE the legs: its pointy
corners sit in the gaps between legs and it pulls back from the flats
where the coxa/femur actually swing.

Size is driven by the WALKING leg envelope (full yaw +/-35 deg, hip
-80..+30 deg, knee -20..+80 deg swept over the leg FK).  When a leg
yaws inboard its hip-pitch servo body swings in to r = 68.7 mm at deck
height (true for any 20-26 mm deck), so the hex CORNERS must stay inside
~64 mm:
    * default 110 mm flat-to-flat -> corners at 63.5 mm -> ~5 mm clear
      while walking.
The Mega + PCA9685 control boards do NOT go on a tooling plate: a plate
big enough for the Mega (~140 mm) cannot clear the walking legs at any
height in the sweep.  They live INSIDE the chassis on the built-in
``electronics_tray`` instead.

It mounts AND stacks on ONE pattern -- the chassis (+/-24.75, +/-24.75)
~48 mm square (= ELEC_CHASSIS_MOUNT_HOLES_XY, the heat-set insert square
on chassis_bottom).  Those 4 points become vertical M3 standoff COLUMNS:

    chassis_bottom inserts
        |  4 x M3 male-female standoffs (e.g. 32 mm)
    tooling plate  #1
        |  4 x M3 male-female standoffs
    tooling plate  #2  ...

Every plate uses the identical 4 holes, so the standoffs stack straight
up in clean columns -- that's how they stack well (the electronics board
uses the same square, so it drops into the same columns).

Working area, all cut STRAIGHT THROUGH the plate:
    * a GRID of M3 clearance holes (Phi 3.4 mm) clipped to the hexagon, and
    * full-length open SLOTS (SLOT_W wide) to pass a zip-tie / strap /
      cable to tie stuff down, or an M3 screw + washer to clamp.

(``--captive`` still offers T-slot nut channels on a 6 mm plate; default
is the plain straight-through field on a 4 mm plate.)

Usage:
    python hexapod_walker/prototype_v1/make_tooling_plate.py
    python hexapod_walker/prototype_v1/make_tooling_plate.py --pitch 12.5 --slots 6
    python hexapod_walker/prototype_v1/make_tooling_plate.py --flat-to-flat 130
"""

from __future__ import annotations

import argparse
import math
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as HP  # noqa: E402  (CAD helpers + hex plate + patterns)


# --- Plate (hexagon sized to clear the legs through the GAIT) -------
# Sized against the full WALKING joint envelope (yaw +/-35 deg, hip
# -80..+30 deg, knee -20..+80 deg) swept over the parametric leg FK, not
# just the standing pose.  When a leg yaws inboard, its hip-pitch servo
# body swings in to r = 68.7 mm at the deck height -- and that holds for
# any deck in the 20-26 mm range (the servo body is tall).  So the hex
# CORNERS (circumradius) must stay inside ~64 mm:
#   110 mm flat-to-flat -> corners at 63.5 mm -> ~5 mm clear while walking.
# (Standing-only it could be 115 mm, but we ARE walking.)
# --boards needs the big plate (the Mega's -X bosses sit at x = -59 mm),
# so it auto-uses 140 mm -- but 140 mm will be HIT by a walking leg, so an
# electronics plate must sit ABOVE the leg sweep, not in it.
PLATE_FTF_LOWDECK = 110.0                  # clears legs through the gait
PLATE_FTF_BOARDS  = HP.CHASSIS_TOP_FLAT_TO_FLAT   # 140 mm (Mega needs it)
PLATE_FTF = PLATE_FTF_LOWDECK              # default
PLATE_T_PLAIN = 4.0                        # plain straight-through plate
PLATE_T = 6.0                              # only if --captive T-slots

M3_CLEAR = HP.BRACKET_BOLT_HOLE   # 3.4 mm M3 clearance (holes + slots)

# --- Captive-nut T-slot geometry (only used with --captive) ---------
NUT_CH_W = 6.0
NUT_CH_H = 3.0
FLOOR_H = 1.0
NUT_DROP_OD = 7.0

# --- Mounting field defaults ----------------------------------------
GRID_PITCH = 15.0      # mm -- hole-grid spacing (both axes)
EDGE_MARGIN = 9.0      # mm -- keep holes/slots this far inside the hex edge
N_SLOTS = 4            # number of full-length open slots
SLOT_W = 8.0           # mm -- open-slot width (zip-tie / strap / cable)

# Standoff / stack columns.  Two chassis patterns, BOTH on a 35 mm radius:
#   * ELEC_CHASSIS_MOUNT_HOLES_XY = (+/-24.75, +/-24.75): the heat-set
#     insert SQUARE on chassis_bottom (the tray-mount square).
#   * CHASSIS_STANDOFF_HOLES_XY  = (+/-35, 0), (0, +/-35): the 4 brass
#     M-F standoff COLUMNS that already carry chassis_top.
# Drilling both gives 8 evenly spaced holes on a 35 mm circle (octagon),
# so a 110 mm plate is supported every 45 deg instead of seesawing on a
# 4-point square -- and it can ride the SAME columns that hold chassis_top.
MOUNT_XY_SQUARE = [(float(x), float(y)) for (x, y) in HP.ELEC_CHASSIS_MOUNT_HOLES_XY]
MOUNT_XY_RING = [(float(x), float(y)) for (x, y) in HP.CHASSIS_STANDOFF_HOLES_XY]
# 4 outer holes at R=44 on the hex-corner directions -- they anchor the
# overhanging corners and line up with the same new ring on chassis_top
# and chassis_bottom.
MOUNT_XY_CORNERS = [(float(x), float(y)) for (x, y) in HP.TOOLING_OUTER_MOUNT_HOLES_XY]
MOUNT_XY_OCTAGON = MOUNT_XY_SQUARE + MOUNT_XY_RING + MOUNT_XY_CORNERS
# Keep grid holes / slots this far from a standoff column so M3 heads land
# on solid plastic.
MOUNT_KO = 7.0

# NOTE: the Mega 2560 + PCA9685 boards live INSIDE the chassis on the
# existing ``electronics_tray`` (see HP.make_electronics_tray), NOT on a
# tooling plate -- a board plate big enough for the Mega (140 mm) cannot
# clear the legs while walking.  The old --boards option was dropped.

# --- Optional buck-converter mount (--buck): XINGYHENG 20A / 300W ----
# DC 6-40 V -> 1.2-36 V CC/CV module that steps the 11 V LiPo down to 6 V
# for the servos.  It is big + HOT (tall inductor/heatsink), so it gets
# its OWN plate.  4 x M3 heat-set bosses on a 53 (X) x 39 (Y) rectangle,
# pushed +X off centre so the bosses clear the (+/-24.75) standoff columns.
BUCK_CENTRE = (12.0, 0.0)
BUCK_HOLE_X = 53.0           # mm centre-to-centre, across (+/-X)
BUCK_HOLE_Y = 39.0           # mm centre-to-centre, front-to-back (+/-Y)
BUCK_BODY = (66.0, 52.0)     # mm approx module footprint (grid/slot keep-out)


def _rect_hole_offsets(long_mm: float, short_mm: float):
    hx = short_mm / 2.0
    hy = long_mm / 2.0
    return [(-hx, -hy), (+hx, -hy), (-hx, +hy), (+hx, +hy)]


def _grid_coords(half_extent: float, pitch: float):
    n = int(half_extent // pitch)
    return [k * pitch for k in range(-n, n + 1)]


def _hex_in(x: float, y: float, apothem: float) -> bool:
    """True if (x, y) is inside a flat-to-X hexagon of given apothem."""
    for th in (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0):
        if abs(x * math.cos(th) + y * math.sin(th)) > apothem:
            return False
    return True


def _hex_halfwidth_x(y: float, apothem: float) -> float:
    """Max |x| inside the hexagon at height y (flats perpendicular to X)."""
    s = math.sin(math.pi / 3.0)   # 0.866
    return max(0.0, min(apothem, 2.0 * (apothem - s * abs(y))))


def _stadium(x0, x1, y, width, z_bot, z_top):
    h = z_top - z_bot
    zc = (z_bot + z_top) / 2.0
    body = HP._box((x1 - x0, width, h), center=((x0 + x1) / 2.0, y, zc))
    cap0 = HP._cyl(width / 2.0, h); cap0.apply_translation([x0, y, zc])
    cap1 = HP._cyl(width / 2.0, h); cap1.apply_translation([x1, y, zc])
    return HP._union(body, cap0, cap1)


def _rail_cutters(x0, x1, y, plate_t, captive, slot_w=SLOT_W):
    # _hex_plate is centred on z = 0, so it spans [-plate_t/2, +plate_t/2].
    # Through-cuts must therefore run from below the bottom face to above
    # the top face -- otherwise the cut is a GROOVE, not a hole.
    z_bot = -plate_t / 2.0 - 0.5
    z_top = +plate_t / 2.0 + 0.5
    if not captive:
        return [_stadium(x0, x1, y, slot_w, z_bot, z_top)]
    # Captive-nut T-slot, measured up from the bottom face: a thin floor,
    # then the nut channel, then an M3 clearance slot up through the top.
    bottom = -plate_t / 2.0
    shoulder_z = bottom + FLOOR_H + NUT_CH_H
    cuts = [
        _stadium(x0, x1, y, M3_CLEAR, shoulder_z, z_top),
        _stadium(x0, x1, y, NUT_CH_W, bottom + FLOOR_H, shoulder_z),
    ]
    drop = HP._cyl(NUT_DROP_OD / 2.0, plate_t * 4.0)
    drop.apply_translation([x0, y, 0.0])
    cuts.append(drop)
    return cuts


def make_plate(pitch: float = GRID_PITCH,
               n_slots: int = N_SLOTS,
               margin: float = EDGE_MARGIN,
               flat_to_flat: float = PLATE_FTF,
               captive: bool = False,
               slot_w: float = SLOT_W,
               with_buck: bool = False,
               mount: str = "octagon"):
    """Build and return the hexagonal tooling-plate mesh + layout info.

    ``mount`` selects the standoff/stack hole pattern:
        "square"  -> 4 holes at (+/-24.75, +/-24.75) (R=35 tray square)
        "ring"    -> 4 holes at (+/-35, 0), (0, +/-35) (R=35 chassis_top cols)
        "octagon" -> all 12 (default): the two R=35 squares (every 45 deg)
                     PLUS 4 outer holes at R=44 on the hex-corner directions
                     so the overhanging corners get their own standoff.
    """
    plate_t = PLATE_T if captive else PLATE_T_PLAIN
    apothem = flat_to_flat / 2.0
    circum = apothem / math.cos(math.pi / 6.0)

    mount_xy = {"square": MOUNT_XY_SQUARE,
                "ring": MOUNT_XY_RING,
                "octagon": MOUNT_XY_OCTAGON}[mount]

    # Hex outline + standoff/stack holes (square and/or ring + outer
    # corner ring for the octagon), no leg cuts.
    plate = HP._hex_plate(flat_to_flat, plate_t,
                          with_centre_holes=mount in ("square", "octagon"),
                          with_chassis_standoffs=mount in ("ring", "octagon"),
                          with_outer_mount_ring=(mount == "octagon"),
                          with_leg_features=False)

    inner = apothem - margin
    cutters = []
    bosses = []

    # Optional buck-converter mount bosses (--buck).  Each site is a
    # Phi 8 mm boss with an M3 heat-set pocket; bosses union onto the top
    # and their footprint then suppresses grid/slots so nothing undermines
    # a boss.
    mount_groups = []   # (centre, [hole offsets])
    footprints = []     # (cx, cy, x_full, y_full) keep-out rectangles
    if with_buck:
        # 53 mm spacing along X, 39 mm along Y (long axis swapped vs boards).
        mount_groups.append((BUCK_CENTRE, _rect_hole_offsets(BUCK_HOLE_Y, BUCK_HOLE_X)))
        footprints.append((BUCK_CENTRE[0], BUCK_CENTRE[1], BUCK_BODY[0], BUCK_BODY[1]))

    n_mounts = 0
    for (centre, offsets) in mount_groups:
        for (hx, hy) in HP._absolute_xy(centre, offsets):
            # _hex_plate extrudes symmetrically about z = 0, so its TOP
            # face is at +plate_t/2 (not +plate_t like the old _box board).
            boss, pocket = HP._board_standoff_boss_and_pocket(
                hx, hy,
                pilot_od=HP.INSERT_M3_PILOT_OD, pilot_depth=HP.INSERT_M3_PILOT_DEPTH,
                boss_od=HP.ELEC_BOSS_OD_M3, boss_height=HP.ELEC_STANDOFF_H,
                tray_top_z=plate_t / 2.0)
            bosses.append(boss)
            cutters.append(pocket)
            n_mounts += 1

    # Ventilation slots straight under the buck body (convection cooling for
    # the hot inductor/heatsink), run along X between the boss rows and kept
    # clear of the Phi 8 bosses (y = +/-19.5) and standoff columns.
    n_vents = 0
    if with_buck:
        vx0 = BUCK_CENTRE[0] - 20.0   # -8
        vx1 = BUCK_CENTRE[0] + 21.0   # +33  (between the L/R bosses)
        for vy in (-13.5, -4.5, 4.5, 13.5):
            cutters.append(_stadium(vx0, vx1, vy, 5.0,
                                    -plate_t / 2.0 - 0.5, plate_t / 2.0 + 0.5))
            n_vents += 1

    def _under_board(x, y, pad=1.0):
        return any(abs(x - cx) <= xf / 2.0 + pad and abs(y - cy) <= yf / 2.0 + pad
                   for (cx, cy, xf, yf) in footprints)

    # Grid of M3 clearance holes, clipped to the hexagon, clear of the
    # standoff columns, and (with --boards) outside the board footprints.
    xs = _grid_coords(apothem, pitch)
    ys = _grid_coords(circum, pitch)
    n_holes = 0
    for x in xs:
        for y in ys:
            if not _hex_in(x, y, inner):
                continue
            if any(math.hypot(x - mx, y - my) < MOUNT_KO for (mx, my) in mount_xy):
                continue
            if _under_board(x, y):
                continue
            h = HP._cyl(M3_CLEAR / 2.0, plate_t * 4.0)
            h.apply_translation([x, y, plate_t / 2.0])
            cutters.append(h)
            n_holes += 1

    # Full-length open slots on half-pitch lines, clipped to the hex,
    # clear of the standoff columns (y = +/-24.75), and not running under
    # any board footprint.
    mount_ys = sorted({abs(my) for (_, my) in mount_xy})
    bolt_clear = max(MOUNT_KO, slot_w / 2.0 + 3.5)

    def _slot_ok(m):
        if any(abs(abs(m) - cy) < bolt_clear for cy in mount_ys):
            return False
        if _hex_halfwidth_x(abs(m) + slot_w / 2.0, inner) <= 2.0 * slot_w:
            return False
        for (_, cy, _, yf) in footprints:
            if abs(m - cy) <= yf / 2.0 + slot_w / 2.0 + 1.0:
                return False
        return True

    midlines = [(ys[i] + ys[i + 1]) / 2.0 for i in range(len(ys) - 1)]
    midlines = [m for m in midlines if _slot_ok(m)]
    midlines.sort(key=lambda v: abs(v))
    chosen = sorted(midlines[:max(0, n_slots)])
    slot_info = []
    for y in chosen:
        hw = _hex_halfwidth_x(abs(y) + slot_w / 2.0, inner)
        x1 = hw - slot_w / 2.0
        cutters += _rail_cutters(-x1, x1, y, plate_t, captive, slot_w)
        slot_info.append((round(y, 1), round(2.0 * x1, 0)))

    body = HP._union(plate, *bosses) if bosses else plate
    info = dict(flat_to_flat=flat_to_flat, circum=circum, plate_t=plate_t,
                n_holes=n_holes, n_slots=len(chosen), slot_w=slot_w,
                slots=slot_info, captive=captive, mount_xy=mount_xy,
                mount=mount, n_mounts=n_mounts, with_buck=with_buck,
                n_vents=n_vents)
    return HP._diff(body, *cutters), info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(HP.STL_DIR, "tooling_plate.stl"),
                    help="output STL path")
    ap.add_argument("--pitch", type=float, default=GRID_PITCH,
                    help="grid pitch in mm (both axes)")
    ap.add_argument("--slots", type=int, default=N_SLOTS,
                    help="number of full-length open slots")
    ap.add_argument("--slot-width", type=float, default=SLOT_W,
                    help="open-slot width in mm (zip-tie / strap / cable)")
    ap.add_argument("--margin", type=float, default=EDGE_MARGIN,
                    help="keep the field this far inside the hex edge (mm)")
    ap.add_argument("--flat-to-flat", type=float, default=None,
                    help="hexagon flat-to-flat size (mm); default = 110 "
                         "(clears the legs through the full walking gait)")
    ap.add_argument("--mount", choices=("square", "ring", "octagon"),
                    default="octagon",
                    help="standoff/stack hole pattern: square=4 R=35 (tray "
                         "inserts), ring=4 R=35 (chassis_top columns), "
                         "octagon=12 (default: both R=35 rings + 4 R=44 "
                         "corner anchors)")
    ap.add_argument("--captive", action="store_true",
                    help="captive-nut T-slots (6 mm plate) instead of plain slots")
    ap.add_argument("--buck", action="store_true",
                    help="add a buck-converter mount (53 x 39 mm M3 heat-set "
                         "bosses) -- run alone for a dedicated buck plate")
    args = ap.parse_args(argv)

    ftf = args.flat_to_flat if args.flat_to_flat is not None else PLATE_FTF_LOWDECK

    mesh, info = make_plate(args.pitch, args.slots, args.margin,
                            ftf, args.captive, args.slot_width,
                            with_buck=args.buck, mount=args.mount)

    print("Hexagonal tooling plate layout (mm, origin = plate centre):")
    print(f"  hexagon               {info['flat_to_flat']:.0f} mm flat-to-flat "
          f"(corners reach {info['circum']:.1f} mm) x {info['plate_t']:.1f} thick")
    if info['circum'] <= 64.0:
        print("    (corners <= 64 mm: clears the legs through the full gait)")
    elif info['circum'] > 64.0:
        print(f"    (WARNING: corners at {info['circum']:.1f} mm reach into the "
              f"walking leg sweep -- keep flat-to-flat <= 110 mm)")
    print(f"  M3 grid               {info['n_holes']} holes @ {args.pitch:.1f} mm pitch "
          f"(Phi {M3_CLEAR:.1f} mm, clipped to hex)")
    if info["captive"]:
        print(f"  T-slots               {info['n_slots']} captive-nut rails")
    else:
        print(f"  open slots            {info['n_slots']} x {info['slot_w']:.1f} mm wide, "
              f"(y, length) = {info['slots']}")
        print("    (tie-down: pass a zip-tie / strap / cable, or M3 screw + washer)")
    cm = [(round(x, 2), round(y, 2)) for x, y in info["mount_xy"]]
    print(f"  {len(cm)} standoff columns    Phi {M3_CLEAR:.1f} mm ({info['mount']}) at {cm}")
    if info["mount"] in ("ring", "octagon"):
        print("    (ride the SAME columns that carry chassis_top -- reuse the brass standoffs)")
    print("    (single M-F standoff per column = the deck height; stack only to add a 2nd plate)")
    if info["with_buck"]:
        print(f"  component mounts      {info['n_mounts']} x Phi {HP.ELEC_BOSS_OD_M3:.0f} mm "
              f"bosses ({HP.ELEC_STANDOFF_H:.0f} mm tall) w/ M3 heat-set inserts")
        print(f"    buck conv @ {BUCK_CENTRE}  ({BUCK_HOLE_X:.0f} x {BUCK_HOLE_Y:.0f} mm holes)")
        print(f"  vents                 {info['n_vents']} x 5 mm slots under the buck (airflow)")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    mesh.export(args.out)
    ext = mesh.extents
    print(f"\nWrote {args.out}")
    print(f"  faces={len(mesh.faces)}  watertight={mesh.is_watertight}  "
          f"envelope {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")


if __name__ == "__main__":
    main()
