#!/usr/bin/env python3
"""Standalone, printable FEETECH STS3215 front-face mount + test cradle.

June 2026 redesign foundation
-----------------------------
The prototype is migrating from the DS3225 PWM servo (mounted by its
protruding side TABS onto a printed tab-shelf + heat-set bosses) to the
FEETECH STS3215 serial-bus servo, which has NO tabs.  The STS3215 bolts
via 4x M2.5 THREADED HOLES IN ITS OWN METAL CASE, arranged on a 9.9 mm
square centred on the output shaft, on the front (output) face.

That is a fundamentally different mounting paradigm than the whole
``hexapod_prototype.py`` well/cradle stack is built around, so before
propagating it into all 18 joint cradles (coxa hip / femur knee /
chassis yaw) this module nails down -- and lets you PRINT and validate
against a real servo -- the single reusable mount feature:

    * a front-face PLATE with an output-hub clearance bore and the
      4x M2.5 case-screw clearance holes on the 9.9 mm square, plus
    * a body-locating shroud (two side walls + back floor) so you can
      confirm the real STS3215 body drops in with the intended
      clearance and the screw pattern lines up with the case.

All STS3215 dimensions below are taken from FEETECH's official STEP
(SO-ARM100 ``STS3215_03a.step``); this file deliberately does NOT import
the (still DS3225-valued) ``SERVO_*`` constants from
``hexapod_prototype`` so it is self-contained and safe to print while the
in-model refit is still in progress.

Frame (matches how the servo is used in a leg):
    +X = body long axis, output shaft offset toward +X
    +Y = body short axis (depth)
    +Z = output-shaft direction (out of the FRONT face)
    origin = centre of the body's BACK (idler) face.

Usage
-----
    python tools/sts3215_testfit.py            # writes stl_prototype/sts3215_testfit.stl
    python tools/sts3215_testfit.py --plate-only   # just the bolt-pattern plate
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Reuse the proven manifold-friendly boolean helpers from the main model.
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import hexapod_prototype as HP  # noqa: E402  (_box/_cyl/_union/_diff)

# ---------------------------------------------------------------------------
# STS3215 geometry (FEETECH official STEP: STS3215_03a.step)
# ---------------------------------------------------------------------------
BODY_W = 45.4    # mm -- long axis (X), output-offset direction
BODY_D = 24.8    # mm -- depth (Y)
BODY_H = 34.3    # mm -- back mount-face -> front(output) mount-face (Z)
OUTPUT_X = 12.5  # mm -- output axis offset from body centre (+X)
OUTPUT_HUB_OD = 20.0   # mm -- output disc-horn OD (sits ABOVE the plate)
OUTPUT_BORE_OD = 10.0  # mm -- output COUPLING bore through the plate (the
                       # dia-20 horn is above the plate; only the ~dia-9
                       # coupling passes through, so the bore clears the
                       # coupling and leaves plate material at r6.9 for the
                       # 4 M2.5 screws)
SPLINE_OD = 5.9        # mm -- 25T output spline OD

# Case-face mount: 4x M2.5 on a 9.9 mm square about the output axis.
MOUNT_SQUARE = 9.9        # mm centre-to-centre (X and Y)
MOUNT_SCREW_CLEAR = 2.7   # mm -- M2.5 clearance hole in the printed plate
MOUNT_HEAD_OD = 4.6       # mm -- M2.5 SHCS head OD (counterbore option)

# ---------------------------------------------------------------------------
# Printed cradle parameters
# ---------------------------------------------------------------------------
PLATE_T = 4.0        # mm -- front mount-plate thickness
PLATE_MARGIN = 4.0   # mm -- plate overhang past the body footprint (X & Y)
BORE_CLEAR = 1.0     # mm -- radial clearance around the output hub bore
WALL_T = 3.0         # mm -- side-shroud wall thickness
FLOOR_T = 3.0        # mm -- back floor thickness
BODY_CL = 0.7        # mm -- per-face clearance around the body in the shroud


def mount_hole_centres():
    """The 4 case-screw centres (X, Y), a 9.9 mm square about the output."""
    h = MOUNT_SQUARE / 2.0
    return [(OUTPUT_X + sx * h, sy * h) for sx in (-1, 1) for sy in (-1, 1)]


def make_mount_plate(*, counterbore: bool = False) -> "HP.trimesh.Trimesh":
    """The front-face mount plate: output bore + 4x M2.5 clearance holes.

    Plate occupies z in [BODY_H, BODY_H + PLATE_T]; the servo's front face
    seats against its underside at z = BODY_H and 4x M2.5 thread up into
    the case.  This is the single reusable feature every joint cradle will
    adopt."""
    plate_w = BODY_W + 2 * PLATE_MARGIN
    plate_d = BODY_D + 2 * PLATE_MARGIN
    plate = HP._box((plate_w, plate_d, PLATE_T),
                    center=(0.0, 0.0, BODY_H + PLATE_T / 2.0))

    cuts = []
    # Output-coupling clearance bore through the plate (clears the ~dia-9
    # coupling; the dia-20 disc horn mounts ABOVE the plate).
    bore = HP._cyl(OUTPUT_BORE_OD / 2.0 + BORE_CLEAR, PLATE_T * 4)
    bore.apply_translation([OUTPUT_X, 0.0, BODY_H + PLATE_T / 2.0])
    cuts.append(bore)

    # 4x M2.5 case-screw clearance holes (+ optional head counterbore).
    for (hx, hy) in mount_hole_centres():
        hole = HP._cyl(MOUNT_SCREW_CLEAR / 2.0, PLATE_T * 4)
        hole.apply_translation([hx, hy, BODY_H + PLATE_T / 2.0])
        cuts.append(hole)
        if counterbore:
            cb = HP._cyl(MOUNT_HEAD_OD / 2.0, PLATE_T)
            cb.apply_translation([hx, hy, BODY_H + PLATE_T])
            cuts.append(cb)

    return HP._diff(plate, *cuts)


def make_testfit_cradle() -> "HP.trimesh.Trimesh":
    """Printable cradle: mount plate + two side shrouds + back floor.

    Open at both X ends so the real servo slides in; bolt up through the
    plate into the case front face to validate the 4x M2.5 pattern, the
    output-hub clearance, and the body fit."""
    plate = make_mount_plate()

    # Two side walls hugging the body +/-Y faces over the full body height.
    wall_inner_y = BODY_D / 2.0 + BODY_CL
    wall_w = BODY_W + 2 * PLATE_MARGIN
    parts = [plate]
    for sy in (-1, 1):
        wall = HP._box((wall_w, WALL_T, BODY_H),
                       center=(0.0,
                                sy * (wall_inner_y + WALL_T / 2.0),
                                BODY_H / 2.0))
        parts.append(wall)

    # Back floor (z in [-FLOOR_T, 0]) tying the two walls together; output
    # idler clearance bored through it so the back hub never bottoms out.
    floor_d = BODY_D + 2 * (BODY_CL + WALL_T)
    floor = HP._box((wall_w, floor_d, FLOOR_T),
                    center=(0.0, 0.0, -FLOOR_T / 2.0))
    idler = HP._cyl(OUTPUT_HUB_OD / 2.0 + BORE_CLEAR, FLOOR_T * 4)
    idler.apply_translation([OUTPUT_X, 0.0, -FLOOR_T / 2.0])
    floor = HP._diff(floor, idler)
    parts.append(floor)

    return HP._union(*parts)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plate-only", action="store_true",
                    help="export just the bolt-pattern plate (fastest print)")
    ap.add_argument("--counterbore", action="store_true",
                    help="counterbore the 4 M2.5 heads flush into the plate")
    ap.add_argument("--out", default=None,
                    help="output STL path (default stl_prototype/sts3215_testfit.stl)")
    args = ap.parse_args(argv)

    mesh = (make_mount_plate(counterbore=args.counterbore)
            if args.plate_only else make_testfit_cradle())

    out = Path(args.out) if args.out else (
        _HERE.parent / "stl_prototype" / "sts3215_testfit.stl")
    out.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(out)

    b = mesh.bounds
    print(f"Wrote {out}")
    print(f"  watertight: {mesh.is_watertight}")
    print(f"  triangles:  {len(mesh.faces):,}")
    print(f"  bbox (mm):  X {b[0][0]:.1f}..{b[1][0]:.1f}  "
          f"Y {b[0][1]:.1f}..{b[1][1]:.1f}  Z {b[0][2]:.1f}..{b[1][2]:.1f}")
    print(f"  4x M2.5 centres (x,y): "
          + ", ".join(f"({hx:.2f},{hy:.2f})" for hx, hy in mount_hole_centres()))


if __name__ == "__main__":
    main()
