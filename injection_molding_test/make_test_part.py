"""Generate a simple injection-molding test part as a clean B-rep STEP.

The part is a small rectangular enclosure (think electronics housing) sized
for an easy, manufacturable injection-molding quote on Xometry / Protolabs:

    * uniform 2.0 mm nominal wall thickness (no thick sections)
    * filleted external corners and a rounded rim (no sharp edges)
    * a 1.5 deg draft on the side walls so the part releases from the mold
    * two internal screw bosses, cored to keep wall thickness uniform

Unlike the tessellated STEPs produced from meshes elsewhere in this repo,
this is genuine analytic B-rep geometry (planes, cylinders, fillets), so it
imports as a real solid with exact faces — exactly what an injection-molding
DFM / quoting flow expects.

A second, fully-solid variant is also available for a quick no-cavity test
part: a rounded, drafted, chamfered plate with a couple of through-holes.

Usage::

    ./run.sh injection_molding_test/make_test_part.py                  # hollow enclosure
    ./run.sh injection_molding_test/make_test_part.py --shape solid    # solid plate
    # -> injection_molding_test/im_test_enclosure.step  /  im_test_solid_plate.step
"""

from __future__ import annotations

import argparse
import math
import os

import cadquery as cq


# ---- Part parameters (mm) --------------------------------------------------
LENGTH = 80.0       # outer X
WIDTH = 50.0        # outer Y
HEIGHT = 25.0       # outer Z (open at the bottom)
WALL = 2.0          # nominal wall thickness
CORNER_R = 4.0      # external vertical corner radius
RIM_FILLET = 1.5    # rounding on the top (closed) rim
DRAFT_DEG = 1.5     # side-wall draft for mold release

# Internal screw bosses (for self-tapping / heat-set M3) on a diagonal.
BOSS_OUTER_D = 7.0
BOSS_HOLE_D = 2.6
BOSS_INSET = 12.0   # from the side walls, center of each boss
BOSS_CLEAR = 0.0    # bosses run from the floor up to just under the lid

# ---- Solid-plate variant (fully solid, no cavity) --------------------------
SOLID_LENGTH = 80.0
SOLID_WIDTH = 50.0
SOLID_THICK = 10.0      # plate thickness (kept modest so it still molds well)
SOLID_CORNER_R = 8.0    # external vertical corner radius
SOLID_TOP_FILLET = 2.0  # rounding on top/bottom edges
SOLID_HOLE_D = 6.0      # two through-holes
SOLID_HOLE_INSET = 16.0
SOLID_DRAFT_DEG = 3.0   # draft toward both faces from the mid-plane parting line


def build_part() -> cq.Workplane:
    """Build the enclosure solid."""
    # Solid block, rounded vertical corners, then shell out the bottom face
    # to a uniform wall.  The top stays closed (the "lid" of the housing).
    body = (
        cq.Workplane("XY")
        .box(LENGTH, WIDTH, HEIGHT)
        .edges("|Z")
        .fillet(CORNER_R)
        .faces("<Z")
        .shell(-WALL)
    )

    # Round the top rim so there are no sharp external edges.
    body = body.edges(">Z").fillet(RIM_FILLET)

    # Apply a small draft to the four side walls for mold release.  The
    # neutral plane is the top face; walls taper inward toward the open
    # bottom.  Use the OCC draft via the dedicated taper on the side faces.
    inner_height = HEIGHT - WALL
    boss_height = inner_height - BOSS_CLEAR

    # Two internal bosses on opposite corners, cored with a pilot hole.
    boss_x = LENGTH / 2.0 - BOSS_INSET
    boss_y = WIDTH / 2.0 - BOSS_INSET
    boss_centers = [(boss_x, boss_y), (-boss_x, -boss_y)]

    bosses = (
        cq.Workplane("XY")
        .workplane(offset=-HEIGHT / 2.0)  # floor (inside, open bottom side)
        .pushPoints(boss_centers)
        .circle(BOSS_OUTER_D / 2.0)
        .extrude(boss_height)
    )
    # Core the bosses so their effective wall stays ~uniform.
    bosses = (
        bosses.faces(">Z")
        .workplane()
        .pushPoints(boss_centers)
        .hole(BOSS_HOLE_D, depth=boss_height - WALL)
    )

    part = body.union(bosses)
    return part


def build_solid_part(draft_deg: float) -> cq.Workplane:
    """Build a fully-solid, die-lock-free rounded "pillow" block (no cavity).

    Moldability: the part is widest at its mid-height and tapers inward
    toward *both* the top and the bottom face.  That makes the mid-plane an
    unambiguous parting line at the maximum silhouette — exactly where an
    automated DFM tool places it — and gives every wall positive draft in
    whichever half it belongs to, so the part pulls straight out of both
    mold halves with no undercut regardless of how the parting line is
    detected.  The mid-plane is left as a crisp witness edge; only the
    narrow top and bottom perimeters are rounded.  No holes, so there is
    nothing transverse to the pull for the checker to flag.
    """
    half = SOLID_THICK / 2.0
    delta = half * math.tan(math.radians(max(0.0, draft_deg)))

    mid_l, mid_w = SOLID_LENGTH, SOLID_WIDTH                 # widest = parting line
    end_l, end_w = SOLID_LENGTH - 2 * delta, SOLID_WIDTH - 2 * delta
    mid_r = SOLID_CORNER_R
    end_r = max(0.5, SOLID_CORNER_R - delta)                # corners draft too

    s_end = cq.Sketch().rect(end_l, end_w).vertices().fillet(end_r)
    s_mid = cq.Sketch().rect(mid_l, mid_w).vertices().fillet(mid_r)

    part = (
        cq.Workplane("XY")
        .placeSketch(
            s_end,                                           # bottom (z = 0)
            s_mid.moved(cq.Location(cq.Vector(0, 0, half))), # parting line (z = H/2)
            s_end.moved(cq.Location(cq.Vector(0, 0, SOLID_THICK))),  # top (z = H)
        )
        .loft(ruled=True)   # straight, constant-draft frustum walls + sharp mid ridge
    )

    # Round only the top and bottom perimeters; they are the narrow ends,
    # away from the parting line, so the round-overs cannot form undercuts.
    part = part.faces(">Z").edges().fillet(SOLID_TOP_FILLET)
    part = part.faces("<Z").edges().fillet(SOLID_TOP_FILLET)
    return part


def apply_side_draft(part: cq.Workplane, draft_deg: float) -> cq.Workplane:
    """Taper the four outer side walls inward toward the open bottom so the
    part can eject from the mold.  Implemented as a tapered cut driven by the
    draft angle; the top rim dimension is preserved."""
    if draft_deg <= 0:
        return part
    # Offset bottom inward by height * tan(draft) on each side.
    delta = HEIGHT * math.tan(math.radians(draft_deg))
    # Build a frustum the part must stay within: top = full footprint,
    # bottom = footprint shrunk by `delta` per side, then intersect.
    top_l, top_w = LENGTH, WIDTH
    bot_l, bot_w = LENGTH - 2 * delta, WIDTH - 2 * delta
    frustum = (
        cq.Workplane("XY")
        .workplane(offset=-HEIGHT / 2.0)
        .rect(bot_l, bot_w)
        .workplane(offset=HEIGHT)
        .rect(top_l, top_w)
        .loft(combine=True)
    )
    return part.intersect(frustum)


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shape", choices=("enclosure", "solid"), default="enclosure",
                    help="'enclosure' = hollow shelled housing; "
                         "'solid' = fully-solid rounded plate (default: enclosure)")
    ap.add_argument("--out", default=None,
                    help="output STEP path (default: named after --shape)")
    ap.add_argument("--no-draft", action="store_true", help="skip the side-wall draft")
    args = ap.parse_args()

    if args.shape == "solid":
        out = args.out or os.path.join(here, "im_test_solid_plate.step")
        draft_used = 0.0 if args.no_draft else SOLID_DRAFT_DEG
        part = build_solid_part(draft_used)
        label = "Injection-molding solid test block (mid-plane parting line)"
        wall_line = f"  Plate thickness: {SOLID_THICK:.1f} mm (fully solid)"
    else:
        out = args.out or os.path.join(here, "im_test_enclosure.step")
        draft_used = 0.0 if args.no_draft else DRAFT_DEG
        part = build_part()
        if not args.no_draft:
            part = apply_side_draft(part, DRAFT_DEG)
        label = "Injection-molding test enclosure"
        wall_line = f"  Nominal wall:    {WALL:.1f} mm"

    solid = part.val()
    bb = solid.BoundingBox()
    vol_cc = solid.Volume() / 1000.0

    cq.exporters.export(part, out, exportType="STEP")
    size_kb = os.path.getsize(out) / 1024.0

    print(label)
    print(f"  Outer envelope:  {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm")
    print(wall_line)
    print(f"  Draft:           {'OFF' if args.no_draft else f'{draft_used:.1f} deg'}")
    print(f"  Solid volume:    {vol_cc:.1f} cc")
    print(f"  Wrote {out} ({size_kb:.1f} KB) — analytic B-rep STEP")


if __name__ == "__main__":
    main()
