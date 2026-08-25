#!/usr/bin/env python3
"""Export STEP-first parts for the rigid-hip variant (concepts/rigid_hip).

Additive sidecar for ``concepts/rigid_hip/make_rigid_hip_variant.py``: it
imports that module's constants (which in turn come from hexapod_prototype)
so no dimension forks, rebuilds the seven variant printables as OpenCascade
BREP solids, and derives STL from the BREP.  Production parts the variant
edits (coxa link, chassis bottom, servo clamp cap) start from the base
sidecar's BREP builders and receive the same boolean edits the mesh
generator applies.
"""

from __future__ import annotations

import argparse
import json
import math
import sys

from build123d import (
    BuildPart,
    BuildSketch,
    Ellipse,
    Plane,
    Pos,
    Rotation,
    extrude,
)

from step_common import (
    OUT_DIR,
    PROTO_DIR,
    THIS_DIR,
    StepPart,
    export_all,
    write_bundle,
)

sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(PROTO_DIR))
sys.path.insert(0, str(PROTO_DIR / "concepts" / "rigid_hip"))

import build_step_first_test as step  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402


# --- span-style primitive adapters -----------------------------------------
# The variant's mesh builders specify cylinders by (r, z0, z1) spans; these
# wrap the base sidecar's (radius, length, center) helpers with the same
# signatures so each BREP builder below stays line-for-line comparable with
# its mesh twin in make_rigid_hip_variant.py.

_box = step._box  # same (extents, center) signature as the variant helper


def _cyl_z(r: float, z0: float, z1: float,
           x: float = 0.0, y: float = 0.0) -> object:
    return step._cyl_z(r, z1 - z0, (x, y, (z0 + z1) / 2.0))


def _cyl_y(r: float, y0: float, y1: float,
           x: float = 0.0, z: float = 0.0) -> object:
    return step._cyl_y(r, y1 - y0, (x, (y0 + y1) / 2.0, z))


def _hex_prism(apothem: float, z0: float, z1: float,
               flats_at_rings: bool = False) -> object:
    """Matches the variant's _hex_prism: default puts hex VERTICES at the
    ring azimuths (30/90/... deg); flats_at_rings rotates 30 deg."""
    circ = apothem / math.cos(math.pi / 6.0)
    a0 = 0.0 if flats_at_rings else math.pi / 6.0
    pts = [
        (circ * math.cos(a0 + i * math.pi / 3.0),
         circ * math.sin(a0 + i * math.pi / 3.0))
        for i in range(6)
    ]
    return Pos(0.0, 0.0, (z0 + z1) / 2.0) * step._xy_polygon_prism(pts, z1 - z0)


# --- variant printables -----------------------------------------------------

def make_hip_clamp_cap_rigid() -> object:
    """Stock clamp cap + yaw-axis pedestal + inner-race press boss + two
    puller notches (rv.make_hip_cap_rigid)."""
    cap = step.make_servo_clamp_cap()
    ped = _cyl_y(rv.PED_OD / 2.0, rv.PED_Y0, rv.PED_Y1, z=rv.AXIS_Z)
    boss = _cyl_y(rv.BOSS_OD / 2.0, rv.PED_Y1 - 1.0, rv.BOSS_Y1, z=rv.AXIS_Z)
    tip = _cyl_y(rv.BOSS_OD / 2.0 - rv.BOSS_TIP_STEP, rv.BOSS_Y1 - 0.1,
                 rv.TIP_Y1, z=rv.AXIS_Z)
    body = step._union(cap, ped, boss, tip)
    notches = []
    for sx in (+1.0, -1.0):
        ext_x = rv.PED_OD / 2.0 - rv.PULLER_NOTCH_R0 + 3.0
        notches.append(_box(
            (ext_x, rv.PULLER_NOTCH_DEPTH + 0.05, rv.PULLER_NOTCH_W),
            (sx * (rv.PULLER_NOTCH_R0 + ext_x / 2.0),
             rv.PED_Y1 - rv.PULLER_NOTCH_DEPTH / 2.0 + 0.025,
             rv.AXIS_Z),
        ))
    return step._diff(body, *notches)


def make_chassis_top_rigid() -> object:
    """Hex frame sheet + six bearing-pocket bosses + hatch opening
    (rv.make_chassis_top_rigid)."""
    solids = [_hex_prism(rv.APOTHEM, rv.SHEET_Z0, rv.SHEET_Z1)]
    cuts = []
    for _i, edge, _R, _R3 in hp._leg_chassis_frames():
        x, y = float(edge[0]), float(edge[1])
        solids.append(_cyl_z(rv.RING_OD / 2.0, rv.RING_BOT_W, rv.SHEET_Z1,
                             x, y))
        cuts.append(_cyl_z(rv.POCKET_BORE / 2.0, rv.RING_BOT_W - 1.0,
                           rv.SHEET_Z0, x, y))
        cuts.append(_cyl_z(rv.POCKET_BORE / 2.0 + rv.POCKET_LEADIN,
                           rv.RING_BOT_W - 1.0,
                           rv.RING_BOT_W + rv.POCKET_LEADIN, x, y))
        cuts.append(_cyl_z(rv.SHOULDER_OD / 2.0, rv.SHEET_Z0 - 0.01,
                           rv.SHEET_Z1 + 1.0, x, y))
    for x, y in rv._access_hole_xy():
        cuts.append(_cyl_z(rv.ACCESS_HOLE_D / 2.0, rv.SHEET_Z0 - 1.0,
                           rv.SHEET_Z1 + 1.0, x, y))
    cuts.append(_hex_prism(rv.HATCH_OPEN_APO, rv.SHEET_Z0 - 1.0,
                           rv.SHEET_Z1 + 1.0, flats_at_rings=True))
    for x, y in rv._hatch_screw_xy():
        cuts.append(_cyl_z(rv.HOLE_D / 2.0, rv.SHEET_Z0 - 1.0,
                           rv.SHEET_Z1 + 1.0, x, y))
    for az in range(0, 360, 60):
        cuts.append(_cyl_z(rv.HOLE_D / 2.0, rv.SHEET_Z0 - 1.0,
                           rv.SHEET_Z1 + 1.0,
                           rv.PILLAR_FRAME_SCREW_RHO * math.cos(math.radians(az)),
                           rv.PILLAR_FRAME_SCREW_RHO * math.sin(math.radians(az))))
    return step._diff(step._union(*solids), *cuts)


def make_top_hatch_rigid() -> object:
    """Removable hex service lid with registration lip and screw ears
    (rv.make_top_hatch_rigid)."""
    lid = _hex_prism(rv.HATCH_APO, rv.SHEET_Z1, rv.SHEET_Z1 + rv.PLATE_T,
                     flats_at_rings=True)
    lip = step._diff(
        _hex_prism(rv.HATCH_OPEN_APO - rv.HATCH_LIP_CL,
                   rv.SHEET_Z1 - rv.HATCH_LIP_H, rv.SHEET_Z1 + 0.1,
                   flats_at_rings=True),
        _hex_prism(rv.HATCH_OPEN_APO - rv.HATCH_LIP_CL - rv.HATCH_LIP_W,
                   rv.SHEET_Z1 - rv.HATCH_LIP_H - 1.0, rv.SHEET_Z1 + 1.0,
                   flats_at_rings=True),
    )
    ears = [_cyl_z(rv.HATCH_EAR_OD / 2.0, rv.SHEET_Z1,
                   rv.SHEET_Z1 + rv.PLATE_T, x, y)
            for (x, y) in rv._hatch_screw_xy()]
    body = step._union(lid, lip, *ears)
    cuts = []
    for (x, y) in list(rv._hatch_screw_xy()) \
            + [(float(x), float(y)) for (x, y) in hp.CHASSIS_STANDOFF_HOLES_XY] \
            + [(float(x), float(y)) for (x, y) in hp.ELEC_CHASSIS_MOUNT_HOLES_XY]:
        cuts.append(_cyl_z(rv.HOLE_D / 2.0, rv.SHEET_Z1 - 1.0,
                           rv.SHEET_Z1 + rv.PLATE_T + 1.0, x, y))
    cuts.append(_cyl_z(rv.CENTRE_HOLE_D / 2.0, rv.SHEET_Z1 - 1.0,
                       rv.SHEET_Z1 + rv.PLATE_T + 1.0))
    return step._diff(body, *cuts)


def make_corner_pillar() -> object:
    """Plain elliptical rim column with Wago-bay foot plate and inboard tab
    (rv.make_corner_pillar), modeled at az 0."""
    top_z = rv.SHEET_Z0 - rv.PILLAR_TOP_GAP

    def _ecyl(r_rad: float, z0: float, z1: float) -> object:
        with BuildPart() as col:
            with BuildSketch(Plane.XY):
                Ellipse(r_rad, r_rad * rv.PILLAR_TAN_SCALE)
            extrude(amount=z1 - z0)
        return Pos(rv.PILLAR_RHO, 0.0, z0) * col.part

    col = _ecyl(rv.PILLAR_OD / 2.0, rv.PILLAR_BOT_Z, top_z)
    bay_x0 = rv.PILLAR_RHO + 2.0
    bay_x1 = rv._BAY_OUT_X - rv.PILLAR_KEY_CL
    bay_half = rv._WAGO_BAY_W / 2.0 - rv.PILLAR_KEY_CL
    bar = _box((bay_x1 - bay_x0, 2.0 * bay_half, rv.PILLAR_FOOT_T),
               ((bay_x0 + bay_x1) / 2.0, 0.0,
                rv.PILLAR_BOT_Z + rv.PILLAR_FOOT_T / 2.0))
    tab = _box((12.0, 12.0, rv.PILLAR_FOOT_T),
               (rv.PILLAR_RHO - rv.PILLAR_OD / 2.0 - 2.0, 0.0,
                rv.PILLAR_BOT_Z + rv.PILLAR_FOOT_T / 2.0))
    body = step._union(col, bar, tab)
    cuts = [
        _cyl_z(rv.PILOT_OD / 2.0, top_z - 8.0, top_z + 1.0,
               x=rv.HATCH_SCREW_RHO),
        _cyl_z(rv.PILOT_OD / 2.0, top_z - 8.0, top_z + 1.0,
               x=rv.PILLAR_FRAME_SCREW_RHO),
        _cyl_z(rv.HOLE_D / 2.0, rv.PILLAR_BOT_Z - 1.0,
               rv.PILLAR_BOT_Z + rv.PILLAR_FOOT_T + 1.0, x=rv.PILLAR_TAB_RHO),
    ]
    for sy in (+1.0, -1.0):
        cuts.append(_cyl_z(rv.HOLE_D / 2.0, rv.PILLAR_BOT_Z - 1.0,
                           rv.PILLAR_BOT_Z + rv.PILLAR_FOOT_T + 1.0,
                           x=rv.PILLAR_BAR_HOLE_X,
                           y=sy * rv.PILLAR_BAR_HOLE_Y))
    return step._diff(body, *cuts)


def make_centre_wago_block() -> object:
    """Central 4-bay Wago 221-415 splice block (rv.make_centre_wago_block)."""
    z0 = rv.PILLAR_BOT_Z
    zf = z0 + rv.WBLK_FLOOR_T
    top = zf + rv.WBLK_WALL_H
    body = _box((2.0 * rv.WBLK_HALF_X, 2.0 * rv.WBLK_HALF_Y, top - z0),
                (0.0, 0.0, (z0 + top) / 2.0))
    x_c = rv.WBLK_WALL_T / 2.0 + rv.WBLK_BAY_W / 2.0
    cuts = []
    for sy in (+1.0, -1.0):
        for sx in (-1.0, +1.0):
            y0 = rv.WBLK_WALL_T / 2.0
            y1 = rv.WBLK_HALF_Y + 2.0
            cuts.append(_box(
                (rv.WBLK_BAY_W, y1 - y0, rv.WBLK_WALL_H + 2.0),
                (sx * x_c, sy * (y0 + y1) / 2.0,
                 zf + (rv.WBLK_WALL_H + 2.0) / 2.0),
            ))
    return step._diff(body, *cuts)


def make_coxa_link_rigid() -> object:
    """Production coxa re-assembled with the SHORTENED hub column
    (rv.make_coxa_link_rigid): the hub sub-solid is truncated at
    rv.HUB_TRIM_Z with the dust-lip skirt / platform disc deleted, the
    slab + cradle sub-solid drops rv.COL_DROP as one rigid body, the
    seat ring + Phi 38 brim are added, the horn-screw shafts are re-cut
    from the VARIANT seat planes (rv.HORN_HEAD_SEAT_Z /
    rv.HORN_CENTRE_SEAT_Z -- 10 mm deeper, tracking the M3x30 -> M3x20
    swap so the tip planes / horn engagement never move; shank
    clearances re-opened through the dropped slab), and the result is
    trimmed to the rotation envelope."""
    hub = step.make_coxa_yaw_hub(one_piece=True)
    skirt_cut = step._diff(
        _cyl_z(hp.YAW_HUB_DUST_LIP_OD / 2.0 + 2.0, rv.SLAB_BOT_Z - 1.5,
               hp.YAW_HUB_BOSS_TOP_Z + 1.0),
        _cyl_z(20.0, rv.SLAB_BOT_Z - 2.5, hp.YAW_HUB_BOSS_TOP_Z + 2.0),
    )
    hub = step._diff(hub, skirt_cut)
    bb = hub.bounding_box()
    hub = step._intersect(hub, _cyl_z(60.0, bb.min.Z - 1.0, rv.HUB_TRIM_Z))
    bracket = (Pos(0.0, 0.0, -rv.COL_DROP)
               * step.make_coxa_hip_bracket(one_piece=True))
    ring = step._diff(
        _cyl_z(rv.HUB_RING_OD / 2.0, rv.HUB_RING_Z0, rv.HUB_RING_Z1),
        _cyl_z(rv.HUB_RING_ID / 2.0, rv.HUB_RING_Z0 - 1.0,
               rv.HUB_RING_Z1 + 1.0),
    )
    brim = step._diff(
        _cyl_z(rv.BRIM_OD / 2.0, rv.BRIM_BOT_Z, rv.BRIM_TOP_Z),
        _cyl_z(rv.HUB_RING_ID / 2.0, rv.BRIM_BOT_Z - 1.0,
               rv.BRIM_TOP_Z + 1.0),
    )
    body = step._union(hub, bracket, ring, brim)
    hip_ax_x, _, hip_ax_z = rv.COXA_HIP_ANCHOR_V
    cuts = [_cyl_y(16.75, ylo, yhi, x=hip_ax_x, z=hip_ax_z)
            for (ylo, yhi) in ((21.75, 30.0), (-31.0, -24.75))]
    shaft_top_z = 80.0
    drive_clear = hp.DISC_HORN_BOLT_OD + 0.3
    stations = [(0.0, 0.0, rv.HORN_CENTRE_SEAT_Z,
                 hp.HORN_CENTRE_OD)]
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations.extend(
        (r * math.cos(t), r * math.sin(t), rv.HORN_HEAD_SEAT_Z,
         drive_clear)
        for t in hp.DISC_HORN_BOLT_ANGLES_RAD)
    for sx, sy, seat_z, shank_d in stations:
        cuts.append(_cyl_z(hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0, seat_z,
                           shaft_top_z, sx, sy))
        cuts.append(_cyl_z(shank_d / 2.0, rv.SLAB_BOT_Z - 1.0,
                           seat_z + 0.5, sx, sy))
    body = step._diff(body, *cuts)
    bb = body.bounding_box()
    keep = _cyl_z(rv.ROT_ENVELOPE_R, bb.min.Z - 1.0, bb.max.Z + 1.0)
    return step._intersect(body, keep)


def make_chassis_bottom_rigid() -> object:
    """Production chassis bottom + corner trim to the tower cylinder, all
    three dead-ear shaves (az 210 flush to the deck top), pillar-foot
    holes, wago-tray deletes, the per-leg wire-corridor + cradle-shell
    flatten (rv.CHB_FLAT_* box minus the tower keep cylinder -- Aug 24
    rev 5 + Aug 25 rev 6), the Aug 25 LOWERED bearing
    pocket: the old tower band above the new deck-level seat plane is
    cut away (leaving the 0.5 mm-proud Phi 34/Phi 37.15 seat ledge)
    and a fresh full-wrap Phi 44/Phi 37.15 ring is unioned from the
    deck band to the new race top, plus the Aug 25 rev-7 tower-flank
    bump shave (the production swing-relief protect ring, r 23.5, is
    cut back to the trim cylinder so the tower outer profile is one
    vertical cylinder sheet-top -> rim), plus the Aug 25 rev-8
    ABOVE-SHEET WHITELIST cut: everything above the bare sheet top
    outside the six tower cylinders goes, at every azimuth
    (rv.make_chassis_bottom_rigid)."""
    cb = step.make_chassis_bottom()
    ear_r = hp.YAW_CAP_BOLT_PCD / 2.0
    cutters = []
    for i in range(6):
        deg = math.degrees((i + 0.5) * math.pi / 3.0)
        box = _box((23.5, 45.0, rv.CHB_PLATE_TOP + 6.5),
                   (rv.APOTHEM + 23.5 / 2.0, 0.0,
                    (rv.CHB_PLATE_TOP - 6.5) / 2.0))
        corner = step._diff(box, _cyl_z(rv.CHB_TRIM_R, -8.0, 7.5,
                                        x=rv.APOTHEM))
        keep = _cyl_z(rv.CHB_KEEP_R, rv.CHB_PLATE_TOP - 1.0, 21.6,
                      x=rv.APOTHEM)
        ear330 = step._diff(
            _cyl_z(rv.CHB_EAR_R, rv.CHB_PLATE_TOP, 20.6,
                   x=rv.APOTHEM + ear_r * math.cos(-math.pi / 6.0),
                   y=ear_r * math.sin(-math.pi / 6.0)),
            keep,
        )
        box90 = step._diff(
            _box((12.0, 10.0, 20.6 - rv.CHB_PLATE_TOP),
                 (rv.APOTHEM, rv.CHB_WALL_FACE_Y + 5.0,
                  (rv.CHB_PLATE_TOP + 20.6) / 2.0)),
            keep,
        )
        ear210 = step._diff(          # inboard ear, flush to the deck top
            _cyl_z(rv.CHB_EAR_R, rv.CHB_DECK_TOP, 20.6,
                   x=rv.APOTHEM + ear_r * math.cos(math.pi * 7.0 / 6.0),
                   y=ear_r * math.sin(math.pi * 7.0 / 6.0)),
            keep,
        )
        flat = step._diff(            # wire-corridor + cradle-shell flatten
            _box(                     # (Aug 24 rev 5 + Aug 25 rev 6): box to
                (rv.CHB_FLAT_X1 - rv.CHB_FLAT_X0,     # the yaw axis MINUS the
                 2.0 * rv.CHB_FLAT_HALF_Y,            # tower keep cylinder
                 rv.CHB_FLAT_Z1 - rv.CHB_FLAT_Z0),    # (in-keep shell carries
                ((rv.CHB_FLAT_X0 + rv.CHB_FLAT_X1) / 2.0, 0.0,  # the 6805
                 (rv.CHB_FLAT_Z0 + rv.CHB_FLAT_Z1) / 2.0)),     # seat's
            _cyl_z(rv.CHB_KEEP_R,                     # inboard arc)
                   rv.CHB_FLAT_Z0 - 0.5, rv.CHB_FLAT_Z1 + 0.5,
                   x=rv.APOTHEM),
        )
        bump = step._diff(            # tower-flank bump shave (Aug 25 rev 7):
            _box((23.5, 45.0,         # production's swing-relief protect ring
                  (rv.CHB_DECK_TOP + 0.25) - (rv.CHB_PLATE_TOP - 0.25)),
                 (rv.APOTHEM + 23.5 / 2.0, 0.0,   # (r 23.5) bulged the outboard
                  ((rv.CHB_PLATE_TOP - 0.25)      # flank in the mount-plate
                   + (rv.CHB_DECK_TOP + 0.25)) / 2.0)),  # band -- shave flush
            _cyl_z(rv.CHB_TRIM_R, rv.CHB_PLATE_TOP - 1.0,
                   rv.CHB_DECK_TOP + 1.0, x=rv.APOTHEM),
        )
        band = _cyl_z(rv.CHB_TOWER_R + 0.1,   # tower band rebuild (Aug 25):
                      rv.CHB_SEAT_W,          # everything above the NEW seat
                      rv.CHB_RIM_OLD_W + 1.5, # plane goes; the 0.5 mm Phi 34
                      x=rv.APOTHEM)           # band left below is the ledge
        cutters.extend(Rotation(0, 0, deg) * c
                       for c in (corner, ear330, box90, ear210, flat, bump,
                                 band))
    for az in range(0, 360, 60):
        for hx, hy in ((rv.PILLAR_BAR_HOLE_X, +rv.PILLAR_BAR_HOLE_Y),
                       (rv.PILLAR_BAR_HOLE_X, -rv.PILLAR_BAR_HOLE_Y),
                       (rv.PILLAR_TAB_RHO, 0.0)):
            cutters.append(Rotation(0, 0, az)
                           * _cyl_z(rv.HOLE_D / 2.0, -12.0, 12.0, hx, hy))
    for M in hp.wago_tray_corner_transforms():   # wago tray delete (Aug 24)
        deg = math.degrees(math.atan2(M[1, 0], M[0, 0]))
        cutters.append(
            Pos(M[0, 3], M[1, 3], M[2, 3])
            * Rotation(0, 0, deg)
            * _box((2.0 * rv.TRAY_HALF_X + 0.4, 2.0 * rv.TRAY_HALF_Y + 0.4,
                    hp.WAGO_MOUNT_WALL_H + 1.0),
                   (0.0, 0.0, (hp.WAGO_MOUNT_WALL_H + 1.0) / 2.0)))
    wl_keeps = [                  # rev-8 ABOVE-SHEET WHITELIST cut (Aug 25):
        _cyl_z(rv.CHB_WL_KEEP_R,  # one global box from the sheet top up,
               rv.CHB_FLAT_Z0 - 1.0, rv.CHB_WL_Z1 + 1.0,   # minus the six
               x=rv.APOTHEM * math.cos((i + 0.5) * math.pi / 3.0),
               y=rv.APOTHEM * math.sin((i + 0.5) * math.pi / 3.0))
        for i in range(6)]        # tower cylinders -- EVERYTHING else above
    cutters.append(step._diff(    # z 2 goes to the sheet.  This is what
        _box((400.0, 400.0,       # kills the L-stubs the blacklist passes
              rv.CHB_WL_Z1 - rv.CHB_FLAT_Z0),  # missed here: the base STEP
             (0.0, 0.0, (rv.CHB_FLAT_Z0 + rv.CHB_WL_Z1) / 2.0)),  # chassis
        *wl_keeps))               # still models the RETIRED two-bay WAGO3
    # corner trays, 4.8 mm wider than the WAGO5 tray the tray-delete
    # cutter above is sized for, so their side walls survived at all six
    # corners (the circled diagonal slashes / L-brackets).  The rims are
    # unioned after, so the towers are untouched.
    body = step._diff(cb, *cutters)
    rims = []                    # rebuilt full-wrap tower rings: Phi 44 /
    for i in range(6):           # Phi 37.15, deck band to the race top
        deg = math.degrees((i + 0.5) * math.pi / 3.0)
        rim = step._diff(
            _cyl_z(rv.CHB_TOWER_R, rv.CHB_SEAT_W - 1.0, rv.CHB_RIM_W,
                   x=rv.APOTHEM),
            _cyl_z(rv.POCKET_BORE / 2.0, rv.CHB_SEAT_W - 2.0,
                   rv.CHB_RIM_W + 1.0, x=rv.APOTHEM))
        rims.append(Rotation(0, 0, deg) * rim)
    return step._union(body, *rims)


# --- specs, checks, main -----------------------------------------------------

def rigid_hip_part_specs() -> list[StepPart]:
    base = PROTO_DIR / "concepts" / "rigid_hip" / "stl"
    return [
        StepPart(
            "hip_clamp_cap_rigid",
            make_hip_clamp_cap_rigid,
            base / "hip_clamp_cap_rigid.stl",
            "Stock clamp cap grown a yaw-axis pedestal and 6805 press boss.",
        ),
        StepPart(
            "chassis_top_rigid",
            make_chassis_top_rigid,
            base / "chassis_top_rigid.stl",
            "Top frame: hex sheet, six bearing pockets, service-hatch opening.",
        ),
        StepPart(
            "top_hatch_rigid",
            make_top_hatch_rigid,
            base / "top_hatch_rigid.stl",
            "Removable service lid with registration lip and screw ears.",
        ),
        StepPart(
            "corner_pillar",
            make_corner_pillar,
            base / "corner_pillar.stl",
            "Elliptical rim column tying the frame to the bottom sheet (6x).",
        ),
        StepPart(
            "centre_wago_block",
            make_centre_wago_block,
            base / "centre_wago_block.stl",
            "Central 4-bay Wago splice block under the hatch.",
        ),
        StepPart(
            "coxa_link_rigid",
            make_coxa_link_rigid,
            base / "coxa_link_rigid.stl",
            "Production coxa + hub seat ring + dust brim, envelope-rounded (6x).",
        ),
        StepPart(
            "chassis_bottom_rigid",
            make_chassis_bottom_rigid,
            base / "chassis_bottom_rigid.stl",
            "Production chassis bottom with tower-cylinder corners, the "
            "lowered deck-level bearing pocket and foot holes.",
        ),
    ]


def _equivalence_problems(rows: list[dict]) -> list[str]:
    """Fail loudly if a BREP part drifts from its mesh-pipeline twin.

    Tessellation slop (192-gon cylinders etc.) keeps volumes within a
    fraction of a percent; anything past 2 percent or 0.6 mm of bbox is a
    porting error, not tessellation.

    chassis_bottom_rigid additionally gets the rev-8 ABOVE-SHEET
    WHITELIST census on the derived STL (the same
    rv.chassis_whitelist_violations the mesh pipeline asserts): the
    pre-rev-8 STEP part carried 12 stale tray-wall stubs the volume
    gate was too loose to catch (+1.5 percent, under the 2 percent
    gate) -- the whitelist census catches a single leftover wall."""
    import trimesh

    from step_common import THIS_DIR as _here

    problems = []
    for row in rows:
        legacy = row["legacy_stl"]
        if legacy is None:
            problems.append(f"{row['name']}: variant mesh STL missing")
            continue
        dv = abs(row["brep_volume_mm3"] - legacy["volume_mm3"])
        if dv / legacy["volume_mm3"] > 0.02:
            problems.append(
                f"{row['name']}: BREP volume {row['brep_volume_mm3']:.0f} mm3 "
                f"vs mesh {legacy['volume_mm3']:.0f} mm3"
            )
        worst = max(abs(d) for d in row["bbox_size_delta_vs_legacy_mm"])
        if worst > 0.6:
            problems.append(
                f"{row['name']}: bbox size delta {worst:.3f} mm vs mesh"
            )
        if row["name"] == "chassis_bottom_rigid":
            derived = trimesh.load(_here / row["stl"])
            n_bad, worst_r = rv.chassis_whitelist_violations(derived)
            if n_bad:
                problems.append(
                    f"{row['name']}: {n_bad} vertices above the sheet "
                    f"outside the tower whitelist (worst r {worst_r:.2f})"
                )
    return problems


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    exported = export_all(rigid_hip_part_specs())
    problems = _equivalence_problems(exported)
    manifest = {
        "units": "mm",
        "source": (
            "build123d/OpenCascade BREP, constants imported from "
            "concepts/rigid_hip/make_rigid_hip_variant.py"
        ),
        "exported_parts": exported,
        "checks": {
            "passed": not problems,
            "problems": problems,
        },
        "files": [rel for row in exported for rel in (row["step"], row["stl"])],
    }
    manifest_path = OUT_DIR / "rigid_hip_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    bundle = write_bundle(manifest, "rigid_hip_step_first_bundle.zip",
                          "rigid_hip_manifest.json")
    print(f"wrote {manifest_path.relative_to(THIS_DIR)}")
    print(f"wrote {bundle.relative_to(THIS_DIR)}")
    if problems:
        print("rigid-hip STEP checks failed:")
        for problem in problems:
            print(f"  - {problem}")
        raise SystemExit(1)
    print("rigid-hip STEP sidecar complete; the mesh generator was not modified.")


if __name__ == "__main__":
    main()
