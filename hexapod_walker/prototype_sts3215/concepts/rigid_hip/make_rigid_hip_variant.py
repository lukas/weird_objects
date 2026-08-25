"""VARIANT: rigid-hip yaw axis -- a THIRD 6805 bearing above each hip,
carried by an extended hip clamp cap, seated in a full-size top chassis.

Concept (user, Aug 2026): the production yaw joint carries the coxa on a
6805-2RS pair in chassis_bottom's tower, but the hip servo above it is
still a cantilever -- leg loads twist the coxa link about the bearing
pair.  This variant closes the loop from the TOP:

  * ``hip_clamp_cap_rigid`` -- the stock ``make_servo_clamp_cap()`` (the
    hip clamshell lid) grown UP along the yaw axis: a Phi 29 pedestal
    (inner-race seat shoulder, same role as the yaw hub's uflange) and a
    Phi 25.15 press boss (= ``YAW_HUB_BOSS_OD``, the bench-tuned +0.15
    interference the lower hub boss already uses) that a third 6805-2RS
    presses onto.  Knee caps stay stock.
  * ``chassis_top_rigid`` -- a SECOND 200 mm flat-to-flat hex plate
    (same footprint/thickness as chassis_bottom's sheet) whose six
    Phi 44 bosses pocket the bearings' outer races at Phi 37.15
    (= ``YAW_TOWER_BORE_OD``, the bench-tuned firm finger-press fit),
    race retained by the Phi 34 shoulder (= ``YAW_TOWER_SHOULDER_OD``).
    Six Phi 7 DRIVER ACCESS holes sit above the inboard cap bolts (legs
    at yaw 0): the cap is a CAPTIVE BEARING CARRIER -- its 6805 is
    pressed on once and never removed; all service unbolts the cap from
    the coxa cradle (both bolts reachable with the plate on) and lifts
    the plate + caps + bearings off as one rigid unit.
  * ``corner_pillar`` (x6) -- plain solid elliptical printed columns
    at the six corner azimuths (rho 81.6, between adjacent rings);
    >= 5 mm leg clearance at every yaw angle is guaranteed by the
    rotating parts' ROT_ENVELOPE_R trim (see ``coxa_link_rigid``),
    not by shaping the column.  They tie the top
    frame to chassis_bottom at the RIM, where each hip moment's force
    couple actually wants to react (push at the bottom tower, pull at
    the top ring) and where torsional leverage is ~4x the old standoff
    radius.  Each pillar doubles as the lid-screw boss: the hatch
    perimeter screw threads into the pillar top through the frame; one
    dedicated frame screw per pillar keeps the frame clamped with the
    lid off; two M3 through-bolts with belly nylocs hold each foot
    through printed holes in ``chassis_bottom_rigid`` (on a STOCK
    chassis print: drill them using the foot as the jig).  The four
    central 90 mm standoffs remain only as hatch/electronics anchors.
  * ``coxa_link_rigid`` (x6) -- the production coxa with four variant
    edits: the vertical hub column SHORTENED by COL_DROP (the Phi 52.4
    platform disc, dust-lip skirt and uflange -- all sized around the
    deleted production cap -- are removed, the horn screws swap
    M3x30 -> M3x25 (same thread engagement, seats 5 mm deeper), and
    the slab + cradle unit drops so the well floor sits just over the
    screw heads; see the COXA SHORTEN constants), servo-cradle corners
    rounded to the 38.2 mm yaw envelope (max 2.16 mm off two vertical
    wall corners), the hub grown a Phi 29 seat ring down to the
    relocated bottom race (see BEARING COUNT), and a small Phi 38
    DUST BRIM hovering 0.5 above the race-top / tower-rim plane --
    non-contact, stepped 3 mm INSIDE the tower Phi 44 so the coxa
    reads as coxa, not as more chassis column (see the BRIM_*
    constants).  Horn interface, cradle pilots and cap seat
    untouched.
  * ``chassis_bottom_rigid`` (x1) -- the production chassis with the
    six tower platforms trimmed to the tower's own Phi 44 cylinder
    (user, Aug 24: corners rounded; rev 2: all corner curves must
    match -- so the tower curve is now the ONLY curve), each tower
    RIM RAISED 3 mm to the race-top plane so the single bearing is
    FULLY housed (full 7 mm outer-race wrap; the Phi 44 column ends
    exactly at the bearing top -- user, Aug 24: "the bearing should
    be just above the horn and the motor should sit just above
    that"), the dead cap-bolt ear lugs shaved (outboard + tangential
    + the inboard one cut flush to the servo-mount deck, its root
    left merged with the well collar below it), the 18 pillar-foot
    holes printed in, the six corner Wago TRAY WALL SETS DELETED
    (user, Aug 24 -- the splices live in centre_wago_block now, so
    the trays are dead geometry), and the wago-era WIRE-CORRIDOR
    APPARATUS inboard of each seated yaw servo FLATTENED to the
    sheet (user, Aug 24 rev 5 -- "that CRADLE WALL isnt doing shit,
    just flatten it out"; see the CHB_FLAT_* constants).  See the
    CHB_* constant block.  Every remaining functional surface
    (pocket, seat, well collar, slots) is production geometry.
  * ``centre_wago_block`` -- the corner Wagos are gone (pillars stand
    there), so the power tree consolidates: 4x 5-port 221-415 (two per
    net, jumpered) in one printed press-fit block at the chassis
    centre, under the open hatch, replacing the 6 corner + 2 trunk
    nuts.

  Load path: hip moment -> cap boss -> top bearing -> top plate ->
  six rim pillars -> chassis_bottom / five other legs.  Each yaw axis
  becomes simply-supported (one bearing below, one above, ~65 mm
  mid-plane to mid-plane) instead of cantilevered.

  BEARING COUNT (user, Aug 24: "we really only need one bearing on
  the bottom -- simplify"): ONE bottom bearing per leg, seated
  directly in chassis_bottom's own open-top Phi 37.15 pocket (the
  production LOWER-race position: drop-in path and z=0.5 seat are
  print-proven).  The bolt-on ``yaw_bearing_cap`` is DELETED -- it
  only existed to house the second race and retain the pair, and the
  top plate now does both jobs: standing loads go UP through the top
  bearing into the plate shoulder, hanging loads go DOWN through the
  hub's new Phi 29 seat ring -> inner race -> outer race -> tower
  seat.  -6 printed caps, -18 M3x8 cap screws, and the leg + bearing
  lift straight out once the plate is off (horn centre screw only).
  The tower rim is RAISED to the race-top plane (full 7 mm wrap), so
  nothing stands proud and the coxa sits directly on the bearing
  (user, Aug 24 -- see the TOWER RIM / BRIM constants).  Net bearings
  per robot: 12, same as production.

TRADE-OFF (measured by the sweep in this script): the full-size top
plate caps the femur's UP-swing.  The production workspace envelope is
femur in [-80, +30] deg; the plate + its bearing bosses cut the up
limit to roughly -55 deg (exact per-yaw contact angles printed at
build time and baked into the scene's joint limits).  Walking gaits
around STANCE_FEMUR_DEG = -25 are unaffected; deep leg tucks (e.g.
stand-up curls) must be re-checked against the new envelope before
this variant goes on the robot.

NOT a production change: nothing in the verified parts registry moves.
``hexapod_prototype.py`` is a read-only input; the printed parts unique
to this variant live in THIS directory's ``stl/`` (never in
``stl_prototype/``).  BuildViz build id: ``sts3215-rigid-hip``.

Run:  <repo .venv python> concepts/rigid_hip/make_rigid_hip_variant.py
      (--skip-sweep for fast geometry-only iterations)
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

HERE = os.path.abspath(os.path.dirname(__file__))
PROTO_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
STL_DIR = os.path.join(HERE, "stl")
sys.path.insert(0, PROTO_DIR)

import hexapod_prototype as hp  # noqa: E402  (read-only input)

# ---------------------------------------------------------------------------
# Derived constants -- every fit re-uses a bench-tuned production constant.
# ---------------------------------------------------------------------------
BEARING_ID = hp.YAW_BEARING_ID            # 25 (6805-2RS)
BEARING_OD = hp.YAW_BEARING_OD            # 37
BEARING_W = hp.YAW_BEARING_W              # 7
BEARING_INNER_OD = hp.YAW_BEARING_INNER_OD  # 29 -- inner-race outer diameter

BOSS_OD = hp.YAW_HUB_BOSS_OD              # 25.15 -- inner-race press (+0.15)
POCKET_BORE = hp.YAW_TOWER_BORE_OD        # 37.15 -- outer-race press bore
SHOULDER_OD = hp.YAW_TOWER_SHOULDER_OD    # 34   -- race retaining lip
RING_OD = BEARING_OD + 2.0 * hp.YAW_TOWER_WALL  # 44 -- same wall as the tower

PED_OD = BEARING_INNER_OD                 # 29 -- pedestal = inner-race seat
PED_H = 5.5                               # pedestal height above the cap face
PULLER_NOTCH_W = 8.0                      # two pry slots under the inner race
PULLER_NOTCH_DEPTH = 2.2                  # slot depth below the race seat
PULLER_NOTCH_R0 = 13.0                    # slot inner face: exposes the race
                                          # underside r 13.0..14.5, stays
                                          # 0.4 mm clear of the Phi 25.15 boss
RING_BOT_CL = 0.5                         # ring bottom vs race bottom (race
                                          # protrudes -> shoulder seats first)
BOSS_TIP_STEP = 0.575                     # stepped lead-in: tip at Phi 24.0
BOSS_TIP_H = 0.8
POCKET_LEADIN = 0.8                       # pocket-mouth lead-in (per side)

PLATE_T = hp.CHASSIS_PLATE_T              # 4 -- same sheet as chassis_bottom
CENTRE_HOLE_D = 40.0                      # wiring / standoff-wrench access
HOLE_D = hp.BRACKET_BOLT_HOLE             # 3.4 -- M3 clearance
ACCESS_HOLE_D = 7.0                       # driver pass-through above the
                                          # INBOARD cap bolt (at yaw 0): the
                                          # cap unbolts from the coxa with
                                          # the plate installed, so the cap +
                                          # its pressed bearing come off as
                                          # one captive unit -- the bearing
                                          # fits are never fought in service

# Removable service hatch (user, Aug 2026): a large hex chunk of the top
# plate is cut out and replaced by a screw-down lid, restoring interior
# access (electronics, wiring, yaw-cap bolts, standoff screws) that the
# full-size plate had buried.  The structural work happens at the RIM --
# the six bearing rings at the edge midpoints -- so the middle can open.
# Opening flats face the rings (14 mm bottom web kept at every ring);
# opening vertices point at the plate corners where the frame is widest.
HATCH_OPEN_APO = 64.0                     # opening apothem (through-cut)
HATCH_APO = 68.0                          # lid apothem: 4 mm overlap onto the
                                          # deck, and its az-30 flats stop
                                          # 1.3 mm short of the Phi 7 driver
                                          # holes so cap access stays clear
HATCH_LIP_CL = 0.3                        # lid registration lip vs opening
HATCH_LIP_H = 1.5                         # lip drop into the opening
HATCH_LIP_W = 2.7                         # lip ring radial width
HATCH_SCREW_RHO = 76.2                    # 6x M3 at the opening's VERTEX
                                          # azimuths (0..300 deg): clear of
                                          # rings, driver holes and the
                                          # opening corner (0.6 mm margins,
                                          # asserted in check_static)
HATCH_EAR_OD = 9.0                        # round lid ears around the screws:
                                          # the bare hex corner leaves only
                                          # ~0.3 mm wall at the hole (hub
                                          # check caught it); the ear gives
                                          # a 2.8 mm annulus, over solid frame
PILOT_OD = hp.CLAMP_BOLT_PILOT_OD         # 2.5 -- M3 self-tap (insert-ready)

# Corner pillars (user, Aug 23 eve): the top-bottom tie must carry each
# hip moment's force couple as RIM SHEAR (push at the bottom tower, pull
# at the top ring) -- the four central 90 mm M3 standoff stacks are
# slender bending columns at rho 44 and were the compliance that would
# have given the rigidity back.  Six printed pillars at the corner
# azimuths (between adjacent rings, the only rim territory outside
# every swing envelope) tie the plates at the rim, where torsion
# leverage scales with r^2.
# Each pillar doubles as the lid-screw boss: the hatch perimeter screw
# passes lid -> frame -> pillar top, and one dedicated frame screw per
# pillar keeps the frame clamped with the lid off.  The four central
# standoffs remain only as hatch/electronics anchors.
PILLAR_OD = 20.0                          # column RADIAL outer diameter
PILLAR_TAN_SCALE = 0.7                    # ELLIPTICAL section: tangential
                                          # half-axis 7 (a round Phi 20 was
                                          # a measured graze on the coxa
                                          # sweep at 2.9 mm; slimming the
                                          # tangential axis buys the margin)
# ROUNDED-CORNER ENVELOPE (user, Aug 24: "round the corners of the
# servo holder instead of making the pillar a weird shape").  The
# scalloped column is gone; instead, everything that rotates with a
# yaw joint is kept inside a 38.2 mm cylinder about its own axis, and
# the column is a plain solid ellipse.  Axis-to-column-surface is
# 43.24 mm (measured: 43.24 - 40.36 gave the old 2.88 graze), so a
# <= 38.2 mm rotating envelope guarantees >= 5 mm clearance at EVERY
# yaw angle -- same guarantee the scallops gave, now carried by the
# rotating parts:
#   * coxa_link_rigid -- the production coxa's servo-cradle corners
#     reached 40.36 mm; they are rounded back to the envelope arc
#     (max 2.16 mm off two vertical wall corners, 8 vertices; the
#     cap-bolt bosses are untouched).  THE COXA IS NOT A STOCK PRINT
#     in this variant (6 reprints) -- the price of the plain column.
#   * hip_clamp_cap_rigid already fits (max reach 36.98) -- asserted.
#   * hip servo: max reach 29.38, COTS, nothing to trim.
ROT_ENVELOPE_R = 38.2                     # max rotating reach, enforced
PILLAR_MIN_CL = 5.0                       # guaranteed clearance to column
ROT_BAND_Z0 = 24.0                        # lowest z where anything rotates
                                          # outside the (static) tower --
                                          # the M3x25 drop puts the slab
                                          # bottom at world 24.25, right on
                                          # this bound (checked vs the wago
                                          # lever tops in check_wago_block)

# BOTTOM JOINT (user, Aug 24: "we really only need one bearing on the
# bottom -- so maybe its possible to simplify the design").  The
# variant already ran one bottom bearing, but kept the production
# apparatus around it: the bolt-on yaw_bearing_cap (3x M3 each)
# housing the race in its own bore, sitting over an EMPTY production
# pocket.  Now the single race sits IN that pocket -- the production
# LOWER-race position (open-top Phi 37.15 bore, seat at coxa-local
# z 0.5, a print-proven drop-in path) -- and the cap is DELETED
# (-6 prints, -18 screws, one less part in the race-to-race tolerance
# stack).  The coxa hub grows a Phi 29 ring (the production uflange
# OD, which bears only on the Phi 25..29 inner-race land) from the
# uflange underside (z 14.5) down to the relocated race top (z 7.5);
# the coxa is already a variant print, so the ring is free.  Axial:
# hanging legs load hub -> ring -> inner race -> outer race -> tower
# seat; standing loads go up through the TOP bearing into the plate
# shoulder -- each bearing takes one direction, no cap lip needed.
# TOWER RIM (user, Aug 24: "the bearing should be just above the horn
# and the motor should sit just above that -- feel free to redesign").
# The race is radially housed by the tower pocket, whose rim is RAISED
# from the production split plane (hp.YAW_SPLIT_Z = 4.5, a 4 mm wrap
# that left the race's top 3 mm proud) to the RACE-TOP plane (z 7.5):
# the full 7 mm outer-race width is wrapped (+75% press area vs the
# 4 mm band -- more is strictly better here, the production 4/3 split
# was cap housing, not a design optimum), nothing stands proud, and
# the Phi 44 chassis column ends exactly at the bearing top.  An
# earlier revision instead covered the proud band with a Phi 44
# skirt+curtain hanging from the coxa, flush with the tower -- which
# made tower + race + skirt read as ONE 16 mm chassis column (the
# user complained twice).  The curtain is DELETED; what remains is a
# small Phi 38 DUST BRIM on the coxa, hovering 0.5 above the rim /
# race-top plane and stepped 3 mm inside the tower OD, so the visual
# stack is horn -> bearing (in the tower) -> coxa.  Grit now has one
# turn (under the brim, through the 0.5 axial gap) to reach the 2RS
# seal instead of three -- acceptable: the seal is the real barrier,
# and the brim still roofs the seal + outer-race band completely.
# Non-contact everywhere: brim-to-rim and brim-to-race gaps are both
# BRIM_GAP, and the rotating seat ring keeps 4.0 mm radial to the
# pocket bore.  The remaining az-210 cap-bolt ear root is below the
# deck (z 10.25 world), far under the raised rim.
#
# COXA SHORTEN (user, Aug 24 rev 3: "from around 25mm to 40mm above
# the platform top seems largely unnecessary.  You need to redesign
# the coxa link to make it shorter, theres a bunch of unneccessary
# stuff there").  Audit of that band (coxa-local z ~10..25) found the
# production coxa's vertical hub column was sized around apparatus
# this variant DELETED:
#   * the Phi 52.4 dust-lip SKIRT (z 14..18) labyrinthed over the
#     production cap ring -- the cap is gone and the variant has its
#     own Phi 38 brim at z 8..10, so the skirt hung in mid-air;
#   * the Phi 52.4 platform DISC (z 18..24) existed to clear the
#     production cap rim at z 16.5 by YAW_HUB_CAP_AXIAL_CL and to
#     hang that skirt -- with the cap deleted and the bearing down at
#     z 0.5..7.5, that clearance stack is filler;
#   * the production uflange (z 14.5..16.5) retained the deleted
#     UPPER race -- the variant's seat ring at the relocated race
#     already does its job.
# What is NOT filler is the horn-screw stack: production bench-pinned
# the M3x30 head seats at YAW_HUB_HORN_HEAD_SEAT_Z = +17.75 (screw
# length + measured print shortfall + tip poke -- the tips cannot go
# deeper without jamming the disc off its seat, see the constants in
# hexapod_prototype).  The redesign drops the whole slab + cradle
# unit (a rigid body: cradle pilots, cap seat, 688 housing, rear tab
# all ride along) by COL_DROP so the well floor lands COL_HEAD_CL
# above the screw heads, deletes the skirt + disc outright, and
# truncates the hub boss at the dropped slab.  The hip axis and
# EVERYTHING keyed to it (cap boss, top bearing, top plate, pillars,
# hatch, standoffs) drop by the same COL_DROP -- shorter pillars,
# shorter robot, same simply-supported stack.
#
# HORN SCREWS SHORTENED M3x30 -> M3x25 (user, Aug 24 rev 4: "shorten
# the screws thats fine, that middle section is pointless").  After
# the rev-3 drop the hub band between the bearing brim and the servo
# well floor existed almost entirely to house the M3x30s' length, so
# all 5 horn screws per hub (4 drive + 1 centre; 30 per robot --
# PURCHASED-HARDWARE CHANGE) become M3x25, and every seat plane drops
# by EXACTLY the 5 mm length delta.  Seat - length = tip, so the tip
# planes and the thread engagement are IDENTICAL to the bench-tuned
# production stack: corner tips still break the disc horn's far face
# by TIP_POKE (full 2 mm tapped-disc engagement), the centre screw
# still reaches the same depth in the output spline's tapped bore.
# check_coxa_column asserts the seat drop == the length delta.
# Why not M3x20: another -5 would put SLAB_BOT_Z at 4.0, i.e. the
# rotating cradle slab 3.5 mm BELOW the static tower rim (7.5) --
# collision.  M3x25 is the shortest standard length that fits this
# joint; it lands the slab 1.5 mm over the rim (>= 0.5 required).
HORN_BOLT_LEN = 25.0                      # M3x25 SHCS -- was M3x30 (30x/robot)
HORN_SEAT_DROP = hp.YAW_HUB_HORN_BOLT_LEN - HORN_BOLT_LEN    # 5.0 -- seats
                                          # follow the screws down 1:1 so the
                                          # tips (= seat - length) never move
HORN_HEAD_SEAT_Z = (hp.YAW_HUB_HORN_HEAD_SEAT_Z
                    - HORN_SEAT_DROP)     # 12.75 -- 4 corner-drive seats
HORN_CENTRE_SEAT_Z = (hp.YAW_HUB_HORN_CENTRE_SEAT_Z
                      - HORN_SEAT_DROP)   # 11.75 -- centre spline-screw seat
COL_HEAD_CL = 1.25                        # servo belly over the M3 heads
                                          # (>= the ~1 mm FDM axial stack-up
                                          # this axis measured on the bench)
COXA_FLOOR_Z = (HORN_HEAD_SEAT_Z
                + hp.INSERT_M3_BOLT_HEAD_H + COL_HEAD_CL)   # 17.0 well floor
COL_DROP = (hp.YAW_HUB_PLATFORM_Z1
            + hp.COXA_WELL_FLOOR_LIFT) - COXA_FLOOR_Z       # 9.0 -- the shrink
SLAB_BOT_Z = hp.YAW_HUB_BOSS_TOP_Z - COL_DROP     # 9 -- dropped slab bottom
HUB_TRIM_Z = SLAB_BOT_Z + 0.75            # hub boss keeps 0.75 into the slab
COXA_HIP_DROP_V = hp.COXA_HIP_DROP - COL_DROP     # 29.4 -- hip axis, coxa z
COXA_HIP_ANCHOR_V = (hp.COXA_HIP_ANCHOR[0], hp.COXA_HIP_ANCHOR[1],
                     COXA_HIP_DROP_V)     # the variant's hip joint anchor
YAWBR_DROP = -hp.YAW_BEARING_W            # -7: race to the tower pocket
HUB_RING_OD = hp.YAW_BEARING_INNER_OD     # 29 -- production uflange OD
HUB_RING_ID = 24.0                        # overlaps the boss wall (22..25.15)
HUB_RING_Z0 = hp.YAW_BEARING_LOWER_TOP_Z  # 7.5 -- relocated race TOP
HUB_RING_Z1 = SLAB_BOT_Z + 1.0            # 10 -- 1 mm into the dropped slab
                                          # (was uflange-referenced; the
                                          # uflange is deleted with the
                                          # shortened column)
BRIM_GAP = 0.5                            # running gap, axial + radial
TOWER_RIM_Z = hp.YAW_BEARING_LOWER_TOP_Z  # 7.5 -- raised rim = race top
                                          # (coxa-local; world 22.75)
BRIM_OD = hp.YAW_BEARING_OD + 2.0 * BRIM_GAP     # 38 -- roofs the seal + outer
                                          # race (r 18.5) plus the gap; 3 mm
                                          # inside the Phi 44 tower so the
                                          # brim reads as coxa, not chassis
BRIM_BOT_Z = TOWER_RIM_Z + BRIM_GAP       # 8.0 -- hovers 0.5 over rim AND race
BRIM_TOP_Z = 10.0                         # 2 mm brim, fuses into the seat ring

# CHASSIS VARIANT (user, Aug 24: "round the corners on the chassis
# bottom below where the bearings go"; rev 2 same day: "I don't like
# the multiple curves here, they should all match").  Each production
# yaw tower stands on a SQUARE outboard platform (leg-frame face
# x 121.2; half-width 21.25 above the sheet, 18.9 for the z -6..-2
# belly skirt; top face z 6.25) whose corners poked 8.4 mm diagonally
# past the Phi 44 tower.  Rev 1 rounded them with a bolt-on R 10
# corner radius, which stacked THREE different arcs at each corner
# (platform R 10 arc, the shallower chord the same cylinder cut into
# the narrower skirt, and the r 22 tower crown right above) -- the
# user rejected the mismatch.  Now the corners are simply TRIMMED TO
# THE TOWER'S OWN CYLINDER (r 22.02 about the yaw axis; 0.02 proud so
# the boolean never grazes the tower's own tessellation).  In
# production the tower already bulges through the platform band (its
# r 22 circle crosses the +-21.25 side faces at x 100+-5.9), so the
# trim just continues that same curve around the end and down the
# skirt: every band shows ONE curve -- the tower's -- and the
# side-face junction is the crease production already has.  The
# bearing pocket, well walls and rim are untouched.  With the
# yaw_bearing_cap deleted, the three M3 cap-bolt ear lugs per tower
# (PCD 47, z 6.25..19.75) are dead weight:
#   * the OUTBOARD ear (coxa az 330) sat ON the trimmed corner --
#     shaved flush to the tower cylinder (0.05 bite into the tower
#     skin for clean topology);
#   * the TANGENTIAL ear (az 90) poked 6.8 mm past the platform
#     silhouette -- shaved flush to the rim-wall face (y 20.45);
#   * the INBOARD ear (az 210, r to 28 from the yaw axis) is shaved
#     flush to the SERVO-MOUNT DECK TOP (z 10.25): the free-standing
#     column above the deck (z 10.25..19.75, its M3 pilot served the
#     deleted cap) is cut away, while the below-deck root (6.25..
#     10.25) STAYS merged with the well-mouth collar -- cutting there
#     risks gouging the collar, and it invisibly stiffens the deck.
#     After this, NOTHING pokes past the tower cylinder above the
#     deck at any azimuth: the six towers read as clean bare columns.
# Since the chassis is a variant print now anyway, the 18 pillar-
# foot bolt holes are PRINTED IN (the foot-as-drill-jig bench mod
# remains the documented path for modifying a STOCK chassis print).
CHB_PLATE_TOP = 6.25          # platform top face = ear-lug bottom (measured)
CHB_DECK_TOP = 10.25          # servo-mount deck top face (measured); the
                              # az-210 ear is shaved flush to THIS plane
CHB_TOWER_R = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL   # 22.0
CHB_TRIM_R = CHB_TOWER_R + 0.02   # corner trim = the tower cylinder itself
CHB_KEEP_R = CHB_TOWER_R - 0.05   # ear cuts bite 0.05 into the tower skin
CHB_RIM_OLD_W = hp.CHASSIS_YAW_OUTPUT_Z + hp.YAW_SPLIT_Z    # 19.75 -- the
                              # production rim (4 mm race wrap), world z;
                              # the raise ring overlaps 1 mm below it
CHB_RIM_W = hp.CHASSIS_YAW_OUTPUT_Z + TOWER_RIM_Z           # 22.75 -- raised
                              # rim = race-top plane, world z (full 7 mm wrap)
CHB_EAR_R = 6.6               # covers the Phi 9 ear boss with margin
CHB_WALL_FACE_Y = 20.45       # rim-wall outer face (20.33 measured) + cl
# CORRIDOR FLATTEN (user, Aug 24 rev 5: "take a step back that CRADLE
# WALL isnt doing shit, just flatten it out and its fine" / "its
# flatening random bumps in the top of the chassis plate that serve
# no purpose").  Inboard of each seated yaw servo the plate top still
# carried the wago-era WIRE-CORRIDOR apparatus: the cradle's
# transverse end wall (leg-frame x 61.5..64.5, full height sheet-top
# z 2 -> deck top 10.25, standing 0.3 mm off the servo's inboard end
# face, with the harness notch at |y| <= 3.5), a "porch" canopy skin
# (z ~6.3/8 -> 10.25) roofing a hollow corridor down to x ~50, and
# side-wall stubs at |y| ~14..18.9.  The wires route to the CENTRE
# splice block in this variant (corner trays deleted), so none of it
# registers, seals or carries anything -- the harness just exits the
# well into open air.  One box per leg flattens the whole band back
# to the bare sheet.  KEPT: the servo-mount deck plateau + well-mouth
# collar outboard of the cut (the servo end face sits at x 64.8; the
# cut face splits the old 0.3 mm wall-to-servo gap), the side walls
# flanking the servo body at |y| ~13..19 outboard of x 64.65, the
# towers, and the yaw_servo_retainer territory: the retainer bolts
# from the BELLY into plate pilots at leg-frame x 71/87.5, |y| 21,
# z -2..1 -- outside the cut in x, y AND z (verified).
CHB_FLAT_X0 = 50.0            # inboard face: past the porch's x~52 lip
CHB_FLAT_X1 = 64.65           # splits wall face 64.5 / servo face 64.8
CHB_FLAT_HALF_Y = 20.5        # covers stubs to |y| 18.9; walls at 19+
                              # near the tower are outboard of X1
CHB_FLAT_Z0 = hp.CHASSIS_PLATE_T / 2.0    # 2.0 -- the bare sheet top
CHB_FLAT_Z1 = 12.0            # clears the deck top (10.25)
PILLAR_RHO = 81.6                         # centre radius at az 0/60/...:
                                          # midway between the lid-screw
                                          # pilot (76.2) and the dedicated
                                          # frame-screw pilot (87.0) so both
                                          # get equal 3.35 mm plug walls
PILLAR_TOP_GAP = 0.1                      # nominal gap to the frame sheet:
                                          # the six RACES define the plate
                                          # plane; the two top screws pull
                                          # the sheet down onto the pillar
                                          # (never the reverse -- shim/sand
                                          # a proud pillar, do not rock)
PILLAR_FRAME_SCREW_RHO = 87.0             # dedicated frame->pillar M3
PILLAR_BOT_Z = hp.CHASSIS_PLATE_T / 2.0   # +2.0 -- bottom sheet top face

# The pillar stands where the production WAGO TRAY sat at each corner
# flat.  With the top frame installed those corner Wagos are buried
# under solid deck (no lever access), so in this variant the corner +
# trunk splices CONSOLIDATE into the central block (see WBLK_* below)
# and the six integrated tray WALL SETS ARE DELETED from the chassis
# print (user, Aug 24: "they dont make any sense anymore") -- see the
# tray-cut block in make_chassis_bottom_rigid.  The pillar foot keeps
# the exact bay-sized footprint (all hole positions unchanged) but now
# registers on its THREE M3 through-bolts instead of the old 0.3 mm
# wall key.  The surrounding corner is otherwise
# claimed (probed against the real solid): the leg cradle's diagonal
# well wall at y ~ +/-19..26 and the retainer's corner pads (z to 9.25)
# forbid any foot wings outside the old bay footprint, so both bar
# bolts sit INSIDE it, plus a small INBOARD tab whose bolt lands under
# the open hatch (driver comes straight down, even with the frame on).
# Nyloc nuts go on the belly (-6 face, verified open at all three
# spots).  On a STOCK chassis print the tray walls still exist; the
# foot was sized to fit them with 0.3 mm clearance, so it still drops
# straight in (the walls just become a bonus shear key there).
_WAGO_BAY_W = hp.WAGO5_W + hp.WAGO_MOUNT_BAY_CLEAR    # 29.85 tangential
PILLAR_KEY_CL = 0.3                       # foot vs (stock-print) wall, per side
_BAY_OUT_X = hp.WAGO_MOUNT_EDGE_R - hp.WAGO_MOUNT_WALL_T   # 97.6 old outer wall
PILLAR_BAR_HOLE_X = 93.0                  # in-bay bolt pair, radial pos
PILLAR_BAR_HOLE_Y = 11.0                  # in-bay bolt pair, tangential +/-
PILLAR_TAB_RHO = 67.6                     # inboard tab bolt radius
PILLAR_FOOT_T = 4.0                       # foot plate thickness
# Tray wall-set envelope (for the delete cut + the gone-check): the
# production tray is 2 side walls + 1 outer wall, 2.4 thick, 6.5 tall,
# on the plate top face at each corner flat (az 0/60/...).
TRAY_HALF_X = (hp.WAGO5_D + hp.WAGO_MOUNT_BAY_CLEAR) / 2.0 \
    + hp.WAGO_MOUNT_WALL_T                # 11.625 radial half-extent
TRAY_HALF_Y = _WAGO_BAY_W / 2.0 + hp.WAGO_MOUNT_WALL_T   # 17.325 tangential

# CENTRAL SPLICE BLOCK (user, Aug 24: consolidate).  With the corner
# trays claimed by the pillars, the power tree collapses into ONE
# printed block at the chassis centre: 4x 5-port 221-415 in two
# back-to-back press-fit rows (north pair = V+, south pair = GND; the
# two nuts of a net are jumpered, leaving battery-in + 6 leg branches
# + a spare port per net).  It replaces the 6 corner nuts AND the two
# trunk nuts: 8 -> 4.  Bays reuse the bench-tuned production tray
# constants (0.15 mm wedge, 2.4 mm walls, walls stop ~1.9 mm below the
# nut top).  The floor sits on probed-solid sheet between the battery
# strap slots (inner edge y = 24), west of the battery-lead trunk pass
# (x 41..55) whose leads enter the east ports directly; the whole
# block is under the open hatch, levers up.  Mounted with a VHB pad --
# the exact scheme the production corner trays used for months before
# they merged into the chassis print (and here the wire pull is a
# gentle radial fan, not one corner's inward yank).
WBLK_BAY_W = hp.WAGO5_W + hp.WAGO_MOUNT_BAY_CLEAR   # 29.85 (block X)
WBLK_BAY_D = hp.WAGO5_D + hp.WAGO_MOUNT_BAY_CLEAR   # 18.45 (block Y)
WBLK_WALL_T = hp.WAGO_MOUNT_WALL_T                  # 2.4
WBLK_WALL_H = hp.WAGO_MOUNT_WALL_H                  # 6.5 above the floor
WBLK_FLOOR_T = 2.0
WBLK_HALF_X = WBLK_BAY_W + 1.5 * WBLK_WALL_T        # 33.45
WBLK_HALF_Y = WBLK_WALL_T / 2.0 + WBLK_BAY_D + WBLK_WALL_T   # 22.05

# Cap-local (well-frame) geometry.  Well frame: origin = hip servo back-face
# centre, +X = body long axis, +Y = out of the open face (world UP at the
# hip), +Z = output axis.  The yaw axis pierces the cap at (x=0, z=AXIS_Z).
CAP_FACE_Y = hp.WELL_D / 2.0 + hp.CLAMP_CAP_T             # 21.9 outer face
AXIS_Z = hp.JOINT_HORN_TOP_Z + hp.COXA_HIP_ANCHOR_Y      # 15.65
PED_Y0 = CAP_FACE_Y - 1.0                                 # 1 mm weld bite
PED_Y1 = CAP_FACE_Y + PED_H                               # race seat plane
BOSS_Y1 = PED_Y1 + BEARING_W - BOSS_TIP_H                 # press band top
TIP_Y1 = PED_Y1 + BEARING_W                               # lead-in tip top

# World-frame stack (z up, chassis_bottom sheet mid-plane at z = 0).
# Uses the variant's SHORTENED hip drop: the whole top stack rides
# COL_DROP lower than the production-coxa numbers.
CAP_FACE_W = hp.CHASSIS_YAW_OUTPUT_Z + COXA_HIP_DROP_V + CAP_FACE_Y  # 66.55
BR_BOT_W = CAP_FACE_W + PED_H             # 72.05 -- race bottom / seat
BR_TOP_W = BR_BOT_W + BEARING_W           # 79.05 -- race top / shoulder
RING_BOT_W = BR_BOT_W + RING_BOT_CL       # 72.55 -- ring bottom face
SHEET_Z0 = BR_TOP_W                       # sheet bottom = race top plane
SHEET_Z1 = SHEET_Z0 + PLATE_T             # 83.05 -- deck face

APOTHEM = hp.CHASSIS_FLAT_TO_FLAT / 2.0   # 100 -- yaw axes sit ON this line

XZ = ((1.0, 0.0, 0.0), hp.LEG_PITCH_AXIS)
M_HIP_JP = hp._joint_place(COXA_HIP_ANCHOR_V, *XZ)      # well -> coxa (hip)
M_KNEE_JP = hp._joint_place((hp.FEMUR_LENGTH, 0.0, 0.0), *XZ)
MH = hp._joint_place((0.0, 0.0, 0.0), *XZ)              # joint -> link local


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


def _rotz(rad: float) -> np.ndarray:
    return rotation_matrix(rad, [0, 0, 1])


def _cyl_z(r: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0,
           sections: int = 128) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=sections)
    c.apply_translation([x, y, 0.5 * (z0 + z1)])
    return c


def _cyl_y(r: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0,
           sections: int = 128) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=sections)
    c.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))  # axis -> +Y
    c.apply_translation([x, 0.5 * (y0 + y1), z])
    return c


def _box(extents, center) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=extents)
    b.apply_translation(list(center))
    return b


def _union(meshes):
    return trimesh.boolean.union(meshes, engine="manifold")


def _diff(base, cutters):
    return trimesh.boolean.difference([base, *cutters], engine="manifold")


def _inter_vol(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """Boolean intersection volume with a cheap AABB prefilter."""
    if (a.bounds[1] < b.bounds[0]).any() or (b.bounds[1] < a.bounds[0]).any():
        return 0.0
    m = trimesh.boolean.intersection([a, b], engine="manifold")
    return 0.0 if m.is_empty else float(m.volume)


# ---------------------------------------------------------------------------
# New printed parts
# ---------------------------------------------------------------------------

def _hex_prism(apothem: float, z0: float, z1: float,
               flats_at_rings: bool = False) -> trimesh.Trimesh:
    """Hexagonal prism.  Default = the PLATE's as-built orientation
    (measured: VERTICES at the ring azimuths 30/90/... deg).  With
    ``flats_at_rings`` the hex is rotated 30 deg so its FLATS face the
    rings -- the hatch orientation, keeping the web at every ring wide
    and reaching toward the inter-leg directions instead."""
    p = _cyl_z(apothem / np.cos(np.pi / 6.0), z0, z1, sections=6)
    if not flats_at_rings:
        p.apply_transform(rotation_matrix(np.pi / 6.0, [0, 0, 1]))
    return p


def _hatch_screw_xy() -> list[tuple[float, float]]:
    return [(HATCH_SCREW_RHO * np.cos(np.deg2rad(az)),
             HATCH_SCREW_RHO * np.sin(np.deg2rad(az)))
            for az in range(0, 360, 60)]


def _access_hole_xy() -> list[tuple[float, float]]:
    """World XY of each leg's INBOARD hip-cap clamp bolt at yaw 0.

    ``hp.servo_clamp_bolt_centres()`` is the single source of truth for
    the (x, z) bolt pattern in the well frame (bolt axis = well +/-Y =
    world vertical at the hip).  One bolt of each pair lands outboard of
    the hex edge (open sky, always reachable); the other hides under the
    plate -- THAT one gets a driver pass-through hole so the cap can be
    unbolted with the plate installed."""
    out = []
    for i in range(6):
        Tc = leg_transforms(i)["hip_cap"]
        for (bx, bz) in hp.servo_clamp_bolt_centres():
            w = Tc @ np.array([bx, 0.0, bz, 1.0])
            if np.hypot(w[0], w[1]) < APOTHEM:      # under the hex sheet
                out.append((float(w[0]), float(w[1])))
    assert len(out) == 6, f"expected 6 under-plate cap bolts, got {len(out)}"
    return out


def make_hip_cap_rigid() -> trimesh.Trimesh:
    """Stock hip clamp cap + yaw-axis pedestal + inner-race press boss.

    The pedestal/boss stack is coaxial with the YAW axis (cap-local
    x = 0, z = AXIS_Z), which lands on solid flange-bar material: the
    flange spans x in +/-31.7, z in [0, 34.7] at the outer face, and the
    Phi 29 pedestal footprint (x +/-14.5, z 1.15..30.15) stays >9 mm
    clear of the two M3 counterbores at (+/-27.2, 17.15)."""
    cap = hp.make_servo_clamp_cap()
    ped = _cyl_y(PED_OD / 2.0, PED_Y0, PED_Y1, x=0.0, z=AXIS_Z)
    boss = _cyl_y(BOSS_OD / 2.0, PED_Y1 - 1.0, BOSS_Y1, x=0.0, z=AXIS_Z)
    tip = _cyl_y(BOSS_OD / 2.0 - BOSS_TIP_STEP, BOSS_Y1 - 0.1, TIP_Y1,
                 x=0.0, z=AXIS_Z)
    body = _union([cap, ped, boss, tip])
    # Two puller notches at +/-x: pry slots exposing the inner race's
    # underside (r 13.0..14.5) so the pressed 6805 can be walked off the
    # boss with a flat screwdriver / 2-jaw puller instead of being a
    # one-way assembly.  The notch floor stays clear of the boss root.
    notches = []
    for sx in (+1.0, -1.0):
        n = trimesh.creation.box(extents=(PED_OD / 2.0 - PULLER_NOTCH_R0 + 3.0,
                                          PULLER_NOTCH_DEPTH + 0.05,
                                          PULLER_NOTCH_W))
        n.apply_translation([
            sx * (PULLER_NOTCH_R0 + n.extents[0] / 2.0),
            PED_Y1 - PULLER_NOTCH_DEPTH / 2.0 + 0.025,
            AXIS_Z])
        notches.append(n)
    return _diff(body, notches)


def make_bearing_6805() -> trimesh.Trimesh:
    """Visual 6805-2RS stand-in in the CAP frame, seated on the pedestal.

    NOT PRINTED.  Bore modeled at BOSS_OD (25.15) so the seated scene has
    zero mesh overlap; the real bearing bore is Phi 25.00 and the 0.15
    diametral interference is the press."""
    outer = _cyl_y(BEARING_OD / 2.0, PED_Y1, PED_Y1 + BEARING_W,
                   x=0.0, z=AXIS_Z)
    bore = _cyl_y(BOSS_OD / 2.0, PED_Y1 - 1.0, PED_Y1 + BEARING_W + 1.0,
                  x=0.0, z=AXIS_Z)
    return _diff(outer, [bore])


def make_chassis_top_rigid() -> trimesh.Trimesh:
    """200 mm flat-to-flat hex sheet + six bearing-pocket bosses.

    Same footprint and sheet thickness as chassis_bottom.  Each leg gets
    a Phi 44 boss (same Phi 37 + 2x3.5 wall as the bottom tower) hanging
    below the sheet, pocketing the third bearing's outer race from BELOW:
    press up until the race top hits the Phi 34 shoulder.  Carries the
    4-hole standoff pattern, the electronics-deck pattern, and a Phi 40
    centre access hole."""
    sheet = _hex_prism(APOTHEM, SHEET_Z0, SHEET_Z1)

    solids = [sheet]
    cuts = []
    for _i, edge, _R, _R3 in hp._leg_chassis_frames():
        x, y = float(edge[0]), float(edge[1])
        # Ring runs to the DECK face: outboard of the hex edge there is no
        # sheet, so a full-height ring is what completes the Phi 34 race
        # shoulder all the way around (flush with the deck, no proud lip).
        solids.append(_cyl_z(RING_OD / 2.0, RING_BOT_W, SHEET_Z1,
                             x=x, y=y))
        cuts.append(_cyl_z(POCKET_BORE / 2.0, RING_BOT_W - 1.0, SHEET_Z0,
                           x=x, y=y))
        cuts.append(_cyl_z(POCKET_BORE / 2.0 + POCKET_LEADIN,
                           RING_BOT_W - 1.0, RING_BOT_W + POCKET_LEADIN,
                           x=x, y=y))
        cuts.append(_cyl_z(SHOULDER_OD / 2.0, SHEET_Z0 - 0.01, SHEET_Z1 + 1.0,
                           x=x, y=y))
    # (The old sub-sheet lid-screw bosses are gone: the corner PILLARS
    # sit under these positions now and receive the screws instead.)
    # Driver pass-throughs above the inboard cap bolts (legs at yaw 0):
    # a hex driver reaches the cap screws with the plate ON, so service
    # unbolts the cap+bearing unit instead of separating any press fit.
    for (x, y) in _access_hole_xy():
        cuts.append(_cyl_z(ACCESS_HOLE_D / 2.0, SHEET_Z0 - 1.0, SHEET_Z1 + 1.0,
                           x=x, y=y, sections=64))
    # SERVICE-HATCH opening: the standoff, electronics-deck and centre
    # holes all fell inside it -- they move onto the removable hatch.
    cuts.append(_hex_prism(HATCH_OPEN_APO, SHEET_Z0 - 1.0, SHEET_Z1 + 1.0,
                           flats_at_rings=True))
    # Lid screws pass THROUGH the frame into the pillar tops (M3
    # clearance), and each pillar gets one dedicated frame screw so the
    # frame stays clamped down with the lid removed.
    for (x, y) in _hatch_screw_xy():
        cuts.append(_cyl_z(HOLE_D / 2.0, SHEET_Z0 - 1.0, SHEET_Z1 + 1.0,
                           x=x, y=y, sections=32))
    for az in range(0, 360, 60):
        cuts.append(_cyl_z(HOLE_D / 2.0, SHEET_Z0 - 1.0, SHEET_Z1 + 1.0,
                           x=PILLAR_FRAME_SCREW_RHO * np.cos(np.deg2rad(az)),
                           y=PILLAR_FRAME_SCREW_RHO * np.sin(np.deg2rad(az)),
                           sections=32))
    return _diff(_union(solids), cuts)


def make_corner_pillar() -> trimesh.Trimesh:
    """One rim pillar, modeled in WORLD position at az 0 (instances are
    placed by 60-deg rotations).  A PLAIN SOLID elliptical column from
    the bottom sheet's top face up to PILLAR_TOP_GAP below the frame
    (leg clearance is guaranteed by the rotating parts' ROT_ENVELOPE_R
    trim, not by shaping the column), with:

      * two Phi 2.5 self-tap pilots in the top face (insert-ready):
        the shared lid screw (rho 76.2) and the dedicated frame screw
        (rho 87);
      * a bottom FOOT plate sized to the old corner Wago tray bay
        (the variant chassis DELETES the tray walls, so the foot
        registers on its bolts; on a stock chassis print the
        surviving walls still fit it with 0.3 mm clearance) carrying
        two Phi 3.4 through-holes, plus a small inboard TAB whose
        bolt lands under the open hatch.  All three are M3
        through-bolts with nyloc nuts on the belly; the matching holes
        are printed into chassis_bottom_rigid (on a STOCK chassis
        print, drill them using the foot as the jig).
    """
    top_z = SHEET_Z0 - PILLAR_TOP_GAP

    def _ecyl(r_rad: float, z0: float, z1: float) -> trimesh.Trimesh:
        c = trimesh.creation.cylinder(radius=r_rad, height=z1 - z0,
                                      sections=64)
        c.apply_transform(np.diag([1.0, PILLAR_TAN_SCALE, 1.0, 1.0]))
        c.apply_translation([PILLAR_RHO, 0.0, (z0 + z1) / 2.0])
        return c

    col = _ecyl(PILLAR_OD / 2.0, PILLAR_BOT_Z, top_z)
    bay_x0 = PILLAR_RHO + 2.0                       # 83.6 -- inside the bay
    bay_x1 = _BAY_OUT_X - PILLAR_KEY_CL             # 97.3
    bay_half = _WAGO_BAY_W / 2.0 - PILLAR_KEY_CL    # 14.63
    bar = trimesh.creation.box(
        extents=(bay_x1 - bay_x0, 2.0 * bay_half, PILLAR_FOOT_T))
    bar.apply_translation([(bay_x0 + bay_x1) / 2.0, 0.0,
                           PILLAR_BOT_Z + PILLAR_FOOT_T / 2.0])
    tab = trimesh.creation.box(extents=(12.0, 12.0, PILLAR_FOOT_T))
    tab.apply_translation([PILLAR_RHO - PILLAR_OD / 2.0 - 2.0, 0.0,
                           PILLAR_BOT_Z + PILLAR_FOOT_T / 2.0])
    body = _union([col, bar, tab])
    cuts = [
        # top pilots: shared lid screw + dedicated frame screw
        _cyl_z(PILOT_OD / 2.0, top_z - 8.0, top_z + 1.0,
               x=HATCH_SCREW_RHO, y=0.0, sections=32),
        _cyl_z(PILOT_OD / 2.0, top_z - 8.0, top_z + 1.0,
               x=PILLAR_FRAME_SCREW_RHO, y=0.0, sections=32),
        # inboard tab bolt (under the open hatch)
        _cyl_z(HOLE_D / 2.0, PILLAR_BOT_Z - 1.0,
               PILLAR_BOT_Z + PILLAR_FOOT_T + 1.0, x=PILLAR_TAB_RHO, y=0.0,
               sections=32),
    ]
    for sy in (+1.0, -1.0):
        cuts.append(_cyl_z(HOLE_D / 2.0, PILLAR_BOT_Z - 1.0,
                           PILLAR_BOT_Z + PILLAR_FOOT_T + 1.0,
                           x=PILLAR_BAR_HOLE_X,
                           y=sy * PILLAR_BAR_HOLE_Y, sections=32))
    return _diff(body, cuts)


def make_coxa_link_rigid() -> trimesh.Trimesh:
    """The production coxa with the four variant edits (see the
    constant blocks above):

      * SHORTENED COLUMN (user, Aug 24 rev 3 + rev 4): instead of
        starting from the merged production print, the coxa is
        re-assembled from its two production sub-solids with the
        vertical filler removed -- the hub is truncated at HUB_TRIM_Z
        (deleting the Phi 52.4 platform disc, the dust-lip skirt and
        the uflange, all of which served the deleted production
        cap/upper race), the horn screws swap M3x30 -> M3x25 with
        every seat plane 5 mm deeper (HORN_HEAD_SEAT_Z /
        HORN_CENTRE_SEAT_Z: identical tip planes and thread
        engagement, see the HORN SCREWS constants), and the whole
        slab + cradle unit drops COL_DROP so the well floor lands
        COL_HEAD_CL above the screw heads.  The horn interface AT the
        horn (bolt pattern, tip depths) does NOT move.  Hip axis:
        COXA_HIP_ANCHOR_V.
      * ENVELOPE ROUND: servo-cradle corners rounded to the
        ROT_ENVELOPE_R arc about its own yaw axis (max 2.16 mm off two
        vertical wall corners that used to reach 40.36 mm) so the plain
        rim columns clear by >= 5 mm at every yaw angle.
      * HUB SEAT RING: the Phi 29 ring runs from the race top (z 7.5,
        tower pocket, LOWER position) up into the dropped slab.  It
        bears only on the Phi 25..29 inner-race land -- same contact
        the production uflange made one race higher -- and merges
        with the boss wall over Phi 24..25.15.
      * DUST BRIM: a Phi 38 brim (z 8..10) roofs the seal + outer-race
        band, hovering 0.5 above the race-top / raised-rim plane and
        stopping 3 mm inside the tower Phi 44.  The old Phi 44
        skirt+curtain is deleted (it read as more chassis column) --
        see the TOWER RIM / BRIM_* constants.

    Cradle pilots, cap seat, 688 housing and the horn drive pattern
    are untouched (the cradle unit translates rigidly).  This makes
    the coxa a VARIANT print (6x)."""
    # Hub: production sub-solid, truncated at the dropped slab.  The
    # explicit annular cut kills the dust-lip skirt (z 14..18, r 23+)
    # so the halfspace trim cannot leave a floating skirt sliver.
    hub = hp.make_coxa_yaw_hub(one_piece=True)
    skirt_cut = _diff(
        _cyl_z(hp.YAW_HUB_DUST_LIP_OD / 2.0 + 2.0, SLAB_BOT_Z - 1.5,
               hp.YAW_HUB_BOSS_TOP_Z + 1.0, sections=64),
        [_cyl_z(20.0, SLAB_BOT_Z - 2.5, hp.YAW_HUB_BOSS_TOP_Z + 2.0,
                sections=64)])
    hub = _diff(hub, [skirt_cut])
    keep_lo = _cyl_z(60.0, hub.bounds[0][2] - 1.0, HUB_TRIM_Z, sections=32)
    hub = trimesh.boolean.intersection([hub, keep_lo], engine="manifold")
    # Slab + cradle: production sub-solid dropped as ONE rigid body
    # (well floor, cradle pilots, cap seat, rear tab, chamfer).
    bracket = hp.make_coxa_hip_bracket(one_piece=True)
    bracket.apply_translation([0.0, 0.0, -COL_DROP])
    ring = _diff(
        _cyl_z(HUB_RING_OD / 2.0, HUB_RING_Z0, HUB_RING_Z1, sections=128),
        [_cyl_z(HUB_RING_ID / 2.0, HUB_RING_Z0 - 1.0, HUB_RING_Z1 + 1.0,
                sections=128)])
    brim = _diff(
        _cyl_z(BRIM_OD / 2.0, BRIM_BOT_Z, BRIM_TOP_Z, sections=192),
        [_cyl_z(HUB_RING_ID / 2.0, BRIM_BOT_Z - 1.0, BRIM_TOP_Z + 1.0,
                sections=128)])
    body = _union([hub, bracket, ring, brim])
    # Yoke-end sweep clearance (mirror of hp.make_coxa_link_part, at
    # the DROPPED hip axis): the femur yoke arm ends are full r~16
    # discs about the hip axis; carve their swept cylinders through
    # the two arm bands.
    hip_ax_x, _, hip_ax_z = COXA_HIP_ANCHOR_V
    sweeps = [_cyl_y(16.75, ylo, yhi, x=hip_ax_x, z=hip_ax_z)
              for (ylo, yhi) in ((21.75, 30.0), (-31.0, -24.75))]
    # Horn-screw plumbing, re-cut through the dropped slab (mirror of
    # hp.make_coxa_link_part, seats at the VARIANT planes -- 5 mm
    # deeper than production, tracking the M3x30 -> M3x25 swap so the
    # tips land on the production planes): the Phi 5.9 head-access
    # shafts run from each seat plane up through the well floor;
    # below each seat the shank clearance (Phi 3.7 drive bolts,
    # Phi 3.4 centre spline screw) is re-opened through the slab band
    # the drop slid over it.  The 0.5 mm shank overshoot past the seat
    # is inside the Phi 5.9 shaft, so the seat annulus is untouched.
    shaft_top_z = 80.0
    drive_clear = hp.DISC_HORN_BOLT_OD + 0.3
    stations = [(0.0, 0.0, HORN_CENTRE_SEAT_Z,
                 hp.HORN_CENTRE_OD)]
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations += [(r * np.cos(t), r * np.sin(t),
                  HORN_HEAD_SEAT_Z, drive_clear)
                 for t in hp.DISC_HORN_BOLT_ANGLES_RAD]
    cuts = list(sweeps)
    for (sx, sy, seat_z, shank_d) in stations:
        cuts.append(_cyl_z(hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0, seat_z,
                           shaft_top_z, x=sx, y=sy, sections=48))
        cuts.append(_cyl_z(shank_d / 2.0, SLAB_BOT_Z - 1.0, seat_z + 0.5,
                           x=sx, y=sy, sections=32))
    body = _diff(body, cuts)
    z0, z1 = body.bounds[0][2] - 1.0, body.bounds[1][2] + 1.0
    keep = _cyl_z(ROT_ENVELOPE_R, z0, z1, sections=256)
    return trimesh.boolean.intersection([body, keep], engine="manifold")


def make_chassis_bottom_rigid() -> trimesh.Trimesh:
    """The production chassis_bottom with the CHB_* variant edits (see
    the constant block above):

      * PLATFORM CORNERS: everything outboard of the hex edge
        (leg-frame x > 100) between the belly (z -6.5) and the
        platform top (z 6.25) is trimmed to the tower's own cylinder
        (r 22.02 about the yaw axis).  The square platform and skirt
        corners disappear and the whole tower base reads as ONE
        cylinder from belly to bearing pocket -- no second corner
        radius anywhere.  The trim is 0.02 proud of the tower wall,
        so pocket/walls/rim keep production geometry.
      * RAISED RIM: each tower is extended 3 mm up (same Phi 44 /
        Phi 37.15 wall, unioned AFTER the cuts so the ear shaves
        never nick it) from the production rim (world 19.75) to the
        race-top plane (world 22.75) -- the single bearing is fully
        housed, full 7 mm outer-race wrap, and the chassis column
        ends exactly at the bearing top.
      * DEAD EARS: the outboard (az 330) cap-bolt ear is shaved flush
        to the tower cylinder, the tangential (az 90) ear flush to the
        rim-wall face, and the inboard (az 210) ear flush to the
        servo-mount deck top (z 10.25) -- its below-deck root stays
        merged with the well-mouth collar.  Above the deck, nothing
        pokes past the tower cylinder at any azimuth.
      * FOOT HOLES: the 18 Phi 3.4 pillar-foot bolt holes are printed
        through the sheet (same PILLAR_* constants as the feet, so
        they line up by construction).
      * WAGO TRAYS DELETED (user, Aug 24): the corner power splices
        moved into centre_wago_block, so the six integrated tray wall
        sets are dead geometry -- every wall above the sheet top is
        cut away (the 1 mm embed band below the top face is interior
        material and stays).  The pillar feet register on their three
        M3 bolts instead of the old wall key.
      * WIRE CORRIDOR FLATTENED (user, Aug 24 rev 5): the cradle end
        wall + porch canopy + side-wall stubs inboard of each seated
        yaw servo are cut back to the bare sheet (one box per leg,
        leg-frame x CHB_FLAT_X0..X1, |y| <= CHB_FLAT_HALF_Y, sheet
        top to over the deck) -- see the CHB_FLAT_* constant block
        for what they were and why nothing needs them.  The deck
        plateau + well collar + flanking walls outboard of the cut
        still register the servo; the retainer pilots live below the
        sheet top, untouched.

    This makes chassis_bottom the variant's second reprinted
    production part (chassis_bottom_rigid.stl)."""
    cb = hp.make_chassis_bottom()
    ear_r = hp.YAW_CAP_BOLT_PCD / 2.0                        # 23.5
    cutters = []
    for i in range(6):
        R = _rotz((i + 0.5) * np.pi / 3.0)                   # leg frame
        leg_cuts = []
        box = _box((23.5, 45.0, CHB_PLATE_TOP + 6.5),        # (a) corner trim:
                   (APOTHEM + 23.5 / 2.0, 0.0,               # x 100..123.5,
                    (CHB_PLATE_TOP - 6.5) / 2.0))            # z -6.5..6.25
        leg_cuts.append(_diff(box, [_cyl_z(CHB_TRIM_R, -8.0, 7.5,
                                           x=APOTHEM, y=0.0, sections=192)]))
        keep = _cyl_z(CHB_KEEP_R, CHB_PLATE_TOP - 1.0, 21.6,
                      x=APOTHEM, y=0.0, sections=128)
        ear330 = _cyl_z(CHB_EAR_R, CHB_PLATE_TOP, 20.6,      # (b) outboard
                        x=APOTHEM + ear_r * np.cos(-np.pi / 6.0),
                        y=ear_r * np.sin(-np.pi / 6.0), sections=48)
        leg_cuts.append(_diff(ear330, [keep.copy()]))
        box90 = _box((12.0, 10.0, 20.6 - CHB_PLATE_TOP),     # (c) tangential
                     (APOTHEM, CHB_WALL_FACE_Y + 5.0,
                      (CHB_PLATE_TOP + 20.6) / 2.0))
        leg_cuts.append(_diff(box90, [keep.copy()]))
        ear210 = _cyl_z(CHB_EAR_R, CHB_DECK_TOP, 20.6,       # (b2) inboard,
                        x=APOTHEM + ear_r * np.cos(np.pi * 7.0 / 6.0),
                        y=ear_r * np.sin(np.pi * 7.0 / 6.0),  # flush to the
                        sections=48)                          # deck top only
        leg_cuts.append(_diff(ear210, [keep]))
        leg_cuts.append(_box(                                # (g) corridor
            (CHB_FLAT_X1 - CHB_FLAT_X0,                      # flatten (rev 5)
             2.0 * CHB_FLAT_HALF_Y, CHB_FLAT_Z1 - CHB_FLAT_Z0),
            ((CHB_FLAT_X0 + CHB_FLAT_X1) / 2.0, 0.0,
             (CHB_FLAT_Z0 + CHB_FLAT_Z1) / 2.0)))
        for c in leg_cuts:
            c.apply_transform(R)
            cutters.append(c)
    for az in range(0, 360, 60):                             # (d) foot holes
        Ra = _rotz(np.deg2rad(az))
        for hx, hy in ((PILLAR_BAR_HOLE_X, +PILLAR_BAR_HOLE_Y),
                       (PILLAR_BAR_HOLE_X, -PILLAR_BAR_HOLE_Y),
                       (PILLAR_TAB_RHO, 0.0)):
            h = _cyl_z(HOLE_D / 2.0, -12.0, 12.0, x=hx, y=hy, sections=32)
            h.apply_transform(Ra)
            cutters.append(h)
    for M in hp.wago_tray_corner_transforms():               # (e) tray delete
        c = _box((2.0 * TRAY_HALF_X + 0.4, 2.0 * TRAY_HALF_Y + 0.4,
                  hp.WAGO_MOUNT_WALL_H + 1.0),
                 center=(0.0, 0.0, (hp.WAGO_MOUNT_WALL_H + 1.0) / 2.0))
        c.apply_transform(M)                                 # local z0 = sheet top
        cutters.append(c)
    cb = _diff(cb, cutters)
    rims = []                                                # (f) raised rims
    for i in range(6):
        R = _rotz((i + 0.5) * np.pi / 3.0)
        rim = _diff(
            _cyl_z(CHB_TOWER_R, CHB_RIM_OLD_W - 1.0, CHB_RIM_W,
                   x=APOTHEM, y=0.0, sections=192),
            [_cyl_z(POCKET_BORE / 2.0, CHB_RIM_OLD_W - 2.0, CHB_RIM_W + 1.0,
                    x=APOTHEM, y=0.0, sections=192)])
        rim.apply_transform(R)
        rims.append(rim)
    return _union([cb, *rims])


def _pillar_meshes(meshes: dict) -> list[trimesh.Trimesh]:
    """The six placed pillar copies (az 0/60/.../300)."""
    out = []
    for az in range(0, 360, 60):
        m = meshes["corner_pillar"].copy()
        m.apply_transform(_rotz(np.deg2rad(az)))
        out.append(m)
    return out


def make_centre_wago_block() -> trimesh.Trimesh:
    """Central power-splice block: 4x 5-port 221-415 bays in two
    back-to-back rows sharing the middle wall, production press-fit
    dims.  Wire entries face outward (north row +Y, south row -Y).
    Modeled in world position: centred on the chassis origin, floor on
    the bottom sheet's top face.  See the WBLK_* constant block."""
    z0 = PILLAR_BOT_Z
    zf = z0 + WBLK_FLOOR_T
    top = zf + WBLK_WALL_H
    # ONE solid minus four bay pockets: cutting (instead of unioning
    # wall boxes) leaves no exactly-flush coplanar seams to T-junction.
    body = _box((2 * WBLK_HALF_X, 2 * WBLK_HALF_Y, top - z0),
                center=(0.0, 0.0, (z0 + top) / 2.0))
    x_c = WBLK_WALL_T / 2.0 + WBLK_BAY_W / 2.0
    cuts = []
    for sy in (+1.0, -1.0):
        for sx in (-1.0, +1.0):
            # pocket runs from the middle wall face out PAST the front
            # edge, so each bay is open at its wire-entry side
            y0 = WBLK_WALL_T / 2.0
            y1 = WBLK_HALF_Y + 2.0
            cuts.append(_box(
                (WBLK_BAY_W, y1 - y0, WBLK_WALL_H + 2.0),
                center=(sx * x_c, sy * (y0 + y1) / 2.0,
                        zf + (WBLK_WALL_H + 2.0) / 2.0)))
    return _diff(body, cuts)


def _wago5_scene_frames() -> list[np.ndarray]:
    """World 4x4 for the four seated splice nuts (visuals).  The wago5
    visual's local -X is the wire-entry face; rotate it outward."""
    y_c = WBLK_WALL_T / 2.0 + WBLK_BAY_D / 2.0     # 10.425
    zf = PILLAR_BOT_Z + WBLK_FLOOR_T
    x_c = WBLK_WALL_T / 2.0 + WBLK_BAY_W / 2.0     # 16.125
    out = []
    for sy in (+1.0, -1.0):                         # north row, south row
        for sx in (-1.0, +1.0):
            out.append(_trans([sx * x_c, sy * y_c, zf])
                       @ _rotz(-sy * np.pi / 2.0))
    return out


def make_top_hatch_rigid() -> trimesh.Trimesh:
    """Removable service hatch: a 4 mm hex lid over the frame opening.

    Sits ON the deck face (4 mm overlap all around), registered by a
    1.5 mm lip that drops just inside the opening, held by 6x M3 into
    the frame's pilot bosses (vertex azimuths) AND the 4 chassis
    standoff screws -- the standoff stacks grow ~4 mm so the standoffs
    anchor the hatch, and chassis-hang loads run standoffs -> hatch ->
    deck face -> frame in pure compression (screws only see rebound).
    Carries the electronics-deck pattern and the Phi 40 centre hole, so
    10 screws lift the lid + electronics out for full interior access."""
    lid = _hex_prism(HATCH_APO, SHEET_Z1, SHEET_Z1 + PLATE_T,
                     flats_at_rings=True)
    lip = _diff(
        _hex_prism(HATCH_OPEN_APO - HATCH_LIP_CL,
                   SHEET_Z1 - HATCH_LIP_H, SHEET_Z1 + 0.1,
                   flats_at_rings=True),
        [_hex_prism(HATCH_OPEN_APO - HATCH_LIP_CL - HATCH_LIP_W,
                    SHEET_Z1 - HATCH_LIP_H - 1.0, SHEET_Z1 + 1.0,
                    flats_at_rings=True)])
    ears = [_cyl_z(HATCH_EAR_OD / 2.0, SHEET_Z1, SHEET_Z1 + PLATE_T,
                   x=x, y=y, sections=48) for (x, y) in _hatch_screw_xy()]
    body = _union([lid, lip, *ears])
    cuts = []
    for (x, y) in list(_hatch_screw_xy()) \
            + [(float(x), float(y)) for (x, y) in hp.CHASSIS_STANDOFF_HOLES_XY] \
            + [(float(x), float(y)) for (x, y) in hp.ELEC_CHASSIS_MOUNT_HOLES_XY]:
        cuts.append(_cyl_z(HOLE_D / 2.0, SHEET_Z1 - 1.0,
                           SHEET_Z1 + PLATE_T + 1.0, x=x, y=y, sections=48))
    cuts.append(_cyl_z(CENTRE_HOLE_D / 2.0, SHEET_Z1 - 1.0,
                       SHEET_Z1 + PLATE_T + 1.0))
    return _diff(body, cuts)


# ---------------------------------------------------------------------------
# Assembly frames (mirrors tools/full_robot_viz_build.py::_servo_M0s /
# _leg0_local_link_parts, with parametric yaw + pitch for the sweeps)
# ---------------------------------------------------------------------------

def leg_transforms(i: int, yaw_deg: float = 0.0,
                   femur_deg: float = None,
                   tibia_deg: float = None) -> dict[str, np.ndarray]:
    femur_deg = hp.STANCE_FEMUR_DEG if femur_deg is None else femur_deg
    tibia_deg = hp.STANCE_TIBIA_DEG if tibia_deg is None else tibia_deg
    a = (i + 0.5) * np.pi / 3.0
    edge = np.array([APOTHEM * np.cos(a), APOTHEM * np.sin(a),
                     hp.CHASSIS_YAW_OUTPUT_Z])
    p = np.deg2rad(femur_deg)
    pt = np.deg2rad(femur_deg + tibia_deg)
    hip_local = np.array(COXA_HIP_ANCHOR_V)   # SHORTENED coxa: hip axis
    #                                           rides COL_DROP lower
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0])

    T_coxa = _trans(edge) @ _rotz(a) @ _rotz(np.deg2rad(yaw_deg))
    T_femur = T_coxa @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = T_coxa @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])
    # Chassis-mounted cradle frame: leg azimuth rotation at chassis z=0, NO
    # yaw and NO yaw-output lift. This is where the yaw_servo_retainer lives
    # (production places it at exactly this frame; parking it on T_coxa put
    # it CHASSIS_YAW_OUTPUT_Z = 15.25 mm too high, jammed into the platform).
    T_cradle = _trans([edge[0], edge[1], 0.0]) @ _rotz(a)
    return {"coxa": T_coxa, "femur": T_femur, "tibia": T_tibia,
            "cradle": T_cradle,
            "hip_cap": T_coxa @ M_HIP_JP,
            "knee_cap": T_femur @ M_KNEE_JP}


def _tibia_extras() -> tuple[trimesh.Trimesh, np.ndarray]:
    """(tube mesh in tibia-local frame, foot-boot local frame)."""
    ta = (MH @ np.array([hp._YOKE_SOCKET_X, 0.0, hp.JOINT_SOCKET_Z, 1.0]))[:3]
    tube_end = ta + np.array([hp.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    tube = hp._tube_between(ta, tube_end, hp.LEG_TUBE_OD / 2.0)
    foot_frame = hp._frame(tube_end, (1, 0, 0), (0, 0, 1))
    return tube, foot_frame


# ---------------------------------------------------------------------------
# Mesh registry (build once, cache to stl/; REBUILD=1 forces)
# ---------------------------------------------------------------------------
MESH_FILES = {
    # NEW printed parts (this variant's printed-parts directory) --
    # always rebuilt, they are what this script iterates on.
    "hip_clamp_cap_rigid": (make_hip_cap_rigid, "hip_clamp_cap_rigid.stl"),
    "chassis_top_rigid": (make_chassis_top_rigid, "chassis_top_rigid.stl"),
    "top_hatch_rigid": (make_top_hatch_rigid, "top_hatch_rigid.stl"),
    "corner_pillar": (make_corner_pillar, "corner_pillar.stl"),
    "centre_wago_block": (make_centre_wago_block, "centre_wago_block.stl"),
    # VARIANT reprints of production parts: the coxa gets its cradle
    # corners rounded to the 38.2 mm yaw envelope + a Phi 29 hub seat
    # ring down to the tower-seated bottom race (make_coxa_link_rigid);
    # the chassis gets its tower platforms trimmed to the tower's own
    # cylinder, the dead cap-bolt ears shaved and the pillar-foot
    # holes printed in (make_chassis_bottom_rigid).
    "coxa_link": (make_coxa_link_rigid, "coxa_link_rigid.stl"),
    "chassis_bottom": (make_chassis_bottom_rigid, "chassis_bottom_rigid.stl"),
    # Unchanged production prints (print from the MAIN stl_prototype/).
    "femur_link": (hp.make_femur_link_part, "femur_link.stl"),
    "tibia_knee_yoke": (hp.make_tibia_knee_yoke, "tibia_knee_yoke.stl"),
    "foot_boot": (hp.make_foot_boot, "foot_boot.stl"),
    "knee_clamp_cap": (hp.make_servo_clamp_cap, "knee_clamp_cap.stl"),
    # NOTE: the production yaw_bearing_cap is DELETED in this variant --
    # the ONE bottom bearing sits in the chassis tower's own pocket
    # (production LOWER-race position) and the top plate provides the
    # retention the cap used to (see the BOTTOM JOINT constant block).
    "yaw_servo_retainer": (hp.make_yaw_servo_retainer,
                           "yaw_servo_retainer.stl"),
    # COTS / visual only.  ONE bottom bearing per leg: the production
    # UPPER-race mesh (built at coxa-local z 7.5..14.5) is instanced
    # with a -7 z drop into the LOWER position, seated on the tower
    # pocket's z=0.5 shoulder.  Net robot bearing count stays 12
    # (6 bottom + 6 top-plate).
    "servo_body": (hp.make_servo_body, "servo_body_DO_NOT_PRINT.stl"),
    "yaw_bearing_upper": (hp.make_yaw_bearing_upper,
                          "yaw_bearing_upper_DO_NOT_PRINT.stl"),
    "bearing_6805": (make_bearing_6805, "bearing_6805_DO_NOT_PRINT.stl"),
    "wago5": (hp.make_wago5_visual, "wago5_DO_NOT_PRINT.stl"),
}
ALWAYS_REBUILD = {"hip_clamp_cap_rigid", "chassis_top_rigid",
                  "top_hatch_rigid", "corner_pillar", "centre_wago_block",
                  "coxa_link", "chassis_bottom", "bearing_6805"}


def build_meshes() -> dict[str, trimesh.Trimesh]:
    os.makedirs(STL_DIR, exist_ok=True)
    rebuild_all = os.environ.get("REBUILD", "") == "1"
    out: dict[str, trimesh.Trimesh] = {}
    for key, (factory, fname) in MESH_FILES.items():
        path = os.path.join(STL_DIR, fname)
        if key not in ALWAYS_REBUILD and not rebuild_all and os.path.exists(path):
            out[key] = trimesh.load(path)
            print(f"  {key:22s} loaded from cache")
            continue
        print(f"  {key:22s} building ...", flush=True)
        m = hp._heal_for_export(factory())   # same heal pass main() uses
        assert m.is_volume, f"{key}: not a volume even after heal"
        m.export(path)
        out[key] = m
    # tibia tube (per-leg primitive, cheap, not cached)
    tube, _ = _tibia_extras()
    tube.export(os.path.join(STL_DIR, "tibia_tube_DO_NOT_PRINT.stl"))
    out["tibia_tube"] = tube
    return out


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_static(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Seated-assembly sanity: new parts watertight, stack lands where the
    constants say, nothing but the boss/bearing enters the plate bore."""
    for key in ("hip_clamp_cap_rigid", "chassis_top_rigid",
                "top_hatch_rigid", "corner_pillar"):
        m = meshes[key]
        assert m.is_watertight, f"{key} not watertight"
        print(f"  {key:22s} watertight, vol {m.volume / 1000.0:.1f} cm3")

    T = leg_transforms(0)
    cap = meshes["hip_clamp_cap_rigid"].copy()
    cap.apply_transform(T["hip_cap"])
    assert abs(cap.bounds[1][2] - TIP_Y1 - (CAP_FACE_W - CAP_FACE_Y)) < 1e-3, \
        "placed cap top does not match the derived world stack"
    plate = meshes["chassis_top_rigid"]
    v = _inter_vol(cap, plate)
    assert v < 1e-6, f"rigid cap intersects top plate ({v:.2f} mm3)"

    br = meshes["bearing_6805"].copy()
    br.apply_transform(T["hip_cap"])
    v = _inter_vol(br, plate)
    assert v < 1e-6, f"bearing intersects top plate ({v:.2f} mm3)"
    print(f"  seated cap/bearing vs plate: no overlap "
          f"(race top z = {br.bounds[1][2]:.2f}, shoulder at {SHEET_Z0:.2f})")

    # --- driver access holes: web clearances + clear line of sight ------
    holes = _access_hole_xy()
    open_vertex_r = HATCH_OPEN_APO / np.cos(np.pi / 6.0)   # 73.90
    hatch_vertex_r = HATCH_APO / np.cos(np.pi / 6.0)       # 78.52
    for k, (hx, hy) in enumerate(holes):
        ax = leg_transforms(k)["coxa"][:2, 3]
        web = np.hypot(hx - ax[0], hy - ax[1]) - RING_OD / 2.0 \
            - ACCESS_HOLE_D / 2.0
        assert web >= 0.5, f"access hole L{k}: only {web:.2f} mm to the ring"
        rho = np.hypot(hx, hy)
        # the hole must stay clear of the hatch opening AND outboard of
        # the hatch lid's az-30 flat (extent = HATCH_APO there)
        assert rho - ACCESS_HOLE_D / 2.0 - HATCH_OPEN_APO >= 1.0, \
            f"access hole L{k} breaks into the hatch opening"
        assert rho - ACCESS_HOLE_D / 2.0 - HATCH_APO >= 1.0, \
            f"access hole L{k} covered by the hatch lid"
        for az in range(0, 360, 60):
            px = PILLAR_RHO * np.cos(np.deg2rad(az))
            py = PILLAR_RHO * np.sin(np.deg2rad(az))
            gap = np.hypot(hx - px, hy - py) \
                - (ACCESS_HOLE_D + PILLAR_OD) / 2.0
            assert gap >= 1.5, (
                f"access hole L{k} within {gap:.2f} mm of a corner pillar")
    # hatch screw geometry: hole inside the lid overlap band at the
    # opening's vertex direction
    assert HATCH_SCREW_RHO - HOLE_D / 2.0 - open_vertex_r >= 0.5, \
        "hatch screw hole breaks into the opening corner"
    assert hatch_vertex_r - HATCH_SCREW_RHO - HOLE_D / 2.0 >= 0.5, \
        "hatch screw hole falls off the lid corner"
    # A Phi 6.5 driver shaft dropped through the L0 hole down to the cap
    # face must touch nothing (leg at yaw 0) -- the screw head sits in the
    # cap flange counterbore right below.  The HATCH must not cover it.
    hx, hy = holes[0]
    shaft = _cyl_z(3.25, CAP_FACE_W + 0.05, SHEET_Z1 + 30.0, x=hx, y=hy,
                   sections=48)
    for key, frame in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                       ("hip_clamp_cap_rigid", "hip_cap"),
                       ("bearing_6805", "hip_cap")):
        m = meshes[key].copy()
        m.apply_transform(leg_transforms(0)[frame])
        v = _inter_vol(shaft, m)
        assert v < 1e-6, f"driver shaft fouls {key} ({v:.2f} mm3)"
    for key in ("chassis_top_rigid", "top_hatch_rigid"):
        v = _inter_vol(shaft, meshes[key])
        assert v < 1e-6, f"driver shaft fouls {key} ({v:.2f} mm3)"
    for j, p in enumerate(_pillar_meshes(meshes)):
        v = _inter_vol(shaft, p)
        assert v < 1e-6, f"driver shaft fouls pillar az{j * 60} ({v:.2f} mm3)"
    print(f"  cap-bolt driver access: 6 holes Phi {ACCESS_HOLE_D:g}, "
          f"ring web {web:.2f} mm, line of sight clear (hatch ON)")


def check_bottom_joint(meshes: dict[str, trimesh.Trimesh]) -> None:
    """ONE tower-seated bottom bearing, NO cap (user, Aug 24): the race
    sits in the production pocket on its z=0.5 seat with only the
    Phi 25.15 boss press touching the coxa, the raised tower rim wraps
    the FULL race width and stops exactly at the race-top plane (the
    Phi 44 column ends at the bearing top -- nothing continues it), the
    hub's new Phi 29 ring lands exactly on the race top, the Phi 38
    dust brim hovers its BRIM_GAP over rim and race without touching
    either, and with the plate off the leg + bearing lift straight out
    of the pocket (horn centre screw only)."""
    T = leg_transforms(0)
    br = meshes["yaw_bearing_upper"].copy()
    br.apply_transform(T["coxa"] @ _trans([0.0, 0.0, YAWBR_DROP]))
    seat_w = hp.CHASSIS_YAW_OUTPUT_Z + hp.YAW_BEARING_LOWER_BOT_Z
    assert abs(br.bounds[0][2] - seat_w) < 1e-3, \
        f"race bottom {br.bounds[0][2]:.2f} not on the tower seat {seat_w:.2f}"
    v_pocket = _inter_vol(br, meshes["chassis_bottom"])
    assert v_pocket < 1.0, f"race overlaps the tower pocket ({v_pocket:.2f} mm3)"

    coxa = meshes["coxa_link"].copy()
    coxa.apply_transform(T["coxa"])
    v_press = _inter_vol(coxa, br)
    assert v_press < 60.0, \
        f"coxa vs seated race: {v_press:.1f} mm3 (want the boss press only)"
    v = _inter_vol(coxa, meshes["chassis_bottom"])
    assert v < 1e-6, f"ringed coxa overlaps chassis_bottom ({v:.2f} mm3)"

    # the seat ring lands ON the race top (kiss): material just above it
    # on the inner-race land, none below it
    ax, ay = T["coxa"][:2, 3]
    race_top = seat_w + hp.YAW_BEARING_W
    r_mid = (HUB_RING_ID + HUB_RING_OD) / 4.0
    got = coxa.contains(np.array([[ax + r_mid, ay, race_top + 0.3],
                                  [ax + r_mid, ay, race_top - 0.3]]))
    assert got[0] and not got[1], "hub seat ring does not land on the race top"

    # full-wrap tower: the rim IS the race-top plane -- wall solid just
    # below it, open sky just above it (at the tower radius: neither
    # chassis nor coxa continues the Phi 44 column past the bearing),
    # and the brim constants keep their derivations
    assert abs(CHB_RIM_W - race_top) < 1e-9, "tower rim is not the race top"
    assert BRIM_BOT_Z - TOWER_RIM_Z == BRIM_GAP
    assert BRIM_OD / 2.0 - hp.YAW_BEARING_OD / 2.0 == BRIM_GAP
    assert BRIM_OD / 2.0 <= CHB_TOWER_R - 2.0, "brim reads as chassis column"
    r_wall = (POCKET_BORE / 2.0 + CHB_TOWER_R) / 2.0          # mid rim wall
    probes = np.array([
        [ax + r_wall, ay, CHB_RIM_W - 0.3],          # in the raised rim wall
        [ax + r_wall, ay, CHB_RIM_W + 0.3],          # air above the rim
        [ax + r_wall, ay, CHB_RIM_OLD_W + 0.3],      # in the raise band
    ])
    in_ch = meshes["chassis_bottom"].contains(probes)
    assert in_ch[0], "tower does not wrap the full race width"
    assert not in_ch[1], "tower rises past the race top"
    assert in_ch[2], "raise ring did not fuse onto the production rim"
    assert not coxa.contains(probes[1:2])[0], \
        "coxa continues the tower column above the rim (curtain back?)"
    # dust brim: present over the seal band, hovering BRIM_GAP over the
    # rim/race plane, gap open where brim and rim overlap radially
    r_brim = (HUB_RING_OD / 2.0 + BRIM_OD / 2.0) / 2.0        # mid brim
    r_gap = (POCKET_BORE / 2.0 + BRIM_OD / 2.0) / 2.0         # overlap band
    brim_pts = np.array([
        [ax + r_brim, ay,
         hp.CHASSIS_YAW_OUTPUT_Z + (BRIM_BOT_Z + BRIM_TOP_Z) / 2.0],
        [ax + r_gap, ay, CHB_RIM_W + BRIM_GAP / 2.0],
    ])
    assert coxa.contains(brim_pts[:1])[0], "dust brim missing"
    in_cx = coxa.contains(brim_pts[1:])[0]
    in_ch = meshes["chassis_bottom"].contains(brim_pts[1:])[0]
    assert not in_cx and not in_ch, "brim running gap is not open"

    # service: plate off -> leg + bearing lift straight out of the pocket
    for dz in (2.0, 5.0, 12.0, 30.0):
        for name, m in (("coxa", coxa), ("race", br)):
            mm = m.copy()
            mm.apply_translation([0.0, 0.0, dz])
            v = _inter_vol(mm, meshes["chassis_bottom"])
            assert v < 1e-6, \
                f"lift +{dz}: {name} fouls chassis_bottom ({v:.1f} mm3)"
    print(f"  bottom joint: race on the tower seat (world z {seat_w:.2f}), "
          f"NO cap; coxa/race contact = boss press ({v_press:.1f} mm3), "
          f"rim at the race top ({CHB_RIM_W:.2f}, full 7 mm wrap), "
          f"Phi {BRIM_OD:g} brim hovers {BRIM_GAP:g} above, "
          f"leg + bearing lift straight out")


def check_coxa_column(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The SHORTENED coxa (user, Aug 24 rev 3 + rev 4): the cradle
    floor sits exactly COL_HEAD_CL over the M3x25 horn-screw heads,
    the seats moved down by EXACTLY the M3x30 -> M3x25 length delta
    (so the tips -- and the thread engagement in the horn -- are the
    bench-tuned production planes), the screw shanks pass the dropped
    slab, the deleted skirt/platform left nothing below the slab
    outside the seat ring + brim, and the hip axis (and with it the
    seated servo) dropped by exactly COL_DROP."""
    coxa = meshes["coxa_link"]

    # constants: floor derivation and head clearance (guards a future
    # constant edit from silently burying the screw heads)
    assert abs((hp.YAW_HUB_PLATFORM_Z1 + hp.COXA_WELL_FLOOR_LIFT)
               - COXA_FLOOR_Z - COL_DROP) < 1e-9
    head_top = HORN_HEAD_SEAT_Z + hp.INSERT_M3_BOLT_HEAD_H
    assert COXA_FLOOR_Z - head_top >= 1.2, \
        f"only {COXA_FLOOR_Z - head_top:.2f} mm over the M3 heads"

    # thread engagement preserved: the seat plane must drop 1:1 with
    # the screw length so tip = seat - length never moves off the
    # production plane (corner tips at the disc bottom + TIP_POKE,
    # centre at its spline-bore depth).  Guards a future seat/length
    # edit from silently shortening the horn engagement.
    assert abs((hp.YAW_HUB_HORN_HEAD_SEAT_Z - HORN_HEAD_SEAT_Z)
               - (hp.YAW_HUB_HORN_BOLT_LEN - HORN_BOLT_LEN)) < 1e-9, \
        "horn-screw tip plane moved: seat drop != screw length delta"
    assert abs((hp.YAW_HUB_HORN_CENTRE_SEAT_Z - HORN_CENTRE_SEAT_Z)
               - (hp.YAW_HUB_HORN_BOLT_LEN - HORN_BOLT_LEN)) < 1e-9, \
        "centre-screw tip plane moved: seat drop != screw length delta"

    # seat planes: solid seat annulus just under each seat plane,
    # shaft void just above it (probe between the Phi 3.7 shank and
    # the Phi 5.9 head shaft)
    drive_r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations = [(0.0, 0.0, HORN_CENTRE_SEAT_Z)]
    stations += [(drive_r * np.cos(t), drive_r * np.sin(t),
                  HORN_HEAD_SEAT_Z)
                 for t in hp.DISC_HORN_BOLT_ANGLES_RAD]
    for (sx, sy, seat_z) in stations:
        n = np.hypot(sx, sy)
        ux, uy = (sx / n, sy / n) if n > 1e-9 else (1.0, 0.0)
        px, py = sx + 2.4 * ux, sy + 2.4 * uy
        below, above = coxa.contains(np.array([[px, py, seat_z - 0.3],
                                               [px, py, seat_z + 0.3]]))
        assert below and not above, \
            f"horn-screw seat at ({sx:.1f},{sy:.1f}) not on z {seat_z}"
        # shank passage open through the dropped slab band
        assert not coxa.contains(
            np.array([[sx, sy, SLAB_BOT_Z + 1.5]]))[0], \
            f"screw shank blocked in the slab at ({sx:.1f},{sy:.1f})"

    # well floor at COXA_FLOOR_Z: the seated servo's belly IS the floor
    T = leg_transforms(0)
    sv = meshes["servo_body"].copy()
    sv.apply_transform(T["hip_cap"])
    floor_w = hp.CHASSIS_YAW_OUTPUT_Z + COXA_FLOOR_Z
    assert abs(sv.bounds[0][2] - floor_w) < 0.05, \
        f"seated servo belly {sv.bounds[0][2]:.2f} != floor {floor_w:.2f}"

    # deleted skirt/platform: below the dropped slab, nothing reaches
    # past the brim radius (the column is boss + seat ring + brim only)
    v = coxa.vertices
    low = v[v[:, 2] < SLAB_BOT_Z - 0.01]
    r_low = float(np.hypot(low[:, 0], low[:, 1]).max())
    assert r_low <= BRIM_OD / 2.0 + 0.05, \
        f"material at r {r_low:.2f} below the slab (skirt/platform back?)"

    # slab bottom (rotating) vs the static tower rim: the M3x25 drop
    # spends the old margin down to the design value of 1.5 mm
    # (>= 0.5 rotating-vs-static required; M3x20 would go NEGATIVE,
    # which is why 25 is the floor -- see the HORN SCREWS constants)
    slab_bot_w = hp.CHASSIS_YAW_OUTPUT_Z + SLAB_BOT_Z
    assert slab_bot_w - CHB_RIM_W >= 1.5 - 1e-9, \
        f"slab bottom only {slab_bot_w - CHB_RIM_W:.2f} mm over the rim"

    print(f"  coxa column: floor z {COXA_FLOOR_Z:g} "
          f"({COXA_FLOOR_Z - head_top:g} mm over the M3x{HORN_BOLT_LEN:g} "
          f"heads), seats at {HORN_HEAD_SEAT_Z:g}/{HORN_CENTRE_SEAT_Z:g} "
          f"(production tips kept: seat drop {HORN_SEAT_DROP:g} = length "
          f"delta), cradle dropped {COL_DROP:g} mm (hip axis world "
          f"{hp.CHASSIS_YAW_OUTPUT_Z + COXA_HIP_DROP_V:g}), "
          f"below-slab column max r {r_low:.1f}, "
          f"slab {slab_bot_w - CHB_RIM_W:g} over the tower rim")


def check_chassis_variant(meshes: dict[str, trimesh.Trimesh]) -> None:
    """chassis_bottom_rigid edits landed as designed: below the platform
    top NOTHING outboard of the hex edge survives past the tower
    cylinder (the corner trim leaves one matching curve), the trim
    never bit the tower wall, above the servo-mount deck nothing pokes
    past the tower cylinder at ANY azimuth all the way to the raised
    rim (all three dead ears cut, the az-210 root left merged below
    the deck), all six towers carry the full-wrap raise ring to the
    race-top plane, the printed foot holes are open exactly where the
    pillar feet expect them, the six Wago tray wall sets are GONE
    above the sheet with the sheet still solid underneath, and the
    wire-corridor band inboard of each yaw servo is FLATTENED to the
    bare sheet (rev 5) with the servo-registering deck plateau /
    flanking walls outboard of the cut still standing.  (Pocket seat
    and retainer territory are re-verified against THIS mesh by the
    other checks.)"""
    cb = meshes["chassis_bottom"]
    assert cb.body_count == 1, "chassis variant not a single body"
    v = trimesh.transform_points(cb.vertices, _rotz(-0.5 * np.pi / 3.0))
    near = v[(np.abs(v[:, 1]) < 40.0) & (v[:, 0] > 55.0)]
    ax = APOTHEM

    # corner trim: in the platform + skirt band, every vertex outboard
    # of the hex edge (within the platform's own half-width -- the
    # diagonal well walls at |y| ~ 25 are untouched production) lies
    # on/inside the tower cylinder: the tower's curve is the ONLY
    # silhouette curve out there
    band = near[(near[:, 2] > -6.4) & (near[:, 2] < CHB_PLATE_TOP - 0.01)
                & (near[:, 0] > ax + 0.1) & (np.abs(near[:, 1]) < 21.4)]
    r_out = np.hypot(band[:, 0] - ax, band[:, 1])
    r_max = float(r_out.max())
    assert r_max <= CHB_TRIM_R + 0.03, \
        f"corner material at r {r_max:.2f} > tower cylinder -- trim missed"

    # tower wall unbitten by the corner trim: full Phi 44 ring solid at
    # the pocket band
    ang = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    ring = np.column_stack([ax + (CHB_TOWER_R - 0.6) * np.cos(ang),
                            (CHB_TOWER_R - 0.6) * np.sin(ang),
                            np.full(24, 17.0)])
    Rz = _rotz(0.5 * np.pi / 3.0)
    got = cb.contains(trimesh.transform_points(ring, Rz))
    assert got.all(), "tower wall bitten by a variant cut"

    # raised rim present all the way around on ALL SIX towers (wall
    # solid mid-band just under the race-top rim, air just above it)
    for i in range(6):
        Ri = _rotz((i + 0.5) * np.pi / 3.0)
        r_wall = (hp.YAW_TOWER_BORE_OD / 2.0 + CHB_TOWER_R) / 2.0
        rim_ring = np.column_stack([ax + r_wall * np.cos(ang),
                                    r_wall * np.sin(ang),
                                    np.full(24, CHB_RIM_W - 0.3)])
        assert cb.contains(trimesh.transform_points(rim_ring, Ri)).all(), \
            f"leg {i}: raised rim incomplete"
        above = rim_ring + [0.0, 0.0, 0.6]
        assert not cb.contains(trimesh.transform_points(above, Ri)).any(), \
            f"leg {i}: material above the race-top rim"

    # ears: above the servo-mount deck (z 10.25; ears ran to 19.75),
    # NOTHING pokes past the tower cylinder at any azimuth, all the
    # way up to the raised rim -- az 330/90 were shaved before, the
    # az-210 column is cut to the deck, and the raise ring itself
    # stays at the tower radius
    band = near[(near[:, 2] > CHB_DECK_TOP + 0.1)
                & (near[:, 2] < CHB_RIM_W + 0.1)]
    d = band[:, :2] - [ax, 0.0]
    rr = np.hypot(d[:, 0], d[:, 1])
    n_out = int((rr > CHB_TOWER_R + 0.05).sum())
    assert n_out == 0, \
        f"{n_out} vertices past the tower cylinder above the deck (an ear " \
        f"cut missed)"

    # az-210 root: still solid just BELOW the deck top (the cut must not
    # bite the collar-merged band), and air just above it
    ear_c = np.array([ax + 23.5 * np.cos(np.pi * 7.0 / 6.0),
                      23.5 * np.sin(np.pi * 7.0 / 6.0)])
    probes = trimesh.transform_points(
        np.array([[ear_c[0], ear_c[1], CHB_DECK_TOP - 0.5],
                  [ear_c[0], ear_c[1], CHB_DECK_TOP + 0.5]]),
        _rotz(0.5 * np.pi / 3.0))
    root_in, above_in = cb.contains(probes)
    assert root_in, "az-210 ear root gouged below the deck top"
    assert not above_in, "az-210 ear column survived above the deck"

    # printed foot holes: open at all 18 spots, floor solid beside them
    centres, beside = [], []
    for az in range(0, 360, 60):
        Ra = _rotz(np.deg2rad(az))
        for hx, hy in ((PILLAR_BAR_HOLE_X, +PILLAR_BAR_HOLE_Y),
                       (PILLAR_BAR_HOLE_X, -PILLAR_BAR_HOLE_Y),
                       (PILLAR_TAB_RHO, 0.0)):
            centres.append(trimesh.transform_points(
                np.array([[hx, hy, 0.0]]), Ra)[0])
            beside.append(trimesh.transform_points(
                np.array([[hx + 3.2, hy, 0.0]]), Ra)[0])
    assert not cb.contains(np.array(centres)).any(), \
        "a printed foot hole is blocked"
    assert cb.contains(np.array(beside)).all(), \
        "sheet not solid beside a printed foot hole"

    # wago trays gone: probe mid-height of all three wall positions of
    # every corner tray (must be air) and the sheet just below each
    # (must still be solid -- the cut may not bite the plate)
    t = hp.WAGO_MOUNT_WALL_T
    gone, under = [], []
    for M in hp.wago_tray_corner_transforms():
        for lx, ly in ((TRAY_HALF_X - t / 2.0, 0.0),
                       (0.0, +(TRAY_HALF_Y - t / 2.0)),
                       (0.0, -(TRAY_HALF_Y - t / 2.0))):
            p = np.array([lx, ly, 0.0, 1.0])
            gone.append((M @ (p + [0, 0, hp.WAGO_MOUNT_WALL_H / 2.0, 0]))[:3])
            under.append((M @ (p + [0, 0, -1.0, 0]))[:3])
    assert not cb.contains(np.array(gone)).any(), "a wago tray wall survived"
    assert cb.contains(np.array(under)).all(), \
        "tray delete cut bit into the sheet"

    # corridor flattened (rev 5): the ENTIRE cut band is air on every
    # leg (grid sample); everything below the sheet-top plane is
    # untouched (probed at spots the pre-cut solid owned -- note the
    # corridor itself was already a floorless tunnel in production:
    # the harness aperture passes through the plate, only the canopy
    # roofed it); and the servo-registering structure outboard of the
    # cut still stands (deck plateau, flanking side walls).
    gx = np.arange(CHB_FLAT_X0 + 0.5, CHB_FLAT_X1 - 0.4, 2.0)
    gy = np.arange(-CHB_FLAT_HALF_Y + 0.5, CHB_FLAT_HALF_Y - 0.4, 2.5)
    gz = (3.0, 6.0, 9.5)
    grid = np.array([[x, y, z] for x in gx for y in gy for z in gz])
    below_keep = np.array([
        [56.0, 19.0, 1.0],    # bare sheet at the band edge (inboard of
        [56.0, -19.0, 1.0],   # x 52 the trunk pass opens on legs 0/5)
        [55.0, 15.0, 0.5],    # side-wall roots below the flatten plane
        [55.0, -15.0, 0.5],
        [63.0, 0.0, -3.0],    # belly band under the old wire notch
    ])
    keep_local = np.array([
        [70.0, 16.0, 9.5],    # deck plateau, outboard of the cut
        [70.0, -16.0, 9.5],
        [90.0, 18.0, 5.0],    # side wall flanking the servo body
        [90.0, -18.0, 5.0],
        [70.0, 0.0, 9.5],     # deck skin directly over the servo end
    ])
    for i in range(6):
        Ri = _rotz((i + 0.5) * np.pi / 3.0)
        band = trimesh.transform_points(grid, Ri)
        n_in = int(cb.contains(band).sum())
        assert n_in == 0, \
            f"leg {i}: {n_in} corridor-band points still solid (flatten missed)"
        assert cb.contains(trimesh.transform_points(below_keep, Ri)).all(), \
            f"leg {i}: flatten cut bit below the sheet-top plane"
        assert cb.contains(trimesh.transform_points(keep_local, Ri)).all(), \
            f"leg {i}: flatten cut removed servo-registering structure"

    print(f"  chassis variant: tower bases trimmed to one r {CHB_TRIM_R:g} "
          f"cylinder (outboard max r {r_max:.2f}), rims raised to the "
          f"race top (world {CHB_RIM_W:g}), all 3 dead ears shaved "
          f"(az 210 flush to the deck), 18 foot holes printed in, "
          f"6 wago trays deleted, wire corridors flattened "
          f"(x {CHB_FLAT_X0:g}..{CHB_FLAT_X1:g}, |y| {CHB_FLAT_HALF_Y:g}, "
          f"deck plateau/side walls kept), {abs(cb.volume) / 1000.0:.0f} cm3")


def check_pillars(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Corner pillars: land exactly on the bottom sheet, stop PILLAR_TOP_GAP
    short of the frame (races define the plate plane, screws close the
    gap), keep their screw pilots inside the solid plug, and stay clear
    of every leg through the whole operating yaw range with margin."""
    pillar = meshes["corner_pillar"]
    b = pillar.bounds
    assert abs(b[0][2] - PILLAR_BOT_Z) < 1e-3, "pillar foot not on the sheet"
    assert abs(b[1][2] - (SHEET_Z0 - PILLAR_TOP_GAP)) < 1e-3, \
        "pillar top not at the frame gap plane"
    vol = pillar.volume / 1000.0
    print(f"  corner_pillar: {vol:.1f} cm3 (~{vol * 1.27:.0f} g PETG), "
          f"top gap {PILLAR_TOP_GAP:g} mm under the frame")

    # screw pilots must sit deep inside the Phi 20 plug with real webs
    for rho in (HATCH_SCREW_RHO, PILLAR_FRAME_SCREW_RHO):
        edge = PILLAR_OD / 2.0 - abs(rho - PILLAR_RHO) - PILOT_OD / 2.0
        assert edge >= 3.0, f"pilot at rho {rho:g}: only {edge:.2f} mm wall"
    web = abs(HATCH_SCREW_RHO - PILLAR_FRAME_SCREW_RHO) - PILOT_OD
    assert web >= 2.0, f"only {web:.2f} mm between the two top pilots"

    placed = _pillar_meshes(meshes)
    # seated robot: every pillar vs every leg's static parts + both plates
    statics = [("chassis_bottom", meshes["chassis_bottom"]),
               ("chassis_top_rigid", meshes["chassis_top_rigid"])]
    for i in range(6):
        T = leg_transforms(i)
        for key, fr in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                        ("hip_clamp_cap_rigid", "hip_cap"),
                        ("yaw_servo_retainer", "cradle")):
            m = meshes[key].copy()
            m.apply_transform(T[fr])
            statics.append((f"L{i}-{key}", m))
        ys = meshes["servo_body"].copy()
        ys.apply_transform(T["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                                               -(hp.HORN_STACK_H
                                                 + hp.WELL_RIM_Z)]))
        statics.append((f"L{i}-yaw_servo", ys))
    for j, p in enumerate(placed):
        for name, m in statics:
            if not _bounds_touch(p, m):
                continue
            v = _inter_vol(p, m)
            assert v < 1e-6, f"pillar az{j * 60} overlaps {name} ({v:.2f} mm3)"
    print("  pillars: seated robot clear (all 6 legs + plates + yaw servos)")

    # operating yaw range with margin: leg 0 vs its two flanking pillars
    for yaw in (-45.0, -35.0, -20.0, 0.0, 20.0, 35.0, 45.0):
        T = leg_transforms(0, yaw_deg=yaw)
        for key, fr in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                        ("hip_clamp_cap_rigid", "hip_cap")):
            m = meshes[key].copy()
            m.apply_transform(T[fr])
            for j in (0, 1):    # pillars at az 0 and az 60 flank leg 0
                if not _bounds_touch(placed[j], m):
                    continue
                v = _inter_vol(placed[j], m)
                assert v < 1e-6, \
                    f"yaw {yaw:+g}: {key} hits pillar az{j * 60} ({v:.2f} mm3)"
    # ROTATING ENVELOPE (user, Aug 24: round the parts, not the pillar)
    # -- every rotating part must fit the 38.2 mm cylinder about its
    # own yaw axis; that alone guarantees the column clearance.
    r_coxa = float(np.linalg.norm(
        meshes["coxa_link"].vertices[:, :2], axis=1).max())
    assert r_coxa <= ROT_ENVELOPE_R + 0.05, \
        f"rounded coxa reaches {r_coxa:.2f} > envelope {ROT_ENVELOPE_R}"
    vc = meshes["hip_clamp_cap_rigid"].vertices
    r_cap = float(np.hypot(vc[:, 0], vc[:, 2] - AXIS_Z).max())
    assert r_cap <= ROT_ENVELOPE_R + 0.05, \
        f"hip cap reaches {r_cap:.2f} > envelope {ROT_ENVELOPE_R}"

    # QUANTITATIVE margin: not just non-colliding -- assert the
    # measured distance stays >= the guaranteed clearance minus a
    # small numerical allowance, across the operating range.
    from trimesh.proximity import signed_distance
    min_marg = 1e9
    for yaw in (-45.0, -20.0, 0.0, 20.0, 45.0):
        T = leg_transforms(0, yaw_deg=yaw)
        for key, fr in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                        ("hip_clamp_cap_rigid", "hip_cap")):
            m = meshes[key].copy()
            m.apply_transform(T[fr])
            for j in (0, 1):
                if not _bounds_touch(placed[j], m):
                    continue
                d = float(-signed_distance(placed[j], m.vertices).max())
                min_marg = min(min_marg, d)
    assert min_marg >= PILLAR_MIN_CL - 0.25, \
        f"pillar clearance margin dropped to {min_marg:.2f} mm"
    print(f"  pillars: coxa/cap envelope {r_coxa:.2f}/{r_cap:.2f} <= "
          f"{ROT_ENVELOPE_R:g}; measured leg clearance >= {min_marg:.2f} mm")

    # informational: full hand-spin (servo out) contact scan
    first_contact = None
    for yaw in np.arange(-180.0, 180.0, 15.0):
        T = leg_transforms(0, yaw_deg=float(yaw))
        hit = False
        for key, fr in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                        ("hip_clamp_cap_rigid", "hip_cap")):
            m = meshes[key].copy()
            m.apply_transform(T[fr])
            for j in (0, 1):
                if _bounds_touch(placed[j], m) \
                        and _inter_vol(placed[j], m) > 1e-6:
                    hit = True
        if hit and abs(yaw) > 46.0:
            first_contact = float(yaw) if first_contact is None \
                else first_contact
        assert not (hit and abs(yaw) <= 46.0), \
            f"pillar contact inside operating yaw range at {yaw:+g}"
    if first_contact is None:
        print("  pillars: clear even for a full 360 hand-spin of the leg")
    else:
        print(f"  pillars: operating range +/-45 clear; hand-spinning a leg "
              f"past {first_contact:+g} deg would touch a pillar (note only)")


def _bounds_touch(a: trimesh.Trimesh, b: trimesh.Trimesh) -> bool:
    return bool(np.all(a.bounds[0] <= b.bounds[1] + 1.0)
                and np.all(b.bounds[0] <= a.bounds[1] + 1.0))


def check_wago_block(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Central splice block: printable, sits on solid sheet, fits its
    territory (strap slots / standoffs / trunk pass), whole footprint
    under the open hatch, and the nut stack stays below the rotating
    band even with a lever flipped up."""
    blk = meshes["centre_wago_block"]
    assert blk.is_watertight, "wago block not watertight"
    assert blk.body_count == 1, "wago block not a single body"

    # nothing on the chassis floor where it sits (walls, pads, ports)
    inter = trimesh.boolean.intersection(
        [blk, meshes["chassis_bottom"]], engine="manifold")
    v = 0.0 if inter.is_empty else abs(inter.volume)
    assert v < 1.0, f"wago block overlaps chassis_bottom ({v:.2f} mm3)"

    # solid sheet under the full footprint (VHB backing, no slot bridged)
    xs = np.linspace(-WBLK_HALF_X + 1, WBLK_HALF_X - 1, 23)
    ys = np.linspace(-WBLK_HALF_Y + 1, WBLK_HALF_Y - 1, 15)
    XX, YY = np.meshgrid(xs, ys)
    pts = np.column_stack([XX.ravel(), YY.ravel(),
                           np.full(XX.size, PILLAR_BOT_Z - 1.0)])
    frac = meshes["chassis_bottom"].contains(pts).mean()
    assert frac > 0.999, f"floor under wago block only {frac:.1%} solid"

    # territory: strap slots (inner edge y=24), standoff posts, trunk pass
    slot_y0 = hp.BATTERY_STRAP_SLOT_Y - 2.0                    # 24
    assert WBLK_HALF_Y + 1.5 <= slot_y0, "block reaches the strap slots"
    assert WBLK_HALF_Y + 6.0 <= abs(hp.CHASSIS_STANDOFF_HOLES_XY[0][1]), \
        "block reaches the standoff posts"
    assert WBLK_HALF_X + 2.0 <= (hp.BATTERY_TRUNK_HOLE_CENTRE[0]
                                 - hp.BATTERY_TRUNK_HOLE_X / 2.0), \
        "block reaches the battery trunk pass"

    # fully under the open hatch, and levers stay below the swing band
    corner_r = float(np.hypot(WBLK_HALF_X, WBLK_HALF_Y))
    assert corner_r <= HATCH_OPEN_APO - 3.0, \
        f"block corner r {corner_r:.1f} not under the hatch opening"
    lever_top = PILLAR_BOT_Z + WBLK_FLOOR_T + hp.WAGO5_H + 10.0
    assert lever_top < ROT_BAND_Z0, \
        f"open lever top z {lever_top:.1f} enters the rotating band"

    # seated nuts: the 0.15 press wedge is the ONLY block contact
    for M in _wago5_scene_frames():
        w = meshes["wago5"].copy()
        w.apply_transform(M)
        inter = trimesh.boolean.intersection([w, blk], engine="manifold")
        v = 0.0 if inter.is_empty else abs(inter.volume)
        assert v < 40.0, f"seated nut vs block: {v:.2f} mm3 (not a wedge)"
    print(f"  wago block: {abs(blk.volume) / 1000.0:.1f} cm3, "
          f"4 bays press-fit, corner r {corner_r:.1f} "
          f"under the {HATCH_OPEN_APO:g} mm opening, "
          f"lever top {lever_top:.1f} < band {ROT_BAND_Z0:g}")


def check_hatch(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The hatch must drop STRAIGHT DOWN into the seated frame (lip into
    the opening) with zero overlap, and its screw/standoff/elec holes
    must land on frame bosses / open interior respectively."""
    frame = meshes["chassis_top_rigid"]
    hatch = meshes["top_hatch_rigid"]
    v = _inter_vol(frame, hatch)
    assert v < 1e-6, f"hatch overlaps the frame when seated ({v:.2f} mm3)"
    for dz in (0.5, 2.0, 5.0, 20.0, 60.0):
        h = hatch.copy()
        h.apply_translation([0, 0, dz])
        v = _inter_vol(h, frame)
        assert v < 1e-6, f"hatch descent +{dz}: fouls the frame ({v:.1f} mm3)"
        for i in range(6):
            T = leg_transforms(i)
            for key, fr in (("hip_clamp_cap_rigid", "hip_cap"),
                            ("bearing_6805", "hip_cap"),
                            ("coxa_link", "coxa")):
                m = meshes[key].copy()
                m.apply_transform(T[fr])
                v = _inter_vol(h, m)
                assert v < 1e-6, \
                    f"hatch descent +{dz}: fouls L{i} {key} ({v:.1f} mm3)"
    print("  hatch: seats with zero overlap, straight drop-in verified")


def check_yaw_sweep(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The coxa-mounted assembly must clear the plate at EVERY yaw angle.
    Analytic guard on all rotating vertices + boolean spot checks."""
    plate = meshes["chassis_top_rigid"]
    rotating = [("coxa_link", "coxa"), ("hip_clamp_cap_rigid", "hip_cap"),
                ("servo_body", "hip_cap")]
    axis_xy = leg_transforms(0)["coxa"][:2, 3]
    for key, frame in rotating:
        m = meshes[key].copy()
        m.apply_transform(leg_transforms(0)[frame])
        v = m.vertices
        rad = np.linalg.norm(v[:, :2] - axis_xy, axis=1)
        hi = v[:, 2] > RING_BOT_W - 0.5
        bad = hi & (rad > SHOULDER_OD / 2.0 - 1.0)
        assert not bad.any(), (
            f"{key}: {int(bad.sum())} vertices sweep into the plate "
            f"(z > {RING_BOT_W - 0.5:.2f} outside the shoulder bore)")
        # anything above the ring bottom must also stay under the shoulder
        # plane minus nothing -- only the boss tip may pass SHEET_Z0
        inbore = hi & (rad <= SHOULDER_OD / 2.0 - 1.0)
        if inbore.any():
            assert v[inbore, 2].max() <= SHEET_Z1 + 1e-6
    for phi in (0.0, 90.0, 180.0, 270.0):
        T = leg_transforms(0, yaw_deg=phi)
        for key, frame in rotating:
            m = meshes[key].copy()
            m.apply_transform(T[frame])
            for pk in ("chassis_top_rigid", "top_hatch_rigid"):
                v = _inter_vol(m, meshes[pk])
                assert v < 1e-6, f"yaw {phi}: {key} hits {pk} ({v:.2f} mm3)"
    print("  yaw sweep (0..360): coxa assembly clears the plate + hatch")


def check_plate_descent(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Assembly: the plate must drop STRAIGHT DOWN onto all six seated
    bearings without fouling anything (races enter the pockets last)."""
    static = []
    for i in range(6):
        T = leg_transforms(i)
        for key, frame in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                           ("hip_clamp_cap_rigid", "hip_cap"),
                           ("bearing_6805", "hip_cap")):
            m = meshes[key].copy()
            m.apply_transform(T[frame])
            static.append((f"L{i}-{key}", m))
    for j, p in enumerate(_pillar_meshes(meshes)):
        static.append((f"pillar-az{j * 60}", p))
    plate = meshes["chassis_top_rigid"]
    for dz in (0.5, 2.0, 5.0, 10.0, 25.0, 50.0):
        p = plate.copy()
        p.apply_translation([0, 0, dz])
        for name, m in static:
            v = _inter_vol(p, m)
            assert v < 1e-6, f"descent +{dz}: plate fouls {name} ({v:.1f} mm3)"
    print("  plate descent: clear drop onto all six bearings")


def sweep_femur_envelope(meshes: dict[str, trimesh.Trimesh],
                         yaw_grid=(-35.0, -20.0, 0.0, 20.0, 35.0),
                         p_hi: float = -90.0, step: float = 2.5):
    """Find, per yaw angle, the first femur UP angle that contacts the new
    top plate (or the pedestal/boss/bearing stack).  Production workspace
    is femur in [-80, +30]; this variant trades away the deep-tuck end."""
    plate = meshes["chassis_top_rigid"]
    tube, _ = _tibia_extras()
    moving_keys = (("femur_link", "femur_mh"), ("servo_body", "knee_cap"),
                   ("knee_clamp_cap", "knee_cap"),
                   ("tibia_knee_yoke", "tibia_mh"), ("tibia_tube", "tibia"))

    def moving(i, yaw, p):
        T = leg_transforms(i, yaw_deg=yaw, femur_deg=p)
        frames = {"femur_mh": T["femur"] @ MH, "knee_cap": T["knee_cap"],
                  "tibia_mh": T["tibia"] @ MH, "tibia": T["tibia"]}
        for key, fr in moving_keys:
            m = (tube if key == "tibia_tube" else meshes[key]).copy()
            m.apply_transform(frames[fr])
            yield key, m

    # (A) vs the yaw-rotating boss/bearing stack -- yaw-independent.
    stack = _union([meshes["hip_clamp_cap_rigid"], meshes["bearing_6805"]])
    stack.apply_transform(leg_transforms(0)["hip_cap"])
    contact_a = None
    for p in np.arange(0.0, p_hi - 1e-9, -step):
        if any(_inter_vol(m, stack) > 1.0 for _k, m in moving(0, 0.0, p)):
            contact_a = float(p)
            break
    # (B) vs the world-fixed plate + hatch, per yaw.
    hatch = meshes["top_hatch_rigid"]
    contact_b = {}
    for yaw in yaw_grid:
        contact_b[yaw] = None
        for p in np.arange(0.0, p_hi - 1e-9, -step):
            hit = False
            for _k, m in moving(0, yaw, p):
                if m.bounds[1][2] < RING_BOT_W:      # cheap z prefilter
                    continue
                if _inter_vol(m, plate) > 1.0 or _inter_vol(m, hatch) > 1.0:
                    hit = True
                    break
            if hit:
                contact_b[yaw] = float(p)
                break
    print(f"  femur-vs-boss/bearing first contact: {contact_a} deg")
    for yaw, c in contact_b.items():
        print(f"  femur-vs-plate first contact @ yaw {yaw:+.0f}: {c} deg")
    worst = [c for c in ([contact_a] + list(contact_b.values())) if c is not None]
    limit = (max(worst) + step + 2.5) if worst else p_hi  # margin: 1 grid + 2.5
    limit = float(np.ceil(limit / 2.5) * 2.5)
    print(f"  => safe femur UP limit (with margin): {limit} deg "
          f"(production workspace allowed -80)")
    assert limit <= -45.0, (
        f"rigid-hip plate limits femur up-swing to {limit} deg -- "
        "walking swing headroom (-45) is gone, redesign the stack heights")
    return limit, contact_a, contact_b


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
COLORS = {
    "hip_clamp_cap_rigid": "#4878b0", "chassis_top_rigid": "#5b8fd4",
    "top_hatch_rigid": "#6fa8dc", "corner_pillar": "#3f6ea6",
    "centre_wago_block": "#4a90a4", "wago5": "#d9822b",
    "bearing_6805": "#303030",
    "yaw_bearing_upper": "#3a3a3a", "servo_body": "#6b6b6b",
    "chassis_bottom": "#8b93a6", "coxa_link": "#7ba1d1",
    "femur_link": "#9aa0a6", "tibia_knee_yoke": "#9aa0a6",
    "knee_clamp_cap": "#9aa0a6",
    "yaw_servo_retainer": "#9aa0a6", "foot_boot": "#5a5f66",
    "tibia_tube": "#404040",
}
COTS = {"servo_body", "yaw_bearing_upper", "bearing_6805", "tibia_tube",
        "wago5"}


def _mat16(M: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(M, float).T.reshape(-1)]


def build_scene(meshes, femur_up_limit: float) -> dict:
    _tube, foot_frame = _tibia_extras()
    mesh_defs = [{"id": f"stl:{k}", "name": fn, "url": f"stl/{fn}"}
                 for k, (_f, fn) in MESH_FILES.items()]
    mesh_defs.append({"id": "stl:tibia_tube", "name": "tibia_tube",
                      "url": "stl/tibia_tube_DO_NOT_PRINT.stl"})

    instances, joints = [], []
    n = 0

    def inst(key, name, M, leg=None, part=None):
        nonlocal n
        iid = f"{n:03d}-{name}"
        n += 1
        instances.append({
            "id": iid, "meshId": f"stl:{key}", "name": name,
            "partType": part or key, "role": "variant", "leg": leg,
            "joint": None, "cots": key in COTS, "color": COLORS[key],
            "transform": _mat16(M)})
        return iid

    inst("chassis_bottom", "chassis_bottom RIGID (full-wrap towers, NEW)",
         np.eye(4))
    inst("chassis_top_rigid", "chassis_top_rigid FRAME (NEW)", np.eye(4))
    inst("top_hatch_rigid", "top_hatch (NEW, removable)", np.eye(4))
    for az in range(0, 360, 60):
        inst("corner_pillar", f"corner pillar az{az} (NEW)",
             _rotz(np.deg2rad(az)))
    inst("centre_wago_block", "central splice block (NEW)", np.eye(4))
    for k, (M, label) in enumerate(zip(_wago5_scene_frames(),
                                       ("V+ west", "V+ east",
                                        "GND west", "GND east"))):
        inst("wago5", f"221-415 {label}", M)
    for i in range(6):
        T = leg_transforms(i)
        a = (i + 0.5) * np.pi / 3.0
        axis_pt = [APOTHEM * np.cos(a), APOTHEM * np.sin(a),
                   hp.CHASSIS_YAW_OUTPUT_Z]
        pitch_ax = (_rotz(a)[:3, :3] @ np.array([0.0, 1.0, 0.0])).tolist()
        hip_pt = (T["coxa"] @ np.array([*COXA_HIP_ANCHOR_V, 1.0]))[:3]
        knee_pt = (T["femur"] @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0, 1.0]))[:3]

        yaw_ids = [
            inst("coxa_link", f"L{i} coxa_link RIGID (NEW)",
                 T["coxa"], leg=i),
            inst("yaw_bearing_upper", f"L{i} yaw bearing (tower-seated)",
                 T["coxa"] @ _trans([0.0, 0.0, YAWBR_DROP]), leg=i),
            inst("servo_body", f"L{i} hip servo", T["hip_cap"], leg=i),
            inst("hip_clamp_cap_rigid", f"L{i} hip cap RIGID (NEW)",
                 T["hip_cap"], leg=i),
            inst("bearing_6805", f"L{i} third 6805 (NEW)", T["hip_cap"], leg=i),
        ]
        inst("yaw_servo_retainer", f"L{i} yaw retainer", T["cradle"], leg=i)
        # Yaw servo is CHASSIS-mounted (does not rotate with the leg), so it
        # is placed via the leg frame but NOT listed in the yaw joint.
        inst("servo_body", f"L{i} yaw servo",
             T["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                                 -(hp.HORN_STACK_H + hp.WELL_RIM_Z)]),
             leg=i)
        hip_ids = [
            inst("femur_link", f"L{i} femur_link", T["femur"] @ MH, leg=i),
            inst("servo_body", f"L{i} knee servo", T["knee_cap"], leg=i),
            inst("knee_clamp_cap", f"L{i} knee cap", T["knee_cap"], leg=i),
        ]
        knee_ids = [
            inst("tibia_knee_yoke", f"L{i} tibia yoke", T["tibia"] @ MH, leg=i),
            inst("tibia_tube", f"L{i} tibia tube", T["tibia"], leg=i),
            inst("foot_boot", f"L{i} foot boot", T["tibia"] @ foot_frame, leg=i),
        ]
        joints += [
            {"id": f"L{i}-yaw", "type": "revolute", "axis": [0, 0, 1],
             "origin": axis_pt, "instances": yaw_ids,
             "limits": {"min": -35.0, "max": 35.0}, "home": 0,
             "label": f"L{i} yaw"},
            {"id": f"L{i}-hip", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in hip_pt], "instances": hip_ids,
             "limits": {"min": femur_up_limit, "max": 30.0}, "home": 0,
             "label": f"L{i} hip (up-limit {femur_up_limit:g} due to plate)"},
            {"id": f"L{i}-knee", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in knee_pt], "instances": knee_ids,
             "limits": {"min": -30.0, "max": 20.0}, "home": 0,
             "label": f"L{i} knee"},
        ]

    up = femur_up_limit - hp.STANCE_FEMUR_DEG  # joint value at the limit
    scene = {
        "name": "sts3215 rigid-hip variant -- third 6805 above each hip",
        "source": "concepts/rigid_hip/make_rigid_hip_variant.py",
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0, 0, 55],
        "checksConfig": {
            "overlapMm3": 80.0, "pitchMm": 2.0,
            "ignoreOverlapPairs": [
                ["chassis_top_rigid", "top_hatch_rigid"],
                ["corner_pillar", "chassis_bottom"],
                ["corner_pillar", "chassis_top_rigid"],
                ["centre_wago_block", "wago5"],   # 0.15 press wedge
                ["chassis_bottom", "servo_body"],
                ["coxa_link", "servo_body"],
                ["coxa_link", "yaw_bearing_upper"],
                ["femur_link", "servo_body"],
                ["hip_clamp_cap_rigid", "servo_body"],
                ["knee_clamp_cap", "servo_body"],
                ["tibia_knee_yoke", "servo_body"],
                ["foot_boot", "tibia_tube"],
                ["tibia_knee_yoke", "tibia_tube"],
            ]},
        "meshes": mesh_defs,
        "instances": instances,
        "joints": joints,
        "poses": [
            {"id": "stance", "name": "Stance (walking)",
             "jointValues": {j["id"]: 0.0 for j in joints}},
            {"id": "femur-up-limit", "name":
                f"L0 femur at the NEW up limit ({femur_up_limit:g} deg abs)",
             "jointValues": {"L0-hip": up}},
        ],
        "animations": [
            {"id": "sweep", "name": "L0: yaw + femur sweep to the new limits",
             "loop": True, "duration": 8.0,
             "keyframes": [
                 {"t": 0.0, "jointValues": {"L0-yaw": 0, "L0-hip": 0}},
                 {"t": 1.5, "jointValues": {"L0-yaw": 35, "L0-hip": 0}},
                 {"t": 3.0, "jointValues": {"L0-yaw": -35, "L0-hip": 0}},
                 {"t": 4.0, "jointValues": {"L0-yaw": 0, "L0-hip": 0}},
                 {"t": 5.5, "jointValues": {"L0-yaw": 0, "L0-hip": up}},
                 {"t": 6.5, "jointValues": {"L0-yaw": 0, "L0-hip": up}},
                 {"t": 8.0, "jointValues": {"L0-yaw": 0, "L0-hip": 0}},
             ]},
        ],
    }
    return scene


def render_preview(meshes) -> None:
    """Radial cross-section through the leg-0 yaw axis: the money shot of
    cap -> pedestal -> boss -> bearing -> pocket -> plate."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = 0.5 * np.pi / 3.0
    Rz = _rotz(-a)  # leg-0 radial -> +x
    T = leg_transforms(0)
    sections = [
        ("chassis_top_rigid", np.eye(4), "#5b8fd4",
         "chassis_top_rigid frame (NEW)"),
        ("top_hatch_rigid", np.eye(4), "#6fa8dc", "service hatch (NEW)"),
        ("hip_clamp_cap_rigid", T["hip_cap"], "#4878b0",
         "hip cap + pedestal + boss (NEW)"),
        ("bearing_6805", T["hip_cap"], "#303030", "top 6805-2RS (NEW)"),
        ("coxa_link", T["coxa"], "#7ba1d1",
         "coxa_link RIGID (rounded + seat ring + dust brim, NEW)"),
        ("yaw_bearing_upper", T["coxa"] @ _trans([0.0, 0.0, YAWBR_DROP]),
         "#303030", "bottom 6805-2RS (fully housed, cap DELETED)"),
        ("chassis_bottom", np.eye(4), "#8b93a6",
         "chassis_bottom RIGID (full-wrap towers, NEW)"),
        ("servo_body", T["hip_cap"], "#c9a227", "hip servo"),
    ]
    fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=130)
    for key, M, color, label in sections:
        m = meshes[key].copy()
        m.apply_transform(Rz @ M)
        sec = m.section(plane_origin=[APOTHEM, 0, 0], plane_normal=[0, 1, 0])
        if sec is None:
            continue
        planar, _ = sec.to_2D(to_2D=rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
        first = True
        for poly in planar.polygons_full:
            xs, ys = poly.exterior.xy
            ax.fill(xs, ys, color=color, alpha=0.6,
                    label=label if first else None)
            first = False
            for ring in poly.interiors:
                ax.fill(*ring.xy, color="white")
    for z, lab in ((CAP_FACE_W, "cap face"), (BR_BOT_W, "race seat"),
                   (SHEET_Z0, "shoulder / sheet bottom"),
                   (SHEET_Z1, "deck face")):
        ax.axhline(z, color="k", lw=0.5, ls=":")
        ax.annotate(f" {lab} z={z:.2f}", (APOTHEM + 42, z), fontsize=7,
                    va="bottom")
    ax.set_xlim(APOTHEM - 65, APOTHEM + 72)
    ax.set_ylim(0, 108)
    ax.set_aspect("equal")
    ax.set_xlabel("radial position from body centre [mm]")
    ax.set_ylabel("world Z [mm]")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("rigid-hip stack -- section through the leg-0 yaw axis")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview.png"))
    print("  wrote preview.png")


def main() -> None:
    skip_sweep = "--skip-sweep" in sys.argv
    print("building meshes ...")
    meshes = build_meshes()

    print("static checks ...")
    check_static(meshes)
    check_bottom_joint(meshes)
    check_coxa_column(meshes)
    check_chassis_variant(meshes)
    check_pillars(meshes)
    check_wago_block(meshes)
    check_hatch(meshes)
    check_yaw_sweep(meshes)
    check_plate_descent(meshes)

    if skip_sweep:
        limit = -55.0
        print("SWEEP SKIPPED (--skip-sweep): using placeholder limit -55")
    else:
        print("femur workspace sweep (this is the slow part) ...")
        limit, _ca, _cb = sweep_femur_envelope(meshes)

    print("writing scene.json ...")
    scene = build_scene(meshes, limit)
    with open(os.path.join(HERE, "scene.json"), "w", encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)

    render_preview(meshes)
    print("done.")


if __name__ == "__main__":
    main()
