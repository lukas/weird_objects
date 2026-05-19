"""Hexapod walker — PROTOTYPE STL generator for a tabletop, hobby-servo build.

This is a scaled-down sibling of `hexapod_walker.py`.  Same architecture
(regular hex chassis, six identical 3-DOF legs, alternating-tripod
gait), but every dimension is shrunk ~ 6 x and every actuator pocket is
re-shaped for a generic 25 kg-cm hobby servo (DS3225 / MG996R class)
instead of an industrial harmonic-drive servomotor.

Why a separate file?
    The two designs share zero usable geometry: a hobby servo is a
    20 x 40 x 40 mm rectangular brick with two M3 mounting tabs and a
    splined output shaft on top, NOT a 170 mm flanged cylinder.  The
    parts that bolt to it (brackets, links, horn adapters) are
    completely different.  Sharing constants would just make every
    `make_*` function a thicket of `if PROTOTYPE:` branches.

Outputs (in ./stl_prototype/):

    Body parts (one each)
        chassis_top.stl         -- 3D-printable hex top plate
        chassis_bottom.stl      -- 3D-printable hex bottom plate
        battery_holder.stl      -- clip-in tray for one 3S 2200 mAh LiPo
        electronics_tray.stl    -- mounting plate for Arduino + PCA9685

    Per-leg parts (one of each — print 6 sets)
        coxa_bracket.stl        -- bolts to the chassis edge, holds the yaw servo
        coxa_link.stl           -- horn-driven U-bracket; holds the hip-pitch servo
        femur_link.stl          -- thigh; horn-driven by hip, holds the knee servo
        tibia_link.stl          -- shin; horn-driven by knee, ends in the foot socket
        foot_pad.stl            -- compliant foot pad (TPU or printed PLA + rubber tip)

    Generic (one — print 18 + spares)
        servo_horn_adapter.stl  -- round disc that bolts to a standard 25T spline
                                   horn and provides 4 x M3 holes on a 20.8 mm PCD
                                   at 0 / 90 / 180 / 270 deg (aligned with the
                                   X-horn arms) so a flat printed link can mate
                                   to a servo horn

    Assembly preview (everything in standing pose — visualization only)
        assembly_preview.stl

Geometry overview
-----------------

    Six identical legs spaced 60 deg around a hexagonal body.  Each
    leg has the same 3 rotary DOF as the full-size walker:

         coxa  (hip yaw,    vertical axis)
         femur (hip pitch,  horizontal radial axis)
         tibia (knee pitch, horizontal radial axis)

    18 hobby servos total.  Recommended part: ANNIMOS / Miuzei DS3225
    (25 kg-cm, metal gear, ~$13 each on AliExpress / ~$18 on Amazon).
    The MG996R (10 kg-cm) is also fine for the lighter-weight builds
    but leave less torque margin at the knee.

    Power: 1 x 3S 2200 mAh LiPo (or any 6-7.4 V supply >= 5 A).
    Brain: Arduino Mega + PCA9685 16-channel PWM driver (need 2 boards
    for 18 servos, daisy-chained over I2C).

Build envelope
--------------

    Chassis flat-to-flat:    200 mm
    Foot-to-foot extended:   ~ 700 mm
    Standing height:         ~ 100 mm
    Total mass:              ~ 1.3 kg
    Per-leg static load:     ~ 4.3 N (~ 0.4 kg)
    Peak knee torque:        ~ 0.6 N*m (~ 6 kg-cm) -- DS3225 has 4x margin

See PROTOTYPE.md for the BOM, wiring diagram, and gait-controller
starter code.
"""

from __future__ import annotations

import os

import numpy as np
import trimesh
from trimesh.creation import box as box_mesh
from trimesh.creation import cylinder as cylinder_mesh
from trimesh.transformations import rotation_matrix


# ---------------------------------------------------------------------------
# Geometry constants  (everything in millimetres)
# ---------------------------------------------------------------------------

# ---- Vehicle envelope -----------------------------------------------------
# A regular hexagonal 4 mm 3D-printed top + bottom plate, 200 mm flat-to-flat.
# Comfortably fits on a 220 x 220 mm hobbyist printer bed (Ender 3 / P1S
# size).  At nominal stance the foot pads sit on a circle of radius
# ~ 350 mm, so the vehicle's outer diameter is ~ 700 mm.
CHASSIS_FLAT_TO_FLAT  = 200.0   # mm -- distance between opposite hex edges
CHASSIS_PLATE_T       =   4.0   # mm -- thickness of each 3D-printed plate
CHASSIS_GAP           =  20.0   # mm -- vertical gap between top + bottom plates
                                #     (room for battery, brain, wiring)
# The TOP plate is intentionally smaller than the bottom plate.  The bottom
# plate is the structural one: it sandwiches the coxa-bracket flanges (with
# their four M3 bolts each) and also takes the per-leg servo body cutouts.
# The TOP plate carries only the battery holder, electronics tray and the
# optional arm bracket -- all of which sit within a ~70 mm radius of the
# chassis centre.  Was 200 mm flat-to-flat to match the bottom; the
# check_workspace_self_collision sweep found that across the full
# (yaw, hip, knee) workspace the coxa_link's hip-pitch well and the
# femur_link's hip pad sweep through z ~ 8..36 mm above the bracket
# flange, which is the same height band the chassis_top occupies
# (chassis_top centre at z = chassis_lift + CHASSIS_GAP + CHASSIS_PLATE_T
#  = chassis_lift + 24, half-thickness 2 mm).  Shrinking the top plate to
# 140 mm flat-to-flat (= apothem 70 mm) keeps the deck for the battery +
# electronics + arm but moves its outer edge inside the radius the leg
# sweeps reach, eliminating ~95 of the 111 collisions found in the
# pre-fix audit.
CHASSIS_TOP_FLAT_TO_FLAT = 140.0  # mm

# ---- Leg link lengths -----------------------------------------------------
# Same 1 : 4 : ~5 ratio as the full-size walker, scaled for a tabletop
# build.  Tibia is intentionally a hair longer than 4 x coxa so the
# foot can lift clear over a small obstacle in swing phase.
COXA_LENGTH    =  25.0   # mm -- yaw axis -> hip-pitch axis
FEMUR_LENGTH   =  90.0   # mm -- hip-pitch axis -> knee axis
TIBIA_LENGTH   = 130.0   # mm -- knee axis -> foot tip

# ---- Coxa link pedestal --------------------------------------------------
# How far above the horn-adapter mating face the coxa-link arm + well
# float on a built-in pedestal.  Without the lift the hip-pitch well
# bottom sits ~2.5 mm below the chassis-plate top and the femur's
# hip-pad swings into the bracket flange + well as the femur pitches.
#
# Sizing constraint:  the femur's hip pad/neck is a CYLINDER of radius
# HIP_PAD_R = 17 mm rotating about the +Y joint axis.  The lowest
# coxa-link z it reaches under rotation is hip_drop - HIP_PAD_R =
# -(WELL_D/2 + arm_t/2) + COXA_LIFT - 17 mm; the bracket's flange
# TOP face sits at world z = BRACKET_FLANGE_T = 15 mm (the flange
# was thickened so a solid top cap closes off the body+tab passage --
# see BRACKET_FLANGE_T below), which equals coxa-link z =
# -yaw_output_z + BRACKET_FLANGE_T = -10.75 mm in the standing pose.
# Setting (lowest pad z in coxa-link) >= -10.75 mm gives
#   COXA_LIFT >= BRACKET_FLANGE_T + WELL_D/2 + arm_t/2
#                + HIP_PAD_R - yaw_output_z
#              = 15 + 14.5 + 3 + 17 - 25.75 = 23.75 mm.
# COXA_LIFT = 26 mm gives ~ 2.25 mm of safety margin against FDM
# print tolerances and the voxel sampler's pitch (1.5 mm).  Was 14 mm
# when the flange was a thin 4 mm slab with a through-slot for the
# body+tab passage; bumping the flange to a solid 15 mm slab (with
# the body cavity as a one-sided pocket open only on the bottom of
# the flange and a small Phi 11 mm gear clearance through the top
# cap) requires the matching COXA_LIFT bump so the femur's swept hip
# pad still clears the taller flange.
#
# Bolt screws need to be ~M3 x 32 mm long to reach through the
# (taller) pedestal + horn adapter + plastic horn into the servo's
# output gear.  The pedestal's structural Z extent is unchanged
# (still 3.5 mm; the lift increases the air gap below the structural
# slab, not the slab itself).
# COXA_LIFT history:
#   COXA_LIFT = 14 was the original value with a thin 4 mm bracket flange
#   and HIP_PAD_R = 17.  Bumping the flange to a solid 15 mm slab (so a
#   top cap closes the body+tab passage) required COXA_LIFT = 26 to keep
#   the femur's swept hip pad clear of the thicker flange.
#
#   Bumping HIP_PAD_R from 17 to 19.5 (so the hollow-annulus neck has a
#   printable 3 mm wall around the horn-stack clearance void -- see the
#   HIP_PAD_R docstring) puts the pad's swept disk +Z edge at
#       hip_drop + HIP_PAD_R = -17.5 + COXA_LIFT + 19.5 = COXA_LIFT + 2 mm
#   which would COLLIDE with the coxa-link arm's bottom face at z =
#   COXA_LIFT (the arm rests directly on the pedestal top, so arm-bottom
#   and pedestal-top share the COXA_LIFT plane).
#
#   We FIRST drop the well 4 mm deeper than the "arm-bottom = well-top"
#   default (WELL_Z_DROP_EXTRA = 4 mm; see below) so the hip axis lives
#   4 mm below arm-bottom and the pad's +Z edge drops by 4 mm relative
#   to the arm.  This gives 2 mm of vertical air between the pad's +Z
#   edge and the arm's bottom face.
#
#   We THEN bump COXA_LIFT to 32 mm so the pad's -Z edge (hip_drop -
#   HIP_PAD_R = COXA_LIFT - 21.5 - 19.5 = COXA_LIFT - 41 mm) still clears
#   the coxa-bracket flange's top face (at coxa-link z = -10.75 mm in the
#   standing pose).  COXA_LIFT = 32 mm gives:
#     pad -Z edge = -9.5 mm > -10.75 mm  (1.25 mm clearance to bracket)
#     pad +Z edge = +30.5 mm < +32 mm    (1.50 mm clearance to arm)
#   Both clearances are sized for FDM print tolerance + voxel sampler
#   pitch (1.5 mm) so check_workspace_self_collision stays clean.
COXA_LIFT     = 32.0     # mm

# Extra drop of the hip-pitch well centre BELOW the natural "arm-bottom =
# well-top" plane.  When 0, the well's +Z face (= the hip-pitch axis +
# WELL_D/2) coincides with the arm's bottom face and the bridge member
# is a thin 0.5 mm boolean kiss.  When > 0, the well sits this many mm
# deeper, the hip axis drops by the same amount, and the bridge box
# extends DOWN to overlap the new well-top.  Used to keep the femur hip
# pad's swept +Z edge clear of the coxa-link arm's bottom face -- see
# COXA_LIFT docstring for the geometry.
WELL_Z_DROP_EXTRA = 4.0  # mm

# See COXA_HIP_DROP below for the derived hip-axis Z position.  It needs
# WELL_D and COXA_ARM_T (defined later) so it lives further down.

# ---- Servo (actuator) ----------------------------------------------------
# Generic 25 kg-cm digital servo (DS3225, MG996R, etc.).  The body is a
# rectangular brick; mounting tabs stick out on the +/-X faces; output
# shaft is on the +Z face, offset from centre.  The horn screws onto a
# 25-tooth spline with M3 fastener.
SERVO_BODY_W      = 40.0   # mm -- length of the servo body (along output-shaft offset)
SERVO_BODY_D      = 20.0   # mm -- depth (perpendicular)
SERVO_BODY_H      = 38.0   # mm -- height of the body (without output gear)

# --- Wire-exit boot on the servo body --------------------------------------
# Measured from a real DS3225: the 3-wire harness emerges from a rectangular
# molded boot on the BODY'S +X SHORT FACE (the same X-end as the output gear,
# which itself sits at +SERVO_OUTPUT_X), at the BOTTOM of the case.
#
# Boot geometry in servo-local coords (origin = bottom face centre,
# +X = output-offset direction, +Y = body short axis, +Z = output-shaft up):
#
#     boot footprint on the +X body face:
#         Y span : +/- WIRE_BOOT_W / 2  (centred on y = 0)
#         Z span : [WIRE_BOOT_Z_BASE, WIRE_BOOT_Z_BASE + WIRE_BOOT_H]
#     boot extrudes OUT in +X by WIRE_BOOT_PROTRUSION from the body face
#         (so boot occupies x in [+SERVO_BODY_W/2, +SERVO_BODY_W/2 + WIRE_BOOT_PROTRUSION]).
#
# The boot is included in ``make_servo_body`` so any render / verification
# script that uses the visual envelope can SEE which side of the body the
# wire harness emerges from, eliminating the recurring confusion that put
# the wire-exit slot on the WRONG side of the well in earlier revisions.
WIRE_BOOT_W           = 7.0    # mm -- boot Y width  (centred on y = 0)
WIRE_BOOT_H           = 3.9    # mm -- boot Z height
WIRE_BOOT_Z_BASE      = 4.1    # mm -- boot lower-edge Z above body base
WIRE_BOOT_PROTRUSION  = 6.5    # mm -- how far the boot sticks out in +X
SERVO_OUTPUT_H    =  6.0   # mm -- output gear stack above the body face
SERVO_OUTPUT_OD   = 10.0   # mm -- top of the output gear (visual)
SERVO_SPLINE_OD   =  6.0   # mm -- 25T spline diameter (M3 horn screw lives in this)
SERVO_OUTPUT_X    = 10.0   # mm -- output shaft offset from centre, toward +X
SERVO_TAB_W       = 54.0   # mm -- tip-to-tip across the two mounting tabs
SERVO_TAB_T       =  2.5   # mm -- thickness of the mounting tabs
SERVO_TAB_HOLE    =  3.2   # mm -- M3 clearance hole in each tab (clearance for the
                            #      M3 screw to pass through the tab itself, drilled
                            #      from above by the user; not modelled in our STL)
SERVO_TAB_HOLE_PCD = 49.5  # mm -- centre-to-centre distance between tab holes
                            #      (along the X axis; 2 holes per tab x 2 tabs = 4 holes)
SERVO_TAB_HOLE_PCD_Y = 10.0   # mm -- centre-to-centre between the 2 holes on the
                            #      same tab (Y / depth direction)
SERVO_TAB_Z       = 27.0   # mm -- height of the tab plane above the body bottom
                            #      (DS3225/MG996R: tabs are ~10 mm down from the top face)
SERVO_PILOT_OD    =  2.5   # mm -- M3 self-tapper PILOT hole drilled into the well
                            #      wall.  The standard servo M3 self-tapper threads
                            #      directly into a 2.5 mm pilot in PA12 / PLA.

# ---- Servo wire-exit slot ------------------------------------------------
# On DS3225 / MG996R / DS3218-class hobby servos the 3-wire harness is
# rigidly moulded into the back-case at the BOTTOM-OUTBOARD corner of
# the body -- specifically the corner where the back case meets the -X
# short-end face (the end OPPOSITE the output spline).  The bundle is
# glued in place, so the servo physically cannot seat in the cradle
# unless that exact corner is open.  The harness then leaves the cradle
# either by routing straight DOWN out of the corner (through the well
# floor) or by routing HORIZONTALLY out the body's -X short-end face
# (through the bottom of the -X wall).  To support both escape paths
# with one boolean cut we put an L-shaped channel at the well's -X
# bottom-outboard corner -- a single rectangular box that punches
# through the floor at the body's -X end AND through the bottom of
# the -X wall above it.
#
# Slot geometry (well-local frame -- see _wire_exit_slot for details):
#     X / Z: use WIRE_SLOT_X_INBOARD, WIRE_SLOT_X_PAST_WALL, WIRE_SLOT_DEPTH,
#            WIRE_SLOT_Z_BELOW_FLOOR for a larger exit than the old 12 x 6 x 8.5
#            mm box so bundled + jacketed harnesses fit.
#     Y: WIRE_SLOT_W stays ~6 mm (symmetric) so M3 pilots at well-y = +/-5 keep
#        ~1+ mm clearance; open more in X/Z instead.
#     Stays well below WELL_RIM_Z so tab-screw seats stay solid.
WIRE_SLOT_W     = 7.0   # mm wide along the body SHORT axis (well Y).  Pilot
                        # columns at y = +/-5 have radius 1.25 mm, so their
                        # inner edges sit at y = +/-3.75; a 7 mm slot has its
                        # edges at y = +/-3.5, leaving 0.25 mm of Y clearance
                        # to the pilots.  Going wider than 7.5 mm starts
                        # punching into the M3 thread engagement zone.
WIRE_SLOT_DEPTH = 13.5  # mm reach UP into the +X wall above the cavity
                        # floor.  Sized so the WIRE_BOOT_* envelope plus
                        # any bent harness above it clears the slot top.
                        # Still well below WELL_RIM_Z (27.25), so M3
                        # pilot tab seats at the rim are untouched.
WIRE_SLOT_X_PAST_WALL = 4.0   # mm beyond outer +X well face for exit
WIRE_SLOT_X_INBOARD   = 2.5   # mm inside cavity +X body face for opening
WIRE_SLOT_Z_BELOW_FLOOR = 4.0  # mm below outer floor for thick boots

# Vertical channel cut into the inside surface of the +X cavity wall so the
# 3-wire harness has a place to lie flat against the wall as it descends to
# the L-shaped exit at the bottom (instead of being pinched between the
# servo back-case and the wall and squirting out over the well rim).
# Pilot clearance: the M3 pilots at (x = +/-24.75, y = +/-5) have ~1.25 mm
# radius columns, so their inner Y edges sit at y = +/-3.75.  The channel
# only spans y = +/-WIRE_SLOT_W/2 (= +/-3.5 with WIRE_SLOT_W = 7), so it
# stays clear of the pilots in Y even when deepened further in X.
#
# Depth math with WIRE_CHANNEL_DEPTH = 5.0:
#   cavity face at +SERVO_BODY_W/2 + WELL_BODY_CL = +20.7;
#   channel outer x = +20.7 + 5.0 = +25.7;
#   pilot at x = +SERVO_TAB_HOLE_PCD/2 = +24.75, pilot radius 1.25 mm so
#     pilot outboard X edge sits at +26.0;
#   channel (y in +/-3.5) ends 0.3 mm shy of pilot outboard X edge in X
#     AND is fully separated from pilots in Y (channel y_max 3.5 vs pilot
#     y_min 3.75) -- the two never touch in 3D.
#   Wall thickness over the pilot threads (face-to-pilot distance):
#     +WELL_W/2 = +29 to pilot at +24.75 = 4.25 mm -- still >= 4 mm.
WIRE_CHANNEL_DEPTH    = 5.0   # mm groove depth INTO the +X wall material
WIRE_CHANNEL_TOP_OVER_RIM = 2.5  # mm extension of the channel ABOVE the well
                                  # rim, so wires exiting near the top of the
                                  # back-case (micro servos) still find it.

# ---- Servo well (open-topped bucket holding ONE servo) -------------------
# The well is the structural pocket that the servo drops into during
# assembly:
#
#     1.  The servo body slides DOWN through the well's wide-open top.
#     2.  Its mounting tabs land on the well's rim at z = WELL_RIM_Z.
#     3.  The user drives 4 standard M3 self-tapping screws DOWN through
#         the tab clearance holes into 4 pilot holes drilled vertically
#         through the well's side walls (positioned at x=+/-SERVO_TAB_
#         HOLE_PCD/2, y=+/-SERVO_TAB_HOLE_PCD_Y/2).
#     4.  Above the rim the body extends another (BODY_H - TAB_Z) mm into
#         open air; the gear stack and horn adapter sit on top of that.
#
# The +X / -X walls (where the M3 pilots live) MUST be thick enough to
# hold the pilot hole AT a comfortable distance inside the wall material.
# With WELL_WALL_X = 9 mm: pilot at x = +/-24.75, wall outer at x =
# +/-29, so 4.25 mm of material sits between the pilot and the outer
# face -- plenty for FDM in PLA / MJF in PA12.
WELL_WALL_X  = 9.0   # mm thick on +X / -X faces (must hold the M3 pilot)
WELL_WALL_Y  = 4.5   # mm thick on +Y / -Y faces.  Was 2.5 mm, which gave a
                     # FINAL printed wall of WELL_WALL_Y - WELL_BODY_CL = 1.8 mm
                     # after the body cavity is cut (the inner face of the
                     # wall sits 0.7 mm inboard of the SERVO_BODY_D/2 line to
                     # leave drop-in clearance for the body).  With FDM
                     # nozzle = 0.4 mm we need >= 3 mm of solid wall to fit
                     # three perimeters with margin.  4.5 mm gives 3.0 mm
                     # of real wall AT THE RIM (where the WELL_LEAD_IN_EXTRA
                     # = 0.8 mm chamfer eats an additional 0.8 mm of wall
                     # over the top 1.8 mm of the cavity) and 3.8 mm of wall
                     # everywhere else, comfortably above the three-perimeter
                     # threshold even with FDM line-width variation.
WELL_FLOOR_T = 3.0   # mm bottom-plate thickness.  Was 2.5 mm which printed as
                     # a 2.5 mm-thick frame strip in Z; 3.0 mm gives the
                     # FDM-friendly three-perimeter minimum.  The plate exists
                     # only as a perimeter FRAME around the (open) cavity
                     # opening, tying the +X/-X tab posts to the +Y/-Y walls
                     # at the bracket's print-bed face.  The servo body does
                     # NOT rest on the floor -- it hangs from its mounting
                     # tabs on the rim, with WELL_TAB_FLOAT mm of clearance
                     # from the nominal floor plane (and the cavity itself
                     # punches all the way through the floor, so a body that
                     # exceeds the nominal SERVO_TAB_Z depth simply pokes out
                     # the bottom of the bracket instead of bottoming the tab
                     # above the rim).
WELL_TAB_FLOAT = 1.5  # mm float distance between body bottom and the nominal
                      # cavity floor (z = 0) when the tabs are seated on the
                      # rim.  Was 0 (the body sat on the floor at the same
                      # instant the tabs reached the rim, so any extra body
                      # height left the tabs hanging above the rim).  Bumping
                      # this means the rim, not the floor, defines seating
                      # depth.
WELL_W       = SERVO_BODY_W + 2 * WELL_WALL_X     # 58 mm
WELL_D       = SERVO_BODY_D + 2 * WELL_WALL_Y     # 25 mm
WELL_RIM_Z   = SERVO_TAB_Z - SERVO_TAB_T / 2.0 + WELL_TAB_FLOAT  # mm: rim
                                                   # sits at the tab bottom
                                                   # face when the body is
                                                   # floating WELL_TAB_FLOAT
                                                   # mm above the floor.
WELL_H       = WELL_RIM_Z + WELL_FLOOR_T          # mm: outer height
WELL_BODY_CL = 0.7   # mm clearance on every body face inside the well.
                     # FDM in PLA / PETG can swallow ~0.3 mm per side just in
                     # line-width / shrinkage, so 0.4 mm is too tight for a
                     # drop-in fit on most desktop printers.  0.7 mm leaves
                     # 0.4 mm of real-world wiggle without the body rattling.

# ---- "Drop-in" assembly features ----------------------------------------
# Two changes to the otherwise solid bucket geometry that make seating the
# servo body MUCH easier without compromising structural function:
#   1. Finger-access notches cut through the +Y / -Y walls above a short
#      bottom pocket.  The bottom pocket guides the servo into the cavity;
#      the notches let you reach in from the sides with thumb + forefinger
#      to pinch / wiggle the body the rest of the way down.  The notches
#      do NOT touch the +X / -X walls (where the M3 pilots and tab seats
#      live), so the bracket's structural posts are unchanged.
#   2. A short lead-in chamfer at the top of the cavity that opens the
#      mouth slightly wider than the body so the body can self-align on
#      its way down rather than catching on a sharp inside corner.
WELL_BOTTOM_POCKET_H = 6.0   # mm of full +Y/-Y wall below the notch (guides
                              # the bottom of the servo body into the cavity)
WELL_FINGER_NOTCH_W  = 24.0  # mm wide (along body long axis X) — wide enough
                              # to pinch the body but narrow enough to leave
                              # ~8 mm of wall material at each +X/-X tab post
WELL_LEAD_IN_H       = 1.8   # mm tall rim chamfer
WELL_LEAD_IN_EXTRA   = 0.8   # mm extra clearance per side at the very top of
                              # the cavity, tapering down to WELL_BODY_CL

# ---- Coxa bracket (yaw-motor housing) -----------------------------------
# A horizontal flange that bolts to the chassis edge plus a servo well
# that hangs from it.  The yaw axis (the output spline of the servo)
# coincides with the chassis hexagon's apothem line, so the whole bracket
# is rotationally symmetric about the chassis perimeter.
BRACKET_FLANGE_T   = 15.0   # mm thick mounting flange.  Was 4 mm; bumped to
                            # 15 mm so the body+tab clearance is a one-sided
                            # POCKET cut into the FLANGE BOTTOM (z in
                            # [0, +12.5]) rather than a through-hole, leaving
                            # a solid 2.5 mm top cap above the entire body
                            # footprint.  Only a small Phi 11 mm gear /
                            # spline clearance hole punches through the top
                            # of the cap, so the chassis-bolt corners and
                            # the perimeter of the bracket are tied together
                            # by a continuous slab of material instead of
                            # two disconnected (+Y, -Y) half-pads with a
                            # rectangular gap between them.  Greatly stiffer
                            # in torsion.  Cascades:
                            #   * COXA_LIFT bumped 14 -> 26 (femur hip pad
                            #     must still clear the taller flange top).
                            #   * Chassis bolt screws need to be M3 x 32
                            #     instead of M3 x 20 (an extra ~ 12 mm of
                            #     flange material to reach through).
BRACKET_FLANGE_X   = 30.0   # mm long (radial -- inboard from chassis edge)
BRACKET_FLANGE_Y   = 52.0   # mm wide (tangential).  Sized so the slot and
                            # the four chassis bolt holes each have >= 4 mm
                            # of material around them.  Was 48 mm; bumped to
                            # 52 mm when BRACKET_BOLT_PCD_Y grew from 36 to
                            # 40 (see below) so the flange still has >= 4 mm
                            # of material outboard of each bolt-circle hole.
BRACKET_BOLT_PCD_X = 16.0   # mm centre-to-centre, inboard vs. outboard bolt pair
BRACKET_BOLT_PCD_Y = 40.0   # mm centre-to-centre, +Y vs. -Y bolt line.  Was
                            # 36 mm; bumped to 40 mm so the bolt clearance
                            # holes stay >= 2.5 mm clear of the chassis-plate
                            # body+tab cutout, whose Y span now grows to
                            # WELL_D + 2 = 31 mm with WELL_D = 29 mm.  At
                            # BRACKET_BOLT_PCD_Y = 36 the bolt holes would
                            # have come within 0.8 mm of the cutout edge --
                            # not enough chassis material around the bolt
                            # head's bearing surface.
BRACKET_BOLT_HOLE  =  3.4   # mm clearance for M3 chassis bolts
BRACKET_FLANGE_INSET = 8.0  # mm distance from the chassis edge (= bracket
                            # origin's outboard face) to the OUTBOARD bolt
                            # line.  >= BRACKET_BOLT_HOLE so the bolt is on
                            # solid chassis material, not in mid-air.

# ---- Servo horn adapter --------------------------------------------------
# A short 4-arm star that screws onto the servo's plastic horn (M3
# centre screw) and presents a 4 x M3 bolt pattern (20.8 mm PCD) on its
# top face.  The flat printed links bolt to this top face, so the link
# itself never touches the servo spline.
HORN_ADAPTER_OD     = 32.0   # mm -- plate OD; gives (32 - 20.8) / 2 - 1.6
                              # = 4.0 mm wall outboard of each M3 bolt
                              # hole on HORN_BOLT_PCD = 20.8 mm
HORN_ADAPTER_T      =  4.0   # mm -- thickness
# Hobby servo plastic horn (the part that ships with the servo and screws
# onto the 25T spline with an M3 centre screw).  Adds ~5 mm of "stack"
# along the output-shaft direction between the servo's gear-stack top
# face and the printed horn adapter's bottom face.
PLASTIC_HORN_H      =  5.0
# Plastic 4-arm X-shaped horn (DS3225 / MG996R / DS3218 -class hardware):
# the arms extend ~19 mm from the spline centre, so the horn sweeps a
# Phi 38 mm cylinder as the servo rotates.  This is BIGGER than the
# HORN_BOLT_PCD = 20.8 mm bolt circle (radius 10.4 mm): each arm runs
# past the bolt position to give the user a thumb-purchase for hand-
# tightening + spare hole positions.  Drives:
#   * ``make_servo_horn`` arm length (the visual mesh's bounding
#     cylinder reflects the real-hardware sweep, so any verifier that
#     reads the mesh extents picks up the correct horn radius).
#   * ``check_horn_sweep_clearance`` in _verify_prototype.py (the
#     cylinder void that the bracket's flange / walls must respect
#     above the seated servo, so the horn can rotate freely without
#     clashing with printed material).
# History: an earlier draft used a 20 mm-long arm (tip at radius
# 10 mm = INSIDE the bolt circle) which silently understated the
# sweep radius by ~9 mm.  The horn-sweep clearance test would have
# missed the recurring "the servo motor doesn't stick out high
# enough" failure if it had read that mesh's bounding cylinder.
PLASTIC_HORN_X_TIP_R = 19.0
# Total height of the servo output stack (plastic horn + printed
# adapter) ABOVE the spline tip.  In the hip-pitch and knee-pitch
# joints this is the offset between the joint AXIS (where the spline
# pokes out of the servo body) and the link's pad MATING face (where
# the link bolts to the adapter).  Driven links must offset their pad
# along the joint axis by this much, then bridge the gap back to the
# spar with a short "neck" -- without this offset the link's pad sits
# directly on the joint axis and overlaps the cradle's "swept volume"
# (the bridge cap / well wall material right above the body).
HORN_STACK_H        = PLASTIC_HORN_H + HORN_ADAPTER_T
# Bolt circle for the plastic horn's 4-arm X-shaped output disc.  20.8 mm
# is the PCD of the SECOND hole position out from the spline on each arm
# of a standard DS3225 / MG996R / DS3218 plastic horn.  The bolt angles
# match the horn arms (0 / 90 / 180 / 270 deg in horn-local coords) so a
# bolt drilled through HORN_BOLT_ANGLES_RAD[i] on the printed adapter
# passes straight through the matching hole in the X arm.
HORN_BOLT_PCD       = 20.8   # mm -- 4 x M3 holes on this PCD
HORN_BOLT_ANGLES_RAD = (0.0, np.pi / 2.0, np.pi, 3.0 * np.pi / 2.0)
HORN_BOLT_OD        =  3.2   # mm -- M3 clearance
HORN_CENTRE_OD      =  3.4   # mm -- M3 centre clearance (for the horn screw)
# Counter-bore for the plastic horn body that sits below the printed
# adapter's bottom face.  Sized so the recess wall ends INSIDE the bolt
# circle (recess radius < HORN_BOLT_PCD/2 - HORN_BOLT_OD/2 - margin =
# 10.4 - 1.6 - 0.4 = 8.4 mm), so the recess never punches into the
# adapter's bolt threads.  Big enough to swallow a typical hobby-horn
# central disc (Phi 12 mm OD on DS3225-class hardware).
HORN_RECESS_OD      = 16.0   # mm
HORN_RECESS_DEPTH   =  1.6   # mm

# ---- Link cross-sections -------------------------------------------------
# All printed in PLA / PETG.  The femur and tibia are flat plates that
# lie in the leg's plane of motion (X-Z plane in leg-local coords) so
# they're stiff against vertical bending.  In LINK-local coords:
#     +X = spar long axis
#     +Y = joint-axis direction (the link's THICKNESS direction;
#          this is also the servo output-shaft direction at each joint)
#     +Z = perpendicular to spar in the leg's motion plane (the spar's
#          structural HEIGHT)
# The link's hip end is a square pad centred on the joint axis (x=0,
# z=0) that contains the 4 horn bolts.  The link rotates rigidly with
# the horn, so the bolt-circle centre MUST coincide with the joint
# axis or the bolts can't physically stay attached.
LINK_THICKNESS   =  6.0   # mm -- Y-direction thickness of every link
FEMUR_SPAR_H     = 34.0   # mm -- Z-direction height of the femur spar.
                          # Must be > SERVO_BODY_D + 8 so the spar's top
                          # and bottom flanges remain after we cut a slot
                          # through it for the knee servo's body to slide
                          # through during assembly.
                          # Was 30 mm -> 4 mm of flange material on each
                          # side of the 22 mm-tall insertion slot, i.e.
                          # 4 mm-tall bridges joining spar to knee well.
                          # Bumped to 34 mm -> 6 mm flanges (50 % taller,
                          # ~ 3.4x stiffer in out-of-plane bending since
                          # I scales with t^3 for a flange in bending).
                          # The hip pad is 2*HIP_PAD_R = 34 mm in Z, so
                          # the spar now exactly fills the hip-pad Z
                          # extent -- no step at the hip end.
TIBIA_SPAR_H     = 18.0   # mm -- Z-direction height of the tibia spar
# Pad radius.  Must satisfy:
#
#   HIP_PAD_R >= (HORN_ADAPTER_OD/2 + HORN_STACK_CLEARANCE)
#                + MIN_PRINTABLE_NECK_WALL_T
#
# so the hollow-annulus neck (see make_femur_link / make_tibia_link, "neck
# torus" block) has a printable wall around the horn-stack clearance void.
# With HORN_ADAPTER_OD = 32 mm, HORN_STACK_CLEARANCE = 0.5 mm (matching
# check_horn_stack_clearance in _verify_prototype), and a 3.0 mm wall the
# minimum is 16.5 + 3.0 = 19.5 mm.  A SHRUNK pad (HIP_PAD_R = 17, the old
# value) leaves only 0.5 mm of wall, which:
#   - cannot be FDM-printed reliably at 0.4 mm nozzle (< 2 perimeters), AND
#   - forces the "neck" to be a SOLID cylinder occupying the same volume
#     as the plastic horn + horn adapter, so the femur sits 4 mm proud of
#     the adapter and the 4 M3 clamp bolts can't engage their threads --
#     the failure the user described as "the femur doesn't let the end of
#     the servo stick out high enough to connect to the tibia link".
#     check_horn_stack_clearance now FAILS the build if this happens.
# Bumping to 19.5 mm of course increases the pad's swept disk diameter
# (from 34 mm to 39 mm) which means the pad reaches further toward the
# coxa_link arm's bottom face.  See COXA_LIFT and WELL_Z_DROP_EXTRA below
# for the matching coxa-link geometry shift that keeps the pad clear of
# the arm + bracket flange across the runtime hip-pitch range.
HIP_PAD_R        = 19.5

# ---- Coxa-link bridge stiffener -----------------------------------------
# The coxa link's "bridge" -- the flat 4 mm-thick arm + 6.5 mm-tall
# bridge member that connects the horn-mating yoke (the inboard 4-bolt
# hub pad that clamps onto the yaw servo's horn adapter) to the
# hip-pitch servo cradle at the outboard end -- carries up to ~2.5 N*m
# of hip-pitch reaction torque about the link's local +Y axis.  In that
# bending direction the arm is the THIN dimension: only 4 mm of Z height
# at the +X end of the bridge (just inboard of the cradle's outer +X
# wall), giving I_y = 22*4^3/12 = 117 mm^4, sigma_max = 2.5 N*m * 2 mm /
# 117 mm^4 = 43 MPa -- a 1.2x safety factor against PA12's 50 MPa yield
# and visibly flexible under hand fit-check load.  For comparison the
# femur's tall 30 mm spar runs sigma_max ~ 3 MPa in the same bending
# direction (SF ~ 18x).
#
# Adding a thin gusset hanging BELOW the arm in the bridge region
# extends the cross-section's Z range to ~8 mm, raising I_y to
# ~570 mm^4 and dropping sigma_max to ~20 MPa (SF ~ 2.5x).  Geometric
# clearance constraints:
#   - Inboard X start >= HIP_PAD_R + 1 mm so the gusset doesn't clip
#     the horn adapter's 32 mm OD circular footprint underneath the hub.
#   - Z depth <= servo-body-top clearance so the gusset's bottom face
#     stays above the hip-pitch servo body's top face during insertion
#     (with arm_t = 6 mm and the well dropped by -(WELL_D/2 + arm_t/2)
#     = -15.5 mm, the seated body top sits at link-z = -5.5 mm; a 5 mm
#     gusset puts the bottom face at -5.0, leaving 0.5 mm of clearance).
#   - Outboard X end stays inside the arm's +X tail (link-x <= 41) so
#     the gusset doesn't overhang past the cradle's +X wall.
#   - Far from the wire-exit slot (which lives at link-x in [-15, -4]).
COXA_BRIDGE_GUSSET_H = 5.0   # mm -- gusset depth in -Z below arm bottom.
                             # Was 4 mm (matched the old arm_t = 4 mm).
                             # With arm_t = 6 mm the seated servo body
                             # top drops to link-z = -5.5, so we can go
                             # 1 mm deeper without violating insertion
                             # clearance.
COXA_BRIDGE_GUSSET_L = 23.0  # mm -- gusset length along the spar +X

# Coxa-link arm thickness in Z.  Originally 4 mm, raised to 6 mm to
# bring the arm's I = arm_w * arm_t^3 / 12 up by 3.4x for cantilever
# bending stiffness.  Bumping to 8 mm would clear the
# ``check_thin_sheets`` verifier directly but drops the servo-well
# by 1 mm (-> coxa_link clips the electronics tray in the
# workspace sweep) and squeezes the femur-pad-vs-bracket-flange
# margin from 1.25 mm to 0.25 mm.  Instead we keep arm_t = 6 and
# stiffen the +Y side of the arm with a top cap rib in
# ``make_coxa_link`` (see COXA_ARM_CAP_T below).
COXA_ARM_T              = 6.0    # mm -- arm slab thickness in +Z.

# Top cap rib stacked ABOVE the arm slab on the +Y side of the arm
# only.  Covers link y in [pad_y_min, arm_w/2] and link x in
# [hub edge, arm +X end].  Stays clear of the femur spar (which
# sits at link y in [-3, +3] at any pitch) and stays above the
# femur hip pad's max Z reach (pad swept disk reaches link z =
# hip_drop + HIP_PAD_R = -3.5 + 17 = +13.5 mm lifted, well below
# the cap bottom at z = arm_t + COXA_LIFT = 20 mm lifted).
#
# What this fixes: the ``check_thin_sheets`` cluster identifies the
# arm slab's +Y portion (cluster centroid at y = +1.2 mm, bbox y
# from -8.4 to +10.8 mm) as a 6 mm-thick (chord_z = 7.2 mm
# voxelized) structural neck between the yaw hub and the hip-pitch
# servo well.  Adding a cap on the +Y side raises chord_z to arm_t
# + COXA_ARM_CAP_T = 10 mm in the cap region.
#
# What this leaves unfixed: the y in [-3, +3] strip of the arm
# (where the femur spar swings through at extreme pitches) is NOT
# capped; a top cap there would collide with the spar at fem = -80
# deg (the firmware's negative-pitch joint limit).  The cluster's
# +Y half is fixed by the cap and the -Y half is already covered
# by the existing bridge / gusset structure (chord_z >= 10 mm in
# the bridge region).  The remaining "uncapped strip" voxels
# represent ~5/16 of the original cluster -- below the
# ``MAX_SHEET_BUDGET_VOX`` = 250 voxel budget.
COXA_ARM_CAP_T          = 4.0    # mm -- cap thickness in +Z above
                                  # arm.  Cap top at z = arm_t +
                                  # COXA_ARM_CAP_T = 10 mm unlifted,
                                  # lifted to z = COXA_LIFT + 10 =
                                  # 24 mm.

# ----- Well-top-wall thickening pad (coxa_link only) ---------------------
# Why this exists.  In the BASELINE ``make_coxa_link`` geometry the
# bridge slab that ties the horn yoke (top hub) to the hip-pitch servo
# well box overlaps the well's outer body by only 0.5 mm in Y (the
# 0.5 mm of bridge that punches past the well's outer +Y face) and
# 1.5 mm in Z (the 1.5 mm of bridge below the well's outer top face),
# giving a 53 x 0.5 x 1.5 = 40 mm^3 bonded interface holding the
# entire leg load.  ``pad_sweep_clear`` -- the cylindrical void that
# lets the femur hip pad swing through the link's interior -- then
# eats a circular hole through the bridge across most of its X span,
# leaving 19 mm^2 of bridge cross-section at the worst Z slice (just
# above the well top face, where the cylinder cut is widest).  The
# user has complained about this exact failure mode multiple times
# ("the top of coxa_link is not attached strongly to the part housing
# the servo, theres an obvious fix of thickening the wall on the
# motor housing so a bigger surface area attaches to the top piece").
#
# This pad is the user's preferred fix.  It's a rectangular block of
# plastic that sits ON TOP of the well's outer top face and rises up
# into the bridge gap, fusing both with the well's outer body (below)
# and the existing bridge / arm (above).
#
# Sizing notes (May 18 2026 revision).  An earlier 5 mm version of the
# pad left a 4.25 mm-wide safe spine outside the ``pad_sweep_clear``
# cylinder's y range of [-18, +18] -- BUT the bridge + pad at x ~
# COXA_LENGTH = +25 mm (where the cylinder cuts the deepest into the
# bridge gap) collapsed to a ~4 mm-thin vertical strip that the user
# flagged as visually + structurally "wrong wall thickened".  See the
# user-pointed screenshot at hexapod_walker/prototype/renders/
# _user_pointed_thin_neck.png and the ASCII diagnostic in
# _find_thin_neck.py (YZ slice at x = +22, x = +25): the surviving
# spine in the middle X range was only 4 mm wide in Y for a 6 mm-tall
# vertical column, which neither the original XY-slice bridge check
# nor casual inspection picked up.
#
# The fix is to extend ``WELL_TOP_PAD_Y_EXT`` to 13.25 mm so the pad's
# far-Y edge lands at y = -32 (= bridge_y_min - 13.25, = well_near_y -
# 13.75).  The pad's far-Y edge stays comfortably INSIDE the well's
# outer footprint (well's far-Y face is at y = -46.5) and OUTSIDE the
# cavity (the cavity sits at y in [-44, -6], BUT at z in [+2.5,
# +22.5] -- well below the pad's z range of [+23.5, +32], so the pad
# does not intrude into the cavity).  The new pad supplies ~14 mm of
# surviving safe spine in Y at every x in the bridge x range, holding
# the well to the bridge / arm / hub through ~14 x 8.5 = 119 mm^2 of
# cross-section at the cylinder-cut middle (vs. ~24 mm^2 with the
# 5 mm pad).
#
# Verified by ``_check_coxa_link_bridge_joint`` AND
# ``_check_coxa_link_bridge_yz_thickness`` in _verify_prototype.py
# (called as part of ``check_flimsy_joints``).  The YZ-thickness
# check FAILS on the 5 mm pad geometry (min YZ-area = 40 mm^2 at x =
# +22) and PASSES on the 13.25 mm pad.
WELL_TOP_PAD_Y_EXT = 13.25 # mm -- distance the pad extends PAST the
                            # bridge's existing -Y face into -Y
                            # direction (= INTO the well's outer body
                            # footprint).  See big docstring above.

# ---- Derived hip-pitch geometry ----------------------------------------
# Hip-pitch axis Z position in coxa-link local frame, AFTER lift.  Equal
# to (well_z_drop + COXA_LIFT) inside make_coxa_link, where
# well_z_drop = -(WELL_D/2 + COXA_ARM_T/2 + WELL_Z_DROP_EXTRA).  Exposed
# as a module-level constant so downstream callers (workspace tests,
# assemblies, renders, integrators) don't have to duplicate the formula
# (which got out of sync when WELL_Z_DROP_EXTRA was introduced).
COXA_HIP_DROP = (-(WELL_D / 2.0 + COXA_ARM_T / 2.0 + WELL_Z_DROP_EXTRA)
                 + COXA_LIFT)

# ---- Foot ----------------------------------------------------------------
# Compliant pad printed in TPU.  The tibia tip ends in a forked CLEVIS
# (two parallel cheeks, knee-axis-parallel through-hole) and the foot
# has a vertical TONGUE that drops into the fork; an M3x16 pan-head
# bolt with a nylock nut clamps the two together.  The bolt becomes
# the pin of a single-axis hinge whose axis is parallel to the knee
# pitch axis, so the foot pitches passively around it to follow
# uneven ground.  TPU material compliance in the pad disk itself
# absorbs roll.
FOOT_PAD_OD          = 28.0   # mm -- outer diameter of the ground-contact disk
FOOT_PAD_BASE_H      =  4.0   # mm -- thickness of the disk (TPU spring)
FOOT_PAD_BOSS_OD     = 14.0   # mm -- short stiffening boss between disk top
                              #        and tongue (gives the tongue a wider
                              #        root than its 10 x 4 mm cross-section)
FOOT_PAD_BOSS_H      =  3.0   # mm -- boss height; tongue starts at
                              #        FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H

# Hinge geometry (shared between tibia clevis and foot tongue).
FOOT_HINGE_CHEEK_T   =  3.5   # mm -- each tibia clevis cheek thickness in
                              #        the knee-axis direction (Y).  Above
                              #        MIN_PRINT_T = 3.0 mm with margin.
FOOT_HINGE_GAP       =  5.0   # mm -- inside-fork clearance (between the
                              #        cheeks' inner faces) for the tongue.
                              #        Total fork width = 2*CHEEK_T + GAP =
                              #        12 mm in Y.
FOOT_HINGE_TONGUE_T  =  4.0   # mm -- foot tongue thickness in Y.  Fits in
                              #        the 5 mm gap with 0.5 mm clearance
                              #        per side; allows the M3 bolt to pinch
                              #        the cheeks lightly on the tongue
                              #        without binding when articulated.
FOOT_HINGE_TONGUE_X  = 10.0   # mm -- tongue width along the foot's X axis
                              #        (the spar direction).  Sized so the
                              #        M3 hole has > 3 mm of material on
                              #        each side of the bore axis in X.
FOOT_HINGE_PIN_HOLE_D =  3.4  # mm -- through-hole diameter for the M3
                              #        hinge bolt; 0.2 mm clearance over a
                              #        nominal 3.2 mm M3 shank so the joint
                              #        rotates freely.
FOOT_HINGE_PIN_LEN    = 16.0  # mm -- pin length specification (M3 x 16
                              #        pan-head + M3 nylock nut).  Stack
                              #        = 3.5 + 5 + 3.5 = 12 mm of clevis
                              #        plus ~4 mm into the nylock nut.

# Hinge axis Z position in tibia local: 10 mm below the tibia spar
# centreline (= 1 mm below the spar's bottom face at z = -9).  This
# pulls the cheeks down past the spar so the fork's mouth opens
# clearly into free space and the foot's tongue has room to swing.
FOOT_HINGE_TIBIA_Z   = -10.0  # mm -- pin axis z in tibia-local

# Hinge axis Z position in foot-local: 14 mm above the disk bottom (=
# 14 mm above the ground when the foot stands).  Computed downstream
# in make_foot_pad() so the tongue reaches the pin with a few mm of
# material above the hole and the foot pad disk sits below the
# tibia's clevis.
FOOT_HINGE_FOOT_Z    = FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H + 7.0   # = 14.0
FOOT_HINGE_TONGUE_OVER_PIN = 4.0  # mm of tongue material above the pin

# Tibia clevis bulk extent.  The clevis is a forked block at the
# spar's far end (around x = TIBIA_LENGTH); the FOOT_CLEVIS_X_*
# constants set how far the block extends inboard / past the spar
# tip and how far the cheeks reach below the pin axis.
FOOT_CLEVIS_X_INBOARD     = 12.0  # mm -- clevis bulk inboard of x=TIBIA_LENGTH
FOOT_CLEVIS_X_BEYOND_TIP  =  6.0  # mm -- bulk extending past x=TIBIA_LENGTH
FOOT_CLEVIS_CHEEK_BELOW_PIN = 5.0 # mm -- material below the pin axis in
                                  #        each cheek (gives a ~1.5 mm
                                  #        ring of material around the M3
                                  #        hole on the bottom side)

# ---- Battery / electronics enclosures ------------------------------------
# Sized for a generic 3S 2200 mAh LiPo (105 x 35 x 25 mm) plus an
# Arduino Nano or Mega + PCA9685 stack.
BATTERY_W = 110.0   # mm
BATTERY_D =  38.0
BATTERY_H =  28.0
BATTERY_WALL = 1.6
BATTERY_STRAP_W = 10.0   # velcro slot width

ELEC_TRAY_W = 100.0
ELEC_TRAY_D =  70.0
ELEC_TRAY_T =   3.0

# ---- Resolutions ---------------------------------------------------------
CYL_SECTIONS = 48     # cylinder facet count -- smooth STL, fast booleans

# ---- Standing pose used for the assembly preview ------------------------
# Same conventions as hexapod_walker.py: pitch angles measured from the
# coxa link's local horizontal X axis.
STANCE_FEMUR_DEG = -25.0
STANCE_TIBIA_DEG =  60.0

# Output directory -- next to this script
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "stl_prototype")
os.makedirs(STL_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _cyl(radius: float, height: float, *,
         sections: int = CYL_SECTIONS) -> trimesh.Trimesh:
    """A cylinder along +Z, centred at the origin."""
    return cylinder_mesh(radius=radius, height=height, sections=sections)


def _cyl_along(radius: float, length: float, axis: str = "x",
               *, sections: int = CYL_SECTIONS) -> trimesh.Trimesh:
    """A cylinder lying along the named axis, one end at origin, the
    other at +length on that axis."""
    m = _cyl(radius, length, sections=sections)
    m.apply_translation([0, 0, length / 2.0])
    if axis == "x":
        m.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == "y":
        m.apply_transform(rotation_matrix(-np.pi / 2, [1, 0, 0]))
    elif axis == "z":
        pass
    else:
        raise ValueError(f"axis must be x|y|z, got {axis}")
    return m


def _box(extents, center=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    """Axis-aligned box with given extents (w, d, h), centred on *center*."""
    m = box_mesh(extents=extents)
    m.apply_translation(center)
    return m


def _union(*meshes) -> trimesh.Trimesh:
    """Boolean union with a manifold-friendly fallback."""
    parts = [m for m in meshes if m is not None]
    if len(parts) == 1:
        return parts[0]
    try:
        return trimesh.boolean.union(parts)
    except Exception:
        return trimesh.util.concatenate(parts)


def _diff(a: trimesh.Trimesh, *cuts) -> trimesh.Trimesh:
    """Boolean difference a - cuts, with a fallback to the original."""
    parts = [m for m in cuts if m is not None]
    if not parts:
        return a
    try:
        return trimesh.boolean.difference([a, *parts])
    except Exception:
        return a


def _save(mesh: trimesh.Trimesh, name: str) -> str:
    path = os.path.join(STL_DIR, name)
    mesh.export(path)
    n_faces = len(mesh.faces)
    extents = mesh.extents
    print(f"  wrote stl_prototype/{name:30s}"
          f"  {n_faces:>6d} faces"
          f"  envelope {extents[0]:5.1f} x {extents[1]:5.1f} x {extents[2]:5.1f} mm")
    return path


# ---------------------------------------------------------------------------
# Servo-mounting primitives
# ---------------------------------------------------------------------------

def _servo_well_solid() -> trimesh.Trimesh:
    """Open-topped servo bucket, returned as one watertight mesh in the
    well's local frame.

    Local frame (matches `_servo_envelope` *body* axes):
        Origin: centre of the body's bottom face (= TOP face of the well's
                floor plate).
        +X = body long axis (= mounting-tab span direction).
        +Y = body short axis (= depth).
        +Z = output-shaft direction.

    The well is a rectangular bucket:

        - Outer:    WELL_W x WELL_D x WELL_H, centred on (0, 0, WELL_H/2 -
                     WELL_FLOOR_T) so its floor outer face is at z =
                     -WELL_FLOOR_T and its rim is at z = WELL_RIM_Z.
        - Body cavity: open at +Z, closed at -Z by the floor.  Spans
                     (SERVO_BODY_W + 2*CL) x (SERVO_BODY_D + 2*CL) x
                     (WELL_RIM_Z + extra).  Cuts straight through the rim
                     so the body can be DROPPED in from above.
        - 4 M3 pilot holes drilled vertically through the +X / -X walls
                     at the standard tab-hole positions (Φ SERVO_PILOT_OD).
                     The walls are WELL_WALL_X = 9 mm thick on each side,
                     so the pilot has a comfortable amount of material on
                     either side of it.

    The *body* sits inside the cavity with its bottom resting on the floor
    (well-z = 0).  The mounting *tabs* land on the rim at well-z =
    WELL_RIM_Z.  The user drives 4 standard M3 self-tappers through the
    tab clearance holes and into these pilot holes."""
    outer = _box((WELL_W, WELL_D, WELL_H),
                 center=(0, 0, WELL_H / 2.0 - WELL_FLOOR_T))

    # Body cavity: punches all the way through the floor so a body that
    # exceeds the nominal SERVO_TAB_Z depth can poke out the bottom of
    # the bracket rather than bottoming out before the tabs reach the
    # rim.  Z range is [-WELL_FLOOR_T - 1, WELL_RIM_Z] -- 1 mm of overshoot
    # past the outer bottom face keeps the boolean cleanly two-sided.
    # The +X / -X tab posts and the +Y / -Y wall perimeter still join to
    # the floor plate around this opening, so the bracket stays rigid.
    cav_z_bot = -WELL_FLOOR_T - 1.0
    cav_z_top = WELL_RIM_Z
    cav_z_ext = cav_z_top - cav_z_bot
    cavity = _box((SERVO_BODY_W + 2 * WELL_BODY_CL,
                   SERVO_BODY_D + 2 * WELL_BODY_CL,
                   cav_z_ext),
                  center=(0, 0, 0.5 * (cav_z_top + cav_z_bot)))

    # 4 vertical M3 pilot holes through the side wall material, each one
    # going from above the rim down through the floor for cleanliness
    # (the screw only engages the top ~6 mm; the lower length is harmless
    # void).
    pilot_h = WELL_RIM_Z + WELL_FLOOR_T + 2.0
    pilots = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = _cyl(SERVO_PILOT_OD / 2.0, pilot_h)
            p.apply_translation([sx * SERVO_TAB_HOLE_PCD / 2.0,
                                  sy * SERVO_TAB_HOLE_PCD_Y / 2.0,
                                  pilot_h / 2.0 - WELL_FLOOR_T - 1.0])
            pilots.append(p)

    # Finger-access notches in the +Y / -Y walls (above the bottom
    # pocket).  The cut spans Y past both wall faces so it punches
    # cleanly through both side walls in one boolean, but it stops
    # short of the +X / -X tab posts in X (notch_w < cavity X span).
    notch_x = WELL_FINGER_NOTCH_W
    notch_y = WELL_D + 2.0
    notch_z_bot = WELL_BOTTOM_POCKET_H
    notch_z_top = WELL_RIM_Z + 1.0          # slice cleanly through the rim
    notch_z_ext = notch_z_top - notch_z_bot
    finger_notch = _box((notch_x, notch_y, notch_z_ext),
                        center=(0.0, 0.0,
                                0.5 * (notch_z_top + notch_z_bot)))

    # Lead-in chamfer at the top of the cavity opening: a wider box
    # sitting just below the rim that opens the cavity mouth outward by
    # WELL_LEAD_IN_EXTRA on every side so the body self-aligns into the
    # narrower cavity below as it drops in.
    lead_in = _box((SERVO_BODY_W + 2 * (WELL_BODY_CL + WELL_LEAD_IN_EXTRA),
                    SERVO_BODY_D + 2 * (WELL_BODY_CL + WELL_LEAD_IN_EXTRA),
                    WELL_LEAD_IN_H + 0.5),
                   center=(0.0, 0.0,
                            WELL_RIM_Z - (WELL_LEAD_IN_H + 0.5) / 2.0
                            + 0.25))

    return _diff(outer, cavity, finger_notch, lead_in, *pilots)


def _wire_exit_slot() -> trimesh.Trimesh:
    """Cutting volume for the L-shaped wire-exit channel at the
    bottom-OUTBOARD corner of a servo well.

    Local frame: same as ``_servo_well_solid`` / ``_servo_envelope``.

    Wire side
    ---------
    DS3225-class hobby servos route their 3-wire harness out of a
    RECTANGULAR MOLDED BOOT on the body's **+X SHORT face** -- i.e. the
    SAME X-end as the output gear (which sits at +SERVO_OUTPUT_X).  The
    boot dimensions (WIRE_BOOT_* near the top of this file) are baked
    into ``_servo_envelope`` so this is visually unambiguous in renders.

    The slot must:

      * clear the WIRE_BOOT_* protrusion (so the boot fits inside the
        well wall), AND
      * give the wire bundle two L-shaped escape paths:
          - a straight-DOWN escape through the well floor at the body's
            +X end (so a bundle that bends 90 deg can drop out the
            bottom of the cradle), AND
          - a horizontal-OUTBOARD escape through the bottom of the +X
            wall (so the bundle can route LATERALLY out of the cradle).

    A single rectangular box at the well's +X bottom-outboard corner
    does both jobs in one boolean cut.

    PRE-2026 versions of this file put the slot on the **-X side** of
    the well (opposite the output gear) on the incorrect assumption
    that the wires emerged from the back of the case.  That left the
    boot punched against solid +X wall material and the slot punched
    against solid +X cavity material -- the user could not seat the
    servo body fully in the cradle without bending or shearing the
    boot.  Fixed by mirroring the slot to the +X face that matches
    the boot.

    Use as:
        well = _servo_well_solid()
        slot = _wire_exit_slot()
        # ... apply the same R / translation to BOTH ...
        body = _diff(body, slot)
    """
    # ---- L-shaped exit at the +X bottom-outboard corner ----------------
    # X span: from the body's +X face (inboard end of the slot, slightly
    # inside the cavity so the slot opens cleanly into the cavity wall)
    # out past the well's outer +X face by WIRE_SLOT_X_PAST_WALL so the
    # bundle exits into free air.
    slot_x_min = +SERVO_BODY_W / 2.0 - WIRE_SLOT_X_INBOARD
    slot_x_max = +WELL_W / 2.0 + WIRE_SLOT_X_PAST_WALL
    slot_x_extent = slot_x_max - slot_x_min
    slot_x_centre = 0.5 * (slot_x_min + slot_x_max)

    slot_y_extent = WIRE_SLOT_W                 # centred on y = 0

    # Z span: must reach BELOW the well floor (to give the downward escape
    # path) AND reach UP at least past the top of the wire boot
    # (WIRE_BOOT_Z_BASE + WIRE_BOOT_H) plus a comfortable margin -- we use
    # WIRE_SLOT_DEPTH which already includes that headroom for a molded
    # boot + bent harness.
    slot_z_bottom = -WELL_FLOOR_T - WIRE_SLOT_Z_BELOW_FLOOR
    slot_z_top    = WIRE_SLOT_DEPTH
    slot_z_extent = slot_z_top - slot_z_bottom
    slot_z_centre = 0.5 * (slot_z_bottom + slot_z_top)

    exit_l = _box((slot_x_extent, slot_y_extent, slot_z_extent),
                  center=(slot_x_centre, 0.0, slot_z_centre))

    # ---- Vertical channel on the INSIDE surface of the +X wall ---------
    # Lets a harness that exits the BOOT laterally lie flat against the
    # inside of the +X wall on its way down to the L-shaped exit.
    # Spans from inside the cavity (so it merges seamlessly with the
    # cavity face) out to WIRE_CHANNEL_DEPTH into the +X wall.  Pilot
    # clearance: cavity face at +SERVO_BODY_W/2 + WELL_BODY_CL = +20.7;
    # channel outer x = +20.7 + WIRE_CHANNEL_DEPTH = +25.7 (default),
    # leaving ~0.3 mm of wall between the channel and the pilot at
    # x = +24.75 (pilot radius 1.25 mm so pilot outer X edge at +26.0).
    ch_x_min = +SERVO_BODY_W / 2.0 - WIRE_SLOT_X_INBOARD
    ch_x_max = +SERVO_BODY_W / 2.0 + WELL_BODY_CL + WIRE_CHANNEL_DEPTH
    ch_x_extent = ch_x_max - ch_x_min
    ch_x_centre = 0.5 * (ch_x_max + ch_x_min)

    ch_z_bottom = 0.0
    ch_z_top    = WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM
    ch_z_extent = ch_z_top - ch_z_bottom
    ch_z_centre = 0.5 * (ch_z_bottom + ch_z_top)

    channel = _box((ch_x_extent, WIRE_SLOT_W, ch_z_extent),
                   center=(ch_x_centre, 0.0, ch_z_centre))

    return _union(exit_l, channel)


def _servo_envelope() -> trimesh.Trimesh:
    """Return the bounding-volume of a hobby servo, with mounting tabs.

    Local frame (matches the way the servo is *used*):
        +Z = output-shaft direction (pointing OUT of the body)
        +X = direction the output shaft is offset toward
        +Y = the long horizontal direction of the mounting tabs

    Wait -- mounting tabs of a hobby servo project along the body's
    long axis (the same axis as the output offset).  So:
        +X = body long axis = mounting-tab direction
              (output shaft sits at +SERVO_OUTPUT_X on this axis)
        +Y = body short axis (depth)
        +Z = output shaft direction

    Origin at the centre of the body's bottom face."""
    body = _box((SERVO_BODY_W, SERVO_BODY_D, SERVO_BODY_H),
                center=(0, 0, SERVO_BODY_H / 2.0))

    # Two mounting tabs sticking out along +/-X
    tab_extra = (SERVO_TAB_W - SERVO_BODY_W) / 2.0
    for sx in (-1, 1):
        tab = _box((tab_extra, SERVO_BODY_D, SERVO_TAB_T),
                   center=(sx * (SERVO_BODY_W / 2.0 + tab_extra / 2.0),
                            0, SERVO_TAB_Z))
        body = _union(body, tab)

    # Output gear stack
    gear = _cyl(SERVO_OUTPUT_OD / 2.0, SERVO_OUTPUT_H)
    gear.apply_translation([SERVO_OUTPUT_X, 0,
                             SERVO_BODY_H + SERVO_OUTPUT_H / 2.0])
    body = _union(body, gear)

    # Output spline (small protruding shaft)
    spline = _cyl(SERVO_SPLINE_OD / 2.0, SERVO_OUTPUT_H + 1.5)
    spline.apply_translation([SERVO_OUTPUT_X, 0,
                                SERVO_BODY_H + (SERVO_OUTPUT_H + 1.5) / 2.0])

    # Wire-exit boot: rectangular molded protrusion on the +X SHORT face
    # (the same X-end as the output gear).  See WIRE_BOOT_* constants
    # near the top of this file for the measured dimensions and the
    # design-failure history that motivated baking the boot into the
    # visual envelope.
    boot_x_centre = SERVO_BODY_W / 2.0 + WIRE_BOOT_PROTRUSION / 2.0
    boot_z_centre = WIRE_BOOT_Z_BASE + WIRE_BOOT_H / 2.0
    boot = _box((WIRE_BOOT_PROTRUSION, WIRE_BOOT_W, WIRE_BOOT_H),
                center=(boot_x_centre, 0.0, boot_z_centre))
    return _union(body, spline, boot)


def _servo_pocket() -> trimesh.Trimesh:
    """Return the void volume of a servo (slightly oversized, for a slip
    fit) for cutting into a bracket.  Same local frame as
    `_servo_envelope`.

    Includes:
        - body cavity (0.4 mm clearance on every face)
        - output-gear clearance (cylindrical bore through the bracket
          face above the body)
        - 4 x M3 mounting holes through the tab plane

    Returns a single union mesh; pass to _diff(bracket, _servo_pocket())."""
    CL = 0.4   # mm clearance
    body = _box((SERVO_BODY_W + 2 * CL,
                 SERVO_BODY_D + 2 * CL,
                 SERVO_BODY_H + 2 * CL),
                center=(0, 0, (SERVO_BODY_H + 2 * CL) / 2.0 - CL))

    # Output-gear clearance (cylinder reaching up through the bracket)
    gear_clear = _cyl(SERVO_OUTPUT_OD / 2.0 + 0.6,
                      SERVO_OUTPUT_H * 4)
    gear_clear.apply_translation([SERVO_OUTPUT_X, 0,
                                    SERVO_BODY_H + SERVO_OUTPUT_H * 2])

    # 4 x M3 tab holes (2 on each side at SERVO_TAB_HOLE_PCD/2 spacing
    # along X; offset along Y by ~7.5 mm from centre is typical for
    # MG996R, but DS3225 is centred -- use centred for simplicity).
    tab_holes = []
    for sx in (-1, 1):
        h = _cyl(SERVO_TAB_HOLE / 2.0, SERVO_TAB_T * 6)
        h.apply_translation([sx * SERVO_TAB_HOLE_PCD / 2.0, 0,
                              SERVO_TAB_Z])
        tab_holes.append(h)

    return _union(body, gear_clear, *tab_holes)


def make_servo_body() -> trimesh.Trimesh:
    """Visual hobby-servo envelope for MuJoCo/CAD fit-checking.

    Same local frame as ``_servo_envelope``:
        +X = body long axis (mounting-tab span; output shaft offset toward +X)
        +Y = body short axis (depth)
        +Z = output-shaft direction (out of body top face)
        origin at the centre of the body's bottom face.

    Intended use:
        - Visual mesh asset in ``mujoco_prototype.py`` (contact-free).
        - Fit-check input for ``_verify_prototype.py``'s servo-clearance test.
    Not for printing -- the parametric well/cradle parts already cut the
    matching pocket out of the printed part, so a printed servo body would
    just clash with itself.
    """
    return _servo_envelope()


def make_servo_horn() -> trimesh.Trimesh:
    """Plastic 4-arm output horn (the part that ships with the servo).

    Local frame:
        +Z = output shaft axis (mates to the servo spline at z = 0)
        Origin at the bottom face of the horn hub.

    The 4 arms point along +X, +Y, -X, -Y (HORN_BOLT_ANGLES_RAD).  Each
    arm carries a row of mounting holes; the SECOND hole out from the
    spline on each arm sits on HORN_BOLT_PCD (= 20.8 mm) -- that's the
    pattern the printed ``servo_horn_adapter`` clamps onto.  This visual
    mesh drills those 4 bolt holes so any render that includes both the
    horn and the adapter shows the bolts lining up.
    Used as a visual stand-in only; the printed ``servo_horn_adapter``
    bolts on top of this.
    """
    parts: list[trimesh.Trimesh] = []
    hub = _cyl(8.0, 2.0)
    hub.apply_translation([0, 0, 1.0])
    parts.append(hub)
    # Arm length 2 * PLASTIC_HORN_X_TIP_R so each arm's tip sits at the
    # real-hardware sweep radius (= 19 mm from the spline centre on a
    # standard DS3225 / MG996R / DS3218 horn).  Previously hard-coded
    # to 20 mm (tip at radius 10 mm), which was INSIDE the
    # HORN_BOLT_PCD = 20.8 mm bolt circle -- nonsensical, and made the
    # mesh's bounding cylinder understate the horn's swept volume.
    # check_horn_sweep_clearance reads the bounding cylinder of this
    # mesh; keep it in sync with real hardware so the verifier
    # measures the right sweep radius.
    for a in HORN_BOLT_ANGLES_RAD:
        arm = _box((2.0 * PLASTIC_HORN_X_TIP_R, 4.0, 1.6))
        arm.apply_transform(rotation_matrix(a, [0, 0, 1]))
        arm.apply_translation([0, 0, 0.8])
        parts.append(arm)
    horn = _union(*parts)

    bolt_holes = []
    for a in HORN_BOLT_ANGLES_RAD:
        h = _cyl(HORN_BOLT_OD / 2.0, 4.0)
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              HORN_BOLT_PCD / 2.0 * np.sin(a),
                              0.8])
        bolt_holes.append(h)
    return _diff(horn, *bolt_holes)


def make_servo_horn_adapter() -> trimesh.Trimesh:
    """Round servo-horn adapter plate.

    Bolts to a standard plastic servo horn from below (single M3 centre
    screw + a counter-bored recess for the horn body) and presents a flat
    4 x M3 bolt pattern on the bolt circle so any link with the matching
    hole pattern can clamp onto it.

    Local frame:
        +Z = servo output axis
        Origin at the bottom face (mating to the plastic horn)
        Bolt holes on HORN_BOLT_PCD (= 20.8 mm) at HORN_BOLT_ANGLES_RAD
        (= 0 / 90 / 180 / 270 deg), aligned with the plastic horn's
        4 X-shaped arms so each bolt drops straight into the second
        hole-position out from the spline on each arm.

    Why a solid disc (and not the 4-arm cross the BOM used to advertise):
        The previous design had four "lightening cuts" sized so they sliced
        straight through the arms, leaving 4 corner blocks disconnected
        from the centre.  A 32 mm OD x 4 mm disc is 3 cm^3 of plastic per
        copy -- lightening it saves nothing and prints far more reliably.
    """
    plate_r = HORN_ADAPTER_OD / 2.0
    plate = _cyl(plate_r, HORN_ADAPTER_T)
    plate.apply_translation([0, 0, HORN_ADAPTER_T / 2.0])

    recess = _cyl(HORN_RECESS_OD / 2.0, HORN_RECESS_DEPTH)
    recess.apply_translation([0, 0, HORN_RECESS_DEPTH / 2.0])

    centre = _cyl(HORN_CENTRE_OD / 2.0, HORN_ADAPTER_T * 4)

    bolts = []
    for a in HORN_BOLT_ANGLES_RAD:
        h = _cyl(HORN_BOLT_OD / 2.0, HORN_ADAPTER_T * 4)
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              HORN_BOLT_PCD / 2.0 * np.sin(a),
                              0])
        bolts.append(h)

    return _diff(plate, recess, centre, *bolts)


# ---------------------------------------------------------------------------
# Body parts
# ---------------------------------------------------------------------------

def _hex_plate(flat_to_flat: float, thickness: float,
               with_centre_holes: bool = False,
               with_leg_features: bool = True) -> trimesh.Trimesh:
    """Return a flat hexagonal plate, centred on origin, axis = +Z.

    Hole pattern (per leg, 6 legs total):
        4 vertical M3 clearance holes (Φ BRACKET_BOLT_HOLE) drilled all
        the way through.  Pattern matches `make_coxa_bracket()`'s flange:
        2 holes on the OUTBOARD edge of the bolt rectangle (just inboard
        of the chassis perimeter) and 2 on the INBOARD edge.  All four
        holes are inboard of the apothem line so the bolt heads have
        chassis material under them.
    """
    apothem = flat_to_flat / 2.0
    circum = apothem / np.cos(np.pi / 6)
    plate = _cyl(circum, thickness, sections=6)
    plate.apply_transform(rotation_matrix(np.pi / 6, [0, 0, 1]))

    # The bracket's flange occupies bracket-local x in [-FLANGE_X, 0]
    # = [-30, 0] (origin = yaw axis = chassis edge).  Bolts at:
    #   outboard pair: x = -BRACKET_FLANGE_INSET (8 mm inboard of edge)
    #   inboard pair:  x = -BRACKET_FLANGE_INSET - BRACKET_BOLT_PCD_X
    #                       (8 + 20 = 28 mm inboard of edge)
    bolt_x_outboard = -BRACKET_FLANGE_INSET
    bolt_x_inboard  = -BRACKET_FLANGE_INSET - BRACKET_BOLT_PCD_X
    bolt_ys         = (-BRACKET_BOLT_PCD_Y / 2.0,
                       +BRACKET_BOLT_PCD_Y / 2.0)

    # Servo body+tab cutout per bracket so the yaw servo's body can
    # hang BELOW the chassis plate while the bracket flange clamps the
    # plate from above.  Without this cutout the user has to drill the
    # opening manually (which was issue #1 the user hit).
    # Size = WELL_W x WELL_D + 1 mm clearance per side so the
    # bracket's well slides through without rubbing.  Sits at bracket-
    # local (body_centre_x, 0) = (-10, 0) -- the well's centre.  Bolts
    # at y = +/-18 stay outside the cutout's y range (+/-13.5), so the
    # 4 chassis bolts still bite into solid chassis material.
    body_centre_x  = -SERVO_OUTPUT_X
    body_cutout_w  = WELL_W + 2.0            # 60 mm along bracket X
    body_cutout_d  = WELL_D + 2.0            # 27 mm along bracket Y

    holes = []
    if with_leg_features:
        for i in range(6):
            a = (i + 0.5) * np.pi / 3
            edge_mid = np.array([apothem * np.cos(a),
                                  apothem * np.sin(a),
                                  0.0])
            R = rotation_matrix(a, [0, 0, 1])
            R3 = R[:3, :3]
            for bx in (bolt_x_outboard, bolt_x_inboard):
                for by in bolt_ys:
                    world = edge_mid + R3 @ np.array([bx, by, 0.0])
                    h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
                    h.apply_translation([world[0], world[1], 0])
                    holes.append(h)

            cutout = _box((body_cutout_w, body_cutout_d, thickness * 4))
            cutout.apply_transform(R)
            cutout_world = edge_mid + R3 @ np.array([body_centre_x, 0.0, 0.0])
            cutout.apply_translation(cutout_world)
            holes.append(cutout)

    if with_centre_holes:
        # 4 holes for the electronics tray + battery holder mounting
        for i in range(4):
            a = np.pi / 4 + i * np.pi / 2
            h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
            h.apply_translation([35.0 * np.cos(a), 35.0 * np.sin(a), 0])
            holes.append(h)

    return _diff(plate, *holes)


def make_chassis_top() -> trimesh.Trimesh:
    """Top hex plate.  3D-printed in PLA, ~ 4 mm thick.

    Intentionally SMALLER than the bottom plate (CHASSIS_TOP_FLAT_TO_FLAT
    = 140 mm vs CHASSIS_FLAT_TO_FLAT = 200 mm).  The bottom plate carries
    the structural load (it sandwiches the coxa brackets and takes their
    M3 bolts) while the top plate just provides a deck for the battery
    holder, electronics tray and optional arm.  The smaller hexagon keeps
    the top plate inside a 70 mm apothem so the legs' hip-pitch wells +
    femur hip pads sweep clear of it across the full joint workspace
    (see check_workspace_self_collision in _verify_prototype.py).  No
    per-leg bracket cutouts or M3 bolt holes are needed because the
    bottom plate already takes them.  Only the 4 centre-hole standoff
    bolts (battery/electronics tray + arm baseplate) remain.
    """
    return _hex_plate(CHASSIS_TOP_FLAT_TO_FLAT, CHASSIS_PLATE_T,
                       with_centre_holes=True,
                       with_leg_features=False)


def make_chassis_bottom() -> trimesh.Trimesh:
    """Bottom hex plate.  Identical to the top."""
    return _hex_plate(CHASSIS_FLAT_TO_FLAT, CHASSIS_PLATE_T,
                       with_centre_holes=True)


def make_battery_holder() -> trimesh.Trimesh:
    """Open-top tray for one 3S 2200 mAh LiPo (105 x 38 x 28 mm).
    Two velcro slots cut through the long walls let the user strap the
    pack down."""
    outer = _box((BATTERY_W, BATTERY_D, BATTERY_H),
                 center=(0, 0, BATTERY_H / 2.0))
    inner = _box((BATTERY_W - 2 * BATTERY_WALL,
                  BATTERY_D - 2 * BATTERY_WALL,
                  BATTERY_H - BATTERY_WALL + 5.0),
                 center=(0, 0, (BATTERY_H - BATTERY_WALL) / 2.0
                                 + BATTERY_WALL))

    # Velcro slots through both long walls
    velcro = []
    for s in (-1, 1):
        slot = _box((BATTERY_STRAP_W, BATTERY_WALL * 6, BATTERY_H * 0.5),
                    center=(s * (BATTERY_W * 0.25), 0, BATTERY_H * 0.55))
        velcro.append(slot)

    # 4 mounting feet with M3 holes (sit on the bottom plate)
    feet = []
    foot_holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            ft = _box((10.0, 10.0, BATTERY_WALL),
                      center=(sx * (BATTERY_W / 2.0 - 5.0),
                               sy * (BATTERY_D / 2.0 + 6.0),
                               BATTERY_WALL / 2.0))
            feet.append(ft)
            h = _cyl(SERVO_TAB_HOLE / 2.0, BATTERY_WALL * 6)
            h.apply_translation([sx * (BATTERY_W / 2.0 - 5.0),
                                  sy * (BATTERY_D / 2.0 + 6.0),
                                  BATTERY_WALL])
            foot_holes.append(h)

    body = _union(outer, *feet)
    return _diff(body, inner, *velcro, *foot_holes)


def make_electronics_tray() -> trimesh.Trimesh:
    """A flat 3D-printed plate with mounting holes for an Arduino Nano
    + a PCA9685 16-channel PWM driver (or two PCA9685s for the full 18
    servos).  Stand-offs are 3 mm tall printed bosses with M3 holes."""
    plate = _box((ELEC_TRAY_W, ELEC_TRAY_D, ELEC_TRAY_T),
                 center=(0, 0, ELEC_TRAY_T / 2.0))

    # Arduino Nano standoffs (43 x 18 mm 4-hole pattern)
    nano_holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            h = _cyl(SERVO_TAB_HOLE / 2.0, ELEC_TRAY_T * 4)
            h.apply_translation([-30.0 + sx * 15.5, sy * 8.0, ELEC_TRAY_T])
            nano_holes.append(h)

    # PCA9685 standoffs (54 x 16 mm 4-hole pattern, on the +X side)
    pca_holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            h = _cyl(SERVO_TAB_HOLE / 2.0, ELEC_TRAY_T * 4)
            h.apply_translation([+25.0 + sx * 27.0, sy * 7.5, ELEC_TRAY_T])
            pca_holes.append(h)

    # Mounting holes to the chassis (4 holes on the same 35 mm radius
    # pattern as `_hex_plate(with_centre_holes=True)`)
    mount_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(SERVO_TAB_HOLE / 2.0, ELEC_TRAY_T * 4)
        h.apply_translation([35.0 * np.cos(a), 35.0 * np.sin(a), 0])
        mount_holes.append(h)

    return _diff(plate, *nano_holes, *pca_holes, *mount_holes)


# ---------------------------------------------------------------------------
# Leg parts
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Joint convention (mirrors hexapod_walker.py)
# ---------------------------------------------------------------------------
#
#   - Yaw axis is global +Z (vertical).
#   - Hip and knee pitch axes are along the leg's local +Y (tangential
#     to the chassis, perpendicular to the leg's outboard direction).
#   - In every leg, all three servos hang on the leg's -Y_local side;
#     the spars run along +X_local in the y_local = 0 plane.
#
# Parts are designed for FDM printing in PLA / PETG: minimum wall 1.6 mm,
# minimum hole 3.2 mm, no overhangs steeper than 45 deg without supports.
# ---------------------------------------------------------------------------


def make_coxa_bracket() -> trimesh.Trimesh:
    """The bracket that bolts to the chassis edge and carries the
    hip-yaw servo with its output shaft pointing UP.

    Local frame:
        Origin: at the YAW AXIS (centre of the servo output spline)
                in the chassis-plate's TOP plane (z = 0 of the bracket
                = top face of the chassis plate).
        +Z = yaw axis (output points UP).
        +X = outboard (away from chassis centre).
        +Y = tangential (along the chassis edge).

    Layout:
        - HORIZONTAL flange at z in [0, BRACKET_FLANGE_T] = [0, 15],
          spanning x in [flange_x_min, flange_x_max] = [-43, +23] and
          y in [-BRACKET_FLANGE_Y/2, +BRACKET_FLANGE_Y/2] = [-26, 26].
          Bolts vertically to the chassis plate (4 M3 bolts go DOWN
          through the flange and the chassis plate together).
        - SERVO WELL hangs below the flange.  The servo's output spline
          is at bracket-local (0, 0, GEAR_TOP_Z).  The well's body
          cavity is offset by -SERVO_OUTPUT_X = -10 in X so the gear
          lands on the yaw axis.
        - SERVO BODY + TAB DROP-IN SLOT is a RECTANGULAR THROUGH-HOLE
          in the flange (z in [-6, +15] = bottom of the rib through
          to top of the flange), wide enough to clear the tab tips in
          X (56 mm) and the body in Y (21 mm).  The flange becomes a
          continuous CLOSED RING of material around the slot,
          measuring 66 x 52 mm with a 5 mm perimeter strip at each
          X-end and a 15.5 mm perimeter strip at each Y-end -- all
          four chassis-bolt corners are tied together through that
          ring.

    Assembly order:
        1.  DROP the yaw servo straight DOWN through the rectangular
            slot from above with the output gear pointing up.  The
            body + tabs slide through the slot opening; the tabs land
            on the well rim at z = 0.  No tilting required.
        2.  Drive 4 M3 self-tappers through the tab holes into the
            well's pilot holes.
        3.  Bolt the bracket flange to the chassis plate (4 M3 cap
            screws + nylock nuts under the chassis plate -- M3 x 32
            because the flange itself is 15 mm tall).
        4.  Add the horn adapter and the coxa link on top of the
            output spline, which pokes up out of the open slot at
            x = 0.

    Earlier revisions of this part tried a closed-top variant (a
    2.5 mm continuous cap of flange material above the body, with
    only an 11 mm output-gear clearance hole through the cap).  That
    made the bracket meaningfully stiffer in torsion, but there was
    no way to physically install the servo: the body could not pass
    through the 11 mm hole, and the rib + chassis-plate underside
    made tilt-and-slide insertion from below impractical.  The
    through-slot is mandatory for assembly; the strength reduction
    is paid back by the chassis plate (which clamps the underside
    of the flange in compression at all four bolt corners).
    """
    body_centre_x = -SERVO_OUTPUT_X        # body offset so output is at x = 0
    well_dz = -WELL_RIM_Z                  # well rim coincides with chassis plate top

    # ---- Servo well (hangs below z=0) -------------------------------
    well = _servo_well_solid()
    well.apply_translation([body_centre_x, 0.0, well_dz])

    # ---- Wire exit slot through the well's inboard (-X) wall --------
    # Lets the 3-wire servo harness route out of the cradle toward the
    # chassis centre, under the flange edge.  Same helper used by the
    # hip-pitch (coxa_link) and knee (femur_link) cradles.
    wire_slot = _wire_exit_slot()
    wire_slot.apply_translation([body_centre_x, 0.0, well_dz])

    # ---- Mounting flange --------------------------------------------
    # The flange is sized so the body+tab insertion slot is fully
    # enclosed by flange material on ALL FOUR SIDES, forming a
    # continuous closed ring around the slot:
    #
    #   slot footprint (X x Y): SERVO_TAB_W + 2 = 56 mm  x  21 mm
    #   flange margin around slot in X:  SLOT_FLANGE_MARGIN = 5 mm/side
    #   flange margin around slot in Y:  BRACKET_FLANGE_Y/2 - slot_d/2
    #                                    = 24 - 10.5 = 13.5 mm/side
    #
    # That gives a flange X extent of 56 + 2*5 = 66 mm.  Earlier the
    # flange was only 30 mm long in X (x in [-30, 0]) so the 56 mm
    # slot extended PAST both flange ends in X and the flange got
    # split into two disconnected half-pads (no perimeter material
    # connecting the +Y strip to the -Y strip across the slot ends).
    # With a 66 mm flange the slot lives entirely INSIDE the flange
    # and the surrounding flange material is a single closed ring.
    SLOT_FLANGE_MARGIN = 5.0
    slot_w = SERVO_TAB_W + 2.0                                    # 56 mm (body+tab)
    slot_d = SERVO_BODY_D + 1.0                                   # 21 mm (body Y)
    flange_x_min  = body_centre_x - slot_w / 2.0 - SLOT_FLANGE_MARGIN  # -43
    flange_x_max  = body_centre_x + slot_w / 2.0 + SLOT_FLANGE_MARGIN  # +23
    flange_x_ext  = flange_x_max - flange_x_min                       # 66 mm
    flange_centre_x = (flange_x_min + flange_x_max) / 2.0             # -10
    flange = _box((flange_x_ext, BRACKET_FLANGE_Y, BRACKET_FLANGE_T),
                  center=(flange_centre_x, 0.0,
                           BRACKET_FLANGE_T / 2.0))

    # Body+tab DROP-IN slot.  Through-hole spanning the FULL HEIGHT of
    # the flange (z = -6 at the rib bottom up to z = BRACKET_FLANGE_T
    # + 0.5 so the cut comfortably exits the top face), letting the
    # user drop the servo straight DOWN through the slot during
    # assembly.  The flange becomes a closed RING around the slot:
    # a 5 mm perimeter strip at each X-end (slot 56 mm wide vs flange
    # 66 mm), and a 15.5 mm perimeter strip at each Y-end (slot
    # 21 mm wide vs flange 52 mm).  All four chassis-bolt corners
    # are tied together through that ring.
    slot_z_min = -6.0
    slot_z_max = BRACKET_FLANGE_T + 0.5
    slot_z_ext = slot_z_max - slot_z_min
    slot_z_cen = (slot_z_min + slot_z_max) / 2.0
    slot = _box((slot_w, slot_d, slot_z_ext),
                center=(body_centre_x, 0.0, slot_z_cen))

    # ---- Chassis bolt holes -----------------------------------------
    bolt_x_outboard = -BRACKET_FLANGE_INSET
    bolt_x_inboard  = -BRACKET_FLANGE_INSET - BRACKET_BOLT_PCD_X
    bolt_ys = (-BRACKET_BOLT_PCD_Y / 2.0, +BRACKET_BOLT_PCD_Y / 2.0)
    chassis_holes = []
    for bx in (bolt_x_outboard, bolt_x_inboard):
        for by in bolt_ys:
            h = _cyl(BRACKET_BOLT_HOLE / 2.0, BRACKET_FLANGE_T * 4)
            h.apply_translation([bx, by, BRACKET_FLANGE_T / 2.0])
            chassis_holes.append(h)

    # ---- Stiffening rib bridging the flange to the well -------------
    # The flange straddles the well in X: the well's inboard +X wall
    # sits at x in [-39, -30] and its outboard -X wall at x in [+10,
    # +19].  The rib spans y over the well's footprint (slightly
    # wider so it catches the entire +Y/-Y face) and z from below
    # the well rim (z = -3) up through the flange's top (z =
    # FLANGE_T) so the flange is welded to the well rim on BOTH
    # sides of the yaw axis, not just the inboard side.  The
    # drop-in slot eats the rib's centre at y in [-10.5, +10.5];
    # what survives is two ±13 to ±10.5 wide column strips that
    # carry the flange top down into the well rim on either side
    # of the body+tab passage.
    #
    # Earlier the rib only spanned x in [-18, 0] (inboard half
    # only), so the +X side of the flange (past the yaw axis, over
    # the outboard wall) was only tied to the well by the slot's
    # 1 mm +X-wall remnant -- a noticeably weaker load path than
    # the -X side, which also has the chassis plate underneath it.
    # The symmetric rib spans x in [-18, +18] so the outboard well
    # wall gets the same direct flange-to-rim connection.
    #
    # IMPORTANT: rib Y width MUST be no wider than the chassis-plate
    # body+tab cutout (WELL_D + 2 = 27 mm) because the rib extends
    # DOWN past z = 0 into the chassis-plate's Z volume (z = [-4, 0]).
    # The earlier 48 mm-wide rib clashed with solid chassis material
    # at y in [+/-13.5, +/-24] and stopped the bracket from seating
    # flush on the chassis plate.
    RIB_X = 36.0                                  # 18 mm each side of yaw axis
    RIB_Y = WELL_D + 1.0                          # 26 mm == ±13, fits in 27 mm cutout
    rib = _box((RIB_X, RIB_Y, 6.0),
               center=(0.0, 0.0, 0.0))            # x in ±18, y in ±13, z in ±3

    # Side gussets riding directly on top of the well's +Y / -Y walls.
    # The body+tab pocket eats the rib at y in [-10.5, +10.5] and
    # reaches DOWN to z = -6, so the only flange-to-well load paths
    # at the well's +Y / -Y wall tops are these two narrow column
    # strips above the well wall (which lives at y in [10.5, 12.5]
    # and y in [-12.5, -10.5]).  Each gusset is bounded in Y to the
    # well's wall footprint so nothing dangles in mid-air past the
    # well's outer Y faces.  Z range = [-6, BRACKET_FLANGE_T], so the
    # gussets merge with the (now 15 mm tall) flange material above
    # z = 0 into one solid slab; the structurally important
    # contribution is the 6 mm of material BELOW z = 0 that ties the
    # flange into the well rim.
    slot_d = SERVO_BODY_D + 1.0
    gusset_z_min = -6.0                                       # 6 mm down into well rim
    gusset_z_max = BRACKET_FLANGE_T                           # merges with flange above z=0
    gusset_z_ext = gusset_z_max - gusset_z_min                # 21 mm
    gusset_z_cen = (gusset_z_min + gusset_z_max) / 2.0
    gusset_x_min = body_centre_x - SERVO_BODY_W / 2.0         # well inboard +X wall
    gusset_x_max = 0.0                                        # flush with flange outboard
    gusset_x_ext = gusset_x_max - gusset_x_min
    gusset_x_cen = (gusset_x_min + gusset_x_max) / 2.0
    gusset_y_inner = slot_d / 2.0 + 0.5                       # just outside slot edge
    gusset_y_outer = WELL_D / 2.0                             # well's outer +Y / -Y face
    gusset_y_ext   = gusset_y_outer - gusset_y_inner
    side_gussets = []
    for sy in (-1, 1):
        g = _box((gusset_x_ext, gusset_y_ext, gusset_z_ext),
                 center=(gusset_x_cen,
                         sy * (gusset_y_inner + gusset_y_outer) / 2.0,
                         gusset_z_cen))
        side_gussets.append(g)

    # ---- Bridge-underside gussets ----------------------------------
    # The flange's X-end bridges (the slot-cap strips at x in
    # [flange_x_min, slot_x_min] and [slot_x_max, flange_x_max]) sit
    # almost entirely PAST the well's outer X faces -- well outer
    # x in [-39, +19], slot ends at x = -38 / +18, flange ends at
    # x = -43 / +23, so each bridge has only ~1 mm of well wall
    # directly under it and ~4 mm hanging in air.  On the inboard
    # side the chassis plate at z in [-4, 0] catches the bridge,
    # but on the outboard side the +X bridge dangles past the
    # chassis edge with nothing tying it down to the well outboard
    # wall.  Add a 3 mm deep box gusset under each bridge that
    # drops onto the well's +X / -X outer face, turning each
    # bridge into a T-beam.  The inboard gusset is bounded inside
    # the chassis plate's body-cutout (x in [-40, +20]) so it
    # doesn't punch into solid chassis plate material.
    slot_x_min_b = body_centre_x - slot_w / 2.0          # -38
    slot_x_max_b = body_centre_x + slot_w / 2.0          # +18
    chassis_cutout_x_min = body_centre_x - (WELL_W + 2.0) / 2.0   # -40
    bridge_gusset_z_min = -3.0
    bridge_gusset_z_max = 0.0
    bridge_gusset_z_ext = bridge_gusset_z_max - bridge_gusset_z_min
    bridge_gusset_z_cen = (bridge_gusset_z_min
                            + bridge_gusset_z_max) / 2.0
    bridge_gusset_y_ext = WELL_D                          # 25, fits in cutout (27)
    inboard_bg_x_min = max(flange_x_min, chassis_cutout_x_min)
    bridge_gusset_x_pairs = (
        (inboard_bg_x_min, slot_x_min_b),                 # -X bridge: x in [-40, -38]
        (slot_x_max_b,     flange_x_max),                 # +X bridge: x in [+18, +23]
    )
    bridge_gussets = []
    for bx_min, bx_max in bridge_gusset_x_pairs:
        bx_ext = bx_max - bx_min
        bx_cen = (bx_min + bx_max) / 2.0
        bg = _box((bx_ext, bridge_gusset_y_ext, bridge_gusset_z_ext),
                  center=(bx_cen, 0.0, bridge_gusset_z_cen))
        bridge_gussets.append(bg)

    # ---- Horn-sweep clearance void above the seated servo --------------
    # The plastic 4-arm X-shaped horn that ships with DS3225 / MG996R
    # class servos reaches PLASTIC_HORN_X_TIP_R = 19 mm to each arm
    # tip, so when the servo rotates the horn sweeps a Phi 38 mm
    # cylinder centred on the yaw axis (bracket-x = 0, bracket-y = 0)
    # in the Z range BETWEEN the body's top face (= +10.75 mm in
    # bracket-local at worst-case body float) and the top of the
    # printed horn adapter (= +25.75 mm).  The drop-in body+tab slot
    # is only +/-10.5 mm wide in Y, so the flange material at y in
    # [+/-10.5, +/-(WELL_D/2 + extra)] above the body's top face
    # SITS IN THE HORN'S SWEEP and physically blocks the horn from
    # rotating (or from being installed on the spline above the
    # bracket at all).
    #
    # This carves a Phi (2 * (HORN_SWEEP_R)) cylindrical pocket
    # centred on the yaw axis that goes from just above the body's
    # nominal top (z = +10.0, ~0.75 mm below worst-case body top
    # so the cut comfortably starts in air) up through the flange's
    # top face plus a 2 mm overshoot so the cut exits the part
    # cleanly above the flange.  Below z = +10.0 the flange remains
    # solid except for the rectangular drop-in slot -- the well-rim
    # / chassis-bolt structural ring is untouched.  The chassis-bolt
    # corners at (x = -8 / -28, y = +/-20) sit at radius >= 21.5 mm
    # from the yaw axis, so they stay outside the cylinder.
    #
    # See _verify_prototype.check_horn_sweep_clearance for the
    # regression test that catches this class of failure: it probes
    # the SAME cylinder against the bracket and requires the bracket
    # to be entirely VOID inside it.  PRIOR TO THIS CUT the flange
    # had ~ 2 cm^3 of material lobing into the cylinder at y in
    # [+/-10.5, +/-19.5] / z in [+10.75, +15] -- the recurring
    # "the servo motor doesn't stick out high enough in the coxa
    # bracket" failure the user has reported multiple times.  Edit
    # any of HORN_SWEEP_*, PLASTIC_HORN_X_TIP_R, HORN_ADAPTER_OD,
    # SERVO_BODY_H, WELL_RIM_Z, BRACKET_FLANGE_T, SERVO_OUTPUT_X --
    # and the verifier will catch any regression here.
    horn_sweep_r = (max(HORN_ADAPTER_OD / 2.0, PLASTIC_HORN_X_TIP_R)
                    + 0.75)                              # 19.75 mm
    sweep_z_lo = SERVO_BODY_H - WELL_RIM_Z - 0.75        # +10.0 mm
    sweep_z_hi = BRACKET_FLANGE_T + 2.0                  # +17.0 mm
    sweep_z_ext = sweep_z_hi - sweep_z_lo
    horn_sweep_void = _cyl(horn_sweep_r, sweep_z_ext)
    horn_sweep_void.apply_translation([0.0, 0.0,
                                        0.5 * (sweep_z_lo + sweep_z_hi)])

    body = _union(flange, well, rib, *side_gussets, *bridge_gussets)
    return _diff(body, slot, wire_slot, horn_sweep_void, *chassis_holes)


def make_coxa_link() -> trimesh.Trimesh:
    """Coxa link: a flat plate, driven by the yaw servo's horn, that
    carries the hip-pitch servo at its outboard end.

    Local frame:
        Origin: bolt-circle centre of the hub (= yaw axis, sitting
                on top of the horn adapter).
        +Z = yaw axis (UP, away from the yaw servo).
        +X = arm direction (outboard at neutral pose).
        +Y = hip-pitch joint axis (= the hip-pitch servo's output
              shaft direction).

    Layout:
        - Pedestal: COXA_LIFT mm tall square pillar from z=0 (horn-adapter
          mating face) up to z=COXA_LIFT.  Without this lift, the hip-pitch
          well dipped ~2.5 mm below the chassis-plate top face and the
          femur's hip-pad swung ~7 mm below the chassis-plate top, both
          of which clashed with the chassis at standard yaw angles.
        - Hub: square pad on TOP of the pedestal, with a 4-bolt hole
          pattern matching the horn adapter.  Bolt holes drill straight
          through the pedestal so the same M3 screws clamp the hub +
          pedestal stack to the horn adapter (use M3 x 25-30 screws).
        - Arm: flat plate extending in +X from the hub, at z >= COXA_LIFT.
        - Hip-pitch servo well: hangs in -Z below the arm at the +X
          end, at z >= COXA_LIFT - 28 (i.e. ~+2 mm above the chassis
          plate top with COXA_LIFT = 10).  Open-topped (well +Z) is
          mapped to link +Y so the servo can be DROPPED in from the
          +Y direction during assembly.
    """
    arm_w = 22.0           # mm, along Y
    arm_t = COXA_ARM_T     # mm, along Z (printed flat against the build
                            # plate).  See COXA_ARM_T docstring for the
                            # constraint that picks this thickness.

    # Hub region (above the yaw servo horn) -- a thicker square.
    # Built at the original (un-lifted) z, then translated up by
    # COXA_LIFT after the whole link union is assembled (see below).
    hub_t = arm_t + 2.0
    hub = _box((34.0, 34.0, hub_t),
               center=(0, 0, hub_t / 2.0))

    # Arm reaching out to the hip-pitch motor mount.  Spans local x in
    # [-12, COXA_LENGTH + 16].
    arm = _box((COXA_LENGTH + 28.0, arm_w, arm_t),
               center=((COXA_LENGTH + 28.0) / 2.0 - 12.0, 0,
                        arm_t / 2.0))

    # Hip-pitch servo well at the outboard end.  Open-topped well, with
    # well +Z -> link +Y so the user can drop the servo in from the +Y
    # direction (= along the hip-pitch joint axis) during assembly.
    # NB: the 4 M3 mounting pilots inside the well are drilled by
    # ``_servo_well_solid`` on the SERVO_TAB_HOLE_PCD x
    # SERVO_TAB_HOLE_PCD_Y (49.5 x 10 mm) pattern -- matching the
    # physical servo's mounting tabs, NOT the 24 mm horn PCD.
    well = _servo_well_solid()
    wire_slot = _wire_exit_slot()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])  # well +Z -> link +Y
    well.apply_transform(R)
    wire_slot.apply_transform(R)
    # Output spline tip in well-local: (SERVO_OUTPUT_X, 0,
    #   SERVO_BODY_H + SERVO_OUTPUT_H) = (10, 0, 44).
    # After R: (10, 44, 0).  We want it at (COXA_LENGTH, 0, 0) (the
    # joint axis position in the link frame, on the arm centreline).
    delta = np.array([COXA_LENGTH - SERVO_OUTPUT_X,
                       -(SERVO_BODY_H + SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)
    wire_slot.apply_translation(delta)
    # Drop the well in -Z so it hangs below the arm rather than
    # interpenetrating it.  WELL_Z_DROP_EXTRA pushes the well an extra
    # 4 mm down (PAST the natural arm-bottom = well-top plane) so the
    # femur hip pad's swept +Z edge clears the arm's bottom face -- see
    # the COXA_LIFT / WELL_Z_DROP_EXTRA docstrings near the top of this
    # file for the full geometry.
    well_z_drop = -(WELL_D / 2.0 + arm_t / 2.0 + WELL_Z_DROP_EXTRA)
    well.apply_translation([0.0, 0.0, well_z_drop])
    wire_slot.apply_translation([0.0, 0.0, well_z_drop])

    # Bridge from the arm's -Y edge (y = -arm_w/2) down to the well's
    # near +Y face (y = WELL_RIM_Z + delta_y) and from the arm's bottom
    # face (z = 0) down to the well's top face (z = well_z_drop +
    # WELL_D/2).  Without this the well dangles >5 mm away from the
    # arm in both Y and Z.
    #
    # Bridge X-extent runs the FULL length of the arm (arm_x_extent
    # below): the well wall material the bridge ties into is 58 mm
    # wide along link +X, so a wider bridge spreads the hip-pitch
    # reaction load over the full well footprint instead of
    # concentrating it in a 30 mm-wide neck at the well's centre.
    arm_x_extent     = COXA_LENGTH + 28.0                       # 53 mm
    arm_x_centre     = arm_x_extent / 2.0 - 12.0                # 14.5
    arm_minus_y_edge = -arm_w / 2.0
    well_near_y      = WELL_RIM_Z + delta[1]                    # ~ -16.75
    well_top_z       = well_z_drop + WELL_D / 2.0               # ~ -3.0
    bridge_y_min = well_near_y - 0.5                            # 0.5 mm overlap into well
    bridge_y_max = arm_minus_y_edge + 0.5                       # 0.5 mm overlap into arm
    bridge_y_extent = bridge_y_max - bridge_y_min
    bridge_y_centre = (bridge_y_min + bridge_y_max) / 2.0
    # Drop the bridge bottom DOWN to overlap the well-top face by 1.5 mm
    # (so the boolean union with the well is a real volumetric fuse and
    # not a 0 mm boolean kiss).  Tied to well_z_drop so the bridge auto-
    # tracks the well when WELL_Z_DROP_EXTRA changes.  Stays well above
    # the seated servo body top face (body top sits at link-z =
    # well_z_drop + SERVO_BODY_D/2, see body_top_z below; the bridge's
    # bottom face at well-top - 1.5 mm is several mm above that body
    # top).
    bridge_z_min = (well_z_drop + WELL_D / 2.0) - 1.5
    bridge_z_max = arm_t                                        # up to arm top
    bridge_z_extent = bridge_z_max - bridge_z_min
    bridge_z_centre = (bridge_z_min + bridge_z_max) / 2.0
    bridge = _box((arm_x_extent, bridge_y_extent, bridge_z_extent),
                  center=(arm_x_centre, bridge_y_centre, bridge_z_centre))

    # Well-top-wall thickening pad.  See WELL_TOP_PAD_Y_EXT's big
    # docstring near the top of this file for full motivation; in
    # short: this is the user's "thicken the motor-housing wall so a
    # bigger surface area attaches to the top piece" fix for the
    # recurring top-of-coxa_link <-> servo-well joint weakness.  The
    # pad spans the well's full outer X range (so its X footprint
    # matches the well perfectly and the bonded interface covers the
    # whole 58 mm well width), extends WELL_TOP_PAD_Y_EXT mm BEYOND
    # the bridge's -Y face into -Y (i.e. INTO the well's outer
    # footprint, so its bottom face fuses with the well's outer top
    # wall above the cavity) AND all the way to the bridge's +Y face
    # (so it merges with the existing bridge + arm), and fills the
    # entire Z gap between the well's outer top face and the arm's
    # bottom face.  Bridge_y_min is the bridge's existing -Y face
    # (= well_near_y - 0.5 = well's +Y face minus 0.5 mm of overlap
    # into the well).
    pad_x_min = -WELL_W / 2.0 + delta[0]                        # = -14
    pad_x_max = +WELL_W / 2.0 + delta[0]                        # = +44
    pad_y_min = bridge_y_min - WELL_TOP_PAD_Y_EXT               # = -22.25
    pad_y_max = bridge_y_max                                    # = -10.5
    pad_z_min = bridge_z_min                                    # = -8.5
    pad_z_max = 0.0                                              # arm bottom
    pad_x_extent = pad_x_max - pad_x_min                        # = 58
    pad_y_extent = pad_y_max - pad_y_min                        # = 11.75
    pad_z_extent = pad_z_max - pad_z_min                        # = 8.5
    well_top_pad = _box(
        (pad_x_extent, pad_y_extent, pad_z_extent),
        center=(
            0.5 * (pad_x_min + pad_x_max),
            0.5 * (pad_y_min + pad_y_max),
            0.5 * (pad_z_min + pad_z_max),
        ),
    )

    # Stiffening gusset stacked on the +X end of the arm on TOP of the
    # well.  Extended in -Y to cover the bridge so the upper flange of
    # the arm-bridge-well section becomes a continuous 26+ mm-wide cap
    # over the joint instead of stepping down at y = -arm_w/2.
    gusset_y_min     = bridge_y_min
    gusset_y_max     = +arm_w / 2.0
    gusset_y_extent  = gusset_y_max - gusset_y_min
    gusset_y_centre  = (gusset_y_min + gusset_y_max) / 2.0
    gusset = _box((30.0, gusset_y_extent, arm_t),
                  center=(COXA_LENGTH - 14.0, gusset_y_centre,
                          arm_t / 2.0))

    # Top cap rib stacked ON TOP of the arm slab.  See the
    # COXA_ARM_CAP_T comment near the top of this file for the
    # motivation and the clearance analysis -- we cap the arm in
    # TWO halves (+Y side and -Y side) and leave a gap of width
    # 2 * LINK_THICKNESS centred on link y = 0, so the femur spar
    # (at link y in [-3, +3] at any pitch, since the spar's Y
    # extent is +/-LINK_THICKNESS/2 and rotation about Y does not
    # change the spar's link Y range) can still swing freely
    # through y = 0 at extreme negative femur_pitch.  The hip
    # pad's swept disk (radius HIP_PAD_R = 17 mm about (COXA_LENGTH,
    # ?, hip_drop) in link X-Z, at any pitch since the pad is
    # rotationally symmetric about its femur Y axis) reaches a
    # maximum link z of hip_drop + HIP_PAD_R = +13.5 mm lifted,
    # which is 6.5 mm BELOW the cap's bottom face at z = arm_t +
    # COXA_LIFT = +20 mm lifted -- so the cap also stays out of
    # the pad's swept volume.
    cap_x_min     = +17.0 - 2.0                # hub +X edge minus 2 mm overlap
    cap_x_max     = arm_x_extent - 12.0        # arm +X end
    cap_x_extent  = cap_x_max - cap_x_min
    cap_x_centre  = (cap_x_min + cap_x_max) / 2.0
    cap_z_min     = arm_t
    cap_z_max     = arm_t + COXA_ARM_CAP_T
    cap_z_extent  = cap_z_max - cap_z_min
    cap_z_centre  = (cap_z_min + cap_z_max) / 2.0

    # +Y half of the cap: y in [LINK_THICKNESS/2, arm_w/2].
    cap_pos_y_min     = LINK_THICKNESS / 2.0
    cap_pos_y_max     = +arm_w / 2.0
    cap_pos_y_extent  = cap_pos_y_max - cap_pos_y_min
    cap_pos_y_centre  = (cap_pos_y_min + cap_pos_y_max) / 2.0
    arm_cap_pos = _box((cap_x_extent, cap_pos_y_extent, cap_z_extent),
                        center=(cap_x_centre, cap_pos_y_centre, cap_z_centre))

    # -Y half of the cap: y in [-arm_w/2, -LINK_THICKNESS/2].
    cap_neg_y_min     = -arm_w / 2.0
    cap_neg_y_max     = -LINK_THICKNESS / 2.0
    cap_neg_y_extent  = cap_neg_y_max - cap_neg_y_min
    cap_neg_y_centre  = (cap_neg_y_min + cap_neg_y_max) / 2.0
    arm_cap_neg = _box((cap_x_extent, cap_neg_y_extent, cap_z_extent),
                        center=(cap_x_centre, cap_neg_y_centre, cap_z_centre))

    # Underside stiffening gusset hanging below the arm in the bridge
    # region.
    gusset_under_x0    = HIP_PAD_R + 1.0
    gusset_under_y_min = bridge_y_min
    gusset_under_y_max = -arm_w / 2.0 + 2.0
    gusset_under_z_min = -COXA_BRIDGE_GUSSET_H
    gusset_under_z_max = 0.5
    gusset_under = _box(
        (COXA_BRIDGE_GUSSET_L,
         gusset_under_y_max - gusset_under_y_min,
         gusset_under_z_max - gusset_under_z_min),
        center=(
            gusset_under_x0 + COXA_BRIDGE_GUSSET_L / 2.0,
            (gusset_under_y_min + gusset_under_y_max) / 2.0,
            (gusset_under_z_min + gusset_under_z_max) / 2.0,
        ),
    )

    # ---- Build the link body in the original (un-lifted) frame ----
    body_unlifted = _union(hub, arm, well, gusset, bridge, gusset_under,
                            arm_cap_pos, arm_cap_neg, well_top_pad)
    body_unlifted = _diff(body_unlifted, wire_slot)
    # Lift everything UP by COXA_LIFT so the well's bottom + the
    # femur's hip-pad clear the chassis-plate top during yaw + pitch
    # rotation.
    body_unlifted.apply_translation([0.0, 0.0, COXA_LIFT])

    # ---- Pedestal underneath the (now lifted) hub ----
    pedestal = _box((34.0, 34.0, COXA_LIFT),
                    center=(0, 0, COXA_LIFT / 2.0))

    # Cut a clearance trough in the pedestal where the hip-pitch
    # servo body protrudes during +Y assembly insertion.  Without
    # this slot the body's "+Z face" (link z = hip_drop + SERVO_BODY_D/2
    # = COXA_LIFT - 5.5 ~ +1.5) and the pedestal's bottom face
    # (z = 0) overlap by ~1.5 mm at link x in [-5, +17] / y in
    # [-17, +17], blocking insertion.  The trough cuts through that
    # region and ~1 mm extra in z for FDM tolerance.
    body_x_min   = -SERVO_BODY_W / 2.0 - 1.0                # -21 + clearance
    body_x_max   =  SERVO_BODY_W / 2.0 + (COXA_LENGTH - SERVO_OUTPUT_X) + 1.0
    trough_x_min = body_x_min                               # -21
    trough_x_max = body_x_max                               # +36 (always outside pedestal +X)
    trough_x_ext = trough_x_max - trough_x_min
    trough_x_cen = (trough_x_min + trough_x_max) / 2.0
    # Body's +Y face Z position in LIFTED coxa-link frame.  The hip-pitch
    # body's centre lives at coxa-link z = well_z_drop + COXA_LIFT (= the
    # well centre in the lifted frame); its +Y face (= body local +Z =
    # body top after the well R rotation maps body local +Z onto coxa-
    # link +Y... no wait, +Y face here means the body's depth/short axis
    # face after the R rotation maps well local +Y onto coxa-link -Z.
    # So body coxa-link z range = well_z_drop + COXA_LIFT +/-
    # SERVO_BODY_D/2.  The TOP of that range is what the trough has to
    # clear during +Y assembly insertion.  Earlier this formula was
    # `COXA_LIFT - WELL_D/2 + SERVO_BODY_D/2` which ignored arm_t/2 and
    # WELL_Z_DROP_EXTRA -- the result over-cut the pedestal by arm_t/2 +
    # WELL_Z_DROP_EXTRA = 7 mm and left a much thinner pedestal cap
    # above the trough than intended.
    body_top_z   = well_z_drop + COXA_LIFT + SERVO_BODY_D / 2.0
    trough_z_max = body_top_z + 1.0                         # +1 mm FDM margin
    trough_z_min = 0.0
    trough_z_ext = trough_z_max - trough_z_min
    trough_z_cen = (trough_z_min + trough_z_max) / 2.0
    trough = _box((trough_x_ext, 34.0, trough_z_ext),
                  center=(trough_x_cen, 0.0, trough_z_cen))

    # ---- Bolt holes through the WHOLE pedestal + hub stack ----
    bolt_total_h = COXA_LIFT + hub_t
    hub_holes = []
    for a in HORN_BOLT_ANGLES_RAD:
        h = _cyl(HORN_BOLT_OD / 2.0, bolt_total_h * 4)
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              HORN_BOLT_PCD / 2.0 * np.sin(a),
                              bolt_total_h / 2.0])
        hub_holes.append(h)
    centre_hole = _cyl(HORN_CENTRE_OD / 2.0, bolt_total_h * 4)
    centre_hole.apply_translation([0, 0, bolt_total_h / 2.0])

    # ---- Femur-spar pass-through slot through arm + hub + pedestal roof
    # The femur spar at the hip end is FEMUR_SPAR_H = 34 mm tall in
    # femur z, on the spar centreline (femur y in [-LINK_THICKNESS/2,
    # +LINK_THICKNESS/2] = [-3, +3]).  Across the runtime hip-pitch
    # workspace (femur_pitch in [-80, +30] deg) the spar's TOP edge
    # (femur z = +17) sweeps a curve in coxa-link (x, z) that arcs
    # UP from neutral z = +23.5 INTO the arm plate's volume at
    # z in [COXA_LIFT, COXA_LIFT+arm_t] = [+26, +32] over a coxa-
    # link x range that depends on the pitch angle.  At any
    # NEGATIVE femur_pitch the spar's top edge crosses the arm's
    # bottom face somewhere between coxa-link x ~ +12 (at theta
    # = -80 deg, just past the hub's +X edge at +17) and coxa-link
    # x = arm_x_max = +41 (at theta ~ -10 deg).  In every case the
    # crossing happens on the spar centreline (femur y near 0), so
    # we cut a Y-narrow slot through the arm plate + outboard hub
    # at y in [-LINK_THICKNESS/2 - 0.5, +LINK_THICKNESS/2 + 0.5] =
    # [-3.5, +3.5] (LINK_THICKNESS-wide plus 0.5 mm FDM clearance on
    # each side).  The slot starts inboard at x = +8 (a few mm
    # INBOARD of the worst-case crossing point so the spar enters
    # the slot cleanly at theta = -80 deg) and ends outboard at x =
    # arm_x_max + 1.0 = +42 (a mm of overhang to make sure boolean
    # CSG punches through the arm's +X face).
    #
    # IMPORTANT (workspace-sweep fix): at theta = -80 deg the spar's
    # TOP flange (femur z = +17) also dips inboard + DOWN into the
    # PEDESTAL roof (the slab of coxa-link material above the
    # assembly trough at z in [trough_z_max, COXA_LIFT] =
    # [body_top_z + 1, COXA_LIFT]) at coxa-link x ~ +10..+17 and
    # z ~ +24..+32.  This is INSIDE the pedestal +X half (pedestal
    # x in [-17, +17]) and ABOVE the trough, so pad_sweep_clear
    # (radius HIP_PAD_R + 0.5 = 20 mm) does not catch it (the
    # collision sits at coxa-link distance 21.6 mm from the hip
    # axis, just OUTSIDE the pad disc).
    #
    # The fix is to extend the spar slot DOWN through the pedestal
    # roof so it meets the trough.  The slot is y-narrow (7 mm) and
    # lives only at x in [+8, +42] (so |y| > 3.5 of the pedestal +
    # both -X halves stay fully solid), and it intersects the +X
    # half of the pedestal only over x in [+8, +17] -- the rest of
    # the slot is outside the pedestal in air.  See
    # check_workspace_self_collision for the failing-pose dump that
    # picked these coordinates.
    # ---- Hip pad/neck swept-clearance cut through pedestal + hub ----
    # The femur's hip pad + neck-torus is a SOLID-walled cylinder of
    # OUTER radius HIP_PAD_R = 19.5 mm around the hip-pitch joint axis,
    # spanning femur y in [-LINK_THICKNESS/2, +LINK_THICKNESS +
    # HORN_STACK_H] = [-3, +15].  Across the runtime hip-pitch range
    # the pad's swept silhouette in coxa-link (x, z) is the same disk
    # of radius HIP_PAD_R about (COXA_LENGTH, ?, hip_drop) regardless
    # of pitch (the pad is rotationally symmetric about the hip-pitch
    # axis).  That disk reaches inboard to coxa-link x = COXA_LENGTH -
    # HIP_PAD_R = 25 - 19.5 = +5.5 mm, INSIDE the pedestal's +X face
    # (at x = +17 for the 34 x 34 pillar) and the hub's +X face (at
    # x = +17 as well).  Without a clearance cut the pad disk physically
    # overlaps the pedestal-solid region (z >= trough_z_max) by ~200-
    # 500 mm^3 even at neutral pose, which check_self_collision flags
    # as a coxa_link-vs-femur_link clash.
    #
    # The cut is a SHORT cylinder along the +Y axis, centred on the
    # hip axis at (COXA_LENGTH, 0, hip_drop) (the well_z_drop +
    # COXA_LIFT plane), with radius HIP_PAD_R + 0.5 (0.5 mm FDM
    # clearance).  Y extent spans the entire pedestal + hub stack
    # (link y in [-17, +17]) so the cut punches cleanly through both.
    pad_sweep_y_extent = 36.0
    pad_sweep_clear = _cyl(HIP_PAD_R + 0.5, pad_sweep_y_extent)
    pad_sweep_clear.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    hip_axis_z_in_link = well_z_drop + COXA_LIFT
    pad_sweep_clear.apply_translation([COXA_LENGTH, 0.0, hip_axis_z_in_link])

    spar_slot_y_half = LINK_THICKNESS / 2.0 + 0.5             # +/- 3.5
    spar_slot_x_min  = +8.0
    spar_slot_x_max  = arm_x_extent - 12.0 + 1.0              # arm +X end + 1
    # Extend DOWN past the pedestal roof and INTO the trough void
    # (trough_z_max = body_top_z + 1) so the boolean union of the
    # pedestal-roof spar-slot and the trough is a single contiguous
    # void.  Stop at trough_z_max - 0.5 (a 0.5 mm overlap with the
    # trough) so we don't accidentally cut deeper than needed.
    spar_slot_z_min  = body_top_z + 0.5
    spar_slot_z_max  = COXA_LIFT + hub_t + 0.1
    spar_slot = _box(
        (spar_slot_x_max - spar_slot_x_min,
         2.0 * spar_slot_y_half,
         spar_slot_z_max - spar_slot_z_min),
        center=((spar_slot_x_min + spar_slot_x_max) / 2.0,
                 0.0,
                 (spar_slot_z_min + spar_slot_z_max) / 2.0),
    )

    body = _union(pedestal, body_unlifted)
    return _diff(body, trough, spar_slot, pad_sweep_clear,
                 *hub_holes, centre_hole)


def make_femur_link() -> trimesh.Trimesh:
    """Femur (thigh).

    Local frame:
        +X = spar long-axis (hip-end at x=0, knee-end at x=FEMUR_LENGTH)
        +Y = pitch joint axis (parallel for hip and knee, points away
             from body)
        +Z = perpendicular to spar, in the leg's plane of motion

    Hip end: a flat 4-bolt pad that bolts to the hip-pitch servo's
    horn-adapter face.  This pad is perpendicular to Y (i.e. it lies
    in the X-Z plane), at y = 0.

    Knee end: an open-topped servo cradle that holds the knee servo,
    output shaft pointing +Y (parallel to the hip-pitch axis), so
    the tibia can be horn-driven and rotates in the same plane the
    femur does.

    Crucially the spar must NOT block the knee servo body's
    insertion path.  The body is 40 x 20 mm in cross-section and
    must be slid in along the +Y direction past the spar to reach
    the well; if the spar's z-extent overlaps the body's z-extent
    in the body's x-range, the body cannot be inserted at all.
    We solve this with two design tricks:

        1. The spar is taller than the body's short dimension
           (FEMUR_SPAR_H = 30 mm, SERVO_BODY_D = 20 mm), so cutting a
           20-mm-tall slot through it leaves 5-mm-thick top/bottom
           flanges that still tie the hip end to the knee end.
        2. We cut an insertion slot through the spar at the knee
           servo's x-range, and use two bridge flanges (above and
           below the body's z-extent) to connect the spar's flanges
           to the well's top and bottom walls.

    The femur prints with the spar's spar-Y axis vertical -- a flat
    plate 130 mm long, 30 mm tall, 6 mm thick -- so it lies on the
    build plate with no overhangs.
    """
    # ---- Spar (with insertion slot at the knee end) ------------------
    spar = _box((FEMUR_LENGTH, LINK_THICKNESS, FEMUR_SPAR_H),
                center=(FEMUR_LENGTH / 2.0, 0, 0))

    # Insertion slot for the knee servo's body.  The slot must be wide
    # enough to admit the SERVO TABS (SERVO_TAB_W = 54 mm), not just the
    # narrower body itself (SERVO_BODY_W = 40 mm) -- during insertion
    # the tab plane sweeps through the slot, so the tabs must clear the
    # slot side walls.  Earlier this was sized for SERVO_BODY_W + 2 mm
    # = 42 mm and the tabs caught on the slot edges when the servo was
    # slid in along +Y.
    body_x_centre = FEMUR_LENGTH - SERVO_OUTPUT_X
    body_x_min = body_x_centre - SERVO_TAB_W / 2.0 - 1.0   # tab span + 1 mm
    body_x_max = body_x_centre + SERVO_TAB_W / 2.0 + 1.0
    slot_x = body_x_max - body_x_min                         # 56 mm
    slot_z = SERVO_BODY_D + 2.0                              # 22 mm
    insertion_slot = _box((slot_x, LINK_THICKNESS + 2.0, slot_z),
                           center=((body_x_min + body_x_max) / 2.0,
                                    0, 0))

    # ---- Hip-end pad + neck -----------------------------------------
    # The femur's hip pad bolts onto the hip-pitch horn adapter, which
    # itself sits on the servo's plastic horn + spline.  In femur-local
    # coords the joint AXIS (spline tip) is at y = 0, and the horn
    # adapter's TOP face (where the pad clamps) is at y = HORN_STACK_H
    # = 9 mm.  Placing the pad at y = 0 (the OLDEST design) put it
    # inside the cradle's "swept volume" above the body and caused the
    # femur vs coxa_bracket / femur vs tibia self-collision failures.
    #
    # Current design (rotationally-symmetric hollow-annulus neck):
    #
    #   1. HIP PAD: a solid disc at y in [+9, +15], radius HIP_PAD_R =
    #      19.5 mm.  The bolt-clamp ring; sits flat on the horn adapter
    #      top face at y = +9 and carries the 4 M3 clamp bolts on
    #      HORN_BOLT_PCD = 20.8 mm.  HIP_PAD_R was bumped from 17 mm to
    #      19.5 mm so the neck (item 2) has a 3 mm-thick printable wall
    #      around the horn-stack clearance void.
    #   2. NECK TORUS: a hollow cylinder at y in [-LINK_THICKNESS/2,
    #      +HORN_STACK_H] = [-3, +9], with outer radius HIP_PAD_R and
    #      inner radius (HORN_ADAPTER_OD/2 + HORN_STACK_CLEARANCE) =
    #      16.5 mm.  The OUTER wall is a 3 mm-thick rotationally-
    #      symmetric ring.  Inside that ring, the cylindrical inner void
    #      is EXACTLY the horn-stack clearance volume that the plastic
    #      horn (Phi ~20 mm, y in [0, +5]) and the printed horn adapter
    #      (Phi HORN_ADAPTER_OD = 32 mm, y in [+5, +9]) need to occupy
    #      at assembly time -- the void is checked by
    #      ``check_horn_stack_clearance`` in _verify_prototype.py.
    #
    #      The torus extends DOWN past y = +3 (the spar's +Y face) into
    #      y in [-3, +3], so the union with the spar is a real 6-mm-
    #      tall fused section spanning the spar's full y range rather
    #      than a 0 mm boolean kiss at y = +3.  At the +X side the
    #      torus's +X-facing material (sqrt(x^2 + z^2) in [16.5, 19.5]
    #      with x > 0) intersects the spar's hip-end region (femur x in
    #      [13.83, 19.5], z in [-FEMUR_SPAR_H/2, +FEMUR_SPAR_H/2]) over
    #      a contiguous "C"-shape on the spar's +X side, giving a
    #      ~36 mm^2 contact patch on the spar's +Y face plus an equal
    #      patch on the -Y face -- well above the hip-pitch reaction
    #      torque shear demand (< 10 N over the patch).
    #
    #      The earlier "neck strip" design (a 6 mm-wide slab on the
    #      spar centreline) put SOLID material at radius 0...
    #      LINK_THICKNESS/2 along the +Y direction, which clipped the
    #      horn-stack volume by ~ 2700 mm^3 (femur) / 2000 mm^3 (tibia)
    #      and made the adapter physically un-installable.
    #      check_horn_stack_clearance catches this exact failure.
    HORN_STACK_VOID_R = HORN_ADAPTER_OD / 2.0 + 0.5      # 16.5 mm
    hip_pad_centre_y = HORN_STACK_H + LINK_THICKNESS / 2.0    # +12
    hip_pad_y_min    = HORN_STACK_H                            # +9
    hip_pad_y_max    = HORN_STACK_H + LINK_THICKNESS           # +15
    hip_pad = _cyl_along(HIP_PAD_R,
                          hip_pad_y_max - hip_pad_y_min,
                          axis="y")
    hip_pad.apply_translation([0, hip_pad_y_min, 0])

    # Neck torus: outer cylinder + inner cylindrical void.  Spans y in
    # [-LINK_THICKNESS/2, +HORN_STACK_H] = [-3, +9] so it overlaps the
    # spar's full y range AND the horn-stack y range simultaneously.
    neck_y_min = -LINK_THICKNESS / 2.0                          # -3
    neck_y_max = HORN_STACK_H                                   # +9
    neck_y_extent = neck_y_max - neck_y_min                     # 12
    neck_y_centre = (neck_y_min + neck_y_max) / 2.0             # +3
    hip_neck_outer = _cyl_along(HIP_PAD_R, neck_y_extent, axis="y")
    hip_neck_outer.apply_translation([0, neck_y_min, 0])
    hip_neck_void = _cyl_along(HORN_STACK_VOID_R, neck_y_extent + 2.0,
                                axis="y")
    hip_neck_void.apply_translation([0, neck_y_min - 1.0, 0])

    hip_holes = []
    for a in HORN_BOLT_ANGLES_RAD:
        # Drill the 4 M3 clamp holes ONLY through the pad's 6 mm-thick
        # clamp ring (y in [+9, +15]).  The hole is a short Phi
        # SERVO_TAB_HOLE = 3.2 mm cylinder centred on the pad's mid-Y
        # plane (y = +12); the cylinder length = LINK_THICKNESS * 4 =
        # 24 mm guarantees a clean punch-through of the 6 mm pad even
        # with voxel/CSG tolerance, but does NOT reach back into the
        # neck torus body (the torus's +Y face is at y = +9; the hole
        # extends from y = 0 to y = +24 so it overlaps the torus's
        # cylindrical wall at radius HORN_BOLT_PCD/2 = 12 mm).  That
        # radial position is INSIDE the torus's void (radius < 16.5),
        # so the bolt hole punches through air inside the torus and
        # doesn't actually remove any neck material.
        h = _cyl(SERVO_TAB_HOLE / 2.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              hip_pad_centre_y,
                              HORN_BOLT_PCD / 2.0 * np.sin(a)])
        hip_holes.append(h)

    # ---- Knee-end servo well -----------------------------------------
    # NB: the 4 M3 mounting pilots in the wall (drilled by
    # ``_servo_well_solid``) sit on the standard SERVO_TAB_HOLE_PCD x
    # SERVO_TAB_HOLE_PCD_Y (49.5 x 10 mm) pattern -- NOT the 24 mm
    # HORN_BOLT_PCD pattern.  HORN_BOLT_PCD is the bolt circle for the
    # hip-pad horn adapter at the OTHER end of the femur.
    well = _servo_well_solid()
    wire_slot = _wire_exit_slot()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])    # well +Z -> femur +Y
    well.apply_transform(R)
    wire_slot.apply_transform(R)
    delta = np.array([FEMUR_LENGTH - SERVO_OUTPUT_X,
                       -(SERVO_BODY_H + SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)
    wire_slot.apply_translation(delta)

    # ---- Two bridge flanges (top + bottom) ---------------------------
    # The body's z range is +/-(SERVO_BODY_D/2) = +/-10.  The spar's z
    # range after slot cut is [SERVO_BODY_D/2+1, FEMUR_SPAR_H/2] =
    # [+11, +17] (top flange) and [-17, -11] (bottom flange).  The well
    # wraps z in [-WELL_D/2, +WELL_D/2] = [-12.5, +12.5].  Each bridge
    # connects a spar flange to the well's wall at the same z.
    spar_far_y      =  LINK_THICKNESS / 2.0           # +3 (spar's +Y face)
    spar_near_y     = -LINK_THICKNESS / 2.0           # -3 (spar's -Y face)
    well_near_y     = WELL_RIM_Z + delta[1]           # well's +Y face

    # Embed the bridge 2.5 mm INTO the well's +Y wall (instead of the
    # previous 0.5 mm kiss).  The well's +Y wall material around the
    # cavity opening is at z in roughly [+10.7, +12.5] / [-12.5, -10.7]
    # (cavity z half-extent = SERVO_BODY_D/2 + WELL_BODY_CL).  Deeper
    # embedment turns the bridge-to-well joint from a 0.5 mm boolean
    # kiss into a real ~ 4.5 mm-thick fused section per side, which is
    # by far the weakest joint in the part.
    BRIDGE_WELL_EMBED = 2.5                           # mm into well +Y wall
    bridge_y_min    = well_near_y - BRIDGE_WELL_EMBED
    # Stop the bridge at the spar's CENTRELINE (y = 0) rather than 0.5 mm
    # shy of the spar's +Y face (the old bridge_y_max = +2.5).  Two
    # consequences:
    #
    #   (a) Real volumetric fuse with the spar: the bridge overlaps the
    #       spar over y in [-3, 0] = 3 mm of solid material instead of a
    #       0.5 mm boolean kiss at y = [+2.5, +3].
    #   (b) The 3 mm of femur material at y in [0, +3] (the spar's +Y
    #       half) is now the ONLY femur material between the well rim
    #       (femur y = -16.75) and the tibia-mounting clearance zone
    #       (femur y >= +3); the bridge cap no longer reaches past the
    #       spar's centreline in +Y.  This gives the tibia's knee pad
    #       neck disk (a 34 mm dia. cylinder centred on the knee axis,
    #       starting at femur y = +3) 3 mm of guaranteed bridge clearance
    #       in the y direction.
    #
    # Why this is the right fix (not a "trim the well walls" fix):
    #   - well rim plane in femur-local Y is at femur y = WELL_RIM_Z
    #     + delta[1] = +27.25 - 44 = -16.75; the well OUTER box itself
    #     does NOT extend past this plane in +Y, so the well walls are
    #     already clear of the tibia mounting zone at femur y > +3.
    #   - The structural members that DO live at femur y > -16.75 are
    #     the spar (y in [-3, +3]), the hip-end pad/neck cylinder (only
    #     at x ~ 0, the hip end, far from the knee), and these bridges.
    #   - Earlier bridge_y_max = +2.5 put the bridge cap's +Y face
    #     INSIDE the FDM-tolerance window (0.3-0.8 mm/side over-extrude)
    #     of the tibia knee-pad neck disk at y = +3 -- real prints
    #     reported the disk seating proud of the horn-adapter face on
    #     the bridge-cap side.  BRIDGE_CAP_H = 6 controls the bridge's
    #     femur-Z extent (height above the spar's top edge), NOT its
    #     femur-Y extent; "the bridge cap extends 6 mm in well +Z past
    #     the well rim" was a coordinate-axis misread.  The well walls
    #     remain untouched.
    bridge_y_max    = 0.0
    bridge_y_extent = bridge_y_max - bridge_y_min
    bridge_y_centre = (bridge_y_min + bridge_y_max) / 2.0
    bridge_x_extent = slot_x                          # span the body's x range
    bridge_x_centre = (body_x_min + body_x_max) / 2.0

    # Top flange bridge: z spans [body_top, spar_top + BRIDGE_CAP_H] so
    # it overlaps the well's top wall, the spar's top flange, AND
    # extends BRIDGE_CAP_H mm PAST the spar's top edge as an integral
    # cap rib.  Without the cap, the bridge cross-section is just
    # (spar_z_max - body_z_max) = 7 mm tall by 18 mm wide.
    # BRIDGE_CAP_H was 4 mm -- with the previous WELL_WALL_Y = 2.5 the
    # well's top wall was only 1.8 mm thick in Z, so the slab of
    # bridge material directly above the cavity (z in [+body_z_max,
    # +well_top_z] = [10, 12.5]) had bridge underside meeting well
    # outside face across a region that voxelised down to a 1-voxel
    # slab and registered as a 933-voxel flimsy cluster.  Bumping the
    # cap to 6 mm widens the bridge's Z cross-section to 13 mm and,
    # together with WELL_WALL_Y = 3.7 (3.0 mm real wall in this
    # direction), removes the thin-slab artefact entirely.
    BRIDGE_CAP_H = 6.0
    body_z_max  = +SERVO_BODY_D / 2.0                 # +10
    spar_z_max  = +FEMUR_SPAR_H / 2.0                 # +17
    bridge_top_z_min = body_z_max                     # +10
    bridge_top_z_max = spar_z_max + BRIDGE_CAP_H      # +21
    bridge_top_z_extent = bridge_top_z_max - bridge_top_z_min
    bridge_top_z_centre = (bridge_top_z_min + bridge_top_z_max) / 2.0
    bridge_top = _box((bridge_x_extent, bridge_y_extent,
                       bridge_top_z_extent),
                      center=(bridge_x_centre, bridge_y_centre,
                               bridge_top_z_centre))

    bridge_bot_z_min = -bridge_top_z_max              # -21
    bridge_bot_z_max = -bridge_top_z_min              # -10
    bridge_bot_z_extent = bridge_top_z_extent
    bridge_bot_z_centre = -bridge_top_z_centre
    bridge_bot = _box((bridge_x_extent, bridge_y_extent,
                       bridge_bot_z_extent),
                      center=(bridge_x_centre, bridge_y_centre,
                               bridge_bot_z_centre))

    # Re-cut the well's body cavity from the bridges so the deeper
    # well embedment (BRIDGE_WELL_EMBED above) doesn't refill the
    # cavity and block servo insertion.  The cavity volume here mirrors
    # the box that ``_servo_well_solid`` subtracts internally, then is
    # transformed into femur-frame.
    cav_z_bot  = -WELL_FLOOR_T - 1.0
    cav_z_top  = WELL_RIM_Z
    cavity_trim = _box((SERVO_BODY_W + 2 * WELL_BODY_CL,
                        SERVO_BODY_D + 2 * WELL_BODY_CL,
                        cav_z_top - cav_z_bot),
                       center=(0, 0, 0.5 * (cav_z_top + cav_z_bot)))
    cavity_trim.apply_transform(R)
    cavity_trim.apply_translation(delta)

    # ---- Lightening holes through the spar ---------------------------
    # Also used to pass jacketed servo harnesses through the spar
    # (radius must clear the plastic bundle, not just the conductors).
    # Only at x < body_x_min so they don't accidentally cut the slot
    # walls or the bridges.
    lightening = []
    n_holes = 2
    for i in range(n_holes):
        x = (i + 1) * body_x_min / (n_holes + 1)
        h = _cyl(6.5, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([x, 0, 0])
        lightening.append(h)

    # ---- Knee-end clearance void for the tibia's pad/neck annulus ----
    # The tibia's knee-pad + neck-torus is a Phi (2 * HIP_PAD_R) = 39 mm
    # cylinder centred on the knee axis (femur x = FEMUR_LENGTH = 90),
    # spanning tibia y in [-LINK_THICKNESS/2, +HORN_STACK_H + 6] =
    # [-3, +15] = femur y in [-3, +15] (rotation about Y preserves Y).
    # The tibia rotates relative to the femur about the knee Y axis, so
    # the swept silhouette of the tibia pad + neck-torus in femur (x, z)
    # is a Phi 39 mm DISK about (FEMUR_LENGTH, _, 0).  At any non-trivial
    # knee pitch (including STANCE_TIBIA_DEG = +60 deg) the tibia neck
    # punches deep into the femur's spar / bridge volume -- without a
    # clearance cut the tibia knee-pad neck +X tip enters the femur's
    # +Y spar top flange (z >= 11 mm, the post-insertion-slot top
    # flange) and registers as a ~ 800 mm^3 self-collision in
    # check_self_collision.
    #
    # The cut is a Phi (HIP_PAD_R + 0.5) * 2 = 40 mm cylinder along
    # +Y, centred on (FEMUR_LENGTH, 0, 0), spanning femur y in
    # [-LINK_THICKNESS/2, HORN_STACK_H] = [-3, +9] -- exactly the
    # tibia's neck-torus y range (we don't need to clear the pad's
    # y in [+9, +15] zone because the pad sits ABOVE the femur's spar
    # and bridge volume in Y).
    # Use a slightly larger clearance radius (+2.5 mm vs the tibia
    # neck's nominal +0.5 mm clearance) to absorb voxel-stair-step
    # discretisation along the tibia neck's curved boundary AND to
    # account for the tibia's swept volume actually being a CYLINDER
    # of radius HIP_PAD_R + 0.5 (the neck-torus outer wall plus the
    # 0.5 mm horn-stack clearance applied to the OUTER boundary by
    # voxelisation).
    knee_clear_R = HIP_PAD_R + 2.5
    knee_clear_y_extent = HORN_STACK_H + LINK_THICKNESS / 2.0 + 1.0
    knee_clear = _cyl(knee_clear_R, knee_clear_y_extent)
    knee_clear.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    knee_clear.apply_translation(
        [FEMUR_LENGTH, (HORN_STACK_H - LINK_THICKNESS / 2.0) / 2.0, 0.0]
    )

    body = _union(hip_pad, hip_neck_outer, spar, well,
                   bridge_top, bridge_bot)
    return _diff(body, hip_neck_void, insertion_slot, wire_slot,
                 cavity_trim, knee_clear, *hip_holes, *lightening)


def make_tibia_link() -> trimesh.Trimesh:
    """Tibia (shin).

    Local frame:
        origin = KNEE joint axis (servo output spline centre)
        +X = spar long-axis (foot socket at x = TIBIA_LENGTH)
        +Y = knee joint axis (parallel to the hip-pitch axis)
        +Z = perpendicular to spar, in the leg's plane of motion

    Knee end: a square pad centred on the joint axis (x=0, z=0) with
    the 4 horn bolt holes drilled in Y.  Bolt-circle CENTRE is on
    the joint axis so the tibia rotates rigidly with the horn.
    Foot end: a forked CLEVIS that captures the foot pad's tongue
    (see ``make_foot_pad``) on an M3 pin parallel to the knee axis.
    The clevis is a 18-mm-long fattened section of the spar tip with
    a Y-axis through-hole at z = FOOT_HINGE_TIBIA_Z.
    """
    spar = _box((TIBIA_LENGTH, LINK_THICKNESS, TIBIA_SPAR_H),
                center=(TIBIA_LENGTH / 2.0, 0, 0))

    # Knee pad + neck: same HOLLOW-ANNULUS-NECK design as the femur's
    # hip-end.  See make_femur_link for the design rationale; the only
    # difference here is the spar height (TIBIA_SPAR_H = 18 vs
    # FEMUR_SPAR_H = 34).  With HIP_PAD_R = 19.5 mm and the void radius
    # = HORN_ADAPTER_OD/2 + 0.5 = 16.5 mm, the neck torus has a 3 mm
    # printable wall around the horn-stack clearance volume.
    HORN_STACK_VOID_R = HORN_ADAPTER_OD / 2.0 + 0.5      # 16.5 mm
    knee_pad_centre_y = HORN_STACK_H + LINK_THICKNESS / 2.0    # +12
    knee_pad_y_min    = HORN_STACK_H                            # +9
    knee_pad_y_max    = HORN_STACK_H + LINK_THICKNESS           # +15
    knee_pad = _cyl_along(HIP_PAD_R,
                           knee_pad_y_max - knee_pad_y_min,
                           axis="y")
    knee_pad.apply_translation([0, knee_pad_y_min, 0])

    # Neck torus: outer solid annulus + inner cylindrical void.  Y
    # span [-LINK_THICKNESS/2, +HORN_STACK_H] = [-3, +9] so the torus
    # overlaps the spar's full +/- LINK_THICKNESS/2 y range AND the
    # horn-stack y range.  For the tibia the spar is only TIBIA_SPAR_H
    # = 18 mm tall in z, so the spar's z range = [-9, +9] sits fully
    # INSIDE the horn-stack cylinder (radius 16.5).  The neck-spar
    # union therefore only fuses where the spar's +X tip enters the
    # annulus's outer wall -- tibia x in [13.83, 19.5] crosses
    # sqrt(x^2 + z^2) >= 16.5 for some z in [-9, +9].  That contact
    # patch is small (~ 18 mm^2 per +/- Y face) but the neck's full
    # +/- 19.5 mm-radius annulus and its 12 mm Y extent give the joint
    # plenty of stiffness in shear.
    neck_y_min = -LINK_THICKNESS / 2.0                          # -3
    neck_y_max = HORN_STACK_H                                   # +9
    neck_y_extent = neck_y_max - neck_y_min                     # 12
    knee_neck_outer = _cyl_along(HIP_PAD_R, neck_y_extent, axis="y")
    knee_neck_outer.apply_translation([0, neck_y_min, 0])
    knee_neck_void = _cyl_along(HORN_STACK_VOID_R, neck_y_extent + 2.0,
                                 axis="y")
    knee_neck_void.apply_translation([0, neck_y_min - 1.0, 0])

    knee_holes = []
    for a in HORN_BOLT_ANGLES_RAD:
        # Bolt holes drilled through the PAD ONLY (y in [+9, +15]);
        # the neck torus's bolt-PCD radius (12 mm) is INSIDE the
        # horn-stack void (16.5 mm), so any over-long hole punches
        # through air inside the torus and doesn't remove material.
        h = _cyl(SERVO_TAB_HOLE / 2.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              knee_pad_centre_y,
                              HORN_BOLT_PCD / 2.0 * np.sin(a)])
        knee_holes.append(h)

    # ----- Foot clevis at the far end (x ~ TIBIA_LENGTH) -----
    # The clevis is a single forked block built in two steps:
    #
    #   1. ``clevis_bulk`` = a solid box that flares the spar tip out
    #      to the full fork width (2*CHEEK_T + GAP = 12 mm) in Y and
    #      drops down past the spar's bottom face to the chosen pin
    #      depth, so the M3 pin axis lives clearly below the spar.
    #   2. ``clevis_slot`` = a Y-axis slab cut through the centre of
    #      the bulk that removes the GAP-wide strip of material in the
    #      middle of the block, leaving two CHEEK_T-thick cheeks on
    #      either side of an open mouth that faces -Z (toward the
    #      ground).  The foot pad's vertical tongue slides up into
    #      this mouth from below.
    #
    # The M3 through-hole is drilled along Y at (x = TIBIA_LENGTH,
    # z = FOOT_HINGE_TIBIA_Z) so the bolt passes through both cheeks
    # and the foot's tongue.  Hinge axis is parallel to the knee axis
    # by construction (both are tibia-local +Y), so the foot pitches
    # passively about an axis parallel to the knee servo's axis --
    # exactly the ankle pitch the user asked for.
    clevis_x_min = TIBIA_LENGTH - FOOT_CLEVIS_X_INBOARD
    clevis_x_max = TIBIA_LENGTH + FOOT_CLEVIS_X_BEYOND_TIP
    clevis_dx    = clevis_x_max - clevis_x_min
    clevis_cx    = (clevis_x_min + clevis_x_max) / 2.0
    clevis_full_y = FOOT_HINGE_GAP + 2.0 * FOOT_HINGE_CHEEK_T   # 12 mm

    clevis_z_min = FOOT_HINGE_TIBIA_Z - FOOT_CLEVIS_CHEEK_BELOW_PIN  # -15
    clevis_z_max = TIBIA_SPAR_H / 2.0                                # +9
    clevis_bulk  = _box((clevis_dx,
                          clevis_full_y,
                          clevis_z_max - clevis_z_min),
                         center=(clevis_cx, 0.0,
                                  (clevis_z_max + clevis_z_min) / 2.0))

    # Slot: GAP-wide strip in Y, open at the -Z (bottom) face of the
    # bulk so the foot tongue has a clear vertical mouth.  Top of the
    # slot stops a few mm above the pin axis so each cheek still has
    # a continuous ring of material around the M3 hole.
    slot_z_min = clevis_z_min - 1.0                # overshoot bottom -> open mouth
    slot_z_max = FOOT_HINGE_TIBIA_Z + FOOT_HINGE_PIN_HOLE_D / 2.0 + 3.0
    slot_x_min = clevis_x_min - 0.5                # overshoot in X -> through-cut
    slot_x_max = clevis_x_max + 0.5
    clevis_slot = _box((slot_x_max - slot_x_min,
                         FOOT_HINGE_GAP,
                         slot_z_max - slot_z_min),
                        center=((slot_x_min + slot_x_max) / 2.0, 0.0,
                                 (slot_z_min + slot_z_max) / 2.0))

    # Pin hole through both cheeks (and the empty slot in between).
    # Length = 2 * clevis_full_y to guarantee a clean cut through
    # both cheeks even with FDM/Hildebrand voxelisation slop.
    pin_hole = _cyl(FOOT_HINGE_PIN_HOLE_D / 2.0, clevis_full_y * 2.0)
    pin_hole.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
    pin_hole.apply_translation([TIBIA_LENGTH, 0.0, FOOT_HINGE_TIBIA_Z])

    # A short taper to blend the 6-mm-thick spar into the 12-mm-wide
    # clevis bulk.  Sits on the spar centreline, narrower than the
    # bulk so it tapers visually but still adds bending stiffness.
    taper = _box((24.0, LINK_THICKNESS * 0.95, TIBIA_SPAR_H * 0.6),
                 center=(TIBIA_LENGTH - 12.0, 0, -3.0))

    lightening = []
    n_holes = 4
    for i in range(n_holes):
        x = (i + 1) * TIBIA_LENGTH / (n_holes + 2)
        h = _cyl(5.5, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([x, 0, 0])
        lightening.append(h)

    body = _union(knee_pad, knee_neck_outer, spar, taper, clevis_bulk)
    return _diff(body, knee_neck_void, *knee_holes, clevis_slot, pin_hole,
                 *lightening)


def make_foot_pad() -> trimesh.Trimesh:
    """Compliant foot pad with a single-axis hinge tongue.

    Stack (foot-local Z, +Z up):

        ground   z = 0 .. FOOT_PAD_BASE_H               -- TPU spring disk
        boss     z = FOOT_PAD_BASE_H
                   .. FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H -- 14 mm OD stub
        tongue   z = boss top
                   .. FOOT_HINGE_FOOT_Z + OVER_PIN      -- 4 x 10 mm tongue
                                                          rising through the
                                                          M3 pin axis

    The tongue is sized to slide into the tibia's clevis slot (4 mm
    Y-thick into a 5 mm gap with 0.5 mm clearance per side).  A
    horizontal M3 clearance hole through the tongue lines up with the
    cheek holes, captured by an M3 x 16 pan-head bolt + nylock nut.

    The hinge axis (tibia-local +Y) is parallel to the knee axis, so
    when the tibia pitches the foot follows through ankle pitch.  TPU
    compliance in the disk itself absorbs roll.

    Local frame: ground-plane at Z = 0; tongue rises in +Z; the
    tongue's broad faces have normals +/-Y (matches the tibia's
    knee-axis direction in the leg's local frame)."""
    pad_base = _cyl(FOOT_PAD_OD / 2.0, FOOT_PAD_BASE_H)
    pad_base.apply_translation([0, 0, FOOT_PAD_BASE_H / 2.0])

    boss = _cyl(FOOT_PAD_BOSS_OD / 2.0, FOOT_PAD_BOSS_H)
    boss.apply_translation([0, 0, FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H / 2.0])

    tongue_z_lo = FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H
    tongue_z_hi = FOOT_HINGE_FOOT_Z + FOOT_HINGE_TONGUE_OVER_PIN
    tongue = _box((FOOT_HINGE_TONGUE_X,
                    FOOT_HINGE_TONGUE_T,
                    tongue_z_hi - tongue_z_lo),
                   center=(0.0, 0.0,
                            (tongue_z_hi + tongue_z_lo) / 2.0))

    hinge_hole = _cyl(FOOT_HINGE_PIN_HOLE_D / 2.0,
                       FOOT_HINGE_TONGUE_T * 4.0)
    hinge_hole.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
    hinge_hole.apply_translation([0.0, 0.0, FOOT_HINGE_FOOT_Z])

    return _diff(_union(pad_base, boss, tongue), hinge_hole)


# ---------------------------------------------------------------------------
# Assembly preview
# ---------------------------------------------------------------------------

def _leg_in_body_frame(leg_index: int) -> trimesh.Trimesh:
    """Return one leg, transformed into the chassis frame, in standing
    pose, as a single concatenated mesh.

    Kinematic chain (mirrors hexapod_walker._leg_in_body_frame):

       coxa_bracket  ->  yaw (Z axis)  ->  coxa_link  ->
       hip_pitch (Y_local axis)        ->  femur      ->
       knee_pitch (Y_local axis)       ->  tibia      ->  foot.
    """
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    outboard = np.array([np.cos(a), np.sin(a), 0.0])
    z_hat    = np.array([0.0, 0.0, 1.0])

    parts = []

    # ------------------- Coxa bracket (chassis-fixed) ------------------
    cb = make_coxa_bracket()
    cb.apply_transform(rotation_matrix(a, [0, 0, 1]))
    # Bracket's pad inboard face is at x = -PAD_T/2 = -2 in local; we
    # want that to land at the chassis edge, so translate by edge_mid -
    # CHASSIS_TUBE/2 outboard.  The hex plate's perimeter is at the
    # apothem, so the bracket's pad sits flush on top of (or under) the
    # plate.
    cb.apply_translation(edge_mid)
    parts.append(cb)

    # ------------------- Coxa link (rotates with yaw) ------------------
    # The coxa link's hub bottom (z = 0 in its local frame) sits on top
    # of the yaw servo's horn adapter.  Vertical stack from the chassis
    # plate's TOP face up to the bottom of the coxa link:
    #   well rim (= bracket origin)          z = 0
    #   body top (= rim + 12.25)             z = 12.25
    #   gear stack top (+ 6)                 z = 18.25
    #   plastic horn top (+ ~ 5)             z = 23.25
    #   horn adapter top (+ HORN_ADAPTER_T)  z = 27.25 = coxa-link z=0
    PLASTIC_HORN_H = 5.0   # mm, hobby-servo plastic horn height
    yaw_output_z = ((SERVO_BODY_H - WELL_RIM_Z)
                     + SERVO_OUTPUT_H
                     + PLASTIC_HORN_H
                     + HORN_ADAPTER_T)

    cl = make_coxa_link()
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts.append(cl)

    # ------------------- Femur (pitched about leg-Y) ------------------
    # In coxa-link local coords, the hip-pitch servo's output spline
    # tip is at (COXA_LENGTH, 0, COXA_HIP_DROP) where COXA_HIP_DROP is
    # the module-level constant that mirrors the well_z_drop +
    # COXA_LIFT formula used inside make_coxa_link.  See COXA_HIP_DROP's
    # docstring near the top of this file for the derivation.
    hip_joint_local = np.array([COXA_LENGTH, 0.0, COXA_HIP_DROP])

    fl = make_femur_link()
    fl.apply_transform(rotation_matrix(np.deg2rad(STANCE_FEMUR_DEG),
                                        [0, 1, 0]))
    # Femur's local origin sits ON the hip joint axis (the bolt-circle
    # centre of its hip pad), so we translate the femur so (0,0,0)
    # lands on hip_joint_local.
    fl.apply_translation(hip_joint_local)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts.append(fl)

    # ------------------- Tibia ----------------------------------------
    # In femur-local coords the knee joint axis is at (FEMUR_LENGTH,
    # 0, 0) -- right on the spar centreline at the spar's far end.
    # After femur rotation about Y by `p` it becomes
    # R_y(p) @ (FEMUR_LENGTH, 0, 0) in coxa-link local, plus the
    # femur's hip-end translation.
    p  = np.deg2rad(STANCE_FEMUR_DEG)
    pt = np.deg2rad(STANCE_FEMUR_DEG + STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + Ry_p @ np.array([FEMUR_LENGTH,
                                                            0.0, 0.0])

    tl = make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local)
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts.append(tl)

    # ------------------- Foot at tibia tip ----------------------------
    # The foot pivots about the hinge pin captured by the tibia's
    # clevis at (TIBIA_LENGTH, 0, FOOT_HINGE_TIBIA_Z) in tibia-local.
    # We place the foot pad so its own hinge hole (at foot-local
    # (0, 0, FOOT_HINGE_FOOT_Z)) lands on the same world point.  The
    # foot is NOT rotated to follow the tibia's pitch -- it hangs on
    # the hinge so its disk stays roughly horizontal on the ground
    # (mimicking passive ankle compliance).  We do rotate it about Z
    # by the leg's azimuth ``a`` so the foot's tongue (whose broad
    # faces have normals +/-Y in foot-local) lines up with the
    # tibia's knee-axis Y (also rotated by a).
    Ry_pt = rotation_matrix(pt, [0, 1, 0])[:3, :3]
    hinge_local = knee_joint_local + Ry_pt @ np.array(
        [TIBIA_LENGTH, 0.0, FOOT_HINGE_TIBIA_Z]
    )
    R_a = rotation_matrix(a, [0, 0, 1])[:3, :3]
    hinge_world = R_a @ hinge_local + edge_mid + yaw_output_z * z_hat

    foot = make_foot_pad()
    foot.apply_transform(rotation_matrix(a, [0, 0, 1]))
    foot.apply_translation([hinge_world[0], hinge_world[1],
                             hinge_world[2] - FOOT_HINGE_FOOT_Z])
    parts.append(foot)

    return _union(*parts)


def make_assembly_preview() -> trimesh.Trimesh:
    """Build the full prototype hexapod in standing pose for visual
    checks.  Computes chassis_lift directly from the leg geometry so
    the foot pads always land on z = 0."""
    probe_leg = _leg_in_body_frame(0)
    z_min = float(probe_leg.bounds[0][2])
    chassis_lift = -z_min

    parts = []

    # Bottom chassis plate (at z = chassis_lift)
    bot = make_chassis_bottom()
    bot.apply_translation([0, 0, chassis_lift])
    parts.append(bot)

    # Top chassis plate (above the bottom by CHASSIS_GAP)
    top = make_chassis_top()
    top.apply_translation([0, 0, chassis_lift + CHASSIS_GAP
                                + CHASSIS_PLATE_T])
    parts.append(top)

    # Battery holder (sits between the plates, slightly aft of centre)
    bh = make_battery_holder()
    bh.apply_translation([-25.0, 0, chassis_lift + CHASSIS_PLATE_T])
    parts.append(bh)

    # Electronics tray (sits between the plates, forward of centre)
    et = make_electronics_tray()
    et.apply_translation([35.0, 0, chassis_lift + CHASSIS_PLATE_T + 1.0])
    parts.append(et)

    # Six legs
    for i in range(6):
        leg = _leg_in_body_frame(i)
        leg.apply_translation([0, 0, chassis_lift])
        parts.append(leg)

    preview = _union(*parts)

    # Z-up -> Y-up so default STL viewers show the walker upright
    preview.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    return preview


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("Hexapod walker PROTOTYPE -- generating STLs in stl_prototype/ ...")

    parts: list[tuple[str, trimesh.Trimesh]] = []

    print("Body parts:")
    parts.append(("chassis_top.stl",      make_chassis_top()))
    parts.append(("chassis_bottom.stl",   make_chassis_bottom()))
    parts.append(("battery_holder.stl",   make_battery_holder()))
    parts.append(("electronics_tray.stl", make_electronics_tray()))

    print("Leg parts (one of each -- print 6 sets):")
    parts.append(("coxa_bracket.stl",     make_coxa_bracket()))
    parts.append(("coxa_link.stl",        make_coxa_link()))
    parts.append(("femur_link.stl",       make_femur_link()))
    parts.append(("tibia_link.stl",       make_tibia_link()))
    parts.append(("foot_pad.stl",         make_foot_pad()))

    print("Generic horn adapter (print 18 + spares):")
    parts.append(("servo_horn_adapter.stl", make_servo_horn_adapter()))

    print("Servo visuals (not for printing -- MuJoCo / fit-check meshes):")
    parts.append(("servo_body.stl", make_servo_body()))
    parts.append(("servo_horn.stl", make_servo_horn()))

    for name, mesh in parts:
        _save(mesh, name)

    print("Assembly preview (everything in standing pose):")
    preview = make_assembly_preview()
    _save(preview, "assembly_preview.stl")

    # ----- Final summary -----
    total_faces = sum(len(m.faces) for _, m in parts) + len(preview.faces)
    foot_to_foot    = preview.extents[0]
    standing_height = preview.extents[1]
    print()
    print(f"OK -- {len(parts) + 1} STL files written.")
    print(f"   Vehicle envelope (foot to foot):  {foot_to_foot/10:.1f} cm")
    print(f"   Vehicle standing height:          {standing_height/10:.1f} cm")
    print(f"   Total geometry triangle count:    {total_faces:,}")
    print()
    print("Estimated parts cost:  ~$150 - $250 in 2026 USD.")
    print("See PROTOTYPE.md for the BOM and wiring guide.")


if __name__ == "__main__":
    main()
