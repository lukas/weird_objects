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

    Generic (DEPRECATED -- Design B retired the printed adapter; see
    HORN_ADAPTER_OD / make_servo_horn_adapter below.  Kept only for
    backward-compatible STL references in old quotes.)
        servo_horn_adapter.stl  -- round disc that bolted to a standard 25T spline
                                   horn and provided 4 x M3 holes on a 20.8 mm PCD
                                   at 0 / 90 / 180 / 270 deg (aligned with the
                                   X-horn arms) so a flat printed link could mate
                                   to a servo horn.  Replaced by direct
                                   link-pad-to-X-horn bolting via 4 x M2 SHCS
                                   self-tap into the X-horn's plastic arms.

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
CHASSIS_GAP           =  32.0   # mm -- vertical gap between top + bottom plates
                                #     (room for battery, brain, wiring).
                                #     Was 20 mm before the May 2026 audit:
                                #     the BATTERY_H = 28 mm holder was
                                #     visibly ramming through the 4 mm
                                #     chassis_top deck (no clearance
                                #     cutout existed).  32 mm leaves
                                #     32 - 28 = 4 mm of headroom above
                                #     the battery inside the plate-to-
                                #     plate void; the brass standoffs in
                                #     SHOPPING_LIST.md / PROTOTYPE_BOM.md
                                #     bump to 32 mm to match.  IMPORTANT:
                                #     any future change to CHASSIS_GAP
                                #     MUST re-check the battery_holder
                                #     and electronics_tray clearance + the
                                #     standoff length (the
                                #     CAD_AGENT_INSTRUCTIONS.md rule
                                #     #9 enforces this).
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
#
#   Design B (May 2026): the printed servo_horn_adapter has been
#   retired, dropping HORN_STACK_H from 9 mm to 5 mm and therefore
#   dropping the world Z of the coxa_link's frame (and the hip-pitch
#   joint axis) by HORN_ADAPTER_T = 4 mm.  Without compensation, the
#   bracket's flange top in coxa-link Z moves from -10.75 mm to
#   -6.75 mm and the femur's swept pad (lowest coxa-link z = -9.5 mm)
#   now collides with it by ~ 2.75 mm.  COXA_LIFT is therefore bumped
#   by the same 4 mm (32 -> 36) so the hip-pitch joint axis stays at
#   the same WORLD Z as before, the kinematic chain is unchanged for
#   RL / gait / MuJoCo, and the documented +1.25 mm bracket and
#   +1.5 mm arm clearances are restored.
COXA_LIFT     = 36.0     # mm  (was 32.0 before Design B)

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

# ---- Servo mounting tab holes (for cradle pilot-hole bolts) --------------
# On a DS3225 / MG996R / DS3218 class hobby servo the 4 M3 mounting holes
# live in the ~2.5 mm-thick TABS that extend along the body's long axis
# (+/- X in servo-local frame).  Tab holes are spaced SERVO_TAB_HOLE_PCD
# along X (= 49.5 mm) and SERVO_TAB_HOLE_PCD_Y along Y (= 10 mm).  The
# tabs sit SERVO_TAB_Z above the body bottom, so the mounting bolt's
# axis crosses the servo at z = SERVO_TAB_Z in body-local coords.
#
# Design D (May 2026 heat-set switch): each servo is bolted into its
# cradle by 4 VERTICAL M3 x 8 SHCS that thread DOWN through each servo
# ear into an M3 brass heat-set insert (McMaster 94459A130) seated in
# a Phi INSERT_M3_PILOT_OD = 4.0 mm pocket in the cradle shelf below.
# Each pocket is surrounded by a Phi CRADLE_BOSS_OD = 8.0 mm printed
# boss (see ``_servo_cradle_insert_pockets`` and the INSERT_M3_* /
# CRADLE_BOSS_* constant block below).  This replaces the prior
# "M3 self-tap into Phi 2.5 mm plastic pilot" scheme, which the user's
# audit found had 0.00-1.50 mm of radial material on 7 of 12 sites --
# the cradle outer walls did not extend far enough in Y beyond the
# servo footprint, and on the +X column the wire channel cut into the
# wall right where the bolt lived.  Bolt geometry in well-local coords:
#
#     bolt enters at  (sx * SERVO_MOUNT_HOLE_X_OFFSET,    -- ear position
#                      sy * SERVO_MOUNT_HOLE_Y_OFFSET,    -- ear position
#                      SERVO_MOUNT_HOLE_Z_OFFSET + SERVO_TAB_T/2) -- ear top
#     bolt axis  = -Z (downward into the heat-set insert)
#     bolt clears the Phi SERVO_MOUNT_BOLT_OD = 3.2 mm ear hole in the
#         servo's tab (factory-drilled, NOT printed) and engages the
#         M3 thread inside the heat-set brass insert seated in the
#         printed Phi 4.0 mm pocket below.
#
# Named constants below mirror the SERVO_TAB_HOLE_* values so the cradle
# code can refer to a clear "mount-hole position" instead of arithmetic
# on tab-pitch constants.
SERVO_MOUNT_HOLE_X_OFFSET = SERVO_TAB_HOLE_PCD / 2.0     # 24.75 mm from
                                                          # body centre
                                                          # along the
                                                          # body's long
                                                          # axis (= ear
                                                          # tip in +/-X).
SERVO_MOUNT_HOLE_Y_OFFSET = SERVO_TAB_HOLE_PCD_Y / 2.0    # 5 mm from
                                                          # body centre
                                                          # along the
                                                          # body's short
                                                          # axis.
SERVO_MOUNT_HOLE_Z_OFFSET = SERVO_TAB_Z                   # 27 mm above
                                                          # the body's
                                                          # bottom face
                                                          # (matches the
                                                          # tab's z
                                                          # height).
SERVO_MOUNT_BOLT_OD       = 3.2    # mm -- M3 clearance through the servo
                                    # ear (factory-drilled, NOT printed).

# ---- Heat-set brass insert for cradle servo mounts (May 2026) -----------
# McMaster 94459A130: M3 brass heat-set insert, knurled.  Pilot Phi 4.0 mm,
# insert OD 5.7 mm (knurled max), insert length 5.0 mm, recommended pilot
# depth 6.0 mm.  Installed with a soldering iron at ~220 deg C, light
# pressure, ~10-15 s per insert.  After install the M3 x 8 SHCS threads
# into the insert from above; the bolt clamps the servo ear down onto
# the boss top surface.
#
# This replaces the earlier "M3 self-tap into Phi 2.5 mm plastic pilot"
# scheme.  The user's audit (May 2026) found that 7 of 12 cradle bolt
# sites had 0.00-1.50 mm of plastic on one side of the Phi 2.5 mm pilot
# in the Y direction -- the cradle outer walls did not extend in Y
# beyond the body footprint, and on the +X column the wire channel ate
# into the wall right where the bolt lived.  The verifier's vertical
# self-tap check (commit b447f88) only confirmed that the pilot
# CYLINDER existed; it never confirmed that the pilot was SURROUNDED
# by enough material radially, so the structural bug shipped.
#
# Switching to a heat-set insert (a) gives real metal threads (replacing
# the borderline thread engagement that the Phi 2.5 mm plastic pilot
# provided in PLA / PA12), (b) requires a Phi 4 mm pilot which is
# physically wider than the previous Phi 2.5 mm pilot's column anyway,
# and (c) gives us license to add a fat Phi ~8 mm boss around each
# pilot so the radial-engagement bug automatically vanishes.  The
# verifier was also extended with an 8-azimuth radial-material probe
# (see ``check_cradle_insert_pockets`` in ``_verify_prototype.py``) so
# the previously missing structural check is now part of the build.
INSERT_M3_PILOT_OD            =  4.0   # mm -- printed pilot Phi (matches
                                        # McMaster 94459A130's
                                        # recommended pilot diameter).
INSERT_M3_PILOT_DEPTH         =  6.0   # mm -- printed pilot depth (insert
                                        # length 5 mm + 1 mm overdrill so
                                        # debris from heat-set insertion
                                        # has somewhere to go).
INSERT_M3_INSERT_OD           =  5.7   # mm -- knurled OD of the installed
                                        # brass body.  The knurl
                                        # displaces / melts the Phi 4 mm
                                        # pilot wall during installation
                                        # to form a press-fit -- standard
                                        # heat-set behaviour.
INSERT_M3_INSERT_LENGTH       =  5.0   # mm -- physical insert length.
INSERT_M3_BOLT_LENGTH         =  8.0   # mm -- M3 x 8 SHCS (unchanged
                                        # stock); threads into the
                                        # insert from above.
INSERT_M3_BOLT_HEAD_OD        =  5.5   # mm -- M3 SHCS head Phi.
INSERT_M3_BOLT_HEAD_H         =  3.0   # mm -- M3 SHCS head height.

# Boss surrounding each insert pilot.  Pilot Phi 4.0 mm + 2 x 1.5 mm wall
# = Phi 7.0 mm minimum.  Round up to Phi 8.0 mm to leave margin for the
# wire-channel reroute on the +X side AND to absorb FDM line-width
# variation around the curved boundary.
#
# Geometry sketch (well-local, one boss at sx*SERVO_MOUNT_HOLE_X_OFFSET,
# sy*SERVO_MOUNT_HOLE_Y_OFFSET):
#
#       boss top at z = SERVO_TAB_Z = 27 mm (servo ear bottom seats on
#                                            this face)
#       +----+--+   ear (factory part of the servo)
#       |    |  |   bolt (M3 x 8 SHCS, head on top)
#       |    |##|<--- heat-set insert (Phi 5.7 mm x 5 mm)
#       |    |##|   pocket (Phi 4 mm x 6 mm, printed)
#       |    |  |
#       +----+--+
#       boss bottom at z = SERVO_TAB_Z - CRADLE_BOSS_HEIGHT_MM = 17 mm
#       boss OD = CRADLE_BOSS_OD = 8 mm
#
# The boss fuses into the existing well outer rim and / or shelf
# material wherever they intersect, so the boss "grows out" of the
# nearest cradle surface rather than dangling in free air.
CRADLE_BOSS_OD                =  8.0   # mm -- boss outer diameter.
CRADLE_BOSS_MIN_WALL_MM       =  1.5   # mm of plastic required radially
                                        # around the pilot (read by the
                                        # verifier's radial-material
                                        # probe in
                                        # ``check_cradle_insert_pockets``).
CRADLE_BOSS_HEIGHT_MM         = 10.0   # mm; boss extends from
                                        # SERVO_TAB_Z DOWN by this much.
                                        # Boss top face = SERVO_TAB_Z so
                                        # the ear bottom rests on top.

# ---- +X cradle bolts: SELF-TAP fallback (mixed-mode Design E, May 2026) --
# The 2 +X cradle bolts at (sx, sy) = (+1, +/-1) CANNOT use the heat-set
# scheme above: the M3 brass insert wants a Phi 8 mm boss centred at
# (+24.75, +/-5), spanning well-z [17, 27], and that boss footprint
# intersects the inside-+X-wall WIRE CHANNEL (y in +/-3.5) that the
# servo's molded wire-exit boot has to pass through DURING INSERTION.
# The boot is WIRE_BOOT_W = 7 mm wide in Y so the channel CANNOT be
# narrowed in Y to clear the boss; the M3 bolt CANNOT be moved in Y
# because it has to drop through the DS3225's factory-drilled tab hole
# at the fixed servo SERVO_TAB_HOLE_PCD_Y = +/-5 offset; and the bolt
# CANNOT be moved in X for the same factory-tab-hole reason.  So the
# heat-set scheme is physically incompatible with the wire-exit
# channel on the +X column.
#
# The user explicitly accepted reverting these 2 bolts to the original
# Phi 2.5 mm M3 SELF-TAP pilot scheme to unblock servo seating
# ("Maybe we need to go back to just drilling the screws into the
# mounts on the servos if the heat set insert wont fit with the
# channels added").  This trades pull-out durability on these 2
# specific bolts for being able to assemble the robot at all; the
# 2 -X bolts per cradle still use the heat-set insert (no channel
# conflict there), so half of the structural improvement from
# f03d59b survives.  See ``check_cradle_insert_pockets`` in
# ``_verify_prototype.py`` for the mixed-mode probe -- only the -X
# sites are checked for 1.5 mm radial wall material; the +X sites
# are documented as known-thin (0.25 mm on the channel side) and
# the radial check is skipped there.  See also
# ``check_servo_insertion_path`` which probes the boot's swept
# volume through the +X wall during insertion so this exact
# regression cannot ship again.
#
# Pilot geometry mirrors the original pre-f03d59b SHCS_PILOT_OD
# value (= 2.5 mm) and runs from the shelf top down by
# INSERT_M3_SELFTAP_PILOT_DEPTH so the M3 x 8 SHCS engages 5+ mm
# of plastic below the 2.5 mm-thick servo tab.  No boss is added
# around the self-tap pilot -- the existing well wall material
# carries the surrounding plastic, and adding a boss would only
# enlarge the column's footprint without solving the inboard
# (channel-side) wall problem.
INSERT_M3_SELFTAP_PILOT_OD    =  2.5   # mm -- Phi 2.5 mm self-tap pilot
                                        # for the 2 +X cradle bolts on
                                        # every cradle (mixed-mode).  Same
                                        # value as the legacy
                                        # ``SERVO_PILOT_OD`` constant -- the
                                        # heat-set switch never removed
                                        # the M3 self-tap convention, just
                                        # stopped using it for the cradle
                                        # bolts; the wire-channel collision
                                        # forces a partial revert.
INSERT_M3_SELFTAP_PILOT_DEPTH = 10.0   # mm -- pilot depth (matches
                                        # CRADLE_BOSS_HEIGHT_MM).  M3 x 8
                                        # SHCS engages 5.5 mm of plastic
                                        # below the 2.5 mm servo tab; the
                                        # extra 4.5 mm of pilot depth is
                                        # head-room for overdrive without
                                        # bottoming the screw.

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
# Depth math with WIRE_CHANNEL_DEPTH = 6.5 (Design E, May 2026):
#   cavity face at +SERVO_BODY_W/2 + WELL_BODY_CL = +20.7;
#   channel outer x = +20.7 + 6.5 = +27.2;
#   boot outboard face at +SERVO_BODY_W/2 + WIRE_BOOT_PROTRUSION =
#     +26.5, so the channel clears the boot's outer face by 0.7 mm --
#     the channel is now genuinely wide enough for the boot to slide
#     all the way down to seated during insertion, instead of forcing
#     the boot to compress 0.8 mm against the +X wall (the pre-Design-E
#     5.0 mm depth left a 0.8 mm interference at the boot's outer edge
#     for all insertion offsets above 9.4 mm; ``check_servo_insertion_
#     path`` flags this regression);
#   pilot at x = +SERVO_TAB_HOLE_PCD/2 = +24.75, pilot radius 1.25 mm so
#     pilot outboard X edge sits at +26.0;
#   channel (y in +/-3.5) and pilots (at y = +/-5, pilot y_min 3.75)
#     are fully separated in Y -- the two never touch in 3D regardless
#     of the channel's X depth, so deepening the channel does not eat
#     pilot material.
#   Wall thickness over the pilot threads (face-to-pilot distance):
#     +WELL_W/2 = +29 to pilot at +24.75 = 4.25 mm -- still >= 4 mm.
#   Remaining +X wall material outboard of the channel (face-to-channel):
#     +WELL_W/2 - channel_outer_x = +29 - 27.2 = 1.8 mm, just above the
#     three-perimeter MIN_PRINT_T = 1.6 mm floor.
WIRE_CHANNEL_DEPTH    = 6.5   # mm groove depth INTO the +X wall material
WIRE_CHANNEL_TOP_OVER_RIM = 2.5  # mm extension of the channel ABOVE the well
                                  # rim, so wires exiting near the top of the
                                  # back-case (micro servos) still find it.

# ---- Cable zip-tie post (printed-in strain relief) ----------------------
# Every servo cradle (yaw / hip-pitch / knee) gets a small printed post on
# the well's +X OUTER wall, just past where the wire-exit slot opens to
# free air.  At assembly time the user loops a 2-3 mm wide zip-tie around
# the post AND the 3-wire bundle, so any tug on the harness pulls the
# bundle against the post (a printed-in strain relief) instead of yanking
# the JST plug at the PCA9685 header.  Without these posts a single
# accidental snag on a leg cable can pop a servo plug off the PCA at the
# worst possible moment (mid-walk gait); with them the harness fails
# safely (the zip-tie bears the load).
#
# Geometry summary (all three cradles use the SAME ``_cable_zip_post()``
# helper applied in well-local frame and then transformed alongside the
# wire_slot, so the post stays anchored to the well wall regardless of
# how each cradle is rotated into its link's frame):
#
#   * Anchored at the well's OUTER +X face (well_x = WELL_W/2 = +29).
#     Inboard face of the post sits AT the wall (= +29); outboard face
#     at +29 + CABLE_POST_X = +31.
#   * Offset in well-Y by CABLE_POST_Y_OFFSET = +7 mm so the post lives
#     OUTSIDE the wire-exit slot's Y span (slot Y in
#     [-WIRE_SLOT_W/2, +WIRE_SLOT_W/2] = [-3.5, +3.5]).  Without the
#     offset the post would sit IN the slot exit and physically block
#     the harness from leaving the cradle.  +7 mm Y centre leaves
#     +7 - CABLE_POST_Y/2 = +4 mm of well-Y clearance from the slot
#     edge at +3.5.
#   * Z centre at CABLE_POST_Z_CENTRE = +6 mm in well-local Z (i.e.,
#     +6 mm above the cavity floor).  Range +2..+10 mm sits comfortably
#     INSIDE the well's outer +X wall material (the wall is solid from
#     well-z = -WELL_FLOOR_T = -3 up to well-z = WELL_RIM_Z = +27.25 at
#     this Y offset since the channel cut only spans y in [-3.5, +3.5]).
#   * 2 x 6 x 8 mm extents (X / Y / Z) -- small enough to print
#     supports-free as a horizontal nub off the vertical well wall in
#     every cradle's print orientation (coxa_bracket prints cradle-up,
#     coxa_link prints with hip cradle opening +Z after the +90 deg X
#     rotation, femur_link prints spar-broad-face-on-bed; in all three
#     the post is a 2 mm-deep ledge along a vertical wall, with the
#     2 mm bottom face printing as a tiny horizontal bridge that FDM
#     handles natively).
#
# Keep-out audit (verified by check_workspace_self_collision +
# check_cable_clearance + check_wire_slot):
#
#   * Yaw cradle (coxa_bracket): post sits BELOW the bracket flange
#     (well-z in [+2, +10] is below WELL_RIM_Z = +27.25 in well frame;
#     in bracket frame the post is at bracket-z in [-25.25, -17.25],
#     well clear of the chassis_bottom plate at bracket-z in [-4, 0]).
#   * Hip-pitch cradle (coxa_link): after R + delta + well_z_drop +
#     COXA_LIFT lift the post sits at link (x ~ +45, y ~ -42, z ~ +5)
#     -- > 21 mm from the hip pad sweep cylinder centre at
#     (COXA_LENGTH, 0, COXA_HIP_DROP), so well clear of HIP_PAD_R = 20
#     mm sweep.
#   * Knee cradle (femur_link): post sits at femur (x ~ +110, y ~ -42,
#     z ~ +5), well outside the tibia knee-pad neck sweep at
#     (FEMUR_LENGTH = +90, *, 0) with HIP_PAD_R = 20 mm clearance.
#
# Cradle bolt boss columns at well-y = +/- SERVO_TAB_HOLE_PCD_Y/2 = +/-5,
# well-x = +/- SERVO_TAB_HOLE_PCD/2 = +/-24.75, well-z in
# [SERVO_TAB_Z - CRADLE_BOSS_HEIGHT_MM, SERVO_TAB_Z] = [+17, +27].  Our
# post at (well_x ~ +30, well_y ~ +7, well_z in [+2, +10]) is FAR from
# the +X bosses' XY (+24.75, +/-5): distance ~ sqrt(5.25^2 + 2^2) =
# 5.6 mm in XY from the (+24.75, +5) boss centre, well outside the
# Phi CRADLE_BOSS_OD/2 = 4 mm boss radius.  Z ranges don't even overlap.
CABLE_POST_X            = 2.0   # mm protrusion in well +X (outward from
                                 # the well's outer face)
CABLE_POST_Y            = 6.0   # mm width along well Y (slot transverse)
CABLE_POST_Z            = 8.0   # mm height along well Z (slot depth axis)
CABLE_POST_Y_OFFSET     = 7.0   # mm centre offset in well +Y from slot
                                 # centreline (slot at y = 0); see audit
                                 # above for clearance to slot + bosses.
CABLE_POST_Z_CENTRE     = 6.0   # mm well-local centre Z; range
                                 # [Z_CENTRE - CABLE_POST_Z/2,
                                 #  Z_CENTRE + CABLE_POST_Z/2] = [+2, +10]
                                 # sits inside solid wall material.

# ---- Per-leg chassis_bottom drop slot (Part B, May 2026) ----------------
# A small 12 x 6 mm slot cut THROUGH the chassis_bottom plate at each of
# the 6 leg positions, located just INBOARD of each coxa_bracket's body
# cutout (which the plate already carries for the servo body's drop-in
# path; see ``_hex_plate``'s body_cutout block).  The slot lets each
# leg's 3-cable harness drop from the bracket-flange-top side of the
# plate INTO the inter-plate volume between chassis_bottom and
# chassis_top -- where the electronics_tray + PCA9685 boards live --
# without the harness having to share the body cutout with the seated
# yaw servo (the body fills ~ 40 x 20 of the cutout's 60 x 31 footprint,
# leaving only ~ 5 mm of margin around the body; cramming 3 wires into
# the margin works but adds risk of pinching the harness against the
# body during assembly).
#
# Slot orientation: LONG axis is along the chassis-radial (bracket +X)
# direction; SHORT axis is tangential (bracket Y).  In each leg's
# bracket frame the slot is centred at:
#     bracket-x = LEG_HARNESS_DROP_X_CENTRE = -46
#                 (10 mm inboard of the body-cutout's -X edge at -40)
#     bracket-y = 0  (on the chassis radial line)
# with extents (LEG_HARNESS_DROP_X_EXTENT, LEG_HARNESS_DROP_Y_EXTENT) =
# (12, 6) mm.  The same hex-leg iterator that ``_hex_plate`` uses to
# place the body cutouts places these slots, so the 6 drop slots
# always line up with their parent cutouts (a regression here is
# caught by the new ``check_leg_harness_drop`` sibling in
# ``_verify_prototype.py``).
LEG_HARNESS_DROP_X_CENTRE  = -46.0  # mm bracket-x (inboard of body cutout)
LEG_HARNESS_DROP_X_EXTENT  =  12.0  # mm along bracket X (radial)
LEG_HARNESS_DROP_Y_EXTENT  =   6.0  # mm along bracket Y (tangential)

# ---- Chassis_bottom cable anchor tab (Part A continued) -----------------
# Each leg also gets a SMALL printed tab hanging DOWN from chassis_bottom's
# BOTTOM face, immediately adjacent to its ``leg_harness_drop_*`` slot, so
# the assembler can loop a 2-3 mm zip-tie around the tab AND the 3-cable
# harness BELOW the chassis plate -- exactly where the cradle wire-exit
# bundle makes its U-turn from the cradle's lateral wire-exit (at well +X)
# back inboard to the drop slot.  Anchoring the bundle here rather than
# on chassis_bottom's TOP face dodges the electronics_tray (chassis-z in
# [+5, +8] over chassis-x in [-80, +80] x y in [-70, +65], which fully
# blankets the inter-plate volume directly above each drop slot), and it
# matches the harness's natural routing: the cradle wire-exit fires at
# bracket-z ~ -21 (= chassis-z ~ -19, well below the plate), the wire
# hugs the underside of the plate inboard to bracket-x ~ -46, then comes
# UP through the drop slot.  The tab grabs the bundle just BEFORE the
# slot, so any cable tug puts load on the tab + plate instead of the
# servo's JST plug or the cradle wire-exit boot.
#
# Geometry:
#   * Anchored at chassis_bottom's BOTTOM face (chassis-z =
#     -CHASSIS_PLATE_T/2 = -2 mm); extends DOWN by CABLE_ANCHOR_TAB_H so
#     the bottom face sits at chassis-z = -CHASSIS_PLATE_T/2 -
#     CABLE_ANCHOR_TAB_H = -12 mm.  In bracket-local frame the same range
#     is bracket-z in [-12, -2].
#   * Placed at bracket-x = LEG_HARNESS_DROP_X_CENTRE (= -46 mm), so the
#     tab's radial position is centred on the drop slot itself.
#   * Offset to the +Y side of the drop slot by CABLE_ANCHOR_TAB_Y_OFFSET
#     = +5 mm (slot Y in [-3, +3]; tab centre at +5 leaves a 1 mm gap
#     between the slot's +Y edge at +3 and the tab's -Y face at
#     +5 - CABLE_ANCHOR_TAB_W/2 = +1).  The harness exits the slot in
#     the slot's Y span, so the tab's -Y face is the natural surface
#     for the zip-tie to bear against.
#
# Keep-out audit (verified by check_workspace_self_collision +
# check_cable_clearance):
#   * Bracket-x = -46 is 7 mm INBOARD of the yaw well's inboard wall
#     (well outer face at bracket-x = body_centre_x - WELL_W/2 = -39).
#     Below the plate the well + seated yaw servo body occupy
#     bracket-x in [-39, +19]; tab at bracket-x = -46 sits in OPEN
#     air well clear of that volume.
#   * Bracket-x = -46 is 16 mm INBOARD of the bracket flange's inboard
#     edge at bracket-x = -30; the flange is ABOVE the plate (bracket-z
#     in [+2, +17]) so there's no overlap in either X or Z.
#   * Bracket-y = +5 sits outside the drop slot's Y span [-3, +3] and
#     well inside the chassis_bottom plate footprint.  Below the plate
#     in the inter-plate region just inboard of the bracket nothing
#     else exists -- the chassis_bottom plate's underside between the
#     legs is otherwise empty.
#   * The dynamic leg workspace (coxa_link, femur_link, tibia_link)
#     sweeps OUTBOARD from the yaw axis; the inboard region around
#     bracket(-46, *) is not reachable by any leg part.
#     check_workspace_self_collision catches any regression.
CABLE_ANCHOR_TAB_T   = 2.0    # mm thickness (along bracket X = chassis
                               # radial); thin enough to print supports-
                               # free as a horizontal nub off the plate
                               # underside.
CABLE_ANCHOR_TAB_W   = 8.0    # mm width (along bracket Y = chassis
                               # tangential); enough to anchor a 2-3 mm
                               # zip-tie loop with a comfortable bearing
                               # surface on the -Y face.
CABLE_ANCHOR_TAB_H   = 10.0   # mm height (chassis -Z direction, below
                               # the plate bottom face); enough vertical
                               # purchase to anchor a zip-tie without
                               # snagging on the lower edge.
CABLE_ANCHOR_TAB_X   = LEG_HARNESS_DROP_X_CENTRE  # = -46 mm bracket-x;
                               # tab centred on the drop slot (so the
                               # bundle exits the slot ABOVE the tab
                               # and the zip-tie loop grabs it at the
                               # plate level).
CABLE_ANCHOR_TAB_Y_OFFSET = 5.0  # mm bracket +Y offset from drop slot
                                  # centreline (slot Y in [-3, +3]; tab
                                  # centred at +5 with width 8 spans
                                  # bracket-y in [+1, +9], with a 1 mm
                                  # gap to the slot's +Y edge so the
                                  # cradle harness can sit against the
                                  # tab's -Y face without binding in
                                  # the slot itself).

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
WELL_D       = SERVO_BODY_D + 2 * WELL_WALL_Y     # 29 mm (was 25 mm
                                                  # when WELL_WALL_Y = 2.5;
                                                  # the chassis_bottom body
                                                  # cutout below is WELL_D + 2
                                                  # = 31 mm and grew with it,
                                                  # so older chassis_bottom
                                                  # prints made before the
                                                  # wall bump have a 27 mm
                                                  # cutout that the current
                                                  # bracket no longer fits
                                                  # through -- reprint the
                                                  # bottom plate.)
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

# ---- Bracket yaw-well TRIM plane ----------------------------------------
# May 2026 user-driven simplification:  the bracket's yaw-servo well used
# to extend ``WELL_RIM_Z + WELL_FLOOR_T = 30.25 mm`` below the chassis-
# plate top face -- a 38 mm DS3225 body fully enclosed by ~27 mm of side
# walls plus a 3 mm perimeter floor.  The user pointed out that the
# walls only need to extend deep enough to host the bracket-level
# heat-set insert bosses at bracket-z in [-13, -3] (= 10 mm boss height
# starting 3 mm below the effective shelf top); below that, the wall
# material is just lateral confinement for a servo body that the
# four tabs already hold up structurally.
#
# ``BRACKET_WELL_TRIM_Z`` is the bracket-z plane below which all bracket
# material is removed by the final boolean subtraction in
# ``make_coxa_bracket``.  Set to -15 mm:
#
#   * 2 mm of margin BELOW the bracket-level insert bosses (boss bottom
#     at bracket-z = -13).
#   * 5 mm of margin below the WELL-level inboard-bolt insert bosses
#     emitted by ``_servo_well_solid`` (bracket-z in [-10, 0]).
#   * The DS3225 wire-exit boot, centred at bracket-z ~ -19, hangs
#     EXPOSED below the trim plane.  The +X insertion channel
#     (boot-drop path) above the trim is preserved -- the boot still
#     slides freely from the rim down to its seated position.
#   * The bracket's lowest point moves from bracket-z = -30.25 to
#     bracket-z = -15 (a 15.25 mm reduction in vertical extent).
#
# The trim is applied to the BRACKET ONLY (``make_coxa_bracket``); the
# coxa_link (hip-pitch cradle) and femur_link (knee cradle) keep their
# existing full-depth walls because their wells participate in those
# links' structural geometry differently.
BRACKET_WELL_TRIM_Z = -15.0

# ---- Servo horn adapter (DEPRECATED -- retained for backward compat) -----
# Design B (May 2026): the printed servo_horn_adapter has been RETIRED.
# Each driven link (coxa_link, femur_link, tibia_link) now bolts DIRECTLY
# onto the plastic 4-arm X-horn that ships with the servo: 4 x M2
# clearance holes are carved straight into the link's mating pad on the
# XHORN_BOLT_PCD = 20.8 mm pattern, and a HORN_RECESS_OD x HORN_RECESS_DEPTH
# counter-bore swallows the horn's central hub.
#
# May 2026 fastener-spec fix: the X-horn bolts were originally drawn as
# M3 SHCS (Phi 3.2 mm clearance through the pad), which is WRONG -- the
# X-horn that ships with DS3225 / MG996R / DS3218-class servos has
# Phi ~ 2.0 mm untapped through-holes in its arms (M2 self-tap-sized),
# not Phi 3.2 mm clearance.  M3 literally won't fit.  The link pad
# clearance bore is now Phi XHORN_BOLT_OD = 2.2 mm and the fastener is
# M2 x 8 SHCS used as a self-tapper into the X-horn plastic.  The
# cradle bolts (4 per servo into a Phi 2.5 mm printed shelf pilot) are
# orthogonal to this fix and stay M3 x 8 (b447f88).  The adapter geometry
# below is left in place so old STL references / Xometry quotes / spare-
# stock screenshots keep resolving, but ``make_servo_horn_adapter`` is
# no longer called from any printable output path and no
# ``servo_horn_adapter.stl`` is written by ``main()``.
HORN_ADAPTER_OD     = 32.0   # mm -- plate OD; gives (32 - 20.8) / 2 - 1.1
                              # = 4.5 mm wall outboard of each M2 bolt
                              # hole on XHORN_BOLT_PCD = 20.8 mm
HORN_ADAPTER_T      =  4.0   # mm -- thickness (LEGACY).  No longer added
                              # to any joint output Z stack.
# Hobby servo plastic horn (the part that ships with the servo and screws
# onto the 25T spline with an M3 centre screw).  Adds 5 mm of "stack"
# along the output-shaft direction between the servo's gear-stack top
# face and the link's pad MATING face.  With the adapter retired, this
# IS the entire horn stack (it used to be PLASTIC_HORN_H + HORN_ADAPTER_T
# = 9 mm).
PLASTIC_HORN_H      =  5.0
# Plastic 4-arm X-shaped horn (DS3225 / MG996R / DS3218 -class hardware):
# the arms extend ~18 mm from the spline centre, so the horn sweeps a
# Phi 36 mm cylinder as the servo rotates.  This is BIGGER than the
# XHORN_BOLT_PCD = 20.8 mm bolt circle (radius 10.4 mm): each arm runs
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
#
# User-measured May 2026: 36 mm horn diameter on the actual hardware
# (user has the physical plastic X-horns in hand and measured them
# with calipers).  Was 19.0 mm tip radius / Phi 38 mm sweep; now
# 18.0 mm / Phi 36 mm.
PLASTIC_HORN_X_TIP_R = 18.0
# Total height of the servo output stack ABOVE the spline tip.  In the
# hip-pitch and knee-pitch joints this is the offset between the joint
# AXIS (where the spline pokes out of the servo body) and the link's
# pad MATING face (where the link bolts directly onto the plastic X-horn).
# Driven links must offset their pad along the joint axis by this much,
# then bridge the gap back to the spar with a short "neck" -- without
# this offset the link's pad sits directly on the joint axis and
# overlaps the cradle's "swept volume" (the bridge cap / well wall
# material right above the body).
#
# May 2026: HORN_STACK_H collapses to PLASTIC_HORN_H = 5 mm (was
# PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm) now that the printed
# horn-adapter disc has been removed.  The link's pad bolts directly
# to the plastic X-horn's top face.
HORN_STACK_H        = PLASTIC_HORN_H
# Bolt circle for the plastic horn's 4-arm X-shaped output disc.  20.8 mm
# is the PCD of the SECOND hole position out from the spline on each arm
# of a standard DS3225 / MG996R / DS3218 plastic horn.  The bolt angles
# match the horn arms (0 / 90 / 180 / 270 deg in horn-local coords) so a
# bolt drilled through XHORN_BOLT_ANGLES_RAD[i] on the link's pad
# passes straight through the matching hole in the X arm.
#
# May 2026 fastener-spec fix (XHORN_BOLT_OD: 3.2 -> 2.2 mm):
#     The plastic X-horn that ships with DS3225 / MG996R / DS3218-class
#     hobby servos has Phi ~ 2.0 mm UNTAPPED through-holes in its arms
#     (intended for M2 self-tap), NOT Phi 3.2 mm clearance for M3.  An
#     M3 SHCS literally won't fit through those holes.  The link pad's
#     clearance bore therefore shrinks to Phi 2.2 mm (M2 clearance with
#     0.2 mm FDM print tolerance) and the link bolts onto the X-horn
#     with 4 x M2 SHCS used as self-tappers into the X-horn's plastic
#     arm.  Mechanically symmetric to the cradle bolt fix that landed
#     in b447f88 -- the cradle bolts stay M3 x 8 (they self-tap into a
#     Phi 2.5 mm printed shelf pilot); the X-horn bolts split off into
#     their own row.  See fastener_registry.py (SPEC_M2X8_SHCS) and
#     fasteners/README.md for the McMaster SKU.
XHORN_BOLT_PCD       = 20.8   # mm -- 4 x M2 holes on this PCD
XHORN_BOLT_ANGLES_RAD = (0.0, np.pi / 2.0, np.pi, 3.0 * np.pi / 2.0)
XHORN_BOLT_OD        =  2.2   # mm -- M2 clearance (0.2 mm FDM tolerance)
                              # for the M2 SHCS shank passing through the
                              # link pad into the X-horn's Phi ~ 2.0 mm
                              # self-tap arm hole below.
HORN_CENTRE_OD      =  3.4   # mm -- M3 centre clearance (for the horn screw)
# Counter-bore for the plastic horn body that sits below the link's
# pad mating face.  Sized so the recess wall ends INSIDE the bolt
# circle (recess radius < XHORN_BOLT_PCD/2 - XHORN_BOLT_OD/2 - margin =
# 10.4 - 1.1 - 0.4 = 8.9 mm), so the recess never punches into the
# X-horn's bolt threads.  Big enough to swallow a typical hobby-horn
# central disc (Phi 12 mm OD on DS3225-class hardware).  Unchanged by
# the May 2026 M3 -> M2 fix: the central hub geometry is identical
# regardless of arm-hole spec.
#
# Depth (HORN_RECESS_DEPTH): user-measured May 2026 -- the spline screw
# head sticks out 1 mm above the X-horn arm plane on the actual
# hardware.  0.2 mm added for FDM print tolerance so the pad still
# seats flush against the arm even at worst-case Z layer height
# (1.0 mm protrusion + 0.2 mm margin = 1.2 mm recess depth).  Was
# 1.6 mm prior to the measurement.
HORN_RECESS_OD      = 16.0   # mm
HORN_RECESS_DEPTH   =  1.2   # mm

# Radius of the cylindrical CLEARANCE VOID inside the hip/knee link's
# flange ring.  The void is the air gap that holds the plastic X-horn
# during assembly: the link slides down over the horn, and the horn's
# arm tips (PLASTIC_HORN_X_TIP_R = 18 mm) must clear the ring's inner
# wall by a small FDM-tolerance margin.
#
# May 2026 (post-shorten-neck refactor): the void switched from
# HORN_ADAPTER_OD/2 + 0.5 = 16.5 mm (sized for the now-retired
# printed servo_horn_adapter disc) to PLASTIC_HORN_X_TIP_R + 0.5 =
# 18.5 mm (sized for the plastic X-horn's actual Phi 36 mm sweep).
# The user found that the previous Phi 33 mm cup physically blocked
# the Phi 36 mm plastic X-horn from fitting -- the horn arm tips
# slammed into the cup's inner wall.  Bumping to Phi 37 mm ID gives
# the horn 0.5 mm radial clearance per side at the arm tips.
#
# Consumers: ``make_femur_link`` + ``make_tibia_link`` (flange-ring
# inner radius), ``keepout_volumes.py`` (femur/tibia horn-stack
# void registry entries), and ``_verify_prototype.py``
# (``check_horn_stack_clearance`` probe radius).
HORN_STACK_VOID_R   = PLASTIC_HORN_X_TIP_R + 0.5

# ---- M2 self-tap into the X-horn (Design B, May 2026 fastener fix) -------
# The X-horn bolt is an M2 SHCS used as a thread-FORMING self-tapper
# into the plastic 4-arm X-horn's existing Phi ~ 2.0 mm untapped arm
# hole.  The link's pad just needs a clean clearance through-hole for
# the M2 shank; the X-horn provides the actual thread engagement.
# Bottom-cap geometry on driven link pads.  Each printed link pad has
# to:
#   (a) be solid in the central region where the 4 M2 X-horn clamp
#       bolts seat (PCD = 20.8 mm, well within the pad's 34 x 34 mm
#       footprint),
#   (b) recess each M2 SHCS head into a counter-bore so the head
#       doesn't protrude into the assembly trough above and block
#       the hip-pitch / knee servo body's insertion path.
# These two requirements drive the cap thickness PEDESTAL_CAP_T and
# the counter-bore depth COUNTERBORE_DEPTH below.  See the
# make_coxa_link / make_femur_link / make_tibia_link docstrings for
# the full geometry.
PEDESTAL_CAP_T          = 4.0   # mm -- thickness of the solid bottom
                                #       cap that carries the 4 M2
                                #       X-horn clamp bolts.  4 mm gives
                                #       a 1.5 mm shaft-clearance run
                                #       + 2.5 mm counter-bore.
COUNTERBORE_DEPTH       = 2.5   # mm -- counter-bore depth (M2 SHCS
                                #       head height = 2 mm + 0.5 mm
                                #       safety margin).  Bolt head TOP
                                #       sits flush with the cap top
                                #       face (= PEDESTAL_CAP_T).
M2_HEAD_OD_CLEARANCE    = 4.0   # mm -- Phi 4 mm clearance pocket for
                                #       the M2 SHCS head (head OD =
                                #       3.8 mm + 0.2 mm FDM tolerance).
XHORN_BOLT_M2_SELFTAP_HOLE_OD = 2.2   # mm -- M2 self-tap clearance
                                       #       through the link's pad
                                       #       (the plastic X-horn's
                                       #       existing Phi 2.0 mm hole
                                       #       is the actual self-tap
                                       #       engagement; the link just
                                       #       needs the bolt shank to
                                       #       pass through).  Numeric
                                       #       duplicate of XHORN_BOLT_OD
                                       #       above; kept under a more
                                       #       explicit name so callers
                                       #       can grep for either.
XHORN_BOLT_THREAD_ENGAGEMENT_MM = 3.0  # mm -- depth of M2 thread
                                       #       engagement INTO the
                                       #       plastic X-horn arm (arm
                                       #       thickness ~ 3 mm on a
                                       #       DS3225-class horn).
                                       #       Drives the M2 x 8 SHCS
                                       #       length pick in
                                       #       fastener_registry.py.

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
#   HIP_PAD_R >= HORN_STACK_VOID_R + MIN_PRINTABLE_NECK_WALL_T
#
# so the hollow-annulus neck (see make_femur_link / make_tibia_link, "flange
# ring" block) has a printable wall around the horn-stack clearance void.
# With HORN_STACK_VOID_R = PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm and a
# 1.5 mm wall the minimum is 18.5 + 1.5 = 20.0 mm.  A SHRUNK pad
# (HIP_PAD_R = 17, the very old value) leaves only 0.5 mm of wall,
# which cannot be FDM-printed reliably at 0.4 mm nozzle.
#
# May 2026 (post-flange-ring-fuse refactor): HIP_PAD_R bumped from
# 19.5 to 20.0 mm.  The previous 19.5 mm value left a 1.0 mm-thick
# neck wall (HIP_PAD_R - HORN_STACK_VOID_R = 1.0 mm), which prints
# as a single perimeter at 0.4 mm nozzle and is fragile.  More
# importantly, the May 2026 "shorten-neck" refactor (7e2ca87)
# collapsed the neck to a 2 mm flange ring at y in [+3, +5] with
# only a 0.5 mm "boolean kiss" volumetric fuse with the spar at
# y in [+2.5, +3].  The user complained twice that the tibia
# spar <-> knee pad joint looked like it would break in service
# (TIBIA_SPAR_H = 18 mm has only ~half the femur's pad-side fuse
# cross-section to work with), and a Y-sweep probe of the cross-
# section confirmed the fuse area dropped from a tall column to
# a 0.5 mm sliver.  The fix is Option A from the May 2026
# pad <-> spar fuse audit: bump HIP_PAD_R to 20.0 (wall thickness
# 1.5 mm, 3 perimeters @ 0.4 mm nozzle) AND deepen the neck back
# to ~8 mm (y in [-LINK_THICKNESS/2, HORN_STACK_H] = [-3, +5]) so
# the full volumetric fusion with the spar at y in [-3, +3] is
# restored.  Cup ID stays at PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm
# so the Phi 36 mm plastic X-horn still fits with 0.5 mm radial
# clearance.
#
# Bumping HIP_PAD_R increases the pad's swept disk diameter (39 mm
# -> 40 mm) which means the pad reaches 0.5 mm further toward the
# coxa_link arm's bottom face and the bracket flange.  COXA_LIFT
# (36 mm) and WELL_Z_DROP_EXTRA (4 mm) were sized with ~2 mm of
# headroom above the swept disk's +Z edge at HIP_PAD_R = 19.5, so
# the 0.5 mm bump shrinks the headroom to 1.5 mm -- still well
# above the WORKSPACE_VOXEL_PITCH = 2.5 mm artefact band.  See
# check_workspace_self_collision for the runtime verification.
HIP_PAD_R        = 20.0

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
# Compliant pad printed in TPU.
#
# May 2026 inversion: the tibia tip now ends in a single TANG (a
# LINK_THICKNESS-wide vertical tongue centred on tibia y=0) and the
# foot_pad has a 2-cheek FORK with a SLOT that captures the tang;
# pre-2026 it was the other way round (tibia = fork, foot = tongue).
# An M3x16 pan-head bolt with a nylock nut still clamps the two
# together and still serves as the single-axis hinge pin whose axis
# is parallel to the knee pitch axis -- the foot pitches passively
# around it to follow uneven ground.  TPU material compliance in
# the pad disk itself absorbs roll.  See the FOOT_HINGE_* block
# below for the dimensional rationale.
FOOT_PAD_OD          = 28.0   # mm -- outer diameter of the ground-contact disk
FOOT_PAD_BASE_H      =  4.0   # mm -- thickness of the disk (TPU spring)
FOOT_PAD_BOSS_OD     = 14.0   # mm -- short stiffening boss between disk top
                              #        and fork (gives the fork a wider
                              #        root than its 10 x 13.4 mm
                              #        cross-section; circular 14 mm OD
                              #        easily contains the rectangular
                              #        fork footprint)
FOOT_PAD_BOSS_H      =  3.0   # mm -- boss height; fork cheeks start at
                              #        FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H

# Hinge geometry.
#
# May 2026 inversion: the foot/tibia hinge used to be "TIBIA = fork,
# FOOT = tongue" -- the tibia ended in a 2-cheek + 4 mm tongue-slot
# clevis (12 mm wide in tibia Y) and the foot_pad had a 4 mm-thick
# vertical tongue that slid up into the slot.  That made the tibia
# 12 mm wide in Y at the foot end but only LINK_THICKNESS = 6 mm
# wide everywhere else, so the clevis protruded 6 mm above the
# spar's broad face in the "spar broad face on bed" print orientation
# and forced supports under the clevis.
#
# After the inversion the geometry is "TIBIA = single tang, FOOT =
# fork": the tibia spar terminates in a single LINK_THICKNESS-wide
# tang centred on tibia y=0 (so the tibia is now 6 mm wide in Y
# EVERYWHERE and prints fully flat), and the foot_pad gains a 2-cheek
# fork with a LINK_THICKNESS + clearance slot that captures the tang.
# Same M3 x 16 pan-head pin + M3 nylock, same hinge axis -- just
# inverted geometry; no BOM change for fasteners.  The foot_pad's
# Y extent across the fork is 2 * FOOT_HINGE_CHEEK_T + SLOT_W =
# 2 * 3.5 + 6.4 = 13.4 mm, which fits comfortably within the disk's
# 28 mm OD (the fork lives above the disk in foot-local +Z anyway).
FOOT_HINGE_CHEEK_T   =  3.5   # mm -- each foot-pad fork cheek thickness in
                              #        the knee-axis direction (Y).  Above
                              #        MIN_PRINT_T = 3.0 mm with margin.
                              #        Unchanged across the inversion --
                              #        the cheeks just live on the FOOT
                              #        now instead of the tibia.
FOOT_HINGE_SLOT_W    = LINK_THICKNESS + 0.4   # = 6.4 mm -- fork slot
                              #        clearance for the tibia's tang.
                              #        Tang nominal width = LINK_THICKNESS
                              #        = 6 mm, so 0.2 mm of clearance per
                              #        side.  Renamed from the pre-2026
                              #        ``FOOT_HINGE_GAP`` (5 mm of slop
                              #        around a separate 4 mm tongue);
                              #        with the tongue gone the slot
                              #        accepts the tang directly.
FOOT_HINGE_PIN_HOLE_D =  3.4  # mm -- through-hole diameter for the M3
                              #        hinge bolt; 0.2 mm clearance over a
                              #        nominal 3.2 mm M3 shank so the joint
                              #        rotates freely.
FOOT_HINGE_PIN_LEN    = 16.0  # mm -- pin length specification (M3 x 16
                              #        pan-head + M3 nylock nut).  Stack
                              #        = 3.5 + 6.4 + 3.5 = 13.4 mm of fork
                              #        + tang plus ~2.6 mm into the nylock
                              #        nut (still well within the nylock's
                              #        engagement range).

# Hinge axis Z position in tibia local: 10 mm below the tibia spar
# centreline (= 1 mm below the spar's bottom face at z = -9).  This
# pulls the tang down past the spar so the foot's fork mouth opens
# clearly onto free space below the spar and the foot pad has room
# to swing.
FOOT_HINGE_TIBIA_Z   = -10.0  # mm -- pin axis z in tibia-local

# Hinge axis Z position in foot-local: 14 mm above the disk bottom (=
# 14 mm above the ground when the foot stands).  Computed downstream
# in make_foot_pad() so the fork cheeks reach the pin with a few mm
# of material above the hole and the foot pad disk sits below the
# tibia's tang.
FOOT_HINGE_FOOT_Z    = FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H + 7.0   # = 14.0
FOOT_HINGE_FORK_OVER_PIN = 4.0    # mm of fork-cheek material above the pin
                                  #        axis (so the M3 hole has a
                                  #        continuous ring of plastic
                                  #        around it on the top side).

# Foot-pad fork X extent (along the foot's +X = spar direction).
# Sized so the M3 hole has > 3 mm of material on each side of the
# bore axis in X.  Matches the pre-inversion FOOT_HINGE_TONGUE_X
# value so the fork preserves the same rotational clearance the old
# tongue + slot pair had.
FOOT_HINGE_FORK_X    = 10.0       # mm

# Tibia tang bulk extent.  The tang is the SINGLE tongue at the spar's
# far end (around x = TIBIA_LENGTH); the FOOT_TANG_X_* constants set
# how far the tang extends inboard / past the spar tip and how far
# it reaches below the pin axis.  Renamed from the pre-2026
# ``FOOT_CLEVIS_X_*`` block when the tibia's clevis became a single
# tang; numeric values are unchanged so the tang occupies the same
# X / Z footprint as the old clevis bulk.
FOOT_TANG_X_INBOARD       = 12.0  # mm -- tang inboard of x=TIBIA_LENGTH
FOOT_TANG_X_BEYOND_TIP    =  6.0  # mm -- tang extending past x=TIBIA_LENGTH
FOOT_TANG_BELOW_PIN       = 5.0   # mm -- material below the pin axis in
                                  #        the tang (gives a ~1.5 mm ring
                                  #        of plastic around the M3 hole
                                  #        on the bottom side)

# ---- Battery / electronics enclosures ------------------------------------
# Sized for a generic 3S 2200 mAh LiPo (105 x 35 x 25 mm) plus an
# Arduino Nano or Mega + PCA9685 stack.
BATTERY_W = 110.0   # mm
BATTERY_D =  38.0
BATTERY_H =  28.0
BATTERY_WALL = 1.6
BATTERY_STRAP_W = 10.0   # velcro slot width

# ---- Battery holder foot geometry ----------------------------------------
# May 2026 fix (landed alongside the CHASSIS_GAP 20 -> 32 mm bump): the
# battery_holder is now bolted to chassis_bottom from BELOW via 4 x M3
# brass heat-set inserts in the foot bosses; the previous design left
# the holder unbolted (4 Phi 3.2 mm clearance holes through the feet
# that were never enumerated in fastener_registry, never matched by
# holes in chassis_bottom, and never even called out in PROTOTYPE.md
# beyond a single "bolt to the bottom plate" sentence).  Mirrors the
# f03d59b cradle insert pattern: each foot is a small printed boss
# that takes a Phi INSERT_M3_PILOT_OD = 4.0 mm x INSERT_M3_PILOT_DEPTH
# = 6.0 mm-deep pocket cut from the foot's BOTTOM face; the bolt
# enters from below (M3 x 10 SHCS through chassis_bottom), threads up
# into the brass insert.  Foot geometry in battery-holder-local
# (origin = centre of the holder's bottom face, +X = long axis,
# +Z = up):
#
#     foot centre (X, Y) = (sx * BATTERY_FOOT_DX, sy * BATTERY_FOOT_DY)
#     foot extents (X x Y x Z) = (BATTERY_FOOT_W x BATTERY_FOOT_D x
#                                  BATTERY_FOOT_T)
#     bottom face at z = 0; top face at z = BATTERY_FOOT_T
#     heat-set insert pocket: Phi INSERT_M3_PILOT_OD = 4 mm,
#         z in [0, INSERT_M3_PILOT_DEPTH] = [0, 6]
#     pocket leaves 2 mm of plastic above the insert (z in [6, 8]).
#
# Foot Y dimension (BATTERY_FOOT_D) is 2 mm larger than the X
# dimension so the foot's -Y face is COINCIDENT with the body's
# +/- Y wall at y = +/- BATTERY_D/2 = +/- 19 mm.  Without this the
# foot floats 1 mm clear of the body (sy * BATTERY_FOOT_DY = +/- 25
# minus BATTERY_FOOT_D/2 = +/- 19 + 1 mm gap) -- the old layout
# bonded the foot to the body via a 0 mm face which produced two
# disconnected manifold components.  The new layout shares a
# BATTERY_FOOT_W x BATTERY_FOOT_T = 10 x 8 mm face with the body
# wall and unions cleanly into a single mesh.
BATTERY_FOOT_W  = 10.0   # mm -- foot X dimension (footprint X)
BATTERY_FOOT_D  = 12.0   # mm -- foot Y dimension (footprint Y; 2 mm
                          #       larger than _W so the foot's -Y
                          #       face bonds onto the BATTERY_D/2 =
                          #       19 mm body wall in a 10 x 8 mm
                          #       face).
BATTERY_FOOT_T  =  8.0   # mm -- foot Z thickness (was BATTERY_WALL =
                          #       1.6 mm; bumped to fit a
                          #       Phi 4.0 mm x 6.0 mm-deep heat-set
                          #       insert pocket from the foot's
                          #       BOTTOM face with 2 mm of plastic
                          #       above the insert and 3 mm of
                          #       plastic radially around it).
BATTERY_FOOT_DX = BATTERY_W / 2.0 - BATTERY_FOOT_W / 2.0  # 50.0 mm
                          # foot centre X position; the foot's +X
                          # face is flush with the holder body's +X
                          # face at x = +BATTERY_W/2 = +55 mm.
BATTERY_FOOT_DY = BATTERY_D / 2.0 + BATTERY_FOOT_D / 2.0 - 1.0
                          # 24.0 mm; foot centre Y -- with
                          # BATTERY_FOOT_D = 12 mm this puts the
                          # foot's -Y face at y = sy * 18 mm, 1 mm
                          # INSIDE the BATTERY_D/2 = 19 mm body
                          # wall (the foot OVERLAPS the body wall
                          # by 1 mm so the boolean union produces
                          # a single connected mesh).
BATTERY_FOOT_INSERT_RECESS = 0.5  # mm -- insert top face is
                          # recessed this far INTO the foot from
                          # the foot's BOTTOM face so the bolt
                          # head's bearing ring clamps the
                          # chassis_bottom plate onto plastic, not
                          # brass (same convention as the cradle
                          # heat-set inserts -- 0.5 mm of plastic
                          # between bolt head and brass insert
                          # face).

# The battery_holder is NOT centred on the chassis -- it sits at
# X = BATTERY_HOLDER_CENTRE_X = -25 mm so the +X half of the
# chassis stays clear for the electronics_tray (at X = +35).  This
# constant is referenced from build_prototype_assembly.py,
# inspect_build.py, _verify_prototype.py's chassis-frame builders,
# fastener_registry's battery-holder bolt emitter, AND _hex_plate's
# battery-holder hole pattern -- they all have to use the same X
# offset or the chassis_bottom holes won't line up with the
# holder's feet and check_fastener_engagement reports the bolts
# "join only 1 part".
BATTERY_HOLDER_CENTRE_X = -25.0   # mm

# ---- M2.5 brass heat-set insert (Raspberry Pi mount) ---------------------
# McMaster 94459A106: M2.5 brass knurled heat-set insert.  Specs:
#   pilot Phi 3.0 mm, recommended pilot depth 4.5 mm,
#   insert OD (knurled max) 3.6 mm, insert length 4.0 mm.
# Installed with a soldering iron (same technique as the M3 inserts that
# carry the cradle servo bolts and the battery_holder foot bolts).  The
# Pi 4 / Pi 5 mounts on the electronics_tray via 4 of these inserts +
# M2.5 x 8 SHCS (PN_M25X8_SHCS).  Same fastener stock as the servo
# spline center screw -- a plain M2.5 x 8 SHCS with the role label
# distinguishing it from the captive spline screw.
INSERT_M25_PILOT_OD       = 3.0   # mm -- printed pilot Phi
INSERT_M25_PILOT_DEPTH    = 4.5   # mm -- printed pilot depth
INSERT_M25_INSERT_OD      = 3.6   # mm -- installed brass body knurled OD
INSERT_M25_INSERT_LENGTH  = 4.0   # mm -- physical insert length
INSERT_M25_BOLT_LENGTH    = 8.0   # mm -- M2.5 x 8 SHCS clamps the Pi
INSERT_M25_BOLT_HEAD_OD   = 4.5   # mm -- M2.5 SHCS head Phi
INSERT_M25_BOLT_HEAD_H    = 2.5   # mm -- M2.5 SHCS head height

# ---- Electronics tray ----------------------------------------------------
#
# Carries the THREE control boards used in the prototype (May 2026
# upgrade -- replaces the Arduino-Nano-only layout):
#
#   * Arduino Mega 2560 R3 (ELEGOO clone) -- 101.5 x 53.3 mm PCB with
#     the standard 4-hole Mega 2560 R3 footprint at offsets
#     MEGA_HOLES below (asymmetric pattern; see Arduino's reference
#     drawing).  4 x M3 SHCS into 4 x M3 brass heat-set inserts
#     (McMaster 94459A130) embedded in printed tray bosses.  Replaces
#     the old Arduino Nano hole pattern; the firmware moved to the
#     Mega so the Nano is no longer on the BOM.
#
#   * Raspberry Pi 4 Model B (or Pi 5; same 85 x 56 mm footprint with
#     the same 49 x 58 mm 4-hole pattern at offsets PI_HOLES below).
#     4 x M2.5 SHCS into 4 x M2.5 heat-set inserts (McMaster
#     94459A106) embedded in printed tray bosses.  NEW -- no Pi
#     mounting existed before the May 2026 hardware-arrival pass.
#
#   * 2 x Adafruit PCA9685 16-channel PWM driver -- 62 x 25 mm PCB
#     with the standard 54 x 16 mm 4-hole pattern (PCA_HOLES below).
#     4 x M3 SHCS into 4 x M3 heat-set inserts per board.  May 2026
#     "essentials" pass: the second PCA9685 (was previously cable-
#     tied / unbolted) now gets its own bolted hole pattern at
#     PCA2_CENTRE.  Two PCA9685 boards together carry the full 18
#     servos (9 per driver, daisy-chained over I2C: primary at
#     ``0x40`` and secondary at ``0x41``).
#
# Board placement (tray-local coords; tray origin at chassis (0, 0);
# tray-local +X = chassis +X):
#
#                    +Y (tray, chassis)
#                       ^
#       Mega2560        |     PCA9685 (long
#       (long axis      |      axis along Y;
#       along X; USB-B  |      servo-pin headers
#       + barrel jack   |      ride on +Z face)
#       on +X edge)     |
#                       |
#   ---------------------(0,0)------------------ +X
#                       |
#       Pi 4            |
#       (long axis      |
#       along X;        |
#       USB-A +         |
#       Ethernet on -Y  |
#       edge)           |
#
# Tray-local board centres come from MEGA_CENTRE / PI_CENTRE /
# PCA_CENTRE; the per-board mount holes come from MEGA_HOLES /
# PI_HOLES / PCA_HOLES (offsets from each board's centre).
#
# Tray sits in the same middle-bay z slot as the original 100 x 70 mm
# tray (z = CHASSIS_PLATE_T / 2 + 3 = 5 mm above chassis_bottom top
# face); the boards sit on ELEC_STANDOFF_H = 5 mm tall bosses.  Each
# boss is Phi CRADLE_BOSS_OD = 8 mm (M3 sites) or Phi 6 mm (M2.5
# sites) so the heat-set insert pocket has >= 1.5 mm of radial
# plastic.  The chassis-side mount holes still match the
# 35-mm-radius / 45-deg square pattern shared with chassis_bottom +
# chassis_top (and the arm baseplate, if present), with a Phi 5.5 mm
# x 3 mm-deep counterbore from the tray's TOP face so the chassis
# bolt heads sit FLUSH with the tray top -- the board standoff bosses
# on top of the tray then have an unobstructed 5 mm of clear air
# above the tray face for the M3 / M2.5 SHCS heads that clamp the
# boards onto the heat-set inserts.
#
# The tray's footprint OVERLAPS the battery_holder body and the two
# +X-side coxa_bracket flanges at the z = 5..8 slab the way the prior
# 100 x 70 mm tray did (the verifier does not flag chassis-fixed
# parts against each other, only against dynamic leg parts via
# ``check_workspace_self_collision``).  Cable-clearance keep-out
# volumes for the Mega's USB-B + barrel jack and the Pi's USB-C /
# HDMI / USB-A / Ethernet stack ARE modelled (May 2026
# "essentials" pass) -- see ``cable_keepouts.py`` for the
# registry and ``_verify_prototype.check_cable_clearance`` for
# the verifier.  Each keep-out is a simple box extending from the
# connector face along the cable's natural exit direction; the
# verifier asserts each keep-out has < 50 mm^3 of overlap with
# every printed STL, every modelled-electronics STL and every
# fastener mesh.
ELEC_TRAY_W = 160.0  # mm -- X dimension (long axis; spans chassis +/-80 mm)
# Tray Y span: May 2026 "Pi cantilever" pass grew the tray from 130 ->
# 135 mm (asymmetric, only in -Y) so the Pi's -Y mount-boss pair at
# tray-local y = -66.5 still lands on tray plastic with ~ 1 mm of
# radial wall around each boss.  The tray's chassis-frame Y centre
# was shifted from 0 -> -2.5 in lockstep so the tray's +Y edge stays
# at chassis y = +65 (the Mega + BEC cradle + PCA9685 layout on the
# +Y half doesn't move in chassis frame); only the -Y edge extends
# from -65 to -67.5 in chassis frame.  See ELEC_TRAY_CENTRE_Y +
# PI_CENTRE comments below for the cantilever rationale.
ELEC_TRAY_D = 135.0  # mm -- Y dimension (spans tray-local +/- 67.5 mm)
ELEC_TRAY_T =   3.0  # mm -- thickness (unchanged from the original tray)

ELEC_STANDOFF_H       = 5.0   # mm -- printed boss height above the tray top
                                # (boards sit ELEC_STANDOFF_H mm above the
                                # tray face so solder joints + headers
                                # underneath the PCB clear the tray).
ELEC_BOSS_OD_M3       = CRADLE_BOSS_OD   # 8 mm -- reuse the cradle constant
ELEC_BOSS_OD_M25      = 6.0    # mm -- smaller M2.5 boss (Phi 3 mm pilot +
                                # 1.5 mm wall = Phi 6 mm minimum).
ELEC_CHASSIS_COUNTERBORE_OD   = 5.5    # mm -- M3 SHCS head clearance Phi
ELEC_CHASSIS_COUNTERBORE_DEPTH = 3.0   # mm -- counterbore depth from tray top
                                        # (= ELEC_TRAY_T so the head sits
                                        # flush with the tray's bottom face;
                                        # the chassis bolt comes UP through
                                        # chassis_bottom + standoff + tray).

# Mega 2560 R3 mounting holes (Arduino reference drawing).  Listed in
# the BOARD'S OWN coordinate frame with origin at the PCB's bottom-left
# corner, +Y = USB-B end, +X = right edge as drawn.  All four are
# Phi 3.2 mm M3 clearance.
MEGA_PCB_W = 53.3   # mm -- PCB width  (board X dimension in board frame)
MEGA_PCB_D = 101.5  # mm -- PCB depth  (board Y dimension; USB-B at +Y end)
_MEGA_HOLES_BOARD_FRAME = (
    ( 2.54, 15.24),     # bottom-left
    (50.80, 15.24),     # bottom-right
    ( 7.62, 66.04),     # mid-left
    (50.80, 90.17),     # top-right (near USB-B)
)
# Hole offsets relative to BOARD CENTRE in the board's frame:
_MEGA_HOLES_BOARD_CENTRE = tuple(
    (bx - MEGA_PCB_W / 2.0, by - MEGA_PCB_D / 2.0)
    for bx, by in _MEGA_HOLES_BOARD_FRAME
)
# 90 deg CW rotation maps board +Y (USB-B end) to tray +X so the USB-B
# + barrel jack faces the +X chassis edge: (bx, by) -> (by, -bx).
MEGA_HOLES = tuple(
    (by, -bx) for (bx, by) in _MEGA_HOLES_BOARD_CENTRE
)
# Mega centre in tray-local coords.  Placed on the -X +Y quadrant so
# the bolt heads at the 35-mm-radius chassis mount pattern don't poke
# up through the Mega PCB (the tray-top counterbore recess takes
# care of the head clearance regardless; the offset just keeps the
# layout tidy).  May 2026 "essentials" pass: nudged 3 mm in -X (from
# -25 to -28) so the corridor between Mega's +X edge and PCA1's
# new -X edge widens to ~ 28 mm -- enough to drop in a snap-fit
# BEC cradle for the 2 x switching BEC bodies (24 x 15 x 8 mm each).
# Mega's -X edge at MEGA_CENTRE_X - MEGA_PCB_D/2 = -78.75 mm sits
# 1.25 mm inside the tray's -X edge at -80, with the Mega's own
# -X-most chassis-mount hole at -63.51 mm (well inboard of the
# tray edge), so the shift doesn't cantilever any board feature
# past the printed plate.
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from +28 -> +30.5
# to compensate for the tray's chassis-frame Y centre shift (0 ->
# -2.5).  The Mega's chassis-frame position (-28, +28) is UNCHANGED.
MEGA_CENTRE = (-28.0, +30.5)

# Raspberry Pi 4 / Pi 5 mounting holes.  Pi PCB is 85 x 56 mm with
# 4 x M2.5 holes on a 58 x 49 mm rectangle (asymmetric -- the long-
# axis hole pair is offset toward one end).  Listed in board frame
# with +X = PCB long axis (85 mm), +Y = short axis (56 mm), origin
# at the bottom-left corner.
PI_PCB_W = 85.0     # mm -- long axis
PI_PCB_D = 56.0     # mm -- short axis
_PI_HOLES_BOARD_FRAME = (
    ( 3.5,  3.5),
    (61.5,  3.5),
    ( 3.5, 52.5),
    (61.5, 52.5),
)
_PI_HOLES_BOARD_CENTRE = tuple(
    (bx - PI_PCB_W / 2.0, by - PI_PCB_D / 2.0)
    for bx, by in _PI_HOLES_BOARD_FRAME
)
# Pi long axis stays along tray +X so the USB-A + Ethernet bank (on
# the -Y long edge) faces chassis -Y, where the user can plug cables
# in from outside the chassis past the chassis_top -Y apothem at
# y = -70.  The USB-C + HDMI bank (on the -X short edge) faces
# chassis -X so HDMI/USB-C cables exit in a separate corridor from
# the +X PCA9685 servo header bank.
PI_HOLES = _PI_HOLES_BOARD_CENTRE
# Pi centre in tray-local coords.  -X -Y quadrant so the Pi's
# connector bank sits opposite the Mega's USB-B + barrel jack.
#
# May 2026 "Pi cantilever" pass: Y moved from -33 -> -42 (in tray-
# local frame, with the tray centre also shifted from chassis y = 0
# to chassis y = -2.5) so the Pi cantilevers south past the
# chassis_top -Y apothem.  Pi PCB -Y edge in chassis frame =
# ELEC_TRAY_CENTRE_Y + PI_CENTRE_Y - PI_PCB_D/2 = -2.5 + (-42) - 28
# = -72.5 mm -- 2.5 mm PAST chassis_top's -Y apothem at -70.  The
# entire USB-A/Ethernet connector bank (which is on the Pi's -Y
# long edge, 16 mm of body sticking off the edge -- chassis y range
# = -72.5 .. -88.5) is OUTSIDE the chassis_top hexagon silhouette
# in plan view, so the 17 mm-tall USB-A stacks + 14 mm-tall Ethernet
# RJ45 tops (which reach into the chassis_top z-band at z in
# [34, 38]) are in free air with no chassis_top above them.  Pre-
# cantilever (PI_CENTRE_Y = -33, tray-centre at 0) the PCB -Y edge
# was at chassis y = -61, inside the chassis_top hexagon, and the
# Ethernet jack body punched THROUGH chassis_top.
#
# Pi -Y mount holes at PI_CENTRE_Y + PI_HOLES y = -42 + (-24.5) =
# -66.5 in tray-local; with the tray's -Y edge at tray-local
# y = -ELEC_TRAY_D/2 = -67.5, the boss has ~ 1 mm of plastic wall
# around its -Y side.  Pi +Y mount holes at -42 + 24.5 = -17.5 sit
# well inboard of the tray's +Y edge.
PI_CENTRE = (-20.0, -42.0)

# Adafruit PCA9685 PWM driver -- 62 x 25 mm PCB with 4 x M3 holes
# on a 54 x 16 mm rectangle centred on the board.  Long axis = 62.
PCA_PCB_W = 62.0
PCA_PCB_D = 25.0
PCA_HOLES_BOARD_CENTRE = (
    (-27.0, -7.5),
    (+27.0, -7.5),
    (-27.0, +7.5),
    (+27.0, +7.5),
)
# Rotate PCA 90 deg so its long axis lies along tray +Y -- tucks
# into the +X strip of the tray between the Mega's +X edge and the
# tray's +X edge without touching either board.
PCA_HOLES = tuple((by, -bx) for (bx, by) in PCA_HOLES_BOARD_CENTRE)
# Primary PCA9685 (I2C 0x40) centre in tray-local coords.  May 2026
# "essentials" pass: shifted from (+55, +20) to (+64, +20) so the
# corridor between the Mega's +X edge and PCA1's -X edge widens
# enough to drop in the snap-fit BEC cradle (see BEC_CRADLE_*
# constants) AND so PCA2 (I2C 0x41) can fit a symmetric hole
# pattern on the -Y half of the tray at PCA2_CENTRE.  PCA1 -X
# edge at PCA_CENTRE_X - PCA_PCB_D/2 = +51.5 mm, +X edge at
# +76.5 mm (3.5 mm inside the tray's +X edge at +80).  +X-most
# mount-boss outer face at +71.5 + CRADLE_BOSS_OD/2 = +75.5 mm,
# 4.5 mm inside the tray's +X edge.
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from +20 ->
# +22.5 to compensate for the tray's chassis-frame Y centre shift
# (0 -> -2.5).  PCA1's chassis-frame position (+64, +20) unchanged.
PCA_CENTRE = (+64.0, +22.5)
# Secondary PCA9685 (I2C 0x41) centre in tray-local coords.  May
# 2026 "essentials" pass: previously this PCA9685 was on the BOM
# but was just cable-tied to the chassis_top deck with no printed
# hole pattern; the "essentials" pass adds a proper bolted mount.
# Placed symmetrically opposite the primary PCA9685 across the
# tray's X axis (PCA1 at +Y, PCA2 at -Y, both in the +X strip
# beyond x = +51.5 mm).  PCA2 servo headers (along its long edge,
# i.e. tray +/-X faces of the rotated PCA9685 PCB) fan out into
# the -Y leg cable routing direction; PCA1's headers do the same
# in +Y.  Together the two boards drive all 18 servos (9 per
# board) without any header crossing the tray midline.
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from -20 ->
# -17.5 (+2.5) to compensate for the tray's chassis-frame Y centre
# shift (0 -> -2.5).  PCA2's chassis-frame position (+64, -20)
# unchanged.
PCA2_CENTRE = (+64.0, -17.5)

# 35-mm-radius / 45-deg-square chassis-mount hole pattern (matches
# ``_hex_plate(with_centre_holes=True)`` on chassis_top + chassis_
# bottom and the 4 brass standoff columns between the plates).
ELEC_CHASSIS_MOUNT_R       = 35.0
ELEC_CHASSIS_MOUNT_HOLES_XY = tuple(
    (ELEC_CHASSIS_MOUNT_R * np.cos(np.pi / 4 + i * np.pi / 2),
     ELEC_CHASSIS_MOUNT_R * np.sin(np.pi / 4 + i * np.pi / 2))
    for i in range(4)
)

# Chassis-frame translation applied to the tray mesh by
# ``build_prototype_assembly._body_battery_parts`` and the verifier's
# ``_build_chassis_world`` / ``_build_world_leg0_printed_parts``.
# Moved from the original (+35, 0) to (0, 0) so the tray's chassis-
# mount holes line up CORRECTLY with the chassis-side
# 35-mm-radius / 45-deg-square pattern at chassis (+/-24.75, +/-24.75).
# The old +35 mm offset had the tray's bolt holes mapping to
# (+10.25, +/-24.75) and (+59.75, +/-24.75) instead -- a longstanding
# bug masked because the assembly preview is purely visual.
#
# May 2026 "Pi cantilever" pass: the Y centre shifted from 0 -> -2.5
# (paired with the asymmetric +5 mm -Y growth of ELEC_TRAY_D) so the
# Pi can cantilever 2.5 mm past the chassis_top -Y apothem at chassis
# y = -70.  The Pi sits at PI_CENTRE = (-20, -42) in tray-local coords,
# so its PCB -Y edge is at chassis y = -2.5 + (-42) - 28 = -72.5 --
# the entire USB-A/Ethernet connector bank sits OUTSIDE the chassis_
# top hexagon silhouette in plan view, so the 17 mm-tall USB-A stacks
# and 14 mm-tall Ethernet jack tops are in free air with no chassis_
# top above them to collide with.  Mega + PCA + BEC cradle tray-local
# Y centres bumped by +2.5 to keep their chassis-frame Y unchanged
# (the only board that actually moves in chassis frame is the Pi --
# everything else stays put; the 2.5 mm tray-local shift just keeps
# the chassis-frame positions invariant against the tray centre move).
# The chassis-mount holes in ``make_electronics_tray`` subtract
# ELEC_TRAY_CENTRE_* from the chassis-frame ELEC_CHASSIS_MOUNT_HOLES_XY
# positions so they STILL land on the 35-mm-radius chassis pattern
# after the tray is translated.
ELEC_TRAY_CENTRE_X = 0.0
ELEC_TRAY_CENTRE_Y = -2.5


# ---- BEC cradle ----------------------------------------------------------
#
# May 2026 "essentials" pass: 2 x 5V 5A switching BECs (Hobbywing
# UBEC form factor: ~ 24 x 15 x 8 mm body, ~ 5 cm input pigtail to
# the anti-spark switch's XT60 output and ~ 10 cm output pigtail to
# the nearest PCA9685's V+ / GND rail).  The BECs used to be cable-
# tied wherever; now they snap into a printed cradle that sits on
# top of the electronics_tray in the corridor between the Mega's
# +X edge and PCA1's -X edge.
#
# Cradle geometry summary:
#
#   * 2 BECs laid flat side-by-side along Y, BEC long axis (24 mm)
#     along tray X so the input and output pigtails exit naturally
#     out the cradle's +/- X end faces.
#   * Inner cavity:
#         X: BEC_BODY_L (24 mm) -- one BEC body's long axis
#         Y: 2 * BEC_BODY_W - BEC_CRADLE_INTERFERENCE (= 29.6 mm) so
#            the 2 BEC bodies share a 0.4 mm friction interference
#            fit across the inner span (no divider wall; the BECs
#            wedge against each other).
#         Z: BEC_BODY_H (8 mm) -- BEC body height
#   * Walls:
#         +/- X end walls: BEC_CRADLE_WALL (1.5 mm) thick, pierced
#                          by a Phi BEC_PIGTAIL_OD = 5 mm wire-
#                          exit channel centred on the cavity.
#         +/- Y side walls: BEC_CRADLE_WALL (1.5 mm) thick, full
#                           cavity height + lip.
#         Floor:           BEC_CRADLE_FLOOR (2 mm) thick.
#         +Z top lip:      BEC_CRADLE_LIP_H (2 mm) of inward
#                          overhang on each +/- Y side wall so the
#                          BEC body snaps in under the lip and
#                          stays captive.  The lip's inner edge
#                          is BEC_CRADLE_LIP_INSET (1.0 mm) inboard
#                          of the cavity wall, giving the BEC body
#                          a 1.0 mm-wide x 2 mm-tall retention tab
#                          to flex past on insertion.
#   * No bolts -- friction fit only.  The cradle sits free on the
#     tray's top face; gravity + the lip tabs hold the BECs and the
#     cradle itself stays in place because the corridor is bounded
#     by the Mega's +X edge to one side and PCA1's -X edge to the
#     other (the cradle floor's X extent is 1 mm shy of each
#     adjacent board's footprint for clearance).
BEC_BODY_L            = 24.0   # mm -- BEC PCB long axis (along cradle X)
BEC_BODY_W            = 15.0   # mm -- BEC PCB short axis (along cradle Y)
BEC_BODY_H            =  8.0   # mm -- BEC body height (along cradle Z)
BEC_PIGTAIL_OD        =  5.0   # mm -- Phi 5 mm wire-exit channel diameter
BEC_CRADLE_WALL       =  1.5   # mm -- side/end wall thickness
BEC_CRADLE_FLOOR      =  2.0   # mm -- floor thickness
BEC_CRADLE_LIP_H      =  2.0   # mm -- retention-lip overhang height
BEC_CRADLE_LIP_INSET  =  1.0   # mm -- lip inboard offset from cavity wall
BEC_CRADLE_INTERFERENCE = 0.4  # mm -- 2 BECs share this much overlap so
                                #     they wedge against each other for a
                                #     friction fit (positive = squeeze).
# Cradle centre in tray-local coords.  Placed in the corridor between
# the Mega's +X edge (at +22.75 mm = MEGA_CENTRE_X + MEGA_PCB_D/2)
# and PCA1's -X edge (at +51.5 mm = PCA_CENTRE_X - PCA_PCB_D/2).  The
# corridor is 28.75 mm wide; the cradle's outer X extent is
# BEC_BODY_L + 2 * BEC_CRADLE_WALL = 27 mm so there's ~ 0.9 mm of
# clearance on each side.  Chassis-frame Y position is +20 so the
# cradle's pigtail exits land within easy cable-routing distance of
# the nearest PCA9685's V+ / GND rail (PCA1's -X servo header row).
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from +20 ->
# +22.5 (+2.5) to compensate for the tray's chassis-frame Y centre
# shift (0 -> -2.5).  Cradle's chassis-frame position (+37, +20)
# unchanged.
BEC_CRADLE_CENTRE     = (+37.0, +22.5)


# ---- Anti-spark switch holster -------------------------------------------
#
# May 2026 "essentials" pass: the anti-spark XT60 on/off switch
# ("LowPro RC", "HRB", typical body ~ 30 x 15 x 15 mm with 2 short
# XT60 pigtails) used to be cable-tied wherever the user could fit
# it.  Now it bolts to a printed holster on chassis_top's +X edge
# between the L0 and L5 coxa_brackets, with the toggle protruding
# past the chassis vertex so the user can flip it from outside the
# chassis without opening the stack.
#
# Holster geometry summary:
#
#   * SOCKET HALF (+X half): a SWITCH_BODY_L x _W x _H box-with-
#     5-walls (open top so the switch body drops in from above)
#     that holds the switch body snugly (SWITCH_BODY_CL mm
#     clearance per axis).  The +X end wall has a SWITCH_TOGGLE_W
#     x SWITCH_TOGGLE_H rectangular cutout so the toggle protrudes
#     for user access.  The -X end wall has 2 x Phi
#     SWITCH_PIGTAIL_OD = 6 mm holes for the XT60 pigtails.
#   * MOUNTING EAR (-X half): a flat SWITCH_EAR_L x _W x
#     SWITCH_HOLSTER_FLOOR plate extending in -X from the socket
#     half's -X wall.  Each ear holds one M3 brass heat-set
#     insert (Phi INSERT_M3_PILOT_OD = 4 mm pocket inside a Phi
#     CRADLE_BOSS_OD = 8 mm boss that extends both UP (into the
#     ear's top face for the insert) and DOWN through chassis_top
#     (via a matching clearance hole in chassis_top).  Bolt enters
#     from ABOVE chassis_top, threads DOWN through chassis_top
#     into the insert -- bolt head bears on chassis_top's TOP
#     face from above; the boss extension DOWN through chassis_top
#     keeps the insert pocket fully captive in plastic.
#
# Wait -- a simpler arrangement that avoids the "boss pokes through
# chassis_top" complication: put the ear's floor flush with
# chassis_top's TOP face and have the insert pocket open at the
# ear's TOP face.  Then the bolt enters from above the holster, the
# bolt head bears on the EAR's top face (not chassis_top's), the
# bolt threads DOWN through the insert -- and a SHORTER bolt that
# DOESN'T pass through chassis_top is needed.  That doesn't bolt
# the holster to chassis_top though; it just bolts a switch into
# the holster.
#
# Final design (matches the battery_holder feet pattern, just
# rotated): the ear's heat-set insert pocket opens DOWNWARD at the
# ear's BOTTOM face (= chassis_top's top face).  Bolt enters from
# BELOW chassis_top (from inside the chassis cavity), threads UP
# through chassis_top into the insert.  Bolt head bears on
# chassis_top's BOTTOM face.  chassis_top carries a Phi
# BRACKET_BOLT_HOLE = 3.4 mm M3 clearance hole at each bolt
# position.  Insert is recessed 0.5 mm above the ear's bottom face
# so the bolt head clamps chassis_top against PLASTIC, not brass
# (same convention as the battery_holder feet inserts).  Bolts are
# captive sub-assembly fasteners -- torqued before chassis_top is
# clamped down with its 4 chassis-centre standoff bolts.
SWITCH_BODY_L         = 32.0   # mm -- switch body length (along X)
SWITCH_BODY_W         = 17.0   # mm -- switch body width (along Y)
SWITCH_BODY_H         = 17.0   # mm -- switch body height (along Z)
SWITCH_BODY_CL        =  0.5   # mm clearance per axis between body and cavity
SWITCH_HOLSTER_WALL   =  2.0   # mm -- holster outer wall thickness
SWITCH_HOLSTER_FLOOR  =  4.0   # mm -- ear thickness.  Just enough
                                #     plastic for the M3 SHCS to drive
                                #     into without dishing the head into
                                #     the printed face.  The insert
                                #     LIVES IN CHASSIS_TOP (in a
                                #     SWITCH_HOLSTER_BOSS_H-tall boss),
                                #     NOT in the ear, so the ear only
                                #     has a Phi BRACKET_BOLT_HOLE
                                #     clearance hole.
SWITCH_EAR_L          = 14.0   # mm -- mounting ear length (X) past
                                #     socket -X face
SWITCH_TOGGLE_W       = 14.0   # mm -- toggle cutout width (Y) in +X face
SWITCH_TOGGLE_H       = 10.0   # mm -- toggle cutout height (Z) in +X face
SWITCH_PIGTAIL_OD     =  6.0   # mm -- Phi 6 mm pigtail exit (clears a
                                #     12 AWG XT60 silicone-wire pigtail)
SWITCH_PIGTAIL_DY     =  5.0   # mm -- spacing between the 2 pigtail
                                #     channels on the -X face (centres
                                #     at +/- DY)

# Derived outer envelope.
SWITCH_SOCKET_OUTER_L = (SWITCH_BODY_L + 2.0 * SWITCH_BODY_CL
                          + 2.0 * SWITCH_HOLSTER_WALL)          # 39 mm
SWITCH_HOLSTER_OUTER_L = SWITCH_SOCKET_OUTER_L + SWITCH_EAR_L   # 53 mm
SWITCH_HOLSTER_OUTER_W = (SWITCH_BODY_W + 2.0 * SWITCH_BODY_CL
                          + 2.0 * SWITCH_HOLSTER_WALL)          # 22 mm
SWITCH_HOLSTER_OUTER_H = (SWITCH_BODY_H + SWITCH_BODY_CL
                          + SWITCH_HOLSTER_FLOOR)               # 23.5 mm

# Holster placement in CHASSIS frame (chassis_top's TOP face sits
# at world z = CHASSIS_GAP + CHASSIS_PLATE_T + CHASSIS_PLATE_T / 2
# = 38 mm; the holster's FLOOR bottom mates with chassis_top's TOP
# face at z = 38 mm).  Placed so the SOCKET +X face is flush with
# the chassis_top vertex at x = CHASSIS_TOP_FLAT_TO_FLAT / 2 /
# cos(30 deg) = 80.83 mm in +X; the toggle protrudes past the
# vertex for user access.  L0 and L5 coxa_bracket flanges are at
# (+86.6, +/- 50) -- well away from y = 0 so the holster's full
# 22 mm Y extent stays clear of them.
# chassis_top has FLATS on the +/-X side (at x = +/- apothem =
# CHASSIS_TOP_FLAT_TO_FLAT / 2 = +/- 70 mm; the flat spans
# y in [-40.41, +40.41] at x = +/- 70).  VERTICES at angles
# +/- 30, +/- 90, +/- 150 deg at distance circum = 80.83 mm.
# The "+X edge between L0 and L1 coxa_brackets" the user refers
# to is the +X flat (L5 is at angle 330 deg = (86.6, -50), L0 at
# angle 30 deg = (86.6, +50); the +X flat spans the gap between
# them along chassis_top's +X face).
SWITCH_CHASSIS_EDGE_X = CHASSIS_TOP_FLAT_TO_FLAT / 2.0  # = 70 mm
# Toggle must be reachable from outside the chassis: cantilever
# the holster so its +X face protrudes past chassis_top's +X
# flat by SWITCH_TOGGLE_REACH mm.  Toggle cutout itself adds
# another few mm of protrusion for the toggle stem.
SWITCH_TOGGLE_REACH    = 15.0  # mm -- holster +X face past chassis edge
# Holster MESH origin = holster X centre (midway between the
# socket's +X face and the ear's -X face).  Place that origin so
# the holster's +X face is at SWITCH_CHASSIS_EDGE_X + TOGGLE_REACH.
SWITCH_HOLSTER_CENTRE_X = (SWITCH_CHASSIS_EDGE_X
                           + SWITCH_TOGGLE_REACH
                           - SWITCH_HOLSTER_OUTER_L / 2.0)
SWITCH_HOLSTER_CENTRE_Y = 0.0

# 2 bolts through chassis_top into the holster's bottom-of-ear
# heat-set inserts.  Placed on the ear (the -X half of the
# holster) so the bolts thread through chassis_top material at
# safe distance from the +X vertex.  Each bolt's HOLSTER-LOCAL
# (x, y) is reported in SWITCH_HOLSTER_BOLT_OFFSETS; the absolute
# chassis-frame positions land at SWITCH_HOLSTER_CENTRE +
# offset.  chassis_top carries a matching Phi BRACKET_BOLT_HOLE =
# 3.4 mm clearance hole at each bolt site (see
# ``make_chassis_top`` -- it picks up SWITCH_HOLSTER_BOLT_OFFSETS
# from this module).
# Bolt at ear's CENTRE (X): 19.5 mm inboard from holster +X face
# (= SWITCH_HOLSTER_OUTER_L / 2 - SWITCH_EAR_L / 2 = 26.5 - 7 = 19.5).
SWITCH_HOLSTER_BOLT_DX = SWITCH_HOLSTER_OUTER_L / 2.0 - SWITCH_EAR_L / 2.0
SWITCH_HOLSTER_BOLT_DY = 5.0      # mm -- bolts at +/- 5 mm in Y
SWITCH_HOLSTER_BOLT_OFFSETS = (
    (-SWITCH_HOLSTER_BOLT_DX, +SWITCH_HOLSTER_BOLT_DY),
    (-SWITCH_HOLSTER_BOLT_DX, -SWITCH_HOLSTER_BOLT_DY),
)

# Chassis-top boss height under each switch_holster bolt position.
# The boss raises the chassis_top top face by SWITCH_HOLSTER_BOSS_H
# locally so the M3 heat-set insert (INSERT_M3_PILOT_DEPTH = 6 mm)
# fits ENTIRELY above the chassis_top BOTTOM face without punching
# through into the chassis cavity:
#   boss top z          = chassis_top top + 3 mm = 41 (design frame)
#   insert pocket depth = INSERT_M3_PILOT_DEPTH = 6 mm
#   insert pocket bottom z = 41 - 6 = 35 (= chassis_top centre - 1)
# Boss OD = SWITCH_HOLSTER_BOSS_OD (= 8 mm) leaves >= 2 mm of wall
# material around the Phi 4 mm pocket on every azimuth -- same
# captive-insert geometry as the electronics_tray standoff bosses.
# The holster ear sits ON TOP of the 2 bosses (the rest of the ear
# floats SWITCH_HOLSTER_BOSS_H above chassis_top's flat top face);
# the bolt threads DOWN from above the ear into the insert.
SWITCH_HOLSTER_BOSS_H  = 3.0   # mm above chassis_top top face
SWITCH_HOLSTER_BOSS_OD = 8.0   # mm boss OD on chassis_top
# CHASSIS-frame XY of the 2 bolt sites.  ``make_chassis_top`` calls
# this to add 2 clearance holes; ``fastener_registry`` calls this
# to enumerate the bolts + inserts.
SWITCH_HOLSTER_BOLT_CHASSIS_XY = tuple(
    (SWITCH_HOLSTER_CENTRE_X + ox, SWITCH_HOLSTER_CENTRE_Y + oy)
    for ox, oy in SWITCH_HOLSTER_BOLT_OFFSETS
)

# ---- IMU pad (MPU-6050 vibration-isolated mount) ------------------------
#
# May 2026: the MPU-6050 / GY-521 was the last "optional" item in the
# SHOPPING_LIST without a fixed mounting location.  Promoted to standard
# kit now that the Pi 4 / Pi 5 is on the robot (ROS 2 / Python brain
# expects orientation feedback for closed-loop body-attitude control on
# uneven terrain).
#
# Vibration isolation: the MPU-6050's HF gyro noise floor is very
# sensitive to high-frequency mechanical vibration, and brushed /
# digital hobby servos generate plenty of it.  The IMU must be
# MECHANICALLY DECOUPLED from the chassis -- bolting the breakout
# rigidly to chassis_top would feed every PWM-update jitter and tooth-
# slop pulse straight into the gyro.
#
# Damping strategy: a small printed pad (``imu_pad.stl``) carries the
# IMU on 4 x M3 brass heat-set inserts, and the pad ITSELF is glued
# to chassis_top via a 3 mm-thick strip of 3M VHB / generic double-
# sided mounting foam tape.  The foam tape doubles as the mount AND
# the vibration damper.  No fasteners between the pad and chassis_top.
#
# (Alternative considered: a 4-corner printed standoff with TPU 95A
# sleeves.  Mechanically nicer but complicates the build -- needs a
# second filament + a separate slicer profile for the TPU sleeves --
# and the foam-tape variant gets you 80 % of the damping with zero
# extra print effort.  See PROTOTYPE.md step 12 for the assembly
# rationale.)
#
# Standard GY-521 breakout PCB dimensions:
#
#     PCB:       21.2 x 16.4 x 1.6 mm
#     Holes:     4 holes in a 15.0 x 11.0 mm pattern, Phi 3.0 mm
#                clearance (most boards; a minority use Phi 2.5 mm --
#                we treat as Phi 3.0 for compatibility).
#     Header:    1 x 8-pin male header along one long edge (VCC, GND,
#                SCL, SDA, XDA, XCL, AD0, INT).
#
# Pad geometry (printed):
#
#     Floor:       IMU_PAD_W x IMU_PAD_D x IMU_PAD_T mm, flat smooth
#                  bottom for foam-tape adhesion.  Slightly LARGER
#                  than the IMU PCB so the foam-tape footprint
#                  extends past the board edges.
#     Bosses:      4 x Phi IMU_PAD_BOSS_OD mm cylinders at the IMU's
#                  15 x 11 mm hole pattern, rising IMU_PAD_BOSS_H mm
#                  above the floor's top face.  Each boss carries
#                  an M3 brass heat-set insert (McMaster 94459A130)
#                  in a Phi INSERT_M3_PILOT_OD x INSERT_M3_PILOT_DEPTH
#                  pocket opening DOWNWARD from the boss top.  The
#                  IMU PCB rides on the 4 boss tops; M3 x 8 SHCS
#                  thread DOWN through the PCB into the inserts.
#
# IMU_PAD_BOSS_H is deliberately TALLER than the SWITCH_HOLSTER_BOSS_H
# pattern (5 mm vs 3 mm) so a full 6 mm-deep insert pocket fits
# ENTIRELY above the pad's bottom face.  With the standard 2 mm pad
# floor + 5 mm bosses = 7 mm total, the 6.4 mm pocket overdrill ends
# at z = 0.6 mm above the pad bottom, leaving the bottom face FLAT
# for foam-tape adhesion.  A nominal "3 mm boss" would leave the
# pocket overhanging the foam-tape face by 1.4 mm -- which is why
# the convention diverges from the cradle / switch_holster boss
# heights here.
IMU_PCB_W = 21.2   # mm -- IMU breakout PCB long axis (X in pad frame)
IMU_PCB_D = 16.4   # mm -- IMU breakout PCB short axis (Y in pad frame)
IMU_PCB_T =  1.6   # mm -- IMU breakout PCB thickness (visual mesh only)
IMU_HOLE_PCD_X = 15.0   # mm -- IMU mounting-hole pattern X spacing
IMU_HOLE_PCD_Y = 11.0   # mm -- IMU mounting-hole pattern Y spacing

IMU_PAD_W       = 25.0   # mm -- pad X extent (slightly > IMU_PCB_W)
IMU_PAD_D       = 20.0   # mm -- pad Y extent (slightly > IMU_PCB_D)
IMU_PAD_T       =  2.0   # mm -- pad floor thickness (thin so foam tape
                          #      dominates the bond compliance)
IMU_PAD_BOSS_OD =  8.0   # mm -- boss OD around each insert pocket
IMU_PAD_BOSS_H  =  5.0   # mm -- boss height above the pad's top face
                          #      (= IMU PCB-to-pad standoff).  See the
                          #      block comment above for why we use
                          #      5 mm here instead of the nominal 3 mm
                          #      that the switch_holster / cradle
                          #      bosses use.
IMU_BOLT_CL     =  3.0   # mm -- Phi 3.0 mm clearance hole through each
                          #      boss for the IMU's M3 SHCS (M3 = Phi
                          #      3.0 mm body, +0 mm slop).  The IMU
                          #      PCB itself has the matching Phi 3.0
                          #      clearance hole.

# IMU pad chassis-frame placement.  Sit the pad on top of chassis_top
# at chassis (0, 0) so the gyro sits at the chassis CENTRE OF MASS --
# putting it at the chassis centre means the rotation rates are NOT
# contaminated by linear acceleration from the body swing.
# IMU_PAD_TAPE_T mm of double-sided mounting foam tape sits between
# the pad bottom and chassis_top top.  The pad's MESH ORIGIN is at
# the pad floor's BOTTOM face (z = 0 in pad-local), so
# build_prototype_assembly / inspect_build / verifier translate the
# pad to world z = chassis_top_top + IMU_PAD_TAPE_T.
IMU_PAD_CENTRE_X = 0.0    # mm -- chassis centre, no horizontal offset
IMU_PAD_CENTRE_Y = 0.0    # mm
IMU_PAD_TAPE_T   = 3.0    # mm -- nominal foam-tape thickness (3M VHB
                          #      / generic double-sided mounting foam).

# IMU hole positions in pad-local frame (mesh origin = pad floor centre).
IMU_PAD_HOLE_OFFSETS = (
    (-IMU_HOLE_PCD_X / 2.0, -IMU_HOLE_PCD_Y / 2.0),
    (-IMU_HOLE_PCD_X / 2.0, +IMU_HOLE_PCD_Y / 2.0),
    (+IMU_HOLE_PCD_X / 2.0, -IMU_HOLE_PCD_Y / 2.0),
    (+IMU_HOLE_PCD_X / 2.0, +IMU_HOLE_PCD_Y / 2.0),
)

# CHASSIS-frame XY of the 4 IMU bolt sites.  ``fastener_registry``
# enumerates the 4 bolts + 4 inserts at these positions.
IMU_PAD_BOLT_CHASSIS_XY = tuple(
    (IMU_PAD_CENTRE_X + ox, IMU_PAD_CENTRE_Y + oy)
    for ox, oy in IMU_PAD_HOLE_OFFSETS
)


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

def _servo_cradle_insert_pockets(
    shelf_top_z: float = None,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Return ``(bosses_union, pockets_union)`` for the 4 cradle bolt
    sites in a cradle shelf (MIXED MODE -- Design E, May 2026).

    Local frame: matches ``_servo_well_solid`` (origin at the body's
    bottom face centre; +X = body long axis = tab-span direction;
    +Y = body short axis; +Z = output shaft direction).

    Two distinct schemes are emitted, one per X column:

      * ``sx == -1`` (the 2 -X bolts per cradle): HEAT-SET INSERT scheme.

        - Boss: a vertical Phi ``CRADLE_BOSS_OD`` cylinder centred on
          ``(sx * SERVO_MOUNT_HOLE_X_OFFSET, sy * SERVO_MOUNT_HOLE_Y_OFFSET)``,
          spanning well-z in ``[shelf_top_z - CRADLE_BOSS_HEIGHT_MM,
          shelf_top_z]``.  Unioned into the cradle so the printed
          material around the insert pocket is at least
          ``CRADLE_BOSS_MIN_WALL_MM`` thick radially.
        - Pocket: a vertical Phi ``INSERT_M3_PILOT_OD`` cylinder
          centred on the same (x, y) axis, spanning well-z in
          ``[shelf_top_z - INSERT_M3_PILOT_DEPTH, shelf_top_z + 2.0]``.
          The pocket extends 2 mm ABOVE the shelf top so the boolean
          cut leaves a clean rim and 1 mm BELOW the insert length
          ``INSERT_M3_INSERT_LENGTH`` so debris from the heat-set
          installation has somewhere to go.  The pocket is subtracted
          from the cradle.

      * ``sx == +1`` (the 2 +X bolts per cradle): SELF-TAP PILOT scheme.

        - No boss is emitted -- the Phi ``CRADLE_BOSS_OD`` = 8 mm boss
          footprint physically intersects the +X WIRE_CHANNEL that the
          servo's molded wire boot (Phi ``WIRE_BOOT_W`` = 7 mm in Y)
          has to pass through during insertion, so the heat-set boss
          cannot coexist with the channel on this column.
        - A bare Phi ``INSERT_M3_SELFTAP_PILOT_OD`` = 2.5 mm pilot is
          sunk straight into the existing well-wall material below the
          servo tab, spanning well-z in
          ``[shelf_top_z - INSERT_M3_SELFTAP_PILOT_DEPTH,
            shelf_top_z + 2.0]``.  The M3 x 8 SHCS self-taps into this
          pilot -- the surrounding plastic is the thread engagement
          medium, NOT a brass insert.

        See the ``INSERT_M3_SELFTAP_*`` constant block near the top of
        this file for the design rationale (mixed-mode Design E,
        May 2026: restored wire channel + +X self-tap fallback) and
        ``check_servo_insertion_path`` in ``_verify_prototype.py`` for
        the regression probe that catches the heat-set-boss-vs-boot
        collision if it ever recurs.

    ``shelf_top_z`` defaults to ``WELL_RIM_Z`` (the nominal cradle rim
    height in well-local).  ``make_coxa_bracket`` passes a lower value
    when the bracket's drop-in slot has eaten the rim down by a known
    amount so its bolt-site bosses + pockets stay correctly aligned
    with the bracket's effective shelf top.

    Use as::

        bosses, pockets = _servo_cradle_insert_pockets()
        body = _union(body, bosses)
        body = _diff(body, pockets)

    Caller responsibility: apply the same well-to-cradle ``R`` /
    translation chain to BOTH meshes so the boss/pocket pair lands on
    the bolt PCD in the cradle's local frame.

    History: this helper replaced ``_servo_cradle_pilot_holes`` (which
    returned only the 4 Phi 2.5 mm self-tap pilot cylinders) in the
    May 2026 heat-set switch (commit f03d59b).  Design E (May 2026)
    REVERTED the 2 +X sites to the Phi 2.5 mm self-tap scheme because
    the heat-set boss + restored wire channel cannot coexist on the
    +X column; the 2 -X sites kept the heat-set scheme intact.
    """
    if shelf_top_z is None:
        # Default: the well's nominal rim height (used by coxa_link's
        # hip-pitch cradle, femur_link's knee cradle, and the well's
        # own internal cradle insert pockets).  ``make_coxa_bracket``
        # passes the bracket's *effective* shelf top (lowered by the
        # drop-in slot's wall bite -- see ``BRACKET_SHELF_DROP_MM``).
        shelf_top_z = WELL_RIM_Z

    # Heat-set (-X) geometry.  Boss extends from shelf top DOWN by
    # CRADLE_BOSS_HEIGHT_MM; pocket spans from 2 mm ABOVE shelf top
    # down to shelf_top_z - INSERT_M3_PILOT_DEPTH (1 mm of debris-
    # overdrill clearance below the 5 mm insert body).
    boss_radius = CRADLE_BOSS_OD / 2.0
    boss_z_top = shelf_top_z
    boss_z_bot = shelf_top_z - CRADLE_BOSS_HEIGHT_MM
    boss_h = boss_z_top - boss_z_bot
    boss_z_cen = 0.5 * (boss_z_top + boss_z_bot)

    heatset_pocket_radius = INSERT_M3_PILOT_OD / 2.0
    heatset_pocket_z_top = shelf_top_z + 2.0
    heatset_pocket_z_bot = shelf_top_z - INSERT_M3_PILOT_DEPTH
    heatset_pocket_h = heatset_pocket_z_top - heatset_pocket_z_bot
    heatset_pocket_z_cen = 0.5 * (heatset_pocket_z_top
                                    + heatset_pocket_z_bot)

    # Self-tap (+X) pilot geometry.  Pocket spans from 2 mm ABOVE
    # shelf top (matches the heat-set overdrill convention so the
    # cut rim looks identical from above) down to
    # shelf_top_z - INSERT_M3_SELFTAP_PILOT_DEPTH.  No boss is added
    # around the self-tap pilot -- adding a Phi 8 mm boss here would
    # reintroduce the very wire-channel collision Design E was
    # created to fix.
    selftap_pocket_radius = INSERT_M3_SELFTAP_PILOT_OD / 2.0
    selftap_pocket_z_top = shelf_top_z + 2.0
    selftap_pocket_z_bot = shelf_top_z - INSERT_M3_SELFTAP_PILOT_DEPTH
    selftap_pocket_h = selftap_pocket_z_top - selftap_pocket_z_bot
    selftap_pocket_z_cen = 0.5 * (selftap_pocket_z_top
                                    + selftap_pocket_z_bot)

    boss_parts: list[trimesh.Trimesh] = []
    pocket_parts: list[trimesh.Trimesh] = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * SERVO_MOUNT_HOLE_X_OFFSET
            y = sy * SERVO_MOUNT_HOLE_Y_OFFSET
            if sx == -1:
                # Heat-set scheme: Phi 8 mm boss + Phi 4 mm insert pocket.
                boss = _cyl(boss_radius, boss_h)
                boss.apply_translation([x, y, boss_z_cen])
                boss_parts.append(boss)
                pocket = _cyl(heatset_pocket_radius, heatset_pocket_h)
                pocket.apply_translation([x, y, heatset_pocket_z_cen])
                pocket_parts.append(pocket)
            else:
                # Self-tap scheme: bare Phi 2.5 mm pilot, no boss.
                pocket = _cyl(selftap_pocket_radius, selftap_pocket_h)
                pocket.apply_translation([x, y, selftap_pocket_z_cen])
                pocket_parts.append(pocket)

    return _union(*boss_parts), _union(*pocket_parts)


def _servo_well_solid(*, remove_floor: bool = False) -> trimesh.Trimesh:
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

    ``remove_floor=True`` cuts the floor frame (the annular slab around
    the body cavity at z in [-WELL_FLOOR_T, 0]) out of the well entirely,
    leaving a 4-wall "pen" that is OPEN on both +Z (rim / mouth) and -Z
    (former floor) faces.  The well's load paths -- 4 side walls,
    the tab shelf at z=WELL_RIM_Z, and the boss columns spanning
    z in [WELL_RIM_Z - CRADLE_BOSS_HEIGHT_MM, WELL_RIM_Z] -- are
    UNCHANGED by the cut: none of them depend on the floor frame for
    support, and the bosses live above z=0 so the cut at z<=0 cannot
    touch them.  Used by ``make_femur_link`` (May 2026 supports-free
    print) so the knee cradle does not bridge a ~30 x 40 mm closed
    ceiling when the femur is printed with the spar's broad face on
    the bed (mouth-down orientation).  See the femur's docstring for
    the full rationale.
        - 4 VERTICAL cradle bolt sites in the shelf material below
                     each servo ear (see ``_servo_cradle_insert_pockets``).
                     Sites split BY X SIGN under Design E (May 2026
                     mixed-mode):
                       * sx = -1 (the 2 -X bolts per cradle): heat-set
                         scheme -- Phi CRADLE_BOSS_OD = 8 mm boss +
                         Phi INSERT_M3_PILOT_OD = 4 mm pocket for an
                         M3 brass heat-set insert (McMaster 94459A130).
                       * sx = +1 (the 2 +X bolts per cradle): self-tap
                         scheme -- no boss, just a bare Phi
                         INSERT_M3_SELFTAP_PILOT_OD = 2.5 mm pilot in
                         the existing well-wall material so the M3 x 8
                         SHCS self-taps into plastic.  The Phi 8 mm
                         boss footprint cannot coexist with the +X
                         wire channel (Phi 7 mm boot must pass through
                         this column during insertion); see the
                         ``INSERT_M3_SELFTAP_*`` block near the top of
                         this file for the design rationale.
                     The boss / pilot tops sit at the shelf top
                     (``WELL_RIM_Z`` by default); each bolt enters from
                     above through the servo's factory-drilled ear hole.

    Design history:
      * Design D (May 2026 commit f03d59b -- heat-set switch): all 4
        bolt sites used the Phi 8 mm boss + Phi 4 mm heat-set pocket.
        Shortened the +X wire channel from z=29.5 down to z=16.5 to
        clear the new +X bosses, which then prevented the servo's
        molded wire boot from descending past the rim during insertion.
      * Design E (May 2026 mixed-mode revert): the 2 +X sites per
        cradle reverted to Phi 2.5 mm self-tap pilots (no boss), the
        wire channel was restored to full height, and the 2 -X sites
        kept the heat-set scheme intact.  See ``check_servo_insertion_
        path`` in ``_verify_prototype.py`` for the regression probe
        that catches the boss-vs-boot collision.

    The 4 servo tab holes ride at z = SERVO_MOUNT_HOLE_Z_OFFSET =
    SERVO_TAB_Z = 27 mm, y = +/- SERVO_MOUNT_HOLE_Y_OFFSET = +/-5 mm
    in well-local coords, which is exactly where the 4 cradle bolt
    sites are placed."""
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

    # 4 cradle bolt sites in the shelf material below each servo ear.
    # MIXED MODE (Design E, May 2026): the 2 -X sites (sx = -1) get a
    # Phi CRADLE_BOSS_OD = 8 mm boss + Phi INSERT_M3_PILOT_OD = 4 mm
    # heat-set insert pocket (McMaster 94459A130), and the 2 +X sites
    # (sx = +1) get a bare Phi INSERT_M3_SELFTAP_PILOT_OD = 2.5 mm
    # self-tap pilot in the existing well-wall material -- no boss is
    # added there because the Phi 8 mm boss footprint cannot coexist
    # with the +X wire channel.  See the INSERT_M3_SELFTAP_* /
    # INSERT_M3_* / CRADLE_BOSS_* blocks at the top of this file and
    # the ``_servo_cradle_insert_pockets`` docstring for the full
    # rationale.
    insert_bosses, insert_pockets = _servo_cradle_insert_pockets()

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
    #
    # The lead-in box's outer X edge sits at
    # +SERVO_BODY_W/2 + WELL_BODY_CL + WELL_LEAD_IN_EXTRA = +21.5 mm,
    # which reaches 0.75 mm INTO each -X heat-set boss (boss inner
    # edge at -SERVO_MOUNT_HOLE_X_OFFSET + CRADLE_BOSS_OD/2 = -20.75
    # mm).  Without protection the chamfer carves a 0.75 mm-wide
    # azimuthal air gap on the boss's inner face at the very top of
    # the rim, defeating the radial-material protection the boss is
    # supposed to provide there.  We subtract the heat-set bosses
    # from the lead-in box BEFORE applying it, so the chamfer cleanly
    # steps around each boss rather than slicing through it.
    #
    # Design E note: only the -X column has heat-set bosses now (the
    # +X column reverted to bare Phi 2.5 mm self-tap pilots so it
    # could coexist with the restored wire channel).  ``insert_bosses``
    # is therefore a 2-cylinder union (-X column) instead of the
    # original 4-cylinder union; the lead-in protection still applies
    # to the bosses that exist.
    lead_in = _box((SERVO_BODY_W + 2 * (WELL_BODY_CL + WELL_LEAD_IN_EXTRA),
                    SERVO_BODY_D + 2 * (WELL_BODY_CL + WELL_LEAD_IN_EXTRA),
                    WELL_LEAD_IN_H + 0.5),
                   center=(0.0, 0.0,
                            WELL_RIM_Z - (WELL_LEAD_IN_H + 0.5) / 2.0
                            + 0.25))
    lead_in = _diff(lead_in, insert_bosses)

    # Union the heat-set bosses INTO the well's outer body BEFORE the
    # boolean subtractions, so the boss material wraps each insert
    # pocket even when the underlying well wall would otherwise leave
    # the pocket half-open in Y (the audit's smoking gun).  Then
    # subtract the cavity, finger notch, lead-in chamfer (already
    # boss-protected above) and the insert pockets themselves.
    solid = _union(outer, insert_bosses)
    if remove_floor:
        # Subtract the entire floor frame (z in [-WELL_FLOOR_T, 0]).
        # X/Y oversized by 1 mm so the cut comfortably engulfs the
        # outer perimeter (extra hangs into air outside the well -- no
        # other geometry exists at well-local x>WELL_W/2 or y>WELL_D/2
        # in this helper's frame).  Z spans -WELL_FLOOR_T-1 .. 0 so
        # the cut overshoots the outer floor face by 1 mm and stops
        # cleanly at z=0; the wall material above z=0 is untouched
        # (walls + bosses span z in [0, WELL_RIM_Z] which is the body
        # cavity's z range plus the boss columns above the shelf).
        fc_z_min = -WELL_FLOOR_T - 1.0
        fc_z_max = 0.0
        floor_cut = _box(
            (WELL_W + 1.0, WELL_D + 1.0, fc_z_max - fc_z_min),
            center=(0.0, 0.0, 0.5 * (fc_z_min + fc_z_max)),
        )
        return _diff(solid, cavity, finger_notch, lead_in,
                     insert_pockets, floor_cut)
    return _diff(solid, cavity, finger_notch, lead_in, insert_pockets)


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
    # cavity face) out to WIRE_CHANNEL_DEPTH into the +X wall.
    #
    # +X cradle-boss reroute history (May 2026):
    #
    # The May 2026 heat-set switch (commit f03d59b) shortened this
    # channel's top from ``WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM =
    # +29.5 mm`` down to ``(WELL_RIM_Z - CRADLE_BOSS_HEIGHT_MM) -
    # 0.5 = +16.5 mm`` so the channel cut would not graze the new
    # Phi CRADLE_BOSS_OD = 8 mm heat-set bosses at
    # (+SERVO_TAB_HOLE_PCD/2, +/-SERVO_TAB_HOLE_PCD_Y/2) =
    # (+24.75, +/-5).  That worked for the SEATED boot
    # (z ~= [4.1, 8.0] in body-local, comfortably below the new
    # 16.5 mm cap) but BROKE the INSERTION path: the boot has to
    # slide DOWN through the +X wall from z = WELL_RIM_Z + boot_h
    # (above the rim) all the way to its seated z, and the 10.5 mm
    # of +X wall material between z = 16.5 and z = WELL_RIM_Z =
    # 27.25 stopped the boot from descending past the rim.  Real
    # DS3225 servos could not be seated in their printed cradles.
    #
    # Fix (Design E, May 2026 mixed-mode): restore the channel cap
    # to its pre-f03d59b value (``WELL_RIM_Z +
    # WIRE_CHANNEL_TOP_OVER_RIM``) so the boot has the full
    # insertion path AND switch the 2 +X heat-set sites per cradle
    # to Phi 2.5 mm M3 SELF-TAP pilots (no boss), since the
    # restored channel would have eaten the inboard half of each
    # +X boss.  Detailed rationale + alternative options
    # considered live in the INSERT_M3_SELFTAP_* constant block
    # near the top of this file; the verifier's
    # ``check_servo_insertion_path`` probe catches this regression
    # if it ever ships again.
    #
    # The 2 -X bolts per cradle KEEP their heat-set inserts (no
    # channel conflict on the -X column).  ``_servo_cradle_insert_
    # pockets`` builds heat-set bosses ONLY on the -X sites; the
    # +X sites get a bare Phi 2.5 mm pilot column in the wall
    # material.
    ch_x_min = +SERVO_BODY_W / 2.0 - WIRE_SLOT_X_INBOARD
    ch_x_max = +SERVO_BODY_W / 2.0 + WELL_BODY_CL + WIRE_CHANNEL_DEPTH
    ch_x_extent = ch_x_max - ch_x_min
    ch_x_centre = 0.5 * (ch_x_max + ch_x_min)

    ch_z_bottom = 0.0
    # Channel top sits ``WIRE_CHANNEL_TOP_OVER_RIM`` above the well
    # rim so the boot's swept volume during insertion clears the
    # +X wall all the way from the seated position
    # (boot bottom ~ z = WIRE_BOOT_Z_BASE + WELL_TAB_FLOAT = 5.6 mm)
    # up past the rim (z = WELL_RIM_Z = 27.25 mm) and a comfortable
    # exit margin above it.  The +X bolts in this z range now use
    # SELF-TAP pilots instead of heat-set bosses, so the channel
    # cut does not collide with any structural boss material on
    # this column.
    ch_z_top = WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM
    ch_z_extent = ch_z_top - ch_z_bottom
    ch_z_centre = 0.5 * (ch_z_bottom + ch_z_top)

    channel = _box((ch_x_extent, WIRE_SLOT_W, ch_z_extent),
                   center=(ch_x_centre, 0.0, ch_z_centre))

    return _union(exit_l, channel)


def _cable_zip_post() -> trimesh.Trimesh:
    """Return the printed-in zip-tie strain-relief post for one servo
    cradle.

    Local frame: same as ``_servo_well_solid`` / ``_wire_exit_slot``.
    The post is a small rectangular nub that protrudes from the well's
    OUTER +X face just past the wire-exit slot, so the assembler can
    loop a 2-3 mm zip-tie around the post AND the 3-wire harness as a
    printed-in strain relief.  Used by ``make_coxa_bracket`` (yaw
    cradle), ``make_coxa_link`` (hip-pitch cradle) and
    ``make_femur_link`` (knee cradle) -- each calls this helper, applies
    the SAME transform that the matching ``_wire_exit_slot()`` cut
    received, and UNIONs the result into the link's body.  See the
    ``CABLE_POST_*`` constants block near the top of this file for the
    geometry rationale, keep-out audit and printability argument.

    The post is anchored to the wall on its -X face (well_x = WELL_W/2)
    and extends OUTWARD by ``CABLE_POST_X``; the centre Y is offset to
    the +Y side of the slot by ``CABLE_POST_Y_OFFSET`` so the post does
    not sit IN the wire-exit slot's Y span; centre Z is
    ``CABLE_POST_Z_CENTRE`` (well-local Z above the cavity floor).  All
    three numeric values live in the constants block; do not hard-code
    here.
    """
    px_min = WELL_W / 2.0
    px_max = px_min + CABLE_POST_X
    py_centre = CABLE_POST_Y_OFFSET
    pz_centre = CABLE_POST_Z_CENTRE
    return _box(
        (CABLE_POST_X, CABLE_POST_Y, CABLE_POST_Z),
        center=(0.5 * (px_min + px_max), py_centre, pz_centre),
    )


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

    The 4 arms point along +X, +Y, -X, -Y (XHORN_BOLT_ANGLES_RAD).  Each
    arm carries a row of mounting holes; the SECOND hole out from the
    spline on each arm sits on XHORN_BOLT_PCD (= 20.8 mm) -- that's the
    pattern the printed ``servo_horn_adapter`` clamps onto.  This visual
    mesh drills those 4 bolt holes so any render that includes both the
    horn and the adapter shows the bolts lining up.
    Used as a visual stand-in only; the printed ``servo_horn_adapter``
    bolts on top of this.
    """
    parts: list[trimesh.Trimesh] = []
    # Hub spans z=[0, PLASTIC_HORN_H] so the mesh's bounding cylinder
    # matches the placement math (HORN_STACK_H = PLASTIC_HORN_H = 5 mm).
    # The pre-2026 mesh had the hub at z=[0, 2] and the arms ALSO at
    # z=[0, 1.6], so the visual stack was 3 mm SHORTER than the math
    # said (HORN_STACK_H = 5).  Inspectors saw a 3 mm air gap between
    # the horn top and the link bottom even when the placement math
    # was correct.  Now the hub fills the full 5 mm and the arms sit
    # on the hub TOP at z=[3.4, 5.0] so the link's bottom face mates
    # to the arms at z = PLASTIC_HORN_H = 5 (= link-local z = 0).
    hub_h = PLASTIC_HORN_H
    arm_t = 1.6
    hub = _cyl(8.0, hub_h)
    hub.apply_translation([0, 0, hub_h / 2.0])
    parts.append(hub)
    # Arm length 2 * PLASTIC_HORN_X_TIP_R so each arm's tip sits at the
    # real-hardware sweep radius (= 19 mm from the spline centre on a
    # standard DS3225 / MG996R / DS3218 horn).  Previously hard-coded
    # to 20 mm (tip at radius 10 mm), which was INSIDE the
    # XHORN_BOLT_PCD = 20.8 mm bolt circle -- nonsensical, and made the
    # mesh's bounding cylinder understate the horn's swept volume.
    # check_horn_sweep_clearance reads the bounding cylinder of this
    # mesh; keep it in sync with real hardware so the verifier
    # measures the right sweep radius.
    arm_z_centre = hub_h - arm_t / 2.0
    for a in XHORN_BOLT_ANGLES_RAD:
        arm = _box((2.0 * PLASTIC_HORN_X_TIP_R, 4.0, arm_t))
        arm.apply_transform(rotation_matrix(a, [0, 0, 1]))
        arm.apply_translation([0, 0, arm_z_centre])
        parts.append(arm)
    horn = _union(*parts)

    # Bolt holes drilled down through the arms, slightly into the hub
    # top (z range = [hub_h - arm_t - 0.4, hub_h]) so the CSG cut
    # cleanly punches through the arm even with FDM/voxel tolerance.
    bolt_holes = []
    hole_h = arm_t + 0.4
    hole_z = hub_h - hole_h / 2.0
    for a in XHORN_BOLT_ANGLES_RAD:
        h = _cyl(XHORN_BOLT_OD / 2.0, hole_h)
        h.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              XHORN_BOLT_PCD / 2.0 * np.sin(a),
                              hole_z])
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
        Bolt holes on XHORN_BOLT_PCD (= 20.8 mm) at XHORN_BOLT_ANGLES_RAD
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
    for a in XHORN_BOLT_ANGLES_RAD:
        h = _cyl(XHORN_BOLT_OD / 2.0, HORN_ADAPTER_T * 4)
        h.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              XHORN_BOLT_PCD / 2.0 * np.sin(a),
                              0])
        bolts.append(h)

    return _diff(plate, recess, centre, *bolts)


# ---------------------------------------------------------------------------
# Body parts
# ---------------------------------------------------------------------------

def _leg_chassis_frames():
    """Yield ``(leg_index, edge_mid_xyz, R_a, R3)`` for each of the 6
    legs in the hex chassis pattern.  Single source of truth for the
    leg-azimuth iteration: ``_hex_plate``, ``make_chassis_bottom``, the
    workspace verifier and the wire-harness planner ALL go through this
    helper so a future change to the leg layout (different leg count,
    different azimuth offset) only happens here.

    Mirrors ``_build_workspace_leg`` / ``_build_standing_leg`` /
    ``build_prototype_assembly._build_leg`` which all derive the same
    azimuth ``a = (leg_index + 0.5) * pi / 3`` and radius
    ``apothem = CHASSIS_FLAT_TO_FLAT / 2``.
    """
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    for i in range(6):
        a = (i + 0.5) * np.pi / 3.0
        edge_mid = np.array([apothem * np.cos(a),
                              apothem * np.sin(a),
                              0.0])
        R = rotation_matrix(a, [0, 0, 1])
        yield i, edge_mid, R, R[:3, :3]


def _hex_plate(flat_to_flat: float, thickness: float,
               with_centre_holes: bool = False,
               with_leg_features: bool = True,
               with_battery_holder_holes: bool = False,
               with_leg_harness_drops: bool = False) -> trimesh.Trimesh:
    """Return a flat hexagonal plate, centred on origin, axis = +Z.

    Hole pattern (per leg, 6 legs total):
        4 vertical M3 clearance holes (Φ BRACKET_BOLT_HOLE) drilled all
        the way through.  Pattern matches `make_coxa_bracket()`'s flange:
        2 holes on the OUTBOARD edge of the bolt rectangle (just inboard
        of the chassis perimeter) and 2 on the INBOARD edge.  All four
        holes are inboard of the apothem line so the bolt heads have
        chassis material under them.

    Optional inboard hole patterns:
        ``with_centre_holes``: 4 vertical M3 clearance holes on a
            35-mm-radius / 45-deg square, shared between the
            electronics tray's standoff bolts and any arm baseplate.
            chassis_top + chassis_bottom both carry this pattern.
        ``with_battery_holder_holes``: 4 vertical M3 clearance holes
            at (+/- BATTERY_FOOT_DX, +/- BATTERY_FOOT_DY) =
            (+/- 50, +/- 24) mm.  Matches the 4 mounting feet on
            battery_holder so the holder bolts to chassis_bottom
            FROM BELOW (M3 x 10 SHCS through this hole, threads up
            into the foot's heat-set insert).  ONLY chassis_bottom
            carries this pattern -- chassis_top sits ABOVE the
            battery and is not bolted to it.  The pattern is
            INTENTIONALLY separate from ``with_centre_holes``: the
            35-mm-radius square is far inboard (24.75 mm from
            origin) of the holder's 110 mm x 38 mm footprint, so
            moving the holder's feet onto the existing pattern
            would put them INSIDE the holder body.
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
    body_cutout_d  = WELL_D + 2.0            # 31 mm along bracket Y

    holes = []
    if with_leg_features:
        for i, edge_mid, R, R3 in _leg_chassis_frames():
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

            if with_leg_harness_drops:
                # Per-leg cable-drop slot through the plate, just
                # INBOARD of the body cutout, on the chassis radial
                # axis (slot long axis = bracket +X = chassis radial,
                # short axis = bracket Y = chassis tangential).  See
                # the LEG_HARNESS_DROP_* constants block above for
                # the geometry rationale.
                drop = _box((LEG_HARNESS_DROP_X_EXTENT,
                              LEG_HARNESS_DROP_Y_EXTENT,
                              thickness * 4))
                drop.apply_transform(R)
                drop_world = edge_mid + R3 @ np.array(
                    [LEG_HARNESS_DROP_X_CENTRE, 0.0, 0.0])
                drop.apply_translation(drop_world)
                holes.append(drop)

    if with_centre_holes:
        # 4 holes for the electronics tray standoffs + optional arm
        # baseplate (35 mm radius, 45 deg square).  IMPORTANT: this
        # is NOT the battery-holder bolt pattern -- see
        # ``with_battery_holder_holes`` below for that.  The battery-
        # holder feet sit at (+/- 50, +/- 24) mm, far outside the
        # 35 mm radius these standoff holes live on.
        for i in range(4):
            a = np.pi / 4 + i * np.pi / 2
            h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
            h.apply_translation([35.0 * np.cos(a), 35.0 * np.sin(a), 0])
            holes.append(h)

    if with_battery_holder_holes:
        # 4 holes aligned with battery_holder's mounting feet at
        # (BATTERY_HOLDER_CENTRE_X +/- BATTERY_FOOT_DX,
        #  +/- BATTERY_FOOT_DY).  Each takes an M3 x 10 SHCS driven
        # from BELOW that threads up into the heat-set insert in
        # the foot above.  Phi BRACKET_BOLT_HOLE = 3.4 mm clearance
        # for an M3 shank.  The holder is OFFSET in X relative to
        # the chassis centre (BATTERY_HOLDER_CENTRE_X = -25 mm)
        # because the +X half of the chassis carries the
        # electronics_tray; the chassis_bottom hole pattern has
        # to apply the same offset or the bolts miss the feet.
        for sx in (-1, 1):
            for sy in (-1, 1):
                h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
                h.apply_translation([BATTERY_HOLDER_CENTRE_X
                                       + sx * BATTERY_FOOT_DX,
                                      sy * BATTERY_FOOT_DY,
                                      0])
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
    bolts (battery/electronics tray + arm baseplate) remain -- PLUS,
    May 2026 "essentials" pass, 2 x switch_holster mount bosses on
    the +X half of the plate at SWITCH_HOLSTER_BOLT_CHASSIS_XY.
    Each boss is a Phi SWITCH_HOLSTER_BOSS_OD = 8 mm cylinder rising
    SWITCH_HOLSTER_BOSS_H = 3 mm above the chassis_top top face, with
    a Phi INSERT_M3_PILOT_OD = 4 mm x INSERT_M3_PILOT_DEPTH = 6 mm
    heat-set insert pocket opening UPWARD from the boss top.  The
    switch_holster ear sits ON TOP of these 2 bosses; an M3 x 10 SHCS
    threads DOWN from above the ear into the insert.
    """
    plate = _hex_plate(CHASSIS_TOP_FLAT_TO_FLAT, CHASSIS_PLATE_T,
                       with_centre_holes=True,
                       with_leg_features=False)

    # 2 switch_holster mounting bosses on the +X half of the plate.
    # Each boss is a Phi SWITCH_HOLSTER_BOSS_OD cylinder centred on
    # chassis (SWITCH_HOLSTER_BOLT_CHASSIS_XY[i]) that extends
    # SWITCH_HOLSTER_BOSS_H above the plate's TOP face (= z =
    # +CHASSIS_PLATE_T/2 = +2 in plate-local coords, since
    # ``_hex_plate`` extrudes symmetrically about z = 0).  Then the
    # Phi INSERT_M3_PILOT_OD insert pocket opens UPWARD from the
    # boss top, depth INSERT_M3_PILOT_DEPTH (+0.4 mm overdrill for
    # debris clearance, same convention as the cradle / battery_
    # holder / electronics_tray insert pockets).
    bosses = []
    pockets = []
    boss_h = SWITCH_HOLSTER_BOSS_H
    boss_top_z = CHASSIS_PLATE_T / 2.0 + boss_h
    pocket_overdrill_h = INSERT_M3_PILOT_DEPTH + 0.4
    for (cx, cy) in SWITCH_HOLSTER_BOLT_CHASSIS_XY:
        # Boss: a Phi BOSS_OD cylinder centred at the bolt position,
        # extending from a tiny bit BELOW plate-top (z = -0.2 mm,
        # for clean CSG-union with the plate) UP to boss_top_z.
        boss_bot_z = -0.2  # 0.2 mm below plate top for clean union
        boss_height = boss_top_z - boss_bot_z
        boss = _cyl(SWITCH_HOLSTER_BOSS_OD / 2.0, boss_height)
        boss_centre_z = (boss_top_z + boss_bot_z) / 2.0
        boss.apply_translation([cx, cy, boss_centre_z])
        bosses.append(boss)

        # Insert pocket: opens UPWARD from boss top, extends DOWN by
        # INSERT_M3_PILOT_DEPTH + 0.4 (overdrill).  Pocket top at
        # z = boss_top_z + 0.2 (cut a bit above for clean CSG),
        # bottom at z = boss_top_z - INSERT_M3_PILOT_DEPTH - 0.2 =
        # +5.0 - 6.2 = -1.2 (INSIDE the plate's z range [-2, +2];
        # leaves 0.8 mm of plate plastic below the pocket bottom).
        pocket = _cyl(INSERT_M3_PILOT_OD / 2.0, pocket_overdrill_h)
        pocket_centre_z = boss_top_z - INSERT_M3_PILOT_DEPTH / 2.0
        pocket.apply_translation([cx, cy, pocket_centre_z])
        pockets.append(pocket)

    return _diff(_union(plate, *bosses), *pockets)


def make_chassis_bottom() -> trimesh.Trimesh:
    """Bottom hex plate.  Structural carrier for the coxa-bracket
    flanges, the electronics tray + arm baseplate standoffs (35 mm
    radius / 45 deg pattern via ``with_centre_holes``), and the
    battery_holder feet (BATTERY_FOOT_DX / DY pattern via
    ``with_battery_holder_holes``).  May 2026 fix: the holder used to
    be unbolted (no chassis-side hole pattern; the holder's feet
    drilled clearance holes that mated to nothing); now 4 x M3 x 10
    SHCS pass UP through this plate into heat-set inserts in the
    battery_holder feet.

    Cable management (Part A + Part B, May 2026):

    * Each leg gets a small ``LEG_HARNESS_DROP_X_EXTENT`` x
      ``LEG_HARNESS_DROP_Y_EXTENT`` mm slot cut THROUGH the plate just
      INBOARD of the body cutout (long axis = chassis radial = bracket
      +X), so the leg's 3-cable harness can drop from the cradle side
      of the plate into the inter-plate volume without sharing the
      body cutout with the seated yaw servo body.  Applied by
      ``_hex_plate`` via ``with_leg_harness_drops=True``.
    * Each leg also gets a small vertical anchor TAB hanging DOWN
      ``CABLE_ANCHOR_TAB_H`` mm from the plate's -Z (bottom) face,
      adjacent to the drop slot, so the assembler can zip-tie the
      leg harness to the tab BELOW the chassis plate -- exactly where
      the cradle-to-drop-slot bundle makes its U-turn.  The tab hangs
      DOWN rather than rising UP because the electronics_tray fully
      blankets the inter-plate volume directly above each drop slot
      (chassis-z in [+5, +8]) and we are NOT allowed to modify the
      tray.  See the CABLE_ANCHOR_TAB_* constants block above for
      the placement rationale and keep-out audit.  Added by THIS
      function AFTER ``_hex_plate`` (the tabs are NOT a plate-
      thickness feature; they are full 3D objects hanging below
      the plate).
    """
    plate = _hex_plate(CHASSIS_FLAT_TO_FLAT, CHASSIS_PLATE_T,
                       with_centre_holes=True,
                       with_battery_holder_holes=True,
                       with_leg_harness_drops=True)

    # Per-leg vertical anchor tabs HANGING DOWN from the plate's -Z
    # (bottom) face.  Each tab is a CABLE_ANCHOR_TAB_T x
    # CABLE_ANCHOR_TAB_W x CABLE_ANCHOR_TAB_H mm box rooted at the
    # plate's BOTTOM face (chassis z = -CHASSIS_PLATE_T/2) and
    # extending DOWN in chassis -Z by CABLE_ANCHOR_TAB_H.  Placed at
    # bracket-local (CABLE_ANCHOR_TAB_X, +CABLE_ANCHOR_TAB_Y_OFFSET,
    # *), then rotated and translated into chassis frame by the same
    # _leg_chassis_frames iterator that places the body cutouts and
    # the drop slots, so the tab's chassis-radial position lines up
    # with its leg's drop slot to within float precision.
    tabs: list[trimesh.Trimesh] = []
    # 0.4 mm of vertical overlap with the plate's bottom face for a
    # clean CSG union (no co-planar face artefacts), so the tab box
    # extends from chassis-z = -CHASSIS_PLATE_T/2 + 0.2 (= -1.8 mm,
    # 0.2 mm INTO the plate) down to chassis-z = -CHASSIS_PLATE_T/2
    # - CABLE_ANCHOR_TAB_H (= -12 mm).  Total box height =
    # CABLE_ANCHOR_TAB_H + 0.2 mm.
    tab_overlap = 0.2
    tab_z_max = -CHASSIS_PLATE_T / 2.0 + tab_overlap
    tab_z_min = tab_z_max - CABLE_ANCHOR_TAB_H - tab_overlap
    tab_z_extent = tab_z_max - tab_z_min
    tab_z_centre = 0.5 * (tab_z_min + tab_z_max)
    for _i, edge_mid, R, R3 in _leg_chassis_frames():
        tab = _box((CABLE_ANCHOR_TAB_T,
                    CABLE_ANCHOR_TAB_W,
                    tab_z_extent),
                   center=(0.0, 0.0, tab_z_centre))
        tab.apply_transform(R)
        tab_world_xy = edge_mid + R3 @ np.array(
            [CABLE_ANCHOR_TAB_X,
             CABLE_ANCHOR_TAB_Y_OFFSET,
             0.0])
        tab.apply_translation([tab_world_xy[0], tab_world_xy[1], 0.0])
        tabs.append(tab)

    return _union(plate, *tabs)


def make_battery_holder() -> trimesh.Trimesh:
    """Open-top tray for one 3S 2200 mAh LiPo (105 x 38 x 28 mm).

    Two velcro slots cut through the long walls let the user strap the
    pack down.  Four mounting feet at (+/- BATTERY_FOOT_DX,
    +/- BATTERY_FOOT_DY) each carry an M3 brass heat-set insert
    (McMaster ``94459A130``) pressed into a
    Phi INSERT_M3_PILOT_OD = 4.0 mm x INSERT_M3_PILOT_DEPTH = 6.0 mm
    pocket cut from the foot's BOTTOM face; the holder bolts to
    chassis_bottom FROM BELOW via 4 x M3 x 10 SHCS that pass through
    the chassis_bottom plate and thread UP into the inserts.

    See the BATTERY_FOOT_* constants block above for the geometry
    rationale (Y-overlap with the body wall for boolean-union
    bonding; recessed insert top so the bolt head clamps the
    chassis plate against plastic, not brass; ditto the f03d59b
    cradle insert pattern this fix mirrors).
    """
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

    # 4 mounting feet, each with a Phi 4 mm x 6 mm heat-set insert
    # pocket cut from the bottom face.  Foot footprint is
    # BATTERY_FOOT_W x BATTERY_FOOT_D x BATTERY_FOOT_T mm; pocket is
    # centred on the foot, opens at z = 0 (foot bottom = holder
    # bottom face = chassis_bottom top face mating plane).
    feet = []
    insert_pockets = []
    pocket_radius = INSERT_M3_PILOT_OD / 2.0
    pocket_overdrill_h = INSERT_M3_PILOT_DEPTH + 0.4   # 0.4 mm slop so
                                                       # the cut clears
                                                       # the foot's
                                                       # bottom face
    for sx in (-1, 1):
        for sy in (-1, 1):
            fx = sx * BATTERY_FOOT_DX
            fy = sy * BATTERY_FOOT_DY
            ft = _box((BATTERY_FOOT_W, BATTERY_FOOT_D, BATTERY_FOOT_T),
                      center=(fx, fy, BATTERY_FOOT_T / 2.0))
            feet.append(ft)
            pocket = _cyl(pocket_radius, pocket_overdrill_h)
            # Pocket extends from z = -0.2 (slightly below the foot's
            # bottom face so the boolean diff cuts cleanly through it)
            # up to z = INSERT_M3_PILOT_DEPTH + 0.2; the resulting
            # void inside the foot is z in [0, INSERT_M3_PILOT_DEPTH]
            # = [0, 6], leaving BATTERY_FOOT_T - INSERT_M3_PILOT_DEPTH
            # = 2 mm of plastic above the insert and ~ 3 mm of
            # plastic radially around it.
            pocket.apply_translation([fx, fy,
                                       pocket_overdrill_h / 2.0 - 0.2])
            insert_pockets.append(pocket)

    # +X-end cable-clearance notch.  The Pi 4 / Pi 5's USB-A 3.0
    # PAIR (blue) plug envelope at chassis (x in [+22.5, +44.5],
    # y in [-20, -8], z in [+22, +30]) overlaps the
    # battery_holder's -Y cradle wall AND its +X end wall at the
    # holder's +X corner -- a known geometry conflict introduced
    # in the May 2026 "essentials" pass when the secondary PCA9685
    # forced the Pi's +X edge to butt up against the battery_holder
    # extent.  We carve a single 14 x 12 x 10 mm notch out of the
    # holder's +X -Y corner (local x in [+43, +57], y in [-20, -8],
    # z in [+18, +28]) so the plug-airspace check passes and a
    # standard USB-A cable can be plugged into the Pi.  The lower
    # 18 mm of the cradle wall + the entire +Y cradle wall + 75 %
    # of the +X end wall remain intact, so the BATTERY_FOOT and
    # velcro-strap retention are unaffected (notch volume ~ 1700
    # mm^3 vs. the holder's ~ 6.5 cm^3 wall mass).
    notch_x_lo = BATTERY_W / 2.0 - 12.0      # local +43
    notch_x_hi = BATTERY_W / 2.0 + 2.0       # local +57
    notch_y_lo = -BATTERY_D / 2.0 - 1.0      # local -20
    notch_y_hi = -BATTERY_D / 2.0 + 11.0     # local  -8
    notch_z_lo = 18.0                         # local +18
    notch_z_hi = BATTERY_H + 2.0              # local +30
    cable_notch = _box((notch_x_hi - notch_x_lo,
                         notch_y_hi - notch_y_lo,
                         notch_z_hi - notch_z_lo),
                        center=((notch_x_lo + notch_x_hi) / 2.0,
                                (notch_y_lo + notch_y_hi) / 2.0,
                                (notch_z_lo + notch_z_hi) / 2.0))

    body = _union(outer, *feet)
    return _diff(body, inner, *velcro, *insert_pockets, cable_notch)


def _board_standoff_boss_and_pocket(
    x: float, y: float, *,
    pilot_od: float,
    pilot_depth: float,
    boss_od: float,
    boss_height: float,
    tray_top_z: float,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Return ``(boss, pocket)`` for one printed board-standoff site.

    The boss is a vertical Phi ``boss_od`` cylinder spanning
    ``[tray_top_z, tray_top_z + boss_height]``.  The pocket is a Phi
    ``pilot_od`` cylinder spanning ``[tray_top_z + boss_height -
    pilot_depth - 0.5, tray_top_z + boss_height + 0.5]``.  The boss
    is to be UNIONed onto the tray and the pocket SUBTRACTed.  The
    pocket extends 0.5 mm above the boss top so the boolean diff
    clears the rim and 0.5 mm below the insert (room for debris).

    Mirrors the cradle-bolt boss/pocket pair (``_servo_cradle_
    insert_pockets``) but lives on top of the electronics tray
    instead of inside a servo cradle.  Used by ``make_electronics_
    tray`` for all 12 board-mount sites (4 x Mega, 4 x Pi, 4 x
    PCA9685).
    """
    boss = _cyl(boss_od / 2.0, boss_height)
    boss.apply_translation([x, y, tray_top_z + boss_height / 2.0])

    pocket_h = pilot_depth + 1.0
    pocket = _cyl(pilot_od / 2.0, pocket_h)
    pocket_z_centre = tray_top_z + boss_height - pilot_depth / 2.0
    pocket.apply_translation([x, y, pocket_z_centre])
    return boss, pocket


def _absolute_xy(centre: tuple[float, float],
                  offsets: tuple[tuple[float, float], ...]):
    """Helper: translate per-board hole offsets into tray-local
    absolute (x, y) tuples."""
    cx, cy = centre
    return tuple((cx + ox, cy + oy) for ox, oy in offsets)


def make_electronics_tray() -> trimesh.Trimesh:
    """Flat 3D-printed deck that carries the Arduino Mega 2560 + the
    Raspberry Pi 4 (or Pi 5) + TWO PCA9685 PWM drivers.

    Replaces the May-2026-earlier "Arduino Nano + PCA9685" layout --
    the user upgraded the firmware host to an ELEGOO Mega 2560 R3,
    added a Pi 4 for higher-level control, and (May 2026
    "essentials" pass) bolted BOTH PCA9685s instead of letting the
    secondary one float on cable ties.  The Mega + Pi + PCA1 + PCA2
    use the parametric MEGA_HOLES / PI_HOLES / PCA_HOLES board
    patterns + MEGA_CENTRE / PI_CENTRE / PCA_CENTRE / PCA2_CENTRE
    placements declared in the constants block above.

    Geometry summary:

        * Plate: ELEC_TRAY_W x ELEC_TRAY_D x ELEC_TRAY_T centred on
          tray-local (0, 0).  Tray-local origin = chassis (0, 0)
          after ``build_prototype_assembly`` places the tray.
        * 4 chassis-mount holes on the 35-mm-radius / 45-deg square
          pattern (shared with chassis_top + chassis_bottom + the 4
          brass standoff columns between the plates).  Each hole is
          Phi BRACKET_BOLT_HOLE = 3.4 mm with a Phi ELEC_CHASSIS_
          COUNTERBORE_OD = 5.5 mm x ELEC_CHASSIS_COUNTERBORE_DEPTH =
          3.0 mm counterbore from the tray's TOP face so the M3
          SHCS head sits flush with the tray top -- boards on the
          ELEC_STANDOFF_H = 5 mm standoff bosses then clear the
          chassis bolts entirely.
        * 4 printed bosses + heat-set insert pockets for the Mega
          (Phi 8 mm boss, Phi 4 mm pocket, McMaster 94459A130
          insert) -- 4 x M3 SHCS clamps the Mega onto the boss tops.
        * 4 printed bosses + heat-set insert pockets for the Pi 4
          (Phi 6 mm boss, Phi 3 mm pocket, McMaster 94459A106
          insert) -- 4 x M2.5 SHCS clamps the Pi onto the boss tops.
        * 4 printed bosses + heat-set insert pockets for the
          PRIMARY PCA9685 at PCA_CENTRE (same Phi 8 / 4 mm M3
          geometry as the Mega).
        * 4 printed bosses + heat-set insert pockets for the
          SECONDARY PCA9685 at PCA2_CENTRE (same geometry as the
          primary).  May 2026 "essentials" pass.

    All 16 board-mount fasteners are CAPTIVE SUB-ASSEMBLY fasteners:
    they are torqued during the electronics install BEFORE the
    chassis_top + arm stack closes over the tray; once the top plate
    is on, a hex key cannot reach the heads.  ``_emit_electronics_
    tray_fasteners`` in fastener_registry.py flags them with
    ``skip_screwdriver_reason`` accordingly.
    """
    plate = _box((ELEC_TRAY_W, ELEC_TRAY_D, ELEC_TRAY_T),
                 center=(0, 0, ELEC_TRAY_T / 2.0))

    tray_top_z = ELEC_TRAY_T

    bosses: list[trimesh.Trimesh] = []
    pockets: list[trimesh.Trimesh] = []

    # --- Mega 2560 board-mount sites (M3 inserts) ---
    for (hx, hy) in _absolute_xy(MEGA_CENTRE, MEGA_HOLES):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M3_PILOT_OD,
            pilot_depth=INSERT_M3_PILOT_DEPTH,
            boss_od=ELEC_BOSS_OD_M3,
            boss_height=ELEC_STANDOFF_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # --- Pi 4 board-mount sites (M2.5 inserts) ---
    for (hx, hy) in _absolute_xy(PI_CENTRE, PI_HOLES):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M25_PILOT_OD,
            pilot_depth=INSERT_M25_PILOT_DEPTH,
            boss_od=ELEC_BOSS_OD_M25,
            boss_height=ELEC_STANDOFF_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # --- Primary PCA9685 board-mount sites (M3 inserts) ---
    for (hx, hy) in _absolute_xy(PCA_CENTRE, PCA_HOLES):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M3_PILOT_OD,
            pilot_depth=INSERT_M3_PILOT_DEPTH,
            boss_od=ELEC_BOSS_OD_M3,
            boss_height=ELEC_STANDOFF_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # --- Secondary PCA9685 board-mount sites (M3 inserts) ---
    # May 2026 "essentials" pass: PCA2 (I2C 0x41) was previously
    # cable-tied to the chassis_top deck; now bolted properly via
    # 4 x M3 SHCS into M3 heat-set inserts on the tray.
    for (hx, hy) in _absolute_xy(PCA2_CENTRE, PCA_HOLES):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M3_PILOT_OD,
            pilot_depth=INSERT_M3_PILOT_DEPTH,
            boss_od=ELEC_BOSS_OD_M3,
            boss_height=ELEC_STANDOFF_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # --- 4 chassis-mount holes (35 mm radius / 45 deg square) ---
    # Phi BRACKET_BOLT_HOLE = 3.4 mm M3 clearance through the full
    # ELEC_TRAY_T thickness, with a Phi ELEC_CHASSIS_COUNTERBORE_OD =
    # 5.5 mm x ELEC_CHASSIS_COUNTERBORE_DEPTH = 3.0 mm counterbore
    # from the tray's TOP face so the chassis-side M3 SHCS head sits
    # flush with the tray top (the head face ends up coincident with
    # tray-local z = 0).  Boards on the 5 mm standoffs above the
    # tray then have an unobstructed 5 mm of clear air above the
    # tray face -- no PCB-vs-bolt-head conflict.
    #
    # ELEC_CHASSIS_MOUNT_HOLES_XY is the CHASSIS-frame pattern (35 mm
    # radius / 45 deg square = (+/-24.75, +/-24.75)).  We subtract
    # (ELEC_TRAY_CENTRE_X, ELEC_TRAY_CENTRE_Y) so the holes' tray-
    # local positions still map onto the chassis pattern after the
    # tray is translated.  Pre-May-2026 the tray centre was (0, 0)
    # and chassis-frame == tray-local; the May 2026 "Pi cantilever"
    # pass shifted the tray centre to (0, -2.5) -- without this
    # subtraction the tray bolts would no longer line up with
    # chassis_top + chassis_bottom's centre hole pattern.
    mount_holes: list[trimesh.Trimesh] = []
    for (mx_chassis, my_chassis) in ELEC_CHASSIS_MOUNT_HOLES_XY:
        mx = mx_chassis - ELEC_TRAY_CENTRE_X
        my = my_chassis - ELEC_TRAY_CENTRE_Y
        through = _cyl(BRACKET_BOLT_HOLE / 2.0, ELEC_TRAY_T * 4.0)
        through.apply_translation([mx, my, ELEC_TRAY_T / 2.0])
        mount_holes.append(through)

        cbore = _cyl(ELEC_CHASSIS_COUNTERBORE_OD / 2.0,
                     ELEC_CHASSIS_COUNTERBORE_DEPTH + 0.4)
        cbore_z_centre = ELEC_TRAY_T - ELEC_CHASSIS_COUNTERBORE_DEPTH / 2.0
        cbore.apply_translation([mx, my, cbore_z_centre + 0.2])
        mount_holes.append(cbore)

    body = _union(plate, *bosses)
    return _diff(body, *pockets, *mount_holes)


def make_bec_cradle() -> trimesh.Trimesh:
    """Snap-fit cradle for the 2 x 5V 5A switching BECs (Hobbywing
    UBEC form factor, ~ 24 x 15 x 8 mm body).

    Sits on top of the electronics_tray in the corridor between the
    Mega's +X edge and PCA1's -X edge.  No bolts -- the 2 BEC bodies
    wedge in side-by-side along Y with a BEC_CRADLE_INTERFERENCE-mm
    interference fit and a 2 mm-tall retention lip on each +/- Y
    side wall snaps over the BEC's top edge so it stays captive
    against vibration.  Both XT60 input pigtails exit out the +X end
    wall and both 3-pin servo-header output pigtails exit out the
    -X end wall via Phi BEC_PIGTAIL_OD = 5 mm wire channels.

    Local frame (mesh origin = cradle centre on the tray's top face):
        +X = pigtail axis (input on +X end, output on -X end)
        +Y = wider axis -- 2 BECs span this dimension
        +Z = UP (cradle floor at z = 0, cavity extends to z =
              BEC_CRADLE_FLOOR + BEC_BODY_H)

    The cradle's CHASSIS-frame placement is
    ``(ELEC_TRAY_CENTRE_X + BEC_CRADLE_CENTRE[0],
       ELEC_TRAY_CENTRE_Y + BEC_CRADLE_CENTRE[1],
       CHASSIS_PLATE_T / 2 + 3 + ELEC_TRAY_T)`` -- i.e. on the
    electronics_tray's top face, in the corridor between Mega +X
    edge (+22.75 mm) and PCA1 -X edge (+51.5 mm).
    """
    cavity_l = BEC_BODY_L
    cavity_w = 2.0 * BEC_BODY_W - BEC_CRADLE_INTERFERENCE
    cavity_h = BEC_BODY_H

    outer_l = cavity_l + 2.0 * BEC_CRADLE_WALL
    outer_w = cavity_w + 2.0 * BEC_CRADLE_WALL
    outer_h = BEC_CRADLE_FLOOR + cavity_h + BEC_CRADLE_LIP_H

    # Solid outer block.
    block = _box((outer_l, outer_w, outer_h),
                  center=(0.0, 0.0, outer_h / 2.0))

    # Subtract the main cavity (BEC body space).  The cavity opens
    # UP to z = floor + body_h; the retention lip lives ABOVE that
    # at z in [floor + body_h, floor + body_h + lip_h] but tapers
    # inboard by BEC_CRADLE_LIP_INSET on each +/- Y side.
    cavity = _box((cavity_l, cavity_w, cavity_h + 0.5),
                   center=(0.0, 0.0,
                            BEC_CRADLE_FLOOR + cavity_h / 2.0 + 0.25))

    # Upper insertion slot (above the lip): narrower in Y so the
    # BEC body has to flex past the retention lip to drop in.
    insert_slot = _box(
        (cavity_l,
         cavity_w - 2.0 * BEC_CRADLE_LIP_INSET,
         BEC_CRADLE_LIP_H + 0.5),
        center=(0.0, 0.0,
                 BEC_CRADLE_FLOOR + cavity_h
                 + BEC_CRADLE_LIP_H / 2.0 + 0.25),
    )

    # Wire-exit channels on +/- X end walls.  Centred vertically on
    # the BEC body (at z = floor + body_h/2).  One channel per BEC
    # would be ideal but a single wider opening that spans both
    # BECs' Y range is mechanically simpler and the user will fish
    # both pigtails through the same end.  Channel height = Phi 5
    # round; widened in Y to cover both BEC pigtail positions
    # (centres at y = +/- (cavity_w/4)).
    channels = []
    for sx in (-1, 1):
        # Single round Phi 5 channel per BEC -- 2 channels per end,
        # 4 total.
        for sy in (-1, 1):
            ch = _cyl_along(BEC_PIGTAIL_OD / 2.0,
                             BEC_CRADLE_WALL + 0.4,
                             axis="x")
            cx = sx * (cavity_l / 2.0 - 0.2)
            cy = sy * (cavity_w / 4.0)
            cz = BEC_CRADLE_FLOOR + cavity_h / 2.0
            ch.apply_translation([cx, cy, cz])
            channels.append(ch)

    return _diff(block, cavity, insert_slot, *channels)


def make_switch_holster() -> trimesh.Trimesh:
    """Printed holster for the anti-spark XT60 on/off switch.

    Mounts to chassis_top's +X edge between the L0 and L5
    coxa_brackets.  Two-part body:

      * SOCKET (+X half): a 5-walled open-top box that snugs the
        switch body in with SWITCH_BODY_CL mm clearance per axis.
        Toggle pokes out a SWITCH_TOGGLE_W x SWITCH_TOGGLE_H
        cutout in the +X end wall (= the chassis +X vertex); the
        2 XT60 pigtails exit out 2 x Phi SWITCH_PIGTAIL_OD = 6 mm
        holes in the -X end wall.
      * MOUNTING EAR (-X half): a flat SWITCH_EAR_L x outer_w x
        SWITCH_HOLSTER_FLOOR plate that sits on chassis_top's TOP
        face.  Two M3 brass heat-set inserts (McMaster 94459A130)
        are pressed into Phi INSERT_M3_PILOT_DEPTH = 6 mm pockets
        opening DOWNWARD at the ear's BOTTOM face -- 2 x M3 x 12
        SHCS thread UP from below chassis_top into them.  Bolt
        head bears on chassis_top's BOTTOM face; the ear's
        plastic clamps chassis_top from above.

    Local frame (mesh origin = MIDPOINT of the holster's X extent
    on chassis_top's TOP face):
        +X = toggle-exit direction (toggle pokes out +X face)
        +Y = tangential (along the chassis +X edge)
        +Z = UP (ear bottom at z = 0, socket cavity opens UP at z
              = SWITCH_HOLSTER_FLOOR)

    CHASSIS-frame placement: ``(SWITCH_HOLSTER_CENTRE_X,
    SWITCH_HOLSTER_CENTRE_Y, CHASSIS_GAP + CHASSIS_PLATE_T +
    CHASSIS_PLATE_T/2)``.  ``make_chassis_top`` adds matching 2 x
    Phi BRACKET_BOLT_HOLE = 3.4 mm clearance holes at
    SWITCH_HOLSTER_BOLT_CHASSIS_XY.
    """
    outer_l = SWITCH_HOLSTER_OUTER_L
    outer_w = SWITCH_HOLSTER_OUTER_W
    socket_l = SWITCH_SOCKET_OUTER_L

    # SOCKET solid block on the +X half.
    socket_centre_x = outer_l / 2.0 - socket_l / 2.0
    socket_outer_h = SWITCH_BODY_H + SWITCH_BODY_CL + SWITCH_HOLSTER_FLOOR
    socket_block = _box(
        (socket_l, outer_w, socket_outer_h),
        center=(socket_centre_x, 0.0, socket_outer_h / 2.0),
    )

    # EAR solid block on the -X half (flat).
    ear_centre_x = -outer_l / 2.0 + SWITCH_EAR_L / 2.0
    ear_block = _box(
        (SWITCH_EAR_L, outer_w, SWITCH_HOLSTER_FLOOR),
        center=(ear_centre_x, 0.0, SWITCH_HOLSTER_FLOOR / 2.0),
    )

    block = _union(socket_block, ear_block)

    # Switch body cavity (open top, inside socket block).
    cavity_l = SWITCH_BODY_L + 2.0 * SWITCH_BODY_CL
    cavity_w = SWITCH_BODY_W + 2.0 * SWITCH_BODY_CL
    cavity_h = SWITCH_BODY_H + SWITCH_BODY_CL
    cavity = _box(
        (cavity_l, cavity_w, cavity_h + 0.5),
        center=(socket_centre_x, 0.0,
                 SWITCH_HOLSTER_FLOOR + cavity_h / 2.0 + 0.25),
    )

    # Toggle cutout in +X end wall of the socket.
    toggle = _box(
        (SWITCH_HOLSTER_WALL + 0.4,
         SWITCH_TOGGLE_W,
         SWITCH_TOGGLE_H),
        center=(outer_l / 2.0 - (SWITCH_HOLSTER_WALL + 0.4) / 2.0 + 0.2,
                 0.0,
                 SWITCH_HOLSTER_FLOOR + SWITCH_BODY_H / 2.0),
    )

    # 2 XT60 pigtail exit channels through -X end wall of the
    # socket.  Phi 6 mm cylinders along X, centred vertically on the
    # body cavity (at z = floor + body_h/2 = 2.5 + 8.5 = 11 mm).
    pigtails = []
    socket_minus_x_face = socket_centre_x - socket_l / 2.0
    for sy in (-1, 1):
        ch = _cyl_along(SWITCH_PIGTAIL_OD / 2.0,
                         SWITCH_HOLSTER_WALL + 0.4,
                         axis="x")
        cx = socket_minus_x_face - 0.2
        cy = sy * SWITCH_PIGTAIL_DY
        cz = SWITCH_HOLSTER_FLOOR + SWITCH_BODY_H / 2.0
        ch.apply_translation([cx, cy, cz])
        pigtails.append(ch)

    # 2 M3 bolt clearance holes through the ear, top to bottom.
    # The bolt enters from ABOVE (head bears on the ear's top face),
    # passes DOWN through the Phi BRACKET_BOLT_HOLE = 3.4 mm
    # clearance hole, and threads into the brass heat-set insert
    # that lives in the matching chassis_top boss (see
    # ``make_chassis_top``).  Insert is in chassis_top so the
    # driver clearance probed by ``check_screwdriver_access``
    # extends UPWARD into open air, not down into the chassis
    # interior where the electronics_tray + battery_holder live.
    clearance_holes = []
    for (bx, by) in SWITCH_HOLSTER_BOLT_OFFSETS:
        hole = _cyl(BRACKET_BOLT_HOLE / 2.0, SWITCH_HOLSTER_FLOOR * 4.0)
        # Centre the cut vertically on the ear (ear z in [0, FLOOR]).
        hole.apply_translation([bx, by, SWITCH_HOLSTER_FLOOR / 2.0])
        clearance_holes.append(hole)

    return _diff(block, cavity, toggle, *pigtails, *clearance_holes)


def make_imu_pad() -> trimesh.Trimesh:
    """Printed vibration-isolated mounting pad for the MPU-6050 / GY-521
    breakout (May 2026 "essentials" pass: IMU promoted from optional to
    standard kit).

    The pad is a thin IMU_PAD_W x IMU_PAD_D x IMU_PAD_T plate with 4
    short Phi IMU_PAD_BOSS_OD bosses at the IMU's 15.0 x 11.0 mm hole
    pattern.  Each boss carries an M3 brass heat-set insert (McMaster
    94459A130) in a Phi INSERT_M3_PILOT_OD x INSERT_M3_PILOT_DEPTH
    pocket opening DOWNWARD from the boss top.  The IMU PCB sits on
    the 4 boss tops; 4 x M3 x 8 SHCS thread DOWN through the PCB
    into the inserts.

    There are NO fasteners between the pad and chassis_top.  The pad
    is glued to chassis_top with a 3 mm-thick strip of 3M VHB /
    generic double-sided mounting foam tape; the foam tape doubles
    as the mount AND the vibration damper that the MPU-6050's HF gyro
    noise floor requires in a servo-driven robot.  The pad's BOTTOM
    face is therefore deliberately FLAT and free of features so the
    foam tape adheres cleanly across its full IMU_PAD_W x IMU_PAD_D
    footprint.

    Local frame (mesh origin = pad floor centre on its BOTTOM face):
        +X = IMU long axis (PCB long edge, = the 8-pin header edge
              after the orientation in build_prototype_assembly)
        +Y = IMU short axis
        +Z = UP (pad floor at z in [0, IMU_PAD_T]; boss tops at
              z = IMU_PAD_T + IMU_PAD_BOSS_H)

    CHASSIS-frame placement (see IMU_PAD_CENTRE_X / Y / IMU_PAD_TAPE_T
    constants): ``(0, 0, chassis_top_top_z + IMU_PAD_TAPE_T)`` -- the
    chassis-frame centre of mass, so rotation rates from the gyro
    are not contaminated by linear-acceleration cross-coupling from
    the body swing.
    """
    # Pad floor.
    floor = _box((IMU_PAD_W, IMU_PAD_D, IMU_PAD_T),
                 center=(0.0, 0.0, IMU_PAD_T / 2.0))

    # 4 bosses + insert pockets at the IMU hole pattern.  Each boss is
    # a Phi IMU_PAD_BOSS_OD cylinder rising IMU_PAD_BOSS_H above the
    # pad's top face (= z = IMU_PAD_T).  The 6 mm insert pocket digs
    # DOWN from the boss top into the boss + pad floor, leaving 0.6
    # mm of plastic above the pad bottom face so the foam-tape side
    # of the pad stays FLAT.
    bosses = []
    pockets = []
    clearance_holes = []
    pocket_overdrill_h = INSERT_M3_PILOT_DEPTH + 0.4   # 0.4 mm slop
                                                       # at the OPEN
                                                       # end of the
                                                       # pocket so the
                                                       # CSG diff
                                                       # clears the
                                                       # boss top rim
    boss_top_z = IMU_PAD_T + IMU_PAD_BOSS_H            # = 7 mm
    pocket_centre_z = boss_top_z - INSERT_M3_PILOT_DEPTH / 2.0 + 0.2
    # ^ pocket centre = (boss_top - depth/2) + small overdrill shift
    #   so the pocket extends from
    #   z = boss_top_z + 0.2 (just above boss top, for clean diff)
    #   down to
    #   z = boss_top_z + 0.2 - pocket_overdrill_h
    #     = boss_top_z + 0.2 - (INSERT_M3_PILOT_DEPTH + 0.4)
    #     = boss_top_z - INSERT_M3_PILOT_DEPTH - 0.2
    # With boss_top_z = 7 and INSERT_M3_PILOT_DEPTH = 6 the pocket
    # bottom is at z = +0.8 mm -- 0.8 mm of pad floor remains below
    # the pocket so the bottom face stays flat for foam-tape adhesion.
    for (bx, by) in IMU_PAD_HOLE_OFFSETS:
        # Boss extends from z = 0 (= pad floor BOTTOM face, so it
        # stays UNIONED with the floor without dropping below it --
        # keeps the pad's BOTTOM face flat for foam-tape adhesion)
        # up to z = boss_top_z + 0.2 (0.2 mm above boss top for
        # clean diff with the insert pocket).
        boss_h_total = boss_top_z + 0.2
        boss = _cyl(IMU_PAD_BOSS_OD / 2.0, boss_h_total)
        boss.apply_translation([bx, by, boss_h_total / 2.0])
        bosses.append(boss)

        pocket = _cyl(INSERT_M3_PILOT_OD / 2.0, pocket_overdrill_h)
        pocket.apply_translation([bx, by, pocket_centre_z])
        pockets.append(pocket)

    # NOTE: the M3 clearance holes for the IMU's bolts are in the
    # IMU PCB itself, not in the pad.  The pad's bosses are SOLID
    # (apart from the insert pocket) -- the bolt threads into the
    # brass insert, not through the boss.  So no additional Phi 3.0
    # clearance cuts are made through the boss material.
    del clearance_holes  # placeholder; left intentionally unused

    body = _union(floor, *bosses)
    return _diff(body, *pockets)


def make_mpu6050_visual() -> trimesh.Trimesh:
    """Visual mesh for the MPU-6050 / GY-521 IMU breakout PCB.

    NOT FOR PRINTING -- this is the commodity blue/red breakout board
    that ships from Amazon; the visual is a simple IMU_PCB_W x
    IMU_PCB_D x IMU_PCB_T flat slab so the build inspector / render
    can show the IMU on top of the printed ``imu_pad``.  Same role
    as ``make_servo_body`` / ``make_servo_horn`` (visual-only meshes
    written to stl_prototype/ alongside the real printables).

    Mesh frame: origin at the PCB centre, +X along the long axis
    (along the 8-pin header edge), +Y along the short axis, +Z UP.
    """
    return _box((IMU_PCB_W, IMU_PCB_D, IMU_PCB_T),
                center=(0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Non-printed electronics visuals (May 2026 follow-up: BuildViz pass)
# ---------------------------------------------------------------------------
#
# These ``make_*_visual()`` functions follow the same convention as
# ``make_servo_body`` / ``make_servo_horn`` / ``make_mpu6050_visual``:
# visual-only meshes written to ``stl_prototype/`` alongside the real
# printables, but NOT added to the Xometry / Bambu-tray printable-
# parts pipelines (``prepare_xometry_upload.PART_REGISTRY``) -- there
# is nothing to print here, the user buys the actual hardware from
# Amazon / McMaster.
#
# Each function returns a mesh whose LOCAL ORIGIN is chosen so the
# inspector's ``_build_assembly_instances`` can translate by a single
# ``_trans(world_x, world_y, world_z)`` without having to apply
# additional local offsets:
#
#   * Bare PCB boards (Mega, Pi, PCA9685) -- origin = PCB midplane,
#     +X = long axis (chassis +X after placement), +Y = short axis,
#     +Z = up.  Connector lumps stick up above the PCB top face.
#   * Free-standing bodies (BEC, anti-spark switch, LiPo) -- origin
#     = body geometric centre.
#
# Triangle counts are kept small: each connector / pigtail is a
# single primitive (box or short cylinder).  These meshes never
# participate in the design verifier -- they are not in
# ``_MESH_BUILDERS`` (see _verify_prototype.py) and the inspector
# marks them as decoration.

# Standard PCB thickness used by the Mega, Pi, and PCA9685 commodity
# breakouts.  IMU_PCB_T (above) already documents the same value for
# the MPU-6050; redefining here so the visual makers don't reach into
# an "imu-only" constant for their generic PCB slab.
COMMODITY_PCB_T = 1.6   # mm


def make_arduino_mega_visual() -> trimesh.Trimesh:
    """Visual mesh for the Arduino Mega 2560 R3 (NOT FOR PRINTING).

    Models the bare PCB (MEGA_PCB_D x MEGA_PCB_W x COMMODITY_PCB_T)
    plus the two -X end connectors that the user wires into: a USB-B
    socket (~ 14 x 13 x 16 mm) and a DC barrel jack (~ Phi 9 x 14 mm,
    axis along the same edge as the USB-B).  The ATmega chip, headers,
    and pin sockets are omitted -- they read as part of the PCB slab
    once colored.

    Mesh frame: origin = PCB midplane.  +X = long axis (chassis +X
    after placement, matching MEGA_PCB_D being the chassis-X extent
    in ``_body_battery_parts``).  +Y = short axis.  +Z = up.
    """
    pcb = _box((MEGA_PCB_D, MEGA_PCB_W, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))

    pcb_top_z = COMMODITY_PCB_T / 2.0
    usb_b_h = 16.0
    usb_b_centre_x = -MEGA_PCB_D / 2.0 + 14.0 / 2.0 + 0.5
    usb_b = _box((14.0, 13.0, usb_b_h),
                 center=(usb_b_centre_x, -10.0,
                          pcb_top_z + usb_b_h / 2.0))

    barrel_d = 9.0
    barrel = _cyl_along(barrel_d / 2.0, 14.0, axis="x")
    barrel.apply_translation([-MEGA_PCB_D / 2.0 - 0.5,
                               +12.0,
                               pcb_top_z + barrel_d / 2.0])

    return _union(pcb, usb_b, barrel)


def make_raspberry_pi_visual() -> trimesh.Trimesh:
    """Visual mesh for the Raspberry Pi 4 / Pi 5 SBC (NOT FOR PRINTING).

    Models the bare PCB (PI_PCB_W x PI_PCB_D x COMMODITY_PCB_T) plus
    the two perpendicular connector edges the user wires into.  Real
    Pi 4 / Pi 5 layout per the published mechanical drawings:

      * -Y LONG edge (chassis -Y after placement; cables exit -Y
        past chassis_top's -Y apothem):
          - USB-A 3.0 PAIR (blue, 2 stacked) ~ 16 x 16 x 17 mm
          - USB-A 2.0 PAIR (black, 2 stacked) ~ 16 x 16 x 17 mm
          - Ethernet RJ45                     ~ 16 x 16 x 14 mm
      * -X SHORT edge (chassis -X after placement; cables exit -X
        in a separate corridor from the +X PCA9685 servo header
        bank):
          - USB-C power           ~ 8 x 12 x 8 mm
          - 2 x micro-HDMI        ~ 8 x 8 x 12 mm each
          - (3.5 mm audio jack omitted -- not used in this robot)
      * +Y LONG edge: 40-pin GPIO header (no jumper cables modelled).
      * +X SHORT edge: micro-SD slot (user reaches in to swap; no
        plug, no keepout needed -- the card sits low on the PCB).

    Mesh frame: origin = PCB midplane.  +X = long axis (PI_PCB_W,
    chassis +X after placement).  +Y = short axis.  +Z = up.

    Pre-May-2026 the visual mesh had USB-C/HDMI on the +Y LONG edge
    and USB-A/Ethernet on the +X SHORT edge -- physically impossible
    for the real Pi (USB-A x2 + Ethernet would overflow the 56 mm
    short edge).  The May 2026 "Pi cantilever" pass corrected the
    bank-to-edge assignment AND fixed the Ethernet RJ45 jack
    out-of-plane height from 25 mm to 14 mm (the 25 mm figure
    confused the in-plane plug depth with the actual ~ 13.5 mm-tall
    metal jack body + ~ 1-2 mm metal EMI shield on top).
    """
    pcb = _box((PI_PCB_W, PI_PCB_D, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))

    pcb_top_z = COMMODITY_PCB_T / 2.0
    half_x = PI_PCB_W / 2.0
    half_y = PI_PCB_D / 2.0

    # USB-C power on -X SHORT edge.  Body sticks 8 mm in -X off the
    # PCB edge; centre at PCB-local y = +half_y - 11 mm (11 mm from
    # the +Y corner along the short axis, matching real Pi 4 USB-C
    # position).
    usb_c_h = 8.0
    usb_c = _box((8.0, 12.0, usb_c_h),
                 center=(-half_x - 4.0,
                          half_y - 11.0,
                          pcb_top_z + usb_c_h / 2.0))

    # Micro-HDMI 0 and 1 on -X SHORT edge.  Same body-depth-in-X
    # convention as USB-C.
    hdmi_h = 12.0
    hdmi_a = _box((8.0, 8.0, hdmi_h),
                   center=(-half_x - 4.0,
                            half_y - 15.0,
                            pcb_top_z + hdmi_h / 2.0))
    hdmi_b = _box((8.0, 8.0, hdmi_h),
                   center=(-half_x - 4.0,
                            0.0,
                            pcb_top_z + hdmi_h / 2.0))

    # USB-A pairs (16 x 16 x 17, 2 receptacles stacked vertically)
    # on the -Y LONG edge.  Body sticks 16 mm in -Y off the PCB
    # edge.  Positions matching real Pi 4 mech drawing: USB 3.0
    # PAIR (blue) at PCB-local x = -3.5 mm, USB 2.0 PAIR (black) at
    # PCB-local x = +13.5 mm.
    usb_a_h = 17.0
    usb_a_3 = _box((16.0, 16.0, usb_a_h),
                    center=(-3.5,
                             -half_y - 8.0,
                             pcb_top_z + usb_a_h / 2.0))
    usb_a_2 = _box((16.0, 16.0, usb_a_h),
                    center=(+13.5,
                             -half_y - 8.0,
                             pcb_top_z + usb_a_h / 2.0))

    # Ethernet RJ45 on -Y LONG edge at PCB-local x = +30 mm.  Body
    # depth 16 mm in -Y; height in Z = 14 mm (~ 13.5 mm jack body
    # + ~ 1-2 mm metal EMI shield, rounded up to 14).
    eth_h = 14.0
    eth = _box((16.0, 16.0, eth_h),
                center=(+30.0,
                         -half_y - 8.0,
                         pcb_top_z + eth_h / 2.0))

    return _union(pcb, usb_c, hdmi_a, hdmi_b, usb_a_3, usb_a_2, eth)


def make_pca9685_visual() -> trimesh.Trimesh:
    """Visual mesh for an Adafruit PCA9685 16-channel PWM driver
    (NOT FOR PRINTING).

    Models the bare PCB (PCA_PCB_D x PCA_PCB_W x COMMODITY_PCB_T --
    short axis along chassis X, long axis along chassis Y to match
    ``_body_battery_parts``'s placement rotation) plus the 3-pin
    servo header row along the +X long edge (modelled as one
    8 x 60 x 8 mm bar so the user sees where the servo jumpers plug
    in).  The I2C terminal block and STEMMA QT connector are omitted.

    Mesh frame: origin = PCB midplane.  +X = chassis short axis.
    +Y = chassis long axis.  +Z = up.

    Same STL is used for both PCA9685(0x40) and PCA9685(0x41) -- the
    address jumper is not visually distinguishable.
    """
    pcb = _box((PCA_PCB_D, PCA_PCB_W, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))

    pcb_top_z = COMMODITY_PCB_T / 2.0
    header_h = 8.0
    header = _box((8.0, 60.0, header_h),
                   center=(PCA_PCB_D / 2.0 + 4.0,
                            0.0,
                            pcb_top_z + header_h / 2.0))

    return _union(pcb, header)


def make_bec_visual() -> trimesh.Trimesh:
    """Visual mesh for one 5V switching BEC body (NOT FOR PRINTING).

    Shrink-wrapped BEC_BODY_L x BEC_BODY_W x BEC_BODY_H body plus 2
    short Phi 3 mm pigtail stubs exiting the +/-X ends (the input
    XT60 lead at one end, the 3-pin servo-header output at the other;
    both modelled as plain cylinders, no connectors).

    Mesh frame: origin = body geometric centre.  +X = pigtail axis
    (matches the bec_cradle's local +X = pigtail axis).  +Y = wider
    axis.  +Z = up.

    Same STL is used for both ``bec_a`` and ``bec_b`` -- the cradle
    holds 2 BECs side-by-side along the cradle's +Y axis.
    """
    body = _box((BEC_BODY_L, BEC_BODY_W, BEC_BODY_H),
                center=(0.0, 0.0, 0.0))

    pigtail_r = 1.5
    pigtail_len = 8.0
    p_plus = _cyl_along(pigtail_r, pigtail_len, axis="x")
    p_plus.apply_translation([+BEC_BODY_L / 2.0, 0.0, 0.0])
    p_minus = _cyl_along(pigtail_r, pigtail_len, axis="x")
    p_minus.apply_translation([-BEC_BODY_L / 2.0 - pigtail_len, 0.0, 0.0])

    return _union(body, p_plus, p_minus)


def make_antispark_switch_body_visual() -> trimesh.Trimesh:
    """Visual mesh for the anti-spark XT60 switch BODY (NOT FOR PRINTING).

    SWITCH_BODY_L x _W x _H switch body (~ 32 x 17 x 17 mm) plus two
    short Phi 4 mm XT60 pigtail stubs exiting the +/-Y end faces.
    The toggle lever is a SEPARATE mesh
    (``make_antispark_switch_toggle_visual``) so the inspector can
    paint it a distinct safety-orange against the body's anodized
    grey.

    Mesh frame: origin = body geometric centre, with +X = toggle
    axis (chassis +X after placement, matching the switch_holster's
    local +X = toggle face), +Y = pigtail axis, +Z = up.
    """
    body = _box((SWITCH_BODY_L, SWITCH_BODY_W, SWITCH_BODY_H),
                center=(0.0, 0.0, 0.0))

    pigtail_r = 2.0
    pigtail_len = 10.0
    p_plus_y = _cyl_along(pigtail_r, pigtail_len, axis="y")
    p_plus_y.apply_translation([0.0, +SWITCH_BODY_W / 2.0, 0.0])
    p_minus_y = _cyl_along(pigtail_r, pigtail_len, axis="y")
    p_minus_y.apply_translation([0.0,
                                  -SWITCH_BODY_W / 2.0 - pigtail_len,
                                  0.0])

    return _union(body, p_plus_y, p_minus_y)


def make_antispark_switch_toggle_visual() -> trimesh.Trimesh:
    """Visual mesh for the anti-spark switch TOGGLE lever (NOT FOR
    PRINTING).

    Phi 4 x 8 mm cylinder protruding from the switch body's +X face.
    Mesh frame: origin at the TOGGLE's centre (the cylinder spans -4
    to +4 in its long axis).  +X is the protrusion direction.

    Drawn as a separate mesh so the inspector can color it safety-
    orange against the body's anodized grey.
    """
    toggle = _cyl_along(2.0, 8.0, axis="x")
    toggle.apply_translation([-4.0, 0.0, 0.0])
    return toggle


def make_lipo_battery_body_visual() -> trimesh.Trimesh:
    """Visual mesh for the 3S 2200 mAh LiPo pack BODY (NOT FOR PRINTING).

    Shrink-wrap slab (~ 105 x 35 x 25 mm).  The XT60 connector +
    balance plug are a SEPARATE mesh (``make_lipo_xt60_visual``) so
    the inspector can paint the XT60 housing safety-yellow against
    the body's "lipo red".

    Mesh frame: origin = body geometric centre.  +X = long axis
    (chassis +X after placement -- BATTERY_HOLDER_CENTRE_X = -25
    centres the pack near the chassis -X half).  +Y = short axis.
    +Z = up.
    """
    return _box((105.0, 35.0, 25.0),
                center=(0.0, 0.0, 0.0))


def make_lipo_xt60_visual() -> trimesh.Trimesh:
    """Visual mesh for the LiPo's XT60 connector + balance plug
    (NOT FOR PRINTING).

    XT60 housing (~ 14 x 17 x 10 mm) plus the smaller balance-plug
    ribbon header (~ 12 x 8 x 4 mm) sitting adjacent.  Both rendered
    safety-yellow.

    Mesh frame: origin at the XT60 housing centre.  +X is the
    direction the connector points (out of the battery body's +X
    short face after placement).
    """
    xt60 = _box((14.0, 17.0, 10.0),
                 center=(0.0, 0.0, 0.0))
    balance = _box((12.0, 8.0, 4.0),
                    center=(0.0, 17.0 / 2.0 + 8.0 / 2.0 + 0.5, -3.0))
    return _union(xt60, balance)


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
        - YAW WELL WALL DEPTH: the well's vertical walls extend only
          down to ``BRACKET_WELL_TRIM_Z = -15 mm`` (the bracket's
          lowest point).  There is NO well floor; the well bottom is
          open.  The lower ~12 mm of the seated DS3225 body (body
          bottom at bracket-z = -27) hangs EXPOSED below the trim
          plane.  This is a deliberate simplification (May 2026):
          the wall material below the bracket-level insert bosses
          (boss bottom at bracket-z = -13) has no structural job --
          the four mounting tabs already hold the servo up at the
          rim -- so trimming it off saves ~15 mm of vertical extent
          plus the floor plate without affecting servo seating,
          fastener engagement, or the +X wire-boot insertion path
          (the boot-drop channel survives above the trim).

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

    # ---- Printed-in zip-tie strain-relief post (Part A, May 2026) ---
    # Small nub on the well's +X outer wall just past the wire-exit
    # slot.  See ``_cable_zip_post`` + ``CABLE_POST_*`` constants for
    # the geometry rationale.  Applied with the SAME translation as
    # the wire_slot so the post sits next to the slot exit.
    cable_post = _cable_zip_post()
    cable_post.apply_translation([body_centre_x, 0.0, well_dz])

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

    # Body+tab DROP-IN slot.  Through-hole spanning the flange's full
    # height PLUS just enough material below the flange to clear the
    # stiffening rib (the rib's top sits at bracket-z = +3 and its
    # bottom at bracket-z = -3; if the slot doesn't cut at least to
    # bracket-z = -3 then rib material at z in [-3, 0] survives in
    # the centre of the body+tab drop-in path and physically blocks
    # the servo from descending into the cradle).  Slot Z range:
    # z = BRACKET_SLOT_Z_MIN_RIB_CLEAR (= rib bottom at z = -3) up to
    # z = BRACKET_FLANGE_T + 0.5 (so the cut comfortably exits the top
    # face).  The flange becomes a closed RING around the slot: a 5 mm
    # perimeter strip at each X-end (slot 56 mm wide vs flange 66 mm),
    # and a 15.5 mm perimeter strip at each Y-end (slot 21 mm wide vs
    # flange 52 mm).  All four chassis-bolt corners are tied together
    # through that ring.
    #
    # May 2026 Option 5 fix: the previous slot_z_min = -6 was an excess
    # 3 mm of wall-material bite below the rib's bottom (the slot only
    # NEEDS to reach the rib bottom at z = -3 to clear the drop-in
    # path).  That excess lowered the EFFECTIVE shelf top at the
    # bracket's bolt sites from well-local +27.25 to +21.25 -- a 6 mm
    # deficit relative to the coxa_link / femur cradles, where the
    # vertical self-tap pilots assume the full WELL_RIM_Z shelf.
    # Raising slot_z_min to -3 reclaims 3 of those 6 mm; the residual
    # 3 mm deficit (well-local +24.25 vs +27.25) is absorbed by
    # passing ``shelf_top_z = WELL_RIM_Z - BRACKET_SHELF_DROP_MM`` to
    # ``_servo_cradle_insert_pockets`` at the bracket level so the
    # bracket's heat-set bosses + pockets stay aligned with the
    # bracket's own effective shelf top.
    BRACKET_SLOT_Z_MIN_RIB_CLEAR = -3.0
    slot_z_min = BRACKET_SLOT_Z_MIN_RIB_CLEAR
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
    # body+tab cutout (WELL_D + 2 = 31 mm) because the rib extends
    # DOWN past z = 0 into the chassis-plate's Z volume (z = [-4, 0]).
    # The earlier 48 mm-wide rib clashed with solid chassis material
    # at y in [+/-15.5, +/-24] and stopped the bracket from seating
    # flush on the chassis plate.
    RIB_X = 36.0                                  # 18 mm each side of yaw axis
    RIB_Y = WELL_D + 1.0                          # 30 mm == ±15, fits in 31 mm cutout
    rib = _box((RIB_X, RIB_Y, 6.0),
               center=(0.0, 0.0, 0.0))            # x in ±18, y in ±13, z in ±3

    # Side gussets riding directly on top of the well's +Y / -Y walls.
    # The body+tab pocket eats the rib at y in [-10.5, +10.5] and now
    # reaches DOWN to z = -3 (the rib's bottom; see the May 2026
    # Option 5 fix above) -- below that the well wall is uncut.  The
    # only flange-to-well load paths at the well's +Y / -Y wall tops
    # are these two narrow column strips above the well wall (which
    # lives at y in [10.5, 12.5] and y in [-12.5, -10.5]).  Each
    # gusset is bounded in Y to the well's wall footprint so nothing
    # dangles in mid-air past the well's outer Y faces.  Z range =
    # [-6, BRACKET_FLANGE_T], so the gussets merge with the (15 mm
    # tall) flange material above z = 0 into one solid slab; the
    # structurally important contribution is the 6 mm of material
    # BELOW z = 0 that ties the flange into the well rim.
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
    bridge_gusset_y_ext = WELL_D                          # 29, fits in cutout (31)
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
    # class servos reaches PLASTIC_HORN_X_TIP_R = 18 mm to each arm
    # tip, so when the servo rotates the horn sweeps a Phi 36 mm
    # cylinder centred on the yaw axis (bracket-x = 0, bracket-y = 0).
    # The drop-in body+tab slot is only +/-10.5 mm wide in Y, so any
    # flange material at y in [+/-10.5, +/-(WELL_D/2 + extra)] above
    # the body's top face sits IN the horn's sweep and physically
    # blocks the horn from rotating (or from being installed on the
    # spline above the bracket at all).
    #
    # Design B (May 2026): with the printed servo_horn_adapter retired,
    # the horn stack above the body now ENDS at the top of the plastic
    # horn (z = SERVO_BODY_H + SERVO_OUTPUT_H + PLASTIC_HORN_H -
    # WELL_RIM_Z = +20.75 mm) rather than the top of the printed
    # adapter (was +25.75 mm).  The coxa_link's pedestal bottom mating
    # face now sits at THAT plane (+20.75 mm) instead of +25.75 mm,
    # 4 mm closer to the bracket flange.  We still need the flange to
    # be CLEAR of the horn's swept silhouette across that whole Z
    # range, so the sweep void extends from just above the body's top
    # face (z = +10.0) UP through the flange's top face plus a 2 mm
    # overshoot.  The flange top face is at z = BRACKET_FLANGE_T =
    # +15 mm, comfortably below +20.75 mm where the horn arm plane
    # actually lives -- so the existing sweep_z_hi = BRACKET_FLANGE_T
    # + 2 = +17 mm continues to be the right top of the void (we just
    # need to clear the flange volume; the horn arms above the flange
    # are in free air).
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
                    + 0.75)                              # 18.75 mm
    sweep_z_lo = SERVO_BODY_H - WELL_RIM_Z - 0.75        # +10.0 mm
    sweep_z_hi = BRACKET_FLANGE_T + 2.0                  # +17.0 mm
    sweep_z_ext = sweep_z_hi - sweep_z_lo
    horn_sweep_void = _cyl(horn_sweep_r, sweep_z_ext)
    horn_sweep_void.apply_translation([0.0, 0.0,
                                        0.5 * (sweep_z_lo + sweep_z_hi)])

    # Design D (May 2026 heat-set switch): re-apply the cradle's insert
    # bosses + pockets at the BRACKET level too.  Two reasons:
    #
    #   (a) The bracket's drop-in slot eats the well's rim down to
    #       bracket-z = -3 (= well-local +24.25), so the effective
    #       shelf top at the bolt sites is 3 mm LOWER than the well's
    #       nominal WELL_RIM_Z.  The bracket-level bosses + pockets
    #       use ``shelf_top_z = WELL_RIM_Z - BRACKET_SHELF_DROP_MM``
    #       so they land on the bracket's effective shelf top instead
    #       of the (cut-away) well rim plane.
    #   (b) ``_servo_well_solid`` already emits bosses + pockets at the
    #       NOMINAL WELL_RIM_Z; the bracket needs an extra pair at the
    #       depressed-shelf plane so the boss extends DOWN to the
    #       bracket's effective rim and the pocket lines up with the
    #       lowered shelf.
    #
    # The bracket-level boss is UNIONED into the bracket body BEFORE
    # the boolean subtractions so the boss material wraps the pocket
    # at the bracket's effective rim.  The well-level boss (added in
    # ``_servo_well_solid``) is already part of ``well`` and merges
    # naturally with this extra boss in the union below.
    BRACKET_SHELF_DROP_MM = -BRACKET_SLOT_Z_MIN_RIB_CLEAR   # = 3.0
    bracket_insert_bosses, bracket_insert_pockets = _servo_cradle_insert_pockets(
        shelf_top_z=WELL_RIM_Z - BRACKET_SHELF_DROP_MM,
    )
    bracket_insert_bosses = bracket_insert_bosses.copy()
    bracket_insert_pockets = bracket_insert_pockets.copy()
    bracket_insert_bosses.apply_translation([body_centre_x, 0.0, well_dz])
    bracket_insert_pockets.apply_translation([body_centre_x, 0.0, well_dz])

    # ---- Yaw-well TRIM block (May 2026 simplification) --------------
    # Remove all bracket material below ``BRACKET_WELL_TRIM_Z``.  The
    # trim box is a large slab whose TOP face sits at the trim plane
    # and whose interior extends well below any feature in the part
    # (well outer floor was at bracket-z = -30.25, the lowest existing
    # feature).  Anchored on the yaw axis in X / Y so the slab fully
    # covers the flange's 66 x 52 mm footprint plus all surrounding
    # walls.  This is the LAST cut in the boolean tree so the trim
    # also removes the matching lower portion of the L-shaped
    # ``wire_slot`` cut (whose lower leg used to vent through the
    # well floor): with the floor gone the wires simply dangle out
    # the open bottom, so the lateral leg of the L is no longer
    # required for harness escape.  The bracket's lowest point goes
    # from bracket-z = -30.25 down to bracket-z = -15.
    trim_box_z_extent = 100.0   # mm; chosen so the slab bottom (=
                                # bracket-z = -65) sits well below
                                # any conceivable bracket feature.
    bracket_well_trim = _box(
        (300.0, 300.0, trim_box_z_extent),
        center=(0.0, 0.0,
                 BRACKET_WELL_TRIM_Z - trim_box_z_extent / 2.0),
    )

    body = _union(flange, well, rib, *side_gussets, *bridge_gussets,
                   bracket_insert_bosses, cable_post)
    return _diff(body, slot, wire_slot, horn_sweep_void,
                 *chassis_holes, bracket_insert_pockets,
                 bracket_well_trim)


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
    cable_post = _cable_zip_post()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])  # well +Z -> link +Y
    well.apply_transform(R)
    wire_slot.apply_transform(R)
    cable_post.apply_transform(R)
    # Output spline tip in well-local: (SERVO_OUTPUT_X, 0,
    #   SERVO_BODY_H + SERVO_OUTPUT_H) = (10, 0, 44).
    # After R: (10, 44, 0).  We want it at (COXA_LENGTH, 0, 0) (the
    # joint axis position in the link frame, on the arm centreline).
    delta = np.array([COXA_LENGTH - SERVO_OUTPUT_X,
                       -(SERVO_BODY_H + SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)
    wire_slot.apply_translation(delta)
    cable_post.apply_translation(delta)
    # Drop the well in -Z so it hangs below the arm rather than
    # interpenetrating it.  WELL_Z_DROP_EXTRA pushes the well an extra
    # 4 mm down (PAST the natural arm-bottom = well-top plane) so the
    # femur hip pad's swept +Z edge clears the arm's bottom face -- see
    # the COXA_LIFT / WELL_Z_DROP_EXTRA docstrings near the top of this
    # file for the full geometry.
    well_z_drop = -(WELL_D / 2.0 + arm_t / 2.0 + WELL_Z_DROP_EXTRA)
    well.apply_translation([0.0, 0.0, well_z_drop])
    wire_slot.apply_translation([0.0, 0.0, well_z_drop])
    cable_post.apply_translation([0.0, 0.0, well_z_drop])

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
    # Cable post (Part A, May 2026): printed-in zip-tie strain relief
    # next to the hip-pitch wire-exit slot.  Built in well-local and
    # transformed alongside ``wire_slot`` so it stays anchored to the
    # well's +X outer wall in every part orientation.
    body_unlifted = _union(hub, arm, well, gusset, bridge, gusset_under,
                            arm_cap_pos, arm_cap_neg, well_top_pad,
                            cable_post)
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
    # Raise the trough floor to PEDESTAL_CAP_T (= 4 mm) so a solid
    # bottom cap covers the full 34 x 34 mm pedestal footprint at
    # z in [0, PEDESTAL_CAP_T].  Pre-2026-05 the trough was rooted at
    # z = 0 -- that sliced the ENTIRE bottom 25.5 mm out of the
    # pedestal and left only a thin "cap" at z in [25.5, 36], with NO
    # material in the central region where the 4 M2 X-horn bolts at
    # PCD 20.8 mm have to seat.  body_bot_z (= well_z_drop +
    # COXA_LIFT - SERVO_BODY_D/2 = 4.5) is the bottom of the hip-pitch
    # servo body's +Y face; we need trough_z_min <= body_bot_z - 0.5
    # for the body to still drop in cleanly, so the new floor at 4 mm
    # leaves 0.5 mm of clearance below the body and preserves a 4 mm
    # thick cap above the X-horn.
    trough_z_min = PEDESTAL_CAP_T
    trough_z_ext = trough_z_max - trough_z_min
    trough_z_cen = (trough_z_min + trough_z_max) / 2.0
    trough = _box((trough_x_ext, 34.0, trough_z_ext),
                  center=(trough_x_cen, 0.0, trough_z_cen))

    # ---- M2 X-horn bolt clearance through the bottom cap -------------
    # 4 x M2 self-tap clearance holes (XHORN_BOLT_OD = 2.2 mm) drilled
    # through the 4 mm bottom cap so the M2 SHCS shank can pass
    # through the printed cap material into the plastic X-horn's
    # Phi ~ 2.0 mm arm hole below.
    #
    # Each bolt gets a STEPPED cut:
    #
    #    z in [0, PEDESTAL_CAP_T - COUNTERBORE_DEPTH]  Phi 2.2 mm shaft
    #         = [0, 1.5]                                clearance
    #
    #    z in [PEDESTAL_CAP_T - COUNTERBORE_DEPTH,     Phi 4 mm head
    #          PEDESTAL_CAP_T] = [1.5, 4.0]            counter-bore
    #
    # so the M2 SHCS head (Phi ~3.8 mm + 0.2 mm tolerance, 2 mm tall)
    # recesses fully into the cap with its TOP face flush at z = 4 mm
    # (just below the trough floor).  A hex key reaches the head from
    # +Y through the existing body-insertion trough above the cap.
    # The shaft hole is intentionally restricted to the cap region:
    # above z = PEDESTAL_CAP_T the trough already supplies a clear
    # vertical path so drilling further is unnecessary.
    cap_holes = []
    counterbore_holes = []
    shaft_h_extent = PEDESTAL_CAP_T + 0.2  # 0.1 mm overshoot top + bottom
    counterbore_h_extent = COUNTERBORE_DEPTH + 0.05
    counterbore_z_centre = PEDESTAL_CAP_T - counterbore_h_extent / 2.0
    for a in XHORN_BOLT_ANGLES_RAD:
        h = _cyl(XHORN_BOLT_OD / 2.0, shaft_h_extent)
        h.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              XHORN_BOLT_PCD / 2.0 * np.sin(a),
                              shaft_h_extent / 2.0 - 0.1])
        cap_holes.append(h)

        cb = _cyl(M2_HEAD_OD_CLEARANCE / 2.0, counterbore_h_extent)
        cb.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              XHORN_BOLT_PCD / 2.0 * np.sin(a),
                              counterbore_z_centre])
        counterbore_holes.append(cb)

    # Central M3 horn-screw clearance.  Stays as a through-cut all the
    # way up the stack so the M2.5 spline screw head (captive under
    # the link's bottom recess) is fully accommodated and assembly
    # tooling has visual line-of-sight along the joint axis.
    bolt_total_h = COXA_LIFT + hub_t
    centre_hole = _cyl(HORN_CENTRE_OD / 2.0, bolt_total_h * 4)
    centre_hole.apply_translation([0, 0, bolt_total_h / 2.0])

    # ---- Central horn-hub recess in the pedestal's BOTTOM face --------
    # Design B (May 2026): the link now bolts DIRECTLY to the plastic
    # 4-arm X-horn (no printed servo_horn_adapter in the stack).  The
    # plastic horn's central spline-screw head protrudes 1 mm above the
    # horn arm plane (user-measured May 2026 -- the centre screw itself
    # remains M3, see HORN_CENTRE_OD; only the 4 outer arm bolts on
    # XHORN_BOLT_PCD are M2 self-tap), so we cut a Phi HORN_RECESS_OD =
    # 16 mm cylindrical pocket HORN_RECESS_DEPTH = 1.2 mm (= 1.0 mm
    # protrusion + 0.2 mm FDM tolerance) deep into the pedestal's
    # BOTTOM mating face (z = 0).
    # Recess opens DOWN (in -Z, toward the seated horn) and removes
    # pedestal material at z in [0, +HORN_RECESS_DEPTH].  Without this
    # recess the link's flat bottom face hits the horn's hub before the
    # 4 M2 clamp bolts can pull the link onto the horn.
    horn_hub_recess = _cyl(HORN_RECESS_OD / 2.0, HORN_RECESS_DEPTH)
    horn_hub_recess.apply_translation([0.0, 0.0,
                                        HORN_RECESS_DEPTH / 2.0])

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
    # clearance).  Y extent originally spanned the entire pedestal +
    # hub stack (link y in [-17, +17]) so the cut punches cleanly
    # through both.
    #
    # Heat-set boss carve-out (May 2026): the cut's -Y end at link
    # y = -18 used to reach 1.25 mm past the well rim plane (which
    # sits at link y = WELL_RIM_Z + delta_y = -16.75), and would
    # therefore eat the top 1.25 mm of each +X heat-set boss (boss
    # top at link y = -16.75, boss bottom at link y =
    # -16.75 - CRADLE_BOSS_HEIGHT_MM = -26.75).  Since the femur's
    # actual hip pad only sweeps at coxa-link y in [+9, +15] (the
    # pad sits at the OUTPUT side of the hip-pitch joint), the cut's
    # -Y reach can be safely trimmed back to the well rim plane plus
    # a 0.5 mm clearance margin.  This leaves the boss tops fully
    # intact while preserving every mm of pad-sweep clearance the
    # original cut provided.
    pad_sweep_y_min = (WELL_RIM_Z + delta[1]) + 0.5            # = -16.25
    pad_sweep_y_max = +18.0                                    # unchanged
    pad_sweep_y_extent = pad_sweep_y_max - pad_sweep_y_min     # = 34.25
    pad_sweep_y_centre = 0.5 * (pad_sweep_y_min + pad_sweep_y_max)
    pad_sweep_clear = _cyl(HIP_PAD_R + 0.5, pad_sweep_y_extent)
    pad_sweep_clear.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    hip_axis_z_in_link = well_z_drop + COXA_LIFT
    pad_sweep_clear.apply_translation([COXA_LENGTH,
                                        pad_sweep_y_centre,
                                        hip_axis_z_in_link])

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
                 horn_hub_recess, *cap_holes, *counterbore_holes,
                 centre_hole)


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

    Knee cradle floor: OPEN (May 2026 supports-free refactor)
    ---------------------------------------------------------
    The knee cradle (``_servo_well_solid`` rotated to point its mouth
    in femur +Y) is built with ``remove_floor=True`` so the floor
    frame at well-local z in [-WELL_FLOOR_T, 0] is cut out entirely.
    Why: in the "spar broad face on bed, mouth-down" print orientation
    used by the slicer, the well's mouth (femur +Y) faces DOWN onto
    the build plate and the well's floor faces UP -- a ~30 x 40 mm
    closed slab becomes a BRIDGED CEILING printed in midair.  The
    floor was purely a cosmetic seal: the servo body's load path is
    the 4 side walls (X / Z bounds), the tab shelf at z=WELL_RIM_Z
    that the servo's mounting tabs rest on, and the 4 boss columns
    that anchor the cradle bolt sites.  None of those depend on the
    floor.  After the cut the cradle is a 4-wall "pen" open on BOTH
    Y faces (mouth +Y and former-floor -Y); the body's -Y face is
    exposed to free air pointing AWAY from the chassis in the
    assembled robot.  No equivalent change for the coxa_bracket yaw
    cradle or the coxa_link hip cradle -- those print in different
    orientations where their floors are NOT bridges.
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

    # ---- Hip-end pad + neck annulus ---------------------------------
    # The femur's hip pad bolts directly onto the plastic 4-arm X-horn
    # that sits on the hip-pitch servo's spline.  In femur-local coords
    # the joint AXIS (spline tip) is at y = 0 and the X-horn's top arm
    # plane (where the pad clamps) is at y = HORN_STACK_H = 5 mm.
    # Placing the pad at y = 0 would put it inside the cradle's "swept
    # volume" above the body and caused the femur vs coxa_bracket /
    # femur vs tibia self-collision failures historically.
    #
    # Current design (May 2026 pad <-> spar fuse refactor -- Option A
    # from the fuse-area audit): a DEEP printable annulus neck that
    # restores the pre-7e2ca87 volumetric fuse with the spar:
    #
    #   1. HIP PAD: a solid disc at y in [+5, +11], radius HIP_PAD_R =
    #      20.0 mm.  The bolt-clamp ring; sits flat on the X-horn arm
    #      tops at y = +5 and carries the 4 M2 SHCS clamp bolts on
    #      XHORN_BOLT_PCD = 20.8 mm.
    #   2. NECK ANNULUS: an 8 mm-tall annulus at y in
    #      [-LINK_THICKNESS/2, HORN_STACK_H] = [-3, +5], with outer
    #      radius HIP_PAD_R = 20.0 mm (Phi 40 mm OD) and inner radius
    #      HORN_STACK_VOID_R = PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm
    #      (Phi 37 mm ID).  Ring wall thickness: 1.5 mm (~3 perimeters
    #      at 0.4 mm nozzle / 0.45 mm line width -- prints solid).
    #
    #      The neck's outer cylinder is FUSED with the spar over the
    #      entire spar Y range (y in [-3, +3], 6 mm of volumetric
    #      fusion), not just at the 0.5 mm "boolean kiss" that the
    #      May 2026 shorten-neck refactor (7e2ca87) left behind.  The
    #      pad <-> spar fuse cross-section in disk R=20.5 around the
    #      joint axis is now ~180 mm^2 sustained over y in [-3, +5]
    #      (8 mm range), up from ~106 mm^2 over y in [+3, +5] (2 mm)
    #      in the 7e2ca87 flange-ring design.  Critically the TIBIA
    #      benefits more than the femur: TIBIA_SPAR_H = 18 mm vs
    #      FEMUR_SPAR_H = 34 mm, so the tibia had ~half the spar
    #      cross-section to fuse with -- the failure mode the user
    #      reported as "the tibia knee-pad connection looks like it
    #      will break in service".
    #
    #      Cup ID stays at Phi 37 mm = PLASTIC_HORN_X_TIP_R*2 + 1 mm
    #      so the Phi 36 mm plastic X-horn still slides up into the
    #      annulus with 0.5 mm radial clearance per side at the arm
    #      tips.  Check_horn_stack_clearance probes this void.
    #
    #   3. HORN_STACK_VOID (radius HORN_STACK_VOID_R = 18.5 mm,
    #      y in [-LINK_THICKNESS/2 - 1, HORN_STACK_H + 1] = [-4, +6]):
    #      a unified clearance void cut as the LAST step in the part's
    #      CSG difference.  Hollows
    #          (a) the neck's interior (y in [-3, +5] at radius < 18.5),
    #          (b) the spar's hip-end material (y in [-3, +3] at
    #              radius < 18.5) so the X-horn (Phi 36 mm) can rotate
    #              freely inside the annulus at any hip-pitch angle, and
    #          (c) the pad's mating face (y in [+5, +6] at radius
    #              < 18.5).  The 1 mm +Y overshoot guarantees no FDM
    #              sliver of plastic sits in the void at the pad
    #              boundary -- carries the same 1 mm bolt-clamp gap
    #              the previous deep-cup design had (within the
    #              ``check_mating_face_contact`` 1.5 mm tolerance).
    #
    #      The void volume is checked by ``check_horn_stack_clearance``
    #      in _verify_prototype.py with probe radius HORN_STACK_VOID_R
    #      (kept in sync via the module-level constant).
    hip_pad_centre_y = HORN_STACK_H + LINK_THICKNESS / 2.0    # +8
    hip_pad_y_min    = HORN_STACK_H                            # +5
    hip_pad_y_max    = HORN_STACK_H + LINK_THICKNESS           # +11
    hip_pad = _cyl_along(HIP_PAD_R,
                          hip_pad_y_max - hip_pad_y_min,
                          axis="y")
    hip_pad.apply_translation([0, hip_pad_y_min, 0])

    # 8 mm-tall neck annulus.  Spans y in [-LINK_THICKNESS/2,
    # HORN_STACK_H] = [-3, +5]: the lower 6 mm (y in [-3, +3]) is a
    # full volumetric fuse with the spar; the upper 2 mm (y in [+3,
    # +5]) is the unsupported annulus above the spar that ties into
    # the pad at y = +5.
    neck_y_min = -LINK_THICKNESS / 2.0                          # -3
    neck_y_max = HORN_STACK_H                                   # +5
    hip_neck_outer = _cyl_along(HIP_PAD_R,
                                 neck_y_max - neck_y_min,
                                 axis="y")
    hip_neck_outer.apply_translation([0, neck_y_min, 0])
    # Unified horn-stack clearance void.  Spans y in
    # [-LINK_THICKNESS/2 - 1, HORN_STACK_H + 1] = [-4, +6], the same
    # 1 mm overshoots at both ends as the previous deep-cup design.
    void_y_lo = -LINK_THICKNESS / 2.0 - 1.0                     # -4
    void_y_hi = HORN_STACK_H + 1.0                              # +6
    hip_neck_void = _cyl_along(HORN_STACK_VOID_R,
                                void_y_hi - void_y_lo, axis="y")
    hip_neck_void.apply_translation([0, void_y_lo, 0])

    hip_holes = []
    hip_counterbores = []
    for a in XHORN_BOLT_ANGLES_RAD:
        # Drill the 4 M2 clamp holes through the pad's 6 mm-thick
        # clamp ring (y in [+5, +11]).  Phi
        # XHORN_BOLT_M2_SELFTAP_HOLE_OD = 2.2 mm clearance through the
        # pad (M2 self-tap into the X-horn's Phi ~ 2.0 mm arm hole
        # below; the X-horn provides the actual thread engagement).
        # The cylinder length = LINK_THICKNESS * 4 = 24 mm guarantees
        # a clean punch-through of the 6 mm pad even with voxel/CSG
        # tolerance; any over-length extends past the pad's -Y face
        # into the flange ring's void (the ring's PCD = 10.4 mm radius
        # sits well inside the void's HORN_STACK_VOID_R = 18.5 mm
        # boundary), so the bolt hole punches through air there and
        # doesn't actually remove any ring material.
        # (May 2026 fastener-spec fix: previously this used
        # SERVO_TAB_HOLE = 3.2 mm because Phi 3.2 also happened to be
        # the (incorrect) M3 X-horn clearance; with M2 X-horn bolts
        # the link-pad clearance is unique to the X-horn pattern.)
        h = _cyl(XHORN_BOLT_M2_SELFTAP_HOLE_OD / 2.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              hip_pad_centre_y,
                              XHORN_BOLT_PCD / 2.0 * np.sin(a)])
        hip_holes.append(h)

        # Counter-bore for the M2 SHCS head, opening AWAY from the
        # X-horn (i.e. at the pad's +Y outer face).  The pad's outer
        # face is at y = hip_pad_y_max = HORN_STACK_H + LINK_THICKNESS;
        # the counter-bore cylinder extends from y = pad_y_max -
        # COUNTERBORE_DEPTH back through the outer face by 0.05 mm of
        # overshoot to guarantee a clean CSG cut.  Head TOP sits flush
        # with the pad's outer +Y face, head BOTTOM at y =
        # pad_y_max - 2 mm (M2 SHCS head height).  The remaining
        # pad material (y in [hip_pad_y_min, hip_pad_y_max -
        # COUNTERBORE_DEPTH] = [5, 8.5]) is the actual clamp face
        # that bears against the X-horn arm at y = HORN_STACK_H = 5.
        cb_len = COUNTERBORE_DEPTH + 0.1
        cb = _cyl(M2_HEAD_OD_CLEARANCE / 2.0, cb_len)
        cb.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        cb.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              hip_pad_y_max - cb_len / 2.0 + 0.05,
                              XHORN_BOLT_PCD / 2.0 * np.sin(a)])
        hip_counterbores.append(cb)

    # ---- Central horn-hub recess in the pad's mating face -----------
    # Design B (May 2026): the femur's hip pad bolts DIRECTLY onto the
    # plastic 4-arm X-horn (no printed servo_horn_adapter in the
    # stack).  Cut a Phi HORN_RECESS_OD = 16 mm cylindrical pocket
    # HORN_RECESS_DEPTH = 1.2 mm deep into the pad's -Y mating face
    # (at y = hip_pad_y_min = HORN_STACK_H) so the plastic horn's
    # central hub (spline collar + M3 centre-screw head) is fully
    # swallowed by the pad.  Recess opens in -Y (toward the horn) and
    # removes pad material at y in [hip_pad_y_min, hip_pad_y_min +
    # HORN_RECESS_DEPTH].
    hip_horn_recess = _cyl_along(HORN_RECESS_OD / 2.0,
                                  HORN_RECESS_DEPTH, axis="y")
    hip_horn_recess.apply_translation([0.0, hip_pad_y_min, 0.0])

    # ---- Knee-end servo well -----------------------------------------
    # NB: the 4 M3 mounting pilots in the wall (drilled by
    # ``_servo_well_solid``) sit on the standard SERVO_TAB_HOLE_PCD x
    # SERVO_TAB_HOLE_PCD_Y (49.5 x 10 mm) pattern -- NOT the 24 mm
    # XHORN_BOLT_PCD pattern.  XHORN_BOLT_PCD is the bolt circle for the
    # hip-pad horn adapter at the OTHER end of the femur.
    #
    # remove_floor=True: cut the cradle's floor frame so the print
    # orientation (spar broad face on bed, mouth-down) no longer
    # bridges a ~30 x 40 mm closed ceiling at the femur's -Y end.
    # See the function docstring "Knee cradle floor: OPEN" section
    # above for the load-path argument.
    well = _servo_well_solid(remove_floor=True)
    wire_slot = _wire_exit_slot()
    cable_post = _cable_zip_post()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])    # well +Z -> femur +Y
    well.apply_transform(R)
    wire_slot.apply_transform(R)
    cable_post.apply_transform(R)
    delta = np.array([FEMUR_LENGTH - SERVO_OUTPUT_X,
                       -(SERVO_BODY_H + SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)
    wire_slot.apply_translation(delta)
    cable_post.apply_translation(delta)

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

    # ---- (Removed) lightening holes through the spar -----------------
    # Earlier revisions drilled 2 x Phi 13 mm cross-spar holes between
    # the hip pad and the knee-end body cradle ("for weight + zip-tie
    # tie-down").  At HIP_PAD_R = 20 mm the first hole's outer rim
    # (x_centre ~ 17 -> x_extent [10.5, 23.5]) lands within 0.1 mm of
    # the M2 X-horn bolt at angle 0 deg (x = 10.4, z = 0) -- the
    # lightening hole eats half the bolt's clearance sleeve and leaves
    # the screw naked on its +X side.  See the analogous tibia code
    # below where the holes sit well away from the foot pin and any
    # bolt circles; on the femur there is no clean place to put them
    # without intruding on the hip pad bolt circle.  We forgo them.
    # The 90 mm femur spar is not heavy enough to require lightening.

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

    # Cable post (Part A, May 2026): printed-in zip-tie strain relief
    # next to the knee wire-exit slot.  Built in well-local and
    # transformed alongside ``wire_slot`` (R + delta) so it stays
    # anchored to the well's +X outer wall.
    body = _union(hip_pad, hip_neck_outer, spar, well,
                   bridge_top, bridge_bot, cable_post)
    return _diff(body, hip_neck_void, insertion_slot, wire_slot,
                 cavity_trim, knee_clear, hip_horn_recess,
                 *hip_holes, *hip_counterbores)


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
    Foot end: a single TANG (a LINK_THICKNESS-wide downward tongue,
    centred on tibia y=0) that slides into the foot pad's clevis
    fork (see ``make_foot_pad``).  The tang carries an M3 through-
    hole at z = FOOT_HINGE_TIBIA_Z, parallel to the knee axis.

    May 2026 supports-free refactor: the tibia is now LINK_THICKNESS
    = 6 mm wide in tibia Y EVERYWHERE (spar + tang) so the entire
    part prints as a flat 6-mm-tall slab when oriented with the
    spar's broad face on the bed.  Pre-2026 the tibia ended in a
    12-mm-wide CLEVIS (2 cheeks + 4 mm tongue slot) that protruded
    6 mm above the spar's broad face in that orientation and forced
    supports under the cheeks.  Inverting the hinge so the FOOT
    carries the fork and the TIBIA carries a single tang removed
    the protrusion; the hinge axis, pin, nut and rotational
    kinematics are otherwise identical to the old design.
    """
    spar = _box((TIBIA_LENGTH, LINK_THICKNESS, TIBIA_SPAR_H),
                center=(TIBIA_LENGTH / 2.0, 0, 0))

    # Knee pad + neck annulus: same DEEP ANNULUS design as the
    # femur's hip-end (May 2026 pad <-> spar fuse refactor, Option A).
    # See make_femur_link for the full design rationale.  The neck
    # annulus is 8 mm tall (y in [-3, +5]) with outer radius
    # HIP_PAD_R = 20 mm and inner radius HORN_STACK_VOID_R = 18.5 mm.
    #
    # The tibia matters MORE than the femur for this fix.
    # TIBIA_SPAR_H = 18 mm vs FEMUR_SPAR_H = 34 mm, so the tibia spar
    # has ~half the femur's pad-side cross-section to fuse with.  The
    # tibia spar's z range [-9, +9] sits ENTIRELY INSIDE the horn-
    # stack void's HORN_STACK_VOID_R = 18.5 mm radius, so after the
    # void cut the tibia spar's knee-end material at
    # sqrt(x^2 + z^2) < 18.5 mm is hollowed away -- the ONLY pad <->
    # spar fuse is the part of the spar+annulus union that lies at
    # radius >= 18.5 mm.  The 7e2ca87 flange-ring design left this
    # fuse as a 0.5 mm "boolean kiss" at y in [+2.5, +3], visibly
    # flimsy in inspect_build (user complaint x 2).  Deepening the
    # annulus to 8 mm (y in [-3, +5]) restores the full 6 mm
    # volumetric fusion zone at y in [-3, +3].
    knee_pad_centre_y = HORN_STACK_H + LINK_THICKNESS / 2.0    # +8
    knee_pad_y_min    = HORN_STACK_H                            # +5
    knee_pad_y_max    = HORN_STACK_H + LINK_THICKNESS           # +11
    knee_pad = _cyl_along(HIP_PAD_R,
                           knee_pad_y_max - knee_pad_y_min,
                           axis="y")
    knee_pad.apply_translation([0, knee_pad_y_min, 0])

    # 8 mm-tall neck annulus.  Spans y in [-LINK_THICKNESS/2,
    # HORN_STACK_H] = [-3, +5]: the lower 6 mm (y in [-3, +3]) is a
    # full volumetric fuse with the spar; the upper 2 mm (y in [+3,
    # +5]) is the unsupported annulus above the spar that ties into
    # the knee pad at y = +5.
    neck_y_min = -LINK_THICKNESS / 2.0                          # -3
    neck_y_max = HORN_STACK_H                                   # +5
    knee_neck_outer = _cyl_along(HIP_PAD_R,
                                  neck_y_max - neck_y_min,
                                  axis="y")
    knee_neck_outer.apply_translation([0, neck_y_min, 0])
    # Unified horn-stack clearance void.  Spans y in
    # [-LINK_THICKNESS/2 - 1, HORN_STACK_H + 1] = [-4, +6].
    void_y_lo = -LINK_THICKNESS / 2.0 - 1.0                     # -4
    void_y_hi = HORN_STACK_H + 1.0                              # +6
    knee_neck_void = _cyl_along(HORN_STACK_VOID_R,
                                 void_y_hi - void_y_lo, axis="y")
    knee_neck_void.apply_translation([0, void_y_lo, 0])

    knee_holes = []
    knee_counterbores = []
    for a in XHORN_BOLT_ANGLES_RAD:
        # Bolt holes drilled through the PAD ONLY (y in [+5, +11]).
        # Phi XHORN_BOLT_M2_SELFTAP_HOLE_OD = 2.2 mm M2 clearance
        # through the pad (M2 self-tap into the X-horn's Phi ~ 2.0 mm
        # arm hole below).  The bolt-PCD radius (10.4 mm) is INSIDE
        # the horn-stack void (HORN_STACK_VOID_R = 18.5 mm), so any
        # over-long hole punches through air inside the ring/void and
        # doesn't remove material.  See make_femur_link for the May
        # 2026 M3 -> M2 X-horn fastener-spec fix history.
        h = _cyl(XHORN_BOLT_M2_SELFTAP_HOLE_OD / 2.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              knee_pad_centre_y,
                              XHORN_BOLT_PCD / 2.0 * np.sin(a)])
        knee_holes.append(h)

        # Counter-bore for the M2 SHCS head, opening AWAY from the
        # X-horn (at the knee pad's +Y outer face).  Same geometry as
        # make_femur_link's hip_counterbores: Phi 4 mm pocket 2.5 mm
        # deep, with the head TOP flush at the pad's outer face.  The
        # head BOTTOM is at y = knee_pad_y_max - 2 mm; the remaining
        # 3.5 mm of pad material below the counter-bore clamps the
        # X-horn arm at y = HORN_STACK_H.
        cb_len = COUNTERBORE_DEPTH + 0.1
        cb = _cyl(M2_HEAD_OD_CLEARANCE / 2.0, cb_len)
        cb.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        cb.apply_translation([XHORN_BOLT_PCD / 2.0 * np.cos(a),
                              knee_pad_y_max - cb_len / 2.0 + 0.05,
                              XHORN_BOLT_PCD / 2.0 * np.sin(a)])
        knee_counterbores.append(cb)

    # ---- Central horn-hub recess in the knee pad's mating face ------
    # Design B (May 2026): see make_femur_link for the full
    # docstring.  Identical geometry, just on the tibia's knee pad
    # at y = knee_pad_y_min = HORN_STACK_H.
    knee_horn_recess = _cyl_along(HORN_RECESS_OD / 2.0,
                                   HORN_RECESS_DEPTH, axis="y")
    knee_horn_recess.apply_translation([0.0, knee_pad_y_min, 0.0])

    # ----- Foot tang at the far end (x ~ TIBIA_LENGTH) -----
    # May 2026 inversion: the tibia ends in a SINGLE TANG that is
    # in-plane with the spar (LINK_THICKNESS wide in Y, centred on
    # tibia y=0), not a forked clevis.  Pre-2026 the tibia had a
    # 12 mm-wide CLEVIS (2 x FOOT_HINGE_CHEEK_T + FOOT_HINGE_GAP)
    # with a tongue-accepting slot; that geometry stuck 6 mm above
    # the spar's broad face in the print orientation and forced
    # supports.  Inverting the hinge so the FOOT carries the fork
    # and the TIBIA carries a single tang keeps the tibia
    # LINK_THICKNESS-wide everywhere; see the docstring above and
    # the FOOT_HINGE_* / FOOT_TANG_* constants block for the full
    # rationale.
    #
    # Tang geometry: a single solid box that occupies the same X/Z
    # footprint as the old clevis bulk (FOOT_TANG_X_INBOARD inboard
    # of TIBIA_LENGTH, FOOT_TANG_X_BEYOND_TIP past the tip, dropping
    # FOOT_TANG_BELOW_PIN below the pin axis at z=FOOT_HINGE_TIBIA_Z)
    # but only LINK_THICKNESS = 6 mm wide in Y instead of 12 mm.  No
    # slot cut.  The M3 through-hole still drills along tibia +Y at
    # (TIBIA_LENGTH, 0, FOOT_HINGE_TIBIA_Z); the bolt now passes
    # through the FOOT's fork cheeks + tang (in that order) instead
    # of the old tibia cheeks + foot tongue.
    tang_x_min = TIBIA_LENGTH - FOOT_TANG_X_INBOARD
    tang_x_max = TIBIA_LENGTH + FOOT_TANG_X_BEYOND_TIP
    tang_dx    = tang_x_max - tang_x_min
    tang_cx    = (tang_x_min + tang_x_max) / 2.0

    tang_z_min = FOOT_HINGE_TIBIA_Z - FOOT_TANG_BELOW_PIN          # -15
    tang_z_max = TIBIA_SPAR_H / 2.0                                # +9
    tang = _box((tang_dx,
                  LINK_THICKNESS,
                  tang_z_max - tang_z_min),
                 center=(tang_cx, 0.0,
                          (tang_z_max + tang_z_min) / 2.0))

    # Pin hole through the tang (single bore in tibia Y).  Length =
    # 4 * LINK_THICKNESS so the cylinder cleanly punches through
    # even with FDM/Hildebrand voxelisation slop.
    pin_hole = _cyl(FOOT_HINGE_PIN_HOLE_D / 2.0, LINK_THICKNESS * 4.0)
    pin_hole.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
    pin_hole.apply_translation([TIBIA_LENGTH, 0.0, FOOT_HINGE_TIBIA_Z])

    # A short taper to blend the spar into the tang.  Sits on the
    # spar centreline; in Y it matches the spar (LINK_THICKNESS) so
    # the whole tibia keeps a single Y thickness end-to-end.
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

    body = _union(knee_pad, knee_neck_outer, spar, taper, tang)
    return _diff(body, knee_neck_void, knee_horn_recess, *knee_holes,
                 *knee_counterbores, pin_hole, *lightening)


def make_foot_pad() -> trimesh.Trimesh:
    """Compliant foot pad with a single-axis hinge fork.

    Stack (foot-local Z, +Z up):

        ground   z = 0 .. FOOT_PAD_BASE_H               -- TPU spring disk
        boss     z = FOOT_PAD_BASE_H
                   .. FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H -- 14 mm OD stub
        fork     z = boss top
                   .. FOOT_HINGE_FOOT_Z + OVER_PIN      -- 2 cheeks (each
                                                          FOOT_HINGE_CHEEK_T
                                                          thick in Y) with
                                                          a FOOT_HINGE_SLOT_W
                                                          slot between them
                                                          for the tibia tang

    May 2026 inversion: the foot now carries the FORK (2 cheeks with
    a tang-accepting slot) and the tibia carries the single TANG;
    pre-2026 it was the other way round (tibia clevis + foot tongue).
    Total foot-pad Y extent across the fork: 2 * FOOT_HINGE_CHEEK_T
    + FOOT_HINGE_SLOT_W = 2 * 3.5 + 6.4 = 13.4 mm, which fits
    comfortably inside the disk's 28 mm OD.  The fork cheeks
    extend above the boss in foot-local +Z, so the disk + boss
    outlines are unchanged.

    The slot accepts the tibia tang (LINK_THICKNESS = 6 mm in
    tibia Y) with 0.2 mm clearance per side.  A horizontal M3
    clearance hole drilled along Y through both cheeks lines up
    with the tang's hole; an M3 x 16 pan-head bolt + nylock nut
    captures the joint just like the old design (no BOM change).

    The hinge axis (tibia-local +Y = foot-local +Y in world frame
    after both pieces are rotated by the leg's azimuth) is
    parallel to the knee axis, so when the tibia pitches the foot
    follows through ankle pitch.  TPU compliance in the disk
    itself absorbs roll.

    Printability note (TPU 95A): each fork cheek is
    FOOT_HINGE_CHEEK_T = 3.5 mm of TPU material in Y, comfortably
    above MIN_PRINT_T = 3.0 mm.  The cheeks print as two vertical
    TPU walls with the disk lying on the bed; the M3 hole is a
    horizontal bore through each cheek (no overhangs > 45 deg).

    Local frame: ground-plane at Z = 0; fork cheeks rise in +Z;
    the cheeks' broad faces have normals +/-Y (matches the
    tibia's knee-axis direction in the leg's local frame)."""
    pad_base = _cyl(FOOT_PAD_OD / 2.0, FOOT_PAD_BASE_H)
    pad_base.apply_translation([0, 0, FOOT_PAD_BASE_H / 2.0])

    boss = _cyl(FOOT_PAD_BOSS_OD / 2.0, FOOT_PAD_BOSS_H)
    boss.apply_translation([0, 0, FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H / 2.0])

    fork_z_lo = FOOT_PAD_BASE_H + FOOT_PAD_BOSS_H
    fork_z_hi = FOOT_HINGE_FOOT_Z + FOOT_HINGE_FORK_OVER_PIN
    fork_z_centre = 0.5 * (fork_z_lo + fork_z_hi)
    fork_z_ext = fork_z_hi - fork_z_lo

    # 2 cheeks, one at +Y and one at -Y.  Inner faces sit at
    # +/- FOOT_HINGE_SLOT_W/2; outer faces at +/-(SLOT_W/2 +
    # CHEEK_T) = +/-6.7 mm.  Cheek X extent = FOOT_HINGE_FORK_X =
    # 10 mm (matches the pre-inversion tongue's X width so the
    # joint preserves the same rotational clearance).
    cheek_y_centre = (FOOT_HINGE_SLOT_W / 2.0
                       + FOOT_HINGE_CHEEK_T / 2.0)
    cheek_plus = _box((FOOT_HINGE_FORK_X,
                        FOOT_HINGE_CHEEK_T,
                        fork_z_ext),
                       center=(0.0, +cheek_y_centre, fork_z_centre))
    cheek_minus = _box((FOOT_HINGE_FORK_X,
                         FOOT_HINGE_CHEEK_T,
                         fork_z_ext),
                        center=(0.0, -cheek_y_centre, fork_z_centre))

    # M3 through-hole drilled along foot-local +Y through both
    # cheeks at z=FOOT_HINGE_FOOT_Z, coaxial with the tibia tang's
    # hole when assembled.  Length oversized (2 cheeks + slot + a
    # comfortable margin) so the cylinder cleanly punches through
    # the entire fork even with FDM/Hildebrand voxelisation slop.
    pin_hole_len = (2.0 * FOOT_HINGE_CHEEK_T
                     + FOOT_HINGE_SLOT_W + 4.0) * 2.0
    hinge_hole = _cyl(FOOT_HINGE_PIN_HOLE_D / 2.0, pin_hole_len)
    hinge_hole.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
    hinge_hole.apply_translation([0.0, 0.0, FOOT_HINGE_FOOT_Z])

    return _diff(_union(pad_base, boss, cheek_plus, cheek_minus),
                 hinge_hole)


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

    # Battery holder (sits between the plates, slightly aft of centre).
    # ``BATTERY_HOLDER_CENTRE_X`` is the single source of truth for
    # this offset; the fastener_registry / _hex_plate / inspect_build
    # all read the same constant.
    bh = make_battery_holder()
    bh.apply_translation([BATTERY_HOLDER_CENTRE_X, 0,
                           chassis_lift + CHASSIS_PLATE_T / 2.0])
    parts.append(bh)

    # Electronics tray (sits between the plates, forward of centre)
    et = make_electronics_tray()
    et.apply_translation([35.0, 0,
                           chassis_lift + CHASSIS_PLATE_T / 2.0 + 1.0])
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
    parts.append(("bec_cradle.stl",       make_bec_cradle()))
    parts.append(("switch_holster.stl",   make_switch_holster()))
    parts.append(("imu_pad.stl",          make_imu_pad()))

    print("Leg parts (one of each -- print 6 sets):")
    parts.append(("coxa_bracket.stl",     make_coxa_bracket()))
    parts.append(("coxa_link.stl",        make_coxa_link()))
    parts.append(("femur_link.stl",       make_femur_link()))
    parts.append(("tibia_link.stl",       make_tibia_link()))
    parts.append(("foot_pad.stl",         make_foot_pad()))

    # Design B (May 2026): the printed servo_horn_adapter has been
    # RETIRED.  Each link now bolts directly to the plastic 4-arm
    # X-horn that ships with the servo (see HORN_RECESS_OD /
    # HORN_RECESS_DEPTH cuts in make_coxa_link / make_femur_link /
    # make_tibia_link, and the Design B notes near the top of this
    # file).  The ``make_servo_horn_adapter()`` factory itself is
    # preserved for backwards compatibility with downstream tooling
    # that still imports it, but is no longer called from any
    # printable-output path.

    print("Servo visuals (not for printing -- MuJoCo / fit-check meshes):")
    parts.append(("servo_body.stl", make_servo_body()))
    parts.append(("servo_horn.stl", make_servo_horn()))
    parts.append(("mpu6050.stl",    make_mpu6050_visual()))

    print("Electronics visuals (NOT FOR PRINTING -- BuildViz / inspector\n"
          "only; user buys these from Amazon / McMaster):")
    parts.append(("arduino_mega.stl",   make_arduino_mega_visual()))
    parts.append(("raspberry_pi.stl",   make_raspberry_pi_visual()))
    parts.append(("pca9685.stl",        make_pca9685_visual()))
    parts.append(("bec_visual.stl",     make_bec_visual()))
    parts.append(("antispark_switch_body.stl",
                  make_antispark_switch_body_visual()))
    parts.append(("antispark_switch_toggle.stl",
                  make_antispark_switch_toggle_visual()))
    parts.append(("lipo_battery_body.stl",
                  make_lipo_battery_body_visual()))
    parts.append(("lipo_xt60.stl",      make_lipo_xt60_visual()))

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
