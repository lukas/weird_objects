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
        servo_horn_adapter.stl  -- 4-arm star that bolts to a standard 25T spline
                                   horn and provides 4 x M3 holes on a 24 mm PCD
                                   so a flat printed link can mate to a servo horn

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

# ---- Leg link lengths -----------------------------------------------------
# Same 1 : 4 : ~5 ratio as the full-size walker, scaled for a tabletop
# build.  Tibia is intentionally a hair longer than 4 x coxa so the
# foot can lift clear over a small obstacle in swing phase.
COXA_LENGTH    =  25.0   # mm -- yaw axis -> hip-pitch axis
FEMUR_LENGTH   =  90.0   # mm -- hip-pitch axis -> knee axis
TIBIA_LENGTH   = 130.0   # mm -- knee axis -> foot tip

# ---- Servo (actuator) ----------------------------------------------------
# Generic 25 kg-cm digital servo (DS3225, MG996R, etc.).  The body is a
# rectangular brick; mounting tabs stick out on the +/-X faces; output
# shaft is on the +Z face, offset from centre.  The horn screws onto a
# 25-tooth spline with M3 fastener.
SERVO_BODY_W      = 40.0   # mm -- length of the servo body (along output-shaft offset)
SERVO_BODY_D      = 20.0   # mm -- depth (perpendicular)
SERVO_BODY_H      = 38.0   # mm -- height of the body (without output gear)
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
WIRE_SLOT_DEPTH = 7.5   # mm reach UP into the -X wall above the cavity
                        # floor -- taller opening for molded strain relief /
                        # bundling to turn out of the well.
WIRE_SLOT_X_PAST_WALL = 4.0   # mm beyond outer -X well face (was 2) for exit
WIRE_SLOT_X_INBOARD   = 2.5   # mm inside cavity -X body face (was 1) for opening
WIRE_SLOT_Z_BELOW_FLOOR = 4.0  # mm below outer floor (was 2) for thick boots

# Vertical channel cut into the inside surface of the -X cavity wall so the
# 3-wire harness has a place to lie flat against the wall as it descends to
# the L-shaped exit at the bottom (instead of being pinched between the
# servo back-case and the wall and squirting out over the well rim).
# Pilot clearance: the M3 pilots at (x = -/+24.75, y = +/-5) have ~1.25 mm
# radius columns, so their inner Y edges sit at y = +/-3.75.  The channel
# only spans y = +/-WIRE_SLOT_W/2 (= +/-3 with WIRE_SLOT_W = 6), giving
# 0.75 mm of Y separation from each pilot column.  Because the pilots are
# clear of the channel in Y, the channel can be deepened in X without
# crossing them in 3D.
WIRE_CHANNEL_DEPTH    = 4.0   # mm groove depth INTO the -X wall material
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
WELL_WALL_Y  = 2.5   # mm thick on +Y / -Y faces
WELL_FLOOR_T = 2.5   # mm bottom-plate thickness.  The plate now exists only
                     # as a perimeter FRAME around the (open) cavity opening,
                     # tying the +X/-X tab posts to the +Y/-Y walls at the
                     # bracket's print-bed face.  The servo body does NOT rest
                     # on the floor -- it hangs from its mounting tabs on the
                     # rim, with WELL_TAB_FLOAT mm of clearance from the
                     # nominal floor plane (and the cavity itself punches all
                     # the way through the floor, so a body that exceeds the
                     # nominal SERVO_TAB_Z depth simply pokes out the bottom
                     # of the bracket instead of bottoming the tab above the
                     # rim).
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
BRACKET_FLANGE_T   =  4.0   # mm thick mounting flange
BRACKET_FLANGE_X   = 30.0   # mm long (radial -- inboard from chassis edge)
BRACKET_FLANGE_Y   = 48.0   # mm wide (tangential).  Sized so the slot and
                            # the four chassis bolt holes each have >= 4 mm
                            # of material around them.
BRACKET_BOLT_PCD_X = 16.0   # mm centre-to-centre, inboard vs. outboard bolt pair
BRACKET_BOLT_PCD_Y = 36.0   # mm centre-to-centre, +Y vs. -Y bolt line
BRACKET_BOLT_HOLE  =  3.4   # mm clearance for M3 chassis bolts
BRACKET_FLANGE_INSET = 8.0  # mm distance from the chassis edge (= bracket
                            # origin's outboard face) to the OUTBOARD bolt
                            # line.  >= BRACKET_BOLT_HOLE so the bolt is on
                            # solid chassis material, not in mid-air.

# ---- Servo horn adapter --------------------------------------------------
# A short 4-arm star that screws onto the servo's plastic horn (M3
# centre screw) and presents a 4 x M3 bolt pattern (24 mm PCD) on its
# top face.  The flat printed links bolt to this top face, so the link
# itself never touches the servo spline.
HORN_ADAPTER_OD     = 32.0   # mm -- plate OD; gives ~2.4 mm wall outboard of
                              # each M3 bolt hole on HORN_BOLT_PCD = 24 mm
HORN_ADAPTER_T      =  4.0   # mm -- thickness
HORN_BOLT_PCD       = 24.0   # mm -- 4 x M3 holes on this PCD
HORN_BOLT_OD        =  3.2   # mm -- M3 clearance
HORN_CENTRE_OD      =  3.4   # mm -- M3 centre clearance (for the horn screw)
HORN_RECESS_OD      = 21.0   # mm -- counter-bore for the plastic horn body
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
HIP_PAD_R        = HORN_BOLT_PCD / 2.0 + 5.0   # 17 mm -- pad radius to
                                                # comfortably contain
                                                # the 24 mm bolt PCD

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

# ---- Foot ----------------------------------------------------------------
# A small printed cup that gets lined with rubber (cut a section of
# bicycle inner tube, glue inside the cup) for traction.
FOOT_PAD_OD     = 28.0   # mm -- outer diameter
FOOT_PAD_HEIGHT = 14.0
FOOT_HUB_OD     = 12.0   # mm -- attaches to the tibia tip
FOOT_HUB_HEIGHT =  6.0

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
    DS3225 / MG996R / DS3218-class hobby servos route their 3-wire
    harness out of the BOTTOM-OUTBOARD corner of the body -- the
    corner where the back case meets the -X short-end face (the end
    opposite the output spline).  The bundle is glued into that
    corner, so to leave room for it the cradle's matching corner has
    to be open for BOTH:

      * a straight-DOWN escape path through the well floor at the
        body's -X end, AND
      * a horizontal-OUTBOARD escape path through the bottom of the
        -X wall, so a harness that's been bent 90 deg around the
        corner of the case can leave the cradle laterally.

    A single rectangular box at the well's -X bottom-outboard corner
    does both jobs in one boolean cut: it punches through the floor
    at the body's -X end AND through the bottom of the -X wall above
    that floor patch.

    Slot footprint in well-local coordinates uses WIRE_SLOT_* constants.
    Pilots at (x = +/-24.75, y = +/-5): keep WIRE_SLOT_W modest so Y clearance
    to the pilot cylinders is preserved; depth/X extensions help the molded
    harness pass without fouling M3 thread engagement high on the -X wall.

    Use as:
        well = _servo_well_solid()
        slot = _wire_exit_slot()
        # ... apply the same R / translation to BOTH ...
        body = _diff(body, slot)
    """
    slot_x_min = -WELL_W / 2.0 - WIRE_SLOT_X_PAST_WALL
    slot_x_max = -SERVO_BODY_W / 2.0 + WIRE_SLOT_X_INBOARD
    slot_x_extent = slot_x_max - slot_x_min
    slot_x_centre = 0.5 * (slot_x_min + slot_x_max)

    slot_y_extent = WIRE_SLOT_W                 # centred on y = 0

    slot_z_bottom = -WELL_FLOOR_T - WIRE_SLOT_Z_BELOW_FLOOR
    slot_z_top    = WIRE_SLOT_DEPTH
    slot_z_extent = slot_z_top - slot_z_bottom
    slot_z_centre = 0.5 * (slot_z_bottom + slot_z_top)

    exit_l = _box((slot_x_extent, slot_y_extent, slot_z_extent),
                  center=(slot_x_centre, 0.0, slot_z_centre))

    # Vertical channel on the INSIDE surface of the -X wall.
    # Spans from inside the cavity (so it merges seamlessly with the cavity
    # face) to WIRE_CHANNEL_DEPTH into the wall.  Pilot clearance:
    # cavity face at -SERVO_BODY_W/2 - WELL_BODY_CL = -20.4; channel
    # outer x = -20.4 - WIRE_CHANNEL_DEPTH = -23.4 (default), leaving
    # ~1.35 mm of wall between the channel and the pilot at x = -24.75.
    ch_x_max = -SERVO_BODY_W / 2.0 + WIRE_SLOT_X_INBOARD
    ch_x_min = -SERVO_BODY_W / 2.0 - WELL_BODY_CL - WIRE_CHANNEL_DEPTH
    ch_x_extent = ch_x_max - ch_x_min
    ch_x_centre = 0.5 * (ch_x_max + ch_x_min)

    # Z: from the floor (where it meets the L-shape) up over the rim so a
    # harness exiting anywhere on the back face has a path down.
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
    return _union(body, spline)


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
    Used as a visual stand-in only; the printed ``servo_horn_adapter`` bolts
    on top of this.
    """
    parts: list[trimesh.Trimesh] = []
    hub = _cyl(8.0, 2.0)
    hub.apply_translation([0, 0, 1.0])
    parts.append(hub)
    for i in range(4):
        ang = i * np.pi / 2.0
        arm = _box((20.0, 4.0, 1.6))
        arm.apply_transform(rotation_matrix(ang, [0, 0, 1]))
        arm.apply_translation([0, 0, 0.8])
        parts.append(arm)
    return _union(*parts)


def make_servo_horn_adapter() -> trimesh.Trimesh:
    """Round servo-horn adapter plate.

    Bolts to a standard plastic servo horn from below (single M3 centre
    screw + a counter-bored recess for the horn body) and presents a flat
    4 x M3 bolt pattern on the bolt circle so any link with the matching
    hole pattern can clamp onto it.

    Local frame:
        +Z = servo output axis
        Origin at the bottom face (mating to the plastic horn)
        Bolt holes at 45, 135, 225, 315 deg on HORN_BOLT_PCD.

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
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
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
               with_centre_holes: bool = False) -> trimesh.Trimesh:
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

    holes = []
    for i in range(6):
        a = (i + 0.5) * np.pi / 3
        edge_mid = np.array([apothem * np.cos(a),
                              apothem * np.sin(a),
                              0.0])
        R = rotation_matrix(a, [0, 0, 1])[:3, :3]
        for bx in (bolt_x_outboard, bolt_x_inboard):
            for by in bolt_ys:
                world = edge_mid + R @ np.array([bx, by, 0.0])
                h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
                h.apply_translation([world[0], world[1], 0])
                holes.append(h)

    if with_centre_holes:
        # 4 holes for the electronics tray + battery holder mounting
        for i in range(4):
            a = np.pi / 4 + i * np.pi / 2
            h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
            h.apply_translation([35.0 * np.cos(a), 35.0 * np.sin(a), 0])
            holes.append(h)

    return _diff(plate, *holes)


def make_chassis_top() -> trimesh.Trimesh:
    """Top hex plate.  3D-printed in PLA, ~ 4 mm thick, with the same
    bolt pattern as `make_chassis_bottom` so the leg coxa brackets are
    sandwiched between the two plates."""
    return _hex_plate(CHASSIS_FLAT_TO_FLAT, CHASSIS_PLATE_T,
                       with_centre_holes=True)


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
        - HORIZONTAL flange at z in [0, BRACKET_FLANGE_T] = [0, 4],
          spanning x in [-BRACKET_FLANGE_X, 0] = [-30, 0] and
          y in [-BRACKET_FLANGE_Y/2, +BRACKET_FLANGE_Y/2] = [-20, 20].
          Bolts vertically to the chassis plate (4 M3 bolts go DOWN
          through the flange and the chassis plate together).
        - SERVO WELL hangs below the flange.  The servo's output spline
          is at bracket-local (0, 0, GEAR_TOP_Z).  The well's body
          cavity is offset by -SERVO_OUTPUT_X = -10 in X so the gear
          lands on the yaw axis.  Cradle is OPEN at the top so the
          servo can be inserted from above.

    Assembly order:
        1.  Drop the yaw servo straight down through the flange's body
            cutout into the well.  Tabs land on the well rim.
        2.  Drive 4 M3 self-tappers through the tab holes into the
            well's pilot holes.
        3.  Bolt the bracket flange to the chassis plate (4 M3 cap
            screws + nylock nuts under the chassis plate).
        4.  Add the horn adapter and the coxa link on top of the
            output spline.
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
    flange_centre_x = -BRACKET_FLANGE_X / 2.0          # flange spans x in [-FLANGE_X, 0]
    flange = _box((BRACKET_FLANGE_X, BRACKET_FLANGE_Y, BRACKET_FLANGE_T),
                  center=(flange_centre_x, 0.0,
                           BRACKET_FLANGE_T / 2.0))

    # Cut a body+tab passage through the flange (and through the rib
    # below it) so the user can drop the servo straight down through
    # the flange into the well.  Slot is 2 mm wider than the tab tip
    # span (X) so the tabs have clearance going through, and 1 mm
    # wider than the body (Y) on each side so the body slips through.
    # Slot z range covers from the WELL RIM (z = 0) all the way up
    # past the flange's top so it cuts through both the flange and
    # the rib.
    slot_w = SERVO_TAB_W + 2.0
    slot_d = SERVO_BODY_D + 1.0
    slot_h = BRACKET_FLANGE_T + 8.0   # 12 mm tall: spans z=[-4, 8]
    slot = _box((slot_w, slot_d, slot_h),
                center=(body_centre_x, 0.0, BRACKET_FLANGE_T / 2.0 - 2.0))

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
    # The flange's outboard edge at x = 0 sits on top of the well's
    # inboard wall; the well's inboard wall is at x in [body_x - WELL_W/2,
    # body_x - WELL_W/2 + WELL_WALL_X] = [-39, -30].  Rib spans y over
    # the flange's width and z from the well rim (z = 0) up to the
    # flange's top (z = FLANGE_T) on the +X edge of the flange so it
    # provides a continuous load path.  We achieve this naturally by
    # making the flange's outboard edge slightly DEEPER than just z=0:
    # extend the flange material DOWN to z = -2 over its outboard
    # ~18 mm, so it overlaps with the well's inboard wall material.
    # The drop-in slot (slot_w x slot_d, centred at body_centre_x = -10)
    # cuts the rib's centre out at y in [-10.5, +10.5]; widening the
    # rib in X (was 10 mm, now 18 mm) doesn't change the slot but does
    # add ~80 % more material to the two surviving side strips that
    # actually carry the flange-to-well load.
    RIB_X = 18.0
    rib = _box((RIB_X, BRACKET_FLANGE_Y, 6.0),
               center=(-RIB_X / 2.0, 0.0, 0.0))   # x in [-18, 0]

    # Side gussets riding directly on top of the well's +Y / -Y walls.
    # The drop-in slot through the flange eats the rib at y in
    # [-10.5, +10.5] and reaches DOWN to z = -4, so the only flange-to-
    # well load paths at the well's +Y / -Y wall tops are these two
    # narrow column strips spanning Z = [-6, +4] above the well wall
    # (which lives at y in [10.5, 12.5] and y in [-12.5, -10.5]).
    # Each gusset is bounded in Y to the well's wall footprint so
    # nothing dangles in mid-air past the well's outer Y faces.
    slot_d = SERVO_BODY_D + 1.0
    gusset_z_min = -6.0                                       # 6 mm down into well rim
    gusset_z_max = BRACKET_FLANGE_T                           # up through flange top
    gusset_z_ext = gusset_z_max - gusset_z_min                # 10 mm
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

    body = _union(flange, well, rib, *side_gussets)
    return _diff(body, slot, wire_slot, *chassis_holes)


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
        - Hub: square pad at the origin, centred on the yaw axis,
          with a 4-bolt hole pattern matching the horn adapter.
        - Arm: flat plate extending in +X from the hub.
        - Hip-pitch servo well: hangs in -Z below the arm at the +X
          end.  Open-topped (well +Z) is mapped to link +Y so the
          servo can be DROPPED in from the +Y direction during
          assembly.
    """
    arm_w = 22.0   # mm, along Y
    arm_t =  6.0   # mm, along Z (printed flat against the build plate).
                   # The arm carries the entire leg below as a cantilever in
                   # +X off the yaw hub; its weak axis is out-of-plane (Y)
                   # bending where I = arm_w * arm_t^3 / 12 scales with t^3.
                   # 6 mm gives ~3.4x the stiffness of the prior 4 mm.

    # Hub region (above the yaw servo horn) -- a thicker square
    hub = _box((34.0, 34.0, arm_t + 2.0),
               center=(0, 0, (arm_t + 2.0) / 2.0))
    # 4-bolt pattern matching the horn adapter
    hub_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(HORN_BOLT_OD / 2.0, (arm_t + 2.0) * 4)
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              HORN_BOLT_PCD / 2.0 * np.sin(a),
                              (arm_t + 2.0) / 2.0])
        hub_holes.append(h)
    centre_hole = _cyl(HORN_CENTRE_OD / 2.0, (arm_t + 2.0) * 4)
    centre_hole.apply_translation([0, 0, (arm_t + 2.0) / 2.0])

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
    # interpenetrating it.
    well_z_drop = -(WELL_D / 2.0 + arm_t / 2.0)
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
    # Drop the bridge bottom DOWN to ~1 mm above the seated servo body
    # top face (body top sits at link-z = well_z_drop + SERVO_BODY_D/2
    # = -5.5 with the current geometry, so a bridge bottom at -4.5
    # leaves 1 mm of insertion clearance).  Was well_top_z - 0.5 =
    # -3.5; deepening to -4.5 grows the bridge Z section from 9.5 mm
    # to 10.5 mm.
    bridge_z_min = -COXA_BRIDGE_GUSSET_H + 0.5                  # ~ -4.5
    bridge_z_max = arm_t                                        # up to arm top
    bridge_z_extent = bridge_z_max - bridge_z_min
    bridge_z_centre = (bridge_z_min + bridge_z_max) / 2.0
    bridge = _box((arm_x_extent, bridge_y_extent, bridge_z_extent),
                  center=(arm_x_centre, bridge_y_centre, bridge_z_centre))

    # Stiffening gusset stacked on the +X end of the arm on TOP of the
    # well.  Extended in -Y to cover the bridge so the upper flange of
    # the arm-bridge-well section becomes a continuous 26+ mm-wide cap
    # over the joint instead of stepping down at y = -arm_w/2.
    gusset_y_min     = bridge_y_min                             # overlaps bridge in -Y
    gusset_y_max     = +arm_w / 2.0                             # arm +Y edge
    gusset_y_extent  = gusset_y_max - gusset_y_min              # ~ 28 mm
    gusset_y_centre  = (gusset_y_min + gusset_y_max) / 2.0
    gusset = _box((30.0, gusset_y_extent, arm_t),
                  center=(COXA_LENGTH - 14.0, gusset_y_centre,
                          arm_t / 2.0))

    # Underside stiffening gusset hanging below the arm in the bridge
    # region.  Adds Z depth to the cross-section so the coxa link can
    # resist hip-pitch reaction torque (about link +Y) without visible
    # flex.  See COXA_BRIDGE_GUSSET_H / COXA_BRIDGE_GUSSET_L for the
    # clearance reasoning.  The gusset's top face overlaps the arm
    # bottom by 0.5 mm, the +Y side overlaps the arm by 2 mm in Y
    # (so the arm-to-gusset joint is a solid 2 x 4 mm bar per X unit
    # rather than the bridge member's 0.5 x 0.5 mm boolean-overlap
    # strip), and the -Y face overlaps the cradle outer's +Y face by
    # 0.5 mm so it tied into the cradle wall on union.
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

    body = _union(hub, arm, well, gusset, bridge, gusset_under)
    return _diff(body, wire_slot, *hub_holes, centre_hole)


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

    # ---- Hip-end pad -------------------------------------------------
    hip_pad = _box((2 * HIP_PAD_R, LINK_THICKNESS, 2 * HIP_PAD_R),
                   center=(0, 0, 0))

    hip_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(SERVO_TAB_HOLE / 2.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              0,
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
    # [+11, +15] (top flange) and [-15, -11] (bottom flange).  The well
    # wraps z in [-WELL_D/2, +WELL_D/2] = [-12.5, +12.5].  Each bridge
    # connects a spar flange to the well's wall at the same z.
    spar_far_y      =  LINK_THICKNESS / 2.0           # +3 (spar's +Y face)
    spar_near_y     = -LINK_THICKNESS / 2.0           # -3 (spar's -Y face)
    well_near_y     = WELL_RIM_Z + delta[1]           # well's +Y face = -18.25
    bridge_y_min    = well_near_y - 0.5
    # Embed the bridge's +Y end 0.5 mm INTO the spar (rather than letting
    # it overshoot 0.5 mm past the spar's +Y face).  Either choice gives
    # the same 5.5 mm of spar-thickness overlap that the boolean union
    # needs, but embedding keeps the spar's +Y face as the part's
    # outermost surface in -Y -- which, after the -90 deg X reorient for
    # printing in ``prepare_xometry_upload.py``, becomes the broad face
    # that sits on the build plate.  Overshooting put a pair of 5 mm
    # bridge-flange ridges 0.5 mm below the spar so the spar floated
    # half a mm above the bed; this version makes the full spar+hip-pad
    # broad face the print's bottom layer.
    bridge_y_max    = spar_far_y - 0.5
    bridge_y_extent = bridge_y_max - bridge_y_min     # 21 mm
    bridge_y_centre = (bridge_y_min + bridge_y_max) / 2.0
    bridge_x_extent = slot_x                          # span the body's x range
    bridge_x_centre = (body_x_min + body_x_max) / 2.0

    # Top flange bridge: z spans [body_top, spar_top] so it overlaps
    # the well's top wall and the spar's top flange.
    body_z_max  = +SERVO_BODY_D / 2.0                 # +10
    spar_z_max  = +FEMUR_SPAR_H / 2.0                 # +15
    bridge_top_z_extent = (spar_z_max - body_z_max)   # 5 mm
    bridge_top_z_centre = (body_z_max + spar_z_max) / 2.0   # +12.5
    bridge_top = _box((bridge_x_extent, bridge_y_extent,
                       bridge_top_z_extent),
                      center=(bridge_x_centre, bridge_y_centre,
                               bridge_top_z_centre))

    bridge_bot_z_extent = bridge_top_z_extent
    bridge_bot_z_centre = -bridge_top_z_centre
    bridge_bot = _box((bridge_x_extent, bridge_y_extent,
                       bridge_bot_z_extent),
                      center=(bridge_x_centre, bridge_y_centre,
                               bridge_bot_z_centre))

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

    body = _union(hip_pad, spar, well, bridge_top, bridge_bot)
    return _diff(body, insertion_slot, wire_slot, *hip_holes, *lightening)


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
    Foot end: a small socket that the foot pad presses or screws into.
    """
    spar = _box((TIBIA_LENGTH, LINK_THICKNESS, TIBIA_SPAR_H),
                center=(TIBIA_LENGTH / 2.0, 0, 0))

    knee_pad = _box((2 * HIP_PAD_R, LINK_THICKNESS, 2 * HIP_PAD_R),
                    center=(0, 0, 0))

    knee_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(SERVO_TAB_HOLE / 2.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              0,
                              HORN_BOLT_PCD / 2.0 * np.sin(a)])
        knee_holes.append(h)

    # Foot socket at the far end -- a short cylinder pointing in -Z
    # (down, when the tibia is in stance pose) that the foot pad's
    # hub plugs into.  Centred on the spar's far end x = TIBIA_LENGTH.
    foot_socket = _cyl(FOOT_HUB_OD / 2.0 + 1.5,
                       FOOT_HUB_HEIGHT + 4.0)
    foot_socket.apply_translation([TIBIA_LENGTH - 4.0, 0,
                                    -(FOOT_HUB_HEIGHT + 4.0) / 2.0])

    foot_bore = _cyl(FOOT_HUB_OD / 2.0 + 0.4,
                     FOOT_HUB_HEIGHT + 6.0)
    foot_bore.apply_translation([TIBIA_LENGTH - 4.0, 0,
                                   -(FOOT_HUB_HEIGHT + 6.0) / 2.0 + 1.5])

    # A short taper to blend the spar into the foot socket
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

    body = _union(knee_pad, spar, taper, foot_socket)
    return _diff(body, *knee_holes, foot_bore, *lightening)


def make_foot_pad() -> trimesh.Trimesh:
    """Compliant foot pad.  Print in TPU for grip, or in PLA with a
    cut-out for a section of bicycle inner-tube glued in as a tread.

    Local frame: ground-plane is at Z = 0; the tibia plugs in on +Z."""
    pad_base = _cyl(FOOT_PAD_OD / 2.0, 4.0)
    pad_base.apply_translation([0, 0, 2.0])

    shoulder = _cyl(FOOT_PAD_OD * 0.42, 6.0)
    shoulder.apply_translation([0, 0, 4.0 + 3.0])

    hub = _cyl(FOOT_HUB_OD / 2.0, FOOT_HUB_HEIGHT)
    hub.apply_translation([0, 0, 4.0 + 6.0 + FOOT_HUB_HEIGHT / 2.0])

    return _union(pad_base, shoulder, hub)


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
    # tip is at (COXA_LENGTH, 0, hip_drop) where hip_drop =
    # -(WELL_D/2 + arm_t/2) (the coxa link's well was dropped below
    # the arm in Z so the servo body hangs cleanly under the link plate).
    # Must match the arm_t used in make_coxa_link (currently 6 mm).
    arm_t = 6.0
    hip_drop = -(WELL_D / 2.0 + arm_t / 2.0)

    hip_joint_local = np.array([COXA_LENGTH, 0.0, hip_drop])

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
    Ry_pt = rotation_matrix(pt, [0, 1, 0])[:3, :3]
    foot_local = knee_joint_local + Ry_pt @ np.array([TIBIA_LENGTH,
                                                       0.0, 0.0])
    R_a = rotation_matrix(a, [0, 0, 1])[:3, :3]
    foot_world = R_a @ foot_local + edge_mid + yaw_output_z * z_hat

    foot = make_foot_pad()
    # Foot's local +Z=top hub at z = 4 + 6 + FOOT_HUB_HEIGHT
    FOOT_TOP_Z = 4.0 + 6.0 + FOOT_HUB_HEIGHT
    foot.apply_translation([foot_world[0], foot_world[1],
                             foot_world[2] - FOOT_TOP_Z])
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
