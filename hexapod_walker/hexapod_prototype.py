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
WELL_FLOOR_T = 2.5   # mm bottom-plate thickness (the servo body rests on this)
WELL_W       = SERVO_BODY_W + 2 * WELL_WALL_X     # 58 mm
WELL_D       = SERVO_BODY_D + 2 * WELL_WALL_Y     # 25 mm
WELL_RIM_Z   = SERVO_TAB_Z - SERVO_TAB_T / 2.0    # 25.75 mm: rim sits at the
                                                   #          bottom face of the tab
WELL_H       = WELL_RIM_Z + WELL_FLOOR_T          # 28.25 mm: outer height
WELL_BODY_CL = 0.4   # mm clearance on every body face inside the well

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
HORN_ADAPTER_OD     = 28.0   # mm -- max OD (arm tip-to-tip)
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
FEMUR_SPAR_H     = 30.0   # mm -- Z-direction height of the femur spar
                          # (must be > SERVO_BODY_D + 8 so the spar's top
                          # and bottom flanges remain after we cut a slot
                          # through it for the knee servo's body to slide
                          # through during assembly).
TIBIA_SPAR_H     = 18.0   # mm -- Z-direction height of the tibia spar
HIP_PAD_R        = HORN_BOLT_PCD / 2.0 + 5.0   # 17 mm -- pad radius to
                                                # comfortably contain
                                                # the 24 mm bolt PCD

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

    # Body cavity: spans z = [0.0, WELL_RIM_Z] EXACTLY so it cleanly
    # cuts through the outer box's top face -- giving a true open-top
    # bucket.  Don't overshoot the outer box: that creates a degenerate
    # boundary that confuses trimesh.boolean.difference.  The body
    # rests on the FLOOR (which spans z = [-WELL_FLOOR_T, 0]).
    cav_h = WELL_RIM_Z
    cavity = _box((SERVO_BODY_W + 2 * WELL_BODY_CL,
                   SERVO_BODY_D + 2 * WELL_BODY_CL,
                   cav_h),
                  center=(0, 0, cav_h / 2.0))

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

    return _diff(outer, cavity, *pilots)


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


def make_servo_horn_adapter() -> trimesh.Trimesh:
    """A small 4-arm star adapter that bolts to a standard plastic
    servo horn (single M3 centre screw + 4 x M2.5 horn-screw holes
    that we don't model) and presents a flat 4 x M3 bolt pattern on
    its top face.

    Local frame:
        Z axis = servo output axis
        Origin at the bottom face (mating to the horn)
        Arms point at 0, 90, 180, 270 degrees
    """
    # Square plate body
    plate = _box((HORN_ADAPTER_OD, HORN_ADAPTER_OD, HORN_ADAPTER_T),
                 center=(0, 0, HORN_ADAPTER_T / 2.0))

    # 4 lightening cuts to leave a 4-arm cross
    light = []
    arm_w = 8.0
    cut_w = (HORN_ADAPTER_OD - arm_w) / 2.0 + 1.0
    cut_l = HORN_ADAPTER_OD * 0.6
    for sx in (-1, 1):
        c = _box((cut_w, cut_l, HORN_ADAPTER_T * 3),
                 center=(sx * (arm_w / 2.0 + cut_w / 2.0), 0,
                          HORN_ADAPTER_T / 2.0))
        light.append(c)
    for sy in (-1, 1):
        c = _box((cut_l, cut_w, HORN_ADAPTER_T * 3),
                 center=(0, sy * (arm_w / 2.0 + cut_w / 2.0),
                          HORN_ADAPTER_T / 2.0))
        light.append(c)

    # Recess for the plastic horn body (round counter-bore on -Z side)
    recess = _cyl(HORN_RECESS_OD / 2.0, HORN_RECESS_DEPTH * 2)
    # Centre on z = 0 so half of it sits below the plate (counter-bore);
    # since the plate spans z in [0, T], we want the recess to span
    # z in [0, HORN_RECESS_DEPTH].
    recess.apply_translation([0, 0, HORN_RECESS_DEPTH])

    # Centre clearance for the horn-attach M3 screw
    centre = _cyl(HORN_CENTRE_OD / 2.0, HORN_ADAPTER_T * 4)

    # 4 x M3 holes on the bolt circle, oriented at 45° between arms
    bolts = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(HORN_BOLT_OD / 2.0, HORN_ADAPTER_T * 4)
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              HORN_BOLT_PCD / 2.0 * np.sin(a),
                              0])
        bolts.append(h)

    return _diff(plate, *light, recess, centre, *bolts)


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
    # 8 mm, so it overlaps with the well's inboard wall material.
    rib = _box((10.0, BRACKET_FLANGE_Y, 6.0),
               center=(-5.0, 0.0, 0.0))   # spans z in [-3, 3], x in [-10, 0]

    body = _union(flange, well, rib)
    return _diff(body, slot, *chassis_holes)


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
    arm_t =  4.0   # mm, along Z (printed flat against the build plate)

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
    well = _servo_well_solid()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])  # well +Z -> link +Y
    well.apply_transform(R)
    # Output spline tip in well-local: (SERVO_OUTPUT_X, 0,
    #   SERVO_BODY_H + SERVO_OUTPUT_H) = (10, 0, 44).
    # After R: (10, 44, 0).  We want it at (COXA_LENGTH, 0, 0) (the
    # joint axis position in the link frame, on the arm centreline).
    delta = np.array([COXA_LENGTH - SERVO_OUTPUT_X,
                       -(SERVO_BODY_H + SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)
    # Drop the well in -Z so it hangs below the arm rather than
    # interpenetrating it.
    well_z_drop = -(WELL_D / 2.0 + arm_t / 2.0)
    well.apply_translation([0.0, 0.0, well_z_drop])

    # Stiffening gusset over the +X end of the arm, on top of the well.
    gusset = _box((30.0, arm_w, arm_t),
                  center=(COXA_LENGTH - 14.0, 0, arm_t / 2.0))

    # Bridge from the arm's -Y edge (y = -arm_w/2) down to the well's
    # near +Y face (y = WELL_RIM_Z + delta_y) and from the arm's bottom
    # face (z = 0) down to the well's top face (z = well_z_drop +
    # WELL_D/2).  Without this the well dangles >5 mm away from the
    # arm in both Y and Z.
    arm_minus_y_edge = -arm_w / 2.0
    well_near_y      = WELL_RIM_Z + delta[1]                    # ~ -18.25
    well_top_z       = well_z_drop + WELL_D / 2.0               # ~ -2.0
    bridge_y_min = well_near_y - 0.5                            # 0.5 mm overlap into well
    bridge_y_max = arm_minus_y_edge + 0.5                       # 0.5 mm overlap into arm
    bridge_y_extent = bridge_y_max - bridge_y_min
    bridge_y_centre = (bridge_y_min + bridge_y_max) / 2.0
    bridge_z_min = well_top_z - 0.5                             # overlap into well wall
    bridge_z_max = arm_t                                        # up to arm top
    bridge_z_extent = bridge_z_max - bridge_z_min
    bridge_z_centre = (bridge_z_min + bridge_z_max) / 2.0
    bridge = _box((30.0, bridge_y_extent, bridge_z_extent),
                  center=(COXA_LENGTH - SERVO_OUTPUT_X,
                           bridge_y_centre, bridge_z_centre))

    body = _union(hub, arm, well, gusset, bridge)
    return _diff(body, *hub_holes, centre_hole)


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

    # Insertion slot for the knee servo's body.  The body's footprint
    # at the well's location is x in [knee_x - 20, knee_x + 20],
    # z in [-10, +10].  We cut a slot through the spar that is wider
    # than that footprint so the body slides in cleanly.
    body_x_centre = FEMUR_LENGTH - SERVO_OUTPUT_X
    body_x_min = body_x_centre - SERVO_BODY_W / 2.0 - 1.0   # 1 mm clearance
    body_x_max = body_x_centre + SERVO_BODY_W / 2.0 + 1.0
    slot_x = body_x_max - body_x_min                         # 42 mm
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
    well = _servo_well_solid()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])    # well +Z -> femur +Y
    well.apply_transform(R)
    delta = np.array([FEMUR_LENGTH - SERVO_OUTPUT_X,
                       -(SERVO_BODY_H + SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)

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
    bridge_y_max    = spar_far_y + 0.5
    bridge_y_extent = bridge_y_max - bridge_y_min     # 22 mm
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
    # Only at x < body_x_min so they don't accidentally cut the slot
    # walls or the bridges.
    lightening = []
    n_holes = 2
    for i in range(n_holes):
        x = (i + 1) * body_x_min / (n_holes + 1)
        h = _cyl(5.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([x, 0, 0])
        lightening.append(h)

    body = _union(hip_pad, spar, well, bridge_top, bridge_bot)
    return _diff(body, insertion_slot, *hip_holes, *lightening)


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
        h = _cyl(4.0, LINK_THICKNESS * 4)
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
    arm_t = 4.0
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
