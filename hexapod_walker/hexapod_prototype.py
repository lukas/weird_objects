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
SERVO_TAB_HOLE    =  3.2   # mm -- M3 clearance hole in each tab
SERVO_TAB_HOLE_PCD = 49.5  # mm -- centre-to-centre distance between tab holes
                            #      (along the X axis; 2 holes per tab x 2 tabs = 4 holes)
SERVO_TAB_Z       = 27.0   # mm -- height of the tab plane above the body bottom
                            #      (DS3225/MG996R: tabs are ~10 mm down from the top face)

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
FEMUR_SPAR_H     = 22.0   # mm -- Z-direction height of the femur spar
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
    """Return a flat hexagonal plate, centred on origin, axis = +Z."""
    apothem = flat_to_flat / 2.0
    circum = apothem / np.cos(np.pi / 6)
    plate = _cyl(circum, thickness, sections=6)
    plate.apply_transform(rotation_matrix(np.pi / 6, [0, 0, 1]))

    # 6 x coxa-bracket bolt patterns: 4 holes per bracket on a 30 x 16 mm
    # rectangle, drilled vertically through the plate.
    holes = []
    for i in range(6):
        a = (i + 0.5) * np.pi / 3
        # The coxa bracket sits with its centre near the apothem line
        # (just inboard of the perimeter).  Bolt pattern centred on
        # (apothem - 12, 0) in the bracket's local frame; rotate that
        # into world.
        cx = (apothem - 14.0) * np.cos(a)
        cy = (apothem - 14.0) * np.sin(a)
        R = rotation_matrix(a, [0, 0, 1])[:3, :3]
        for sx in (-1, 1):
            for sy in (-1, 1):
                local = np.array([sx * 14.0, sy * 8.0, 0.0])
                world = np.array([cx, cy, 0.0]) + R @ local
                h = _cyl(SERVO_TAB_HOLE / 2.0, thickness * 4)
                h.apply_translation([world[0], world[1], 0])
                holes.append(h)

    if with_centre_holes:
        # 4 holes for the electronics tray + battery holder mounting
        for i in range(4):
            a = np.pi / 4 + i * np.pi / 2
            h = _cyl(SERVO_TAB_HOLE / 2.0, thickness * 4)
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
        +Z = yaw axis (output shaft)
        +X = outboard (away from chassis centre)
        +Y = tangential

    Construction:
        - Mounting flange (in the YZ plane, bolted to the chassis pad)
        - U-shaped servo cradle — the servo body slides into the
          U-slot from the +Z side; the tab plane bolts to the bracket
          on both +/-X faces
    """
    # Mounting pads (4 holes on a 28 x 16 rectangle, matching the
    # chassis-side bolt pattern).  Pad is a flat plate flush with the
    # bracket's inboard face.
    PAD_T = 4.0
    PAD_W = 36.0   # along Y
    PAD_H = 22.0   # along Z

    pad = _box((PAD_T, PAD_W, PAD_H),
               center=(-PAD_T / 2.0, 0, 0))

    # 4 chassis-side mounting holes (axis = X)
    chassis_holes = []
    for sy in (-1, 1):
        for sz in (-1, 1):
            h = _cyl(SERVO_TAB_HOLE / 2.0, PAD_T * 6)
            h.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
            h.apply_translation([-PAD_T / 2.0, sy * 14.0, sz * 8.0])
            chassis_holes.append(h)

    # Servo cradle.  The yaw servo sits with its output shaft pointing
    # +Z, body extending downward in -Z.  Servo's local +X (output offset
    # direction) aligns with the leg's +X (outboard).
    # Cradle envelope: a box big enough to contain the servo body +
    # mounting tabs + a 2 mm wall.
    WALL = 2.5
    cradle_w = SERVO_BODY_W + 2 * WALL
    cradle_d = SERVO_BODY_D + 2 * WALL
    cradle_h = SERVO_BODY_H + 2 * WALL  # capped on the bottom

    # Position the cradle so the servo's output shaft lands at
    # (COXA_LENGTH, 0, OUTPUT_PLANE_Z).  We place the servo body's
    # output face (= top of body) in the chassis's mid-plane (Z = 0),
    # so the servo's centre is below the chassis.
    # Servo bottom is at z = -SERVO_BODY_H, top at z = 0.
    # Servo's output shaft is offset by SERVO_OUTPUT_X in +X from the
    # body centre; place body centre at x = -SERVO_OUTPUT_X so the
    # output shaft lands at x = 0 in the bracket's local frame.
    servo_centre = np.array([-SERVO_OUTPUT_X, 0.0, -SERVO_BODY_H / 2.0])
    cradle_centre = servo_centre.copy()

    cradle = _box((cradle_w, cradle_d, cradle_h),
                  center=(cradle_centre[0], cradle_centre[1],
                           cradle_centre[2]))
    # Open the top so the output gear pokes through.
    top_open = _box((cradle_w + 1, cradle_d + 1, WALL * 1.6),
                    center=(cradle_centre[0], cradle_centre[1],
                             cradle_centre[2] + cradle_h / 2.0
                                 - WALL * 0.7))

    # Web from the pad to the cradle for stiffness
    web = _box((PAD_T + cradle_centre[0] - (-cradle_w / 2.0
                                              + PAD_T / 2.0)
                  + 0.1,
                cradle_d * 0.7, PAD_H),
               center=((-PAD_T / 2.0
                          + (cradle_centre[0] + (-cradle_w / 2.0))) / 2.0
                          + PAD_T,
                        0, 0))

    body = _union(pad, cradle, web)

    # Cut the servo pocket out of the cradle.  The pocket function
    # assumes the servo's body bottom is at z=0 in its local frame, so
    # translate accordingly: servo bottom is at z = -SERVO_BODY_H, and
    # the servo's local origin is its body centre at servo_centre.
    pocket = _servo_pocket()
    # Translate so the servo's local frame matches: pocket's local
    # origin is body bottom centre.  Move it to the servo's bottom
    # centre in bracket coords.
    pocket.apply_translation([servo_centre[0],
                                servo_centre[1],
                                servo_centre[2] - SERVO_BODY_H / 2.0])

    return _diff(body, *chassis_holes, pocket, top_open)


def make_coxa_link() -> trimesh.Trimesh:
    """Coxa link: a horizontal U-bracket, driven by the yaw servo's
    horn, that carries the hip-pitch servo at its outboard end.

    Local frame:
        +Z = yaw axis (downward at the hub centre)
        +X = arm direction (outboard at neutral pose)
        +Y = hip-pitch joint axis

    Bottom face of the link (z = 0) sits on the yaw servo horn (via
    `servo_horn_adapter.stl`).  The hip-pitch servo hangs in -Z below
    the link's outboard end, with its output shaft pointing along
    -Y_local (so the femur swings in the X-Z plane).
    """
    arm_w = 22.0   # mm, along Y
    arm_t =  4.0   # mm, along Z (printed flat against the build plate)

    # Hub region (above the yaw servo horn) -- a thicker square
    hub = _box((34.0, 34.0, arm_t + 2.0),
               center=(0, 0, (arm_t + 2.0) / 2.0))
    # Bolt pattern matching the horn adapter
    hub_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(SERVO_TAB_HOLE / 2.0, (arm_t + 2.0) * 4)
        h.apply_translation([HORN_BOLT_PCD / 2.0 * np.cos(a),
                              HORN_BOLT_PCD / 2.0 * np.sin(a),
                              (arm_t + 2.0) / 2.0])
        hub_holes.append(h)

    # Arm reaching out to the hip-pitch motor mount
    arm = _box((COXA_LENGTH + 28.0, arm_w, arm_t),
               center=((COXA_LENGTH + 28.0) / 2.0 - 12.0, 0,
                        arm_t / 2.0))

    # Servo cradle at the outboard end.  The hip-pitch servo's output
    # shaft points in -Y_local; the servo body hangs below the arm in
    # -Z.  Servo's local axes mapped to leg-local:
    #     servo +Z -> leg -Y
    #     servo +X -> leg +X (output offset toward outboard)
    #     servo +Y -> leg -Z (body extends downward)
    # We model the cradle in the servo's local frame, then transform.
    WALL = 2.5
    cradle_w = SERVO_BODY_W + 2 * WALL
    cradle_d = SERVO_BODY_D + 2 * WALL
    cradle_h = SERVO_BODY_H + 2 * WALL

    cradle = _box((cradle_w, cradle_d, cradle_h),
                  center=(0, 0, cradle_h / 2.0 - WALL))
    pocket = _servo_pocket()  # servo's local frame: +Z = output
    # Align: rotate cradle so servo's +Z (the output direction) becomes
    # the bracket's -Y_local.  That's a -90 deg rotation about +X.
    R_servo_to_bracket = rotation_matrix(np.pi / 2.0, [1, 0, 0])
    cradle.apply_transform(R_servo_to_bracket)
    pocket.apply_transform(R_servo_to_bracket)

    # Place the cradle so the servo output shaft is at:
    # (COXA_LENGTH, 0, 0) + (-SERVO_OUTPUT_X) along the servo's +X dir.
    # In bracket coords the servo's +X is still +X (rotation was about
    # X only).  So translate so that the servo's output point (in its
    # local frame: x=SERVO_OUTPUT_X, y=0, z=SERVO_BODY_H) maps to
    # (COXA_LENGTH, 0, 0).  After rotation about +X by +90 deg:
    #   servo (X, Y, Z) -> bracket (X, -Z, Y)
    # So servo output local (SERVO_OUTPUT_X, 0, SERVO_BODY_H) maps to
    # bracket (SERVO_OUTPUT_X, -SERVO_BODY_H, 0).  We want that at
    # (COXA_LENGTH, 0, 0), so translate by:
    delta = np.array([COXA_LENGTH - SERVO_OUTPUT_X,
                       0 - (-SERVO_BODY_H),
                       0 - 0])
    cradle.apply_translation(delta)
    pocket.apply_translation(delta)

    # Drop the cradle a bit so its top sits on the arm's bottom face
    # (no -- already aligned: the top of the cradle in bracket coords
    # is the y_max face which is at y = SERVO_BODY_H + delta_y =
    # SERVO_BODY_H - SERVO_BODY_H = 0, perfect; output centerline is
    # at y = 0).
    # Actually we want the servo body to hang BELOW the arm (in -Z),
    # not be coplanar.  Re-think: the U-bracket arm is at z=0 to
    # arm_t.  The servo cradle should be at -Z from there.  Currently
    # the cradle's top face is at y_bracket = ?  Let me trace:
    # After rotation+translate: cradle box is centred at (0, cradle_h/2 - WALL, 0)
    # in pre-translation coords, then translated by delta = (COXA_L -
    # OFFX, +BODY_H, 0).  Original cradle in servo coords spans:
    # x in [-cradle_w/2, cradle_w/2], y in [-cradle_d/2, cradle_d/2],
    # z in [-WALL, cradle_h - WALL].  After R_servo_to_bracket
    # (rotation +90 about X): (x,y,z) -> (x, -z, y).  So bracket-space
    # bounds are:
    #   x in [-cradle_w/2, cradle_w/2]
    #   y_bracket in [-(cradle_h - WALL), -(-WALL)] = [-(cradle_h-WALL), WALL]
    #   z_bracket in [-cradle_d/2, cradle_d/2]
    # Then add delta = (COXA_L - OFFX, +BODY_H, 0):
    #   y_bracket in [BODY_H - (cradle_h-WALL), BODY_H + WALL]
    #             = [BODY_H - BODY_H - 2*WALL + WALL, BODY_H + WALL]
    #             = [-WALL, BODY_H + WALL]
    # Hmm that's wrong -- the cradle ends up on the +Y side, not -Y.
    # Let me redo.

    # Actually we want the hip-pitch servo's output shaft to lie along
    # -Y_local (so a positive servo rotation lifts the femur).  Then
    # the body extends in +Y_local from the output.  That places the
    # servo on the "+Y side" of the leg.  Hmm but the hexapod_walker
    # convention puts everything on the -Y side...
    #
    # Let's actually look at what hexapod_walker does for the same
    # joint.  Refer to make_coxa_link in HW: the hip-pitch plate sits
    # on the -Y side of the arm at x=COXA_LENGTH, the motor extends
    # in -Y from that plate.  So the motor body is on -Y.  To match
    # that here, the hobby servo's output should point along +Y_local
    # (output face at y_local = plate_y), with the body extending in
    # -Y_local.
    #
    # Need to rotate the servo so its +Z (output) -> bracket +Y, body
    # extends to -Y in bracket frame.  That's a -90 about +X (rather
    # than +90).  Let me redo:

    # Reset
    cradle = _box((cradle_w, cradle_d, cradle_h),
                  center=(0, 0, cradle_h / 2.0 - WALL))
    pocket = _servo_pocket()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    cradle.apply_transform(R)
    pocket.apply_transform(R)
    # After R: servo (X, Y, Z) -> bracket (X, +Z, -Y).  So servo's +Z
    # output direction maps to bracket's +Y, perfect.
    # Servo output local (SERVO_OUTPUT_X, 0, SERVO_BODY_H) ->
    # bracket (SERVO_OUTPUT_X, SERVO_BODY_H, 0).
    # We want output at (COXA_LENGTH, 0, 0)?  Wait -- looking at the
    # full-size walker the hip-pitch joint plane is at y =
    # plate_y_coxa, NOT at y = 0.  The plate sits below the arm
    # (negative Y).  But for our hobby setup, the servo cradle BECOMES
    # the "plate" -- the femur clamps to the servo's output via the
    # horn adapter.  Let me just place the joint at y = 0 (the arm's
    # centreline) with the servo body extending in -Y from there.
    #
    # So servo output point at (COXA_LENGTH, 0, 0).  Translate by
    # (COXA_LENGTH - SERVO_OUTPUT_X, -SERVO_BODY_H, 0):
    delta = np.array([COXA_LENGTH - SERVO_OUTPUT_X,
                       -SERVO_BODY_H, 0])
    cradle.apply_translation(delta)
    pocket.apply_translation(delta)
    # Now the cradle's bracket-y span is:
    #   y_bracket (pre-translation) in [-cradle_h+WALL, WALL]
    #   plus delta_y = -SERVO_BODY_H = -(cradle_h - 2*WALL):
    #   y in [-(cradle_h - WALL) - (cradle_h - 2*WALL),
    #         WALL - (cradle_h - 2*WALL)]
    #     = [-2*cradle_h + 3*WALL, 3*WALL - cradle_h]
    # For SERVO_BODY_H = 38, cradle_h = 43, WALL = 2.5:
    #   y in [-86+7.5, 7.5-43] = [-78.5, -35.5].  All on -Y side, good.

    # Lower the cradle by half the arm thickness so its top isn't
    # poking up through the arm (cradle's top face at y = ~ -35.5 is
    # safely below z=arm_t/2 = 2 so this is fine; also the cradle
    # needs to sit BELOW the arm in Z so it's drooping).  Actually we
    # need to also drop the cradle in Z since the arm is at z=[0,
    # arm_t] and the cradle is centred at z=0 right now.  Drop by
    # cradle_d/2 + arm_t/2 to put the cradle's top face flush with
    # the arm's bottom face:
    drop = -(cradle_d / 2.0 + arm_t / 2.0)
    cradle.apply_translation([0, 0, drop])
    pocket.apply_translation([0, 0, drop])

    # Triangular gusset bridging the arm to the cradle (visual stiffener)
    gusset = _box((30.0, arm_w, arm_t),
                  center=(COXA_LENGTH - 14.0, 0, arm_t / 2.0))

    body = _union(hub, arm, cradle, gusset)
    return _diff(body, *hub_holes, pocket)


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

    Knee end: a U-shaped servo cradle that holds the knee servo body,
    output shaft pointing +Y (parallel to the hip-pitch axis), so the
    tibia can be horn-driven and rotates in the same plane the femur
    does.

    The femur is a flat plate -- LINK_THICKNESS thick in Y -- that
    lies in the leg's X-Z motion plane.  Both the hip pad and the
    knee servo cradle live in the spar's Y-extent so the whole part
    prints flat on the build plate.
    """
    # Spar -- flat plate, X long-axis, Y is plate thickness, Z is
    # plate height (the bending-stiffness axis).
    spar = _box((FEMUR_LENGTH, LINK_THICKNESS, FEMUR_SPAR_H),
                center=(FEMUR_LENGTH / 2.0, 0, 0))

    # Hip-end pad: a square plate centred ON THE JOINT AXIS (x=0,
    # z=0).  Same Y thickness as the spar.  The bolt-circle CENTRE
    # must sit on the joint axis so the femur rotates rigidly with
    # the horn -- otherwise the bolts would trace a circle around
    # the spline as the femur swings, and they'd shear off.
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

    # Knee-end servo cradle.  Knee servo's output shaft points +Y so
    # the tibia clamped to the horn swings in the X-Z plane.  The
    # cradle sits OFF the spar in -Y so the servo body hangs to one
    # side of the link plate (away from the +Y face where the horn
    # lives -- so the tibia, when bolted on, doesn't collide with
    # the body).
    WALL = 2.5
    cradle_w = SERVO_BODY_W + 2 * WALL
    cradle_d = SERVO_BODY_D + 2 * WALL
    cradle_h = SERVO_BODY_H + 2 * WALL

    cradle = _box((cradle_w, cradle_d, cradle_h),
                  center=(0, 0, cradle_h / 2.0 - WALL))
    pocket = _servo_pocket()
    R = rotation_matrix(-np.pi / 2.0, [1, 0, 0])  # servo +Z -> +Y
    cradle.apply_transform(R)
    pocket.apply_transform(R)
    # Translate so the servo output point (post-R, at
    # (SERVO_OUTPUT_X, SERVO_BODY_H, 0)) lands on the knee joint axis
    # at (FEMUR_LENGTH, 0, 0) -- i.e. on the spar centreline at the
    # spar's far end.
    delta = np.array([FEMUR_LENGTH - SERVO_OUTPUT_X,
                       -SERVO_BODY_H, 0])
    cradle.apply_translation(delta)
    pocket.apply_translation(delta)

    # Lightening holes through the spar (drilled along Y)
    lightening = []
    n_holes = 3
    for i in range(n_holes):
        x = (i + 1) * FEMUR_LENGTH / (n_holes + 1)
        h = _cyl(5.0, LINK_THICKNESS * 4)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([x, 0, 0])
        lightening.append(h)

    body = _union(hip_pad, spar, cradle)
    return _diff(body, *hip_holes, pocket, *lightening)


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
    # The coxa link's bottom face (z = 0 in its local frame) sits on
    # top of the yaw servo's output gear, which is at z =
    # SERVO_OUTPUT_H above the coxa bracket's reference (which itself
    # is at z = 0 = chassis edge plane).  Add a small horn-adapter
    # height too.
    yaw_output_z = SERVO_OUTPUT_H + HORN_ADAPTER_T
    cl = make_coxa_link()
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts.append(cl)

    # ------------------- Femur (pitched about leg-Y) ------------------
    # In coxa-link local coords, the hip-pitch servo's output is at
    # (COXA_LENGTH, 0, hip_drop) where hip_drop = -(cradle_d/2 + arm_t/2)
    # (the coxa link's cradle was dropped below the arm in Z so the
    # servo body hangs cleanly under the link plate).
    arm_t = 4.0
    WALL = 2.5
    cradle_d = SERVO_BODY_D + 2 * WALL
    hip_drop = -(cradle_d / 2.0 + arm_t / 2.0)

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
