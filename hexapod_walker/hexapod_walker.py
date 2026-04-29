"""Hexapod walker — STL generator for a human-carrying six-legged vehicle.

Outputs (in ./stl/):

    Body parts (one each)
        chassis_hex.stl        -- main hexagonal welded-tube chassis frame
        chassis_top_deck.stl   -- bolt-on top deck plate (rider floor)
        saddle_mount.stl       -- rider saddle / harness post
        battery_box.stl        -- battery + BMS enclosure
        electronics_bay.stl    -- main controller + drive electronics bay

    Per-leg parts (one of each — print/cast 6 sets)
        coxa_bracket.stl       -- bolts to chassis, carries the hip-yaw actuator
        coxa_link.stl          -- rotates with the hip-yaw motor; carries hip-pitch motor
        femur_link.stl         -- thigh: hip-pitch hub  ->  knee-pitch hub
        tibia_link.stl         -- shin: knee-pitch hub  ->  foot
        foot_pad.stl           -- compliant ground-contact pad

    Generic (one — print 18 for the joints, plus spares)
        motor_flange.stl       -- universal D=150 mm flange-mount adapter
                                  for an industrial harmonic-drive servo

    Assembly preview (all parts in their standing pose, for visualization)
        assembly_preview.stl

Geometry overview
-----------------

    Six identical legs spaced 60 deg around a hexagonal body.
    Each leg has 3 rotary DOF, identical to a real-world insect leg:

         coxa  (hip yaw,    vertical axis)            -- swings the leg fore/aft
         femur (hip pitch,  horizontal radial axis)   -- raises/lowers the thigh
         tibia (knee pitch, horizontal radial axis)   -- folds the shin

    18 actuators total.  Each joint is sized for an industrial
    harmonic-drive or cycloidal-reducer servo with a flat output flange
    (D ~= 150 mm bolt circle, T_continuous ~= 400 N*m).  See ASSEMBLY.md
    for specific motor recommendations.

The script is parameter-driven — edit the SCALE / GEOMETRY block below
to adjust load-bearing capacity, reach, or stance height.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import trimesh
from trimesh.creation import box as box_mesh
from trimesh.creation import cylinder as cylinder_mesh
from trimesh.creation import annulus as annulus_mesh
from trimesh.transformations import rotation_matrix, translation_matrix


# ---------------------------------------------------------------------------
# Geometry constants  (everything in millimetres)
# ---------------------------------------------------------------------------

# ---- Vehicle envelope -----------------------------------------------------
# A regular hexagonal chassis 1200 mm across the flats.  Six legs attach
# at the six edges (not vertices), pointing radially outward.  At nominal
# stance the foot pads sit on a circle of radius ~ 1100 mm, so the
# vehicle's outer diameter (foot to foot) is ~ 2.2 m -- comfortable
# clearance for one seated rider with a full leg-swing envelope.
CHASSIS_FLAT_TO_FLAT  = 1200.0   # mm -- distance between opposite hex edges
CHASSIS_HEIGHT        = 220.0    # mm -- vertical depth of the chassis frame
CHASSIS_TUBE          =  80.0    # mm -- square steel tube cross-section
CHASSIS_TUBE_WALL     =   6.0    # mm -- tube wall thickness (visualised)

# ---- Leg link lengths -----------------------------------------------------
# Anatomical proportions roughly mimic a Phidippus jumping spider:
#   coxa  : femur : tibia  ~=  1 : 4 : 5
# Long tibia gives a wide stride at modest knee torque.  Coxa is short
# because it only has to carry the hip-pitch axis outboard of the
# hip-yaw axis -- not used for forward reach.
COXA_LENGTH    = 150.0   # mm -- hip-yaw hub center  ->  hip-pitch hub center
FEMUR_LENGTH   = 600.0   # mm -- hip-pitch hub       ->  knee hub
TIBIA_LENGTH   = 800.0   # mm -- knee hub            ->  foot tip

# ---- Motor (actuator) housing ---------------------------------------------
# Modelled after an industrial harmonic-drive servo of the
# Harmonic Drive(R) FHA-32C / Nidec(R) AccuDrive class:
#   outer flange diameter  ~ 170 mm
#   bolt-circle diameter   ~ 150 mm
#   total axial length     ~ 130 mm (motor + reducer + brake)
# Continuous torque ~ 400 N*m, peak ~ 850 N*m.  At the knee that yields a
# safety factor ~ 3 against the static stance load (see ASSEMBLY.md s.7).
MOTOR_OD          = 170.0   # mm -- flange OD (outer cylinder of housing)
MOTOR_OUTPUT_OD   = 100.0   # mm -- output-flange OD (the bit that rotates)
MOTOR_BOLT_CIRCLE = 150.0   # mm -- 8 x M8 bolts on a 150 mm PCD
MOTOR_LENGTH      = 130.0   # mm -- axial length, motor side  +  reducer side
MOTOR_BOLT_OD     =   9.0   # mm -- M8 clearance hole (Φ 9.0 mm)
MOTOR_BOLT_COUNT  =   8

# ---- Link cross-sections --------------------------------------------------
# Femur and tibia are 6061-T6 aluminum extrusions ("I-beam-ish" rectangular
# tube).  The 3D-printed (or cast) end caps modelled here bolt onto the
# extrusion ends and carry the motor flange.  We model the link as a
# single solid box of the extrusion outline plus the end-cap details.
FEMUR_TUBE_W   =  90.0   # mm -- wide face of the femur extrusion
FEMUR_TUBE_H   = 120.0   # mm -- tall face of the femur extrusion (load axis)
TIBIA_TUBE_W   =  70.0   # mm -- shin extrusion width
TIBIA_TUBE_H   =  90.0   # mm -- shin extrusion height
LINK_END_CAP_T =  20.0   # mm -- thickness of the cast/printed end cap
LINK_HUB_T     =  35.0   # mm -- axial thickness of the rotary hub

# ---- Foot ------------------------------------------------------------------
# Wide, low foot pad with a small toe spike for traction on soft ground.
# The pad is moulded from 60-A urethane around a 6 mm steel disc; the
# STL here is the mould-pattern profile only.
FOOT_PAD_OD     = 220.0   # mm -- ground-contact disc OD
FOOT_PAD_HEIGHT =  60.0   # mm -- disc + tapered shoulder
FOOT_HUB_OD     =  90.0   # mm -- attaches to the tibia tip
FOOT_HUB_HEIGHT =  40.0

# ---- Saddle / rider mount --------------------------------------------------
SADDLE_POST_OD     =  40.0
SADDLE_POST_LENGTH = 220.0
SADDLE_PLATE_W     = 320.0
SADDLE_PLATE_D     = 220.0
SADDLE_PLATE_T     =  18.0

# ---- Battery / electronics enclosures --------------------------------------
BATTERY_BOX_W  = 480.0    # mm -- holds 2 x 48 V 50 Ah LiFePO4 packs side-by-side
BATTERY_BOX_D  = 320.0
BATTERY_BOX_H  = 220.0
BATTERY_BOX_WALL =  10.0  # 8 mm cast wall + 2 mm fire-rated liner

ELEC_BAY_W = 360.0
ELEC_BAY_D = 240.0
ELEC_BAY_H = 140.0
ELEC_BAY_WALL = 6.0

# ---- Resolutions -----------------------------------------------------------
CYL_SECTIONS = 48     # cylinder facet count -- smooth STL, fast booleans

# ---- Standing pose used for the assembly preview --------------------------
# Joint angles relative to "neutral":
#     coxa  = 0 deg   (foot directly outboard of hip)
#     femur = -25 deg  (thigh lifted 25 deg above horizontal)
#     tibia = +60 deg  (shin folded 60 deg below the thigh)
STANCE_FEMUR_DEG = -25.0
STANCE_TIBIA_DEG =  60.0

# Output directory -- next to this script
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
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
    """A cylinder lying along the named axis, one end at origin, the other
    at +length on that axis."""
    m = _cyl(radius, length, sections=sections)
    # cylinder is along Z, centred at origin -> shift up so it spans [0, length]
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
        # Fall back to a non-manifold concatenation -- still loads in viewers
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


def _bolt_circle_holes(radius: float, count: int, hole_od: float,
                       depth: float, axis: str = "z") -> trimesh.Trimesh:
    """Return a single mesh that is the union of `count` cylindrical
    bolt-clearance holes on a circle of radius `radius` (centred at the
    origin), each cylinder oriented along `axis`, of full length `depth`,
    centred at zero on the bolt-axis."""
    holes = []
    for i in range(count):
        a = 2 * np.pi * i / count
        m = _cyl(hole_od / 2.0, depth + 1.0, sections=24)
        if axis == "x":
            m.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
        elif axis == "y":
            m.apply_transform(rotation_matrix(-np.pi / 2, [1, 0, 0]))
        # axis == z -> already aligned
        m.apply_translation([radius * np.cos(a), radius * np.sin(a), 0.0])
        holes.append(m)
    return _union(*holes)


def _save(mesh: trimesh.Trimesh, name: str) -> str:
    path = os.path.join(STL_DIR, name)
    mesh.export(path)
    n_faces = len(mesh.faces)
    extents = mesh.extents
    print(f"  wrote stl/{name:32s}"
          f"  {n_faces:>7d} faces"
          f"  envelope {extents[0]:6.0f} x {extents[1]:6.0f} x {extents[2]:6.0f} mm")
    return path


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------

def make_motor_housing() -> trimesh.Trimesh:
    """A representational model of the joint actuator's external envelope.

    Local frame:  output flange face is at +X, motor body extends to -X.
    Output rotation axis is the X axis.  This is the part that spins
    relative to the motor body."""
    # Motor body (the non-rotating part)
    body = _cyl_along(MOTOR_OD / 2.0, MOTOR_LENGTH * 0.7, axis="x")
    body.apply_translation([-MOTOR_LENGTH * 0.7, 0, 0])

    # Output flange (rotating)
    flange = _cyl_along(MOTOR_OUTPUT_OD / 2.0, MOTOR_LENGTH * 0.3, axis="x")
    # spans  x in [-(MOTOR_LENGTH * 0.3) + 0, 0]  ?  shift so it sits in front of body
    # actual: cyl_along with axis=x lays it from x=0 to x=length.
    # We want output flange spanning x in [-MOTOR_LENGTH*0.3, 0]
    flange.apply_translation([-MOTOR_LENGTH * 0.3, 0, 0])

    return _union(body, flange)


def make_motor_flange() -> trimesh.Trimesh:
    """A bolt-on adapter ring that turns a flat plate into a motor mount.

    Geometry (centred at origin, axis = Z):
        - Outer ring D = MOTOR_OD + 30 mm, thickness LINK_END_CAP_T
        - 8 x M8 clearance holes on the MOTOR_BOLT_CIRCLE
        - Central bore D = MOTOR_OUTPUT_OD + 4 mm clearance
    The plate side (-Z) is flat for bolting to the link end cap; the
    motor side (+Z) gets the 8 motor-side bolts.

    This single STL is the same on both ends of every link, so you print
    18 copies (one per joint) plus spares."""
    plate_od = MOTOR_OD + 30.0
    bore_od  = MOTOR_OUTPUT_OD + 4.0
    plate_t  = LINK_END_CAP_T

    plate = _cyl(plate_od / 2.0, plate_t)
    bore = _cyl(bore_od / 2.0, plate_t * 1.5)
    bolts = _bolt_circle_holes(MOTOR_BOLT_CIRCLE / 2.0,
                               MOTOR_BOLT_COUNT, MOTOR_BOLT_OD,
                               plate_t * 1.5)

    # Add 4 mounting tabs to bolt the flange to the link extrusion.
    # Tabs are short ribs sticking radially outward to the link's bolt
    # pattern (4 x M10 on a 200 mm square).
    tab_extent = (40.0, plate_t, plate_t)
    tabs = []
    tab_radius = plate_od / 2.0 + 10.0
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        tab = _box(tab_extent, center=(tab_radius * np.cos(a),
                                       tab_radius * np.sin(a),
                                       0.0))
        tab.apply_transform(rotation_matrix(a, [0, 0, 1],
                                            point=(tab_radius * np.cos(a),
                                                   tab_radius * np.sin(a),
                                                   0.0)))
        tabs.append(tab)

    body = _union(plate, *tabs)
    return _diff(body, bore, bolts)


# ---------------------------------------------------------------------------
# Body parts
# ---------------------------------------------------------------------------

def make_chassis_hex() -> trimesh.Trimesh:
    """The main welded square-tube hexagonal chassis.

    Local frame: chassis-up = +Z, vehicle-forward = +X.  Origin at the
    geometric centre of the hexagon at mid-height of the tube.

    Construction:
        6 x outer perimeter tubes forming a regular hexagon
        6 x radial tubes from each outer-edge midpoint to the centre hub
        1 x central thick-wall hub disc that the rider's column sits on
        6 x coxa-bracket bolt pads on each outer edge midpoint
    """
    # Apothem (centre -> edge midpoint) is half the flat-to-flat dimension.
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0

    # Hex vertex circumradius
    circum = apothem / np.cos(np.pi / 6)

    # ------------------------------------------------------------------
    # Outer perimeter -- 6 tubes between consecutive vertices.
    # Stretched 6 mm at each end so adjacent tubes overlap into a
    # single manifold corner rather than meeting face-to-face.
    # ------------------------------------------------------------------
    OVERLAP = 6.0
    perimeter = []
    for i in range(6):
        a0 = i * np.pi / 3
        a1 = (i + 1) * np.pi / 3
        v0 = np.array([circum * np.cos(a0), circum * np.sin(a0), 0.0])
        v1 = np.array([circum * np.cos(a1), circum * np.sin(a1), 0.0])
        mid = (v0 + v1) / 2.0
        edge_dir = v1 - v0
        edge_len = np.linalg.norm(edge_dir)
        edge_dir = edge_dir / edge_len

        seg = _box((edge_len + 2 * OVERLAP, CHASSIS_TUBE, CHASSIS_TUBE),
                   center=(0, 0, 0))
        # rotate so its long axis aligns with edge_dir (currently +X)
        # angle from +X
        angle = np.arctan2(edge_dir[1], edge_dir[0])
        seg.apply_transform(rotation_matrix(angle, [0, 0, 1]))
        seg.apply_translation(mid)
        perimeter.append(seg)

    # ------------------------------------------------------------------
    # Radial spokes from each edge midpoint to a small central hub.
    # Slightly longer than the apothem so they overlap the central hub
    # and the perimeter tube on both ends.
    # ------------------------------------------------------------------
    spokes = []
    for i in range(6):
        a = (i + 0.5) * np.pi / 3   # edge midpoints sit halfway between vertices
        mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
        seg = _box((apothem + 2 * OVERLAP, CHASSIS_TUBE * 0.9,
                    CHASSIS_TUBE * 0.9))
        seg.apply_translation([apothem / 2.0, 0, 0])
        seg.apply_transform(rotation_matrix(a, [0, 0, 1]))
        spokes.append(seg)

    # ------------------------------------------------------------------
    # Central hub disc -- the rider's saddle column bolts down through here.
    # ------------------------------------------------------------------
    hub = _cyl(180.0, CHASSIS_HEIGHT * 0.55)
    # hub is centred on origin, height direction is Z -- already correct.

    # ------------------------------------------------------------------
    # 6 x coxa-bracket bolt pads on the outer edge midpoints.  Each pad
    # is a 200 mm wide x 100 mm tall flat boss extending outward from
    # the perimeter tube, with 4 x M10 holes for the coxa bracket.
    # ------------------------------------------------------------------
    pads = []
    pad_holes = []
    PAD_RADIAL = 40.0   # mm pad thickness (radial direction)
    PAD_OVERLAP = 8.0   # mm pad sinks inboard into the perimeter tube
    for i in range(6):
        a = (i + 0.5) * np.pi / 3
        mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
        outboard_dir = np.array([np.cos(a), np.sin(a), 0.0])

        # Pad inboard face is 8 mm inside the perimeter tube; pad outboard
        # face sticks out beyond the tube by  PAD_RADIAL - 8 - CHASSIS_TUBE/2.
        pad_center = mid + (CHASSIS_TUBE / 2.0
                            - PAD_OVERLAP
                            + PAD_RADIAL / 2.0) * outboard_dir
        pad = _box((PAD_RADIAL, 200.0, CHASSIS_TUBE * 1.4))
        pad.apply_transform(rotation_matrix(a, [0, 0, 1]))
        pad.apply_translation(pad_center)
        pads.append(pad)

        # 4 x M10 clearance holes (Φ 11) in a 160 x 80 rectangle, drilled
        # radially through the pad.
        for sx in (-1, 1):
            for sy in (-1, 1):
                R = rotation_matrix(a, [0, 0, 1])[:3, :3]
                hole_local = np.array([0.0, 80.0 * sx, 30.0 * sy])
                hole_pos = pad_center + R @ hole_local
                h = _cyl(11.0 / 2.0, PAD_RADIAL * 3, sections=16)
                # cylinder is along Z; rotate so axis is the radial dir
                h.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
                h.apply_transform(rotation_matrix(a, [0, 0, 1]))
                h.apply_translation(hole_pos)
                pad_holes.append(h)

    body = _union(*perimeter, *spokes, hub, *pads)
    return _diff(body, *pad_holes)


def make_chassis_top_deck() -> trimesh.Trimesh:
    """Hex-shaped flat plate that bolts to the top of the chassis frame
    and serves as the rider's footrest / mounting surface for the
    saddle, battery box, and electronics bay."""
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    circum = apothem / np.cos(np.pi / 6)

    # Build the hex outline as a 6-sided prism by intersecting 6 half-spaces.
    plate = _cyl(circum, 12.0, sections=6)  # hex prism, flat-to-flat = 2*apothem along Y if not rotated
    plate.apply_transform(rotation_matrix(np.pi / 6, [0, 0, 1]))

    # Lighten with a hex pattern of holes (visual + weight)
    holes = []
    for r, n in [(150.0, 6), (320.0, 12), (480.0, 18)]:
        for i in range(n):
            a = 2 * np.pi * i / n
            h = _cyl(28.0, 30.0, sections=16)
            h.apply_translation([r * np.cos(a), r * np.sin(a), 0.0])
            holes.append(h)

    return _diff(plate, *holes)


def make_saddle_mount() -> trimesh.Trimesh:
    """Vertical post + flat saddle plate that bolts to the chassis hub.

    Post is a hollow tube (modelled solid for STL) sized for a generic
    bicycle-style suspension seatpost.  The plate accepts the rider's
    seat or harness frame.
    """
    post = _cyl(SADDLE_POST_OD / 2.0, SADDLE_POST_LENGTH)
    post.apply_translation([0, 0, SADDLE_POST_LENGTH / 2.0])

    plate = _box((SADDLE_PLATE_W, SADDLE_PLATE_D, SADDLE_PLATE_T),
                 center=(0, 0, SADDLE_POST_LENGTH + SADDLE_PLATE_T / 2.0))

    base = _cyl(70.0, 18.0)
    base.apply_translation([0, 0, 9.0])

    # 4 mounting holes through the base plate
    base_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(11.0 / 2.0, 30.0, sections=16)
        h.apply_translation([45 * np.cos(a), 45 * np.sin(a), 9.0])
        base_holes.append(h)

    body = _union(base, post, plate)
    return _diff(body, *base_holes)


def make_battery_box() -> trimesh.Trimesh:
    """Walled box for two 48 V LiFePO4 packs, sitting on the top deck.

    Modelled as an open-top box (lid is bolted on -- not part of this STL)
    with a hinged flap modelled as a flat lid notch on +X."""
    outer = _box((BATTERY_BOX_W, BATTERY_BOX_D, BATTERY_BOX_H),
                 center=(0, 0, BATTERY_BOX_H / 2.0))
    inner = _box((BATTERY_BOX_W - 2 * BATTERY_BOX_WALL,
                  BATTERY_BOX_D - 2 * BATTERY_BOX_WALL,
                  BATTERY_BOX_H - BATTERY_BOX_WALL),
                 center=(0, 0, (BATTERY_BOX_H - BATTERY_BOX_WALL) / 2.0
                                 + BATTERY_BOX_WALL))

    # Cooling vents on the long sides
    vents = []
    for s in (-1, 1):
        for i in range(-2, 3):
            v = _box((BATTERY_BOX_WALL * 3.0, 30.0, 8.0),
                     center=(s * (BATTERY_BOX_W / 2.0),
                             i * 60.0,
                             BATTERY_BOX_H * 0.65))
            vents.append(v)

    body = _diff(outer, inner, *vents)
    return body


def make_electronics_bay() -> trimesh.Trimesh:
    """Sealed box for the central controller + 18 motor drivers."""
    outer = _box((ELEC_BAY_W, ELEC_BAY_D, ELEC_BAY_H),
                 center=(0, 0, ELEC_BAY_H / 2.0))
    inner = _box((ELEC_BAY_W - 2 * ELEC_BAY_WALL,
                  ELEC_BAY_D - 2 * ELEC_BAY_WALL,
                  ELEC_BAY_H - ELEC_BAY_WALL),
                 center=(0, 0, (ELEC_BAY_H - ELEC_BAY_WALL) / 2.0
                                 + ELEC_BAY_WALL))

    # Cable-gland penetrations on the +Y face
    glands = []
    for i in range(-3, 4):
        g = _cyl(12.0, ELEC_BAY_WALL * 2.5, sections=24)
        g.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        g.apply_translation([i * 35.0, ELEC_BAY_D / 2.0, ELEC_BAY_H / 2.0])
        glands.append(g)

    return _diff(outer, inner, *glands)


# ---------------------------------------------------------------------------
# Leg parts
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Joint convention (NEW, after the kinematic redesign)
# ---------------------------------------------------------------------------
#
#   - The yaw axis is global +Z (vertical).
#   - All pitch axes (hip, knee) are along the leg's local +Y (tangential
#     to the chassis, perpendicular to the leg's outboard direction).
#   - In every leg, all three motors hang on the leg's -Y_local side; the
#     spars run along +X_local in the y_local = 0 plane.
#   - For each pitch joint, the proximal link's plate bolts to the motor
#     body's mounting flange; the distal link's plate bolts to the motor's
#     output flange.  Both plates are perpendicular to Y, separated by
#     MOTOR_LENGTH along Y.
#
# This is the "two-link planar leg with parallel pitch axes" topology
# you see in real walking robots (Spot, ANYmal, hexapods, etc.).
# ---------------------------------------------------------------------------

JOINT_PLATE_OUTER_R    = (MOTOR_BOLT_CIRCLE / 2.0) + 30.0  # 105 mm
JOINT_PLATE_T          = LINK_END_CAP_T                    # 20 mm thick
# Plate offset: in the link's local frame, the plate centre sits this many
# mm in -Y from the spar's centreline so the plate's +Y face just overlaps
# (welds into) the spar's -Y face.
def _plate_y_center(spar_width: float) -> float:
    return -(spar_width / 2.0 + JOINT_PLATE_T / 2.0 - 1.0)


def _make_y_axis_plate(x: float, y: float, z: float,
                        bore_radius: float = None,
                        outer_radius: float = JOINT_PLATE_OUTER_R,
                        thickness: float = JOINT_PLATE_T) -> trimesh.Trimesh:
    """Make a flat circular plate of given outer radius and thickness,
    perpendicular to Y, centred at (x, y, z).  Bore + bolt circle are NOT
    cut here — pass the result through `_diff` with the desired holes."""
    plate = _cyl(outer_radius, thickness)
    # cylinder default axis is Z; rotate so axis is Y
    plate.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    plate.apply_translation([x, y, z])
    return plate


def _y_axis_bore_and_bolts(x: float, y: float, z: float,
                            output_or_body: str,
                            depth: float = JOINT_PLATE_T * 3) -> trimesh.Trimesh:
    """Return the union of the central clearance bore + 8 bolt holes for
    a Y-axis motor flange centred at (x, y, z).

    `output_or_body`: "output" -> small bore (just clearance for the
    rotating output collar), "body" -> larger bore (clearance for the
    full motor body shoulder)."""
    if output_or_body == "output":
        bore_r = (MOTOR_OUTPUT_OD - 8.0) / 2.0
    else:
        bore_r = MOTOR_OD / 2.0 + 2.0
    bore = _cyl(bore_r, depth, sections=32)
    bore.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    bore.apply_translation([x, y, z])
    bolts = _bolt_circle_holes(MOTOR_BOLT_CIRCLE / 2.0,
                                MOTOR_BOLT_COUNT, MOTOR_BOLT_OD,
                                depth, axis="y")
    bolts.apply_translation([x, y, z])
    return _union(bore, bolts)


def make_coxa_bracket() -> trimesh.Trimesh:
    """The bracket that bolts to the chassis edge and houses the
    hip-yaw motor.

    Local frame: hip-yaw axis is +Z.  +X points outboard (away from the
    chassis centre).  The rear face of the bracket (facing -X) bolts to
    the chassis pad.

    Construction:
        - Mounting flange (the part touching the chassis pad), in the YZ plane
        - Vertical cradle that holds the yaw motor, axis Z
        - Top + bottom plates that the motor flange bolts to
    """
    # Mounting flange (the part bolted to the chassis edge)
    flange_t = 18.0
    flange = _box((flange_t, 220.0, 220.0),
                  center=(-flange_t / 2.0, 0.0, 0.0))

    # 4 chassis-side bolt holes
    chassis_holes = []
    for sy, sz in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        h = _cyl(11.0 / 2.0, flange_t * 3, sections=16)
        h.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
        h.apply_translation([-flange_t / 2.0, sy * 80.0, sz * 30.0])
        chassis_holes.append(h)

    # Cradle: a U-shape of three plates around the yaw motor body.
    # The yaw motor body axis is Z.  Top plate is at +Z, bottom at -Z.
    cradle_h = MOTOR_LENGTH + 2 * LINK_END_CAP_T
    cradle_w = MOTOR_OD + 50.0
    cradle_d = MOTOR_OD + 50.0

    # Bottom plate (carries the motor body flange)
    bot_plate = _cyl((MOTOR_OD + 40.0) / 2.0, LINK_END_CAP_T)
    bot_plate.apply_translation([COXA_LENGTH * 0.0, 0, -cradle_h / 2.0
                                 + LINK_END_CAP_T / 2.0])
    # Top plate (carries the rotating output)
    top_plate = _cyl((MOTOR_OD + 40.0) / 2.0, LINK_END_CAP_T)
    top_plate.apply_translation([0, 0, cradle_h / 2.0 - LINK_END_CAP_T / 2.0])

    # Two side webs connecting flange to the cradle plates
    web_t = 14.0
    side_webs = []
    for sy in (-1, 1):
        web = _box((90.0, web_t, cradle_h),
                   center=(45.0, sy * (MOTOR_OD / 2.0 + 20.0),
                           0.0))
        side_webs.append(web)

    body = _union(flange, bot_plate, top_plate, *side_webs)

    # Bores in the top + bottom plates for the motor to pass through
    top_bore = _cyl((MOTOR_OUTPUT_OD + 4.0) / 2.0, LINK_END_CAP_T * 3,
                    sections=32)
    top_bore.apply_translation([0, 0, cradle_h / 2.0 - LINK_END_CAP_T / 2.0])
    bot_bore = _cyl(MOTOR_OD / 2.0 + 2.0, LINK_END_CAP_T * 3, sections=32)
    bot_bore.apply_translation([0, 0, -cradle_h / 2.0 + LINK_END_CAP_T / 2.0])

    # Bolt patterns in the plates
    top_bolts = _bolt_circle_holes(MOTOR_BOLT_CIRCLE / 2.0,
                                   MOTOR_BOLT_COUNT, MOTOR_BOLT_OD,
                                   LINK_END_CAP_T * 3)
    top_bolts.apply_translation([0, 0, cradle_h / 2.0])
    bot_bolts = _bolt_circle_holes(MOTOR_BOLT_CIRCLE / 2.0,
                                   MOTOR_BOLT_COUNT, MOTOR_BOLT_OD,
                                   LINK_END_CAP_T * 3)
    bot_bolts.apply_translation([0, 0, -cradle_h / 2.0])

    return _diff(body, *chassis_holes, top_bore, bot_bore, top_bolts, bot_bolts)


def make_coxa_link() -> trimesh.Trimesh:
    """Coxa link (yaw -> hip-pitch).

    Local frame:
        +Z = yaw axis (top hub clamps to the yaw motor's output flange)
        +X = arm direction (outboard at neutral pose)
        +Y = hip-pitch joint axis (perpendicular to arm, tangential)

    The arm is a short box stub running outboard from the yaw hub.  At
    its outboard end, on the -Y side, sits a flat circular plate
    perpendicular to +Y.  The hip-pitch motor's body bolts to the +Y
    face of this plate.  After bolting, the motor extends in -Y from
    the plate; its output flange (at the motor's -Y end) is what the
    femur clamps to."""
    arm_w = 90.0
    arm_t = 60.0
    plate_y = _plate_y_center(arm_w)   # plate just below the arm's -Y face

    # Top yaw hub
    top_hub = _cyl((MOTOR_OUTPUT_OD + 30.0) / 2.0, LINK_HUB_T)

    # Arm stub
    arm = _box((COXA_LENGTH + 20.0, arm_w, arm_t),
               center=(COXA_LENGTH / 2.0, 0.0, 0.0))

    # Hip-pitch plate at outboard end, on -Y side
    pitch_plate = _make_y_axis_plate(COXA_LENGTH, plate_y, 0.0)

    # Triangular gusset bridging the spar end to the plate's outboard
    # corner so the plate has visual support (just a small box for
    # simplicity)
    gusset = _box((40.0, JOINT_PLATE_T, arm_t),
                  center=(COXA_LENGTH - 20.0, plate_y, 0.0))

    body = _union(top_hub, arm, pitch_plate, gusset)

    # Cuts
    yaw_bolts = _bolt_circle_holes(MOTOR_BOLT_CIRCLE / 2.0,
                                   MOTOR_BOLT_COUNT, MOTOR_BOLT_OD,
                                   LINK_HUB_T * 3)
    yaw_bore = _cyl((MOTOR_OUTPUT_OD - 8.0) / 2.0, LINK_HUB_T * 3,
                    sections=32)

    pitch_holes = _y_axis_bore_and_bolts(COXA_LENGTH, plate_y, 0.0,
                                          output_or_body="body")

    return _diff(body, yaw_bolts, yaw_bore, pitch_holes)


def make_femur_link() -> trimesh.Trimesh:
    """Thigh link.

    Local frame:
        +X = spar long-axis (from hip-end at x=0 to knee-end at x=FEMUR_LENGTH)
        +Y = pitch joint axis (parallel for both hip and knee)
        +Z = perpendicular to spar in the leg's plane of motion

    Hip plate sits on the -Y side of the spar at x=0 (clamps to the hip
    motor's output flange — femur is the *distal* link at the hip).
    Knee plate sits on the -Y side of the spar at x=FEMUR_LENGTH (holds
    the knee motor's body — femur is the *proximal* link at the knee).
    Both plates are perpendicular to Y, on the same side, so the
    motors extend straight down in -Y from the spar."""
    plate_y = _plate_y_center(FEMUR_TUBE_W)

    spar = _box((FEMUR_LENGTH, FEMUR_TUBE_W, FEMUR_TUBE_H),
                center=(FEMUR_LENGTH / 2.0, 0, 0))

    hip_plate  = _make_y_axis_plate(0.0,           plate_y, 0.0)
    knee_plate = _make_y_axis_plate(FEMUR_LENGTH,  plate_y, 0.0)

    body = _union(spar, hip_plate, knee_plate)

    # Hip plate: bolts to motor OUTPUT (small bore, output bolt circle)
    hip_holes  = _y_axis_bore_and_bolts(0.0,          plate_y, 0.0,
                                          output_or_body="output")
    # Knee plate: bolts to motor BODY (large bore, body bolt circle)
    knee_holes = _y_axis_bore_and_bolts(FEMUR_LENGTH, plate_y, 0.0,
                                          output_or_body="body")

    # Lightening holes through the spar (Y direction)
    lightening = []
    n_holes = 5
    for i in range(n_holes):
        x = (i + 1) * FEMUR_LENGTH / (n_holes + 1)
        h = _cyl(35.0, FEMUR_TUBE_W * 1.5, sections=24)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([x, 0, 0])
        lightening.append(h)

    return _diff(body, hip_holes, knee_holes, *lightening)


def make_tibia_link() -> trimesh.Trimesh:
    """Shin link.

    Local frame:
        +X = spar long-axis (knee at x=0, foot at x=TIBIA_LENGTH)
        +Y = knee-pitch joint axis (parallel to femur's pitch axis)
        +Z = perpendicular to spar in the leg's plane of motion

    Knee plate sits on the -Y side of the spar at x=0; it bolts to the
    knee motor's *output* flange (tibia is the distal link at the knee).
    Foot socket extends in -Z at the spar's far end."""
    plate_y = _plate_y_center(TIBIA_TUBE_W)

    spar = _box((TIBIA_LENGTH, TIBIA_TUBE_W, TIBIA_TUBE_H),
                center=(TIBIA_LENGTH / 2.0, 0, 0))

    knee_plate = _make_y_axis_plate(0.0, plate_y, 0.0)

    foot_socket = _cyl(FOOT_HUB_OD / 2.0 + 8.0, FOOT_HUB_HEIGHT + 8.0)
    foot_socket.apply_translation([TIBIA_LENGTH - 4.0, 0,
                                    -FOOT_HUB_HEIGHT / 2.0])

    taper = _box((90.0, TIBIA_TUBE_W * 0.9, TIBIA_TUBE_H * 0.9),
                 center=(TIBIA_LENGTH - 45.0, 0, -8.0))

    body = _union(knee_plate, spar, taper, foot_socket)

    # Knee plate: bolts to motor OUTPUT
    knee_holes = _y_axis_bore_and_bolts(0.0, plate_y, 0.0,
                                          output_or_body="output")

    # 4 x M10 bolt holes for the foot pad attachment
    foot_bolts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            h = _cyl(11.0 / 2.0, FOOT_HUB_HEIGHT * 3, sections=16)
            h.apply_translation([TIBIA_LENGTH + 18.0 * sx,
                                 18.0 * sy, -FOOT_HUB_HEIGHT / 2.0])
            foot_bolts.append(h)

    lightening = []
    n_holes = 6
    for i in range(n_holes):
        x = (i + 1) * TIBIA_LENGTH / (n_holes + 1)
        h = _cyl(28.0, TIBIA_TUBE_W * 1.5, sections=24)
        h.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        h.apply_translation([x, 0, 0])
        lightening.append(h)

    return _diff(body, knee_holes, *foot_bolts, *lightening)


def make_foot_pad() -> trimesh.Trimesh:
    """Compliant ground pad with a 4-bolt steel-plate insert pattern.

    Local frame: foot ground-plane is at Z = 0; tibia attaches on +Z."""
    # Conical pad: wide base, narrower hub
    pad_base = _cyl(FOOT_PAD_OD / 2.0, 14.0)
    pad_base.apply_translation([0, 0, 7.0])

    # Tapered shoulder via two stacked cylinders + a small annular blend
    shoulder1 = _cyl(FOOT_PAD_OD * 0.45, 18.0)
    shoulder1.apply_translation([0, 0, 14.0 + 9.0])

    shoulder2 = _cyl(FOOT_HUB_OD / 2.0, FOOT_HUB_HEIGHT)
    shoulder2.apply_translation([0, 0, 14.0 + 18.0 + FOOT_HUB_HEIGHT / 2.0])

    body = _union(pad_base, shoulder1, shoulder2)

    # 4-bolt pattern through the hub for tibia attachment
    bolts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            h = _cyl(11.0 / 2.0, FOOT_PAD_HEIGHT * 3, sections=16)
            h.apply_translation([18.0 * sx, 18.0 * sy, FOOT_PAD_HEIGHT / 2.0])
            bolts.append(h)

    # A small toe spike (shifted forward in +X) for soft-ground traction
    spike = _cyl(8.0, 22.0, sections=24)
    spike.apply_translation([FOOT_PAD_OD * 0.32, 0, -11.0])

    return _diff(_union(body, spike), *bolts)


# ---------------------------------------------------------------------------
# Assembly preview
# ---------------------------------------------------------------------------

def _leg_in_body_frame(leg_index: int) -> trimesh.Trimesh:
    """Return the full leg geometry transformed into the chassis frame
    for leg `leg_index`, in standing pose, as a single concatenated mesh.

    Kinematic chain (after rotating each leg by its azimuth `a` about Z):

       coxa_bracket  ->  yaw (Z axis)  ->  coxa_link  ->
       hip_pitch (Y_local axis)        ->  femur      ->
       knee_pitch (Y_local axis)       ->  tibia      ->  foot.

    The coxa link's outboard end carries the hip-pitch plate on its -Y
    side.  The femur hangs in -Y from that plate (motor extends -Y from
    coxa link, femur clamps to motor output at the far -Y end).  Same
    for the knee.  All hip and knee pitch axes are parallel."""
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    outboard = np.array([np.cos(a), np.sin(a), 0.0])
    tangent  = np.array([-np.sin(a), np.cos(a), 0.0])  # +Y_local
    z_hat    = np.array([0.0, 0.0, 1.0])

    parts = []

    # ------------------- Coxa bracket (chassis-fixed) ------------------
    cb = make_coxa_bracket()
    cb.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cb.apply_translation(edge_mid + (CHASSIS_TUBE / 2.0) * outboard)
    parts.append(cb)

    # ------------------- Coxa link (rotates with yaw) ------------------
    cl = make_coxa_link()
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    yaw_output_height = (MOTOR_LENGTH + LINK_END_CAP_T) / 2.0
    coxa_link_origin = edge_mid + (CHASSIS_TUBE / 2.0
                                    + MOTOR_OD / 2.0 + 30.0) * outboard \
        + yaw_output_height * z_hat
    cl.apply_translation(coxa_link_origin)
    parts.append(cl)

    # ------------------- Femur (pitched about leg-Y) ------------------
    # The hip joint plane lives on the -Y side of the coxa link's pitch
    # plate, MOTOR_LENGTH out in -Y from the coxa pitch plate.  In the
    # leg's local frame:
    #   coxa link's pitch plate centre: (COXA_LENGTH, plate_y_coxa, 0)
    #   hip joint plane:                (COXA_LENGTH, plate_y_coxa - MOTOR_LENGTH, 0)
    plate_y_coxa = _plate_y_center(90.0)         # arm width = 90 mm
    hip_joint_origin_local = np.array([COXA_LENGTH,
                                        plate_y_coxa - MOTOR_LENGTH,
                                        0.0])
    # The femur's local origin is at its hip-end plate centre, on the
    # plate's +Y face (i.e. the femur's spar is at y_local = 0 above the
    # plate at y_local = plate_y_femur).  When we mate the femur to the
    # hip joint we want the femur's PLATE CENTRE to land on the hip
    # joint plane.  In femur local coords the plate centre is at
    # (0, plate_y_femur, 0); the spar is at y_femur=0 (a bit above).
    # So we translate the femur so that its plate centre maps to the
    # hip-joint origin in coxa link local coordinates.
    plate_y_femur = _plate_y_center(FEMUR_TUBE_W)

    fl = make_femur_link()
    # 1) Pitch the femur about its local Y axis (the joint axis).
    #    Pitching rotates the spar in the X-Z plane.
    fl.apply_transform(rotation_matrix(np.deg2rad(STANCE_FEMUR_DEG),
                                        [0, 1, 0]))
    # 2) Translate so that the femur's PLATE CENTRE aligns with the
    #    hip-joint origin in leg-local coords.  Because the rotation in
    #    step 1 leaves the +Y axis alone, the plate centre in pre-rotated
    #    femur space at (0, plate_y_femur, 0) stays at (0, plate_y_femur,
    #    0) in post-rotated femur space.  We want that point to map to
    #    (COXA_LENGTH, plate_y_coxa - MOTOR_LENGTH, 0) in leg-local
    #    coords -- so translate by the difference.
    fl.apply_translation(hip_joint_origin_local
                          - np.array([0.0, plate_y_femur, 0.0]))
    # 3) Rotate the whole leg into world by `a` about Z and translate
    #    to the chassis edge.
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + (CHASSIS_TUBE / 2.0
                                      + MOTOR_OD / 2.0 + 30.0) * outboard
                          + yaw_output_height * z_hat)
    parts.append(fl)

    # ------------------- Tibia ----------------------------------------
    # In femur local frame, the knee joint plane is at:
    #   knee_joint_origin_femur = (FEMUR_LENGTH, plate_y_femur - MOTOR_LENGTH, 0)
    # because the knee motor extends -MOTOR_LENGTH in -Y from the femur
    # knee plate.  The tibia clamps to the motor output at that point.
    # In the tibia's local frame, its plate centre is at (0,
    # plate_y_tibia, 0); we translate the tibia so its plate centre
    # aligns with the knee-joint origin in femur local coords.
    plate_y_tibia = _plate_y_center(TIBIA_TUBE_W)

    p = np.deg2rad(STANCE_FEMUR_DEG)
    pt = np.deg2rad(STANCE_FEMUR_DEG + STANCE_TIBIA_DEG)

    tl = make_tibia_link()
    # 1) Pitch the tibia about its local Y by its absolute pitch angle.
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    # 2) Translate so the tibia's plate centre aligns with the knee
    #    joint plane in femur local coords -- BUT the femur is itself
    #    rotated by femur_pitch about Y, so the knee joint plane in
    #    leg-local coords (after femur's rotation) is:
    #       R_y(femur_pitch) @ (FEMUR_LENGTH, plate_y_femur, 0)
    #       + (0, -MOTOR_LENGTH, 0)
    #    (the femur plate at (FL, plate_y_femur, 0) in femur local
    #    is rotated; the motor extends -MOTOR_LENGTH along +Y_local
    #    which is unaffected by the X-Z rotation.)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_after_femur_translate = Ry_p @ np.array([FEMUR_LENGTH,
                                                         plate_y_femur,
                                                         0.0]) \
        + np.array([0.0, -MOTOR_LENGTH, 0.0])
    # We then add the femur's translation in leg-local coords (so the
    # femur plate ends up at (COXA_LENGTH, plate_y_coxa-MOTOR_LENGTH, 0)).
    femur_translation_local = hip_joint_origin_local \
        - np.array([0.0, plate_y_femur, 0.0])
    knee_joint_origin_local = femur_translation_local \
        + knee_joint_after_femur_translate
    # The tibia's plate centre after its X-Z rotation is at (0,
    # plate_y_tibia, 0) (Y unchanged).  We want it at
    # knee_joint_origin_local.
    tl.apply_translation(knee_joint_origin_local
                          - np.array([0.0, plate_y_tibia, 0.0]))
    # 3) Rotate into world.
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + (CHASSIS_TUBE / 2.0
                                      + MOTOR_OD / 2.0 + 30.0) * outboard
                          + yaw_output_height * z_hat)
    parts.append(tl)

    # ------------------- Foot -----------------------------------------
    # Foot tip at the end of the tibia.  Compute in leg-local then
    # convert to world.
    Ry_pt = rotation_matrix(pt, [0, 1, 0])[:3, :3]
    tibia_translation_local = knee_joint_origin_local \
        - np.array([0.0, plate_y_tibia, 0.0])
    foot_tip_after_tibia_rot = Ry_pt @ np.array([TIBIA_LENGTH, 0.0, 0.0])
    foot_local = tibia_translation_local + foot_tip_after_tibia_rot

    R_a = rotation_matrix(a, [0, 0, 1])[:3, :3]
    base = edge_mid + (CHASSIS_TUBE / 2.0
                        + MOTOR_OD / 2.0 + 30.0) * outboard \
        + yaw_output_height * z_hat
    foot_pos = base + R_a @ foot_local

    foot = make_foot_pad()
    # Put the foot pad's TOP face (its tibia-connecting hub at local
    # z=72) right at the tibia tip; the pad then dangles below the tip.
    FOOT_PAD_TOP_Z = 14.0 + 18.0 + FOOT_HUB_HEIGHT
    foot.apply_translation([foot_pos[0], foot_pos[1],
                             foot_pos[2] - FOOT_PAD_TOP_Z])
    parts.append(foot)

    return _union(*parts)


def make_assembly_preview() -> trimesh.Trimesh:
    """Build the full hexapod walker in standing pose for visual checks.

    Computes chassis_lift directly from the leg's actual geometry so
    the foot pads sit on z = 0.  Robust to any change in stance angles
    or link lengths."""
    # Build one leg to discover its lowest world Z; whatever that is,
    # the chassis must lift the same amount so that point lands on z=0.
    probe_leg = _leg_in_body_frame(0)
    z_min = float(probe_leg.bounds[0][2])
    chassis_lift = -z_min

    parts = []

    # Chassis frame
    chassis = make_chassis_hex()
    chassis.apply_translation([0, 0, chassis_lift])
    parts.append(chassis)

    # Top deck
    deck = make_chassis_top_deck()
    deck.apply_translation([0, 0, chassis_lift + CHASSIS_HEIGHT / 2.0 + 6.0])
    parts.append(deck)

    # Saddle
    saddle = make_saddle_mount()
    saddle.apply_translation([0, 0, chassis_lift + CHASSIS_HEIGHT / 2.0 + 18.0])
    parts.append(saddle)

    # Battery box (slightly aft of centre)
    bb = make_battery_box()
    bb.apply_translation([-260.0, 0, chassis_lift + CHASSIS_HEIGHT / 2.0 + 18.0])
    parts.append(bb)

    # Electronics bay (slightly forward of centre)
    eb = make_electronics_bay()
    eb.apply_translation([260.0, 0, chassis_lift + CHASSIS_HEIGHT / 2.0 + 18.0])
    parts.append(eb)

    # Six legs
    for i in range(6):
        leg = _leg_in_body_frame(i)
        leg.apply_translation([0, 0, chassis_lift])
        parts.append(leg)

    preview = _union(*parts)

    # Rotate Z-up (the convention used while building each part in its
    # own local frame) into Y-up (the convention used by Cursor's STL
    # viewer, MeshLab default, three.js, etc.) so "up" looks up when
    # the file is loaded without any view manipulation.
    preview.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    return preview


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("Hexapod walker -- generating STLs in stl/ ...")

    parts: list[tuple[str, trimesh.Trimesh]] = []

    print("Body parts:")
    parts.append(("chassis_hex.stl",        make_chassis_hex()))
    parts.append(("chassis_top_deck.stl",   make_chassis_top_deck()))
    parts.append(("saddle_mount.stl",       make_saddle_mount()))
    parts.append(("battery_box.stl",        make_battery_box()))
    parts.append(("electronics_bay.stl",    make_electronics_bay()))

    print("Leg parts (one of each -- print 6 sets for the full vehicle):")
    parts.append(("coxa_bracket.stl",       make_coxa_bracket()))
    parts.append(("coxa_link.stl",          make_coxa_link()))
    parts.append(("femur_link.stl",         make_femur_link()))
    parts.append(("tibia_link.stl",         make_tibia_link()))
    parts.append(("foot_pad.stl",           make_foot_pad()))

    print("Generic motor adapter:")
    parts.append(("motor_flange.stl",       make_motor_flange()))

    for name, mesh in parts:
        _save(mesh, name)

    print("Assembly preview (everything in standing pose):")
    preview = make_assembly_preview()
    _save(preview, "assembly_preview.stl")

    # ----- Final summary -----
    total_faces = sum(len(m.faces) for _, m in parts) + len(preview.faces)
    # Preview is Y-up (rotated for STL viewers), so vertical = extents[1].
    foot_to_foot    = preview.extents[0]
    standing_height = preview.extents[1]
    print()
    print(f"OK -- {len(parts) + 1} STL files written.")
    print(f"   Vehicle envelope (foot to foot):  {foot_to_foot/1000:.2f} m")
    print(f"   Vehicle standing height:          {standing_height/1000:.2f} m")
    print(f"   Total geometry triangle count:    {total_faces:,}")
    print()
    print("See ASSEMBLY.md for the build process, motor specs, controller, and gait.")


if __name__ == "__main__":
    main()
