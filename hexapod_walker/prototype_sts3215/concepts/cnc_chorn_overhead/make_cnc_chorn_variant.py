"""VARIANT: CNC aluminum C-clamps sized for LEGS-OVER-HEAD femur travel.

Concept (user, Aug 2026): "make a version of this [the rigid-hip
variant] with metal CNCed C clamps -- a little bigger so this hexapod
can put its legs over its head".

WHAT THE C-CLAMP IS (repo evidence: cad_step_test/build_chorn_step.py,
docs/CHORN_VARIANT.md, tools/make_chorn_variant.py): the C-shaped
bracket at each pitch joint (hip + knee) that clamps the servo's driven
20 mm disc horn on one side and the stock passive disc on the rear
idler boss on the other -- the part that replaces the PRINTED moving
clevis/yoke halves of the production femur and tibia.  The earlier
"C-horn variant" experiment assumed a BOUGHT stock aluminum C horn plus
printed adapters and loose spacers; THIS variant designs the part
proper, as a CNC-machined 6061-T6 bracket with integral standoff
bosses, sized so the femur can swing past vertical.

WHY BIGGER = OVERHEAD (probed, not guessed -- see _probe_envelope.py,
which builds a free-angle map Theta(r, t) of the first collision angle
for femur-frame material at radius r from the hip axis, axial station
t, against the REAL rigid-hip meshes):

  * rigid_hip's femur yoke put structure at r ~29..36 mm from the hip
    axis in the leg's mid-plane -- blocked by the top-plate bearing
    ring + sheet at 72..87 deg up (the measured -47.5 limit came from
    exactly this).
  * The map shows a diagonal free corridor: at the ARM stations (just
    outside the joint's disc span, past the plate ring's tangential
    reach) radii below ~36 are free to ANY angle, and radii 38..46
    clear to 106..119 deg if the material's angular offset is kept at
    or below the swing direction (leading-edge droop).
  * The physics ceiling of the femur assembly is the KNEE SERVO case
    corner at 122 deg (COTS, untouchable).  Next in line was the stock
    knee cap tail at 103.5 deg and the printed knee-block corner at
    ~102.7 deg -- both printed, both chamfered in this variant along
    the same wedge plane (which stays complementary across the parts).

  So the clamp legs GROW from the chorn experiment's web-at-29.5 mm to
  a web at 42.0..45.8 mm (outer face exactly on the production femur
  wall plane, x 58.3 in joint coords), the blades run at the arm
  stations with a drooped leading edge, and the printed knee body +
  knee cap get a matching overhead chamfer.  Swept result (boolean, in
  this script): first contact at -117.5 deg on the 2.5 deg grid ->
  baked limit -110 deg (>= 5 deg true margin), vs -47.5 on rigid_hip
  and -80 in the production workspace.  Past vertical: the foot ends
  ~40 mm inboard of the hip axis, over the body.

PARTS (this concept's step/ + stl/, BuildViz sts3215-cnc-chorn-overhead):

  * ``chorn_clamp_cnc``  (12x, CNC 6061-T6, NOT printed) -- the C-clamp:
    two 3 mm blades with integral Phi 19 ring bosses (4.5 mm below,
    4.0 mm above -- replaces the chorn experiment's 8 loose spacers per
    joint), R13 rounded end over the disc, drooped leading edge, and a
    3.8 mm web whose outer face is the joint's mounting plane, with 4
    TAPPED M3 holes (threads in metal -- no printed threads, no
    inserts in the load path).  R2.5 internal corner gussets + R3 plan
    fillets are modeled (>= 2 mm tool radius everywhere).
  * ``femur_ovh_body``   (6x, PETG) -- the production knee block
    (cradle + 688 housing + far-wall pad, via
    step._femur_knee_fixed_solid) with: the overhead WEDGE chamfer on
    its inboard-top corner, 4 screw cbores for the web joint, the
    knee-cap up-screw bore, and the pocket for the variant cap's
    insert boss.  The yoke arms + spine of the production femur are
    GONE -- the metal clamp is the femur's hip half.
  * ``tibia_ovh_socket`` (6x, PETG) -- flange + CF-tube socket that
    bolts to the tibia-side clamp web the same way (8 mm flange, tube
    mouth pulled 10.6 mm outboard; the tube gets ~16.6 mm shorter and
    almost touches the metal web -- stiffer, lighter).
  * ``knee_clamp_cap_ovh`` (6x, PETG) -- production servo clamp cap
    minus the overhead wedge (its tail was the 103.5 deg blocker) and
    minus its inboard side-screw boss (which died with the tail);
    retention is redesigned as an M3x25 UP-SCREW from the cradle
    underside into a Phi 4 x 6 heat-set insert in a hang-boss, bridged
    to the surviving tongue rib by a riser + a strap over the servo
    top (see the CAP_* constant block).  Horn hooks, back hook,
    outboard screw and the tongue rib's outboard run are untouched.

UNCHANGED: all six rigid_hip printables (chassis plates, hatch, coxa,
hip cap, wago block), every bearing/servo/COTS item, all frame
constants -- robot height and stack are IDENTICAL to rigid_hip.  This
driver re-runs rigid_hip's ENTIRE check suite on the shared parts, then
adds the clamp/joint checks and the overhead sweep.

PRESS-FIT / TOLERANCE WARNING (PETG -> 6061): every bench-tuned
interference in this stack was tuned on printed PETG.  The clamp's
disc-pad span is therefore NOT carried over as an interference: the
production yoke squeezed the discs by 2 x YOKE_SEAT_INTERF = 0.26 mm
(plastic compliance); a rigid aluminum C at that squeeze would bend the
blades, not seat.  The CNC part is dimensioned to the NOMINAL disc span
38.04 mm at +0.00/+0.05, preload comes from the disc screws.  Flagged
as needs-bench-tuning in design_spec.yaml -- do not "fix" it back.

STEP-FIRST PIPELINE: geometry is authored as build123d/OpenCascade BREP
in build_cnc_chorn_step.py (STEP files are the CNC quoting deliverable);
this driver runs that exporter, loads the tessellations, assembles,
checks, sweeps and writes the BuildViz scene.

Run:  uv run python concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py
      (--skip-sweep for fast geometry iterations;
       --skip-brep to reuse existing step/stl/ exports)
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

HERE = os.path.abspath(os.path.dirname(__file__))
PROTO_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
RIGID_DIR = os.path.join(PROTO_DIR, "concepts", "rigid_hip")
STL_DIR = os.path.join(HERE, "stl")
sys.path.insert(0, PROTO_DIR)
sys.path.insert(0, RIGID_DIR)

import hexapod_prototype as hp  # noqa: E402  (read-only input)
import make_rigid_hip_variant as rv  # noqa: E402  (chassis/stack + checks)

# ---------------------------------------------------------------------------
# Joint-frame constants.  Frame = the femur/tibia LINK frame the production
# yoke parts are authored in (femur_link.stl's frame): +x along the link,
# joint axis along +z at x = AXIS_X, +y = the up-swing (cap) side.
# ---------------------------------------------------------------------------
AXIS_X = hp.SERVO_OUTPUT_X                     # 12.5 -- joint axis station
PAD_STACK = hp.YOKE_ARM_PAD + hp.YOKE_SEAT_INTERF          # 5.13
DISC_TOP_FACE_Z = hp.JOINT_HORN_TOP_Z - PAD_STACK          # 36.17 driven disc
DISC_BOT_FACE_Z = hp.JOINT_HORN_BOT_Z + PAD_STACK          # -1.87 passive disc
DISC_SPAN = DISC_TOP_FACE_Z - DISC_BOT_FACE_Z              # 38.04 NOMINAL --
#   the production yoke adds YOKE_SEAT_INTERF per side (PETG squeeze); the
#   rigid aluminum clamp is cut to the nominal span, see the module doc.

ARM_T = 3.0                    # blade thickness (was 2.0 plate + spacers in
#                                the chorn experiment; 3.0 in metal)
PAD_TOP_H = 4.0                # integral ring boss, driven-disc side
PAD_BOT_H = 4.5                # integral ring boss, passive side
PAD_OD = hp.DISC_HORN_OD - 1.0                 # 19 -- rides inside the same
#                                Phi 24 horn openings the production pads use
ARM_TOP_Z0 = DISC_TOP_FACE_Z + PAD_TOP_H       # 40.17
ARM_TOP_Z1 = ARM_TOP_Z0 + ARM_T                # 43.17 (production yoke: 45.3)
ARM_BOT_Z1 = DISC_BOT_FACE_Z - PAD_BOT_H       # -6.37
ARM_BOT_Z0 = ARM_BOT_Z1 - ARM_T                # -9.37 (production yoke: -11)

WALL_FACE_X = hp._YOKE_SOCKET_X + hp.FEMUR_SPAR_LEN        # 58.3 -- frozen
#   production femur wall plane; the web's OUTER face lands exactly here so
#   the printed knee block needs no adapter flange at the hip.
WEB_T = 3.8
WEB_X1 = WALL_FACE_X                           # 58.3 outer face = DATUM A
WEB_X0 = WEB_X1 - WEB_T                        # 54.5 inner face
CLAMP_R1 = WEB_X1 - AXIS_X                     # 45.8 -- "grown" C reach
CLAMP_R0 = WEB_X0 - AXIS_X                     # 42.0 (chorn experiment: 29.5)

# Blade plan profile (x_rel = x - AXIS_X, y).  The LEADING edge (+y = the
# up-swing side) droops through the plate band: the free-angle map gives
# Theta ~106..107 deg at the arm stations for r 38..43, so material there
# must sit at phi <= Theta - 115.5 ~ -9.5 deg (y ~ -6.5..-7.5).  Radii
# below ~36 at the arm stations never hit anything -- full width there.
BLADE_END_R = 13.0             # rounded end over the Phi 20 disc
BLADE_W_UP = 13.0              # full leading half-width in the free zone
BLADE_Y_BOT = -20.0            # trailing/bottom edge at the DISC end (x <=
#                                BLADE_STEP[0]; the aft corridor below dips
#                                deeper, see CORR_Y_BOT)
BLADE_LEAD = [(30.0, 13.0), (34.5, -6.5), (43.0, -7.5)]    # droop polyline
PLAN_FILLET_R = 3.0            # plan-profile fillets (>= 2 mm tool radius)
GUSSET_R = 2.5                 # internal web/blade corner radius (tool R)

# --- SWAN-NECK aft corridor + plate-band notch (the legs-over-head cut) ---
# A raster first-contact map (1 mm grid over the blade plan, both blade
# z-slabs, yaw {0,+-20,+-35}, pitch -95..-122.5) showed the top plate's
# rim sweeps a diagonal BAND through the old aft-bottom box: every plan
# cell in roughly x 43..53 / y -5..-19 (and the whole old droop strip
# x>=46.5) contacts the plate at -95..-112.5.  BUT the rows BELOW y -19
# are clear through -112.5 at every tested yaw (first contact -115 or
# deeper), and the down-swing map (+25..+35) is clear to y -30.  So the
# blade->web load path becomes a SWAN NECK: the disc limb keeps its
# plan, the band is NOTCHED out (NOTCH_PTS, straight edges with >=0.9 mm
# margin to every mapped contact cell), and the load crosses through a
# deeper bottom corridor (notch floor y -19.4/-19.5 down to CORR_Y_BOT
# -24, ~4.5 x 3 mm per blade at the pinch) that rises along the band's
# right margin into the web root.  Expected sweep result: first plate
# contact -115 (blade corridor margin), web bevel contact -117.5 =>
# safe up-limit -107.5 with the 7.5-deg grid margin.
CORR_Y_BOT = -24.0             # corridor + web bottom edge
BLADE_STEP = [(39.0, -20.0), (42.0, -24.0)]  # bottom step down into the
#                                corridor (disc region keeps -20: the zone
#                                x<38 below -20 was not mapped, and the disc
#                                end needs no more depth)
NOTCH_PTS = [                  # plate-band notch, CUT region (plan, CW from
    (42.2, -16.8),             # bottom-left; margins >= ~0.9 mm to every
    (45.3, -6.0),              # -95..-112.5 map cell)
    (45.3, 30.0),              # open through the droop strip / lead line
    (54.6, 30.0),              # right edge 0.1 INTO the web face so no
    (54.6, -6.0),              # 0.1 mm fin survives (web bevel material
    (53.9, -8.0),              # starts 0.3 below the notch floor there)
    (52.9, -11.0),             # right/floor knuckles track the band's
    (51.9, -13.0),             # right margin so the keep-wedge hugging
    (50.9, -15.9),             # the web root stays (extra web root
    (49.8, -18.8),             # section)
    (46.2, -19.4),
]
NOTCH_FILLET_R = 2.0           # tool radius on every notch wall corner
# Where the notch's LEFT wall (x = NOTCH_PTS[1][0]) crosses the blade's
# lead/droop line -- the one internal corner the cutter's own edge
# fillets cannot round; the exporter fillets this vertical edge R2 after
# the subtraction.
NOTCH_JCT_XY = (NOTCH_PTS[1][0],
                BLADE_LEAD[0][1]
                + (NOTCH_PTS[1][0] - (AXIS_X + BLADE_LEAD[0][0]))
                * (BLADE_LEAD[1][1] - BLADE_LEAD[0][1])
                / (BLADE_LEAD[1][0] - BLADE_LEAD[0][0]))   # (45.3, 0.87)
GUSSET_Y = (-23.5, -14.0)      # web/blade gusset span, moved down into the
#                                corridor (the old -19.5..-8 span crossed
#                                the notch)

# Web top bevel: the web's leading edge tracks the 115.7-deg iso-contact
# line of the map almost exactly -- from (r 42.0, y -6.5) to (r 45.8, +1.5).
WEB_BEVEL_IN = (WEB_X0, -6.5)                  # (x, y) at the inner face
WEB_BEVEL_OUT = (WEB_X1, +1.5)                 # (x, y) at the outer face
WEB_Y_BOT = CORR_Y_BOT         # web follows the corridor down (full-height
#                                junction; the map is clear at the web band
#                                y -20..-24 through -122.5)

# Web joint, 4x M3 in two styles (geometry forced the split: SHCS heads
# on the printed side are only reachable through the open servo well,
# whose floor limits them to y >= ~-10; the web's bevel limits tapped
# metal cover to y <= ~-6.9 -- so the UPPER pair is tapped-metal, and
# the LOWER pair, which lives under the cradle floor, becomes CSK
# through-bolts into T-slot nylocs opened from the printed underside,
# exactly the chorn experiment's nut-pocket scheme):
#   A (2x): M3x10 SHCS from the printed side, head cbore opening into
#      the well cavity (driven before the servo drops in), threads in
#      the TAPPED aluminum web (M3-6H x 3.8 -- threads in metal, the
#      DFM ask).  Tip lands ~1.4 mm below the web on the trailing side
#      (swing-safe, asserted).
#   B (2x): M3x12 CSK from the METAL side (90-deg csk sunk in the web's
#      inner face), through printed clearance into an M3 NYLOC in a
#      T-slot pocket cut from the cradle/flange underside.
# z staggered about JOINT_SOCKET_Z (17.15) so no two head recesses or
# metal cuts ever merge (the B stagger is 7.5, not less: the nyloc
# T-slots must clear the up-screw's underside head cbore at z 13.9..20.4
# by >= 1.4 of printed wall).
WEB_A_YZ = [(-9.4, hp.JOINT_SOCKET_Z - 13.0),
            (-9.4, hp.JOINT_SOCKET_Z + 13.0)]
WEB_B_YZ = [(-15.5, hp.JOINT_SOCKET_Z - 7.5),
            (-15.5, hp.JOINT_SOCKET_Z + 7.5)]
WEB_TAP_D = 3.0                # A: M3-6H modeled at Phi 3.0 (2.5 pilot +
#                                tap is the shop's business)
WEB_A_LEN = 10.0               # M3x10 SHCS
WEB_CBORE_D = 6.0              # A: SHCS head pocket in the printed parts
WEB_CBORE_DEPTH = 3.2
WEB_B_D = 3.4                  # B: clearance through metal AND print
WEB_B_LEN = 12.0               # M3x12 CSK flat-head
WEB_CSK_D = 6.72               # B: 90-deg csk major in the web inner face
NYLOC_AF = 5.5                 # M3 nyloc across flats
NYLOC_H = 4.0                  # M3 nyloc height
NUT_SLOT_X0 = 60.8             # nut pocket: 2.5 printed wall past DATUM A
NUT_SLOT_X1 = NUT_SLOT_X0 + NYLOC_H + 0.3      # 65.1
WEB_SHANK_D = 3.4              # printed-side clearance bores

# Overhead WEDGE chamfer (link coords, z-uniform): cuts the printed knee
# block's inboard-top corner and the knee cap's tail along the SAME plane,
# so the parts stay complementary.  Line chosen on the 115.7-deg
# iso-contact of the free-angle map: y = 2 at x 58.3 -> y = 22 at x 68.
WEDGE_XY0 = (WALL_FACE_X, 2.0)
WEDGE_XY1 = (68.0, 22.0)
WEDGE_SLOPE = (WEDGE_XY1[1] - WEDGE_XY0[1]) / (WEDGE_XY1[0] - WEDGE_XY0[0])


def wedge_y(x: float) -> float:
    """Material ceiling of the overhead wedge at link station x."""
    return WEDGE_XY0[1] + WEDGE_SLOPE * (x - WEDGE_XY0[0])


# Knee cap redesign around the wedge: the production INBOARD side screw
# (well x -27.2 -> link 62.8, head on the flange top) dies with the tail.
# Replacement: M3x25 SHCS UP-SCREW from the cradle underside into a
# Phi 4 x 6 heat-set insert in a hang-BOSS that lives in a pocket cut
# into the block's near wall.  The boss cannot weld straight to the
# tongue (the tongue only exists outboard of the servo end face at link
# 67.3, and everything above the wall is wedge-cut inboard of ~66), so
# it bridges over: boss -> RISER (up past the servo top, staying inboard
# of the servo end face) -> STRAP (rides 0.3 over the servo top face,
# welds into the tongue's underside band).  All members stay below the
# wedge plane.  Link coords (cap-local x = link x - FEMUR_LENGTH):
CAP_BOSS_X0, CAP_BOSS_X1 = 60.0, 67.0          # boss: inside the wall span
CAP_BOSS_Y0 = 4.5                              # boss bottom = pocket floor
CAP_BOSS_Y1 = hp.SERVO_BODY_D / 2.0 - hp.CLAMP_TONGUE_INTERF + 0.1  # 11.5
CAP_BOSS_Z0, CAP_BOSS_Z1 = 13.0, 21.3          # >= 2 walls for the insert
CAP_RISER_X0 = 65.0                            # riser: x 65..67 (servo end
#                                                face is 67.3 -- 0.3 clear)
CAP_STRAP_X1 = 68.0                            # strap: x 65..68, welds into
CAP_STRAP_Y0 = hp.SERVO_BODY_D / 2.0 + 0.3     # 12.7 -- 0.3 over the servo
CAP_STRAP_Y1 = 16.5                            # the tongue band (y 11.4-16.9
#                                                at link x >= 67.1)
UPSCREW_X = 63.5                               # link x of the up-screw axis

# Crown shave at the cap's HIP-side tail (WELL-LOCAL coords, i.e. link
# x - FEMUR_LENGTH, the frame the cap is authored in): a -110/-112.5
# raster of plate+hatch occupancy (yaw {0,+-20,+-35}) forbids crown
# material y >= 19 across the full z width for x <= -20.5, plus two
# corner wedges at the z extremes down to y ~16-17.  Shaving these
# moves the cap's first plate contact from -112.5 to -115 (the crown at
# x >= -19.2 stays and IS the -115 contact), so the sweep's safe limit
# rises to -107.5 and the remaining wall is the clamp web's own -117.5
# bevel contact.  The shave zone is clear of the boss/riser/strap (all
# y <= 16.5 at x >= -19.8) and leaves >= 5 mm of arch over the servo.
CAP_SHAVE_X1 = -19.5           # shave applies for well-local x <= this
CAP_SHAVE_ROOF_Y = 18.0        # roof: cut y >= this, full z width
CAP_SHAVE_HI = (16.0, 36.4)    # corner wedge: cut y >= [0] AND z >= [1]
CAP_SHAVE_LO = (16.5, -3.4)    # corner wedge: cut y >= [0] AND z <= [1]
UPSCREW_Z = hp.JOINT_SOCKET_Z                  # 17.15
UPSCREW_D = 3.4                                # clearance bore in the block
UPSCREW_CB_D = 6.5                             # SHCS head cbore, underside
UPSCREW_CB_DEPTH = 4.0
CAP_INSERT_D = 4.0                             # Phi 4 x 6 brass insert
CAP_INSERT_LEN = 6.0
CAP_INSERT_RELIEF_D = 5.5                      # melt relief at entry
CAP_INSERT_RELIEF_DEPTH = 0.4
POCKET_CL = 0.25                               # boss-to-pocket clearance
BLOCK_Y_BOT = -(hp.WELL_D / 2.0)               # -16.9 cradle underside

# Tibia adapter: flange thick enough for the same M3x10 stack, tube mouth
# pulled outboard past the screw hardware.  Socket constants are the
# bench-tuned production tube fit.
TIB_FLANGE_T = 8.0
TIB_FLANGE_X0 = WALL_FACE_X                    # 58.3 (mates DATUM A)
TIB_FLANGE_X1 = TIB_FLANGE_X0 + TIB_FLANGE_T   # 66.3
TIB_FLANGE_Y0, TIB_FLANGE_Y1 = -20.0, 13.0
TIB_FLANGE_Z0, TIB_FLANGE_Z1 = -1.0, 35.3      # covers screws + tube boss
TIB_BORE_X0 = WALL_FACE_X + 0.3                # tube tip 0.3 off the metal
TIB_MOUTH_X = TIB_BORE_X0 + hp.LEG_TUBE_SOCKET_DEPTH       # 72.6 (prod: 62)
TIB_BOSS_R = hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL  # 9.0

# Sweep target + materials
SWEEP_STEP = 2.5
SWEEP_MARGIN = SWEEP_STEP + 5.0                # 1 grid cell + 5 deg true
SWEEP_TARGET = -100.0                          # baked limit must beat this
RHO_ALU = 2.70e-3                              # g/mm3, 6061-T6
RHO_PETG = 1.27e-3                             # g/mm3

# Shared frames from the rigid-hip driver (identical chassis/stack).
MH = rv.MH
M_KNEE_JP = rv.M_KNEE_JP
_trans = rv._trans
_inter_vol = rv._inter_vol
leg_transforms = rv.leg_transforms


# ---------------------------------------------------------------------------
# Mesh registry: rigid_hip's full set + this concept's four BREP parts.
# ---------------------------------------------------------------------------
BREP_STL_DIR = os.path.join(HERE, "step", "stl")
BREP_BUILDER = os.path.join(HERE, "build_cnc_chorn_step.py")
BREP_EXPORT_CMD = ["uv", "run", "--no-project", "--python", "3.12",
                   "--with", "build123d", "--with", "trimesh",
                   "--with", "numpy", "--with", "manifold3d",
                   "python", BREP_BUILDER]
RIGID_BREP_CMD = ["uv", "run", "--no-project", "--python", "3.12",
                  "--with", "build123d", "--with", "trimesh",
                  "--with", "numpy", "--with", "manifold3d",
                  "python", os.path.join(RIGID_DIR, "build_rigid_hip_step.py")]

OVH_PARTS = ("chorn_clamp_cnc", "femur_ovh_body", "tibia_ovh_socket",
             "knee_clamp_cap_ovh")


def _load_brep(fname: str) -> trimesh.Trimesh:
    path = os.path.join(BREP_STL_DIR, fname)
    if not os.path.exists(path):
        raise SystemExit(
            f"{fname}: no BREP tessellation at {path} -- run "
            "build_cnc_chorn_step.py first (main() does that for you "
            "unless --skip-brep)")
    m = trimesh.load(path, process=True)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(
            [g for g in m.geometry.values() if len(g.faces) > 0])
    return m


def _tibia_tube() -> tuple[trimesh.Trimesh, np.ndarray]:
    """Shortened tube visual + the UNCHANGED foot frame (kinematics are
    production; only the buried end moves outboard to the new mouth)."""
    ta_prod = (MH @ np.array([hp._YOKE_SOCKET_X, 0.0,
                              hp.JOINT_SOCKET_Z, 1.0]))[:3]
    tube_end = ta_prod + np.array([hp.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    ta = (MH @ np.array([TIB_BORE_X0, 0.0, hp.JOINT_SOCKET_Z, 1.0]))[:3]
    tube = hp._tube_between(ta, tube_end, hp.LEG_TUBE_OD / 2.0)
    foot_frame = hp._frame(tube_end, (1, 0, 0), (0, 0, 1))
    return tube, foot_frame


def build_meshes() -> dict[str, trimesh.Trimesh]:
    print("building rigid_hip mesh set (shared chassis/stack) ...")
    meshes = rv.build_meshes()
    for name in OVH_PARTS:
        print(f"  {name:22s} loading BREP tessellation ...", flush=True)
        m = hp._heal_for_export(_load_brep(f"{name}.stl"))
        assert m.is_volume, f"{name}: not a volume even after heal"
        meshes[name] = m
    tube, _ = _tibia_tube()
    meshes["tibia_tube_ovh"] = tube

    # Mirror everything this concept's scene references into OUR stl/
    # (print set for the new parts, visuals for the rest).
    os.makedirs(STL_DIR, exist_ok=True)
    for key in SCENE_MESH_KEYS:
        fname = SCENE_MESH_FILES[key]
        meshes[key].export(os.path.join(STL_DIR, fname))
    return meshes


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def _placed(meshes, key, M) -> trimesh.Trimesh:
    m = meshes[key].copy()
    m.apply_transform(M)
    return m


def check_parts(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Part-level gates: watertight, mass budget, DFM-critical dims that
    the BREP builder must have honored (guards future constant edits)."""
    for key, rho, what in (
            ("chorn_clamp_cnc", RHO_ALU, "CNC 6061-T6"),
            ("femur_ovh_body", RHO_PETG, "PETG"),
            ("tibia_ovh_socket", RHO_PETG, "PETG"),
            ("knee_clamp_cap_ovh", RHO_PETG, "PETG")):
        m = meshes[key]
        assert m.is_watertight, f"{key} not watertight"
        print(f"  {key:22s} watertight, vol {m.volume / 1000.0:6.1f} cm3, "
              f"{m.volume * rho:6.1f} g ({what})")

    clamp = meshes["chorn_clamp_cnc"]
    lo, hi = clamp.bounds
    # pads span exactly the nominal disc faces; arms outside them
    assert abs(lo[2] - ARM_BOT_Z0) < 0.02 and abs(hi[2] - ARM_TOP_Z1) < 0.02, \
        f"clamp z envelope {lo[2]:.2f}..{hi[2]:.2f}"
    v = clamp.vertices
    pad_band = v[(v[:, 2] > DISC_BOT_FACE_Z - 0.01)
                 & (v[:, 2] < DISC_TOP_FACE_Z + 0.01)]
    r_pad = np.hypot(pad_band[:, 0] - AXIS_X, pad_band[:, 1])
    # between the disc faces only the Phi 19 bosses (r <= 9.5) and the
    # WEB at the far end (r >= CLAMP_R0) may exist -- the C opening
    inner = r_pad[r_pad < CLAMP_R0 - 0.02]
    assert float(inner.max()) <= PAD_OD / 2.0 + 0.02, \
        f"material at r {inner.max():.2f} inside the disc span (only the " \
        f"Phi {PAD_OD:g} bosses may live there)"
    # pad faces land ON the disc faces (kiss, no PETG-era interference).
    # Probe at azimuth 45 deg -- midway between the 0/90/180/270 deg disc
    # bolt holes, whose Phi 3.4 voids would swallow an on-axis probe.
    px = AXIS_X + (PAD_OD / 2.0 - 1.0) * np.cos(np.pi / 4.0)
    py = (PAD_OD / 2.0 - 1.0) * np.sin(np.pi / 4.0)
    for z, side in ((DISC_BOT_FACE_Z, -1), (DISC_TOP_FACE_Z, +1)):
        probe = np.array([[px, py, z - 0.2 * side],
                          [px, py, z + 0.2 * side]])
        inside = clamp.contains(probe)
        assert not inside[0] and inside[1], \
            f"pad face not exactly on the disc plane z={z:.2f}"
    # web outer face is the wall plane (DATUM A)
    assert abs(hi[0] - WEB_X1) < 0.02, f"web outer face at {hi[0]:.2f}"
    # disc-screw + centre + tap paths open
    pcd_r = hp.DISC_HORN_BOLT_PCD / 2.0
    probes = [[AXIS_X + pcd_r * np.cos(t), pcd_r * np.sin(t), zc]
              for t in hp.DISC_HORN_BOLT_ANGLES_RAD
              for zc in ((ARM_BOT_Z0 + ARM_BOT_Z1) / 2.0,
                         (ARM_TOP_Z0 + ARM_TOP_Z1) / 2.0)]
    probes += [[AXIS_X, 0.0, (ARM_BOT_Z0 + ARM_BOT_Z1) / 2.0],
               [AXIS_X, 0.0, (ARM_TOP_Z0 + ARM_TOP_Z1) / 2.0]]
    assert not clamp.contains(np.asarray(probes)).any(), \
        "a disc-screw / centre bore is blocked in the clamp"
    holes = [[(WEB_X0 + WEB_X1) / 2.0, y, z]
             for (y, z) in WEB_A_YZ + WEB_B_YZ]
    assert not clamp.contains(np.asarray(holes)).any(), \
        "a web screw bore is blocked in the metal"
    # metal cover: full ring around every web hole at the inner face
    # (guards the bevel from biting into a thread/csk boss)
    ring = []
    for (y, z) in WEB_A_YZ + WEB_B_YZ:
        for t in np.linspace(0.0, 2 * np.pi, 8, endpoint=False):
            ring.append([WEB_X0 + 0.4, y + 2.3 * np.cos(t),
                         z + 2.3 * np.sin(t)])
    ring = np.asarray(ring)
    # B holes (the last 2 x 8 points) carry a csk at the inner face:
    # probe their rings at the OUTER face instead, past the csk cone
    ring[16:, 0] = WEB_X1 - 0.4
    assert clamp.contains(ring).all(), \
        "a web screw hole lacks full metal cover (bevel bit the boss ring)"
    print(f"  clamp: pads kiss the discs at span {DISC_SPAN:.2f} "
          f"(nominal, needs-bench-tuning note in the spec), web face on "
          f"DATUM A x={WEB_X1:g}, 2 taps + 2 csk bolt paths open")


def check_clamp_joint(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Seated + spinning clamp vs each joint's fixed side.

    Hip: spin across -120..+35 (past the plate-limited sweep result; a
    full 360 is impossible BY DESIGN for any femur -- the fixed cap's
    tail reaches r 44.2 on the inboard azimuth, and even the production
    yoke crossed it outside its -80..+30 workspace).
    Knee: spin across the tibia ROM -50..+40 (beyond the -30..+20 baked
    limits) against femur body + knee servo + variant cap."""
    T = leg_transforms(0)

    fixed_hip = [_placed(meshes, "servo_body", T["hip_cap"]),
                 _placed(meshes, "hip_clamp_cap_rigid", T["hip_cap"]),
                 _placed(meshes, "bearing_6805", T["hip_cap"]),
                 _placed(meshes, "coxa_link", T["coxa"])]
    for ang in np.arange(-120.0, 35.0 + 1e-9, 5.0):
        Tf = leg_transforms(0, femur_deg=float(ang))
        c = _placed(meshes, "chorn_clamp_cnc", Tf["femur"] @ MH)
        for name, f in zip(("hip servo", "hip cap", "bearing", "coxa"),
                           fixed_hip):
            v = _inter_vol(c, f)
            assert v < 1e-6, \
                f"hip clamp @ {ang:+.0f} deg intersects {name} ({v:.2f} mm3)"
    print("  hip clamp: -120..+35 spin clear of servo/cap/bearing/coxa")

    # knee: the same clamp on the tibia side vs the femur-fixed side
    knee_fixed = [_placed(meshes, "femur_ovh_body", T["femur"] @ MH),
                  _placed(meshes, "servo_body", T["knee_cap"]),
                  _placed(meshes, "knee_clamp_cap_ovh", T["knee_cap"])]
    for td in np.arange(-50.0, 40.0 + 1e-9, 10.0):
        Tt = leg_transforms(0, tibia_deg=float(td))
        c = _placed(meshes, "chorn_clamp_cnc", Tt["tibia"] @ MH)
        sock = _placed(meshes, "tibia_ovh_socket", Tt["tibia"] @ MH)
        for name, f in zip(("femur body", "knee servo", "knee cap"),
                           knee_fixed):
            for part, pm in (("clamp", c), ("socket", sock)):
                v = _inter_vol(pm, f)
                assert v < 1e-6, (f"knee {part} @ tibia {td:+.0f} deg "
                                  f"intersects {name} ({v:.2f} mm3)")
    print("  knee clamp + socket: tibia spin -50..+40 clear of the "
          "femur-fixed side")


def check_web_joint(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Clamp web <-> printed part mating at both joints: faces coplanar,
    zero overlap, both screw styles' paths open end to end, A-screw
    tips swing-safe, B nylocs seated in real pockets with solid bearing
    material above them."""
    clamp = meshes["chorn_clamp_cnc"]
    for key, cav_face in (("femur_ovh_body", hp.FEMUR_LENGTH
                           - hp.SERVO_BODY_W / 2.0),   # 67.3 well face
                          ("tibia_ovh_socket", TIB_FLANGE_X1)):
        part = meshes[key]
        v = _inter_vol(clamp, part)
        assert v < 1e-6, f"clamp overlaps {key} ({v:.2f} mm3)"
        # printed material actually present on the mating plane -- probed
        # as an annulus AROUND each screw (the axis itself is drilled);
        # y-2.6 is skipped because the B row sits 1.4 from the underside
        face_probe = [[WEB_X1 + 0.3, yy, zz]
                      for (y, z) in WEB_A_YZ + WEB_B_YZ
                      for (yy, zz) in ((y + 2.6, z), (y, z + 2.6),
                                       (y, z - 2.6))]
        assert part.contains(np.asarray(face_probe)).all(), \
            f"{key}: no printed material behind the web face at a screw"

        # A screws: axis open from the web's inner face through the tap,
        # printed shank and head cbore out to the cavity/flange face
        for (y, z) in WEB_A_YZ:
            xs = np.arange(WEB_X0 - 0.6, cav_face + 0.5, 0.35)
            line = np.array([[x, y, z] for x in xs])
            blocked = part.contains(line) | clamp.contains(line)
            assert not blocked.any(), \
                f"{key}: A-screw path blocked at (y={y}, z={z})"
            # head seat annulus is solid at the cbore bottom plane
            seat_x = cav_face - WEB_CBORE_DEPTH - 0.4
            seat = [[seat_x, y + 3.6, z], [seat_x, y - 3.6, z],
                    [seat_x, y, z + 3.6], [seat_x, y, z - 3.6]]
            assert part.contains(np.asarray(seat)).all(), \
                f"{key}: A-screw head has no seat at (y={y}, z={z})"
            # M3x10 tip below the web: trailing side, swing-safe
            tip_x = cav_face - WEB_CBORE_DEPTH - WEB_A_LEN
            r = np.hypot(tip_x - AXIS_X, y)
            phi = np.degrees(np.arctan2(y, tip_x - AXIS_X))
            assert r < 44.0 and phi < -12.0, \
                f"A tip at (r {r:.1f}, phi {phi:.1f}) not swing-safe"

        # B bolts: axis open from just outside the web's csk face into
        # the nut pocket; nyloc pocket void present; solid material
        # between pocket top and the mating face for the nut to bear on
        for (y, z) in WEB_B_YZ:
            xs = np.arange(WEB_X0 - 0.6, NUT_SLOT_X1 + 1.2, 0.35)
            line = np.array([[x, y, z] for x in xs])
            blocked = part.contains(line) | clamp.contains(line)
            assert not blocked.any(), \
                f"{key}: B-bolt path blocked at (y={y}, z={z})"
            mid_slot = (NUT_SLOT_X0 + NUT_SLOT_X1) / 2.0
            void = [[mid_slot, y + 2.2, z], [mid_slot, y - 2.2, z]]
            assert not part.contains(np.asarray(void)).any(), \
                f"{key}: nyloc pocket missing at (y={y}, z={z})"
            # probed at z +/- 2.5: outside the Phi 3.4 bore, still under
            # the nyloc face (AF 5.5), inside the wall band either side
            bearing = [[NUT_SLOT_X0 - 1.2, y, z + 2.5],
                       [NUT_SLOT_X0 - 1.2, y, z - 2.5]]
            assert part.contains(np.asarray(bearing)).all(), \
                f"{key}: no bearing wall ahead of the nyloc at (y={y})"
    print(f"  web joint: coplanar mate on DATUM A; 2x M3x{WEB_A_LEN:g} SHCS "
          f"into tapped metal + 2x M3x{WEB_B_LEN:g} CSK into T-slot "
          f"nylocs, per joint; all paths open, tips trailing")


def check_knee_cap(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Variant knee cap: still seats like production (servo squeeze is the
    only contact), boss lands in the block pocket with clearance, the
    up-screw path is open from the cradle underside into the insert,
    drop-in path clear, and the wedge really removed the old blockers."""
    T = leg_transforms(0)
    cap = _placed(meshes, "knee_clamp_cap_ovh", T["knee_cap"])
    block = _placed(meshes, "femur_ovh_body", T["femur"] @ MH)
    servo = _placed(meshes, "servo_body", T["knee_cap"])

    # cap seats FLUSH on the block by design (tongue face on the cavity
    # face, flange on the wall top): coincident faces leave sub-1e-3
    # boolean slivers -- anything real would be orders bigger
    v = _inter_vol(cap, block)
    assert v < 0.01, f"variant cap vs femur body: {v:.3f} mm3"
    # the cap-vs-servo overlap IS the design press: the tongue reaches
    # CLAMP_TONGUE_INTERF = 1.0 past the servo face over the full cavity
    # (~1.6 cm3, production behavior).  Assert it stays in that band --
    # anything above means the new boss/riser/strap bit into the case.
    v = _inter_vol(cap, servo)
    assert 800.0 < v < 1900.0, \
        f"variant cap vs servo: {v:.1f} mm3 (want tongue squeeze only)"

    # production comparison: volume delta = the tail the wedge removed
    dv = (meshes["knee_clamp_cap"].volume
          - meshes["knee_clamp_cap_ovh"].volume) / 1000.0
    assert dv > 0.5, "wedge cut removed almost nothing from the cap"

    # up-screw path (leg frame, along the cap-local -Y... probe in link
    # coords through the block bore into the cap insert bore)
    Mf = T["femur"] @ MH
    ys = np.arange(BLOCK_Y_BOT - 0.5, CAP_BOSS_Y0 + CAP_INSERT_LEN - 0.6, 0.4)
    line = trimesh.transform_points(
        np.array([[UPSCREW_X, y, UPSCREW_Z] for y in ys]), Mf)
    assert not block.contains(line).any(), "up-screw bore blocked in block"
    assert not cap.contains(line).any(), "up-screw bore blocked in cap boss"
    # insert boss really present around the bore
    ringpts = []
    for t in np.linspace(0.0, 2 * np.pi, 6, endpoint=False):
        ringpts.append([UPSCREW_X + 2.6 * np.cos(t),
                        CAP_BOSS_Y0 + 2.0,
                        UPSCREW_Z + 2.6 * np.sin(t)])
    ring = trimesh.transform_points(np.asarray(ringpts), Mf)
    assert cap.contains(ring).all(), "cap insert boss missing around the bore"

    # drop-in: cap lifts straight off (+y in link frame).  The servo is
    # only probed from +2 (the 1 mm tongue press is still engaged below
    # that -- by design); the block must be clear at every height.
    up = Mf[:3, :3] @ np.array([0.0, 1.0, 0.0])
    for d in (0.5, 2.0, 5.0, 15.0):
        c = cap.copy()
        c.apply_translation(up * d)
        fixed = [("block", block, 0.01)] \
            + ([("servo", servo, 25.0)] if d >= 2.0 else [])
        for name, f, lim in fixed:
            # block 0.01: tongue/cavity faces stay COINCIDENT while the
            # cap slides out -- same sub-1e-3 boolean slivers as seated.
            # servo 25.0: the production-style snap hooks sweep a ~1 mm
            # band across the case lip on a STRAIGHT lift (measured
            # 4.2 mm3 at +2); the real PETG cap tilts/flexes over the
            # hooks exactly as in production, so only a small transient
            # is tolerated and +15 must be fully clear
            lim_eff = lim if d < 15.0 else 0.01
            v = _inter_vol(c, f)
            assert v < lim_eff, f"cap lift +{d}: fouls {name} ({v:.2f} mm3)"

    # the wedge did its job: no cap/block/clamp material above the plane
    for key, M in (("knee_clamp_cap_ovh", T["knee_cap"]),
                   ("femur_ovh_body", Mf)):
        m = meshes[key]
        vloc = m.vertices if key == "femur_ovh_body" else \
            trimesh.transform_points(m.vertices,
                                     np.linalg.inv(MH) @ M_KNEE_JP)
        sel = (vloc[:, 0] > WEDGE_XY0[0] - 0.01) \
            & (vloc[:, 0] < WEDGE_XY1[0] - 0.01)
        excess = vloc[sel][:, 1] - wedge_y(vloc[sel][:, 0]) \
            if sel.any() else np.array([-1.0])
        assert float(excess.max()) < 0.05, \
            f"{key}: material {excess.max():.2f} above the overhead wedge"
    print(f"  knee cap OVH: seats + lifts clean, tail wedge removed "
          f"{dv:.1f} cm3, up-screw M3x25 path open into the "
          f"Phi {CAP_INSERT_D:g}x{CAP_INSERT_LEN:g} insert boss")


def sweep_femur_envelope(meshes: dict[str, trimesh.Trimesh],
                         yaw_grid=(-35.0, -20.0, 0.0, 20.0, 35.0),
                         p_hi: float = -140.0, step: float = SWEEP_STEP):
    """The variant's whole point: per yaw angle, the first femur UP angle
    at which ANY femur/tibia-frame part contacts (A) the leg's own
    yaw-rotating fixed side (hip cap + bearing + coxa + hip servo) or
    (B) the world-fixed plate + hatch.  rigid_hip measured -47.5 here;
    the target is past vertical."""
    plate = meshes["chassis_top_rigid"]
    hatch = meshes["top_hatch_rigid"]
    tube = meshes["tibia_tube_ovh"]
    moving_keys = (("chorn_clamp_cnc", "femur_mh"),
                   ("femur_ovh_body", "femur_mh"),
                   ("servo_body", "knee_cap"),
                   ("knee_clamp_cap_ovh", "knee_cap"),
                   ("chorn_clamp_cnc", "tibia_mh"),
                   ("tibia_ovh_socket", "tibia_mh"),
                   ("tibia_tube_ovh", "tibia"),   # tube is authored in the
                   ("foot_boot", "foot"))         # JOINT frame (MH baked in)

    _, foot_frame = _tibia_tube()

    def moving(i, yaw, p):
        T = leg_transforms(i, yaw_deg=yaw, femur_deg=p)
        frames = {"femur_mh": T["femur"] @ MH, "knee_cap": T["knee_cap"],
                  "tibia_mh": T["tibia"] @ MH, "tibia": T["tibia"],
                  "foot": T["tibia"] @ foot_frame}
        for key, fr in moving_keys:
            m = (tube if key == "tibia_tube_ovh" else meshes[key]).copy()
            m.apply_transform(frames[fr])
            yield key, m

    # (A) vs the leg's own yaw-rotating stack -- yaw-independent
    T0 = leg_transforms(0)
    stack = trimesh.util.concatenate([
        _placed(meshes, "hip_clamp_cap_rigid", T0["hip_cap"]),
        _placed(meshes, "bearing_6805", T0["hip_cap"]),
        _placed(meshes, "coxa_link", T0["coxa"]),
        _placed(meshes, "servo_body", T0["hip_cap"])])
    contact_a, part_a = None, None
    for p in np.arange(0.0, p_hi - 1e-9, -step):
        hits = [k for k, m in moving(0, 0.0, p) if _inter_vol(m, stack) > 1.0]
        if hits:
            contact_a, part_a = float(p), hits[0]
            break
    # (B) vs the world-fixed plate + hatch, per yaw
    contact_b, part_b = {}, {}
    for yaw in yaw_grid:
        contact_b[yaw], part_b[yaw] = None, None
        for p in np.arange(0.0, p_hi - 1e-9, -step):
            hit = None
            for k, m in moving(0, yaw, p):
                if m.bounds[1][2] < rv.RING_BOT_W:      # cheap z prefilter
                    continue
                if _inter_vol(m, plate) > 1.0 or _inter_vol(m, hatch) > 1.0:
                    hit = k
                    break
            if hit:
                contact_b[yaw], part_b[yaw] = float(p), hit
                break
    print(f"  vs own coxa/cap/servo stack: first contact {contact_a} deg"
          f" ({part_a})")
    for yaw in yaw_grid:
        print(f"  vs plate+hatch @ yaw {yaw:+5.1f}: first contact "
              f"{contact_b[yaw]} deg ({part_b[yaw]})")
    worst = [c for c in ([contact_a] + list(contact_b.values()))
             if c is not None]
    limit = (max(worst) + SWEEP_MARGIN) if worst else p_hi
    limit = float(np.ceil(limit / 2.5) * 2.5)
    print(f"  => safe femur UP limit (first grid contact + {SWEEP_MARGIN:g}"
          f" margin): {limit} deg  [rigid_hip: -47.5, production: -80]")
    assert limit <= SWEEP_TARGET, (
        f"overhead sweep only reached {limit} deg -- the clamp growth "
        f"missed the legs-over-head target ({SWEEP_TARGET})")
    return limit, (contact_a, part_a), contact_b, part_b


def check_down_and_knee_range(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The grown clamp must not cost the OLD workspace: femur down-swing
    to +30 and the knee's -30..+20 at stance all stay contact-free
    against chassis bottom + the leg's own fixed side."""
    cb = meshes["chassis_bottom"]
    T0 = leg_transforms(0)
    stack = trimesh.util.concatenate([
        _placed(meshes, "hip_clamp_cap_rigid", T0["hip_cap"]),
        _placed(meshes, "coxa_link", T0["coxa"]),
        _placed(meshes, "servo_body", T0["hip_cap"])])
    _, foot_frame = _tibia_tube()
    for p in (30.0, 15.0, 0.0):
        T = leg_transforms(0, femur_deg=p)
        for key, M in (("chorn_clamp_cnc", T["femur"] @ MH),
                       ("femur_ovh_body", T["femur"] @ MH),
                       ("chorn_clamp_cnc", T["tibia"] @ MH),
                       ("tibia_ovh_socket", T["tibia"] @ MH)):
            m = _placed(meshes, key, M)
            for name, f in (("chassis_bottom", cb), ("hip stack", stack)):
                v = _inter_vol(m, f)
                assert v < 1e-6, \
                    f"femur {p:+.0f}: {key} hits {name} ({v:.1f} mm3)"
    print("  down-swing +30 and stance knee range: clear "
          "(old workspace preserved)")


def report_masses(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """Mass/BOM delta table vs rigid_hip (which used the production
    printed femur + tibia yoke)."""
    def g(key, rho):
        return meshes[key].volume * rho

    old_femur = g("femur_link", RHO_PETG)
    old_yoke = g("tibia_knee_yoke", RHO_PETG)
    old_cap = g("knee_clamp_cap", RHO_PETG)
    clamp = g("chorn_clamp_cnc", RHO_ALU)
    body = g("femur_ovh_body", RHO_PETG)
    sock = g("tibia_ovh_socket", RHO_PETG)
    cap = g("knee_clamp_cap_ovh", RHO_PETG)
    per_leg_old = old_femur + old_yoke + old_cap
    per_leg_new = 2 * clamp + body + sock + cap
    rows = {
        "chorn_clamp_cnc_g": round(clamp, 1),
        "femur_ovh_body_g": round(body, 1),
        "tibia_ovh_socket_g": round(sock, 1),
        "knee_clamp_cap_ovh_g": round(cap, 1),
        "old_femur_link_g": round(old_femur, 1),
        "old_tibia_knee_yoke_g": round(old_yoke, 1),
        "old_knee_clamp_cap_g": round(old_cap, 1),
        "per_leg_delta_g": round(per_leg_new - per_leg_old, 1),
        "robot_delta_g": round(6 * (per_leg_new - per_leg_old), 1),
    }
    print(f"  masses: clamp {clamp:.1f} g (alu) x12, femur body {body:.1f}, "
          f"tibia socket {sock:.1f}, cap OVH {cap:.1f} g")
    print(f"  vs rigid_hip leg parts (femur {old_femur:.1f} + yoke "
          f"{old_yoke:.1f} + cap {old_cap:.1f} g): "
          f"{rows['per_leg_delta_g']:+.1f} g/leg, "
          f"{rows['robot_delta_g']:+.1f} g/robot")
    return rows


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
SCENE_MESH_FILES = {
    # this concept's parts (print set / CNC quoting set)
    "chorn_clamp_cnc": "chorn_clamp_cnc_CNC_6061.stl",
    "femur_ovh_body": "femur_ovh_body.stl",
    "tibia_ovh_socket": "tibia_ovh_socket.stl",
    "knee_clamp_cap_ovh": "knee_clamp_cap_ovh.stl",
    "tibia_tube_ovh": "tibia_tube_ovh_DO_NOT_PRINT.stl",
    # inherited rigid_hip prints + COTS visuals (copied for the viewer)
    "hip_clamp_cap_rigid": "hip_clamp_cap_rigid.stl",
    "chassis_top_rigid": "chassis_top_rigid.stl",
    "top_hatch_rigid": "top_hatch_rigid.stl",
    "centre_wago_block": "centre_wago_block.stl",
    "coxa_link": "coxa_link_rigid.stl",
    "chassis_bottom": "chassis_bottom_rigid.stl",
    "foot_boot": "foot_boot.stl",
    "yaw_servo_retainer": "yaw_servo_retainer.stl",
    "servo_body": "servo_body_DO_NOT_PRINT.stl",
    "yaw_bearing_upper": "yaw_bearing_upper_DO_NOT_PRINT.stl",
    "bearing_6805": "bearing_6805_DO_NOT_PRINT.stl",
    "wago5": "wago5_DO_NOT_PRINT.stl",
}
SCENE_MESH_KEYS = list(SCENE_MESH_FILES)

COLORS = dict(rv.COLORS)
COLORS.update({
    "chorn_clamp_cnc": "#c8ccd2",      # machined aluminum
    "femur_ovh_body": "#7fb069",
    "tibia_ovh_socket": "#7fb069",
    "knee_clamp_cap_ovh": "#9dc183",
    "tibia_tube_ovh": "#404040",
})
COTS = set(rv.COTS) | {"tibia_tube_ovh"}


def _mat16(M: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(M, float).T.reshape(-1)]


def build_scene(meshes, femur_up_limit: float) -> dict:
    _, foot_frame = _tibia_tube()
    mesh_defs = [{"id": f"stl:{k}", "name": fn, "url": f"stl/{fn}"}
                 for k, fn in SCENE_MESH_FILES.items()]

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

    inst("chassis_bottom", "chassis_bottom RIGID", np.eye(4))
    inst("chassis_top_rigid", "chassis_top_rigid FRAME", np.eye(4))
    inst("top_hatch_rigid", "top_hatch (removable)", np.eye(4))
    inst("centre_wago_block", "central splice block", np.eye(4))
    for M, label in zip(rv._wago5_scene_frames(),
                        ("V+ west", "V+ east", "GND west", "GND east")):
        inst("wago5", f"221-415 {label}", M)
    for i in range(6):
        T = leg_transforms(i)
        a = (i + 0.5) * np.pi / 3.0
        axis_pt = [rv.APOTHEM * np.cos(a), rv.APOTHEM * np.sin(a),
                   hp.CHASSIS_YAW_OUTPUT_Z]
        pitch_ax = (rv._rotz(a)[:3, :3] @ np.array([0.0, 1.0, 0.0])).tolist()
        hip_pt = (T["coxa"] @ np.array([*rv.COXA_HIP_ANCHOR_V, 1.0]))[:3]
        knee_pt = (T["femur"]
                   @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0, 1.0]))[:3]

        yaw_ids = [
            inst("coxa_link", f"L{i} coxa_link RIGID", T["coxa"], leg=i),
            inst("yaw_bearing_upper", f"L{i} yaw bearing (tower-seated)",
                 T["coxa"] @ _trans([0.0, 0.0, rv.YAWBR_DROP]), leg=i),
            inst("servo_body", f"L{i} hip servo", T["hip_cap"], leg=i),
            inst("hip_clamp_cap_rigid", f"L{i} hip cap RIGID",
                 T["hip_cap"], leg=i),
            inst("bearing_6805", f"L{i} third 6805", T["hip_cap"], leg=i),
        ]
        inst("yaw_servo_retainer", f"L{i} yaw retainer", T["cradle"], leg=i)
        inst("servo_body", f"L{i} yaw servo",
             T["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                                 -(hp.HORN_STACK_H + hp.WELL_RIM_Z)]),
             leg=i)
        hip_ids = [
            inst("chorn_clamp_cnc", f"L{i} hip C-clamp (CNC 6061, NEW)",
                 T["femur"] @ MH, leg=i),
            inst("femur_ovh_body", f"L{i} femur body OVH (NEW)",
                 T["femur"] @ MH, leg=i),
            inst("servo_body", f"L{i} knee servo", T["knee_cap"], leg=i),
            inst("knee_clamp_cap_ovh", f"L{i} knee cap OVH (NEW)",
                 T["knee_cap"], leg=i),
        ]
        knee_ids = [
            inst("chorn_clamp_cnc", f"L{i} knee C-clamp (CNC 6061, NEW)",
                 T["tibia"] @ MH, leg=i),
            inst("tibia_ovh_socket", f"L{i} tibia socket OVH (NEW)",
                 T["tibia"] @ MH, leg=i),
            inst("tibia_tube_ovh", f"L{i} tibia tube (shortened)",
                 T["tibia"], leg=i),
            inst("foot_boot", f"L{i} foot boot",
                 T["tibia"] @ foot_frame, leg=i),
        ]
        joints += [
            {"id": f"L{i}-yaw", "type": "revolute", "axis": [0, 0, 1],
             "origin": axis_pt, "instances": yaw_ids,
             "limits": {"min": -35.0, "max": 35.0}, "home": 0,
             "label": f"L{i} yaw"},
            {"id": f"L{i}-hip", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in hip_pt], "instances": hip_ids,
             "limits": {"min": femur_up_limit, "max": 30.0}, "home": 0,
             "label": f"L{i} hip (OVERHEAD limit {femur_up_limit:g})"},
            {"id": f"L{i}-knee", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in knee_pt], "instances": knee_ids,
             "limits": {"min": -30.0, "max": 20.0}, "home": 0,
             "label": f"L{i} knee"},
        ]

    up = femur_up_limit - hp.STANCE_FEMUR_DEG   # joint value at the limit
    scene = {
        "name": "sts3215 CNC C-clamp overhead variant -- legs over head",
        "source": "concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py",
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0, 0, 55],
        "checksConfig": {
            "overlapMm3": 80.0, "pitchMm": 2.0,
            "ignoreOverlapPairs": [
                ["chassis_top_rigid", "top_hatch_rigid"],
                ["centre_wago_block", "wago5"],
                ["chassis_bottom", "servo_body"],
                ["coxa_link", "servo_body"],
                ["coxa_link", "yaw_bearing_upper"],
                ["hip_clamp_cap_rigid", "servo_body"],
                ["femur_ovh_body", "servo_body"],
                ["knee_clamp_cap_ovh", "servo_body"],
                ["foot_boot", "tibia_tube_ovh"],
                ["tibia_ovh_socket", "tibia_tube_ovh"],
            ]},
        "meshes": mesh_defs,
        "instances": instances,
        "joints": joints,
        "poses": [
            {"id": "stance", "name": "Stance (walking)",
             "jointValues": {j["id"]: 0.0 for j in joints}},
            {"id": "legs-over-head",
             "name": f"LEGS OVER HEAD (all hips at {femur_up_limit:g} deg)",
             "jointValues": {f"L{i}-hip": up for i in range(6)}},
            {"id": "femur-up-limit",
             "name": f"L0 femur at the new up limit ({femur_up_limit:g})",
             "jointValues": {"L0-hip": up}},
        ],
        "animations": [
            {"id": "overhead", "name": "all legs: stance -> over the head",
             "loop": True, "duration": 10.0,
             "keyframes": [
                 {"t": 0.0, "jointValues":
                     {f"L{i}-hip": 0 for i in range(6)}},
                 {"t": 2.0, "jointValues": {"L0-hip": up}},
                 {"t": 4.5, "jointValues":
                     {f"L{i}-hip": up for i in range(6)}},
                 {"t": 6.5, "jointValues":
                     {f"L{i}-hip": up for i in range(6)}},
                 {"t": 9.0, "jointValues":
                     {f"L{i}-hip": 0 for i in range(6)}},
             ]},
        ],
    }
    return scene


def render_preview(meshes, limit: float) -> None:
    """Elevation through leg 0's pitch plane at the overhead limit: the
    money shot of the femur past vertical over the top plate."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = 0.5 * np.pi / 3.0
    Rz = rv._rotz(-a)
    T = leg_transforms(0, femur_deg=limit)
    Ts = leg_transforms(0)
    _, foot_frame = _tibia_tube()
    sections = [
        ("chassis_bottom", np.eye(4), "#8b93a6", "chassis bottom"),
        ("chassis_top_rigid", np.eye(4), "#5b8fd4", "top plate"),
        ("top_hatch_rigid", np.eye(4), "#6fa8dc", "hatch"),
        ("coxa_link", Ts["coxa"], "#7ba1d1", "coxa"),
        ("servo_body", Ts["hip_cap"], "#c9a227", "hip servo"),
        ("hip_clamp_cap_rigid", Ts["hip_cap"], "#4878b0", "hip cap"),
        ("chorn_clamp_cnc", T["femur"] @ MH, "#8e959e",
         "CNC C-clamp (hip, at the limit)"),
        ("femur_ovh_body", T["femur"] @ MH, "#7fb069", "femur body OVH"),
        ("servo_body", T["knee_cap"], "#c9a227", None),
        ("knee_clamp_cap_ovh", T["knee_cap"], "#9dc183", "knee cap OVH"),
        ("chorn_clamp_cnc", T["tibia"] @ MH, "#8e959e", None),
        ("tibia_ovh_socket", T["tibia"] @ MH, "#7fb069", "tibia socket OVH"),
        ("tibia_tube_ovh", T["tibia"], "#404040", "tibia tube"),
        ("foot_boot", T["tibia"] @ foot_frame, "#5a5f66", "foot"),
    ]
    fig, ax = plt.subplots(figsize=(10.5, 8.0), dpi=130)
    for key, M, color, label in sections:
        m = meshes[key].copy()
        m.apply_transform(Rz @ M)
        sec = m.section(plane_origin=[rv.APOTHEM, 0, 0],
                        plane_normal=[0, 1, 0])
        if sec is None:
            continue
        planar, _ = sec.to_2D(to_2D=rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
        first = True
        for poly in planar.polygons_full:
            xs, ys = poly.exterior.xy
            ax.fill(xs, ys, color=color, alpha=0.65,
                    label=label if first else None)
            first = False if label else first
            for ring in poly.interiors:
                ax.fill(*ring.xy, color="white")
    ax.axhline(rv.SHEET_Z1, color="k", lw=0.5, ls=":")
    ax.annotate(f" plate deck z={rv.SHEET_Z1:.2f}",
                (rv.APOTHEM + 55, rv.SHEET_Z1), fontsize=7, va="bottom")
    ax.set_xlim(rv.APOTHEM - 120, rv.APOTHEM + 90)
    ax.set_ylim(-15, 245)
    ax.set_aspect("equal")
    ax.set_xlabel("radial position from body centre [mm]")
    ax.set_ylabel("world Z [mm]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"cnc_chorn_overhead -- leg 0 at the {limit:g} deg limit "
                 "(femur past vertical, over the plate)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview.png"))
    print("  wrote preview.png")


def main() -> None:
    skip_sweep = "--skip-sweep" in sys.argv
    skip_brep = "--skip-brep" in sys.argv
    if not skip_brep:
        # rigid_hip's exports are prerequisites (shared chassis/stack)
        need = [k for k in ("chassis_top_rigid", "hip_clamp_cap_rigid",
                            "coxa_link_rigid", "chassis_bottom_rigid",
                            "top_hatch_rigid", "centre_wago_block")
                if not os.path.exists(os.path.join(
                    RIGID_DIR, "step", "stl", f"{k}.stl"))]
        if need:
            print(f"rigid_hip BREP exports missing ({need}) -- building ...")
            subprocess.run(RIGID_BREP_CMD, check=True)
        print("exporting BREP geometry (build_cnc_chorn_step.py) ...")
        subprocess.run(BREP_EXPORT_CMD, check=True)
    else:
        print("BREP EXPORT SKIPPED (--skip-brep): reusing step/stl/")
    meshes = build_meshes()

    print("rigid_hip inherited checks (shared chassis/stack) ...")
    rv.check_static(meshes)
    rv.check_bottom_joint(meshes)
    rv.check_coxa_column(meshes)
    rv.check_chassis_variant(meshes)
    rv.check_rot_envelope(meshes)
    rv.check_wago_block(meshes)
    rv.check_hatch(meshes)
    rv.check_yaw_sweep(meshes)
    rv.check_plate_descent(meshes)

    print("cnc_chorn_overhead checks ...")
    check_parts(meshes)
    check_clamp_joint(meshes)
    check_web_joint(meshes)
    check_knee_cap(meshes)
    check_down_and_knee_range(meshes)
    masses = report_masses(meshes)

    if skip_sweep:
        limit = -110.0
        print("SWEEP SKIPPED (--skip-sweep): using placeholder limit -110")
        sweep_info = {"skipped": True}
    else:
        print("overhead femur sweep (the slow part) ...")
        limit, ca, cb, pb = sweep_femur_envelope(meshes)
        sweep_info = {
            "limit_deg": limit,
            "contact_vs_own_stack": {"deg": ca[0], "part": ca[1]},
            "contact_vs_plate_by_yaw": {
                str(y): {"deg": cb[y], "part": pb[y]} for y in cb},
            "margin_deg": SWEEP_MARGIN, "step_deg": SWEEP_STEP,
        }

    print("writing scene.json ...")
    scene = build_scene(meshes, limit)
    with open(os.path.join(HERE, "scene.json"), "w", encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)
    with open(os.path.join(HERE, "sweep_report.json"), "w",
              encoding="utf-8") as fh:
        json.dump({"sweep": sweep_info, "masses": masses}, fh, indent=1)

    render_preview(meshes, limit)
    print("done.")


if __name__ == "__main__":
    main()
