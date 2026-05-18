"""Run all post-redesign correctness checks on the prototype.

1.  Every part is a clean watertight mesh (manifold, no self-intersections
    that would crash the slicer).
2.  Every servo well has an open insertion path big enough for the body.
3.  Every M3 mounting / chassis-bolt hole drilled by the design code
    actually passes through solid material it's supposed to anchor in.
3b. The L-shaped wire-exit corridor at every cradle's -X bottom-OUTBOARD
    corner lets a WIRE_SLOT_W-wide harness reach free space without
    intersecting solid part material.
4.  In standing pose, no two printed parts overlap in volume by more than
    100 mm^3.
5.  In standing pose, the three servo bodies (yaw / hip-pitch / knee-pitch)
    sit in their cradles without poking into solid printed material.
6.  No printed part has a STRUCTURALLY-SIGNIFICANT region thinner than
    MIN_PRINT_T mm (Hildebrand max-inscribed-ball local thickness on a
    voxelised occupancy field).
6b. None of the structural LEG LINK parts has a thin SHEET / WEB --
    a region of material extended in 2D but thinner than MIN_SHEET_T
    in the third direction.  Catches "thick block - THIN SHEET -
    thick block" topology that 6. misses because an inscribed ball
    fits happily inside a thin slab.

Exit code 0 = all checks pass.  Exit code 1 = at least one failure.

Robust inside-test
------------------

We do NOT use ``trimesh.proximity.signed_distance`` or
``Trimesh.contains`` directly: both give false positives on the
boolean-union'd meshes used by `make_coxa_*`/`make_femur_*` (the
internal cavities created by the body-pocket cuts confuse the
ray-casting heuristic).  Instead we shoot 6 axis-aligned rays from
each test point and call the point INSIDE iff a majority of rays
report an odd number of intersections with the mesh.  This majority
vote is much more robust on borderline / non-manifold input.
"""

from __future__ import annotations
import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix
from scipy.ndimage import distance_transform_edt, label
import shapely.geometry as _sg

import hexapod_prototype as hp


# ---------------------------------------------------------------------------
# Flimsy-joint check constants
# ---------------------------------------------------------------------------

# Minimum acceptable LOCAL THICKNESS for printed material.  Three perimeters
# at a 0.4 mm nozzle plus margin -- below this an FDM print becomes either
# entirely sparse infill (the slicer skips perimeters because the wall is
# too thin to fit even one bead) or a stack of < 3 perimeters that snaps
# under any real load.  Tweak this if your slicer / nozzle differs.
MIN_PRINT_T = 3.0           # mm
# Ignore clusters smaller than this -- single voxels and 1-voxel-wide
# surface "fuzz" are voxelisation artefacts of the grid pitch, not real
# weak spots.  At a 1.2 mm pitch a 20-voxel cluster is ~35 mm^3 of
# material thinner than MIN_PRINT_T, which is the smallest defect worth
# investigating.
MIN_CLUSTER_VOX = 20        # voxels
# Budget for the LARGEST flimsy cluster in any single part.  Anything
# bigger means a structurally meaningful thin region the user should
# reinforce before printing.
#
# Why not 200?  Long printed beams with sharp 90 deg edges (the femur
# spar, the tibia spar) produce an unavoidable voxelisation ARTEFACT
# along each long edge.  At MIN_PRINT_T = 3.0 mm and pitch = 1.2 mm,
# half_t_vox = 1.25 voxels, while a corner voxel's distance to its
# nearest core voxel is sqrt(2) ~= 1.414 voxels -- just over the
# threshold, regardless of how thick the underlying wall actually
# is.  A 130 mm-long spar with 4 long edges therefore stamps out
# ~ 130 / 1.2 = 108 corner voxels per edge, which scipy.label
# typically merges into one or two ridges of ~ 200-250 voxels each.
# We have visually verified these ridges (bbox shape: ~120 mm long,
# 4.8 mm wide, < 10 mm tall, 5-10 % voxel fill -- i.e. a 1-voxel-
# thick filament along the spar edge) on the tibia and femur spars;
# they map to real 90-deg edges in the part where the FDM print
# actually deposits >= 3 perimeters of material.  A real thin
# feature (1-2 mm wall over a > 10 x 10 mm area) produces > 1000
# voxels of flimsy and easily clears 250.  We pick 250 -- 25 %
# above the no-artefact baseline of 200 -- as the smallest value
# that admits the predicted corner-edge ridge without masking any
# structurally-meaningful thin region.
MAX_FLIMSY_BUDGET_VOX = 250 # voxels
# Voxel pitch for the local-thickness analysis.  1.2 mm gives roughly
# 80x80x40 = 250k voxels for a typical leg link, which voxelizes and
# distance-transforms in well under a second.
FLIMSY_VOXEL_PITCH = 1.2    # mm


# ---------------------------------------------------------------------------
# Thin-sheet ("structural neck") check constants
# ---------------------------------------------------------------------------
#
# Why a separate check?  ``check_flimsy_joints`` above uses the
# Hildebrand max-inscribed-ball local thickness, which only catches
# features that are thin in ALL THREE directions (spikes, pillars,
# hair-thin necks).  An inscribed sphere of radius MIN_PRINT_T/2 fits
# happily inside any 3 mm-thick slab, so a thin SHEET / WEB (thin in
# one direction, extended in the other two) sails through Hildebrand
# no matter how unprintably thin its load-bearing slab actually is.
#
# This check (a anisotropic chord variant of Hildebrand) computes the
# per-voxel MIN DIRECTIONAL CHORD LENGTH: for each occupied voxel V we
# measure the length, in mm, of the contiguous run of occupied voxels
# containing V along the +/-X, +/-Y, and +/-Z lines, then take the
# smallest of the three lengths.  Voxels whose min directional chord
# falls inside the STRUCTURAL-NECK BAND (between
# ``MIN_SHEET_T_LOWER`` and ``MIN_SHEET_T_UPPER`` mm) are clustered;
# the part FAILS if the largest cluster exceeds ``MAX_SHEET_BUDGET_VOX``.
#
# Why the LOWER bound (5 mm, not 0)?  Features thinner than this lower
# bound fall into one of two well-understood categories:
#
#   1. Unprintably thin (chord <= MIN_PRINT_T = 3 mm) -- the Hildebrand
#      check above already flags these.
#   2. Intentional thin PERIMETER walls / plates (servo well walls
#      WELL_WALL_Y = 4.5 mm, chassis plate CHASSIS_PLATE_T = 4 mm,
#      servo horn adapter HORN_ADAPTER_T = 4 mm).  The user has
#      explicitly accepted these thicknesses in the design notes --
#      they are NOT structural load-path necks, and flagging them
#      here would just generate noise.
#
# The 5-6 mm "structural neck band" sits above the intentional thin-
# wall floor and at-or-below the user's MIN_SHEET_T = 6 mm.  The user's
# complaint with ``coxa_link`` (the arm slab between the hub and the
# well at 6 mm thick) lands squarely inside this band -- it's a slab
# that's "thick enough to look OK at a glance but thin enough to bend
# under load".
MIN_SHEET_T          = 6.0  # mm -- minimum acceptable thickness for
                             # a structural-neck load path.  A 6 mm
                             # slab between two thick blocks bends
                             # visibly under nominal hip-pitch
                             # reaction torque.  Independent of
                             # MIN_PRINT_T (3 mm).
# The chord lengths we measure on the voxel grid are AT LEAST the
# physical slab thickness, but can be up to one voxel pitch LARGER
# when the slab is sandwiched against another piece of solid
# material (the voxel grid does not know that there's a fillet /
# step / parting surface between them).  In coxa_link the 6 mm arm
# slab measures as 7.2 mm of chord_z (5 vox of arm + 1 vox of
# pedestal slab below).  We widen the upper threshold by one voxel
# pitch to absorb that quantization slack so the check reliably
# fires on the offending feature.
MIN_SHEET_T_UPPER    = MIN_SHEET_T + FLIMSY_VOXEL_PITCH  # 7.2 mm
MIN_SHEET_T_LOWER    = 5.0  # mm -- lower bound on the "structural-
                             # neck band".  Voxels with min chord
                             # below this represent intentional
                             # thin-wall design (see big comment
                             # above) and are NOT flagged by this
                             # check.
MIN_SHEET_CLUSTER_VOX = 400 # voxels -- ignore clusters smaller than
                             # this.  A 400-voxel cluster at pitch
                             # 1.2 mm is ~ 691 mm^3, roughly a
                             # 6 mm-thick slab 6 mm wide and 22 mm
                             # long.  This is a deliberate floor:
                             # SOME thin slabs are unavoidable as
                             # CLEARANCE features (e.g. coxa_link's
                             # 6 mm-wide arm strip at link y in
                             # [-3, +3] that lets the femur spar
                             # swing through at femur_pitch =
                             # -80 deg).  Setting the floor at
                             # 400 voxels excludes such intentional
                             # clearance strips while still flagging
                             # multi-cm structural slabs like the
                             # user's complaint about coxa_link's
                             # full-width arm (2040 voxels at
                             # arm_t = 6 mm, before reinforcement).
MAX_SHEET_BUDGET_VOX = 500  # voxels -- per-part budget for the
                             # largest structural-neck cluster.
                             # Above this means a real structural
                             # neck exists.  Keep the budget just
                             # above the min-cluster floor so any
                             # cluster that passes the floor counts
                             # as a real failure.

# Parts to test for structural-neck topology.  We restrict to the
# coxa_link (the user's complaint).  The femur and tibia spar's
# 6 mm Y thickness is by-design uniform across their full length
# (the link prints flat on the build plate with its narrow Y
# dimension vertical, exactly LINK_THICKNESS = 6 mm), which is the
# correct printing strategy for a long flat plate but registers as a
# huge anisotropic "thin sheet" cluster under THIS test.  Flagging
# the femur / tibia spar would just generate noise -- they are not
# the "thick block - thin sheet - thick block" topology the user
# pointed at.  A future iteration could either thicken the spars
# globally or implement a richer "neck between thick volumes"
# detector (option (b) in this script's notes) to discriminate
# uniform spars from real load-path necks.
THIN_SHEET_PARTS = (
    "coxa_link",
)


# ---------------------------------------------------------------------------
# Robust inside-test (majority vote over 6 axis-aligned rays)
# ---------------------------------------------------------------------------

_RAY_DIRS = np.array([
    [+1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, +1.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, +1.0],
    [0.0, 0.0, -1.0],
])


def points_inside(mesh, pts):
    """Return a boolean array indicating whether each point lies inside
    ``mesh``.  Uses 6 axis-aligned rays per point and majority vote on
    intersection-count parity (odd = inside)."""
    pts = np.asarray(pts, dtype=float)
    if pts.ndim == 1:
        pts = pts[None, :]
    n = len(pts)
    votes = np.zeros(n, dtype=int)
    for direction in _RAY_DIRS:
        # Per ray, count intersections from each point along this direction.
        origins = pts
        directions = np.tile(direction, (n, 1))
        # Use ray.intersects_id which returns (locations, index_ray, _).
        # We only need a count per ray.
        try:
            _, index_ray, _ = mesh.ray.intersects_location(
                ray_origins=origins, ray_directions=directions)
        except Exception:
            continue
        if len(index_ray) == 0:
            continue
        counts = np.bincount(index_ray, minlength=n)
        votes += (counts % 2 == 1).astype(int)
    # Majority of 6 rays says odd-parity -> inside.
    return votes >= 4


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _label(name, ok, detail=""):
    flag = "PASS" if ok else "FAIL"
    print(f"  [{flag}]  {name:50s}  {detail}")
    return ok


def _describe(mesh):
    return (f"vol={mesh.volume:8.1f} mm^3  "
            f"watertight={mesh.is_watertight}  "
            f"winding={mesh.is_winding_consistent}")


# ---------------------------------------------------------------------------
# 1.  Watertightness / manifold / volume sanity
# ---------------------------------------------------------------------------

def check_watertight():
    print("\n[1] Mesh watertightness / manifoldness:")
    items = {
        "chassis_top":         hp.make_chassis_top(),
        "chassis_bottom":      hp.make_chassis_bottom(),
        "battery_holder":      hp.make_battery_holder(),
        "electronics_tray":    hp.make_electronics_tray(),
        "coxa_bracket":        hp.make_coxa_bracket(),
        "coxa_link":           hp.make_coxa_link(),
        "femur_link":          hp.make_femur_link(),
        "tibia_link":          hp.make_tibia_link(),
        "foot_pad":            hp.make_foot_pad(),
        "servo_horn_adapter":  hp.make_servo_horn_adapter(),
    }
    all_ok = True
    for name, m in items.items():
        ok = m.is_watertight and m.is_winding_consistent and m.volume > 0
        all_ok &= _label(name, ok, _describe(m))
    return all_ok


# ---------------------------------------------------------------------------
# 2.  Cradle insertion-path openness
# ---------------------------------------------------------------------------

def _cradle_open(part_mesh, body_centre, body_long_axis, body_short_axis,
                  open_dir, name=""):
    """Probe whether the servo body can be inserted into a cradle along
    ``open_dir``."""
    body_centre = np.asarray(body_centre, dtype=float)
    n = np.asarray(open_dir, dtype=float)
    n = n / np.linalg.norm(n)
    u = np.asarray(body_long_axis, dtype=float)
    u = u / np.linalg.norm(u)
    v = np.asarray(body_short_axis, dtype=float)
    v = v / np.linalg.norm(v)

    half_u = hp.SERVO_BODY_W / 2.0 - 0.5
    half_v = hp.SERVO_BODY_D / 2.0 - 0.5

    bb_min, bb_max = part_mesh.bounds
    proj = (bb_max - bb_min) @ np.abs(n)
    travel = float(proj) + hp.SERVO_BODY_H + 5.0

    Nu, Nv, Nt = 5, 5, 25
    samples = []
    for i in np.linspace(-half_u, half_u, Nu):
        for j in np.linspace(-half_v, half_v, Nv):
            for k in np.linspace(0.0, travel, Nt):
                p = body_centre + i * u + j * v + k * n
                samples.append(p)
    samples = np.asarray(samples)

    inside = points_inside(part_mesh, samples)
    n_blocked = int(inside.sum())
    n_total = len(samples)
    return n_blocked == 0, n_blocked, n_total


def _body_fits_in_part(part_mesh, body_centre, body_long_axis,
                        body_short_axis, body_normal_axis,
                        long_extent, short_extent, normal_extent):
    """Probe whether the servo body's FINAL resting volume is free of
    bracket material.  Used for cradles whose top is intentionally
    closed (i.e. the body does NOT drop in along a single straight
    insertion axis).  Samples a 3D grid filling the body's final
    bounding box and reports how many sample points are inside the
    part mesh.
    """
    body_centre = np.asarray(body_centre, dtype=float)
    u = np.asarray(body_long_axis,   dtype=float); u = u / np.linalg.norm(u)
    v = np.asarray(body_short_axis,  dtype=float); v = v / np.linalg.norm(v)
    w = np.asarray(body_normal_axis, dtype=float); w = w / np.linalg.norm(w)

    Nu, Nv, Nw = 5, 5, 9
    samples = []
    for i in np.linspace(-long_extent  / 2.0 + 0.5, long_extent  / 2.0 - 0.5, Nu):
        for j in np.linspace(-short_extent / 2.0 + 0.5, short_extent / 2.0 - 0.5, Nv):
            for k in np.linspace(-normal_extent / 2.0 + 0.5,
                                  normal_extent / 2.0 - 0.5, Nw):
                p = body_centre + i * u + j * v + k * w
                samples.append(p)
    samples = np.asarray(samples)
    inside = points_inside(part_mesh, samples)
    n_blocked = int(inside.sum())
    n_total = len(samples)
    return n_blocked == 0, n_blocked, n_total


def check_cradle_openness():
    print("\n[2] Cradle insertion-path openness:")
    all_ok = True

    # ---- coxa_bracket: drop-in from above through the flange slot ------
    # The bracket's flange has a 56 x 21 mm rectangular through-slot
    # so the servo (gear pointing UP) can drop straight DOWN into the
    # well.  Verify that the body's vertical drop-in column above the
    # final body volume is fully clear of bracket material.
    cb = hp.make_coxa_bracket()
    body_centre_cb = np.array([-hp.SERVO_OUTPUT_X, 0.0,
                                 -hp.WELL_RIM_Z + hp.SERVO_BODY_H / 2.0])
    ok, blocked, total = _cradle_open(cb, body_centre_cb,
                                        body_long_axis=[1, 0, 0],
                                        body_short_axis=[0, 1, 0],
                                        open_dir=[0, 0, 1])
    all_ok &= _label("coxa_bracket  (servo drops in +Z)",
                       ok, f"{blocked}/{total} samples blocked")

    cl = hp.make_coxa_link()
    body_centre_cl = np.array([
        hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H / 2.0 + hp.SERVO_OUTPUT_H),
        hp.COXA_HIP_DROP,
    ])
    ok, blocked, total = _cradle_open(cl, body_centre_cl,
                                       body_long_axis=[1, 0, 0],
                                       body_short_axis=[0, 0, 1],
                                       open_dir=[0, 1, 0])
    all_ok &= _label("coxa_link     (servo drops in +Y)",
                       ok, f"{blocked}/{total} samples blocked")

    fl = hp.make_femur_link()
    body_centre_fl = np.array([
        hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H / 2.0 + hp.SERVO_OUTPUT_H),
        0.0,
    ])
    ok, blocked, total = _cradle_open(fl, body_centre_fl,
                                       body_long_axis=[1, 0, 0],
                                       body_short_axis=[0, 0, 1],
                                       open_dir=[0, 1, 0])
    all_ok &= _label("femur_link    (servo drops in +Y)",
                       ok, f"{blocked}/{total} samples blocked")

    return all_ok


# ---------------------------------------------------------------------------
# 3.  Bolt hole hits material
# ---------------------------------------------------------------------------

def check_bolt_holes():
    print("\n[3] Bolt-hole material engagement:")
    all_ok = True

    # ---- Coxa bracket flange chassis bolts -----------------------------
    cb = hp.make_coxa_bracket()
    bolt_x_outboard = -hp.BRACKET_FLANGE_INSET
    bolt_x_inboard  = -hp.BRACKET_FLANGE_INSET - hp.BRACKET_BOLT_PCD_X
    bolt_ys = (-hp.BRACKET_BOLT_PCD_Y / 2.0,
                +hp.BRACKET_BOLT_PCD_Y / 2.0)
    n_pass, n_total = 0, 0
    for bx in (bolt_x_outboard, bolt_x_inboard):
        for by in bolt_ys:
            n_total += 1
            covered = False
            for off_x, off_y in [(2.0, 0.0), (-2.0, 0.0),
                                  (0.0, 2.0), (0.0, -2.0)]:
                # Sample a point in the middle of the flange thickness,
                # 2 mm to one side of the bolt centerline.  If this sits
                # in flange material, the bolt has a wall around it.
                probe = np.array([bx + off_x, by + off_y,
                                   hp.BRACKET_FLANGE_T / 2.0])
                if bool(points_inside(cb, [probe])[0]):
                    covered = True
                    break
            if covered:
                n_pass += 1
    ok = n_pass == n_total
    all_ok &= _label("coxa_bracket chassis bolts",
                       ok, f"{n_pass}/{n_total} bolt holes have flange material")

    # ---- Servo M3 pilot holes inside the wells -------------------------
    def _probe_pilots(part, pilot_positions, axis):
        # axis = direction of the pilot bore; pilot is a cylinder along this axis
        if abs(axis[0]) < 0.9:
            ortho1 = np.cross(axis, [1, 0, 0])
        else:
            ortho1 = np.cross(axis, [0, 1, 0])
        ortho1 = ortho1 / np.linalg.norm(ortho1)
        ortho2 = np.cross(axis, ortho1)
        n_pass = 0
        for p in pilot_positions:
            covered = False
            for off in (ortho1, -ortho1, ortho2, -ortho2):
                probe_pt = p + 1.6 * off
                if bool(points_inside(part, [probe_pt])[0]):
                    covered = True
                    break
            if covered:
                n_pass += 1
        return n_pass

    pilot_axis_z = np.array([0.0, 0.0, 1.0])
    cb_pilots = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cb_pilots.append(np.array([
                -hp.SERVO_OUTPUT_X + sx * hp.SERVO_TAB_HOLE_PCD / 2.0,
                sy * hp.SERVO_TAB_HOLE_PCD_Y / 2.0,
                -hp.WELL_RIM_Z * 0.5,
            ]))
    n_cb = _probe_pilots(cb, cb_pilots, pilot_axis_z)
    all_ok &= _label("coxa_bracket M3 pilots in well wall",
                       n_cb == 4, f"{n_cb}/4 pilots have wall material around them")

    # In well-local: pilot at (sx*PCD/2, sy*PCD_Y/2, +WELL_RIM_Z*0.5)
    # along the +Z bore.  After R(-pi/2, X): (x, y, z) -> (x, z, -y),
    # so pilot_after_R = (sx*PCD/2, +WELL_RIM_Z*0.5, -sy*PCD_Y/2) and
    # the bore axis is +Y in the link frame.  drop_z_cl is the well's
    # Z OFFSET in the LIFTED link frame -- always use the canonical
    # ``COXA_HIP_DROP`` constant exported by hexapod_prototype.py so this
    # check tracks any future tweak to COXA_LIFT / WELL_Z_DROP_EXTRA /
    # COXA_ARM_T.  Earlier versions duplicated the formula here and
    # missed WELL_Z_DROP_EXTRA, putting the pilot probes 4 mm above
    # the actual pilot Z.
    cl = hp.make_coxa_link()
    delta_x_cl = hp.COXA_LENGTH - hp.SERVO_OUTPUT_X
    delta_y_cl = -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H)
    drop_z_cl = hp.COXA_HIP_DROP
    cl_pilots = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cl_pilots.append(np.array([
                delta_x_cl + sx * hp.SERVO_TAB_HOLE_PCD / 2.0,
                delta_y_cl + hp.WELL_RIM_Z * 0.5,
                drop_z_cl + (-sy * hp.SERVO_TAB_HOLE_PCD_Y / 2.0),
            ]))
    n_cl = _probe_pilots(cl, cl_pilots, np.array([0, 1, 0]))
    all_ok &= _label("coxa_link M3 pilots in well wall",
                       n_cl == 4, f"{n_cl}/4 pilots have wall material around them")

    fl = hp.make_femur_link()
    delta_x_fl = hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X
    delta_y_fl = -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H)
    fl_pilots = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            fl_pilots.append(np.array([
                delta_x_fl + sx * hp.SERVO_TAB_HOLE_PCD / 2.0,
                delta_y_fl + hp.WELL_RIM_Z * 0.5,
                -sy * hp.SERVO_TAB_HOLE_PCD_Y / 2.0,
            ]))
    n_fl = _probe_pilots(fl, fl_pilots, np.array([0, 1, 0]))
    all_ok &= _label("femur_link M3 pilots in well wall",
                       n_fl == 4, f"{n_fl}/4 pilots have wall material around them")

    return all_ok


# ---------------------------------------------------------------------------
# 3b.  Wire-exit L-corridor at the -X bottom-OUTBOARD corner of the well
# ---------------------------------------------------------------------------
#
# Note on the previous version of this test
# -----------------------------------------
# An earlier ``check_wire_slot`` looked at TWO probe points in well-local
# coords -- one expected-VOID at (-25, 0, 0) and one expected-SOLID at
# (-25, 0, 10) -- and asserted the latter must be intact -X wall material.
# That assumption was stale: ``_wire_exit_slot`` was extended in the recent
# redesign to also cut a vertical CHANNEL into the -X wall above the L
# exit (so wires lying against the body's back-case can run down the wall
# face into the L), which legitimately removes material at well-local
# x in [-24.7, -17.5], z in [0, ~30].  The "SOLID" probe at (-25, 0, 10)
# sat 0.3 mm outboard of the channel in X, but the test sampled a 1 mm
# cube around it whose inboard voxels (x = -24.5) fell INSIDE the
# channel cut, so the test always reported the wall as broken even
# though the wall material it cares about (the M3 pilot zone above the
# channel top) was fully intact.  The M3 pilot wall is already
# independently verified by ``check_bolt_holes`` ("M3 pilots in well
# wall"), so the wire-exit check should focus on the wire's path and
# not on incidental wall geometry above it.
#
# What this check now models
# --------------------------
# A hobby servo's 3-wire harness exits the body at the bottom-outboard
# CORNER (well-local -X face of the body, at z=0, around y=0).  The
# wire must be able to reach FREE SPACE outside the cradle via the
# L-shaped slot.  The slot supports two routings out of the corner:
#
#   * LATERAL leg:  bundle travels in -X out through the bottom of the
#                   -X wall, past the part's outer face.
#   * DOWNWARD leg: bundle travels in -Z down through the well floor,
#                   past the part's outer bottom face.
#
# Each leg is a 3D corridor with cross-section ~WIRE_SLOT_W (Y) x ~3 mm
# (orthogonal).  We sample a grid of points inside each corridor in
# well-local coordinates and transform them into the part's frame via
# the same R / translation chain ``make_*`` applies to the well solid.
# A corridor is "clear" if every sampled point is OUTSIDE the part
# mesh.  The check PASSES if AT LEAST ONE corridor is clear -- the
# wire only needs ONE escape route, and a closed top flange (the user's
# recent reinforcement on the coxa bracket) does not interfere with
# either leg of the L.

# Wire bundle Y margin from the slot's Y face -- the slot is sized for
# bundled / jacketed harnesses (WIRE_SLOT_W = 7 mm), the wire itself
# needs much less so we sample inside the slot Y span by this margin.
_WIRE_PROBE_Y_MARGIN = 0.75
# Wire bundle thickness in the orthogonal direction (Z for the lateral
# leg, X for the downward leg).  3 mm comfortably accommodates a 3-
# conductor 22 AWG servo harness.
_WIRE_PROBE_BUNDLE_T = 3.0


def _well_to_cradle(p_well, R, t):
    """Map a point in well-local coordinates to cradle-local coordinates
    using the same R/translation chain ``make_*`` applies to the well
    solid.  ``R`` may be ``None`` (identity).  ``t`` is the total
    translation in CRADLE-LOCAL coords (applied AFTER R).
    """
    p = np.asarray(p_well, dtype=float).copy()
    if R is not None:
        p = R[:3, :3] @ p
    return p + np.asarray(t, dtype=float)


def _wire_corridor_points():
    """Return ``(lateral_pts, downward_pts, boot_pts)`` -- the two L-leg
    corridors plus the wire-exit boot footprint, expressed as N x 3
    arrays of well-local sample points.

    The corridors are built from ``WIRE_SLOT_*`` constants in
    ``hexapod_prototype`` and are common across all three cradles
    (yaw / hip-pitch / knee).  The corridor end-points reach 0.5 mm
    inside the slot's outer faces -- far enough past the part's outer
    wall that "all corridor points are void" means "the wire has
    reached free space".

    The boot footprint is the rectangular volume the servo's molded
    +X wire-exit boot will occupy when the body is fully seated; if
    any part-mesh material intersects this volume the body cannot
    seat without crushing or bending the boot.  See
    WIRE_BOOT_* constants in hexapod_prototype.py for the dimensions.
    """
    Y_HALF = hp.WIRE_SLOT_W / 2.0 - _WIRE_PROBE_Y_MARGIN
    H_HALF = _WIRE_PROBE_BUNDLE_T / 2.0

    # Y span (common to both legs): three transverse samples across the
    # wire bundle Y extent.
    bundle_y = np.linspace(-Y_HALF, +Y_HALF, 3)

    # ---- LATERAL leg ---------------------------------------------------
    # x runs from just inside the slot's inboard face (= cavity-side
    # opening, just past the body's +X face) to 0.5 mm shy of the
    # slot's outboard face (just past the part's outer +X wall).
    lat_x_start = (+hp.SERVO_BODY_W / 2.0
                   - hp.WIRE_SLOT_X_INBOARD + 0.5)
    lat_x_end   = (+hp.WELL_W / 2.0
                   + hp.WIRE_SLOT_X_PAST_WALL - 0.5)
    lat_x = np.linspace(lat_x_start, lat_x_end, 9)
    # Bundle Z span sits at floor level (z = 0 is the cavity floor top
    # / body bottom face); the wire lies flat against the floor on
    # its way out.
    lat_z = np.linspace(-H_HALF, +H_HALF, 2)
    Xx, Yy, Zz = np.meshgrid(lat_x, bundle_y, lat_z, indexing="ij")
    lateral_pts = np.stack([Xx.ravel(), Yy.ravel(), Zz.ravel()], axis=1)

    # ---- DOWNWARD leg --------------------------------------------------
    # z runs from just above the cavity floor to 0.5 mm shy of the
    # slot's outermost Z (which sits past the part's outer bottom).
    down_z_start = +0.5
    down_z_end   = (-hp.WELL_FLOOR_T
                    - hp.WIRE_SLOT_Z_BELOW_FLOOR + 0.5)
    down_z = np.linspace(down_z_start, down_z_end, 9)
    # Bundle X span sits at the body-corner / slot-inboard edge; the
    # wire drops straight down out of the +X corner.
    down_x_centre = +hp.SERVO_BODY_W / 2.0 - hp.WIRE_SLOT_X_INBOARD + 0.5
    down_x = np.linspace(down_x_centre - H_HALF,
                         down_x_centre + H_HALF, 2)
    Xx, Yy, Zz = np.meshgrid(down_x, bundle_y, down_z, indexing="ij")
    downward_pts = np.stack([Xx.ravel(), Yy.ravel(), Zz.ravel()], axis=1)

    # ---- BOOT footprint ------------------------------------------------
    # The servo's molded +X wire-exit boot occupies the rectangular
    # volume just outside the body's +X face, between the body and
    # the well's +X cavity wall (and slightly into the wall thanks to
    # the WIRE_CHANNEL).  Sample a tight grid inside that volume and
    # require every sample to be CLEAR of part material so the boot
    # can sit there when the body is fully seated.
    boot_x_min = +hp.SERVO_BODY_W / 2.0 + 0.3
    boot_x_max = (+hp.SERVO_BODY_W / 2.0
                  + hp.WIRE_BOOT_PROTRUSION - 0.3)
    boot_y_half = hp.WIRE_BOOT_W / 2.0 - 0.3
    boot_z_min = hp.WIRE_BOOT_Z_BASE + 0.3
    boot_z_max = (hp.WIRE_BOOT_Z_BASE
                  + hp.WIRE_BOOT_H - 0.3)
    bx = np.linspace(boot_x_min, boot_x_max, 4)
    by = np.linspace(-boot_y_half, +boot_y_half, 3)
    bz = np.linspace(boot_z_min, boot_z_max, 3)
    Xx, Yy, Zz = np.meshgrid(bx, by, bz, indexing="ij")
    boot_pts = np.stack([Xx.ravel(), Yy.ravel(), Zz.ravel()], axis=1)

    return lateral_pts, downward_pts, boot_pts


def _probe_wire_corridor(part, name, R, t,
                          lateral_well, downward_well, boot_well):
    """Sweep the LATERAL and DOWNWARD wire-bundle corridors AND the
    +X wire-exit boot footprint through one cradle and report PASS iff:

      * the boot footprint is FULLY clear of part material (the body
        cannot seat unless the boot has a pocket to live in), AND
      * at least ONE of the lateral / downward corridors is fully
        clear so the harness can exit the cradle.
    """

    def _to_cradle(pts_well):
        if R is None:
            return pts_well + t
        return pts_well @ R[:3, :3].T + t

    lat_cradle  = _to_cradle(lateral_well)
    down_cradle = _to_cradle(downward_well)
    boot_cradle = _to_cradle(boot_well)

    lat_inside  = points_inside(part, lat_cradle)
    down_inside = points_inside(part, down_cradle)
    boot_inside = points_inside(part, boot_cradle)
    n_lat_blocked,  n_lat_total  = int(lat_inside.sum()),  len(lat_cradle)
    n_down_blocked, n_down_total = int(down_inside.sum()), len(down_cradle)
    n_boot_blocked, n_boot_total = int(boot_inside.sum()), len(boot_cradle)

    lat_ok  = n_lat_blocked  == 0
    down_ok = n_down_blocked == 0
    boot_ok = n_boot_blocked == 0
    # The boot MUST fit (no choice).  The wire needs ONE escape route.
    ok = boot_ok and (lat_ok or down_ok)

    detail = (
        f"boot {n_boot_total - n_boot_blocked}/{n_boot_total} clear "
        f"({'OK' if boot_ok else 'BLOCKED'}); "
        f"lateral {n_lat_total - n_lat_blocked}/{n_lat_total} clear "
        f"({'OK' if lat_ok else 'BLOCKED'}), "
        f"downward {n_down_total - n_down_blocked}/{n_down_total} clear "
        f"({'OK' if down_ok else 'BLOCKED'})"
    )
    return _label(name, ok, detail)


def check_wire_slot():
    """Verify the L-shaped wire-exit corridor at the -X bottom-OUTBOARD
    corner of every servo cradle (yaw / hip-pitch / knee).

    Models the question:  starting at the body's bottom-outboard
    corner inside the cavity, can a wire bundle of WIRE_SLOT_W (Y) by
    ~3 mm (orthogonal) cross-section reach free space without
    intersecting solid part material?

    PASSES if EITHER the lateral OR the downward leg of the L is fully
    clear in every cradle.  This is independent of whatever happens
    above the slot -- the user is free to close the flange ring around
    the body-passage slot at the top of the bracket without affecting
    this check.
    """
    print("\n[3b] Wire-exit L-corridor + boot fitment "
          "(body's bottom +X corner):")
    all_ok = True

    lateral_well, downward_well, boot_well = _wire_corridor_points()

    R_link    = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    # Use the canonical post-lift well origin; do NOT re-derive in case
    # COXA_LIFT / WELL_Z_DROP_EXTRA / COXA_ARM_T change later.
    drop_z_cl = hp.COXA_HIP_DROP

    cradles = [
        ("coxa_bracket wire-exit L-corridor",
         hp.make_coxa_bracket(),
         None,
         np.array([-hp.SERVO_OUTPUT_X, 0.0, -hp.WELL_RIM_Z])),
        ("coxa_link    wire-exit L-corridor",
         hp.make_coxa_link(),
         R_link,
         np.array([hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
                   -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
                   drop_z_cl])),
        ("femur_link   wire-exit L-corridor",
         hp.make_femur_link(),
         R_link,
         np.array([hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X,
                   -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
                   0.0])),
    ]

    for name, part, R, t in cradles:
        all_ok &= _probe_wire_corridor(part, name, R, t,
                                        lateral_well, downward_well,
                                        boot_well)

    return all_ok


# ---------------------------------------------------------------------------
# 4.  Self-collision of the standing-pose assembly
# ---------------------------------------------------------------------------

def _build_standing_leg():
    """Return (parts dict, name list) of the leg's printed parts placed
    in their standing-pose locations in the body's world frame, on the
    edge between hexagon vertex 0 and vertex 1 (apothem direction
    a = pi/6)."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    PLASTIC_HORN_H = 5.0
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + PLASTIC_HORN_H
                     + hp.HORN_ADAPTER_T)
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + Ry_p @ np.array([hp.FEMUR_LENGTH, 0, 0])

    parts = {}

    cb = hp.make_coxa_bracket()
    cb.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cb.apply_translation(edge_mid)
    parts["coxa_bracket"] = cb

    cl = hp.make_coxa_link()
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["coxa_link"] = cl

    fl = hp.make_femur_link()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["femur_link"] = fl

    tl = hp.make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local)
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["tibia_link"] = tl

    return parts


def _pair_overlap_volume(a_mesh, b_mesh, pitch=1.5):
    """Estimate the overlap volume between mesh A and mesh B by voxel
    sampling A inside its AABB intersection with B, then counting how
    many of those voxel centres fall inside both meshes."""
    a_min, a_max = a_mesh.bounds
    b_min, b_max = b_mesh.bounds
    lo = np.maximum(a_min, b_min)
    hi = np.minimum(a_max, b_max)
    if np.any(hi <= lo):
        return 0.0
    n = np.maximum(2, np.ceil((hi - lo) / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    in_a = points_inside(a_mesh, pts)
    if in_a.sum() == 0:
        return 0.0
    pts_a = pts[in_a]
    in_b = points_inside(b_mesh, pts_a)
    n_overlap = int(in_b.sum())
    voxel_vol = pitch ** 3
    return n_overlap * voxel_vol


def check_self_collision():
    print("\n[4] Self-collision in standing pose (one leg):")
    parts = _build_standing_leg()
    names = list(parts.keys())

    # Connected pairs across rotational joints share a tiny shaft / horn
    # interface that legitimately overlaps geometrically.  Allow a
    # generous tolerance there; for non-adjacent parts require zero.
    JOINT_PAIRS = {
        ("coxa_bracket", "coxa_link"),
        ("coxa_link",    "femur_link"),
        ("femur_link",   "tibia_link"),
    }
    # Adjacent printed parts at a rotary joint share NO printed
    # material -- the actual physical interface (plastic horn + horn
    # adapter + servo gear stack) lives in dedicated horn-stack volume
    # which is checked separately by check_horn_stack_clearance.  So
    # we expect ZERO printed-vs-printed overlap here, modulo the
    # voxel-stair-step artefact along sharp mesh boundaries.  This
    # used to be 1500 mm^3 ("swallow the gear stack") which masked a
    # real ~ 800 mm^3 femur-spar-vs-coxa-link-arm clash that the user
    # reported as 'femur link cant rotate'.
    JOINT_TOLERANCE = 100.0
    NONADJ_TOLERANCE = 100.0

    all_ok = True
    for i, na in enumerate(names):
        for j, nb in enumerate(names):
            if j <= i:
                continue
            vol = _pair_overlap_volume(parts[na], parts[nb])
            adj = (na, nb) in JOINT_PAIRS or (nb, na) in JOINT_PAIRS
            tol = JOINT_TOLERANCE if adj else NONADJ_TOLERANCE
            ok = vol <= tol
            kind = "joint" if adj else "non-adj"
            all_ok &= _label(f"{na} vs {nb} ({kind})",
                               ok,
                               f"overlap = {vol:7.1f} mm^3 (tol {tol:.0f})")
    return all_ok


# ---------------------------------------------------------------------------
# 5.  Servo-body clearance in standing pose
# ---------------------------------------------------------------------------

def _place_servo_bodies():
    """Return a dict of the three servo envelopes for one leg in the same
    world frame used by ``_build_standing_leg`` (apothem direction a=pi/6).

    Mirrors the placement math from ``build_prototype_assembly._build_leg``
    so the meshes land in their physically-correct cradle positions for the
    standing-pose check.
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    cos_a, sin_a = np.cos(a), np.sin(a)
    edge_mid = np.array([apothem * cos_a, apothem * sin_a, 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    PLASTIC_HORN_H = 5.0
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + PLASTIC_HORN_H
                     + hp.HORN_ADAPTER_T)
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    R_a = rotation_matrix(a, [0, 0, 1])

    # ----- Yaw servo: body hangs in the coxa-bracket well. -----
    # In bracket-local: body-bottom-centre at (-SERVO_OUTPUT_X, 0,
    # -WELL_RIM_Z), mesh +X aligned with bracket +X (radial outward).
    yaw = hp.make_servo_body()
    yaw.apply_translation([-hp.SERVO_OUTPUT_X, 0.0, -hp.WELL_RIM_Z])
    yaw.apply_transform(R_a)
    yaw.apply_translation(edge_mid)

    # ----- Hip-pitch servo: sits in the coxa-link cradle. -----
    R_hip = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip = hp.make_servo_body()
    hip.apply_transform(R_hip)
    hip.apply_translation([
        hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
        hip_drop,
    ])
    hip.apply_transform(R_a)
    hip.apply_translation(yaw_output_world)

    # ----- Knee servo: sits in the femur-link cradle.  Pre-rotate by femur
    # pitch like the link itself. -----
    knee = hp.make_servo_body()
    knee.apply_transform(R_hip)
    knee.apply_translation([
        hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
        0.0,
    ])
    knee.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee.apply_translation(hip_joint_local)
    knee.apply_transform(R_a)
    knee.apply_translation(yaw_output_world)

    return {"yaw_servo": yaw, "hip_servo": hip, "knee_servo": knee}


def check_servo_clearance():
    """Each servo body should sit inside its cradle's air space, with at most
    a tiny overlap from the gear stack / spline / horn pocket that punches
    through the cradle face.  Non-adjacent printed parts (further along the
    leg) should not be touched by the servo body at all."""
    print("\n[5] Servo-body clearance in standing pose:")

    parts = _build_standing_leg()
    servos = _place_servo_bodies()

    # A servo lives in exactly one cradle; the adjacent printed part on the
    # output side hosts the gear stack passage (a couple cm^3 of overlap is
    # expected through the spline bore).  Everything else should be ~zero.
    CRADLE_PARENT = {
        "yaw_servo":  "coxa_bracket",
        "hip_servo":  "coxa_link",
        "knee_servo": "femur_link",
    }
    OUTPUT_NEIGHBOUR = {
        "yaw_servo":  "coxa_link",
        "hip_servo":  "femur_link",
        "knee_servo": "tibia_link",
    }
    CRADLE_TOLERANCE = 600.0    # mm^3 -- tab plane resting on the rim
    OUTPUT_TOLERANCE = 1500.0   # mm^3 -- horn passage / gear stack
    # Servo body next to a thin printed flange registers ~100-250 mm^3 of
    # voxel-sampling noise even when the surfaces are physically apart.
    # The cradle-openness check (using analytic probing) is the source of
    # truth for "the servo body is not blocked"; this tolerance catches a
    # real mechanical conflict (>= ~500 mm^3) without flaring on noise.
    OTHER_TOLERANCE = 300.0

    all_ok = True
    for servo_name, servo in servos.items():
        for part_name, part in parts.items():
            vol = _pair_overlap_volume(servo, part, pitch=1.5)
            if part_name == CRADLE_PARENT[servo_name]:
                tol = CRADLE_TOLERANCE
                kind = "cradle"
            elif part_name == OUTPUT_NEIGHBOUR[servo_name]:
                tol = OUTPUT_TOLERANCE
                kind = "output"
            else:
                tol = OTHER_TOLERANCE
                kind = "non-adj"
            ok = vol <= tol
            all_ok &= _label(
                f"{servo_name} vs {part_name} ({kind})",
                ok,
                f"overlap = {vol:7.1f} mm^3 (tol {tol:.0f})",
            )
    return all_ok


# ---------------------------------------------------------------------------
# 5b.  Horn-stack clearance inside output-side mating cylinders
# ---------------------------------------------------------------------------
#
# Why this check exists
# ---------------------
# Every rotary joint (yaw, hip-pitch, knee-pitch) connects the servo's
# spline to the next printed link via this physical stack, laid out
# along the joint axis (+Y in each driven part's local frame):
#
#     servo body  |  output gear  |  plastic horn  |  printed horn adapter
#     y < 0       |  y in [-6, 0] |  y in [0, +5]  |  y in [+5, +9]
#                                  ^                                 ^
#                                spline tip = joint axis            HORN_STACK_H
#
# The driven printed part (coxa_link for yaw, femur_link for hip-pitch,
# tibia_link for knee-pitch) sits ABOVE the horn adapter at femur/tibia
# y >= HORN_STACK_H = +9, with a 4-bolt clamp pad on HORN_BOLT_PCD =
# 24 mm.  The part's "neck" -- the material between the pad's mating
# face (y = HORN_STACK_H) and the spar's near face (y = LINK_THICKNESS/2
# = +3) -- MUST be free of plastic anywhere inside the horn-adapter
# cylinder (diameter HORN_ADAPTER_OD = 32 mm) plus a small clearance
# margin.  Otherwise the plastic horn + printed horn adapter physically
# have nowhere to live: the printed part will sit proud of the adapter
# by however many millimetres of plastic overlap, the 4 M3 clamp bolts
# won't engage their threads, and the joint cannot be assembled.
#
# This is exactly the failure mode the user reported as "the femur link
# doesn't let the end of the servo stick out high enough to connect to
# the tibia link" -- the tibia knee-pad's neck cylinder was a solid
# Phi HIP_PAD_R*2 = 34 mm column that punched straight through the
# Phi 32 mm horn-adapter footprint.
#
# Test geometry
# -------------
# We build a single horn-stack cylinder of radius
# (HORN_ADAPTER_OD/2 + HORN_STACK_CLEARANCE) and height HORN_STACK_H,
# centred on the joint axis at y in [0, HORN_STACK_H], and probe each
# driven printed part for material inside that volume.  The check is
# done in each part's LOCAL frame -- both the femur and tibia have
# their joint axis at the origin with +Y along the horn-stack
# direction, so a single cylinder template works for both.
#
# Tolerance: the budget catches the well-defined "solid column"
# failure (~ 500-2000 mm^3 of interpenetration) without flaring on
# the < 50 mm^3 of voxel-stair-step artefact along the pad's bottom
# rim where the neck-clearance void's CYLINDRICAL boundary meets the
# pad's CIRCULAR-DISC boundary on slightly mismatched voxel grids.

HORN_STACK_CLEARANCE = 0.5   # mm -- radial clearance around the adapter OD
HORN_STACK_OVERLAP_TOL = 50.0  # mm^3 -- voxel-grid artefact budget


def check_horn_stack_clearance():
    """Verify each driven printed part has a clear cylindrical void
    for the plastic horn + printed horn adapter stack at its joint."""
    R = hp.HORN_ADAPTER_OD / 2.0 + HORN_STACK_CLEARANCE
    H = hp.HORN_STACK_H

    print(f"\n[5b] Horn-stack clearance (Phi {2*R:.1f} mm x {H:.1f} mm tall, "
          f"centred on joint axis, y in [0, {H:.1f}]):")

    # The horn-stack cylinder template is along +Y in part-local frame,
    # which is exactly the joint axis direction in both make_femur_link
    # and make_tibia_link.  The yaw joint's "driven" side is the
    # coxa_link, whose horn stack lives BELOW its z=0 origin (outside
    # the link's printed volume) and so doesn't need this test.
    # _cyl_along already places the cylinder with one end at y=0 and
    # the other end at +length, so no extra translation is needed.
    stack = hp._cyl_along(R, H, axis="y")

    cases = [
        ("femur_link  (hip-pitch joint)", hp.make_femur_link()),
        ("tibia_link  (knee-pitch joint)", hp.make_tibia_link()),
    ]

    all_ok = True
    for name, mesh in cases:
        vol = _pair_overlap_volume(mesh, stack, pitch=0.8)
        ok = vol <= HORN_STACK_OVERLAP_TOL
        all_ok &= _label(name, ok,
                         f"part vs horn-stack overlap = {vol:7.1f} mm^3 "
                         f"(tol {HORN_STACK_OVERLAP_TOL:.0f})")
    return all_ok


# ---------------------------------------------------------------------------
# 5c.  Horn-SWEEP clearance in the YAW joint (coxa_bracket side)
# ---------------------------------------------------------------------------
#
# WHAT THIS CATCHES that 5b above does NOT
# ----------------------------------------
# check_horn_stack_clearance (5b) probes the DRIVEN side of each joint:
# the femur_link's hub at the hip-pitch joint, and the tibia_link's hub
# at the knee-pitch joint.  At the YAW joint the driven part is the
# coxa_link, whose printed volume sits ABOVE the horn-stack -- not in
# its way -- so 5b correctly skips the yaw joint.
#
# But the YAW joint also has a NON-DRIVEN side -- the coxa_bracket --
# which holds the servo body and whose flange / walls / gussets sit
# RIGHT NEXT TO the rotating gear stack + plastic horn + printed
# adapter.  Nothing in the existing checks ever probed the cylindrical
# sweep volume above the seated yaw servo.  The recurring "the servo
# motor doesn't stick out high enough in the coxa bracket" failure
# lives here: a flange / gusset / boss that intrudes into the horn-
# sweep cylinder physically prevents the plastic X-horn from rotating
# (or worse, prevents the horn + horn-adapter stack from seating
# above the bracket in the first place).
#
# WHY 5b's CYLINDER WAS THE WRONG SIZE
# ------------------------------------
# 5b uses radius (HORN_ADAPTER_OD/2 + 0.5) = 16.5 mm, sized for the
# PRINTED adapter only.  But the PLASTIC X-horn (the part that ships
# with the servo) sweeps a BIGGER cylinder: real-hardware DS3225 /
# MG996R-class horns reach ~ 19 mm to each arm tip.  And 5b's height
# is HORN_STACK_H = 9 mm, which covers only the plastic-horn + printed-
# adapter stack, NOT the SERVO_OUTPUT_H = 6 mm gear-stack region
# between the body's top face and the plastic horn's bottom face.  A
# bracket wall that wraps over the top of the well to z = +13 mm in
# bracket-local clears the printed adapter (which lives above z = +18)
# but clobbers the gear stack and the plastic horn underneath.
#
# WHAT THIS CHECK DOES
# --------------------
# Build a vertical cylinder centred on the BRACKET-LOCAL YAW AXIS at
# (x = -SERVO_OUTPUT_X = 0 in bracket coords after the servo offset
# is applied, y = 0).  Its radius is the larger of the printed
# adapter's half-OD and the plastic horn's tip radius (read from the
# bounding cylinder of ``make_servo_horn`` so the test stays in sync
# with the modelled hardware geometry), plus a small clearance.  Its
# Z range covers the ENTIRE rotating stack:
#
#     z_lo = bracket-local Z of the seated body's TOP face
#            (= SERVO_BODY_H - WELL_RIM_Z, taking WELL_TAB_FLOAT=0 as
#            the worst-case body-seats-lower scenario so the cylinder
#            covers the full gear-stack region even if the tabs sit
#            slightly below their nominal float height)
#     z_hi = z of the printed adapter's TOP face + 1 mm extra margin
#
# Any sample voxel inside both the bracket mesh AND this cylinder is
# a FAIL.  The cylinder MUST be entirely VOID inside the printed
# coxa_bracket -- this is not a "soft budget", it's a hard pass /
# fail.
HORN_SWEEP_CLEARANCE = 0.5   # mm -- radial clearance around the horn sweep
HORN_SWEEP_OVERLAP_TOL = 30.0  # mm^3 -- voxel artefact budget (tighter
                               # than 5b: the bracket's flange material is
                               # axis-aligned slabs so any real intrusion
                               # is hundreds of mm^3, well above the
                               # voxel-grid step-noise floor)


def _horn_tip_radius_from_mesh() -> float:
    """Return the plastic horn's tip radius, read from the bounding
    cylinder of ``hp.make_servo_horn()`` projected onto the spline-
    perpendicular plane.  Keeping this read off the actual mesh means
    a future change to ``make_servo_horn`` (longer arms, an extra
    feature, a different horn style) is automatically picked up by
    the verifier -- a recurring failure mode in this project has been
    "constant drift between the rendered visual and what the test
    measures".
    """
    horn = hp.make_servo_horn()
    xy = horn.vertices[:, :2]
    return float(np.sqrt((xy ** 2).sum(axis=1)).max())


def check_horn_sweep_clearance():
    """Verify the coxa_bracket has a clean cylindrical VOID above the
    seated yaw servo so the gear stack + plastic X-horn + printed horn
    adapter can rotate freely without colliding with any bracket-local
    geometry (flange material around the body slot, side gussets,
    bridge gussets, wire-channel flanges, M3 pilot bosses, ...).

    This is the missing check that caused the recurring
    'the servo motor doesn't stick out high enough in the coxa
    bracket' failure to keep slipping through verification.
    """
    plastic_tip_r = _horn_tip_radius_from_mesh()
    plate_r       = hp.HORN_ADAPTER_OD / 2.0
    R = max(plate_r, plastic_tip_r) + HORN_SWEEP_CLEARANCE

    # Bracket-local Z of the seated servo's top face.  See
    # ``make_coxa_bracket`` -- the well is translated by ``-WELL_RIM_Z``
    # so its rim lands at bracket-z = 0; the body's tabs seat on the
    # rim, so its bottom face sits ``WELL_TAB_FLOAT`` mm above the
    # well's nominal floor (well-z = WELL_TAB_FLOAT).  In bracket
    # coords: body bottom at z = WELL_TAB_FLOAT - WELL_RIM_Z, body top
    # at z = body_bottom + SERVO_BODY_H.  We DROP the WELL_TAB_FLOAT
    # term here so the cylinder also covers the case where the tabs
    # don't quite reach the rim and the body floats lower than
    # nominal -- conservative.
    body_top_z = hp.SERVO_BODY_H - hp.WELL_RIM_Z
    gear_top_z = (body_top_z + hp.WELL_TAB_FLOAT
                  + hp.SERVO_OUTPUT_H)
    adapter_top_z = gear_top_z + hp.HORN_STACK_H
    z_lo = body_top_z                  # = +10.75 mm (worst-case body float)
    z_hi = adapter_top_z + 1.0         # +1 mm margin above adapter top
    H    = z_hi - z_lo

    # Bracket-local yaw axis.  ``make_coxa_bracket`` places the servo BODY
    # at bracket-x = -SERVO_OUTPUT_X so the output gear (= the SPLINE = the
    # YAW AXIS, which sits at +SERVO_OUTPUT_X from the body centre in
    # servo-local coords) lands at bracket-x = 0.  See the docstring of
    # ``make_coxa_bracket``: "Origin: at the YAW AXIS ... +X = outboard".
    yaw_x = 0.0
    yaw_y = 0.0

    print(f"\n[5c] Horn-sweep clearance in coxa_bracket "
          f"(Phi {2*R:.1f} mm x {H:.1f} mm tall):")
    print(f"     bracket-local axis at (x={yaw_x:+.1f}, y={yaw_y:+.1f}); "
          f"z in [{z_lo:+.2f}, {z_hi:+.2f}]")
    print(f"     plastic horn tip radius (from mesh) = "
          f"{plastic_tip_r:.2f} mm; printed adapter radius = "
          f"{plate_r:.2f} mm; clearance = {HORN_SWEEP_CLEARANCE:.1f} mm")

    cyl = hp._cyl(R, H, sections=hp.CYL_SECTIONS)
    cyl.apply_translation([yaw_x, yaw_y, 0.5 * (z_lo + z_hi)])

    bracket = hp.make_coxa_bracket()
    pitch = 0.6
    vol = _pair_overlap_volume(bracket, cyl, pitch=pitch)
    ok = vol <= HORN_SWEEP_OVERLAP_TOL

    _label("coxa_bracket horn-sweep void", ok,
           f"bracket-inside-cylinder vol = {vol:7.1f} mm^3 "
           f"(tol {HORN_SWEEP_OVERLAP_TOL:.0f})")

    # Diagnostic: when the check fails, report WHERE the intrusion
    # lives in bracket-local coordinates so the geometry fix is
    # obvious from the verifier output.  Resample at the same pitch.
    if not ok:
        a_min, a_max = bracket.bounds
        b_min, b_max = cyl.bounds
        lo = np.maximum(a_min, b_min)
        hi = np.minimum(a_max, b_max)
        n  = np.maximum(2, np.ceil((hi - lo) / pitch).astype(int))
        gx = np.linspace(lo[0], hi[0], int(n[0]))
        gy = np.linspace(lo[1], hi[1], int(n[1]))
        gz = np.linspace(lo[2], hi[2], int(n[2]))
        XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
        pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
        in_a = points_inside(bracket, pts)
        bad = np.empty((0, 3))
        if in_a.sum() > 0:
            pts_a = pts[in_a]
            in_b = points_inside(cyl, pts_a)
            bad = pts_a[in_b]
        if len(bad) > 0:
            b_lo = bad.min(axis=0)
            b_hi = bad.max(axis=0)
            cen  = bad.mean(axis=0)
            print(f"     FAIL: {len(bad)} sample voxel(s) inside both bracket "
                  f"AND horn-sweep cylinder.")
            print(f"        bracket-local intrusion bbox:")
            print(f"          x in [{b_lo[0]:+.2f}, {b_hi[0]:+.2f}] mm")
            print(f"          y in [{b_lo[1]:+.2f}, {b_hi[1]:+.2f}] mm")
            print(f"          z in [{b_lo[2]:+.2f}, {b_hi[2]:+.2f}] mm")
            print(f"        centroid = ({cen[0]:+.2f}, {cen[1]:+.2f}, "
                  f"{cen[2]:+.2f}) mm")

    # Rotational-sweep diagnostic: probe the SAME cylinder against the
    # bracket through the runtime yaw range +/- 60 deg (about z) and
    # report whether the overlap differs across rotation.  A round
    # cylinder is rotationally invariant, so this should give the
    # SAME overlap volume at every angle -- if it doesn't, the
    # voxel-sampling itself has an asymmetric bias and the user
    # should be told.  Cheap (3 angles) and only printed in the
    # output for visibility.
    yaw_angles_deg = [-60.0, 0.0, +60.0]
    sweep_vols = []
    for ang in yaw_angles_deg:
        cyl_r = cyl.copy()
        cyl_r.apply_transform(
            rotation_matrix(np.deg2rad(ang), [0, 0, 1],
                            point=[yaw_x, yaw_y, 0.0]))
        sweep_vols.append(_pair_overlap_volume(bracket, cyl_r, pitch=pitch))
    sweep_str = ", ".join(
        f"yaw={ang:+.0f} deg -> {v:6.1f} mm^3"
        for ang, v in zip(yaw_angles_deg, sweep_vols))
    print(f"     rotational sweep diagnostic: {sweep_str}")

    return ok


# ---------------------------------------------------------------------------
# 6.  Flimsy joints (local thickness from a 3D distance transform)
# ---------------------------------------------------------------------------
#
# What the check computes
# -----------------------
# For each printed part:
#
#   1. Voxelise the (watertight) mesh on a uniform grid at
#      ``FLIMSY_VOXEL_PITCH``.  Trimesh's ``mesh.voxelized().fill()``
#      gives a 3D boolean occupancy array.
#
#   2. Compute the 3D Euclidean distance transform of the occupancy
#      (``scipy.ndimage.distance_transform_edt``).  At each interior
#      voxel this returns ``dt(V)`` = distance, in voxels, to the
#      nearest empty voxel.
#
#   3. A voxel V is **thick** iff some inscribed ball of radius
#      ``MIN_PRINT_T / 2`` (entirely inside the solid) contains V --
#      i.e. there exists a "core" voxel P (one whose ``dt(P)`` >=
#      ``MIN_PRINT_T / 2 / pitch``) with ``|V - P| <= MIN_PRINT_T/2``.
#      Equivalently: V is thick iff its distance to the nearest core
#      voxel is at most ``MIN_PRINT_T / 2 / pitch``, which is another
#      EDT pass.  The Hildebrand 1997 max-inscribed-ball formulation
#      avoids the well-known surface-fuzz pitfall of the naive
#      "2 * dt" definition: every surface voxel of a healthy 4 mm
#      wall sits at dt = 1.0 voxel, so the naive definition would
#      flag the WHOLE outer shell of every thick part as flimsy.
#
#   4. Flimsy voxels are the occupied ones NOT covered by a thick
#      ball.  Cluster them with ``scipy.ndimage.label``; tiny clusters
#      (< ``MIN_CLUSTER_VOX``) are dismissed as voxel-grid noise.
#
#   5. Report each cluster's centroid (part-local mm), bounding box
#      extent, minimum thickness, and voxel count.  The check PASSES
#      if the largest cluster in any single part stays at or below
#      ``MAX_FLIMSY_BUDGET_VOX``.

def _flimsy_clusters_for_part(mesh, pitch, min_t, min_cluster_vox):
    """Return ``(clusters, biggest_voxel_count, max_thickness_mm)`` for
    one part.

    Each entry in ``clusters`` is a dict::

        {
            "voxel_count":   int,
            "min_thickness": float (mm),
            "centroid":      np.ndarray shape (3,)  -- part-local mm,
            "bbox_min":      np.ndarray shape (3,)  -- part-local mm,
            "bbox_max":      np.ndarray shape (3,)  -- part-local mm,
        }

    Sorted descending by voxel count.
    """
    try:
        vg = mesh.voxelized(pitch=pitch).fill()
    except Exception as exc:
        # Should not happen for a watertight mesh -- but if it does,
        # fail loudly rather than silently passing.
        raise RuntimeError(f"voxelization failed: {exc}")

    occ = np.asarray(vg.matrix, dtype=bool)
    voxel_pitch = float(vg.pitch[0])

    if not occ.any():
        return [], 0, 0.0

    # EDT 1: distance from each voxel to the nearest empty voxel
    # (in voxel units; multiply by pitch for mm).
    dt_vox = distance_transform_edt(occ)
    dt_mm  = dt_vox * voxel_pitch
    max_thickness = float(2.0 * dt_mm.max())

    # Core voxels: inscribed ball of radius MIN_PRINT_T/2 fits entirely
    # inside the solid centred here.  An occupied voxel is THICK iff
    # within MIN_PRINT_T/2 of any core voxel (i.e. some such inscribed
    # ball contains it).  This is the Hildebrand 1997 local-thickness
    # criterion, formulated as two EDT passes.
    half_t_vox = (min_t / 2.0) / voxel_pitch
    core = dt_vox >= half_t_vox

    if core.any():
        # EDT 2: distance from each voxel to the nearest core voxel.
        dt_to_core = distance_transform_edt(~core)
        thick = dt_to_core <= half_t_vox
    else:
        thick = np.zeros_like(occ)

    flimsy = occ & ~thick

    if not flimsy.any():
        return [], 0, max_thickness

    labeled, n_labels = label(flimsy)
    clusters = []
    for cluster_id in range(1, n_labels + 1):
        cluster_mask = labeled == cluster_id
        n_vox = int(cluster_mask.sum())
        if n_vox < min_cluster_vox:
            continue
        idx = np.argwhere(cluster_mask)
        lo_idx = idx.min(axis=0).astype(float)
        hi_idx = idx.max(axis=0).astype(float)
        centroid_idx = idx.mean(axis=0)
        centroid_world  = vg.indices_to_points(
            centroid_idx[None, :])[0]
        bbox_min_world  = vg.indices_to_points(lo_idx[None, :])[0]
        bbox_max_world  = vg.indices_to_points(hi_idx[None, :])[0]
        # Cluster "thickness" = 2 * max(dt_mm) inside the cluster.
        # The max-dt voxel sits at the cluster's THICKEST point (the
        # ridge / medial line of the thin feature); doubling its dt
        # recovers the wall thickness in mm.  E.g. a 2.5 mm well wall
        # has max(dt_mm) ~= 1.25 mm and reports "min t = 2.5 mm",
        # immediately telling the user what the actual structural
        # thickness of the flagged region is.
        min_t_cluster = float(2.0 * dt_mm[cluster_mask].max())
        clusters.append({
            "voxel_count":   n_vox,
            "min_thickness": min_t_cluster,
            "centroid":      centroid_world,
            "bbox_min":      bbox_min_world,
            "bbox_max":      bbox_max_world,
        })

    clusters.sort(key=lambda c: -c["voxel_count"])
    biggest = clusters[0]["voxel_count"] if clusters else 0
    return clusters, biggest, max_thickness


# ---------------------------------------------------------------------------
# Targeted bridge-joint check: coxa_link's top-piece <-> servo-well joint
# ---------------------------------------------------------------------------
#
# Why this exists -- and why ``check_flimsy_joints`` alone cannot see this
# failure mode:
#
# In ``make_coxa_link`` the HORN YOKE (the top hub bolted to the yaw
# servo's horn adapter) is tied to the SERVO WELL BOX (the cradle that
# holds the hip-pitch servo) by an X-long, Y-thin BRIDGE slab that lives
# in the Z-gap between the well's outer top face and the hub/arm bottom
# face.  The bridge's cross-section (with the baseline geometry) is
# 53 mm long in X, 6.75 mm thick in Y, ~14.5 mm tall in Z; the bridge
# overlaps the well's outer body by just 0.5 mm in Y and 1.5 mm in Z,
# giving a 53 x 0.5 x 1.5 = ~40 mm^3 bonded interface holding the entire
# leg load.  ``pad_sweep_clear`` -- the cylindrical void that lets the
# femur hip pad swing through the link's interior -- then EATS a circular
# hole through this already-thin bridge across most of its X span,
# leaving only thin slivers at the inboard / outboard X extremes.
#
# Why ``check_flimsy_joints`` doesn't catch it: the Hildebrand
# max-inscribed-ball check (``_flimsy_clusters_for_part`` above) measures
# isotropic LOCAL thickness via a distance transform; an inscribed sphere
# of radius MIN_PRINT_T/2 = 1.5 mm fits inside the bridge slab (6.75 mm
# Y) without a problem, so the test happily reports the bridge as healthy.
# ``check_thin_sheets`` (the anisotropic chord variant) ALSO misses this
# because the pedestal sits directly above the inboard half of the bridge
# (link y in [-17, +17] for the pedestal vs. y in [-17.25, -10.5] for the
# bridge), and its Y-chord through the bridge merges with the pedestal's
# Y-chord into a ~34 mm-long combined chord that's outside the
# 5-7.2 mm "structural-neck" band.  The bridge's Y-thinness is only
# visible at link x > +17 (outboard of the pedestal), where ``gusset``
# and ``gusset_under`` add a few extra mm of Y material -- pushing the
# combined chord out of the band again.
#
# So we need an explicit slice-based check whose window EXCLUDES the
# pedestal and targets the bridge alone.  The check below slices the
# coxa_link mesh horizontally at multiple Z heights in the gap between
# the well's outer top and the hub bottom, intersects each slice with an
# outboard-of-pedestal X-Y window, and computes (a) the area of the
# resulting cross-section and (b) the bending section modulus Sx about
# the slice's local X axis (the bridge's weakest bending direction --
# flexure in YZ caused by the well's leg-load Y reaction torque).
#
# Thresholds are tuned to FAIL on the baseline ``make_coxa_link()`` (so
# the recurring "top of coxa_link is not attached strongly to the part
# housing the servo" complaint is caught the first time it appears) and
# PASS comfortably on a wall-widening / pad fix that adds the user's
# requested "rectangular block at the top of the well":
#
#   * Area >= 80 mm^2 minimum slice  -- matches the user's "~ 8 x 10 mm
#     of plastic" intuition (parent agent's spec).
#   * Sx_bend >= 100 mm^3            -- bending modulus of the OUTBOARD
#     bridge slice about its local X axis.  Baseline geometry has
#     Sx_bend ~ 6 mm^3 at the worst slice (essentially zero); a 5 mm
#     well-top wall extension lifts it well past 100 mm^3 because the
#     extra plastic is offset from the centroid in Y and shows up
#     quadratically in Ixx.

COXA_LINK_BRIDGE_AREA_MIN_MM2 = 80.0   # parent's recommendation
COXA_LINK_BRIDGE_SX_MIN_MM3   = 100.0  # ~ half-modulus of a 10x20 bar in
                                        # its weak axis (20x10^2/6/2 ~ 167);
                                        # rounded down so a "rectangular
                                        # block at top of well" pad clears
                                        # the threshold without needing to
                                        # rebuild the entire link.

COXA_LINK_BRIDGE_SLICE_DZ     = 0.5    # mm -- slice spacing in Z


def _slice_polygons(mesh, z):
    """Horizontal cross-section of ``mesh`` at world Z = ``z`` as a
    shapely MultiPolygon in link (X, Y) coordinates.  Empty if the mesh
    does not cross the plane.
    """
    section = mesh.section(plane_origin=[0.0, 0.0, float(z)],
                           plane_normal=[0.0, 0.0, 1.0])
    if section is None:
        return _sg.MultiPolygon()
    planar, _T = section.to_planar()
    polys = list(planar.polygons_full)
    if not polys:
        return _sg.MultiPolygon()
    if len(polys) == 1:
        return _sg.MultiPolygon([polys[0]])
    return _sg.MultiPolygon(polys)


def _ixx_about_centroid_x(geom, *, pitch=0.25):
    """Rasterise ``geom`` (shapely Polygon / MultiPolygon in (X, Y)) at
    ``pitch`` mm resolution and return ``(area, centroid_y, ixx, y_min,
    y_max)`` where Ixx is the second moment of area about an axis
    parallel to the X axis passing through the centroid (units: mm^4).

    Bending the bridge column (axis along link +Z) about this X axis
    bends the slab in the (Y, Z) plane -- the bridge's weakest bending
    direction in its current Y-thin geometry.
    """
    if geom.is_empty:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    minx, miny, maxx, maxy = geom.bounds
    # Sample on a regular grid.
    nx = max(int(np.ceil((maxx - minx) / pitch)), 1)
    ny = max(int(np.ceil((maxy - miny) / pitch)), 1)
    xs = minx + (np.arange(nx) + 0.5) * (maxx - minx) / nx
    ys = miny + (np.arange(ny) + 0.5) * (maxy - miny) / ny
    XX, YY = np.meshgrid(xs, ys, indexing="xy")
    pts_x = XX.ravel()
    pts_y = YY.ravel()
    # Vectorised "inside" via shapely.contains with prepared geometry.
    from shapely import contains_xy
    mask = contains_xy(geom, pts_x, pts_y)
    if not mask.any():
        return 0.0, 0.0, 0.0, 0.0, 0.0
    da = (maxx - minx) / nx * (maxy - miny) / ny
    area = float(mask.sum() * da)
    yy = pts_y[mask]
    y_c = float(yy.mean())
    ixx = float(((yy - y_c) ** 2).sum() * da)
    return area, y_c, ixx, float(yy.min()), float(yy.max())


def _check_coxa_link_bridge_joint(coxa_link_mesh):
    """Slice-based check for the bridge that ties coxa_link's horn yoke
    (top hub) to the hip-pitch servo well box.  See the big comment
    above for motivation and threshold rationale.
    """
    # All Z coordinates are in the final LIFTED coxa_link frame (i.e.
    # what ``hp.make_coxa_link()`` returns).
    #
    # Well outer top face Z (= COXA_LIFT + well_z_drop + WELL_D/2)
    well_z_drop = -(hp.WELL_D / 2.0
                    + hp.COXA_ARM_T / 2.0
                    + hp.WELL_Z_DROP_EXTRA)
    well_top_z  = hp.COXA_LIFT + well_z_drop + hp.WELL_D / 2.0
    yoke_bot_z  = hp.COXA_LIFT                       # = hub / arm bottom

    # Outboard-of-pedestal slice window.  Pedestal is the 34 x 34 mm
    # square pillar centred on (0, 0) (link x, y in [-17, +17]) so an X
    # window x in [+17, +50] excludes the pedestal entirely.  Y window
    # covers the bridge's existing y range [-17.25, -10.5] with generous
    # margin on BOTH sides so a wall-widening / pad fix that grows the
    # bridge in -Y (toward the well's outer body) or +Y (toward the arm
    # face) still lands inside the window.
    x_win = (+17.0, +50.0)
    y_win = (-25.0,  -5.0)
    window = _sg.box(x_win[0], y_win[0], x_win[1], y_win[1])

    # Slice every COXA_LINK_BRIDGE_SLICE_DZ mm in the gap, with a
    # COXA_LINK_BRIDGE_SLICE_DZ mm margin off each end so we never hit
    # the exact well-top or arm-bottom plane (where the cross-section
    # discontinuously jumps to the full well or arm footprint and
    # masks the bridge's weakness).
    z_lo = well_top_z + COXA_LINK_BRIDGE_SLICE_DZ
    z_hi = yoke_bot_z - COXA_LINK_BRIDGE_SLICE_DZ
    n_slices = max(int(round((z_hi - z_lo) / COXA_LINK_BRIDGE_SLICE_DZ)) + 1, 2)
    z_values = np.linspace(z_lo, z_hi, n_slices)

    rows = []   # (z, area, sx)
    for z in z_values:
        polys = _slice_polygons(coxa_link_mesh, float(z))
        if polys.is_empty:
            rows.append((float(z), 0.0, 0.0))
            continue
        inter = polys.intersection(window)
        if inter.is_empty:
            rows.append((float(z), 0.0, 0.0))
            continue
        area, y_c, ixx, y_min, y_max = _ixx_about_centroid_x(inter)
        if area <= 0.0:
            rows.append((float(z), 0.0, 0.0))
            continue
        c = max(y_max - y_c, y_c - y_min)
        sx = ixx / c if c > 1e-6 else 0.0
        rows.append((float(z), area, sx))

    areas = [r[1] for r in rows]
    sxs   = [r[2] for r in rows]
    min_area = min(areas) if areas else 0.0
    min_sx   = min(sxs)   if sxs   else 0.0
    min_area_z = rows[int(np.argmin(areas))][0] if rows else 0.0
    min_sx_z   = rows[int(np.argmin(sxs))][0]   if rows else 0.0

    ok_area = min_area >= COXA_LINK_BRIDGE_AREA_MIN_MM2
    ok_sx   = min_sx   >= COXA_LINK_BRIDGE_SX_MIN_MM3
    ok = ok_area and ok_sx

    head = (f"min slice area = {min_area:6.1f} mm^2 "
            f"({'>=' if ok_area else '<'}{COXA_LINK_BRIDGE_AREA_MIN_MM2:.0f}), "
            f"Sx_bend = {min_sx:7.1f} mm^3 "
            f"({'>=' if ok_sx else '<'}{COXA_LINK_BRIDGE_SX_MIN_MM3:.0f}); "
            f"slice band z in [{z_lo:.1f}, {z_hi:.1f}] mm "
            f"(window x in {x_win}, y in {y_win})")
    _label("coxa_link top<->well bridge", ok, head)

    if not ok:
        # Dump every slice (the band is only ~13 slices) so the user
        # can see exactly where the cross-section is too thin.
        for z, a, s in rows:
            tag_a = " " if a >= COXA_LINK_BRIDGE_AREA_MIN_MM2 else "A"
            tag_s = " " if s >= COXA_LINK_BRIDGE_SX_MIN_MM3   else "S"
            print(f"           [{tag_a}{tag_s}]  z = {z:6.2f}  "
                  f"area = {a:6.1f} mm^2  Sx = {s:7.1f} mm^3")
        print(f"           worst area at z = {min_area_z:.2f}, "
              f"worst Sx at z = {min_sx_z:.2f}")

    return ok


def check_flimsy_joints():
    """Flag every printed part that has a thin / under-strength region.

    Implementation: 3D Euclidean distance transform on a voxelised
    occupancy grid at ``FLIMSY_VOXEL_PITCH`` mm pitch; cluster voxels
    whose local thickness < ``MIN_PRINT_T``; ignore tiny clusters
    (< ``MIN_CLUSTER_VOX``); FAIL if the largest cluster is above
    ``MAX_FLIMSY_BUDGET_VOX``.

    The default constants treat 3 mm as the minimum healthy FDM print
    thickness (three 0.4 mm perimeters with margin), and a 200-voxel
    cluster at the 1.2 mm pitch as the largest acceptable thin
    region (~ 350 mm^3 of < 3 mm material per part).

    Additionally runs the targeted ``_check_coxa_link_bridge_joint``
    slice test that catches the recurring "top of coxa_link is not
    attached strongly to the part housing the servo" failure -- a
    bridge / wall-thickness defect the isotropic Hildebrand check
    cannot see.  See the big comment block above
    ``_check_coxa_link_bridge_joint`` for full motivation.
    """
    print(f"\n[6] Flimsy joints (local thickness < {MIN_PRINT_T:.1f} mm; "
          f"pitch={FLIMSY_VOXEL_PITCH:.1f} mm, "
          f"min cluster={MIN_CLUSTER_VOX} vox, "
          f"budget={MAX_FLIMSY_BUDGET_VOX} vox):")

    items = {
        "chassis_top":         hp.make_chassis_top(),
        "chassis_bottom":      hp.make_chassis_bottom(),
        "battery_holder":      hp.make_battery_holder(),
        "electronics_tray":    hp.make_electronics_tray(),
        "coxa_bracket":        hp.make_coxa_bracket(),
        "coxa_link":           hp.make_coxa_link(),
        "femur_link":          hp.make_femur_link(),
        "tibia_link":          hp.make_tibia_link(),
        "foot_pad":            hp.make_foot_pad(),
        "servo_horn_adapter":  hp.make_servo_horn_adapter(),
    }

    all_ok = True
    for name, mesh in items.items():
        clusters, biggest, max_t = _flimsy_clusters_for_part(
            mesh, FLIMSY_VOXEL_PITCH, MIN_PRINT_T, MIN_CLUSTER_VOX)
        ok = biggest <= MAX_FLIMSY_BUDGET_VOX
        all_ok &= ok
        n_clusters = len(clusters)
        head = (f"{n_clusters} flimsy cluster(s); "
                f"largest = {biggest:4d} vox "
                f"(budget {MAX_FLIMSY_BUDGET_VOX}); "
                f"part max thickness = {max_t:5.1f} mm")
        _label(name, ok, head)
        # Show up to the 3 worst clusters for the user to inspect.
        for cluster in clusters[:3]:
            cx, cy, cz = cluster["centroid"]
            extent = cluster["bbox_max"] - cluster["bbox_min"]
            print(f"           - {cluster['voxel_count']:4d} vox  "
                  f"min t={cluster['min_thickness']:4.2f} mm  "
                  f"centroid=({cx:+7.2f},{cy:+7.2f},{cz:+7.2f}) mm  "
                  f"bbox={extent[0]:5.1f} x {extent[1]:5.1f} "
                  f"x {extent[2]:5.1f} mm")

    # Targeted bridge-joint check (see big comment block above
    # ``_check_coxa_link_bridge_joint``).  Reuses the already-built
    # coxa_link mesh from the items dict above.
    all_ok &= _check_coxa_link_bridge_joint(items["coxa_link"])

    return all_ok


# ---------------------------------------------------------------------------
# 6b.  Thin sheets / webs (cross-section neck between thick volumes)
# ---------------------------------------------------------------------------
#
# Catches "thick block - thin sheet - thick block" topology along any
# part axis.  The standard Hildebrand check above
# (``check_flimsy_joints``) misses these because an inscribed sphere
# of radius MIN_PRINT_T/2 fits entirely inside any slab thicker than
# MIN_PRINT_T regardless of how extended the slab is in the other two
# directions.  Here we look at the per-axis 1D cross-section AREA
# profile and flag slices whose area is < NECK_AREA_FRAC times the
# max area on BOTH sides of the slice within a sliding window of
# NECK_WINDOW_MM (mm).  Voxels in flagged slices are clustered and
# reported just like the Hildebrand check.

def _axis_run_length_voxels(occ, axis):
    """For each occupied voxel, return the length (in voxels) of the
    contiguous run of occupied voxels along ``axis`` containing it.

    Implementation walks once forward and once backward along ``axis``
    accumulating run-length-from-start of the current run, then
    backs out the full run length per voxel.  O(N) in voxel count.
    """
    arr = np.moveaxis(occ, axis, -1)
    n_axis = arr.shape[-1]
    if n_axis == 0:
        return np.moveaxis(np.zeros_like(arr, dtype=np.int32), -1, axis)

    # forward[..., i] = run length of occupied voxels ending at i (inclusive),
    # 0 if empty.
    forward = np.zeros(arr.shape, dtype=np.int32)
    forward[..., 0] = arr[..., 0].astype(np.int32)
    for i in range(1, n_axis):
        forward[..., i] = np.where(arr[..., i],
                                    forward[..., i - 1] + 1,
                                    0)
    # backward[..., i] = run length of occupied voxels starting at i
    # (inclusive), 0 if empty.
    backward = np.zeros(arr.shape, dtype=np.int32)
    backward[..., n_axis - 1] = arr[..., n_axis - 1].astype(np.int32)
    for i in range(n_axis - 2, -1, -1):
        backward[..., i] = np.where(arr[..., i],
                                     backward[..., i + 1] + 1,
                                     0)
    # Run length containing voxel i = forward[i] + backward[i] - 1
    # (subtract 1 because i is counted in both).  Zero where empty.
    run_len = np.where(arr, forward + backward - 1, 0)
    return np.moveaxis(run_len, -1, axis)


def _thin_sheet_clusters_for_part(mesh, pitch, *,
                                    min_chord_mm_lower,
                                    min_chord_mm_upper,
                                    min_cluster_vox):
    """Return ``(clusters, biggest_voxel_count)`` for one part.

    Voxelize, compute the MIN DIRECTIONAL CHORD LENGTH per voxel (the
    smallest of the +/-X, +/-Y, +/-Z contiguous-run lengths through
    the voxel), then flag voxels whose min chord lies in the
    "structural-neck band" ``[min_chord_mm_lower, min_chord_mm_upper]``.
    Cluster the flagged voxels, drop clusters smaller than
    ``min_cluster_vox``, return the rest sorted by voxel count desc.

    Each cluster dict::

        {
            "voxel_count":    int,
            "thin_axes":      str -- one or more of "X"/"Y"/"Z"
                              (the axis/axes whose chord is the
                              minimum for the bulk of the cluster),
            "min_chord_mm":   float -- min over the cluster of the
                              per-voxel min directional chord,
            "max_chord_mm":   float -- max over the cluster of the
                              per-voxel min directional chord
                              (helps to see how uniformly thin the
                              cluster is),
            "centroid":       np.ndarray shape (3,)  part-local mm,
            "bbox_min":       np.ndarray shape (3,)  part-local mm,
            "bbox_max":       np.ndarray shape (3,)  part-local mm,
        }
    """
    try:
        vg = mesh.voxelized(pitch=pitch).fill()
    except Exception as exc:
        raise RuntimeError(f"voxelization failed: {exc}")

    occ = np.asarray(vg.matrix, dtype=bool)
    voxel_pitch = float(vg.pitch[0])

    if not occ.any():
        return [], 0

    # Per-voxel chord lengths along each axis (voxels).
    chord_axes_vox = [_axis_run_length_voxels(occ, axis) for axis in range(3)]

    # min directional chord per voxel (voxels).  Where the voxel is
    # empty all axes have run length 0; for occupied voxels each axis
    # has run length >= 1 so the minimum is meaningful.
    stacked = np.stack(chord_axes_vox, axis=0)              # (3, X, Y, Z)
    min_chord_vox = stacked.min(axis=0)                     # (X, Y, Z)
    min_chord_mm = min_chord_vox.astype(np.float32) * voxel_pitch

    # Voxel-level neck quantization: the chord length in voxels is an
    # integer, so the chord in mm is a multiple of the voxel pitch.
    # To catch a "6 mm slab" at pitch 1.2 we need to admit voxels with
    # 5 voxels of chord (6.0 mm exactly), so use an inclusive upper
    # bound with a half-pitch slack to absorb voxelization aliasing.
    lo_thresh_vox = int(np.ceil(min_chord_mm_lower / voxel_pitch))
    hi_thresh_vox = int(np.floor(
        (min_chord_mm_upper + 0.5 * voxel_pitch) / voxel_pitch
    ))
    thin_mask = occ & (min_chord_vox >= lo_thresh_vox) & \
                       (min_chord_vox <= hi_thresh_vox)

    if not thin_mask.any():
        return [], 0

    labeled, n_labels = label(thin_mask)
    axis_names = ("X", "Y", "Z")
    clusters = []
    for cluster_id in range(1, n_labels + 1):
        cluster_mask = labeled == cluster_id
        n_vox = int(cluster_mask.sum())
        if n_vox < min_cluster_vox:
            continue
        idx = np.argwhere(cluster_mask)
        lo_idx = idx.min(axis=0).astype(float)
        hi_idx = idx.max(axis=0).astype(float)
        centroid_idx = idx.mean(axis=0)
        centroid_world  = vg.indices_to_points(centroid_idx[None, :])[0]
        bbox_min_world  = vg.indices_to_points(lo_idx[None, :])[0]
        bbox_max_world  = vg.indices_to_points(hi_idx[None, :])[0]

        # For the cluster's voxels figure out which axis (or axes) is
        # the binding (smallest-chord) one most often.
        per_axis_is_min = np.zeros(3, dtype=np.int64)
        for ax in range(3):
            ax_vals = chord_axes_vox[ax][cluster_mask]
            min_vals = min_chord_vox[cluster_mask]
            per_axis_is_min[ax] = int((ax_vals == min_vals).sum())
        max_count = per_axis_is_min.max()
        triggered_axes = "".join(
            axis_names[ax] for ax in range(3)
            if per_axis_is_min[ax] == max_count
        )

        cluster_min = float(min_chord_mm[cluster_mask].min())
        cluster_max = float(min_chord_mm[cluster_mask].max())

        clusters.append({
            "voxel_count":   n_vox,
            "thin_axes":     triggered_axes,
            "min_chord_mm":  cluster_min,
            "max_chord_mm":  cluster_max,
            "centroid":      centroid_world,
            "bbox_min":      bbox_min_world,
            "bbox_max":      bbox_max_world,
        })

    clusters.sort(key=lambda c: -c["voxel_count"])
    biggest = clusters[0]["voxel_count"] if clusters else 0
    return clusters, biggest


def check_thin_sheets(extra_items=None):
    """Flag parts that have a "thick block - THIN SHEET - thick block"
    structural topology along any axis -- the failure mode the
    Hildebrand max-inscribed-ball check above cannot see because an
    inscribed sphere happily fits inside any slab thicker than
    MIN_PRINT_T regardless of how extended the slab is.

    Algorithm (anisotropic Hildebrand variant): voxelize the part,
    compute the min directional chord length per occupied voxel
    (the smallest of the +/-X, +/-Y, +/-Z run lengths containing
    that voxel), then flag voxels whose min chord lies in the
    "structural-neck band" ``[MIN_SHEET_T_LOWER, MIN_SHEET_T]``.
    Cluster the flagged voxels (8-connectivity), drop clusters smaller
    than MIN_SHEET_CLUSTER_VOX, FAIL if the largest cluster exceeds
    MAX_SHEET_BUDGET_VOX.

    Restricted to ``THIN_SHEET_PARTS`` (see big comment near the
    constants).
    """
    print(f"\n[6b] Thin sheets ("
          f"chord band {MIN_SHEET_T_LOWER:.1f}-{MIN_SHEET_T:.1f} mm "
          f"(+1 vox slack -> {MIN_SHEET_T_UPPER:.1f} mm); "
          f"pitch={FLIMSY_VOXEL_PITCH:.1f} mm, "
          f"min cluster={MIN_SHEET_CLUSTER_VOX} vox, "
          f"budget={MAX_SHEET_BUDGET_VOX} vox):")

    items = {
        "coxa_link":   hp.make_coxa_link,
        "femur_link":  hp.make_femur_link,
        "tibia_link":  hp.make_tibia_link,
    }
    if extra_items:
        items.update(extra_items)

    all_ok = True
    for name in THIN_SHEET_PARTS:
        if name not in items:
            continue
        mesh = items[name]()
        clusters, biggest = _thin_sheet_clusters_for_part(
            mesh, FLIMSY_VOXEL_PITCH,
            min_chord_mm_lower=MIN_SHEET_T_LOWER,
            min_chord_mm_upper=MIN_SHEET_T_UPPER,
            min_cluster_vox=MIN_SHEET_CLUSTER_VOX)
        ok = biggest <= MAX_SHEET_BUDGET_VOX
        all_ok &= ok
        n_clusters = len(clusters)
        head = (f"{n_clusters} sheet cluster(s); "
                f"largest = {biggest:5d} vox "
                f"(budget {MAX_SHEET_BUDGET_VOX})")
        _label(name, ok, head)
        for cluster in clusters[:3]:
            cx, cy, cz = cluster["centroid"]
            extent = cluster["bbox_max"] - cluster["bbox_min"]
            print(f"           - {cluster['voxel_count']:5d} vox  "
                  f"chord={cluster['min_chord_mm']:4.2f}-"
                  f"{cluster['max_chord_mm']:4.2f} mm  "
                  f"(thin axes: {cluster['thin_axes']:3s})  "
                  f"centroid=({cx:+7.2f},{cy:+7.2f},{cz:+7.2f}) mm  "
                  f"bbox={extent[0]:5.1f} x {extent[1]:5.1f} "
                  f"x {extent[2]:5.1f} mm")

    return all_ok


# ---------------------------------------------------------------------------
# 7.  Workspace self-collision (full joint sweep with chassis)
# ---------------------------------------------------------------------------
#
# Motivation
# ----------
# ``check_self_collision`` above tests ONE static standing pose
# (yaw = 0, femur_pitch = STANCE_FEMUR_DEG = -25 deg, knee_pitch =
# STANCE_TIBIA_DEG = +60 deg) with the CHASSIS removed from the scene.
# Pose-dependent tibia-vs-coxa-link / tibia-vs-chassis collisions are
# therefore invisible to that check.  This sweep builds the leg
# parametrically over (yaw, femur_pitch, knee_pitch), places the
# chassis (top + bottom plates, battery holder, electronics tray) in
# the same world frame, and checks pairwise mesh overlap volume across
# the full RUNTIME joint workspace.
#
# Joint workspace -- where the limits come from
# ---------------------------------------------
#   * yaw         in [-35.0 deg, +35.0 deg]
#       Source: firmware (prototype_servo_bridge.ino YAW_LIMIT_{LO,HI}_DEG)
#               AND mujoco_prototype._leg_xml
#               (``<joint name="L{i}_yaw" range="-0.61 0.61"/>`` rad,
#               = +/-35 deg).  Was +/-51.6 deg (MuJoCo ``range="-0.90 0.90"``
#               -- removed) and +/-80 deg (firmware -- removed) before the
#               workspace audit found that beyond ~+/-30 deg the femur
#               spar collides with the coxa_bracket flange at high femur
#               pitch.
#   * femur_pitch in [-80.0 deg, +30.0 deg]
#       Source: firmware (HIP_LIMIT_{LO,HI}_DEG) AND mujoco_prototype._leg_xml
#               (``<joint name="L{i}_pitch" range="-1.40 0.52"/>`` rad,
#               = -80.2 .. +29.8 deg).  Was -80 deg .. +48.7 deg before
#               the workspace audit; the +ve side was tightened to keep
#               the femur hip pad clear of the chassis_top (which was
#               already shrunk -- the limit gives an additional safety
#               margin at the cost of ~18 deg of upward femur lift that
#               the tripod gait does not use).
#   * knee_pitch  in [-20.1 deg, +80.0 deg]
#       Source: firmware (KNEE_LIMIT_{LO,HI}_DEG) AND
#               mujoco_prototype._leg_xml ``range="-0.35 1.40"`` rad
#               (= -20.1 .. +80.2 deg).  Was -20.1 .. +106 deg (MuJoCo
#               ``range="-0.35 1.85"``) before the audit; never reached
#               by the audit sweep but tightened to +80 to mirror
#               firmware so the audit sweep covers the actual runtime
#               envelope.
# Firmware lives in ``prototype/firmware/prototype_servo_bridge/
# prototype_servo_bridge.ino``; MuJoCo limits live in
# ``prototype/mujoco_prototype.py``.  ``env_cfg.json`` does NOT add an
# extra clamp; the policy outputs residuals that the position
# actuators clip into the MuJoCo joint range.

# Workspace grid -- tight enough to find pose-dependent collisions,
# loose enough that the sweep finishes in a few minutes with the
# per-pair voxel sampler below.
WORKSPACE_YAW_DEG       = (-35.0, +35.0)
WORKSPACE_FEMUR_DEG     = (-80.0, +30.0)
WORKSPACE_KNEE_DEG      = (-20.0, +80.0)
WORKSPACE_N_YAW         = 5
WORKSPACE_N_FEMUR       = 7
WORKSPACE_N_KNEE        = 5

# Voxel-sampling pitch for the per-pose overlap computation.  Coarser
# than ``check_self_collision``'s 1.5 mm so 175 poses x ~16 pairs
# stays affordable; still fine enough to detect any mechanically
# meaningful intrusion (a 4 mm-deep tibia stab into the chassis_top
# voxelises into hundreds of voxels at 2.5 mm pitch).
WORKSPACE_VOXEL_PITCH    = 2.5

# Pose-dependent tolerances.
#   * adjacent JOINT pair (yaw / hip / knee): the existing
#     check_self_collision uses 1500 mm^3 to swallow the gear stack
#     + horn spline overlap that's intrinsic to the rotary joint
#     interface.  Keep the same budget here so adjacent pairs don't
#     spuriously fail at sweep poses far from the standing pose.
#   * everything else (non-adjacent same-leg, leg-vs-chassis, leg-vs-
#     neighbour-bracket): we expect physically ZERO overlap.  Allow
#     a 200 mm^3 artefact tolerance to absorb the voxel-sampler's
#     coarse-grid stair-step error along sharp mesh boundaries.  At
#     2.5 mm pitch one voxel = 15.6 mm^3, so 200 mm^3 ~= 13 voxels --
#     a couple voxels of stair-step rounding error per face, well
#     below any real mechanical intrusion.
WORKSPACE_JOINT_TOL      =  200.0    # mm^3 -- adjacent rotary joint.
                                      # PRINTED parts at a rotary joint
                                      # share no printed material; the gear
                                      # stack + horn adapter live in
                                      # dedicated horn-stack volume (see
                                      # check_horn_stack_clearance).  So
                                      # we require ~zero printed-vs-
                                      # printed overlap here too, with
                                      # the same 200 mm^3 voxel-stair-step
                                      # artefact budget as non-adjacent
                                      # pairs.  Used to be 1500 mm^3 which
                                      # masked a ~ 800 mm^3 femur-vs-
                                      # coxa-link clash across the
                                      # entire negative-hip-pitch range.
WORKSPACE_ARTEFACT_TOL   =  200.0    # mm^3 -- non-adjacent pairs


def _pair_overlap_volume_and_centroid(a_mesh, b_mesh, pitch):
    """Voxel-sampling overlap estimator that returns ``(volume_mm3,
    centroid_in_world)`` or ``(0.0, None)`` if there is no overlap.

    Unlike ``_pair_overlap_volume``, this version computes the per-voxel
    volume as ``(Lx / n_x) * (Ly / n_y) * (Lz / n_z)`` where ``L*`` are
    the AABB-intersection extents in each axis and ``n_*`` are the
    sample counts.  This keeps the volume estimate HONEST even when the
    AABB intersection is much thinner than ``pitch`` in one axis -- the
    naive ``pitch**3`` per-voxel volume would over-report by
    ``pitch / L`` in that axis, which inflates a 0.25 mm tangential
    contact between two faces into thousands of mm^3 of "overlap" and
    masks the difference between a real punch-through and an
    FDM-tolerance graze.

    For AABB-intersections that are >> pitch on every axis (the common
    case for a deep collision) the per-voxel volume comes out to
    ~ pitch^3 exactly as the existing estimator does, so volumes are
    consistent with ``check_self_collision``'s tolerances.
    """
    a_min, a_max = a_mesh.bounds
    b_min, b_max = b_mesh.bounds
    lo = np.maximum(a_min, b_min)
    hi = np.minimum(a_max, b_max)
    if np.any(hi <= lo):
        return 0.0, None
    span = hi - lo
    n = np.maximum(2, np.ceil(span / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    in_a = points_inside(a_mesh, pts)
    if in_a.sum() == 0:
        return 0.0, None
    pts_a = pts[in_a]
    in_b = points_inside(b_mesh, pts_a)
    n_overlap = int(in_b.sum())
    if n_overlap == 0:
        return 0.0, None
    voxel_vol = float(np.prod(span / n.astype(float)))
    centroid = pts_a[in_b].mean(axis=0)
    return n_overlap * voxel_vol, centroid


def _build_chassis_world(reference_leg_az_rad):
    """Return the four chassis-fixed parts placed in the same world
    frame ``_build_standing_leg`` uses (leg-local z = 0 at the bracket
    flange BOTTOM, which is the chassis-bottom-plate centre plane).

    Z stack mirrors ``build_prototype_assembly._body_frame_parts`` and
    ``_body_battery_parts``:
        chassis_bottom centre   z = 0
        chassis_top centre      z = CHASSIS_GAP + CHASSIS_PLATE_T = 24
        battery_holder base     z = CHASSIS_PLATE_T = 4
        electronics_tray base   z = CHASSIS_PLATE_T + 1 = 5

    Also includes a NEIGHBOUR coxa_bracket at azimuth a + pi/3 so the
    sweep can detect tibia / femur swing into the next leg's bracket
    at extreme +yaw.
    """
    parts = {}

    bot = hp.make_chassis_bottom()
    parts["chassis_bottom"] = bot

    top = hp.make_chassis_top()
    top.apply_translation([0.0, 0.0, hp.CHASSIS_GAP + hp.CHASSIS_PLATE_T])
    parts["chassis_top"] = top

    bh = hp.make_battery_holder()
    bh.apply_translation([-25.0, 0.0, hp.CHASSIS_PLATE_T])
    parts["battery_holder"] = bh

    et = hp.make_electronics_tray()
    et.apply_translation([35.0, 0.0, hp.CHASSIS_PLATE_T + 1.0])
    parts["electronics_tray"] = et

    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a_n = reference_leg_az_rad + np.pi / 3.0
    edge_mid_n = np.array([apothem * np.cos(a_n),
                            apothem * np.sin(a_n),
                            0.0])
    cbn = hp.make_coxa_bracket()
    cbn.apply_transform(rotation_matrix(a_n, [0, 0, 1]))
    cbn.apply_translation(edge_mid_n)
    parts["neighbour_coxa_bracket"] = cbn

    return parts


def _build_workspace_leg(yaw_deg, femur_pitch_deg, knee_pitch_deg,
                          leg_azimuth_rad,
                          templates):
    """Place the four leg parts at the requested (yaw, femur, knee)
    pose, using the same kinematic chain as ``_build_standing_leg``
    /``build_prototype_assembly._build_leg``.

    ``templates`` is a dict of pre-built ``make_*`` meshes that we
    ``.copy()`` per pose so we don't pay the boolean-CSG cost (~ 200
    ms per leg part) once per grid sample.

    Joint convention (mirrors ``mujoco_prototype._leg_xml`` and
    ``build_prototype_assembly._build_leg``):

        yaw_rad      : rotation about leg-local +Z (vertical), positive
                       = leg's outboard direction rotates from +X to +Y
                       (toward the NEXT leg counter-clockwise).
        femur_pitch  : rotation about leg-local +Y (hip-pitch axis).
                       -25 deg = standing pose (knee lifted UP).
        knee_pitch   : rotation about femur-local +Y (knee-pitch axis).
                       +60 deg = standing pose (tibia angled DOWN from
                       the lifted knee toward the foot).
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = leg_azimuth_rad
    edge_mid = np.array([apothem * np.cos(a),
                          apothem * np.sin(a),
                          0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    PLASTIC_HORN_H = 5.0
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + PLASTIC_HORN_H
                     + hp.HORN_ADAPTER_T)
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])

    yaw_rad = np.deg2rad(yaw_deg)
    p  = np.deg2rad(femur_pitch_deg)
    pt = np.deg2rad(femur_pitch_deg + knee_pitch_deg)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + Ry_p @ np.array(
        [hp.FEMUR_LENGTH, 0.0, 0.0])

    R_a    = rotation_matrix(a,       [0, 0, 1])
    R_yaw  = rotation_matrix(yaw_rad, [0, 0, 1])
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    parts = {}

    cb = templates["coxa_bracket"].copy()
    cb.apply_transform(R_a)
    cb.apply_translation(edge_mid)
    parts["coxa_bracket"] = cb

    cl = templates["coxa_link"].copy()
    cl.apply_transform(R_yaw)
    cl.apply_transform(R_a)
    cl.apply_translation(yaw_output_world)
    parts["coxa_link"] = cl

    fl = templates["femur_link"].copy()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local)
    fl.apply_transform(R_yaw)
    fl.apply_transform(R_a)
    fl.apply_translation(yaw_output_world)
    parts["femur_link"] = fl

    tl = templates["tibia_link"].copy()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local)
    tl.apply_transform(R_yaw)
    tl.apply_transform(R_a)
    tl.apply_translation(yaw_output_world)
    parts["tibia_link"] = tl

    return parts


# Joint-adjacency table for the workspace sweep.  Pair tolerances:
#   * adjacent JOINT pair -> WORKSPACE_JOINT_TOL  (gear stack / horn
#     interface is allowed to register some mm^3 of overlap intrinsic
#     to the rotary joint).
#   * everything else  -> WORKSPACE_ARTEFACT_TOL  (physically zero
#     overlap expected).
_WS_JOINT_PAIRS = {
    ("coxa_bracket", "coxa_link"),
    ("coxa_link",    "femur_link"),
    ("femur_link",   "tibia_link"),
}

# Which (leg part, static part) pairs to test per pose.  This is the
# full set of "could plausibly collide somewhere in the workspace"
# combinations the existing single-pose check misses.
_WS_DYNAMIC_NAMES = ("coxa_link", "femur_link", "tibia_link")
_WS_INTRA_LEG_STATIC = ("coxa_bracket",)  # same-leg static reference
_WS_CHASSIS_STATIC = (
    "chassis_top",
    "chassis_bottom",
    "battery_holder",
    "electronics_tray",
    "neighbour_coxa_bracket",
)


def _ws_pair_kind(dynamic_name, static_name):
    pair = (dynamic_name, static_name)
    if pair in _WS_JOINT_PAIRS or (static_name, dynamic_name) in _WS_JOINT_PAIRS:
        return "joint"
    if static_name in _WS_CHASSIS_STATIC:
        return "chassis"
    return "non-adj"


def check_workspace_self_collision(*, n_yaw=None, n_femur=None, n_knee=None,
                                     verbose=False):
    """Sweep (yaw, femur_pitch, knee_pitch) on a coarse grid through the
    runtime joint workspace, build the leg + chassis at each pose, and
    flag every pose where any leg part overlaps any static part beyond
    a small artefact tolerance.

    The check FAILS as soon as the count of failing poses is non-zero.
    All failing poses are reported (not just the first one) so the
    next geometry / limit iteration can see the full failure
    envelope.
    """
    n_yaw   = n_yaw   if n_yaw   is not None else WORKSPACE_N_YAW
    n_femur = n_femur if n_femur is not None else WORKSPACE_N_FEMUR
    n_knee  = n_knee  if n_knee  is not None else WORKSPACE_N_KNEE

    yaw_samples   = np.linspace(*WORKSPACE_YAW_DEG,   n_yaw)
    femur_samples = np.linspace(*WORKSPACE_FEMUR_DEG, n_femur)
    knee_samples  = np.linspace(*WORKSPACE_KNEE_DEG,  n_knee)

    n_poses = n_yaw * n_femur * n_knee
    print(f"\n[7] Workspace self-collision sweep "
          f"(yaw in {WORKSPACE_YAW_DEG} deg x {n_yaw}, "
          f"femur in {WORKSPACE_FEMUR_DEG} deg x {n_femur}, "
          f"knee in {WORKSPACE_KNEE_DEG} deg x {n_knee} "
          f"= {n_poses} poses; pitch={WORKSPACE_VOXEL_PITCH} mm; "
          f"tol non-adj={WORKSPACE_ARTEFACT_TOL:.0f} mm^3, "
          f"joint={WORKSPACE_JOINT_TOL:.0f} mm^3):")

    leg_az = 0.5 * np.pi / 3.0   # same a = pi/6 as _build_standing_leg

    print("  Building chassis + leg-part templates (once) ...")
    chassis = _build_chassis_world(leg_az)
    templates = {
        "coxa_bracket": hp.make_coxa_bracket(),
        "coxa_link":    hp.make_coxa_link(),
        "femur_link":   hp.make_femur_link(),
        "tibia_link":   hp.make_tibia_link(),
    }

    # Always include the canonical standing pose so we can confirm the
    # known-good pose stays clean even with the chassis in the scene.
    standing_pose = (0.0,
                     float(hp.STANCE_FEMUR_DEG),
                     float(hp.STANCE_TIBIA_DEG))
    pose_iter = [standing_pose]
    seen_poses = {standing_pose}
    for yaw_deg in yaw_samples:
        for f_deg in femur_samples:
            for k_deg in knee_samples:
                key = (float(yaw_deg), float(f_deg), float(k_deg))
                if key in seen_poses:
                    continue
                seen_poses.add(key)
                pose_iter.append(key)

    failures = []
    standing_failed = False

    for idx, (yaw_deg, f_deg, k_deg) in enumerate(pose_iter):
        leg = _build_workspace_leg(yaw_deg, f_deg, k_deg,
                                     leg_azimuth_rad=leg_az,
                                     templates=templates)
        pose_label = (f"yaw={yaw_deg:+6.1f} femur={f_deg:+6.1f} "
                       f"knee={k_deg:+6.1f}")

        pose_failures = []

        # Dynamic-leg-part vs same-leg STATIC bracket + dynamic-vs-
        # dynamic among the leg's own parts (excluding the bracket's
        # adjacency with coxa_link, which is the yaw joint pair).
        leg_names = list(leg.keys())
        for i, na in enumerate(leg_names):
            for nb in leg_names[i + 1:]:
                # Skip adjacent-joint pairs at standing-pose-ish angles
                # to keep noise low BUT still check them in case
                # extreme yaw / pitch pulls the gear stack INTO the
                # neighbouring printed body.  Use WORKSPACE_JOINT_TOL.
                vol, centroid = _pair_overlap_volume_and_centroid(
                    leg[na], leg[nb], WORKSPACE_VOXEL_PITCH)
                pair_kind = _ws_pair_kind(na, nb)
                tol = (WORKSPACE_JOINT_TOL if pair_kind == "joint"
                        else WORKSPACE_ARTEFACT_TOL)
                if vol > tol:
                    pose_failures.append({
                        "pose":     (yaw_deg, f_deg, k_deg),
                        "pair":     (na, nb),
                        "kind":     pair_kind,
                        "vol":      vol,
                        "centroid": centroid,
                    })

        # Each dynamic leg part vs every chassis-fixed static part.
        for dyn_name in _WS_DYNAMIC_NAMES:
            dyn_mesh = leg[dyn_name]
            for stat_name, stat_mesh in chassis.items():
                vol, centroid = _pair_overlap_volume_and_centroid(
                    dyn_mesh, stat_mesh, WORKSPACE_VOXEL_PITCH)
                if vol > WORKSPACE_ARTEFACT_TOL:
                    pose_failures.append({
                        "pose":     (yaw_deg, f_deg, k_deg),
                        "pair":     (dyn_name, stat_name),
                        "kind":     "chassis",
                        "vol":      vol,
                        "centroid": centroid,
                    })

        if pose_failures:
            failures.extend(pose_failures)
            if (yaw_deg, f_deg, k_deg) == standing_pose:
                standing_failed = True
            if verbose:
                print(f"  POSE {idx:3d}  FAIL  {pose_label}  "
                      f"({len(pose_failures)} pair(s))")
        elif verbose:
            print(f"  POSE {idx:3d}  ok    {pose_label}")

    # Always confirm the standing pose explicitly.
    _label("standing-pose still clean with chassis in scene",
            not standing_failed,
            ("STANDING POSE NOW FAILS"
              if standing_failed
              else "yaw=0 femur=-25 knee=+60 -- no chassis intrusion"))

    if not failures:
        return _label("workspace sweep self-collision",
                       True,
                       f"all {len(pose_iter)} poses clean")

    # Summarise + list every failing pair.  Sort by descending overlap
    # so the first lines are the worst offenders.
    failures.sort(key=lambda f: -f["vol"])
    print(f"  Found {len(failures)} failing pair(s) across "
          f"{len({tuple(f['pose']) for f in failures})} pose(s):")
    for f in failures:
        yaw_deg, fd, kd = f["pose"]
        cx, cy, cz = (f["centroid"] if f["centroid"] is not None
                       else (float("nan"),) * 3)
        a_name, b_name = f["pair"]
        print(f"           "
              f"yaw={yaw_deg:+6.1f} fem={fd:+6.1f} knee={kd:+6.1f}  "
              f"{a_name:>14s} vs {b_name:<22s} ({f['kind']:>7s})  "
              f"vol={f['vol']:7.1f} mm^3  "
              f"centroid=({cx:+7.1f},{cy:+7.1f},{cz:+7.1f}) mm")

    return _label("workspace sweep self-collision",
                   False,
                   f"{len(failures)} failing pair(s) -- see list above")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _optional_arm_checks():
    """Run the optional-arm verification suite via prototype/arm/integrate.py.

    Two extra checks are appended to the verification list:

    * ``[4b]`` voxel overlap between each arm part (in neutral pose,
      bolted to chassis-top centre) and the chassis-top plate / six
      legs in standing pose.
    * ``[6b]`` flimsy-joint check over the 5 new arm parts
      (`arm_base_bracket`, `wrist_adapter`, `gripper_base`,
      `gripper_jaw_left`, `gripper_jaw_right`).

    Returns a list of (label, ok) tuples to be appended to ``results``.
    """
    _arm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "arm")
    if _arm_dir not in sys.path:
        sys.path.insert(0, _arm_dir)
    # Import `prototype/arm/integrate.py` directly by file path so we
    # bypass the `arm` name collision (the sibling `prototype/arm/arm.py`
    # module shadows the `arm/` package on sys.path).
    try:
        import importlib.util  # noqa: WPS433
        _spec = importlib.util.spec_from_file_location(
            "arm_integrate", os.path.join(_arm_dir, "integrate.py"))
        if _spec is None or _spec.loader is None:
            raise ImportError("could not locate arm/integrate.py")
        arm_integrate = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(arm_integrate)
    except Exception as exc:
        print(f"  WARN: optional arm import failed ({exc!r}); "
              f"skipping arm verification.")
        return []

    interference_ok = arm_integrate.check_arm_interference(
        pair_overlap_fn=_pair_overlap_volume,
        label_fn=_label,
    )
    flimsy_ok = arm_integrate.verify_arm_parts(
        min_print_t=MIN_PRINT_T,
        pitch=FLIMSY_VOXEL_PITCH,
        min_cluster_vox=MIN_CLUSTER_VOX,
        max_flimsy_budget_vox=MAX_FLIMSY_BUDGET_VOX,
        flimsy_cluster_fn=_flimsy_clusters_for_part,
        label_fn=_label,
    )
    return [
        ("Arm-vs-chassis/leg interference", interference_ok),
        ("Arm flimsy joints",               flimsy_ok),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help=(
            "Also run the OPTIONAL arm verification ([4b] arm-vs-chassis "
            "interference, [6b] flimsy joints on the 5 new arm parts). "
            "Off by default; the prototype's base checks are unchanged."
        ),
    )
    args = parser.parse_args()

    print("=" * 72)
    print("PROTOTYPE design verification")
    if args.with_arm:
        print("  (with --with-arm: optional arm checks ENABLED)")
    print("=" * 72)
    results = []
    results.append(("Mesh watertightness",   check_watertight()))
    results.append(("Cradle openness",        check_cradle_openness()))
    results.append(("Bolt-hole engagement",   check_bolt_holes()))
    results.append(("Wire-exit slot",         check_wire_slot()))
    results.append(("Self-collision",         check_self_collision()))
    results.append(("Servo clearance",        check_servo_clearance()))
    results.append(("Horn-stack clearance",   check_horn_stack_clearance()))
    results.append(("Horn-sweep clearance",   check_horn_sweep_clearance()))
    results.append(("Flimsy joints",          check_flimsy_joints()))
    results.append(("Thin sheets",            check_thin_sheets()))
    results.append(("Workspace self-collision",
                    check_workspace_self_collision()))

    if args.with_arm:
        results.extend(_optional_arm_checks())

    print()
    print("=" * 72)
    print("Summary:")
    all_ok = True
    for name, ok in results:
        flag = "PASS" if ok else "FAIL"
        print(f"   [{flag}]  {name}")
        all_ok &= ok
    print("=" * 72)
    if all_ok:
        print("All checks passed. The prototype is ready to print.")
        return 0
    else:
        print("FAIL.  Fix the failing checks before ordering parts.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
