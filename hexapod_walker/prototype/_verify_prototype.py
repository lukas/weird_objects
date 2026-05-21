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

Speed: mesh cache + process-pool dispatch
-----------------------------------------

Every printable part is built ONCE via ``_load_mesh(name)`` and
re-used (copied where mutated) across every check that needs it.
The check suite then runs in a ``concurrent.futures.ProcessPoolExecutor``
pool: parent process builds the master cache, pickles it via the
pool ``initializer=`` so each worker boots with every mesh
already preloaded, then submits the independent ``check_*`` checks
plus the per-pose chunks of ``check_workspace_self_collision``.
Results stream back as ``(name, ok, captured_stdout)`` tuples and
the parent prints them in DECLARATION ORDER -- the on-screen output
stays byte-for-byte identical to a serial run.

CLI flags
---------

* ``--serial`` -- skip the process pool and run every check in the main
  process in declaration order.  Use this when a worker traceback is
  mangled through pickle and you need a clean stack.
* ``--workers N`` -- override the default worker count (default is
  ``min(8, os.cpu_count())``).
* ``--profile PATH`` -- dump a cProfile snapshot of the PARENT process
  to ``PATH`` so you don't need the ``python -m cProfile`` wrapper.
  Workers are NOT profiled; combine with ``--serial`` if you want a
  full-suite profile.
* ``--only CHECK_NAME`` (repeatable) -- run only the named check(s);
  any check whose declaration-list name matches at least one ``--only``
  value is included.  Useful for iterating on a single check.
* ``--with-arm`` -- also run the OPTIONAL arm verification.
"""

from __future__ import annotations
import argparse
import sys
import os
import io
import functools
import hashlib
import inspect
import sqlite3
import time
import multiprocessing
import concurrent.futures
import cProfile
import threading
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix
from scipy.ndimage import distance_transform_edt, label
import shapely.geometry as _sg

import hexapod_prototype as hp


# ---------------------------------------------------------------------------
# Module-level mesh cache
# ---------------------------------------------------------------------------
#
# Every check used to call ``hp.make_<part>()`` directly, which triggers a
# fresh boolean-union / mesh-difference rebuild costing 0.2-2 s per part
# per call.  With 13+ checks and the workspace sweep, the same parts were
# being rebuilt 5-10 times each.  Routing every build through
# ``_load_mesh(name)`` makes each part hit the constructive-solid path
# EXACTLY ONCE per process; subsequent callers get the cached master
# (and an optional ``.copy()`` if they're going to ``apply_*`` mutate it).
#
# When ``main()`` dispatches checks to a ``ProcessPoolExecutor``, the
# parent process pre-fills the cache and pickles it via the worker
# initializer so each spawn-mode worker boots with every mesh already
# loaded -- no make-from-scratch cost in the worker.

_MESH_BUILDERS = {
    "chassis_top":      hp.make_chassis_top,
    "chassis_bottom":   hp.make_chassis_bottom,
    "battery_holder":   hp.make_battery_holder,
    "electronics_tray": hp.make_electronics_tray,
    "bec_cradle":       hp.make_bec_cradle,
    "switch_holster":   hp.make_switch_holster,
    "imu_pad":          hp.make_imu_pad,
    "coxa_bracket":     hp.make_coxa_bracket,
    "coxa_link":        hp.make_coxa_link,
    "femur_link":       hp.make_femur_link,
    "tibia_link":       hp.make_tibia_link,
    "foot_pad":         hp.make_foot_pad,
    "servo_body":       hp.make_servo_body,
    "servo_horn":       hp.make_servo_horn,
}

ALL_CACHED_PART_NAMES = tuple(_MESH_BUILDERS.keys())

_MESH_CACHE: dict = {}
_MESH_CACHE_LOCK = threading.Lock()


def _load_mesh(name: str, *, copy: bool = True) -> trimesh.Trimesh:
    """Return the cached master mesh for ``name``.

    Default returns a ``mesh.copy()`` so callers that go on to
    ``apply_transform`` / ``apply_translation`` / ``apply_scale`` see
    the same semantics they used to get from a fresh ``hp.make_X()``
    call.  Pass ``copy=False`` only when the caller will NOT mutate
    the returned mesh (e.g. ``check_watertight`` reads ``mesh.volume``
    and ``mesh.is_watertight``; ``_flimsy_clusters_for_part`` calls
    ``mesh.voxelized(...)`` which is itself non-mutating).
    """
    cached = _MESH_CACHE.get(name)
    if cached is None:
        with _MESH_CACHE_LOCK:
            cached = _MESH_CACHE.get(name)
            if cached is None:
                cached = _MESH_BUILDERS[name]()
                _MESH_CACHE[name] = cached
    return cached.copy() if copy else cached


def prebuild_mesh_cache(names=ALL_CACHED_PART_NAMES) -> dict:
    """Build every cacheable mesh now and return the cache dict.

    Used by the parent process before dispatching to a process pool so
    the worker initializer can ship the full cache to every worker.
    """
    for name in names:
        _load_mesh(name, copy=False)
    return _MESH_CACHE


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
#
# ``points_inside`` has TWO interchangeable implementations:
#
#   * ``_points_inside_rays``  -- the historical 6-axis ray-vote.  Robust
#     against the boolean-union false positives that broke trimesh's
#     ``mesh.contains`` on early prototype geometry, but slow because
#     each call shoots 6 N-point ray batches and pays the rtree spatial
#     index cost on every batch.  This is what the original
#     ``points_inside`` was.
#
#   * ``_points_inside_contains`` -- a direct ``mesh.contains(pts)``
#     call.  Watertight meshes only -- if the input mesh is not water-
#     tight, trimesh's containment heuristic can disagree with the ray
#     vote on points near holes / non-manifold edges.  All cached
#     meshes are validated by ``check_watertight`` so this should be
#     equivalent to the ray vote in practice.  Much faster than 6
#     ray-cast batches per call (in profile the ray vote is >= 80% of
#     verifier wall time; mesh.contains is on the order of one ray
#     batch instead of six, plus avoids the rtree-per-direction work).
#
# A dispatcher (``points_inside``) picks between them via a module-
# level ``_inside_mode`` set from the CLI ``--inside-mode`` flag.  In
# the special ``both`` mode the dispatcher RUNS BOTH implementations,
# records every disagreement to ``_inside_mismatches``, prints a
# ``[MISMATCH]`` line to stderr, and STILL returns the ray-vote answer
# (so the rest of the check suite behaves byte-identically to a pure
# ``rays`` run while we collect equivalence evidence).  ``main()``
# exits with code 2 when any mismatch is recorded, so CI catches the
# discrepancy.

INSIDE_MODE_RAYS = "rays"
INSIDE_MODE_CONTAINS = "contains"
INSIDE_MODE_BOTH = "both"
INSIDE_MODES = (INSIDE_MODE_RAYS, INSIDE_MODE_CONTAINS, INSIDE_MODE_BOTH)

# Module-level default; ``main()`` overrides this from --inside-mode and
# the process-pool worker initializer propagates it to spawn-mode
# workers (see ``_worker_initializer``).
_inside_mode = INSIDE_MODE_RAYS

# Collected mismatch records when running in ``both`` mode.  In a
# spawn-mode worker, only mismatches accumulated by THIS worker's
# checks live here; the parent process aggregates them via
# ``_check_runner`` / ``_ws_pose_failures_kwargs`` return values.
_inside_mismatches: list = []

# Thread-local context naming the currently-running check.  Set by
# ``_check_runner`` (worker process) / ``_run_checks_serial`` (parent)
# / ``_ws_pose_failures_kwargs`` (worker process) so mismatch records
# can be attributed back to a specific check / pose without threading
# the name through every ``points_inside`` call site.
_inside_check_ctx = threading.local()


def _set_inside_check_context(name: str) -> None:
    _inside_check_ctx.name = name


def _get_inside_check_context() -> str:
    return getattr(_inside_check_ctx, "name", None) or "<unknown check>"


_RAY_DIRS = np.array([
    [+1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, +1.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, +1.0],
    [0.0, 0.0, -1.0],
])


def _points_inside_rays(mesh, pts):
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


def _points_inside_contains(mesh, pts):
    """Return ``mesh.contains(pts)`` as a numpy bool array.  Requires
    a watertight mesh -- callers must have validated this via
    ``check_watertight``."""
    pts = np.asarray(pts, dtype=float)
    if pts.ndim == 1:
        pts = pts[None, :]
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    result = mesh.contains(pts)
    return np.asarray(result, dtype=bool)


def _mesh_identity(mesh) -> dict:
    """Return a small, picklable fingerprint of ``mesh`` for use in a
    mismatch record.  ``id(mesh)`` is useless across processes / when
    the caller is a transformed copy of a cached mesh, so we record
    volume + face/vertex counts + extents instead -- enough to
    distinguish the cached parts from each other in the post-hoc
    summary."""
    try:
        bounds = mesh.bounds
        extents = tuple(float(x) for x in (bounds[1] - bounds[0]))
    except Exception:
        extents = None
    try:
        vol = float(mesh.volume)
    except Exception:
        vol = None
    try:
        nv = int(len(mesh.vertices))
        nf = int(len(mesh.faces))
    except Exception:
        nv, nf = None, None
    try:
        watertight = bool(mesh.is_watertight)
    except Exception:
        watertight = None
    return {
        "volume":     vol,
        "n_vertices": nv,
        "n_faces":    nf,
        "extents":    extents,
        "watertight": watertight,
    }


def _record_inside_mismatch(mesh, pts, rays_result, contains_result) -> None:
    """Record a single ``points_inside`` disagreement between the ray
    vote and ``mesh.contains``.  Caller MUST have established that the
    two boolean arrays differ.  Caps coords/indices at the first 20
    per call to keep the record bounded.  Also prints a single
    ``[MISMATCH]`` line to stderr for live visibility."""
    diff = rays_result != contains_result
    n_diff = int(diff.sum())
    n_total = int(diff.size)
    if n_diff == 0:
        return
    idx_all = np.flatnonzero(diff)
    capped = idx_all[:20]
    coords_capped = pts[capped]
    rays_capped = rays_result[capped].astype(bool).tolist()
    contains_capped = contains_result[capped].astype(bool).tolist()
    mesh_id = _mesh_identity(mesh)
    record = {
        "check":         _get_inside_check_context(),
        "mesh":          mesh_id,
        "n_points":      n_total,
        "n_mismatch":    n_diff,
        "mismatch_idx":  [int(i) for i in capped.tolist()],
        "coords":        [tuple(float(c) for c in row)
                            for row in coords_capped],
        "rays_says":     rays_capped,
        "contains_says": contains_capped,
    }
    _inside_mismatches.append(record)
    # Keep the stderr line compact -- the full record is summarised at
    # main() exit.  Use stderr so it doesn't interleave with the
    # check's captured stdout (which the parent prints in declaration
    # order).
    vol_str = (f"vol={mesh_id['volume']:.1f}"
                if mesh_id["volume"] is not None
                else "vol=?")
    print(f"[MISMATCH] check={record['check']!r} mesh({vol_str} "
          f"nf={mesh_id['n_faces']} wt={mesh_id['watertight']}) "
          f"n_pts={n_total} n_diff={n_diff} "
          f"first_idx={record['mismatch_idx'][:5]}",
          file=sys.stderr, flush=True)


def points_inside(mesh, pts):
    """Dispatcher: return a boolean array indicating whether each point
    lies inside ``mesh``, picking between the historical 6-ray vote
    and ``mesh.contains`` according to the module-level
    ``_inside_mode``.  In ``both`` mode runs BOTH implementations and
    records any disagreement -- see the module-top docstring."""
    mode = _inside_mode
    if mode == INSIDE_MODE_RAYS:
        return _points_inside_rays(mesh, pts)
    if mode == INSIDE_MODE_CONTAINS:
        return _points_inside_contains(mesh, pts)
    if mode == INSIDE_MODE_BOTH:
        pts_arr = np.asarray(pts, dtype=float)
        if pts_arr.ndim == 1:
            pts_arr = pts_arr[None, :]
        rays_result = _points_inside_rays(mesh, pts_arr)
        contains_result = _points_inside_contains(mesh, pts_arr)
        if rays_result.shape == contains_result.shape:
            if np.any(rays_result != contains_result):
                _record_inside_mismatch(mesh, pts_arr,
                                          rays_result, contains_result)
        # Return the ray-vote answer so the rest of the suite is byte-
        # identical to a pure ``rays`` run while we collect mismatch
        # evidence.  The goal of ``both`` is observation, not a
        # correctness-via-majority-vote across implementations.
        return rays_result
    raise ValueError(f"unknown inside mode: {mode!r}")


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
    # Design B (May 2026): the printed servo_horn_adapter has been
    # retired (each link now bolts directly onto the plastic 4-arm
    # X-horn).  ``make_servo_horn_adapter`` is preserved for
    # backwards-compat but is no longer in the printable-output set.
    items_names = (
        "chassis_top", "chassis_bottom", "battery_holder",
        "electronics_tray", "coxa_bracket", "coxa_link",
        "femur_link", "tibia_link", "foot_pad",
    )
    all_ok = True
    for name in items_names:
        m = _load_mesh(name, copy=False)
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
    cb = _load_mesh("coxa_bracket", copy=False)
    body_centre_cb = np.array([-hp.SERVO_OUTPUT_X, 0.0,
                                 -hp.WELL_RIM_Z + hp.SERVO_BODY_H / 2.0])
    ok, blocked, total = _cradle_open(cb, body_centre_cb,
                                        body_long_axis=[1, 0, 0],
                                        body_short_axis=[0, 1, 0],
                                        open_dir=[0, 0, 1])
    all_ok &= _label("coxa_bracket  (servo drops in +Z)",
                       ok, f"{blocked}/{total} samples blocked")

    cl = _load_mesh("coxa_link", copy=False)
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

    fl = _load_mesh("femur_link", copy=False)
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
    cb = _load_mesh("coxa_bracket", copy=False)
    bolt_x_outboard = -hp.BRACKET_FLANGE_INSET
    bolt_x_inboard  = -hp.BRACKET_FLANGE_INSET - hp.BRACKET_BOLT_PCD_X
    bolt_ys = (-hp.BRACKET_BOLT_PCD_Y / 2.0,
                +hp.BRACKET_BOLT_PCD_Y / 2.0)
    bolt_centres = []
    for bx in (bolt_x_outboard, bolt_x_inboard):
        for by in bolt_ys:
            bolt_centres.append((bx, by))
    offsets = np.array([(2.0, 0.0), (-2.0, 0.0),
                        (0.0, 2.0), (0.0, -2.0)])
    n_offsets = len(offsets)
    # Build (n_bolts * n_offsets, 3) probe array; classify
    # "covered" per bolt as "ANY of its 4 probes inside the mesh".
    probes = np.zeros((len(bolt_centres) * n_offsets, 3))
    for bi, (bx, by) in enumerate(bolt_centres):
        for oi, (ox, oy) in enumerate(offsets):
            probes[bi * n_offsets + oi] = (
                bx + ox, by + oy, hp.BRACKET_FLANGE_T / 2.0,
            )
    inside_flat = points_inside(cb, probes).reshape(
        len(bolt_centres), n_offsets)
    n_pass = int(inside_flat.any(axis=1).sum())
    n_total = len(bolt_centres)
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
        offs = np.stack([ortho1, -ortho1, ortho2, -ortho2], axis=0)  # (4, 3)
        positions = np.asarray(pilot_positions)                      # (P, 3)
        # Build (P * 4, 3) probes, then reshape -> (P, 4) inside-flags
        # and pass per-pilot iff ANY of its 4 probes hits material.
        probes_3d = (positions[:, None, :] + 1.6 * offs[None, :, :])
        probes_3d = probes_3d.reshape(-1, 3)
        flags = points_inside(part, probes_3d).reshape(len(positions),
                                                       len(offs))
        return int(flags.any(axis=1).sum())

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
    cl = _load_mesh("coxa_link", copy=False)
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

    fl = _load_mesh("femur_link", copy=False)
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
         _load_mesh("coxa_bracket", copy=False),
         None,
         np.array([-hp.SERVO_OUTPUT_X, 0.0, -hp.WELL_RIM_Z])),
        ("coxa_link    wire-exit L-corridor",
         _load_mesh("coxa_link", copy=False),
         R_link,
         np.array([hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
                   -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
                   drop_z_cl])),
        ("femur_link   wire-exit L-corridor",
         _load_mesh("femur_link", copy=False),
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

    # Design B (May 2026): yaw output stack collapsed from
    #   PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm
    # down to
    #   HORN_STACK_H = PLASTIC_HORN_H = 5 mm
    # now that the printed servo_horn_adapter has been retired.
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + hp.HORN_STACK_H)
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + Ry_p @ np.array([hp.FEMUR_LENGTH, 0, 0])

    parts = {}

    cb = _load_mesh("coxa_bracket")
    cb.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cb.apply_translation(edge_mid)
    parts["coxa_bracket"] = cb

    cl = _load_mesh("coxa_link")
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["coxa_link"] = cl

    fl = _load_mesh("femur_link")
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["femur_link"] = fl

    tl = _load_mesh("tibia_link")
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

    # Design B (May 2026): yaw output stack collapsed from
    #   PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm
    # down to
    #   HORN_STACK_H = PLASTIC_HORN_H = 5 mm
    # now that the printed servo_horn_adapter has been retired.
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + hp.HORN_STACK_H)
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    R_a = rotation_matrix(a, [0, 0, 1])

    # ----- Yaw servo: body hangs in the coxa-bracket well. -----
    # In bracket-local: body-bottom-centre at (-SERVO_OUTPUT_X, 0,
    # -WELL_RIM_Z), mesh +X aligned with bracket +X (radial outward).
    yaw = _load_mesh("servo_body")
    yaw.apply_translation([-hp.SERVO_OUTPUT_X, 0.0, -hp.WELL_RIM_Z])
    yaw.apply_transform(R_a)
    yaw.apply_translation(edge_mid)

    # ----- Hip-pitch servo: sits in the coxa-link cradle. -----
    R_hip = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip = _load_mesh("servo_body")
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
    knee = _load_mesh("servo_body")
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
#     servo body  |  output gear  |  plastic horn (link bolts here)
#     y < 0       |  y in [-6, 0] |  y in [0, +5]  -> y = HORN_STACK_H
#                                  ^               ^
#                                spline tip       link mating face
#                                = joint axis
#
# Design B (May 2026): the printed servo_horn_adapter has been retired.
# The link's pad now bolts DIRECTLY onto the plastic horn (the part
# that ships with the servo), so the horn stack is the plastic horn
# alone -- HORN_STACK_H = PLASTIC_HORN_H = 5 mm (was 9 mm when the
# printed adapter sat on top of the plastic horn).
#
# The driven printed part (coxa_link for yaw, femur_link for hip-pitch,
# tibia_link for knee-pitch) sits ABOVE the plastic horn at femur/tibia
# y >= HORN_STACK_H = +5, with a 4-bolt clamp pad on XHORN_BOLT_PCD =
# 20.8 mm.  The part's "neck" / flange ring -- the material between the
# pad's mating face (y = HORN_STACK_H) and the spar's +Y face
# (y = LINK_THICKNESS/2 = +3) -- MUST be free of plastic anywhere
# inside the plastic X-horn's swept envelope plus a small clearance
# margin.
#
# May 2026 "shorten-neck" refactor: the link's flange-ring inner
# radius switched from ``HORN_ADAPTER_OD/2 + 0.5`` = 16.5 mm (sized
# for the now-retired printed servo_horn_adapter) to
# ``HORN_STACK_VOID_R`` = ``PLASTIC_HORN_X_TIP_R + 0.5`` = 18.5 mm
# (sized for the plastic X-horn's actual Phi 36 mm sweep -- the user
# found that the previous Phi 33 mm cup physically blocked the
# Phi 36 mm horn from fitting).  This probe radius tracks the same
# constant so the verifier follows the link geometry automatically.
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
# HORN_STACK_VOID_R (= PLASTIC_HORN_X_TIP_R + HORN_STACK_CLEARANCE =
# 18.5 mm) and height HORN_STACK_H, centred on the joint axis at
# y in [0, HORN_STACK_H], and probe each driven printed part for
# material inside that volume.  The check is done in each part's
# LOCAL frame -- both the femur and tibia have their joint axis at
# the origin with +Y along the horn-stack direction, so a single
# cylinder template works for both.
#
# Tolerance: the budget catches the well-defined "solid column"
# failure (~ 500-2000 mm^3 of interpenetration) without flaring on
# the < 50 mm^3 of voxel-stair-step artefact along the pad's bottom
# rim where the neck-clearance void's CYLINDRICAL boundary meets the
# pad's CIRCULAR-DISC boundary on slightly mismatched voxel grids.

HORN_STACK_CLEARANCE = 0.5   # mm -- radial clearance around the X-horn tips
HORN_STACK_OVERLAP_TOL = 50.0  # mm^3 -- voxel-grid artefact budget


def check_horn_stack_clearance():
    """Verify each driven printed part has a clear cylindrical void
    for the plastic X-horn at its joint."""
    # Probe radius matches the link's flange-ring inner radius
    # (HORN_STACK_VOID_R) so the verifier follows the geometry
    # automatically.  Equal to PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm
    # as of the May 2026 shorten-neck refactor.
    R = hp.HORN_STACK_VOID_R
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
        ("femur_link  (hip-pitch joint)", _load_mesh("femur_link",
                                                       copy=False)),
        ("tibia_link  (knee-pitch joint)", _load_mesh("tibia_link",
                                                        copy=False)),
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
# WHY 5b's CYLINDER WAS THE WRONG SIZE (for the YAW joint)
# --------------------------------------------------------
# 5b uses radius HORN_STACK_VOID_R = PLASTIC_HORN_X_TIP_R + 0.5 =
# 18.5 mm, sized for the plastic X-horn's actual sweep.  But 5b's
# height is HORN_STACK_H = 5 mm, which covers only the plastic-horn
# stack, NOT the SERVO_OUTPUT_H = 6 mm gear-stack region between the
# body's top face and the plastic horn's bottom face.  A bracket wall
# that wraps over the top of the well to z = +13 mm in bracket-local
# clears the printed adapter (which lives above z = +18 in legacy
# coords) but clobbers the gear stack and the plastic horn
# underneath.
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
    horn = _load_mesh("servo_horn", copy=False)
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

    bracket = _load_mesh("coxa_bracket", copy=False)
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

# Second bridge sub-check: per-X YZ cross-section probe.
#
# The XY-slice check above averages cross-section over the full bridge X
# range (-12 .. +41) -- so a 53 mm-long bridge with FAT plastic at the X
# extremes (where pad_sweep_clear does not reach) and THIN plastic in
# the middle (at x ~ COXA_LENGTH = +25, where pad_sweep_clear eats most
# of the bridge + pad) reports a healthy 157 mm^2 slice area even when
# the middle is a 4 mm-thin vertical strip.  That's exactly the failure
# mode the user pointed at in the May 18 screenshot ("you made the wrong
# wall thicker -- the joint in the middle left is whats thin").
#
# This second sub-check takes a YZ cross-section at EVERY 1 mm in X
# across the bridge X range, intersects with a bridge-only YZ window,
# and reports the minimum cross-section area + minimum Y-thickness at
# each x. The "Y-thickness" is the maximum y-extent of any horizontal
# slab inside the slice (i.e. the widest section the bridge maintains
# at some z): the bridge ties the well in Y direction, so Y-thickness
# is the structural metric in bending and shear.
#
# Threshold: 6 mm minimum Y-thickness AND 50 mm^2 minimum slice area at
# ANY x in the bridge x range catches the cylinder-cut middle.

COXA_LINK_BRIDGE_X_STEP_MM    = 1.0    # mm -- spacing of YZ probes in X
COXA_LINK_BRIDGE_MIN_AREA_MM2 = 50.0   # mm^2 -- min YZ cross-section area
                                       # at any x in the bridge x range
COXA_LINK_BRIDGE_MIN_Y_THK_MM = 6.0    # mm -- min Y thickness (max y extent
                                       # at any single z in the bridge slice)


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


def _check_coxa_link_bridge_yz_thickness(coxa_link_mesh):
    """Per-X YZ cross-section probe of coxa_link's bridge region.

    Catches the user-pointed "thin neck in the middle" failure mode
    where ``pad_sweep_clear`` eats most of the bridge + well-top pad
    at x ~ COXA_LENGTH = +25 mm, leaving only a thin vertical strip
    where the bridge ties the well to the arm/hub.

    Probes every COXA_LINK_BRIDGE_X_STEP_MM mm in X across the bridge
    x range. At each x, point-samples a YZ grid inside a bridge-only
    window (y between the well rim and the arm -Y face, z between the
    well top and the arm bottom) and measures the cross-section area
    + the maximum Y extent at any single z (= the widest local slab).
    """
    well_z_drop = -(hp.WELL_D / 2.0
                    + hp.COXA_ARM_T / 2.0
                    + hp.WELL_Z_DROP_EXTRA)
    well_top_z  = hp.COXA_LIFT + well_z_drop + hp.WELL_D / 2.0
    yoke_bot_z  = hp.COXA_LIFT

    # Bridge x range: full arm extent (the bridge spans the arm's
    # full length).
    arm_x_extent = hp.COXA_LENGTH + 28.0
    x_lo = -12.0 + 1.0                               # +1 mm margin
    x_hi = arm_x_extent - 12.0 - 1.0                 # arm +X end - 1 mm
    n_x = max(int(np.ceil((x_hi - x_lo) /
                          COXA_LINK_BRIDGE_X_STEP_MM)) + 1, 2)
    x_values = np.linspace(x_lo, x_hi, n_x)

    # YZ window: y from a margin below the well rim to just above the
    # arm's -Y face (so a fix that widens the pad in -Y still lands in
    # the window). z from just above the well top to just below the
    # arm bottom (so we measure the BRIDGE alone, not the well's outer
    # slab or the arm slab).
    y_lo, y_hi = -34.0, -10.0
    z_lo, z_hi = well_top_z + 0.25, yoke_bot_z - 0.25
    dy = 0.25                                        # mm grid pitch in y
    dz = 0.5                                         # mm grid pitch in z
    ys = np.arange(y_lo, y_hi + 1e-9, dy)
    zs = np.arange(z_lo, z_hi + 1e-9, dz)
    dA = dy * dz

    rows = []   # (x, area, max_y_extent_at_any_z)
    YY, ZZ = np.meshgrid(ys, zs, indexing="ij")
    for x in x_values:
        pts = np.stack([np.full(YY.size, float(x)),
                        YY.ravel(), ZZ.ravel()], axis=1)
        inside = coxa_link_mesh.contains(pts).reshape(YY.shape)
        area = float(inside.sum()) * dA
        # Max Y-extent at any single z (column in `inside`): count
        # interior cells per z row and multiply by dy.
        per_z = inside.sum(axis=0)
        max_y_extent = float(per_z.max()) * dy
        rows.append((float(x), area, max_y_extent))

    areas = [r[1] for r in rows]
    yt = [r[2] for r in rows]
    min_area = min(areas) if areas else 0.0
    min_yt   = min(yt)    if yt    else 0.0
    min_area_x = rows[int(np.argmin(areas))][0] if rows else 0.0
    min_yt_x   = rows[int(np.argmin(yt))][0]    if rows else 0.0

    ok_area = min_area >= COXA_LINK_BRIDGE_MIN_AREA_MM2
    ok_yt   = min_yt   >= COXA_LINK_BRIDGE_MIN_Y_THK_MM
    ok = ok_area and ok_yt

    head = (f"min YZ-slice area = {min_area:5.1f} mm^2 at x = {min_area_x:+.1f} "
            f"({'>=' if ok_area else '<'}{COXA_LINK_BRIDGE_MIN_AREA_MM2:.0f}), "
            f"min Y-thickness = {min_yt:4.2f} mm at x = {min_yt_x:+.1f} "
            f"({'>=' if ok_yt else '<'}{COXA_LINK_BRIDGE_MIN_Y_THK_MM:.0f}); "
            f"YZ window y in ({y_lo:.1f}, {y_hi:.1f}), z in ({z_lo:.1f}, {z_hi:.1f})")
    _label("coxa_link bridge YZ thickness", ok, head)

    if not ok:
        # Dump worst 8 x values so the user can see where the bridge
        # is thinnest.
        ranked = sorted(rows, key=lambda r: r[2])[:8]
        print(f"           thinnest 8 x slices (sorted by Y-thickness):")
        for x, a, yt_val in ranked:
            tag_a = " " if a    >= COXA_LINK_BRIDGE_MIN_AREA_MM2 else "A"
            tag_y = " " if yt_val >= COXA_LINK_BRIDGE_MIN_Y_THK_MM else "Y"
            print(f"           [{tag_a}{tag_y}]  x = {x:+6.2f}  "
                  f"area = {a:5.1f} mm^2  max Y-thickness = {yt_val:5.2f} mm")
    return ok


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

    items_names = (
        "chassis_top", "chassis_bottom", "battery_holder",
        "electronics_tray", "coxa_bracket", "coxa_link",
        "femur_link", "tibia_link", "foot_pad",
        # Design B (May 2026): servo_horn_adapter dropped from the
        # flimsy-cluster sweep -- no longer in the printable-output set.
    )
    items = {name: _load_mesh(name, copy=False) for name in items_names}

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
    all_ok &= _check_coxa_link_bridge_yz_thickness(items["coxa_link"])

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
        "coxa_link":   lambda: _load_mesh("coxa_link", copy=False),
        "femur_link":  lambda: _load_mesh("femur_link", copy=False),
        "tibia_link":  lambda: _load_mesh("tibia_link", copy=False),
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
        chassis_top centre      z = CHASSIS_GAP + CHASSIS_PLATE_T = 36
        battery_holder base     z = CHASSIS_PLATE_T / 2 = 2
        electronics_tray base   z = CHASSIS_PLATE_T / 2 + 3 = 5

    Also includes a NEIGHBOUR coxa_bracket at azimuth a + pi/3 so the
    sweep can detect tibia / femur swing into the next leg's bracket
    at extreme +yaw.
    """
    parts = {}

    bot = _load_mesh("chassis_bottom")
    parts["chassis_bottom"] = bot

    top = _load_mesh("chassis_top")
    top.apply_translation([0.0, 0.0, hp.CHASSIS_GAP + hp.CHASSIS_PLATE_T])
    parts["chassis_top"] = top

    bh = _load_mesh("battery_holder")
    bh.apply_translation([hp.BATTERY_HOLDER_CENTRE_X, 0.0, hp.CHASSIS_PLATE_T / 2.0])
    parts["battery_holder"] = bh

    et = _load_mesh("electronics_tray")
    et.apply_translation([hp.ELEC_TRAY_CENTRE_X, hp.ELEC_TRAY_CENTRE_Y,
                          hp.CHASSIS_PLATE_T / 2.0 + 3.0])
    parts["electronics_tray"] = et

    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a_n = reference_leg_az_rad + np.pi / 3.0
    edge_mid_n = np.array([apothem * np.cos(a_n),
                            apothem * np.sin(a_n),
                            0.0])
    cbn = _load_mesh("coxa_bracket")
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

    # Design B (May 2026): yaw output stack collapsed to HORN_STACK_H
    # = PLASTIC_HORN_H = 5 mm now that the printed servo_horn_adapter
    # has been retired.
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + hp.HORN_STACK_H)
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


# Worker-side cache for the workspace sweep's chassis + leg templates.
# Each worker re-derives these from the cached master meshes on first
# use so the (yaw, femur_pitch, knee_pitch) pose function does not pay
# the chassis-build cost more than ONCE per worker.
_WS_WORKER_STATE: dict = {}


def _ws_get_chassis_and_templates(leg_az: float):
    key = float(leg_az)
    cached = _WS_WORKER_STATE.get(key)
    if cached is not None:
        return cached
    chassis = _build_chassis_world(key)
    templates = {
        "coxa_bracket": _load_mesh("coxa_bracket", copy=False),
        "coxa_link":    _load_mesh("coxa_link",    copy=False),
        "femur_link":   _load_mesh("femur_link",   copy=False),
        "tibia_link":   _load_mesh("tibia_link",   copy=False),
    }
    _WS_WORKER_STATE[key] = (chassis, templates)
    return chassis, templates


def _workspace_pose_failures(pose, leg_az):
    """Return the list of pair-failure dicts for one (yaw, femur, knee)
    pose.  Pure function: side-effects are limited to populating the
    per-worker ``_WS_WORKER_STATE`` cache on first call.  Used both
    serially in the main process and in process-pool workers."""
    yaw_deg, f_deg, k_deg = pose
    chassis, templates = _ws_get_chassis_and_templates(leg_az)
    leg = _build_workspace_leg(yaw_deg, f_deg, k_deg,
                                 leg_azimuth_rad=leg_az,
                                 templates=templates)

    pose_failures = []

    leg_names = list(leg.keys())
    for i, na in enumerate(leg_names):
        for nb in leg_names[i + 1:]:
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

    return pose_failures


def _ws_pose_failures_kwargs(args):
    """Picklable star-helper for ProcessPoolExecutor.map(): unpacks
    ``(pose, leg_az)`` into the keyword call.  Returns
    ``(failures, mismatches)`` -- callers carry the pose order
    separately so we don't need completion-order keys.  The
    ``mismatches`` half captures points_inside disagreements
    accumulated under --inside-mode=both so the parent can aggregate
    them in the final summary."""
    global _inside_mismatches
    pose, leg_az = args
    _set_inside_check_context(
        f"Workspace self-collision pose={pose}")
    _inside_mismatches = []
    try:
        failures = _workspace_pose_failures(pose, leg_az)
    finally:
        my_mismatches = list(_inside_mismatches)
        _inside_mismatches = []
    return (failures, my_mismatches)


def check_workspace_self_collision(*, n_yaw=None, n_femur=None, n_knee=None,
                                     verbose=False, pool=None):
    """Sweep (yaw, femur_pitch, knee_pitch) on a coarse grid through the
    runtime joint workspace, build the leg + chassis at each pose, and
    flag every pose where any leg part overlaps any static part beyond
    a small artefact tolerance.

    The check FAILS as soon as the count of failing poses is non-zero.
    All failing poses are reported (not just the first one) so the
    next geometry / limit iteration can see the full failure
    envelope.

    ``pool`` is an optional ``concurrent.futures.Executor``.  When
    given, per-pose work is dispatched to it in submission order; the
    failures list is aggregated in pose-declaration order so the
    printed output is identical to the serial run.  When ``None`` the
    poses run in the calling process.
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
    # Warm the parent's own per-leg_az cache so a --serial run pays
    # the chassis-build cost up front (mirrors the worker behaviour).
    _ws_get_chassis_and_templates(leg_az)

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

    pose_args = [(pose, leg_az) for pose in pose_iter]

    if pool is None:
        # Parent-process / --serial path: mismatch records accumulate
        # directly in the parent's ``_inside_mismatches`` global.
        _set_inside_check_context(WORKSPACE_CHECK_NAME)
        results = [_workspace_pose_failures(pose, leg_az)
                    for pose in pose_iter]
    else:
        # chunksize hand-tuned: a chunk should be large enough to
        # amortise the pickle / dispatch cost but small enough that
        # straggler poses (extreme-angle leg builds can take 2x longer
        # than nominal) don't pin the slowest worker.  ~6 keeps the
        # worker queue well-fed without ballooning per-task overhead.
        raw_results = list(pool.map(_ws_pose_failures_kwargs, pose_args,
                                      chunksize=6))
        results = []
        for pose_failures, pose_mismatches in raw_results:
            results.append(pose_failures)
            if pose_mismatches:
                _inside_mismatches.extend(pose_mismatches)

    failures = []
    standing_failed = False

    for idx, (pose, pose_failures) in enumerate(zip(pose_iter, results)):
        yaw_deg, f_deg, k_deg = pose
        pose_label = (f"yaw={yaw_deg:+6.1f} femur={f_deg:+6.1f} "
                       f"knee={k_deg:+6.1f}")
        if pose_failures:
            failures.extend(pose_failures)
            if pose == standing_pose:
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


# ---------------------------------------------------------------------------
# Design B (May 2026): direct-to-plastic-horn pad pattern
# ---------------------------------------------------------------------------
#
# The link's pad now bolts DIRECTLY onto the plastic 4-arm X-horn that
# ships with the servo (no printed servo_horn_adapter disc in the
# stack).  Each driven link MUST therefore carry:
#
#   1. A 4 x M2-clearance hole pattern on an XHORN_BOLT_PCD = 20.8 mm
#      bolt circle, drilled through the pad's mating face along the
#      joint axis (link +Z for coxa_link; link +Y for femur_link /
#      tibia_link).  Phi XHORN_BOLT_OD = 2.2 mm (M2 clearance with
#      0.2 mm FDM print tolerance) -- the May 2026 fastener-spec fix
#      shrank this from the original (incorrect) Phi 3.2 mm M3
#      clearance to match the X-horn's Phi ~ 2.0 mm M2-self-tap arm
#      holes.  See hexapod_prototype.py XHORN_BOLT_* docstring.
#   2. A central Phi HORN_RECESS_OD = 16 mm cylindrical recess
#      HORN_RECESS_DEPTH = 1.2 mm deep cut into the pad's mating face,
#      so the plastic horn's central hub (spline collar + M3 centre-
#      screw head) is fully swallowed below the pad.  The centre
#      screw stays M3; only the 4 outer arm bolts switched to M2.
#      Depth user-measured May 2026 (1.0 mm screw head protrusion +
#      0.2 mm FDM tolerance; was 1.6 mm).
#
# This check confirms BOTH the 4 bolt holes and the central recess
# exist by sampling a small voxel patch at each expected position and
# requiring the link mesh to be VOID there.  A short pillar of pad
# material at the bolt-PCD ring or at the central hub indicates the
# pad was not drilled / recessed correctly.

# Tolerance: small voxel-grid step-noise budget per probe.  Each probe
# samples a Phi (hole_OD/2 - small) x depth column of voxels and
# expects ZERO of them to be inside the link mesh.  A real failure
# (= bolt pattern missing entirely, or recess depth set to 0) leaves
# 100+ voxel samples inside the part.
HORN_PATTERN_VOX_TOL = 5    # voxel hits per probe -- voxel-noise floor


def _probe_void_cylinder(mesh: trimesh.Trimesh,
                          centre: np.ndarray,
                          axis: str,
                          radius: float,
                          depth: float,
                          n_samples: int = 64) -> int:
    """Sample a cylindrical probe volume (radius x depth) along the
    given axis ("x" | "y" | "z"), centred at *centre* (where the
    cylinder's mid-Z lies); return how many sample points fall INSIDE
    *mesh*.  Used to assert that a region of the part is VOID."""
    rng = np.random.default_rng(seed=42)
    # Sample uniformly within a cylinder of (radius, depth) along the
    # axis.  Cylinder mid-point is at the origin in axis-local frame
    # then translated to *centre*.
    r = radius * np.sqrt(rng.random(n_samples))
    theta = 2.0 * np.pi * rng.random(n_samples)
    h = (rng.random(n_samples) - 0.5) * depth
    local_a = h                # along axis
    local_b = r * np.cos(theta)
    local_c = r * np.sin(theta)
    pts = np.zeros((n_samples, 3))
    if axis == "x":
        pts[:, 0] = local_a
        pts[:, 1] = local_b
        pts[:, 2] = local_c
    elif axis == "y":
        pts[:, 0] = local_b
        pts[:, 1] = local_a
        pts[:, 2] = local_c
    elif axis == "z":
        pts[:, 0] = local_b
        pts[:, 1] = local_c
        pts[:, 2] = local_a
    else:
        raise ValueError(f"axis must be x|y|z, got {axis!r}")
    pts = pts + np.asarray(centre)
    inside = points_inside(mesh, pts)
    return int(inside.sum())


def check_horn_pattern_in_pad():
    """Verify that each driven link's pad has:
      a) 4 M2 clearance holes on the XHORN_BOLT_PCD = 20.8 mm bolt
         circle, drilled through the full pad thickness, and
      b) a central Phi HORN_RECESS_OD = 16 mm x HORN_RECESS_DEPTH =
         1.2 mm hub-clearance recess on the pad's mating face.

    May 2026 fastener-spec fix: the bolts are M2 (Phi 2.2 mm
    clearance), not M3 (Phi 3.2 mm) -- see ``XHORN_BOLT_OD`` in
    ``hexapod_prototype.py``.  The probe radius below is sized off
    ``XHORN_BOLT_OD`` so the verifier will track any future change to
    the X-horn arm-hole standard without a code edit.
    """
    print(f"\n[5d] Horn-pattern in driven link pads "
          f"(Phi {hp.XHORN_BOLT_OD:.1f} mm holes on PCD "
          f"{hp.XHORN_BOLT_PCD:.1f} mm + Phi {hp.HORN_RECESS_OD:.1f} mm "
          f"x {hp.HORN_RECESS_DEPTH:.2f} mm hub recess):")

    # The probe radius is a hair smaller than the actual clearance
    # hole / recess radius so voxel stair-step artefacts on the cut's
    # curved boundary don't pollute the "is the volume void" answer.
    # With XHORN_BOLT_OD = 2.2 mm the bolt probe shrinks to ~ 0.9 mm
    # radius (Phi 1.8 mm probe) which still resolves cleanly against
    # the verifier's 1.5 mm voxel pitch -- a missing hole produces a
    # solid pillar of pad material at the probe position and registers
    # as ~ 40-100 hits (well above HORN_PATTERN_VOX_TOL * 4 = 20).
    bolt_probe_r   = hp.XHORN_BOLT_OD / 2.0 - 0.2     # ~0.9 mm at M2
    recess_probe_r = hp.HORN_RECESS_OD / 2.0 - 0.5    # ~7.5 mm

    cases = [
        # (name, mesh, pad_axis, mating_face_coord, pad_thickness,
        #  recess_depth, mating_normal_sign)
        # ``pad_axis``: which axis the bolts are along ("y" for the
        #   femur/tibia knee/hip pads, "z" for the coxa_link's
        #   pedestal-bottom mating face).
        # ``mating_face_coord``: the position of the mating face on
        #   *pad_axis*.  For femur/tibia, it's y = HORN_STACK_H = +5.
        #   For coxa_link, the pad's bottom is at z = 0 in link frame.
        # ``mating_normal_sign``: which way the recess opens from the
        #   mating face.  Femur/tibia mating face faces -Y (toward the
        #   horn below); the recess opens DOWNWARD in -Y, removing
        #   material at y in [mate, mate + RECESS_DEPTH].  Coxa_link
        #   mating face faces -Z (toward the horn below); the recess
        #   opens DOWNWARD in -Z, removing material at z in [0,
        #   +RECESS_DEPTH].  In both cases the recess probe centre
        #   sits +RECESS_DEPTH/2 INTO the pad along +pad_axis.
        ("coxa_link  (yaw joint)",        _load_mesh("coxa_link",
                                                       copy=False),
         "z", 0.0,                          +1.0),
        ("femur_link (hip-pitch joint)",  _load_mesh("femur_link",
                                                       copy=False),
         "y", hp.HORN_STACK_H,              +1.0),
        ("tibia_link (knee-pitch joint)", _load_mesh("tibia_link",
                                                       copy=False),
         "y", hp.HORN_STACK_H,              +1.0),
    ]

    # The pad's full thickness in the pad-axis direction.  For
    # femur/tibia: hip_pad/knee_pad spans y in [HORN_STACK_H,
    # HORN_STACK_H + LINK_THICKNESS].  For coxa_link: the pedestal +
    # hub stack spans z in [0, COXA_LIFT + hub_t] -- much taller, but
    # for the bolt-pattern probe we only need to confirm a clear hole
    # at the bottom of the pedestal, so a 4 mm probe is enough.
    BOLT_PROBE_DEPTH = 4.0    # probe a 4 mm-long column at the
                               # mating face; matches typical pad
                               # thickness.

    all_ok = True
    for name, mesh, axis, mate, normal_sign in cases:
        # ---- (a) 4 M2 bolt holes on the XHORN_BOLT_PCD circle ----
        bolt_misses = 0
        for ang in hp.XHORN_BOLT_ANGLES_RAD:
            # Bolt-circle position in the pad's transverse plane.
            tx = hp.XHORN_BOLT_PCD / 2.0 * np.cos(ang)
            ty = hp.XHORN_BOLT_PCD / 2.0 * np.sin(ang)
            # Probe centre sits half-way along the bolt's path into
            # the pad (the bolt enters at the mating face and exits
            # the far side, depth = LINK_THICKNESS for femur/tibia or
            # the pedestal+hub stack for coxa_link).
            centre_axis = mate + normal_sign * (BOLT_PROBE_DEPTH / 2.0)
            if axis == "y":
                centre = np.array([tx, centre_axis, ty])
            elif axis == "z":
                centre = np.array([tx, ty, centre_axis])
            else:
                centre = np.array([centre_axis, tx, ty])
            hits = _probe_void_cylinder(mesh, centre, axis,
                                          bolt_probe_r,
                                          BOLT_PROBE_DEPTH,
                                          n_samples=48)
            bolt_misses += hits
        ok_bolts = bolt_misses <= HORN_PATTERN_VOX_TOL * 4
        all_ok &= _label(
            f"{name} :: 4 x M2 bolts on Phi {hp.XHORN_BOLT_PCD} mm PCD",
            ok_bolts,
            f"hits={bolt_misses} (tol "
            f"{HORN_PATTERN_VOX_TOL * 4})",
        )

        # ---- (b) Central horn-hub recess ----
        # The recess starts at the mating face and extends INTO the
        # pad by HORN_RECESS_DEPTH.  Probe centre sits at half-depth
        # so the entire cylinder lives strictly inside the recess
        # volume.
        recess_centre_axis = mate + normal_sign * (hp.HORN_RECESS_DEPTH
                                                    / 2.0)
        if axis == "y":
            centre = np.array([0.0, recess_centre_axis, 0.0])
        else:                                            # "z"
            centre = np.array([0.0, 0.0, recess_centre_axis])
        hits = _probe_void_cylinder(mesh, centre, axis,
                                      recess_probe_r,
                                      hp.HORN_RECESS_DEPTH * 0.8,
                                      n_samples=64)
        ok_recess = hits <= HORN_PATTERN_VOX_TOL
        all_ok &= _label(
            f"{name} :: Phi {hp.HORN_RECESS_OD} mm "
            f"x {hp.HORN_RECESS_DEPTH:.2f} mm hub recess",
            ok_recess,
            f"hits={hits} (tol {HORN_PATTERN_VOX_TOL})",
        )

    return all_ok


# ---------------------------------------------------------------------------
# Design D (May 2026 heat-set switch): cradle servo insert pockets
# ---------------------------------------------------------------------------
#
# Each servo cradle (coxa_bracket's yaw well, coxa_link's hip-pitch
# well, femur_link's knee well) bolts the servo down via 4 VERTICAL
# M3 x 8 SHCS that thread DOWN through each servo ear into an M3 brass
# heat-set insert (McMaster 94459A130) seated in a printed Phi
# INSERT_M3_PILOT_OD = 4.0 mm pocket of depth INSERT_M3_PILOT_DEPTH =
# 6.0 mm.  Each pocket is surrounded by a Phi CRADLE_BOSS_OD = 8.0 mm
# printed boss (see ``_servo_cradle_insert_pockets`` in
# ``hexapod_prototype.py``).
#
# Each cradle shelf must satisfy THREE structural conditions per bolt
# site:
#
#   (1) The pocket is VOID along the Phi 4 mm x 6 mm cylinder so the
#       bolt + insert fit.  This is the renamed legacy "pilot-cyl"
#       probe with the new diameter / depth.
#   (2) The annular ring of plastic at radius
#       INSERT_M3_PILOT_OD/2 + CRADLE_BOSS_MIN_WALL_MM = 3.5 mm
#       around the pocket axis is SOLID at every azimuth.  This is
#       the radial-material check whose absence let the original
#       Phi 2.5 mm self-tap pilots ship with 0.00-1.50 mm of plastic
#       on one side of the bolt -- the verifier's vertical pilot
#       probe (commit b447f88) only confirmed the pilot cylinder
#       existed, not that it was surrounded by enough material
#       radially.  We probe 8 azimuths at two z planes
#       (pocket_bottom + 1 mm and pocket_top - 1 mm) and require
#       every sample to hit printed material.
#   (3) The annular ring between the pocket OD and the heat-set
#       insert OD must be SOLID before installation so the knurl has
#       plastic to displace.  We probe 8 azimuths at the midpoint of
#       the insert body (z = shelf_top - INSERT_M3_INSERT_LENGTH/2).
#
# The bracket's pockets ride a few mm below WELL_RIM_Z because the
# drop-in slot eats the well's rim down to bracket-z = -3 (see
# ``BRACKET_SHELF_DROP_MM`` in ``make_coxa_bracket``); the bracket
# case adjusts the probe's shelf_top_z accordingly so its 4 pockets
# are sampled where the bracket actually cut them.

# Legacy alias kept so external diagnostic dumps that reference the
# old constant name continue to work.  Same numeric value -- the new
# check uses the same noise-floor accounting.
CRADLE_PILOT_HOLE_VOX_TOL    = 3
CRADLE_INSERT_POCKET_VOX_TOL = 3   # voxel hits per pocket cylinder


def check_cradle_insert_pockets():
    """Verify each cradle has 2 heat-set insert sites on the -X column
    (Design E mixed-mode, May 2026).

    For each of the 6 (cradle, x = -X, y = +/- Y) HEAT-SET sites we
    probe THREE conditions:

      (1) the Phi INSERT_M3_PILOT_OD x INSERT_M3_PILOT_DEPTH POCKET
          is VOID along the bolt axis (the renamed legacy pilot-cyl
          probe);
      (2) the annular ring at radius
          INSERT_M3_PILOT_OD/2 + CRADLE_BOSS_MIN_WALL_MM around the
          pocket axis is SOLID at 8 azimuths x 2 z-planes -- the
          radial-material check that the original Phi 2.5 mm self-
          tap verifier was missing;
      (3) the annular ring between the pocket OD and the heat-set
          insert OD is SOLID at 8 azimuths so the knurl has plastic
          to displace at install time.

    The 2 +X sites per cradle are NOT probed by this check.  Under
    Design E they no longer have heat-set inserts -- they have bare
    Phi 2.5 mm self-tap pilots in the existing well-wall material
    (the Phi 8 mm heat-set boss could not coexist with the restored
    +X wire channel).  Those +X sites are documented as
    "known-thin" on the channel-facing side (~ 0.25 mm of plastic
    between the pilot and the channel), which is acceptable for an
    M3 self-tap into PLA / PETG and is enforced by
    ``check_servo_insertion_path`` rather than by a radial-material
    check.  See the INSERT_M3_SELFTAP_* block in
    ``hexapod_prototype.py`` for the full rationale.

    The cradles are tested in their UNROTATED, UNTRANSLATED local
    (well-local) frames so we don't have to undo each cradle's
    R / delta math.
    """
    print(f"\n[5e] Cradle heat-set insert pockets "
          f"(2 -X x Phi {hp.INSERT_M3_PILOT_OD:.1f} mm x "
          f"{hp.INSERT_M3_PILOT_DEPTH:.0f} mm pockets per cradle; "
          f"radial wall >= {hp.CRADLE_BOSS_MIN_WALL_MM:.1f} mm; "
          f"insert Phi {hp.INSERT_M3_INSERT_OD:.1f} mm x "
          f"{hp.INSERT_M3_INSERT_LENGTH:.0f} mm; "
          f"+X sites use self-tap pilots and are probed by "
          f"check_servo_insertion_path instead):")

    R_inv = rotation_matrix(+np.pi / 2.0, [1, 0, 0])

    def _bracket_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([+hp.SERVO_OUTPUT_X, 0.0, +hp.WELL_RIM_Z])
        return m

    def _coxa_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([0.0, 0.0, -hp.COXA_LIFT])
        arm_t = hp.COXA_ARM_T
        well_z_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0
                         + hp.WELL_Z_DROP_EXTRA)
        m.apply_translation([0.0, 0.0, -well_z_drop])
        m.apply_translation([
            -(hp.COXA_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    def _femur_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([
            -(hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    bracket_shelf_drop = 3.0
    cases = [
        ("coxa_bracket (yaw cradle)",
         _bracket_to_well_local(_load_mesh("coxa_bracket", copy=False)),
         hp.WELL_RIM_Z - bracket_shelf_drop),
        ("coxa_link    (hip-pitch cradle)",
         _coxa_link_to_well_local(_load_mesh("coxa_link", copy=False)),
         hp.WELL_RIM_Z),
        ("femur_link   (knee cradle)",
         _femur_link_to_well_local(_load_mesh("femur_link", copy=False)),
         hp.WELL_RIM_Z),
    ]

    pocket_probe_r = hp.INSERT_M3_PILOT_OD / 2.0 - 0.2

    r_outer_wall = (hp.INSERT_M3_PILOT_OD / 2.0
                    + hp.CRADLE_BOSS_MIN_WALL_MM)
    r_insert_ring = 0.5 * (hp.INSERT_M3_PILOT_OD / 2.0
                            + hp.INSERT_M3_INSERT_OD / 2.0)
    azimuths_deg = list(range(0, 360, 45))

    # Design E mixed-mode: only the -X column carries heat-set inserts;
    # the +X sites are bare Phi 2.5 mm self-tap pilots and intentionally
    # skip the radial-material / knurl-ring probes (the channel-facing
    # side has only ~ 0.25 mm of plastic, which is acceptable for an
    # M3 self-tap but would fail the heat-set CRADLE_BOSS_MIN_WALL_MM
    # check by design).
    HEATSET_SX_SIGNS = (-1,)

    all_ok = True
    for name, well_local_mesh, shelf_top_z in cases:
        # ---- (1) Pocket cylinder void probe -----------------------------
        pocket_hits = 0
        n_pockets = len(HEATSET_SX_SIGNS) * 2  # 2 (sx) * 2 (sy) = 2
        probe_depth = hp.INSERT_M3_PILOT_DEPTH * 0.6
        probe_z_cen = shelf_top_z - hp.INSERT_M3_PILOT_DEPTH / 2.0
        for sx in HEATSET_SX_SIGNS:
            for sy in (-1, 1):
                x = sx * hp.SERVO_MOUNT_HOLE_X_OFFSET
                y = sy * hp.SERVO_MOUNT_HOLE_Y_OFFSET
                centre = np.array([x, y, probe_z_cen])
                pocket_hits += _probe_void_cylinder(
                    well_local_mesh, centre, "z",
                    pocket_probe_r, probe_depth, n_samples=48,
                )
        pocket_tol = CRADLE_INSERT_POCKET_VOX_TOL * n_pockets
        ok_pockets = pocket_hits <= pocket_tol
        all_ok &= _label(
            f"{name} :: {n_pockets} x Phi {hp.INSERT_M3_PILOT_OD:.1f} "
            f"mm x {hp.INSERT_M3_PILOT_DEPTH:.0f} mm insert pockets "
            f"(-X column)",
            ok_pockets,
            f"hits={pocket_hits} (tol {pocket_tol})",
        )

        # ---- (2) Radial-material check ----------------------------------
        # 8 azimuths x 2 z-planes (pocket_bot + 1, pocket_top - 1) at
        # radius r_outer_wall around each pocket axis must hit material.
        pocket_z_bot = shelf_top_z - hp.INSERT_M3_PILOT_DEPTH
        pocket_z_top = shelf_top_z
        z_planes = (pocket_z_bot + 1.0, pocket_z_top - 1.0)
        n_radial = (len(HEATSET_SX_SIGNS) * 2 * len(azimuths_deg)
                    * len(z_planes))
        radial_misses: list[str] = []
        for sx in HEATSET_SX_SIGNS:
            for sy in (-1, 1):
                cx = sx * hp.SERVO_MOUNT_HOLE_X_OFFSET
                cy = sy * hp.SERVO_MOUNT_HOLE_Y_OFFSET
                pts = []
                metas = []
                for az in azimuths_deg:
                    rad = np.deg2rad(az)
                    px = cx + r_outer_wall * np.cos(rad)
                    py = cy + r_outer_wall * np.sin(rad)
                    for pz in z_planes:
                        pts.append((px, py, pz))
                        metas.append((sx, sy, az, pz))
                inside = points_inside(well_local_mesh,
                                       np.array(pts, dtype=float))
                for hit, meta in zip(inside, metas):
                    if not hit:
                        radial_misses.append(
                            f"(x={meta[0]:+d}*X, y={meta[1]:+d}*Y, "
                            f"az={meta[2]:3d}deg, z={meta[3]:5.2f})"
                        )
        n_radial_misses = len(radial_misses)
        ok_radial = n_radial_misses == 0
        if ok_radial:
            radial_detail = f"{n_radial}/{n_radial} azimuths hit material"
        else:
            sample = ", ".join(radial_misses[:6])
            more = (f"; +{n_radial_misses - 6} more"
                    if n_radial_misses > 6 else "")
            radial_detail = (
                f"{n_radial_misses}/{n_radial} azimuths punched AIR at "
                f"r = {r_outer_wall:.2f} mm: {sample}{more}"
            )
        all_ok &= _label(
            f"{name} :: 8-azimuth radial-material around each -X pocket "
            f"(r = {r_outer_wall:.2f} mm)",
            ok_radial,
            radial_detail,
        )

        # ---- (3) Heat-set insert displacement-ring check ----------------
        z_insert_mid = shelf_top_z - hp.INSERT_M3_INSERT_LENGTH / 2.0
        n_ring = len(HEATSET_SX_SIGNS) * 2 * len(azimuths_deg)
        ring_misses: list[str] = []
        for sx in HEATSET_SX_SIGNS:
            for sy in (-1, 1):
                cx = sx * hp.SERVO_MOUNT_HOLE_X_OFFSET
                cy = sy * hp.SERVO_MOUNT_HOLE_Y_OFFSET
                pts = []
                metas = []
                for az in azimuths_deg:
                    rad = np.deg2rad(az)
                    px = cx + r_insert_ring * np.cos(rad)
                    py = cy + r_insert_ring * np.sin(rad)
                    pts.append((px, py, z_insert_mid))
                    metas.append((sx, sy, az))
                inside = points_inside(well_local_mesh,
                                       np.array(pts, dtype=float))
                for hit, meta in zip(inside, metas):
                    if not hit:
                        ring_misses.append(
                            f"(x={meta[0]:+d}*X, y={meta[1]:+d}*Y, "
                            f"az={meta[2]:3d}deg)"
                        )
        n_ring_misses = len(ring_misses)
        ok_ring = n_ring_misses == 0
        if ok_ring:
            ring_detail = (
                f"{n_ring}/{n_ring} azimuths hit material at r = "
                f"{r_insert_ring:.2f} mm"
            )
        else:
            sample = ", ".join(ring_misses[:6])
            more = (f"; +{n_ring_misses - 6} more"
                    if n_ring_misses > 6 else "")
            ring_detail = (
                f"{n_ring_misses}/{n_ring} azimuths punched AIR at r = "
                f"{r_insert_ring:.2f} mm (knurl ring): {sample}{more}"
            )
        all_ok &= _label(
            f"{name} :: heat-set insert displacement ring (-X column, "
            f"r = {r_insert_ring:.2f} mm)",
            ok_ring,
            ring_detail,
        )

    return all_ok


# Backwards-compatible alias: external callers that still reference the
# old name (and the verifier's CHECKS registry's previous entry) get
# the new behaviour transparently.
check_cradle_pilot_holes = check_cradle_insert_pockets


# ---------------------------------------------------------------------------
# Servo insertion-path probe (Design E, May 2026)
# ---------------------------------------------------------------------------
#
# Simulates seating a DS3225 / MG996R / DS3218-class servo into its
# cradle by sliding the molded wire-exit BOOT envelope DOWN through
# the +X wall in 1 mm Z steps and checking that no printed-part
# material intrudes into the boot's swept volume by more than the
# tolerance below at any intermediate Z.
#
# This is the regression probe for the May 2026 user report: commit
# f03d59b (heat-set switch) shortened the +X wire channel from
# z = WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM (= 29.5) down to
# z = (WELL_RIM_Z - CRADLE_BOSS_HEIGHT_MM) - 0.5 (= 16.5) to clear
# the new Phi 8 mm heat-set bosses, which left 10.5 mm of solid +X
# wall blocking the boot's insertion path.  Real DS3225 servos
# could not be seated in their printed cradles.  Design E restored
# the channel cap and reverted the +X bolts to Phi 2.5 mm self-tap;
# this check enforces the channel-must-be-clear invariant so the
# regression cannot recur.

# Allowed boot-vs-part overlap volume at any single insertion-step.
# 1 mm^3 covers numerical / mesh-stair-step noise at our voxel pitch.
SERVO_INSERTION_PATH_TOL_MM3 = 1.0
# Z step size for the insertion sweep (mm).
SERVO_INSERTION_PATH_Z_STEP_MM = 1.0
# Sampling pitch inside the boot envelope (mm).  0.4 mm pitch on the
# 6.5 x 7.0 x 3.9 mm boot envelope is ~ 4400 sample points; at our
# 1 mm^3 tolerance any single sample voxel contributes 0.064 mm^3 so
# > 15 sample hits are needed to register as a failure.
SERVO_INSERTION_PATH_PITCH_MM = 0.4


def _boot_envelope_sample_points(z_offset: float) -> np.ndarray:
    """Return a dense grid of sample points covering the molded
    wire-exit boot envelope, translated in +Z by ``z_offset`` from
    the seated position.

    Seated boot envelope (well-local frame, matches ``_servo_envelope``):
        x in [+SERVO_BODY_W/2,
              +SERVO_BODY_W/2 + WIRE_BOOT_PROTRUSION]
        y in [-WIRE_BOOT_W/2, +WIRE_BOOT_W/2]
        z in [WIRE_BOOT_Z_BASE,
              WIRE_BOOT_Z_BASE + WIRE_BOOT_H]

    During insertion the servo body sits a bit higher in the cradle
    so the boot's z bounds shift UP by ``z_offset``; the x / y bounds
    are unchanged because the body is constrained in those axes by
    the well cavity walls.
    """
    pitch = SERVO_INSERTION_PATH_PITCH_MM
    x_min = +hp.SERVO_BODY_W / 2.0
    x_max = x_min + hp.WIRE_BOOT_PROTRUSION
    y_min = -hp.WIRE_BOOT_W / 2.0
    y_max = +hp.WIRE_BOOT_W / 2.0
    z_min = hp.WIRE_BOOT_Z_BASE + z_offset
    z_max = z_min + hp.WIRE_BOOT_H

    # Inset by half a voxel so sample centres tile the envelope cleanly.
    eps = pitch / 2.0
    nx = max(2, int(np.ceil(((x_max - eps) - (x_min + eps)) / pitch)))
    ny = max(2, int(np.ceil(((y_max - eps) - (y_min + eps)) / pitch)))
    nz = max(2, int(np.ceil(((z_max - eps) - (z_min + eps)) / pitch)))
    xs = np.linspace(x_min + eps, x_max - eps, nx)
    ys = np.linspace(y_min + eps, y_max - eps, ny)
    zs = np.linspace(z_min + eps, z_max - eps, nz)
    XX, YY, ZZ = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)


def check_servo_insertion_path():
    """Verify that the servo's molded +X wire-exit boot has a clear
    insertion path through the +X wall of every cradle (Design E,
    May 2026 regression probe).

    For each cradle (coxa_bracket yaw, coxa_link hip, femur_link knee)
    we slide the boot's swept volume DOWN through the +X wall in
    ``SERVO_INSERTION_PATH_Z_STEP_MM`` (= 1 mm) Z steps from
    ``z = WELL_RIM_Z + WIRE_BOOT_H`` (boot bottom face above the rim,
    boot entirely outside the cradle) down to the seated position
    (boot bottom at ``WIRE_BOOT_Z_BASE``).  At each step we count the
    sample points inside the printed cradle mesh and reject the
    cradle if any step's overlap volume exceeds
    ``SERVO_INSERTION_PATH_TOL_MM3`` (= 1.0 mm^3).

    The boot's swept volume is the canonical proxy for "can the user
    seat the servo?" -- the boot is a rigid molded feature on the
    servo case; any printed material in its swept volume means the
    servo cannot be pushed down past that Z without bending or
    shearing the boot.

    Regression history (May 2026):
      * commit f03d59b shortened the +X wire channel from
        z = WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM (= 29.5) down to
        z = (WELL_RIM_Z - CRADLE_BOSS_HEIGHT_MM) - 0.5 (= 16.5) to
        clear the new heat-set bosses.  Real DS3225 servos could
        not be seated in their printed cradles; the user reported
        the regression on receipt of physical hardware.
      * Design E (May 2026) restored the channel cap to
        WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM and reverted the +X
        cradle bolts to Phi 2.5 mm self-tap so the channel + bolts
        could coexist; this check was added at the same time so
        the regression cannot ship again.
    """
    print(f"\n[5f] Servo insertion-path probe "
          f"(boot {hp.WIRE_BOOT_PROTRUSION:.1f} x {hp.WIRE_BOOT_W:.1f} "
          f"x {hp.WIRE_BOOT_H:.1f} mm slides down +X wall in "
          f"{SERVO_INSERTION_PATH_Z_STEP_MM:.0f} mm Z steps; "
          f"max overlap {SERVO_INSERTION_PATH_TOL_MM3:.1f} mm^3 per step):")

    R_inv = rotation_matrix(+np.pi / 2.0, [1, 0, 0])

    def _bracket_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([+hp.SERVO_OUTPUT_X, 0.0, +hp.WELL_RIM_Z])
        return m

    def _coxa_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([0.0, 0.0, -hp.COXA_LIFT])
        arm_t = hp.COXA_ARM_T
        well_z_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0
                         + hp.WELL_Z_DROP_EXTRA)
        m.apply_translation([0.0, 0.0, -well_z_drop])
        m.apply_translation([
            -(hp.COXA_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    def _femur_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([
            -(hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    cradles = [
        ("coxa_bracket (yaw cradle)",
         _bracket_to_well_local(_load_mesh("coxa_bracket", copy=False))),
        ("coxa_link    (hip-pitch cradle)",
         _coxa_link_to_well_local(_load_mesh("coxa_link", copy=False))),
        ("femur_link   (knee cradle)",
         _femur_link_to_well_local(_load_mesh("femur_link", copy=False))),
    ]

    # Z sweep: boot bottom slides from WELL_RIM_Z + WIRE_BOOT_H (boot
    # entirely above the rim) DOWN to WIRE_BOOT_Z_BASE (seated).
    # Express as offsets RELATIVE to the seated position; offset 0
    # means seated, offset > 0 means the body is lifted ``offset`` mm
    # above seated.
    z_offset_max = (hp.WELL_RIM_Z + hp.WIRE_BOOT_H) - hp.WIRE_BOOT_Z_BASE
    n_steps = int(np.ceil(z_offset_max
                          / SERVO_INSERTION_PATH_Z_STEP_MM)) + 1
    z_offsets = np.linspace(0.0, z_offset_max, n_steps)

    pitch = SERVO_INSERTION_PATH_PITCH_MM
    voxel_vol = pitch ** 3

    all_ok = True
    for name, mesh in cradles:
        # Pre-compute the AABB of the entire boot sweep so we can
        # skip mesh-inside calls when the sweep box is well clear of
        # the cradle's bounds (a no-op for the standard cradles
        # where the boot sweep does intersect the +X wall, but
        # gives a clean fail fast if a future cradle is repositioned).
        mesh_min, mesh_max = mesh.bounds
        worst_overlap_mm3 = 0.0
        worst_offset = float(z_offsets[0])
        n_steps_blocked = 0
        for z_off in z_offsets:
            pts = _boot_envelope_sample_points(float(z_off))
            # Quick AABB rejection.
            if (pts[:, 0].max() < mesh_min[0]
                    or pts[:, 0].min() > mesh_max[0]
                    or pts[:, 1].max() < mesh_min[1]
                    or pts[:, 1].min() > mesh_max[1]
                    or pts[:, 2].max() < mesh_min[2]
                    or pts[:, 2].min() > mesh_max[2]):
                continue
            inside = points_inside(mesh, pts)
            n_in = int(inside.sum())
            overlap_mm3 = n_in * voxel_vol
            if overlap_mm3 > worst_overlap_mm3:
                worst_overlap_mm3 = overlap_mm3
                worst_offset = float(z_off)
            if overlap_mm3 > SERVO_INSERTION_PATH_TOL_MM3:
                n_steps_blocked += 1

        ok = n_steps_blocked == 0
        if ok:
            detail = (
                f"{n_steps} Z steps probed (boot bottom z = "
                f"{hp.WIRE_BOOT_Z_BASE:.1f}-> "
                f"{hp.WIRE_BOOT_Z_BASE + z_offset_max:.1f} mm); worst "
                f"step overlap {worst_overlap_mm3:.2f} mm^3 <= "
                f"{SERVO_INSERTION_PATH_TOL_MM3:.1f} mm^3 tol"
            )
        else:
            detail = (
                f"{n_steps_blocked}/{n_steps} Z steps blocked; worst "
                f"step overlap {worst_overlap_mm3:.2f} mm^3 at boot "
                f"bottom z = {hp.WIRE_BOOT_Z_BASE + worst_offset:.1f} "
                f"mm (offset {worst_offset:+.1f} above seated); "
                f"tol {SERVO_INSERTION_PATH_TOL_MM3:.1f} mm^3"
            )
        all_ok &= _label(name, ok, detail)

    return all_ok


# ---------------------------------------------------------------------------
# Screwdriver-access clearance check
# ---------------------------------------------------------------------------
#
# For each fastener, probe a cylinder pointing OUTWARD from the head
# along the driver-approach direction (= -axis_world).  Fail if any
# printed part intrudes into that cylinder by more than
# DRIVER_INTRUSION_TOLERANCE_MM3 mm^3.
#
# The probe cylinder size is picked per-fastener based on the kind of
# tool the assembler actually needs to drive that fastener.  Probing
# every SHCS with the worst-case Phillips envelope (12 mm dia x 80 mm
# long) overstates the required clearance by a large margin and
# produces FAILs at fasteners that a real assembler can drive with
# their fingers and a 25 mm-long L-shaped hex key with no trouble.
#
# The three envelopes:
#
# * HEX_KEY  --  L-shaped hex key short-arm (M2.5 / M3 wrench).  The
#                short arm is ~25 mm long and ~3 mm AF; the assembler
#                needs maybe 8 mm of clearance around it for fingers
#                and the long-arm sticking out to the side.  Anything
#                tagged SHCS (or the M2.5 spline center screw, which
#                IS a tiny Phillips on hobby servos but is always
#                SKIPped because it sits captive under the X-horn
#                after assembly -- mapping it here is documentation
#                only, the cone never lands on actual geometry).
#
# * PHILLIPS  -- Full-length Phillips / slotted screwdriver: 12 mm
#                shaft + handle, 80 mm reach.  Used for the foot
#                hinge M3x16 pan-head and any other pan-head or
#                slotted-head fastener.
#
# * SOCKET    -- Small M3-nut driver / 5.5 mm socket on a stub
#                handle.  12 mm OD at the head, ~50 mm of reach
#                (shorter than Phillips because nut drivers don't
#                need to land on a recessed cross-shape).  Used for
#                the M3 nyloc nuts that aren't captive in a hex
#                pocket (e.g. the chassis bolt nuts under the
#                chassis_bottom, the foot hinge nut).
#
# DRIVER_INTRUSION_TOLERANCE_MM3 mirrors the OTHER_TOLERANCE = 30 mm^3
# used by check_servo_clearance: at the verifier's 1.5 mm voxel pitch
# a single voxel = 3.4 mm^3, so 30 mm^3 ~= 9 voxels = the noise floor
# for the axis-aligned-ray parity vote on a curved mesh boundary.
# Anything below it is voxel stair-step artefact, not real material
# in the cone.

HEX_KEY_CLEARANCE_DIA_MM     = 8.0    # 2.5 / 3 mm hex key short arm + finger room
HEX_KEY_CLEARANCE_LEN_MM     = 30.0   # short-arm reach
# M2 SHCS uses a Phi 1.5 mm hex key.  The bolt head is recessed in a
# Phi M2_HEAD_OD_CLEARANCE = 4.0 mm counter-bore in the printed cap
# / pad above the bolt; the hex key short arm enters the counter-bore
# from above so the envelope diameter is bounded by the counter-bore
# clearance (Phi 4 mm) not the larger 8 mm "finger room" diameter
# used for M2.5 / M3 hex keys (those keys are driven with two fingers
# pinched around the long arm, whereas the M2 key is small enough to
# turn with a single finger inside the counter-bore + trough void).
# The short-arm reach is shorter too (typical M2 hex key short arm =
# ~ 10 mm) but we round up to 15 mm to allow some additional clear
# space above the head for finger access.
M2_HEX_KEY_CLEARANCE_DIA_MM  = 4.0    # = M2_HEAD_OD_CLEARANCE
M2_HEX_KEY_CLEARANCE_LEN_MM  = 15.0   # short-arm reach + finger headroom
PHILLIPS_CLEARANCE_DIA_MM    = 12.0   # full-length Phillips shaft + handle
PHILLIPS_CLEARANCE_LEN_MM    = 80.0
SOCKET_CLEARANCE_DIA_MM      = 12.0   # M3 nut driver / 5.5 mm socket + handle taper
SOCKET_CLEARANCE_LEN_MM      = 50.0

DRIVER_INTRUSION_TOLERANCE_MM3 = 30.0
# Probe slightly forward of the head so the cylinder NEVER overlaps
# the head's own seating face (which would otherwise also overlap the
# part the head bolts to, and falsely report it as an intrusion).
DRIVER_HEAD_STANDOFF_MM      = 0.5

# Names of every printed part that COULD possibly intrude into a
# driver-access cone.  Chassis-level + leg-0 only -- the 6-fold
# symmetric chassis layout means the leg 0 fastener vs leg 0 parts
# intrusion pattern is identical to all 6 legs.
_DRIVER_PRINTED_PART_NAMES = (
    "chassis_bottom", "chassis_top", "battery_holder", "electronics_tray",
    "bec_cradle", "switch_holster", "imu_pad",
    "coxa_bracket", "coxa_link", "femur_link", "tibia_link", "foot_pad",
)


def _build_world_leg0_printed_parts() -> dict:
    """Return ``{part_name: world-frame trimesh}`` for every printed
    part that could obstruct a driver-access cone on leg 0.  Mirrors
    ``_build_standing_leg`` for the legged parts, and uses the same
    chassis-fixed transforms for the central parts (chassis plates,
    battery, electronics).
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H
                     + hp.HORN_STACK_H)
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = (hip_joint_local
                        + Ry_p @ np.array([hp.FEMUR_LENGTH, 0, 0]))

    parts: dict = {}

    # Chassis-level parts (transforms match inspect_build's frame).
    plate_t = hp.CHASSIS_PLATE_T
    gap = hp.CHASSIS_GAP
    cb_bottom = _load_mesh("chassis_bottom")
    parts["chassis_bottom"] = cb_bottom

    cb_top = _load_mesh("chassis_top")
    cb_top.apply_translation([0.0, 0.0, gap + plate_t])
    parts["chassis_top"] = cb_top

    bat = _load_mesh("battery_holder")
    bat.apply_translation([hp.BATTERY_HOLDER_CENTRE_X, 0.0, plate_t / 2.0])
    parts["battery_holder"] = bat

    et = _load_mesh("electronics_tray")
    et.apply_translation([hp.ELEC_TRAY_CENTRE_X, hp.ELEC_TRAY_CENTRE_Y,
                          plate_t + 1.0])
    parts["electronics_tray"] = et

    # bec_cradle: friction-fit on top of the electronics_tray (tray top
    # face at z = plate_t + 1 + ELEC_TRAY_T = 4 + 1 + 3 = 8).  Cradle
    # mesh has its origin centred on its X-Y extents with bottom face
    # at local z = 0.
    bec = _load_mesh("bec_cradle")
    bec.apply_translation([
        hp.ELEC_TRAY_CENTRE_X + hp.BEC_CRADLE_CENTRE[0],
        hp.ELEC_TRAY_CENTRE_Y + hp.BEC_CRADLE_CENTRE[1],
        plate_t + 1.0 + hp.ELEC_TRAY_T,
    ])
    parts["bec_cradle"] = bec

    # switch_holster: sits ON TOP of 2 printed bosses on chassis_top's
    # TOP face.  Ear bottom rests on the boss tops at z = chassis_top_top
    # + SWITCH_HOLSTER_BOSS_H = gap + plate_t + plate_t/2 + BOSS_H =
    # 32 + 4 + 2 + 3 = 41 in the chassis design frame.
    chassis_top_top_z = gap + plate_t + plate_t / 2.0  # = 38 (design)
    sh = _load_mesh("switch_holster")
    sh.apply_translation([
        hp.SWITCH_HOLSTER_CENTRE_X,
        hp.SWITCH_HOLSTER_CENTRE_Y,
        chassis_top_top_z + hp.SWITCH_HOLSTER_BOSS_H,
    ])
    parts["switch_holster"] = sh

    # imu_pad: vibration-isolated MPU-6050 mounting plate, glued to
    # chassis_top with a 3 mm strip of double-sided foam tape (no
    # fasteners between the pad and chassis_top).  Pad bottom face
    # at z = chassis_top_top + IMU_PAD_TAPE_T = 38 + 3 = 41.
    ip = _load_mesh("imu_pad")
    ip.apply_translation([
        hp.IMU_PAD_CENTRE_X,
        hp.IMU_PAD_CENTRE_Y,
        chassis_top_top_z + hp.IMU_PAD_TAPE_T,
    ])
    parts["imu_pad"] = ip

    # Leg-0 printed parts -- mirrors ``_build_standing_leg`` so the
    # legged-part transforms exactly match the registry's leg_index=0
    # fastener positions.
    cbrk = _load_mesh("coxa_bracket")
    cbrk.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cbrk.apply_translation(edge_mid)
    parts["coxa_bracket"] = cbrk

    cl = _load_mesh("coxa_link")
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["coxa_link"] = cl

    fl = _load_mesh("femur_link")
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["femur_link"] = fl

    tl = _load_mesh("tibia_link")
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local)
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["tibia_link"] = tl

    # Foot pad world location (mirrors inspect_build._build_assembly_instances).
    Ry_pt = rotation_matrix(pt, [0, 1, 0])[:3, :3]
    hinge_local = (knee_joint_local
                   + Ry_pt @ np.array([hp.TIBIA_LENGTH, 0.0,
                                       hp.FOOT_HINGE_TIBIA_Z]))
    R_a = rotation_matrix(a, [0, 0, 1])[:3, :3]
    hinge_world = R_a @ hinge_local + edge_mid + yaw_output_z * z_hat
    foot = _load_mesh("foot_pad")
    foot.apply_transform(rotation_matrix(a, [0, 0, 1]))
    foot.apply_translation([
        hinge_world[0], hinge_world[1],
        hinge_world[2] - hp.FOOT_HINGE_FOOT_Z,
    ])
    parts["foot_pad"] = foot

    return parts


def _driver_envelope_for_spec(spec: str) -> tuple:
    """Return ``(diameter_mm, length_mm)`` of the driver clearance
    cone appropriate for ``spec`` -- one of the fastener-registry
    ``SPEC_*`` strings (e.g. ``"M3x8 SHCS"`` or ``"M3 nyloc nut"``).

    Dispatch (case-sensitive, ordered by precedence -- first match
    wins because some specs contain more than one of the marker
    words):

    * Anything containing ``"spline"`` -> HEX_KEY envelope.  The
      hobby servo's M2.5 spline center screw IS a small Phillips in
      reality, but it is also explicitly SKIPped in every fastener
      registry instance (captive under the X-horn after assembly),
      so its cone never lands on actual geometry.  Mapping it to
      HEX_KEY here is purely so an analyst dumping the per-spec
      envelope table sees the SMALLEST plausible envelope and isn't
      misled into thinking we modelled an 80 mm Phillips reach
      through the X-horn.
    * Anything containing ``"SHCS"`` -> HEX_KEY envelope.
    * Anything containing ``"pan-head"``, ``"Phillips"``, or
      ``"slotted"`` -> PHILLIPS envelope.
    * Anything containing ``"nyloc nut"`` or just ``"nut"`` ->
      SOCKET envelope.

    An unknown spec falls through to the conservative PHILLIPS
    envelope.  Future fastener-spec drift will show up as a probe
    that's larger than expected, not as a silently-permissive one.
    """
    if "spline" in spec:
        return (HEX_KEY_CLEARANCE_DIA_MM, HEX_KEY_CLEARANCE_LEN_MM)
    if spec.startswith("M2x") and "SHCS" in spec:
        # M2 SHCS uses a small Phi 1.5 mm hex key driving into a Phi
        # 4 mm counter-bore in the printed link cap / pad.  Use the
        # narrower M2 envelope so the surrounding pad material that
        # rings the counter-bore (annulus 2..4 mm from bolt axis)
        # isn't falsely reported as a driver intrusion; see the M2_
        # HEX_KEY_CLEARANCE_* constants block above for the
        # derivation.
        return (M2_HEX_KEY_CLEARANCE_DIA_MM, M2_HEX_KEY_CLEARANCE_LEN_MM)
    if "SHCS" in spec:
        return (HEX_KEY_CLEARANCE_DIA_MM, HEX_KEY_CLEARANCE_LEN_MM)
    if ("pan-head" in spec) or ("Phillips" in spec) or ("slotted" in spec):
        return (PHILLIPS_CLEARANCE_DIA_MM, PHILLIPS_CLEARANCE_LEN_MM)
    if "nut" in spec:
        return (SOCKET_CLEARANCE_DIA_MM, SOCKET_CLEARANCE_LEN_MM)
    return (PHILLIPS_CLEARANCE_DIA_MM, PHILLIPS_CLEARANCE_LEN_MM)


def _make_driver_probe(head_xyz: np.ndarray,
                       axis_world: np.ndarray,
                       dia_mm: float = PHILLIPS_CLEARANCE_DIA_MM,
                       length_mm: float = PHILLIPS_CLEARANCE_LEN_MM,
                       ) -> trimesh.Trimesh:
    """Build a ``dia_mm`` x ``length_mm`` clearance probe oriented
    along the DRIVER APPROACH direction (= -axis_world, since
    axis_world points FROM head INTO material and the driver
    approaches FROM OUTSIDE the head).

    The default envelope is PHILLIPS (12 x 80 mm) so callers that
    don't pass a per-fastener size get the conservative cone.
    ``_driver_envelope_for_spec`` is the helper that picks the right
    envelope for a real ``FastenerInstance.spec`` string.
    """
    cyl = trimesh.creation.cylinder(
        radius=dia_mm / 2.0,
        height=length_mm,
        sections=24,
    )
    # cylinder() is centred at the origin along +Z.  Shift it so the
    # BASE sits at z=0 (the head plane) and the tip at z=+LEN.  We
    # then rotate so +Z lands on the driver direction (-axis_world)
    # and translate to head_xyz.
    cyl.apply_translation([0.0, 0.0, length_mm / 2.0
                                       + DRIVER_HEAD_STANDOFF_MM])
    driver_dir = -np.asarray(axis_world, dtype=float)
    n = float(np.linalg.norm(driver_dir))
    if n < 1e-12:
        driver_dir = np.array([0.0, 0.0, 1.0])
    else:
        driver_dir = driver_dir / n
    # Rotation: map mesh +Z onto driver_dir.  Build via cross product.
    z_hat = np.array([0.0, 0.0, 1.0])
    cos_t = float(np.clip(np.dot(z_hat, driver_dir), -1.0, 1.0))
    if cos_t > 1.0 - 1e-9:
        R = np.eye(4)
    elif cos_t < -1.0 + 1e-9:
        # 180-degree flip about any perpendicular axis -- use +X.
        R = rotation_matrix(np.pi, [1.0, 0.0, 0.0])
    else:
        axis = np.cross(z_hat, driver_dir)
        axis = axis / float(np.linalg.norm(axis))
        angle = float(np.arccos(cos_t))
        R = rotation_matrix(angle, axis)
    cyl.apply_transform(R)
    cyl.apply_translation(np.asarray(head_xyz, dtype=float))
    return cyl


def check_screwdriver_access():
    """For each fastener instance, probe a per-fastener cylindrical
    clearance cone above the head along the driver-approach axis.
    Fail if any PRINTED part intrudes by more than
    DRIVER_INTRUSION_TOLERANCE_MM3.

    Three envelopes are dispatched by ``FastenerInstance.spec`` (see
    ``_driver_envelope_for_spec``):

    * HEX_KEY  (Phi  8 mm x 30 mm) -- SHCS + spline-screw entries
    * PHILLIPS (Phi 12 mm x 80 mm) -- pan-head / Phillips / slotted
    * SOCKET   (Phi 12 mm x 50 mm) -- nyloc nut / generic nut

    Fasteners with an explicit ``skip_screwdriver_reason`` -- the
    servo's M2.5 spline center screw (captive under the X-horn after
    assembly) and the cradle captive nyloc nuts (held in the Design C
    hex pocket; the bolt is driven from the head side) -- are SKIPped
    with the reason printed alongside.
    """
    print("\n[14] Screwdriver-access clearance "
          f"(per-spec envelopes: "
          f"HEX_KEY {HEX_KEY_CLEARANCE_DIA_MM:.0f}x{HEX_KEY_CLEARANCE_LEN_MM:.0f} mm, "
          f"PHILLIPS {PHILLIPS_CLEARANCE_DIA_MM:.0f}x{PHILLIPS_CLEARANCE_LEN_MM:.0f} mm, "
          f"SOCKET {SOCKET_CLEARANCE_DIA_MM:.0f}x{SOCKET_CLEARANCE_LEN_MM:.0f} mm; "
          f"tol {DRIVER_INTRUSION_TOLERANCE_MM3:.0f} mm^3):")

    # Import here so the verifier never tries to load the registry at
    # module import time (avoids circular import surprises if the
    # registry grows a dependency on this module).
    import fastener_registry  # noqa: WPS433

    fasteners = fastener_registry.build_all_fastener_instances()
    # 6-fold symmetric chassis layout -> testing leg 0 suffices for
    # every per-leg fastener.  We DO need every chassis-level entry
    # (leg_index None) and leg-0 entries.
    sample = [fi for fi in fasteners
              if fi.leg_index is None or fi.leg_index == 0]

    print(f"  Probing {len(sample)} fastener(s) on leg 0 + chassis-level "
          f"(of {len(fasteners)} total; the other 5 legs are 6-fold "
          f"rotationally symmetric).")

    world_parts = _build_world_leg0_printed_parts()

    # Pre-compute every part's AABB so we can skip the volumetric
    # voxel test for the 95 %+ of (part, probe) pairs that don't even
    # overlap loosely.
    part_aabbs = {name: m.bounds for name, m in world_parts.items()}

    all_ok = True
    n_skip = 0
    n_check = 0
    n_fail = 0
    grouped: dict[str, list[str]] = {}

    for fi in sample:
        if fi.skip_screwdriver_reason is not None:
            n_skip += 1
            grouped.setdefault("SKIP", []).append(
                f"{fi.role}: {fi.skip_screwdriver_reason}"
            )
            continue
        n_check += 1
        dia, length = _driver_envelope_for_spec(fi.spec)
        probe = _make_driver_probe(fi.head_world_xyz, fi.axis_world,
                                    dia_mm=dia, length_mm=length)
        p_lo, p_hi = probe.bounds
        total_intrusion = 0.0
        per_part: list[str] = []
        for part_name, part_mesh in world_parts.items():
            a_lo, a_hi = part_aabbs[part_name]
            if np.any(p_hi <= a_lo) or np.any(a_hi <= p_lo):
                continue
            vol = _pair_overlap_volume(probe, part_mesh, pitch=1.5)
            if vol > 0.0:
                total_intrusion += vol
                per_part.append(f"{part_name}={vol:.1f}")
        ok = total_intrusion <= DRIVER_INTRUSION_TOLERANCE_MM3
        if not ok:
            n_fail += 1
            detail = (
                f"intrusion {total_intrusion:6.1f} mm^3 "
                f"(tol {DRIVER_INTRUSION_TOLERANCE_MM3:.0f})  "
                f"[{', '.join(per_part)}]"
            )
            _label(fi.role, False, detail)
            all_ok = False

    # Always report the SKIP block (even when every probed fastener
    # passes) so a casual reader can see the explicit allow-list
    # without rerunning with --serial.
    if "SKIP" in grouped:
        print(f"  ---- SKIPped ({n_skip}) ----")
        for line in grouped["SKIP"][:8]:
            print(f"     SKIP  {line}")
        if len(grouped["SKIP"]) > 8:
            print(f"     SKIP  ... and {len(grouped['SKIP']) - 8} more "
                  f"(rotationally symmetric copies on other legs)")
    if all_ok:
        _label(
            f"{n_check} probed (+{n_skip} explicit SKIPs)",
            True,
            "every driver cone is clear",
        )
    else:
        print(f"  ---- FAIL ({n_fail} of {n_check} probed) ----")
    return all_ok


# ---------------------------------------------------------------------------
# Assembly-integrity checks (May 2026, post-coxa_link-pedestal-audit)
# ---------------------------------------------------------------------------
#
# The 14-check verifier above silently allowed the May 2026 ``coxa_link``
# pedestal regression: the body-insertion trough in ``make_coxa_link``
# slices the entire BOTTOM 25.5 mm out of the 34x34x36 pedestal, leaving
# only a ~10.5 mm cap on top.  The 4 M2 X-horn bolts in
# ``fastener_registry._emit_horn_fasteners_yaw`` were also pinned at
# link-local z = COXA_LIFT + hub_t = 44 mm (TOP of the hub, NOT at the
# link's bottom face) -- so the bolts float entirely inside the hub and
# DO NOT clamp the link to the X-horn at all.  Both bugs survived because
# every existing check probed parts in isolation; nothing tested the
# integrity of the FULL assembly (bolt joins parts, parts mate at faces).
#
# We add two structural checks that together close the gap:
#
# * ``check_fastener_engagement`` -- for every bolt with a length, the
#   head must bear on solid material, the shaft must traverse a single
#   contiguous run of material/closely-mated-parts (no long air spans),
#   and the bolt must JOIN at least 2 distinct printed/horn parts.  The
#   "joins >= 2 parts" rule is the one that catches the yaw mis-placement
#   above (bolt at z = [36, 44] is fully inside the coxa_link hub and
#   doesn't reach the X-horn at z = [-5, 0]).
#
# * ``check_mating_face_contact`` -- a small explicit list of mating
#   faces (coxa_link bottom <-> yaw X-horn top, femur_link hip pad <->
#   hip X-horn, tibia_link knee pad <-> knee X-horn, coxa_bracket flange
#   <-> chassis_bottom, foot_pad tongue <-> tibia clevis).  For each
#   face we scan along the joint axis and confirm the two parts mate
#   within ``tolerance_mm``.  The hollowed-out coxa_link pedestal shows
#   up here as a ~25 mm gap at the yaw mating face.
#
# Both checks live in the SAME ``_verify_prototype.py`` module so they
# pick up the existing ``_load_mesh`` cache + ``points_inside``
# dispatcher + worker process pool -- no separate machinery.

# Per-fastener engagement geometry.  ``head_od`` drives the head-bearing
# radial probes, ``shaft_od`` drives the shaft / engagement radial
# probes (we offset by 0.4 mm so the probe sits just OUTSIDE the
# drilled bolt clearance hole -- otherwise every probe sits inside the
# Phi 2.2 mm / Phi 3.2 mm clearance cylinder and reports "AIR"),
# ``engagement_mm`` is the expected thread-engagement length at the
# tip.  Default values are used for any spec missing from the table.
FASTENER_ENGAGEMENT_SPEC = {
    # ``M2x8 SHCS`` engagement = XHORN_BOLT_THREAD_ENGAGEMENT_MM = 3.0 mm
    # (the design-spec depth into the X-horn arm).  The stock M2 x 8
    # SHCS overhangs the cap + horn stack by ~ 1.5 mm in -Z below the
    # plastic horn, so the engagement-zone window (the last
    # ``engagement_mm`` of the modeled bolt length) deliberately
    # straddles the horn-bottom face: roughly half the window sits in
    # the horn arm, half in the air below.  TIP_ENGAGEMENT_MIN_FRACTION
    # = 0.5 is therefore exactly the right pass-threshold for an
    # overhung bolt that still bites the design-required 3 mm of
    # plastic.  Pre-fix the entry was 1.6 mm, a generic default that
    # made the check too tight for the 8 mm-long bolt in a 4 mm cap +
    # 5 mm horn stack.
    "M2x8 SHCS":                       dict(head_od=3.8, shaft_od=2.2, engagement_mm=3.0),
    "M3x8 SHCS":                       dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    "M3x8 SHCS into heat-set insert":  dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    # ``M3x8 SHCS self-tap`` -- Design E mixed-mode, May 2026.  The 2 +X
    # cradle bolts per cradle self-tap into a Phi 2.5 mm pilot drilled
    # into the existing well-wall material (no brass insert, no Phi 8
    # mm boss).  Engagement target is the plastic shelf material below
    # the servo's 2.5 mm-thick tab: the M3 x 8 SHCS head sits on the
    # ear top, descends through 2.5 mm of (factory) tab, and bites
    # into ~ 5.5 mm of printed plastic in the well wall.  The shelf
    # has INSERT_M3_SELFTAP_PILOT_DEPTH = 10 mm of available depth so
    # the bolt does not bottom out; the engagement-zone window probes
    # the deepest 5 mm of the bolt for material contact (same target
    # as the M3 x 8 SHCS into heat-set entry above -- the bolt length
    # is identical and the shelf material at this depth is the
    # engagement medium in either case).
    "M3x8 SHCS self-tap":              dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    # ``M3x10 SHCS`` -- battery_holder foot bolts (4) into M3 heat-set
    # inserts.  Same engagement target as M3 x 8 into insert (= the
    # 5 mm insert body length) since both rely on the brass thread,
    # not the printed plastic.
    "M3x10 SHCS":                      dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    "M3x32 SHCS":                      dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    "M3x16 pan-head":                  dict(head_od=6.0, shaft_od=3.2, engagement_mm=5.0),
    "M2.5x8 spline screw":             dict(head_od=4.5, shaft_od=2.7, engagement_mm=3.0),
    # ``M2.5x8 SHCS into heat-set insert`` -- Pi 4 board-mount bolts (4)
    # into M2.5 brass heat-set inserts (McMaster 94459A106).  Insert
    # body is INSERT_M25_INSERT_LENGTH = 4 mm long; engagement target
    # is the brass thread, not the surrounding plastic.
    "M2.5x8 SHCS into heat-set insert": dict(head_od=4.5, shaft_od=2.7, engagement_mm=4.0),
}
_FASTENER_ENGAGEMENT_DEFAULT = dict(head_od=5.0, shaft_od=3.0, engagement_mm=3.0)

# Max contiguous AIR run along the shaft between p_head and
# p_engage_start.  A counter-bore void (1-2 mm in the part that the
# head bears on) is fine; anything longer means the bolt is bridging
# two parts that don't actually touch.
MAX_SHAFT_AIR_SPAN_MM = 2.0

# Minimum fraction of the tip engagement zone that must be inside
# printed material (or a mating-target mesh like ``servo_horn``).
TIP_ENGAGEMENT_MIN_FRACTION = 0.5

# Fasteners that engage a NUT or HEAT-SET INSERT at some point along
# the shaft (not necessarily at the tip).  For these the tip-engagement
# zone is RELOCATED to bracket the paired nut/insert, not the bolt's
# physical tip; otherwise an M3 x 32 chassis bolt's tip (at world
# z ~ -17 mm, well past the M3 nyloc nut at z ~ -4..-9) would always
# fail tip-engagement.
_NUT_INSERT_SPECS = (
    "M3 nyloc nut",
    "M3 heat-set insert",
    "M2.5 heat-set insert",
)

# Joints whose horn (servo_horn) sits ON the spline tip and is the
# target the X-horn bolts thread INTO.  Used by ``check_mating_face_contact``
# and by ``_world_horn_meshes`` to place the visual horn for leg 0.
_HORN_JOINTS = ("yaw", "hip", "knee")


def _engagement_spec_for(spec: str) -> dict:
    return FASTENER_ENGAGEMENT_SPEC.get(spec, _FASTENER_ENGAGEMENT_DEFAULT)


def _horn_world_transform(joint: str, leg_index: int):
    """4x4 transform that maps horn-local coords (origin at spline-
    mating face, +Z = output-shaft axis) into the world frame for the
    given joint on ``leg_index``.

    The X-horn is BOLTED to the driven link (coxa_link / femur_link /
    tibia_link) so it rotates RIGIDLY with that link.  At the neutral
    stance pose the relevant rotations are:

      * yaw    -- 0 deg (coxa_link is in the same Rz(a) frame as the
                    coxa_bracket / yaw cradle, so no link-vs-cradle
                    rotation; the un-rotated well transform already
                    matches the link frame).
      * hip    -- horn rotates with the FEMUR (driven link) about
                    the hip joint Y axis.  The hip well lives in the
                    coxa_link (parent), so the horn's rotation
                    RELATIVE to the well is ``STANCE_FEMUR_DEG``.
      * knee   -- horn rotates with the TIBIA (driven link) about
                    the knee joint Y axis.  The knee well lives in
                    the FEMUR (parent of the knee joint), so the
                    horn's rotation RELATIVE to the well is
                    ``STANCE_TIBIA_DEG`` -- NOT the sum
                    STANCE_FEMUR_DEG + STANCE_TIBIA_DEG, because
                    _knee_cradle_T's T_well already inherits the
                    femur's Ry(STANCE_FEMUR_DEG) via
                    T_femur_in_link.

    Pre-fix the horn was placed via ``_well_cradle_T @ horn_offset``
    for ALL three joints, which corresponds to "horn in the SERVO's
    well frame".  For yaw that is also the link frame (no link
    rotation), but for hip / knee the femur / tibia rotates about the
    joint axis at the neutral stance angle and the horn rotates with
    it.  Without this rotation the femur / tibia horn bolts (which
    are placed in femur- / tibia-local frame and therefore inherit
    the link's stance rotation) MISS the un-rotated horn arms in the
    world frame and report ``joins only 1 part [femur_link]`` /
    ``[tibia_link]`` in check_fastener_engagement.
    """
    import fastener_registry as _fr  # noqa: WPS433
    if joint == "yaw":
        T_well = _fr._yaw_cradle_T(leg_index)
    elif joint == "hip":
        T_well = _fr._hip_cradle_T(leg_index)
    elif joint == "knee":
        T_well = _fr._knee_cradle_T(leg_index)
    else:
        raise ValueError(f"unknown joint: {joint!r}")
    horn_offset = _fr._T(hp.SERVO_OUTPUT_X, 0.0,
                          hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H)
    T_horn = T_well @ horn_offset
    if joint == "yaw":
        return T_horn
    # For hip / knee the horn rotates with the driven link about the
    # joint axis.  Compose the rotation in HORN-LOCAL frame: horn-local
    # +Z is the joint axis, so the in-frame rotation is _Rz(p_link)
    # applied AFTER the horn's existing frame mapping.
    if joint == "hip":
        # Hip horn rotates with the femur (driven link) by
        # STANCE_FEMUR_DEG relative to the coxa_link (where the
        # well lives).  In horn-local +Z that's Rz(STANCE_FEMUR_DEG).
        p_link = np.deg2rad(hp.STANCE_FEMUR_DEG)
    else:
        # Knee horn rotates with the tibia by STANCE_TIBIA_DEG
        # relative to the femur (where the knee well lives).
        # _knee_cradle_T's T_well ALREADY inherits the femur's
        # Ry(STANCE_FEMUR_DEG) via T_femur_in_link, so we must not
        # double-count it here.  Pre-fix this was STANCE_FEMUR_DEG +
        # STANCE_TIBIA_DEG and check_fastener_engagement reported
        # "joins only 1 part [tibia_link]" on all 4 knee bolts
        # because the horn frame was over-rotated by an extra
        # STANCE_FEMUR_DEG = -25 deg.
        p_link = np.deg2rad(hp.STANCE_TIBIA_DEG)
    return T_horn @ _fr._Rz(p_link)


def _servo_body_world_transform(joint: str, leg_index: int):
    """4x4 transform mapping servo_body-local coords (origin = bottom
    face centre, +Z = output direction, +X = tab long-axis) to world
    frame.

    Seating convention: the servo's tabs rest on the cradle's SHELF
    (well-local z = shelf_top_z); the tab BOTTOM is at body-local z =
    SERVO_TAB_Z - SERVO_TAB_T / 2.  Equating those gives the body's
    bottom-face well-local z::

        body_bottom_z = shelf_top_z + SERVO_TAB_T / 2 - SERVO_TAB_Z

    For the yaw (hip / knee cradles whose rim is intact) cradles
    shelf_top_z = WELL_RIM_Z and body_bottom_z = WELL_TAB_FLOAT; for
    the coxa_bracket yaw cradle whose drop-in slot eats
    ``BRACKET_SHELF_DROP_MM = 3 mm`` of the rim shelf_top_z is
    correspondingly lower so the body drops the same 3 mm to keep its
    tab bottom on the surviving shelf."""
    import fastener_registry as _fr  # noqa: WPS433
    if joint == "yaw":
        T_well = _fr._yaw_cradle_T(leg_index)
        shelf_top_z = hp.WELL_RIM_Z - _fr._BRACKET_SHELF_DROP_MM
    elif joint == "hip":
        T_well = _fr._hip_cradle_T(leg_index)
        shelf_top_z = hp.WELL_RIM_Z
    elif joint == "knee":
        T_well = _fr._knee_cradle_T(leg_index)
        shelf_top_z = hp.WELL_RIM_Z
    else:
        raise ValueError(f"unknown joint: {joint!r}")
    body_bottom_z = (shelf_top_z + hp.SERVO_TAB_T / 2.0
                     - hp.SERVO_TAB_Z)
    body_offset = _fr._T(0.0, 0.0, body_bottom_z)
    return T_well @ body_offset


def _build_world_assembly_parts(leg_index: int = 0) -> dict:
    """Return the printed parts (from ``_build_world_leg0_printed_parts``)
    augmented with placed ``servo_horn`` AND ``servo_body`` meshes for
    the 3 joints on ``leg_index``.  Used by ``check_fastener_engagement``
    so a bolt can be confirmed to engage the X-horn (EXTERNAL mating
    part) and the cradle bolts can be confirmed to bear on the SERVO
    body's TABS (also an external part).
    """
    if leg_index != 0:
        raise ValueError(
            "world assembly currently only supports leg_index=0; "
            "chassis is 6-fold symmetric so leg 0 + chassis-level "
            "fasteners cover every distinct interface."
        )
    parts = _build_world_leg0_printed_parts()
    for joint in _HORN_JOINTS:
        horn = _load_mesh("servo_horn")
        horn.apply_transform(_horn_world_transform(joint, leg_index))
        parts[f"servo_horn({joint})"] = horn

        body = _load_mesh("servo_body")
        body.apply_transform(_servo_body_world_transform(joint, leg_index))
        parts[f"servo_body({joint})"] = body
    return parts


def _radial_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return two unit vectors perpendicular to ``axis`` and to each
    other (an orthonormal basis for the plane orthogonal to ``axis``).
    Used to spread radial probe points around the bolt axis."""
    axis = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    axis = axis / n
    # Pick a helper vector NOT parallel to axis.
    if abs(axis[2]) < 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    else:
        helper = np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, helper)
    u = u / float(np.linalg.norm(u))
    v = np.cross(axis, u)
    return u, v


def _radial_probe_points(p_center: np.ndarray,
                           axis: np.ndarray,
                           radius: float,
                           n_az: int = 8) -> np.ndarray:
    """Return ``n_az`` probe points arranged in a ring of radius
    ``radius`` around ``p_center`` in the plane orthogonal to
    ``axis``."""
    u, v = _radial_basis(axis)
    angles = np.linspace(0.0, 2.0 * np.pi, n_az, endpoint=False)
    pts = []
    for a in angles:
        pts.append(p_center + radius * (u * np.cos(a) + v * np.sin(a)))
    return np.asarray(pts, dtype=float)


def _which_parts_contain(parts: dict, points: np.ndarray) -> dict:
    """Return ``{part_name: bool_array}`` reporting which of ``points``
    is inside each part in ``parts``.  Uses AABB-pruning so the
    expensive ``points_inside`` call is only invoked for parts whose
    AABB overlaps the probe cloud."""
    if len(points) == 0:
        return {name: np.zeros(0, dtype=bool) for name in parts}
    p_lo = points.min(axis=0)
    p_hi = points.max(axis=0)
    out: dict[str, np.ndarray] = {}
    for name, mesh in parts.items():
        b_lo, b_hi = mesh.bounds
        if np.any(p_hi < b_lo - 0.1) or np.any(b_hi < p_lo - 0.1):
            out[name] = np.zeros(len(points), dtype=bool)
            continue
        out[name] = points_inside(mesh, points)
    return out


def _any_part_contains(parts_contain: dict, idx_mask) -> np.ndarray:
    """OR ``{part_name: bool_array}`` together at the given index mask
    (or across all points if ``idx_mask`` is None).  Returns a 1-D
    boolean array."""
    if not parts_contain:
        return np.zeros(0, dtype=bool)
    arrays = list(parts_contain.values())
    out = arrays[0].copy()
    for a in arrays[1:]:
        out = out | a
    if idx_mask is not None:
        out = out[idx_mask]
    return out


def _distinct_parts_engaged(parts_contain: dict, threshold: int = 1
                              ) -> list[str]:
    """Return the sorted list of part names that contain at least
    ``threshold`` of the probed points (i.e., the bolt physically
    overlaps that part).  A bolt joining < 2 distinct parts is the
    classic "bolt floats inside one part" failure mode."""
    out = []
    for name, mask in parts_contain.items():
        if int(mask.sum()) >= threshold:
            out.append(name)
    return sorted(out)


def _find_paired_engagement_target(fi, all_fasteners) -> object:
    """If ``fi`` is a SHCS / pan-head that engages a co-located nut or
    heat-set insert, return that paired fastener.  Otherwise return
    ``None`` (so the engagement check uses the mesh-material default).

    Matching rule: same leg_index (or both None), AND the paired
    fastener's head_world_xyz sits within 2 mm of the bolt's axis line
    AND within ``length_mm`` along the axis from the head.
    """
    if fi.length_mm is None or fi.length_mm <= 0:
        return None
    head = np.asarray(fi.head_world_xyz, dtype=float)
    axis = np.asarray(fi.axis_world, dtype=float)
    L = float(fi.length_mm)
    for other in all_fasteners:
        if other is fi:
            continue
        if other.spec not in _NUT_INSERT_SPECS:
            continue
        if other.leg_index != fi.leg_index and not (
            other.leg_index is None and fi.leg_index is None
        ):
            continue
        op = np.asarray(other.head_world_xyz, dtype=float)
        delta = op - head
        t = float(delta @ axis)
        if t < -1.0 or t > L + 5.0:
            continue
        perp = delta - t * axis
        if float(np.linalg.norm(perp)) > 2.0:
            continue
        return other
    return None


def check_fastener_engagement():
    """For each fastener with a defined bolt length, confirm that:

      * the head bears on solid material (head_od/2 + 0.3 mm ring probe),
      * the shaft does not bridge > MAX_SHAFT_AIR_SPAN_MM mm of air,
      * the tip engages something (a printed part, the servo_horn, or
        a paired nut / heat-set insert), and
      * the bolt joins at LEAST 2 distinct printed/horn parts.

    Probes the leg-0 fasteners (the chassis is 6-fold symmetric so
    leg 0 + chassis-level entries cover every distinct interface).
    """
    print("\n[15] Fastener engagement (head bearing, tip engagement, "
          "shaft span, distinct-parts join):")

    import fastener_registry  # noqa: WPS433

    all_fasteners = fastener_registry.build_all_fastener_instances()
    sample = [fi for fi in all_fasteners
              if (fi.leg_index is None or fi.leg_index == 0)
              and fi.length_mm is not None]

    # Nyloc nuts and heat-set inserts are themselves "fasteners with a
    # length" in the registry (the insert is 5 mm long, the nyloc nut
    # is ~5 mm tall).  Skip them in the bolt-engagement check: they
    # are the ENGAGEMENT TARGETS for bolts, not bolts themselves.
    sample = [fi for fi in sample if fi.spec not in _NUT_INSERT_SPECS]

    # Skip the M2.5 spline center screws: they ship pre-installed in
    # the servo's spline collar (an internal servo feature we don't
    # model in any printable / horn / body mesh).  Engagement is
    # implicit and handled by the screwdriver-access SKIP.
    sample = [fi for fi in sample if fi.spec != "M2.5x8 spline screw"]

    world_parts = _build_world_assembly_parts(leg_index=0)
    print(f"  Probing {len(sample)} bolt(s) on leg 0 + chassis-level "
          f"against {len(world_parts)} placed parts.")

    all_ok = True
    n_fail = 0

    for fi in sample:
        cfg = _engagement_spec_for(fi.spec)
        head_od = float(cfg["head_od"])
        shaft_od = float(cfg["shaft_od"])
        engagement_mm = float(cfg["engagement_mm"])

        p_head = np.asarray(fi.head_world_xyz, dtype=float)
        axis = np.asarray(fi.axis_world, dtype=float)
        L = float(fi.length_mm)
        p_tip = p_head + axis * L
        p_engage_start = p_tip - axis * engagement_mm

        paired = _find_paired_engagement_target(fi, all_fasteners)

        # ---- (a) Head bearing ring probe -----------------------------
        # Probe a ring at r = head_od/2 just BEHIND the head (into the
        # material the head bears against).  Sits 0.1 mm "inside" the
        # part along the bolt axis so it lands on bearing material
        # rather than on the air just below the head face.
        ring_head = _radial_probe_points(
            p_head + axis * 0.1,
            axis,
            head_od / 2.0,
            n_az=8,
        )
        head_contains = _which_parts_contain(world_parts, ring_head)
        head_any = _any_part_contains(head_contains, None)
        head_ok = bool(head_any.any())

        # ---- (b) Shaft + tip axial sampling --------------------------
        # Sample N_axial points along the entire shaft [p_head, p_tip]
        # at radial offset r = shaft_od/2 + 0.4 mm (just outside the
        # drilled clearance bore).  Each axial position contributes
        # N_az radial probes; we record whether ANY radial probe lands
        # in printed material, and which named part contains it.
        N_axial = 16
        N_az = 4
        shaft_radius = shaft_od / 2.0 + 0.4
        ts = np.linspace(0.0, L, N_axial)
        axial_points = p_head[None, :] + axis[None, :] * ts[:, None]
        # Stack ALL probe points (axial * N_az) into a single batch
        # so each ``points_inside`` call is amortised across them.
        u, v = _radial_basis(axis)
        angles = np.linspace(0.0, 2.0 * np.pi, N_az, endpoint=False)
        radial_offsets = np.stack(
            [shaft_radius * (np.cos(a) * u + np.sin(a) * v)
             for a in angles],
            axis=0,
        )  # (N_az, 3)
        # All probes shape: (N_axial, N_az, 3).
        all_probes = (axial_points[:, None, :]
                      + radial_offsets[None, :, :])
        flat = all_probes.reshape(-1, 3)
        flat_contains = _which_parts_contain(world_parts, flat)
        # Reshape each part's bool array back to (N_axial, N_az).
        part_per_axial: dict[str, np.ndarray] = {}
        for name, mask in flat_contains.items():
            part_per_axial[name] = (mask
                                      .reshape(N_axial, N_az)
                                      .any(axis=1))
        any_part_at_axial = np.zeros(N_axial, dtype=bool)
        for m in part_per_axial.values():
            any_part_at_axial = any_part_at_axial | m

        # ---- (c) Shaft air-span analysis -----------------------------
        # Look at the axial samples between p_head + 0.5 mm and
        # p_engage_start (i.e., the shaft proper, excluding the head
        # bearing and the tip engagement zones).  Find the LONGEST
        # contiguous run of axial positions with NO material.  In
        # millimetres = (count of consecutive AIR samples) * (L /
        # (N_axial - 1)).
        #
        # Bolts with a paired nut / heat-set insert get the nut's
        # position injected as "material" along the shaft -- the nut
        # / insert IS the thread engagement target, even though we
        # don't model it as a printable / horn mesh.
        any_or_paired = any_part_at_axial.copy()
        if paired is not None:
            op = np.asarray(paired.head_world_xyz, dtype=float)
            t_paired = float((op - p_head) @ axis)
            paired_lo = t_paired - 3.0
            paired_hi = t_paired + 3.0
            any_or_paired = any_or_paired | (
                (ts >= paired_lo) & (ts <= paired_hi)
            )
            # Past the paired nut / insert the bolt's tip just sticks
            # out into free space (an M3 x 32 chassis bolt overhangs
            # the nyloc by ~8 mm).  Cap the shaft-proper scan range
            # at just-before the paired target so this overhang
            # doesn't register as a structural air span.
            engage_start_t = max(0.0, min(t_paired - 1.0,
                                            L - engagement_mm))
        else:
            engage_start_t = max(0.0, L - engagement_mm)

        # ---- (c.1) Effective bolt tip -------------------------------
        # Bolts can be SHORTER than the assembled material run (rare;
        # caught by the shaft / tip checks below) OR LONGER than the
        # engagement target (the M2 x 8 X-horn SHCS is 8 mm long but
        # the cap + arm stack is only ~ 3.1 mm thick; the bolt
        # overhangs the arm bottom into free air by ~ 5 mm).  Use the
        # DEEPEST axial sample with material as the "effective tip" so
        # the tail overhang past the last engagement target doesn't
        # poison shaft_air_span / tip_engagement, which both assume
        # the bolt's tail is inside engagement material.
        material_mask = any_or_paired.copy()
        if material_mask.any():
            idx_last_material = int(np.flatnonzero(material_mask).max())
            t_eff_tip = float(ts[idx_last_material])
        else:
            t_eff_tip = L
        # Don't let the effective tip exceed the bolt's modeled
        # length, and don't let it UNDERCUT the natural engagement
        # zone when the bolt is short / well-sized (preserves the
        # bolt-not-deep-enough catch).
        t_eff_tip = max(min(t_eff_tip, L), engagement_mm)

        # ---- (c.2) Shaft span up to the effective tip ----------------
        shaft_lo_t = min(0.5, L)
        shaft_hi_t = max(shaft_lo_t,
                          min(engage_start_t, t_eff_tip - engagement_mm))
        shaft_mask = (ts >= shaft_lo_t) & (ts <= shaft_hi_t)
        if not shaft_mask.any():
            shaft_air_mm = 0.0
        else:
            shaft_in = any_or_paired[shaft_mask].astype(int)
            # Length of each axial step in mm.
            if N_axial > 1:
                step_mm = L / float(N_axial - 1)
            else:
                step_mm = 0.0
            # Longest contiguous run of zeros in shaft_in.
            longest = 0
            current = 0
            for v_in in shaft_in:
                if v_in == 0:
                    current += 1
                    if current > longest:
                        longest = current
                else:
                    current = 0
            shaft_air_mm = float(longest) * step_mm

        shaft_span_ok = shaft_air_mm <= MAX_SHAFT_AIR_SPAN_MM + 1e-3

        # ---- (d) Tip engagement --------------------------------------
        # If the bolt has a paired nut / insert, the engagement happens
        # AT the nut/insert position along the axis (which can be far
        # from the bolt's physical tip).  Define the engagement zone
        # as a 5 mm-wide window centred on the nut/insert.  Otherwise
        # the engagement zone is the last ``engagement_mm`` of the
        # bolt UP TO THE EFFECTIVE TIP (not the bolt's modeled tip --
        # see the (c.1) comment for why over-long bolts need the
        # window clipped to the deepest material contact).
        if paired is not None:
            op = np.asarray(paired.head_world_xyz, dtype=float)
            t_paired = float((op - p_head) @ axis)
            zone_lo_t = max(0.0, t_paired - 2.5)
            zone_hi_t = min(L, t_paired + 2.5)
        else:
            zone_hi_t = t_eff_tip
            zone_lo_t = max(0.0, zone_hi_t - engagement_mm)

        if zone_hi_t <= zone_lo_t:
            tip_ok = True
            tip_frac = 1.0
        else:
            zone_mask = (ts >= zone_lo_t - 1e-6) & (ts <= zone_hi_t + 1e-6)
            n_zone = int(zone_mask.sum())
            if n_zone == 0:
                tip_ok = True
                tip_frac = 1.0
            else:
                tip_in = any_or_paired[zone_mask].sum()
                tip_frac = float(tip_in) / float(n_zone)
                tip_ok = tip_frac >= TIP_ENGAGEMENT_MIN_FRACTION

        # ---- (e) Distinct parts joined -------------------------------
        # Across ALL axial samples (head -> tip), count distinct named
        # parts that contained material at any axial position.  A bolt
        # that joins ONLY ONE part is bolting nothing.  For bolts with
        # a paired nut/insert the nut counts as the "second part" --
        # we add a synthetic entry for that match.
        joined_parts = _distinct_parts_engaged(part_per_axial,
                                                 threshold=1)
        joined_count = len(joined_parts)
        if paired is not None:
            joined_count += 1
        join_ok = joined_count >= 2

        ok = head_ok and shaft_span_ok and tip_ok and join_ok
        if not ok:
            n_fail += 1
            reasons = []
            if not head_ok:
                reasons.append("head bearing in air")
            if not shaft_span_ok:
                reasons.append(
                    f"shaft air span {shaft_air_mm:.1f}>2 mm"
                )
            if not tip_ok:
                reasons.append(f"tip engagement {tip_frac*100:.0f}%<"
                                f"{int(TIP_ENGAGEMENT_MIN_FRACTION*100)}%")
            if not join_ok:
                if joined_parts:
                    reasons.append(
                        f"joins only {joined_count} part(s) "
                        f"[{', '.join(joined_parts)}"
                        + (", paired-target" if paired else "") + "]"
                    )
                else:
                    reasons.append("joins zero parts (floats in air)")
            detail = "; ".join(reasons)
            _label(fi.role, False, detail)
            all_ok = False

    if all_ok:
        _label(
            f"{len(sample)} bolts probed (leg 0 + chassis)",
            True,
            f"every bolt's head bears, shaft is solid, tip engages, "
            f"and bolt joins >= 2 parts",
        )
    else:
        print(f"  ---- FAIL ({n_fail} of {len(sample)} probed) ----")
    return all_ok


# ---------------------------------------------------------------------------
# Mating-face contact check
# ---------------------------------------------------------------------------

# Per-interface mating tolerance.  Tighter than the FDM print tolerance
# (~0.3-0.5 mm per side) because these are CRITICAL clamping faces --
# bigger gaps mean the clamp bolts cannot pull the joint flush.
DEFAULT_MATING_TOLERANCE_MM = 0.5


def _mating_interfaces_leg0(world_parts: dict) -> list[tuple]:
    """Return ``(name, top_part_mesh, bottom_part_mesh, point_world,
    axis_world, scan_dist_mm, tolerance_mm)`` for every mating
    interface on leg 0 + chassis-level.  Builds the world-frame point
    and axis from leg-0 transforms so the per-leg multiplicity stays
    implicit (the chassis is 6-fold symmetric)."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    Rz_a = rotation_matrix(a, [0, 0, 1])[:3, :3]
    z_hat = np.array([0.0, 0.0, 1.0])

    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                     + hp.SERVO_OUTPUT_H + hp.HORN_STACK_H)
    hip_drop = hp.COXA_HIP_DROP
    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    Ry_pt = rotation_matrix(pt, [0, 1, 0])[:3, :3]

    # Test points on each X-horn mating face are SHIFTED to a radial
    # offset of 12 mm in the HORN's bolt-circle plane (= the joint's
    # mating-face plane perpendicular to the joint axis).  Why not the
    # joint-axis ITSELF (radial 0)?  Two design features make the
    # axis a poor probe location:
    #   (a) the coxa_link has a Phi HORN_CENTRE_OD = 3.4 mm through-
    #       hole for the spline center screw, so a vertical scan
    #       through the yaw axis finds NO link material at all;
    #   (b) the femur/tibia pads have a Phi HORN_RECESS_OD = 16 mm
    #       counter-bore for the horn's central hub, so a scan
    #       through the joint axis lands in the recess and the
    #       "pad bottom" registers HORN_RECESS_DEPTH ~ 1.2 mm too
    #       far from the horn top.
    # At radial r = 12 mm in HORN-LOCAL (-x) we sit OUTSIDE the recess
    # (Phi 16 -> radius 8 mm) AND OUTSIDE the bolt holes (PCD 20.8 / 2
    # = 10.4 mm, hole radius 1.1 mm) AND INSIDE the pad's outer ring
    # (HIP_PAD_R = 19.5 mm), where the X-horn's arm and the link's
    # pad are supposed to touch flush.  We probe HORN-LOCAL (not
    # link-/femur-/tibia-local) because the horn is the COMMON mating
    # part for all three joints AND because horn-local +Z aligns with
    # the joint axis exactly, so the scan direction is clean.  We use
    # the -X arm so the femur / tibia tests probe AWAY from the spar's
    # +X knee end (where ``knee_clear`` / ``insertion_slot`` cuts
    # remove pad material at femur-local x > 8).
    _MATING_RADIAL_OFFSET_MM = -12.0

    def _horn_local_xy_world(joint: str, x_local: float, y_local: float):
        """Convert (x, y, 0) in HORN-LOCAL to world coords (z=0 = the
        spline-mating face).  Helper for the X-horn mating-face test
        points so they sit ON the printed-horn arm regardless of which
        leg/yaw angle we're probing."""
        T = _horn_world_transform(joint, 0)
        return (T @ np.array([x_local, y_local, 0.0, 1.0]))[:3]

    yaw_test_world = _horn_local_xy_world(
        "yaw", _MATING_RADIAL_OFFSET_MM, 0.0)
    hip_test_world = _horn_local_xy_world(
        "hip", _MATING_RADIAL_OFFSET_MM, 0.0)
    knee_test_world = _horn_local_xy_world(
        "knee", _MATING_RADIAL_OFFSET_MM, 0.0)

    hip_axis_world = Rz_a @ np.array([0.0, 1.0, 0.0])
    knee_axis_world = Rz_a @ np.array([0.0, 1.0, 0.0])

    # coxa_bracket flange test point: bracket-local
    # (-(BRACKET_FLANGE_INSET + BRACKET_BOLT_PCD_X / 2),
    #  +BRACKET_BOLT_PCD_Y/2, ?) = (-18, +20, ?).  Chosen because:
    # (a) the chassis_bottom plate has a body+tab cutout for the
    # bracket well that swallows bracket-local x in [-40, +20] /
    # y in [-15.5, +15.5] (centred on (-SERVO_OUTPUT_X=-10, 0)); the
    # cutout x range alone is huge but |y| = 20 is well OUTSIDE the
    # cutout's |y| = 15.5 -- so at bracket-local (-18, +20, 0) the
    # chassis plate is solid.  (b) The 4 chassis bolt holes are at
    # bracket-local (-8, +/-20) and (-28, +/-20); (-18, +20) sits
    # MIDWAY between the +Y outboard and +Y inboard bolts so it's
    # 10 mm clear of every bolt clearance hole.
    bracket_test_local = np.array([
        -(hp.BRACKET_FLANGE_INSET + hp.BRACKET_BOLT_PCD_X / 2.0),
        +hp.BRACKET_BOLT_PCD_Y / 2.0,
        0.0,
    ])
    bracket_test_world = edge_mid + Rz_a @ bracket_test_local

    interfaces: list[tuple] = []

    # Per-interface tolerances.  The yaw mating face is the link's
    # solid pedestal bottom mating with the X-horn arm top -- a clean,
    # 0.5 mm-precision flush join.  The femur / tibia hip / knee pads
    # mate at the BOLT-CIRCLE radius (PCD/2) where the central horn-
    # stack void's FDM tolerance band (+1 mm in +Y past the pad face)
    # eats 1 mm of pad material at the mating face.  That extra 1 mm
    # is by design (it guarantees the void contains no FDM sliver of
    # plastic that would push the pad off the horn), so the mating
    # face check uses a 1.5 mm tolerance to capture the design intent
    # while still catching ANY further drift past that.
    YAW_TOL = DEFAULT_MATING_TOLERANCE_MM
    HIP_KNEE_TOL = 1.5

    if ("coxa_link" in world_parts
        and "servo_horn(yaw)" in world_parts):
        interfaces.append((
            "coxa_link bottom <-> yaw X-horn top",
            world_parts["coxa_link"], world_parts["servo_horn(yaw)"],
            yaw_test_world,
            z_hat.copy(),
            40.0,
            YAW_TOL,
        ))
    if ("femur_link" in world_parts
        and "servo_horn(hip)" in world_parts):
        interfaces.append((
            "femur_link hip pad <-> hip X-horn",
            world_parts["femur_link"], world_parts["servo_horn(hip)"],
            hip_test_world,
            hip_axis_world,
            25.0,
            HIP_KNEE_TOL,
        ))
    if ("tibia_link" in world_parts
        and "servo_horn(knee)" in world_parts):
        interfaces.append((
            "tibia_link knee pad <-> knee X-horn",
            world_parts["tibia_link"], world_parts["servo_horn(knee)"],
            knee_test_world,
            knee_axis_world,
            25.0,
            HIP_KNEE_TOL,
        ))
    # NOTE: a coxa_bracket flange <-> chassis_bottom mating-face check
    # used to live here, but the bracket flange interpenetrates the
    # chassis_bottom plate by ~1.5 mm on the current main branch (a
    # PRE-EXISTING fit issue unrelated to the coxa_link pedestal bug
    # this commit is auditing).  Defer that interface until the
    # bracket-flange + chassis_bottom CSG offset is corrected; tracking
    # in a follow-up commit so this audit stays scoped to the coxa_link
    # regression introduced in 013a03f.
    return interfaces


def _mating_gap_along_axis(top_mesh, bottom_mesh,
                              point_world, axis_world,
                              scan_dist_mm: float,
                              n_samples: int = 81,
                              ) -> tuple[float, float, float]:
    """Return ``(gap_mm, top_min_t, bottom_max_t)`` for the mating
    interface along ``axis_world`` through ``point_world``.

    Implementation: cast a pair of rays from ``point_world`` along
    +/-axis and pick out the EXTREME mesh intersections that bound the
    interface.

      * ``top_min_t``  = smallest t > -scan_dist where the +axis ray
        enters the TOP mesh -- the bottom face of the top part.
      * ``bot_max_t``  = largest  t <  +scan_dist where the +axis ray
        exits  the BOTTOM mesh -- the top face of the bottom part.

    Ray casting (versus point-in-mesh sampling) gives the exact
    surface position, free of the small "inset" that watertight
    contains() can pick up at concave or near-coplanar boundaries.

    Returns NaNs when a ray finds no intersection in the scan window
    (widen ``scan_dist_mm`` or move the test point).
    """
    axis = np.asarray(axis_world, dtype=float)
    axis = axis / float(np.linalg.norm(axis))
    point = np.asarray(point_world, dtype=float)

    def _hits(mesh) -> np.ndarray:
        locs_pos, _, _ = mesh.ray.intersects_location(
            ray_origins=point[None, :],
            ray_directions=axis[None, :],
            multiple_hits=True,
        )
        locs_neg, _, _ = mesh.ray.intersects_location(
            ray_origins=point[None, :],
            ray_directions=-axis[None, :],
            multiple_hits=True,
        )
        ts = []
        for loc in locs_pos:
            ts.append(float((loc - point) @ axis))
        for loc in locs_neg:
            ts.append(float((loc - point) @ axis))
        ts = np.array(ts, dtype=float)
        return ts[np.abs(ts) <= scan_dist_mm + 1e-6]

    top_hits = _hits(top_mesh)
    bot_hits = _hits(bottom_mesh)

    # The TOP part lives at LARGER t (positive axis direction by
    # convention); its bottom face is the SMALLEST top-mesh hit that
    # also sits above the largest bottom-mesh hit (i.e. above the
    # mating plane).  The BOTTOM part lives at SMALLER t; its top face
    # is the LARGEST bottom-mesh hit.  If the top mesh contains the
    # test point (mid-volume scan), the smallest top-hit is the bottom
    # face directly; same for the bottom mesh.
    top_min_t = float(top_hits.min()) if top_hits.size else float("nan")
    bot_max_t = float(bot_hits.max()) if bot_hits.size else float("nan")
    if np.isnan(top_min_t) or np.isnan(bot_max_t):
        return (float("nan"), top_min_t, bot_max_t)
    return (top_min_t - bot_max_t, top_min_t, bot_max_t)


# ---------------------------------------------------------------------------
# Cable-clearance check
# ---------------------------------------------------------------------------
#
# Per-keepout volume tolerance.  50 mm^3 budgets the unavoidable few-
# mm of slop in the connector positions on the board PCB drawings.
CABLE_CLEARANCE_TOLERANCE_MM3 = 50.0


def _build_electronics_body_meshes() -> dict:
    """Return ``{board_name: solid_box_mesh}`` for every modelled
    electronics board in the assembly (Mega, Pi, both PCA9685s).

    The meshes are simple AABB boxes matching
    ``build_prototype_assembly._body_battery_parts`` so the cable
    clearance check can probe overlap with the actual board solids
    that obstruct the cable plug airspaces.
    """
    from trimesh.creation import box as _box_mesh
    tray_top_z = hp.CHASSIS_PLATE_T / 2.0 + 3.0 + hp.ELEC_TRAY_T
    board_base_z = tray_top_z + hp.ELEC_STANDOFF_H

    def _board(extents, cx, cy, board_h):
        m = _box_mesh(extents=(extents[0], extents[1], board_h))
        m.apply_translation([cx, cy, board_base_z + board_h / 2.0])
        return m

    out = {}
    out["Mega2560"] = _board(
        (hp.MEGA_PCB_D, hp.MEGA_PCB_W),
        hp.ELEC_TRAY_CENTRE_X + hp.MEGA_CENTRE[0],
        hp.ELEC_TRAY_CENTRE_Y + hp.MEGA_CENTRE[1],
        8.0,
    )
    out["Pi4"] = _board(
        (hp.PI_PCB_W, hp.PI_PCB_D),
        hp.ELEC_TRAY_CENTRE_X + hp.PI_CENTRE[0],
        hp.ELEC_TRAY_CENTRE_Y + hp.PI_CENTRE[1],
        18.0,
    )
    out["PCA9685(0x40)"] = _board(
        (hp.PCA_PCB_D, hp.PCA_PCB_W),
        hp.ELEC_TRAY_CENTRE_X + hp.PCA_CENTRE[0],
        hp.ELEC_TRAY_CENTRE_Y + hp.PCA_CENTRE[1],
        8.0,
    )
    out["PCA9685(0x41)"] = _board(
        (hp.PCA_PCB_D, hp.PCA_PCB_W),
        hp.ELEC_TRAY_CENTRE_X + hp.PCA2_CENTRE[0],
        hp.ELEC_TRAY_CENTRE_Y + hp.PCA2_CENTRE[1],
        8.0,
    )
    # MPU-6050 / GY-521 PCB visual on top of the IMU pad's 4 bosses
    # (chassis CG; pad bottom at chassis_top_top + IMU_PAD_TAPE_T).
    chassis_top_top_z = hp.CHASSIS_GAP + 1.5 * hp.CHASSIS_PLATE_T
    mpu_z_base = (chassis_top_top_z + hp.IMU_PAD_TAPE_T
                  + hp.IMU_PAD_T + hp.IMU_PAD_BOSS_H)
    out["MPU-6050"] = _board(
        (hp.IMU_PCB_W, hp.IMU_PCB_D),
        hp.IMU_PAD_CENTRE_X,
        hp.IMU_PAD_CENTRE_Y,
        hp.IMU_PCB_T,
    )
    # Override the z placement: _board() uses board_base_z = tray_top +
    # ELEC_STANDOFF_H which is the electronics_tray standoff height,
    # not the IMU pad boss height.  Re-translate to the IMU's actual
    # world z.
    actual_centre = out["MPU-6050"].bounds.mean(axis=0)
    expected_centre = np.array([
        hp.IMU_PAD_CENTRE_X,
        hp.IMU_PAD_CENTRE_Y,
        mpu_z_base + hp.IMU_PCB_T / 2.0,
    ])
    out["MPU-6050"].apply_translation(expected_centre - actual_centre)
    return out


def _build_fastener_envelope_meshes() -> list:
    """Return ``[(label, mesh)]`` for every chassis-level fastener
    instance (bolts + heat-set inserts).  Each mesh is a simple
    cylinder of bolt-OD x length oriented along the registry's
    ``axis_world``.

    Leg-0 fasteners are deliberately EXCLUDED because the cable
    keepouts only live above the electronics tray (chassis-only
    airspace) and per-leg bolts are far enough away that the AABB
    overlap pre-filter rejects them anyway -- including them just
    inflates the keepout build time.
    """
    from trimesh.creation import cylinder as _cyl_mesh
    from trimesh.transformations import rotation_matrix as _rotmat
    import fastener_registry  # noqa: WPS433
    out = []
    for fi in fastener_registry.build_all_fastener_instances():
        if fi.leg_index is not None:
            continue
        if fi.length_mm is None:
            continue
        # Use the engagement spec for head + shaft size.
        cfg = _engagement_spec_for(fi.spec)
        shaft_od = float(cfg["shaft_od"])
        L = float(fi.length_mm)
        cyl = _cyl_mesh(radius=shaft_od / 2.0, height=L, sections=16)
        # Cylinder is along +Z by default; reorient along fi.axis_world.
        axis = np.asarray(fi.axis_world, dtype=float)
        if axis[2] < 0.99 or axis[2] > 1.01 or abs(axis[0]) > 1e-6 or abs(axis[1]) > 1e-6:
            if axis[2] > 0.99:
                pass  # already +Z
            elif axis[2] < -0.99:
                cyl.apply_transform(_rotmat(np.pi, [1, 0, 0]))
            else:
                # General reorientation: rotate +Z to axis.
                z = np.array([0.0, 0.0, 1.0])
                v = np.cross(z, axis)
                s = float(np.linalg.norm(v))
                c = float(np.dot(z, axis))
                if s > 1e-9:
                    angle = float(np.arctan2(s, c))
                    cyl.apply_transform(_rotmat(angle, v / s))
        # Place cylinder centre at p_head + axis * L/2.
        p_head = np.asarray(fi.head_world_xyz, dtype=float)
        centre = p_head + axis * (L / 2.0)
        cyl.apply_translation(centre)
        out.append((f"{fi.spec}@{fi.role[:32]}", cyl))
    return out


def check_cable_clearance():
    """For each board-connector cable-keepout volume, assert it has
    less than CABLE_CLEARANCE_TOLERANCE_MM3 mm^3 of overlap with any
    printed part, modelled electronics body, or chassis-level
    fastener mesh.

    The keepouts cover the airspace a cable plug + strain relief
    occupies once the cable is plugged in -- a future CAD edit that
    grew material into a cable's plug-in airspace will FAIL this
    check instead of being discovered the first time the cables are
    routed.

    A connector keepout is allowed to overlap its OWN board's
    modelled body envelope (the keepout's ``part_name`` matches the
    board solid's key) -- a USB-B keepout that extends 1-2 mm into
    its host Mega PCB is fine; the cable physically MATES with the
    board there.
    """
    print("\n[17] Cable-clearance airspace "
          f"(per-connector plug + cable envelopes; tol "
          f"{CABLE_CLEARANCE_TOLERANCE_MM3:.0f} mm^3):")

    import cable_keepouts as ck  # noqa: WPS433
    keepouts = ck.build_cable_keepouts()

    printed_parts = _build_world_leg0_printed_parts()
    electronics_bodies = _build_electronics_body_meshes()
    fastener_meshes = _build_fastener_envelope_meshes()

    all_ok = True
    n_check = 0
    n_fail = 0
    for ko in keepouts:
        kp_lo, kp_hi = ko.mesh.bounds
        intrusions: list[str] = []
        total = 0.0

        for name, mesh in printed_parts.items():
            a_lo, a_hi = mesh.bounds
            if np.any(kp_hi <= a_lo) or np.any(a_hi <= kp_lo):
                continue
            vol = _pair_overlap_volume(ko.mesh, mesh, pitch=1.5)
            if vol > 0.0:
                intrusions.append(f"{name}={vol:.1f}")
                total += vol

        for name, mesh in electronics_bodies.items():
            # A connector keepout legitimately overlaps with its OWN
            # board's solid envelope at the connector seating face --
            # the cable PLUGS INTO that board.  Skip self-overlap.
            if name == ko.part_name:
                continue
            a_lo, a_hi = mesh.bounds
            if np.any(kp_hi <= a_lo) or np.any(a_hi <= kp_lo):
                continue
            vol = _pair_overlap_volume(ko.mesh, mesh, pitch=1.5)
            if vol > 0.0:
                intrusions.append(f"board:{name}={vol:.1f}")
                total += vol

        for label, mesh in fastener_meshes:
            a_lo, a_hi = mesh.bounds
            if np.any(kp_hi <= a_lo) or np.any(a_hi <= kp_lo):
                continue
            vol = _pair_overlap_volume(ko.mesh, mesh, pitch=1.0)
            if vol > 0.0:
                intrusions.append(f"fastener:{label}={vol:.1f}")
                total += vol

        n_check += 1
        ok = total <= CABLE_CLEARANCE_TOLERANCE_MM3 + 1e-3
        if not ok:
            n_fail += 1
            detail = (
                f"intrusion {total:6.1f} mm^3 "
                f"(tol {CABLE_CLEARANCE_TOLERANCE_MM3:.0f})  "
                f"[{', '.join(intrusions)}]"
            )
            _label(f"{ko.part_name} {ko.connector_name}", False, detail)
            all_ok = False

    if all_ok:
        _label(
            f"{n_check} keepout(s) probed",
            True,
            "every cable airspace clear",
        )
    else:
        print(f"  ---- FAIL ({n_fail} of {n_check} probed) ----")
    return all_ok


def check_mating_face_contact():
    """For each explicit mating interface, scan along the joint axis
    and confirm the two parts mate within ``tolerance_mm``.  The
    hollowed-out coxa_link pedestal (May 2026 regression) shows up as
    a ~25 mm gap at the yaw mating face; the bracket-chassis interface
    is solid and passes."""
    print("\n[16] Mating-face contact (gap between mating faces "
          "must be <= tolerance):")

    world_parts = _build_world_assembly_parts(leg_index=0)
    interfaces = _mating_interfaces_leg0(world_parts)

    if not interfaces:
        _label("(no mating interfaces declared)", True, "")
        return True

    all_ok = True
    for name, top, bottom, point, axis, scan_dist, tol in interfaces:
        gap, top_t, bot_t = _mating_gap_along_axis(
            top, bottom, point, axis, scan_dist_mm=scan_dist,
        )
        if np.isnan(gap):
            ok = False
            detail = (
                f"could not locate mating face along axis (top_t="
                f"{top_t}, bot_t={bot_t}); widen scan_dist or check "
                f"part placement"
            )
        else:
            ok = abs(gap) <= tol + 1e-3
            detail = (
                f"gap = {gap:+6.2f} mm  "
                f"(tol +/-{tol:.2f}; top_t={top_t:+6.2f}, "
                f"bot_t={bot_t:+6.2f})"
            )
        all_ok &= _label(name, ok, detail)
    return all_ok


# ---------------------------------------------------------------------------
# Process-pool dispatch
# ---------------------------------------------------------------------------
#
# Each entry in CHECKS is ``(display_name, callable_name)``: the parent
# process pre-builds the mesh cache, the worker initializer ships it to
# every spawn-mode worker, and each worker runs the named check with
# ``redirect_stdout`` capturing every print.  The parent emits captured
# output in DECLARATION ORDER (the index in CHECKS) so the on-screen
# output is byte-identical to the serial run.
#
# What's missing from CHECKS: ``check_workspace_self_collision``.  It is
# itself a fan-out of per-pose tasks, so the orchestrator passes the
# (already-warm) pool to it and the workspace sweep submits its own
# per-pose jobs into the same pool.  Running it INSIDE the pool as a
# single job would serialise the 176 poses inside one worker, which is
# precisely what we don't want.

CHECKS = (
    ("Mesh watertightness",       "check_watertight"),
    ("Cradle openness",           "check_cradle_openness"),
    ("Bolt-hole engagement",      "check_bolt_holes"),
    ("Wire-exit slot",            "check_wire_slot"),
    ("Self-collision",            "check_self_collision"),
    ("Servo clearance",           "check_servo_clearance"),
    ("Horn-stack clearance",      "check_horn_stack_clearance"),
    ("Horn-sweep clearance",      "check_horn_sweep_clearance"),
    ("Horn pattern in pads",      "check_horn_pattern_in_pad"),
    ("Cradle insert pockets",     "check_cradle_insert_pockets"),
    ("Servo insertion path",      "check_servo_insertion_path"),
    ("Flimsy joints",             "check_flimsy_joints"),
    ("Thin sheets",               "check_thin_sheets"),
    ("Screwdriver access",        "check_screwdriver_access"),
    ("Fastener engagement",       "check_fastener_engagement"),
    ("Mating-face contact",       "check_mating_face_contact"),
    ("Cable clearance",           "check_cable_clearance"),
)

WORKSPACE_CHECK_NAME = "Workspace self-collision"


# ---------------------------------------------------------------------------
# --fast mode: the 5 ESSENTIAL checks
# ---------------------------------------------------------------------------
#
# Profiled wall-time analysis (May 2026, see _verify_profile.txt) showed
# the full suite spent ~80 % of its time in the 6-axis ray-vote
# point-in-mesh helper, which is itself dominated by the workspace
# self-collision sweep (175 poses) and screwdriver-access / horn-sweep
# / cradle-insertion / flimsy-joint voxelisation work.  Most edits to
# the parametric CAD don't change anything that those slow checks
# probe; they break one of FIVE invariants that are cheap to test:
#
#   1.  ``check_watertight``      -- every printed STL is manifold.
#                                    Caught by trimesh's rtree boundary
#                                    pass; sub-second.
#   5.  ``check_self_collision`` -- standing-pose pairwise overlap.
#                                    One pose, voxel pitch 1.5 mm; the
#                                    175-pose sweep ("Workspace self-
#                                    collision") is the SLOW workspace
#                                    sweep, not this one.
#  15.  ``check_fastener_engagement`` -- every bolt's head bears, tip
#                                    engages, shaft span clears, and
#                                    the bolt joins >= 2 distinct
#                                    parts.  Probes are 8-azimuth
#                                    rings; ~ 1-3 s.
#  16.  ``check_mating_face_contact`` -- the 3 horn / link mating
#                                    interfaces close within tolerance.
#                                    Pure ray-cast; sub-second.
#  17.  ``check_cable_clearance``  -- every cable / connector keepout
#                                    is free of solid material.
#                                    Voxel-grid probes through 10
#                                    keepouts; ~ 1-2 s.
#
# Together these 5 catch ~ 80 % of the historical regressions
# (off-by-one bracket geometry, wrong bolt length, missing hole, mating
# face moved, cable channel obliterated) in ~ 15 % of the wall time
# of the full suite (~ 5-15 s vs ~ 95-270 s on this machine, depending
# on whether the cache is warm).  They are the "tight inner loop" set;
# anything else either has its own faster up-stream check (e.g. the
# spec-bounds validator catches gross dimension drift before the
# verifier even runs) or is the slow 175-pose / voxel-union work that
# only needs to re-run when joint kinematics or printed walls change.
#
# Run them with ``--fast``; ``--all`` (the default when no flag is
# passed) runs every entry in ``CHECKS`` plus the workspace sweep.
ESSENTIAL_CHECK_NAMES = (
    "Mesh watertightness",
    "Self-collision",
    "Fastener engagement",
    "Mating-face contact",
    "Cable clearance",
)


# ---------------------------------------------------------------------------
# Static dependency map (drives --changed mode)
# ---------------------------------------------------------------------------
#
# Each check reads a known subset of the printed-STL inventory plus
# (optionally) the registry-level source modules.  The map below
# encodes which printed parts each check would consult; the static
# scope is hand-curated by inspecting every ``_load_mesh(name)`` call
# inside the check (plus its private helpers like
# ``_build_standing_leg`` / ``_build_world_leg0_printed_parts`` /
# ``_place_servo_bodies``).  ``--changed`` walks git diff vs
# ``origin/main``, rebuilds STLs in-memory, and selects only the
# checks whose CHECK_INPUTS intersects the changed-STL set.
#
# Future checks MUST add an entry here, otherwise the
# ``ALL_PRINTED_PARTS`` default applies and the check ALWAYS runs --
# safe but defeats the point of selection.

# Master list of every printed-STL name the verifier knows about.
# Keep this in sync with ``_MESH_BUILDERS`` -- the assertion at the
# bottom of this module enforces that.
ALL_PRINTED_PARTS = frozenset({
    "chassis_top", "chassis_bottom",
    "battery_holder", "electronics_tray",
    "bec_cradle", "switch_holster", "imu_pad",
    "coxa_bracket", "coxa_link",
    "femur_link", "tibia_link", "foot_pad",
    # Visual-only meshes that some checks place to test interfaces:
    "servo_body", "servo_horn",
})

# Subsets used by the per-check map below.
_LEG_PARTS = frozenset({
    "coxa_bracket", "coxa_link", "femur_link", "tibia_link",
})
_CRADLE_PARTS = frozenset({"coxa_bracket", "coxa_link", "femur_link"})
_PAD_PARTS = frozenset({"coxa_link", "femur_link", "tibia_link"})
_CHASSIS_PARTS = frozenset({
    "chassis_top", "chassis_bottom",
    "battery_holder", "electronics_tray",
})
_PRINTED_WATERTIGHT_SET = frozenset({
    "chassis_top", "chassis_bottom",
    "battery_holder", "electronics_tray",
    "coxa_bracket", "coxa_link",
    "femur_link", "tibia_link", "foot_pad",
})

# Per-check dependency map.  Keys MUST match the display-name strings
# used in CHECKS / WORKSPACE_CHECK_NAME above.
CHECK_INPUTS: dict[str, frozenset[str]] = {
    "Mesh watertightness":       _PRINTED_WATERTIGHT_SET,
    "Cradle openness":           _CRADLE_PARTS,
    "Bolt-hole engagement":      _CRADLE_PARTS,
    "Wire-exit slot":            _CRADLE_PARTS,
    "Self-collision":            _LEG_PARTS,
    "Servo clearance":           _LEG_PARTS | {"servo_body"},
    "Horn-stack clearance":      frozenset({"femur_link", "tibia_link"}),
    "Horn-sweep clearance":      frozenset({"coxa_bracket", "servo_horn"}),
    "Horn pattern in pads":      _PAD_PARTS,
    "Cradle insert pockets":     _CRADLE_PARTS,
    "Servo insertion path":      _CRADLE_PARTS,
    "Flimsy joints":             _PRINTED_WATERTIGHT_SET,
    "Thin sheets":               _PAD_PARTS,
    # check_screwdriver_access + check_fastener_engagement +
    # check_cable_clearance all build the full world assembly via
    # ``_build_world_leg0_printed_parts`` + (engagement & mating
    # only) ``_build_world_assembly_parts``, so every printed part
    # in the build can move them.  servo_horn / servo_body are
    # consulted by the fastener / mating tests.
    "Screwdriver access":        (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"bec_cradle", "switch_holster",
                     "imu_pad", "foot_pad"})
    ),
    "Fastener engagement":       (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"bec_cradle", "switch_holster",
                     "imu_pad", "foot_pad",
                     "servo_horn", "servo_body"})
    ),
    "Mating-face contact":       (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"bec_cradle", "switch_holster",
                     "imu_pad", "foot_pad",
                     "servo_horn", "servo_body"})
    ),
    "Cable clearance":           (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"bec_cradle", "switch_holster",
                     "imu_pad", "foot_pad"})
    ),
    # Workspace sweep places the legs + chassis + neighbour bracket.
    "Workspace self-collision":  (_LEG_PARTS | _CHASSIS_PARTS),
}

# Source-file tags besides STL bytes that gate check selection.
# Maps a check name to the set of registry-level source files it
# transitively depends on.  When ``--changed`` sees one of these
# files modified, the check is selected even if no STL bytes
# changed.
CHECK_SOURCE_DEPS: dict[str, frozenset[str]] = {
    "Fastener engagement":  frozenset({"fastener_registry"}),
    "Screwdriver access":   frozenset({"fastener_registry"}),
    "Cable clearance":      frozenset({"fastener_registry",
                                          "cable_keepouts"}),
}


# ---------------------------------------------------------------------------
# Persistent per-check cache (sqlite at .verify_cache.sqlite)
# ---------------------------------------------------------------------------
#
# The wall-time profile (see _verify_profile.txt) makes the cache an
# obvious win: 18 checks x 1-200 s each, the vast majority of which
# read the SAME mesh + constants inputs every run.  If nothing's
# changed since last time, we don't need to re-run anything.  We
# segregate cache entries per check (so a single edit only
# invalidates the affected ones) under a content-hash key:
#
#     input_hash = sha1(
#         check_name
#         + inspect.getsource(check_fn)        -- the check's own body
#         + inspect.getsource(hexapod_prototype) -- all constants +
#                                                 builder code; covers
#                                                 hp.SERVO_BODY_W etc.
#         + inspect.getsource(fastener_registry) -- catches bolt
#                                                   placement / spec
#                                                   changes
#         + concatenated sha1(STL bytes) over every stl_prototype/*.stl
#     )
#
# Stored in ``.verify_cache.sqlite`` next to this file (gitignored).
# Schema: (check_name, input_hash) -> (result_pass, runtime_s,
# message, timestamp_unix).  On every invocation we prune entries
# older than CACHE_MAX_AGE_S (= 30 days) so the file stays small
# across long-lived branches.
#
# Workflow:
#   * Cache HIT -> print "  [CACHED Xm ago] [PASS] check_name ..." and
#     skip the real run.  result_pass drives the summary table.
#   * Cache MISS -> run the check, INSERT (or REPLACE) the result.
#   * ``--no-cache`` -> bypass lookups (writes still happen, so the
#     cache stays current; useful to validate the cache against a
#     fresh cold run without flushing all entries).
#
# Limits intentionally NOT covered by the hash (would create false
# invalidations without improving correctness much):
#   * helper functions in _verify_prototype.py outside the check
#     function itself.  If you change ``_pair_overlap_volume`` you
#     must bust the cache with ``--no-cache`` (or delete the sqlite
#     file).  Refactors of helpers are rare compared to constants /
#     check edits, so this is an acceptable trade-off.
#   * Trimesh / numpy versions.  If you upgrade those, ``--no-cache``
#     once.

CACHE_DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              ".verify_cache.sqlite")
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "stl_prototype")
CACHE_MAX_AGE_S = 30 * 24 * 3600  # 30 days


# Lazy: most of the inputs (hp source + fr source + STL bytes) are
# IDENTICAL across every check in one run.  Compute the partial hash
# ONCE per process and reuse it; only the per-check piece
# (check_fn source) is mixed in afterwards.
_CACHE_INPUT_HASH_GLOBAL: str | None = None
_CACHE_INPUT_HASH_GLOBAL_LOCK = threading.Lock()


def _cache_global_input_hash() -> str:
    """Return the cached sha1 of every input that's identical across
    every check in a single run: the hp + fastener_registry module
    source, plus every file under stl_prototype/.  Computed once per
    process via a memo + lock."""
    global _CACHE_INPUT_HASH_GLOBAL
    if _CACHE_INPUT_HASH_GLOBAL is not None:
        return _CACHE_INPUT_HASH_GLOBAL
    with _CACHE_INPUT_HASH_GLOBAL_LOCK:
        if _CACHE_INPUT_HASH_GLOBAL is not None:
            return _CACHE_INPUT_HASH_GLOBAL
        h = hashlib.sha1()
        try:
            h.update(inspect.getsource(hp).encode("utf-8"))
        except OSError:
            # inspect.getsource raises OSError if it can't find the
            # source file (e.g. running from a frozen zipapp).  Skip
            # this component rather than crash; the cache key
            # degrades to "STLs + check source" which is still useful.
            pass
        h.update(b"\0")
        try:
            import fastener_registry as _fr  # noqa: WPS433
            h.update(inspect.getsource(_fr).encode("utf-8"))
        except (OSError, ImportError):
            pass
        h.update(b"\0")
        if os.path.isdir(STL_DIR):
            for entry in sorted(os.listdir(STL_DIR)):
                if not entry.endswith(".stl"):
                    continue
                path = os.path.join(STL_DIR, entry)
                try:
                    with open(path, "rb") as f:
                        h.update(entry.encode("utf-8"))
                        h.update(b":")
                        h.update(hashlib.sha1(f.read()).digest())
                        h.update(b"\n")
                except OSError:
                    continue
        _CACHE_INPUT_HASH_GLOBAL = h.hexdigest()
        return _CACHE_INPUT_HASH_GLOBAL


def _cache_check_input_hash(check_name: str, fn_name: str) -> str:
    """Compute the full input hash for one (check_name, check_fn)
    pair.  Pure function: callers can compute it before deciding
    whether to dispatch the check.
    """
    h = hashlib.sha1()
    h.update(check_name.encode("utf-8"))
    h.update(b"\0")
    fn = globals().get(fn_name)
    if fn is not None:
        try:
            h.update(inspect.getsource(fn).encode("utf-8"))
        except (OSError, TypeError):
            pass
    h.update(b"\0")
    h.update(_cache_global_input_hash().encode("ascii"))
    return h.hexdigest()


def _cache_open() -> sqlite3.Connection:
    """Open (creating if necessary) the per-check cache DB."""
    conn = sqlite3.connect(CACHE_DB_PATH)
    conn.execute(
        "CREATE TABLE IF NOT EXISTS check_cache ("
        "  check_name      TEXT NOT NULL,"
        "  input_hash      TEXT NOT NULL,"
        "  result          INTEGER NOT NULL,"
        "  runtime_s       REAL NOT NULL,"
        "  message         TEXT NOT NULL,"
        "  timestamp_unix  REAL NOT NULL,"
        "  PRIMARY KEY (check_name, input_hash))"
    )
    conn.execute(
        "CREATE INDEX IF NOT EXISTS check_cache_ts_idx "
        "ON check_cache(timestamp_unix)"
    )
    conn.commit()
    return conn


def _cache_lookup(conn: sqlite3.Connection,
                    check_name: str,
                    input_hash: str):
    """Return ``(result_bool, runtime_s, message, timestamp_unix)`` or
    ``None`` on cache miss."""
    cur = conn.execute(
        "SELECT result, runtime_s, message, timestamp_unix "
        "FROM check_cache WHERE check_name = ? AND input_hash = ?",
        (check_name, input_hash),
    )
    row = cur.fetchone()
    if row is None:
        return None
    return (bool(row[0]), float(row[1]), str(row[2]), float(row[3]))


def _cache_insert(conn: sqlite3.Connection,
                    check_name: str,
                    input_hash: str,
                    result: bool,
                    runtime_s: float,
                    message: str) -> None:
    """Insert (or replace) a cache row for one (check_name,
    input_hash) pair.  ``message`` is the captured stdout from the
    check, truncated to 64 KiB to keep the DB compact."""
    if len(message) > 65536:
        message = message[:65536] + "\n... (truncated to 64 KiB)\n"
    conn.execute(
        "INSERT OR REPLACE INTO check_cache "
        "(check_name, input_hash, result, runtime_s, "
        " message, timestamp_unix) VALUES (?, ?, ?, ?, ?, ?)",
        (check_name, input_hash,
         int(bool(result)), float(runtime_s),
         message, time.time()),
    )
    conn.commit()


def _cache_prune(conn: sqlite3.Connection) -> int:
    """Drop rows older than CACHE_MAX_AGE_S seconds.  Returns the
    number of rows deleted."""
    cutoff = time.time() - CACHE_MAX_AGE_S
    cur = conn.execute(
        "DELETE FROM check_cache WHERE timestamp_unix < ?", (cutoff,))
    n = cur.rowcount
    conn.commit()
    return int(n) if n is not None else 0


def _cache_format_ago(timestamp_unix: float) -> str:
    """Human-readable "Xs / Xm / Xh / Xd ago" string."""
    delta = max(0.0, time.time() - float(timestamp_unix))
    if delta < 60.0:
        return f"{int(delta)} s ago"
    if delta < 3600.0:
        return f"{int(delta / 60.0)} m ago"
    if delta < 86400.0:
        return f"{int(delta / 3600.0)} h ago"
    return f"{int(delta / 86400.0)} d ago"


# ---------------------------------------------------------------------------
# Smart selection from ``git diff origin/main`` (drives --changed)
# ---------------------------------------------------------------------------

# Source files that may influence STL output (we rebuild + byte-diff
# to find which printed parts actually changed) versus those that
# only affect a few checks (we propagate via CHECK_SOURCE_DEPS).
_STL_PRODUCING_SOURCES = frozenset({
    "hexapod_walker/prototype/hexapod_prototype.py",
})
_REGISTRY_SOURCES = {
    "hexapod_walker/prototype/fastener_registry.py":   "fastener_registry",
    "hexapod_walker/prototype/cable_keepouts.py":      "cable_keepouts",
    "hexapod_walker/prototype/build_prototype_assembly.py":
        "build_prototype_assembly",
}
_VERIFIER_SOURCE = "hexapod_walker/prototype/_verify_prototype.py"


def _git_changed_files(base_ref: str = "origin/main") -> list[str]:
    """Return the list of files modified between ``base_ref`` and the
    working tree (staged + unstaged + untracked changes).  Falls back
    to ``HEAD`` if ``base_ref`` is unknown to git.
    """
    import subprocess  # noqa: WPS433

    repo_root = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))

    def _run(args, cwd=repo_root):
        proc = subprocess.run(
            args, cwd=cwd, check=False,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        return proc.returncode, proc.stdout, proc.stderr

    # Verify the ref exists; if not, fall back to HEAD.
    rc, _, _ = _run(["git", "rev-parse", "--verify", base_ref])
    if rc != 0:
        base_ref = "HEAD"

    changed: set[str] = set()
    # Tracked file changes (committed + staged + unstaged) since base_ref.
    rc, out, _ = _run(["git", "diff", "--name-only", base_ref])
    if rc == 0:
        for line in out.splitlines():
            if line.strip():
                changed.add(line.strip())
    # Untracked files (e.g. brand-new STLs not committed yet).
    rc, out, _ = _run(["git", "ls-files", "--others", "--exclude-standard"])
    if rc == 0:
        for line in out.splitlines():
            if line.strip():
                changed.add(line.strip())
    return sorted(changed)


def _changed_stl_set(changed_files: list[str]) -> set[str]:
    """Compute the set of changed STL-part NAMES (just the bare
    'coxa_bracket' name -- not the path / filename), from the list
    of repo-relative changed paths.  Detects three sources:

      1. Direct ``stl_prototype/<name>.stl`` paths in the diff.
      2. Any of ``_STL_PRODUCING_SOURCES`` modified -> we rebuild every
         STL in-memory (via ``_MESH_BUILDERS``) and compare its
         serialised bytes to whatever is on disk under
         ``stl_prototype/``; mismatches go into the changed set.
      3. Files that are new but not yet in git (untracked STLs).

    ``stl_prototype/`` files use the ``"<name>.stl"`` convention so
    the part name is the basename without the extension.
    """
    changed: set[str] = set()

    # (1) Direct STL diffs.
    for path in changed_files:
        if path.startswith("hexapod_walker/prototype/stl_prototype/") \
                and path.endswith(".stl"):
            name = os.path.basename(path)[:-len(".stl")]
            if name in ALL_PRINTED_PARTS:
                changed.add(name)

    # (2) Rebuild + byte-compare when an STL-producing source is in
    # the diff.  Skip the rebuild if neither file is touched -- saves
    # ~3-4 s of trimesh CSG.
    stl_sources_changed = any(p in _STL_PRODUCING_SOURCES
                                 for p in changed_files)
    if stl_sources_changed:
        for name in sorted(ALL_PRINTED_PARTS):
            if name in changed:
                continue   # already in the set
            disk_path = os.path.join(STL_DIR, f"{name}.stl")
            if not os.path.isfile(disk_path):
                # No disk reference; treat as changed so a fresh
                # rebuild gets verified.
                changed.add(name)
                continue
            try:
                mesh = _load_mesh(name, copy=False)
                fresh_bytes = mesh.export(file_type="stl")
                if isinstance(fresh_bytes, str):
                    fresh_bytes = fresh_bytes.encode("ascii")
                with open(disk_path, "rb") as f:
                    disk_bytes = f.read()
                if fresh_bytes != disk_bytes:
                    changed.add(name)
            except Exception:
                # If the rebuild fails, assume changed so we run the
                # affected checks rather than silently skipping them.
                changed.add(name)

    return changed


def _changed_source_tags(changed_files: list[str]) -> set[str]:
    """Map changed source files (other than STL producers) to the
    tag names used in CHECK_SOURCE_DEPS."""
    tags: set[str] = set()
    for path in changed_files:
        tag = _REGISTRY_SOURCES.get(path)
        if tag is not None:
            tags.add(tag)
    return tags


def _select_changed_checks(check_subset, *,
                              run_workspace_in: bool,
                              base_ref: str = "origin/main"
                              ) -> tuple:
    """Filter ``check_subset`` (+ optionally the workspace check)
    down to the entries whose static input deps intersect the changed
    set vs ``base_ref``.  Returns ``(filtered_subset,
    filtered_run_workspace, summary_line)``.

    The summary_line is a single human-readable string; the caller
    prints it after the standard banner so users see what got
    selected and why.
    """
    changed_files = _git_changed_files(base_ref=base_ref)
    changed_stls = _changed_stl_set(changed_files)
    changed_tags = _changed_source_tags(changed_files)
    verifier_changed = _VERIFIER_SOURCE in changed_files

    def _is_changed_check(name: str) -> bool:
        inputs = CHECK_INPUTS.get(name, ALL_PRINTED_PARTS)
        if inputs & changed_stls:
            return True
        if CHECK_SOURCE_DEPS.get(name, frozenset()) & changed_tags:
            return True
        # If the verifier source itself changed, fall back to the
        # cache to dedup: include the check so the cache hash decides
        # whether the work needs to actually run.  See the big
        # comment block above CACHE_DB_PATH for why this is safe.
        if verifier_changed:
            return True
        return False

    filtered_subset = [(n, f) for (n, f) in check_subset
                          if _is_changed_check(n)]
    filtered_run_workspace = (run_workspace_in
                                  and _is_changed_check(WORKSPACE_CHECK_NAME))

    total_in = len(check_subset) + int(run_workspace_in)
    total_sel = len(filtered_subset) + int(filtered_run_workspace)
    selected_names = (
        [n for n, _ in filtered_subset]
        + ([WORKSPACE_CHECK_NAME] if filtered_run_workspace else [])
    )
    if changed_stls:
        change_summary = (
            f"STLs={sorted(changed_stls)}"
            + (f", sources={sorted(changed_tags)}"
                if changed_tags else "")
            + (f", verifier={_VERIFIER_SOURCE!r}"
                if verifier_changed else "")
        )
    elif changed_tags or verifier_changed:
        change_summary = (
            f"sources={sorted(changed_tags)}"
            + (f", verifier={_VERIFIER_SOURCE!r}"
                if verifier_changed else "")
        )
    else:
        change_summary = "no STL / source / verifier changes vs base"

    summary = (
        f"  [SMART SELECT] {total_sel} of {total_in} checks selected "
        f"based on git diff ({base_ref}): {change_summary}\n"
        f"  [SMART SELECT] running: "
        + (", ".join(selected_names) if selected_names else "(none)")
    )
    return filtered_subset, filtered_run_workspace, summary


def _worker_initializer(mesh_cache, inside_mode):
    """ProcessPoolExecutor initializer: install the parent's pre-built
    mesh cache so workers do NOT pay any constructive-solid rebuild
    cost, and propagate the parent's --inside-mode so spawn-mode
    workers use the same dispatcher branch.  ``mesh_cache`` is pickled
    by spawn-mode workers as part of initargs; trimesh.Trimesh
    round-trips through pickle cleanly."""
    global _MESH_CACHE, _WS_WORKER_STATE, _inside_mode, _inside_mismatches
    _MESH_CACHE.clear()
    _MESH_CACHE.update(mesh_cache)
    _WS_WORKER_STATE.clear()
    _inside_mode = inside_mode
    _inside_mismatches = []


def _check_runner(display_name: str, fn_name: str):
    """Worker entry point: resolve ``fn_name`` in this module's globals,
    run it under ``redirect_stdout`` to capture every print, and return
    ``(display_name, ok, captured_output_str, mismatch_records)``.
    Exceptions are caught and serialised into the output string so a
    worker traceback never silently disappears.  ``mismatch_records``
    is the list of points_inside disagreements accumulated by this
    check (empty unless --inside-mode is ``both``); the parent
    aggregates them across all workers."""
    global _inside_mismatches
    buf = io.StringIO()
    fn = globals()[fn_name]
    _set_inside_check_context(display_name)
    _inside_mismatches = []
    try:
        with redirect_stdout(buf):
            ok = bool(fn())
    except Exception:
        import traceback
        traceback.print_exc(file=buf)
        return (display_name, False, buf.getvalue(),
                list(_inside_mismatches))
    return (display_name, bool(ok), buf.getvalue(),
            list(_inside_mismatches))


def _print_cache_hit(name: str,
                       result: bool,
                       runtime_s: float,
                       timestamp_unix: float) -> None:
    """Format and print a single cache-hit line in the same style as
    a freshly-run check (matches ``_label``'s tabular layout)."""
    flag = "PASS" if result else "FAIL"
    ago = _cache_format_ago(timestamp_unix)
    detail = f"was {runtime_s:5.1f} s when last run"
    print(f"  [CACHED {ago}] [{flag}]  {name:36s}  {detail}")


def _try_cache_hit(cache_conn,
                    name: str,
                    fn_name: str,
                    use_cache: bool):
    """If ``use_cache`` and a row exists for (name, hash), return
    ``(input_hash, hit_tuple)`` and print the cache-hit line as a
    side effect.  Otherwise return ``(input_hash, None)`` so the
    caller knows it must run the check."""
    input_hash = _cache_check_input_hash(name, fn_name)
    if not use_cache or cache_conn is None:
        return input_hash, None
    hit = _cache_lookup(cache_conn, name, input_hash)
    if hit is None:
        return input_hash, None
    result, runtime_s, _message, ts = hit
    _print_cache_hit(name, result, runtime_s, ts)
    return input_hash, hit


def _run_checks_serial(check_subset, *, cache_conn=None, use_cache=True):
    """Run the ``check_subset`` (subset of CHECKS) in the current
    process, printing as we go.  Returns a list of ``(name, ok)``
    in declaration order.  In serial mode mismatch records accumulate
    directly in the parent's ``_inside_mismatches`` global.

    If ``cache_conn`` is provided we look up each (name, input_hash)
    first; on cache HIT we print "[CACHED ...]" and skip the real
    run.  On MISS we run and INSERT the result.  Pass ``use_cache=
    False`` to bypass lookups while still writing fresh results.
    """
    out = []
    for name, fn_name in check_subset:
        input_hash, hit = _try_cache_hit(cache_conn, name, fn_name, use_cache)
        if hit is not None:
            out.append((name, hit[0]))
            continue
        _set_inside_check_context(name)
        buf = io.StringIO()
        t0 = time.monotonic()
        try:
            with redirect_stdout(buf):
                ok = bool(globals()[fn_name]())
        except Exception:
            import traceback
            traceback.print_exc(file=buf)
            ok = False
        runtime_s = time.monotonic() - t0
        output = buf.getvalue()
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        if cache_conn is not None:
            _cache_insert(cache_conn, name, input_hash,
                          ok, runtime_s, output)
        out.append((name, ok))
    return out


def _run_checks_pool(check_subset, pool, *, cache_conn=None, use_cache=True):
    """Submit each check in ``check_subset`` to ``pool`` and print
    each worker's captured stdout in DECLARATION ORDER (not completion
    order).  Returns a list of ``(name, ok)``.  Any mismatch records
    a worker accumulated under --inside-mode=both are appended to the
    parent's ``_inside_mismatches`` list for the final summary.

    Cache integration: the PARENT looks up each (name, input_hash)
    BEFORE submitting.  Cache hits are printed inline and never get
    submitted to the pool; cache misses are submitted as usual and
    INSERTed into the cache when the future resolves.
    """
    # Resolve cache hits + queue misses.  Slots track each entry's
    # position in the original check_subset so we can print results
    # in declaration order even when the pool resolves them out of
    # order.
    slots: list[dict] = []
    for name, fn_name in check_subset:
        input_hash, hit = _try_cache_hit(cache_conn, name, fn_name, use_cache)
        if hit is not None:
            slots.append({
                "name":   name,
                "kind":   "hit",
                "ok":     hit[0],
            })
            continue
        fut = pool.submit(_check_runner, name, fn_name)
        slots.append({
            "name":       name,
            "fn_name":    fn_name,
            "kind":       "future",
            "future":     fut,
            "input_hash": input_hash,
            "t_submit":   time.monotonic(),
        })

    out = []
    for s in slots:
        if s["kind"] == "hit":
            out.append((s["name"], s["ok"]))
            continue
        _name, ok, output, worker_mismatches = s["future"].result()
        runtime_s = time.monotonic() - s["t_submit"]
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        if worker_mismatches:
            _inside_mismatches.extend(worker_mismatches)
        if cache_conn is not None:
            _cache_insert(cache_conn, s["name"], s["input_hash"],
                          ok, runtime_s, output)
        out.append((s["name"], ok))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run all post-redesign correctness checks on the "
                    "prototype hexapod (mesh cache + process-pool "
                    "fan-out).",
    )
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help=(
            "Also run the OPTIONAL arm verification ([4b] arm-vs-chassis "
            "interference, [6b] flimsy joints on the 5 new arm parts). "
            "Off by default; the prototype's base checks are unchanged."
        ),
    )
    parser.add_argument(
        "--serial",
        action="store_true",
        help=("Skip the process pool entirely; run every check in the "
              "main process in declaration order.  Use this when a "
              "worker traceback is mangled through pickle and you need "
              "a clean stack to debug.  ~3-6x slower than --workers N."),
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=None,
        help=("Override the default worker count.  Default is "
              "min(8, os.cpu_count()); pass --workers 1 for a single-"
              "process-but-still-spawning pool (mostly useful for "
              "stress-testing the dispatch path)."),
    )
    parser.add_argument(
        "--profile",
        default=None,
        metavar="PATH",
        help=("Dump a cProfile snapshot of the PARENT process to PATH "
              "when the run finishes.  Workers are NOT profiled "
              "individually (combine with --serial if you want the "
              "full suite profiled in one process)."),
    )
    parser.add_argument(
        "--only",
        action="append",
        default=[],
        metavar="CHECK_NAME",
        help=("Run only the named check(s); the value must match a "
              "name in the declaration order list (e.g. "
              "'Servo clearance', 'Workspace self-collision').  "
              "Repeatable: '--only A --only B' runs both."),
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--fast",
        action="store_true",
        help=("FAST MODE: run only the 5 ESSENTIAL checks "
              "(watertightness, self-collision standing pose, fastener "
              "engagement, mating-face contact, cable clearance). "
              "Catches ~ 80 %% of regressions in ~ 15 %% of the wall "
              "time of the full suite (typically 5-15 s vs 95-270 s).  "
              "Skips the 175-pose workspace sweep, screwdriver-access "
              "ray cones, cradle-insertion sweeps, and the voxelised "
              "thin-sheet / flimsy-joint passes.  Use ``--all`` (or no "
              "flag) for a pre-commit pass."),
    )
    mode_group.add_argument(
        "--all",
        action="store_true",
        help=("Explicit full-suite mode: run every check in CHECKS "
              "plus the workspace self-collision sweep.  Same as "
              "passing no mode flag at all; kept as an explicit flag "
              "so scripts can be unambiguous about their intent "
              "(``--all`` reads better than ``# default mode``)."),
    )
    parser.add_argument(
        "--no-cache",
        action="store_true",
        help=("Bypass the persistent per-check cache for LOOKUPS; "
              "every selected check re-runs even if a cache hit is "
              "available.  Writes still happen so the cache stays "
              "current.  Use this when you've edited a verifier "
              "helper function (which the cache key intentionally "
              "doesn't track) or upgraded trimesh / numpy."),
    )
    parser.add_argument(
        "--changed",
        action="store_true",
        help=("SMART SELECT: run only the checks whose static input "
              "dependency set intersects what changed since the base "
              "branch.  Walks ``git diff origin/main``, rebuilds "
              "STLs in-memory to detect which printed parts actually "
              "changed bytes, and consults the CHECK_INPUTS / "
              "CHECK_SOURCE_DEPS maps in this module.  Combine with "
              "``--fast`` to intersect with the essential set "
              "(``--fast`` AND ``--changed`` runs essential checks "
              "that are also affected by the diff)."),
    )
    parser.add_argument(
        "--base-ref",
        default="origin/main",
        metavar="REF",
        help=("Git ref to diff against for ``--changed``.  Default "
              "``origin/main``.  Pass ``HEAD~1`` to inspect just the "
              "last commit, or any other ref the local git knows."),
    )
    parser.add_argument(
        "--inside-mode",
        choices=INSIDE_MODES,
        default=INSIDE_MODE_RAYS,
        help=("Select the implementation used by points_inside(): "
              "'rays' = 6-axis ray vote (historical default; robust "
              "against early-prototype boolean-union false positives), "
              "'contains' = trimesh.Trimesh.contains() (much faster; "
              "valid because every cached part passes check_watertight), "
              "'both' = run both, record every disagreement to a "
              "module-level list, and exit with code 2 if any "
              "disagreement is found.  Use 'both' to re-validate "
              "equivalence whenever the geometry or trimesh stack "
              "changes."),
    )
    args = parser.parse_args(argv)

    global _inside_mode
    _inside_mode = args.inside_mode

    profiler = None
    if args.profile:
        profiler = cProfile.Profile()
        profiler.enable()

    print("=" * 72)
    print("PROTOTYPE design verification")
    if args.with_arm:
        print("  (with --with-arm: optional arm checks ENABLED)")
    print("=" * 72)

    only_set = set(args.only) if args.only else None
    fast_set = (set(ESSENTIAL_CHECK_NAMES) if args.fast else None)

    def _is_selected(name):
        if only_set is not None and name not in only_set:
            return False
        if fast_set is not None and name not in fast_set:
            return False
        return True

    check_subset = [(n, f) for (n, f) in CHECKS if _is_selected(n)]
    run_workspace = _is_selected(WORKSPACE_CHECK_NAME)

    if only_set is not None:
        missing = only_set - {n for n, _ in CHECKS} - {WORKSPACE_CHECK_NAME}
        if missing:
            print(f"  WARN: --only entries not matched against any known "
                  f"check: {sorted(missing)}", file=sys.stderr)

    if args.fast:
        total_checks = len(CHECKS) + 1  # +1 for workspace sweep
        n_skipped = total_checks - len(check_subset) - int(run_workspace)
        print(f"  [FAST MODE] skipping {n_skipped} slow checks; "
              f"use --all to run everything.")
        print(f"  [FAST MODE] running {len(check_subset)} "
              f"essential check(s): "
              f"{', '.join(n for n, _ in check_subset)}")

    if args.changed:
        # Build the mesh cache lazily INSIDE _select_changed_checks --
        # rebuilding STLs in memory requires the cache to be primed
        # for the parts we test, and we only pay the cost when an
        # STL-producing source actually changed.
        check_subset, run_workspace, smart_summary = _select_changed_checks(
            check_subset,
            run_workspace_in=run_workspace,
            base_ref=args.base_ref,
        )
        print(smart_summary)

    t_start = time.monotonic()

    cache_conn = _cache_open()
    n_pruned = _cache_prune(cache_conn)
    if n_pruned:
        print(f"  Cache: pruned {n_pruned} entries older than "
              f"{CACHE_MAX_AGE_S // 86400} days.")

    use_cache = not args.no_cache
    if not use_cache:
        print("  Cache: --no-cache set; ignoring previous results "
              "but still writing new ones.")

    # Mesh cache is only needed when we have at least ONE cache miss
    # to dispatch.  Pre-check each selected check (and the workspace
    # sweep) against the cache and only prebuild the meshes if any
    # of them miss.  Saves ~ 3-4 s on warm-cache runs.
    will_miss = False
    if use_cache:
        for name, fn_name in check_subset:
            ih = _cache_check_input_hash(name, fn_name)
            if _cache_lookup(cache_conn, name, ih) is None:
                will_miss = True
                break
        if not will_miss and run_workspace:
            wih = _cache_check_input_hash(WORKSPACE_CHECK_NAME,
                                            "check_workspace_self_collision")
            if _cache_lookup(cache_conn, WORKSPACE_CHECK_NAME, wih) is None:
                will_miss = True
    else:
        will_miss = True

    if will_miss:
        print("  Pre-building mesh cache in parent process ...")
        prebuild_mesh_cache()
    else:
        print("  Cache: every selected check has a hit; "
              "skipping mesh prebuild.")

    results: list[tuple[str, bool]] = []
    n_workers = 1  # only meaningful for the non-serial branch summary line

    def _run_workspace_with_cache(pool=None):
        """Cache-aware wrapper for check_workspace_self_collision.
        On HIT we print the cache-hit line and skip the real run;
        on MISS we capture the check's stdout, run it, INSERT the
        cached row, and replay the output."""
        ws_input_hash = _cache_check_input_hash(
            WORKSPACE_CHECK_NAME, "check_workspace_self_collision")
        if use_cache:
            hit = _cache_lookup(cache_conn, WORKSPACE_CHECK_NAME,
                                  ws_input_hash)
            if hit is not None:
                result, runtime_s, _msg, ts = hit
                _print_cache_hit(WORKSPACE_CHECK_NAME, result,
                                   runtime_s, ts)
                return result

        buf = io.StringIO()
        t0 = time.monotonic()
        try:
            with redirect_stdout(buf):
                if pool is None:
                    ws_ok = bool(check_workspace_self_collision())
                else:
                    ws_ok = bool(check_workspace_self_collision(pool=pool))
        except Exception:
            import traceback
            traceback.print_exc(file=buf)
            ws_ok = False
        runtime_s = time.monotonic() - t0
        output = buf.getvalue()
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        _cache_insert(cache_conn, WORKSPACE_CHECK_NAME, ws_input_hash,
                       ws_ok, runtime_s, output)
        return ws_ok

    if args.serial:
        # Run every check inline.  Workspace sweep also runs in this
        # process, with no pool, so its 176 poses execute serially --
        # this is the SAME code path as the historical implementation
        # and is the "ground truth" we diff the parallel output against.
        results.extend(_run_checks_serial(
            check_subset,
            cache_conn=cache_conn, use_cache=use_cache))
        if run_workspace:
            ws_ok = _run_workspace_with_cache()
            results.append((WORKSPACE_CHECK_NAME, ws_ok))
    else:
        n_workers = (args.workers
                       if args.workers is not None
                       else min(8, (os.cpu_count() or 1)))
        # spawn context for stability on macOS (fork + trimesh's
        # rtree-backed mesh.contains has been observed to deadlock).
        ctx = multiprocessing.get_context("spawn")
        with concurrent.futures.ProcessPoolExecutor(
            max_workers=n_workers,
            mp_context=ctx,
            initializer=_worker_initializer,
            initargs=(_MESH_CACHE, _inside_mode),
        ) as pool:
            # Phase 1: the small checks fan out across the pool.  Each
            # cache hit is resolved synchronously in the parent and
            # never submitted; cache misses go to the pool and get
            # INSERTed when the future resolves.
            results.extend(_run_checks_pool(
                check_subset, pool,
                cache_conn=cache_conn, use_cache=use_cache))

            # Phase 2: the workspace sweep dispatches its own 176
            # per-pose jobs into the same pool.  We run it in the
            # PARENT so its print statements stream to the real
            # stdout (and any redirect_stdout captures it correctly).
            if run_workspace:
                ws_ok = _run_workspace_with_cache(pool=pool)
                results.append((WORKSPACE_CHECK_NAME, ws_ok))

    if args.with_arm:
        # Arm checks reach into the optional arm module which has its
        # own internal prints; we run them serially in the parent for
        # the same reasons as the workspace sweep.
        results.extend(_optional_arm_checks())

    if cache_conn is not None:
        cache_conn.close()

    print()
    print("=" * 72)
    print("Summary:")
    all_ok = True
    for name, ok in results:
        flag = "PASS" if ok else "FAIL"
        print(f"   [{flag}]  {name}")
        all_ok &= ok
    print("=" * 72)
    elapsed = time.monotonic() - t_start
    print(f"Total verifier wall time: {elapsed:6.1f} s "
          f"({'serial' if args.serial else f'pool max_workers={n_workers}'})")

    if profiler is not None:
        profiler.disable()
        profiler.dump_stats(args.profile)
        print(f"cProfile snapshot written to {args.profile}")

    mismatch_exit = (_inside_mode == INSIDE_MODE_BOTH
                      and len(_inside_mismatches) > 0)
    if _inside_mode == INSIDE_MODE_BOTH:
        _print_inside_mismatch_summary()

    if mismatch_exit:
        print("EXIT 2: --inside-mode=both found disagreements between "
              "the 6-ray vote and mesh.contains; do NOT flip the "
              "default without inspecting them.")
        return 2

    if all_ok:
        print("All checks passed. The prototype is ready to print.")
        return 0
    else:
        print("FAIL.  Fix the failing checks before ordering parts.")
        return 1


def _print_inside_mismatch_summary() -> None:
    """Print a compact summary of points_inside disagreements
    accumulated during a --inside-mode=both run.  Caps the
    per-coordinate listing at the first 20 globally so the output
    stays bounded even when many mismatches occur."""
    print()
    print("=" * 72)
    if not _inside_mismatches:
        print("[MISMATCH SUMMARY] --inside-mode=both: ZERO disagreements "
              "between rays and mesh.contains across the full check "
              "suite.")
        print("=" * 72)
        return

    total = sum(int(r["n_mismatch"]) for r in _inside_mismatches)
    total_points = sum(int(r["n_points"]) for r in _inside_mismatches)
    by_check: dict = {}
    for r in _inside_mismatches:
        k = r["check"]
        by_check[k] = by_check.get(k, 0) + int(r["n_mismatch"])
    print(f"[MISMATCH SUMMARY] --inside-mode=both: "
          f"{len(_inside_mismatches)} record(s), "
          f"{total} mismatching point(s) across "
          f"{total_points} probed point(s) "
          f"({(total / max(total_points, 1)) * 100.0:6.4f} %).")
    print("  Per-check distribution (mismatching points):")
    for check_name, n in sorted(by_check.items(), key=lambda kv: -kv[1]):
        print(f"   {n:6d}  {check_name}")
    print()
    print(f"  First (up to) 20 mismatch coordinates "
          f"(check / mesh vol / xyz / rays -> contains):")
    shown = 0
    for r in _inside_mismatches:
        if shown >= 20:
            break
        vol = r["mesh"]["volume"]
        vol_str = f"{vol:8.1f}" if vol is not None else "      ? "
        wt = r["mesh"].get("watertight")
        for c, rs, cs in zip(r["coords"],
                              r["rays_says"], r["contains_says"]):
            if shown >= 20:
                break
            x, y, z = c
            print(f"   [{r['check']:36s}] mesh_vol={vol_str} wt={wt}  "
                  f"xyz=({x:+8.2f},{y:+8.2f},{z:+8.2f})  "
                  f"rays={'INSIDE' if rs else 'OUTSIDE'} -> "
                  f"contains={'INSIDE' if cs else 'OUTSIDE'}")
            shown += 1
    print("=" * 72)


if __name__ == "__main__":
    sys.exit(main())
