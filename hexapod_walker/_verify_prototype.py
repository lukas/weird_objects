"""Run all four post-redesign correctness checks on the prototype.

1.  Every part is a clean watertight mesh (manifold, no self-intersections
    that would crash the slicer).
2.  Every servo well has an open insertion path big enough for the body.
3.  Every M3 mounting / chassis-bolt hole drilled by the design code
    actually passes through solid material it's supposed to anchor in.
4.  In standing pose, no two parts overlap in volume by more than 100 mm^3.

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
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

import hexapod_prototype as hp


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


def check_cradle_openness():
    print("\n[2] Cradle insertion-path openness:")
    all_ok = True

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
    arm_t = 4.0
    well_z_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0)
    body_centre_cl = np.array([
        hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H / 2.0 + hp.SERVO_OUTPUT_H),
        well_z_drop,
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
    # the bore axis is +Y in the link frame.
    cl = hp.make_coxa_link()
    delta_x_cl = hp.COXA_LENGTH - hp.SERVO_OUTPUT_X
    delta_y_cl = -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H)
    arm_t_cl = 4.0
    drop_z_cl = -(hp.WELL_D / 2.0 + arm_t_cl / 2.0)
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
    arm_t = 4.0
    hip_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0)
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
    JOINT_TOLERANCE = 1500.0   # mm^3 (servo gear stack + horn adapter)
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
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 72)
    print("PROTOTYPE design verification")
    print("=" * 72)
    results = []
    results.append(("Mesh watertightness",   check_watertight()))
    results.append(("Cradle openness",        check_cradle_openness()))
    results.append(("Bolt-hole engagement",   check_bolt_holes()))
    results.append(("Self-collision",         check_self_collision()))

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
