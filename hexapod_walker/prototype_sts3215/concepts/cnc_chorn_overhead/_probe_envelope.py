"""Free-swing envelope probe for the cnc_chorn_overhead concept.

Answers, from the REAL rigid_hip meshes (not slab algebra):

1. The free-angle map Theta(r, t): for a material point rigidly attached
   to the femur at radius ``r`` from the hip axis, axial station ``t``
   (link-local z, the joint axis coordinate) and zero angular offset,
   the first femur UP angle at which its swept circle enters any static
   part (top plate, hatch, hip cap + bearing stack, coxa, hip servo).
   A point with angular offset ``phi`` (up-side positive) contacts at
   ``Theta(r_pt, t) - phi``.

2. Per-part predicted first contact for the femur-frame production
   parts (femur_link, knee servo, knee cap) using that map on their
   actual vertices -- this is the physics ceiling the CNC clamp design
   must not be blamed for, and tells us which corners of OUR printed
   femur body would pay to be chamfered.

3. Knee-side anatomy of femur_link in link-local coords (wall faces,
   well gap, far-wall pad, cradle top heights) -- the receiving
   geometry for the clamp web joint.

Run:  uv run python concepts/cnc_chorn_overhead/_probe_envelope.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

HERE = os.path.abspath(os.path.dirname(__file__))
PROTO_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
RIGID_DIR = os.path.join(PROTO_DIR, "concepts", "rigid_hip")
sys.path.insert(0, PROTO_DIR)
sys.path.insert(0, RIGID_DIR)

import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402


def _load(name: str) -> trimesh.Trimesh:
    m = trimesh.load(os.path.join(RIGID_DIR, "stl", name), process=True)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(
            [g for g in m.geometry.values() if len(g.faces) > 0])
    return m


def main() -> None:
    T = rv.leg_transforms(0)
    statics = [
        ("chassis_top_rigid", _load("chassis_top_rigid.stl"), np.eye(4)),
        ("top_hatch_rigid", _load("top_hatch_rigid.stl"), np.eye(4)),
        ("hip_clamp_cap_rigid", _load("hip_clamp_cap_rigid.stl"),
         T["hip_cap"]),
        ("bearing_6805", _load("bearing_6805_DO_NOT_PRINT.stl"),
         T["hip_cap"]),
        ("hip_servo", _load("servo_body_DO_NOT_PRINT.stl"), T["hip_cap"]),
        ("coxa_link", _load("coxa_link_rigid.stl"), T["coxa"]),
    ]
    placed = []
    for name, m, M in statics:
        mm = m.copy()
        mm.apply_transform(M)
        placed.append((name, mm))

    # ---- probe grid in link coords: axis at x=12.5, axial along z ----
    r_grid = np.arange(14.0, 72.0, 1.0)
    t_grid = np.arange(-32.0, 54.0, 2.0)
    th_grid = np.arange(30.0, 141.0, 1.0)

    # Link->world transform per femur angle p (up = negative p): the map
    # samples every (r,t) circle point once, batched per static mesh.
    MH = rv.MH
    hip_local = np.array(rv.COXA_HIP_ANCHOR_V)
    pts_link = np.array([[12.5 + r, 0.0, t, 1.0]
                         for r in r_grid for t in t_grid])
    n_pts = len(pts_link)
    all_world = np.empty((len(th_grid), n_pts, 3))
    for k, th in enumerate(th_grid):
        M = (T["coxa"] @ rv._trans(hip_local)
             @ rotation_matrix(np.deg2rad(-th), [0, 1, 0]) @ MH)
        all_world[k] = (pts_link @ M.T)[:, :3]
    flat = all_world.reshape(-1, 3)

    contained = np.zeros(len(flat), dtype=bool)
    for name, mm in placed:
        lo, hi = mm.bounds
        sel = np.all((flat > lo - 0.5) & (flat < hi + 0.5), axis=1)
        sel &= ~contained
        if sel.any():
            contained[np.where(sel)[0][mm.contains(flat[sel])]] = True
        print(f"  static {name:22s} newly-hit "
              f"{int(contained.sum())} total", flush=True)
    hit = contained.reshape(len(th_grid), n_pts)
    theta = np.full(n_pts, np.inf)
    for k in range(len(th_grid) - 1, -1, -1):
        theta[hit[k]] = th_grid[k]
    theta_map = theta.reshape(len(r_grid), len(t_grid))

    np.savez(os.path.join(HERE, "_envelope_map.npz"),
             r=r_grid, t=t_grid, theta=theta_map)

    # compact text map: rows r, cols t (subsampled)
    print("\nTheta(r,t) free-angle map (deg, inf -> '...'):  cols = "
          "station t (link z)")
    t_show = t_grid[::4]
    print("     r\\t " + " ".join(f"{t:5.0f}" for t in t_show))
    for i, r in enumerate(r_grid):
        if r % 2 != 0:
            continue
        row = theta_map[i, ::4]
        cells = " ".join("  ..." if not np.isfinite(v) else f"{v:5.0f}"
                         for v in row)
        print(f"    {r:5.1f} {cells}")

    # ---- per-part predicted contact from vertices --------------------
    def predict(name: str, mesh: trimesh.Trimesh, M_link: np.ndarray):
        v = trimesh.transform_points(mesh.vertices, M_link)
        r = np.hypot(v[:, 0] - 12.5, v[:, 1])
        phi = np.degrees(np.arctan2(v[:, 1], v[:, 0] - 12.5))
        ri = np.clip(np.searchsorted(r_grid, r), 0, len(r_grid) - 1)
        ti = np.clip(np.searchsorted(t_grid, v[:, 2]), 0, len(t_grid) - 1)
        th = theta_map[ri, ti] - phi
        j = int(np.argmin(th))
        print(f"  {name:18s} predicted first contact {th[j]:7.1f} deg  "
              f"at link ({v[j, 0]:6.1f},{v[j, 1]:6.1f},{v[j, 2]:6.1f})  "
              f"r={r[j]:5.1f} phi={phi[j]:+6.1f}")
        return float(th[j])

    print("\nPredicted per-part first contact (production femur parts):")
    femur = _load("femur_link.stl")
    knee_cap = _load("knee_clamp_cap.stl")
    servo = _load("servo_body_DO_NOT_PRINT.stl")
    M_KNEE_IN_LINK = np.linalg.inv(MH) @ rv.M_KNEE_JP
    predict("femur_link", femur, np.eye(4))
    predict("knee_servo", servo, M_KNEE_IN_LINK)
    predict("knee_cap", knee_cap, M_KNEE_IN_LINK)

    # ---- knee-side anatomy of femur_link ------------------------------
    print("\nfemur_link anatomy (link coords; hip axis x=12.5, knee axis "
          "x=102.5):")
    v = femur.vertices
    for x0 in np.arange(40.0, 135.0, 5.0):
        band = v[(v[:, 0] >= x0) & (v[:, 0] < x0 + 5.0)]
        if len(band) == 0:
            print(f"    x {x0:5.1f}..{x0 + 5:5.1f}: (no vertices)")
            continue
        print(f"    x {x0:5.1f}..{x0 + 5:5.1f}: y [{band[:, 1].min():6.1f},"
              f"{band[:, 1].max():6.1f}]  z [{band[:, 2].min():6.1f},"
              f"{band[:, 2].max():6.1f}]  n={len(band)}")

    # solid/air probe along the spar line and the well gap
    line = np.array([[x, 0.0, hp.JOINT_SOCKET_Z] for x in
                     np.arange(40.0, 120.0, 1.0)])
    inside = femur.contains(line)
    runs, cur = [], None
    for x, s in zip(line[:, 0], inside):
        if s and cur is None:
            cur = x
        elif not s and cur is not None:
            runs.append((cur, x))
            cur = None
    if cur is not None:
        runs.append((cur, line[-1, 0]))
    print(f"    solid runs along (x, 0, {hp.JOINT_SOCKET_Z:.2f}): "
          + ", ".join(f"{a:.0f}..{b:.0f}" for a, b in runs))

    # knee cap tongue positions in link coords
    vc = trimesh.transform_points(knee_cap.vertices, M_KNEE_IN_LINK)
    print(f"    knee cap extents: x [{vc[:, 0].min():.1f},"
          f"{vc[:, 0].max():.1f}]  y [{vc[:, 1].min():.1f},"
          f"{vc[:, 1].max():.1f}]  z [{vc[:, 2].min():.1f},"
          f"{vc[:, 2].max():.1f}]")
    vs = trimesh.transform_points(servo.vertices, M_KNEE_IN_LINK)
    print(f"    knee servo extents: x [{vs[:, 0].min():.1f},"
          f"{vs[:, 0].max():.1f}]  y [{vs[:, 1].min():.1f},"
          f"{vs[:, 1].max():.1f}]  z [{vs[:, 2].min():.1f},"
          f"{vs[:, 2].max():.1f}]")


if __name__ == "__main__":
    main()
