"""Generate per-face acrylic panel outlines for the polyhedron chandelier.

For every face of every solid in :mod:`polyhedra`, this script writes a
2-D polygon describing the cut outline of the captured acrylic panel
that goes into that face.  Outputs are dumped as a single ``.dxf``
(suitable for laser-cutting) and a parallel ``panels.json`` for tooling
use.

Conventions match :mod:`all_polyhedra`:

* Pre-scale unit = 1 mm (the same canonical size used by the STL build).
* All output coordinates are multiplied by ``SCALE_FACTOR`` to land in
  the post-cast millimetre space the foundry parts live in.
* The panel for each face is the face polygon, offset inward by
  ``PANEL_INSET_POST`` (= slot floor depth + perimeter slack), and
  defined in the face's own 2-D coordinate frame.

The lowest face of each polyhedron in hanging orientation is left out
(matches the "open access face" convention in the STL build).

Usage
-----

    python make_panel_outlines.py                     # writes panels.dxf, panels.json
    python make_panel_outlines.py --out-dir ./panels  # alternative output dir
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np
import trimesh

import polyhedra as P
from all_polyhedra import (
    EDGE_RADIUS,
    PANEL_SLOT_DEPTH,
    NODE_DIAMETER_MM,
)

# Final chandelier scale factor: matches the value reported by
# `all_polyhedra.py` ("Scale applied: 3.509x").  It is determined by the
# chandelier's overall horizontal extent / target outer diameter, so it
# is fixed once the geometry script's TARGET_OUTER_DIAMETER_MM and
# layout constants are set.  If you change those, regenerate the STL
# and copy the new "Scale applied" number here.
SCALE_FACTOR = 3.598

# Panel sits on each slot's floor (inward by edge_radius - panel_slot_depth)
# plus a small perimeter slack for thermal expansion.
_PANEL_INSET_PRE = (EDGE_RADIUS - PANEL_SLOT_DEPTH) + 0.14   # ≈ 0.55 + 0.14 mm pre
PANEL_INSET_POST = _PANEL_INSET_PRE * SCALE_FACTOR           # ≈ 2.43 mm post


def _orient_world_verts(mesh):
    """Reproduce the same vertex orientation used by all_polyhedra.make_wire_solid:
    centre the mesh, rotate so the highest-radius vertex points to +Y, scale
    to the standard pre-scale node diameter (1.0)."""
    verts = np.asarray(mesh.vertices) - mesh.vertices.mean(axis=0)
    radii = np.linalg.norm(verts, axis=1)
    if radii.max() < 1e-9:
        return verts
    top_idx = int(np.argmax(verts[:, 1]))
    R = trimesh.geometry.align_vectors(
        verts[top_idx] / np.linalg.norm(verts[top_idx]),
        np.array([0.0, 1.0, 0.0]),
    )[:3, :3]
    verts = verts @ R.T
    # Normalise to unit "pre-scale" diameter so panel coords are in the
    # same units as all_polyhedra.py uses (mm pre-scale).  Then we apply
    # SCALE_FACTOR once at the end of the panel polygon construction.
    # Match all_polyhedra.make_wire_solid: each polyhedron is built at
    # NODE_DIAMETER_MM (= 36 mm pre-scale).  We apply SCALE_FACTOR at the
    # very end to land in post-cast millimetre space.
    radii = np.linalg.norm(verts, axis=1)
    if radii.max() > 1e-9:
        verts = verts * (NODE_DIAMETER_MM / 2.0 / radii.max())
    return verts


def _face_centroid(world_verts, face_vert_indices):
    return world_verts[face_vert_indices].mean(axis=0)


def _enumerate_faces(mesh, world_verts):
    """Yield ``(face_idx, vert_indices, centroid, ordered_boundary)`` for every
    polygon face of ``mesh``, using ``world_verts`` as transformed vertex
    coordinates.  ``ordered_boundary`` is a Numpy array of vertex indices
    in CCW order around the face."""
    faceted_tris = set()
    polygon_faces = []  # list of (vert_indices_unique, ordered_boundary_indices)

    for facet, boundary_edges in zip(mesh.facets, mesh.facets_boundary):
        for tri_idx in facet:
            faceted_tris.add(int(tri_idx))
        unique_verts = np.unique(mesh.faces[np.asarray(facet)].flatten())
        ordered = _order_boundary(boundary_edges)
        polygon_faces.append((unique_verts, ordered))

    for tri_idx in range(len(mesh.faces)):
        if tri_idx in faceted_tris:
            continue
        tri = mesh.faces[tri_idx]
        polygon_faces.append((tri, np.asarray(tri)))

    for fi, (unique_verts, ordered) in enumerate(polygon_faces):
        centroid = world_verts[unique_verts].mean(axis=0)
        yield fi, unique_verts, centroid, ordered


def _order_boundary(boundary_edges):
    """Walk a list of (u, v) edges into a single closed loop of vertex
    indices (in some consistent winding)."""
    edges = [(int(a), int(b)) for a, b in boundary_edges]
    if not edges:
        return np.empty(0, dtype=int)
    # Build adjacency map
    nbrs = {}
    for a, b in edges:
        nbrs.setdefault(a, []).append(b)
        nbrs.setdefault(b, []).append(a)
    start = edges[0][0]
    loop = [start]
    prev = None
    cur = start
    while True:
        next_choices = [v for v in nbrs[cur] if v != prev]
        if not next_choices:
            break
        nxt = next_choices[0]
        if nxt == start:
            break
        loop.append(nxt)
        prev = cur
        cur = nxt
        if len(loop) > len(edges) + 1:
            break
    return np.asarray(loop, dtype=int)


def _project_to_face_plane(face_verts_3d, centroid_3d, face_normal):
    """Return 2-D coordinates of ``face_verts_3d`` in the face's local
    plane.  Uses an arbitrary orthonormal basis perpendicular to
    ``face_normal``."""
    n = face_normal / max(np.linalg.norm(face_normal), 1e-9)
    # Pick u perpendicular to n
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = helper - np.dot(helper, n) * n
    u /= max(np.linalg.norm(u), 1e-9)
    v = np.cross(n, u)
    rel = face_verts_3d - centroid_3d
    coords = np.column_stack([rel @ u, rel @ v])
    return coords


def _polygon_inset(coords_2d, inset):
    """Inset a CCW or CW 2-D polygon by ``inset`` (positive = inward).

    Robust enough for convex faces (which all the Platonic, Archimedean
    and Catalan faces are by construction)."""
    n = len(coords_2d)
    if n < 3:
        return coords_2d
    # Detect winding
    signed_area = 0.0
    for i in range(n):
        x1, y1 = coords_2d[i]
        x2, y2 = coords_2d[(i + 1) % n]
        signed_area += (x2 - x1) * (y2 + y1)
    ccw = signed_area < 0  # standard "shoelace" — negative => CCW

    inset_lines = []
    for i in range(n):
        a = coords_2d[i]
        b = coords_2d[(i + 1) % n]
        edge = b - a
        edge_len = np.linalg.norm(edge)
        if edge_len < 1e-9:
            inset_lines.append((a, b))
            continue
        edge_dir = edge / edge_len
        # Inward normal: rotate edge_dir by -90° (CCW polygon) or +90° (CW)
        if ccw:
            normal = np.array([-edge_dir[1], edge_dir[0]])
        else:
            normal = np.array([edge_dir[1], -edge_dir[0]])
        a_off = a + normal * inset
        b_off = b + normal * inset
        inset_lines.append((a_off, b_off))

    # Intersect successive offset lines to get inset polygon vertices
    out = []
    for i in range(n):
        p1, p2 = inset_lines[(i - 1) % n]
        p3, p4 = inset_lines[i]
        d1 = p2 - p1
        d2 = p4 - p3
        denom = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(denom) < 1e-9:
            out.append(p3)
            continue
        t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / denom
        out.append(p1 + t * d1)
    return np.asarray(out)


def _write_dxf(path, panels):
    """Write a minimal DXF with one LWPOLYLINE per panel.  Coordinates
    are post-scale millimetres.  Panels are tiled in a loose grid so a
    laser-cut shop can pick them up directly."""
    # Tile panels into a rough grid: 200 mm pitch
    pitch = 200.0
    cols = 8

    lines = ["0", "SECTION", "2", "ENTITIES"]
    for idx, panel in enumerate(panels):
        col = idx % cols
        row = idx // cols
        dx = col * pitch
        dy = -row * pitch  # negative so panels go downward in DXF
        coords = panel["coords_2d"]
        lines += ["0", "LWPOLYLINE",
                  "8", f"PANEL_{panel['solid']}_{panel['face_idx']}",
                  "90", str(len(coords)),
                  "70", "1"]  # 1 = closed polyline
        for x, y in coords:
            lines += ["10", f"{x + dx:.4f}", "20", f"{y + dy:.4f}"]
        # Add a panel ID text at the polygon centroid
        cx = float(np.mean([c[0] for c in coords]))
        cy = float(np.mean([c[1] for c in coords]))
        lines += ["0", "TEXT",
                  "8", f"PANEL_LABEL",
                  "10", f"{cx + dx:.4f}",
                  "20", f"{cy + dy:.4f}",
                  "40", "5",
                  "1", panel["label"]]
    lines += ["0", "ENDSEC", "0", "EOF"]

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=".",
                        help="Directory to write panels.dxf and panels.json into.")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    panels = []
    for solid in P.ALL_SOLIDS:
        mesh = P.generate_mesh(solid)
        world_verts = _orient_world_verts(mesh)

        # Enumerate faces and find the lowest-centroid one (open access)
        all_face_data = list(_enumerate_faces(mesh, world_verts))
        bottom_idx = int(np.argmin([fc[2][1] for fc in all_face_data]))

        for fi, unique_verts, centroid, ordered in all_face_data:
            if fi == bottom_idx:
                continue  # open access face — no panel
            ordered_3d = world_verts[ordered]
            # Use the average outward direction of vertex offsets from centroid
            # (works for convex faces) as the face normal — sign chosen so it
            # points away from the polyhedron centre
            face_centre_to_world = centroid - world_verts.mean(axis=0)
            face_normal = face_centre_to_world / max(np.linalg.norm(face_centre_to_world), 1e-9)
            coords_2d = _project_to_face_plane(ordered_3d, centroid, face_normal)
            inset_2d = _polygon_inset(coords_2d, PANEL_INSET_POST / SCALE_FACTOR)
            inset_2d_post = inset_2d * SCALE_FACTOR

            label = f"{solid.name[:4].upper()}-{fi:02d}"
            panels.append({
                "solid": solid.name.replace(" ", "_"),
                "face_idx": fi,
                "label": label,
                "n_sides": int(len(inset_2d_post)),
                "coords_2d": inset_2d_post.tolist(),
            })

    json_path = os.path.join(args.out_dir, "panels.json")
    dxf_path = os.path.join(args.out_dir, "panels.dxf")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({
            "scale_factor": SCALE_FACTOR,
            "panel_inset_post_mm": PANEL_INSET_POST,
            "panel_count": len(panels),
            "panels": panels,
        }, f, indent=2)
    _write_dxf(dxf_path, panels)

    sides_hist = {}
    for p in panels:
        sides_hist[p["n_sides"]] = sides_hist.get(p["n_sides"], 0) + 1
    print(f"Wrote {len(panels)} panel outlines:")
    for n in sorted(sides_hist):
        print(f"  {sides_hist[n]:5d} × {n}-gon")
    print(f"Output: {dxf_path}, {json_path}")


if __name__ == "__main__":
    main()
