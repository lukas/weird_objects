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
    PLATONIC_ORDER,
    ARCHIMEDEAN_ORDER,
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


def _safe_filename(name: str) -> str:
    return (name.lower()
                .replace(" ", "_")
                .replace("/", "_")
                .replace("(", "")
                .replace(")", ""))


def _split_index_for_filename(solid) -> int:
    if solid.category == "platonic":
        return PLATONIC_ORDER.index(solid.name) + 1
    if solid.category == "archimedean":
        return ARCHIMEDEAN_ORDER.index(solid.name) + 1
    if solid.category == "catalan":
        for i, arch_name in enumerate(ARCHIMEDEAN_ORDER):
            cat = next((s for s in P.CATALAN if s.parent == arch_name), None)
            if cat is not None and cat.name == solid.name:
                return i + 1
    return 0


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


def _panel_pack_variants(panel):
    """Return normalized 0° and 90° coordinate variants for one panel."""
    coords = np.asarray(panel["coords_2d"], dtype=float)
    variants = []
    for candidate in (coords, np.column_stack([-coords[:, 1], coords[:, 0]])):
        min_xy = candidate.min(axis=0)
        normalized = candidate - min_xy
        max_xy = normalized.max(axis=0)
        variants.append((normalized, float(max_xy[0]), float(max_xy[1])))
    return variants


def _pack_shelf(items, max_width, spacing):
    """Pack normalized panel variants into shelves for one candidate width."""
    placed = []
    x_cursor = 0.0
    y_cursor = 0.0
    row_height = 0.0
    used_width = 0.0

    for panel, variants in items:
        fitting = []
        for coords, width, height in variants:
            if x_cursor == 0.0 or x_cursor + width <= max_width:
                fitting.append((max(row_height, height), width, height, coords))

        if not fitting:
            x_cursor = 0.0
            y_cursor -= row_height + spacing
            row_height = 0.0
            fitting = [(height, width, height, coords)
                       for coords, width, height in variants
                       if width <= max_width]

        if not fitting:
            return None

        _, width, height, coords = min(fitting, key=lambda v: (v[0], v[1]))
        packed_coords = coords + np.array([x_cursor, y_cursor])
        placed.append((panel, packed_coords))

        used_width = max(used_width, x_cursor + width)
        x_cursor += width + spacing
        row_height = max(row_height, height)

    all_coords = np.vstack([coords for _, coords in placed])
    min_xy = all_coords.min(axis=0)
    max_xy = all_coords.max(axis=0)
    used_width, used_height = max_xy - min_xy
    return placed, used_width, used_height


def _packed_panel_positions(panels, *, spacing=3.0):
    """Return ``(panel, packed_coords)`` using the best compact shelf layout.

    The panel coordinates are local to each face, so first normalize every
    polygon to its bounding box, then try several sheet widths and 90-degree
    rotations.  This is not full nesting, but it keeps quote bounding boxes
    realistic for sheet-cutting services.
    """
    items = [(panel, _panel_pack_variants(panel)) for panel in panels]
    items.sort(key=lambda item: max(v[1] * v[2] for v in item[1]),
               reverse=True)

    largest_width = max(max(v[1] for v in variants) for _, variants in items)
    total_area = sum(max(v[1] * v[2] for v in variants) for _, variants in items)
    target = max(largest_width, np.sqrt(total_area) * 1.35)
    max_candidate_width = max(600.0, target * 1.6, largest_width)
    candidate_widths = np.arange(largest_width, max_candidate_width + 10.0, 10.0)

    best = None
    for width in candidate_widths:
        result = _pack_shelf(items, width, spacing)
        if result is None:
            continue
        placed, used_width, used_height = result
        aspect = max(used_width / max(used_height, 1e-9),
                     used_height / max(used_width, 1e-9))
        aspect_penalty = 1.0 + max(0.0, aspect - 2.0) * 0.35
        score = used_width * used_height * aspect_penalty
        if best is None or score < best[0]:
            best = (score, placed)

    return best[1]


def _write_dxf(path, panels, *, dxf_units="mm", include_labels=True):
    """Write a minimal DXF with one LWPOLYLINE per panel.  Coordinates
    are post-scale millimetres.  Panels are tightly shelf-packed so a
    laser-cut shop sees a realistic sheet-cutting envelope."""
    if dxf_units == "in":
        coord_scale = 1.0 / 25.4
        insunits = "1"      # inches
        measurement = "0"   # English
    else:
        coord_scale = 1.0
        insunits = "4"      # millimetres
        measurement = "1"   # metric

    lines = [
        "0", "SECTION", "2", "HEADER",
        "9", "$INSUNITS", "70", insunits,
        "9", "$MEASUREMENT", "70", measurement,
        "0", "ENDSEC",
        "0", "SECTION", "2", "ENTITIES",
    ]
    for panel, coords in _packed_panel_positions(panels):
        coords = coords * coord_scale
        lines += ["0", "LWPOLYLINE",
                  "8", f"PANEL_{panel['solid']}_{panel['face_idx']}",
                  "90", str(len(coords)),
                  "70", "1"]  # 1 = closed polyline
        for x, y in coords:
            lines += ["10", f"{x:.4f}", "20", f"{y:.4f}"]
        if include_labels:
            # Add a panel ID text at the polygon centroid
            cx = float(np.mean([c[0] for c in coords]))
            cy = float(np.mean([c[1] for c in coords]))
            lines += ["0", "TEXT",
                      "8", f"PANEL_LABEL",
                      "10", f"{cx:.4f}",
                      "20", f"{cy:.4f}",
                      "40", f"{5.0 * coord_scale:.4f}",
                      "1", panel["label"]]
    lines += ["0", "ENDSEC", "0", "EOF"]

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=".",
                        help="Directory to write panels.dxf and panels.json into.")
    parser.add_argument("--per-solid", action="store_true",
                        help="also write one tightly packed DXF per polyhedron")
    parser.add_argument("--dxf-units", choices=("mm", "in"), default="mm",
                        help="coordinate units to write in the DXF (default: mm)")
    parser.add_argument("--no-labels", action="store_true",
                        help="omit TEXT labels for stricter CAM importers")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    panels = []
    panels_by_solid = []
    for solid in P.ALL_SOLIDS:
        mesh = P.generate_mesh(solid)
        world_verts = _orient_world_verts(mesh)
        solid_panels = []

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
            panel = {
                "solid": solid.name.replace(" ", "_"),
                "face_idx": fi,
                "label": label,
                "n_sides": int(len(inset_2d_post)),
                "coords_2d": inset_2d_post.tolist(),
            }
            panels.append(panel)
            solid_panels.append(panel)
        panels_by_solid.append((solid, solid_panels))

    json_path = os.path.join(args.out_dir, "panels.json")
    dxf_path = os.path.join(args.out_dir, "panels.dxf")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({
            "scale_factor": SCALE_FACTOR,
            "panel_inset_post_mm": PANEL_INSET_POST,
            "panel_count": len(panels),
            "panels": panels,
        }, f, indent=2)
    _write_dxf(dxf_path, panels,
               dxf_units=args.dxf_units,
               include_labels=not args.no_labels)

    if args.per_solid:
        for solid, solid_panels in panels_by_solid:
            idx = _split_index_for_filename(solid)
            fname = f"{solid.category}_{idx:02d}_{_safe_filename(solid.name)}.dxf"
            _write_dxf(os.path.join(args.out_dir, fname), solid_panels,
                       dxf_units=args.dxf_units,
                       include_labels=not args.no_labels)

    sides_hist = {}
    for p in panels:
        sides_hist[p["n_sides"]] = sides_hist.get(p["n_sides"], 0) + 1
    print(f"Wrote {len(panels)} panel outlines:")
    for n in sorted(sides_hist):
        print(f"  {sides_hist[n]:5d} × {n}-gon")
    print(f"Output: {dxf_path}, {json_path}")
    if args.per_solid:
        print(f"Per-polyhedron DXFs: {args.out_dir}/*.dxf")


if __name__ == "__main__":
    main()
