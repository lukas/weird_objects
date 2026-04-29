"""Interactive simulation of the polyhedron-constellation chandelier
with LEDs glowing through translucent acrylic panels, hung in a dark
room so you can see the light patterns it casts on the walls.

Loads the metal frame from `all_polyhedra_31.stl`, builds 3-D extruded
acrylic panels for every face of every polyhedron (matching the shapes
in `panels.dxf`), and surrounds the chandelier with a simple
box-shaped room.  Light from the LEDs falls on the floor, walls, and
ceiling — turn on shadows (`--shadows`) to see the chandelier's
silhouette projected onto everything.

Usage
-----

    # Open the interactive viewer
    ./run.sh simulate_chandelier.py

    # Render a screenshot and exit (off-screen)
    ./run.sh simulate_chandelier.py --screenshot render.png

    # Tweak the look
    ./run.sh simulate_chandelier.py --background "#0a0a14" --panel-opacity 0.18
    ./run.sh simulate_chandelier.py --no-room      # just chandelier on dark
    ./run.sh simulate_chandelier.py --shadows      # cast chandelier shadow on walls

Tweakables (see ``--help``):

  --panel-opacity        How translucent the acrylic looks (0..1)
  --panel-color          Hex color of the panels (e.g. frosted vs amber)
  --metal-color          Hex color of the metal frame (e.g. brass vs aluminum)
  --led-color            LED warm-white tint
  --led-intensity        Per-LED light intensity (0..1+)
  --no-panels            Skip panels (just see the metal frame)
  --no-leds              Skip LED sources (no glow, no light)
  --no-room              Skip the surrounding room (just chandelier in space)
  --room-width           Width / depth of the room in metres (default 5)
  --room-height          Floor-to-ceiling height in metres (default 3.5)
  --shadows              Enable shadow casting (slower but shows projected patterns)
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import pyvista as pv
import trimesh
from shapely.geometry import Polygon

import polyhedra as P
from all_polyhedra import (
    EDGE_RADIUS,
    NODE_DIAMETER_MM,
    PANEL_SLOT_DEPTH,
    _layout_positions,
)


# Final chandelier scale (matches the value reported by all_polyhedra.py
# and used in make_panel_outlines.py).
SCALE_FACTOR = 3.598

# Acrylic panel geometry, all in post-cast millimetres.
PANEL_THICKNESS_POST = 1.6
PANEL_INSET_POST = (EDGE_RADIUS - PANEL_SLOT_DEPTH) * SCALE_FACTOR + 0.5  # ≈ 2.48 mm


# ---------------------------------------------------------------------------
# Polyhedron geometry helpers (mirror the orientation used by make_wire_solid)
# ---------------------------------------------------------------------------


def _orient_verts(mesh):
    """Centre the mesh, rotate so the highest-radius vertex points to +Y,
    and scale to NODE_DIAMETER_MM diameter (pre-scale coords).  This is
    the same transform applied by ``all_polyhedra.make_wire_solid``."""
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
    radius = np.linalg.norm(verts, axis=1).max()
    if radius > 1e-9:
        verts = verts * (NODE_DIAMETER_MM / 2.0 / radius)
    return verts


def _order_boundary(boundary_edges):
    """Walk a list of (u, v) edges into a single closed loop of vertex
    indices.  Polygons are convex by construction so this works on
    every Platonic / Archimedean / Catalan face."""
    edges = [(int(a), int(b)) for a, b in boundary_edges]
    if not edges:
        return np.empty(0, dtype=int)
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


def _enumerate_polygon_faces(mesh, world_verts_post):
    """Yield ``(face_idx, vert_indices_unique, centroid_post, ordered_3d_post)``
    for every polygon face of ``mesh``."""
    faceted_tris = set()
    polygon_faces = []  # list of (unique_verts, ordered_loop)

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
        centroid = world_verts_post[unique_verts].mean(axis=0)
        ordered_3d = world_verts_post[ordered]
        yield fi, unique_verts, centroid, ordered_3d


def _build_panel_polygon(face_3d_verts, face_centroid, face_normal,
                         inset_post=PANEL_INSET_POST,
                         panel_thickness_post=PANEL_THICKNESS_POST):
    """Build a single 3-D extruded acrylic panel mesh for one face.
    Returns ``trimesh.Trimesh`` or ``None`` if the polygon is degenerate."""
    n = face_normal / max(np.linalg.norm(face_normal), 1e-9)
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = helper - np.dot(helper, n) * n
    u = u / max(np.linalg.norm(u), 1e-9)
    v = np.cross(n, u)

    rel = face_3d_verts - face_centroid
    coords_2d = np.column_stack([rel @ u, rel @ v])

    poly = Polygon(coords_2d)
    if not poly.is_valid:
        poly = poly.buffer(0)  # fix self-intersections from winding
        if not poly.is_valid or poly.is_empty or poly.geom_type != "Polygon":
            return None

    insetted = poly.buffer(-inset_post, join_style=2, mitre_limit=10)
    if insetted.is_empty or insetted.geom_type != "Polygon":
        return None

    try:
        panel = trimesh.creation.extrude_polygon(insetted, height=panel_thickness_post)
    except Exception:
        return None
    panel.apply_translation([0, 0, -panel_thickness_post / 2])

    R = np.column_stack([u, v, n])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = face_centroid
    panel.apply_transform(T)
    return panel


def build_all_panels():
    """Build every acrylic panel for every polyhedron in the chandelier,
    in post-cast world coordinates.  Returns ``(combined_trimesh,
    led_centers_post)``."""
    panels = []
    led_centers = []

    for solid, center in _layout_positions():
        mesh = P.generate_mesh(solid)
        verts_pre = _orient_verts(mesh)
        center_pre = np.asarray(center, dtype=float)

        # Centre is the polyhedron's geometric centre — also the LED location.
        center_post = center_pre * SCALE_FACTOR
        led_centers.append(center_post)

        world_verts_post = (verts_pre + center_pre) * SCALE_FACTOR

        all_faces = list(_enumerate_polygon_faces(mesh, world_verts_post))
        bottom_idx = int(np.argmin([fc[2][1] for fc in all_faces]))

        for fi, _unique, centroid, ordered_3d in all_faces:
            if fi == bottom_idx:
                continue
            normal = centroid - center_post
            normal = normal / max(np.linalg.norm(normal), 1e-9)
            panel = _build_panel_polygon(ordered_3d, centroid, normal)
            if panel is not None:
                panels.append(panel)

    if not panels:
        return None, led_centers

    combined = trimesh.util.concatenate(panels)
    return combined, led_centers


def _trimesh_to_pv(tm):
    """Convert a trimesh.Trimesh to a pyvista.PolyData with triangular faces."""
    faces = np.column_stack(
        [np.full(len(tm.faces), 3, dtype=np.int64), tm.faces.astype(np.int64)]
    ).flatten()
    return pv.PolyData(np.asarray(tm.vertices), faces)


# ---------------------------------------------------------------------------
# Room geometry — six planes around the chandelier so the LEDs can paint
# their light onto something visible.
# ---------------------------------------------------------------------------


def build_room(chandelier_bounds, room_width_mm, room_height_mm):
    """Return a list of (label, pv.Plane, kwargs) describing the floor,
    ceiling, and four walls of a box room around the chandelier.  The
    chandelier is positioned with its eye loop just below the ceiling."""
    cx = (chandelier_bounds[0] + chandelier_bounds[1]) / 2
    cz = (chandelier_bounds[4] + chandelier_bounds[5]) / 2
    eye_y = chandelier_bounds[3]            # top of the chandelier
    floor_to_eye_clearance = 200.0          # mm — typical clearance from ceiling to fixture top
    ceiling_y = eye_y + floor_to_eye_clearance
    floor_y = ceiling_y - room_height_mm

    half = room_width_mm / 2.0

    # Make the planes finely tessellated so per-vertex lighting actually
    # paints a smooth gradient across them (default Plane is just 2 tris).
    res = 96  # 96 × 96 quads per surface

    def plane(center, direction, i_size, j_size):
        return pv.Plane(center=center, direction=direction,
                        i_size=i_size, j_size=j_size,
                        i_resolution=res, j_resolution=res)

    floor   = plane((cx, floor_y, cz),  (0, 1, 0), room_width_mm, room_width_mm)
    ceiling = plane((cx, ceiling_y, cz), (0, -1, 0), room_width_mm, room_width_mm)
    wall_n  = plane((cx, (floor_y + ceiling_y) / 2, cz + half),
                    (0, 0, -1), room_width_mm, room_height_mm)
    wall_s  = plane((cx, (floor_y + ceiling_y) / 2, cz - half),
                    (0, 0, 1), room_width_mm, room_height_mm)
    wall_e  = plane((cx + half, (floor_y + ceiling_y) / 2, cz),
                    (-1, 0, 0), room_width_mm, room_height_mm)
    wall_w  = plane((cx - half, (floor_y + ceiling_y) / 2, cz),
                    (1, 0, 0), room_width_mm, room_height_mm)

    return [
        ("floor",   floor,   {"color": "#1f1d1c"}),  # dark hardwood
        ("ceiling", ceiling, {"color": "#262626"}),  # dim ceiling
        ("wall_n",  wall_n,  {"color": "#2a2c33"}),  # cool gray
        ("wall_s",  wall_s,  {"color": "#2a2c33"}),
        ("wall_e",  wall_e,  {"color": "#2a2c33"}),
        ("wall_w",  wall_w,  {"color": "#2a2c33"}),
    ]


# ---------------------------------------------------------------------------
# Main viewer
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stl", default="all_polyhedra_31.stl",
                        help="Path to the chandelier STL.")
    parser.add_argument("--screenshot", default=None,
                        help="If set, render off-screen to this PNG and exit "
                             "instead of opening the interactive viewer.")
    parser.add_argument("--export-blender", default=None, metavar="DIR",
                        help="Build panels + LED-position metadata and write to "
                             "DIR/panels.ply + DIR/led_positions.json, then exit. "
                             "Used by render_blender.py for photoreal Cycles renders.")
    parser.add_argument("--background", default="#05060a",
                        help="Background color when no room is shown (default: near-black).")
    parser.add_argument("--panel-opacity", type=float, default=0.55,
                        help="Opacity of acrylic panels (0..1).")
    parser.add_argument("--panel-color", default="#fff0c8",
                        help="Acrylic panel tint (hex; default warm-white frost).")
    parser.add_argument("--panel-glow", type=float, default=0.95,
                        help="Panel self-illumination amount, 0..1 (default 0.95). "
                             "PyVista can't transmit light through panels so we fake "
                             "the LED backlight by making panels emissive.")
    parser.add_argument("--metal-color", default="#a07a45",
                        help="Metal frame color (hex; default antique brass).")
    parser.add_argument("--led-color", default="#ffd9a3",
                        help="LED warm-white color (hex).")
    parser.add_argument("--led-intensity", type=float, default=0.18,
                        help="Per-LED point-light intensity (default: 0.18).")
    parser.add_argument("--led-sphere-radius", type=float, default=12.0,
                        help="Visible glow-ball size at each LED (mm).")
    parser.add_argument("--no-panels", action="store_true",
                        help="Skip rendering acrylic panels (frame only).")
    parser.add_argument("--no-leds", action="store_true",
                        help="Skip LED sources (just metal + panels in ambient).")
    parser.add_argument("--no-room", action="store_true",
                        help="Skip the surrounding room (chandelier on dark background).")
    parser.add_argument("--room-width", type=float, default=5.0,
                        help="Room width (and depth) in metres (default 5 m).")
    parser.add_argument("--room-height", type=float, default=3.5,
                        help="Floor-to-ceiling height in metres (default 3.5 m).")
    parser.add_argument("--shadows", action="store_true",
                        help="Enable shadow casting from LEDs onto room surfaces "
                             "(slower; produces visible silhouette patterns).")
    parser.add_argument("--window", default="1280x960",
                        help="Window/screenshot size as WxH pixels (default 1280x960).")
    parser.add_argument("--view", default="room",
                        choices=("room", "front", "below", "above"),
                        help="Default camera angle.")
    args = parser.parse_args()

    width, height = (int(x) for x in args.window.lower().split("x"))

    if not os.path.exists(args.stl):
        raise SystemExit(
            f"STL not found: {args.stl} — run `./run.sh all_polyhedra.py` first."
        )

    if args.export_blender:
        import json
        os.makedirs(args.export_blender, exist_ok=True)
        print(f"Building panels for export to {args.export_blender} ...", flush=True)
        panel_trimesh, led_centers = build_all_panels()
        ply_path = os.path.join(args.export_blender, "panels.ply")
        json_path = os.path.join(args.export_blender, "led_positions.json")
        panel_trimesh.export(ply_path)
        with open(json_path, "w") as f:
            json.dump({
                "scale_factor": SCALE_FACTOR,
                "panel_thickness_post_mm": PANEL_THICKNESS_POST,
                "panel_inset_post_mm": PANEL_INSET_POST,
                "led_count": len(led_centers),
                "led_positions_post_mm": [c.tolist() for c in led_centers],
            }, f, indent=2)
        print(f"  wrote {ply_path} ({os.path.getsize(ply_path):,} bytes)", flush=True)
        print(f"  wrote {json_path}", flush=True)
        print("Run `./render_blender.sh` to render with Blender Cycles.", flush=True)
        return

    print(f"Loading chandelier mesh from {args.stl} ...", flush=True)
    metal_mesh = pv.read(args.stl)
    print(f"  {metal_mesh.n_points:,} points, {metal_mesh.n_cells:,} cells", flush=True)

    if args.no_panels:
        panel_pv = None
        led_centers = [np.asarray(c, dtype=float) * SCALE_FACTOR
                       for _, c in _layout_positions()]
    else:
        print("Building acrylic panels (1,029 polygons across 31 solids) ...", flush=True)
        panel_trimesh, led_centers = build_all_panels()
        panel_pv = _trimesh_to_pv(panel_trimesh)
        print(f"  {panel_pv.n_points:,} points, {panel_pv.n_cells:,} cells", flush=True)

    print(f"Setting up scene with {len(led_centers)} LED sources ...", flush=True)
    off_screen = bool(args.screenshot)
    plotter = pv.Plotter(window_size=(width, height),
                         lighting="none",
                         off_screen=off_screen)
    plotter.set_background(args.background)
    plotter.enable_anti_aliasing("ssaa")
    if panel_pv is not None:
        plotter.enable_depth_peeling(number_of_peels=12, occlusion_ratio=0.0)

    # Metal frame — PBR with metallic / roughness for an anodized look.
    plotter.add_mesh(
        metal_mesh,
        color=args.metal_color,
        pbr=True,
        metallic=0.85,
        roughness=0.35,
        smooth_shading=True,
        specular=1.0,
        specular_power=20.0,
    )

    # Acrylic panels — translucent and self-illuminated to simulate the LED
    # behind each one shining through frosted acrylic.
    if panel_pv is not None:
        plotter.add_mesh(
            panel_pv,
            color=args.panel_color,
            opacity=args.panel_opacity,
            ambient=args.panel_glow,   # high ambient = "glowing" look
            diffuse=0.6,
            specular=0.5,
            specular_power=18.0,
            smooth_shading=True,
        )

    # Surrounding room — six matte planes so LED light has surfaces to fall on.
    chandelier_bounds = metal_mesh.bounds
    if not args.no_room:
        room_w = args.room_width * 1000.0
        room_h = args.room_height * 1000.0
        print(f"Adding room: {args.room_width} m × {args.room_width} m × "
              f"{args.room_height} m tall", flush=True)
        for label, plane_mesh, kw in build_room(chandelier_bounds, room_w, room_h):
            plotter.add_mesh(
                plane_mesh,
                ambient=0.04,        # very dark when unlit
                diffuse=0.95,
                specular=0.05,
                smooth_shading=False,
                **kw,
            )

    # LED visible glow balls — always rendered so the eye finds the LEDs.
    if not args.no_leds:
        for c in led_centers:
            sphere = pv.Sphere(radius=args.led_sphere_radius, center=c,
                               theta_resolution=14, phi_resolution=14)
            plotter.add_mesh(
                sphere,
                color=args.led_color,
                ambient=1.0,
                diffuse=0.0,
                specular=0.0,
                lighting=False,
            )

    # Lighting strategy:
    #   • without --shadows: 31 per-LED positional lights for soft, even fill
    #   • with --shadows:    1 bright central light + a few accent lights —
    #     VTK's shadow-mapping shader caps at ~8 lights, so we collapse the
    #     31 LEDs into a single representative source at the chandelier
    #     centroid so it can cast a crisp silhouette of the whole fixture.
    if not args.no_leds:
        if args.shadows:
            chandelier_centroid = np.mean(led_centers, axis=0)
            big = pv.Light(
                position=tuple(chandelier_centroid),
                color=args.led_color,
                intensity=min(2.5, args.led_intensity * 6.0),
            )
            big.positional = True
            big.cone_angle = 180.0
            plotter.add_light(big)
            print("  Shadow mode: using 1 central light to cast the chandelier silhouette",
                  flush=True)
        else:
            for c in led_centers:
                light = pv.Light(
                    position=tuple(c),
                    color=args.led_color,
                    intensity=args.led_intensity,
                )
                light.positional = True
                light.cone_angle = 180.0
                plotter.add_light(light)

    # Faint ambient fill so the room is barely visible even without LEDs.
    plotter.add_light(
        pv.Light(light_type="scene light", color="#202028", intensity=0.04)
    )

    if args.shadows:
        print("Enabling shadow casting ...", flush=True)
        plotter.enable_shadows()

    # Camera framing.
    bounds = chandelier_bounds
    cx = (bounds[0] + bounds[1]) / 2
    cy = (bounds[2] + bounds[3]) / 2
    cz = (bounds[4] + bounds[5]) / 2
    chandelier_extent = max(bounds[1] - bounds[0],
                             bounds[3] - bounds[2],
                             bounds[5] - bounds[4])
    if args.view == "room" and not args.no_room:
        # Camera on the floor in one corner of the room, like a person
        # standing across the room from the chandelier and looking up at it.
        room_w = args.room_width * 1000.0
        room_h = args.room_height * 1000.0
        ceiling_y = bounds[3] + 200
        floor_y = ceiling_y - room_h
        # Eye level ~ 1.6 m above the floor (a standing person)
        eye_y = floor_y + 1600.0
        # 80 % of the way to a corner of the room from the chandelier axis
        cam_x = cx + room_w * 0.42
        cam_z = cz + room_w * 0.42
        plotter.camera.position = (cam_x, eye_y, cam_z)
        # Look at the centre of the chandelier
        plotter.camera.focal_point = (cx, cy, cz)
    elif args.view == "front":
        plotter.camera.position = (cx, cy, cz + chandelier_extent * 1.6)
        plotter.camera.focal_point = (cx, cy, cz)
    elif args.view == "below":
        plotter.camera.position = (cx, cy - chandelier_extent * 1.0, cz + chandelier_extent * 0.4)
        plotter.camera.focal_point = (cx, cy, cz)
    elif args.view == "above":
        plotter.camera.position = (cx, cy + chandelier_extent * 1.2, cz + chandelier_extent * 0.4)
        plotter.camera.focal_point = (cx, cy, cz)
    else:
        plotter.camera.position = (cx + chandelier_extent * 0.9,
                                   cy + chandelier_extent * 0.05,
                                   cz + chandelier_extent * 1.4)
        plotter.camera.focal_point = (cx, cy, cz)
    plotter.camera.up = (0, 1, 0)

    if args.screenshot:
        out = os.path.abspath(args.screenshot)
        print(f"Rendering off-screen to {out} ({width}×{height}) ...", flush=True)
        plotter.show(screenshot=out, auto_close=True, interactive=False)
        if os.path.exists(out):
            print(f"Saved {os.path.getsize(out):,} bytes to {out}", flush=True)
        else:
            print(f"WARNING: screenshot was not written to {out}", flush=True)
    else:
        print("Opening interactive viewer.  Drag to orbit, scroll to zoom, 'q' to quit.",
              flush=True)
        plotter.show()


if __name__ == "__main__":
    main()
