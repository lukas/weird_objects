"""Render every Platonic, Archimedean, and Catalan solid as a wireframe
on three concentric rings, exported as one STL.

Layout:
    top    ring (z = +Δ)   -> 5 Platonic     on a small ring (5 points)
    middle ring (z =  0 )  -> 13 Archimedean on a wide ring  (13 points)
    bottom ring (z = -Δ)   -> 13 Catalan     on a wide ring  (13 points)

Each Catalan is positioned directly below its Archimedean parent so the
dual pairing is visible at a glance.  Each solid is rendered as a
metal-friendly cylinder wireframe with a small sphere at every vertex.
"""

from __future__ import annotations

import numpy as np
import trimesh

import polyhedra as P


# ---------------------------------------------------------------------------
# Wireframe rendering
# ---------------------------------------------------------------------------

# Edge & node thicknesses are sized to accommodate two opposing slots
# (one per adjacent face) cut into each edge rod for capturing the
# acrylic panels.  With slot depth 0.55 mm pre on each side, an edge
# rod of 1.10 mm pre radius retains 1.10 mm of metal at its centre
# (post ≈ 4.0 mm) — comfortable for cast aluminum and structurally rigid.
EDGE_RADIUS = 1.10         # mm — wireframe rod thickness  (post-scale OD ≈ 7.9 mm)
NODE_RADIUS = 1.40         # mm — vertex sphere radius     (post-scale OD ≈ 10.1 mm)
NODE_DIAMETER_MM = 36.0    # target outer diameter for each polyhedron

# Acrylic panel slots — every face except the lowest gets a captured
# panel.  The lowest face is left open as the assembly access path.
# Slots are cast (or post-machined) into the inward side of each edge
# rod.  Sized for 1/16" (1.6 mm) acrylic with ~0.4 mm of clearance.
PANEL_SLOT_WIDTH       = 0.55  # mm — slot width (panel thickness + clearance) (post ≈ 2.0 mm)
PANEL_SLOT_DEPTH       = 0.55  # mm — slot depth into the rod                  (post ≈ 2.0 mm)
PANEL_THICKNESS_NOMINAL = 0.45 # mm — target panel thickness                    (post ≈ 1.62 mm — 1/16" acrylic)
# Slots terminate this far short of each vertex node so they do not
# carve into the node sphere.  Edges shorter than 2× this distance get
# no slot (rare, only on the busiest Archimedean / Catalan faces).
PANEL_SLOT_NODE_CLEARANCE = NODE_RADIUS * 1.2  # mm

# Internal "spider" — 4 rods from 4 spread-out vertices of each polyhedron
# meeting at a central LED-mount boss.  The spider gives the polyhedron a
# load path through its centre and a place to attach a small LED.  The
# rods are deliberately thick enough that a small wire-channel can be
# drilled through the top one without violating min-wall thickness.
INTERNAL_ROD_RADIUS = 1.20   # mm — thick enough for an in-rod wire channel + visual harmony with thicker edges  (post-scale OD ≈ 8.6 mm)
LED_MOUNT_RADIUS    = 4.00   # mm — central boss radius                      (post-scale OD ≈ 28.8 mm)
INTERNAL_VERTEX_COUNT = 4    # tetrahedral spread of attachment points

# Wire-routing.  The top spider arm + boss are drilled through with a
# small channel (so wires exit out the top vertex of each polyhedron),
# and U-grooves are cut into the underside of the external carriers
# (hangers, hub spokes, canopy column) so wires snap in and are hidden
# from below.  Both features are sized for a 26 AWG, 2-conductor cable
# (~3 mm OD bundle post-scale).
WIRE_CHANNEL_RADIUS = 0.30   # mm — drilled internal channel    (post ≈ 2.16 mm)
WIRE_GROOVE_RADIUS  = 0.40   # mm — U-groove cutter             (post ≈ 2.88 mm wide × 1.44 mm deep)

# Wire trunk on each Archimedean — three rods (bottom vertex → outward
# kink → upward → inward kink → top vertex) running on the radially-
# outward side of the polyhedron.  The trunk carries the Catalan LED's
# wire pair from the bottom hanger up to the top hanger so it never
# has to cross the polyhedron's wireframe.
TRUNK_ROD_RADIUS  = 0.80     # mm — thick enough for an internal wire channel + wall
TRUNK_NODE_RADIUS = 1.10     # mm — small bead at each trunk corner
# Radial distance (from the polyhedron's vertical axis) of the
# Archimedean trunk's vertical rod.  Placed just outside the LED boss
# (LED_MOUNT_RADIUS = 4.0) so the rod can carry the Catalan's wires up
# through the polyhedron's interior without crossing the LED light cone.
TRUNK_INTERNAL_OFFSET = 6.0  # mm — centred on radially-outward side of the polyhedron interior

# LED pocket — sized for a 12 V, 1–3 W COB LED module.  Common modules are
# 10–16 mm OD × 1–2 mm thick; the pocket is 18 mm OD × 5.4 mm deep
# (post-scale) so any of those drops in flush with adhesive room.  The
# pocket axis points downward so the LED's emitting face shines out the
# bottom of each polyhedron — light fans out below the chandelier.
LED_POCKET_RADIUS    = 2.50    # mm — pocket bore  (post-scale OD ≈ 18.0 mm)
LED_POCKET_DEPTH     = 1.50    # mm — pocket depth (post-scale ≈ 5.4 mm)
LED_POCKET_AXIS      = np.array([0.0, -1.0, 0.0])
# Spider rods must not intrude into the pocket.  Each rod terminates at
# 0.9 × LED_MOUNT_RADIUS from the polyhedron centre; for that endpoint
# to land outside the pocket cylinder, the rod's direction must make
# more than ~50° with the pocket axis (analysis below).  We exclude any
# candidate vertex whose direction from the centre is within that cone.
#   tip = 0.9 R · dir;  pocket spans  y ∈ [-R, -R + d]  and  radial ≤ p
#   tip outside pocket  iff  |tip.y| < R - d  OR  radial(tip) > p
#   With R = 4, d = 1.5, p = 2.5  →  exclusion half-angle ≈ 47°
# cos(50°) gives a small safety margin over that.
LED_POCKET_EXCLUDE_COS = 0.643

# Ring geometry.  Adjacent-solid chord distances:
#   5-ring at r=58 mm  -> ~68 mm chord
#  13-ring at r=130 mm -> ~62 mm chord
# Rings are horizontal circles in the X–Z plane.  Y is the "up" axis used
# by most STL viewers (Cursor preview, Three.js, MeshLab default).
# Layout:  Archimedean (top)  /  Platonic (middle)  /  Catalan (bottom)
# A flat double-star ornament floats above everything at Y_STAR_PLATE.
PLATONIC_RING_R     = 58.0
ARCHIMEDEAN_RING_R  = 130.0
CATALAN_RING_R      = 130.0
Y_ARCHIMEDEAN       =  80.0
Y_PLATONIC          =   0.0
Y_CATALAN           = -80.0

# Double-star ornament — two criss-crossing-rod star polygons at the same
# height, above the Archimedean ring.
#   * Inner pentagram  {5/2}   — 5 rods, tips above the 5 Platonic solids.
#   * Outer 13-pt star {13/5}  — 13 rods, tips above the Archimedean/Catalan column.
Y_STAR              = 150.0       # mm — height of the double-star ornament
STAR_OUTER_R        = ARCHIMEDEAN_RING_R   # 130 mm — vertices align with Archimedean column
STAR_INNER_R        = PLATONIC_RING_R      # 58  mm — vertices align with Platonic positions
STAR_OUTER_STEP     = 5           # connect every 5th vertex  -> {13/5}
STAR_INNER_STEP     = 2           # connect every 2nd vertex  -> {5/2} pentagram
STAR_ROD_RADIUS     = 1.30        # mm — rod thickness for the criss-cross stars (post ≈ 9.4 mm OD)
STAR_NODE_RADIUS    = 2.00        # mm — node sphere at each tip                  (post ≈ 14.4 mm OD)

RING_PHASE = np.pi / 2.0          # one solid sits "north" (+z direction) on each ring

# ---------------------------------------------------------------------------
# Structural hardware — hangers, hub, canopy.  All in pre-scale mm; they
# scale with the rest of the chandelier at the final uniform-scale pass.
# ---------------------------------------------------------------------------
HANGER_ROD_RADIUS    = 1.00       # mm — star tip → polyhedron and arch → catalan (post ≈ 7.2 mm OD)
HANGER_NODE_RADIUS   = 1.50       # mm — small bead at each hanger end
HUB_RADIUS           = 3.00       # mm — central sphere at the star plane centre
HUB_SPOKE_RADIUS     = 1.00       # mm — in-plane spokes from hub to all 18 star tips
COLUMN_RADIUS        = 2.00       # mm — vertical column canopy → hub  (post ≈ 14.4 mm OD)

# The canopy is now a hollow housing.  An axial cylindrical cavity
# inside the disk holds a small 12 V LED driver (sized for a Mean Well
# LPV-35-12 or similar: 88 × 38 × 22 mm).  The cavity opens at the
# bottom of the disk so the driver slides up into it during assembly.
CANOPY_DISK_RADIUS    = 35.0      # mm  — post ≈ 126 mm OD  (~5 in)
CANOPY_DISK_HEIGHT    = 16.0      # mm  — post ≈ 58 mm tall (~2.3 in)
DRIVER_CAVITY_RADIUS  = 30.0      # mm  — post ≈ 108 mm cavity ID (~4.25 in)
DRIVER_CAVITY_DEPTH   = 13.0      # mm  — post ≈ 47 mm deep (~1.85 in)

CANOPY_NECK_RADIUS   =  1.4       # mm — short rod between disk top and eye
EYE_MAJOR_RADIUS     =  8.0       # mm — radius of the hanging eye loop
EYE_MINOR_RADIUS     =  1.4       # mm — eye loop tube thickness  (post ≈ 10.1 mm OD — strong enough for the full load with safety factor)
EYE_GAP              =  3.0       # mm — gap between disk top and eye bottom

Y_CANOPY_DISK_BOTTOM = Y_STAR + 50.0        # canopy floats well above the star plane
Y_CANOPY_DISK_TOP    = Y_CANOPY_DISK_BOTTOM + CANOPY_DISK_HEIGHT
Y_EYE_BOTTOM         = Y_CANOPY_DISK_TOP + EYE_GAP
# Lowest point of the torus tube is EYE_MAJOR_RADIUS + EYE_MINOR_RADIUS
# below the torus centre; add both so the visible bottom of the eye sits
# at exactly Y_EYE_BOTTOM.
Y_EYE_CENTER         = Y_EYE_BOTTOM + EYE_MAJOR_RADIUS + EYE_MINOR_RADIUS

# Final-pass uniform scale.  Set by target horizontal diameter; everything
# (ring radius, polyhedron size, rod thickness, node spheres) scales together.
TARGET_OUTER_DIAMETER_MM = 1066.8  # 3.5 ft = 42 in
MM_PER_INCH = 25.4

OUTFILE = "all_polyhedra_31.stl"
METAL_CORE_OUTFILE = "chandelier_metal_core.stl"
METAL_MINIMAL_OUTFILE = "chandelier_metal_minimal.stl"

# A356-T6 cast aluminum density (g/cc), used for foundry mass estimates.
A356_DENSITY_G_PER_CC = 2.68

# Minimalist canopy plate: a wider, slightly thicker disk than the full-cast
# canopy so it has room for 31+ small drilled holes (cable + fiber drops)
# arranged on inner / outer rings to match the polyhedron columns below.
MINIMAL_CANOPY_DISK_RADIUS  = 28.0   # mm pre-scale  (~ 100 mm post-scale)
MINIMAL_CANOPY_DISK_HEIGHT  =  5.0   # mm pre-scale  (~  18 mm post-scale)


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v if n == 0 else v / n


def _cylinder_between(p1, p2, radius, sections=14):
    p1, p2 = np.asarray(p1), np.asarray(p2)
    direction = p2 - p1
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return None
    cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    cyl.apply_translation([0, 0, length / 2.0])
    z = np.array([0.0, 0.0, 1.0])
    T = trimesh.geometry.align_vectors(z, _unit(direction))
    cyl.apply_transform(T)
    cyl.apply_translation(p1)
    return cyl


def _face_slot_cutter(v_a, v_b, face_centroid, edge_radius=EDGE_RADIUS):
    """Build a single rectangular cutter that, when subtracted from the
    polyhedron mesh, carves a panel-retention slot in the edge rod
    between vertices ``v_a`` and ``v_b``.  The slot's open face points
    toward ``face_centroid`` (i.e. into the polyhedron face whose panel
    edge will sit there) and is recessed PANEL_SLOT_DEPTH into the rod.
    The slot terminates short of each vertex node by
    PANEL_SLOT_NODE_CLEARANCE so it never carves into the node sphere."""
    v_a = np.asarray(v_a, dtype=float)
    v_b = np.asarray(v_b, dtype=float)
    edge_vec = v_b - v_a
    edge_length = np.linalg.norm(edge_vec)
    if edge_length < 1e-6:
        return None
    edge_axis = edge_vec / edge_length
    edge_midpoint = (v_a + v_b) / 2.0

    # Slot length: leave room near each vertex for the node sphere.
    slot_length = edge_length - 2.0 * PANEL_SLOT_NODE_CLEARANCE
    if slot_length < 1.0:  # too short to be useful — skip this edge
        return None

    # In-plane direction from edge midpoint toward face centroid,
    # projected perpendicular to the edge axis.  This is the direction
    # the slot opens toward.
    d_in_raw = face_centroid - edge_midpoint
    d_in = d_in_raw - np.dot(d_in_raw, edge_axis) * edge_axis
    d_in_norm = np.linalg.norm(d_in)
    if d_in_norm < 1e-6:
        return None
    d_in = d_in / d_in_norm

    # Face normal: perpendicular to both edge axis and slot opening direction.
    n_f = np.cross(edge_axis, d_in)
    n_f_norm = np.linalg.norm(n_f)
    if n_f_norm < 1e-6:
        return None
    n_f = n_f / n_f_norm

    # Box extents in its local frame:
    #   x_local = N_F           → slot width  (panel-thickness direction)
    #   y_local = edge axis     → slot length (along the rod)
    #   z_local = d_in          → slot depth  (oversized so cutter exits rod)
    slot_box_depth = PANEL_SLOT_DEPTH * 2.5
    box = trimesh.creation.box(
        extents=[PANEL_SLOT_WIDTH, slot_length, slot_box_depth]
    )

    # Position: box center placed so the box's deepest face sits at
    # (edge_radius - PANEL_SLOT_DEPTH) from the rod axis, and the box
    # extends past the rod surface (clean boolean exit).
    center_offset = (edge_radius - PANEL_SLOT_DEPTH) + slot_box_depth / 2.0
    box_center = edge_midpoint + center_offset * d_in

    R = np.column_stack([n_f, edge_axis, d_in])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = box_center
    box.apply_transform(T)
    return box


def _build_panel_slot_cutters(solid, world_verts, edge_radius=EDGE_RADIUS):
    """Return a list of box cutters that carve panel-retention slots
    into the polyhedron's edge rods — one slot per (face, edge) pair,
    skipping the lowest face so it can serve as the assembly-access
    opening (panels for all upper faces slide in through it)."""
    mesh = P.generate_mesh(solid)

    # Group triangles into polygon faces using trimesh's facet detection
    # (coplanar adjacent triangles form one polygon).  Triangles that
    # aren't part of any multi-triangle facet are themselves single
    # triangular polygon faces.
    faceted_tris = set()
    for facet in mesh.facets:
        for tri_idx in facet:
            faceted_tris.add(int(tri_idx))

    faces = []
    for facet, boundary_edges in zip(mesh.facets, mesh.facets_boundary):
        vert_indices = np.unique(mesh.faces[np.asarray(facet)].flatten())
        centroid = world_verts[vert_indices].mean(axis=0)
        edges_list = [(int(e[0]), int(e[1])) for e in boundary_edges]
        faces.append((centroid, edges_list))

    for tri_idx in range(len(mesh.faces)):
        if tri_idx in faceted_tris:
            continue
        tri = mesh.faces[tri_idx]
        centroid = world_verts[tri].mean(axis=0)
        edges_list = [
            (int(tri[0]), int(tri[1])),
            (int(tri[1]), int(tri[2])),
            (int(tri[2]), int(tri[0])),
        ]
        faces.append((centroid, edges_list))

    if not faces:
        return []

    # Lowest-centroid face is left open for assembly access.
    bottom_idx = int(np.argmin([c[1] for c, _ in faces]))

    cutters = []
    for fi, (centroid, edges_list) in enumerate(faces):
        if fi == bottom_idx:
            continue
        for v_i, v_j in edges_list:
            cutter = _face_slot_cutter(world_verts[v_i], world_verts[v_j],
                                       centroid, edge_radius)
            if cutter is not None:
                cutters.append(cutter)
    return cutters


def _apply_groove(mesh, p1, p2, rod_radius, groove_radius=WIRE_GROOVE_RADIUS):
    """Cut a U-groove of radius ``groove_radius`` along the surface of a
    cylindrical rod going from ``p1`` to ``p2`` (rod radius ``rod_radius``).
    Groove direction is chosen to face upward (+Y) for mostly horizontal
    rods, falling back to +X for rods that are nearly vertical, so wires
    sit on top of horizontal members and are invisible from below."""
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    axis = p2 - p1
    length = np.linalg.norm(axis)
    if length < 1e-6 or groove_radius <= 0 or rod_radius <= groove_radius:
        return mesh
    a = axis / length

    prefer = np.array([0.0, 1.0, 0.0])
    p_perp = prefer - np.dot(prefer, a) * a
    if np.linalg.norm(p_perp) < 0.1:
        prefer = np.array([1.0, 0.0, 0.0])
        p_perp = prefer - np.dot(prefer, a) * a
    groove_dir = p_perp / np.linalg.norm(p_perp)

    # Cutter: same axis as rod, offset perpendicular by rod_radius so its
    # axis lies on the rod surface.  Oversized in length so it pokes past
    # both endpoints — produces a clean half-circle notch all the way
    # along the rod and through any joining nodes at the ends.
    overshoot = 2.0 * groove_radius * a
    cutter_p1 = p1 - overshoot + rod_radius * groove_dir
    cutter_p2 = p2 + overshoot + rod_radius * groove_dir
    cutter = _cylinder_between(cutter_p1, cutter_p2, groove_radius, sections=24)
    if cutter is None:
        return mesh
    return trimesh.boolean.difference([mesh, cutter], engine="manifold")


def _watertight_union(parts, label=""):
    """Boolean-union a list of (already-watertight) primitives into one
    watertight, manifold mesh.  Each input must be closed; the result is
    a single connected piece.  Falls back to concatenation if the boolean
    op fails (with a warning)."""
    parts = [p for p in parts if p is not None and len(p.faces) > 0]
    if not parts:
        return None
    if len(parts) == 1:
        return parts[0].copy()
    try:
        return trimesh.boolean.union(parts, engine="manifold")
    except Exception as e:  # pragma: no cover — defensive fallback
        print(f"  boolean union failed for {label!r}: {e}; falling back to concatenate")
        return trimesh.util.concatenate(parts)


def _spider_indices(world_verts, center_world, count, start_idx):
    """Pick ``count`` vertex indices for the internal LED spider, starting
    at ``start_idx`` (the suspension top vertex) and using greedy
    farthest-point sampling.  Vertices whose direction from the polyhedron
    centre falls inside the LED pocket cone are excluded so that no
    spider rod can intrude into the pocket carved out for the LED."""
    directions = world_verts - center_world
    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    unit_dirs = directions / np.maximum(norms, 1e-9)
    in_pocket_cone = unit_dirs @ LED_POCKET_AXIS > LED_POCKET_EXCLUDE_COS
    in_pocket_cone[start_idx] = False  # the suspension top vertex is always allowed

    chosen = [int(start_idx)]
    dists = np.linalg.norm(world_verts - world_verts[start_idx], axis=1)
    dists[in_pocket_cone] = -np.inf
    while len(chosen) < min(count, len(world_verts)):
        next_idx = int(np.argmax(dists))
        if dists[next_idx] <= 0:
            break
        chosen.append(next_idx)
        new_d = np.linalg.norm(world_verts - world_verts[next_idx], axis=1)
        new_d[in_pocket_cone] = -np.inf
        dists = np.minimum(dists, new_d)
    return chosen


def _archimedean_trunk_endpoints(world_verts, center_world):
    """Return ``(bottom_v, bend_bottom, bend_top, top_v)`` — the four
    waypoints of the internal Archimedean wire trunk.  The trunk runs
    from the polyhedron's bottom vertex node up its radially-outward
    side, parallel to +Y just outside the LED boss, then back into the
    top vertex node.  Same routing as the old external trunk but tucked
    inside the polyhedron so no rods are visible from outside."""
    cw = np.asarray(center_world, dtype=float)

    outward = np.array([cw[0], 0.0, cw[2]])
    if np.linalg.norm(outward) < 1e-6:
        outward = np.array([1.0, 0.0, 0.0])
    outward = outward / np.linalg.norm(outward)

    bottom_v = world_verts[int(np.argmin(world_verts[:, 1]))]
    top_v    = world_verts[int(np.argmax(world_verts[:, 1]))]

    offset_vec = TRUNK_INTERNAL_OFFSET * outward
    # Bend points sit just inside the bottom and top vertex spheres on
    # the radially-outward side, so the stubs are short and straight.
    bend_bottom = np.array([cw[0] + offset_vec[0],
                            bottom_v[1] + 2.0,
                            cw[2] + offset_vec[2]])
    bend_top    = np.array([cw[0] + offset_vec[0],
                            top_v[1] - 2.0,
                            cw[2] + offset_vec[2]])
    return bottom_v, bend_bottom, bend_top, top_v


def _archimedean_trunk_parts(world_verts, center_world, target_diameter):
    """Three rods + two small bead nodes forming the *internal* wire
    trunk that each Archimedean carries on its radially-outward side,
    just outside the LED boss.  Wires coming up from the Catalan below
    travel from the polyhedron's bottom vertex node, up the trunk
    parallel to the polyhedron's vertical axis, back into the top
    vertex node, and out into the upper hanger — never visible from
    outside the polyhedron."""
    del target_diameter  # no longer used; kept for backward call signature
    bottom_v, bend_bottom, bend_top, top_v = _archimedean_trunk_endpoints(
        world_verts, center_world
    )

    parts = []
    for p1, p2 in [(bottom_v, bend_bottom),
                   (bend_bottom, bend_top),
                   (bend_top, top_v)]:
        cyl = _cylinder_between(p1, p2, TRUNK_ROD_RADIUS)
        if cyl is not None:
            parts.append(cyl)

    for p in (bend_bottom, bend_top):
        s = trimesh.creation.icosphere(subdivisions=2, radius=TRUNK_NODE_RADIUS)
        s.apply_translation(p)
        parts.append(s)
    return parts


def _make_led_mount(center_world):
    """Spherical boss with a downward-facing cylindrical pocket sized for
    a 16 mm OD COB LED module.  Manifold-watertight after the boolean
    subtraction so it unions cleanly with the spider rods."""
    boss = trimesh.creation.icosphere(subdivisions=3, radius=LED_MOUNT_RADIUS)

    # Pocket cylinder.  Native cylinder axis is +Z, so rotate to +Y.  The
    # height is generously oversized (4× boss radius) so its bottom face
    # sits well below the sphere — boolean.difference clips it cleanly.
    pocket_h = 4.0 * LED_MOUNT_RADIUS
    pocket = trimesh.creation.cylinder(radius=LED_POCKET_RADIUS,
                                       height=pocket_h,
                                       sections=48)
    pocket.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1.0, 0.0, 0.0]))
    # Position so the pocket's TOP face sits LED_POCKET_DEPTH above the
    # bottom of the sphere — i.e., the pocket digs LED_POCKET_DEPTH into
    # the boss measured from the very bottom of the sphere.
    pocket_top_y    = -LED_MOUNT_RADIUS + LED_POCKET_DEPTH
    pocket_center_y = pocket_top_y - pocket_h / 2.0
    pocket.apply_translation([0.0, pocket_center_y, 0.0])

    boss = trimesh.boolean.difference([boss, pocket], engine="manifold")
    boss.apply_translation(center_world)
    return boss


def make_wire_solid(solid, center, target_diameter=NODE_DIAMETER_MM,
                    edge_radius=EDGE_RADIUS, node_radius=NODE_RADIUS):
    """Build a metal-friendly wireframe of the named solid at ``center``,
    scaled so its outer diameter is ``target_diameter`` mm.

    The solid is rotated so that one of its vertices points along +Y (the
    scene's "up" axis), giving every wireframe a clean vertex-on-top pose.
    A 4-rod internal "spider" runs from 4 farthest-spread vertices to a
    central LED-mount boss.

    Returns ``(mesh, world_verts)`` where ``world_verts`` is the (N, 3)
    array of vertex positions of this solid in world coordinates — used
    to locate the top / bottom anchor points for hanger rods."""
    verts, edges = P.generate_wireframe_data(solid)
    verts = np.asarray(verts, dtype=float)
    verts = verts - verts.mean(axis=0)

    # Rotate so a single vertex sits at the top (+Y).  Pick the vertex that
    # already has the largest y-component as the one to align — minimises
    # the rotation needed and is deterministic per solid.
    top_idx = int(np.argmax(verts[:, 1]))
    R = trimesh.geometry.align_vectors(_unit(verts[top_idx]),
                                       np.array([0.0, 1.0, 0.0]))[:3, :3]
    verts = verts @ R.T

    radius = np.linalg.norm(verts, axis=1).max()
    world_verts = verts * (target_diameter / 2.0 / radius) + np.asarray(center)
    center_world = np.asarray(center, dtype=float)

    parts = []
    for i, j in edges:
        cyl = _cylinder_between(world_verts[i], world_verts[j], edge_radius)
        if cyl is not None:
            parts.append(cyl)
    for v in world_verts:
        s = trimesh.creation.icosphere(subdivisions=1, radius=node_radius)
        s.apply_translation(v)
        parts.append(s)

    # Internal spider — start at the suspension top vertex so the LED's
    # load path runs through it, then add 3 more spread-out anchors that
    # avoid the LED pocket cone.  Each rod terminates just inside the
    # boss surface (at 0.9 × LED_MOUNT_RADIUS from the centre) so it
    # welds to the boss via boolean union but cannot reach the pocket.
    top_world_idx = int(np.argmax(world_verts[:, 1]))
    spider_idx = _spider_indices(world_verts, center_world,
                                 INTERNAL_VERTEX_COUNT,
                                 start_idx=top_world_idx)
    for i in spider_idx:
        direction = _unit(world_verts[i] - center_world)
        rod_end = center_world + 0.9 * LED_MOUNT_RADIUS * direction
        rod = _cylinder_between(world_verts[i], rod_end, INTERNAL_ROD_RADIUS)
        if rod is not None:
            parts.append(rod)

    parts.append(_make_led_mount(center_world))

    # Each Archimedean carries an external wire trunk on its radially
    # outward side so the Catalan dangling beneath it can route its
    # wires up to the star tier without crossing the wireframe.
    if solid.category == "archimedean":
        parts.extend(_archimedean_trunk_parts(world_verts, center_world,
                                              target_diameter))

    mesh = _watertight_union(parts, label=solid.name)

    # Drill a small wire-routing channel along +Y from the back of the LED
    # pocket all the way out the top of the suspension vertex node, so
    # LED wires can run inside the boss + top spider arm + top vertex
    # node.  After this, the wire enters the U-groove on the hanger above.
    top_world_vertex = world_verts[top_world_idx]
    pocket_back = center_world + np.array(
        [0.0, -LED_MOUNT_RADIUS + LED_POCKET_DEPTH, 0.0]
    )
    channel_exit = top_world_vertex + np.array([0.0, NODE_RADIUS * 1.2, 0.0])
    cutter = _cylinder_between(pocket_back, channel_exit,
                               WIRE_CHANNEL_RADIUS, sections=24)
    if cutter is not None:
        mesh = trimesh.boolean.difference([mesh, cutter], engine="manifold")

    # For Archimedeans: drill the internal wire trunk's channels.
    # Wire path:
    #   below the bottom vertex (incoming from Catalan-to-Archi hanger)
    #     → straight up into the bottom vertex node (vertical entry)
    #     → angled stub through the trunk's bottom rod
    #     → vertical trunk rod (parallel to +Y, just outside the boss)
    #     → angled stub through the trunk's top rod
    #     → into the top vertex node, where it joins the LED's own
    #       wire and exits up through `channel_exit`.
    if solid.category == "archimedean":
        bv, bb, bt, tv = _archimedean_trunk_endpoints(world_verts, center_world)
        bottom_entry = bv + np.array([0.0, -NODE_RADIUS * 1.2, 0.0])
        trunk_cutters = [
            _cylinder_between(bottom_entry, bv, WIRE_CHANNEL_RADIUS, sections=24),
            _cylinder_between(bv, bb, WIRE_CHANNEL_RADIUS, sections=24),
            _cylinder_between(bb, bt, WIRE_CHANNEL_RADIUS, sections=24),
            _cylinder_between(bt, tv, WIRE_CHANNEL_RADIUS, sections=24),
        ]
        trunk_cutters = [c for c in trunk_cutters if c is not None]
        if trunk_cutters:
            mesh = trimesh.boolean.difference([mesh] + trunk_cutters,
                                              engine="manifold")

    # Carve panel-retention slots into every edge rod so each face
    # (except the lowest one, kept open as the assembly access path)
    # captures a flat acrylic panel.  All slot cutters are subtracted in
    # one boolean pass for performance.
    slot_cutters = _build_panel_slot_cutters(solid, world_verts)
    if slot_cutters:
        mesh = trimesh.boolean.difference([mesh] + slot_cutters,
                                          engine="manifold")

    return mesh, world_verts


def _top_vertex(world_verts):
    """World-coord position of the highest-Y vertex of a solid."""
    return world_verts[int(np.argmax(world_verts[:, 1]))]


def _bottom_vertex(world_verts):
    """World-coord position of the lowest-Y vertex of a solid."""
    return world_verts[int(np.argmin(world_verts[:, 1]))]


# ---------------------------------------------------------------------------
# Hangers, hub spokes, canopy
# ---------------------------------------------------------------------------


def _hanger_rod(p1, p2, rod_radius=HANGER_ROD_RADIUS,
                node_radius=HANGER_NODE_RADIUS):
    """Build a single suspension rod between ``p1`` and ``p2`` with small
    bead spheres at each endpoint so the link unions cleanly with both
    polyhedra it bridges.  The rod itself carries a U-groove on its
    underside / outer side so the LED wires routed through this hanger
    are hidden from a viewer looking up from below."""
    parts = []
    cyl = _cylinder_between(p1, p2, rod_radius)
    if cyl is not None:
        cyl = _apply_groove(cyl, p1, p2, rod_radius)
        parts.append(cyl)
    for p in (p1, p2):
        s = trimesh.creation.icosphere(subdivisions=2, radius=node_radius)
        s.apply_translation(p)
        parts.append(s)
    return parts


def _hanger_pieces(arch_world_verts, plat_world_verts, cat_world_verts):
    """Return ``[(label, mesh)]`` for all 31 hangers as separately-watertight
    meshes.  Used both by the unioned ``build_hangers`` (for the full STL)
    and by the metal-core export (where each hanger ships as its own
    quoted/cast part)."""
    out = []

    outer_tips = _star_tip_positions(13, STAR_OUTER_R, Y_STAR)
    inner_tips = _star_tip_positions( 5, STAR_INNER_R, Y_STAR)

    for tip, plat_verts in zip(inner_tips, plat_world_verts):
        m = _watertight_union(_hanger_rod(tip, _top_vertex(plat_verts)),
                              label="hanger star→Platonic")
        out.append((f"Hanger {len(out):02d} (star→Platonic)", m))

    for tip, arch_verts in zip(outer_tips, arch_world_verts):
        m = _watertight_union(_hanger_rod(tip, _top_vertex(arch_verts)),
                              label="hanger star→Archimedean")
        out.append((f"Hanger {len(out):02d} (star→Archimedean)", m))

    for arch_verts, cat_verts in zip(arch_world_verts, cat_world_verts):
        m = _watertight_union(_hanger_rod(_bottom_vertex(arch_verts),
                                          _top_vertex(cat_verts)),
                              label="hanger Archimedean→Catalan")
        out.append((f"Hanger {len(out):02d} (Archimedean→Catalan)", m))

    return out


def build_hangers(arch_world_verts, plat_world_verts, cat_world_verts):
    """Build all hanging links:

    * 5 inner pentagram tips → top vertex of each Platonic
    * 13 outer 13-star tips  → top vertex of each Archimedean
    * 13 Archimedean bottoms → top vertex of the matched Catalan
    """
    pieces = _hanger_pieces(arch_world_verts, plat_world_verts, cat_world_verts)
    return _watertight_union([m for _, m in pieces], label="hangers")


def build_hub_spokes():
    """Solid hub at the centre of the star plane plus 18 spokes radiating
    out to all star tips.  This is the load-bearing yoke that ties the
    inner pentagram and outer 13-star together and carries the entire
    chandelier from the canopy column above."""
    parts = []

    hub = trimesh.creation.icosphere(subdivisions=3, radius=HUB_RADIUS)
    hub.apply_translation([0.0, Y_STAR, 0.0])
    parts.append(hub)

    hub_centre = [0.0, Y_STAR, 0.0]
    for tip in (_star_tip_positions(13, STAR_OUTER_R, Y_STAR)
                + _star_tip_positions(5, STAR_INNER_R, Y_STAR)):
        rod = _cylinder_between(hub_centre, tip, HUB_SPOKE_RADIUS)
        if rod is not None:
            # Spokes are horizontal: groove faces +Y (top of spoke), so
            # wires lying in it are invisible to anyone below.
            rod = _apply_groove(rod, hub_centre, tip, HUB_SPOKE_RADIUS)
            parts.append(rod)

    return _watertight_union(parts, label="hub + spokes")


def build_canopy():
    """Top canopy: hollow housing for the LED driver, capped by a short
    neck and a circular eye loop, plus the vertical column that drops
    from the disk's solid top down to the central hub.

    Layout (Y is up):
        Y_EYE_CENTER          ─── eye loop centre
        Y_CANOPY_DISK_TOP     ─── solid top of housing
                              ┐
                              │   solid annular wall + top wall
        cavity top            │   driver cavity (cylindrical, opens at
                              │   the bottom face for assembly access)
        Y_CANOPY_DISK_BOTTOM  ─── bottom face of housing, cavity opens here
        ...column descends to hub...

    The column passes through the centre of the cavity; the driver sits
    in the annular space around it.  The U-groove on the column extends
    into the cavity so wires emerge into the driver compartment cleanly.
    """
    parts = []

    # Vertical column: from just inside the central hub up to inside the
    # solid TOP of the canopy disk (above the cavity) so it welds to
    # solid material.  The column carries the entire wire bundle (31
    # LED feeds in parallel) and gets a U-groove along its full length.
    solid_top_mid_y = (Y_CANOPY_DISK_BOTTOM + DRIVER_CAVITY_DEPTH
                       + (CANOPY_DISK_HEIGHT - DRIVER_CAVITY_DEPTH) / 2.0)
    column_p1 = [0.0, Y_STAR - HUB_RADIUS * 0.5, 0.0]
    column_p2 = [0.0, solid_top_mid_y,           0.0]
    column = _cylinder_between(column_p1, column_p2, COLUMN_RADIUS)
    if column is not None:
        column = _apply_groove(column, column_p1, column_p2, COLUMN_RADIUS)
        parts.append(column)

    # Canopy disk: outer cylinder.  Trimesh cylinders extend along Z by
    # default; rotate to put the axis along Y so the disk's flat faces
    # are horizontal.
    disk = trimesh.creation.cylinder(
        radius=CANOPY_DISK_RADIUS,
        height=CANOPY_DISK_HEIGHT,
        sections=64,
    )
    disk.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1.0, 0.0, 0.0]))
    disk.apply_translation([0.0,
                            Y_CANOPY_DISK_BOTTOM + CANOPY_DISK_HEIGHT / 2.0,
                            0.0])

    # Driver cavity: subtract a smaller cylinder from the disk, opening
    # at the bottom face.  Oversize the cavity height so its lower face
    # exits cleanly past the disk's bottom face.
    cavity_h = DRIVER_CAVITY_DEPTH + CANOPY_DISK_HEIGHT  # generous overshoot
    cavity = trimesh.creation.cylinder(
        radius=DRIVER_CAVITY_RADIUS,
        height=cavity_h,
        sections=64,
    )
    cavity.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1.0, 0.0, 0.0]))
    cavity_top_y    = Y_CANOPY_DISK_BOTTOM + DRIVER_CAVITY_DEPTH
    cavity_center_y = cavity_top_y - cavity_h / 2.0
    cavity.apply_translation([0.0, cavity_center_y, 0.0])
    disk = trimesh.boolean.difference([disk, cavity], engine="manifold")
    parts.append(disk)

    # Short neck from disk top all the way up to the centre of the eye
    # (overshoots into both for guaranteed boolean overlap).
    neck = _cylinder_between(
        [0.0, Y_CANOPY_DISK_TOP - 0.5, 0.0],
        [0.0, Y_EYE_CENTER, 0.0],
        CANOPY_NECK_RADIUS,
    )
    if neck is not None:
        parts.append(neck)

    # Eye loop: trimesh's torus is built around the +Z axis, so its ring
    # already lies in the X-Y (vertical) plane with its hole facing ±Z —
    # exactly the orientation a ceiling hook would pass through.
    eye = trimesh.creation.torus(
        major_radius=EYE_MAJOR_RADIUS,
        minor_radius=EYE_MINOR_RADIUS,
        major_sections=48,
        minor_sections=18,
    )
    eye.apply_translation([0.0, Y_EYE_CENTER, 0.0])
    parts.append(eye)

    return _watertight_union(parts, label="canopy + eye")


# ---------------------------------------------------------------------------
# Layout: three concentric rings (Platonic / Archimedean / Catalan)
# ---------------------------------------------------------------------------

# Platonic ring (5 points) — ordered by family (tetrahedral, octahedral × 2,
# icosahedral × 2) so the cube/octahedron and icosahedron/dodecahedron dual
# pairs sit adjacent on the ring.
PLATONIC_ORDER = [
    "Tetrahedron",
    "Cube",
    "Octahedron",
    "Dodecahedron",
    "Icosahedron",
]

# Archimedean ring (13 points) — grouped by symmetry family then sorted by
# vertex count within each family.  This same ordering is used for the
# Catalan ring so each Catalan sits directly below its Archimedean parent.
ARCHIMEDEAN_ORDER = [
    # Tetrahedral family
    "Truncated Tetrahedron",
    # Octahedral family
    "Cuboctahedron",
    "Truncated Cube",
    "Truncated Octahedron",
    "Rhombicuboctahedron",
    "Snub Cube",
    "Truncated Cuboctahedron",
    # Icosahedral family
    "Icosidodecahedron",
    "Truncated Dodecahedron",
    "Truncated Icosahedron",
    "Rhombicosidodecahedron",
    "Snub Dodecahedron",
    "Truncated Icosidodecahedron",
]


def _name_to_solid():
    return {s.name: s for s in P.ALL_SOLIDS}


def _ring_positions(n_points, radius, y, phase=0.0):
    """``n_points`` evenly spaced (x, y, z) positions around a horizontal
    ring at height ``y`` in the X–Z plane.  Y is treated as the "up" axis."""
    out = []
    for i in range(n_points):
        theta = phase + 2.0 * np.pi * i / n_points
        out.append((radius * np.cos(theta), y, radius * np.sin(theta)))
    return out


# ---------------------------------------------------------------------------
# Double-star ornament: criss-crossing rods
# ---------------------------------------------------------------------------


def _star_tip_positions(n_points, radius, y, phase=RING_PHASE):
    """Vertex positions of an n-gon in the X-Z plane at height ``y``."""
    return [
        (radius * np.cos(phase + 2.0 * np.pi * i / n_points),
         y,
         radius * np.sin(phase + 2.0 * np.pi * i / n_points))
        for i in range(n_points)
    ]


def _star_polygon_rods(vertices, step, rod_radius, node_radius):
    """Build the cylinders+nodes for an {n/step} star polygon whose tips
    are at ``vertices``.  Each rod connects vertex i to vertex (i+step) mod n,
    producing n criss-crossing rods that meet only at the n outer tips."""
    n = len(vertices)
    parts = []
    for i in range(n):
        j = (i + step) % n
        cyl = _cylinder_between(vertices[i], vertices[j], rod_radius)
        if cyl is not None:
            parts.append(cyl)
    for v in vertices:
        s = trimesh.creation.icosphere(subdivisions=2, radius=node_radius)
        s.apply_translation(v)
        parts.append(s)
    return parts


def _double_star_pieces():
    """Return ``(outer_13_star_mesh, inner_pentagram_mesh)`` as two
    separately-watertight meshes — same star geometry as the unioned
    ornament but split for foundry quoting (each piece ships as its own
    cast part since the combined ornament is too large for most
    investment-casting cells)."""
    outer_v = _star_tip_positions(13, STAR_OUTER_R, Y_STAR)
    inner_v = _star_tip_positions( 5, STAR_INNER_R, Y_STAR)
    outer = _watertight_union(
        _star_polygon_rods(outer_v, STAR_OUTER_STEP,
                           STAR_ROD_RADIUS, STAR_NODE_RADIUS),
        label="outer 13-star",
    )
    inner = _watertight_union(
        _star_polygon_rods(inner_v, STAR_INNER_STEP,
                           STAR_ROD_RADIUS, STAR_NODE_RADIUS),
        label="inner pentagram",
    )
    return outer, inner


def build_double_star_rods():
    """Inner pentagram (5 criss-crossing rods, {5/2}) plus outer 13-pointed
    star ({13/5}) of criss-crossing rods, both at height ``Y_STAR``.  Tips
    align with the rings below: the 5 inner tips sit directly above the
    Platonic solids and the 13 outer tips sit directly above the Archimedean
    / Catalan columns, so a string tied at any tip drops onto the matching
    solid."""
    outer, inner = _double_star_pieces()
    return _watertight_union([outer, inner], label="double-star ornament")


def _layout_positions():
    """Return ``[(solid, (x, y, z))]`` for every solid (31 total)."""
    by_name = _name_to_solid()

    # Catalan order: dual of each Archimedean, in the same ring order so
    # each Catalan sits directly below its Archimedean parent.
    catalan_order = []
    for arch_name in ARCHIMEDEAN_ORDER:
        catalan = next(s for s in P.CATALAN if s.parent == arch_name)
        catalan_order.append(catalan.name)

    items = []
    # Top ring (highest y): 13 Archimedean.
    for name, pos in zip(
        ARCHIMEDEAN_ORDER,
        _ring_positions(13, ARCHIMEDEAN_RING_R, Y_ARCHIMEDEAN, phase=RING_PHASE),
    ):
        items.append((by_name[name], pos))
    # Middle ring: 5 Platonic.
    for name, pos in zip(
        PLATONIC_ORDER,
        _ring_positions(5, PLATONIC_RING_R, Y_PLATONIC, phase=RING_PHASE),
    ):
        items.append((by_name[name], pos))
    # Bottom ring: 13 Catalan, aligned in the same column as their Archimedean parents.
    for name, pos in zip(
        catalan_order,
        _ring_positions(13, CATALAN_RING_R, Y_CATALAN, phase=RING_PHASE),
    ):
        items.append((by_name[name], pos))
    return items


# ---------------------------------------------------------------------------
# Minimalist canopy plate (fiber-optic + cable-suspension build)
# ---------------------------------------------------------------------------


def build_canopy_minimal():
    """Minimalist canopy plate for the fiber-optic + cable-suspension
    build path.  Just a flat round plate + short neck + circular eye loop
    — no driver cavity, no downward column, no central hub below.

    In the minimalist build the plate sits flush against the ceiling
    canopy and carries:

    * the eye loop on top (single suspension point to a J-hook),
    * an LED fiber illuminator strapped to its underside, and
    * 4 drilled holes (added post-cast, not modelled here, located by
      the ``make_drill_jigs.py`` jig — see ASSEMBLY.md §6.4):
        - 1 × 15 mm Ø at centre, for the fiber-optic bundle exiting
          the illuminator (all 31 fibers share this single bundle
          hole, then fan out to the polyhedra below)
        - 2 × 3 mm Ø at ±18 mm from centre, for the 2 structural
          cables that suspend the inner pentagram and outer 13-star
        - 1 × 9.5 mm Ø near the rim, for the wall-wart cord pass-through
    """
    parts = []

    disk = trimesh.creation.cylinder(
        radius=MINIMAL_CANOPY_DISK_RADIUS,
        height=MINIMAL_CANOPY_DISK_HEIGHT,
        sections=64,
    )
    disk.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1.0, 0.0, 0.0]))
    disk.apply_translation([0.0,
                            Y_CANOPY_DISK_BOTTOM + MINIMAL_CANOPY_DISK_HEIGHT / 2.0,
                            0.0])
    parts.append(disk)

    neck = _cylinder_between(
        [0.0, Y_CANOPY_DISK_BOTTOM + MINIMAL_CANOPY_DISK_HEIGHT - 0.5, 0.0],
        [0.0, Y_EYE_CENTER, 0.0],
        CANOPY_NECK_RADIUS,
    )
    if neck is not None:
        parts.append(neck)

    eye = trimesh.creation.torus(
        major_radius=EYE_MAJOR_RADIUS,
        minor_radius=EYE_MINOR_RADIUS,
        major_sections=48,
        minor_sections=18,
    )
    eye.apply_translation([0.0, Y_EYE_CENTER, 0.0])
    parts.append(eye)

    return _watertight_union(parts, label="minimal canopy plate")


# ---------------------------------------------------------------------------
# Metal-core export (foundry quote subset)
# ---------------------------------------------------------------------------


def export_metal_core(scale, arch_world_verts, plat_world_verts, cat_world_verts):
    """Export just the structural / load-bearing subset of the chandelier
    as a separate STL suitable for sending to a foundry (Xometry, PCBWay,
    art foundries, etc.) for an investment-cast aluminum quote.

    The hybrid build approach: cast metal only the parts that actually
    carry load (canopy, hub, stars, hangers).  The 31 decorative
    polyhedra are then 3D-printed in plastic with metallic paint and
    bolted onto the cast skeleton.

    The output STL contains 35 separately-watertight bodies:
        * 1   canopy + eye
        * 1   hub + 18 in-plane spokes
        * 2   star ornament pieces (outer 13-star, inner pentagram, split
              for envelope reasons — the unioned star is ~600 mm tip-to-tip
              and exceeds most investment-casting cells)
        * 31  individual hangers (each as its own watertight body so the
              foundry can quote and cast them individually)

    All bodies are scaled by the same uniform factor as the main file so
    dimensions match the full chandelier exactly.
    """
    print(f"\nBuilding metal-core foundry export (scale {scale:.3f}x)...")

    pieces = []  # list of (label, mesh)

    print("  canopy + eye")
    pieces.append(("Canopy + Eye", build_canopy()))

    print("  hub + 18 spokes")
    pieces.append(("Hub + Spokes", build_hub_spokes()))

    print("  outer 13-star + inner pentagram (separate)")
    outer, inner = _double_star_pieces()
    pieces.append(("Outer 13-Star",   outer))
    pieces.append(("Inner Pentagram", inner))

    print("  31 individual hangers")
    pieces.extend(_hanger_pieces(arch_world_verts, plat_world_verts,
                                 cat_world_verts))

    for _, m in pieces:
        m.apply_scale(scale)

    n_ok = 0
    total_volume = 0.0
    for label, m in pieces:
        ok = m.is_watertight and m.is_volume and m.is_winding_consistent
        n_ok += int(ok)
        if m.is_volume:
            total_volume += m.volume

    print(f"\n  Metal-core watertight check ({len(pieces)} bodies):")
    print(f"    {n_ok}/{len(pieces)} fully watertight + manifold + consistent winding")

    combined = trimesh.util.concatenate([m for _, m in pieces])
    combined.export(METAL_CORE_OUTFILE)

    mass_kg = total_volume * A356_DENSITY_G_PER_CC / 1.0e6
    extents = combined.bounds[1] - combined.bounds[0]
    print(f"  Bodies in STL:   {len(pieces)} (1 canopy + 1 hub + 2 stars + 31 hangers)")
    print(f"  Total volume:    {total_volume / 1.0e6:.3f} L")
    print(f"  Est. cast mass:  {mass_kg:.2f} kg ({mass_kg * 2.205:.2f} lb) in A356-T6 aluminum")
    print(f"  Bounds (mm):     {extents[0]:.0f} x {extents[1]:.0f} x {extents[2]:.0f}")
    print(f"  Bounds (in):     {extents[0]/MM_PER_INCH:.1f} x {extents[1]/MM_PER_INCH:.1f} x {extents[2]/MM_PER_INCH:.1f}")
    print(f"  Saved {METAL_CORE_OUTFILE}")


def export_metal_minimal(scale):
    """Export the absolute-minimum cast metal subset for the fiber-optic
    + cable-suspension build path.  Only three bodies:

        * 1   minimalist canopy plate + eye loop
        * 1   outer 13-star
        * 1   inner pentagram

    Everything else — the central hub, 18 spokes, vertical column, and
    31 individual hangers — is replaced by stainless aircraft cable +
    end-glow fiber-optic strand drops.  The 31 polyhedra are 3D-printed
    plastic with metallic paint.  See ASSEMBLY.md for the full build.

    All bodies are scaled by the same uniform factor as the main file so
    dimensions match the full chandelier exactly.
    """
    print(f"\nBuilding metal-minimal foundry export (scale {scale:.3f}x)...")

    pieces = []  # list of (label, mesh)

    print("  canopy plate + eye (no column, no driver cavity)")
    pieces.append(("Canopy Plate + Eye", build_canopy_minimal()))

    print("  outer 13-star + inner pentagram (separate)")
    outer, inner = _double_star_pieces()
    pieces.append(("Outer 13-Star",   outer))
    pieces.append(("Inner Pentagram", inner))

    for _, m in pieces:
        m.apply_scale(scale)

    n_ok = 0
    total_volume = 0.0
    for label, m in pieces:
        ok = m.is_watertight and m.is_volume and m.is_winding_consistent
        n_ok += int(ok)
        if m.is_volume:
            total_volume += m.volume

    print(f"\n  Metal-minimal watertight check ({len(pieces)} bodies):")
    for label, m in pieces:
        ok = m.is_watertight and m.is_volume and m.is_winding_consistent
        flag = "OK " if ok else "BAD"
        ext = m.bounds[1] - m.bounds[0]
        print(f"    [{flag}] {label:22s}  volume={m.volume/1.0e3:6.1f} cc  "
              f"bbox={ext[0]:>5.0f} x {ext[1]:>5.0f} x {ext[2]:>5.0f} mm")
    print(f"    -> {n_ok}/{len(pieces)} fully watertight + manifold + consistent winding")

    combined = trimesh.util.concatenate([m for _, m in pieces])
    combined.export(METAL_MINIMAL_OUTFILE)

    mass_kg = total_volume * A356_DENSITY_G_PER_CC / 1.0e6
    extents = combined.bounds[1] - combined.bounds[0]
    print(f"  Bodies in STL:   {len(pieces)} (canopy plate + 2 stars)")
    print(f"  Total volume:    {total_volume / 1.0e6:.3f} L")
    print(f"  Est. cast mass:  {mass_kg:.2f} kg ({mass_kg * 2.205:.2f} lb) in A356-T6 aluminum")
    print(f"  Bounds (mm):     {extents[0]:.0f} x {extents[1]:.0f} x {extents[2]:.0f}")
    print(f"  Bounds (in):     {extents[0]/MM_PER_INCH:.1f} x {extents[1]/MM_PER_INCH:.1f} x {extents[2]/MM_PER_INCH:.1f}")
    print(f"  Saved {METAL_MINIMAL_OUTFILE}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    items = _layout_positions()
    expected = len(P.ALL_SOLIDS)
    assert len(items) == expected, f"layout has {len(items)} items, expected {expected}"

    # Sanity-check that every name in the layout is real and unique.
    layout_names = [s.name for s, _ in items]
    assert len(set(layout_names)) == len(layout_names), "duplicate name in layout"
    assert set(layout_names) == {s.name for s in P.ALL_SOLIDS}, "layout misses solids"

    parts = []
    labelled_parts = []  # (label, mesh) — used for the per-object watertight report
    # Track world-space vertex arrays per solid so we can attach hangers
    # to the right top / bottom anchor points later.
    arch_world_verts = []
    plat_world_verts = []
    cat_world_verts  = []
    for solid, center in items:
        x, y, z = center
        print(f"Building {solid.category:11s}  {solid.name:32s}  at ({x:6.1f}, {y:6.1f}, {z:6.1f})")
        m, world_verts = make_wire_solid(solid, center)
        parts.append(m)
        labelled_parts.append((solid.name, m))
        if solid.category == "archimedean":
            arch_world_verts.append(world_verts)
        elif solid.category == "platonic":
            plat_world_verts.append(world_verts)
        elif solid.category == "catalan":
            cat_world_verts.append(world_verts)

    print(f"Building double-star ornament (criss-cross rods) at y={Y_STAR:.1f}")
    star = build_double_star_rods()
    parts.append(star)
    labelled_parts.append(("Double-Star Ornament", star))

    print("Building hub + 18 in-plane spokes at the star centre")
    hub = build_hub_spokes()
    parts.append(hub)
    labelled_parts.append(("Hub + Spokes", hub))

    print("Building 31 hanger rods (star→Platonic, star→Archimedean, Archimedean→Catalan)")
    hangers = build_hangers(arch_world_verts, plat_world_verts, cat_world_verts)
    parts.append(hangers)
    labelled_parts.append(("Hangers", hangers))

    print(f"Building canopy with hanging eye at y={Y_EYE_CENTER:.1f}")
    canopy = build_canopy()
    parts.append(canopy)
    labelled_parts.append(("Canopy + Eye", canopy))

    # Uniform-scale up to the requested outer diameter, applied to each part
    # so its watertight status survives the transform unchanged.
    raw = trimesh.util.concatenate(parts)
    raw_extents = raw.bounds[1] - raw.bounds[0]
    horizontal_extent = max(raw_extents[0], raw_extents[2])
    scale = TARGET_OUTER_DIAMETER_MM / horizontal_extent
    for _, m in labelled_parts:
        m.apply_scale(scale)

    print("\nWatertight check (per object):")
    n_ok = 0
    for label, m in labelled_parts:
        ok = m.is_watertight and m.is_volume and m.is_winding_consistent
        n_ok += int(ok)
        flag = "OK " if ok else "BAD"
        print(f"  [{flag}] {label:32s}  faces={len(m.faces):>6d}  "
              f"watertight={m.is_watertight}  is_volume={m.is_volume}  "
              f"winding={m.is_winding_consistent}")
    print(f"  -> {n_ok}/{len(labelled_parts)} objects fully watertight + manifold + consistent winding")

    combined = trimesh.util.concatenate([m for _, m in labelled_parts])
    extents = combined.bounds[1] - combined.bounds[0]
    print(f"\nTotal faces:        {len(combined.faces):,}")
    print(f"Scale applied:      {scale:.3f}x")
    print(f"Bounds (mm):        {extents[0]:.1f} x {extents[1]:.1f} x {extents[2]:.1f}")
    print(f"Bounds (in):        {extents[0]/MM_PER_INCH:.1f} x {extents[1]/MM_PER_INCH:.1f} x {extents[2]/MM_PER_INCH:.1f}")
    print(f"Bounds (ft):        {extents[0]/MM_PER_INCH/12:.2f} x {extents[1]/MM_PER_INCH/12:.2f} x {extents[2]/MM_PER_INCH/12:.2f}")
    volume_l = combined.volume / 1.0e6
    print(f"Approx volume L:    {volume_l:.2f}")

    # Material weight estimates so the chandelier is sized to a real ceiling.
    # Densities in g/cc (kg/L).
    materials = [
        ("Aluminum (cast A356)",      2.68),
        ("Bronze (silicon bronze)",   8.53),
        ("Brass (cartridge brass)",   8.50),
        ("Stainless 316L",            7.99),
        ("Steel (mild)",              7.85),
        ("Titanium (Ti-6Al-4V)",      4.43),
        ("PLA-cast investment wax",   1.24),
    ]
    print("\nEstimated finished weight (full part, no LEDs / wiring):")
    for name, rho in materials:
        kg = volume_l * rho
        print(f"  {name:30s}  {kg:6.1f} kg  ({kg * 2.20462:6.1f} lb)")

    combined.export(OUTFILE)
    print(f"Saved {OUTFILE}")

    # Also export the structural / load-bearing subset on its own, for the
    # hybrid build path: cast metal only the 35 structural parts (canopy,
    # hub, stars, hangers) and 3D-print the 31 decorative polyhedra.
    export_metal_core(scale, arch_world_verts, plat_world_verts, cat_world_verts)

    # And the absolute-minimum subset for the fiber-optic + cable-suspension
    # build path: just the canopy plate + 2 stars (3 cast bodies total).
    export_metal_minimal(scale)


if __name__ == "__main__":
    main()
