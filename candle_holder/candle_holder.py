# candle_holder_stl.py

# pip install numpy trimesh scipy

import numpy as np

import trimesh

OUTFILE = "spiral_shadow_candle_holder.stl"

# -----------------------

# Dimensions in millimeters

# -----------------------

outer_radius = 80

inner_radius = 24          # space for tealight / candle cup

height = 32

bar_radius = 2.2           # metal rib thickness; keep >= 2mm

num_spiral_ribs = 36

num_radial_steps = 120

tube_segments = 12

base_ring_radius = 72

top_ring_radius = 62

# -----------------------

# Helpers

# -----------------------

def make_tube_along_curve(points, radius=2.0, sections=12, closed=False):

    """

    Creates a watertight tube mesh following a 3D polyline.

    If closed=True, treats the polyline as a closed loop and the start/end rings
    are stitched together. Otherwise, both ends are capped with triangle fans.

    """

    points = np.asarray(points, dtype=float)

    # If closed and the caller duplicated the first point at the end, drop it.
    if closed and len(points) > 1 and np.allclose(points[0], points[-1]):
        points = points[:-1]

    n = len(points)

    vertices = []

    faces = []

    for i in range(n):

        if closed:

            tangent = points[(i + 1) % n] - points[(i - 1) % n]

        elif i == 0:

            tangent = points[1] - points[0]

        elif i == n - 1:

            tangent = points[-1] - points[-2]

        else:

            tangent = points[i + 1] - points[i - 1]

        tangent = tangent / np.linalg.norm(tangent)

        up = np.array([0, 0, 1.0])

        if abs(np.dot(tangent, up)) > 0.9:

            up = np.array([1.0, 0, 0])

        normal = np.cross(tangent, up)

        normal /= np.linalg.norm(normal)

        binormal = np.cross(tangent, normal)

        binormal /= np.linalg.norm(binormal)

        for j in range(sections):

            a = 2 * np.pi * j / sections

            v = points[i] + radius * (np.cos(a) * normal + np.sin(a) * binormal)

            vertices.append(v)

    rings = n if closed else n - 1

    for i in range(rings):

        i_next = (i + 1) % n if closed else i + 1

        for j in range(sections):

            a = i * sections + j

            b = i * sections + (j + 1) % sections

            c = i_next * sections + (j + 1) % sections

            d = i_next * sections + j

            faces.append([a, b, c])

            faces.append([a, c, d])

    if not closed:

        # Cap both ends with a triangle fan to a center vertex.
        center_start = len(vertices)
        vertices.append(points[0])
        center_end = len(vertices)
        vertices.append(points[-1])

        last_ring = (n - 1) * sections
        for j in range(sections):
            j_next = (j + 1) % sections
            # Start cap: winding flipped so normal points outward (away from tube).
            faces.append([center_start, j_next, j])
            # End cap.
            faces.append([center_end, last_ring + j, last_ring + j_next])

    return trimesh.Trimesh(vertices=np.array(vertices), faces=np.array(faces), process=True)

def ring_tube(radius, z, tube_radius, sections=192):

    points = []

    for i in range(sections):

        a = 2 * np.pi * i / sections

        points.append([radius * np.cos(a), radius * np.sin(a), z])

    return make_tube_along_curve(points, tube_radius, tube_segments, closed=True)

# -----------------------

# Main spiral lattice

# -----------------------

meshes = []

for k in range(num_spiral_ribs):

    phase = 2 * np.pi * k / num_spiral_ribs

    points = []

    for i in range(num_radial_steps):

        t = i / (num_radial_steps - 1)

        # radius goes from inner to outer

        r = inner_radius + (outer_radius - inner_radius) * t

        # twisting creates the shadow pattern

        twist = 1.65 * np.pi * t

        angle = phase + twist

        # low dome shape

        z = height * np.sin(np.pi * t) ** 0.75

        # slight wave creates richer projection

        z += 3.0 * np.sin(6 * np.pi * t + phase)

        points.append([

            r * np.cos(angle),

            r * np.sin(angle),

            z

        ])

    meshes.append(make_tube_along_curve(points, bar_radius, tube_segments))

# Structural circular rings

meshes.append(ring_tube(inner_radius, 0, bar_radius * 1.25))

meshes.append(ring_tube(outer_radius, 0, bar_radius * 1.25))

meshes.append(ring_tube(base_ring_radius, 5, bar_radius))

meshes.append(ring_tube(top_ring_radius, height * 0.62, bar_radius))

# Candle cup / seat: shallow metal ring around tealight

cup_outer = trimesh.creation.cylinder(radius=inner_radius + 5, height=5, sections=128)

cup_inner = trimesh.creation.cylinder(radius=inner_radius - 2, height=7, sections=128)

cup_outer.apply_translation([0, 0, 1.5])

cup_inner.apply_translation([0, 0, 1.5])

# Boolean difference may require blender/manifold on some systems.

# If it fails, just use the solid ring-like visual support below.

try:

    cup = cup_outer.difference(cup_inner)

    meshes.append(cup)

except Exception:

    meshes.append(ring_tube(inner_radius + 2, 1.5, 3.0))

    meshes.append(ring_tube(inner_radius - 2, 1.5, 2.0))

# Four small feet

for a in np.linspace(0, 2 * np.pi, 4, endpoint=False):

    foot = trimesh.creation.cylinder(radius=4, height=3, sections=32)

    foot.apply_translation([

        58 * np.cos(a),

        58 * np.sin(a),

        -2

    ])

    meshes.append(foot)

# -----------------------

# Combine and export

# -----------------------

def boolean_union_all(parts):
    """Boolean-union a list of meshes into one watertight shell.

    Filters out any non-volume parts (manifold3d requires watertight inputs).
    Falls back to plain concatenation if the union still fails.
    """
    volumes = [p for p in parts if p.is_volume]
    skipped = len(parts) - len(volumes)
    if skipped:
        print(f"[warn] skipping {skipped} non-volume parts before union.")
    try:
        return trimesh.boolean.union(volumes, engine="manifold")
    except Exception as e:
        print(f"[warn] boolean union failed ({e}); falling back to concatenate.")
        out = trimesh.util.concatenate(parts)
        out.merge_vertices()
        return out


print(f"Booleaning {len(meshes)} parts into one watertight shell...")
mesh = boolean_union_all(meshes)
print(f"  union result: faces={len(mesh.faces):,} watertight={mesh.is_watertight} volume={mesh.is_volume}")

# manifold3d's output is already clean. Only do safe touch-ups.
mesh.remove_unreferenced_vertices()

mesh.fix_normals()

print("Faces:", len(mesh.faces))

print("Watertight (in-memory):", mesh.is_watertight)

print("Bounds mm:", mesh.bounds)

mesh.export(OUTFILE)
print(f"Saved {OUTFILE}")

# Also export PLY: preserves explicit vertex indices so the watertightness
# survives a roundtrip. STL strips topology, so reloaded STLs may appear
# non-watertight even when the geometry is fine for slicing.
PLY_OUTFILE = OUTFILE.replace(".stl", ".ply")
mesh.export(PLY_OUTFILE)
print(f"Saved {PLY_OUTFILE}  (use this if you need a strict watertight reload)")