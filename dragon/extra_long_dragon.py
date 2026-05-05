"""extra_long_dragon.py - Generate an extra-long ARTICULATED print-in-place
flexi dragon as STL.

Inspired by popular print-in-place flexi designs (Cinderwing's Crystal Dragon,
McGybeer's Articulated Dragon) and articulated snakes (DanielAlex's Articulated
Snake, Crazy3D's Ultra Articulated Snake).  Each body segment terminates in a
ball at the head end and a captive socket cavity at the tail end.  The next
segment's ball sits inside the previous segment's cavity, separated by a
0.4-0.5 mm radial clearance, with the cavity opening narrower than the ball so
the ball is captured during print-in-place.

Design decisions following established conventions:
    * 0.45 mm radial clearance between ball and socket cavity
    * Socket opening radius = ball radius * (1 - 0.22), capturing ~22 %
    * ~1.5 mm wall thickness around the socket
    * Adjacent socket bulbs sized so they never overlap material with each other
      (PITCH > 2 * outer_socket_radius + small print gap)
    * Each joint allows roughly 20-25 deg of bend in any direction; with 40+
      segments that's well over a full coil of total flex
    * Spine sampled at uniform arc length so each ball lands exactly in the
      previous segment's cavity center

The dragon prints flat in the XY plane (no Z bumps), with dorsal spikes
pointing up (+Z), a horned head, and a flat horizontal tail fin.

Output is a single .stl with one solid mesh per segment - the slicer will treat
them as a print-in-place assembly.

Usage:
    python extra_long_dragon.py
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon
from trimesh import transformations as tf

# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
OUTFILE = HERE / "stl" / "extra_long_dragon.stl"

# ---------------------------------------------------------------------------
# Top-level parameters
# ---------------------------------------------------------------------------

# Chain length / shape
N_SEGMENTS = 40
X_LEN_TARGET = 560.0           # base X extent of the spine before scaling (mm)
SERPENT_AMP = 85.0             # XY serpentine amplitude (mm)
SERPENT_WAVES = 1.8            # number of full S-waves along the spine

# Ball/socket joint  (the heart of the flexi design)
R_BALL_HEAD = 6.0              # ball radius at the head end (mm)
R_BALL_TAIL = 3.8              # ball radius at the tail end (mm)
CLEARANCE = 0.45               # radial gap between ball and socket cavity (mm)
CAPTURE_FRAC = 0.22            # opening radius = r_ball * (1 - CAPTURE_FRAC)
WALL_THICK = 1.55              # socket outer wall thickness (mm)
NECK_RATIO = 0.42              # body cylinder radius = r_ball * NECK_RATIO
MIN_NECK_R = 1.5               # absolute minimum neck/cylinder radius (mm)

# Detail
ICOS_SUB = 3                   # icosphere subdivisions for spheres
CYL_SECTIONS = 18

# Decoration
SPIKE_BASE_R = 2.2
SPIKE_LEN_MAX = 8.0


# ---------------------------------------------------------------------------
# Spine curve (flat in XY plane for printability)
# ---------------------------------------------------------------------------

def curve_unit(t: np.ndarray) -> np.ndarray:
    """Spine in normalized form before arc-length resampling."""
    t = np.asarray(t, dtype=float)
    x = X_LEN_TARGET * t
    env = np.sin(np.pi * t) ** 0.20
    y = SERPENT_AMP * np.sin(2.0 * np.pi * SERPENT_WAVES * t) * env
    z = np.zeros_like(t)
    return np.column_stack([x, y, z])


def arc_length_resample(n_samples: int):
    """Resample the spine at uniform arc length intervals.

    Returns ``(spine_pts, total_arc, pitch)``.  ``pitch`` is the world-space
    distance between consecutive samples and is what the segment builder uses
    to place each segment's ball and socket on the joint connection points.
    """
    n_fine = 4000
    t_fine = np.linspace(0.0, 1.0, n_fine)
    pts_fine = curve_unit(t_fine)
    deltas = np.linalg.norm(np.diff(pts_fine, axis=0), axis=1)
    s_fine = np.concatenate([[0.0], np.cumsum(deltas)])
    total_arc = float(s_fine[-1])

    s_sample = np.linspace(0.0, total_arc, n_samples)
    sample_pts = np.zeros((n_samples, 3))
    for d in range(3):
        sample_pts[:, d] = np.interp(s_sample, s_fine, pts_fine[:, d])

    pitch = total_arc / (n_samples - 1)
    return sample_pts, total_arc, pitch


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------

def frame_transform(x_axis_world: np.ndarray, z_axis_world: np.ndarray) -> np.ndarray:
    """4x4 transform that maps local +X and +Z onto the given world axes."""
    x_w = np.asarray(x_axis_world, dtype=float)
    x_w = x_w / np.linalg.norm(x_w)
    z_w = np.asarray(z_axis_world, dtype=float)
    z_w = z_w - np.dot(z_w, x_w) * x_w
    z_w = z_w / np.linalg.norm(z_w)
    y_w = np.cross(z_w, x_w)
    R = np.eye(4)
    R[:3, 0] = x_w
    R[:3, 1] = y_w
    R[:3, 2] = z_w
    return R


def parallel_transport_frames(points: np.ndarray):
    """Tangent/normal/binormal arrays for a polyline (used by swept_tube)."""
    points = np.asarray(points, dtype=float)
    n = len(points)
    tangents = np.zeros_like(points)
    tangents[1:-1] = points[2:] - points[:-2]
    tangents[0] = points[1] - points[0]
    tangents[-1] = points[-1] - points[-2]
    tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)

    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(tangents[0], up)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    n0 = np.cross(np.cross(tangents[0], up), tangents[0])
    n0 /= np.linalg.norm(n0)

    normals = np.zeros_like(points)
    normals[0] = n0
    for i in range(1, n):
        v1, v2 = tangents[i - 1], tangents[i]
        b = np.cross(v1, v2)
        bn = np.linalg.norm(b)
        if bn < 1e-9:
            normals[i] = normals[i - 1]
        else:
            b /= bn
            theta = np.arctan2(bn, float(np.dot(v1, v2)))
            ct, st = np.cos(theta), np.sin(theta)
            np_ = normals[i - 1]
            normals[i] = (
                np_ * ct
                + np.cross(b, np_) * st
                + b * float(np.dot(b, np_)) * (1.0 - ct)
            )
            normals[i] /= np.linalg.norm(normals[i])
    binormals = np.cross(tangents, normals)
    return tangents, normals, binormals


def swept_tube(points, radii, sections: int = 16) -> trimesh.Trimesh:
    """Build a triangulated tube of variable circular cross-section."""
    pts = np.asarray(points, dtype=float)
    rs = np.asarray(radii, dtype=float)
    if rs.ndim == 0:
        rs = np.full(len(pts), float(rs))
    T, N, B = parallel_transport_frames(pts)
    angles = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    cs, sn = np.cos(angles), np.sin(angles)
    n_pts = len(pts)
    verts = np.empty((n_pts * sections, 3))
    for i in range(n_pts):
        ring = pts[i] + rs[i] * (cs[:, None] * N[i] + sn[:, None] * B[i])
        verts[i * sections:(i + 1) * sections] = ring

    faces = []
    for i in range(n_pts - 1):
        for j in range(sections):
            jn = (j + 1) % sections
            a = i * sections + j
            b = i * sections + jn
            c = (i + 1) * sections + jn
            d = (i + 1) * sections + j
            faces.append([a, b, c])
            faces.append([a, c, d])

    cs_idx = len(verts)
    verts = np.vstack([verts, pts[0:1]])
    ce_idx = len(verts)
    verts = np.vstack([verts, pts[-1:]])
    last = (n_pts - 1) * sections
    for j in range(sections):
        jn = (j + 1) % sections
        faces.append([cs_idx, jn, j])
        faces.append([ce_idx, last + j, last + jn])

    return trimesh.Trimesh(vertices=verts, faces=np.asarray(faces), process=True)


# ---------------------------------------------------------------------------
# Joint dimension profiles
# ---------------------------------------------------------------------------

def ball_radius_at(idx: int) -> float:
    """Ball radius for segment ``idx`` (taper from R_BALL_HEAD to R_BALL_TAIL)."""
    t = idx / max(1, N_SEGMENTS - 1)
    head_bulge = 1.0 + 0.18 * np.exp(-((t - 0.04) / 0.06) ** 2)
    base = R_BALL_HEAD - (R_BALL_HEAD - R_BALL_TAIL) * t
    return float(base * head_bulge)


def derived_dims(rb: float):
    """Compute (cavity, opening, outer, neck) radii for a given ball radius."""
    r_cav = rb + CLEARANCE
    r_open = rb * (1.0 - CAPTURE_FRAC)
    r_outer = r_cav + WALL_THICK
    r_neck = max(rb * NECK_RATIO, MIN_NECK_R)
    return r_cav, r_open, r_outer, r_neck


# ---------------------------------------------------------------------------
# Single segment builder
# ---------------------------------------------------------------------------

def make_segment(idx: int, pitch: float) -> tuple[trimesh.Trimesh, float]:
    """Build one articulated segment in local frame.

    Local +X = tail direction (toward next segment).  Ball at -X (head-ward,
    plugs into the previous segment's cavity).  Socket opens in +X (tail-ward).
    Origin is the joint midpoint, which is also the segment center along the
    spine.
    """
    is_first = (idx == 0)
    is_last = (idx == N_SEGMENTS - 1)

    rb = ball_radius_at(idx)
    r_cav, r_open, r_outer, r_neck = derived_dims(rb)

    parts: list[trimesh.Trimesh] = []

    # Body cylinder: spans between the ball region and the socket region.
    cyl_x_start = -pitch / 2.0 + (rb if not is_first else 0.0)
    cyl_x_end = +pitch / 2.0 - (r_outer if not is_last else 0.0)
    cyl_len = cyl_x_end - cyl_x_start
    if cyl_len > 0.4:
        cyl = trimesh.creation.cylinder(
            radius=r_neck, height=cyl_len, sections=CYL_SECTIONS
        )
        cyl.apply_transform(tf.rotation_matrix(np.pi / 2, [0, 1, 0]))
        cyl.apply_translation([(cyl_x_start + cyl_x_end) / 2.0, 0.0, 0.0])
        parts.append(cyl)

    if not is_first:
        ball = trimesh.creation.icosphere(subdivisions=ICOS_SUB, radius=rb)
        ball.apply_translation([-pitch / 2.0, 0.0, 0.0])
        parts.append(ball)
    else:
        # First segment has no ball; cap the head end so the head sculpt has
        # something to weld onto when concatenated.
        head_cap = trimesh.creation.icosphere(
            subdivisions=ICOS_SUB, radius=rb * 0.95
        )
        head_cap.apply_translation([-pitch / 2.0 + rb * 0.4, 0.0, 0.0])
        parts.append(head_cap)

    if not is_last:
        outer = trimesh.creation.icosphere(subdivisions=ICOS_SUB, radius=r_outer)
        outer.apply_translation([+pitch / 2.0, 0.0, 0.0])
        parts.append(outer)
    else:
        # Last segment has no socket; round off the tail.
        tail_cap = trimesh.creation.icosphere(
            subdivisions=ICOS_SUB, radius=r_neck * 1.8
        )
        tail_cap.apply_translation([+pitch / 2.0 - r_neck * 0.5, 0.0, 0.0])
        parts.append(tail_cap)

    shell = trimesh.boolean.union(parts, engine="manifold")

    # Carve out the socket cavity + entrance opening (only if there's a socket).
    if not is_last:
        cavity = trimesh.creation.icosphere(subdivisions=ICOS_SUB, radius=r_cav)
        cavity.apply_translation([+pitch / 2.0, 0.0, 0.0])
        opn_len = pitch * 1.6
        opening = trimesh.creation.cylinder(
            radius=r_open, height=opn_len, sections=CYL_SECTIONS
        )
        opening.apply_transform(tf.rotation_matrix(np.pi / 2, [0, 1, 0]))
        opening.apply_translation(
            [+pitch / 2.0 + opn_len / 2.0 - r_cav * 0.6, 0.0, 0.0]
        )
        cuts = trimesh.boolean.union([cavity, opening], engine="manifold")
        shell = trimesh.boolean.difference([shell, cuts], engine="manifold")

    return shell, r_outer


# ---------------------------------------------------------------------------
# Head sculpt (concatenated, in segment 0's local frame)
# ---------------------------------------------------------------------------

def build_head(scale: float, attach_x: float) -> trimesh.Trimesh:
    """Dragon head sculpt.  Built so the snout points along -X, with the back
    of the skull sitting at ``attach_x`` (= segment 0's -X end)."""
    s = scale
    parts: list[trimesh.Trimesh] = []

    # Skull bulb that overlaps segment 0's head-end cap.
    skull = trimesh.creation.icosphere(subdivisions=3, radius=11.0 * s)
    skull.apply_transform(np.diag([1.4, 1.0, 0.85, 1.0]))
    skull.apply_translation([attach_x - 4.0 * s, 0.0, 0.0])
    parts.append(skull)

    # Upper jaw / snout - tapered tube extending in -X.
    n_seg = 12
    upper_pts, upper_rs = [], []
    for i in range(n_seg):
        u = i / (n_seg - 1)
        upper_pts.append([attach_x - 8.0 * s - 22.0 * s * u, 0.0, -0.5 * s * u])
        upper_rs.append(((1 - u) * 9.0 + u * 3.5) * s)
    parts.append(swept_tube(np.array(upper_pts), np.array(upper_rs), sections=14))

    # Lower jaw - drooping a bit, mouth open.
    lower_pts, lower_rs = [], []
    for i in range(n_seg):
        u = i / (n_seg - 1)
        lower_pts.append(
            [attach_x - 8.0 * s - 22.0 * s * u, 0.0, -7.0 * s - 1.0 * s * u]
        )
        lower_rs.append(((1 - u) * 7.5 + u * 3.0) * s)
    parts.append(swept_tube(np.array(lower_pts), np.array(lower_rs), sections=12))

    # Eyes + brow ridges (left/right).
    for sgn in (-1, 1):
        eye = trimesh.creation.icosphere(subdivisions=2, radius=2.5 * s)
        eye.apply_translation([attach_x - 8.0 * s, sgn * 9.0 * s, 3.0 * s])
        parts.append(eye)
        ridge = trimesh.creation.icosphere(subdivisions=2, radius=4.0 * s)
        ridge.apply_transform(np.diag([1.5, 0.9, 0.55, 1.0]))
        ridge.apply_translation([attach_x - 6.0 * s, sgn * 8.0 * s, 5.0 * s])
        parts.append(ridge)

    # Nostril bumps near the snout tip.
    for sgn in (-1, 1):
        bump = trimesh.creation.icosphere(subdivisions=2, radius=1.6 * s)
        bump.apply_translation([attach_x - 26.0 * s, sgn * 3.0 * s, 1.0 * s])
        parts.append(bump)

    # Two main horns, sweeping back (+X = body direction) and up.
    for sgn in (-1, 1):
        n_h = 16
        horn_pts, horn_rs = [], []
        for i in range(n_h):
            u = i / (n_h - 1)
            horn_pts.append(
                [
                    attach_x + 2.0 * s + 18.0 * s * u,
                    sgn * (6.0 * s + 7.0 * s * u),
                    10.0 * s + 16.0 * s * u - 6.0 * s * u * u,
                ]
            )
            horn_rs.append((3.5 * (1 - u) + 0.5 * u) * s)
        parts.append(
            swept_tube(np.array(horn_pts), np.array(horn_rs), sections=10)
        )

    # Cheek frills - small backward-facing spikes along the jaw line.
    for sgn in (-1, 1):
        for k in range(2):
            sp = trimesh.creation.cone(
                radius=2.0 * s, height=8.0 * s, sections=8
            )
            sp.apply_transform(tf.rotation_matrix(-np.pi / 2, [0, 1, 0]))
            sp.apply_transform(
                tf.rotation_matrix(sgn * np.deg2rad(20 + 12 * k), [0, 0, 1])
            )
            sp.apply_translation(
                [attach_x - 4.0 * s - 2.0 * k * s, sgn * 9.5 * s, -2.0 * s]
            )
            parts.append(sp)

    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
# Tail fin (flat in XY plane - prints lying down)
# ---------------------------------------------------------------------------

def build_tail_fin_flat(
    pitch: float,
    length: float = 42.0,
    width: float = 26.0,
    thickness: float = 2.5,
) -> trimesh.Trimesh:
    """Horizontal leaf-shaped fin in local XY plane, attached at +pitch/2."""
    n = 28
    pts2d = []
    for i in range(n):
        u = i / (n - 1)
        pts2d.append([u * length, (width / 2.0) * np.sin(np.pi * u) ** 1.2])
    for i in range(n):
        u = 1.0 - i / (n - 1)
        pts2d.append([u * length, -(width / 2.0) * np.sin(np.pi * u) ** 1.2])

    poly = Polygon(pts2d).buffer(0)  # auto-fix any tiny self-intersection
    fin = trimesh.creation.extrude_polygon(poly, height=thickness)
    fin.apply_translation([+pitch / 2.0, 0.0, -thickness / 2.0])
    return fin


# ---------------------------------------------------------------------------
# Build the chain
# ---------------------------------------------------------------------------

print("Resampling spine to uniform arc length...")
spine_pts, total_arc, PITCH = arc_length_resample(N_SEGMENTS)
print(f"  total arc length : {total_arc:.1f} mm")
print(f"  derived PITCH    : {PITCH:.2f} mm")

# Sanity check: PITCH must be larger than 2 * largest socket outer radius +
# a small print gap, otherwise adjacent socket bulbs would overlap material.
max_r_outer = derived_dims(R_BALL_HEAD)[2]
required_pitch = 2.0 * max_r_outer + 0.4
if PITCH < required_pitch:
    raise RuntimeError(
        f"PITCH {PITCH:.2f} mm is too small for max socket outer radius "
        f"{max_r_outer:.2f} mm (need >= {required_pitch:.2f} mm). "
        f"Increase N_SEGMENTS or X_LEN_TARGET, or reduce R_BALL_HEAD."
    )

# Tangents at each segment (centered difference).
tangents = np.zeros_like(spine_pts)
tangents[1:-1] = spine_pts[2:] - spine_pts[:-2]
tangents[0] = spine_pts[1] - spine_pts[0]
tangents[-1] = spine_pts[-1] - spine_pts[-2]
tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)

# Local "up" per segment - just world +Z projected (spine is flat).
ups = np.zeros_like(spine_pts)
for i in range(N_SEGMENTS):
    u = np.array([0.0, 0.0, 1.0])
    u = u - np.dot(u, tangents[i]) * tangents[i]
    n = np.linalg.norm(u)
    ups[i] = u / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])

print(f"Building {N_SEGMENTS} articulated segments...")
segments_world: list[trimesh.Trimesh] = []
for i in range(N_SEGMENTS):
    seg, r_outer = make_segment(i, PITCH)

    # Add a dorsal spike on top of the socket bulb (only on segments that
    # have a socket - i.e. all but the last).
    if 0 < i < N_SEGMENTS - 1:
        t_norm = i / (N_SEGMENTS - 1)
        spike_h = max(SPIKE_LEN_MAX * (0.45 + 0.55 * np.sin(np.pi * t_norm))
                      * (1.0 - 0.35 * t_norm), 2.5)
        spike_r = max(SPIKE_BASE_R * (0.7 + 0.4 * (1.0 - t_norm)), 1.0)
        spike = trimesh.creation.cone(
            radius=spike_r, height=spike_h, sections=8
        )
        spike.apply_translation([+PITCH / 2.0, 0.0, r_outer * 0.92])
        try:
            seg = trimesh.boolean.union([seg, spike], engine="manifold")
        except Exception:
            seg = trimesh.util.concatenate([seg, spike])

    # Place segment in world frame.
    R = frame_transform(tangents[i], ups[i])
    seg.apply_transform(R)
    seg.apply_translation(spine_pts[i])

    segments_world.append(seg)
    if (i + 1) % 5 == 0 or i == N_SEGMENTS - 1:
        print(f"  built {i + 1}/{N_SEGMENTS}  faces={len(seg.faces):,}")

# ---------------------------------------------------------------------------
# Head (concatenated to segment 0 - rigid, prints with the head segment)
# ---------------------------------------------------------------------------

print("Building head sculpt...")
head_local = build_head(scale=0.95, attach_x=-PITCH / 2.0)
R_head = frame_transform(tangents[0], ups[0])
head_local.apply_transform(R_head)
head_local.apply_translation(spine_pts[0])
segments_world[0] = trimesh.util.concatenate([segments_world[0], head_local])

# ---------------------------------------------------------------------------
# Tail fin (concatenated to last segment)
# ---------------------------------------------------------------------------

print("Building tail fin...")
fin_local = build_tail_fin_flat(PITCH)
R_fin = frame_transform(tangents[-1], ups[-1])
fin_local.apply_transform(R_fin)
fin_local.apply_translation(spine_pts[-1])
segments_world[-1] = trimesh.util.concatenate([segments_world[-1], fin_local])

# ---------------------------------------------------------------------------
# Combine into a single STL (multiple disconnected solids = print-in-place)
# ---------------------------------------------------------------------------

print("Concatenating into final STL (multiple solids)...")
all_mesh = trimesh.util.concatenate(segments_world)

# Lift everything so the lowest point is at z=0 (sit on bed).
min_z = float(all_mesh.bounds[0, 2])
if min_z < 0:
    all_mesh.apply_translation([0.0, 0.0, -min_z + 0.02])

print()
print(f"  segments       : {N_SEGMENTS}")
print(f"  vertices       : {len(all_mesh.vertices):,}")
print(f"  faces          : {len(all_mesh.faces):,}")
print(f"  bounds (mm)    : {all_mesh.bounds}")
ext = all_mesh.bounds[1] - all_mesh.bounds[0]
print(f"  extents (mm)   : L={ext[0]:.1f}  W={ext[1]:.1f}  H={ext[2]:.1f}")
print(f"  joint clearance: {CLEARANCE} mm radial")
print(f"  joint capture  : opening = {1 - CAPTURE_FRAC:.2f} * r_ball")

os.makedirs(OUTFILE.parent, exist_ok=True)
all_mesh.export(OUTFILE)
print(f"Saved {OUTFILE}")
