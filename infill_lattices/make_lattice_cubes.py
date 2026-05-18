"""Generate cubes of mathematically interesting infill lattice structures.

Each output is a single cube specimen with its interior filled by a different
periodic / fractal lattice, exported as an STL. Three families are included:

* Triply-periodic minimal surfaces (TPMS) — sheet-like single-surface infill
  whose mean curvature is zero, defined implicitly by F(x, y, z) = 0:

      gyroid     :  sin x cos y + sin y cos z + sin z cos x = 0
      schwarz_p  :  cos x + cos y + cos z = 0
      schwarz_d  :  sin x sin y sin z + sin x cos y cos z
                      + cos x sin y cos z + cos x cos y sin z = 0
      neovius    :  3 (cos x + cos y + cos z) + 4 cos x cos y cos z = 0
      iwp        :  2 (cos x cos y + cos y cos z + cos z cos x)
                      - (cos 2x + cos 2y + cos 2z) = 0          (Schoen I-WP)
      lidinoid   :  (Lidin–Hyde — see lidinoid_F below)

  Each is thickened to a printable sheet via |F| < t.

* Strut (rod-and-node) lattices — graph of cylinders between symmetry points
  of a cubic unit cell:

      bcc            :  cell center → all 8 corners (8 struts / cell)
      fcc            :  each corner → its 3 adjacent face centers
      octet          :  fcc + the octahedron between face centers
                       (Buckminster Fuller's octet truss)
      diamond_cubic  :  two interpenetrating FCC sublattices, tetrahedral
                       bonding (the silicon / diamond crystal)

* Fractal:

      menger     :  the Menger sponge at recursion depth N (default 3)

All cubes share the same outer size so they can be compared side by side.
"""

import argparse
from pathlib import Path

import numpy as np
from skimage import measure
import trimesh


# ============================================================
# TPMS implicit functions  (k = 2π / cell_period)
# ============================================================

def gyroid_F(X, Y, Z, k):
    return (np.sin(k * X) * np.cos(k * Y)
            + np.sin(k * Y) * np.cos(k * Z)
            + np.sin(k * Z) * np.cos(k * X))


def schwarz_p_F(X, Y, Z, k):
    return np.cos(k * X) + np.cos(k * Y) + np.cos(k * Z)


def schwarz_d_F(X, Y, Z, k):
    sx, sy, sz = np.sin(k * X), np.sin(k * Y), np.sin(k * Z)
    cx, cy, cz = np.cos(k * X), np.cos(k * Y), np.cos(k * Z)
    return sx * sy * sz + sx * cy * cz + cx * sy * cz + cx * cy * sz


def neovius_F(X, Y, Z, k):
    cx, cy, cz = np.cos(k * X), np.cos(k * Y), np.cos(k * Z)
    return 3 * (cx + cy + cz) + 4 * cx * cy * cz


def iwp_F(X, Y, Z, k):
    cx, cy, cz = np.cos(k * X), np.cos(k * Y), np.cos(k * Z)
    c2x, c2y, c2z = np.cos(2 * k * X), np.cos(2 * k * Y), np.cos(2 * k * Z)
    return 2 * (cx * cy + cy * cz + cz * cx) - (c2x + c2y + c2z)


def lidinoid_F(X, Y, Z, k):
    # Lidin's approximation of the Lidinoid (single-sheet TPMS).
    sx, sy, sz = np.sin(k * X), np.sin(k * Y), np.sin(k * Z)
    cx, cy, cz = np.cos(k * X), np.cos(k * Y), np.cos(k * Z)
    s2x, s2y, s2z = np.sin(2 * k * X), np.sin(2 * k * Y), np.sin(2 * k * Z)
    c2x, c2y, c2z = np.cos(2 * k * X), np.cos(2 * k * Y), np.cos(2 * k * Z)
    t1 = 0.5 * (s2x * cy * sz + s2y * cz * sx + s2z * cx * sy)
    t2 = 0.5 * (c2x * c2y + c2y * c2z + c2z * c2x)
    return t1 - t2 + 0.15


# (function, default |F|<t threshold giving ~30% volume fraction)
TPMS_FUNCS = {
    "gyroid":    (gyroid_F,    0.55),
    "schwarz_p": (schwarz_p_F, 0.80),
    "schwarz_d": (schwarz_d_F, 0.40),
    "neovius":   (neovius_F,   0.80),
    "iwp":       (iwp_F,       1.20),
    "lidinoid":  (lidinoid_F,  0.30),
}


def tpms_field(X, Y, Z, name, cell_mm, threshold=None):
    """Implicit field for a thickened TPMS sheet: solid where |F| < threshold."""
    func, default_t = TPMS_FUNCS[name]
    if threshold is None:
        threshold = default_t
    k = 2.0 * np.pi / cell_mm
    return (np.abs(func(X, Y, Z, k)) - threshold).astype(np.float32)


# ============================================================
# Icosahedral quasicrystal
# ============================================================
#
# Aperiodic 3-D structure with the full icosahedral point group but ZERO
# translational symmetry. Built as a sum of plane waves along the six
# 5-fold axes of an icosahedron:
#
#     F(r) = Σ_{i=1..6} cos(k · aᵢ · r)
#
# where the aᵢ are the six 5-fold axes (one per antipodal vertex pair of
# the icosahedron). Because the φ-irrational components of the aᵢ are
# incommensurate with each other, F has no period along any direction
# yet inherits the icosahedron's 5-, 3-, and 2-fold rotation axes —
# producing the canonical 3-D icosahedral quasicrystal pattern observed
# in real Al-Mn / Al-Cu-Fe / Cd-Yb alloys.

_PHI = (1.0 + np.sqrt(5.0)) / 2.0
_AXIS_NORM = np.sqrt(1.0 + _PHI * _PHI)
ICOSAHEDRAL_AXES = np.array([
    (1.0,  _PHI,  0.0),
    (1.0, -_PHI,  0.0),
    (_PHI,  0.0,  1.0),
    (_PHI,  0.0, -1.0),
    (0.0,  1.0,  _PHI),
    (0.0,  1.0, -_PHI),
]) / _AXIS_NORM


def quasicrystal_F(X, Y, Z, k):
    """Icosahedral quasicrystal density field."""
    F = np.zeros_like(X, dtype=np.float32)
    for ax in ICOSAHEDRAL_AXES:
        F = F + np.cos(k * (ax[0] * X + ax[1] * Y + ax[2] * Z)).astype(np.float32)
    return F


def quasicrystal_field(X, Y, Z, scale_mm, threshold=1.20):
    """Thickened |F| < t sheet of the icosahedral quasicrystal.

    `scale_mm` is the wavelength of each constituent plane wave (analogous
    to the cell period of a TPMS). The aperiodic surface has features at
    that scale plus golden-ratio multiples thereof.
    """
    k = 2.0 * np.pi / scale_mm
    F = quasicrystal_F(X, Y, Z, k)
    return (np.abs(F) - np.float32(threshold)).astype(np.float32)


# ---- Cut-and-project vertices + edges (wireframe quasicrystal) ----
#
# Project the 6-D simple cubic integer lattice ℤ⁶ through a 3-D "physical"
# subspace whose basis vectors are the 6 five-fold axes of an icosahedron.
# The orthogonal 3-D "internal" subspace uses the algebraic conjugate
# φ → −1/φ. An integer 6-tuple n ∈ ℤ⁶ becomes a vertex of the icosahedral
# quasicrystal iff its internal-space projection P⊥·n lies inside the
# acceptance window W (a sphere in this implementation — a slightly less
# canonical but icosahedrally-symmetric stand-in for the rhombic
# triacontahedron W = P⊥·[0,1]⁶). Two vertices are joined by an edge iff
# their 6-D pre-images differ by ±eᵢ for some basis direction i.

_PHI = (1.0 + np.sqrt(5.0)) / 2.0
_SIGMA = -1.0 / _PHI  # algebraic conjugate of φ in ℚ(√5)

_P_PAR_RAW = np.array([
    [1.0, _PHI,  0.0, -1.0, _PHI,  0.0],
    [_PHI, 0.0,  1.0, _PHI,  0.0, -1.0],
    [0.0,  1.0, _PHI,  0.0, -1.0, _PHI],
])
_P_PERP_RAW = np.array([
    [1.0, _SIGMA,  0.0, -1.0, _SIGMA,  0.0],
    [_SIGMA, 0.0,  1.0, _SIGMA,  0.0, -1.0],
    [0.0,  1.0, _SIGMA,  0.0, -1.0, _SIGMA],
])


def icosahedral_quasi_lattice(cube_size_mm, edge_mm, acceptance_radius=1.5):
    """Generate vertices + edges of the canonical icosahedral quasicrystal.

    Returns:
        vertices : (N, 3) float64 array of vertex positions, in mm,
                   roughly inside the cube of side `cube_size_mm`.
        edges    : list of (i, j) tuples — vertex index pairs.
    """
    half = cube_size_mm / 2.0
    raw_basis_len = float(np.sqrt(1 + _PHI ** 2))
    proj_scale = edge_mm / raw_basis_len  # so a single ±eᵢ step has length edge_mm

    # 6-D enumeration range — large enough that every cube-interior
    # accepted vertex has its 6-D pre-image in [-max_n, max_n]⁶.
    max_n = int(np.ceil(half / edge_mm)) + 3
    rng = np.arange(-max_n, max_n + 1, dtype=np.int32)

    # Enumerate 6-D integers in chunks over the first axis to keep peak
    # memory bounded even at higher `max_n`.
    coords_accepted = []
    v_par_accepted = []
    for n0 in rng:
        g = list(np.meshgrid(*[rng] * 5, indexing="ij"))
        n_chunk = np.stack(
            [np.full_like(g[0], n0), *g], axis=-1
        ).reshape(-1, 6).astype(np.float64)
        v_perp = n_chunk @ _P_PERP_RAW.T
        mask = np.sum(v_perp * v_perp, axis=1) < acceptance_radius ** 2
        if not mask.any():
            continue
        kept = n_chunk[mask].astype(np.int32)
        coords_accepted.append(kept)
        v_par_accepted.append((kept.astype(np.float64) @ _P_PAR_RAW.T) * proj_scale)

    if not coords_accepted:
        return np.zeros((0, 3)), []

    coords = np.concatenate(coords_accepted, axis=0)
    v_par = np.concatenate(v_par_accepted, axis=0)

    # Trim to cube + a small overhang so edges crossing the boundary still
    # rasterise cleanly (the cube SDF clip handles the actual cropping).
    pad = 0.5 * edge_mm
    in_cube = np.all(np.abs(v_par) <= half + pad, axis=1)
    coords = coords[in_cube]
    v_par = v_par[in_cube]

    # Build the edge list: neighbours in ℤ⁶ that both survived acceptance.
    n_to_idx = {tuple(int(c) for c in row): i for i, row in enumerate(coords)}
    edges = []
    for i, row in enumerate(coords):
        base = tuple(int(c) for c in row)
        for ax in range(6):
            neighbour = base[:ax] + (base[ax] + 1,) + base[ax + 1:]
            j = n_to_idx.get(neighbour)
            if j is not None:
                edges.append((i, j))
    return v_par, edges


def _draw_sphere_into_sdf(best, x_coords, centre, radius, spacing):
    """In-place min-of-SDFs of a sphere over only the voxels in its bbox."""
    margin = radius + 2 * spacing
    res = len(x_coords)
    lo = np.searchsorted(x_coords, np.asarray(centre) - margin, side="left")
    hi = np.searchsorted(x_coords, np.asarray(centre) + margin, side="right")
    lo = np.clip(lo, 0, res - 1)
    hi = np.clip(hi, 1, res)
    if np.any(hi <= lo):
        return
    xs = x_coords[lo[0]:hi[0]][:, None, None]
    ys = x_coords[lo[1]:hi[1]][None, :, None]
    zs = x_coords[lo[2]:hi[2]][None, None, :]
    d = np.sqrt((xs - centre[0]) ** 2 + (ys - centre[1]) ** 2
                + (zs - centre[2]) ** 2) - radius
    target = best[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
    np.minimum(target, d.astype(np.float32), out=target)


def _draw_cylinder_into_sdf(best, x_coords, p1, p2, radius, spacing):
    """In-place min-of-SDFs of a capped-cylinder segment in its bbox."""
    margin = radius + 2 * spacing
    res = len(x_coords)
    bbox_min = np.minimum(p1, p2) - margin
    bbox_max = np.maximum(p1, p2) + margin
    lo = np.searchsorted(x_coords, bbox_min, side="left")
    hi = np.searchsorted(x_coords, bbox_max, side="right")
    lo = np.clip(lo, 0, res - 1)
    hi = np.clip(hi, 1, res)
    if np.any(hi <= lo):
        return
    xs = x_coords[lo[0]:hi[0]][:, None, None]
    ys = x_coords[lo[1]:hi[1]][None, :, None]
    zs = x_coords[lo[2]:hi[2]][None, None, :]
    AB = np.asarray(p2) - np.asarray(p1)
    L2 = float(AB @ AB)
    if L2 < 1e-12:
        return
    APx = xs - p1[0]
    APy = ys - p1[1]
    APz = zs - p1[2]
    t = (APx * AB[0] + APy * AB[1] + APz * AB[2]) / L2
    np.clip(t, 0.0, 1.0, out=t)
    cx = p1[0] + t * AB[0]
    cy = p1[1] + t * AB[1]
    cz = p1[2] + t * AB[2]
    d = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2 + (zs - cz) ** 2) - radius
    target = best[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
    np.minimum(target, d.astype(np.float32), out=target)


def _report_lattice_printability(
    vertices, edges, layer_height_mm, edge_radius_mm, max_bridge_mm,
):
    """Print an FDM printability summary for a wireframe lattice (assumes
    +Z is the print direction, i.e. the part sits on its z=-half face).

    An edge is self-supporting (no scaffolding needed) when its angle from
    horizontal exceeds `arctan(layer_height / edge_radius)`. Below that
    angle, it must either be short enough to bridge between supported
    endpoints, or it needs supports.
    """
    if len(edges) == 0:
        return
    pos_a = np.array([vertices[i] for i, _ in edges])
    pos_b = np.array([vertices[j] for _, j in edges])
    delta = pos_b - pos_a
    L = np.linalg.norm(delta, axis=1)
    horiz = np.linalg.norm(delta[:, :2], axis=1)
    # Angle from horizontal plane, ∈ [0, 90°].
    angles = np.degrees(np.arctan2(np.abs(delta[:, 2]), horiz))
    threshold = np.degrees(np.arctan2(layer_height_mm, edge_radius_mm))
    bridges = (angles < threshold) & (L <= max_bridge_mm)
    self_supp = angles >= threshold
    unprintable = ~(bridges | self_supp)
    print(f"  printability @ layer={layer_height_mm} mm, r={edge_radius_mm} mm, "
          f"max-bridge={max_bridge_mm} mm:")
    print(f"    self-support threshold: {threshold:.1f}° from horizontal")
    print(f"    min edge angle  : {angles.min():.1f}°  "
          f"max edge angle : {angles.max():.1f}°")
    print(f"    self-supporting : {int(self_supp.sum())} / {len(edges)} edges")
    print(f"    bridge (short, shallow): {int(bridges.sum())} / {len(edges)} edges")
    print(f"    UNPRINTABLE     : {int(unprintable.sum())} / {len(edges)} edges")
    if int(unprintable.sum()):
        worst = np.argsort(angles + 1e3 * (~unprintable))[: min(5, int(unprintable.sum()))]
        for k in worst:
            print(f"      • {angles[k]:.1f}° over {L[k]:.1f} mm")


def quasicrystal_lattice_field(
    X, Y, Z, cube_size_mm, edge_mm,
    vertex_radius_mm, edge_radius_mm, acceptance_radius=1.5,
    layer_height_mm=0.2, max_bridge_mm=10.0,
):
    """SDF of the cut-and-project icosahedral quasicrystal drawn as
    spheres-at-vertices + cylinders-along-edges. Also prints an FDM
    printability summary for the (default Bambu) layer height + edge
    radius."""
    vertices, edges = icosahedral_quasi_lattice(
        cube_size_mm, edge_mm, acceptance_radius=acceptance_radius,
    )
    print(f"  quasicrystal: {len(vertices)} vertices, {len(edges)} edges")
    _report_lattice_printability(
        vertices, edges, layer_height_mm, edge_radius_mm, max_bridge_mm,
    )
    x_coords = X[:, 0, 0].copy()
    spacing = float(x_coords[1] - x_coords[0])
    best = np.full(X.shape, 1.0e6, dtype=np.float32)
    for v in vertices:
        _draw_sphere_into_sdf(best, x_coords, v, vertex_radius_mm, spacing)
    for i, j in edges:
        _draw_cylinder_into_sdf(
            best, x_coords, vertices[i], vertices[j], edge_radius_mm, spacing,
        )
    return best


# ============================================================
# Strut (rod-and-node) lattices
# ============================================================

def _segment_dist(X, Y, Z, A, B):
    """Distance from each grid point to the segment AB (vectorized, float32)."""
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    AB = B - A
    L2 = float(AB @ AB)
    if L2 < 1e-12:
        return np.sqrt((X - A[0]) ** 2 + (Y - A[1]) ** 2 + (Z - A[2]) ** 2).astype(
            np.float32
        )
    APx = X - A[0]
    APy = Y - A[1]
    APz = Z - A[2]
    t = (APx * AB[0] + APy * AB[1] + APz * AB[2]) / L2
    np.clip(t, 0.0, 1.0, out=t)
    cx = A[0] + t * AB[0]
    cy = A[1] + t * AB[1]
    cz = A[2] + t * AB[2]
    return np.sqrt((X - cx) ** 2 + (Y - cy) ** 2 + (Z - cz) ** 2).astype(np.float32)


def _bcc_struts(c):
    center = np.array([c / 2, c / 2, c / 2])
    corners = [np.array([sx * c, sy * c, sz * c])
               for sx in (0, 1) for sy in (0, 1) for sz in (0, 1)]
    return [(center, corner) for corner in corners]


def _face_centers(c):
    out = []
    for ax in (0, 1, 2):
        for s in (0, 1):
            p = [c / 2, c / 2, c / 2]
            p[ax] = s * c
            out.append(np.array(p))
    return out


def _fcc_struts(c):
    """Each corner connects to its 3 adjacent face centers (length c/√2)."""
    corners = [np.array([sx * c, sy * c, sz * c])
               for sx in (0, 1) for sy in (0, 1) for sz in (0, 1)]
    face_centers = _face_centers(c)
    target = c / np.sqrt(2)
    struts = []
    for corner in corners:
        for fc in face_centers:
            if abs(np.linalg.norm(corner - fc) - target) < 1e-6:
                struts.append((corner, fc))
    return struts


def _octet_struts(c):
    """Octet truss = FCC corner-to-face struts + central octahedron edges."""
    struts = _fcc_struts(c)
    face_centers = _face_centers(c)
    target = c / np.sqrt(2)
    for i, fc1 in enumerate(face_centers):
        for fc2 in face_centers[i + 1:]:
            if abs(np.linalg.norm(fc1 - fc2) - target) < 1e-6:
                struts.append((fc1, fc2))
    return struts


def _diamond_cubic_struts(c):
    """Diamond cubic: two FCC sublattices offset by (c/4, c/4, c/4)."""
    A_nodes = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, c / 2, c / 2]),
        np.array([c / 2, 0.0, c / 2]),
        np.array([c / 2, c / 2, 0.0]),
    ]
    B_offset = np.array([c / 4, c / 4, c / 4])
    B_nodes = [a + B_offset for a in A_nodes]
    bond_len = c * np.sqrt(3) / 4
    struts = []
    seen = set()
    for a in A_nodes:
        for b in B_nodes:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        b_shift = b + np.array([dx * c, dy * c, dz * c])
                        if abs(np.linalg.norm(a - b_shift) - bond_len) < 1e-6:
                            key = tuple(
                                sorted([tuple(np.round(a, 4)),
                                        tuple(np.round(b_shift, 4))])
                            )
                            if key not in seen:
                                seen.add(key)
                                struts.append((a, b_shift))
    return struts


STRUT_LATTICES = {
    "bcc": _bcc_struts,
    "fcc": _fcc_struts,
    "octet": _octet_struts,
    "diamond_cubic": _diamond_cubic_struts,
}


def strut_field(X, Y, Z, name, cell_mm, strut_radius_mm):
    """SDF (in mm) for a periodic strut lattice: negative inside the rods.

    The lattice is exactly periodic with period `cell_mm`, so we evaluate the
    distance-to-nearest-strut on ONE unit cell at sub-grid resolution and then
    sample that cached field at every point of the full grid via (x mod c).
    With ~30 sub-voxels per cell this is two orders of magnitude faster than
    evaluating every strut over the entire cube grid.
    """
    struts = STRUT_LATTICES[name](cell_mm)
    # Sub-grid resolution per cell: match the full grid's spacing so that
    # nearest-neighbour sampling of the cached field is exact.
    spacing = float(X[1, 0, 0] - X[0, 0, 0])
    sub_res = max(24, int(np.ceil(cell_mm / spacing)))
    cs = (np.arange(sub_res, dtype=np.float32) + 0.5) * (cell_mm / sub_res)
    Xs, Ys, Zs = np.meshgrid(cs, cs, cs, indexing="ij")
    best = np.full_like(Xs, np.inf, dtype=np.float32)
    # Check the 27 candidate neighbour shifts: every strut is contained in
    # its unit cell, so any strut whose nearest point to a query lives within
    # one cell of [0, c)^3 sits in one of these shifts.
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                shift = np.array([dx * cell_mm, dy * cell_mm, dz * cell_mm])
                for A, B in struts:
                    d = _segment_dist(Xs, Ys, Zs, A + shift, B + shift)
                    np.minimum(best, d, out=best)
    # Sample cached SDF at the full grid points (nearest-neighbour, exact
    # because sub_res was chosen to match the full grid spacing).
    Xw = np.mod(X, cell_mm)
    Yw = np.mod(Y, cell_mm)
    Zw = np.mod(Z, cell_mm)
    ix = np.clip((Xw / cell_mm * sub_res).astype(np.int32), 0, sub_res - 1)
    iy = np.clip((Yw / cell_mm * sub_res).astype(np.int32), 0, sub_res - 1)
    iz = np.clip((Zw / cell_mm * sub_res).astype(np.int32), 0, sub_res - 1)
    return best[ix, iy, iz] - np.float32(strut_radius_mm)


# ============================================================
# Crystal lattices — atomic ball-and-stick representations
# ============================================================
#
# Each builder returns (atoms, bonds) where
#     atoms = list of (centre_xyz_in_unit_cell_mm, sphere_radius_mm)
#     bonds = list of ((endpoint_a, endpoint_b), cylinder_radius_mm)
# Radii are chosen so that nearest neighbours overlap a little (so the cube
# prints as one connected piece). For each crystal the geometry mirrors a
# real ionic / covalent compound; the cell is treated as cubic with period
# `cell_mm` so the same wrap-then-27-shift pipeline used for the strut
# lattices also handles crystals exactly.

def rock_salt_crystal(c):
    """NaCl rock salt — two interpenetrating FCC sub-lattices.

    Each ion sits on a simple-cubic grid of period c/2, with type
    alternating by parity. Every Na+ has six Cl- nearest neighbours at
    distance c/2 (the iconic six-coordinated ionic crystal).
    """
    half = c / 2
    # Sum of radii just exceeds c/2 so the cube prints as one connected
    # piece, but stays small enough (~0.1 mm overlap at default c = 10 mm)
    # that adjacent ions read as distinct spheres rather than a packed slab.
    # Asymmetric ratio (~7 : 10) is the whole visual point of NaCl.
    r_cation = c * 0.21   # "Na+" — smaller
    r_anion = c * 0.30    # "Cl-" — larger; r_cation + r_anion > c/2 ⇒ touching
    atoms = []
    for ix in (0, 1):
        for iy in (0, 1):
            for iz in (0, 1):
                pos = np.array([ix * half, iy * half, iz * half])
                odd_parity = (ix + iy + iz) % 2
                radius = r_anion if odd_parity else r_cation
                atoms.append((pos, radius))
    return atoms, []


def fluorite_crystal(c):
    """Fluorite CaF₂ — FCC cations + all 8 tetrahedral-hole anions.

    Each F- has 4 Ca²⁺ nearest neighbours at distance a√3⁄4; each Ca²⁺
    is 8-coordinated by F-. Drawn here as a ball-and-stick with explicit
    Ca-F bonds (cylinders along the 8 tetrahedral directions) so the
    atoms can be small enough to read as distinct without needing the
    sphere-overlap density to keep the cube connected.
    """
    h = c / 2
    q = c / 4
    r_ca = c * 0.16
    r_f = c * 0.11
    bond_radius = c * 0.045
    bond_len = c * np.sqrt(3) / 4
    atoms = []
    ca_positions = [np.array(p, dtype=np.float64)
                    for p in [(0, 0, 0), (h, h, 0), (h, 0, h), (0, h, h)]]
    for pos in ca_positions:
        atoms.append((pos, r_ca))
    f_positions = []
    for ix in (1, 3):
        for iy in (1, 3):
            for iz in (1, 3):
                p = np.array([ix * q, iy * q, iz * q])
                f_positions.append(p)
                atoms.append((p, r_f))
    # Ca-F bonds: every Ca sees 8 F at distance a√3⁄4; iterate periodic copies.
    bonds = []
    seen = set()
    for ca in ca_positions:
        for f in f_positions:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        f_shift = f + np.array([dx * c, dy * c, dz * c])
                        if abs(np.linalg.norm(ca - f_shift) - bond_len) < 1e-6:
                            key = tuple(sorted([
                                tuple(np.round(ca, 4)),
                                tuple(np.round(f_shift, 4)),
                            ]))
                            if key not in seen:
                                seen.add(key)
                                bonds.append(((ca, f_shift), bond_radius))
    return atoms, bonds


def diamond_crystal_lattice(c):
    """Carbon diamond — ball-and-stick form of the diamond cubic lattice.

    Eight carbon atoms per conventional cell (two interpenetrating FCC
    sub-lattices offset by (c/4, c/4, c/4)); each atom sp³-bonded to its
    four tetrahedral neighbours at bond length a√3⁄4 and the classic
    109.47° tetrahedral angle.
    """
    A_nodes = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, c / 2, c / 2]),
        np.array([c / 2, 0.0, c / 2]),
        np.array([c / 2, c / 2, 0.0]),
    ]
    B_offset = np.array([c / 4, c / 4, c / 4])
    B_nodes = [a + B_offset for a in A_nodes]
    atom_radius = c * 0.16
    bond_radius = c * 0.065
    bond_len = c * np.sqrt(3) / 4
    atoms = [(np.asarray(p), atom_radius) for p in A_nodes + B_nodes]
    bonds = []
    seen = set()
    for a in A_nodes:
        for b in B_nodes:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        b_shift = b + np.array([dx * c, dy * c, dz * c])
                        if abs(np.linalg.norm(a - b_shift) - bond_len) < 1e-6:
                            key = tuple(sorted([
                                tuple(np.round(a, 4)),
                                tuple(np.round(b_shift, 4)),
                            ]))
                            if key not in seen:
                                seen.add(key)
                                bonds.append(((a, b_shift), bond_radius))
    return atoms, bonds


CRYSTAL_LATTICES = {
    "rock_salt": rock_salt_crystal,
    "fluorite": fluorite_crystal,
    "diamond_crystal": diamond_crystal_lattice,
}


def crystal_field(X, Y, Z, name, cell_mm):
    """SDF (mm) for a crystalline ball-and-stick lattice. Uses the same
    cache-the-unit-cell-then-tile trick as `strut_field`.

    We apply a uniform (c/4, c/4, c/4) phase shift to every atom + bond so
    the outer cube's faces (at multiples of c/2 from the cube centre) don't
    cleanly bisect any ion — which otherwise produces ugly half-sphere
    dimples instead of clean inter-atomic gaps on the cube surface.
    """
    atoms, bonds = CRYSTAL_LATTICES[name](cell_mm)
    phase = np.array([cell_mm * 0.25, cell_mm * 0.25, cell_mm * 0.25])
    atoms = [(np.asarray(pos) + phase, r) for pos, r in atoms]
    bonds = [((np.asarray(A) + phase, np.asarray(B) + phase), r)
             for (A, B), r in bonds]
    spacing = float(X[1, 0, 0] - X[0, 0, 0])
    sub_res = max(40, int(np.ceil(cell_mm / spacing)))
    cs = (np.arange(sub_res, dtype=np.float32) + 0.5) * (cell_mm / sub_res)
    Xs, Ys, Zs = np.meshgrid(cs, cs, cs, indexing="ij")
    best = np.full_like(Xs, np.inf, dtype=np.float32)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                shift = np.array([dx * cell_mm, dy * cell_mm, dz * cell_mm])
                for pos, rad in atoms:
                    cx, cy, cz = pos + shift
                    d = (np.sqrt((Xs - cx) ** 2 + (Ys - cy) ** 2 + (Zs - cz) ** 2)
                         .astype(np.float32) - np.float32(rad))
                    np.minimum(best, d, out=best)
                for (A, B), rad in bonds:
                    d = _segment_dist(Xs, Ys, Zs, A + shift, B + shift) - np.float32(rad)
                    np.minimum(best, d, out=best)
    Xw = np.mod(X, cell_mm)
    Yw = np.mod(Y, cell_mm)
    Zw = np.mod(Z, cell_mm)
    ix = np.clip((Xw / cell_mm * sub_res).astype(np.int32), 0, sub_res - 1)
    iy = np.clip((Yw / cell_mm * sub_res).astype(np.int32), 0, sub_res - 1)
    iz = np.clip((Zw / cell_mm * sub_res).astype(np.int32), 0, sub_res - 1)
    return best[ix, iy, iz]


# ============================================================
# Menger sponge (fractal)
# ============================================================

def menger_field(X, Y, Z, cube_size_mm, level):
    """Menger sponge: at each of `level` iterations remove sub-cubes whose
    (i,j,k) ternary digit triple has two or more '1's."""
    half = cube_size_mm / 2
    u = (X + half) / cube_size_mm
    v = (Y + half) / cube_size_mm
    w = (Z + half) / cube_size_mm
    inside = (u >= 0) & (u <= 1) & (v >= 0) & (v <= 1) & (w >= 0) & (w <= 1)
    for lvl in range(level):
        scale = 3 ** lvl
        eps = 1e-9
        a = np.floor(np.clip(u * 3 * scale, 0, 3 * scale - eps)).astype(np.int32) % 3
        b = np.floor(np.clip(v * 3 * scale, 0, 3 * scale - eps)).astype(np.int32) % 3
        c = np.floor(np.clip(w * 3 * scale, 0, 3 * scale - eps)).astype(np.int32) % 3
        removed = ((a == 1).astype(np.int8)
                   + (b == 1).astype(np.int8)
                   + (c == 1).astype(np.int8)) >= 2
        inside &= ~removed
    # Marching cubes needs a sign change at the boundary; ±1 around 0 works.
    return np.where(inside, -1.0, 1.0).astype(np.float32)


# ============================================================
# Cube clipping + mesh extraction
# ============================================================

def cube_sdf(X, Y, Z, half_size):
    dx = np.abs(X) - half_size
    dy = np.abs(Y) - half_size
    dz = np.abs(Z) - half_size
    outside = np.sqrt(
        np.maximum(dx, 0) ** 2 + np.maximum(dy, 0) ** 2 + np.maximum(dz, 0) ** 2
    )
    inside = np.minimum(np.maximum(np.maximum(dx, dy), dz), 0)
    return (outside + inside).astype(np.float32)


def make_grid(half_extent, resolution):
    x = np.linspace(-half_extent, half_extent, resolution).astype(np.float32)
    X, Y, Z = np.meshgrid(x, x, x, indexing="ij")
    spacing = (2 * half_extent) / (resolution - 1)
    return X, Y, Z, spacing


def field_to_mesh(field, spacing, half_extent):
    """Marching cubes + light cleanup. Skips the expensive split-by-component
    pass: the cube-clip ensures one big connected lattice, and marching cubes
    already produces a manifold mesh that trimesh's `process=True` handles."""
    verts, faces, _, _ = measure.marching_cubes(
        field, level=0.0, spacing=(spacing, spacing, spacing)
    )
    verts -= half_extent
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    return mesh


# ============================================================
# Driver
# ============================================================

LATTICE_INFO = {
    "gyroid":          ("tpms",    "Schoen's gyroid"),
    "schwarz_p":       ("tpms",    "Schwarz primitive (P)"),
    "schwarz_d":       ("tpms",    "Schwarz diamond (D)"),
    "neovius":         ("tpms",    "Neovius surface"),
    "iwp":             ("tpms",    "Schoen I-WP"),
    "lidinoid":        ("tpms",    "Lidinoid"),
    "bcc":             ("strut",   "Body-centred cubic struts"),
    "fcc":             ("strut",   "Face-centred cubic struts"),
    "octet":           ("strut",   "Octet truss"),
    "diamond_cubic":   ("strut",   "Diamond cubic crystal"),
    "rock_salt":       ("crystal", "Rock salt (NaCl)"),
    "fluorite":        ("crystal", "Fluorite (CaF\u2082)"),
    "diamond_crystal": ("crystal", "Diamond crystal (C, ball+stick)"),
    "quasicrystal":          ("quasi", "Icosahedral quasicrystal (TPMS-like)"),
    "quasicrystal_lattice":  ("quasi", "Icosahedral quasicrystal (vertices + edges)"),
    "menger":          ("fractal", "Menger sponge"),
}


def make_lattice_cube(name, cube_size_mm, cell_mm, strut_radius_mm,
                      tpms_threshold, menger_level, resolution, outfile):
    half = cube_size_mm / 2
    padding = max(cell_mm * 0.1, 1.0)
    extent = half + padding
    X, Y, Z, spacing = make_grid(extent, resolution)

    category, _ = LATTICE_INFO[name]
    if category == "tpms":
        latt = tpms_field(X, Y, Z, name, cell_mm, tpms_threshold)
    elif category == "strut":
        latt = strut_field(X, Y, Z, name, cell_mm, strut_radius_mm)
    elif category == "crystal":
        latt = crystal_field(X, Y, Z, name, cell_mm)
    elif category == "quasi":
        if name == "quasicrystal_lattice":
            # Radii chosen so that with Bambu's 0.2 mm layer height, the
            # self-support cone (arctan(0.2 / r)) is below the lattice's
            # shallowest non-horizontal edge angle (31.7° = arctan(1/φ)).
            # With r = 0.6 mm the threshold is 18.4°, giving ~ 13° margin.
            # The horizontal edges (φ-axis family) bridge in 6 mm — well
            # within Bambu's comfortable 10 mm bridging range.
            latt = quasicrystal_lattice_field(
                X, Y, Z, cube_size_mm,
                edge_mm=cell_mm * 0.60,
                vertex_radius_mm=strut_radius_mm * 1.00,
                edge_radius_mm=strut_radius_mm * 0.55,
            )
        else:
            latt = quasicrystal_field(
                X, Y, Z, cell_mm,
                threshold=1.20 if tpms_threshold is None else tpms_threshold,
            )
    elif category == "fractal":
        latt = menger_field(X, Y, Z, cube_size_mm, menger_level)
    else:
        raise ValueError(f"Unknown category for {name!r}")

    cube = cube_sdf(X, Y, Z, half)
    field = np.maximum(latt, cube)

    mesh = field_to_mesh(field, spacing, extent)

    # For the wireframe quasicrystal, cube clipping leaves small isolated
    # fragments where edges graze the boundary — fine to look at, but they
    # won't stick to a printer's build plate. Keep only the largest
    # connected component for that variant.
    if name == "quasicrystal_lattice":
        components = mesh.split(only_watertight=False)
        if len(components) > 1:
            components = sorted(components, key=lambda m: -len(m.faces))
            dropped_faces = sum(len(c.faces) for c in components[1:])
            print(f"  dropped {len(components) - 1} disconnected fragment(s) "
                  f"({dropped_faces} face(s) total) — keeping main lattice")
            mesh = components[0]

    Path(outfile).parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(outfile))
    return mesh


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    choices = list(LATTICE_INFO.keys())
    p.add_argument("--lattices", nargs="+", default=choices,
                   choices=choices + ["all"],
                   help="Which lattices to generate (default: every one).")
    p.add_argument("--outdir", default="stl")
    p.add_argument("--cube-mm", type=float, default=40.0,
                   help="Outer cube side length, mm.")
    p.add_argument("--cell-mm", type=float, default=10.0,
                   help="Unit cell period for periodic lattices, mm.")
    p.add_argument("--strut-radius-mm", type=float, default=1.1,
                   help="Cylinder radius for strut lattices, mm.")
    p.add_argument("--tpms-threshold", type=float, default=None,
                   help="Override |F| < t threshold (otherwise tuned per-TPMS).")
    p.add_argument("--menger-level", type=int, default=3,
                   help="Menger sponge recursion depth.")
    p.add_argument("--resolution", type=int, default=160,
                   help="Voxels per axis for marching cubes (bump to 240 for "
                        "smoother surfaces, drop to 120 for faster iteration).")
    return p.parse_args()


def main():
    args = parse_args()
    lattices = args.lattices
    if "all" in lattices:
        lattices = list(LATTICE_INFO.keys())

    for name in lattices:
        _, desc = LATTICE_INFO[name]
        outfile = Path(args.outdir) / f"{name}_cube.stl"
        print(f"\n=== {name}  —  {desc} ===")
        mesh = make_lattice_cube(
            name=name,
            cube_size_mm=args.cube_mm,
            cell_mm=args.cell_mm,
            strut_radius_mm=args.strut_radius_mm,
            tpms_threshold=args.tpms_threshold,
            menger_level=args.menger_level,
            resolution=args.resolution,
            outfile=outfile,
        )
        print(f"  faces       : {len(mesh.faces):,}")
        bmin, bmax = mesh.bounds
        print(f"  bounds (mm) : [{bmin[0]:.2f}, {bmin[1]:.2f}, {bmin[2]:.2f}]"
              f"  →  [{bmax[0]:.2f}, {bmax[1]:.2f}, {bmax[2]:.2f}]")
        print(f"  watertight  : {mesh.is_watertight}")
        print(f"  volume mm^3 : {mesh.volume:.1f}"
              f"  ({100 * mesh.volume / args.cube_mm**3:.1f}% fill)")
        print(f"  → {outfile}")


if __name__ == "__main__":
    main()
