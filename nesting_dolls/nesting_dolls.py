"""Printable Russian nesting dolls (classic split-half matryoshka).

Sizes a nest of hollow egg-shaped dolls for a Bambu Lab H2D
(325 x 320 x 325 mm), each splitting into top + bottom halves with a
press-fit lip at the waist. Exports STLs and a nestability report.

Usage:
    ./run.sh nesting_dolls/nesting_dolls.py
    ./run.sh nesting_dolls/nesting_dolls.py --report-only
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass

import numpy as np
import trimesh


# ---------------------------------------------------------------------------
# Printer + design defaults (mm)
# ---------------------------------------------------------------------------

BED_X_MM = 325.0
BED_Y_MM = 320.0
BED_Z_MM = 325.0

H0_MM = 300.0
D0_MM = 160.0

# Full-size joint features (used at D >= D_FULL_MM). Below that they
# lerp down toward printable floors so small dolls keep a usable bore.
WALL_MM = 1.2
GAP_RADIAL_MM = 0.6
GAP_AXIAL_MM = 0.6

# Radial play between male lip OD and female socket ID.
# ~0.12 mm is a snug FDM press-fit; 0.25 felt loose on print.
LIP_CLEARANCE_MM = 0.12
# Outer skirt of the hat around the female socket. Too thin and the
# socket wall snaps off the body when handling — keep this chunky.
RIM_OUTER_MM = 1.8
WAIST_FRAC = 0.55  # split height as fraction of outer height (from bottom)

H_MIN_MM = 12.0
D_MIN_MM = 8.0
LIP_H_MIN_MM = 1.5
LIP_T_MIN_MM = 0.8

# Feature scaling: full at D_FULL, floors near D_MIN.
D_FULL_MM = 80.0
WALL_FLOOR_MM = 0.50
RIM_FLOOR_MM = 1.0
LIP_T_FLOOR_MM = 0.50
LIP_H_FLOOR_MM = 1.0
CLEAR_FLOOR_MM = 0.08  # still some play on tiny dolls
GAP_FLOOR_MM = 0.35

# Tight sensitivity (report only; does not change exported set)
TIGHT_WALL_MM = 0.8
TIGHT_GAP_MM = 0.4
TIGHT_LIP_T_MIN_MM = 0.55  # ~2 perimeters @ 0.4 mm nozzle

PLA_DENSITY_G_PER_MM3 = 1.24e-3  # ~1.24 g/cm^3

# Flat sitting base as a fraction of max outer radius (so base diameter =
# 2 * BASE_R_NORM * (D/2) = BASE_R_NORM * D). Classic matryoshka stance.
BASE_R_NORM = 0.42

# Keep moderate — very dense revolves go non-manifold on tiny dolls.
PROFILE_SAMPLES = 48
REVOLVE_SECTIONS = 48


def _mesh_res(diameter: float) -> tuple[int, int]:
    """Profile samples and revolve sections scaled to doll size."""
    if diameter >= 60.0:
        return 64, 48
    if diameter >= 20.0:
        return 48, 40
    return 32, 32

_HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_HERE, "stl")


# ---------------------------------------------------------------------------
# Egg silhouette
# ---------------------------------------------------------------------------

def egg_radius_norm(t: np.ndarray) -> np.ndarray:
    """Unit-max egg radius vs normalized height t in [0, 1] (0 = bottom).

    Bulbous lower body with a flat sitting base, tapering crown — classic
    matryoshka silhouette. Max radius sits near t ≈ 0.38. At t=0 the
    radius is BASE_R_NORM (not a point), so the revolved shell has a
    circular face that sits on the table.
    """
    t = np.asarray(t, dtype=float)
    # Two half-superellipses joined near the max-radius station.
    peak = 0.38
    r = np.empty_like(t)
    lower = t <= peak
    # Bottom: start at the flat-base radius and rise to the bulge.
    u = np.clip(t[lower] / peak, 0.0, 1.0)
    r_curve = (1.0 - (1.0 - u) ** 2.4) ** 0.55
    r[lower] = BASE_R_NORM + (1.0 - BASE_R_NORM) * r_curve
    # Top: circular dome (hemisphere in normalized v) — round head, not a spike.
    v = np.clip((t[~lower] - peak) / (1.0 - peak), 0.0, 1.0)
    r[~lower] = np.sqrt(np.maximum(1.0 - v * v, 0.0))
    r = np.maximum(r, 1e-4)
    # Tiny apex for a clean revolve (keep the flat base at t=0).
    if t.size >= 3 and t[-1] >= 1.0 - 1e-9:
        r[-1] = 1e-4
    return r


def egg_outer_radius(z: np.ndarray, height: float, diameter: float) -> np.ndarray:
    t = np.clip(np.asarray(z, dtype=float) / max(height, 1e-9), 0.0, 1.0)
    return 0.5 * diameter * egg_radius_norm(t)


# ---------------------------------------------------------------------------
# Nest sizing
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class JointFeatures:
    wall: float
    rim: float
    lip_t: float
    lip_h: float
    clear: float
    gap_radial: float
    gap_axial: float


@dataclass(frozen=True)
class DollSize:
    index: int
    height: float
    diameter: float
    wall: float
    waist_z: float
    lip_height: float
    lip_radial: float
    clear: float
    rim: float
    hollow: bool
    cavity_height: float
    cavity_diameter: float

    @property
    def mass_g_est(self) -> float:
        """Rough shell mass: outer egg volume minus cavity (or solid if !hollow)."""
        outer_v = _egg_volume(self.height, self.diameter)
        if not self.hollow:
            return outer_v * PLA_DENSITY_G_PER_MM3
        # Approximate cavity as similar egg inset by wall on all sides.
        inner_h = max(self.height - 2.0 * self.wall, 0.0)
        inner_d = max(self.diameter - 2.0 * self.wall, 0.0)
        inner_v = _egg_volume(inner_h, inner_d) if inner_h > 0 and inner_d > 0 else 0.0
        return max(outer_v - inner_v, 0.0) * PLA_DENSITY_G_PER_MM3


def _egg_volume(height: float, diameter: float, samples: int = 256) -> float:
    """Solid of revolution volume via disk method."""
    if height <= 0 or diameter <= 0:
        return 0.0
    z = np.linspace(0.0, height, samples)
    r = egg_outer_radius(z, height, diameter)
    # trapz of pi r^2 dz
    area = math.pi * r * r
    trapz = getattr(np, "trapezoid", None) or np.trapz
    return float(trapz(area, z))


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def joint_features(
    height: float,
    diameter: float,
    *,
    wall_full: float = WALL_MM,
    gap_radial_full: float = GAP_RADIAL_MM,
    gap_axial_full: float = GAP_AXIAL_MM,
    clear_full: float = LIP_CLEARANCE_MM,
    rim_full: float = RIM_OUTER_MM,
    lip_t_full: float = LIP_T_MIN_MM,
) -> JointFeatures:
    """Wall/joint dims for this doll — full-size above D_FULL, floors near D_MIN."""
    u = float(np.clip((diameter - D_MIN_MM) / (D_FULL_MM - D_MIN_MM), 0.0, 1.0))
    wall = _lerp(WALL_FLOOR_MM, wall_full, u)
    rim = _lerp(RIM_FLOOR_MM, rim_full, u)
    lip_t = _lerp(LIP_T_FLOOR_MM, lip_t_full, u)
    # Lip height tracks doll height but floors/ceils with scale.
    lip_h_floor = _lerp(LIP_H_FLOOR_MM, LIP_H_MIN_MM, u)
    lip_h = max(0.04 * height, lip_h_floor)
    clear = _lerp(CLEAR_FLOOR_MM, clear_full, u)
    gap_r = _lerp(GAP_FLOOR_MM, gap_radial_full, u)
    gap_a = _lerp(GAP_FLOOR_MM, gap_axial_full, u)
    return JointFeatures(wall, rim, lip_t, lip_h, clear, gap_r, gap_a)


def cavity_dims(height: float, diameter: float, wall: float) -> tuple[float, float]:
    """Inner cavity height / max diameter for a hollow egg of given outer size."""
    return height - 2.0 * wall, diameter - 2.0 * wall


def waist_opening_radius(height: float, diameter: float, feat: JointFeatures) -> float:
    """Clear bore radius at the split — what an inner doll must pass through."""
    waist = WAIST_FRAC * height
    r_waist = float(egg_outer_radius(np.array([waist]), height, diameter)[0])
    lip_outer_r = r_waist - feat.rim - feat.clear
    return max(lip_outer_r - feat.lip_t, 0.0)


def can_be_hollow(height: float, diameter: float, feat: JointFeatures) -> bool:
    """True if wall + joint still leave a usable nesting bore and cavity."""
    open_r = waist_opening_radius(height, diameter, feat)
    if open_r < 1.5:
        return False
    cav_h, cav_d = cavity_dims(height, diameter, feat.wall)
    if cav_h < 2.0 * feat.wall or cav_d < 2.0 * feat.wall:
        return False
    r_waist = float(
        egg_outer_radius(np.array([WAIST_FRAC * height]), height, diameter)[0]
    )
    # Need room for rim + clearance + lip + a little bore.
    return r_waist >= feat.rim + feat.clear + feat.lip_t + 1.5


def is_printable_doll(
    height: float,
    diameter: float,
    feat: JointFeatures,
    hollow: bool,
) -> tuple[bool, str]:
    if height < H_MIN_MM:
        return False, f"height {height:.2f} < H_min {H_MIN_MM}"
    if diameter < D_MIN_MM:
        return False, f"diameter {diameter:.2f} < D_min {D_MIN_MM}"
    if hollow and feat.wall < WALL_FLOOR_MM - 1e-9:
        return False, f"wall {feat.wall:.2f} < floor {WALL_FLOOR_MM}"
    if feat.lip_h < LIP_H_FLOOR_MM - 1e-9:
        return False, f"lip height {feat.lip_h:.2f} < {LIP_H_FLOOR_MM}"
    if feat.lip_t < LIP_T_FLOOR_MM - 1e-9:
        return False, f"lip radial {feat.lip_t:.2f} < {LIP_T_FLOOR_MM}"
    # Each half (plus lip) must fit the H2D bed when printed cut-face down.
    waist = WAIST_FRAC * height
    bottom_h = waist + feat.lip_h
    top_h = height - waist + feat.lip_h
    half_xy = diameter
    for label, hz in (("bottom", bottom_h), ("top", top_h)):
        if half_xy > BED_X_MM or half_xy > BED_Y_MM or hz > BED_Z_MM:
            return False, f"{label} half {half_xy:.1f}x{half_xy:.1f}x{hz:.1f} exceeds bed"
    if hollow and not can_be_hollow(height, diameter, feat):
        return False, "joint/cavity too tight for hollow shell"
    return True, "ok"


def similar_next_size(
    height: float,
    diameter: float,
    feat: JointFeatures,
) -> tuple[float, float]:
    """Next outer size: same H/D aspect, able to drop in through the waist."""
    aspect = height / diameter
    cav_h, _cav_d = cavity_dims(height, diameter, feat.wall)
    max_h = cav_h - 2.0 * feat.gap_axial
    open_r = waist_opening_radius(height, diameter, feat) - feat.gap_radial
    max_d = 2.0 * open_r
    d_from_h = max_h / aspect
    h_from_d = max_d * aspect
    if d_from_h <= max_d:
        return max_h, d_from_h
    return h_from_d, max_d


def _make_doll(index: int, height: float, diameter: float, hollow: bool) -> DollSize:
    feat = joint_features(height, diameter)
    if hollow:
        cav_h, cav_d = cavity_dims(height, diameter, feat.wall)
        wall = feat.wall
    else:
        cav_h, cav_d = 0.0, 0.0
        wall = height * 0.5
    return DollSize(
        index=index,
        height=height,
        diameter=diameter,
        wall=wall,
        waist_z=WAIST_FRAC * height,
        lip_height=feat.lip_h,
        lip_radial=feat.lip_t,
        clear=feat.clear,
        rim=feat.rim,
        hollow=hollow,
        cavity_height=cav_h,
        cavity_diameter=cav_d,
    )


def size_nest(
    h0: float = H0_MM,
    d0: float = D0_MM,
    *,
    wall_full: float = WALL_MM,
    gap_radial_full: float = GAP_RADIAL_MM,
    gap_axial_full: float = GAP_AXIAL_MM,
) -> tuple[list[DollSize], str]:
    """Largest→smallest nest of similar eggs. Returns (dolls, stop_reason)."""
    dolls: list[DollSize] = []
    h, d = h0, d0
    stop = "unknown"

    def feats(hh: float, dd: float) -> JointFeatures:
        return joint_features(
            hh,
            dd,
            wall_full=wall_full,
            gap_radial_full=gap_radial_full,
            gap_axial_full=gap_axial_full,
        )

    while True:
        feat = feats(h, d)
        hollow = can_be_hollow(h, d, feat)
        ok, reason = is_printable_doll(h, d, feat, hollow)
        if not ok:
            stop = reason
            break

        dolls.append(_make_doll(len(dolls), h, d, hollow))

        if not hollow:
            stop = "innermost solid doll reached"
            break

        next_h, next_d = similar_next_size(h, d, feat)
        next_feat = feats(next_h, next_d)

        if is_printable_doll(next_h, next_d, next_feat, hollow=True)[0]:
            h, d = next_h, next_d
            continue

        # Fall back to a solid core that still fits through this opening.
        hollow_reason = is_printable_doll(next_h, next_d, next_feat, True)[1]
        ok_s, _ = is_printable_doll(next_h, next_d, next_feat, hollow=False)
        if ok_s:
            dolls.append(_make_doll(len(dolls), next_h, next_d, hollow=False))
            stop = f"next hollow failed ({hollow_reason}); added solid innermost"
        else:
            stop = hollow_reason
        break

    return dolls, stop


def verify_nest(dolls: list[DollSize]) -> None:
    """Assert nest clearances and bed fit; raise AssertionError on failure."""
    assert len(dolls) >= 2, f"nest count {len(dolls)} < 2"

    for doll in dolls:
        lip_h = doll.lip_height
        bottom_h = doll.waist_z + lip_h
        top_h = doll.height - doll.waist_z + lip_h
        assert doll.diameter <= BED_X_MM + 1e-6
        assert doll.diameter <= BED_Y_MM + 1e-6
        assert bottom_h <= BED_Z_MM + 1e-6
        assert top_h <= BED_Z_MM + 1e-6

    for outer, inner in zip(dolls, dolls[1:]):
        assert outer.hollow, f"doll {outer.index} is not hollow but has an inner"
        feat = joint_features(outer.height, outer.diameter)
        cav_h = outer.cavity_height
        need_h = inner.height + 2.0 * feat.gap_axial
        assert need_h <= cav_h + 1e-6, (
            f"doll {inner.index} height {inner.height:.3f} + gaps does not fit "
            f"in doll {outer.index} cavity H={cav_h:.3f}"
        )
        open_r = waist_opening_radius(outer.height, outer.diameter, feat)
        need_r = 0.5 * inner.diameter + feat.gap_radial
        assert need_r <= open_r + 1e-6, (
            f"doll {inner.index} max radius {0.5 * inner.diameter:.3f} + gap "
            f"does not fit through doll {outer.index} waist opening r={open_r:.3f}"
        )


def format_report(
    dolls: list[DollSize],
    stop: str,
    tight_n: int,
    tight_stop: str,
) -> str:
    lines = [
        "Russian nesting dolls — nestability report",
        "=" * 72,
        f"Printer: Bambu H2D ({BED_X_MM:.0f} x {BED_Y_MM:.0f} x {BED_Z_MM:.0f} mm)",
        f"Largest: H={H0_MM:.1f} mm, D={D0_MM:.1f} mm",
        f"Full-size wall/gaps: {WALL_MM:.2f} / {GAP_RADIAL_MM:.2f} mm "
        f"(scale down toward floors below D={D_FULL_MM:.0f} mm)",
        f"Feature floors: wall={WALL_FLOOR_MM}, rim={RIM_FLOOR_MM}, "
        f"lip_t={LIP_T_FLOOR_MM}, clear={CLEAR_FLOOR_MM} mm",
        "",
        f"NEST COUNT (conservative export): {len(dolls)}",
        f"Stop reason: {stop}",
        f"Tight sensitivity (wall_full={TIGHT_WALL_MM}, gap={TIGHT_GAP_MM}): "
        f"{tight_n} dolls — {tight_stop}",
        "",
        f"{'#':>3}  {'H_mm':>8}  {'D_mm':>8}  {'wall':>6}  {'lip_h':>6}  "
        f"{'lip_t':>6}  {'rim':>5}  {'cav_H':>8}  {'cav_D':>8}  {'mass_g':>7}  body",
        "-" * 104,
    ]
    for d in dolls:
        body = "hollow" if d.hollow else "SOLID"
        wall_s = f"{d.wall:6.2f}" if d.hollow else f"{'—':>6}"
        lines.append(
            f"{d.index:3d}  {d.height:8.2f}  {d.diameter:8.2f}  {wall_s}  "
            f"{d.lip_height:6.2f}  {d.lip_radial:6.2f}  {d.rim:5.2f}  "
            f"{d.cavity_height:8.2f}  {d.cavity_diameter:8.2f}  "
            f"{d.mass_g_est:7.1f}  {body}"
        )
    lines.append("")
    lines.append(
        f"Total estimated PLA mass (shells): "
        f"{sum(d.mass_g_est for d in dolls):.0f} g"
    )
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Mesh generation
# ---------------------------------------------------------------------------

def _as_trimesh(result) -> trimesh.Trimesh:
    if isinstance(result, trimesh.Trimesh):
        return result
    if isinstance(result, (list, tuple)):
        return trimesh.util.concatenate(result)
    if hasattr(result, "geometry"):
        return trimesh.util.concatenate(list(result.geometry.values()))
    if hasattr(result, "dump"):
        dumped = result.dump()
        return dumped if isinstance(dumped, trimesh.Trimesh) else trimesh.util.concatenate(dumped)
    raise TypeError(f"Cannot coerce boolean result to Trimesh: {type(result)}")


def _boolean(a: trimesh.Trimesh, b: trimesh.Trimesh, op: str) -> trimesh.Trimesh:
    fn = {
        "difference": a.difference,
        "union": a.union,
        "intersection": a.intersection,
    }[op]
    try:
        out = fn(b, engine="manifold")
    except Exception:
        out = fn(b)
    mesh = _as_trimesh(out)
    mesh.process(validate=True)
    mesh.fix_normals()
    return mesh


def _revolve_closed(poly_rz: np.ndarray, sections: int) -> trimesh.Trimesh:
    """Revolve a closed (r, z) polygon about Z. poly_rz rows are [r, z]."""
    poly = np.asarray(poly_rz, dtype=float)
    if not np.allclose(poly[0], poly[-1]):
        poly = np.vstack([poly, poly[0]])
    mesh = trimesh.creation.revolve(poly, sections=sections)
    mesh.process(validate=True)
    mesh.fix_normals()
    if not mesh.is_volume:
        trimesh.repair.fix_normals(mesh)
        trimesh.repair.fix_winding(mesh)
        mesh.process(validate=True)
    return mesh


def _egg_band_solid(
    height: float,
    diameter: float,
    z0: float,
    z1: float,
    n: int | None = None,
    sections: int | None = None,
) -> trimesh.Trimesh:
    """Solid egg band between z0 and z1 (inclusive), revolved and capped on axis."""
    z0 = max(z0, 0.0)
    z1 = min(z1, height)
    if z1 <= z0 + 1e-6:
        raise ValueError(f"empty egg band [{z0}, {z1}]")
    n_def, sec_def = _mesh_res(diameter)
    n = n_def if n is None else n
    sections = sec_def if sections is None else sections
    z = np.linspace(z0, z1, max(n, 8))
    r = np.maximum(egg_outer_radius(z, height, diameter), 1e-3)
    # Closed meridian: axis@z0 → curve → axis@z1 → back down axis.
    poly = np.vstack(
        [
            [[0.0, z0]],
            np.column_stack([r, z]),
            [[0.0, z1]],
            [[0.0, z0]],
        ]
    )
    return _revolve_closed(poly, sections=sections)


def _annulus(
    r_outer: float,
    r_inner: float,
    z0: float,
    z1: float,
    sections: int = 48,
) -> trimesh.Trimesh:
    """Vertical annular ring from z0..z1."""
    if r_outer <= r_inner + 1e-6 or z1 <= z0:
        raise ValueError("degenerate annulus")
    poly = np.array(
        [
            [r_inner, z0],
            [r_outer, z0],
            [r_outer, z1],
            [r_inner, z1],
            [r_inner, z0],
        ],
        dtype=float,
    )
    return _revolve_closed(poly, sections=sections)


def _lip_radii(
    height: float,
    diameter: float,
    waist: float,
    lip_t: float,
    rim: float,
    clear: float,
    hollow: bool,
) -> tuple[float, float]:
    """Male lip radii sized so the hat can have a real female socket + outer rim.

    Returns (lip_outer_r, lip_inner_r) with lip_inner_r = nesting bore.
    Layout at the waist (outside → inside):
      outer wall | rim | socket clearance | male lip | bore
    """
    r_waist = float(egg_outer_radius(np.array([waist]), height, diameter)[0])
    if hollow:
        lip_outer_r = r_waist - rim - clear
        lip_inner_r = lip_outer_r - lip_t
        if lip_inner_r < 0.8:
            raise RuntimeError(
                f"lip bore too small (inner={lip_inner_r:.3f} at r_waist={r_waist:.3f})"
            )
    else:
        # Solid core: still a short male lip so halves join, bore is cosmetic.
        lip_outer_r = min(r_waist - rim, r_waist * 0.92)
        lip_inner_r = max(lip_outer_r - lip_t, 0.25)
    if lip_outer_r <= lip_inner_r + 0.15:
        raise RuntimeError(
            f"lip radii collapsed (outer={lip_outer_r:.3f}, inner={lip_inner_r:.3f})"
        )
    return lip_outer_r, lip_inner_r


def _hollow_bottom_mesh(
    height: float,
    diameter: float,
    waist: float,
    lip_h: float,
    lip_outer_r: float,
    lip_inner_r: float,
    wall: float,
    n: int,
    sections: int,
) -> trimesh.Trimesh:
    """Hollow bottom cup with a male press-fit lip sticking up at the rim."""
    inner_h = height - 2.0 * wall
    inner_d = diameter - 2.0 * wall

    z_outer = np.linspace(0.0, waist, n)
    r_outer = np.maximum(egg_outer_radius(z_outer, height, diameter), 1e-3)

    # Inner cavity below the bore (skip z=waist — already at lip_inner_r).
    z_inner = np.linspace(waist, wall, n)[1:]
    local = np.clip(z_inner - wall, 0.0, inner_h)
    r_inner = np.maximum(egg_outer_radius(local, inner_h, inner_d), 1e-3)

    poly = np.vstack(
        [
            [[0.0, 0.0]],
            np.column_stack([r_outer, z_outer]),
            [[lip_outer_r, waist]],
            [[lip_outer_r, waist + lip_h]],
            [[lip_inner_r, waist + lip_h]],
            [[lip_inner_r, waist]],
            np.column_stack([r_inner, z_inner]),
            [[0.0, wall]],
            [[0.0, 0.0]],
        ]
    )
    return _revolve_closed(poly, sections=sections)


def _hollow_top_mesh(
    height: float,
    diameter: float,
    waist: float,
    lip_h: float,
    lip_outer_r: float,
    lip_inner_r: float,
    wall: float,
    clear: float,
    rim: float,
    n: int,
    sections: int,
) -> trimesh.Trimesh:
    """Hollow top (hat) with a cylindrical female socket that slides onto the lip.

    Looking into the opening you should see, in order:
      1. flat rim annulus on the cut face
      2. cylindrical socket wall (depth ≈ lip height) that rides on the male lip
      3. inward shoulder that seats on top of the male lip
      4. the main hollow cavity above
    """
    inner_h = height - 2.0 * wall
    inner_d = diameter - 2.0 * wall
    socket_h = lip_h + 0.20
    # Socket bore clears the male lip OD; shoulder lands on the lip's top face.
    socket_r = lip_outer_r + clear
    shoulder_r = max(lip_inner_r - clear, 0.15)
    z_sock = waist + socket_h

    # Outer wall: waist → tip. Keep a full-thickness collar over the whole
    # socket height so the rim doesn't taper to a breakable wafer.
    z_up = np.linspace(waist, height, n)
    r_up = np.maximum(egg_outer_radius(z_up, height, diameter), 1e-3)
    collar_r = socket_r + rim
    r_up = np.where(z_up <= z_sock, np.maximum(r_up, collar_r), r_up)
    if float(r_up[0]) < collar_r - 1e-6:
        raise RuntimeError(
            f"top rim too thin at waist: outer={float(r_up[0]):.3f} "
            f"need>={collar_r:.3f}"
        )

    # Main cavity above the shoulder.
    z_inner = np.linspace(height - wall, z_sock, n)
    local = np.clip(z_inner - wall, 0.0, inner_h)
    r_inner = np.maximum(egg_outer_radius(local, inner_h, inner_d), 1e-3)

    poly = np.vstack(
        [
            np.column_stack([r_up, z_up]),
            [[0.0, height]],
            [[0.0, height - wall]],
            np.column_stack([r_inner, z_inner]),
            # Shoulder (seats on male lip top) then socket cylinder down to cut face.
            [[shoulder_r, z_sock]],
            [[socket_r, z_sock]],
            [[socket_r, waist]],
            # Flat rim annulus out to the outer wall at the cut face.
            [[float(r_up[0]), waist]],
        ]
    )
    return _revolve_closed(poly, sections=sections)


def _solid_bottom_mesh(
    height: float,
    diameter: float,
    waist: float,
    lip_h: float,
    lip_outer_r: float,
    lip_inner_r: float,
    n: int,
    sections: int,
) -> trimesh.Trimesh:
    """Solid bottom half with a male lip (innermost core)."""
    z_outer = np.linspace(0.0, waist, n)
    r_outer = np.maximum(egg_outer_radius(z_outer, height, diameter), 1e-3)
    poly = np.vstack(
        [
            [[0.0, 0.0]],
            np.column_stack([r_outer, z_outer]),
            [[lip_outer_r, waist]],
            [[lip_outer_r, waist + lip_h]],
            [[lip_inner_r, waist + lip_h]],
            [[lip_inner_r, waist]],
            [[0.0, waist]],
            [[0.0, 0.0]],
        ]
    )
    return _revolve_closed(poly, sections=sections)


def _solid_top_mesh(
    height: float,
    diameter: float,
    waist: float,
    lip_h: float,
    lip_outer_r: float,
    lip_inner_r: float,
    clear: float,
    rim: float,
    n: int,
    sections: int,
) -> trimesh.Trimesh:
    """Solid top half with female socket (innermost core)."""
    socket_h = lip_h + 0.20
    socket_r = lip_outer_r + clear
    shoulder_r = max(lip_inner_r - clear, 0.15)
    z_sock = waist + socket_h

    z_up = np.linspace(waist, height, n)
    r_up = np.maximum(egg_outer_radius(z_up, height, diameter), 1e-3)
    collar_r = socket_r + rim
    r_up = np.where(z_up <= z_sock, np.maximum(r_up, collar_r), r_up)
    if float(r_up[0]) < collar_r - 1e-6:
        raise RuntimeError(
            f"solid top rim too thin: outer={float(r_up[0]):.3f} need>={collar_r:.3f}"
        )

    poly = np.vstack(
        [
            np.column_stack([r_up, z_up]),
            [[0.0, height]],
            [[0.0, z_sock]],
            [[shoulder_r, z_sock]],
            [[socket_r, z_sock]],
            [[socket_r, waist]],
            [[float(r_up[0]), waist]],
        ]
    )
    return _revolve_closed(poly, sections=sections)


def build_halves(doll: DollSize) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Return (bottom, top) meshes in world coords (doll sitting on z=0)."""
    h, d = doll.height, doll.diameter
    waist = doll.waist_z
    lip_h, lip_t = doll.lip_height, doll.lip_radial
    n, sections = _mesh_res(d)

    lip_outer_r, lip_inner_r = _lip_radii(
        h, d, waist, lip_t, doll.rim, doll.clear, hollow=doll.hollow
    )

    if doll.hollow:
        bottom = _hollow_bottom_mesh(
            h, d, waist, lip_h, lip_outer_r, lip_inner_r, doll.wall, n, sections
        )
        top = _hollow_top_mesh(
            h,
            d,
            waist,
            lip_h,
            lip_outer_r,
            lip_inner_r,
            doll.wall,
            doll.clear,
            doll.rim,
            n,
            sections,
        )
    else:
        bottom = _solid_bottom_mesh(
            h, d, waist, lip_h, lip_outer_r, lip_inner_r, n, sections
        )
        top = _solid_top_mesh(
            h, d, waist, lip_h, lip_outer_r, lip_inner_r, doll.clear, doll.rim, n, sections
        )

    for m, name in ((bottom, "bottom"), (top, "top")):
        m.remove_unreferenced_vertices()
        m.fix_normals()
        if not m.is_volume:
            raise RuntimeError(
                f"doll {doll.index} {name} is not a volume "
                f"(watertight={m.is_watertight}, faces={len(m.faces)})"
            )
    return bottom, top


def orient_for_print(bottom: trimesh.Trimesh, top: trimesh.Trimesh, waist: float) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Lay cut faces on the bed (z=0)."""
    b = bottom.copy()
    t = top.copy()
    # Bottom: cut face is at z=waist (+ lip above). Flip so cut/lip side down? 
    # Plan: "cut face on bed". For bottom, cut face is the top of the cup
    # (z≈waist), so rotate 180° about X and shift.
    b.apply_translation([0.0, 0.0, -waist])
    b.apply_transform(
        trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0])
    )
    b.apply_translation([0.0, 0.0, -b.bounds[0, 2]])

    # Top: cut face already at z=waist (bottom of top half). Translate down.
    t.apply_translation([0.0, 0.0, -waist])
    t.apply_translation([0.0, 0.0, -t.bounds[0, 2]])
    return b, t


def export_dolls(dolls: list[DollSize], out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    for doll in dolls:
        print(
            f"  meshing doll {doll.index:02d}  "
            f"H={doll.height:.1f} D={doll.diameter:.1f} "
            f"{'hollow' if doll.hollow else 'SOLID'} ..."
        )
        bottom, top = build_halves(doll)
        bottom, top = orient_for_print(bottom, top, doll.waist_z)

        for half, mesh in (("bottom", bottom), ("top", top)):
            extents = mesh.extents
            assert extents[0] <= BED_X_MM + 0.5, f"{half} X {extents[0]}"
            assert extents[1] <= BED_Y_MM + 0.5, f"{half} Y {extents[1]}"
            assert extents[2] <= BED_Z_MM + 0.5, f"{half} Z {extents[2]}"
            if not mesh.is_volume:
                print(f"    [warn] doll {doll.index:02d} {half} is_volume=False "
                      f"(watertight={mesh.is_watertight})")
            path = os.path.join(out_dir, f"doll_{doll.index:02d}_{half}.stl")
            mesh.export(path)
            print(
                f"    wrote {path}  faces={len(mesh.faces):,}  "
                f"extents={extents.round(2).tolist()}"
            )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--report-only",
        action="store_true",
        help="Print/write nest report without generating STLs.",
    )
    p.add_argument(
        "--out-dir",
        default=OUT_DIR,
        help=f"STL output directory (default: {OUT_DIR})",
    )
    p.add_argument("--max-export", type=int, default=None,
                   help="Optional cap on how many dolls to mesh (largest first).")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    dolls, stop = size_nest()
    tight, tight_stop = size_nest(
        wall_full=TIGHT_WALL_MM,
        gap_axial_full=TIGHT_GAP_MM,
        gap_radial_full=TIGHT_GAP_MM,
    )
    verify_nest(dolls)

    report = format_report(dolls, stop, len(tight), tight_stop)
    print(report)

    os.makedirs(args.out_dir, exist_ok=True)
    report_path = os.path.join(args.out_dir, "nest_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    print(f"Wrote {report_path}")

    if args.report_only:
        return

    # Drop stale STLs from a previous larger nest before rewriting.
    if os.path.isdir(args.out_dir):
        for name in os.listdir(args.out_dir):
            if name.startswith("doll_") and name.endswith(".stl"):
                os.remove(os.path.join(args.out_dir, name))

    export_list = dolls if args.max_export is None else dolls[: args.max_export]
    print(f"Exporting {len(export_list)} dolls to {args.out_dir}/ ...")
    export_dolls(export_list, args.out_dir)
    print(f"Done. Nest count = {len(dolls)}")


if __name__ == "__main__":
    main()
