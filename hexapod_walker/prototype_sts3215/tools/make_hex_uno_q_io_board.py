"""Custom Uno Q hex I/O board: Ø110 hex plate + edge connector fixtures.

Aug 2026 (user: "design me a custom hex board for my uno q ... similar to
the current flat hex board with each corner clear and the same holes for
offsets/magnets.  Along the edges i want help mounting two 5 wide waygos,
2 mx5264-4p connectors and one mx5264-2p connector").

Baseline = ``hex_mount_plate_110_with_leg_holes`` (the magnet-held board in
the as-built stack): Ø110 hex, 2 mm base, BOTH M3 standoff/magnet squares
(old 49.5 mm + current 62.2 mm) and the 6 corner leg holes for the raised
platform feet.  All of that is preserved; the 6 corners stay clear.

New edge fixtures (all walls rise from the plate top; prints flat):

  * 2x WAGO 221-415 holsters (NE + NW edges).  The 221-415 is 29.9 W x
    8.4 H x 18.6 D (WAGO datasheet) -- too wide to lie flat between the
    raised platform's corner legs (each edge channel is only ~24.7 mm), so
    the Wago stands VERTICALLY: wire-entry face DOWN over a 5-wire slot
    through the plate (power comes up from the PDB/chassis below), levers
    facing sideways.  Snug pocket, open top; lift the unit out to work the
    levers, drop it back in.
  * 2x Molex 5264 4-circuit cups (E outboard + W edges) and 1x 5264
    2-circuit cup (E inboard).  Housing 12.4 L x 3.9 W (Molex 50-37-504x /
    5264-0x, 2.5 mm pitch); the cup holds it mating-face UP with the wires
    dropping through a slot in the plate.  A notch in the outboard wall
    clears the friction-lock ramp.
  * Mid-edge TABS on the NE/NW/E/W flats extend the plate locally to
    apothem ~56 so the fixtures fit outboard of the Uno Q; the coxa yaw
    sweep only reaches inward to ~59.6 mm at the flat azimuths, and the
    margin is asserted at runtime below (same policy as the base plate).

Also new: 4x M3 holes on the standard Arduino UNO mounting pattern for the
Uno Q (UNO form factor), centred at (-3.5, -12) -- the as-built spot nudged
3.5 mm west so all four standoffs clear the SE platform foot.

Everything is verified at build time with shapely: fixtures + tabs vs the
Uno Q footprint, corner foot pads, platform leg blades, standoff/magnet
holes and the coxa sweep.  The breakout placeholder's old (0, 36) spot is
consumed by nothing here -- but the Wago wedges DO cover the (0, 36) area's
east/west flanks, so a wide shield should stack on the Uno Q instead.

Outputs under ``extra_stl/``:
  * ``hex_uno_q_io_board_110.stl``
  * ``hex_uno_q_io_board_110_preview.png``  (top-view layout + keepouts)

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_hex_uno_q_io_board.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")))
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_walker.prototype_sts3215.hexapod_prototype as hp  # noqa: E402
from hexapod_walker.prototype_sts3215.tools.make_xtool_hex_mount_plate import (  # noqa: E402
    CIRCUMRADIUS, FLAT_TO_FLAT, THICKNESS, _hex_coords, _hole_xy_r,
)
from hexapod_walker.prototype_sts3215.tools.make_xtool_hex_raised_platform import (  # noqa: E402
    CORNER_ANGLES_DEG, FOOT_PAD, FOOT_T, HOLE_R, LEG_T, LEG_W, OUT_DIR,
    _foot_xy,
)

APOTHEM = FLAT_TO_FLAT / 2.0                       # 47.63

# ---------------------------------------------------------------------------
# Connector envelopes (datasheet values -- print, then check fit and tweak
# the *_CL clearances if your printer runs tight/loose).
# ---------------------------------------------------------------------------
WAGO_W = 29.9        # mm -- across the 5 levers (WAGO 221-415 datasheet)
WAGO_H = 8.4         # mm -- body height, levers closed
WAGO_D = 18.6        # mm -- wire direction (vertical here, wires DOWN)
WAGO_CL = 0.5        # mm -- total pocket clearance per axis

MX4_L = 12.4         # mm -- Molex 5264 4-cir housing length (50-37-5043)
MX2_L = 7.4          # mm -- 2-cir ((n-1)*2.5 + 4.9)
MX_W = 3.9           # mm -- housing width
MX_CL_L = 0.35       # mm -- pocket clearance along the pin row
MX_CL_W = 0.4        # mm -- pocket clearance across
MX_RAMP_NOTCH_W = 5.0  # mm -- friction-lock ramp relief in the outboard wall

WALL_T = 2.0         # mm -- fixture wall thickness
WAGO_WALL_H = 11.0   # mm -- of the 18.6 body: snug grip, 7.6 mm to pull on
MX_WALL_H = 7.0      # mm -- of the ~11.7 housing
FLOOR_Z = THICKNESS  # fixture walls rise from the plate top

# Wire slots through the plate (wires come up from the chassis below).
WAGO_SLOT_L = 27.0   # spans the 5 wire entries (pitch ~5.8)
WAGO_SLOT_W = 4.5
MX4_SLOT_L = 10.0
MX2_SLOT_L = 5.0
MX_SLOT_W = 3.0

# ---------------------------------------------------------------------------
# Placement (edge-local frames: +r radial out along the flat normal,
# +t tangential, right-handed looking down).
# ---------------------------------------------------------------------------
TAB_R_OUT = 56.0     # tab outer apothem (coxa sweep reaches in to ~59.6)
TAB_R_IN = 44.0

# NE / NW flats; per-edge tangential shift TOWARD the N vertex (the +t
# axis flips between the two edge frames) -- clears the Uno Q top edge on
# the inboard corner AND keeps the (+-31.1, 31.1) standoff holes uncovered
# (margins asserted below).
WAGO_EDGES = ((60.0, +1.5), (120.0, -1.5))
# Pocket outer envelope (radial x tangential), Wago standing on its
# wire-entry face: footprint = W x H.
WAGO_POCK_R = WAGO_W + WAGO_CL        # 30.4 radial
WAGO_POCK_T = WAGO_H + WAGO_CL        # 8.9 tangential
WAGO_R_OUTER = 55.8                   # outboard wall outer face (inside tab)

E_EDGE_DEG = 0.0
W_EDGE_DEG = 180.0
MX4_POCK_T = MX4_L + MX_CL_L          # 12.75 tangential
MX2_POCK_T = MX2_L + MX_CL_L          # 7.75
MX_POCK_R = MX_W + MX_CL_W            # 4.3 radial
MX_R_OUTER = 55.3                     # outboard cup wall outer face
MX_ROW_GAP = 1.5                      # radial gap between E-edge rows

UNO_W, UNO_D = 68.58, 53.34
UNO_CENTRE = (-3.5, -12.0)  # as-built (0,-12) nudged 3.5 mm west so every
                            # standoff clears the SE platform foot pad
# Standard Arduino UNO hole pattern (mm from board bottom-left corner).
UNO_HOLES_CORNER = ((13.97, 2.54), (15.24, 50.8),
                    (66.04, 17.78), (66.04, 45.72))
UNO_HOLE_R = hp.BRACKET_BOLT_HOLE / 2.0   # M3 clearance, same as the rest

FOOT_R = np.hypot(*_foot_xy()[0])


def _edge_frame(deg: float):
    a = np.deg2rad(deg)
    rhat = np.array([np.cos(a), np.sin(a)])
    that = np.array([-np.sin(a), np.cos(a)])
    return rhat, that


def _rect_world(deg: float, r0: float, r1: float, t0: float, t1: float):
    """Corner list (CCW) of an edge-frame rectangle in world XY."""
    rhat, that = _edge_frame(deg)
    return [tuple(r * rhat + t * that)
            for (r, t) in ((r0, t0), (r1, t0), (r1, t1), (r0, t1))]


def _box_from_rect(corners, z0: float, z1: float) -> trimesh.Trimesh:
    """Extruded convex quad (used for tabs / walls in edge frames)."""
    from shapely.geometry import Polygon
    mesh = trimesh.creation.extrude_polygon(Polygon(corners), z1 - z0)
    mesh.apply_translation([0, 0, z0])
    return mesh


def _uno_hole_xy():
    cx = UNO_CENTRE[0] - UNO_W / 2.0
    cy = UNO_CENTRE[1] - UNO_D / 2.0
    return [(cx + hx, cy + hy) for (hx, hy) in UNO_HOLES_CORNER]


# ---------------------------------------------------------------------------
# Fixture definitions -> (solid walls, plate slot rects, pocket rects)
# ---------------------------------------------------------------------------
def _u_pocket(deg: float, r_in: float, r_out: float, t_c: float,
              pock_r: float, pock_t: float, wall_h: float,
              notch_t: float | None = None):
    """Closed rectangular pocket: 4 walls around (pock_r x pock_t), outer
    face at ``r_out``.  ``notch_t`` cuts a ramp-relief notch (that wide,
    full height) centred in the OUTBOARD wall."""
    z0, z1 = FLOOR_Z, FLOOR_Z + wall_h
    outer = _box_from_rect(
        _rect_world(deg, r_in, r_out,
                    t_c - pock_t / 2.0 - WALL_T, t_c + pock_t / 2.0 + WALL_T),
        z0, z1)
    cavity = _box_from_rect(
        _rect_world(deg, r_in + WALL_T, r_out - WALL_T,
                    t_c - pock_t / 2.0, t_c + pock_t / 2.0),
        z0 - 1.0, z1 + 1.0)
    cuts = [cavity]
    if notch_t is not None:
        cuts.append(_box_from_rect(
            _rect_world(deg, r_out - WALL_T - 0.5, r_out + 1.0,
                        t_c - notch_t / 2.0, t_c + notch_t / 2.0),
            z0 - 1.0, z1 + 1.0))
    return outer, cuts


def _wago_fixture(deg: float, t_shift: float):
    r_out = WAGO_R_OUTER
    r_in = r_out - WAGO_POCK_R - 2 * WALL_T
    walls, cuts = _u_pocket(deg, r_in, r_out, t_shift,
                            WAGO_POCK_R, WAGO_POCK_T, WAGO_WALL_H)
    r_c = 0.5 * (r_in + r_out)
    slot = _rect_world(deg, r_c - WAGO_SLOT_L / 2.0, r_c + WAGO_SLOT_L / 2.0,
                       t_shift - WAGO_SLOT_W / 2.0,
                       t_shift + WAGO_SLOT_W / 2.0)
    outline = _rect_world(deg, r_in, r_out,
                          t_shift - WAGO_POCK_T / 2.0 - WALL_T,
                          t_shift + WAGO_POCK_T / 2.0 + WALL_T)
    return walls, cuts, [slot], outline


def _mx_fixture(deg: float, r_out: float, pock_t: float, slot_l: float):
    r_in = r_out - MX_POCK_R - 2 * WALL_T
    walls, cuts = _u_pocket(deg, r_in, r_out, 0.0,
                            MX_POCK_R, pock_t, MX_WALL_H,
                            notch_t=MX_RAMP_NOTCH_W)
    r_c = 0.5 * (r_in + r_out)
    slot = _rect_world(deg, r_c - MX_SLOT_W / 2.0, r_c + MX_SLOT_W / 2.0,
                       -slot_l / 2.0, slot_l / 2.0)
    outline = _rect_world(deg, r_in, r_out,
                          -pock_t / 2.0 - WALL_T, pock_t / 2.0 + WALL_T)
    return walls, cuts, [slot], outline


def _fixtures():
    """[(name, walls_mesh, wall_cuts, slot_rects, outline_rect), ...]"""
    out = []
    for (deg, t_shift) in WAGO_EDGES:
        out.append((f"wago_221_415 @{deg:.0f}deg",
                    *_wago_fixture(deg, t_shift)))
    out.append(("mx5264_4p @0deg (E outer)",
                *_mx_fixture(E_EDGE_DEG, MX_R_OUTER, MX4_POCK_T, MX4_SLOT_L)))
    r2 = MX_R_OUTER - (MX_POCK_R + 2 * WALL_T) - MX_ROW_GAP
    out.append(("mx5264_2p @0deg (E inner)",
                *_mx_fixture(E_EDGE_DEG, r2, MX2_POCK_T, MX2_SLOT_L)))
    out.append(("mx5264_4p @180deg (W)",
                *_mx_fixture(W_EDGE_DEG, MX_R_OUTER, MX4_POCK_T, MX4_SLOT_L)))
    return out


def _tabs():
    """Mid-edge plate extensions under the fixtures."""
    tabs = []
    for (deg, t_shift) in WAGO_EDGES:
        tabs.append(_rect_world(deg, TAB_R_IN, TAB_R_OUT,
                                t_shift - 10.0, t_shift + 10.0))
    for deg in (E_EDGE_DEG, W_EDGE_DEG):
        tabs.append(_rect_world(deg, TAB_R_IN, TAB_R_OUT - 0.4,
                                -10.5, 10.5))
    return tabs


# ---------------------------------------------------------------------------
# Keepouts (shapely) + margin report
# ---------------------------------------------------------------------------
def _keepouts():
    from shapely.geometry import Point, Polygon

    def rot_square(cx, cy, half, deg):
        a = np.deg2rad(deg)
        c, s = np.cos(a), np.sin(a)
        pts = []
        for lx, ly in ((half, half), (half, -half),
                       (-half, -half), (-half, half)):
            pts.append((cx + c * lx - s * ly, cy + s * lx + c * ly))
        return Polygon(pts)

    ko = {}
    ko["uno_q"] = Polygon([
        (UNO_CENTRE[0] - UNO_W / 2, UNO_CENTRE[1] - UNO_D / 2),
        (UNO_CENTRE[0] + UNO_W / 2, UNO_CENTRE[1] - UNO_D / 2),
        (UNO_CENTRE[0] + UNO_W / 2, UNO_CENTRE[1] + UNO_D / 2),
        (UNO_CENTRE[0] - UNO_W / 2, UNO_CENTRE[1] + UNO_D / 2)])
    for i, (fx, fy) in enumerate(_foot_xy()):
        deg = np.degrees(np.arctan2(fy, fx))
        ko[f"platform_foot_{i}"] = rot_square(fx, fy, FOOT_PAD / 2.0, deg)
        # Leg blade cross-section (full fixture height -> hard keepout).
        a = np.deg2rad(deg)
        c, s = np.cos(a), np.sin(a)
        pts = []
        for lx, ly in ((LEG_T / 2, LEG_W / 2), (LEG_T / 2, -LEG_W / 2),
                       (-LEG_T / 2, -LEG_W / 2), (-LEG_T / 2, LEG_W / 2)):
            pts.append((fx + c * lx - s * ly, fy + s * lx + c * ly))
        ko[f"platform_leg_{i}"] = Polygon(pts)
    for (cx, cy, r) in _hole_xy_r():
        ko[f"m3_hole_{cx:+.1f}_{cy:+.1f}"] = Point(cx, cy).buffer(r + 1.0)
    for (cx, cy) in _foot_xy():
        ko[f"leg_hole_{cx:+.1f}_{cy:+.1f}"] = Point(cx, cy).buffer(HOLE_R + 1.0)
    return ko


def _coxa_sweep_circles():
    coxa = hp.make_coxa_link()
    reach = float(np.hypot(coxa.vertices[:, 0], coxa.vertices[:, 1]).max())
    out = []
    for (_leg, edge_mid, _R, _R3) in hp._leg_chassis_frames():
        out.append((edge_mid[0], edge_mid[1], reach))
    return out


def check_layout(verbose: bool = True) -> float:
    """Assert every fixture/tab clears every keepout; return worst margin."""
    from shapely.geometry import Point, Polygon

    ko = _keepouts()
    # Fixture walls + plate slots must clear every keepout (walls block
    # driver access over a hole; slots would merge with one).  Tabs are
    # plain base plate -- holes drill through plate, feet may stand on it
    # -- so tabs are only held to the coxa-sweep boundary check.
    shapes = []
    for (name, _w, _c, slots, outline) in _fixtures():
        shapes.append((name, Polygon(outline), True))
        for j, s in enumerate(slots):
            shapes.append((f"{name} slot{j}", Polygon(s), True))
    for i, tab in enumerate(_tabs()):
        shapes.append((f"tab_{i}", Polygon(tab), False))

    worst = np.inf
    for sname, poly, check_ko in shapes:
        if check_ko:
            for kname, kpoly in ko.items():
                d = poly.distance(kpoly)
                if poly.intersects(kpoly):
                    raise RuntimeError(f"{sname} OVERLAPS {kname}")
                if d < worst:
                    worst, worst_pair = d, (sname, kname)
                if d < 0.8:
                    raise RuntimeError(
                        f"{sname} only {d:.2f} mm from {kname} (need >= 0.8)")
        # coxa sweep (outer boundary check)
        for (cx, cy, r) in _coxa_sweep_circles():
            dmin = min(np.hypot(px - cx, py - cy)
                       for (px, py) in poly.exterior.coords) - r
            if dmin < 2.0:
                raise RuntimeError(
                    f"{sname} within {dmin:.2f} mm of the coxa sweep")
    if verbose:
        print(f"  layout OK -- tightest margin {worst:.2f} mm "
              f"({worst_pair[0]} <-> {worst_pair[1]})")
    return worst


# ---------------------------------------------------------------------------
# Solid
# ---------------------------------------------------------------------------
def make_board() -> trimesh.Trimesh:
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    base2d = unary_union([Polygon(_hex_coords())]
                         + [Polygon(t) for t in _tabs()])
    base = trimesh.creation.extrude_polygon(base2d, THICKNESS)

    walls, wall_cuts = [], []
    slot_cuts = []
    for (_name, w, cuts, slots, _outline) in _fixtures():
        walls.append(w)
        wall_cuts.extend(cuts)
        for s in slots:
            slot_cuts.append(_box_from_rect(s, -1.0, THICKNESS + 1.0))

    holes = []
    for (cx, cy, r) in _hole_xy_r():
        h = trimesh.creation.cylinder(radius=r, height=THICKNESS * 4)
        h.apply_translation([cx, cy, THICKNESS / 2.0])
        holes.append(h)
    for (cx, cy) in _foot_xy():
        h = trimesh.creation.cylinder(radius=HOLE_R, height=THICKNESS * 4)
        h.apply_translation([cx, cy, THICKNESS / 2.0])
        holes.append(h)
    for (cx, cy) in _uno_hole_xy():
        h = trimesh.creation.cylinder(radius=UNO_HOLE_R, height=THICKNESS * 4)
        h.apply_translation([cx, cy, THICKNESS / 2.0])
        holes.append(h)

    solid = trimesh.boolean.union([base, *walls])
    solid = trimesh.boolean.difference(
        [solid, *wall_cuts, *slot_cuts, *holes])
    solid.merge_vertices()
    assert solid.is_watertight, "boolean result not watertight"
    return solid


# ---------------------------------------------------------------------------
# Preview
# ---------------------------------------------------------------------------
def write_preview() -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MplPoly

    fig, ax = plt.subplots(figsize=(8.4, 8.4), dpi=120)
    ax.set_aspect("equal")
    ax.set_title("hex_uno_q_io_board_110 -- layout (top view, mm)",
                 fontsize=11)

    ax.add_patch(MplPoly(_hex_coords(), closed=True, fill=True,
                         facecolor="#dbeafe", edgecolor="#1d4ed8",
                         linewidth=1.4, alpha=0.7, label="Ø110 hex base"))
    for i, tab in enumerate(_tabs()):
        ax.add_patch(MplPoly(tab, closed=True, fill=True,
                             facecolor="#bfdbfe", edgecolor="#1d4ed8",
                             linewidth=1.0, alpha=0.7,
                             label="edge tabs" if i == 0 else None))
    ko = _keepouts()
    first_pad = first_leg = True
    for name, poly in ko.items():
        xy = list(poly.exterior.coords)
        if name.startswith("platform_foot"):
            ax.add_patch(MplPoly(xy, closed=True, fill=True,
                                 facecolor="#fde68a", edgecolor="#b45309",
                                 alpha=0.8,
                                 label="platform foot pads" if first_pad
                                 else None))
            first_pad = False
        elif name.startswith("platform_leg"):
            ax.add_patch(MplPoly(xy, closed=True, fill=True,
                                 facecolor="#fca5a5", edgecolor="#b91c1c",
                                 alpha=0.8,
                                 label="platform legs (full height)"
                                 if first_leg else None))
            first_leg = False
    ax.add_patch(MplPoly(list(ko["uno_q"].exterior.coords), closed=True,
                         fill=False, edgecolor="#047857", linewidth=1.6,
                         linestyle="--", label="Uno Q (68.6 x 53.3)"))
    for (cx, cy, r) in _hole_xy_r():
        ax.add_patch(Circle((cx, cy), r, fill=False, edgecolor="#111827"))
    for (cx, cy) in _foot_xy():
        ax.add_patch(Circle((cx, cy), HOLE_R, fill=False,
                            edgecolor="#b45309"))
    for i, (cx, cy) in enumerate(_uno_hole_xy()):
        ax.add_patch(Circle((cx, cy), UNO_HOLE_R, fill=False,
                            edgecolor="#047857",
                            label="Uno Q M3 pattern" if i == 0 else None))
    for k, (name, _w, _c, slots, outline) in enumerate(_fixtures()):
        ax.add_patch(MplPoly(outline, closed=True, fill=True,
                             facecolor="#c4b5fd", edgecolor="#5b21b6",
                             alpha=0.85,
                             label="connector fixtures" if k == 0 else None))
        for s in slots:
            ax.add_patch(MplPoly(s, closed=True, fill=True,
                                 facecolor="white", edgecolor="#5b21b6"))
        cx = np.mean([p[0] for p in outline])
        cy = np.mean([p[1] for p in outline])
        ax.annotate(name.split(" @")[0], (cx, cy),
                    ha="center", va="center", fontsize=6.5, color="#3b0764")
    for i, (cx, cy, r) in enumerate(_coxa_sweep_circles()):
        ax.add_patch(Circle((cx, cy), r, fill=False, edgecolor="#be185d",
                            alpha=0.6, linewidth=0.8, linestyle=":",
                            label="coxa yaw sweep" if i == 0 else None))
    ax.set_xlim(-90, 90)
    ax.set_ylim(-90, 90)
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.6)
    ax.legend(loc="upper right", fontsize=7.5, framealpha=0.92)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "hex_uno_q_io_board_110_preview.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"hex Uno Q I/O board: Ø{2 * CIRCUMRADIUS:.0f} hex + edge tabs to "
          f"apothem {TAB_R_OUT:.0f}, base {THICKNESS:.0f} mm")
    print(f"  WAGO 221-415 x2 (vertical, wires down): pocket "
          f"{WAGO_POCK_R:.1f} x {WAGO_POCK_T:.1f}, walls {WAGO_WALL_H:.0f} tall")
    print(f"  Molex 5264: 4P x2 + 2P x1, pockets {MX4_POCK_T:.2f}/"
          f"{MX2_POCK_T:.2f} x {MX_POCK_R:.1f}, walls {MX_WALL_H:.0f} tall")
    check_layout()
    solid = make_board()
    path = os.path.join(OUT_DIR, "hex_uno_q_io_board_110.stl")
    solid.export(path)
    print(f"wrote {path}")
    print(f"wrote {write_preview()}")
    print("NOTE: clearances are datasheet-based -- test-fit the first print "
          "and adjust WAGO_CL / MX_CL_* if needed.  A wide breakout no "
          "longer fits at (0, 36); stack it on the Uno Q instead.")


if __name__ == "__main__":
    main()
