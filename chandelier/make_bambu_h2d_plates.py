"""Lay out quarter-scale chandelier polyhedra on Bambu H2D build plates.

The full chandelier decorative polyhedra are about 130 mm across.  This
script reuses the FDM-oriented per-polyhedron geometry from
``split_polyhedra.py``, scales each body to 25%, and packs all 31 shapes onto
one or more Bambu H2D single-nozzle plates.

Output:

    bambu_h2d_plates/
        README.md
        layout_manifest.csv
        plate_01_quarter_scale_polyhedra.stl

Each plate STL is a multi-body STL with the polyhedra already translated onto
the build plate.  Import one plate at a time into Bambu Studio.
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import dataclass

import trimesh

import polyhedra as P
from all_polyhedra import NODE_DIAMETER_MM
from split_polyhedra import SCALE_FACTOR, _safe_filename, _split_index_for_filename, split_one


HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "bambu_h2d_plates")

# Effective Bambu H2D single-nozzle build area, in millimetres.
BED_X_MM = 325.0
BED_Y_MM = 320.0
BED_Z_MM = 325.0
EDGE_MARGIN_MM = 12.0
PART_CLEARANCE_MM = 6.0
DEFAULT_SCALE = 0.25


@dataclass(frozen=True)
class PlatePart:
    solid: P.Solid
    filename: str
    mesh: trimesh.Trimesh


@dataclass(frozen=True)
class LayoutItem:
    part: PlatePart
    x_mm: float
    y_mm: float


@dataclass(frozen=True)
class PlatePlan:
    name: str
    items: tuple[LayoutItem, ...]


def _normalize_for_plate(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Center XY around the local origin and put the lowest point on Z=0."""
    mesh = mesh.copy()
    mins, maxs = mesh.bounds
    center_xy = (mins[:2] + maxs[:2]) / 2.0
    mesh.apply_translation([-center_xy[0], -center_xy[1], -mins[2]])
    return mesh


def _build_plate_part(solid: P.Solid, scale: float) -> PlatePart:
    idx = _split_index_for_filename(solid.category, solid.name)
    filename = f"{solid.category}_{idx:02d}_{_safe_filename(solid.name)}.stl"
    mesh = split_one(
        solid,
        with_pocket=False,
        with_fiber_hole=False,
        hole_diameter_mm=0.0,
        hole_depth_mm=0.0,
        fiber_hole_diameter_mm=0.0,
        fiber_hole_depth_mm=0.0,
        fiber_hole_offset_mm=0.0,
    )
    mesh.apply_scale(scale)
    mesh = _normalize_for_plate(mesh)
    return PlatePart(solid=solid, filename=filename, mesh=mesh)


def _part_sort_key(part: PlatePart) -> tuple[float, float, str]:
    ext = part.mesh.extents
    return (float(max(ext[0], ext[1])), float(ext[0] * ext[1]), part.filename)


def _pack_parts(parts: list[PlatePart]) -> list[PlatePlan]:
    """Deterministic shelf pack into as many H2D plates as needed."""
    sorted_parts = sorted(parts, key=_part_sort_key, reverse=True)

    usable_w = BED_X_MM - 2.0 * EDGE_MARGIN_MM
    usable_h = BED_Y_MM - 2.0 * EDGE_MARGIN_MM
    plates: list[PlatePlan] = []
    plate_items: list[LayoutItem] = []

    x = -usable_w / 2.0
    y = -usable_h / 2.0
    row_h = 0.0

    def flush_plate() -> None:
        nonlocal plate_items, x, y, row_h
        if plate_items:
            plates.append(PlatePlan(
                name=f"plate_{len(plates) + 1:02d}_quarter_scale_polyhedra",
                items=tuple(plate_items),
            ))
        plate_items = []
        x = -usable_w / 2.0
        y = -usable_h / 2.0
        row_h = 0.0

    for part in sorted_parts:
        ext = part.mesh.extents
        w = float(ext[0])
        h = float(ext[1])
        if w > usable_w or h > usable_h or float(ext[2]) > BED_Z_MM:
            raise RuntimeError(
                f"{part.filename} does not fit H2D area at this scale: "
                f"{w:.1f} x {h:.1f} x {float(ext[2]):.1f} mm"
            )

        if x + w > usable_w / 2.0 and plate_items:
            x = -usable_w / 2.0
            y += row_h + PART_CLEARANCE_MM
            row_h = 0.0

        if y + h > usable_h / 2.0 and plate_items:
            flush_plate()

        plate_items.append(LayoutItem(part=part, x_mm=x + w / 2.0, y_mm=y + h / 2.0))
        x += w + PART_CLEARANCE_MM
        row_h = max(row_h, h)

    flush_plate()
    return plates


def _make_plate_mesh(plan: PlatePlan) -> trimesh.Trimesh:
    meshes: list[trimesh.Trimesh] = []
    for item in plan.items:
        mesh = item.part.mesh.copy()
        mesh.apply_translation([item.x_mm, item.y_mm, 0.0])
        meshes.append(mesh)
    combined = trimesh.util.concatenate(meshes)

    mins, maxs = combined.bounds
    if mins[0] < -BED_X_MM / 2.0 or maxs[0] > BED_X_MM / 2.0:
        raise RuntimeError(f"{plan.name} exceeds H2D X build area: {mins[0]:.1f}..{maxs[0]:.1f}")
    if mins[1] < -BED_Y_MM / 2.0 or maxs[1] > BED_Y_MM / 2.0:
        raise RuntimeError(f"{plan.name} exceeds H2D Y build area: {mins[1]:.1f}..{maxs[1]:.1f}")
    if mins[2] < -1e-6 or maxs[2] > BED_Z_MM:
        raise RuntimeError(f"{plan.name} exceeds H2D Z build area: {mins[2]:.1f}..{maxs[2]:.1f}")

    return combined


def _write_manifest(plans: list[PlatePlan], scale: float) -> None:
    path = os.path.join(OUT_DIR, "layout_manifest.csv")
    rows: list[dict[str, object]] = []
    for plan in plans:
        for item in plan.items:
            ext = item.part.mesh.extents
            rows.append({
                "plate": plan.name,
                "part": item.part.filename,
                "solid": item.part.solid.name,
                "category": item.part.solid.category,
                "scale": f"{scale:.4f}",
                "x_mm": f"{item.x_mm:.1f}",
                "y_mm": f"{item.y_mm:.1f}",
                "size_x_mm": f"{float(ext[0]):.1f}",
                "size_y_mm": f"{float(ext[1]):.1f}",
                "size_z_mm": f"{float(ext[2]):.1f}",
            })

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_readme(plans: list[PlatePlan], scale: float) -> None:
    path = os.path.join(OUT_DIR, "README.md")
    full_diameter = NODE_DIAMETER_MM * SCALE_FACTOR
    scaled_diameter = full_diameter * scale

    lines = [
        "# Bambu H2D plates - quarter-scale chandelier polyhedra",
        "",
        "Generated by `make_bambu_h2d_plates.py` from the same parametric wireframe polyhedra used by the chandelier build.",
        "",
        f"Scale: `{scale:.2f}x` of the print-ready chandelier polyhedra. The reference full-size decorative polyhedra are about {full_diameter:.0f} mm across, so these quarter-scale pieces are about {scaled_diameter:.0f} mm across.",
        "",
        "Assumption: these layouts target Bambu's published H2D single-nozzle effective build area of 325 x 320 x 325 mm. The physical bed is wider, but this is the safer slicer target.",
        "",
        "The quarter-scale plates omit the full-size heat-set insert pocket and fiber pass-through from `split_polyhedra.py`; those hardware features would be oversized for small display/test prints.",
        "",
        "## Plates",
        "",
    ]
    for plan in plans:
        counts: dict[str, int] = {}
        for item in plan.items:
            counts[item.part.solid.category] = counts.get(item.part.solid.category, 0) + 1
        summary = ", ".join(f"{qty} {category}" for category, qty in sorted(counts.items()))
        lines.append(f"- `{plan.name}.stl`: {len(plan.items)} pieces ({summary})")
    lines.extend([
        "",
        "Import one plate STL at a time into Bambu Studio. The parts are already oriented with a broad face on the bed and placed with 6 mm nominal clearance plus a 12 mm bed-edge margin.",
        "",
        "`layout_manifest.csv` records each piece's XY center, category, and bounding box.",
        "",
    ])
    with open(path, "w") as f:
        f.write("\n".join(lines))


def build_plates(scale: float, out_dir: str = OUT_DIR) -> None:
    global OUT_DIR
    OUT_DIR = out_dir
    os.makedirs(OUT_DIR, exist_ok=True)

    parts = [_build_plate_part(solid, scale) for solid in P.ALL_SOLIDS]
    plans = _pack_parts(parts)

    print("Bambu H2D quarter-scale chandelier plates")
    print(f"Effective build area: {BED_X_MM:.0f} x {BED_Y_MM:.0f} x {BED_Z_MM:.0f} mm")
    print(f"Scale: {scale:.3f}x")
    print(f"Writing {OUT_DIR}")
    print()

    for plan in plans:
        mesh = _make_plate_mesh(plan)
        out_path = os.path.join(OUT_DIR, f"{plan.name}.stl")
        mesh.export(out_path)
        print(f"  wrote {os.path.basename(out_path):36s}"
              f" {len(plan.items):2d} parts"
              f" envelope {mesh.extents[0]:5.1f} x {mesh.extents[1]:5.1f} x {mesh.extents[2]:5.1f} mm")

    _write_manifest(plans, scale)
    _write_readme(plans, scale)
    print()
    print(f"OK -- {len(plans)} build-plate STL(s) plus README.md and layout_manifest.csv.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                    help=f"scale applied after print orientation (default: {DEFAULT_SCALE})")
    ap.add_argument("--out-dir", default=OUT_DIR,
                    help="output directory (default: bambu_h2d_plates)")
    args = ap.parse_args()

    if args.scale <= 0:
        raise SystemExit("--scale must be positive")

    build_plates(args.scale, args.out_dir)


if __name__ == "__main__":
    main()
