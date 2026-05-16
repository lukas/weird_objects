"""Shared build-plate packing for tabletop hexapod prototype parts.

Used by ``make_bambu_h2d_trays.py`` and ``make_bambu_x1_trays.py``.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Callable

import trimesh
from trimesh.transformations import rotation_matrix

from prepare_xometry_upload import PART_REGISTRY


@dataclass(frozen=True)
class TrayPrinterConfig:
    slug: str
    title: str
    bed_x_mm: float
    bed_y_mm: float
    bed_z_mm: float
    edge_margin_mm: float
    part_clearance_mm: float
    """Nominal gap between part bounding boxes (packing + grid rows)."""
    split_long_link_plates: bool
    """If True, emit separate plates for tibia and femur rows (needed on 256 mm beds)."""
    split_hardware_plate: bool
    """If True, large hardware packs alone and horn adapters get their own plate."""


@dataclass(frozen=True)
class PartSpec:
    filename: str
    make_fn: Callable[[], trimesh.Trimesh]
    reorient_fn: Callable[[trimesh.Trimesh], trimesh.Trimesh]
    qty: int
    material: str


@dataclass(frozen=True)
class LayoutItem:
    part: PartSpec
    copy_index: int
    x_mm: float
    y_mm: float
    rotate_z_deg: int


@dataclass(frozen=True)
class PlatePlan:
    name: str
    material: str
    items: tuple[LayoutItem, ...]


_MESH_CACHE: dict[tuple[str, int], trimesh.Trimesh] = {}
_FOOTPRINT_CACHE: dict[tuple[str, int], tuple[float, float]] = {}


def clear_layout_caches() -> None:
    _MESH_CACHE.clear()
    _FOOTPRINT_CACHE.clear()


def _part_specs() -> dict[str, PartSpec]:
    specs: dict[str, PartSpec] = {}
    for filename, make_fn, reorient_fn, qty, material, *_ in PART_REGISTRY:
        specs[filename] = PartSpec(filename, make_fn, reorient_fn, qty, material)
    return specs


def _oriented_mesh(part: PartSpec, rotate_z_deg: int) -> trimesh.Trimesh:
    key = (part.filename, rotate_z_deg)
    cached = _MESH_CACHE.get(key)
    if cached is not None:
        return cached.copy()

    mesh = part.reorient_fn(part.make_fn())
    if rotate_z_deg:
        mesh.apply_transform(rotation_matrix(math.radians(rotate_z_deg), [0, 0, 1]))
    _MESH_CACHE[key] = mesh.copy()
    return mesh


def _footprint(part: PartSpec, rotate_z_deg: int) -> tuple[float, float]:
    key = (part.filename, rotate_z_deg)
    cached = _FOOTPRINT_CACHE.get(key)
    if cached is not None:
        return cached

    mesh = _oriented_mesh(part, rotate_z_deg)
    footprint = (float(mesh.extents[0]), float(mesh.extents[1]))
    _FOOTPRINT_CACHE[key] = footprint
    return footprint


def _instance(part: PartSpec, copy_index: int, x_mm: float, y_mm: float,
              rotate_z_deg: int = 0) -> LayoutItem:
    return LayoutItem(part, copy_index, x_mm, y_mm, rotate_z_deg)


def _grid_row(cfg: TrayPrinterConfig, part: PartSpec, qty: int, *,
              y_mm: float, rotate_z_deg: int,
              start_index: int = 1) -> list[LayoutItem]:
    """Center ``qty`` copies in one row along X."""
    mesh = _oriented_mesh(part, rotate_z_deg)
    item_w = float(mesh.extents[0])
    c = cfg.part_clearance_mm
    total_w = qty * item_w + (qty - 1) * c
    x0 = -total_w / 2.0 + item_w / 2.0
    return [
        _instance(part, start_index + i, x0 + i * (item_w + c),
                  y_mm, rotate_z_deg)
        for i in range(qty)
    ]


def _row_fits_x(cfg: TrayPrinterConfig, part: PartSpec, qty: int,
                rotate_z_deg: int) -> bool:
    """Does a centred row of ``qty`` rotated copies fit the bed in X
    (accounting for edge margin)?  Used to decide whether a single
    plate of all six femurs/tibias fits, or whether we must split
    them across multiple plates."""
    mesh = _oriented_mesh(part, rotate_z_deg)
    item_w = float(mesh.extents[0])
    c = cfg.part_clearance_mm
    total_w = qty * item_w + (qty - 1) * c
    return total_w <= cfg.bed_x_mm - 2.0 * cfg.edge_margin_mm


def _rectangles_intersect(a: tuple[float, float, float, float],
                          b: tuple[float, float, float, float]) -> bool:
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def _pack_hardware(
    cfg: TrayPrinterConfig,
    parts: dict[str, PartSpec],
    *,
    requests: list[tuple[str, int]] | None = None,
) -> list[LayoutItem]:
    """Simple deterministic bottom-left pack for rigid hardware pieces."""
    if requests is None:
        requests = [
            ("battery_holder.stl", 1),
            ("electronics_tray.stl", 1),
            ("coxa_bracket.stl", 6),
            ("coxa_link.stl", 6),
            ("servo_horn_adapter.stl", 18),
        ]

    expanded: list[tuple[PartSpec, int]] = []
    for filename, qty in requests:
        expanded.extend((parts[filename], i) for i in range(1, qty + 1))

    expanded.sort(
        key=lambda entry: max(_footprint(entry[0], 0)),
        reverse=True,
    )

    bx, by = cfg.bed_x_mm, cfg.bed_y_mm
    m = cfg.edge_margin_mm
    min_x = -bx / 2.0 + m
    max_x = bx / 2.0 - m
    min_y = -by / 2.0 + m
    max_y = by / 2.0 - m
    step = 1.0 if cfg.slug == "x1" else 2.0
    pad = cfg.part_clearance_mm

    used: list[tuple[float, float, float, float]] = []
    items: list[LayoutItem] = []

    for part, copy_index in expanded:
        placed = False
        for y in _frange(min_y, max_y, step):
            if placed:
                break
            for x in _frange(min_x, max_x, step):
                variants = (0, 90)
                for rotate_z_deg in variants:
                    w, h = _footprint(part, rotate_z_deg)
                    if x + w > max_x or y + h > max_y:
                        continue
                    rect = (x, y, x + w + pad, y + h + pad)
                    if any(_rectangles_intersect(rect, other) for other in used):
                        continue
                    used.append(rect)
                    items.append(_instance(part, copy_index, x + w / 2.0, y + h / 2.0,
                                           rotate_z_deg))
                    placed = True
                    break
                if placed:
                    break
        if not placed:
            raise RuntimeError(
                f"{cfg.slug}: could not fit {part.filename} copy {copy_index} "
                f"on hardware plate ({bx:.0f} x {by:.0f} mm bed)"
            )

    return items


def _battery_and_electronics_row(cfg: TrayPrinterConfig,
                                  parts: dict[str, PartSpec]) -> list[LayoutItem]:
    """Side-by-side layout for the two tray-like body parts."""
    bat = parts["battery_holder.stl"]
    et = parts["electronics_tray.stl"]
    mb = _oriented_mesh(bat, 0)
    me = _oriented_mesh(et, 0)
    gap = cfg.part_clearance_mm
    total_w = float(mb.extents[0]) + gap + float(me.extents[0])
    x_bat = -total_w / 2.0 + float(mb.extents[0]) / 2.0
    x_et = total_w / 2.0 - float(me.extents[0]) / 2.0
    return [
        _instance(bat, 1, x_bat, 0.0, 0),
        _instance(et, 1, x_et, 0.0, 0),
    ]


def _horn_adapter_grid(cfg: TrayPrinterConfig,
                       parts: dict[str, PartSpec]) -> list[LayoutItem]:
    """Three centred rows of six adapters (18 total)."""
    part = parts["servo_horn_adapter.stl"]
    mesh = _oriented_mesh(part, 0)
    pitch_y = float(mesh.extents[1]) + cfg.part_clearance_mm
    y_centres = (pitch_y, 0.0, -pitch_y)
    items: list[LayoutItem] = []
    copy = 1
    for y_mm in y_centres:
        items.extend(_grid_row(cfg, part, 6, y_mm=y_mm, rotate_z_deg=0,
                               start_index=copy))
        copy += 6
    return items


def _frange(start: float, stop: float, step: float):
    value = start
    while value <= stop:
        yield value
        value += step


def _make_plate_mesh(cfg: TrayPrinterConfig, plan: PlatePlan) -> trimesh.Trimesh:
    meshes: list[trimesh.Trimesh] = []
    for item in plan.items:
        mesh = _oriented_mesh(item.part, item.rotate_z_deg)
        mesh.apply_translation([item.x_mm, item.y_mm, 0.0])
        meshes.append(mesh)
    combined = trimesh.util.concatenate(meshes)

    bx, by, bz = cfg.bed_x_mm, cfg.bed_y_mm, cfg.bed_z_mm
    mins, maxs = combined.bounds
    label = cfg.title
    if mins[0] < -bx / 2.0 or maxs[0] > bx / 2.0:
        raise RuntimeError(f"{plan.name} exceeds {label} X build area: {mins[0]:.1f}..{maxs[0]:.1f}")
    if mins[1] < -by / 2.0 or maxs[1] > by / 2.0:
        raise RuntimeError(f"{plan.name} exceeds {label} Y build area: {mins[1]:.1f}..{maxs[1]:.1f}")
    if mins[2] < -1e-6 or maxs[2] > bz:
        raise RuntimeError(f"{plan.name} exceeds {label} Z build area: {mins[2]:.1f}..{maxs[2]:.1f}")

    return combined


def build_plate_plans(cfg: TrayPrinterConfig) -> list[PlatePlan]:
    parts = _part_specs()
    chassis_top = parts["chassis_top.stl"]
    chassis_bottom = parts["chassis_bottom.stl"]
    femur = parts["femur_link.stl"]
    tibia = parts["tibia_link.stl"]
    foot = parts["foot_pad.stl"]

    # chassis_top is the smaller 140 mm deck (battery + electronics + arm),
    # chassis_bottom is the full 200 mm structural plate with per-leg
    # bracket cutouts and bolt holes.  They are no longer interchangeable,
    # so each gets its own plate.
    plans: list[PlatePlan] = [
        PlatePlan(
            "plate_01_chassis_top",
            "PLA/PETG rigid",
            (_instance(chassis_top, 1, 0.0, 0.0),),
        ),
        PlatePlan(
            "plate_02_chassis_bottom",
            "PLA/PETG rigid",
            (_instance(chassis_bottom, 1, 0.0, 0.0),),
        ),
    ]

    if cfg.split_long_link_plates:
        # Femur post-reorient + 90 deg tray rotation is ~46 mm wide in
        # X-on-plate; six in a row at the 5 mm part_clearance on an X1
        # is 6*46 + 5*5 = 301 mm and overflows the 256 mm bed.  Auto-
        # split the femur row into two plates of three when a single
        # six-femur row no longer fits the bed.  Tibias still fit six
        # in a row on a 256 mm bed (6*34 + 5*5 = 229 mm).
        femur_fits_six = _row_fits_x(cfg, femur, 6, rotate_z_deg=90)
        femur_plates: list[PlatePlan]
        if femur_fits_six:
            femur_plates = [
                PlatePlan(
                    "plate_04_rigid_femur_links",
                    "PLA/PETG rigid",
                    tuple(_grid_row(cfg, femur, 6, y_mm=0.0,
                                     rotate_z_deg=90)),
                ),
            ]
        else:
            femur_plates = [
                PlatePlan(
                    "plate_04a_rigid_femur_links_1of2",
                    "PLA/PETG rigid",
                    tuple(_grid_row(cfg, femur, 3, y_mm=0.0,
                                     rotate_z_deg=90, start_index=1)),
                ),
                PlatePlan(
                    "plate_04b_rigid_femur_links_2of2",
                    "PLA/PETG rigid",
                    tuple(_grid_row(cfg, femur, 3, y_mm=0.0,
                                     rotate_z_deg=90, start_index=4)),
                ),
            ]

        if cfg.split_hardware_plate:
            plans.extend([
                PlatePlan(
                    "plate_03_rigid_tibia_links",
                    "PLA/PETG rigid",
                    tuple(_grid_row(cfg, tibia, 6, y_mm=0.0, rotate_z_deg=90)),
                ),
                *femur_plates,
                PlatePlan(
                    "plate_05_rigid_battery_electronics",
                    "PLA/PETG rigid",
                    tuple(_battery_and_electronics_row(cfg, parts)),
                ),
                PlatePlan(
                    "plate_06_rigid_coxa_brackets_links",
                    "PLA/PETG rigid",
                    tuple(_pack_hardware(cfg, parts, requests=[
                        ("coxa_bracket.stl", 6),
                        ("coxa_link.stl", 6),
                    ])),
                ),
                PlatePlan(
                    "plate_07_rigid_servo_horn_adapters",
                    "PLA/PETG rigid",
                    tuple(_horn_adapter_grid(cfg, parts)),
                ),
                PlatePlan(
                    "plate_08_tpu_foot_pads",
                    "TPU 95A",
                    tuple(_grid_row(cfg, foot, 6, y_mm=0.0, rotate_z_deg=0)),
                ),
            ])
        else:
            plans.extend([
                PlatePlan(
                    "plate_03_rigid_tibia_links",
                    "PLA/PETG rigid",
                    tuple(_grid_row(cfg, tibia, 6, y_mm=0.0, rotate_z_deg=90)),
                ),
                *femur_plates,
                PlatePlan(
                    "plate_05_rigid_hardware",
                    "PLA/PETG rigid",
                    tuple(_pack_hardware(cfg, parts)),
                ),
                PlatePlan(
                    "plate_06_tpu_foot_pads",
                    "TPU 95A",
                    tuple(_grid_row(cfg, foot, 6, y_mm=0.0, rotate_z_deg=0)),
                ),
            ])
    else:
        long_links = [
            *_grid_row(cfg, tibia, 6, y_mm=-70.0, rotate_z_deg=90),
            *_grid_row(cfg, femur, 6, y_mm=78.0, rotate_z_deg=90),
        ]
        # Horn adapters get their own plate even on the large H2D bed -- the
        # 32 mm OD pucks don't pack efficiently next to the bigger hardware
        # and a dedicated grid is more reliable to print.
        hardware_requests = [
            ("battery_holder.stl", 1),
            ("electronics_tray.stl", 1),
            ("coxa_bracket.stl", 6),
            ("coxa_link.stl", 6),
        ]
        plans.extend([
            PlatePlan(
                "plate_03_rigid_long_links",
                "PLA/PETG rigid",
                tuple(long_links),
            ),
            PlatePlan(
                "plate_04_rigid_hardware",
                "PLA/PETG rigid",
                tuple(_pack_hardware(cfg, parts, requests=hardware_requests)),
            ),
            PlatePlan(
                "plate_05_rigid_servo_horn_adapters",
                "PLA/PETG rigid",
                tuple(_horn_adapter_grid(cfg, parts)),
            ),
            PlatePlan(
                "plate_06_tpu_foot_pads",
                "TPU 95A",
                tuple(_grid_row(cfg, foot, 6, y_mm=0.0, rotate_z_deg=0)),
            ),
        ])

    return plans


def write_manifest(out_dir: str, plans: list[PlatePlan]) -> None:
    path = os.path.join(out_dir, "layout_manifest.csv")
    rows: list[dict[str, object]] = []
    for plan in plans:
        for item in plan.items:
            rows.append({
                "plate": plan.name,
                "plate_material": plan.material,
                "part": item.part.filename,
                "copy": item.copy_index,
                "x_mm": f"{item.x_mm:.1f}",
                "y_mm": f"{item.y_mm:.1f}",
                "rotate_z_deg": item.rotate_z_deg,
            })

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_readme(cfg: TrayPrinterConfig, out_dir: str, plans: list[PlatePlan]) -> None:
    path = os.path.join(out_dir, "README.md")
    bed_line = (
        f"{cfg.bed_x_mm:.0f} x {cfg.bed_y_mm:.0f} x {cfg.bed_z_mm:.0f} mm"
    )
    lines = [
        f"# Bambu {cfg.title} trays — hexapod prototype",
        "",
        f"Generated for a **{bed_line}** build volume.",
        "",
        "## Plates",
        "",
    ]
    for plan in plans:
        mesh_path = f"{plan.name}.stl"
        counts: dict[str, int] = {}
        for item in plan.items:
            counts[item.part.filename] = counts.get(item.part.filename, 0) + 1
        summary = ", ".join(f"{qty} x `{name}`" for name, qty in sorted(counts.items()))
        lines.append(f"- `{mesh_path}` ({plan.material}): {summary}")
    lines.extend([
        "",
        "Import one plate STL at a time into Bambu Studio. Parts are oriented for FDM.",
        f"Edge margin: {cfg.edge_margin_mm:.0f} mm; nominal part spacing: {cfg.part_clearance_mm:.0f} mm.",
        "",
        "`layout_manifest.csv` records each copy's XY centre and Z rotation.",
        "",
        "## Print reliability (dense plates / “spaghetti”)",
        "",
        "Large tray STLs are many separate shells on one plate. Reliability is mostly **slicer and environment**, not the STL:",
        "",
        "- **Brim (recommended)** — not stored in the STL; turn it on in Bambu Studio. Start with **auto brim** or **outer brim**, **3–5 mm** width, especially on **`foot_pad.stl`** (round bases) and **`servo_horn_adapter.stl`** (small footprint). Per-object: select the part → **Others** → brim.",
        "- **First layer** — slightly lower **initial layer speed** (e.g. 80–120 mm/s effective vs printing max) and confirm **Z offset / bed mesh** so nothing grazes loose mid-print.",
        "- **Draft / cooling** — avoid fans blasting the bed corner on tall skinny features; keep enclosure closed when using one.",
        "- **Filament / moisture** — wet PETG/TPU strings badly; dry filament if you see random blobs or snapped threads.",
        "- **Still failing** — split into fewer parts per plate (we already split X1 vs H2D), slow outer walls for tiny pieces, or print the worst offenders alone from `stl_prototype/`.",
        "",
    ])
    with open(path, "w") as f:
        f.write("\n".join(lines))


def generate_trays(cfg: TrayPrinterConfig, out_dir: str) -> None:
    clear_layout_caches()
    os.makedirs(out_dir, exist_ok=True)
    plans = build_plate_plans(cfg)

    print(f"Bambu {cfg.title} prototype trays ({cfg.slug})")
    print(f"Build area: {cfg.bed_x_mm:.0f} x {cfg.bed_y_mm:.0f} x {cfg.bed_z_mm:.0f} mm")
    print(f"Writing {out_dir}")
    print()

    for plan in plans:
        mesh = _make_plate_mesh(cfg, plan)
        out_path = os.path.join(out_dir, f"{plan.name}.stl")
        mesh.export(out_path)
        print(f"  wrote {os.path.basename(out_path):36s}"
              f" {len(plan.items):2d} parts"
              f" envelope {mesh.extents[0]:5.1f} x {mesh.extents[1]:5.1f} x {mesh.extents[2]:5.1f} mm")

    write_manifest(out_dir, plans)
    write_readme(cfg, out_dir, plans)
    print()
    print(f"OK -- {len(plans)} build-plate STL(s) plus README.md and layout_manifest.csv.")
