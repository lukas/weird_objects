"""Shared build-plate packing for the optional hexapod arm parts.

Used by ``make_bambu_h2d_plates.py`` and ``make_bambu_x1_plates.py``.

Mirrors the layout style used by ``prototype/bambu_tray_common.py`` but
reads the part geometry from the on-disk STLs in ``arm/stl_arm/`` and
``prototype/stl_prototype/`` instead of regenerating it.  This keeps
the arm Bambu scripts independent of any concurrent edits to the
prototype's part generators.

Outputs (per call to :func:`generate_plates`):

    <out_dir>/
        plate_01_arm.stl              -- multi-body STL (one arm worth)
        plate_01_arm.3mf              -- multi-object 3MF with build items
        layout_manifest.csv           -- per-copy XY centres + rotation
        README.md                     -- orientation + slicer notes
"""

from __future__ import annotations

import csv
import html
import math
import os
import zipfile
from dataclasses import dataclass

import trimesh
from trimesh.transformations import rotation_matrix


HERE = os.path.dirname(os.path.abspath(__file__))
ARM_STL_DIR = os.path.join(HERE, "stl_arm")
# The prototype's stl_prototype dir is the PARENT directory's stl_prototype/
# (the arm now lives at prototype/arm/, so go up one level).
PROTO_STL_DIR = os.path.normpath(
    os.path.join(HERE, os.pardir, "stl_prototype")
)


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TrayPrinterConfig:
    slug: str
    title: str
    bed_x_mm: float
    bed_y_mm: float
    bed_z_mm: float
    edge_margin_mm: float
    part_clearance_mm: float


@dataclass(frozen=True)
class PartSpec:
    filename: str
    """Part filename used in the manifest / README; not necessarily the
    source filename (e.g. the leg's `servo_horn_adapter.stl`)."""
    source_path: str
    qty: int
    orient_rotate_x_deg: float
    """Rotation about the X axis applied to the source STL BEFORE
    dropping it to the bed, to get the part into a sensible FDM
    orientation."""
    orient_notes: str
    print_time_min_per_copy: float
    filament_g_per_copy: float


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


# ---------------------------------------------------------------------------
# Arm part list (one arm)
# ---------------------------------------------------------------------------
# Print-time / filament estimates come from the table in arm/ARM.md (which
# itself was tuned to a Bambu P1S-class FDM machine at 0.2 mm layer, 15-20%
# infill).  Horn adapter is ~ 5 g / 12 min based on the leg-prototype data.

ARM_PART_LIST: tuple[PartSpec, ...] = (
    PartSpec(
        filename="arm_base_bracket.stl",
        source_path=os.path.join(ARM_STL_DIR, "arm_base_bracket.stl"),
        qty=1,
        orient_rotate_x_deg=0.0,
        orient_notes=(
            "Flange flat on the bed (z = 0). The J1 DS3225 servo well opens "
            "upward — no supports needed inside the well."
        ),
        print_time_min_per_copy=120.0,
        filament_g_per_copy=32.0,
    ),
    PartSpec(
        filename="arm_shoulder_link.stl",
        source_path=os.path.join(ARM_STL_DIR, "arm_shoulder_link.stl"),
        qty=1,
        orient_rotate_x_deg=+90.0,
        orient_notes=(
            "Rotate +90 deg about X so the hip-pitch cradle opens upward "
            "(same orientation the leg's coxa_link uses on the prototype "
            "build plates)."
        ),
        print_time_min_per_copy=135.0,
        filament_g_per_copy=38.0,
    ),
    PartSpec(
        filename="arm_upper.stl",
        source_path=os.path.join(ARM_STL_DIR, "arm_upper.stl"),
        qty=1,
        orient_rotate_x_deg=-90.0,
        orient_notes=(
            "Rotate -90 deg about X (= leg femur_link orientation): the "
            "spar's broad face sits on the bed and the knee cradle stands "
            "up ~28 mm with a short bridged ceiling — minimal supports."
        ),
        print_time_min_per_copy=150.0,
        filament_g_per_copy=48.0,
    ),
    PartSpec(
        filename="arm_forearm.stl",
        source_path=os.path.join(ARM_STL_DIR, "arm_forearm.stl"),
        qty=1,
        orient_rotate_x_deg=+90.0,
        orient_notes=(
            "Rotate +90 deg about X (= leg tibia_link orientation): long "
            "thin link laid flat with the foot socket bore open upward."
        ),
        print_time_min_per_copy=130.0,
        filament_g_per_copy=36.0,
    ),
    PartSpec(
        filename="wrist_adapter.stl",
        source_path=os.path.join(ARM_STL_DIR, "wrist_adapter.stl"),
        qty=1,
        orient_rotate_x_deg=0.0,
        orient_notes=(
            "Plate flat on the bed; the 7 mm foot-socket plug projects "
            "upward — printable without supports."
        ),
        print_time_min_per_copy=30.0,
        filament_g_per_copy=7.0,
    ),
    PartSpec(
        filename="gripper_base.stl",
        source_path=os.path.join(ARM_STL_DIR, "gripper_base.stl"),
        qty=1,
        orient_rotate_x_deg=0.0,
        orient_notes=(
            "Wrist-mating face on the bed; the J4 servo cavity opens "
            "upward and the J5 pocket opens out the +Y side — both span "
            "short enough distances to bridge cleanly."
        ),
        print_time_min_per_copy=105.0,
        filament_g_per_copy=28.0,
    ),
    PartSpec(
        filename="gripper_jaw_left.stl",
        source_path=os.path.join(ARM_STL_DIR, "gripper_jaw_left.stl"),
        qty=1,
        orient_rotate_x_deg=+90.0,
        orient_notes=(
            "Rotate +90 deg about X so the jaw lies flat on its broad "
            "X-Z face (4 mm tall). Tiny part; brim helps."
        ),
        print_time_min_per_copy=20.0,
        filament_g_per_copy=5.0,
    ),
    PartSpec(
        filename="gripper_jaw_right.stl",
        source_path=os.path.join(ARM_STL_DIR, "gripper_jaw_right.stl"),
        qty=1,
        orient_rotate_x_deg=+90.0,
        orient_notes="Mirror of jaw_left; same flat orientation.",
        print_time_min_per_copy=20.0,
        filament_g_per_copy=5.0,
    ),
    # 5 horn adapters: one per arm joint J1 — J5.  The source STL is the
    # leg-prototype's adapter; this script READS it but never modifies it.
    PartSpec(
        filename="servo_horn_adapter.stl",
        source_path=os.path.join(PROTO_STL_DIR, "servo_horn_adapter.stl"),
        qty=5,
        orient_rotate_x_deg=0.0,
        orient_notes=(
            "Source STL already has the counter-bore for the plastic horn "
            "facing -Z, so it lands on the bed correctly with no extra "
            "reorient. Bolt-pattern face is up."
        ),
        print_time_min_per_copy=12.0,
        filament_g_per_copy=5.0,
    ),
)


# ---------------------------------------------------------------------------
# Mesh helpers (load + reorient cached per part/rotation)
# ---------------------------------------------------------------------------

_MESH_CACHE: dict[tuple[str, int], trimesh.Trimesh] = {}


def clear_layout_caches() -> None:
    _MESH_CACHE.clear()


def _drop_to_bed(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Centre XY at origin and translate so z_min = 0."""
    out = mesh.copy()
    mins, maxs = out.bounds
    cx = (float(mins[0]) + float(maxs[0])) / 2.0
    cy = (float(mins[1]) + float(maxs[1])) / 2.0
    out.apply_translation([-cx, -cy, -float(mins[2])])
    return out


def _oriented_mesh(part: PartSpec, rotate_z_deg: int) -> trimesh.Trimesh:
    """Return the part loaded from disk, re-oriented for FDM, rotated by
    ``rotate_z_deg`` about Z, and dropped to z = 0 with its XY centroid
    at the origin."""
    key = (part.filename, int(rotate_z_deg))
    cached = _MESH_CACHE.get(key)
    if cached is not None:
        return cached.copy()

    if not os.path.isfile(part.source_path):
        raise FileNotFoundError(
            f"Part STL not found: {part.source_path}. "
            f"Re-run the upstream generator first "
            f"(./run.sh hexapod_walker/prototype/arm/arm.py or "
            f"./run.sh hexapod_walker/prototype/hexapod_prototype.py)."
        )

    mesh = trimesh.load(part.source_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError(
            f"Expected a single mesh in {part.source_path}, got {type(mesh)!r}"
        )

    if abs(part.orient_rotate_x_deg) > 1e-6:
        mesh.apply_transform(rotation_matrix(
            math.radians(part.orient_rotate_x_deg), [1, 0, 0]))
    if rotate_z_deg:
        mesh.apply_transform(rotation_matrix(
            math.radians(rotate_z_deg), [0, 0, 1]))
    mesh = _drop_to_bed(mesh)

    _MESH_CACHE[key] = mesh.copy()
    return mesh


def _footprint(part: PartSpec, rotate_z_deg: int) -> tuple[float, float]:
    mesh = _oriented_mesh(part, rotate_z_deg)
    return float(mesh.extents[0]), float(mesh.extents[1])


# ---------------------------------------------------------------------------
# Single-plate bottom-left packing
# ---------------------------------------------------------------------------

def _rect_overlaps(a: tuple[float, float, float, float],
                   b: tuple[float, float, float, float]) -> bool:
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def _frange(start: float, stop: float, step: float):
    value = start
    while value <= stop + 1e-9:
        yield value
        value += step


def pack_single_plate(cfg: TrayPrinterConfig,
                      parts: tuple[PartSpec, ...]) -> list[LayoutItem]:
    """Deterministic bottom-left pack onto a single bed.

    Each part is tried in 0 or 90 deg rotation about Z. Parts are
    placed in order of largest footprint side first, which leaves the
    tail of small parts (the horn adapters) for the leftover spaces.
    """
    expanded: list[tuple[PartSpec, int]] = []
    for part in parts:
        for i in range(1, part.qty + 1):
            expanded.append((part, i))

    expanded.sort(
        key=lambda entry: max(_footprint(entry[0], 0)),
        reverse=True,
    )

    m = cfg.edge_margin_mm
    pad = cfg.part_clearance_mm
    min_x = -cfg.bed_x_mm / 2.0 + m
    max_x = cfg.bed_x_mm / 2.0 - m
    min_y = -cfg.bed_y_mm / 2.0 + m
    max_y = cfg.bed_y_mm / 2.0 - m
    step = 2.0

    used: list[tuple[float, float, float, float]] = []
    items: list[LayoutItem] = []

    for part, copy_index in expanded:
        placed = False
        for y in _frange(min_y, max_y, step):
            if placed:
                break
            for x in _frange(min_x, max_x, step):
                for rz in (0, 90):
                    w, h = _footprint(part, rz)
                    if x + w > max_x or y + h > max_y:
                        continue
                    rect = (x, y, x + w + pad, y + h + pad)
                    if any(_rect_overlaps(rect, other) for other in used):
                        continue
                    used.append(rect)
                    items.append(LayoutItem(
                        part, copy_index, x + w / 2.0, y + h / 2.0, rz))
                    placed = True
                    break
                if placed:
                    break
        if not placed:
            raise RuntimeError(
                f"{cfg.slug}: could not fit {part.filename} copy "
                f"{copy_index} on plate "
                f"({cfg.bed_x_mm:.0f} x {cfg.bed_y_mm:.0f} mm)"
            )

    return items


# ---------------------------------------------------------------------------
# Plate STL (combined) and 3MF (multi-object) writers
# ---------------------------------------------------------------------------

def make_plate_mesh(cfg: TrayPrinterConfig, plan: PlatePlan) -> trimesh.Trimesh:
    """Concatenate all items into a single STL-friendly multi-body mesh."""
    meshes: list[trimesh.Trimesh] = []
    for item in plan.items:
        m = _oriented_mesh(item.part, item.rotate_z_deg)
        m.apply_translation([item.x_mm, item.y_mm, 0.0])
        meshes.append(m)
    combined = trimesh.util.concatenate(meshes)

    bx, by, bz = cfg.bed_x_mm, cfg.bed_y_mm, cfg.bed_z_mm
    mins, maxs = combined.bounds
    if mins[0] < -bx / 2.0 - 1e-3 or maxs[0] > bx / 2.0 + 1e-3:
        raise RuntimeError(
            f"{plan.name}: X envelope {mins[0]:.1f}..{maxs[0]:.1f} mm "
            f"exceeds {cfg.title} bed ({bx:.0f} mm)"
        )
    if mins[1] < -by / 2.0 - 1e-3 or maxs[1] > by / 2.0 + 1e-3:
        raise RuntimeError(
            f"{plan.name}: Y envelope {mins[1]:.1f}..{maxs[1]:.1f} mm "
            f"exceeds {cfg.title} bed ({by:.0f} mm)"
        )
    if mins[2] < -1e-3 or maxs[2] > bz + 1e-3:
        raise RuntimeError(
            f"{plan.name}: Z envelope {mins[2]:.1f}..{maxs[2]:.1f} mm "
            f"exceeds {cfg.title} build height ({bz:.0f} mm)"
        )
    return combined


_CT_XML = (
    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
    '<Default Extension="rels" '
    'ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
    '<Default Extension="model" '
    'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
    '</Types>'
)

_RELS_XML = (
    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
    '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
    'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
    '</Relationships>'
)


def write_3mf(cfg: TrayPrinterConfig, plan: PlatePlan, path: str) -> None:
    """Write a multi-object 3MF where each plate copy is its own
    ``<object>`` placed via a ``<build><item transform=...>``.

    The build items use absolute plate coordinates whose origin is the
    FRONT-LEFT corner of the printer bed (i.e. positive XY only), so
    the 3MF opens with parts already in the right physical spots in
    Bambu Studio / OrcaSlicer.
    """
    object_xml_parts: list[str] = []
    build_xml_parts: list[str] = []

    for idx, item in enumerate(plan.items, start=1):
        mesh = _oriented_mesh(item.part, item.rotate_z_deg)
        verts = mesh.vertices
        faces = mesh.faces

        vert_xml = "".join(
            f'<vertex x="{float(v[0]):.4f}" '
            f'y="{float(v[1]):.4f}" '
            f'z="{float(v[2]):.4f}"/>'
            for v in verts
        )
        tri_xml = "".join(
            f'<triangle v1="{int(t[0])}" v2="{int(t[1])}" v3="{int(t[2])}"/>'
            for t in faces
        )
        name = html.escape(f"{item.part.filename}#{item.copy_index}")
        object_xml_parts.append(
            f'<object id="{idx}" type="model" name="{name}">'
            f'<mesh><vertices>{vert_xml}</vertices>'
            f'<triangles>{tri_xml}</triangles></mesh>'
            f'</object>'
        )

        tx = item.x_mm + cfg.bed_x_mm / 2.0
        ty = item.y_mm + cfg.bed_y_mm / 2.0
        build_xml_parts.append(
            f'<item objectid="{idx}" '
            f'transform="1 0 0 0 1 0 0 0 1 {tx:.4f} {ty:.4f} 0"/>'
        )

    model_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        f'<resources>{"".join(object_xml_parts)}</resources>'
        f'<build>{"".join(build_xml_parts)}</build>'
        '</model>'
    )

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", _CT_XML)
        zf.writestr("_rels/.rels", _RELS_XML)
        zf.writestr("3D/3dmodel.model", model_xml)


# ---------------------------------------------------------------------------
# Manifest CSV + README writers
# ---------------------------------------------------------------------------

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


def _format_hours(minutes: float) -> str:
    h = int(minutes // 60)
    m = int(round(minutes - h * 60))
    if h <= 0:
        return f"{m} min"
    return f"{h} h {m:02d} min"


def write_readme(cfg: TrayPrinterConfig, out_dir: str,
                 plans: list[PlatePlan]) -> None:
    path = os.path.join(out_dir, "README.md")
    bed_line = (
        f"{cfg.bed_x_mm:.0f} x {cfg.bed_y_mm:.0f} x {cfg.bed_z_mm:.0f} mm"
    )

    total_minutes = 0.0
    total_grams = 0.0
    for plan in plans:
        for item in plan.items:
            total_minutes += item.part.print_time_min_per_copy
            total_grams += item.part.filament_g_per_copy

    lines = [
        f"# Bambu {cfg.title} plates — optional hexapod arm",
        "",
        f"Generated by `make_bambu_{cfg.slug}_plates.py` for a "
        f"**{bed_line}** build volume.",
        "",
        "These plates contain everything you need to print **one full arm**",
        "(see `prototype/arm/ARM.md`).  All parts are in PLA / PETG rigid; "
        "no TPU.",
        "",
        "## Plates",
        "",
    ]
    for plan in plans:
        stl_name = f"{plan.name}.stl"
        threemf_name = f"{plan.name}.3mf"
        counts: dict[str, int] = {}
        for item in plan.items:
            counts[item.part.filename] = counts.get(item.part.filename, 0) + 1
        summary = ", ".join(
            f"{qty} x `{name}`" for name, qty in sorted(counts.items())
        )
        plan_minutes = sum(it.part.print_time_min_per_copy for it in plan.items)
        plan_grams = sum(it.part.filament_g_per_copy for it in plan.items)
        lines.append(
            f"- `{threemf_name}` / `{stl_name}` ({plan.material}): "
            f"{len(plan.items)} parts — {summary}  "
            f"\n  Estimated **{_format_hours(plan_minutes)}**, "
            f"**~ {plan_grams:.0f} g** of filament."
        )

    lines.extend([
        "",
        f"**Total for one arm:** {_format_hours(total_minutes)} of print "
        f"time, ~ {total_grams:.0f} g of PLA / PETG.",
        "",
        f"Edge margin: {cfg.edge_margin_mm:.0f} mm; nominal part spacing: "
        f"{cfg.part_clearance_mm:.0f} mm.",
        "",
        "Import the 3MF (preferred) or the STL into Bambu Studio. The 3MF",
        "carries each part as a separate object with its own build-plate",
        "position and Z rotation; the STL is provided as a fallback / for",
        "preview viewers that don't open 3MFs.",
        "",
        "`layout_manifest.csv` lists each copy's XY centre (relative to",
        "the bed centre, +X to the right, +Y away from you) and its Z",
        "rotation in degrees.",
        "",
        "## Per-part orientation tips",
        "",
    ])
    for part in ARM_PART_LIST:
        lines.append(
            f"- **`{part.filename}`** ({part.qty}x, "
            f"~ {_format_hours(part.print_time_min_per_copy)} @ "
            f"{part.filament_g_per_copy:.0f} g each): {part.orient_notes}"
        )

    lines.extend([
        "",
        "## Slicer settings",
        "",
        "- **Material:** PLA or PETG; PETG is tougher under servo torque.",
        "- **Layer height:** 0.2 mm gives a good speed / strength balance.",
        "- **Infill:** **25 % gyroid** for the structural links",
        "  (`arm_shoulder_link`, `arm_upper`, `arm_forearm`, `gripper_base`,",
        "  `arm_base_bracket`).  **15 %** for the small parts",
        "  (`wrist_adapter`, `gripper_jaw_*`, `servo_horn_adapter`).",
        "- **Walls:** 3 perimeters everywhere — the parts have plenty of",
        "  small bolt-bores whose pull-out strength comes from the walls.",
        "- **Supports:** *tree* supports only on the `gripper_base` (J5",
        "  pocket entry on the +Y side); every other part is shaped to",
        "  print support-free in the orientations listed above.",
        "- **Brim:** 5 mm outer brim on `gripper_jaw_*` and",
        "  `servo_horn_adapter` (small footprints), `auto` brim elsewhere.",
        "",
        "## Print reliability (one busy plate)",
        "",
        "The whole arm fits on a single dense plate. A few things that",
        "trip people up:",
        "",
        "- **Brim** — turn it on in Bambu Studio (it's not encoded in the",
        "  3MF mesh). 3 – 5 mm outer brim on the tiny parts is the biggest",
        "  win against random first-layer fails.",
        "- **Bed levelling** — re-tram before a long plate; a 0.05 mm Z",
        "  offset is enough to lose a small part halfway through.",
        "- **Filament moisture** — wet PETG strings badly; dry it if you",
        "  see random blobs or snapped threads.",
        "- **Splitting** — if you'd rather print one part at a time, the",
        "  source STLs live in `prototype/arm/stl_arm/` and",
        "  `prototype/stl_prototype/servo_horn_adapter.stl`. The 3MF here",
        "  just bundles them onto one plate.",
        "",
    ])

    with open(path, "w") as f:
        f.write("\n".join(lines))


# ---------------------------------------------------------------------------
# Top-level entry point
# ---------------------------------------------------------------------------

def generate_plates(cfg: TrayPrinterConfig, out_dir: str) -> None:
    """Pack one full arm onto a single plate and write all four output
    artefacts: per-plate STL + 3MF, ``layout_manifest.csv`` and
    ``README.md``."""
    clear_layout_caches()
    os.makedirs(out_dir, exist_ok=True)

    items = pack_single_plate(cfg, ARM_PART_LIST)
    plan = PlatePlan(
        name="plate_01_arm",
        material="PLA/PETG rigid",
        items=tuple(items),
    )
    plans = [plan]

    print(f"Bambu {cfg.title} arm plates ({cfg.slug})")
    print(f"Build area: {cfg.bed_x_mm:.0f} x {cfg.bed_y_mm:.0f} x "
          f"{cfg.bed_z_mm:.0f} mm")
    print(f"Writing {out_dir}")
    print()

    for plan in plans:
        stl_path = os.path.join(out_dir, f"{plan.name}.stl")
        mesh = make_plate_mesh(cfg, plan)
        mesh.export(stl_path)

        threemf_path = os.path.join(out_dir, f"{plan.name}.3mf")
        write_3mf(cfg, plan, threemf_path)

        print(f"  wrote {os.path.basename(stl_path):28s}"
              f" {len(plan.items):2d} parts"
              f" envelope {mesh.extents[0]:5.1f} x {mesh.extents[1]:5.1f} x "
              f"{mesh.extents[2]:5.1f} mm")
        print(f"  wrote {os.path.basename(threemf_path):28s}"
              f" {os.path.getsize(threemf_path) / 1024.0:6.1f} KiB")

    write_manifest(out_dir, plans)
    write_readme(cfg, out_dir, plans)
    print()
    print(f"OK -- {len(plans)} build-plate file(s) (STL + 3MF) plus "
          f"README.md and layout_manifest.csv.")
