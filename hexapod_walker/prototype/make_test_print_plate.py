"""Build a single Bambu print plate with one copy of every printed leg
part to test-print before committing to six full sets.

Plate contents (one copy of each, oriented for print as defined in
``prepare_xometry_upload.PART_REGISTRY``):

    - coxa_bracket.stl   (yaw servo cradle, drops onto chassis edge)
    - coxa_link.stl      (rotates on yaw axis, carries the hip servo)
    - femur_link.stl     (rotates on hip-pitch axis, carries the knee servo)
    - tibia_link.stl     (rotates on knee axis, ends in the foot socket)

These are the four printed parts that define a single LEG -- once a
plate prints clean and a real servo seats correctly in each of the three
cradles AND the femur + tibia hinge together, you've validated the whole
leg subassembly and can commit to printing all 6 copies.

Output: ``hexapod_walker/prototype/test_print_plate/test_print_plate.stl``
plus a tiny layout manifest.

Run::

    ./run.sh hexapod_walker/prototype/make_test_print_plate.py            # X1 (256 mm)
    ./run.sh hexapod_walker/prototype/make_test_print_plate.py --bed h2d  # H2D (350 mm)
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import asdict

import bambu_tray_common as btc


HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "test_print_plate")

_X1 = btc.TrayPrinterConfig(
    slug="x1",
    title="X1 / X1 Carbon",
    bed_x_mm=256.0,
    bed_y_mm=256.0,
    bed_z_mm=256.0,
    edge_margin_mm=12.0,
    part_clearance_mm=5.0,
    split_long_link_plates=True,
    split_hardware_plate=True,
)

_H2D = btc.TrayPrinterConfig(
    slug="h2d",
    title="H2D",
    bed_x_mm=350.0,
    bed_y_mm=320.0,
    bed_z_mm=325.0,
    edge_margin_mm=12.0,
    part_clearance_mm=6.0,
    split_long_link_plates=False,
    split_hardware_plate=False,
)


_REQUESTS = [
    ("coxa_bracket.stl", 1),
    ("coxa_link.stl", 1),
    ("femur_link.stl", 1),
    ("tibia_link.stl", 1),
]


def _bed_config(slug: str) -> btc.TrayPrinterConfig:
    if slug == "x1":
        return _X1
    if slug == "h2d":
        return _H2D
    raise ValueError(f"unknown bed slug: {slug!r}")


def build_plate(bed: str, out_dir: str = OUT_DIR) -> str:
    """Programmatic entry-point used by ``build_all.py``.

    Builds (or rebuilds) the test print plate STL + layout manifest for
    the requested Bambu bed (``"x1"`` or ``"h2d"``).  Returns the path
    to the written STL.  Always loads the CURRENT printed-part geometry
    from ``hexapod_prototype.py`` -- there is no caching across calls
    (``btc.clear_layout_caches()`` is invoked to make sure we don't
    re-use a stale ``_part_specs()`` set from a previous module run).
    """
    cfg = _bed_config(bed)
    btc.clear_layout_caches()
    parts = btc._part_specs()
    items = tuple(btc._pack_hardware(cfg, parts, requests=_REQUESTS))
    plan = btc.PlatePlan(
        name="test_print_plate",
        material="PLA/PETG rigid",
        items=items,
    )

    os.makedirs(out_dir, exist_ok=True)
    mesh = btc._make_plate_mesh(cfg, plan)
    stl_path = os.path.join(out_dir, f"{plan.name}.stl")
    mesh.export(stl_path)

    counts: dict[str, int] = {}
    for item in items:
        counts[item.part.filename] = counts.get(item.part.filename, 0) + 1

    print(f"Bambu {cfg.title} test plate (bed {cfg.bed_x_mm:.0f} x "
          f"{cfg.bed_y_mm:.0f} mm)")
    print(f"Wrote {stl_path}")
    print(f"  envelope: {mesh.extents[0]:5.1f} x {mesh.extents[1]:5.1f} "
          f"x {mesh.extents[2]:5.1f} mm")
    for name, qty in sorted(counts.items()):
        print(f"  {qty} x {name}")

    manifest_path = os.path.join(out_dir, "layout_manifest.csv")
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["part", "copy", "x_mm", "y_mm", "rotate_z_deg"],
        )
        writer.writeheader()
        for item in items:
            writer.writerow({
                "part": item.part.filename,
                "copy": item.copy_index,
                "x_mm": f"{item.x_mm:.1f}",
                "y_mm": f"{item.y_mm:.1f}",
                "rotate_z_deg": item.rotate_z_deg,
            })
    print(f"Wrote {manifest_path}")
    return stl_path


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bed", choices=("x1", "h2d"), default="x1",
                    help="Which Bambu bed size to lay out for.")
    ap.add_argument("--out-dir", default=OUT_DIR,
                    help="Output directory (default: test_print_plate/)")
    # Match build_prototype_assembly: when invoked via importlib from
    # build_all.py we want argv defaulted to [] (don't read sys.argv,
    # which holds build_all.py's flags).  Passing argv=None preserves
    # the "real CLI" behaviour for direct script invocation.
    if argv is None:
        import sys
        argv = sys.argv[1:]
    args = ap.parse_args(argv)
    build_plate(args.bed, out_dir=args.out_dir)


if __name__ == "__main__":
    main()
