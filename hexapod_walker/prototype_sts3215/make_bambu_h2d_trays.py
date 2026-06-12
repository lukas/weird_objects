"""Lay out prototype hexapod parts on Bambu H2D Carbon build plates.

See ``bambu_tray_common.py`` for geometry. H2D single-nozzle effective area
is published as 325 x 320 x 325 mm.

Output: ``bambu_h2d_trays/`` — README, layout_manifest.csv, and plate STLs.
"""

from __future__ import annotations

import os

from bambu_tray_common import TrayPrinterConfig, generate_trays


HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "bambu_h2d_trays")

H2D_CONFIG = TrayPrinterConfig(
    slug="h2d",
    title="H2D Carbon (single nozzle)",
    bed_x_mm=325.0,
    bed_y_mm=320.0,
    bed_z_mm=325.0,
    edge_margin_mm=12.0,
    part_clearance_mm=8.0,
    # Bearing-sandwich leg: the femur/tibia segments are bought Ø8 CF
    # tubes, so there are no long printed links to split anymore -- only
    # small socket fittings that bottom-left pack onto shared plates.
    # ``split_long_link_plates`` is retained for dataclass compatibility
    # but no longer consumed by build_plate_plans.  The 325 mm bed fits
    # the battery + electronics trays together, so no hardware split.
    split_long_link_plates=False,
    split_hardware_plate=False,
)


def main() -> None:
    generate_trays(H2D_CONFIG, OUT_DIR)


if __name__ == "__main__":
    main()
