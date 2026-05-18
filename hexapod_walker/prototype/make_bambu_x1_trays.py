"""Lay out prototype hexapod parts for Bambu Lab X1 / X1 Carbon (256 mm cube).

The X1 series uses a **256 x 256 x 256 mm** build volume. Layout differs from the
H2D preset:

    - Tibia and femur links are **separate plates** (the combined two-row plate
      used on the H2D is too tall in Y for a 256 mm bed).
    - Battery holder + electronics tray share one plate (side by side).
    - Coxa brackets and links pack on their own plate.
    - The 18 servo horn adapters use a compact **6 × 3 grid** on one plate.

Output: ``bambu_x1_trays/`` — README, layout_manifest.csv, and **eight** plate STLs.

Run::

    ./run.sh hexapod_walker/prototype/make_bambu_x1_trays.py
"""

from __future__ import annotations

import os

from bambu_tray_common import TrayPrinterConfig, generate_trays


HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "bambu_x1_trays")

X1_CONFIG = TrayPrinterConfig(
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


def main() -> None:
    generate_trays(X1_CONFIG, OUT_DIR)


if __name__ == "__main__":
    main()
