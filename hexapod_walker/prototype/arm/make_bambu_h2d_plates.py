"""Lay out the optional hexapod arm parts on a Bambu H2D Carbon build plate.

The arm is small enough (~ 200 g of plastic) that everything for ONE arm
fits on a single H2D single-nozzle plate (effective build area
325 x 320 x 325 mm).

Output:

    prototype/arm/bambu_h2d_plates/
        README.md
        layout_manifest.csv
        plate_01_arm.stl    -- combined multi-body STL
        plate_01_arm.3mf    -- multi-object 3MF (preferred slicer import)

Run::

    ./run.sh hexapod_walker/prototype/arm/make_bambu_h2d_plates.py
"""

from __future__ import annotations

import os
import sys

# Make sibling `bambu_arm_common.py` importable when this script is run
# from anywhere (run.sh cds into this dir, but child callers from
# prototype/build_all.py invoke us via importlib).
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from bambu_arm_common import TrayPrinterConfig, generate_plates  # noqa: E402


OUT_DIR = os.path.join(_HERE, "bambu_h2d_plates")

H2D_CONFIG = TrayPrinterConfig(
    slug="h2d",
    title="H2D Carbon (single nozzle)",
    bed_x_mm=325.0,
    bed_y_mm=320.0,
    bed_z_mm=325.0,
    edge_margin_mm=12.0,
    part_clearance_mm=8.0,
)


def main() -> None:
    generate_plates(H2D_CONFIG, OUT_DIR)


if __name__ == "__main__":
    main()
