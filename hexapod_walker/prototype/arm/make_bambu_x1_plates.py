"""Lay out the optional hexapod arm parts on a Bambu X1 / X1 Carbon plate.

The X1 series uses a **256 x 256 x 256 mm** build volume. The longest arm
part (`arm_forearm.stl` re-export of the leg tibia, ~ 150 mm tip-to-tip)
fits comfortably along either axis, so we keep the whole arm on a single
plate — same as the H2D layout. The tighter bed just runs with a smaller
edge margin and tighter part clearance.

Output:

    prototype/arm/bambu_x1_plates/
        README.md
        layout_manifest.csv
        plate_01_arm.stl    -- combined multi-body STL
        plate_01_arm.3mf    -- multi-object 3MF (preferred slicer import)

Run::

    ./run.sh hexapod_walker/prototype/arm/make_bambu_x1_plates.py
"""

from __future__ import annotations

import os
import sys

# Make sibling `bambu_arm_common.py` importable when called via importlib
# (the H2D script does the same dance).
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from bambu_arm_common import TrayPrinterConfig, generate_plates  # noqa: E402


OUT_DIR = os.path.join(_HERE, "bambu_x1_plates")

X1_CONFIG = TrayPrinterConfig(
    slug="x1",
    title="X1 / X1 Carbon",
    bed_x_mm=256.0,
    bed_y_mm=256.0,
    bed_z_mm=256.0,
    edge_margin_mm=10.0,
    part_clearance_mm=5.0,
)


def main() -> None:
    generate_plates(X1_CONFIG, OUT_DIR)


if __name__ == "__main__":
    main()
