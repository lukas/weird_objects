"""Generate every printable tabletop-prototype STL bundle.

Run from the repository root:

    ./run.sh hexapod_walker/prototype/build_all.py

This writes:
    - stl_prototype/       individual slicer-ready prototype parts
    - prototype_assembly/  visual assembly/category STLs
    - xometry_upload/      upload-ready per-part STLs and manifest
    - bambu_h2d_trays/     laid-out Bambu H2D Carbon build-plate STLs
    - bambu_x1_trays/      laid-out Bambu X1 / X1 Carbon (256 mm) plate STLs
    - test_print_plate/    3-part servo-well test plate (Bambu X1 layout)
"""

from __future__ import annotations

import argparse
import importlib
import os
import time
from collections.abc import Callable


HERE = os.path.dirname(os.path.abspath(__file__))


def _run(label: str, module_name: str) -> None:
    print()
    print("=" * 72)
    print(label)
    print("=" * 72)
    started = time.monotonic()
    module = importlib.import_module(module_name)
    main: Callable[..., None] = module.main
    # Modules invoked from build_all should receive argv=[] explicitly --
    # otherwise their argparse calls re-read build_all's own sys.argv
    # (e.g. ``--skip-xometry``) and crash with "unrecognized arguments".
    # Modules that don't take argv (older bundle scripts) work the same
    # either way because their main() signature ignores the kwarg.
    try:
        main(argv=[])
    except TypeError:
        main()
    elapsed = time.monotonic() - started
    print(f"{label} finished in {elapsed:.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-assembly",
        action="store_true",
        help="Skip the visual prototype_assembly/*.stl exports.",
    )
    parser.add_argument(
        "--skip-xometry",
        action="store_true",
        help="Skip the xometry_upload/*.stl export bundle.",
    )
    parser.add_argument(
        "--skip-bambu",
        action="store_true",
        help="Skip Bambu build-plate trays (H2D and X1).",
    )
    parser.add_argument(
        "--skip-test-plate",
        action="store_true",
        help=(
            "Skip the 3-part servo-well test print plate "
            "(test_print_plate/test_print_plate.stl, X1 layout)."
        ),
    )
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help=(
            "Also build the OPTIONAL 5-DOF arm add-on (prototype/arm/). "
            "Regenerates stl_arm/*.stl + arm Bambu plates and runs the "
            "arm-inclusive assembly preview. Default behaviour (no flag) "
            "is unchanged."
        ),
    )
    args = parser.parse_args()

    os.chdir(HERE)
    print(f"Generating tabletop hexapod prototype STL bundles in {HERE}")

    _run("Individual prototype STLs", "hexapod_prototype")
    if not args.skip_assembly:
        # Forward --with-arm to the assembly stage so it can place the
        # arm on top of the standing hexapod when requested.
        if args.with_arm:
            os.environ["HEXAPOD_PROTOTYPE_WITH_ARM"] = "1"
        try:
            _run("Prototype assembly STLs", "build_prototype_assembly")
        finally:
            os.environ.pop("HEXAPOD_PROTOTYPE_WITH_ARM", None)
    if not args.skip_xometry:
        _run("Xometry upload STLs", "prepare_xometry_upload")
    if not args.skip_bambu:
        _run("Bambu H2D Carbon tray STLs", "make_bambu_h2d_trays")
        _run("Bambu X1 / X1 Carbon tray STLs", "make_bambu_x1_trays")
    if not args.skip_test_plate:
        # The 3-part servo-well test plate is the smallest reasonable
        # smoke print before committing to all 8 X1 (or 6 H2D) plates.
        # It rides on the same hexapod_prototype.py geometry that just
        # got regenerated above, so this step exists to make sure the
        # test plate STL never falls out of sync with the rest of the
        # bundle.  X1 layout by default (the more-constrained bed); run
        # ``make_test_print_plate.py --bed h2d`` separately for an H2D
        # version.
        _run("Test print plate STL (X1 layout)", "make_test_print_plate")
    if args.with_arm:
        # Load `prototype/arm/integrate.py` directly by file path so the
        # `arm/` package and the sibling `arm.py` module don't collide
        # on sys.path.  Keeps the default codepath untouched.
        import importlib.util  # noqa: WPS433
        _arm_dir = os.path.join(HERE, "arm")
        if _arm_dir not in __import__("sys").path:
            __import__("sys").path.insert(0, _arm_dir)
        _spec = importlib.util.spec_from_file_location(
            "arm_integrate", os.path.join(_arm_dir, "integrate.py"))
        _arm_integrate = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_arm_integrate)
        _arm_integrate.build_with_arm(
            render_preview=True,
            build_bambu=not args.skip_bambu,
        )

    print()
    print("Done. Tray guides: bambu_h2d_trays/README.md and bambu_x1_trays/README.md.")
    if args.with_arm:
        print("Optional arm: prototype/arm/stl_arm/ + bambu_*_plates/ "
              "(see prototype/arm/ARM.md).")


if __name__ == "__main__":
    main()
