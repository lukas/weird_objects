"""Generate every printable tabletop-prototype STL bundle.

Run from the repository root:

    ./run.sh hexapod_walker/prototype_sts3215/build_all.py

This writes:
    - stl_prototype/   slicer-ready printables only
    - stl_reference/   bought-part / fused-link visuals (MuJoCo, BuildViz)
    - artifacts/assembly/  visual category STLs (optional)
    - xometry_upload/  upload-ready per-part STLs and manifest
      (OPT-IN via ``--xometry``; the bundle is no longer checked in --
      Aug 2026 cleanup -- and is only generated when ordering prints)

Bambu tray / test-plate packing was removed — print individuals from
``stl_prototype/`` (or run ``--xometry`` for a service order bundle).
"""

from __future__ import annotations

import argparse
import importlib
import os
import sys
import time
from collections.abc import Callable


HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "scripts")
if SCRIPTS not in sys.path:
    # build_prototype_assembly / prepare_xometry_upload live under scripts/
    sys.path.insert(0, SCRIPTS)


def _run(label: str, module_name: str) -> None:
    print()
    print("=" * 72)
    print(label)
    print("=" * 72)
    started = time.monotonic()
    module = importlib.import_module(module_name)
    main: Callable[..., None] = module.main
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
        help="Skip artifacts/assembly/*.stl visual exports.",
    )
    parser.add_argument(
        "--xometry",
        action="store_true",
        help=(
            "ALSO build the xometry_upload/*.stl order bundle "
            "(opt-in since Aug 2026; only needed when ordering prints)."
        ),
    )
    # Deprecated: xometry is now opt-in, so --skip-xometry is a no-op.
    # Kept so old Makefile / muscle-memory commands don't crash.
    parser.add_argument("--skip-xometry", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help=(
            "Also build the OPTIONAL 5-DOF arm add-on (arm/). "
            "Regenerates stl_arm/*.stl and runs the arm-inclusive "
            "assembly preview."
        ),
    )
    # Deprecated flags kept so old Makefile / muscle-memory commands
    # don't crash (trays were removed).
    parser.add_argument("--skip-bambu", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--skip-test-plate", action="store_true",
                        help=argparse.SUPPRESS)
    args = parser.parse_args()

    os.chdir(HERE)
    print(f"Generating tabletop hexapod prototype STL bundles in {HERE}")

    _run("Individual prototype STLs", "hexapod_prototype")
    if not args.skip_assembly:
        if args.with_arm:
            os.environ["HEXAPOD_PROTOTYPE_WITH_ARM"] = "1"
        try:
            _run("Prototype assembly STLs", "build_prototype_assembly")
        finally:
            os.environ.pop("HEXAPOD_PROTOTYPE_WITH_ARM", None)
    if args.xometry:
        _run("Xometry upload STLs", "prepare_xometry_upload")
    if args.with_arm:
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
            build_bambu=False,
        )

    print()
    print("Done. Printables: stl_prototype/  ·  sim visuals: stl_reference/")
    if args.with_arm:
        print("Optional arm: arm/stl_arm/ (see arm/ARM.md).")


if __name__ == "__main__":
    main()
