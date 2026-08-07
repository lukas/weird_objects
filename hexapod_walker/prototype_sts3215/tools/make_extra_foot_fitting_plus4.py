"""Write ``extra_stl/tibia_foot_fitting_plus4.stl`` for short CF legs 0/4.

The +4 mm longer foot fitting replaces the old thicker ``foot_pad_plus4``
so tip reach matches the 128 mm legs with a normal TPU pad.

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_extra_foot_fitting_plus4.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "tibia_foot_fitting_plus4.stl"


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    mesh = hp.make_tibia_foot_fitting_plus4()
    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    nom = hp.make_tibia_foot_fitting()
    print(f"wrote {path}")
    print(f"  nominal fitting extents: {nom.extents}")
    print(f"  plus4 fitting extents:   {mesh.extents}")
    print(f"  hinge X: {hp.FOOT_FITTING_HINGE_X:.1f} → "
          f"{hp.FOOT_FITTING_HINGE_X + hp.FOOT_FITTING_SHORT_EXTRA:.1f} mm "
          f"(+{hp.FOOT_FITTING_SHORT_EXTRA:.0f})")
    print("Use on short CF legs 0/4 with a normal foot_pad.stl.")


if __name__ == "__main__":
    main()
