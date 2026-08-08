"""Write ``extra_stl/foot_boot_plus4.stl`` for the short CF legs 0/4.

Aug 2026: the hinged tibia_foot_fitting + foot_pad are retired; each
tibia tube takes a pressed-on TPU 95A ``foot_boot`` instead.  Legs 0/4
have CF tubes cut 4 mm short, so their boot gets a +4 mm longer solid
tip (FOOT_BOOT_SHORT_EXTRA) and every tip still lands at tibia-local
x = TIBIA_LENGTH.  Print in TPU 95A, tip face on the bed.

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_extra_foot_boot_plus4.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "foot_boot_plus4.stl"


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    mesh = hp.make_foot_boot_plus4()
    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    nom = hp.make_foot_boot()
    print(f"wrote {path}")
    print(f"  nominal boot extents: {nom.extents}")
    print(f"  plus4 boot extents:   {mesh.extents}")
    print(f"  tip length: {hp.FOOT_BOOT_TIP_L:.1f} -> "
          f"{hp.FOOT_BOOT_TIP_L + hp.FOOT_BOOT_SHORT_EXTRA:.1f} mm "
          f"(+{hp.FOOT_BOOT_SHORT_EXTRA:.0f})")
    print("Use on short CF legs 0/4 (normal foot_boot.stl on the rest).")


if __name__ == "__main__":
    main()
