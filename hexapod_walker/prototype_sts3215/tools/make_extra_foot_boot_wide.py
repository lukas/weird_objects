"""Write ``extra_stl/foot_boot_wide.stl`` -- EXPERIMENTAL wide dome boot.

Aug 19 2026 (user + GPT walking-gait review): third leg of the PETG
foot-test trio.  Same dome geometry as the production ``foot_boot``
(Phi 8.1 bore, 20 mm socket, dome apex at tibia-local x = TIBIA_LENGTH)
but Phi FOOT_BOOT_WIDE_OD = 17 OD (dome R 8.5 vs 7).

The trio, walked on the same gait to compare catching / sudden release:

  1. solid dome   = stl_prototype/foot_boot.stl, sliced solid
  2. hollow dome  = stl_prototype/foot_boot.stl, sliced 2 walls / ~8%
  3. wide hollow  = THIS part,                   sliced 2 walls / ~8%

PETG notes: the Phi 8.1 bore is a rigid slip fit (same bore the tibia
yoke socket proved on the bench) -- PETG will not grip like TPU, so add
a CA/epoxy dab.  Print MOUTH face on the bed (the bore's blind end is a
45-deg internal cone, nothing bridges) -- a dome tip cannot be the bed
face.

EXPERIMENTAL: not in the production print set, the scene, or the BOM.
If the bench prefers the wide dome, fold FOOT_BOOT_WIDE_OD into
FOOT_BOOT_OD (and bump mujoco_prototype's FOOT_R with it -- the sim
contact sphere radius is derived from FOOT_BOOT_OD).

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_extra_foot_boot_wide.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "foot_boot_wide.stl"


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    mesh = hp.make_foot_boot_wide()
    assert mesh.is_watertight, "wide boot mesh is not watertight"
    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    nom = hp.make_foot_boot()
    print(f"wrote {path}")
    print(f"  production boot extents: {nom.extents}")
    print(f"  wide boot extents:       {mesh.extents}")
    print(f"  dome OD Phi {hp.FOOT_BOOT_OD:.0f} -> Phi {hp.FOOT_BOOT_WIDE_OD:.0f} "
          f"(bore Phi {hp.FOOT_BOOT_BORE_D:.1f} x "
          f"{hp.FOOT_BOOT_SOCKET_DEPTH:.0f} unchanged)")
    print("Print PETG, MOUTH face on the bed; slice 2 walls / ~8% infill "
          "for the hollow-compliance variant.")


if __name__ == "__main__":
    main()
