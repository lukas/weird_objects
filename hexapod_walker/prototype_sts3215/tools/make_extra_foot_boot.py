"""Write ``extra_stl/foot_boot.stl`` for the standard TPU foot boot.

This is the quick bench/slicer path for the as-built boot: all six legs
use the same standard spherical-ended TPU 95A boot, nominal 9 mm OD, with
the furthest rounded tip at the 150 mm knee-centre-to-tip station.

Run from the repo root:

    uv run --with numpy --with trimesh \
        python hexapod_walker/prototype_sts3215/tools/make_extra_foot_boot.py
"""
from __future__ import annotations

import os
import sys
import math

import trimesh
from trimesh.transformations import rotation_matrix

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "foot_boot.stl"


def _drop_to_bed(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    out = mesh.copy()
    cx, cy = (out.bounds[0, :2] + out.bounds[1, :2]) / 2.0
    out.apply_translation([-cx, -cy, -float(out.bounds[0, 2])])
    return out


def print_oriented_foot_boot() -> trimesh.Trimesh:
    """Return the boot with mouth face down and dome tip upward."""
    mesh = hp.make_foot_boot()
    mesh.apply_transform(rotation_matrix(-math.pi / 2.0, [0, 1, 0]))
    return _drop_to_bed(mesh)


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    raw = hp.make_foot_boot()
    mesh = print_oriented_foot_boot()
    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    print(f"wrote {path}")
    print(f"  source-frame bounds: {raw.bounds.round(3).tolist()}")
    print(f"  print-frame extents: {mesh.extents.round(3).tolist()} mm")
    print(f"  nominal OD: {hp.FOOT_BOOT_OD:.1f} mm")
    print(f"  knee-centre-to-tip length: {hp.TIBIA_LENGTH:.1f} mm")
    print("  print: mouth face down, dome up; TPU 95A, no supports")


if __name__ == "__main__":
    main()
