"""Prepare a clean Xometry upload bundle from the prototype STL parts.

Reads the parts defined in `hexapod_prototype.py`, re-orients each one for
3D printing (broad flat face on the build plate, hollow pockets opening
toward +Z, z_min = 0 so every part rests on the bed), and writes them to
`xometry_upload/` with a per-part quantity manifest and a README.

Output:

    xometry_upload/
        README.md                  -- upload + ordering instructions
        manifest.csv               -- per-file: qty, material, color,
                                      finish, volume, notes
        chassis_plate.stl          -- qty 2  (identical top + bottom hex)
        battery_holder.stl         -- qty 1
        electronics_tray.stl       -- qty 1
        coxa_bracket.stl           -- qty 6
        coxa_link.stl              -- qty 6
        femur_link.stl             -- qty 6
        tibia_link.stl             -- qty 6
        foot_pad.stl               -- qty 6  (TPU 95A, separate quote)
        servo_horn_adapter.stl     -- qty 18

For Multi-Jet Fusion (Xometry's PA12 process) the orientation does not
affect part quality — it only affects nesting and cost — so the
re-orientation is mostly cosmetic; for FDM PLA / PETG it really matters,
which is why we bother.
"""

from __future__ import annotations

import csv
import os
from typing import Callable

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

from hexapod_prototype import (
    make_battery_holder,
    make_chassis_top,
    make_coxa_bracket,
    make_coxa_link,
    make_electronics_tray,
    make_femur_link,
    make_foot_pad,
    make_servo_horn_adapter,
    make_tibia_link,
)


HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "xometry_upload")
os.makedirs(OUT_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Re-orientation helpers
# ---------------------------------------------------------------------------

def _rotate(mesh: trimesh.Trimesh, angle_rad: float,
            axis: tuple[float, float, float]) -> trimesh.Trimesh:
    out = mesh.copy()
    out.apply_transform(rotation_matrix(angle_rad, axis))
    return out


def _drop_to_bed(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Translate so z_min = 0 and the part is centred over (x=0, y=0)."""
    out = mesh.copy()
    cx, cy = (out.bounds[0, :2] + out.bounds[1, :2]) / 2.0
    out.apply_translation([-cx, -cy, -float(out.bounds[0, 2])])
    return out


# ---------------------------------------------------------------------------
# Per-part orientation rules
# ---------------------------------------------------------------------------
# Rule of thumb: for FDM, hollow pockets must open toward +Z, broadest flat
# face goes on the bed. For MJF (Xometry's preferred process for these
# parts) orientation is irrelevant, but the manual rules below also produce
# a sensible visual layout.

def _reorient_chassis_plate(mesh):
    """Hex 4 mm plate — already flat. Just drop to z=0."""
    return _drop_to_bed(mesh)


def _reorient_battery_holder(mesh):
    """Open-top tray: top face is +Z, drop to bed."""
    return _drop_to_bed(mesh)


def _reorient_electronics_tray(mesh):
    """Flat 3 mm plate."""
    return _drop_to_bed(mesh)


def _reorient_coxa_bracket(mesh):
    """U-cradle with mounting pad. Cradle's open top is at +Z in the
    bracket's assembly local frame — so flat-bottom of the cradle on
    the bed, mounting pad as a vertical fin sticking out the side."""
    return _drop_to_bed(mesh)


def _reorient_coxa_link(mesh):
    """The hip-pitch cradle hangs in -Z below the arm, with its open
    face pointing -Y. Rotate +90 deg about X to put the open face up
    (-Y -> +Z)."""
    out = _rotate(mesh, np.pi / 2, [1, 0, 0])
    return _drop_to_bed(out)


def _reorient_femur_link(mesh):
    """Flat plate with knee cradle hanging off in -Y. Rotate +90 deg
    about X so the cradle's open face points +Z."""
    out = _rotate(mesh, np.pi / 2, [1, 0, 0])
    return _drop_to_bed(out)


def _reorient_tibia_link(mesh):
    """Flat plate (130 x 6 x 18 mm) with knee pad and foot socket.
    Lay it flat on the bed -- broadest face (X-Z plane) downward.
    Rotate +90 deg about X to put the spar's broad face on the bed
    (Y becomes vertical, so layers are 6 mm of part)."""
    out = _rotate(mesh, np.pi / 2, [1, 0, 0])
    return _drop_to_bed(out)


def _reorient_foot_pad(mesh):
    """Cylindrical disc with hub, hub up. Already in correct orientation."""
    return _drop_to_bed(mesh)


def _reorient_servo_horn_adapter(mesh):
    """Flat 4 mm star, recess on -Z side. The recess (counter-bore for
    the plastic horn body) sits on the build plate; the bolt-pattern
    face is up. Already correct."""
    return _drop_to_bed(mesh)


# ---------------------------------------------------------------------------
# Part registry
# ---------------------------------------------------------------------------
# Each entry: (filename, make-fn, reorient-fn, qty, material, color, finish,
#              notes for the manifest)
# The "material" column is a recommendation only; you can change it on the
# Xometry quote page.
PART_REGISTRY: list[tuple[str,
                          Callable[[], trimesh.Trimesh],
                          Callable[[trimesh.Trimesh], trimesh.Trimesh],
                          int, str, str, str, str]] = [
    ("chassis_plate.stl",        make_chassis_top,         _reorient_chassis_plate,
     2, "MJF PA12",      "white", "as-printed",
     "Identical top + bottom hex plate. Order qty 2."),

    ("battery_holder.stl",       make_battery_holder,      _reorient_battery_holder,
     1, "MJF PA12",      "white", "as-printed",
     "Open-top LiPo tray; velcro slots through the long walls."),

    ("electronics_tray.stl",     make_electronics_tray,    _reorient_electronics_tray,
     1, "MJF PA12",      "white", "as-printed",
     "Mounts Arduino + 2x PCA9685; standoffs are part of the geometry."),

    ("coxa_bracket.stl",         make_coxa_bracket,        _reorient_coxa_bracket,
     6, "MJF PA12",      "white", "as-printed",
     "Holds the yaw servo. Cradle pocket prints opening +Z."),

    ("coxa_link.stl",            make_coxa_link,           _reorient_coxa_link,
     6, "MJF PA12",      "white", "as-printed",
     "Yaw-driven arm; oriented so the hip-pitch cradle opens +Z."),

    ("femur_link.stl",           make_femur_link,          _reorient_femur_link,
     6, "MJF PA12",      "white", "as-printed",
     "Thigh; oriented so the knee servo cradle opens +Z."),

    ("tibia_link.stl",           make_tibia_link,          _reorient_tibia_link,
     6, "MJF PA12",      "white", "as-printed",
     "Shin link, ends in foot socket. Lay flat for FDM."),

    ("foot_pad.stl",             make_foot_pad,            _reorient_foot_pad,
     6, "FDM TPU 95A",   "black", "as-printed",
     "*** SEPARATE QUOTE *** -- needs flexible TPU for grip. "
     "If TPU isn't available, FDM PLA works but the foot will slip."),

    ("servo_horn_adapter.stl",   make_servo_horn_adapter,  _reorient_servo_horn_adapter,
     18, "MJF PA12",     "white", "as-printed",
     "Tiny part -- consider FDM PLA at home if you have a printer."),
]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print(f"Preparing Xometry upload bundle in {OUT_DIR} ...")
    print()

    rows: list[dict[str, object]] = []
    total_volume_cm3 = 0.0
    total_part_count = 0

    for (name, make_fn, reorient_fn,
         qty, material, color, finish, notes) in PART_REGISTRY:
        mesh = make_fn()
        mesh = reorient_fn(mesh)

        if not mesh.is_watertight:
            print(f"  WARN: {name} is not watertight after reorient; "
                  f"euler={mesh.euler_number}")

        path = os.path.join(OUT_DIR, name)
        mesh.export(path)

        per_unit_cm3 = float(mesh.volume) / 1000.0   # mm^3 -> cm^3
        total_unit_cm3 = per_unit_cm3 * qty
        total_volume_cm3 += total_unit_cm3
        total_part_count += qty

        rows.append({
            "filename": name,
            "qty": qty,
            "material": material,
            "color": color,
            "finish": finish,
            "envelope_x_mm": f"{mesh.extents[0]:.1f}",
            "envelope_y_mm": f"{mesh.extents[1]:.1f}",
            "envelope_z_mm": f"{mesh.extents[2]:.1f}",
            "volume_cm3_per_part": f"{per_unit_cm3:.2f}",
            "volume_cm3_total": f"{total_unit_cm3:.2f}",
            "notes": notes,
        })

        print(f"  wrote {name:30s}  qty={qty:>2d}  "
              f"{mesh.extents[0]:6.1f} x {mesh.extents[1]:6.1f} x "
              f"{mesh.extents[2]:6.1f} mm   {per_unit_cm3:5.2f} cm^3/each")

    # Write manifest.csv
    manifest_path = os.path.join(OUT_DIR, "manifest.csv")
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print()
    print(f"OK -- {len(PART_REGISTRY)} unique STLs ({total_part_count} parts "
          f"total) written.")
    print(f"   Total material volume:  {total_volume_cm3:6.1f} cm^3 "
          f"({total_volume_cm3 * 1.05:5.1f} g of PA12 at 1.05 g/cm^3)")
    print(f"   Manifest:               {manifest_path}")
    print(f"   Upload dir:             {OUT_DIR}")
    print()
    print("Next steps: see xometry_upload/README.md.")


if __name__ == "__main__":
    main()
