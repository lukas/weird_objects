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
        chassis_top.stl            -- qty 1  (small 140 mm hex deck)
        chassis_bottom.stl         -- qty 1  (full 200 mm structural hex)
                # Design F (May 2026): coxa_bracket.stl retired -- the yaw
        # servo now drops INTO an integrated cradle inside
        # chassis_bottom (see make_chassis_bottom).
        coxa_link.stl              -- qty 6
        # Bearing-sandwich leg: the WHOLE femur is ONE printed part
        # (Jul 2026 merge #2 of hip yoke + solid spar + knee bracket);
        # the tibia SEGMENT is a bought Ø8 CF tube.
        femur_link.stl             -- qty 6
        tibia_knee_yoke.stl        -- qty 6
        tibia_foot_fitting.stl     -- qty 6
        foot_pad.stl               -- qty 6  (TPU 95A; all legs)
        # Short CF legs 0/4: print extra_stl/tibia_foot_fitting_plus4.stl
        # instead of a thicker pad (retired foot_pad_plus4).
        # June 2026 disc-horn switch: servo_horn_adapter.stl retired --
        # the links now bolt directly onto the 20 mm aluminum 25T disc
        # horn that seats on the servo spline.

For Multi-Jet Fusion (Xometry's PA12 process) the orientation does not
affect part quality — it only affects nesting and cost — so the
re-orientation is mostly cosmetic; for FDM PLA / PETG it really matters,
which is why we bother.
"""

from __future__ import annotations

import csv
import os
import sys
from typing import Callable

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(SCRIPT_DIR)
if PROTO_DIR not in sys.path:
    sys.path.insert(0, PROTO_DIR)

import hexapod_prototype as hp  # noqa: E402
from hexapod_prototype import (
    make_chassis_bottom,
    make_chassis_top,
    make_coxa_link_part,
    make_femur_link_part,
    make_foot_pad,
    make_servo_clamp_cap,
    make_tibia_foot_fitting,
    make_tibia_knee_yoke,
    make_yaw_bearing_cap,
    make_yaw_servo_retainer,
)


HERE = PROTO_DIR
OUT_DIR = os.path.join(HERE, "xometry_upload")
# NOTE: OUT_DIR is only created inside main().  This module is ALSO
# imported purely for PART_REGISTRY (the canonical print poses used by
# the verifier's flat-bottom check), and a module-level makedirs kept
# resurrecting an empty xometry_upload/ on every verify run after the
# bundle was removed from the repo (Aug 2026 cleanup).


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
    """Hex plate — already flat. Just drop to z=0.

    Jun 2026 single-part merge: ``chassis_bottom`` is the whole bottom plate
    (flat plate + bearing tower + the folded-in flat floor slab).  Its broad
    flat -6 mm underside is the bed face, so it prints face-DOWN exactly as
    built (bearing tower up) — drop to bed, no flip."""
    return _drop_to_bed(mesh)


def _reorient_deck_tray(mesh):
    """Flat deck tray (Uno Q / buck): bosses point +Z, drop to bed."""
    return _drop_to_bed(mesh)


def _reorient_coxa_link(mesh):
    """One-piece coxa (Aug 2026 merge of the yaw turntable hub + hip
    bracket).  Lay it on its SIDE (rotate -90 deg about Y: the yaw axis
    goes horizontal, the cradle's outboard end wall becomes the bed face)
    -- the flattest stable pose; both the boss-down and cradle-down
    uprights leave 20+ mm of overhang hanging below the main plane."""
    out = _rotate(mesh, -np.pi / 2, [0, 1, 0])
    return _drop_to_bed(out)


def _lay_flat(mesh):
    """Generic FDM reorient for the bearing-sandwich leg socket fittings
    (hip yoke, knee bracket, knee yoke, foot fitting).  Picks whichever
    of the three axis-aligned orientations leaves the SHORTEST Z extent
    so the part lies on its broadest face (largest bed contact, lowest
    centre of gravity, least support).  The CF-tube socket bore ends up
    roughly horizontal -- the user can spin individual parts in Bambu
    Studio if a particular bore would print better vertically."""
    best = None
    for axis, angle in ((None, 0.0), ((1, 0, 0), np.pi / 2), ((0, 1, 0), np.pi / 2)):
        out = mesh.copy()
        if axis is not None:
            out.apply_transform(rotation_matrix(angle, axis))
        z_extent = float(out.extents[2])
        if best is None or z_extent < best[0]:
            best = (z_extent, out)
    return _drop_to_bed(best[1])


def _reorient_foot_pad(mesh):
    """Cylindrical disc with hub, hub up. Already in correct orientation."""
    return _drop_to_bed(mesh)


def _reorient_yaw_bearing_cap(mesh):
    """yaw_bearing_cap: built open-pocket-UP in coxa-local (z[-1,+6]).  Flip
    180 deg about X so the broad Phi 44 top-rim annulus lies FLAT on the bed
    (the clean Phi 37 through-bore opens straight up); the 3 join-bolt ear
    lugs point UP.  No internal overhang -- the bore is a clean through-hole,
    so no supports are needed."""
    out = mesh.copy()
    out.apply_transform(rotation_matrix(np.pi, (1, 0, 0)))
    return _drop_to_bed(out)


def _reorient_carapace(mesh):
    """Spider carapace dome: print rim-down (open skirt on the bed, apex up)
    so the shell self-supports as a gentle overhang and the 4 mount-foot
    insert pockets open downward onto the bed.  Built that way already; just
    drop to z = 0."""
    return _drop_to_bed(mesh)


# June 2026 disc-horn switch: _reorient_servo_horn_adapter removed along
# with make_servo_horn_adapter (printed adapter retired -- links bolt
# straight onto the aluminum disc horn).


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
    ("chassis_top.stl",          make_chassis_top,         _reorient_chassis_plate,
     1, "MJF PA12",      "white", "as-printed",
     "Top deck (140 mm flat-to-flat hex). Carries battery + electronics "
     "+ optional arm; smaller than bottom so the legs sweep clear."),

    ("chassis_bottom.stl",       make_chassis_bottom,      _reorient_chassis_plate,
     1, "MJF PA12",      "white", "as-printed",
     "Single merged bottom plate (200 mm flat-to-flat hex; Jun 2026 re-merge "
     "of the old HIGH/LOW print split). The flat plate + bearing tower + a "
     "solid flat floor slab folded onto the underside (perimeter ~8 mm thick) "
     "give a broad flat -6 mm bed face, so it prints face-DOWN (bearing tower "
     "+ tray bosses up) with NO supports. Takes the six coxa-bracket M3 bolts; "
     "the yaw servo body drops through a per-leg cutout and is captured by the "
     "bolt-on yaw_servo_retainer stirrup (2 self-tap pilots per leg)."),



    # Aug 2026 merge: the coxa is ONE printed part again -- the Jun 2026
    # hub/bracket split (4x M3 join bolts) is retired.  Five vertical
    # head-access shafts through the foot plate let the 4x M3x20 disc-horn
    # bolts + centre spline screw be dropped in and torqued from above
    # (through the still-empty hip servo well) after the part rides the
    # spaced 6706 pair in the chassis tower.
    ("coxa_link.stl",            make_coxa_link_part,      _reorient_coxa_link,
     6, "MJF PA12",      "white", "as-printed",
     "One-piece coxa (Aug 2026 merge): yaw turntable hub (bolts DOWN onto "
     "the 20 mm disc horn, 4x M3 clamp-through torque-only holes; rides the "
     "inner races of the spaced 6706-2RS pair) fused to the hip-pitch servo "
     "cradle (symmetric disc-horn sandwich; passive horn on the rear idler "
     "boss, no 688 bearing).  5 head-access shafts reach the horn screws "
     "through the hip servo well."),

    # Bearing-sandwich leg (Jun 2026): the tibia SEGMENT is a bought Ø8
    # carbon-fibre tube (cut to length, epoxied into the printed sockets
    # below) -- NOT printed.  The assembled tibia_link visual/sim mesh
    # (yoke + SOLID tube + fitting) must not be printed.
    # Jul 2026 merge #2 (user: "combine the femur knee bracket with the
    # femur hip yoke and make that connection very solid"): the WHOLE femur
    # is ONE printed part -- hip yoke + solid Ø14 spar + knee bracket.  Two
    # parts fewer per leg vs the original design, no slip fits, no pins.
    ("femur_link.stl",           make_femur_link_part,     _lay_flat,
     6, "MJF PA12",      "white", "as-printed",
     "ONE-PIECE femur: hip moving-yoke (bolts to the hip disc horn), "
     "SOLID Ø14 spar bridging the 90 mm hip-to-knee span, and the knee "
     "servo cradle + 688 bearing housing -- a single printed body with "
     "no sockets or retention pins."),

    ("tibia_knee_yoke.stl",      make_tibia_knee_yoke,     _lay_flat,
     6, "MJF PA12",      "white", "as-printed",
     "Tibia knee moving-yoke: bolts to the knee disc horn, sockets the "
     "Ø8 CF tibia tube."),

    ("tibia_foot_fitting.stl",   make_tibia_foot_fitting,  _lay_flat,
     6, "MJF PA12",      "white", "as-printed",
     "Tibia foot fitting: sockets the far end of the Ø8 CF tibia tube "
     "and carries the foot-pad hinge tang."),

    ("foot_pad.stl",             make_foot_pad,            _reorient_foot_pad,
     6, "FDM TPU 95A",   "black", "as-printed",
     "*** SEPARATE QUOTE *** -- TPU. Same pad on all 6 legs. Short CF legs "
     "0/4 use extra_stl/tibia_foot_fitting_plus4.stl instead of a thicker pad. "
     "If TPU isn't available, PLA works but the foot will slip."),

    ("yaw_servo_retainer.stl",   make_yaw_servo_retainer,  _drop_to_bed,
     6, "MJF PA12",      "white", "as-printed",
     "Anti-rotation saddle under each yaw servo (Aug 2026 flat-belly "
     "rework: the permanent 38 mm ground stand is removed -- the belly is "
     "flat except the hanging servos + saddles).  U-channel open on +X, "
     "central wire drop window.  4× M3 chassis anchors + 4× M2.5 rear-case "
     "self-taps, all drivvable straight up from the open air below."),

    ("yaw_bearing_cap.stl",      make_yaw_bearing_cap,     _reorient_yaw_bearing_cap,
     6, "MJF PA12",      "white", "as-printed",
     "TOP half of the SPLIT yaw-bearing tower (Jun 2026 insertion fix). "
     "Holds the UPPER 6706 outer race in a clean Phi 37 through-bore; bolts "
     "DOWN onto each chassis_bottom tower with 3 x M3 x 8 self-tap join "
     "screws in outboard ear lugs (concentricity comes from the bolts into "
     "the tower pilots). Lets each race drop onto an OPEN face during "
     "assembly (the old one-piece tower trapped both races)."),

    ("servo_clamp_cap.stl",      make_servo_clamp_cap,     _lay_flat,
     12, "MJF PA12",     "white", "as-printed",
     "Clamshell cap closing the OPEN +Y face of each sandwich-joint servo "
     "cradle (hip-pitch coxa_link + the femur_link knee cradle = 2/leg). Bolts "
     "-Y with 2 x M3 into the cradle's +/-X wall ends to clamp the STS3215 "
     "body; the tongue now seats FLUSH on the body's +Y face so the bolts "
     "trap it with no slop; centre is bored so the disc horn spins free. The "
     "2 bolt heads RECESS into Phi 6 mm counterbores (flush with the +Y face) "
     "so the swept femur/tibia yoke clears them."),

    # Jul 2026 stock-horn refit: passive_horn_adapter.stl removed -- the
    # STS3215's STOCK metal passive horn centres itself on the rear idler
    # boss (its bore rides the boss, seating flush on the back face), so no
    # printed centering/standoff bushing is quoted.


    # June 2026 disc-horn switch: servo_horn_adapter.stl removed -- each
    # moving yoke now bolts directly onto the 20 mm aluminum 25T disc horn
    # that seats on the servo spline (DISC_HORN_COLLAR_OD recess + 4 x
    # DISC_HORN_BOLT_PCD = 14 mm pattern cut into the yoke pad in
    # make_coxa_link / make_femur_link_part / make_tibia_knee_yoke).  No
    # 18 x adapter discs to quote.
]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Preparing Xometry upload bundle in {OUT_DIR} ...")
    print()

    rows: list[dict[str, object]] = []
    total_volume_cm3 = 0.0
    total_part_count = 0

    for (name, make_fn, reorient_fn,
         qty, material, color, finish, notes) in PART_REGISTRY:
        mesh = make_fn()
        mesh = reorient_fn(mesh)
        mesh = hp._heal_for_export(mesh)

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
