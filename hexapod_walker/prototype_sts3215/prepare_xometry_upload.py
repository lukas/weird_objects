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
        uno_q_tray.stl             -- qty 6  (lower stacked electronics deck)
        buck_tray.stl              -- qty 6  (upper stacked electronics deck)
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
        foot_pad.stl               -- qty 6  (TPU 95A, separate quote)
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
from typing import Callable

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

import hexapod_prototype as hp
from hexapod_prototype import (
    make_buck_tray,
    make_chassis_bottom,
    make_chassis_top,
    make_coxa_hip_bracket,
    make_coxa_yaw_hub,
    make_femur_link_part,
    make_foot_pad,
    make_servo_clamp_cap,
    make_tibia_foot_fitting,
    make_spider_carapace,
    make_tibia_knee_yoke,
    make_uno_q_tray,
    make_yaw_bearing_cap,
    make_yaw_servo_retainer,
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
    """Hex plate — already flat. Just drop to z=0.

    Jun 2026 single-part merge: ``chassis_bottom`` is the whole bottom plate
    (flat plate + bearing tower + the folded-in flat floor slab).  Its broad
    flat -6 mm underside is the bed face, so it prints face-DOWN exactly as
    built (bearing tower up) — drop to bed, no flip."""
    return _drop_to_bed(mesh)


def _reorient_deck_tray(mesh):
    """Flat deck tray (Uno Q / buck): bosses point +Z, drop to bed."""
    return _drop_to_bed(mesh)


def _reorient_coxa_hip_bracket(mesh):
    """The hip-pitch cradle hangs in -Z below the arm, with its open
    face pointing -Y. Rotate +90 deg about X to put the open face up
    (-Y -> +Z)."""
    out = _rotate(mesh, np.pi / 2, [1, 0, 0])
    return _drop_to_bed(out)


def _reorient_coxa_yaw_hub(mesh):
    """Yaw turntable hub: a flat disc.  Print platform-up so the disc-horn
    clamp counterbores + Part B join pilots open +Z; boss points down."""
    out = _rotate(mesh, np.pi, [1, 0, 0])   # flip so the boss faces -Z (down)
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

    ("uno_q_tray.stl",           make_uno_q_tray,          _reorient_deck_tray,
     6, "PLA/PETG rigid", "white", "as-printed",
     "Lower stacked electronics deck: carries the Arduino Uno Q on its "
     "non-square UNO hole pattern; bolts onto the 4 standoff columns "
     "rising above chassis_top."),

    ("buck_tray.stl",            make_buck_tray,           _reorient_deck_tray,
     6, "PLA/PETG rigid", "white", "as-printed",
     "Upper stacked electronics deck: carries the XINGYHENG 12V->5V buck "
     "converter on its 53x39 mm M3 hole pattern, with cooling vents; "
     "bolts onto the columns above the Uno Q tray."),

    # Yaw joint split into TWO printed parts (Jun 2026): the turntable HUB
    # bolts to the disc horn + rides the SPACED 6706 bearing pair; the hip
    # BRACKET bolts on top (4x M3) and carries the hip-pitch cradle.  The hub
    # is bolted to the chassis-borne bearing tower BEFORE the bracket goes on.
    ("coxa_yaw_hub.stl",         make_coxa_yaw_hub,        _reorient_coxa_yaw_hub,
     6, "MJF PA12",      "white", "as-printed",
     "Yaw turntable hub: bolts DOWN onto the 20 mm disc horn (4x M3 "
     "clamp-through, oversized/torque-only holes) and rides the inner races "
     "of the spaced 6706-2RS pair; Part B bolts onto its top platform."),

    ("coxa_hip_bracket.stl",     make_coxa_hip_bracket,    _reorient_coxa_hip_bracket,
     6, "MJF PA12",      "white", "as-printed",
     "Hip bracket: bolts onto the yaw hub top (4x M3) and carries the "
     "hip-pitch servo cradle (symmetric disc-horn sandwich -- the passive "
     "disc horn rides the servo's rear idler boss, no 688 bearing). Oriented "
     "so the cradle opens +Z."),

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
     "*** SEPARATE QUOTE *** -- needs flexible TPU for grip. "
     "If TPU isn't available, FDM PLA works but the foot will slip."),

    ("yaw_servo_retainer.stl",   make_yaw_servo_retainer,  _drop_to_bed,
     6, "MJF PA12",      "white", "as-printed",
     "Anti-rotation saddle under each yaw servo (Jun 2026 redesign): a U-channel "
     "(open on +X for the wire boot) snugly hugs the hanging STS3215 case on its "
     "+/-Y + -X faces to react reaction torque, with a flush backstop floor under "
     "the case bottom. Two THICK flange-tab bosses bolt UP to the merged "
     "chassis_bottom floor via 2 M3x6 self-tap screws whose heads RECESS into a "
     "counterbore (driven straight up from the open under-chassis cavity). Walls "
     "sized to the real 39 mm case so they seat flush to the floor. Printed floor-down."),

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

    ("spider_carapace.stl",      make_spider_carapace,     _reorient_carapace,
     1, "PLA/PETG rigid", "black", "as-printed",
     "Spider cephalothorax/prosoma dome (~141 x 124 x 34 mm). Bolts on as a "
     "3rd deck level above the buck tray on 4 M3 standoffs at DECK_COLUMN_XY; "
     "open skirt + rear window leave the electronics + wire exits vented. "
     "Front face carries the 8-eye spider arrangement as raised lenses. "
     "Print rim-down; large but single-piece on the 256/350 mm Bambu beds."),

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
