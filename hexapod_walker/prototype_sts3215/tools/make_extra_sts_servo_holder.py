"""Write ``extra_stl/sts_servo_holder.stl`` -- standalone STS3215 holder.

Aug 19 2026 (user: "just make me a sts servo holder like the top part of
the coxa link but with screw holes on the bottom with the centers 23mm
apart in a square. The bottom should be 8mm thick").

The holder is the CURRENT production hip cradle -- the exact "top part"
of the merged ``coxa_link`` (``_sandwich_fixed_side(end_face_bolts=False,
wire_exit=False, rear_tab=True)``: 4-wall clamshell, output-face lip with
the 4 front-case self-tap counterbores, clamp-cap pilots for a
``servo_clamp_cap.stl``, and the rear retention tab) -- in the same
orientation it has on the coxa (solid long wall DOWN, clamp mouth UP,
output axis horizontal), SUNK into a base plate spanning the cradle
footprint + 1 mm.

SEAT_H = 8 mm is the height the SERVO SITS above the base's bottom face
(Aug 19 2026 rev 2, user: "the 8mm should be the height that the servo
sits above the base - you have an additional raised thing here you need
to remove that"): the cradle is sunk so its cavity floor IS the base
top -- the cradle's own 3.8 mm bottom wall is swallowed by the slab
instead of stacking a raised step on top of it.  One flat 8 mm floor,
servo belly at exactly +8.

Base mounting pattern: 4x M3 clearance bores (Phi 3.4) on a 23 x 23 mm
square, CENTRED on the servo body centre (well x = 0, well z =
SERVO_BODY_H/2), running vertically through the floor.  Each bore gets
a Phi 6.0 head pocket sunk from the floor top so an M3 SHCS head sits
FLUSH under the seated servo: drop the 4 screws in and torque them
BEFORE the servo goes in (same head-access idea as the merged coxa's
shafts).  Plastic under the head is ~4.6 mm, so an M3 x 8 leaves
~3.4 mm of thread proud of the base bottom (M3 x 10 -> ~5.4 mm).

Assembly: bolt the base down, drop the 4 base screws first, seat the
servo, close with ``servo_clamp_cap.stl`` (2x M3), then the 4x M2.5x6
front-case self-tappers and 2x M2.5x6 into the rear tab.

EXTRA part: not in the production print set, the scene, or the BOM.

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_extra_sts_servo_holder.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp  # noqa: E402
from trimesh.transformations import rotation_matrix  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "sts_servo_holder.stl"

SEAT_H = 8.0            # mm -- servo seat height above the base bottom (user spec)
MOUNT_SQUARE = 23.0     # mm -- centre-to-centre of the 4 base screws (user spec)
MOUNT_BORE_OD = 3.4     # mm -- M3 clearance
MOUNT_HEAD_OD = 6.0     # mm -- M3 SHCS head (5.5) + 0.5 pocket clearance
MOUNT_HEAD_H = 3.4      # mm -- pocket depth (head 3.0 + 0.4 flush margin)


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)

    # The coxa link's cradle, exactly as production builds it.
    cradle = hp._sandwich_fixed_side(end_face_bolts=False, wire_exit=False,
                                     rear_tab=True)
    # Holder frame: rotate +90 deg about X so the well's solid -Y wall
    # faces DOWN and the +Y clamp mouth faces UP (well +Z output ->
    # horizontal -Y), matching the cradle's pose on the coxa.
    R = rotation_matrix(np.pi / 2.0, [1, 0, 0])
    cradle.apply_transform(R)
    # SINK the cradle so its cavity floor (the servo's resting plane,
    # well y = -(SERVO_BODY_D/2 + CL) before rotation) lands exactly at
    # z = SEAT_H.  The cradle's 3.8 mm bottom wall ends up INSIDE the
    # base slab (no raised step -- rev 2).
    wall_inner_y = -(hp.SERVO_BODY_D / 2.0 + hp.WELL_BODY_CL)
    dz = SEAT_H - wall_inner_y          # rotation maps well y -> holder z
    cradle.apply_translation([0.0, 0.0, dz])
    b = cradle.bounds

    # Base slab: full cradle footprint + 1 mm, from the bottom face (z=0)
    # up to the cavity floor plane -- one flush floor under the servo.
    base = hp._box((float(b[1][0] - b[0][0]) + 2.0,
                    float(b[1][1] - b[0][1]) + 2.0, SEAT_H),
                   center=(0.5 * float(b[0][0] + b[1][0]),
                           0.5 * float(b[0][1] + b[1][1]), SEAT_H / 2.0))
    body = hp._union(base, cradle)

    # Frame bookkeeping for the screw pattern.  In holder coords the well
    # maps (x, y, z)_well -> (x, -z, y)_well + dz', so:
    #   pattern x = well x = +/-11.5 about the body centre (x = 0),
    #   pattern y = -(well z) = -(SERVO_BODY_H/2 +/- 11.5).
    cav_floor_z = SEAT_H
    half = MOUNT_SQUARE / 2.0
    centres = [(sx * half, -(hp.SERVO_BODY_H / 2.0 + sy * half))
               for sx in (-1, 1) for sy in (-1, 1)]
    cuts = []
    for (cx, cy) in centres:
        bore = hp._cyl(MOUNT_BORE_OD / 2.0, cav_floor_z + 4.0)
        bore.apply_translation([cx, cy, (cav_floor_z + 4.0) / 2.0 - 1.0])
        cuts.append(bore)
        pocket = hp._cyl(MOUNT_HEAD_OD / 2.0, MOUNT_HEAD_H + 1.0)
        pocket.apply_translation(
            [cx, cy, cav_floor_z - MOUNT_HEAD_H + (MOUNT_HEAD_H + 1.0) / 2.0])
        cuts.append(pocket)
    mesh = hp._diff(body, *cuts)
    assert mesh.is_watertight, "servo holder mesh is not watertight"

    print("feature checks:")
    def probe(pts, want_solid, label):
        inside = mesh.contains(np.asarray(pts, dtype=float))
        ok = inside.all() if want_solid else (~inside).all()
        assert ok, f"{label}: contains={inside.tolist()}"
        print(f"  OK  {label}")

    probe([(cx, cy, 1.0) for cx, cy in centres], False,
          "4 base bores on the 23 mm square (void through the base)")
    probe([(cx, cy, cav_floor_z - 1.0) for cx, cy in centres], False,
          "4 flush head pockets at the cavity floor (void)")
    probe([(0.0, cy, 1.0) for cy in sorted({c[1] for c in centres})]
          + [(0.0, -hp.SERVO_BODY_H / 2.0, 1.0)], True,
          "base material between the bores (solid)")
    # The cradle's own motor-screw features survive: front-case bores.
    fc = [(fx, -(hp.WELL_H - hp.FRONT_CASE_CBORE_DEPTH / 2.0), fy + dz)
          for fx, fy in hp.servo_front_case_hole_centres()]
    probe(fc, False, "front-case screw counterbores (void)")

    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    print(f"wrote {path}")
    print(f"  extents: {np.round(mesh.extents, 1)}  "
          f"volume: {mesh.volume / 1000.0:.1f} cm3")
    print(f"  servo seat at z = {cav_floor_z:.0f} (one flush floor); "
          f"4x Phi {MOUNT_BORE_OD} bores on a "
          f"{MOUNT_SQUARE:.0f} x {MOUNT_SQUARE:.0f} square, heads pocketed "
          f"flush at the floor")
    print("Print base-down.  Pairs with servo_clamp_cap.stl + 2x M3, "
          "4x M2.5x6 front-case self-tappers, 2x M2.5x6 rear-tab screws, "
          "4x M3x8 base screws (dropped in before the servo).")


if __name__ == "__main__":
    main()
