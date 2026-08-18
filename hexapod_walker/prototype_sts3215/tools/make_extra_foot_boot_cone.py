"""Write ``extra_stl/foot_boot_cone.stl`` -- EXPERIMENTAL conical boot.

Aug 17 2026 (user: "try making another experimental version of these
boots with more conical shape").  Same bore (FOOT_BOOT_BORE_D, = the
tibia yoke socket's slip fit since the Aug 17 2026 loosen), 20 mm socket and
28 mm overall length as the production ``foot_boot`` (the tip still
lands at tibia-local x = TIBIA_LENGTH), but the outer profile is a
cone: Phi 15 mouth -> Phi 13 at the nose start -> steep nose cone to a
Phi 6 flat ground contact (vs the straight Phi 14 sleeve + Phi 10 flat
of the production boot).  See the FOOT_BOOT_CONE_* constants block in
``hexapod_prototype.py`` for the wall-thickness limits that shape it.

Print in TPU 95A, MOUTH face on the bed (wide stable base; the bore's
blind end is a 45-deg internal cone, so nothing bridges) -- NOT tip
down like the production boot.

EXPERIMENTAL: not in the production print set, the scene, or the BOM
fastener counts.  If the bench likes it, promote it into
``make_foot_boot`` proper (and give the short legs 0/4 a +4 variant).

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_extra_foot_boot_cone.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "foot_boot_cone.stl"


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    mesh = hp.make_foot_boot_cone()
    assert mesh.is_watertight, "cone boot mesh is not watertight"
    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    nom = hp.make_foot_boot()
    print(f"wrote {path}")
    print(f"  production boot extents: {nom.extents}")
    print(f"  cone boot extents:       {mesh.extents}")
    print(f"  mouth Phi {hp.FOOT_BOOT_CONE_MOUTH_OD:.1f} -> nose start "
          f"Phi {hp.FOOT_BOOT_CONE_BOT_OD:.1f} -> tip flat "
          f"Phi {hp.FOOT_BOOT_CONE_TIP_OD:.1f} "
          f"(bore Phi {hp.FOOT_BOOT_BORE_D:.1f} x "
          f"{hp.FOOT_BOOT_SOCKET_DEPTH:.0f})")
    print("Print TPU 95A, MOUTH face on the bed (not tip-down).")


if __name__ == "__main__":
    main()
