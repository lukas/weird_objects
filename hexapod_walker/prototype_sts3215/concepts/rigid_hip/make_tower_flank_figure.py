"""Before/after close-up of the rev-7 tower-flank bump shave (Aug 25).

User, Aug 25 rev 7 (viewer close-up of a yaw tower): "can you also
remove this weird bump outside the bottom chassis in the part that
hols the bearing and make it vertically smooth".  The bump is the
production +X swing relief's protect ring: it spares a cylinder
1.5 mm fatter than the Phi 44 tower (r 23.5), leaving two Phi 47
ring-sector arcs per leg on the outboard flank across the mount-plate
band (z 6.25..10.25).  Rev 7 shaves them to the corner-trim cylinder
(CHB_TRIM_R) so the tower outer profile is one vertical cylinder from
the sheet top to the rim.

This figure re-renders the leg-0 tower from roughly the user's camera
angle: BEFORE = make_chassis_bottom_rigid(bump_shave=False) with the
bump faces painted red, AFTER = the shipped chassis_bottom_rigid.stl.

Run from anywhere:  uv run python make_tower_flank_figure.py
Requires stl/chassis_bottom_rigid.stl (run make_rigid_hip_variant
first).  Output: tower_flank_smoothed.png
"""
import os
import sys

import numpy as np
import trimesh
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

HERE = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, HERE)

import make_rigid_hip_variant as mv  # noqa: E402

OUT = os.path.join(HERE, "tower_flank_smoothed.png")

print("building the pre-shave variant chassis (BEFORE panel) ...", flush=True)
before = mv.make_chassis_bottom_rigid(bump_shave=False)
after = trimesh.load(os.path.join(HERE, "stl", "chassis_bottom_rigid.stl"))

# world -> leg-0 frame (yaw axis at (100, 0); leg azimuth 30 deg)
RINV = np.linalg.inv(mv._rotz(0.5 * np.pi / 3.0))
for m in (before, after):
    m.apply_transform(RINV)

AX = mv.APOTHEM  # 100 -- yaw axis x in the leg frame


def tower_patch(mesh):
    """Faces near the leg-0 tower, plus per-face bump flag + shade."""
    tri = mesh.triangles
    cen = tri.mean(axis=1)
    r = np.hypot(cen[:, 0] - AX, cen[:, 1])
    keep = (r < 40.0) & (cen[:, 2] > -6.2)
    tri, cen, r = tri[keep], cen[keep], r[keep]
    nrm = mesh.face_normals[keep]
    bump = (r > mv.CHB_TOWER_R + 0.05) & (cen[:, 2] > mv.CHB_PLATE_TOP - 0.1) \
        & (cen[:, 2] < mv.CHB_DECK_TOP + 0.1)
    light = np.array([0.55, -0.35, 0.75])
    light /= np.linalg.norm(light)
    lam = np.clip(nrm @ light, 0.0, 1.0)
    return tri, bump, lam


fig = plt.figure(figsize=(13.6, 6.8), dpi=130)
panels = [
    (1, before, "BEFORE (rev 6) — swing-relief protect ring (r 23.5, red)\n"
                "bulges the flank at z 6.25–10.25"),
    (2, after, "AFTER (rev 7) — one vertical cylinder, sheet top → rim\n"
               "(r 22.00–22.02 at every z, all six legs)"),
]
for idx, mesh, title in panels:
    ax = fig.add_subplot(1, 2, idx, projection="3d")
    tri, bump, lam = tower_patch(mesh)
    base = np.array([0.62, 0.66, 0.74])
    cols = 0.35 * base + 0.65 * base * lam[:, None]
    red = np.array([0.80, 0.16, 0.16])
    cols[bump] = 0.45 * red + 0.55 * red * lam[bump, None]
    pc = Poly3DCollection(tri, facecolors=np.clip(cols, 0, 1),
                          edgecolors="none")
    ax.add_collection3d(pc)
    # camera roughly matching the user's viewer close-up: from outboard,
    # a touch tangential (toward the az ~17-55 lobe), slightly above
    ax.view_init(elev=14, azim=28)
    ax.set_xlim(AX - 26, AX + 26)
    ax.set_ylim(-26, 26)
    ax.set_zlim(-8, 24)
    ax.set_box_aspect((52, 52, 32))
    ax.set_axis_off()
    ax.set_title(title, fontsize=10.5, fontweight="bold")

fig.suptitle("rigid_hip rev 7: the tower-flank bump (production swing-relief "
             "protect ring, carries nothing) is shaved to the trim cylinder — "
             "leg-0 close-up, chassis_bottom_rigid",
             fontsize=11.5, fontweight="bold")
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig(OUT, bbox_inches="tight")
print("wrote", OUT)
