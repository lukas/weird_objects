"""Annotated 3D before/after of the corner-pillar "gray stubs" (rev 6).

User, Aug 25: "how are there still these two gray things from the waygo
sticking up on each side of the column".  They were the inboard ends of
the two adjacent legs' yaw-servo CRADLE SHELLS (side walls + the
deck-skin roof they carried), left standing by the rev-5 corridor
flatten.  Rev 6 removes the whole shell run outside the tower keep
cylinder; this figure reproduces roughly the user's viewer angle
(inboard, looking out at the az-0 corner) so the change is unambiguous:
where the stubs stood there is now bare sheet, and the gray thing you
DO see beside each tower boss is the yaw servo itself.

BEFORE is rebuilt in-process by monkeypatching the rev-5 cut extent
(CHB_FLAT_X1 = 64.65) before calling the chassis builder; AFTER is the
shipped stl/chassis_bottom_rigid.stl.  Meshes are boolean-clipped to
the corner window and subdivided so matplotlib's painter sort holds up.

Run from anywhere:  uv run python make_pillar_stub_figure.py
Output: pillar_stubs_annotated.png
"""
import os
import sys

import numpy as np
import trimesh
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import proj3d  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

HERE = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, HERE)

import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as mv  # noqa: E402

OUT = os.path.join(HERE, "pillar_stubs_annotated.png")

# ---- geometry ---------------------------------------------------------------
after = trimesh.load(os.path.join(HERE, "stl", "chassis_bottom_rigid.stl"))

print("rebuilding the rev-5 chassis (BEFORE panel) ...", flush=True)
_x1 = mv.CHB_FLAT_X1
mv.CHB_FLAT_X1 = 64.65          # the rev-5 cut stopped at the old wall face
before = mv.make_chassis_bottom_rigid()
mv.CHB_FLAT_X1 = _x1

pillar = trimesh.load(os.path.join(HERE, "stl", "corner_pillar.stl"))
servo = trimesh.load(os.path.join(HERE, "stl", "servo_body_DO_NOT_PRINT.stl"))

servos = []
for i in (0, 5):                # the two legs adjacent to the az-0 corner
    T = mv.leg_transforms(i)
    s = servo.copy()
    s.apply_transform(T["coxa"] @ mv._trans(
        [-hp.SERVO_OUTPUT_X, 0.0, -(hp.HORN_STACK_H + hp.WELL_RIM_Z)]))
    servos.append(s)

LO = np.array([48.0, -42.0, -5.0])
HI = np.array([120.0, 42.0, 42.0])
CLIP_BOX = trimesh.creation.box(HI - LO, transform=mv._trans((LO + HI) / 2.0))


def window_tris(mesh, max_edge=6.0):
    """Boolean-clip a mesh to the corner window (fallback: face filter),
    then subdivide so no triangle is large enough to break depth sort."""
    try:
        m = mesh.intersection(CLIP_BOX)
        v, f = m.vertices, m.faces
    except Exception:
        v, f = mesh.vertices, mesh.faces
        c = v[f].mean(axis=1)
        f = f[np.all((c > LO) & (c < HI), axis=1)]
    v, f = trimesh.remesh.subdivide_to_size(v, f, max_edge=max_edge)
    return v[f]


def shaded(tris, base_rgb):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([-0.5, 0.15, 0.85])
    lam = np.clip(n @ light, 0.0, 1.0) * 0.5 + 0.5
    return np.clip(np.outer(lam, np.asarray(base_rgb)), 0, 1)


def draw(ax, chassis, title):
    parts = [(chassis, (0.70, 0.73, 0.79)),
             (pillar, (0.35, 0.47, 0.72)),
             (servos[0], (0.30, 0.30, 0.32)),
             (servos[1], (0.30, 0.30, 0.32))]
    for mesh, rgb in parts:
        tris = window_tris(mesh)
        ax.add_collection3d(Poly3DCollection(
            tris, facecolors=shaded(tris, rgb), edgecolor="none"))
    ax.set_xlim(52, 112)
    ax.set_ylim(-36, 36)
    ax.set_zlim(-4, 42)
    ax.set_box_aspect((60, 72, 46))
    ax.view_init(elev=17, azim=183)
    ax.set_proj_type("persp", focal_length=0.30)
    ax.set_axis_off()
    ax.set_title(title, fontsize=11, fontweight="bold", pad=0)


fig = plt.figure(figsize=(14.5, 7.6), dpi=130)
axB = fig.add_subplot(1, 2, 1, projection="3d")
axA = fig.add_subplot(1, 2, 2, projection="3d")
draw(axB, before, "BEFORE (rev 5) — what the user saw")
draw(axA, after, "AFTER (rev 6) — shipped chassis_bottom_rigid")


def note(ax, xyz, text, fx, fy, fc="#fff8d8", ec="#a08c30",
         ac="#c03030", fs=8.2):
    x2, y2, _ = proj3d.proj_transform(*xyz, ax.get_proj())
    ax.annotate(text, xy=(x2, y2), xycoords="data",
                xytext=(fx, fy), textcoords="axes fraction",
                fontsize=fs, ha="center", zorder=20,
                bbox=dict(boxstyle="round,pad=0.3", fc=fc, ec=ec, lw=0.8),
                arrowprops=dict(arrowstyle="->", color=ac, lw=1.4))


# BEFORE: the two stubs (corner frame: wall ends at ~(64, +/-17.5), z 2..10.25)
note(axB, (64.0, 18.5, 8.5),
     "gray stub = inboard end of leg-0's\ncradle shell (side wall + roof)",
     0.20, 0.88)
note(axB, (64.0, -18.5, 8.5),
     "gray stub = same shell of leg 5;\nthey flank the pillar at ~10 mm",
     0.20, 0.10)
note(axB, (81.6, 0.0, 32.0), "corner pillar", 0.82, 0.88,
     fc="#e8edf8", ec="#4d6fa8", ac="#4d6fa8")

# AFTER: what is there now, and why the keeps are keeps
note(axA, (68.0, 32.0, 4.0),
     "the gray block beside the boss IS the\nYAW SERVO (case 4.25 mm proud of "
     "the sheet;\nregistered by the sheet well + belly retainer)",
     0.20, 0.90)
note(axA, (64.0, -17.5, 2.2),
     "bare sheet — the stubs stood here\n(their roof touched nothing)",
     0.20, 0.08)
note(axA, (90.0, -38.0, 14.0),
     "\u03a644 tower boss — KEPT: 6805 bearing\nseat + servo well mouth "
     "(the shell inside\nit carries the seat's inboard arc)",
     0.84, 0.10, fc="#e2f0e2", ec="#2c8a2c", ac="#2c8a2c")

fig.suptitle('rigid_hip rev 6 — the "two gray things from the waygo" beside '
             "each corner pillar: identified and flattened "
             "(az-0 corner, viewed from inboard like the user's screenshot)",
             fontsize=12, fontweight="bold")
fig.tight_layout(rect=(0, 0, 1, 0.965))
fig.savefig(OUT, bbox_inches="tight")
print("wrote", OUT)
