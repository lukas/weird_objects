"""Regenerate joint_column_annotated.png for the rigid-hip variant.

Radial section through the leg-0 yaw axis, zoomed on the joint column,
with every z plane annotated: full-wrap tower stack (Aug 24 rev 2) plus
the SHORTENED coxa hub column (Aug 24 rev 3 skirt/platform delete +
rev 4 M3x30 -> M3x25 horn screws; the cradle slab drops COL_DROP = 9 mm
total, so the hip servo sits just above the bearing).

Run from anywhere:  uv run python make_joint_column_figure.py
Requires the freshly generated stl/ meshes (run make_rigid_hip_variant
first).
"""
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PROTO)
sys.path.insert(0, HERE)

import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from trimesh.transformations import rotation_matrix  # noqa: E402

STL = os.path.join(HERE, "stl")


def load(key):
    return trimesh.load(os.path.join(STL, rv.MESH_FILES[key][1]))


T = rv.leg_transforms(0)
a = 0.5 * np.pi / 3.0
Rz = rv._rotz(-a)  # leg-0 radial -> +x

yaw_servo_M = T["coxa"] @ rv._trans(
    [-hp.SERVO_OUTPUT_X, 0.0, -(hp.HORN_STACK_H + hp.WELL_RIM_Z)])

sections = [
    ("chassis_bottom", np.eye(4), "#8b93a6", 0.75,
     "chassis_bottom_rigid (static; full-wrap tower)"),
    ("yaw_bearing_upper", T["coxa"] @ rv._trans([0.0, 0.0, rv.YAWBR_DROP]),
     "#303030", 0.9, "6805-2RS -- fully housed, top flush with the rim"),
    ("coxa_link", T["coxa"], "#7ba1d1", 0.75,
     "coxa_link_rigid (rotates; SHORT column, Aug 24 rev 3)"),
    ("servo_body", T["hip_cap"], "#c9a227", 0.55,
     "hip servo (in the lowered cradle)"),
    ("servo_body", yaw_servo_M, "#c9a227", 0.55, "yaw servo (below deck)"),
    ("yaw_servo_retainer", T["cradle"], "#9aa0a6", 0.55, None),
]

fig, ax = plt.subplots(figsize=(11.5, 8.4), dpi=140)
for key, M, color, alpha, label in sections:
    m = load(key)
    m.apply_transform(Rz @ M)
    sec = m.section(plane_origin=[rv.APOTHEM, 0, 0], plane_normal=[0, 1, 0])
    if sec is None:
        continue
    planar, _ = sec.to_2D(to_2D=rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    first = True
    for poly in planar.polygons_full:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color=color, alpha=alpha,
                label=label if (first and label) else None)
        first = False
        for ring in poly.interiors:
            ax.fill(*ring.xy, color="white")

A = rv.APOTHEM
DECK = 10.25
HORN = hp.CHASSIS_YAW_OUTPUT_Z                        # 15.25
SEAT = HORN + hp.YAW_BEARING_LOWER_BOT_Z              # 15.75
RIM = rv.CHB_RIM_W                                    # 22.75
BRIM0 = HORN + rv.BRIM_BOT_Z                          # 23.25
BRIM1 = HORN + rv.BRIM_TOP_Z                          # 25.25
SLAB = HORN + rv.SLAB_BOT_Z                           # 24.25
RING1 = HORN + rv.HUB_RING_Z1                         # 25.25 (= brim top)
HEADS = HORN + rv.HORN_HEAD_SEAT_Z + hp.INSERT_M3_BOLT_HEAD_H  # 31.0
FLOOR = HORN + rv.COXA_FLOOR_Z                        # 32.25
HIP = HORN + rv.COXA_HIP_DROP_V                       # 44.65
OLD_SLAB = SLAB + rv.HORN_SEAT_DROP                   # 29.25 (M3x30 build)
OLD_FLOOR = FLOOR + rv.HORN_SEAT_DROP                 # 37.25 (M3x30 build)

# z-plane lines with staggered, leader-lined labels (crowded stack)
planes = [
    (DECK, 5.4, "z 10.25   servo-mount deck top"),
    (HORN, 9.2, "z 15.25   disc-horn top = coxa mount plane"),
    (SEAT, 13.0, "z 15.75   race seat (0.5 over the horn + screws)"),
    (RIM, 16.8, "z 22.75   tower rim = race top -- \u03a644 column ENDS here"),
    (BRIM0, 20.6, "z 23.25   \u03a638 dust brim underside (0.5 running gap)"),
    (SLAB, 24.4, "z 24.25   cradle slab underside -- DROPPED 9 mm total "
                 "(1.5 over the rim)"),
    (BRIM1, 28.2, "z 25.25   brim top = \u03a629 seat ring top "
                  "(1 mm inside the slab)"),
    (HEADS, 32.0, "z 31.00   M3x25 horn screw head tops "
                  "(seats 5 mm deeper, tips/engagement unchanged)"),
    (FLOOR, 35.8, "z 32.25   servo well floor -- 1.25 over the heads"),
    (HIP, 44.65, "z 44.65   hip axis (was 49.65 with M3x30s)"),
]
for z, ytxt, lab in planes:
    ax.axhline(z, color="k", lw=0.5, ls=":")
    ax.annotate(lab, xy=(A + 23.5, z), xytext=(A + 29.5, ytxt),
                fontsize=7.5, va="center",
                arrowprops=dict(arrowstyle="-", lw=0.55, color="#555555",
                                shrinkA=0.0, shrinkB=0.0))


# vertical band braces in the empty pocket left of the tower
def brace(x, z0, z1, text, color):
    ax.annotate("", xy=(x, z0), xytext=(x, z1),
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.2))
    ax.annotate(text, (x - 0.9, (z0 + z1) / 2.0), fontsize=7.5, color=color,
                va="center", ha="center", rotation=90)


brace(A - 29.5, DECK, RIM, "\u03a644 tower: 12.5 mm", "#4a5568")
brace(A - 25.5, SEAT, RIM, "6805 fully housed", "#1a1a1a")
brace(A - 33.5, RIM, FLOOR, "bearing \u2192 servo: 9.5 mm (was 14.5)",
      "#166534")
for z, lab in ((OLD_SLAB, "M3x30-era slab bottom 29.25"),
               (OLD_FLOOR, "M3x30-era well floor 37.25")):
    ax.axhline(z, color="#b91c1c", lw=0.6, ls="--", alpha=0.6)
    ax.annotate(lab + " -- dropped a further 5 mm (M3x25 screws)",
                xy=(A - 20.0, z),
                fontsize=7.0, color="#b91c1c", style="italic",
                va="bottom", ha="left")
ax.annotate("hub band over the heads existed only to house M3x30 length"
            " -- screws shortened, seats follow, tips unchanged",
            xy=(A + 10.0, OLD_SLAB + 2.0),
            xytext=(A + 26, 3.2), fontsize=7.5,
            color="#b91c1c", style="italic", va="center",
            arrowprops=dict(arrowstyle="-", lw=0.7, color="#b91c1c",
                            ls="--", shrinkA=0.0, shrinkB=0.0))

ax.set_xlim(A - 46, A + 70)
ax.set_ylim(-8, 53)
ax.set_aspect("equal")
ax.set_xlabel("radial position from body centre [mm]  (leg-0 yaw axis at 100)")
ax.set_ylabel("world Z [mm]")
ax.legend(loc="upper left", fontsize=8)
ax.set_title("rigid-hip joint column, shortened Aug 24 (M3x25 horn screws)"
             " -- horn \u2192 bearing \u2192 coxa \u2192 hip servo "
             "(section through the leg-0 yaw axis)")
fig.tight_layout()
out = os.path.join(HERE, "joint_column_annotated.png")
fig.savefig(out)
print("wrote", out)
