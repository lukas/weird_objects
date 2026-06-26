#!/usr/bin/env python3
"""Offscreen render of the assembled robot cat for visual verification.

Loads every STL referenced by ``cat_viz/scene.json`` (colored by instance)
and saves matplotlib 3D views from a few angles to ``robot_cat/renders/``.
Matplotlib is used because it has no GPU/display dependency.

Run:  ./run.sh robot_cat/render_cat.py
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

HERE = Path(__file__).resolve().parent
BUILD = HERE / "cat_viz"
OUT = HERE / "renders"


def load_instances():
    scene = json.loads((BUILD / "scene.json").read_text())
    mesh_by_id = {m["id"]: m for m in scene["meshes"]}
    out = []
    for inst in scene["instances"]:
        mj = mesh_by_id[inst["meshId"]]
        m = trimesh.load(BUILD / mj["url"], process=False)
        out.append((m, inst["color"], inst["partType"], inst.get("role", "part")))
    return out


# Parts to GHOST (draw translucent) in the electronics-reveal view so the
# controller + battery packaged inside the body are visible.
GHOST_TYPES = {"shell_left", "shell_right", "belly_hatch",
               "torso", "neck", "head", "muzzle", "cheek", "nose", "ear",
               "ear_inner", "eye_white", "iris", "pupil", "eye_gleam",
               "tail_segment", "collar", "bell", "knee"}

# All hardware (servos, horns, electronics, mounts, bosses, screws) lives fully
# ENCLOSED inside the body shell or the FUR knee bulges.  matplotlib has no depth
# buffer across collections, so it paints them THROUGH their covers (a render
# artifact -- in the GPU BuildViz viewer they're hidden).  Skip these roles in
# the beauty views; the reveal + exploded views show them honestly.
INTERNAL_ROLES = {"motor", "horn", "controller", "battery", "bracket",
                  "boss", "fastener"}

# Articulated (MOVING) part types -- highlighted in the kinematics view.
MOVING_TYPES = {"head", "cheek", "ear", "ear_inner", "muzzle", "nose",
                "eye_white", "iris", "pupil", "eye_gleam", "head_mount",
                "upper_arm", "forearm", "front_paw", "knee",
                "femur", "shank", "hind_paw", "servo_horn"}


def explode_offset(ptype, centroid):
    """Per-part translation (mm) for the exploded assembly diagram: shell halves
    out sideways, belly hatch + electronics down, head up/forward, tail back,
    legs splayed out + down by segment."""
    sy = 1.0 if centroid[1] > 4.0 else (-1.0 if centroid[1] < -4.0 else 0.0)
    table = {
        "shell_left": (0, 120, 0), "shell_right": (0, -120, 0),
        "belly_hatch": (0, 0, -95), "hatch_frame": (0, 0, -55),
        "seam_bosses": (0, 22, 0), "hatch_screws": (0, 0, -120),
        "shell_screws": (0, 60, 0),
        "head": (55, 0, 120), "cheek": (55, 0, 120), "muzzle": (55, 0, 120),
        "nose": (55, 0, 120), "ear": (55, 0, 120), "ear_inner": (55, 0, 120),
        "eye_white": (55, 0, 120), "iris": (55, 0, 120), "pupil": (55, 0, 120),
        "eye_gleam": (55, 0, 120), "head_mount": (45, 0, 95),
        "collar": (85, 0, 35), "bell": (100, 0, 30),
        "tail_segment": (-95, 0, 60),
        "controller": (0, 0, -160), "controller_tray": (0, 0, -160),
        "board_standoffs": (0, 0, -160),
        "battery": (0, 0, -210), "battery_cradle": (0, 0, -210),
        "battery_strap": (0, 0, -235),
    }
    if ptype in table:
        return np.array(table[ptype], float)
    leg = {"upper_arm": (0, 150, 25), "femur": (0, 150, 25),
           "forearm": (0, 180, -25), "shank": (0, 180, -25),
           "front_paw": (0, 205, -75), "hind_paw": (0, 205, -75),
           "knee": (0, 190, -45), "servo_mount": (0, 140, 10)}
    if ptype in leg:
        v = np.array(leg[ptype], float); v[1] *= (sy if sy else 1.0); return v
    if ptype in ("micro_servo", "servo_horn"):
        if abs(centroid[1]) > 6.0:
            return np.array([0.0, sy * 165.0, 5.0])
        return np.array([35.0, 0.0, 80.0])   # neck servo
    return np.zeros(3)

# Directional light for a simple Lambert shade so the matplotlib preview shows
# 3D FORM (the GPU BuildViz viewer shades properly; matplotlib does not by
# default, which made smooth meshes look flat/faceted in earlier previews).
_LIGHT = np.array([0.35, 0.55, 0.75])
_LIGHT = _LIGHT / np.linalg.norm(_LIGHT)


def _hex_rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)])


def _shade(mesh, color):
    """Per-face Lambert-shaded RGB array for a mesh."""
    base = _hex_rgb(color)
    n = mesh.face_normals
    lam = np.clip(n @ _LIGHT, 0.0, 1.0)
    inten = 0.35 + 0.65 * lam            # ambient + diffuse
    return np.clip(base[None, :] * inten[:, None], 0.0, 1.0)


def render(view, elev, azim, fname, ghost=None, reveal=False, explode=False,
           kinematics=False):
    ghost = ghost or set()
    insts = load_instances()
    if kinematics:
        # show the structural skeleton + moving chains; hide deep internals
        insts = [ic for ic in insts
                 if ic[3] not in {"motor", "horn", "controller", "battery",
                                  "boss", "fastener"}]
    elif not (reveal or explode):
        insts = [ic for ic in insts if ic[3] not in INTERNAL_ROLES]
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    allpts = []
    # matplotlib has no real depth buffer ACROSS collections (later-drawn parts
    # paint over earlier ones regardless of depth), which made covered hardware
    # show through.  Draw ghosts first, then solids back->front by camera depth
    # so enclosing shells (e.g. the knee pods over the servos) occlude correctly.
    er, ar = np.radians(elev), np.radians(azim)
    cam = np.array([np.cos(er) * np.cos(ar), np.cos(er) * np.sin(ar), np.sin(er)])
    prepared = []
    for m, color, pt, role in insts:
        mm = m
        if explode:
            mm = m.copy()
            mm.apply_translation(explode_offset(pt, m.centroid))
        if kinematics:
            moving = pt in MOVING_TYPES
            color = "#ff8a3d" if moving else "#c7ccd6"   # moving=orange, static=grey
        prepared.append((mm, color, pt, role))
    order = sorted(prepared, key=lambda ic: (0 if ic[2] in ghost else 1,
                                             float(np.dot(ic[0].centroid, cam))))
    for m, color, _pt, _role in order:
        tris = m.vertices[m.faces]
        allpts.append(m.vertices)
        is_ghost = _pt in ghost
        if is_ghost:
            pc = Poly3DCollection(tris, alpha=0.08)
            pc.set_facecolor(color)
            pc.set_edgecolor((0, 0, 0, 0.0))
        else:
            pc = Poly3DCollection(tris, alpha=1.0)
            pc.set_facecolor(_shade(m, color))
            pc.set_edgecolor("none")
        ax.add_collection3d(pc)
    pts = np.vstack(allpts)
    mn, mx = pts.min(0), pts.max(0)
    ctr = (mn + mx) / 2.0
    r = (mx - mn).max() / 2.0 * 1.05
    ax.set_xlim(ctr[0] - r, ctr[0] + r)
    ax.set_ylim(ctr[1] - r, ctr[1] + r)
    ax.set_zlim(ctr[2] - r, ctr[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X fwd")
    ax.set_ylabel("Y left")
    ax.set_zlabel("Z up")
    ax.set_title(f"Robot Cat — {view}")
    # Light background, hide busy panes.
    ax.set_facecolor("white")
    fig.patch.set_facecolor("white")
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / fname, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT / fname}")


def main():
    # Side view (the money shot: arched spine + leg zig-zag + tail).
    render("left side", elev=8, azim=-90, fname="cat_side.png")
    # Three-quarter iso.
    render("three-quarter", elev=18, azim=-60, fname="cat_iso.png")
    # Front (face on: big eyes, cheeks, ears, nose -- where the cute lives).
    render("front face", elev=8, azim=8, fname="cat_front.png")
    # Top view (spine arch + leg splay + tail curl).
    render("top", elev=80, azim=-90, fname="cat_top.png")
    # Electronics reveal: ghost the body so the board + battery + mounts show.
    render("electronics (ghosted body)", elev=14, azim=-72,
           fname="cat_electronics.png", ghost=GHOST_TYPES, reveal=True)
    # Exploded assembly: every separable printed part pulled apart.
    render("exploded assembly", elev=16, azim=-66, fname="cat_exploded.png",
           explode=True)
    # Kinematics: MOVING (orange) vs STATIC structure (grey).
    render("kinematics (orange = moving)", elev=16, azim=-62,
           fname="cat_kinematics.png", kinematics=True)


if __name__ == "__main__":
    main()
