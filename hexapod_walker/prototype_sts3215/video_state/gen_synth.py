"""Generate synthetic training data for the state CNN from the MuJoCo model.

Perfectly-labeled data for free: pose the sim robot (joints + body
roll/pitch), render it offscreen, and composite the robot pixels (via the
segmentation mask) onto random patches of real bench-video background.
Domain randomization on colors (red/blue palette matching the real print),
camera pose, body yaw, blur and noise.

Poses come from two sources:
  - real trace rows harvested from EVERY rl_*.csv under hardware_traces
    (video not required) with small jitter -> the true pose manifold,
    including tips and collapses;
  - uniform samples inside the joint ranges -> diversity the traces lack
    (this is what the knees need).

Output schema matches data/state_dataset.npz (images/labels/bbox) so the
trainer can consume either.

Usage:  python gen_synth.py [--n 12000] [--out data/synth_dataset.npz]
"""

import argparse
import csv
import glob
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import mujoco  # noqa: E402
import mujoco_prototype as mp  # noqa: E402

from detect import RobotDetector  # noqa: E402
from build_dataset import CROP, ROOT, square_crop  # noqa: E402

RENDER_W, RENDER_H = 640, 360   # same 16:9 aspect as the bench camera
N_JOINTS = 18


# ---------------------------------------------------------------- poses

def harvest_trace_poses():
    rows = []
    for p in glob.glob(os.path.join(ROOT, "**", "rl_*.csv"), recursive=True):
        try:
            with open(p) as f:
                for r in csv.DictReader(f):
                    rows.append(
                        [float(r["roll_deg"]), float(r["pitch_deg"])]
                        + [float(r[f"q{i}_deg"]) for i in range(N_JOINTS)])
        except (KeyError, ValueError):
            continue
    return np.array(rows, dtype=np.float32)


def joint_ranges(model):
    lo, hi = np.zeros(N_JOINTS), np.zeros(N_JOINTS)
    for leg in range(6):
        for a, ax in enumerate(("yaw", "pitch", "knee")):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{leg}_{ax}")
            lo[3 * leg + a], hi[3 * leg + a] = model.jnt_range[j]
    return np.degrees(lo), np.degrees(hi)


def sample_pose(rng, traces, lo, hi):
    """Returns (roll_deg, pitch_deg, q_deg[18])."""
    if len(traces) and rng.random() < 0.65:
        row = traces[rng.integers(len(traces))].copy()
        row[0] += rng.normal(0, 2.0)
        row[1] += rng.normal(0, 2.0)
        row[2:] += rng.normal(0, 3.0, N_JOINTS)
        return row[0], row[1], np.clip(row[2:], lo, hi)
    q = rng.uniform(lo, hi)
    if rng.random() < 0.25:  # occasionally big body tilt (tip regime)
        roll, pitch = rng.uniform(-45, 45), rng.uniform(-30, 30)
    else:
        roll, pitch = rng.normal(0, 8), rng.normal(0, 6)
    return roll, pitch, q


# ---------------------------------------------------------------- scene

def quat_from_rpy(roll, pitch, yaw):
    """ZYX euler (matches the IMU roll=atan2(ay,az) convention)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    ])


def min_z(model, data, robot_geoms):
    """Exact-ish lowest point of the robot (mesh verts + primitive extents)."""
    z = np.inf
    for g in robot_geoms:
        pos, R = data.geom_xpos[g], data.geom_xmat[g].reshape(3, 3)
        typ, size = model.geom_type[g], model.geom_size[g]
        if typ == mujoco.mjtGeom.mjGEOM_MESH:
            mid = model.geom_dataid[g]
            v0, nv = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            verts = model.mesh_vert[v0:v0 + nv]
            z = min(z, float((verts @ R.T)[:, 2].min() + pos[2]))
        elif typ == mujoco.mjtGeom.mjGEOM_BOX:
            corners = np.array([[sx, sy, sz] for sx in (-size[0], size[0])
                                for sy in (-size[1], size[1])
                                for sz in (-size[2], size[2])])
            z = min(z, float((corners @ R.T)[:, 2].min() + pos[2]))
        elif typ in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE,
                     mujoco.mjtGeom.mjGEOM_CYLINDER):
            z = min(z, float(pos[2] - model.geom_rbound[g]))
    return z


def randomize_colors(model, robot_geoms, geom_names, rng):
    def jitter(base):
        c = np.array(base) + rng.uniform(-0.12, 0.12, 3)
        return np.clip(c, 0, 1)

    red = jitter([0.75, 0.12, 0.12])
    blue = jitter([0.15, 0.30, 0.80])
    dark = jitter([0.12, 0.12, 0.12])
    swap = rng.random() < 0.15
    if swap:
        red, blue = blue, red
    for g in robot_geoms:
        n = geom_names[g]
        if "tibia" in n:
            c = dark
        elif "servo" in n or "lipo" in n:
            c = blue
        elif "foot" in n or "horn" in n or "clamp" in n:
            c = red
        else:  # chassis, links, coxa
            c = red if rng.random() < 0.8 else blue
        model.geom_rgba[g, :3] = np.clip(c + rng.uniform(-0.05, 0.05, 3), 0, 1)
        model.geom_rgba[g, 3] = 1.0


# ---------------------------------------------------------------- backgrounds

def collect_backgrounds(n=300, rng=None):
    det = RobotDetector()
    patches = []
    videos = sorted(glob.glob(os.path.join(ROOT, "bench_blast_*", "camera.mp4")))
    for video in videos:
        cap = cv2.VideoCapture(video)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not cap.isOpened() or total <= 0:   # e.g. a recording in progress
            cap.release()
            continue
        per_video = max(3, n // max(1, len(videos)) // 3)
        for _ in range(per_video * 3):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(rng.integers(total)))
            ok, frame = cap.read()
            if not ok:
                continue
            d = det.detect(frame)
            h, w = frame.shape[:2]
            for _ in range(3):
                side = int(rng.integers(220, 560))
                x0 = int(rng.integers(0, max(1, w - side)))
                y0 = int(rng.integers(0, max(1, h - side)))
                if d.box is not None:
                    bx0, by0, bx1, by1 = d.box
                    if not (x0 + side < bx0 or x0 > bx1 or y0 + side < by0 or y0 > by1):
                        continue  # overlaps the real robot
                patches.append(cv2.resize(frame[y0:y0 + side, x0:x0 + side],
                                          (CROP, CROP)))
        cap.release()
    return patches


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=12000)
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(__file__), "data", "synth_dataset.npz"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--preview", type=int, default=0,
                    help="also write the first N composites as pngs to /tmp")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)

    model = mujoco.MjModel.from_xml_string(mp.build_xml())
    data = mujoco.MjData(model)
    geom_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
                  for g in range(model.ngeom)]
    robot_geoms = [g for g, n in enumerate(geom_names)
                   if n not in ("floor", "terrain")]
    lo, hi = joint_ranges(model)
    qadr = {}
    for leg in range(6):
        for a, ax in enumerate(("yaw", "pitch", "knee")):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{leg}_{ax}")
            qadr[3 * leg + a] = model.jnt_qposadr[j]

    traces = harvest_trace_poses()
    print(f"harvested {len(traces)} trace poses")
    backgrounds = collect_backgrounds(rng=rng)
    print(f"collected {len(backgrounds)} background patches")

    renderer = mujoco.Renderer(model, RENDER_H, RENDER_W)
    cam = mujoco.MjvCamera()

    images, labels, bboxes = [], [], []
    tries = 0
    while len(images) < args.n and tries < args.n * 4:
        tries += 1
        roll, pitch, q = sample_pose(rng, traces, lo, hi)
        for i in range(N_JOINTS):
            data.qpos[qadr[i]] = math.radians(q[i])
        yaw = rng.uniform(0, 2 * math.pi)
        data.qpos[0:3] = [0, 0, 0.3]
        data.qpos[3:7] = quat_from_rpy(math.radians(roll), math.radians(pitch), yaw)
        mujoco.mj_forward(model, data)
        z0 = min_z(model, data, robot_geoms)
        data.qpos[2] = 0.3 - z0 + rng.uniform(-0.002, 0.004)
        mujoco.mj_forward(model, data)

        randomize_colors(model, robot_geoms, geom_names, rng)

        cam.lookat[:] = [rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08),
                         data.qpos[2] * 0.6]
        cam.distance = rng.uniform(1.0, 2.6)
        cam.azimuth = rng.uniform(0, 360)
        cam.elevation = rng.uniform(-38, -8)

        renderer.update_scene(data, cam)
        rgb = renderer.render()
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, cam)
        seg = renderer.render()
        renderer.disable_segmentation_rendering()

        mask = np.isin(seg[..., 0], robot_geoms)
        ys, xs = np.nonzero(mask)
        if len(xs) < 300:
            continue
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        # robot must be inside the render and at a plausible pixel size
        if x0 <= 2 or y0 <= 2 or x1 >= RENDER_W - 3 or y1 >= RENDER_H - 3:
            continue
        if not (0.12 < (x1 - x0) / RENDER_W < 0.55):
            continue

        crop_rgb = square_crop(rgb, (x0, y0, x1 + 1, y1 + 1))
        crop_mask = square_crop(mask.astype(np.uint8) * 255,
                                (x0, y0, x1 + 1, y1 + 1))
        alpha = cv2.GaussianBlur(crop_mask, (3, 3), 0)[..., None] / 255.0
        bg = backgrounds[rng.integers(len(backgrounds))]
        bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB).astype(np.float32)
        bg *= rng.uniform(0.75, 1.2)   # backgrounds vary in exposure
        comp = crop_rgb.astype(np.float32) * alpha + bg * (1 - alpha)

        # sensor-ish degradation: blur + noise
        if rng.random() < 0.7:
            k = rng.uniform(0.4, 1.4)
            comp = cv2.GaussianBlur(comp, (5, 5), k)
        comp += rng.normal(0, rng.uniform(1, 6), comp.shape)
        comp = np.clip(comp, 0, 255).astype(np.uint8)

        images.append(comp)
        labels.append(np.array([roll, pitch] + list(q), dtype=np.float32))
        bboxes.append(np.array(
            [(x0 + x1) / 2 / RENDER_W, (y0 + y1) / 2 / RENDER_H,
             (x1 - x0) / RENDER_W, (y1 - y0) / RENDER_H], dtype=np.float32))
        if args.preview and len(images) <= args.preview:
            cv2.imwrite(f"/tmp/synth_{len(images):03d}.png",
                        cv2.cvtColor(comp, cv2.COLOR_RGB2BGR))
        if len(images) % 2000 == 0:
            print(f"{len(images)}/{args.n}")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez_compressed(
        args.out,
        images=np.stack(images), labels=np.stack(labels),
        bbox=np.stack(bboxes),
        walk_id=np.full(len(images), -1, dtype=np.int16),
        walk_names=np.array(["synth"]),
        label_names=np.array(["roll_deg", "pitch_deg"]
                             + [f"q{i}_deg" for i in range(N_JOINTS)]),
    )
    print(f"wrote {args.out}: {len(images)} frames ({tries} tries)")


if __name__ == "__main__":
    main()
