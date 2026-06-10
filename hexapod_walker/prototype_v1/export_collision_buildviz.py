#!/usr/bin/env python3
"""Export the assembly POSED at the workspace-sweep collision orientations
into BuildViz so the flat-bottom <-> chassis / femur interference is visible.

Writes two builds:
  * hexapod-collision-chassis  (yaw = -35 deg: well bottom sweeps over chassis_top)
  * hexapod-collision-femur    (femur = +30 deg: thigh swings into the bottom)

Each scene bakes the pose into the exported STL (identity instance transform)
and adds the actual boolean OVERLAP volume as a bright-red highlight mesh.
"""
from __future__ import annotations
import json, os, sys
from pathlib import Path
import numpy as np
import trimesh

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(THIS_DIR.parent.parent))

import hexapod_prototype as hp           # noqa: E402
import _verify_prototype as V            # noqa: E402

BUILDVIZ_ROOT = Path(os.environ.get("BUILDVIZ_ROOT", Path.home() / "buildviz")).resolve()

COLORS = {
    "coxa_link":   "#4f9dde",
    "femur_link":  "#7bc86c",
    "tibia_link":  "#c9a14a",
    "chassis_top": "#9aa0a6",
    "chassis_bottom": "#6b7075",
    "battery_holder": "#555a60",
    "electronics_tray": "#555a60",
    "overlap":     "#ff2020",
}


def _three_matrix(m):
    return [float(v) for v in np.asarray(m, float).T.reshape(-1)]


def _overlap_mesh(a, b):
    """Best-effort boolean intersection; returns (mesh_or_None, volume_mm3)."""
    for engine in ("manifold", "blender", None):
        try:
            inter = trimesh.boolean.intersection([a, b], engine=engine) if engine \
                else trimesh.boolean.intersection([a, b])
            if inter is not None and not inter.is_empty and inter.volume > 1e-6:
                return inter, float(inter.volume)
        except Exception:
            continue
    # Fallback: voxel overlap -> box cloud
    pitch = 1.0
    try:
        va = a.voxelized(pitch).fill()
        vb = b.voxelized(pitch).fill()
        pa = set(map(tuple, np.round(va.points / pitch).astype(int)))
        ptsb = np.round(vb.points / pitch).astype(int)
        keep = np.array([tuple(p) in pa for p in map(tuple, ptsb)])
        if keep.any():
            centers = vb.points[keep]
            boxes = [trimesh.creation.box(extents=(pitch,)*3,
                     transform=trimesh.transformations.translation_matrix(c))
                     for c in centers]
            return trimesh.util.concatenate(boxes), float(len(centers) * pitch**3)
    except Exception:
        pass
    return None, 0.0


def build_scene(tag, yaw, fem, knee, partner):
    out_dir = BUILDVIZ_ROOT / "public" / "builds" / f"hexapod-collision-{tag}"
    stl_dir = out_dir / "stl"
    stl_dir.mkdir(parents=True, exist_ok=True)

    leg_az = 0.0
    chassis, templates = V._ws_get_chassis_and_templates(leg_az)
    templates = dict(templates)
    templates["coxa_link"] = hp.make_coxa_link()   # current (flat-bottom) mesh
    leg = V._build_workspace_leg(yaw, fem, knee, leg_azimuth_rad=leg_az,
                                 templates=templates)

    parts = {}
    parts.update({k: leg[k] for k in ("coxa_link", "femur_link", "tibia_link")})
    for k in ("chassis_top", "chassis_bottom"):
        if k in chassis:
            parts[k] = chassis[k]

    overlap, vol = _overlap_mesh(leg["coxa_link"], parts[partner])
    print(f"[{tag}] yaw={yaw} fem={fem}: coxa_link n {partner} overlap = {vol:.1f} mm^3")

    meshes, instances = [], []
    allc = []
    for name, mesh in parts.items():
        fn = f"{name}.stl"
        mesh.export(stl_dir / fn)
        meshes.append({"id": name, "name": fn,
                       "url": f"/builds/hexapod-collision-{tag}/stl/{fn}"})
        c = mesh.bounds.mean(axis=0); allc.append(c)
        instances.append({
            "id": name, "meshId": name, "name": name, "partType": name,
            "role": "leg" if "link" in name else "chassis",
            "leg": "L0" if "link" in name else None, "joint": None,
            "color": COLORS.get(name, "#888888"),
            "transform": _three_matrix(np.eye(4)),
            "centroid": [float(v) for v in c],
            "focusGroup": "collision",
        })
    if overlap is not None:
        overlap.export(stl_dir / "overlap.stl")
        meshes.append({"id": "overlap", "name": "overlap.stl",
                       "url": f"/builds/hexapod-collision-{tag}/stl/overlap.stl"})
        c = overlap.bounds.mean(axis=0)
        instances.append({
            "id": "overlap", "meshId": "overlap",
            "name": f"INTERFERENCE {vol:.0f} mm^3", "partType": "overlap",
            "role": "collision", "leg": None, "joint": None,
            "color": COLORS["overlap"], "transform": _three_matrix(np.eye(4)),
            "centroid": [float(v) for v in c], "focusGroup": "collision",
        })

    center = np.mean(np.vstack(allc), axis=0)
    manifest = {
        "name": f"Collision @ {tag} (coxa_link flat bottom vs {partner})",
        "source": str(THIS_DIR), "units": "mm",
        "center": [float(v) for v in center],
        "meshes": meshes, "instances": instances,
    }
    (out_dir / "scene.json").write_text(json.dumps(manifest, indent=2))
    print(f"   -> {out_dir/'scene.json'}")


def main():
    build_scene("chassis", yaw=-35.0, fem=-6.7, knee=30.0, partner="chassis_top")
    build_scene("femur",   yaw=0.0,  fem=30.0, knee=30.0, partner="femur_link")


if __name__ == "__main__":
    main()
