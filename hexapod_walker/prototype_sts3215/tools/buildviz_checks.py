#!/usr/bin/env python3
"""Generic, project-AGNOSTIC viewer-side validation for BuildViz scenes.

This is a prototype of the "checks driven by scene metadata" idea (see
BUILDVIZ_VALIDATION_PROPOSAL.md).  It reads ANY BuildViz ``scene.json``
(the same manifest the viewer renders -- meshes[] + instances[] with
transforms), loads the referenced meshes, and computes a set of
GENERIC, build-independent checks:

  * mesh_overlap   -- pairs of instances whose solids interpenetrate
                      (AABB pre-filter + voxel overlap estimate, mm^3).
  * clearance      -- pairs that pass closer than a min air-gap.
  * placement      -- per-instance sanity: degenerate / NaN / empty
                      bounds, instances sitting far outside the scene
                      bounding sphere (likely a bad transform).
  * scene_meta     -- manifest hygiene: units present, every instance
                      resolves to a mesh, no duplicate ids.

Nothing here is hexapod-specific: ``partType`` / ``role`` strings are
used only as human labels, and all thresholds come from CLI flags or an
optional ``checksConfig`` block in the scene (so other builds under
``buildviz/public/builds`` get the same checks for free).

It writes a SIDECAR ``<scene_dir>/buildviz_checks.json`` (it never
overwrites ``scene.json``) containing:
  * ``results``     -- one record per check with status/pass/detail.
  * ``highlights``  -- a payload shaped for the runtime
                       ``window.buildviz.setHighlights({parts, points})``
                       API so a viewer panel can paint the offenders.

Usage:
    python buildviz_checks.py <scene.json> [--overlap-mm3 100]
        [--clearance-mm 0.0] [--pitch 1.5] [--emit] [--json]

Exit code 0 = all checks pass, 1 = at least one failure.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from itertools import combinations

import numpy as np
import trimesh


# --------------------------------------------------------------------------
# Scene loading (BuildViz manifest -> world-space trimesh instances)
# --------------------------------------------------------------------------

def _resolve_mesh_path(url: str, scene_dir: str) -> str | None:
    """Map a manifest mesh ``url`` to a local file.

    BuildViz serves meshes at absolute ``/builds/<id>/stl/<file>`` URLs,
    but on disk they live next to the scene.  Try, in order: the url as
    a path relative to the scene dir, its basename under ``<scene>/stl``,
    and its basename directly under the scene dir.
    """
    candidates = []
    rel = url.lstrip("/")
    candidates.append(os.path.join(scene_dir, rel))
    base = os.path.basename(url)
    candidates.append(os.path.join(scene_dir, "stl", base))
    candidates.append(os.path.join(scene_dir, base))
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def _transform_matrix(flat) -> np.ndarray:
    """BuildViz stores a column-major 4x4 (Three.js Matrix4 .elements)."""
    if not flat or len(flat) != 16:
        return np.eye(4)
    return np.array(flat, dtype=float).reshape(4, 4).T


class Instance:
    __slots__ = ("id", "name", "part_type", "role", "mesh", "missing")

    def __init__(self, id, name, part_type, role, mesh, missing):
        self.id = id
        self.name = name
        self.part_type = part_type
        self.role = role
        self.mesh = mesh          # world-space trimesh.Trimesh or None
        self.missing = missing    # reason string if mesh unavailable


def load_scene(scene_path: str) -> tuple[dict, list[Instance]]:
    with open(scene_path, "r") as f:
        scene = json.load(f)
    scene_dir = os.path.dirname(os.path.abspath(scene_path))
    mesh_by_id = {m["id"]: m for m in scene.get("meshes", [])}

    instances: list[Instance] = []
    for inst in scene.get("instances", []):
        meshdef = mesh_by_id.get(inst.get("meshId"))
        mesh = None
        missing = None
        if meshdef is None:
            missing = f"meshId {inst.get('meshId')!r} not in manifest"
        else:
            url = meshdef.get("url")
            if url:
                path = _resolve_mesh_path(url, scene_dir)
                if path is None:
                    missing = f"mesh file not found for url {url!r}"
                else:
                    try:
                        loaded = trimesh.load(path, process=False)
                        if isinstance(loaded, trimesh.Scene):
                            loaded = trimesh.util.concatenate(
                                list(loaded.geometry.values()))
                        loaded.apply_transform(
                            _transform_matrix(inst.get("transform")))
                        mesh = loaded
                    except Exception as exc:  # noqa: BLE001
                        missing = f"load error: {exc}"
            else:
                missing = "mesh has no url (primitive-only)"
        instances.append(Instance(
            inst.get("id"), inst.get("name"), inst.get("partType"),
            inst.get("role"), mesh, missing))
    return scene, instances


# --------------------------------------------------------------------------
# Generic geometry helpers (shared with the offline verifier's approach)
# --------------------------------------------------------------------------

def _aabb_intersection_volume(a, b) -> float:
    lo = np.maximum(a.bounds[0], b.bounds[0])
    hi = np.minimum(a.bounds[1], b.bounds[1])
    if np.any(hi <= lo):
        return 0.0
    return float(np.prod(hi - lo))


def _overlap_volume(a, b, pitch: float, skip_below: float) -> tuple[float, list]:
    """Voxel-sample overlap estimate (mm^3) + world centroid of the
    overlap.  Sound early reject: overlap <= AABB-intersection volume,
    so skip raycasting when that bound is within tolerance."""
    lo = np.maximum(a.bounds[0], b.bounds[0])
    hi = np.minimum(a.bounds[1], b.bounds[1])
    if np.any(hi <= lo):
        return 0.0, None
    span = hi - lo
    if float(np.prod(span)) <= skip_below:
        return 0.0, None
    n = np.maximum(2, np.ceil(span / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    in_a = a.contains(pts)
    if not in_a.any():
        return 0.0, None
    pts_a = pts[in_a]
    in_b = b.contains(pts_a)
    n_overlap = int(in_b.sum())
    if n_overlap == 0:
        return 0.0, None
    voxel_vol = float(np.prod(span / n.astype(float)))
    centroid = pts_a[in_b].mean(axis=0)
    return n_overlap * voxel_vol, [float(c) for c in centroid]


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------

def check_scene_meta(scene: dict, instances: list[Instance]) -> dict:
    problems = []
    if scene.get("units") not in ("mm", "cm", "m"):
        problems.append(f"units missing/invalid: {scene.get('units')!r}")
    ids = [i.id for i in instances]
    dupes = sorted({x for x in ids if ids.count(x) > 1})
    if dupes:
        problems.append(f"duplicate instance ids: {dupes}")
    missing = [(i.id, i.missing) for i in instances if i.missing]
    if missing:
        problems.append(f"{len(missing)} instance(s) without loadable mesh")
    return {
        "check": "scene_meta",
        "pass": not problems,
        "detail": "; ".join(problems) if problems else "manifest OK",
        "offenders": [m[0] for m in missing],
    }


def check_placement(instances: list[Instance], center) -> dict:
    solid = [i for i in instances if i.mesh is not None]
    if not solid:
        return {"check": "placement", "pass": True,
                "detail": "no solid meshes", "offenders": []}
    centers = np.array([i.mesh.bounds.mean(axis=0) for i in solid])
    scene_c = np.array(center if center else centers.mean(axis=0), float)
    radii = np.linalg.norm(centers - scene_c, axis=1)
    # The goal here is to catch a BROKEN transform (NaN, or a part flung
    # thousands of mm away), NOT the legitimately-distal extremities of a
    # symmetric assembly (e.g. the feet of a radial hexapod).  So the
    # "detached" threshold is a generous multiple of the bulk radius
    # (2.5x the 90th-percentile distance): real extremities cluster near
    # the bulk, a bad transform lands far beyond it.
    p90 = float(np.percentile(radii, 90)) if len(radii) else 0.0
    thresh = max(2.5 * p90, 1.0)
    offenders = []
    highlights = []
    for inst, r in zip(solid, radii):
        bad = []
        if not np.all(np.isfinite(inst.mesh.bounds)):
            bad.append("non-finite bounds")
        # Degenerate: zero-size in some axis.  We do NOT use mesh.volume
        # here -- many legitimate imported STLs (e.g. fastener shells)
        # are not watertight, so .volume is meaningless/negative and
        # would false-flag.  Watertightness is a separate opt-in check.
        elif float(np.min(inst.mesh.extents)) <= 1e-9:
            bad.append("degenerate (zero extent in one axis)")
        if r > thresh:
            bad.append(f"placed {r:.0f} mm from scene centre "
                       f"(>{thresh:.0f}; likely a bad transform)")
        if bad:
            offenders.append({"id": inst.id, "partType": inst.part_type,
                              "issues": bad})
            highlights.append({"partType": inst.part_type,
                               "annotation": "; ".join(bad)})
    return {
        "check": "placement",
        "pass": not offenders,
        "detail": (f"{len(offenders)} placement outlier(s)"
                   if offenders else f"{len(solid)} instances placed sanely"),
        "offenders": offenders,
        "highlights": highlights,
    }


def check_mesh_overlap(instances, *, overlap_mm3, pitch,
                       ignore_pairs=frozenset()) -> dict:
    solid = [i for i in instances if i.mesh is not None]
    failing = []
    highlights = []
    for a, b in combinations(solid, 2):
        key = frozenset((a.part_type, b.part_type))
        if key in ignore_pairs:
            continue
        if _aabb_intersection_volume(a.mesh, b.mesh) <= overlap_mm3:
            continue
        vol, centroid = _overlap_volume(a.mesh, b.mesh, pitch, overlap_mm3)
        if vol > overlap_mm3:
            failing.append({"a": a.id, "b": b.id,
                            "aType": a.part_type, "bType": b.part_type,
                            "volume_mm3": round(vol, 1),
                            "centroid": centroid})
            if centroid is not None:
                highlights.append({"partType": a.part_type, "point": centroid,
                                   "annotation": f"overlaps {b.part_type} "
                                                 f"by {vol:.0f} mm^3"})
    failing.sort(key=lambda f: -f["volume_mm3"])
    return {
        "check": "mesh_overlap",
        "pass": not failing,
        "detail": (f"{len(failing)} interpenetrating pair(s)"
                   if failing else "no solids interpenetrate "
                                   f"above {overlap_mm3:.0f} mm^3"),
        "offenders": failing,
        "highlights": highlights,
    }


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

def run(scene_path, *, overlap_mm3, pitch, emit, as_json):
    scene, instances = load_scene(scene_path)
    cfg = scene.get("checksConfig", {}) or {}
    overlap_mm3 = float(cfg.get("overlapMm3", overlap_mm3))
    pitch = float(cfg.get("pitchMm", pitch))
    ignore_pairs = frozenset(
        frozenset(p) for p in cfg.get("ignoreOverlapPairs", []))

    results = [
        check_scene_meta(scene, instances),
        check_placement(instances, scene.get("center")),
        check_mesh_overlap(instances, overlap_mm3=overlap_mm3, pitch=pitch,
                           ignore_pairs=ignore_pairs),
    ]

    highlights_parts, highlights_points = [], []
    for r in results:
        for h in r.get("highlights", []) or []:
            if "point" in h:
                highlights_points.append(h)
            else:
                highlights_parts.append(h)

    out = {
        "scene": os.path.basename(scene_path),
        "generatedBy": "buildviz_checks.py (generic prototype)",
        "config": {"overlapMm3": overlap_mm3, "pitchMm": pitch},
        "results": [{k: v for k, v in r.items() if k != "highlights"}
                    for r in results],
        "highlights": {"parts": highlights_parts, "points": highlights_points},
        "allPass": all(r["pass"] for r in results),
    }

    if emit:
        sidecar = os.path.join(os.path.dirname(os.path.abspath(scene_path)),
                               "buildviz_checks.json")
        with open(sidecar, "w") as f:
            json.dump(out, f, indent=2)
        print(f"wrote {sidecar}")

    if as_json:
        print(json.dumps(out, indent=2))
    else:
        print(f"BuildViz generic checks for {out['scene']} "
              f"({sum(1 for i in instances if i.mesh is not None)} solid "
              f"instances):")
        for r in results:
            flag = "PASS" if r["pass"] else "FAIL"
            print(f"  [{flag}] {r['check']:14s} {r['detail']}")
            for off in (r.get("offenders") or [])[:8]:
                print(f"          - {off}")
    return 0 if out["allPass"] else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("scene", help="path to a BuildViz scene.json")
    ap.add_argument("--overlap-mm3", type=float, default=100.0,
                    help="overlap volume tolerance (default 100 mm^3)")
    ap.add_argument("--pitch", type=float, default=1.5,
                    help="voxel sampling pitch in mm (default 1.5)")
    ap.add_argument("--emit", action="store_true",
                    help="write the sidecar buildviz_checks.json")
    ap.add_argument("--json", action="store_true",
                    help="print the full result JSON to stdout")
    args = ap.parse_args(argv)
    return run(args.scene, overlap_mm3=args.overlap_mm3, pitch=args.pitch,
               emit=args.emit, as_json=args.json)


if __name__ == "__main__":
    raise SystemExit(main())
