#!/usr/bin/env python3
"""Pairwise self-intersection / overlap sanity check for the robot-cat build.

Loads every instance mesh from ``cat_viz/scene.json`` and measures the solid
intersection VOLUME between structural parts that should NOT interpenetrate
(body shells, bones, head, tail, paws).  Drive-system hardware (motors,
brackets, axles/fasteners) is excluded because that geometry is *meant* to
bridge/overlap the segments it joins.

A small residual where a limb root meets the trunk, or where two body shells
butt at a spine-flex joint, is expected and reported as such.

Reports the worst offenders (sorted by intersection volume) so the geometry can
be tuned until adjacent parts meet cleanly instead of clipping.

Run:  ./run.sh robot_cat/check_overlaps.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
BUILD = HERE / "cat_viz"

# Hardware / printed-mount roles that are SUPPOSED to interface with the parts
# they connect (servos, horns, brackets, screw bosses, fasteners).
EXCLUDE_ROLES = {"motor", "servo", "fastener", "bracket", "axle", "horn", "boss"}

# Trunk = the split body shell halves + belly hatch + neck.  Rev 9 splits the
# body sagittally; the halves + hatch + neck MATE along seams (kept ~0.2 mm
# apart) and the limb roots / electronics tuck inside the chubby silhouette, so
# trunk<->trunk (mating) and trunk<->root / trunk<->electronics are all expected.
# A FUR knee bulge wraps the elbow/knee servo and overlaps its own leg bones and
# the paw below it (short stubby leg) -- intended (a printed cover, not a clip).
TRUNK_TYPES = {"torso", "shell_left", "shell_right", "belly_hatch", "neck"}
ROOT_TYPES = {"upper_arm", "femur", "knee"}
LEG_TYPES = {"upper_arm", "forearm", "femur", "shank", "knee",
             "front_paw", "hind_paw"}

# Electronics live INSIDE the (really hollow) body cavity; the trunk is modelled
# as a solid silhouette, so electronics<->trunk "overlap" is expected, not a bug.
ELECTRONICS_TYPES = {"battery", "controller", "controller_tray",
                     "battery_cradle"}

# The head prints as ONE piece -> every face feature abuts/overlaps its
# neighbours cosmetically; any pair among these is expected.
HEAD_FEATURES = {"head", "cheek", "muzzle", "nose", "ear", "ear_inner",
                 "eye_white", "iris", "pupil", "eye_gleam"}

# The collar + bell are a cosmetic accessory hugging the neck/chest.
COSMETIC_TYPES = {"collar", "bell"}
COSMETIC_HOSTS = {"neck", "torso", "head", "collar", "bell",
                  "shell_left", "shell_right", "belly_hatch"}

# Adjacent body parts that are MEANT to touch at a connection (foot bonded to the
# lower leg).  Head<->trunk and tail<->trunk are handled by the trunk rules below.
EXPECTED_TYPE_PAIRS = {
    frozenset({"neck", "head"}),
    # the foot is bonded to the bottom of the lower-leg segment (intended joint)
    frozenset({"forearm", "front_paw"}),
    frozenset({"shank", "hind_paw"}),
}


def aabb_overlap_volume(a: np.ndarray, b: np.ndarray) -> float:
    lo = np.maximum(a[0], b[0])
    hi = np.minimum(a[1], b[1])
    d = hi - lo
    if np.any(d <= 0):
        return 0.0
    return float(d[0] * d[1] * d[2])


def solid_intersection_volume(mi: trimesh.Trimesh, mj: trimesh.Trimesh) -> float:
    """Best-effort solid intersection volume (mm^3).  Falls back to the AABB
    overlap box volume if the boolean engine is unavailable/fails."""
    try:
        inter = trimesh.boolean.intersection([mi, mj])
        if inter is None or inter.is_empty:
            return 0.0
        v = float(inter.volume)
        return v if np.isfinite(v) and v > 0 else 0.0
    except Exception:
        return aabb_overlap_volume(mi.bounds, mj.bounds)


def load_structural():
    scene = json.loads((BUILD / "scene.json").read_text())
    mesh_by_id = {m["id"]: m for m in scene["meshes"]}
    parts = []
    for inst in scene["instances"]:
        if inst.get("role") in EXCLUDE_ROLES:
            continue
        mj = mesh_by_id[inst["meshId"]]
        m = trimesh.load(BUILD / mj["url"], process=False)
        parts.append((inst["name"], inst["partType"], m))
    return parts


def main():
    parts = load_structural()
    n = len(parts)
    print(f"Checking {n} structural parts ({n * (n - 1) // 2} pairs) for overlap...")

    offenders = []   # (volume, name_i, name_j, expected)
    total_bad = 0.0
    for i in range(n):
        ni, ti, mi = parts[i]
        for j in range(i + 1, n):
            nj, tj, mj = parts[j]
            if aabb_overlap_volume(mi.bounds, mj.bounds) <= 1.0:
                continue
            vol = solid_intersection_volume(mi, mj)
            if vol <= 5.0:   # ignore sub-5 mm^3 numerical touches
                continue
            pair = frozenset({ti, tj})
            expected = (# split shell halves + hatch + neck MATE along seams
                        (ti in TRUNK_TYPES and tj in TRUNK_TYPES)
                        # limb roots / electronics tuck inside the body shell
                        or (ti in TRUNK_TYPES and tj in ROOT_TYPES)
                        or (tj in TRUNK_TYPES and ti in ROOT_TYPES)
                        or (ti in TRUNK_TYPES and tj in ELECTRONICS_TYPES)
                        or (tj in TRUNK_TYPES and ti in ELECTRONICS_TYPES)
                        or (ti in ELECTRONICS_TYPES and tj in ELECTRONICS_TYPES)
                        # the head + tail meet the front/rear of the body shell
                        or ("head" in (ti, tj)
                            and (ti in TRUNK_TYPES or tj in TRUNK_TYPES))
                        or ("tail_segment" in (ti, tj)
                            and (ti in TRUNK_TYPES or tj in TRUNK_TYPES))
                        # the head is one printed piece -> features abut
                        or (ti in HEAD_FEATURES and tj in HEAD_FEATURES)
                        # cosmetic collar/bell hugging the neck/chest
                        or (ti in COSMETIC_TYPES and tj in COSMETIC_HOSTS)
                        or (tj in COSMETIC_TYPES and ti in COSMETIC_HOSTS)
                        # FUR knee bulge over its joint bones
                        or ("knee" in (ti, tj) and ti in LEG_TYPES and tj in LEG_TYPES)
                        or pair in EXPECTED_TYPE_PAIRS)
            offenders.append((vol, ni, nj, expected))
            if not expected:
                total_bad += vol

    offenders.sort(key=lambda r: -r[0])
    bad = [o for o in offenders if not o[3]]
    print(f"\nWorst overlaps (top 15 of {len(offenders)} overlapping pairs):")
    print(f"{'vol mm^3':>12}  {'exp?':>4}  part A  <->  part B")
    for vol, ni, nj, expected in offenders[:15]:
        tag = "ok" if expected else "BAD"
        print(f"{vol:12.0f}  {tag:>4}  {ni}  <->  {nj}")
    if bad:
        print(f"\nAll {len(bad)} UNEXPECTED (BAD) overlaps:")
        for vol, ni, nj, _ in bad:
            print(f"{vol:12.0f}  BAD  {ni}  <->  {nj}")
    print(f"\nTotal UNEXPECTED overlap volume: {total_bad:.0f} mm^3 "
          f"({sum(1 for o in offenders if not o[3])} bad pairs)")


if __name__ == "__main__":
    main()
