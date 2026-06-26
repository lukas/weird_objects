#!/usr/bin/env python3
"""Connectivity / "hovering" audit for the robot-cat build.

The overlap checker (``check_overlaps.py``) proves parts don't INTERPENETRATE.
This checker proves the opposite failure mode does not happen either: that no
part FLOATS in mid-air.  A robot is only buildable if every part physically
TOUCHES the thing it bolts to.

Method
------
For every instance we measure the nearest-surface gap to every other instance
(AABB-prefiltered, then exact vertex->surface distance both directions; an AABB
overlap counts as a 0 mm touch).  Two checks:

1. **Nearest-neighbour** -- every part must touch SOMETHING within ``GAP_OK``.
2. **Connected components** -- the parts that touch within ``GAP_OK`` form a
   graph; the whole robot must be ONE connected component.  This catches the
   sneaky failure where (say) a servo + its clamp touch each OTHER but the pair
   floats off the leg it is supposed to drive -- nearest-neighbour alone would
   pass that, connectivity will not.

We additionally check that every paw touches the ground plane z = 0 (standing).

Run:  ./run.sh robot_cat/check_assembly.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh
from trimesh.proximity import ProximityQuery

HERE = Path(__file__).resolve().parent
BUILD = HERE / "cat_viz"

GAP_OK = 2.0          # mm; a connecting interface above this gap = hovering
GROUND_OK = 1.5       # mm; a paw above this off z=0 = not on the ground


def aabb_gap(a: np.ndarray, b: np.ndarray) -> float:
    """Min Euclidean distance between two axis-aligned boxes (0 if they touch
    or overlap).  Lower bound on the true surface gap."""
    lo = np.maximum(a[0], b[0])
    hi = np.minimum(a[1], b[1])
    sep = np.maximum(0.0, np.maximum(a[0] - b[1], b[0] - a[1]))
    return float(np.linalg.norm(sep))


def surface_gap(mi: trimesh.Trimesh, mj: trimesh.Trimesh) -> float:
    """Nearest-surface distance between two meshes (0 if they overlap)."""
    g = aabb_gap(mi.bounds, mj.bounds)
    if g > GAP_OK + 12.0:
        return g                       # far apart -> AABB bound is enough
    di = ProximityQuery(mj).on_surface(mi.vertices)[1].min()
    dj = ProximityQuery(mi).on_surface(mj.vertices)[1].min()
    gap = float(min(di, dj))
    # if the AABBs interpenetrate AND a vertex sits inside, treat as touching
    if g == 0.0:
        return 0.0 if gap < 1.0 else gap
    return gap


def load_instances():
    scene = json.loads((BUILD / "scene.json").read_text())
    mesh_by_id = {m["id"]: m for m in scene["meshes"]}
    parts = []
    for inst in scene["instances"]:
        mj = mesh_by_id[inst["meshId"]]
        m = trimesh.load(BUILD / mj["url"], process=False)
        parts.append((inst["name"], inst["partType"], inst.get("role", "part"), m))
    return parts


class _UF:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, a):
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


def main():
    parts = load_instances()
    n = len(parts)
    print(f"Connectivity audit: {n} parts (GAP_OK={GAP_OK} mm)\n")

    best = [(1e9, None) for _ in range(n)]   # (gap, neighbour idx) per part
    uf = _UF(n)
    for i in range(n):
        ni, ti, ri, mi = parts[i]
        for j in range(i + 1, n):
            nj, tj, rj, mj = parts[j]
            lb = aabb_gap(mi.bounds, mj.bounds)
            # need precise gap if it could be an edge (<=GAP_OK) or a new nearest
            if lb > GAP_OK and lb >= best[i][0] and lb >= best[j][0]:
                continue
            g = surface_gap(mi, mj)
            if g <= GAP_OK:
                uf.union(i, j)
            if g < best[i][0]:
                best[i] = (g, j)
            if g < best[j][0]:
                best[j] = (g, i)

    # connected components
    comps = {}
    for i in range(n):
        comps.setdefault(uf.find(i), []).append(i)
    comp_list = sorted(comps.values(), key=len, reverse=True)

    rows = []
    for i in range(n):
        ni, ti, ri, mi = parts[i]
        gap, jn = best[i]
        neigh = parts[jn][0] if jn is not None else "(none)"
        rows.append((gap, ni, ti, neigh))

    rows.sort(key=lambda r: -r[0])
    print("Largest nearest-neighbour gaps (top 15):")
    print(f"{'gap mm':>8}  {'flag':>5}  part  ->  nearest neighbour")
    for gap, ni, ti, neigh in rows[:15]:
        flag = "HOVER" if gap > GAP_OK else "ok"
        print(f"{gap:8.2f}  {flag:>5}  {ni}  ->  {neigh}")

    hover = [r for r in rows if r[0] > GAP_OK]
    print(f"\nHovering parts (> {GAP_OK} mm from anything): {len(hover)}")
    for gap, ni, ti, neigh in hover:
        print(f"  {gap:7.2f} mm  {ni}  (nearest: {neigh})")

    # --- feet on the ground ---
    print("\nGround contact (paws, want bottom z ~ 0):")
    worst_foot = 0.0
    for ni, ti, ri, mi in parts:
        if ri == "paw":
            zb = float(mi.bounds[0][2])
            worst_foot = max(worst_foot, abs(zb))
            flag = "HOVER" if zb > GROUND_OK else "ok"
            print(f"  {zb:7.2f} mm  {flag:>5}  {ni}")

    # --- connected components (the whole robot must be ONE) ---
    print(f"\nConnected components (parts touching within {GAP_OK} mm): {len(comp_list)}")
    for k, comp in enumerate(comp_list):
        head = ", ".join(parts[i][0] for i in comp[:6])
        more = "" if len(comp) <= 6 else f", +{len(comp) - 6} more"
        tag = "MAIN" if k == 0 else "FLOATING ISLAND"
        print(f"  [{tag}] {len(comp)} parts: {head}{more}")

    worst = rows[0][0]
    one_body = len(comp_list) == 1
    ok = len(hover) == 0 and worst_foot <= GROUND_OK and one_body
    print(f"\nWorst connecting gap: {worst:.2f} mm   |   worst foot-off-ground: "
          f"{worst_foot:.2f} mm   |   hovering parts: {len(hover)}   |   "
          f"components: {len(comp_list)}")
    print("RESULT:", "OK - one connected assembly, nothing hovers" if ok
          else "HOVERING / DISCONNECTED DETECTED")


if __name__ == "__main__":
    main()
