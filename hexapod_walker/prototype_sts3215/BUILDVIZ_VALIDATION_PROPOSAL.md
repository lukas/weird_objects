# Moving validation into BuildViz — proposal + generic design

Status: proposal + working prototype (`tools/buildviz_checks.py`). Nothing
here changes the offline verifier or the external `buildviz/` package.

## 1. Which checks belong in BuildViz vs the offline verifier

The offline verifier (`_verify_prototype.py`) should keep everything that is a
**hard pass/fail gate on the parametric CAD** — i.e. things you must not be
able to merge/order parts past, and that need exact geometry the viewer
doesn't have (watertightness, fastener thread engagement, wire-harness reach,
print-thinness/voxel analysis). Those stay.

Checks that are fundamentally **spatial/visual "is this placed sanely" questions**
are far more useful *interactively in the viewer*, where a human can see the
offending geometry, rotate it, and decide intent in one second instead of
reading a `centroid=(x,y,z)` line in a log. Concretely, move (or mirror) these:

| Check (today, offline) | Why it's better in BuildViz |
| --- | --- |
| `Self-collision` (standing pose pairwise overlap) | The viewer already renders the exact verified pose; an overlap overlay turns a `mm^3` number into a red lens on the clash. |
| `Workspace self-collision` (175-pose sweep) | A scrub slider over (yaw, femur, knee) with a live overlap badge lets the engineer *watch* where legs hit the chassis — vastly more informative than 106 log lines, and it is the single slowest offline check. |
| `Servo clearance` / `Horn-stack` / `Horn-sweep clearance` | These are "does moving hardware poke into a printed wall" — i.e. clearance overlays between a swept envelope and a part. |
| `Cradle openness` / `Servo insertion path` | "can the body drop into the well" is an insertion-axis sweep that reads as an animation, not a number. |
| Placement sanity (no formal check today) | Catch a broken transform / mis-scaled mesh the instant the scene loads. |

Keep offline-only (need exact geometry / are gates): `Mesh watertightness`,
`Bolt-hole engagement`, `Flimsy joints`, `Thin sheets`, `Fastener engagement`,
`Mating-face contact`, `Cable clearance`, `Harness reach`, `Screwdriver access`.

The recommended split is **"viewer surfaces it, verifier gates it"**: the
verifier remains the CI gate, but emits its findings *into the scene* so the
viewer can paint them (see §3). The viewer additionally runs cheap generic
checks live so problems are visible before CI runs.

## 2. Generalizing: project-agnostic viewer-side validation

The key realization: a BuildViz `scene.json` already carries everything a large
class of checks needs — `meshes[]` (geometry URLs) and `instances[]`
(`transform`, `partType`, `role`, `centroid`). So generic checks can be driven
purely by **scene metadata**, with zero per-project hardcoding. This applies to
every build under `buildviz/public/builds`, not just the hexapod.

Generic, build-independent checks:

- **mesh_overlap** — pairwise solid interpenetration (AABB pre-filter →
  voxel overlap estimate, mm³). Sound early reject: overlap ≤ AABB-intersection
  volume, so near-misses skip raycasting.
- **clearance** — pairs closer than a configurable air-gap (proximity query).
- **placement** — per-instance sanity: non-finite/degenerate bounds, or a part
  flung far outside the scene's bulk radius (a broken transform).
- **scene_meta** — manifest hygiene: units present, every instance resolves to
  a mesh, no duplicate ids.

Per-project intent is supplied as data, never code, via an optional
`checksConfig` block in the scene:

```jsonc
"checksConfig": {
  "overlapMm3": 100,            // tolerance
  "pitchMm": 1.5,               // sampling pitch
  "ignoreOverlapPairs": [       // INTENTIONAL matings to suppress
    ["coxa_hip_bracket", "hip_servo"],
    ["chassis_bottom", "yaw_servo"]
  ]
}
```

`ignoreOverlapPairs` is the crucial generalization of the offline verifier's
hardcoded `JOINT_PAIRS` / `CLAMP_PAIRS` whitelists — expressed as scene data so
any project declares its own intended interferences.

## 3. Proposed BuildViz-side design (non-breaking, additive)

1. **`checks` field in `scene.json` (additive).** The scene loader is purely
   structural and ignores unknown top-level fields (`src/buildScene.ts`), so a
   `scene.checks` block is backward-compatible. Either the scene builder or the
   verifier writes it:

   ```jsonc
   "checks": [
     { "id": "overlap-040-051", "kind": "mesh_overlap", "status": "warn",
       "label": "coxa_hip_bracket ∩ hip_servo = 6288 mm³",
       "instances": ["040-coxa_hip_bracket", "051-hip_servo"],
       "point": [-98.9, 28.8, 62.7] }
   ]
   ```

2. **A generic "Checks" panel** (new React component) that lists `scene.checks`
   grouped by `kind`/`status`, with pass/warn/fail counts. Clicking a row calls
   the existing runtime highlight API:

   ```js
   window.buildviz.setHighlights({ parts: [...], points: [...] })
   ```

   No per-project code — it renders whatever `kind`/`label`/`instances` the
   scene declares.

3. **In-viewer overlap/clearance overlay** computed client-side with
   `three-mesh-bvh` (BVH-accelerated): for visible instance pairs, intersect
   BVHs and shade the contact region. Gated to the focused part / a toggle so
   it stays cheap. This is the live, interactive version of `mesh_overlap`.

4. **Workspace scrubber** (hexapod-flavored but generic mechanism): if a scene
   ships multiple posed variants (or joint transforms), expose a slider that
   re-evaluates the overlap overlay per pose.

## 4. Prototype delivered now

`tools/buildviz_checks.py` is a standalone, project-agnostic engine that
implements §2 today against any scene.json and is **non-destructive** (writes a
sidecar `buildviz_checks.json`, never touches `scene.json`):

```sh
python tools/buildviz_checks.py full_robot_viz/scene.json --emit
```

On the real 289-instance hexapod scene it runs in ~8 s (with `embreex`), passes
`scene_meta` + `placement`, and flags 37 interpenetrating pairs — almost all of
which are *intentional* clamp/cradle matings, demonstrating exactly why the
`ignoreOverlapPairs` triage (vs a hardcoded whitelist) is the right model. The
emitted sidecar already contains a `highlights: { parts, points }` payload
shaped for `window.buildviz.setHighlights`, so wiring step §3.2 is mechanical.

This is the seam to grow viewer-side validation across all projects: scene
metadata in, generic checks + highlight payloads out, project intent supplied
as config data.
