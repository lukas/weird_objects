# BuildViz

This project is set up for BuildViz. Agents should use BuildViz when inspecting
or debugging CAD/build outputs, STL assets, `scene.json`, or
`design_spec.yaml`.

## Files

- `scene.json`: BuildViz scene manifest. It lists meshes, instances, transforms,
  colors, roles, and focus groups — **and now the baked-in motion blocks**
  (`joints[]` + `poses[]` + `animations[]`). It lives in `full_robot_viz/`.
- `design_spec.yaml`: Semantic design source for part descriptions, features,
  holes, dimensions, and LLM context.
- STL files: Mesh assets referenced by `scene.json`.

## One build, motion baked in

There is exactly ONE build for the full robot: **`prototype_sts3215`**, served
from `full_robot_viz/`. Motion is part of that single `scene.json` — it carries
BuildViz's `joints[]` (18 leg DOFs: 6 legs × yaw/hip/knee), `poses[]` (stance
variants + a tripod march) and a looping `animations[]` walk clip, on the exact
same meshes and home-pose transforms as the static geometry. This mirrors the
prototype_v1 `hexapod-prototype` build.

There is **no** separate `scene_motion.json` file and **no** separate
`prototype_sts3215_motion` build id any more — both were retired when motion was
folded into the one build.

### Why motion is always on (but the sweep is opt-in)

Measured Jul 2026: baking the motion blocks costs ~0.01 s on top of the ~4 s STL
rebuild — free — so motion is **always** generated and published. Every push also
runs the cheap `buildviz validate` (~1 s) to confirm the joints/poses are
well-formed. The one expensive motion test is the swept self-overlap check
(`buildviz sweep`, ~13 s ≈ 3× the whole build), so it is **opt-in**:

```sh
make -C hexapod_walker/prototype_sts3215 verify-buildviz          # fast: build + validate + push (motion baked in)
make -C hexapod_walker/prototype_sts3215 verify-buildviz SWEEP=1  # + swept self-overlap motion test (~13 s)
```

## Commands

Validate the build:

```sh
npx buildviz validate . --json
```

View the build — use the ONE machine-wide hub (default port `5183`), never a
new per-project dev server:

```sh
npx buildviz hub --detach                        # idempotent; single hub on :5183
# publishing is normally done by `make verify-buildviz`; to register the dir directly:
npx buildviz register full_robot_viz --build-id prototype_sts3215
# then open: http://127.0.0.1:5183/?build=prototype_sts3215
```

Do NOT run `npm run dev` or `npx buildviz --port <n>` (e.g. the old
`--port 5174` motion-preview pattern) to view builds — the hub already serves
every project's builds at once. See `~/buildviz/README.md` ("How to run
BuildViz") for the full convention.

Regenerate a starter scene only when `scene.json` is missing:

```sh
npx buildviz init --dry-run
npx buildviz init
```

## Agent Notes

- Do not guess part semantics from STL geometry alone. Use `design_spec.yaml`
  as the source of truth.
- If asking a human a visual question, use `window.buildviz.setHighlights(...)`
  with `annotation` text after opening the viewer.
- This initial scene may place STL files in a simple grid. For a true assembly,
  update `scene.json` with real transforms from the CAD/export pipeline.

Build name: prototype
