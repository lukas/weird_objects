# BuildViz

This project is set up for BuildViz. Agents should use BuildViz when inspecting
or debugging CAD/build outputs, STL assets, `scene.json`, or
`design_spec.yaml`.

## Files

- `scene.json`: BuildViz scene manifest. It lists meshes, instances, transforms,
  colors, roles, and focus groups.
- `design_spec.yaml`: Semantic design source for part descriptions, features,
  holes, dimensions, and LLM context.
- STL files: Mesh assets referenced by `scene.json`.

## Commands

Validate the build:

```sh
npx buildviz validate . --json
```

Start the viewer:

```sh
npx buildviz --port 5174
```

Attach a different design spec while debugging:

```sh
npx buildviz . --design-spec /path/to/design_spec.yaml --port 5174
```

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
