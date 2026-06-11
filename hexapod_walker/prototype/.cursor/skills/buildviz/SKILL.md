---
name: buildviz
description: Use BuildViz for CAD/build visualization workflows. Use when working with scene.json, design_spec.yaml, STL files, physical build assets, feature labels, dimensions, or when the user asks to inspect, visualize, validate, or annotate a mechanical design.
---

# BuildViz

## When To Use

Use this skill when the task involves BuildViz, `scene.json`,
`design_spec.yaml`, STL assets, CAD build visualization, feature labels,
dimensions, or human-in-the-loop visual debugging.

## Workflow

1. Check for `BUILDVIZ.md`, `scene.json`, and `design_spec.yaml` in the
   project root.
2. If `scene.json` is missing, run:

   ```sh
   npx buildviz init --dry-run
   ```

   Only run `npx buildviz init` after confirming it will not overwrite useful
   project files.

3. Validate before relying on the viewer:

   ```sh
   npx buildviz validate . --json
   ```

4. Start the viewer on a configurable port:

   ```sh
   npx buildviz --port 5174
   ```

5. To attach a different spec while debugging:

   ```sh
   npx buildviz . --design-spec /path/to/design_spec.yaml --port 5174
   ```

## Asking Human Questions

Use the runtime highlight API in the browser:

```js
window.buildviz.setHighlights({
  parts: [{ partType: 'coxa_link', annotation: 'Should this part be wider?' }],
  points: [{ partType: 'coxa_link', point: [0, 0, 0], annotation: 'Is this origin right?' }],
})
```

Clear highlights:

```js
window.buildviz.clearHighlights()
```

## Rules

- Treat `design_spec.yaml` as the source of truth for design intent.
- Treat `scene.json` as the source of truth for what BuildViz renders.
- Do not infer design intent from STL geometry alone.
- Do not overwrite `scene.json`, `BUILDVIZ.md`, or Cursor skill files unless
  the user explicitly asks.
