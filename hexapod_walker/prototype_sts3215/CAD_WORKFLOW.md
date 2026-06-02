# Hexapod prototype — CAD workflow

This folder builds a tabletop hexapod from 3D-printed parts.  The
pipeline is parametric end-to-end: there is **no GUI CAD file** — the
STLs in `stl_prototype/` are derived artifacts.  This document is the
short version of how the pieces fit together and how to validate a
change before printing.

## Source of truth

| Layer                                  | File                          | What it is                                                              |
|----------------------------------------|-------------------------------|-------------------------------------------------------------------------|
| Parametric geometry                    | `hexapod_prototype.py`        | Python + `trimesh`.  All `make_*` functions; constants block at the top.|
| Derived STL artifacts                  | `stl_prototype/*.stl`         | Auto-generated from `hexapod_prototype.py` via `build_all.py`.          |
| Human-readable design contract         | `design_spec.yaml`            | Local frames, bounding boxes, holes, channels, keep-out volumes.        |
| Code-driven keep-out / clearance shapes| `keepout_volumes.py`          | Named `trimesh` factories tracking the parametric constants.            |
| Deep validation                        | `_verify_prototype.py`        | The 11-check correctness suite (manifoldness, cradle openness, …).      |
| Light validation + reporting           | `scripts/validate_geometry.py`| Spec-vs-STL regression check + report generator.                        |

The contract is: **constants in `hexapod_prototype.py` are the truth**.
`design_spec.yaml` mirrors a curated subset of those constants and adds
human-readable documentation.  The validator catches drift between the
two.

## One-liner: validate everything before you commit

```
make -C hexapod_walker/prototype check-cad
```

This runs (from the repo root):

1. `build_all.py` to ensure `stl_prototype/*.stl` exists.
2. `scripts/validate_geometry.py` — bounding boxes, holes, keep-out
   volumes (resolved against `keepout_volumes.KEEP_OUT_VOLUMES`), and
   the full `_verify_prototype.main()` suite.
3. `scripts/render_views.py` — 4 standard views (front / side / top /
   iso) per part, with keep-out volumes overlaid translucently, under
   `artifacts/views/<part>/`.
4. `artifacts/cad_report.md` — copy-pastable Markdown summary of pass/
   fail status, discovered dimensions, missing `TODO`s in the spec,
   exact error messages from the validator, and links to all generated
   images.

`make check-cad-fast` runs the same pipeline but reduces the
`_verify_prototype` workspace-sweep collision check to a single pose
(useful for the inner-loop edit cycle; **don't trust a fast-mode pass
as the final go-ahead** — run the full sweep at least once before
committing).

## Individual steps

| Goal                         | Make target            | Equivalent direct invocation                                                                          |
|------------------------------|------------------------|-------------------------------------------------------------------------------------------------------|
| Regenerate STLs              | `make build`           | `./run.sh hexapod_walker/prototype/build_all.py --skip-assembly --skip-xometry --skip-bambu --skip-test-plate` |
| Run spec validator only      | `make validate`        | `./run.sh hexapod_walker/prototype/scripts/validate_geometry.py`                                      |
| Render PNG views only        | `make render`          | `./run.sh hexapod_walker/prototype/scripts/render_views.py`                                            |
| Legacy verify only           | `make verify-prototype`| `./run.sh hexapod_walker/prototype/_verify_prototype.py`                                              |

The render backend tries `pyrender` → `pyvista` → `matplotlib` in
order; the first one that works is used.  `pyvista` ships with the
repo's default `requirements.txt`, so the pipeline degrades gracefully
on a fresh checkout.

## Recommended edit loop

1. Read the relevant `make_*` function's docstring and the constants
   it references.
2. Make the smallest possible parametric change.
3. `make -C hexapod_walker/prototype check-cad-fast` for a quick check
   (~30 s).
4. If FAIL: open `artifacts/cad_report.md`, find the failing check,
   paste its line into your LLM along with the relevant `make_*` source.
5. If PASS: run `make -C hexapod_walker/prototype check-cad` (the full
   workspace sweep, ~ 2 min) before committing.

See `CAD_AGENT_INSTRUCTIONS.md` for the rules an LLM coding agent
should follow when editing CAD.
