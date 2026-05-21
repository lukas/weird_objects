# hexapod prototype CAD pipeline.
#
# All targets shell out to ../../run.sh so the shared repo-root venv
# (with trimesh, scipy, shapely, pyvista, matplotlib, etc.) is on the
# PATH for every Python invocation.

REPO_ROOT := $(abspath $(CURDIR)/../..)
RUN       := $(REPO_ROOT)/run.sh

# ``mujoco.viewer.launch_passive`` requires ``mjpython`` on macOS (it
# needs to own the main thread for Cocoa to spin up an NSApplication).
# Ship via the repo's shared venv, falling back to plain python on
# Linux / other platforms where the regular interpreter is fine.
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
MJ_PY  := $(REPO_ROOT)/.venv/bin/mjpython
else
MJ_PY  := $(REPO_ROOT)/.venv/bin/python
endif

.PHONY: check-cad check-cad-fast validate render build verify-prototype verify-fast verify-changed clean-artifacts view view-static view-build view-buildviz regen-fasteners bom-fasteners help

# Path to the BuildViz web inspector (https://github.com/lukas/buildviz).
# Override with `make view-buildviz BUILDVIZ_ROOT=/path/to/buildviz`.
BUILDVIZ_ROOT ?= $(HOME)/buildviz

help:
	@echo "Hexapod prototype CAD targets:"
	@echo "  check-cad         Build STLs, validate against design_spec.yaml, render views, write cad_report.md."
	@echo "  check-cad-fast    Same as check-cad but skip the slow workspace-sweep collision check."
	@echo "  validate          Run scripts/validate_geometry.py only (no renders, no report)."
	@echo "  render            Run scripts/render_views.py only."
	@echo "  build             Re-run build_all.py to regenerate stl_prototype/*.stl."
	@echo "  verify-prototype  Run the legacy _verify_prototype.main() directly (full suite, ~95-270 s)."
	@echo "  verify-fast       Run _verify_prototype.py --fast (5 essential checks, ~5-15 s)."
	@echo "  verify-changed    Run _verify_prototype.py --changed (only checks affected by git diff vs origin/main)."
	@echo "  view              Open the interactive MuJoCo viewer with the prototype walking in tripod gait."
	@echo "  view-static       Same viewer, but freeze the legs in the standing-pose keyframe (--no-gait)."
	@echo "  view-build        Open a PyVista build inspector with colored, labeled parts and an exploded-view slider."
	@echo "  view-buildviz     Re-export to lukas/buildviz (Three.js web inspector) and serve at http://localhost:5173."
	@echo "  regen-fasteners   Regenerate fasteners/<P/N>.cache.stl from real STEPs or parametric fallback."
	@echo "  bom-fasteners     Inject the auto-derived fastener BOM into PROTOTYPE_BOM.md and SHOPPING_LIST.md."
	@echo "  clean-artifacts   Delete artifacts/ (renders + cad_report.md)."

check-cad:
	$(RUN) hexapod_walker/prototype/scripts/check_cad.py

check-cad-fast:
	$(RUN) hexapod_walker/prototype/scripts/check_cad.py --fast

validate:
	$(RUN) hexapod_walker/prototype/scripts/validate_geometry.py

render:
	$(RUN) hexapod_walker/prototype/scripts/render_views.py

build:
	$(RUN) hexapod_walker/prototype/build_all.py --skip-assembly --skip-xometry --skip-bambu --skip-test-plate

verify-prototype:
	$(RUN) hexapod_walker/prototype/_verify_prototype.py

# Inner-loop verifier: skip the 12 slow checks (workspace sweep,
# screwdriver-access ray cones, voxelised thin-sheet / flimsy-joint
# passes, ...) and run only the 5 ESSENTIAL invariants that catch
# ~80% of regressions in ~15% of the wall time.  Use this between
# every edit; gate commits on the full verify-prototype.
verify-fast:
	$(RUN) hexapod_walker/prototype/_verify_prototype.py --fast

# Smart-selection verifier: only run the checks whose static
# input dependency set intersects what `git diff origin/main`
# changed.  Rebuilds STLs in-memory to detect which printed parts
# actually changed bytes (so a cosmetic comment-only edit to
# hexapod_prototype.py runs ~0 checks).  Combine with the cache
# from a previous full pass to skip everything else.
verify-changed:
	$(RUN) hexapod_walker/prototype/_verify_prototype.py --changed

# Open the interactive MuJoCo viewer (passive mode) with the prototype
# walker in the procedurally generated terrain + obstacle world.  Mouse
# orbits; right-click drag applies an external force; Space pauses;
# Backspace snaps back to the keyframe.  Forward `make view ARGS=...`
# any extra CLI flag from mujoco_prototype.py, e.g.:
#   make view ARGS="--vx 0.20 --no-terrain --obstacles 0"
view:
	cd $(REPO_ROOT) && PYTHONPATH=$(REPO_ROOT) $(MJ_PY) hexapod_walker/prototype/mujoco_prototype.py $(ARGS)

# Same viewer but with the gait disabled: the prototype freezes in its
# stance-pose keyframe so you can orbit and inspect part fitment without
# the legs cycling underneath you.
view-static:
	cd $(REPO_ROOT) && PYTHONPATH=$(REPO_ROOT) $(MJ_PY) hexapod_walker/prototype/mujoco_prototype.py --no-gait --no-terrain --obstacles 0 $(ARGS)

# PyVista build inspector: every printable part placed at the assembled
# pose, colored by part type (matching the MuJoCo viewer's palette),
# with floating role labels, an exploded-view slider, and per-part-type
# hide/show checkboxes.  Plain python is fine -- pyvista does not need
# mjpython.  Forward ARGS=... for things like
#   make view-build ARGS="--screenshot /tmp/build.png --explode 0.8"
view-build:
	cd $(REPO_ROOT) && PYTHONPATH=$(REPO_ROOT) $(REPO_ROOT)/.venv/bin/python hexapod_walker/prototype/inspect_build.py $(ARGS)

# Browser-based build inspector (https://github.com/lukas/buildviz): a
# Three.js / React port of inspect_build.py.  Workflow:
#   1. Run buildviz's Python exporter (under our own venv via
#      WEIRD_OBJECTS_ROOT), which re-reads inspect_build._build_assembly_*
#      and writes buildviz/public/builds/hexapod-prototype/{manifest.json,
#      *.stl}.  This is what keeps the web viewer in sync with whatever
#      the PyVista inspector would show.
#   2. Start the Vite dev server.  --open hands the URL to the OS so the
#      default browser pops up once Vite is listening.
#
# Override the BuildViz checkout location with
#   make view-buildviz BUILDVIZ_ROOT=/path/to/buildviz
view-buildviz:
	@if [ ! -d "$(BUILDVIZ_ROOT)" ]; then \
		echo "Error: BUILDVIZ_ROOT=$(BUILDVIZ_ROOT) not found."; \
		echo "Clone with: gh repo clone lukas/buildviz $(BUILDVIZ_ROOT)"; \
		exit 1; \
	fi
	@if [ ! -d "$(BUILDVIZ_ROOT)/node_modules" ]; then \
		echo ">>> Installing buildviz npm deps in $(BUILDVIZ_ROOT)..."; \
		cd $(BUILDVIZ_ROOT) && npm install; \
	fi
	cd $(BUILDVIZ_ROOT) && WEIRD_OBJECTS_ROOT=$(REPO_ROOT) npm run export:hexapod
	cd $(BUILDVIZ_ROOT) && npm run dev -- --open

# ---------------------------------------------------------------------------
# Fastener pipeline (May 2026 -- see fastener_registry.py + fasteners/README.md)
# ---------------------------------------------------------------------------

# Regenerate fasteners/<P/N>.cache.stl for every unique part number in
# fastener_registry.build_all_fastener_instances().  Source priority:
# real McMaster STEP -> user STL -> NopSCADlib via OpenSCAD CLI ->
# parametric fallback.  Both inspect_build.py and the verifier's
# check_screwdriver_access consume the cache STLs.  Run this whenever
# you drop a real STEP into fasteners/, edit a fasteners/scad/*.scad
# file, or upgrade NopSCADlib.  See fasteners/README.md for the
# install commands (openscad CLI + NopSCADlib clone).
regen-fasteners:
	$(RUN) hexapod_walker/prototype/make_fastener_meshes.py

# Auto-derive the fastener BOM table from fastener_registry and inject
# it (between the BEGIN FASTENERS / END FASTENERS markers) into the
# user-facing BOM docs.  Run after editing fastener_registry.py.
#
# Paths must be absolute because run.sh cd's into the script's own
# directory before executing -- a path relative to the repo root
# (e.g. ``hexapod_walker/prototype/PROTOTYPE_BOM.md``) would then
# resolve against ``prototype/scripts/``, where the BOM file does
# not live.
bom-fasteners:
	$(RUN) hexapod_walker/prototype/scripts/render_fastener_bom.py \
		--inject $(CURDIR)/PROTOTYPE_BOM.md \
		--inject $(CURDIR)/SHOPPING_LIST.md

clean-artifacts:
	rm -rf $(CURDIR)/artifacts
