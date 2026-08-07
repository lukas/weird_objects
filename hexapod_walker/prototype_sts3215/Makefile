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

.PHONY: check-cad check-cad-fast check-strength validate render build verify-prototype verify-fast verify-changed clean-artifacts view view-static view-build view-buildviz verify-buildviz regen-fasteners bom-fasteners help

# Path to the BuildViz web inspector (https://github.com/lukas/buildviz).
# Override with `make view-buildviz BUILDVIZ_ROOT=/path/to/buildviz`.
BUILDVIZ_ROOT ?= $(HOME)/buildviz

help:
	@echo "Hexapod prototype CAD targets:"
	@echo "  check-cad         Build STLs, validate against design_spec.yaml, render views, write cad_report.md."
	@echo "  check-cad-fast    Same as check-cad but skip the slow workspace-sweep collision check."
	@echo "  check-strength    OPT-IN failure-point check (Gmsh + CalculiX FEA + closed-form beam bending)."
	@echo "                    Not wired into check-cad.  Example:  make check-strength MATERIAL=petg"
	@echo "  validate          Run scripts/validate_geometry.py only (no renders, no report)."
	@echo "  render            Run scripts/render_views.py only."
	@echo "  build             Re-run build_all.py → stl_prototype/ + stl_reference/ (CURRENT design)."
	@echo "  verify-prototype  Run the legacy _verify_prototype.main() directly (full suite, ~95-270 s)."
	@echo "  verify-fast       Run _verify_prototype.py --fast (5 essential checks, ~5-15 s)."
	@echo "  verify-changed    Run _verify_prototype.py --changed (only checks affected by git diff vs origin/main)."
	@echo "  view              Open the interactive MuJoCo viewer with the prototype walking in tripod gait."
	@echo "  view-static       Same viewer, but freeze the legs in the standing-pose keyframe (--no-gait)."
	@echo "  view-build        Open a PyVista build inspector with colored, labeled parts and an exploded-view slider."
	@echo "  view-buildviz     Re-export + REGISTER into the central BuildViz hub, open http://127.0.0.1:5183/?build=hexapod-prototype."
	@echo "  verify-buildviz   Regenerate full_robot_viz/scene.json (ONE scene, motion baked in), run the BuildViz"
	@echo "                    verifier + cheap validate, and BUMP a new versioned snapshot into the hub cache"
	@echo "                    (buildviz push --bump; keep BUILDVIZ_KEEP).  Pass SWEEP=1 for the ~13 s swept motion test."
	@echo "  regen-fasteners   Regenerate fasteners/<P/N>.cache.stl from real STEPs or parametric fallback."
	@echo "  bom-fasteners     Inject the auto-derived fastener BOM into docs/PROTOTYPE_BOM.md and docs/SHOPPING_LIST.md."
	@echo "  clean-artifacts   Delete artifacts/ (renders + cad_report.md)."

check-cad:
	$(RUN) hexapod_walker/prototype_sts3215/scripts/check_cad.py

check-cad-fast:
	$(RUN) hexapod_walker/prototype_sts3215/scripts/check_cad.py --fast

# Opt-in strength / failure-point pipeline.  NOT a dependency of
# check-cad: this is a separate observation-only pass that runs
# Gmsh + CalculiX (and a closed-form Euler-Bernoulli beam-bending
# sanity check) over the structural printed parts and writes
# ``artifacts/strength_report.md`` (with per-part stress-field PNGs
# under artifacts/strength/).  Install ``ccx`` and ``gmsh`` via
# Homebrew on macOS (see PROTOTYPE.md "Opt-in strength check"
# section) -- if either is missing the run falls back to the beam
# check alone and notes the missing solver in the report.
#
# Usage:
#   make check-strength                       # PETG default
#   make check-strength MATERIAL=pla          # PLA derated yield
#   make check-strength PARTS=tibia_link,femur_link SOLVER=beam
MATERIAL ?= petg
PARTS    ?=
SOLVER   ?= both
STRENGTH_ARGS := --material $(MATERIAL) --solver $(SOLVER)
ifneq ($(strip $(PARTS)),)
STRENGTH_ARGS += --parts $(PARTS)
endif
check-strength:
	$(RUN) hexapod_walker/prototype_sts3215/strength/__main__.py $(STRENGTH_ARGS)

validate:
	$(RUN) hexapod_walker/prototype_sts3215/scripts/validate_geometry.py

render:
	$(RUN) hexapod_walker/prototype_sts3215/scripts/render_views.py

build:
	$(RUN) hexapod_walker/prototype_sts3215/build_all.py --skip-assembly --skip-xometry --skip-bambu --skip-test-plate

verify-prototype:
	$(RUN) hexapod_walker/prototype_sts3215/_verify_prototype.py

# Inner-loop verifier: skip the 12 slow checks (workspace sweep,
# screwdriver-access ray cones, voxelised thin-sheet / flimsy-joint
# passes, ...) and run only the 5 ESSENTIAL invariants that catch
# ~80% of regressions in ~15% of the wall time.  Use this between
# every edit; gate commits on the full verify-prototype.
verify-fast:
	$(RUN) hexapod_walker/prototype_sts3215/_verify_prototype.py --fast

# Smart-selection verifier: only run the checks whose static
# input dependency set intersects what `git diff origin/main`
# changed.  Rebuilds STLs in-memory to detect which printed parts
# actually changed bytes (so a cosmetic comment-only edit to
# hexapod_prototype.py runs ~0 checks).  Combine with the cache
# from a previous full pass to skip everything else.
verify-changed:
	$(RUN) hexapod_walker/prototype_sts3215/_verify_prototype.py --changed

# Open the interactive MuJoCo viewer (passive mode) with the prototype
# walker in the procedurally generated terrain + obstacle world.  Mouse
# orbits; right-click drag applies an external force; Space pauses;
# Backspace snaps back to the keyframe.  Forward `make view ARGS=...`
# any extra CLI flag from mujoco_prototype.py, e.g.:
#   make view ARGS="--vx 0.20 --no-terrain --obstacles 0"
view:
	cd $(REPO_ROOT) && PYTHONPATH=$(REPO_ROOT) $(MJ_PY) hexapod_walker/prototype_sts3215/mujoco_prototype.py $(ARGS)

# Same viewer but with the gait disabled: the prototype freezes in its
# stance-pose keyframe so you can orbit and inspect part fitment without
# the legs cycling underneath you.
view-static:
	cd $(REPO_ROOT) && PYTHONPATH=$(REPO_ROOT) $(MJ_PY) hexapod_walker/prototype_sts3215/mujoco_prototype.py --no-gait --no-terrain --obstacles 0 $(ARGS)

# PyVista build inspector: every printable part placed at the assembled
# pose, colored by part type (matching the MuJoCo viewer's palette),
# with floating role labels, an exploded-view slider, and per-part-type
# hide/show checkboxes.  Plain python is fine -- pyvista does not need
# mjpython.  Forward ARGS=... for things like
#   make view-build ARGS="--screenshot /tmp/build.png --explode 0.8"
view-build:
	cd $(REPO_ROOT) && PYTHONPATH=$(REPO_ROOT) $(REPO_ROOT)/.venv/bin/python hexapod_walker/prototype_sts3215/scripts/inspect_build.py $(ARGS)

# Browser-based build inspector (https://github.com/lukas/buildviz).  To VIEW
# the sts3215 full robot, publish it into the ONE machine-wide BuildViz hub
# (default port 5183) with `make verify-buildviz` (buildviz push --bump), then
# open http://127.0.0.1:5183/?build=prototype_sts3215 .  Never start a
# per-project Vite dev server on 5173/etc -- the hub serves every build at once.
#
# This legacy target exports the prototype_v1 hexapod-prototype build and opens
# it in the central hub (kept for parity with prototype_v1/Makefile).  Override
# the BuildViz checkout location with:
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
	cd $(BUILDVIZ_ROOT) && npx buildviz register public/builds/hexapod-prototype --build-id hexapod-prototype
	@echo ">>> Central BuildViz hub: http://127.0.0.1:5183/?build=hexapod-prototype"
	@command -v open >/dev/null 2>&1 && open "http://127.0.0.1:5183/?build=hexapod-prototype" || true

# Versioned BuildViz publish: regenerate the full-robot scene (ONE scene.json
# that now bakes in the BuildViz motion blocks -- joints[]/poses[]/animations[] --
# alongside the static geometry; there is no separate scene_motion.json or
# prototype_sts3215_motion build any more), run the generic geometry verifier
# (tools/buildviz_checks.py), then BUMP a brand-new version into the machine-wide
# BuildViz hub cache via `buildviz push --bump`.  Every run archives the
# just-verified revision as the next v<N> (newest = default) -- ONE bump per push
# for the ONE build -- so `buildviz versions prototype_sts3215` and the hub
# accumulate history with no manual bookkeeping.  `--upload-assets` makes the
# pushed build self-contained (relative STL bytes are uploaded + content-deduped);
# BUILDVIZ_KEEP caps how many versions are retained (oldest non-default pruned).
#
# Motion cost (measured Jul 2026): baking joints/poses/animations is ~0.01 s on
# top of the ~4 s STL rebuild (free), so motion is ALWAYS generated + published.
# Every push also runs the CHEAP `buildviz validate` (~1 s) which confirms the
# baked joints/poses are well-formed.  The one EXPENSIVE motion test is the swept
# self-overlap check (`buildviz sweep` ~13 s -- 3x the whole build), so it is
# OPT-IN: pass SWEEP=1 to run it (e.g. before shipping a kinematics change).
#
#   make -C hexapod_walker/prototype_sts3215 verify-buildviz            # fast: build+validate+push
#   make -C hexapod_walker/prototype_sts3215 verify-buildviz SWEEP=1    # + swept self-overlap test (~13 s)
#   make -C hexapod_walker/prototype_sts3215 verify-buildviz BUILDVIZ_KEEP=10
BUILDVIZ_BUILD_ID ?= prototype_sts3215
BUILDVIZ_KEEP     ?= 20
SWEEP             ?=
verify-buildviz:
	$(RUN) hexapod_walker/prototype_sts3215/tools/full_robot_viz_build.py
	$(RUN) hexapod_walker/prototype_sts3215/tools/buildviz_checks.py \
		$(CURDIR)/full_robot_viz/scene.json \
		--emit --publish --build-id $(BUILDVIZ_BUILD_ID) --keep $(BUILDVIZ_KEEP)
	@echo ">>> Validating published build (cheap motion + manifest check)..."
	npx buildviz validate $(BUILDVIZ_BUILD_ID) --json >/dev/null && \
		echo "    buildviz validate: OK"
	@if [ -n "$(SWEEP)" ]; then \
		echo ">>> SWEEP=1: running swept self-overlap motion test (~13 s)..."; \
		npx buildviz sweep $(BUILDVIZ_BUILD_ID) --json >/dev/null && \
			echo "    buildviz sweep: OK"; \
	else \
		echo ">>> (skipping expensive swept self-overlap test; pass SWEEP=1 to run it)"; \
	fi

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
	$(RUN) hexapod_walker/prototype_sts3215/scripts/make_fastener_meshes.py

# Auto-derive the fastener BOM table from fastener_registry and inject
# it (between the BEGIN FASTENERS / END FASTENERS markers) into the
# user-facing BOM docs.  Run after editing fastener_registry.py.
#
# Paths must be absolute because run.sh cd's into the script's own
# directory before executing -- a path relative to the repo root
# (e.g. ``hexapod_walker/prototype_sts3215/docs/PROTOTYPE_BOM.md``) would
# then resolve against ``prototype_sts3215/scripts/``, where the BOM
# file does not live.
bom-fasteners:
	$(RUN) hexapod_walker/prototype_sts3215/scripts/render_fastener_bom.py \
		--inject $(CURDIR)/docs/PROTOTYPE_BOM.md \
		--inject $(CURDIR)/docs/SHOPPING_LIST.md

clean-artifacts:
	rm -rf $(CURDIR)/artifacts
