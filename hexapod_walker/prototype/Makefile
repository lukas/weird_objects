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

.PHONY: check-cad check-cad-fast validate render build verify-prototype clean-artifacts view view-static help

help:
	@echo "Hexapod prototype CAD targets:"
	@echo "  check-cad         Build STLs, validate against design_spec.yaml, render views, write cad_report.md."
	@echo "  check-cad-fast    Same as check-cad but skip the slow workspace-sweep collision check."
	@echo "  validate          Run scripts/validate_geometry.py only (no renders, no report)."
	@echo "  render            Run scripts/render_views.py only."
	@echo "  build             Re-run build_all.py to regenerate stl_prototype/*.stl."
	@echo "  verify-prototype  Run the legacy _verify_prototype.main() directly."
	@echo "  view              Open the interactive MuJoCo viewer with the prototype walking in tripod gait."
	@echo "  view-static       Same viewer, but freeze the legs in the standing-pose keyframe (--no-gait)."
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

clean-artifacts:
	rm -rf $(CURDIR)/artifacts
