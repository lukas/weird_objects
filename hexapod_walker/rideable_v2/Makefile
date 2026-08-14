# rideable_v2 BuildViz + design-check pipeline.
#
# The rideable v2 design is a markdown-only mechanical draft (no parametric
# CAD), so `viz` GENERATES primitive stand-in geometry from design_spec.yaml
# into full_robot_viz/ (scene.json + stl/*.stl), `check` re-derives
# every load / torque / belt / lock / power / cost figure from
# design_spec.yaml (plus the geometry verifier and the FULL buildviz check
# suite once the scene exists), and `render` writes preview PNGs.
#
# Targets shell out to the shared repo-root venv's python directly (run.sh
# predates the uv-managed venv and tries to pip-install).

REPO_ROOT := $(abspath $(CURDIR)/../..)
PY        := $(REPO_ROOT)/.venv/bin/python
STS       := $(REPO_ROOT)/hexapod_walker/prototype_sts3215
# BuildViz checkout that hosts the ONE machine-wide hub (port 5183).
BUILDVIZ_ROOT ?= $(HOME)/buildviz

.PHONY: help viz check render all view-buildviz

help:
	@echo "rideable_v2 targets:"
	@echo "  viz     Generate full_robot_viz/scene.json + stl/*.stl from design_spec.yaml."
	@echo "  check   Repo geometry verifier + design-consistency numbers + FULL buildviz"
	@echo "          check suite (mesh overlap / mating / watertight / joint articulation)."
	@echo "  render  Write PyVista preview PNGs (iso/side/front/top) into full_robot_viz/."
	@echo "  all     viz + check + render."
	@echo "  view-buildviz  Register the scene into the central BuildViz hub, open http://127.0.0.1:5183/?build=rideable_v2."

viz:
	$(PY) $(CURDIR)/tools/rideable_viz_build.py

# check = three layers, all must pass:
#   1. the generic repo-side geometry verifier (scene sanity, voxel overlap),
#   2. the design-consistency regression script (mass/torque/belt/lock/power
#      numbers re-derived from design_spec.yaml + doc-drift greps),
#   3. the FULL BuildViz check suite via --buildviz (watertight, mesh_overlap,
#      mating_contact, and the joint checks against the 18-DOF joints[] block).
check:
	$(PY) $(STS)/tools/buildviz_checks.py \
		$(CURDIR)/full_robot_viz/scene.json --emit --json
	cd $(CURDIR) && BUILDVIZ_ROOT=$(BUILDVIZ_ROOT) $(PY) \
		tools/design_consistency_check.py --buildviz

render:
	$(PY) $(CURDIR)/tools/render_rideable.py

all: viz check render

# View in the ONE machine-wide BuildViz hub (port 5183): regenerate the scene,
# REGISTER full_robot_viz/ into the hub (auto-starts a detached hub if none is
# running), open its URL.  Never start a per-project dev server.
view-buildviz: viz
	cd $(BUILDVIZ_ROOT) && npx buildviz register $(CURDIR)/full_robot_viz --build-id rideable_v2
	@echo ">>> Central BuildViz hub: http://127.0.0.1:5183/?build=rideable_v2"
	@command -v open >/dev/null 2>&1 && open "http://127.0.0.1:5183/?build=rideable_v2" || true
