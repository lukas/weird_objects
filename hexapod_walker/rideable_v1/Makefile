# rideable_v1 BuildViz pipeline.
#
# The rideable design is a markdown-only mechanical draft (no parametric CAD),
# so `viz` GENERATES primitive stand-in geometry from design_spec.yaml into
# full_robot_viz/ (scene.json + stl/*.stl), `check` runs the generic geometry
# verifier, and `render` writes preview PNGs.  All targets shell out to the
# shared repo-root venv via ../../run.sh.

REPO_ROOT := $(abspath $(CURDIR)/../..)
RUN       := $(REPO_ROOT)/run.sh
STS       := $(REPO_ROOT)/hexapod_walker/prototype_sts3215
# BuildViz checkout that hosts the ONE machine-wide hub (default port 5183).
BUILDVIZ_ROOT ?= $(HOME)/buildviz

.PHONY: help viz check render all view-buildviz

help:
	@echo "rideable_v1 BuildViz targets:"
	@echo "  viz     Generate full_robot_viz/scene.json + stl/*.stl from design_spec.yaml."
	@echo "  check   Repo geometry verifier + design-consistency numbers + FULL buildviz"
	@echo "          check suite (mesh overlap / mating / watertight / joint articulation)."
	@echo "  render  Write PyVista preview PNGs (iso/side/front/top) into full_robot_viz/."
	@echo "  all     viz + check + render."
	@echo "  view-buildviz  Register the scene into the central BuildViz hub, open http://127.0.0.1:5183/?build=rideable_v1."

viz:
	$(RUN) hexapod_walker/rideable_v1/tools/rideable_viz_build.py

# check = three layers, all must pass:
#   1. the generic repo-side geometry verifier (scene sanity, voxel overlap),
#   2. the design-consistency regression script (mass/torque/chain/brake/power
#      numbers re-derived from design_spec.yaml + doc-drift greps),
#   3. the FULL BuildViz check suite via --buildviz (watertight, mesh_overlap,
#      mating_contact, and the joint_fused/joint_fastener/joint_articulation
#      checks against the scene's 18-DOF joints[] block).
check:
	$(RUN) $(STS)/tools/buildviz_checks.py \
		$(CURDIR)/full_robot_viz/scene.json --emit --json
	BUILDVIZ_ROOT=$(BUILDVIZ_ROOT) $(RUN) \
		hexapod_walker/rideable_v1/tools/design_consistency_check.py --buildviz

render:
	$(RUN) hexapod_walker/rideable_v1/tools/render_rideable.py

all: viz check render

# View in the ONE machine-wide BuildViz hub (default port 5183): regenerate the
# scene, then REGISTER full_robot_viz/ into the hub (auto-starts a detached hub
# if none is running) and open its URL.  We never start a per-project dev server
# on 5175/etc -- the hub serves every project's builds at once.
view-buildviz: viz
	cd $(BUILDVIZ_ROOT) && npx buildviz register $(CURDIR)/full_robot_viz --build-id rideable_v1
	@echo ">>> Central BuildViz hub: http://127.0.0.1:5183/?build=rideable_v1"
	@command -v open >/dev/null 2>&1 && open "http://127.0.0.1:5183/?build=rideable_v1" || true
