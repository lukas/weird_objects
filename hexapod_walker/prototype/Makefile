# hexapod prototype CAD pipeline.
#
# All targets shell out to ../../run.sh so the shared repo-root venv
# (with trimesh, scipy, shapely, pyvista, matplotlib, etc.) is on the
# PATH for every Python invocation.

REPO_ROOT := $(abspath $(CURDIR)/../..)
RUN       := $(REPO_ROOT)/run.sh

.PHONY: check-cad check-cad-fast validate render build verify-prototype clean-artifacts help

help:
	@echo "Hexapod prototype CAD targets:"
	@echo "  check-cad         Build STLs, validate against design_spec.yaml, render views, write cad_report.md."
	@echo "  check-cad-fast    Same as check-cad but skip the slow workspace-sweep collision check."
	@echo "  validate          Run scripts/validate_geometry.py only (no renders, no report)."
	@echo "  render            Run scripts/render_views.py only."
	@echo "  build             Re-run build_all.py to regenerate stl_prototype/*.stl."
	@echo "  verify-prototype  Run the legacy _verify_prototype.main() directly."
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

clean-artifacts:
	rm -rf $(CURDIR)/artifacts
