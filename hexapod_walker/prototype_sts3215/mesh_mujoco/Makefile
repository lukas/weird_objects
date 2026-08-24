VENV ?= /Users/lukas/weird_objects/.venv

.PHONY: build view view-stance clean

build:
	$(VENV)/bin/python build_mesh_model.py

view:
	./view_mesh.sh plant

view-stance:
	./view_mesh.sh stance

clean:
	rm -rf assets hexapod_mesh.xml
