# STEP-first CAD sidecar test

This directory is an additive migration experiment. It does not overwrite the
existing `hexapod_prototype.py` / `stl_prototype/` mesh pipeline.

It is not intended to become a second permanent geometry system. The migration
rule is:

1. Keep the current mesh generator as the trusted printer/reference path while
   STEP-first parts are being proven.
2. Import dimensions from `hexapod_prototype.py` during the experiment so
   constants do not fork.
3. Once the STEP-first builders cover the real printables, move them inward as
   the source of truth and make STL export a derived final step.
4. Delete or retire duplicated mesh builders after equivalence checks pass.

The goal is to prove the more typical CAD flow:

```text
build123d/OpenCascade BREP solid -> STEP for Onshape/CAD -> STL as final slicer mesh
```

Run from `dynrep-launch-clean/`:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_step_first_test.py
```

Assembly-view STEP compounds are generated separately because BuildViz scene
transforms expect assembly-local frames, while several printable STEP parts are
stored in print pose:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_step_assembly_views.py --with-servos
```

Outputs are written to `cad_step_test/out/`:

- `step/*.step`: clean CAD/BREP exchange files.
- `stl/*.stl`: final tessellated slicer meshes generated from the BREP solids.
- `manifest.json`: dimensions, BREP face counts, STL triangle counts, and
  legacy-STL comparison when a baseline STL exists.
- `assembled_robot*_manifest.json`: composed robot-view STEP/STL diagnostics
  using BuildViz's assembly frames.
- `hexapod_step_first_test_bundle.zip`: a small bundle suitable for uploading
  to Onshape as a test.

## Current migration scope

Migrated in this sidecar:

- `chassis_top`
- `chassis_bottom`
- `coxa_link`
- `disc_horn`
- `femur_link`
- `foot_boot`
- `foot_boot_plus4`
- `servo_body`
- `servo_clamp_cap`
- `switch_holster`
- `tibia_knee_yoke`
- `tibia_tube_printable`
- `yaw_bearing_cap`
- `yaw_bearing_lower`
- `yaw_servo_retainer`
- `yaw_bearing_upper`

Still pending:

- None. The current production printables covered by this sidecar now have
  native BREP builders and derived STL exports.

A mesh-to-STEP wrapper would preserve the triangles and would not solve the
Onshape artifact issue; this sidecar uses native BREP solids before exporting
STEP.
