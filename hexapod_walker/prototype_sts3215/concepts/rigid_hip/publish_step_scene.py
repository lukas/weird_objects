#!/usr/bin/env python3
"""Assemble + publish the ``sts3215-rigid-hip-step`` BuildViz build.

Takes the concept's assembled ``scene.json`` (transforms, joints, poses)
and substitutes the BREP-tessellated STLs from the STEP-first sidecar
(``concepts/rigid_hip/step/stl/``, written by
``cad_step_test/build_rigid_hip_step.py``) for the seven variant
printables.  Every other mesh (femur, servo bodies, tibia tube, COTS
visuals) keeps the mesh-pipeline geometry from ``concepts/rigid_hip/stl/``.

Pushes to the LOCAL hub only (http://127.0.0.1:5183), overwriting the
``main`` version in place (--no-snapshot: no version pile-up).  Mirror to
the cloud hub separately with
``tools/push_cloud_buildviz.py --build-id sts3215-rigid-hip-step``.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import trimesh

CONCEPT_DIR = Path(__file__).resolve().parent
MESH_STL_DIR = CONCEPT_DIR / "stl"          # mesh-pipeline print set
BREP_STL_DIR = CONCEPT_DIR / "step" / "stl"  # BREP-derived (STEP sidecar)
BUILD_ID = "sts3215-rigid-hip-step"

# Scene meshes swapped to their BREP-derived twins (the seven variant
# printables the STEP sidecar exports).
STEP_PARTS = {
    "hip_clamp_cap_rigid.stl",
    "chassis_top_rigid.stl",
    "top_hatch_rigid.stl",
    "corner_pillar.stl",
    "centre_wago_block.stl",
    "coxa_link_rigid.stl",
    "chassis_bottom_rigid.stl",
}


def stage_scene(stage: Path) -> dict:
    scene = json.loads((CONCEPT_DIR / "scene.json").read_text())
    (stage / "stl").mkdir(parents=True)
    swapped, kept = [], []
    for mesh in scene["meshes"]:
        rel = mesh["url"]                      # e.g. stl/foot_boot.stl
        name = Path(rel).name
        dst = stage / rel
        if name in STEP_PARTS:
            src = BREP_STL_DIR / name
            # sanity: the BREP twin must agree with the print mesh closely
            a = trimesh.load(MESH_STL_DIR / name)
            b = trimesh.load(src)
            delta = float(np.abs(a.extents - b.extents).max())
            assert delta < 0.6, f"{name}: bbox delta {delta:.3f} mm vs mesh"
            swapped.append(f"{name} (bbox d={delta:.3f})")
        else:
            src = CONCEPT_DIR / rel
            kept.append(name)
        shutil.copy(src, dst)

    scene["name"] = "STS3215 rigid-hip (STEP/BREP pipeline, assembled)"
    scene["source"] = (
        "concepts/rigid_hip/scene.json transforms; printable meshes "
        "tessellated from cad_step_test build123d BREP where a STEP twin "
        "exists (femur/servo_body/tibia tube + COTS visuals keep the "
        "mesh-pipeline geometry)"
    )
    (stage / "scene.json").write_text(json.dumps(scene, indent=1))
    shutil.copy(CONCEPT_DIR / "design_spec.yaml", stage / "design_spec.yaml")
    print("swapped to BREP-derived STLs:", *swapped, sep="\n  ")
    print("kept mesh-pipeline geometry:", ", ".join(kept))
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-m", "--message", default=(
        "assembled from concepts/rigid_hip/step/stl BREP-derived meshes"))
    args = parser.parse_args()

    missing = [n for n in STEP_PARTS if not (BREP_STL_DIR / n).exists()]
    if missing:
        raise SystemExit(
            f"missing BREP STLs in {BREP_STL_DIR}: {sorted(missing)} "
            "-- run cad_step_test/build_rigid_hip_step.py first")

    with tempfile.TemporaryDirectory(prefix="rigid_hip_step_scene_") as tmp:
        stage = Path(tmp)
        stage_scene(stage)
        subprocess.run(
            ["npx", "buildviz", "push", "--build-id", BUILD_ID,
             "--scene", str(stage / "scene.json"),
             "--upload-assets", "--no-snapshot", "-m", args.message],
            check=True,
        )
    print(f"http://127.0.0.1:5183/?build={BUILD_ID}")


if __name__ == "__main__":
    main()
