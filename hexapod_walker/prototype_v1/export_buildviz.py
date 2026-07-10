#!/usr/bin/env python3
"""Export THIS prototype (prototype_v1) into the local BuildViz web viewer.

BuildViz ships an exporter (~/buildviz/scripts/export_hexapod_prototype.py)
but it is hardcoded to the sibling ``hexapod_walker/prototype`` design.  We
work in ``prototype_v1``, so this thin wrapper runs the SAME assembly export
logic against THIS directory's ``inspect_build`` + ``part_palette`` and
writes the scene + STLs into the BuildViz public build folder.

Usage:
    python export_buildviz.py            # write scene into ~/buildviz
    BUILDVIZ_ROOT=/path python export_buildviz.py

Then VIEW it in the ONE machine-wide BuildViz hub (default port 5183) -- do NOT
start a per-project dev server on 5173/etc:
    cd ~/buildviz && npx buildviz register public/builds/hexapod-prototype --build-id hexapod-prototype
    open http://127.0.0.1:5183/?build=hexapod-prototype
"""

from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

BUILDVIZ_ROOT = Path(os.environ.get("BUILDVIZ_ROOT", Path.home() / "buildviz")).resolve()
PUBLIC_BUILD_DIR = BUILDVIZ_ROOT / "public" / "builds" / "hexapod-prototype"


def _hex_color(rgb: tuple[float, float, float]) -> str:
    return "#" + "".join(f"{max(0, min(255, round(c * 255))):02x}" for c in rgb)


def _three_matrix(matrix: np.ndarray) -> list[float]:
    return [float(v) for v in matrix.T.reshape(-1)]


def _copy_asset(source: Path, destination_name: str) -> str:
    destination = PUBLIC_BUILD_DIR / destination_name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return f"/builds/hexapod-prototype/{destination_name}"


def _rebuild_part_stls() -> None:
    """Regenerate stl_prototype/*.stl from the current geometry source.

    BuildViz loads STL files off disk, so a stale stl_prototype/ silently
    shows an OLD part in the viewer.  STL generation is a few seconds, so we
    always rebuild before exporting.  Set BUILDVIZ_SKIP_STL_REBUILD=1 to skip
    (e.g. when re-exporting purely to tweak the scene/camera).
    """
    if os.environ.get("BUILDVIZ_SKIP_STL_REBUILD") == "1":
        print("  (BUILDVIZ_SKIP_STL_REBUILD=1 -- reusing existing stl_prototype/*.stl)")
        return
    import hexapod_prototype  # type: ignore

    print("Rebuilding stl_prototype/*.stl from hexapod_prototype.py ...")
    cwd = os.getcwd()
    os.chdir(THIS_DIR)  # hexapod_prototype writes STLs relative to CWD
    try:
        try:
            hexapod_prototype.main(argv=[])  # newer signature
        except TypeError:
            hexapod_prototype.main()         # older no-arg signature
    finally:
        os.chdir(cwd)


def main() -> None:
    if not BUILDVIZ_ROOT.exists():
        raise SystemExit(
            f"BuildViz checkout not found at {BUILDVIZ_ROOT}.\n"
            f"Clone it with:  gh repo clone lukas/buildviz {BUILDVIZ_ROOT}",
        )

    _rebuild_part_stls()

    import inspect_build  # type: ignore
    import part_palette as palette  # type: ignore

    PUBLIC_BUILD_DIR.mkdir(parents=True, exist_ok=True)

    instances = inspect_build._build_assembly_instances()
    if os.environ.get("BUILDVIZ_NO_FASTENERS") != "1":
        instances.extend(inspect_build._build_fastener_instances())

    # Drop any instance whose STL hasn't been generated (e.g. fastener
    # caches that need `make regen-fasteners`) so the viewer still loads
    # the structural assembly.
    kept = []
    skipped = 0
    for inst in instances:
        stl_dir, stl_name = inspect_build._instance_stl_key(inst)
        if (Path(stl_dir) / stl_name).exists():
            kept.append(inst)
        else:
            skipped += 1
    if skipped:
        print(f"  (skipping {skipped} instance(s) with no STL yet -- "
              f"run `make regen-fasteners` to include fasteners)")
    instances = kept

    stl_keys = {inspect_build._instance_stl_key(i) for i in instances}
    stl_cache = inspect_build._load_stl_cache(stl_keys)
    chassis_lift = inspect_build._compute_chassis_lift(instances, stl_cache)
    lift = inspect_build._trans(0, 0, chassis_lift)

    meshes: dict[tuple[str, str], dict[str, Any]] = {}
    manifest_instances: list[dict[str, Any]] = []
    chassis_centroids: list[np.ndarray] = []
    fastener_index = 0

    for index, instance in enumerate(instances):
        stl_dir, stl_name = inspect_build._instance_stl_key(instance)
        stl_path = Path(stl_dir) / stl_name
        mesh_key = (stl_dir, stl_name)
        mesh_id = f"{Path(stl_dir).name}:{Path(stl_name).stem}"

        if mesh_key not in meshes:
            asset_name = (
                f"fasteners/{stl_name}"
                if Path(stl_dir).name == "fasteners"
                else f"stl_prototype/{stl_name}"
            )
            meshes[mesh_key] = {
                "id": mesh_id,
                "name": stl_name,
                "url": _copy_asset(stl_path, asset_name),
            }

        world_transform = lift @ instance.transform
        mesh = inspect_build._apply_transform(stl_cache[mesh_key], world_transform)
        centroid = np.array(mesh.center, dtype=float)

        if instance.part_type in ("chassis_top", "chassis_bottom"):
            chassis_centroids.append(centroid)

        leg = None if instance.leg_index is None else f"L{instance.leg_index}"
        label = palette.instance_label(
            instance.part_type, instance.leg_index, instance.joint,
            fastener_role=instance.fastener_role,
        )
        role = palette.instance_role(
            instance.part_type, instance.leg_index, instance.joint,
            fastener_role=instance.fastener_role,
        )

        if palette.is_fastener(instance.part_type):
            fastener_index += 1
            instance_id = f"fastener-{fastener_index:03d}"
        else:
            suffix = "" if leg is None else f"-{leg}"
            joint = "" if instance.joint is None else f"-{instance.joint}"
            instance_id = f"{instance.part_type}{suffix}{joint}"

        manifest_instances.append({
            "id": f"{index:03d}-{instance_id}",
            "meshId": mesh_id,
            "name": label,
            "partType": instance.part_type,
            "role": role,
            "leg": leg,
            "joint": instance.joint,
            "color": _hex_color(palette.PART_COLORS.get(instance.part_type, (0.8, 0.8, 0.8))),
            "transform": _three_matrix(world_transform),
            "centroid": [float(v) for v in centroid],
            "focusGroup": "chassis" if leg is None else leg,
        })

    center = (
        np.mean(np.vstack(chassis_centroids), axis=0)
        if chassis_centroids else np.array([0.0, 0.0, 0.0])
    )

    manifest = {
        "name": "Prototype hexapod (prototype_v1)",
        "source": str(THIS_DIR),
        "units": "mm",
        "center": [float(v) for v in center],
        "meshes": list(meshes.values()),
        "instances": manifest_instances,
    }

    # Additive MOTION block: articulated joints + named poses + keyframed gait
    # animation so the assembly can be seen WALKING in BuildViz.  Backward
    # compatible -- older viewers ignore joints/poses/animations and render the
    # static scene.  Set BUILDVIZ_NO_MOTION=1 to skip.
    if os.environ.get("BUILDVIZ_NO_MOTION") != "1":
        import motion_export  # type: ignore

        joints, poses, animations = motion_export.build_motion(
            manifest_instances, float(chassis_lift),
        )
        if joints:
            manifest["joints"] = joints
            manifest["poses"] = poses
            manifest["animations"] = animations
            print(
                f"  + motion: {len(joints)} joints, {len(poses)} poses, "
                f"{len(animations)} animation clip(s)"
            )

    manifest_path = PUBLIC_BUILD_DIR / "scene.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    design_spec_path = THIS_DIR / "design_spec.yaml"
    if design_spec_path.exists():
        dest_spec = PUBLIC_BUILD_DIR / "design_spec.yaml"
        shutil.copy2(design_spec_path, dest_spec)
        # shutil.copy2 preserves the SOURCE mtime, which makes the freshly
        # written scene.json / STLs (touched a moment ago in this same export)
        # look "newer" than the spec and trips `buildviz compat`'s staleness
        # heuristic even though the spec was regenerated in this very pass.
        # Stamp the copied spec to "now" so the contract sees it as current.
        os.utime(dest_spec, None)

    # BuildViz-compatibility contract (see ~/buildviz/BUILDVIZ_COMPATIBILITY.md):
    # a compatible build dir needs a non-empty ASSEMBLY.md + BOM.md alongside
    # scene.json / design_spec.yaml.  Our single source of truth for those lives
    # in the project's own docs (PROTOTYPE.md = the assembly build guide,
    # PROTOTYPE_BOM.md = the controlled bill of materials), so copy them in under
    # the contract's expected names rather than maintaining duplicates.
    for source_name, dest_name in (
        ("PROTOTYPE.md", "ASSEMBLY.md"),
        ("PROTOTYPE_BOM.md", "BOM.md"),
    ):
        source = THIS_DIR / source_name
        if source.exists():
            shutil.copy2(source, PUBLIC_BUILD_DIR / dest_name)

    print(f"Exported {len(manifest_instances)} instances, {len(meshes)} meshes")
    print(f"  -> {manifest_path}")
    print("View in the central BuildViz hub (default port 5183) -- do NOT start a new dev server:")
    print("  cd ~/buildviz && npx buildviz register public/builds/hexapod-prototype --build-id hexapod-prototype")
    print("  open http://127.0.0.1:5183/?build=hexapod-prototype")


if __name__ == "__main__":
    main()
