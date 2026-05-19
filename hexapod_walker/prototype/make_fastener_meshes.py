"""Regenerate the fastener cache STLs in ``fasteners/``.

For each unique part number in ``fastener_registry.build_all_fastener_instances()``
this script picks a source mesh by priority:

    1. ``fasteners/<part_number>.step`` (real McMaster-Carr STEP file).
    2. ``fasteners/<part_number>.stl``  (user-supplied STL).
    3. ``fasteners/_parametric.build_for_spec(<spec>)`` -- the
       parametric fallback (visualization only).

The chosen source is converted to a single Trimesh and written out as
``fasteners/<part_number>.cache.stl``.  The build inspector and the
verifier read ONLY the cache STL at runtime, so a parametric fallback
today silently becomes a real STEP later when the user drops one in
and re-runs ``make regen-fasteners``.

Run with ``python make_fastener_meshes.py`` from the prototype
directory (or via ``make regen-fasteners``).
"""

from __future__ import annotations

import os
import sys
import traceback
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
FASTENERS = HERE / "fasteners"

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from fastener_registry import build_all_fastener_instances  # noqa: E402
from fasteners import _parametric  # noqa: E402


def _try_load_step(path: Path) -> trimesh.Trimesh | None:
    try:
        loaded = trimesh.load(path, force="mesh")
    except Exception as exc:  # noqa: BLE001
        print(f"  STEP load failed for {path.name}: {exc}", file=sys.stderr)
        return None
    if loaded is None or not hasattr(loaded, "vertices"):
        return None
    if isinstance(loaded, trimesh.Scene):
        if not loaded.geometry:
            return None
        loaded = trimesh.util.concatenate(list(loaded.geometry.values()))
    return loaded


def _try_load_stl(path: Path) -> trimesh.Trimesh | None:
    try:
        return trimesh.load(path, force="mesh")
    except Exception as exc:  # noqa: BLE001
        print(f"  STL load failed for {path.name}: {exc}", file=sys.stderr)
        return None


def regenerate() -> dict:
    """Return a report dict ``{part_number: source}`` recording which
    source produced each cache."""
    FASTENERS.mkdir(parents=True, exist_ok=True)
    seen: dict[str, str] = {}  # part_number -> spec
    for fi in build_all_fastener_instances():
        if fi.part_number in seen:
            continue
        seen[fi.part_number] = fi.spec

    report: dict[str, str] = {}
    for pn, spec in sorted(seen.items()):
        step_path = FASTENERS / f"{pn}.step"
        stl_path  = FASTENERS / f"{pn}.stl"
        cache_path = FASTENERS / f"{pn}.cache.stl"
        mesh = None
        source = "parametric"
        if step_path.exists():
            mesh = _try_load_step(step_path)
            if mesh is not None:
                source = "STEP"
        if mesh is None and stl_path.exists():
            mesh = _try_load_stl(stl_path)
            if mesh is not None:
                source = "user STL"
        if mesh is None:
            try:
                mesh = _parametric.build_for_spec(spec)
            except Exception:  # noqa: BLE001
                traceback.print_exc()
                continue
        if mesh is None:
            print(f"  ERROR: no source for {pn} ({spec})", file=sys.stderr)
            continue
        mesh.export(cache_path)
        report[pn] = source
        print(f"  cache {pn:12s} <- {source:10s}  ({spec})")
    return report


if __name__ == "__main__":
    print(f"Regenerating fastener cache in {FASTENERS}/ ...")
    rep = regenerate()
    print()
    print("Summary:")
    for pn, source in sorted(rep.items()):
        print(f"  {pn:12s}  {source}")
