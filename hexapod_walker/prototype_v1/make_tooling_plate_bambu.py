#!/usr/bin/env python3
"""Lay the hexagonal tooling plate out on a Bambu X1 / X1 Carbon plate.

The tooling plate is a standalone bench part (not one of the chassis trays),
so this wraps make_tooling_plate.make_plate() into a single-object Bambu
plate: it drops the part flat on the bed, centres it, and writes both an
STL and a multi-object .3mf (preferred slicer import) plus a short README.

Run::

    python hexapod_walker/prototype_v1/make_tooling_plate_bambu.py
    python hexapod_walker/prototype_v1/make_tooling_plate_bambu.py --buck
"""

from __future__ import annotations

import argparse
import html
import os
import sys
import zipfile

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, os.path.join(THIS_DIR, "arm"))

import make_tooling_plate as MTP  # noqa: E402
from bambu_arm_common import _CT_XML, _RELS_XML  # noqa: E402  (reuse 3MF parts)

# Bambu Lab X1 / X1 Carbon: 256 mm cube bed.
BED_MM = 256.0


def _drop_to_bed(mesh):
    """Centre XY at origin and translate so z_min = 0."""
    lo, hi = mesh.bounds
    cx = (lo[0] + hi[0]) / 2.0
    cy = (lo[1] + hi[1]) / 2.0
    mesh.apply_translation([-cx, -cy, -lo[2]])
    return mesh


def _write_3mf(mesh, path, name):
    """Single-object 3MF placed at bed centre (front-left-corner origin)."""
    verts = mesh.vertices
    faces = mesh.faces
    vert_xml = "".join(
        f'<vertex x="{float(v[0]):.4f}" y="{float(v[1]):.4f}" z="{float(v[2]):.4f}"/>'
        for v in verts
    )
    tri_xml = "".join(
        f'<triangle v1="{int(t[0])}" v2="{int(t[1])}" v3="{int(t[2])}"/>'
        for t in faces
    )
    object_xml = (
        f'<object id="1" type="model" name="{html.escape(name)}">'
        f'<mesh><vertices>{vert_xml}</vertices>'
        f'<triangles>{tri_xml}</triangles></mesh></object>'
    )
    tx = ty = BED_MM / 2.0
    build_xml = f'<item objectid="1" transform="1 0 0 0 1 0 0 0 1 {tx:.4f} {ty:.4f} 0"/>'
    model_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        f'<resources>{object_xml}</resources>'
        f'<build>{build_xml}</build>'
        '</model>'
    )
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", _CT_XML)
        zf.writestr("_rels/.rels", _RELS_XML)
        zf.writestr("3D/3dmodel.model", model_xml)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--buck", action="store_true",
                    help="lay out the buck-converter tooling plate instead")
    ap.add_argument("--out-dir", default=os.path.join(THIS_DIR, "bambu_tooling_plate"))
    args = ap.parse_args(argv)

    mesh, info = MTP.make_plate(with_buck=args.buck)
    mesh = _drop_to_bed(mesh)

    ext = mesh.extents
    if ext[0] > BED_MM or ext[1] > BED_MM:
        raise SystemExit(f"Part {ext[0]:.0f} x {ext[1]:.0f} mm exceeds the "
                         f"{BED_MM:.0f} mm X1 bed.")

    os.makedirs(args.out_dir, exist_ok=True)
    stem = "tooling_plate_buck" if args.buck else "tooling_plate"
    stl_path = os.path.join(args.out_dir, f"{stem}.stl")
    threemf_path = os.path.join(args.out_dir, f"{stem}.3mf")
    mesh.export(stl_path)
    _write_3mf(mesh, threemf_path, f"{stem}.stl")

    readme = os.path.join(args.out_dir, "README.md")
    with open(readme, "w") as f:
        f.write(
            f"# Tooling plate -- Bambu X1 / X1 Carbon plate\n\n"
            f"Bed: **{BED_MM:.0f} x {BED_MM:.0f} x {BED_MM:.0f} mm**.  "
            f"Single part, dropped flat on the bed and centred.\n\n"
            f"- `{stem}.3mf` -- preferred import (opens already positioned)\n"
            f"- `{stem}.stl` -- same geometry, plain STL\n\n"
            f"## Geometry\n"
            f"- {info['flat_to_flat']:.0f} mm flat-to-flat hex "
            f"(corners {info['circum']:.1f} mm) x {info['plate_t']:.1f} mm\n"
            f"- footprint {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.1f} mm\n"
            f"- {info['n_holes']} M3 grid holes, {info['n_slots']} through slots\n"
            f"- {len(info['mount_xy'])} standoff columns ({info['mount']})\n\n"
            f"## Print notes\n"
            f"- PLA or PETG, 0.2 mm layers, 3+ walls, 30-40% infill.\n"
            f"- No supports (flat plate, all cuts vertical).\n"
        )

    print(f"Wrote Bambu X1 tooling plate ({info['mount']}, "
          f"{len(info['mount_xy'])} standoff holes):")
    print(f"  {threemf_path}")
    print(f"  {stl_path}")
    print(f"  footprint {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.1f} mm  "
          f"(watertight={mesh.is_watertight})")


if __name__ == "__main__":
    main()
