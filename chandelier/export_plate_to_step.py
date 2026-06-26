"""Convert the packed plate STL (multi-body mesh) to STEP.

Uses OpenCascade via OCP (install ``cadquery`` in the repo venv: ``pip install cadquery``).

Native STL contains disconnected solids as one triangle soup; OpenCascade's STL reader
fails on the merged file, so this script splits the mesh into connected components with
trimesh, imports each shell separately, and unions them into a compound STEP shape.

STEP expands tessellated geometry enormously (often hundreds of MB per hundred thousand
triangles).  By default each shell is decimated with PyVista (``trimesh`` optional deps)
before STEP export.  Pass ``--decimate 0`` for full-resolution STEP (very slow and huge).

Examples::

    ./run.sh chandelier/export_plate_to_step.py
    ./run.sh chandelier/export_plate_to_step.py --decimate 0 --stl plate.stl --out plate.step
"""

from __future__ import annotations

import argparse
import os
import tempfile

import trimesh


def _require_ocp():
    try:
        from OCP.BRep import BRep_Builder  # noqa: F401
        from OCP.STEPControl import STEPControl_AsIs  # noqa: F401
        from OCP.StlAPI import StlAPI_Reader  # noqa: F401
        from OCP.TopoDS import TopoDS_Compound  # noqa: F401
    except ImportError as e:
        raise SystemExit(
            "Missing OCP / OpenCascade Python bindings. Install with:\n"
            "  pip install cadquery\n"
            f"Import error: {e}"
        ) from e


def _decimate_part(mesh: trimesh.Trimesh, reduction: float) -> trimesh.Trimesh:
    """Return mesh decimated by keeping ``1 - reduction`` fraction of triangles."""
    import pyvista as pv

    pv_mesh = pv.wrap(mesh)
    decimated = pv_mesh.decimate(reduction)
    faces_flat = decimated.faces
    if faces_flat is None or len(faces_flat) == 0:
        return mesh
    tri_faces = faces_flat.reshape(-1, 4)[:, 1:4]
    return trimesh.Trimesh(decimated.points.copy(), tri_faces.copy(), process=False)


def trimeshes_to_step(parts, step_path: str, *, decimate: float,
                      verbose: bool = True) -> int:
    """Convert one or more trimesh bodies into a single STEP compound.

    Each body is tessellated (optionally decimated) and read into
    OpenCascade via its STL reader, then added to a STEP compound so the
    output preserves every body as its own shell.  Returns the size of the
    written STEP file in bytes.

    Note: STEP produced this way is tessellated (triangle shells), not
    analytic B-rep surfaces — it round-trips the mesh faithfully but does
    not recover parametric faces.
    """
    _require_ocp()
    from OCP.BRep import BRep_Builder
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    from OCP.StlAPI import StlAPI_Reader
    from OCP.TopoDS import TopoDS_Compound, TopoDS_Shape

    parts = [p for p in parts if p is not None and len(p.faces) > 0]
    if not parts:
        raise SystemExit("No triangle geometry to export.")

    reader = StlAPI_Reader()
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)

    def _try_add(mesh, seg_path) -> bool:
        """Export ``mesh`` to ``seg_path`` and add it to the compound.
        Returns True on success.  OpenCascade's STL reader raises (rather
        than returning False) on degenerate or non-manifold shells, so the
        failure is caught here and reported by the caller.

        Zero-area facets (common after boolean slot/pocket cuts) make
        OpenCascade's reader abort because it cannot compute a facet
        normal, so they are stripped first.  This is harmless for a
        tessellated STEP — the dropped triangles carry no surface."""
        mesh = mesh.copy()
        mesh.update_faces(mesh.nondegenerate_faces(height=1e-6))
        mesh.remove_unreferenced_vertices()
        if len(mesh.faces) < 4:
            return False
        mesh.export(seg_path)
        shape = TopoDS_Shape()
        try:
            if not reader.Read(shape, seg_path):
                return False
        except Exception:
            return False
        builder.Add(compound, shape)
        return True

    n_added = 0
    tmp_dir = tempfile.mkdtemp(prefix="mesh_step_")
    try:
        for idx, part in enumerate(parts):
            if decimate > 0.0:
                if decimate >= 1.0:
                    raise SystemExit("--decimate must be < 1 when nonzero.")
                part = _decimate_part(part, decimate)

            seg_path = os.path.join(tmp_dir, f"body_{idx:03d}.stl")
            if _try_add(part, seg_path):
                n_added += 1
                if verbose:
                    print(f"  body {idx + 1}/{len(parts)}  triangles={len(part.faces):>7}")
                continue

            # OpenCascade rejected the body whole — usually a non-manifold
            # junction or a stray sliver shell from the boolean ops.  Retry
            # per connected component and drop only the degenerate shards.
            subparts = [sp for sp in part.split(only_watertight=False)
                        if len(sp.faces) >= 4]
            sub_added = 0
            for sidx, sp in enumerate(subparts):
                sp_path = os.path.join(tmp_dir, f"body_{idx:03d}_{sidx:03d}.stl")
                if _try_add(sp, sp_path):
                    sub_added += 1
            if sub_added:
                n_added += 1
                if verbose:
                    print(f"  body {idx + 1}/{len(parts)}  "
                          f"recovered {sub_added}/{len(subparts)} shells")
            else:
                print(f"  WARNING: body {idx + 1}/{len(parts)} could not be "
                      f"read by OpenCascade; skipped")
    finally:
        for name in os.listdir(tmp_dir):
            try:
                os.remove(os.path.join(tmp_dir, name))
            except OSError:
                pass
        try:
            os.rmdir(tmp_dir)
        except OSError:
            pass

    if n_added == 0:
        raise RuntimeError("No bodies could be read by OpenCascade.")

    writer = STEPControl_Writer()
    stat = writer.Transfer(compound, STEPControl_AsIs)
    if stat != IFSelect_ReturnStatus.IFSelect_RetDone:
        raise RuntimeError(f"STEP transfer failed with status {stat}")

    writer.Write(step_path)
    return os.path.getsize(step_path)


def stl_plate_to_step(stl_path: str, step_path: str, *, decimate: float) -> None:
    mesh = trimesh.load(stl_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise SystemExit(f"Expected single Trimesh, got {type(mesh)!r}")

    parts = mesh.split()
    if not parts:
        raise SystemExit("STL contained no triangle geometry.")

    size_bytes = trimeshes_to_step(parts, step_path, decimate=decimate)
    print(f"\nWrote {step_path} ({size_bytes / 1e6:.1f} MB)")


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    default_stl = os.path.join(here, "bambu_h2d_plates", "plate_01_quarter_scale_polyhedra.stl")
    default_step = os.path.splitext(default_stl)[0] + ".step"

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl", default=default_stl,
                    help=f"path to plate STL (default: {default_stl})")
    ap.add_argument("--out", default=default_step,
                    help=f"output STEP path (default: {default_step})")
    ap.add_argument("--decimate", type=float, default=0.95,
                    help="PyVista decimation reduction in (0,1); 0 disables decimation "
                         "(default: 0.95)")
    args = ap.parse_args()

    if args.decimate < 0 or args.decimate >= 1:
        raise SystemExit("--decimate must be 0 or in the interval (0, 1)")

    if not os.path.isfile(args.stl):
        raise SystemExit(f"STL not found: {args.stl}")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)

    print(f"Input : {args.stl}")
    print(f"Output: {args.out}")
    if args.decimate > 0:
        print(f"Decimate: keep ~{(1 - args.decimate) * 100:.1f}% of triangles per shell")
    else:
        print("Decimate: OFF (full triangle count — STEP may be multi‑GB)")

    stl_plate_to_step(args.stl, args.out, decimate=args.decimate)


if __name__ == "__main__":
    main()
