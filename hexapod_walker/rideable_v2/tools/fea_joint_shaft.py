#!/usr/bin/env python3
"""FEA of the rideable_v2 SINGLE-SHEAR hip/knee joint shaft (open-source
stack: gmsh mesh -> scikit-fem 3D linear elasticity -> pyvista render).

This is the node the design review flagged (DRIVETRAIN.md §9.7 /
STRUCTURE.md §6.5): the clevis plate sits on ONE side of the joint only
(the belt wrap and the parking lock own the pulley face), so the Ø25
4140 shaft cantilevers the femur hub off the coxa clevis and carries the
belt shaft pull in bending.

Model (mm / N / MPa; y = joint axis, matches the viz leg frame):

    shaft: Ø25, y in [-40, +75]                    (PIN_NEAR/FAR in the viz)
    fixed: outer surface y in [+45, +70]           (clevis-plate bearing band)
    load : uniform -z traction, y in [-40, 0]      (pulley-side half of the
           femur hub clamp band — the belt pull enters biased toward the
           pulley, which bolts to the hub's -y face)

Load cases:
    A. 5.0 kN  — the belt shaft-pull bound (pretension + working, hip peak)
    B. 6.19 kN — case A + the 1.19 kN design foot load, stacked colinear
                 (worst azimuth)

Hand-calc anchor: resultant at y=-20, support edge at y=+45 -> arm 65 mm,
M = 325 N.m, sigma = M/Z = 325e3/1534 = 212 MPa nominal; the review's
~360 MPa took the full pulley-plane arm (~110 mm), which is the upper
bound if the hub tilts rigidly.  The FEA resolves where between those
the real peak lands, plus the clamp-edge concentration.

Run:  .venv/bin/python tools/fea_joint_shaft.py   (from rideable_v2/)
Outputs: full_robot_viz/fea_joint_shaft.png + numbers on stdout.
"""
from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "full_robot_viz"

# --- geometry / material ----------------------------------------------------
R = 12.5                  # shaft radius, mm
Y0, Y1 = -40.0, 75.0      # shaft extent along the joint axis
FIX = (45.0, 70.0)        # clevis bearing band (fully constrained surface)
LOAD = (-40.0, 0.0)       # load-introduction band (pulley side of the hub)
E, NU = 205_000.0, 0.29   # 4140 HT steel, MPa
YIELD = 655.0             # 4140 @ 28-32 HRC, MPa
CASES = {"A: belt pull 5.0 kN": 5000.0,
         "B: belt + foot 6.19 kN": 6190.0}
LC = float(__import__("sys").argv[1]) if len(__import__("sys").argv) > 1 \
    else 2.5              # gmsh characteristic length, mm (CLI override)


def make_mesh() -> "skfem.Mesh":
    import gmsh
    from skfem.io.meshio import from_meshio
    import meshio

    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add("shaft")
    gmsh.model.occ.addCylinder(0, Y0, 0, 0, Y1 - Y0, 0, R)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", LC)
    gmsh.model.mesh.generate(3)
    with tempfile.NamedTemporaryFile(suffix=".msh", delete=False) as f:
        path = f.name
    gmsh.write(path)
    gmsh.finalize()
    m = meshio.read(path)
    Path(path).unlink()
    # keep only tets (drop surface tris gmsh also writes)
    tets = meshio.Mesh(m.points, [c for c in m.cells if c.type == "tetra"])
    return from_meshio(tets)


def main() -> None:
    from skfem import (Basis, ElementTetP2, ElementVector, FacetBasis,
                       asm, condense, solve)
    from skfem.helpers import ddot, sym_grad, trace
    from skfem import BilinearForm, LinearForm

    mesh = make_mesh()
    print(f"mesh: {mesh.p.shape[1]} nodes, {mesh.t.shape[1]} tets (P2 elements)")

    e = ElementVector(ElementTetP2())
    basis = Basis(mesh, e)

    lam = E * NU / ((1 + NU) * (1 - 2 * NU))
    mu = E / (2 * (1 + NU))

    @BilinearForm
    def stiffness(u, v, w):
        eps_u, eps_v = sym_grad(u), sym_grad(v)
        return 2 * mu * ddot(eps_u, eps_v) + lam * trace(eps_u) * trace(eps_v)

    K = asm(stiffness, basis)

    # Facet midpoints on the faceted cylinder sit up to lc^2/(8R) inside the
    # true radius — use a chord-sagitta-sized tolerance, not an epsilon.
    surf_tol = LC ** 2 / (8 * R) * 2 + 0.05

    # fixed: every dof on the outer surface inside the clevis bearing band
    fixed_facets = mesh.facets_satisfying(
        lambda x: (np.abs(np.sqrt(x[0] ** 2 + x[2] ** 2) - R) < surf_tol)
        & (x[1] >= FIX[0]) & (x[1] <= FIX[1]))
    fixed_dofs = basis.get_dofs(facets=fixed_facets).all()

    # load: uniform -z traction on the outer surface in the hub band
    load_facets = mesh.facets_satisfying(
        lambda x: (np.abs(np.sqrt(x[0] ** 2 + x[2] ** 2) - R) < surf_tol)
        & (x[1] >= LOAD[0]) & (x[1] <= LOAD[1]))
    if len(fixed_facets) == 0 or len(load_facets) == 0:
        raise RuntimeError(f"empty facet selection: fixed={len(fixed_facets)}"
                           f" load={len(load_facets)}")
    print(f"facets: {len(fixed_facets)} fixed, {len(load_facets)} loaded")
    fbasis = FacetBasis(mesh, e, facets=load_facets)
    band_area = 2 * np.pi * R * (LOAD[1] - LOAD[0])

    results = {}
    for label, force_N in CASES.items():
        traction = force_N / band_area  # N/mm^2, applied along -z

        @LinearForm
        def load(v, w):
            return -traction * v[2]

        f = asm(load, fbasis)
        u = solve(*condense(K, f, D=fixed_dofs))

        # stress recovery at quadrature points -> von Mises
        ug = basis.interpolate(u)
        eps = sym_grad(ug)
        tr = trace(eps)
        vm2 = 0.0
        s = [[2 * mu * eps[i, j] + (lam * tr if i == j else 0.0)
              for j in range(3)] for i in range(3)]
        vm = np.sqrt(0.5 * ((s[0][0] - s[1][1]) ** 2
                            + (s[1][1] - s[2][2]) ** 2
                            + (s[2][2] - s[0][0]) ** 2
                            + 6 * (s[0][1] ** 2 + s[1][2] ** 2 + s[0][2] ** 2)))
        vm = np.asarray(vm)
        # peak away from the artificial clamp-edge singularity: report both
        # the raw max and the 99.5th percentile (mesh-stable working number)
        qy = basis.global_coordinates()[1]
        vm_flat, qy_flat = vm.flatten(), np.asarray(qy).flatten()
        raw = float(vm_flat.max())
        p995 = float(np.percentile(vm_flat, 99.5))
        tip = np.abs(u[basis.get_dofs(
            facets=mesh.facets_satisfying(lambda x: x[1] < Y0 + 1e-3)
        ).all()]).max()
        at = float(qy_flat[vm_flat.argmax()])
        results[label] = (raw, p995, tip, at)
        print(f"\n{label}")
        print(f"  peak von Mises (raw / 99.5%): {raw:6.0f} / {p995:6.0f} MPa"
              f"   (peak at y={at:+.0f} mm)")
        print(f"  vs 4140 HT yield {YIELD:.0f} MPa -> SF "
              f"{YIELD / raw:.2f} raw / {YIELD / p995:.2f} at 99.5%")
        print(f"  free-end deflection: {tip:.3f} mm")

    # --- render case B stress field ------------------------------------------
    try:
        import pyvista as pv
        import meshio as mio

        # nodal von Mises via P1 projection (nearest quadrature value is fine
        # for a picture): use cell-averaged vm on P1 mesh points
        grid = pv.make_tri_grid = None  # noqa - keep pyvista import obvious
        cells = np.hstack([np.full((mesh.t.shape[1], 1), 4, dtype=np.int64),
                           mesh.t.T[:, :4]]).ravel()
        ug = pv.UnstructuredGrid(
            cells, np.full(mesh.t.shape[1], 10, dtype=np.uint8),
            mesh.p.T.astype(float))
        ug.cell_data["von Mises (MPa)"] = vm.mean(axis=1)[:mesh.t.shape[1]] \
            if vm.ndim == 2 else np.resize(vm_flat, mesh.t.shape[1])
        pl = pv.Plotter(off_screen=True, window_size=(1200, 700))
        pl.add_mesh(ug, scalars="von Mises (MPa)", cmap="turbo",
                    show_edges=False, clim=(0, min(raw, YIELD)))
        pl.add_text("rideable_v2 joint shaft - case B (6.19 kN, single shear)\n"
                    "fixed: clevis band y=45..70; load: hub band y=-40..0",
                    font_size=11)
        pl.view_vector((1, -0.6, 0.4))
        OUT.mkdir(exist_ok=True)
        pl.screenshot(str(OUT / "fea_joint_shaft.png"))
        print(f"\nwrote {OUT / 'fea_joint_shaft.png'}")
    except Exception as exc:  # rendering is best-effort
        print(f"\n(render skipped: {exc})")


if __name__ == "__main__":
    main()
