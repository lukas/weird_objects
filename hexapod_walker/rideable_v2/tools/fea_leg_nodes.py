#!/usr/bin/env python3
"""FEA of the two remaining flagged leg nodes (STRUCTURE.md §6.4):

  1. FEMUR HUB (6061-T6): the machined hip-end hub — Ø90 disc that carries
     the 100T pulley bolt circle and clamps the joint shaft, the Ø56 collar,
     and the transition into the 60x40x3 box beam.  Loads: pulley-face belt
     pull (5 kN) + drive torque (418 N.m peak), bore reaction (belt + foot),
     beam cut fixed 220 mm out.  The belt pull on the pulley face and its
     reaction on the bore nearly cancel (a ~255 N.m couple from the pulley
     plane offset), and the fixed-end moment from the foot shear reproduces
     the real femur moment diagram (~270 N.m at the cut) — the local model
     is consistent with README §2's load basis.

  2. COXA CLEVIS NODE (6061-T6): the single-shear clevis boss (40 mm) with
     the Ø52 bearing bore + the 25 mm plate wing back to the crossbar.
     The single-shear joint moment (~526 N.m at the clevis center) becomes
     a force COUPLE across the 30205 pair: with the pair BACK-TO-BACK
     (O-arrangement, effective spread ~48 mm incl. the tapered-roller
     pressure-center offset) the bands see ~+14.1 / -7.9 kN.  Face-to-face
     would shrink the effective spread toward zero and blow the bearings —
     which is why PARTS.md §3 specifies back-to-back.

Same open-source stack as fea_joint_shaft.py: gmsh OCC solids ->
scikit-fem P2 tets -> pyvista render.  mm / N / MPa.

Run:  .venv/bin/python tools/fea_leg_nodes.py [lc_mm]
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "full_robot_viz"

LC = float(sys.argv[1]) if len(sys.argv) > 1 else 3.5
E_ALU, NU_ALU, YIELD_ALU = 68_900.0, 0.33, 276.0   # 6061-T6


# --------------------------------------------------------------------------
# meshing helpers
# --------------------------------------------------------------------------
def occ_mesh(build) -> "skfem.Mesh":
    """Run `build(occ)` inside a fresh gmsh session, return a skfem tet mesh."""
    import gmsh
    import meshio
    from skfem.io.meshio import from_meshio

    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add("part")
    build(gmsh.model.occ)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", LC)
    gmsh.model.mesh.generate(3)
    with tempfile.NamedTemporaryFile(suffix=".msh", delete=False) as f:
        path = f.name
    gmsh.write(path)
    gmsh.finalize()
    m = meshio.read(path)
    Path(path).unlink()
    return from_meshio(
        meshio.Mesh(m.points, [c for c in m.cells if c.type == "tetra"]))


def solve_case(mesh, fixed_sel, loads, label, png=None, view=(1, -0.7, 0.5)):
    """loads = [(facet_selector, traction_fn(x)->(3,n) array)] in N/mm^2."""
    from skfem import (Basis, BilinearForm, ElementTetP2, ElementVector,
                       FacetBasis, LinearForm, asm, condense, solve)
    from skfem.helpers import ddot, sym_grad, trace

    e = ElementVector(ElementTetP2())
    basis = Basis(mesh, e)
    lam = E_ALU * NU_ALU / ((1 + NU_ALU) * (1 - 2 * NU_ALU))
    mu = E_ALU / (2 * (1 + NU_ALU))

    @BilinearForm
    def stiffness(u, v, w):
        eu, ev = sym_grad(u), sym_grad(v)
        return 2 * mu * ddot(eu, ev) + lam * trace(eu) * trace(ev)

    K = asm(stiffness, basis)
    fixed = basis.get_dofs(facets=mesh.facets_satisfying(fixed_sel)).all()

    f = 0
    resultant = np.zeros(3)
    for sel, traction in loads:
        facets = mesh.facets_satisfying(sel)
        if len(facets) == 0:
            raise RuntimeError(f"{label}: empty facet selection")
        fb = FacetBasis(mesh, e, facets=facets)

        @LinearForm
        def load(v, w, tr=traction):
            t = tr(w.x)
            return t[0] * v[0] + t[1] * v[1] + t[2] * v[2]

        fi = asm(load, fb)
        f = f + fi
        # resultant bookkeeping (sum of nodal forces per component)
        resultant += [fi[basis.nodal_dofs[c]].sum()
                      + (fi[basis.edge_dofs[c]].sum()
                         if hasattr(basis, "edge_dofs") else 0.0)
                      for c in range(3)]

    u = solve(*condense(K, f, D=fixed))

    ug = basis.interpolate(u)
    eps = sym_grad(ug)
    tr = trace(eps)
    s = [[2 * mu * eps[i, j] + (lam * tr if i == j else 0.0)
          for j in range(3)] for i in range(3)]
    vm = np.asarray(np.sqrt(0.5 * ((s[0][0] - s[1][1]) ** 2
                                   + (s[1][1] - s[2][2]) ** 2
                                   + (s[2][2] - s[0][0]) ** 2
                                   + 6 * (s[0][1] ** 2 + s[1][2] ** 2
                                          + s[0][2] ** 2))))
    flat = vm.flatten()
    raw, p995 = float(flat.max()), float(np.percentile(flat, 99.5))
    umax = float(np.abs(u[:basis.N]).max())
    print(f"\n{label}")
    print(f"  mesh: {mesh.t.shape[1]} tets; load resultant "
          f"~({resultant[0]:.0f}, {resultant[1]:.0f}, {resultant[2]:.0f}) N")
    print(f"  von Mises raw / 99.5%: {raw:6.0f} / {p995:6.0f} MPa"
          f"  -> SF vs 6061-T6 {YIELD_ALU:.0f}: "
          f"{YIELD_ALU / raw:.2f} raw / {YIELD_ALU / p995:.2f} at 99.5%")
    print(f"  max displacement: {umax:.3f} mm")

    if png:
        try:
            import pyvista as pv
            cells = np.hstack([np.full((mesh.t.shape[1], 1), 4, np.int64),
                               mesh.t.T[:, :4]]).ravel()
            grid = pv.UnstructuredGrid(
                cells, np.full(mesh.t.shape[1], 10, np.uint8),
                mesh.p.T.astype(float))
            grid.cell_data["von Mises (MPa)"] = vm.mean(axis=1)
            pl = pv.Plotter(off_screen=True, window_size=(1200, 700))
            pl.add_mesh(grid, scalars="von Mises (MPa)", cmap="turbo",
                        clim=(0, min(raw, YIELD_ALU)))
            pl.add_text(label, font_size=11)
            pl.view_vector(view)
            OUT.mkdir(exist_ok=True)
            pl.screenshot(str(OUT / png))
            print(f"  wrote {OUT / png}")
        except Exception as exc:
            print(f"  (render skipped: {exc})")
    return p995


def band_selector(axis_r, band, r_tol):
    """Facets on a cylindrical surface about the y-axis within a y-band."""
    lo, hi = band

    def sel(x):
        return ((np.abs(np.sqrt(x[0] ** 2 + x[2] ** 2) - axis_r) < r_tol)
                & (x[1] >= lo) & (x[1] <= hi))
    return sel


def uniform(force_vec, area):
    t = np.asarray(force_vec, float) / area

    def traction(x):
        return np.tile(t[:, None, None], (1, x.shape[1], x.shape[2])) \
            if x.ndim == 3 else np.tile(t[:, None], (1, x.shape[1]))
    return traction


def facet_area(mesh, sel) -> float:
    from skfem import Basis, ElementTetP1, FacetBasis, Functional

    fb = FacetBasis(mesh, ElementTetP1(), facets=mesh.facets_satisfying(sel))

    @Functional
    def one(w):
        return 1.0 + 0.0 * w.x[0]
    return float(one.assemble(fb))


# --------------------------------------------------------------------------
# 1. femur hub
# --------------------------------------------------------------------------
def femur_hub():
    HUB_R, HUB_Y = 45.0, (-45.0, 40.0)
    BORE_R = 12.5
    COLLAR_R, COLLAR_X = 28.0, 70.0
    BOX_X0, BOX_X1 = 30.0, 220.0
    WALL = 3.0

    def build(occ):
        disc = occ.addCylinder(0, HUB_Y[0], 0, 0, HUB_Y[1] - HUB_Y[0], 0, HUB_R)
        collar = occ.addCylinder(0, 0, 0, COLLAR_X, 0, 0, COLLAR_R)
        outer = occ.addBox(BOX_X0, -20, -30, BOX_X1 - BOX_X0, 40, 60)
        body, _ = occ.fuse([(3, disc)], [(3, collar), (3, outer)])
        bore = occ.addCylinder(0, HUB_Y[0] - 2, 0, 0,
                               HUB_Y[1] - HUB_Y[0] + 4, 0, BORE_R)
        hollow = occ.addBox(COLLAR_X + 5, -17, -27,
                            BOX_X1 - COLLAR_X, 34, 54)
        occ.cut(body, [(3, bore), (3, hollow)])

    mesh = occ_mesh(build)
    r_tol = LC ** 2 / (8 * BORE_R) * 2 + 0.1

    fixed = lambda x: x[0] > BOX_X1 - 1e-2  # beam cut

    # pulley bolt-ring on the -y hub face: belt pull (-x) + 418 N.m torque
    def ring_sel(x):
        r = np.sqrt(x[0] ** 2 + x[2] ** 2)
        return (np.abs(x[1] - HUB_Y[0]) < 1e-2) & (r > 29.0) & (r < 43.0)

    ring_area = facet_area(mesh, ring_sel)
    # integral of r over the ring (for torque traction scaling)
    r_mean = (29.0 + 43.0) / 2.0
    TORQUE = 418e3  # N.mm
    p_tan = TORQUE / (ring_area * r_mean)

    def ring_traction(x):
        r = np.sqrt(x[0] ** 2 + x[2] ** 2) + 1e-9
        tx = -5000.0 / ring_area + p_tan * (-x[2] / r)
        ty = np.zeros_like(r)
        tz = p_tan * (x[0] / r)
        return np.stack([tx * np.ones_like(r), ty, tz])

    bore_sel = band_selector(BORE_R, (HUB_Y[0] + 1, HUB_Y[1] - 1), r_tol)
    bore_area = facet_area(mesh, bore_sel)

    solve_case(
        mesh, fixed,
        [(ring_sel, ring_traction),
         (bore_sel, uniform([5000.0, 0.0, -1190.0], bore_area))],
        "femur hub (6061-T6) - belt pull 5 kN + 418 N.m + 1.19 kN foot",
        png="fea_femur_hub.png")


# --------------------------------------------------------------------------
# 2. coxa clevis node
# --------------------------------------------------------------------------
def coxa_clevis():
    BOSS = dict(x=(-45.0, 45.0), y=(45.0, 85.0), z=(-45.0, 45.0))
    BORE_R = 26.0
    WING_X1 = 180.0

    def build(occ):
        boss = occ.addBox(BOSS["x"][0], BOSS["y"][0], BOSS["z"][0],
                          BOSS["x"][1] - BOSS["x"][0],
                          BOSS["y"][1] - BOSS["y"][0],
                          BOSS["z"][1] - BOSS["z"][0])
        wing = occ.addBox(40, 45, -27, WING_X1 - 40, 25, 54)
        body, _ = occ.fuse([(3, boss)], [(3, wing)])
        bore = occ.addCylinder(0, BOSS["y"][0] - 2, 0, 0,
                               (BOSS["y"][1] - BOSS["y"][0]) + 4, 0, BORE_R)
        occ.cut(body, [(3, bore)])

    mesh = occ_mesh(build)
    r_tol = LC ** 2 / (8 * BORE_R) * 2 + 0.1

    fixed = lambda x: x[0] > WING_X1 - 1e-2  # crossbar attachment

    # Joint reaction 6.19 kN at load centroid y=-20 -> M ~526 N.m about the
    # clevis center.  30205 pair BACK-TO-BACK: effective spread ~48 mm ->
    # couple +-10.96 kN; plus direct R/2 = 3.1 kN each.  Worst direction: -z.
    M, R_kN, SPREAD = 526e3, 6190.0, 48.0
    F_couple = M / SPREAD
    b1 = band_selector(BORE_R, (47.0, 63.0), r_tol)   # inner bearing band
    b2 = band_selector(BORE_R, (67.0, 83.0), r_tol)   # outer bearing band
    a1, a2 = facet_area(mesh, b1), facet_area(mesh, b2)

    solve_case(
        mesh, fixed,
        [(b1, uniform([0, 0, -(F_couple + R_kN / 2)], a1)),
         (b2, uniform([0, 0, +(F_couple - R_kN / 2)], a2))],
        "coxa clevis node (6061-T6) - single-shear couple, "
        "30205s back-to-back (+14.1/-7.9 kN)",
        png="fea_coxa_clevis.png", view=(0.6, 1.0, 0.5))


if __name__ == "__main__":
    femur_hub()
    coxa_clevis()
