"""Render a contact sheet of every lattice cube STL as a single PNG.

Uses pyvista off-screen so it works headless. Produces
`infill_lattices/contact_sheet.png` — a grid of axonometric renders, one per
lattice.
"""

import argparse
import math
from pathlib import Path

import numpy as np
import pyvista as pv

# Layout order: TPMS, then struts, then crystals, quasicrystals, fractal.
LATTICE_ORDER = [
    ("gyroid",               "Gyroid"),
    ("schwarz_p",            "Schwarz P"),
    ("schwarz_d",            "Schwarz D"),
    ("neovius",              "Neovius"),
    ("iwp",                  "Schoen I-WP"),
    ("lidinoid",             "Lidinoid"),
    ("bcc",                  "BCC struts"),
    ("fcc",                  "FCC struts"),
    ("octet",                "Octet truss"),
    ("diamond_cubic",        "Diamond cubic"),
    ("rock_salt",            "Rock salt (NaCl)"),
    ("fluorite",             "Fluorite (CaF\u2082)"),
    ("diamond_crystal",      "Diamond crystal"),
    ("quasicrystal",         "Quasicrystal (TPMS)"),
    ("quasicrystal_lattice", "Quasicrystal (lattice)"),
    ("menger",               "Menger sponge"),
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stl-dir", default="stl")
    p.add_argument("--out", default="contact_sheet.png")
    p.add_argument("--cell-px", type=int, default=520,
                   help="Per-cell pixel size in the grid.")
    p.add_argument("--cols", type=int, default=4)
    return p.parse_args()


def render_tile(plotter, mesh, title):
    plotter.clear()
    plotter.set_background("white")
    plotter.enable_lightkit()
    plotter.add_mesh(
        mesh,
        color="#cfa15d",      # warm metallic
        smooth_shading=True,
        specular=0.45,
        specular_power=22,
        diffuse=0.92,
        ambient=0.18,
    )
    # Look down a body diagonal of the cube so all three axes are visible.
    bmin, bmax = np.array(mesh.bounds[::2]), np.array(mesh.bounds[1::2])
    centre = (bmin + bmax) / 2
    diag = float(np.linalg.norm(bmax - bmin))
    cam_dir = np.array([1.0, 0.8, 0.9])
    cam_dir /= np.linalg.norm(cam_dir)
    cam_pos = centre + cam_dir * diag * 1.35
    plotter.camera_position = [tuple(cam_pos), tuple(centre), (0, 0, 1)]
    # NB: do NOT use plotter.camera.zoom() — zoom compounds across renders
    # because the same camera object is reused after plotter.clear().
    plotter.reset_camera_clipping_range()
    plotter.add_text(
        title, position="upper_edge", font_size=14, color="black",
    )
    return plotter.screenshot(return_img=True)


def main():
    args = parse_args()
    stl_dir = Path(args.stl_dir)

    cols = args.cols
    rows = math.ceil(len(LATTICE_ORDER) / cols)
    cell = args.cell_px

    plotter = pv.Plotter(off_screen=True, window_size=(cell, cell))

    tiles = []
    for slug, title in LATTICE_ORDER:
        stl_path = stl_dir / f"{slug}_cube.stl"
        if not stl_path.exists():
            print(f"  skip (missing): {stl_path}")
            tiles.append(None)
            continue
        print(f"  rendering {slug}")
        mesh = pv.read(str(stl_path))
        img = render_tile(plotter, mesh, title)
        tiles.append(img)

    plotter.close()

    # Stitch
    grid = np.full((rows * cell, cols * cell, 3), 255, dtype=np.uint8)
    for i, img in enumerate(tiles):
        if img is None:
            continue
        r, c = divmod(i, cols)
        # pyvista screenshots are RGBA float or uint8 — coerce shape
        if img.shape[-1] == 4:
            img = img[..., :3]
        if img.shape[0] != cell or img.shape[1] != cell:
            from PIL import Image
            img = np.asarray(
                Image.fromarray(img).resize((cell, cell), Image.LANCZOS)
            )
        grid[r * cell : (r + 1) * cell, c * cell : (c + 1) * cell] = img

    out = Path(args.out)
    try:
        from PIL import Image
        Image.fromarray(grid).save(out)
    except ImportError:
        import imageio.v2 as imageio
        imageio.imwrite(str(out), grid)
    print(f"\nSaved → {out}  ({grid.shape[1]} × {grid.shape[0]} px)")


if __name__ == "__main__":
    main()
