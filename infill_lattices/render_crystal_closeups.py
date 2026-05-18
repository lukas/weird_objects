"""Higher-resolution close-up render of the three crystalline-lattice cubes.

Outputs `infill_lattices/crystals.png`: a 3-up panel of the rock salt,
fluorite, and diamond crystal cubes at large render size so the ion / atom
structure is clearly readable.
"""

from pathlib import Path

import numpy as np
import pyvista as pv

CRYSTALS = [
    ("rock_salt",       "Rock salt  (NaCl)"),
    ("fluorite",        "Fluorite  (CaF\u2082)"),
    ("diamond_crystal", "Diamond crystal  (C, ball+stick)"),
]


def render_tile(plotter, mesh, title):
    plotter.clear()
    plotter.set_background("white")
    plotter.enable_lightkit()
    plotter.add_mesh(
        mesh,
        color="#cfa15d",
        smooth_shading=True,
        specular=0.45,
        specular_power=22,
        diffuse=0.92,
        ambient=0.18,
    )
    bmin, bmax = np.array(mesh.bounds[::2]), np.array(mesh.bounds[1::2])
    centre = (bmin + bmax) / 2
    diag = float(np.linalg.norm(bmax - bmin))
    cam_dir = np.array([1.0, 0.8, 0.9])
    cam_dir /= np.linalg.norm(cam_dir)
    cam_pos = centre + cam_dir * diag * 1.35
    plotter.camera_position = [tuple(cam_pos), tuple(centre), (0, 0, 1)]
    plotter.reset_camera_clipping_range()
    plotter.add_text(title, position="upper_edge", font_size=20, color="black")
    return plotter.screenshot(return_img=True)


def main():
    stl_dir = Path("stl")
    cell = 900
    plotter = pv.Plotter(off_screen=True, window_size=(cell, cell))
    tiles = []
    for slug, title in CRYSTALS:
        mesh = pv.read(str(stl_dir / f"{slug}_cube.stl"))
        print(f"  rendering {slug}")
        tiles.append(render_tile(plotter, mesh, title))
    plotter.close()

    grid = np.full((cell, len(tiles) * cell, 3), 255, dtype=np.uint8)
    for i, img in enumerate(tiles):
        if img.shape[-1] == 4:
            img = img[..., :3]
        grid[:, i * cell : (i + 1) * cell] = img

    out = Path("crystals.png")
    from PIL import Image
    Image.fromarray(grid).save(out)
    print(f"\nSaved → {out}  ({grid.shape[1]} × {grid.shape[0]} px)")


if __name__ == "__main__":
    main()
