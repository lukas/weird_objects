"""Three-panel render of the rhombic-triacontahedron quasicrystal:
down a 2-fold face normal (the print orientation), down a 5-fold vertex,
and down a 3-fold vertex. Each view highlights one of the three classes
of triacontahedron vertices."""

from pathlib import Path

import numpy as np
import pyvista as pv
from PIL import Image

PHI = (1 + np.sqrt(5)) / 2


def render_view(plotter, mesh, cam_dir, title, up=None):
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
    bmin = np.array(mesh.bounds[::2])
    bmax = np.array(mesh.bounds[1::2])
    centre = (bmin + bmax) / 2
    diag = float(np.linalg.norm(bmax - bmin))
    cam_dir = np.asarray(cam_dir, dtype=float)
    cam_dir /= np.linalg.norm(cam_dir)
    cam_pos = centre + cam_dir * diag * 1.05
    if up is None:
        up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(up, cam_dir))) > 0.95:
            up = np.array([0.0, 1.0, 0.0])
    plotter.camera_position = [tuple(cam_pos), tuple(centre), tuple(up)]
    plotter.reset_camera_clipping_range()
    plotter.add_text(title, position="upper_edge", font_size=18, color="black")
    return plotter.screenshot(return_img=True)


def main():
    stl = Path("stl") / "quasicrystal_triacontahedron.stl"
    if not stl.exists():
        raise SystemExit(f"missing {stl} — run make_quasicrystal_triacontahedron.py first")

    mesh = pv.read(str(stl))
    cell = 900
    plotter = pv.Plotter(off_screen=True, window_size=(cell, cell))

    views = [
        ((0.0, 0.0, 1.0), "down a 2-fold axis (print orientation)", None),
        ((0.0, 1.0, PHI), "down a 5-fold axis (icosahedron vertex)", None),
        ((1.0, 1.0, 1.0), "down a 3-fold axis (dodecahedron vertex)", None),
    ]
    tiles = []
    for cam_dir, title, up in views:
        print(f"  rendering {title}")
        tiles.append(render_view(plotter, mesh, cam_dir, title, up))
    plotter.close()

    grid = np.full((cell, len(tiles) * cell, 3), 255, dtype=np.uint8)
    for i, img in enumerate(tiles):
        if img.shape[-1] == 4:
            img = img[..., :3]
        grid[:, i * cell : (i + 1) * cell] = img

    out = Path("quasicrystal_triacontahedron.png")
    Image.fromarray(grid).save(out)
    print(f"  Saved → {out}  ({grid.shape[1]} × {grid.shape[0]} px)")


if __name__ == "__main__":
    main()
