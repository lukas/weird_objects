"""Higher-resolution render of the icosahedral quasicrystal cubes from
three viewing axes — body diagonal, a 5-fold axis, and a 3-fold axis — so
the aperiodic icosahedral symmetry is clearly readable.

Produces two PNGs:
    quasicrystal.png         — the TPMS-like density-wave variant
    quasicrystal_lattice.png — the cut-and-project vertices + edges variant
"""

import argparse
from pathlib import Path

import numpy as np
import pyvista as pv

PHI = (1 + np.sqrt(5)) / 2


def render_view(plotter, mesh, cam_dir, title):
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
    cam_dir = np.asarray(cam_dir, dtype=float)
    cam_dir /= np.linalg.norm(cam_dir)
    cam_pos = centre + cam_dir * diag * 1.35
    # Pick an up vector roughly orthogonal to view direction so the camera
    # doesn't degenerate when looking down a coordinate axis.
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(up, cam_dir)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    plotter.camera_position = [tuple(cam_pos), tuple(centre), tuple(up)]
    plotter.reset_camera_clipping_range()
    plotter.add_text(title, position="upper_edge", font_size=18, color="black")
    return plotter.screenshot(return_img=True)


def render_one_stl(stl_path, out_path, cell=900):
    """Render a 3-up panel (axonometric / 5-fold / 3-fold) for one STL."""
    mesh = pv.read(str(stl_path))
    plotter = pv.Plotter(off_screen=True, window_size=(cell, cell))
    views = [
        ((1.0, 0.8, 0.9), "axonometric"),
        ((1.0, PHI, 0.0), "down a 5-fold axis"),
        ((1.0, 1.0, 1.0), "down a 3-fold axis"),
    ]
    tiles = []
    for cam_dir, title in views:
        print(f"  [{stl_path.name}] rendering {title}")
        tiles.append(render_view(plotter, mesh, cam_dir, title))
    plotter.close()

    grid = np.full((cell, len(tiles) * cell, 3), 255, dtype=np.uint8)
    for i, img in enumerate(tiles):
        if img.shape[-1] == 4:
            img = img[..., :3]
        grid[:, i * cell : (i + 1) * cell] = img

    from PIL import Image
    Image.fromarray(grid).save(out_path)
    print(f"  Saved → {out_path}  ({grid.shape[1]} × {grid.shape[0]} px)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variants", nargs="+",
                        default=["quasicrystal", "quasicrystal_lattice"])
    args = parser.parse_args()
    for slug in args.variants:
        stl = Path("stl") / f"{slug}_cube.stl"
        if not stl.exists():
            print(f"  skip (missing): {stl}")
            continue
        out = Path(f"{slug}.png")
        render_one_stl(stl, out)


if __name__ == "__main__":
    main()
