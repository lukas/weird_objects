import argparse

import numpy as np
from skimage import measure
from scipy.ndimage import gaussian_filter
import trimesh


def parse_args():
    p = argparse.ArgumentParser(
        description="Generate a manufacturable gyroid-shell sphere STL."
    )
    p.add_argument("--outfile", default="manufacturable_gyroid_sphere.stl")
    p.add_argument("--diameter-mm", type=float, default=80.0,
                   help="Final object diameter in mm.")
    p.add_argument("--inner-radius-mm", type=float, default=18.0,
                   help="Hollow center radius; increase for lighter parts.")
    p.add_argument("--cell-size-mm", type=float, default=24.0,
                   help="Gyroid cell period in mm at the center (bigger = larger holes).")
    p.add_argument("--cell-size-outer-mm", type=float, default=None,
                   help="Cell period at the outer surface. If unset, the cell size is "
                        "constant. Set differently from --cell-size-mm for a graded "
                        "gyroid (holes change size from center to surface).")
    p.add_argument("--wall-thickness-mm", type=float, default=2.2,
                   help=">= 1.5mm is safer for nylon/MJF/SLS.")
    p.add_argument("--resolution", type=int, default=220,
                   help="Voxel grid resolution per axis.")
    p.add_argument("--smooth-sigma", type=float, default=1.0)
    p.add_argument("--padding-mm", type=float, default=4.0)
    return p.parse_args()


def main():
    args = parse_args()

    diameter_mm = args.diameter_mm
    outer_radius = diameter_mm / 2
    inner_radius = args.inner_radius_mm
    cell_size_mm = args.cell_size_mm
    wall_thickness_mm = args.wall_thickness_mm
    resolution = args.resolution
    smooth_sigma = args.smooth_sigma
    padding_mm = args.padding_mm

    extent = outer_radius + padding_mm
    x = np.linspace(-extent, extent, resolution)
    y = np.linspace(-extent, extent, resolution)
    z = np.linspace(-extent, extent, resolution)
    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")

    R = np.sqrt(X**2 + Y**2 + Z**2)

    # Local cell period (mm). Linearly graded along R if --cell-size-outer-mm is given,
    # otherwise constant. k is then a scalar or a 3D field.
    cell_outer = args.cell_size_outer_mm
    if cell_outer is None or cell_outer == cell_size_mm:
        k = 2 * np.pi / cell_size_mm
    else:
        t = np.clip(R / outer_radius, 0.0, 1.0)
        cell_local = cell_size_mm + (cell_outer - cell_size_mm) * t
        k = 2 * np.pi / cell_local

    G = (
        np.sin(k * X) * np.cos(k * Y)
        + np.sin(k * Y) * np.cos(k * Z)
        + np.sin(k * Z) * np.cos(k * X)
    )

    # Empirical mapping from wall thickness in mm to gyroid band width.
    # k may be a scalar or per-voxel array; band scales with it so wall
    # thickness stays roughly constant across the gradient.
    band = wall_thickness_mm * k * 0.75

    gyroid_sheet = np.abs(G) - band

    outer_sphere = R - outer_radius
    inner_sphere = inner_radius - R

    field = np.maximum.reduce([gyroid_sheet, outer_sphere, inner_sphere])

    field = gaussian_filter(field, sigma=smooth_sigma)

    spacing = (2 * extent) / (resolution - 1)

    verts, faces, normals, values = measure.marching_cubes(
        field, level=0, spacing=(spacing, spacing, spacing)
    )

    verts -= extent

    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)

    components = mesh.split(only_watertight=False)
    mesh = max(components, key=lambda m: m.area)

    # trimesh 4.x: explicit nondegenerate/unique helpers replace the old methods.
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()

    print("Watertight:", mesh.is_watertight)
    print("Bounds mm:", mesh.bounds)
    print("Faces:", len(mesh.faces))

    mesh.export(args.outfile)
    print(f"Saved {args.outfile}")


if __name__ == "__main__":
    main()
