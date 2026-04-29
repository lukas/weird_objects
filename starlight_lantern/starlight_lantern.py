"""Generate a metal-friendly cylindrical "starlight" candle lantern.

The lantern is a hollow cylinder with a top and bottom solid band for structure,
and a Poisson-disk scatter of round through-holes in the middle. With a candle
inside, light shoots through each hole and projects an oval bright spot onto the
ground, producing a constellation-like shadow pattern.

For metal fabrication: laser-cut the unrolled rectangle (2*pi*radius wide,
height tall) from 1.0-2.0 mm aluminum, brass, or steel sheet, then roll into a
cylinder and weld/rivet the seam.
"""

import argparse

import numpy as np
import trimesh
from scipy.ndimage import gaussian_filter
from skimage import measure


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--outfile", default="starlight_lantern.stl")
    p.add_argument("--radius-mm", type=float, default=50.0,
                   help="Outer cylinder radius in mm.")
    p.add_argument("--height-mm", type=float, default=150.0,
                   help="Total cylinder height in mm.")
    p.add_argument("--wall-mm", type=float, default=1.5,
                   help="Sheet metal thickness in mm (1.0-2.0 typical).")
    p.add_argument("--top-band-mm", type=float, default=10.0,
                   help="Solid top band height (no holes).")
    p.add_argument("--bottom-band-mm", type=float, default=15.0,
                   help="Solid bottom band height (no holes); makes a stable foot.")
    p.add_argument("--num-holes", type=int, default=120,
                   help="Target number of through-holes.")
    p.add_argument("--hole-min-mm", type=float, default=3.5,
                   help="Minimum hole radius.")
    p.add_argument("--hole-max-mm", type=float, default=11.0,
                   help="Maximum hole radius.")
    p.add_argument("--min-spacing-mm", type=float, default=2.5,
                   help="Minimum metal between hole edges (Poisson-disk spacing).")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--voxel-mm", type=float, default=0.6,
                   help="Voxel size for marching cubes. Smaller = sharper but slower.")
    p.add_argument("--smooth-sigma", type=float, default=0.5)
    return p.parse_args()


def poisson_disk_periodic(num_target, x_range, y_range, x_period, radii_fn, min_spacing,
                          rng, max_tries=10000):
    """Sample points and per-point radii in (x, y) with x periodic.

    Each new point must be at least `r_existing + r_new + min_spacing` away
    from every existing point (in x-periodic Euclidean distance).
    """
    pts = []  # list of (x, y, r)
    tries = 0
    while len(pts) < num_target and tries < max_tries:
        tries += 1
        x = rng.uniform(*x_range)
        y = rng.uniform(*y_range)
        r = radii_fn(rng)
        ok = True
        for px, py, pr in pts:
            dx = abs(x - px)
            dx = min(dx, x_period - dx)
            dy = y - py
            dist = np.sqrt(dx * dx + dy * dy)
            if dist < r + pr + min_spacing:
                ok = False
                break
        if ok:
            pts.append((x, y, r))
    return np.array(pts) if pts else np.zeros((0, 3))


def main():
    args = parse_args()

    radius = args.radius_mm
    height = args.height_mm
    wall = args.wall_mm
    top_band = args.top_band_mm
    bot_band = args.bottom_band_mm
    voxel = args.voxel_mm
    rng = np.random.default_rng(args.seed)

    # Cylinder perimeter and the (arc-length, z) sampling region for hole centers.
    perim = 2.0 * np.pi * radius
    z_lo = bot_band + args.hole_max_mm
    z_hi = height - top_band - args.hole_max_mm
    if z_hi <= z_lo:
        raise SystemExit("Holes don't fit between top and bottom bands; reduce them or grow the lantern.")

    def random_radius(r):
        return r.uniform(args.hole_min_mm, args.hole_max_mm)

    holes = poisson_disk_periodic(
        num_target=args.num_holes,
        x_range=(0.0, perim),
        y_range=(z_lo, z_hi),
        x_period=perim,
        radii_fn=random_radius,
        min_spacing=args.min_spacing_mm,
        rng=rng,
    )
    print(f"Placed {len(holes)} / {args.num_holes} holes after Poisson-disk filtering.")

    # Build a voxel grid in millimetres.
    pad = max(2.0 * voxel, wall * 2.0)
    extent_xy = radius + pad + wall
    nx = int(np.ceil(2 * extent_xy / voxel)) + 1
    nz = int(np.ceil((height + 2 * pad) / voxel)) + 1
    print(f"Voxel grid: {nx} x {nx} x {nz} = {nx * nx * nz / 1e6:.1f} M voxels")

    x = np.linspace(-extent_xy, extent_xy, nx)
    y = np.linspace(-extent_xy, extent_xy, nx)
    z = np.linspace(-pad, height + pad, nz)
    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")

    R = np.sqrt(X * X + Y * Y)
    THETA = np.arctan2(Y, X)
    THETA_ARC = THETA * radius  # arc length along the cylinder surface (mm).

    # Cylinder shell: negative inside the wall.
    shell_sdf = np.abs(R - radius) - wall / 2.0

    # Vertical bounds: negative for z in [0, height].
    z_bounds_sdf = np.maximum(-Z, Z - height)

    # Hole SDF: minimum over all holes of the (arc-length, z) plane distance
    # to that hole's circle. Negative inside any hole.
    hole_sdf = np.full(X.shape, np.inf, dtype=np.float32)
    for h_theta_arc, h_z, h_r in holes:
        d_theta = THETA_ARC - h_theta_arc
        # Wrap to nearest periodic copy.
        d_theta = np.mod(d_theta + perim / 2.0, perim) - perim / 2.0
        d_z = Z - h_z
        d = np.sqrt(d_theta * d_theta + d_z * d_z) - h_r
        np.minimum(hole_sdf, d, out=hole_sdf)

    # Material region = shell AND inside z-band AND NOT in any hole.
    material_sdf = np.maximum.reduce([
        shell_sdf.astype(np.float32),
        z_bounds_sdf.astype(np.float32),
        -hole_sdf,
    ])

    if args.smooth_sigma > 0:
        material_sdf = gaussian_filter(material_sdf, sigma=args.smooth_sigma)

    print("Running marching cubes...")
    spacing = (x[1] - x[0], y[1] - y[0], z[1] - z[0])
    verts, faces, _, _ = measure.marching_cubes(
        material_sdf, level=0.0, spacing=spacing
    )
    # Move origin back to lantern frame (center of base).
    verts[:, 0] -= extent_xy
    verts[:, 1] -= extent_xy
    verts[:, 2] -= pad

    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)

    # Cleanup with trimesh 4.x API.
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()

    # Keep the largest connected component (any tiny floaters from voxel noise).
    comps = mesh.split(only_watertight=False)
    if len(comps) > 1:
        biggest = max(comps, key=lambda m: m.area)
        print(f"Discarded {len(comps) - 1} small floater component(s); keeping main shell.")
        mesh = biggest

    print(f"Faces:        {len(mesh.faces):,}")
    print(f"Watertight:   {mesh.is_watertight}")
    print(f"Bounds (mm):  {mesh.bounds.tolist()}")
    print(f"Volume (cm3): {mesh.volume / 1000:.2f}")

    mesh.export(args.outfile)
    print(f"Saved {args.outfile}")
    ply = args.outfile.replace(".stl", ".ply")
    mesh.export(ply)
    print(f"Saved {ply}")


if __name__ == "__main__":
    main()
