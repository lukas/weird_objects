"""Ray-traced preview of the shadow a candle lantern casts onto the floor.

Computes two images side-by-side:
  1. A perfectly point-like candle (sharp shadows).
  2. A finite-radius flame (soft shadows / penumbra).
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import trimesh


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--mesh", default="starlight_lantern.stl")
    p.add_argument("--candle-height-mm", type=float, default=70.0,
                   help="Vertical position of the flame above the ground.")
    p.add_argument("--flame-radius-mm", type=float, default=5.0,
                   help="Effective light-source radius in mm. A real tealight flame is "
                        "about 4-7 mm; tapers around 5-10 mm.")
    p.add_argument("--samples", type=int, default=24,
                   help="Light-source samples for the area-light render (higher = smoother).")
    p.add_argument("--grid", type=int, default=240,
                   help="Floor pixels per side.")
    p.add_argument("--extent-mm", type=float, default=220.0,
                   help="Half-extent of the floor region, in mm.")
    p.add_argument("--lantern-radius-mm", type=float, default=50.0,
                   help="Drawn as a dashed circle on the preview for context.")
    p.add_argument("--out", default="shadow_preview.png")
    return p.parse_args()


def cast_one_light(mesh, light_pos, floor_pts):
    """For one light position, return per-pixel visibility (1 = lit, 0 = blocked)."""
    directions = floor_pts - light_pos
    lengths = np.linalg.norm(directions, axis=1, keepdims=True)
    # Avoid div by zero; pixels exactly under the light are degenerate edge cases.
    lengths = np.maximum(lengths, 1e-6)
    directions = directions / lengths
    origins = np.broadcast_to(light_pos, floor_pts.shape).copy()
    hit = mesh.ray.intersects_any(ray_origins=origins, ray_directions=directions)
    return (~hit).astype(np.float32)


def sample_in_sphere(rng, radius):
    """Uniform sample inside a sphere of given radius."""
    d = rng.normal(size=3)
    d /= np.linalg.norm(d) + 1e-12
    r = radius * rng.random() ** (1.0 / 3.0)
    return d * r


def compute_shadow_image(mesh, candle_height, flame_radius, samples, grid, extent, rng):
    xs = np.linspace(-extent, extent, grid)
    ys = np.linspace(-extent, extent, grid)
    XX, YY = np.meshgrid(xs, ys, indexing="xy")
    floor_pts = np.stack(
        [XX.ravel(), YY.ravel(), np.zeros(XX.size)], axis=-1
    ).astype(np.float64)

    visibility = np.zeros(floor_pts.shape[0], dtype=np.float32)

    candle_origin = np.array([0.0, 0.0, candle_height])
    if flame_radius <= 0:
        visibility += cast_one_light(mesh, candle_origin, floor_pts)
        n_samples = 1
    else:
        for s in range(samples):
            light = candle_origin + sample_in_sphere(rng, flame_radius)
            visibility += cast_one_light(mesh, light, floor_pts)
            if (s + 1) % max(1, samples // 4) == 0:
                print(f"    {s + 1}/{samples} samples")
        n_samples = samples
    visibility /= n_samples

    # Inverse-square style falloff so the image shows realistic brightness vs radius.
    # We use 1 / (1 + (d/d0)^2) so the lantern interior doesn't blow out.
    floor_dist = np.linalg.norm(floor_pts - candle_origin, axis=1)
    falloff = 1.0 / (1.0 + (floor_dist / 80.0) ** 2)
    return (visibility * falloff).reshape(grid, grid)


def main():
    args = parse_args()
    rng = np.random.default_rng(0)

    print(f"Loading mesh: {args.mesh}")
    mesh = trimesh.load(args.mesh, process=False)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])
    print(f"  faces:  {len(mesh.faces):,}")
    print(f"  bounds: {mesh.bounds.tolist()}")
    print(f"  ray engine: {type(mesh.ray).__name__}")

    print("\nRendering point-source shadow (sharp)...")
    img_point = compute_shadow_image(
        mesh, args.candle_height_mm, 0.0, 1, args.grid, args.extent_mm, rng
    )

    print(f"\nRendering flame shadow (radius {args.flame_radius_mm} mm, "
          f"{args.samples} samples)...")
    img_flame = compute_shadow_image(
        mesh, args.candle_height_mm, args.flame_radius_mm,
        args.samples, args.grid, args.extent_mm, rng,
    )

    vmax = max(img_point.max(), img_flame.max())
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    titles = [
        f"Point candle  (z = {args.candle_height_mm:.0f} mm)",
        f"Flame radius {args.flame_radius_mm:.1f} mm  "
        f"(z = {args.candle_height_mm:.0f} mm, {args.samples} samples)",
    ]
    for ax, img, title in zip(axes, [img_point, img_flame], titles):
        ax.imshow(
            img,
            extent=[-args.extent_mm, args.extent_mm, -args.extent_mm, args.extent_mm],
            cmap="inferno",
            origin="lower",
            vmin=0,
            vmax=vmax,
        )
        ax.set_title(title)
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        # Show lantern footprint for scale.
        circle = plt.Circle(
            (0, 0), args.lantern_radius_mm, fill=False,
            edgecolor="cyan", linewidth=1.0, linestyle="--", alpha=0.7,
        )
        ax.add_patch(circle)
        ax.set_aspect("equal")

    fig.suptitle(f"Shadow on the ground from {args.mesh}", y=1.02)
    plt.tight_layout()
    plt.savefig(args.out, dpi=120, bbox_inches="tight")
    print(f"\nSaved {args.out}")


if __name__ == "__main__":
    main()
