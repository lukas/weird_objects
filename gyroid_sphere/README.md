# Gyroid sphere

A sphere whose internal structure is a triply-periodic minimal
surface (the gyroid), extracted via marching cubes. Two variants:

- `weird_mesh.py` — early exploratory version (uniform-density gyroid
  shell)
- `weird_mesh2.py` — manufacturable version with graded density: thin
  near the outer surface and progressively denser toward the centre,
  which keeps it 3-D-printable in PA12 / SLS while looking like a
  solid lattice ball from outside

## Files

| File | Purpose |
|---|---|
| `weird_mesh.py`  | Uniform-density gyroid sphere. |
| `weird_mesh2.py` | Manufacturable graded-density variant with `argparse` controls for diameter, lattice frequency, and density gradient. |
| `stl/`           | (gitignored) Generated `gyroid_sphere.stl`, `manufacturable_gyroid_sphere.stl`, `graded_gyroid_sphere.stl`. |

## Build

```bash
./run.sh gyroid_sphere/weird_mesh.py
./run.sh gyroid_sphere/weird_mesh2.py --diameter-mm 80
```
