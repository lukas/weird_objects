# Polyhedral constellation

A 14-node polyhedral constellation: every distinct convex solid in the
icosahedral family — 2 Platonic + 6 Archimedean + 6 Catalan — assembled
into a single wireframe sculpture. A predecessor experiment to the full
31-polyhedron [chandelier](../chandelier/).

## Files

| File | Purpose |
|---|---|
| `weird_objects.py` | Generates the constellation STL. Run via `./run.sh constellation/weird_objects.py`. Pulls vertices/edges/duals from the shared `polyhedra.py` library at the repo root. |
| `stl/`             | (gitignored) Generated `polyhedron_constellation_14_node.stl`. |

## Build

```bash
./run.sh constellation/weird_objects.py
```
