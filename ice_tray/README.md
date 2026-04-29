# Ice tray

A two-part ice tray with 31 icosahedron-shaped cells. Top + bottom
moulds nest together and snap apart cleanly so the ice cubes drop out.

## Files

| File | Purpose |
|---|---|
| `ice_tray.py` | Generates the top + bottom STLs. Run via `./run.sh ice_tray/ice_tray.py`. Reuses the icosahedron geometry from the shared `polyhedra.py` library at the repo root. |
| `stl/`        | (gitignored) Generated `ice_tray_31_top.stl` and `ice_tray_31_bottom.stl`. |

## Build

```bash
./run.sh ice_tray/ice_tray.py
```
