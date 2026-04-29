# Starlight lantern

An investment-cast tealight lantern that projects a starfield onto the
floor when lit. The lantern's wall is pierced by hundreds of tiny
star-shaped windows whose positions and orientations are chosen so the
shadows they cast under a tealight match a real constellation.

## Files

| File | Purpose |
|---|---|
| `starlight_lantern.py` | Generates the lantern STL. Run via `./run.sh starlight_lantern/starlight_lantern.py`. |
| `shadow_preview.py`    | Monte-Carlo ray-traced preview that renders both a sharp (point-source) and soft (finite-radius flame) shadow on the floor below the lantern. Used to verify the star projections before printing. |
| `stl/`                 | (gitignored) Generated `starlight_lantern.stl` / `.ply`. Rebuild via the script. |

## Build + preview

```bash
./run.sh starlight_lantern/starlight_lantern.py
./run.sh starlight_lantern/shadow_preview.py --mesh stl/starlight_lantern.stl
```

`shadow_preview.py` writes a side-by-side PNG comparing the shadow
from a point flame vs. a finite-radius flame so you can tune the wall
thickness and star-window geometry to keep the projection sharp.
