# Hexapod STS3215 prototype

Tabletop 3D-printed hexapod driven by Feetech STS3215 bus servos and an
Arduino Uno Q.  Design entry point: [`PROTOTYPE.md`](PROTOTYPE.md).

## Layout

| Path | What |
|------|------|
| `hexapod_prototype.py` | Parametric CAD source of truth |
| `design_spec.yaml` | Human-readable geometry contract |
| `build_all.py` / `Makefile` | Regenerate STLs + common targets |
| `docs/` | BOM, shopping list, CAD workflow, BuildViz notes |
| `scripts/` | CLI helpers (verify helpers, renders, Xometry, inspect) |
| `tools/` | BuildViz / diagnostic utilities |
| `stl_prototype/` | Slicer-ready printables |
| `stl_reference/` | Sim / viz meshes (not for printing) |
| `firmware/` / `linux_control/` / `motor_setup/` | On-robot software |
| `full_robot_viz/` | BuildViz scene + local `buildviz` npm dep |

## Quick commands

```sh
make -C hexapod_walker/prototype_sts3215 help
make -C hexapod_walker/prototype_sts3215 build
make -C hexapod_walker/prototype_sts3215 verify-fast
```
