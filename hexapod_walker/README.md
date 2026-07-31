# Hexapod Walker

A family of six-legged walking-robot designs, from a tabletop 3D-printed
prototype up to a human-carrying vehicle. Each variant lives in its own
subdirectory with its own README / build guide:

| Directory | What it is |
|---|---|
| [`fullsize_v1/`](fullsize_v1/) | The original human-carrying walker (18 harmonic-drive servos, ~4 m foot-to-foot). Parametric STL generator, MuJoCo simulation, RL gait training, Blender renders. See [`fullsize_v1/README.md`](fullsize_v1/README.md) and [`fullsize_v1/ASSEMBLY.md`](fullsize_v1/ASSEMBLY.md). |
| [`prototype_v1/`](prototype_v1/) | First tabletop prototype for hobby servos (DS3225 / MG996R class) and FDM printing. See [`prototype_v1/PROTOTYPE.md`](prototype_v1/PROTOTYPE.md). |
| [`prototype_sts3215/`](prototype_sts3215/) | **Current tabletop prototype** — Feetech STS3215 serial-bus servos, Arduino UNO Q brain, full CAD validation pipeline. See [`prototype_sts3215/PROTOTYPE.md`](prototype_sts3215/PROTOTYPE.md) and [`prototype_sts3215/CAD_WORKFLOW.md`](prototype_sts3215/CAD_WORKFLOW.md). |
| [`rideable_v1/`](rideable_v1/) | Design study for an affordable rideable walker (cheaper actuators than `fullsize_v1`). See [`rideable_v1/README.md`](rideable_v1/README.md). |

Also here: [`hex`](hex) — one-shot command CLI for the physical
`prototype_sts3215` robot (sends commands to the UNO Q over SSH/adb).

The gait controller and inverse-kinematics math are shared across scales:
the prototypes reuse the full-size RL environment and training code from
`fullsize_v1/` (`hexapod_env.py`, `train_walker.py`) via thin wrapper
modules in each prototype directory.
