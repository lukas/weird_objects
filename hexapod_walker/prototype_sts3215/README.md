# Hexapod STS3215 prototype

Tabletop 3D-printed hexapod driven by Feetech STS3215 bus servos and an
Arduino Uno Q. This directory holds THREE projects that share the robot;
start at the entry point for the one you're working on:

| You are here to… | Start at |
|------------------|----------|
| Design/print/assemble the robot (CAD, BOM) | [`PROTOTYPE.md`](PROTOTYPE.md) |
| Run the physical robot (firmware, control, safety) | `firmware/`, `linux_control/`, `rl_move/API.md` — **read the hardware-safety rules in the repo root `AGENTS.md` first** |
| Train it in simulation (RL campaign + autonomous agent loop) | [`RL_GOALS.md`](RL_GOALS.md) — the two goals in plain English; then [`rl_docs/README.md`](rl_docs/README.md) (doc index), `RL_PLAN.md`, `RL_LOG.md` |

## Layout

| Path | What |
|------|------|
| `hexapod_prototype.py` | Parametric constants + trimesh twins (probes, MuJoCo, BuildViz) |
| `cad_step_test/build_step_first_test.py` | Printable BREP builders (STEP-first geometry source) |
| `build_step_prototype.py` / `step_pipeline.py` | Print-set exporter + equivalence gates / shared plumbing |
| `design_spec.yaml` | Human-readable geometry contract |
| `build_all.py` / `Makefile` | Regenerate STEP + STLs + common targets |
| `docs/` | BOM, shopping list, CAD workflow, BuildViz notes |
| `scripts/` | CLI helpers (verify helpers, renders, print orientation, inspect) |
| `tools/` | BuildViz / diagnostic utilities |
| `step_prototype/` | Per-printable `.step` CAD truth + BREP tessellations + manifest |
| `stl_prototype/` | Slicer-ready printables (healed BREP tessellations) |
| `stl_reference/` | Sim / viz meshes (not for printing) |
| `firmware/` / `linux_control/` / `motor_setup/` | On-robot software |
| `full_robot_viz/` | BuildViz scene + local `buildviz` npm dep |
| `rl_docs/` | RL campaign docs index: goal, operator wishlist, commands, log conventions |
| `RL_PLAN.md` / `RL_LOG.md` | Current RL plan + condensed campaign history (full history in `archive/`) |
| `rl_move/` | RL code: `sim/` (MuJoCo/MJX envs + training), `orchestrator/` (autonomous loop: watcher, launcher, guardrails), robot-side control |
| `logs/` | Eval artifacts + per-experiment summaries (`logs/experiments/<run>/`) |
| `archive/` | Dated reviews, rulings, full plan/log history — search, don't read |

## Quick commands

Local Python convention: use `uv run python ...` or `uv run python -m ...`;
do not copy old bare `python3` examples from logs/archive. The repo-root
`AGENTS.md` and this project's `AGENTS.md` record the rule. Native
MuJoCo GUI/viewer launches on macOS are the special exception: use
`uv run mjpython ...` or the Makefile viewer targets.

```sh
make -C hexapod_walker/prototype_sts3215 help
make -C hexapod_walker/prototype_sts3215 build
make -C hexapod_walker/prototype_sts3215 verify-fast
make -C hexapod_walker/prototype_sts3215 robot-check       # safe local robot/web checks
make -C hexapod_walker/prototype_sts3215 robot-deploy      # check + SSH deploy + remote health
```

Robot-control dev loop details live in `linux_control/README.md` and
`linux_control/dev_loop.sh`. Use `robot-resolve` for a temporary IP when
`hexapod.local` is flaky; do not commit fixed board IPs.
