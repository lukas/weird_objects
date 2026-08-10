# sim_viewer — running the RL champions in MuJoCo on the Mac

Everything needed to watch / drive the trained policies locally.
Written 2026-08-09 after re-deriving all of this one too many times.

## TL;DR

```sh
cd hexapod_walker/prototype_sts3215
sim_viewer/sim_play.sh     # ← the one to use: stand up AND walk, one window
sim_viewer/sim_stand.sh    # stance champion alone (mujoco.viewer)
sim_viewer/sim_walk.sh     # walk champion alone (cv2 drive window)
```

`sim_play.sh` keys (shown in the window too): `7` stand up (fully
automatic, ~11 s), `I/K/J/L` walk / strafe, `Space` stop, `8` sit,
`9` reset standing, `B` belly-down, `Q` quit.

## Environment (repo-root `.venv`, managed with uv — NEVER recreate)

```sh
cd ~/weird_objects && uv pip install ftservo-python-sdk \
  mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh
```

- torch IS required: sb3 loads the PPO zips; there is no torch-free
  viewer path. First download ~2 min; uv caches wheels afterwards.
- `trimesh` is imported by the sim's CAD geometry builder
  (`hexapod_prototype.py`), not just CAD scripts.

## Checkpoints

Champions live on the CoreWeave pods, NOT in git. As of 2026-08-09
(both pulled into `rl_move/sim/policies/`):

| role | file | md5 (prefix) | notes |
|---|---|---|---|
| stance (stand/sit/heights) | `ppo_goal_cw_stance_dr10.zip` | `da1d912a` | "crown jewels", solved at DR 1.0 |
| walk | `ppo_goal_cw_walk_longdist_r2.zip` | `bcddc65c` | sim-champion; NOT hardware-ready (paddle-slide) |

Pull others with:

```sh
KUBECONFIG=~/.kube/coreweave.yaml kubectl cp \
  <pod>:/workspace/prototype_sts3215/rl_move/sim/policies/<name>.zip \
  rl_move/sim/policies/<name>.zip
```

Verify md5 against RL_LOG.md. `hexapod-mjx-train-0` and
`hexapod-sweep-friction` hold the most checkpoints.

## How the pieces fit (don't re-derive)

- **Both champions are per-joint policies**: action = 18 dims in
  [-1,1] mapped affinely to absolute joint angles
  (`joint_task.action_to_q_rad`).
- **Obs widths**: stance = 59 proprio + 9 goal = **68**
  (`SimHexapodJointGoalEnv`, i.e. `view.py --task joint_goal`); walk =
  68 + [vx_ref, vy_ref, vx_meas, vy_meas] = **72**
  (`SimHexapodJointWalkEnv`). Because the walk obs is the stance obs
  with 4 dims appended, `play.py` runs BOTH policies in one walk env —
  the stance policy reads `obs[:68]`.
- **`view.py`'s default `--task goal` is a different env** (56 obs,
  6-dim body-offset actions). Loading a champion there dies with
  "Unexpected observation shape (56,)". Always `--task joint_goal`.
- **The two champions stand in different poses.** The stance line's
  belly rise ends in a ~72 mm crouch-stand; the walk champion operates
  around the ~142 mm plant stance (joint gap up to 104°). Handing
  control straight from one to the other collapses into a belly
  shuffle (measured 8 cm/10 s vs 52 cm/10 s from plant). `play.py`'s
  `7` key bridges it: stance-policy rise → scripted 1.5 s joint-target
  blend to the plant pose → episode re-anchor in the plant frame.
- **Height refs are relative to the episode's start pose**: belly-start
  episodes stand at +45 mm; plant-start episodes crouch at −60 mm.
- **The ~5 s motionless curl** before a belly stand-up is the training
  rise profile (`rise_hold_s`), not a hang.
- **Goal refs must ramp** (tilt ~4°/s, height 12 mm/s, velocity
  ~0.06 m/s²) — the policies only ever saw ramped references; step
  inputs are out-of-distribution and they just sit there.

## Interpreters (this bites every time)

- `rl_move.sim.view` must run under **mjpython** (`mujoco.viewer`
  needs the macOS main-thread event loop). `view.py` re-execs itself
  under mjpython automatically — do not "fix" that.
- `rl_move.sim.drive_policy` and `rl_move.sim.play` must run under
  **plain python**: they render offscreen + cv2 because mjpython
  segfaults intermittently on macOS. Bonus: cv2 windows have NO
  built-in key bindings, unlike `mujoco.viewer` where nearly every
  letter is a display toggle (that's why `view.py` uses digits only).

## Files

- `sim_play.sh` → `rl_move/sim/play.py` — combined stand+walk player.
- `sim_stand.sh` → `rl_move/sim/view.py --task joint_goal` — stance
  viewer with in-window HUD (`viewer.set_texts`).
- `sim_walk.sh` → `rl_move/sim/drive_policy.py` — walk driver.

Robot deployment of these policies: see `../linux_control/rl_policy.py`
(numpy export, no torch on the board) and `rl_move/API.md`.
