# sim_viewer — running the RL champions in MuJoCo on the Mac

Everything needed to watch / drive the trained policies locally.
Written 2026-08-09 after re-deriving all of this one too many times.

## TL;DR

```sh
cd hexapod_walker/prototype_sts3215
sim_viewer/sim_play.sh     # ← the one to use: stand up AND walk, one window
sim_viewer/sim_web.sh      # same web UI as the robot, routed to MuJoCo
sim_viewer/sim_stand.sh    # stance champion alone (mujoco.viewer)
sim_viewer/sim_walk.sh     # walk champion alone (cv2 drive window)
sim_viewer/sim_quad.sh     # tip-back QUAD walk playground (scripted, no RL)
```

`sim_web.sh` starts a native `mujoco.viewer` window and serves the real
robot's `linux_control/webui/` at `http://127.0.0.1:8898/rl`. The web
page is the controller; the MuJoCo window is the visual surface. The
shared UI routes the RL tab's stand/lower/held-key
drive/policy-picker commands into the local sim session. Sim-only
controls live under `/api/sim/*` and are shown only against this
backend: reset stand, reset belly, fall, recover, and push.

The browser JPEG preview is off in native-viewer mode. For headless
debugging, run `python3 -m rl_move.sim.web_server` without `--viewer`,
or pass `--browser-frames on` if you intentionally want both surfaces.

If you run from a fresh git worktree, checkpoint zips may be absent
because `rl_move/sim/policies/` is intentionally ignored. Point the
server at the populated cache:

```sh
POLICY_DIR=/Users/lukas/weird_objects/hexapod_walker/prototype_sts3215/rl_move/sim/policies \
  sim_viewer/sim_web.sh
```

`sim_quad.sh` (no checkpoints involved) drives the webui Quad tab's
scripted gaits — `motor_setup/quad_walk.py`, the tip-back 4-leg animal
walk and the diagonal-pair TROT — as a live state machine in the
fitted servo twin: `7`/`W` rear up (`W` auto-walks once reared), `T`
toggle walk/trot (applies at the next rear-up), `Space` stop (freezes
at the next all-feet-down window), `8` sit back down (ends at the
plant pose), `9` reset, `-`/`=` gait speed 0.25–2× (the same live knob
as the webui slider), `P` a 4 N forward shove to poke at robustness,
drag to orbit, `Z`/`X` zoom, `Q` quit. Keys are drawn in the window.

`sim_play.sh` controls (shown in the window too):

- **Hold arrow keys to drive** (release = stop; diagonals = two arrows).
  Detected via OS key auto-repeat — no repeat for ~0.65 s counts as
  released. No modifier key is needed: the cv2 window owns every key.
- `7` stand up **in place** (no teleport: a crouch just rises; a belly
  start re-anchors the episode where the robot is, then auto-rises
  ~11 s), `8` sit, `9` reset standing (true reset, back to origin),
  `B` belly-down, `F` **fall over** (torque-off tumble; cycles sprawl →
  left side → back → right side → nose-over), `R` **run the recovery
  policy** (recover-to-plant line, `--recover`; stands back up from
  sprawls/tangles/crouches/belly — side/back inversion is not in its
  curriculum yet), `I/K/J/L` persistent cruise trim, `Space` stop,
  `=`/`-` height, `Q` quit.
- **Model picker panel** on the right of the window (like the robot
  webui): checkpoints in `rl_move/sim/policies/` are classified at
  startup by obs width read from the sb3 zip's JSON metadata (68 →
  stance slot; 72, 78 = +mode one-hot [transdagger GRU], 1152 = 16
  stacked frames [transformer] → walk slot; no torch load) and listed
  in two groups — click a row to load it. By default only a curated
  TOP TEN + the 08-18 experiments is shown (best per category: rise,
  all-round walk, no-slip, steering, speed, on-robot, the transformer
  and transdagger experiments, plus the clamp-fit scripted gait and
  the two tripod dance rows) — the
  full scan outgrew the panel and off-screen rows were unclickable;
  pass `--all` for everything (rows shrink to fit). The mode one-hot
  (+6 obs) is always appended env-side now — prefix-slicing keeps every
  older checkpoint's layout intact; the GRU model threads recurrent
  state through predict (cleared on true resets only) and the
  transformer walker runs on a client-side 16-frame stack seeded like
  a fresh episode whenever driving engages. `[` / `]` and `,` / `.` cycle
  the same lists. Swaps load on the spot (~1 s stall) and work
  mid-walk; the active pair is highlighted. Each row carries a one-line
  description (distilled from RL_LOG.md into `_DESC` in `play.py`).
  Markers: `R` = deployed on the physical robot (per the `meta` blocks
  of `linux_control/rl_*_weights.json`; pinned to the top with the
  other most-worth-trying checkpoints), `*` = trained on sim-only
  observations — privileged body velocity (`walk_obs_body_vel=1.0`)
  that the board cannot sense, so it can never run honestly on
  hardware. Only the `dep_*` line (meas:=ref) and the stance group
  (encoder/IMU-only obs) are asterisk-free. The walk list ends with
  three `S` rows — not checkpoints but the scripted no-slip gait
  (`linux_control/noslip_gait.py`) at three settings:
  `noslip_scripted_gait` (alpha 0, the original step-then-shift),
  `noslip_hybrid_a50` (alpha 0.5 — half the body travel rides a
  constant drift through swings/dwells), and `noslip_clampfit_gait`
  (`NoSlipGait.CLAMP_FIT_KW`: period 6 s, swing-heavy, alpha 1 — the
  08-12 sweep's cleanest timing under the env's fitted ~31 deg/s servo
  clamp; zero true scrub and ~4x less loaded foot-centre drift than
  the default timing). Select one and the same drive inputs run that
  gait instead of a policy (stop still hands back to the stance
  policy); slower than the RL band but zero commanded foot scrub at
  every alpha (verified: `verify_noslip --alpha 0/0.5/1` all measure
  0.0 mm true scrub, travel ratio ~0.96), and it turns: `U`/`O` trim a
  yaw rate ±0.05 rad/s per tap, including turn-in-place.
  `sim_noslip.sh` drives the gait alone with live alpha keys
  (`4`/`5`/`6` = 0/0.5/1) and `7` = the clamp-fit preset.
  Two more `S` rows run the TRIPOD gait (`linux_control/
  tripod_gait.py`) — the dance_walk victory-lap drivers:
  `tripod_prance_gait` (aggressive horse settings: 0.58 s cadence,
  32 mm knees, cruise 0.09 m/s — 1.5× the RL band) and
  `tripod_walk_gait` (stock gentle walk-demo settings, for
  comparison). Tripod rows sim under the prance's own write regime
  (speed 1500 counts/s, ACC 80): measured 08-18, the prance cadence is
  ACCELERATION-limited, not velocity-limited — ACC 20 smears it to
  0.012 m/s, ACC 80 realizes ~0.038 m/s upright at full height. `U`/`O`
  turn up to the row's omega.
  **`V` plays the whole dance_walk VICTORY LAP end-to-end** (horse
  prance out 7.5 s → ABOUT-FACE 180° turn 9.3 s → prance home 7.5 s)
  and `P` plays the about-face alone — both need NO row selection,
  auto-stand the robot first if it's sitting/low, and cancel on
  P/V/space/arrows. The turn duration is sim-calibrated (19.3 deg/s
  realized at the prance regime); out and home share the same gait and
  duration, so the return distance matches by symmetry.
- **`--phase-obs`** (pass through `sim_play.sh`): enables the walk
  env's phase clock (+2 obs dims, sin/cos at the tail; `--phase-hz`
  default 1/6 = one revolution per 6 s clamp-fit cycle) so the
  PHASE-CLOCK no-slip RL checkpoints (obs 74) are playable in the WALK
  slot — the champion is `ppo_goal_cw_arch_noslipphase1_r4` (gate PASS
  08-12: return 943, loadslip 0.54, anchor frac 1.00; BC from the
  clamp-fit scripted teacher + minimal-drift PPO). Legacy 72-obs walk
  champions still work in this mode (they are fed `obs[:72]`, their
  exact layout).

```sh
sim_viewer/sim_play.sh --phase-obs \
    --walk rl_move/sim/policies/ppo_goal_cw_arch_noslipphase1_r4.zip
```
- **Gamepad** (optional, hot-plugs whenever it's turned on; pygame/SDL,
  Xbox/PS/Switch all normalized): left stick = analog walk, `A` stand,
  `B` sit, `Y` reset, `X` belly, `LB/RB` height, d-pad U/D / L/R =
  cycle stance / walk model.

The player still auto-switches policies like the real robot: STANCE
policy while no velocity is commanded, WALK policy the moment one is,
and back on stop. The startup `objc … SDL2 … duplicate class` warnings
are expected (cv2 and pygame both bundle SDL2) and harmless.

## Environment (repo-root `.venv`, managed with uv — NEVER recreate)

```sh
cd ~/weird_objects && uv pip install ftservo-python-sdk \
  mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh \
  pygame-ce
```

(`pygame-ce` is the gamepad reader for `sim_play.sh` — headless SDL,
no pygame window.)

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
