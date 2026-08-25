# webui/ — the hexapod web control panel

This directory is the **entire browser UI** for `../web_drive.py`, the
HTTP/HTTPS control server that runs on the Uno Q (systemd unit
`hexapod-web.service`). It used to live inside `web_drive.py` as one giant
embedded string; it is now plain files served raw — **no framework, no build
step, no bundler**. Edit a file, reload the browser, done (the server reads
these files fresh on every request and sends `Cache-Control: no-cache`).

The same files are also served by `rl_move.sim.web_server` for local
MuJoCo testing (`sim_viewer/sim_web.sh`). Keep robot-compatible route
shapes stable; sim-only affordances are gated by `/api/ping` returning
`service:"hexapod-sim"` and use `/api/sim/*`. In the normal sim path,
the native MuJoCo viewer is the display and this page is only the
controller; browser JPEG frames are an optional headless/debug mode.
`rl_move.sim.web_server` serves through the laptop hub: `/api/ping`
returns `service:"hexapod-hub"` plus the active target, and the header
target picker can route commands to `sim`, `robot`, or `both`. The robot
target can be passed at startup with `--robot-url` or attached later from
the header's robot URL field (`POST /api/hub {robot_url, target}`).

## Files

| File | What it is |
|---|---|
| `index.html` | Page markup: left sidebar nav + sticky header (arm bar, status panel) + all tab views (one page, tabs toggle views). Contains the `__HTTPS_PORT__` placeholder (see below). |
| `style.css` | All styling. Dark theme; accent blue `#2b6cff`. |
| `app.js` | All behavior: tab routing, joysticks, keyboard, Xbox gamepad, polling loops, arm/disarm gate, every API call. |
| `favicon.svg` | Hand-written hexapod mark (hexagon + six legs) so the tab is findable in a crowded browser. |
| `README.md` | This file. |

## How the server serves these files

`web_drive.py` resolves this directory as `Path(__file__).parent / "webui"`
— **never** the CWD, because systemd starts the server from an arbitrary
directory (`/home/arduino/.local/bin/uv run python /home/arduino/hexapod_sts/linux_control/web_drive.py …`).

- `/`, `/index.html`, `/motors`, `/demos`, `/debug`, `/rl`, `/calibrate`
  all serve `index.html`, with the literal `__HTTPS_PORT__` replaced by the
  HTTPS port that actually bound (default 8443). `index.html` stores it as
  `window.HEXAPOD_HTTPS_PORT`; `app.js` uses it to build the "open secure
  page" link the gamepad hint shows (browser Gamepad API needs HTTPS).
- `/style.css`, `/app.js` are served with `Cache-Control: no-cache`;
  `/favicon.svg` with `max-age=86400`. Correct MIME types on all three
  (`image/svg+xml` for the favicon).
- Only these exact whitelisted names are served — there is **no** generic
  static-file handler, so nothing else in the directory (or outside it) is
  reachable.
- If a file is missing the server answers **500 with the expected absolute
  path in the body**, and `/api/ping` keeps working — so a botched deploy is
  obvious and diagnosable from the laptop.

## Tabs — what each control sends

Command letters go to `POST /cmd` (plain text body → `DriveController`);
everything else is JSON over `/api/*`. **Do not change server routes or JSON
shapes** — `rl_move/remote.py`, `rl_move/scripts/tape_measure_walk.py`, and
the RL tooling depend on them.

### Layout (2026-08-19 redesign)

- **Left sidebar** (`#sidebar`): brand + the vertical tab nav. Collapses to
  a wrapping tab strip on top below 760 px (phones).
- **Sticky top header**: the compact arm bar on the left, the status panel
  on the right. Sticky so EMERGENCY STOP stays reachable while scrolled.
- **Status panel** (`#statusbox`, top right): link state (`#conn`), robot
  activity pill (`#robotact`), controller hint (`#gp`) and the last-command
  line (`#sent`) each get a full line, all text-selectable, plus a **Copy**
  button that puts every line, labelled, on the clipboard (for pasting into
  a chat/issue without screenshotting the corner of the screen).

### Header arm bar (always visible)

- **Enable servos** → `ARM` · **Disarm** → `SETTLE` (gentle lower, then
  power off) · **Set zero HERE** → `POST /api/set_zero` (no motion; the
  current hand pose becomes logical 0° — limp + hand-pose first) ·
  **EMERGENCY STOP** → `X` (instant limp, robot drops). The
  long lower-vs-drop explanation lives in the button tooltips.
- Link heartbeat: `GET /api/ping` every 1.5 s. Robot activity pill:
  `GET /api/robot` every 2 s.
- Every servo-driving control in every tab is gated on the arm state
  (`needArm()`), which defaults OFF on each page load.

### Drive (`#drive`) — bench-test workflow, in order of use

1. *Zero & stand*: limp (Motors → **Limp all**, or E-STOP) + hand-pose,
   **Set zero HERE** (top bar) · **Stand up**. If already upright, Stand
   adjusts/re-verifies the sim walk-ready stance; otherwise it safe-zeros first,
   then uses STEP stand-up via `POST /api/zero {pose:"stand"}` ·
   **Preflight** → `GET /api/rl/preflight?mode=lower` (read-only).
2. *Scripted gait walk*: **Start walk** sends `J vx vy ω` (confirm dialog;
   caps |vx|≤60, |vy|≤40 mm/s, |ω|≤0.5 rad/s, 3–60 s), timed stop or
   **STOP GAIT** sends `J 0 0 0`. Swing lift slider → `K <mm>`.
3. *Manual drive*: on-screen sticks / WASD+QE / Xbox left+right stick →
   throttled `J vx vy ω gait` stream at ≤20 Hz (only on this tab).
   Keyboard: Space = stand, C = sit.
4. *Wind down*: **Center / Sit** → STEP lower if standing, safe-zero
   recovery if not standing/tangled, via
   `POST /api/standup {mode:"step", direction:"down"}` ·
   **Sit & power off** → `SETTLE`.
- Telemetry strip: `GET /api/feedback` at 2 Hz while the tab is open.
- Xbox (HTTPS page only): face buttons alone X=sit · Y=stand ·
  A=set-zero-here · B=stop-demo; hold LB/LT/RB/RT + face = 16 demo chords
  (`POST /api/demo`).

### Calibrate (`#calibrate`)

Mode buttons (Checkup / Step / Shake / Plant height / Geo plant / IMU rest) →
`POST /api/calibrate {mode, step_deg, nudge_deg, axis, quad_body_frame}` ·
sim-ready report `GET /api/calibration/report` · **Stop** →
`POST /api/calibrate/stop` · status/results poll `GET /api/calibrate` at
0.8 s · **Reset plant default** → `POST /api/plant/reset` (confirm) ·
**Clear IMU calib** → `POST /api/imu/reset` (confirm). Checkup rows are
backend-reported phases; geometry plausibility and IMU-frame validation are
advisory gates, stability margin records safe reversible lean response, mass
shift records pitch/roll change from lifted limb groups, the traction row
records repeated gentle shear ranges for slip triage, and the bus/power row
records live servo count, voltage, current, and temperature. The standalone
slip tool keeps the more aggressive per-leg loaded-vs-hover drag. The
proprioception row scores expected zero pose vs live servo feedback, while the
camera witness row is intentionally optional until a synced camera source is
supplied. Switching to this tab sends `HOLD` once if armed (freezes the
background re-hold).

### Motors (`#motors`)

`GET /api/status` poll at 1.5 s (full bus scan table) · **Wiggle** →
`POST /api/wiggle {joint, amp}` · **Go zero** → `POST /api/zero` ·
**Set zero HERE** → `POST /api/set_zero` (confirm) · **Limp all** → `X`.
Switching to this tab sends `HOLD` once if armed.

### Demos (`#demos`)

Demo list from `GET /api/demos` (each item carries `group`
air/stand/plant/walk and `live_speed`); card click → `POST /api/demo
{name, speed, torque, seconds[, size, rate, softness]}` · **Stop demo** →
`POST /api/demo/stop` · sit/stand zero → `POST /api/zero` · status poll
`GET /api/robot` at 0.5 s (zero probe every 4th poll). The canvas preview
is schematic only — nothing is sent on hover.

**Live speed** (2026-08-17): dragging the speed slider while a demo runs
posts `POST /api/demo/speed {speed}` (debounced 150 ms, 0.25–3×).
Streamed demos — the standing dances (`stand_*`) plus the air wiggles —
run through `stream_pose_fn` in `inplace_demos.py`, the stand-up lab's
20 Hz pursuit engine (carrot lookahead, per-tick speed/acc sizing,
3 A stall-fight guard), and re-read the multiplier every tick; breathe
stays glide-based and picks it up at the next half-breath. `seconds`
only applies to air + streamed demos (5–300 s); planted shows keep
their scripted timing. Standing dances home through the validated 10×
keyframe stand-up and dance around the standing pose at τ900.

### RL (`#rl`)

**Stand up / Lower** → smart standard motion outside the Experiments page:
standing Stand adjusts to the sim walk-ready stance, not-standing Stand safe-zeros then
STEP-ups, standing Lower STEP-downs, not-standing Lower safe-zeros ·
**Drive — hold keys**: **Start driving** → `POST /api/rl/drive/start`,
then arrow keys / WASD (or the on-screen pad, pointerdown/up) stream
`POST /api/rl/drive/cmd {vx, vy}` heartbeats at 5 Hz while the session
is active — held key = walk that way, released = the robot decels and
holds (the robot treats a heartbeat older than 0.6 s as "keys
released", so a dead tab stops it). **End session** →
`POST /api/rl/drive/stop` (decel + hold). Keys only act on this tab
(the Drive tab's own key loop owns WASD there); leaving the tab or
window blur releases everything. A page reload reconnects to a live
session and resumes heartbeats. · **Timed walk** (details block) →
`POST /api/rl/walk {vx, vy, duration_s}` · **Model roles** selects →
`GET/POST /api/rl/roles` (which policy file serves walk / hold /
stand / lower; no motion) · **Stand up** → STEP stand-up into the sim
walk-ready stance · **Stop** →
`POST /api/rl/stop` · readiness checks →
`GET /api/rl/preflight?mode=stand|lower|walk` (read-only) · policy info →
`GET /api/rl/policy`.

### Debug (`#debug`)

Per-servo jog → `# <joint> <deg>` (throttled/de-duped like the drive
stream) · **Test this servo** → `Q <joint> <amp>` · **Test ALL** → `C` then
`Q 0..17` in sequence · **Stop test** → `C`.

## Navigation

- Every tab is deep-linkable: switching tabs rewrites the URL hash
  (`#drive`, `#motors`, …) via `history.replaceState`, and loading either a
  `#hash` or a path-style link (`/motors`, `/rl`, …) opens that tab. The
  hash wins if both are present.
- `document.title` follows the active tab (`Hexapod · Drive`, …).
- The subtitle under the H1 shows which robot host this tab is talking to
  (filled from `location.host`).
- The sidebar nav is grouped by use: **Drive** (operate) · **Calibrate,
  Motors** (setup/tuning) · **Demos, Dance, Quad, RL, Experiments, Measure,
  Debug**.

## Testing locally without a robot

From anywhere on the Mac (paths are `__file__`-relative):

```bash
uv run python hexapod_walker/prototype_sts3215/linux_control/web_drive.py \
    --dry-run --http-port 8899
open http://127.0.0.1:8899/
```

`--dry-run` serves the full UI without opening the servo bus; the Motors
table shows "No servos answering (dry-run)". `curl` checks:
`/api/ping` → `{"ok": true, …}`, `/` → HTML with the real HTTPS port
substituted, `/favicon.svg` → `image/svg+xml`.

## Deploying to the robot

The real workflow is `../deploy_adb.sh` (see `../README.md`): it pushes the
individual `linux_control` files **plus this whole `webui/` directory** to
`/home/arduino/hexapod_sts/linux_control/` over USB adb, then restarts
`hexapod-web.service` (or a nohup fallback). Because the server re-reads
these files per request, a robot-side edit during bring-up shows up on the
next browser reload — no service restart needed.
