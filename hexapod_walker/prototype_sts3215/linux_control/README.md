# STS3215 linux control (Uno Q web + Xbox)

Drive the Feetech STS3215 hexapod from a browser on your laptop, or with an
Xbox controller.  The Python drive loop runs on the **Arduino Uno Q** Linux
side.  Prefer the **MCU Feetech bridge** (STM32 owns D0/D1 → FE-URT UART at
1 Mbps); fall back to a USB URT-2 / Waveshare adapter if needed.  See
`../firmware/WIRING.md` and `mcu_feetech_bus.py`.

```
Laptop browser / Xbox ──HTTPS──▶ Uno Q web_drive.py
                                      │
                                      ▼
                               DriveController + TripodGait
                                      │
                    ┌─────────────────┴─────────────────┐
                    ▼                                   ▼
            McuFeetechBus                         FeetechBus (USB)
            /dev/ttyHS1 @921600                   /dev/ttyUSB* @1Mbps
                    │                                   │
                    ▼                                   │
            feetech_bridge (MCU)                        │
            FE-URT UART @1Mbps                          │
                    └─────────────────┬─────────────────┘
                                      ▼
                               STS3215 daisy chain (IDs 2–19)
```

## Autostart on the Uno Q

`web_drive` is meant to come up on every boot via systemd:

```bash
# on the board (or via ssh / adb shell):
cd ~/hexapod_sts/linux_control/systemd
chmod +x install_autostart.sh
./install_autostart.sh          # needs sudo (password: arduino)
```

Then: `systemctl status hexapod-web`, `journalctl -u hexapod-web -f`.
After a power cycle the UI should answer on `:8080` / `:8443` without a manual start.

### Boot chain (2026-08-07)

The STM32 (feetech_bridge + TFT) does **not** auto-run its sketch after an
SoC reboot — stock images rely on `arduino-router` bringing it up ~80 s into
boot. Two things make boot fast and deterministic instead:

- **`hexapod-mcu-ready.service`** (installed by `install_autostart.sh`) runs
  right after `local-fs.target` and hard-boots the MCU over SWD
  (`systemd/mcu_boot.sh` → `remoteocd upload -f mcu_reset.cfg`). The panel
  splash + "linux Ns" boot ticker appear a few seconds after power.
- **`hexapod-web.service`** no longer waits for `network-online` (it binds
  `0.0.0.0`), and the bus constructor retries HELLO in-process — including
  the same SWD reset as a rescue — instead of exiting into a systemd
  restart loop.

Result: web up ~30 s after power (was 60 s+, or wedged forever if the port
claim raced `arduino-router`'s MCU bring-up). If the bridge ever goes
silent, `uv run python -c "import mcu_feetech_bus as m; m.mcu_reset()"` reboots
the MCU without touching servo power.

## Timestamped command / log stream → laptop

Every drive command, HTTP POST, MCU bridge command (non-chatty), print, and
motion-telemetry tick is written as JSONL on the robot and UDP-streamed to
any laptop running the receiver — **no IP to configure**. The robot
broadcasts on the LAN and learns your laptop from beacons on port 9378.

```bash
# on the laptop — leave running while you operate the robot
cd hexapod_walker/prototype_sts3215/linux_control
uv run python receive_robot_logs.py
# backup path if UDP is firewalled:
uv run python receive_robot_logs.py --ssh arduino@hexapod.local
```

Robot side: `logs/events.jsonl`. Optional override only if you need it:
`HEXAPOD_LOG_HOST=<ip>` in the systemd unit.

### Error log (website refusals / failures)

Every error the web UI shows — API `ok:false` / `"error"` responses
(e.g. `refused sit zero: …`), HTTP 4xx/5xx, and async demo/worker
errors that only surface via status polling — is logged with
`level="error"`: it lands in `events.jsonl`, is mirrored to a
dedicated `logs/errors.jsonl`, and streams over the same UDP fanout.
Identical repeats (a poll loop hitting the same failure) are deduped
to once per 10 s. Browse the recent ones without SSH:
`GET /api/errors?n=100`.

### Phone-video diagnosis

When a run fails, save the phone video and generate timestamped stills plus a
contact sheet before guessing from memory:

```bash
make robot-video VIDEO=/Users/lukas/Downloads/IMG_3613.MOV
# optional: exact timestamps and a fractional crop around the robot
make robot-video VIDEO=/path/run.mov TIMES=0,8,16,24,32 CROP=0,0.33,1,0.98
```

The helper lives at `linux_control/video_contact_sheet.py` and writes under
`artifacts/video_frames/...`.

## Fast test / deploy loop

For UI and controller edits, use the local helper instead of retyping the
whole cautious sequence. These commands do not move the robot; only
`hex_deploy` restarts the web service.

```bash
# from prototype_sts3215/
make robot-check       # syntax + web JS + git diff hygiene; no robot access
make robot-unit-check  # robot-check + fake-bus/off-robot tests
make robot-resolve     # refresh/print cached robot IP for hexapod.local
make robot-status      # read-only /api/ping + compact /api/robot summary
make robot-deploy      # robot-check + SSH deploy + remote compile + status
```

The same helpers are available as shell functions:

```bash
source linux_control/dev_loop.sh
hex_check
hex_unit_check
hex_resolve
hex_status
hex_deploy
```

Useful overrides:

```bash
HEXAPOD_HOST=http://hexapod.local:8080
HEXAPOD_SSH=arduino@hexapod.local
HEXAPOD_SSH_HOSTKEY_ALIAS=hexapod.local
```

If `hexapod.local` resolution is flaky, do not commit an IP address. Resolve
the current address once and let the helper cache it in `~/.hexapod/last_ip`.
`make robot-status`, `make robot-deploy`, `make robot-deploy-fast`, and remote
compile prefer the cached raw IP and only fall back to mDNS when the cache is
missing or stale:

```bash
make robot-resolve
make robot-status
make robot-deploy
```

There is also an explicit-path commit helper, intended to avoid
accidentally staging unrelated work from another process:

```bash
hex_commit_push "Add safer robot telemetry" \
  linux_control/web_drive.py linux_control/webui/app.js
```

## Calibration checkup / geometry sweep

The Web UI **Checkup** route runs, in order: safe zero, IMU rest/bias,
ground-contact plant search, per-leg dimension sweep, geometry plausibility,
quad IMU body-frame map, IMU-frame validation, stability-margin probe,
mass-shift response, traction/slip probe, return-to-zero, a proprioception
consistency score, optional camera witness metadata, bus error-rate checks
while still and during the motion phases, bus/power health, actuator snapshot,
then one calibration report.  The
proprioception phase is read-only after return-to-zero: it compares the expected
zero pose with live servo encoders/current/voltage/temperature and flags large
command-vs-feedback errors.  It cannot prove where the feet moved in the room;
synced camera/video is the separate witness needed for slip and body
translation.  The moving checks are deliberately less conservative than older
runs: weak geometry, small IMU-frame samples, and recoverable slip-probe guards
are recorded as issues/warnings while the checkup continues to collect
independent evidence. Operator stop/E-stop, hard command failure, high current,
or sustained large tilt still stop motion. The stability-margin phase uses small
reversible stance biases in four directions and reports a lower-bound usable
tilt, not a fall angle. The mass-shift phase lifts small limb groups and records
steady pitch/roll response, which is the first useful proxy for center-of-mass
and limb-mass mismatch in MuJoCo. The standard checkup traction phase uses two
repeated planted shear trials at several small yaw amplitudes and reports
min/max ranges; the more aggressive per-leg loaded-vs-hover drag remains the
explicit standalone slip probe. The dimension sweep
keeps five feet planted and probes several same-floor hip/knee contact poses
with the sixth leg.  Treat it as a contact-height consistency diagnostic:
vertical floor contacts alone do **not** identify absolute femur/tibia lengths
because link lengths and body height are scale-ambiguous without an independent
height/vision measurement.  The report therefore keeps configured/manual link
lengths as the dimension source and uses the sweep for per-leg height residuals,
zero-offset hints, and mismatch warnings.  Boot edges, footpad angle, floor
compliance, and servo lag can all make first contact differ from the ideal tibia
endpoint.  The sweep records weak first-contact brushes separately from firm
loaded contacts and backs the probe leg off immediately after a contact hint;
continuing to push mostly measures boot/chassis flex.  When the operator
measures knee-to-boot-tip, use the boot-radius contact model
`height = (tip_mm - boot_radius_mm) * sin(theta) + boot_radius_mm`, not a
pin-foot line to the tip.  Coxa length and chassis width remain nominal/manual
because vertical floor contacts do not observe horizontal geometry.

Deploys paint a temporary TFT **DEPLOYING** banner while the web service is
stopped.  After `/api/ping` succeeds, the deploy scripts now call
`POST /api/tft/ready` to do one normal screen repaint and clear that banner
without enabling the continuous TFT status loop.  The route skips if a motion or
calibration already owns the MCU link.

Raw sweep samples are saved on the robot as
`linux_control/logs/geometry_sweep_*.json` and copied into
`calibration_report_*.json` under `geometry.contact_sweep`.  Off-robot math
coverage lives in `linux_control/test_geometry_sweep_fit.py` and is included
in `make robot-unit-check`; it does not touch hardware.

Hand measurements live separately from the moving checkup:
`GET/POST /api/geometry/manual` reads or writes
`linux_control/logs/geometry_manual.json` with `hip_pitch_height_mm`,
`hip_center_radius_mm`, `femur_mm`, and `tibia_mm`.  The report shows these
as operator measurements, uses manual hip height for MuJoCo height hints, and
keeps FK-derived height plus current-vs-absolute knee convention comparisons
visible as consistency checks.  If the contact/FK height disagrees with the
operator measurement, the sweep is marked `manual_geometry_mismatch` and is not
used as a dimension source.

Measured-geometry decision, 2026-08-21: the old 128 mm tibia/contact length
was retired.  MuJoCo, the gait IK, and the checkup geometry now use
`femur_mm=90.0`, `tibia_mm=150.0` (knee axis to boot apex/contact tip), and
`boot_diameter_mm=14.0`.  The nominal hip-center radius remains
`200/2 + 12.5 = 112.5 mm`, close to the operator's about-114 mm measurement;
keep using `/api/geometry/manual` for measured height/radius annotations.  For
future geometry changes, update motion constants only after the measurements,
contact sweep residuals, and MuJoCo behavior agree.

### RL episode traces (automatic)

Every RL stand / lower / walk run additionally writes per-tick telemetry
on the robot: `logs/rl_<mode>_<stamp>.csv` (25 Hz — attitude, gyro, goal
refs, measured + commanded q, raw action, per-servo current) and a
matching `_summary.json` (params + result), with `rl_episode` start/end
markers in `events.jsonl`. Full column list and analysis notes:
`../rl_move/API.md` § "RL episode logging". Pull with
`scp arduino@hexapod.local:hexapod_sts/linux_control/logs/rl_*.csv`.

## Quick start (Uno Q over USB adb)

1. Plug the Uno Q into the Mac with USB-C (adb).
2. Ensure the FE-URT is on the MCU UART path (preferred) **or** on USB-C OTG.
3. From the Mac:

```bash
cd hexapod_walker/prototype_sts3215/linux_control
chmod +x deploy_adb.sh
./deploy_adb.sh            # pushes code; --dry-run if no bus yet
./deploy_adb.sh --bus       # when you want a live bus bring-up
```

4. Open:
   - UI / keyboard / on-screen sticks: **http://127.0.0.1:8080**
   - Xbox on the **Mac** (browser Gamepad API): **https://127.0.0.1:8443**  
     (accept the self-signed cert warning once)

`deploy_adb.sh` sets `adb forward` for 8080 and 8443 automatically.

### Tabs

| Tab | What it does |
|---|---|
| **Drive** | Walk teleop (sticks / keyboard / Xbox) |
| **Motors** | Live bus status (ID, load, current, alarms), wiggle, limp, go-zero |
| **Demos** | `inplace_demos` suite (shimmy, rise, walk, …) |
| **Calibrate** | Plant height / joint tests |
| **Debug** | Per-joint jog / sequential wiggle test |

### Controls

| Input | Action |
|---|---|
| Enable servos | ARM (torque on; nothing moves yet) |
| Stand | planted stand (default hip **+19°**, knee **+28°**, or learned plant) |
| Left stick / WASD | walk (tripod gait) |
| Turn stick / Q·E | yaw rate |
| Sit & power off | gentle lower, then limp |
| EMERGENCY STOP | limp immediately |
| Xbox alone | X=sit · Y=stand · A=set-here-as-zero · B=stop demo |
| Xbox chords | hold LB/LT/RB/RT then tap X/Y/A/B → 16 demos |

## Onboard Bluetooth Xbox

Pair once on the board (`bluetoothctl`), then:

```bash
adb shell
cd ~/hexapod_sts/linux_control
PYTHONPATH=vendor:urt2_setup:../motor_setup:. uv run python xbox_drive.py --list
PYTHONPATH=vendor:urt2_setup:../motor_setup:. uv run python xbox_drive.py
```

(User is already in the `input` group on stock Uno Q images.)

## Wi‑Fi (optional)

Right now the board’s `wlan0` is down, so the Mac reaches the UI through
**adb port-forward**.  To use the LAN instead:

```bash
adb shell
nmcli device wifi connect 'YOUR_SSID' password 'YOUR_PASS'
ip -4 addr show wlan0
```

Then open `http://<board-ip>:8080` (and `https://…:8443` for gamepad).

## Files

| File | Role |
|---|---|
| `web_drive.py` | HTTPS/HTTP server + bench glue (serves `webui/`) |
| `webui/` | the browser UI — HTML/CSS/JS + favicon (see `webui/README.md`) |
| `drive_controller.py` | command parser + 50 Hz gait → bus |
| `bus_bench.py` | measure feedback/tick rates over the bus (read-only) |
| `tripod_gait.py` | open-loop tripod (stdlib only; plant-synced) |
| `mcu_feetech_bus.py` | preferred MCU bridge client |
| `xbox_drive.py` | Bluetooth pad on the board |
| `vendor/` | offline `pyserial` + `scservo_sdk` |
| `deploy_adb.sh` | push + start over USB |

## Safety

- Boots **disarmed** (torque off).
- Never use Feetech `speed=0` (that means max) — the driver coerces holds.
- First walks: keep max speed low (~40 mm/s), clear floor, be ready on E-stop.

## RL / sim

Learning stack lives in `../../sts/` (MuJoCo + residual PPO).  Strategy notes:
`../RL_PLAN.md` (single RL plan doc; sim↔real facts in its Appendix A).
