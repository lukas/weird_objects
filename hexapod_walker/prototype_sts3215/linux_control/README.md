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
silent, `python3 -c "import mcu_feetech_bus as m; m.mcu_reset()"` reboots
the MCU without touching servo power.

## Timestamped command / log stream → laptop

Every drive command, HTTP POST, MCU bridge command (non-chatty), print, and
motion-telemetry tick is written as JSONL on the robot and UDP-streamed to
any laptop running the receiver — **no IP to configure**. The robot
broadcasts on the LAN and learns your laptop from beacons on port 9378.

```bash
# on the laptop — leave running while you operate the robot
cd hexapod_walker/prototype_sts3215/linux_control
python3 receive_robot_logs.py
# backup path if UDP is firewalled:
python3 receive_robot_logs.py --ssh arduino@hexapod.local
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
| Stand | planted stand (default hip **+20°**, knee **+80°**, or learned plant) |
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
PYTHONPATH=vendor:urt2_setup:../motor_setup:. python3 xbox_drive.py --list
PYTHONPATH=vendor:urt2_setup:../motor_setup:. python3 xbox_drive.py
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
