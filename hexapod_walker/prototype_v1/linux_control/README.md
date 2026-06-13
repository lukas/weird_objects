# Xbox gamepad drive bridge (`xbox_drive.py`)

Steer the hexapod with a Bluetooth Xbox controller. The script runs on the
robot's **onboard Linux side** (the Arduino UNO Q's Qualcomm processor),
reads the gamepad directly from the Linux input layer, and sends drive
commands to the STM32 firmware through the on-board `arduino-router` monitor
port (`tcp:127.0.0.1:7500`) — the same pipe the interactive
`socat tcp:127.0.0.1:7500 -` session uses.

No Python packages are required (pure standard library).

## Firmware requirement

Flash the build of
`firmware/prototype_servo_test/prototype_servo_test.ino` that has the live
analog-drive command `J vx vy omega [gait]` (added Jun 2026 with the gait
selector + dances). The bridge streams `J` packets at ~20 Hz; the firmware
fades the foot-lift to zero at ~0 speed, so a centred stick just stands in
place and steering is instant.

## 1. Pair the controller (one time)

Put the controller in pairing mode (hold the **Xbox** button until it
flashes, then hold the small **pair** button on the top edge until it flashes
fast). On the board:

```bash
bluetoothctl
# inside the bluetoothctl prompt:
power on
agent on
default-agent
scan on                 # wait for "Xbox Wireless Controller" + its MAC
# ...note the MAC, e.g. DC:26:...:AB
pair  DC:26:...:AB
trust DC:26:...:AB      # auto-reconnect on future power-ups
connect DC:26:...:AB
scan off
quit
```

Confirm it shows up as an input device:

```bash
python3 xbox_drive.py --list
# look for a line like:
#   /dev/input/event3   Xbox Wireless Controller   <-- gamepad?
```

> Tip: for the steadiest connection use the controller's latest firmware
> (via the Xbox Accessories app) so it advertises the standard HID gamepad.

## 2. Run

```bash
# reading /dev/input needs permission -> sudo, or add yourself to 'input':
sudo python3 xbox_drive.py

# options:
python3 xbox_drive.py --list                 # list input devices
python3 xbox_drive.py --device /dev/input/event3
python3 xbox_drive.py --selftest             # no gamepad: scripted demo
python3 xbox_drive.py --max-vx 70 --max-omega 1.0
```

To avoid `sudo`, add your user to the `input` group once and re-login:

```bash
sudo usermod -aG input "$USER"
```

### Run it from your Mac over SSH

```bash
scp xbox_drive.py arduino@192.168.0.192:~
ssh -t arduino@192.168.0.192 'sudo python3 ~/xbox_drive.py'
```

## Controls

| Input | Action |
|---|---|
| **Left stick** | drive — up/down = forward/back, left/right = strafe |
| **Right stick (X)** | turn left / right |
| **RT** (right trigger) | speed boost (hold for turbo) |
| **LB / RB** | cycle gait: tripod ↔ ripple ↔ wave |
| **D-pad Up** | dance: stadium wave |
| **D-pad Down** | dance: hula sway |
| **D-pad Left** | dance: say hi (front legs wave) |
| **D-pad Right** | dance: twist & dip |
| **A** | stand / park (planted stance) |
| **B** | relax all motors (limp) |
| **X** | centre every joint |
| **Y** | legs up over the body |
| **Start** | arm / begin driving (pushing a stick also arms) |
| **Back / Xbox** | quit (parks the robot first) |

A dance plays until you grab a stick again, which resumes walking. Tune the
walk with the firmware's own `K` (swing lift) and `E` (COM lean trim)
commands over a `socat` session if it leans or rocks.

## Troubleshooting

- **No gamepad found** — re-pair (above), check `--list`, or pass `--device`.
- **`Cannot open /dev/input/...: Permission denied`** — use `sudo` or the
  `input` group.
- **Connects but nothing moves** — make sure the STM32 has the `J`-command
  firmware (`?` over `socat` should list `J vx vy w`), and that the
  `0x40`/`0x41` servo boards are powered (`I` rescans the I²C bus).
- **Laggy / drops** — keep the controller close; Wi-Fi/BT coexistence on the
  board can stutter. Lower `--max-vx` for finer control.
