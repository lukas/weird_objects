# URT-2 motor setup on the Uno Q

Copy of `motor_setup/urt2_motor_setup.py` plus its Python deps and the
vendored Feetech SDK, meant to run on the board’s Linux side.

```bash
# from the Mac (USB adb):
./deploy_urt2_setup.sh

# on the Uno Q:
cd ~/hexapod_sts/urt2_setup
./run.sh
# or:  ssh hexapod
cd ~/hexapod_sts/urt2_setup && ./run.sh
```

Plug the **FE-URT-2** into the Uno Q USB-C OTG/hub.  `./run.sh` auto-picks
`/dev/ttyUSB*` / `ttyCH343*` (not the board’s own `ttyACM`).

Included: `urt2_motor_setup.py`, `feetech_bus.py`, `urt2_bench.py`,
`inplace_demos.py`, `motion_telemetry.py`, registry JSON, `vendor/`.
