# `motor_setup/` — STS3215 bus bring-up + control

Everything that talks to the FEETECH STS3215 bus from a laptop (URT-2)
or the Arduino Uno Q lives here. Formerly split across `pi_control` /
`host` / `test_setup`.

| File | Role |
|------|------|
| `urt2_motor_setup.py` | Name / heal / test / zero-pose wizard |
| `feetech_bus.py` | STS bus driver + CLI (`scan`, `stance`, `hold`, …) |
| `urt2_bench.py` | Shared bench helpers (heal, identity, abort) |
| `inplace_demos.py` | Demos (wave / rise) + motion telemetry |
| `motion_telemetry.py` | CSV logger / shake diagnosis |
| `wire_harness_plan.py` | Cable length model |
| `motor_setup_registry.json` | Named servo IDs |

```bash
cd hexapod_walker/prototype_sts3215/motor_setup
python urt2_motor_setup.py
python inplace_demos.py --demo shimmy
python feetech_bus.py scan
```

To run the same wizard **on the Uno Q**, deploy the bundled copy:

```bash
cd ../linux_control
./deploy_urt2_setup.sh          # adb
# on board:  cd ~/hexapod_sts/urt2_setup && ./run.sh
```
