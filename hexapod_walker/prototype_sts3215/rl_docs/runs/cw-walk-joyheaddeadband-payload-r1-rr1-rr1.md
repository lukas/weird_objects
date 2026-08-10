# cw-walk-joyheaddeadband-payload-r1-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T04:23:03+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-joyheaddeadband-payload

**wandb_id**: r5ua1b50

**hypothesis**: Retry (r1) of cw-walk-joyheaddeadband-payload, which was REFUSED this cycle by a launcher scheduling race (target pod grabbed by a concurrent drain a moment earlier) -- not a science result. Same hypothesis unchanged: payload (1.0-1.4x mass) x servo deadband (1.0-3.0x) on the widest +-90deg driving package (joyheaddeadband, seed-confirmed 2/2).

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x+mass1.0-1.4x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 nominal retention det 6/6 gv, prog med and slip within noise of joyheaddeadband's own retention band; frames watched det

