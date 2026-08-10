# cw-walk-joyheadfric-payload-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T02:06:39+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-joyheadfric-payload-r1

**wandb_id**: 1atl0z2c

**hypothesis**: Seed twin of cw-walk-joyheadfric-payload-r1 (this cycle PASS: payload 1.0-1.4x composes cleanly onto the widest +-90deg friction-hardened driving package). One variable: seed 0->1. Ruling-7 panel start for this compose. If-true: seed1 reproduces gv 6/6, JOYSTICK GATE 0 falls, DR0 retention clean -- recipe is seed-robust. If-false: seed1 shows falls or erosion the seed0 draw didn't -- the pass was seed-lucky.

**gate**: JOYSTICK GATE @DR0.2 heading90 0 in-envelope falls; own-cfg (DR0.5+lat+fric+mass) det+sto gv 6/6, 0 term, prog_ratio med>=0.75; DR0 retention gv 6/6 prog med>=0.85; frames watched det.

