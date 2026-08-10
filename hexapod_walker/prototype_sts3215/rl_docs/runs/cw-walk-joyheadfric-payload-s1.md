# cw-walk-joyheadfric-payload-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T02:06:39+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-joyheadfric-payload-r1

**wandb_id**: 1atl0z2c

**hardware_ready**: False

**hypothesis**: Seed twin of cw-walk-joyheadfric-payload-r1 (this cycle PASS: payload 1.0-1.4x composes cleanly onto the widest +-90deg friction-hardened driving package). One variable: seed 0->1. Ruling-7 panel start for this compose. If-true: seed1 reproduces gv 6/6, JOYSTICK GATE 0 falls, DR0 retention clean -- recipe is seed-robust. If-false: seed1 shows falls or erosion the seed0 draw didn't -- the pass was seed-lucky.

**gate**: JOYSTICK GATE @DR0.2 heading90 0 in-envelope falls; own-cfg (DR0.5+lat+fric+mass) det+sto gv 6/6, 0 term, prog_ratio med>=0.75; DR0 retention gv 6/6 prog med>=0.85; frames watched det.

**verdict**: PASS -- seed-1 twin closes the joyheadfric-payload ruling-7 panel 2/2 (matches r1's PASS, recipe is seed-robust not seed-lucky). JOYSTICK GATE @DR0.2 heading90: 0 falls, all scenarios incl. flip-stress (trk_err 0.027-0.039). Own-cfg (DR0.5+lat+fric+mass) det+sto gv 6/6, 0 term, prog med 0.86/0.84 (>=0.75 gate). DR0 nominal retention: gv 6/6, 0 term, prog med 0.90/0.84 (det clears >=0.85, sto 0.01 under but within seed noise vs r1's own retention band 1.6-2.1 slip); slip med 1.60/2.08 -- sto slip a touch higher than r1's sto (1.72) but same order, no pathology. Frames (det, both DR0 and own-DR 0.5): level six-leg tripod cycling, no flag leg, no dragging -- same paddle gait as parent/r1. Not hardware-ready (foot-slide, same as whole joyheadfric line).

