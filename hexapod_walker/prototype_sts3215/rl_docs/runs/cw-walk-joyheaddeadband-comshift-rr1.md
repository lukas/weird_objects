# cw-walk-joyheaddeadband-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:59:16+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joyheaddeadband

**wandb_id**: rqejfza2

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m, the comshift PASS envelope used everywhere on the driving lines) x servo deadband (1.0-3.0x) on the widest +-90deg driving package (joyheaddeadband, seed-confirmed 2/2). comshift composes for free onto the plain champion, groundtilt5, and multiple driving packages (joyfric-comshift, joyheadfric-comshift, both currently training); deadband specifically has not been tried with an off-axis CoM load. If-true: JOYSTICK GATE @90 0 falls; own-cfg (DR0.5+lat+deadband+comshift) det+sto 6/6 gv, 0 term, prog med>=0.80; DR0 nominal retention clean vs joyheaddeadband's own band. If-false: the off-axis load biases turning response under deadband-degraded actuation, costing falls or a retention-cap miss that mass-only payload composability did not show.

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 nominal retention det 6/6 gv, prog med and slip within noise of joyheaddeadband's own retention band; frames watched det

