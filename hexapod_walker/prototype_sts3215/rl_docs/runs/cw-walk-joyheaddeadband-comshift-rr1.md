# cw-walk-joyheaddeadband-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:59:16+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joyheaddeadband

**wandb_id**: rqejfza2

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m, the comshift PASS envelope used everywhere on the driving lines) x servo deadband (1.0-3.0x) on the widest +-90deg driving package (joyheaddeadband, seed-confirmed 2/2). comshift composes for free onto the plain champion, groundtilt5, and multiple driving packages (joyfric-comshift, joyheadfric-comshift, both currently training); deadband specifically has not been tried with an off-axis CoM load. If-true: JOYSTICK GATE @90 0 falls; own-cfg (DR0.5+lat+deadband+comshift) det+sto 6/6 gv, 0 term, prog med>=0.80; DR0 nominal retention clean vs joyheaddeadband's own band. If-false: the off-axis load biases turning response under deadband-degraded actuation, costing falls or a retention-cap miss that mass-only payload composability did not show.

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 nominal retention det 6/6 gv, prog med and slip within noise of joyheaddeadband's own retention band; frames watched det

**verdict**: PASS: off-center CoM payload (0.03m) composes onto the widest +-90deg deadband-hardened driving package. JOYSTICK GATE @90deg 0 in-envelope falls (all scenarios+flip-stress). Own-cfg (DR0.5+lat+deadband+comshift) det/sto gv 6/6, 0 term, prog med 0.90/0.92 (>=0.80 gate). DR0 nominal retention det/sto gv 6/6, 0 term, prog 0.93/0.92, slip 1.42/1.42 -- inside joyheaddeadband's own retention band (prog 0.90-0.93, slip 1.5-1.8). No erosion from the comshift compose.

