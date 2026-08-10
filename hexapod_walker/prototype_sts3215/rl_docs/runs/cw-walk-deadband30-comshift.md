# cw-walk-deadband30-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T05:19:27+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-deadband30

**wandb_id**: 5atknxoi

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m, the comshift envelope PASSed onto the plain champion, groundtilt5, and multiple driving packages) x servo deadband hardening (1.0-3.0x, PASS on deadband30, walk-only). Deadband has composed with payload/latency on driving lines but never with an off-axis CoM load on the isolated walk-only deadband package. If-true: own-cfg (deadband+comshift) det+sto 6/6 gv, 0 term, prog med matching deadband30's own band; DR0 flat-no-offset-no-deadband retention clean. If-false: coarse actuation resolution interacts with the off-center load to crater progress or cost falls that plain comshift composes did not show.

**gate**: Own-cfg (dr.deadband_scale=1.0,3.0 + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog med>=0.85; DR0 flat-nominal retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: PASS -- deadband hardening (1-3x) composes cleanly onto the off-center CoM payload shift (0.03m). Own-cfg det+sto 6/6 gv, 0 term, det prog med 0.95 (>=0.85 gate), det slip/m med 1.13, fwd med 1.29m. DR0 flat-nominal retention clean: det 6/6 gv, slip/m med 1.11 (<=1.24 gate), fwd med 1.50m -- no cost to the plain no-DR gait. Two det episodes (idx4/5, prog 0.56/0.43, slip 2.08/2.80) are the lineage's known fixed-draw-stall/crater draws (same indices crater in the sibling deadband30-comshift-r1 eval and other deadband30 siblings) -- not a new pathology. Frames (det 0 and 4) show the standard low-amplitude six-leg creep gait, all legs cycling, no flag leg, no drag/skate visible. Compose class continues clean (deadband x comshift joins deadband x payload/latency as another free pairing).

