# cw-dep-vref1-r1-cmddrop-velscale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T18:08:13+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-dep-vref1-r1-cmddrop

**wandb_id**: ld20fp7m

**hardware_ready**: True

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if two different kinds of actuator-bus unreliability happen together -- dropped serial packets (the servo just holds its last position for a tick) AND a wrong assumption about the servo's top speed (today's finding: loaded peak velocity is 1.6-2.2x the sim's air-fit ceiling)? Both PASSed alone on vref1-r1 but never together, and both are properties of the SAME physical actuator/bus stack, unlike pairs that combine unrelated subsystems (e.g. floor tilt + friction). Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- actuator-uncertainty composes free like every other axis pairing tonight. If-false: stale commands plus a wrong speed ceiling interact badly (plausible: a dropped tick forces a bigger catch-up move right when the speed ceiling is also uncertain) -- flag as a real pre-attempt-#2 risk tied directly to today's loaded-actuator finding.

**gate**: own-cfg (DR0.35 + dr.cmd_drop_prob_max=0.05 + dr.latency_scale=0.5,2.5 + dr.vel_scale=0.6,2.2) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**verdict**: PASS -- dropped-command-ticks (5% prob) AND wrong loaded-speed-ceiling (0.6-2.2x) compose free on the hardware candidate, confirming if-true. Own-cfg det+sto 6/6 gait_valid, 0 term; slip/m med det 1.26 sto 1.09 -- within/better than either axis alone (cmddrop-alone det 1.08, velscale-alone det 1.30). One det episode (idx4: prog 0.12 slip 17.9) is the SAME fixed-seed-draw stall already documented on vref1-r1 and every single-axis child at the identical index (clean halt, no flag leg, gait_valid True) -- not a new pathology. DR0-no-override retention (det 6/6 gv, 0 term, prog 1.04 slip 1.02) reproduces vref1-r1's own clean band. Contact sheet + det/sto frame strips show the standard low-amplitude six-leg creep gait throughout, no dragging/skating beyond the known band.

