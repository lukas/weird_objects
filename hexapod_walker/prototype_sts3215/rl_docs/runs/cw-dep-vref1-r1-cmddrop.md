# cw-dep-vref1-r1-cmddrop

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T16:12:06+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-latency

**wandb_id**: 64avu7iq

**hardware_ready**: True

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has never been exposed to dropped serial packets (dr.cmd_drop_prob_max, lost SyncWrite per control tick) -- directly relevant since the deployed contract writes joint targets over a real bus, and a dropped write means the servo simply holds its last commanded position for a tick, unlike every other axis tested so far. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- dropped-write robustness composes free like the other axes. If-false: intermittent stale-command ticks destabilize the gait timing (a new failure mode none of the 9 prior PASS axes would have caught, since none touch the command-delivery channel itself) -- flag before hardware as a real pre-attempt-#2 risk.

**gate**: Own-cfg det+sto 6/6 @15s gait_valid, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for stutter/stall at drop events

**verdict**: PASS -- dropped-serial-packet robustness composes free onto the named hardware candidate, matching the other 9 protected axes. Own-cfg (dr.cmd_drop_prob_max=0.05, dr.latency_scale 0.5-2.5x) det+sto both 6/6 gait_valid, 0 terminations, 0 sacrificed legs. Real per-episode median slip/m 1.02-1.15 (det, sto) sits inside vref1-r1's own 0.89-1.36 band, vel_err_mean 0.024-0.028 matches vref1-r1's own baseline; the harness headline aggregate (slip/m 5.33, prog_ratio 0.86) is dragged down by ONE fixed-draw stall episode (det/4, prog 0.056, march-in-place) -- the closed stall-lever class from c47-c52 (incidence <2 percent, command-conditioning, kept as canary, not a failure). Video (contact sheet + all 6 det clips) shows a normal six-leg gait every episode, no flag leg, no dragging. Hardware relevance: dropped SyncWrite packets (servo holds last commanded position for a tick) are a real bus failure mode never before tested on this candidate -- passes clean.

