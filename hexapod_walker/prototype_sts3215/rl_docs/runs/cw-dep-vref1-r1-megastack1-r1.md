# cw-dep-vref1-r1-megastack1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T20:37:06+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: does the hardware-attempt-#2 checkpoint still walk cleanly if EVERY sensor/actuator/assembly error tested one-at-a-time tonight (14+ single/2-axis PASSes: session-start placement+zero-bias+CoM-shift, floor tilt, IMU mount/bias/position, gyro bias/noise, tilt-reading noise, encoder noise+bus latency, servo gain-spread+deadband+battery-sag, contact stiffness, floor friction, per-leg build/link-length tolerance, command-drop, velocity-ceiling widening) hit the SAME episode AT ONCE, instead of isolated pairs? Real hardware has all of these simultaneously -- this is the logical endpoint of tonight's protect-the-candidate sweep, not a new generic pair. Excludes the two known non-free axes (payload/mass-DR, elevated action-noise) and the currently-flagged kv-alone axis (uses the PASSED gainvar kp+kv combo instead). If-true: own-cfg det+sto gv >=5/6, 0 term, slip/m within vref1-r1's own +-20% tolerance -- none of tonight's individually-free margins compound when ALL stacked, closing the sweep with high confidence. If-false: gv drops or slip blows past tolerance -- real evidence combined real-world noise (not any single axis) is the actual hardware risk, worth flagging before the operator's session. (r1: retry under a new name -- cw-dep-vref1-r1-megastack1 crashed instantly on tuple-vs-scalar --cfg-set syntax, then the drain silently dropped the backlog re-queue because a W&B run with that name already existed -- fixed cfg below, new name to dodge the dedup.)

**gate**: own-cfg (DR0.35 + all listed dr.* overrides) det+sto @15s: gait_valid >=5/6 each mode, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention (no overrides) matches vref1-r1 baseline; video clean six-leg gait, no flag-leg/drag

**refused_reason**: experiments require --phase {discovery|hardening|composition|transfer} (operator 08-10). SPECIFICATION work never trains. If this continues an existing lineage, pass --parent and the phase is inherited.

