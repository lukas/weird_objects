# cw-dep-vref1-r1-actnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T17:01:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: jbttknkp

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has been hardened against SENSING noise (encoder, gyro, tilt) and COMMAND-path faults (cmddrop, deadband, latency) but never ACTUATOR OUTPUT noise (dr.action_noise -- random jitter applied to the commanded action itself, modeling servo-internal control-loop imprecision distinct from anything on the sensing or command-transport side). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- output-noise composes free like every other single axis so far. If-false: action-level noise compounds with the loaded servo model's settling dynamics in a way sensing/transport noise did not -- flag as a real pre-attempt-#2 actuator risk.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

