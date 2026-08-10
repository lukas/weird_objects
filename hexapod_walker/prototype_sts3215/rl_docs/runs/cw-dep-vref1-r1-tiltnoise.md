# cw-dep-vref1-r1-tiltnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T16:46:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 3mbhohc7

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: hardware attempt #2's tilt trip (P0 rule 1) is 25deg angle + a rate term, and the rate channel was already stress-tested (cw-dep-vref1-r1-gyronoise PASS) but the ANGLE-reading noise floor (dr.tilt_noise_deg, kept at full strength regardless of dr-scale) has never been elevated on this line. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- noisier tilt reading composes free like gyro rate noise did. If-false: noisy tilt angle interacts badly with the wide 25deg envelope (spurious near-threshold trips or missed real tilts) -- flag before hardware, since it bears directly on the deploy tilt-trip logic. Per P0 rule 3, k_current=0.

**gate**: Own-cfg (DR0.35+tilt_noise_deg=1.0, ~3x default 0.3) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-elevated-noise retention clean; frames watched det

