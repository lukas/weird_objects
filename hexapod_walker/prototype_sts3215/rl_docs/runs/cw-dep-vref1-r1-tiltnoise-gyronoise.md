# cw-dep-vref1-r1-tiltnoise-gyronoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:59:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-tiltnoise

**wandb_id**: wtyv7iyv

**hardware_ready**: False

**hypothesis**: Test whether the hardware candidate still walks cleanly when BOTH of its IMU readings are noisy at once -- a realistic cheap-IMU model: tilt-angle noise at 3x default (just PASSed alone as tiltnoise) plus gyro-rate noise at 1.5deg/s (PASSed alone as gyronoise). The two channels feed the same complementary attitude filter, which suppresses accel-tilt noise (~10x) but passes gyro noise into the integrated attitude -- their interaction is untested and bears directly on the deploy 25deg tilt-trip logic. Per P0 rule 3, k_current=0. If-true: composes free like every prior benign pair -- gv 6/6 det+sto own-cfg, 0 term, slip medians in vref1-r1's band. If-false: combined attitude noise degrades gait or produces spurious near-threshold tilt behavior -- flag before relying on a rate term in the hardware trip.

**gate**: Own-cfg (DR0.35 + tilt_noise_deg=1.0 + gyro_noise_deg_s=1.5) det+sto @15s: gait_valid 6/6 both, 0 term, slip/m MEDIANS within vref1-r1's band (0.89-1.13 det / 1.13-1.36 sto); the lineage's KNOWN fixed-seed hard-draw episodes (det/5, sto/0, sto/1) are allowed to degrade no worse than the sibling family floor (prog >= 0.45, no term, no flag-leg on frames); DR0 no-noise retention clean; frames watched det

**verdict**: PASS (12th dep-line protective compose) -- cheap-noisy-IMU interaction (tilt-angle noise 1.0deg + gyro-rate noise 1.5deg/s together) composes free onto the hardware-contract base: DR0 retention gv 12/12 0 term (det/4 known lineage crater only), own-cfg (DR0.35+both axes) gv 12/12 0 term, det slip/m med 1.06 sto med 1.31 both within vref1-r1's own band, and the 3 pre-registered fixed-draw episodes (det/5, sto/0, sto/1) degrade exactly as pre-allowed (prog 0.62-0.68, no term, no flag-leg) not a new pathology. Video (det_0..5) shows clean six-leg creep, no drag/skate/flag-leg. hardware_ready=false (derisking vref1-r1 only).

