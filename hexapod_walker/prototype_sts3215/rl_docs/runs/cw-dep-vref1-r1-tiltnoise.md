# cw-dep-vref1-r1-tiltnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T16:46:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 3mbhohc7

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: hardware attempt #2's tilt trip (P0 rule 1) is 25deg angle + a rate term, and the rate channel was already stress-tested (cw-dep-vref1-r1-gyronoise PASS) but the ANGLE-reading noise floor (dr.tilt_noise_deg, kept at full strength regardless of dr-scale) has never been elevated on this line. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- noisier tilt reading composes free like gyro rate noise did. If-false: noisy tilt angle interacts badly with the wide 25deg envelope (spurious near-threshold trips or missed real tilts) -- flag before hardware, since it bears directly on the deploy tilt-trip logic. Per P0 rule 3, k_current=0.

**gate**: Own-cfg (DR0.35+tilt_noise_deg=1.0, ~3x default 0.3) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-elevated-noise retention clean; frames watched det

**verdict**: PASS: 3x tilt-angle-noise floor (dr.tilt_noise_deg 0.3->1.0) composes free onto the contract-exact hardware base. Own-cfg (DR0.35+override) gv 6/6 det+sto, 0 term, slip/m med 1.00 det (band 0.89-1.13) / 1.16 sto (band 1.13-1.36); DR0 retention clean (one known fixed-draw crater det/4). The 3/12 partially-degraded eps that triggered the dig-in (det/5, sto/0,1 -- prog 0.62-0.74, slip 1.7-2.0) are the lineage's FIXED-SEED hard-DR-draw fingerprint: the identical 3 episode indices are degraded in ALL 7 PASSed siblings (torquescale worst at prog 0.47-0.57), and tiltnoise handles them among the mildest of the family; frame-checked clean six-leg gait, no flag-leg/fall. No tilt-noise/25deg-envelope interaction: noise enters via the alpha=0.98 complementary filter (~0.1deg effective on obs) and no spurious tilt trips occurred (0 term in 24 eps). Hardware attempt #2 not blocked.

