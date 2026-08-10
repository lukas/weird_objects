# cw-dep-vref1-r1-encnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:29:43+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: mpmuv6om

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's whole point is the contract-exact meas:=ref velocity obs, but it has never been trained WITH realistic joint-encoder noise (0.5deg, validated elsewhere) layered on top -- directly relevant since real STS3215 position reads are noisy and the velocity-obs contract is built on top of those reads. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; encoder noise composes free like it does elsewhere. If-false: the honest-velocity contract amplifies encoder noise into a worse velocity estimate than the old privileged-velocity obs would have -- a genuine new hardware risk to flag before deployment.

**gate**: Own-cfg (DR0.35+encoder_noise0.5deg) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-noise retention clean; frames watched det

**verdict**: PASS -- 0.5deg joint-encoder noise composes onto the contract-exact hardware candidate (k_current=0 per P0). Own-cfg det+sto 6/6 gait_valid, 0 term, slip/m det med 0.99 / sto med 1.01 (within vref1-r1's own 0.89-1.13/1.13-1.36 band). Named-baseline retention pass (identical eval minus the noise cfg-set) gives clean det 6/6 med 0.96 fwd 0.77m, matching vref1-r1's own gate (det 0.89) within noise, and reproduces vref1-r1's OWN pre-existing single-seed (idx4) sto-only paddling stall (prog 0.31 slip 4.83, same as vref1-r1's own 0.26/5.97) -- confirming the checkpoint itself is unchanged. Under encoder noise specifically that same seed-4 draw's stall migrates into det too (prog~0.01 slip~30, fwd 0.19m) but stays a march-in-place (gait_valid True, body level, no flag leg -- confirmed on video frames det_0..5) matching the already-root-caused lineage paddling-attractor class (c79 dig-in), not a new defect; medians/gate hold regardless. Not hardware-ready on its own (still the low-amplitude paddle gait); result clears encoder noise as a safe axis for the hardware candidate.

