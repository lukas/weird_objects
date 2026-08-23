# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed31

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T09:19:02+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

**hypothesis**: Plain English: is the tipfrac05 turn-in-place recipe's SAFETY 1-in-3-unsafe-seed-basin rate (seed13/s3 fell + sacrificed 3 legs even hazard-free, while seed7/seed23 were clean) a real ~33% failure rate or a small-n fluke? Same exact recipe (50% dedicated turn-episode exposure on the composed turn+push+fault stack, overshoot pricing keys ON, same pre-cheat turnfault-seq1 init, 2M) -- only the RNG seed changes (7/23/13 already run; this arm adds seed=31). Prediction-if-real-33%-rate: roughly 1-2 of these 4 new seeds also show a fall/sacrificed-leg-heavy own-cfg DR-0 gate despite the hazard-free walk section. Prediction-if-fluke: all 4 are clean like seed7/23, narrowing the true rate toward ~1/7.

**gate**: Read as a 4-seed batch (this arm is one of seed31/37/41/43). SAFE = own-cfg DR-0 gait_valid >=11/12, zero falls, <=1 sacrificed leg (matches seed7/seed23's clean profile). UNSAFE = any fall (TERM tilt_roll/tilt_pitch) or >=2 sacrificed legs on hazard-free walk episodes (matches seed13/s3's pattern). Judge tip-tracking (eval_amp_m5 yaw section) as a secondary read only -- this batch's question is safety rate, not tip-err (already known in-band for seed7/23, FAIL-on-safety-only for seed13). Batch verdict: count SAFE/UNSAFE across all 4 (+ the 3 already-run) to pin the true rate; >=2/4 UNSAFE here confirms a real ~1/3 basin risk needing root-cause before promotion, 0-1/4 UNSAFE narrows it toward a seed13-specific fluke.

