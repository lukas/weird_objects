# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed41

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T09:25:53+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

**wandb_id**: 579uyo15

**hypothesis**: Plain English: is the tipfrac05 turn-in-place recipe's SAFETY 1-in-3-unsafe-seed-basin rate (seed13/s3 fell + sacrificed 3 legs even hazard-free, while seed7/seed23 were clean) a real ~33% failure rate or a small-n fluke? Same exact recipe -- only the RNG seed changes; this arm adds seed=41 (batch sibling of seed31/37/43).

**gate**: Read as a 4-seed batch with seed31/37/43. SAFE = own-cfg DR-0 gait_valid >=11/12, zero falls, <=1 sacrificed leg. UNSAFE = any fall or >=2 sacrificed legs on hazard-free walk episodes. Tip-tracking is a secondary read. Batch verdict: count SAFE/UNSAFE across all 4 (+3 already-run) to pin the true rate.

