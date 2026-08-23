# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed37

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T09:22:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

**wandb_id**: bz1snszu

**hypothesis**: Plain English: is the tipfrac05 turn-in-place recipe's SAFETY 1-in-3-unsafe-seed-basin rate (seed13/s3 fell + sacrificed 3 legs even hazard-free, while seed7/seed23 were clean) a real ~33% failure rate or a small-n fluke? Same exact recipe (50% dedicated turn-episode exposure on the composed turn+push+fault stack, overshoot pricing keys ON, same pre-cheat turnfault-seq1 init, 2M) -- only the RNG seed changes; this arm adds seed=37 (batch sibling of seed31/41/43).

**gate**: Read as a 4-seed batch with seed31/41/43. SAFE = own-cfg DR-0 gait_valid >=11/12, zero falls, <=1 sacrificed leg (matches seed7/seed23's clean profile). UNSAFE = any fall or >=2 sacrificed legs on hazard-free walk episodes (matches seed13/s3's pattern). Tip-tracking is a secondary read. Batch verdict: count SAFE/UNSAFE across all 4 (+3 already-run) to pin the true rate.

**verdict**: Orphaned batch sibling (its prestage never fired -- no pullckpt/pod-evals lines in orchestrator.log, unlike its seed31/41/43 siblings; not claimed by the concurrent cycle either). Ran podeval by hand on its own pod (train-0, idle CPUs). Own-cfg DR-0 gate (hazards zeroed, 12 hazard-free walk episodes): gait_valid metric nominally 6/6+6/6 (same non-zeroing-on-TERM harness quirk as seed41), but walk/det/3 shows a video-confirmed topple (TERM tilt_roll, frame strip clean fall on last frame). Same slip-on-turn-in-place-episode artifact as siblings (55-67 slip/m on 6 of 12 episodes, no forward progress denominator -- known, not new). Per the batch's pre-registered rule (UNSAFE = any fall on hazard-free walk episodes), this is UNSAFE -- 1 fall, matching seed31's pattern (also 1 fall, det/3 tilt_roll, read for context not verdicted here as it belongs to the concurrent cycle). Contributes to the batch tally recorded on seed41's verdict this same cycle: 6 completed seeds (7,23,13,31,37,41) = 2 SAFE / 4 UNSAFE. eval_amp_m5 not run (safety alone answers the batch's pre-registered question; tip-tracking is secondary per the gate).

