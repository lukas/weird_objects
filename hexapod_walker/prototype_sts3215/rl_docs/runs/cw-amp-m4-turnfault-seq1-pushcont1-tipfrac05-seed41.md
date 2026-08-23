# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed41

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T09:25:53+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

**wandb_id**: 579uyo15

**hypothesis**: Plain English: is the tipfrac05 turn-in-place recipe's SAFETY 1-in-3-unsafe-seed-basin rate (seed13/s3 fell + sacrificed 3 legs even hazard-free, while seed7/seed23 were clean) a real ~33% failure rate or a small-n fluke? Same exact recipe -- only the RNG seed changes; this arm adds seed=41 (batch sibling of seed31/37/43).

**gate**: Read as a 4-seed batch with seed31/37/43. SAFE = own-cfg DR-0 gait_valid >=11/12, zero falls, <=1 sacrificed leg. UNSAFE = any fall or >=2 sacrificed legs on hazard-free walk episodes. Tip-tracking is a secondary read. Batch verdict: count SAFE/UNSAFE across all 4 (+3 already-run) to pin the true rate.

**verdict**: Seed-safety-variance batch (seed31/37/41/43): UNSAFE, worst of the batch so far. Own-cfg DR-0 gate (hazards zeroed, 12 hazard-free walk episodes): gait_valid metric reads 6/6 det + 6/6 sto (a harness quirk -- it does not zero out on TERM), but the raw per-episode report shows THREE separate video-confirmed topples -- walk/det/2 (TERM tilt_pitch), walk/sto/0 (TERM tilt_pitch), walk/sto/5 (TERM tilt_roll), all three frame strips show a clean fall on the last frame, no sacrificed-leg workaround this time (sac=[] everywhere). Slip is also wild on 3 more non-terminating episodes (57-70 slip/m) consistent with the known turn-in-place slip-metric artifact (near-zero forward denominator), not a new defect. Per the batch's pre-registered rule (UNSAFE = any fall on hazard-free walk episodes) this is a clear UNSAFE, worse than seed13/s3's 1-fall-plus-3-sacrificed-legs pattern. Combined with this cycle's read of the unclaimed sibling seed37 (also UNSAFE: 1 video-confirmed fall, walk/det/3 TERM tilt_roll) and the concurrent cycle's seed31 (read-only peek, not verdicted here: report also shows a det/3 TERM tilt_roll), the tipfrac05 turn-exposure recipe's basin-safety picture across 6 completed seeds (7,23,13,31,37,41) is now 2 SAFE (seed7,seed23) / 4 UNSAFE (seed13,seed31,seed37,seed41) -- roughly 2/3 unsafe, not the ~1/3 the n=3 read suggested. Tip-tracking (secondary per the gate) not read this cycle -- safety alone already answers the batch question. NEXT: do not promote tipfrac05 past PASS-candidate status; the seed-safety-variance root-cause (named prerequisite in amp/STATUS.md) is now the dominant open blocker, more urgent than the hold/forward income-repricing question. seed43 (batch's 4th arm) still training, owned by this run's watcher cycle.

