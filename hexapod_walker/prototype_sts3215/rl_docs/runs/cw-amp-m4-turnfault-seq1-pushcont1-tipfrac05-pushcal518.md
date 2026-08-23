# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T10:32:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: gzpaovbp

**hypothesis**: Plain English: does recalibrating the push-force range (which this cycle's pushcal518 arm proved eliminates 4/12 real falls on fault+push alone, 0/12 with fresh retrain) ALSO fix the near-universal (5/6 seeds, ~09:5x-b/c batch) fall on the FULL turn-in-place+fault+push composition (tipfrac05 recipe), or does turn-in-place add its own independent fall risk on top of push magnitude? Single lever vs tipfrac05 itself: dr.ext_push_n 10-25N (code default, untouched in tipfrac05) -> 5-18N, everything else byte-identical (seed=7, 2M, same turn_in_place_frac=0.5, same yaw overshoot-decay pricing, same fresh init from turnfault_seq1).

**gate**: Own-cfg DR-0 walk gate (12 episodes), read RAW terminated field via ops.sh report, NOT gait_valid. PASS = 0/12 real falls (recalibration transfers to the full composition too -- promote as new M5-candidate base). PARTIAL = falls reduced vs tipfrac05's own 2/12 but not zero (turn-in-place adds independent fall risk beyond push magnitude; needs its own fix). FAIL = still >=2/12 falls at the SAME episode pattern (det/3-style) -- recalibration alone does not generalize past the fault+push tier; escalate to push-timing-vs-gait-phase or recovery-reward pricing.

