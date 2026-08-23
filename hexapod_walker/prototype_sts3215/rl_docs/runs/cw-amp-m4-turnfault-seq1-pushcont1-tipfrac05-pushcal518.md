# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T10:32:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: gzpaovbp

**hypothesis**: Plain English: does recalibrating the push-force range (which this cycle's pushcal518 arm proved eliminates 4/12 real falls on fault+push alone, 0/12 with fresh retrain) ALSO fix the near-universal (5/6 seeds, ~09:5x-b/c batch) fall on the FULL turn-in-place+fault+push composition (tipfrac05 recipe), or does turn-in-place add its own independent fall risk on top of push magnitude? Single lever vs tipfrac05 itself: dr.ext_push_n 10-25N (code default, untouched in tipfrac05) -> 5-18N, everything else byte-identical (seed=7, 2M, same turn_in_place_frac=0.5, same yaw overshoot-decay pricing, same fresh init from turnfault_seq1).

**gate**: Own-cfg DR-0 walk gate (12 episodes), read RAW terminated field via ops.sh report, NOT gait_valid. PASS = 0/12 real falls (recalibration transfers to the full composition too -- promote as new M5-candidate base). PARTIAL = falls reduced vs tipfrac05's own 2/12 but not zero (turn-in-place adds independent fall risk beyond push magnitude; needs its own fix). FAIL = still >=2/12 falls at the SAME episode pattern (det/3-style) -- recalibration alone does not generalize past the fault+push tier; escalate to push-timing-vs-gait-phase or recovery-reward pricing.

**verdict**: DIG-IN COMPLETE (extends the earlier safety-PASS verdict; status stays PASS). The flagged 'systematic recalibration trade' (walk-slip 3.62-3.82 / yaw-tips 0.216-0.249 past the M5 v1 bars on 3/3 recalibrated seeds) is a BASELINE ARTIFACT, not a cost of the 5-18N push range. The comparison anchored on parent tipfrac05-seed7's tips 0.162/0.184 + slip 3.36 — the full family table exposes that read as a 1-in-11 outlier: across ALL 11 old-range (10-25N) tipfrac05-family m5 reads (seed twins s2/s3/seed31/seed41, 5 kernelema arms, acq1), tips span 0.198-0.317 (median ~0.23) and walk det slip 2.97-4.33 (median ~3.8); the 3 recalibrated seeds sit INSIDE that distribution, at-or-better than family median, while dominating on safety (0/12 vs widespread real falls) and fault gait_valid (12/11/11 vs 9-12). Clincher: acq1 (+6M at the OLD push range, reward rising) also fails BOTH bars (slip 3.52, tips 0.204/0.269) — more optimization crosses the bars at ANY push range. ROOT CAUSE: reward<->M5-bar misalignment (08-21 ruling), pre-existing and family-wide — the reward optimum sits at tip-err ~0.21-0.25 (robot turns in place at ~25% of the commanded 0.3 rad/s; W&B env/walk_yaw_err ~0.25 at end of training) and slip ~3.6 (env/walk_loadslip_ratio 3.55-3.73, priced at ZERO: walk_loadslip_gate=0, k_loadslip_excess=0 in the whole recipe family). Recalibration merely removed the fall-noise that hid it. FORK DECISION: (a) pricing nudge, NOT (c) bar loosening — the bars encode real behavioral quality (0.25 tip err = 17% of commanded turn rate; eval_yaw's own harness gate is 0.1) and both needed mechanisms already exist bank-validated (k_yaw_prog with yaw_prog_overshoot_decay=1.0 already on; k_loadslip_excess additive charge, never yet trained; 23/23 semantics tests green this cycle). pushcal518 stays the M5-candidate safety base; single-lever pricing dose grid launched on it (k_yaw_prog 2/3; k_loadslip_excess 6/12 at bank-calibrated loadslip_ok=1.5).

