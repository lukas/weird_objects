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

**verdict**: Recalibrating dr.ext_push_n 10-25N->5-18N on a FRESH retrain (2M, seed7, byte-identical otherwise) of the full turn-in-place+fault+push composition (tipfrac05 recipe) ELIMINATES the real falls: raw 'terminated' field is False on all 12/12 det+sto own-cfg DR-0 episodes (roll_peak max 12.3 det / 12.9 sto, vs parent tipfrac05's 41.3/23.7 with 2 real det falls at the same command script). Video-confirmed: contact sheet + per-episode frame strips show clean upright six-leg walking throughout, including the specific walk_det_3 episode that toppled 5/6 of the seed batch's checkpoints pre-recalibration -- now clean. One sto episode (sto/4) sacrifices a leg (fault-carry pattern, gait_valid=False) but does not fall -- expected fault behavior, not a new pathology. Direction-error/slip-per-m stay in the same range as the un-recalibrated parent (39-76deg dir_err, slip_per_m ~3.6-11.5 excluding two near-zero-cmd sto episodes with the known cmd_dist=0 divide artifact) -- no new regression traded in for the safety fix. This directly answers the pre-registered hypothesis: push-force recalibration (found on the pushcont1 ancestor) TRANSFERS to the full composition on a fresh retrain, confirming push magnitude (not turn-in-place or fault) was the root cause of the near-universal seed-safety-lottery finding. Promoting this checkpoint as the new M5-candidate base; next step is the full eval_amp_m5 cross-engine suite plus a small seed-reproduction check before any track-level promotion.

