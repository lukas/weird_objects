# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T10:40:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: 4u7a6y8d

**hypothesis**: Seed-robustness twin (see -seed23 sibling arm launched same cycle) of tipfrac05-pushcal518 (recalibrated dr.ext_push_n=5-18N on the full turn+fault+push composition). Original (uncalibrated) seed13 fell 1/12 plus 3 sacrificed-leg episodes; this checks whether recalibration fixes it too.

**gate**: Same as tipfrac05-pushcal518: own-cfg DR-0 gate, RAW terminated field. PASS = 0/12 real falls. Compare against seed13's original (uncalibrated) result (1 fall + 3 sacrificed legs).

**verdict**: The push-force recalibration fix holds on the third and final seed: this robot never falls once across all 12 test episodes, closing the 3-seed robustness grid at 3/3 clean. Raw per-episode terminated field (not gait_valid) is False on all 12/12 own-cfg DR-0 det+sto episodes, roll_peak max 14.2deg — including walk/det/3, the episode index where the old 10-25N push range toppled this same seed (TERM tilt_roll, 36.3deg roll_peak) and 5/6 of the original seed batch. Video: upright six-leg cycling throughout det/3 and the full det strip, no topple frame. Training healthy (reward quarters 42/116/203/244, no canary). Honest caveats, same pattern as seed7/seed23: one sto episode sacrifices leg 0 (gait_valid False, degraded-but-stable, not a fall) and 2 sto episodes are near-stationary (fwd <0.1m, the known cmd_dist divide artifact inflates their slip to ~63) — locomotion quality under sto remains the lineage's weak axis, but the gate was falls and it is 0/12. With seed7 and seed23 already PASSed by concurrent cycles, the grid is 3/3: dr.ext_push_n 5-18N is promoted as the turnfault-seq1 lineage's safety base; the 10-25N range is retired on its descendants. Next: re-run the eval_amp_m5 cross-engine suite on the recalibrated seed7 checkpoint (was only run on un-recalibrated tipfrac05) to finish the M5-candidate promotion.

