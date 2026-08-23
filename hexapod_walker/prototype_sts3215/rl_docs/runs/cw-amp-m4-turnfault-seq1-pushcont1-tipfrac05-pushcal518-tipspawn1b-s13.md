# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn1b-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED_FAIL

**created**: 2026-08-23T15:07:34+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn1b

**wandb_id**: ze8tqgat

**hypothesis**: Plain English: the only arm ever to beat the walk-slip bar (tipspawn1b: mid-walk RSI start_frac=0.5 + live spawn omega wz=1.0 combined, m5 walk det_slip_med 3.1855 vs parent 3.67, bar 3.5) might be a one-seed fluke -- this arm re-runs the EXACT tipspawn1b recipe with only the seed changed to test whether the combined state-visitation effect replicates. Both halves alone are now measured unmoved (startonly 3.6015, wzonly 3.59), so only the interaction is live. Prediction-if-true (effect real): det_slip_med lands <=3.5 again. Prediction-if-false (noise): slip clusters at the 3.55-3.7 family baseline like the two single-lever arms. Strongest alternative: a real but fragile interaction that replicates in some seeds only -- that is why this is a 3-seed batch (s11/s13/s17) read at grid level.

**gate**: eval_amp_m5 walk section on own cfg, per-arm: PASS = det_slip_med <=3.5 with 0/12 falls and gait_valid 12/12; FAIL = >3.5. GRID ruling (3 replicates + original s7): >=2/3 replicates PASS = combined start_frac+spawn_wz effect REAL -> promote both levers together into the lineage (next: acquisition arm on pushcal518+both). 1/3 = fragile -- dig in on the passing seed before any promotion. 0/3 = tipspawn1b was seed/eval noise -> close the state-visitation slip fork; remaining live slip mechanism is the train-noise-floor anneal family (stdanneal45-r2, other cycle).

**verdict**: The slip win did not replicate at this seed either: exact tipspawn1b recipe, seed 13, m5 walk det_slip_med 3.73 vs bar 3.5 (original tipspawn1b 3.1855; parent 3.67) -- squarely in the family baseline band the prediction-if-false named. Safety/gait clean: 0 falls all sections, gait_valid 6/6 det, healthy six-leg det strip; reward trained normally (41->252). Second of three FAILs in the replicate grid (s11 3.55, s17 3.765): 0/3 PASS, so per the pre-registered grid ruling the combined start_frac x spawn_wz slip effect was seed/eval noise and the state-visitation slip fork is CLOSED.

