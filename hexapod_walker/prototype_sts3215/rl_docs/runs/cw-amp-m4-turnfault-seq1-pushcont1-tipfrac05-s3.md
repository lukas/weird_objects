# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T06:47:11+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: h56m3nn7

**hypothesis**: Plain English: second seed twin of the tipfrac05 M5-suite pass (see -s2): same exact config (50% turn-episode exposure, composed turn+push+fault, pricing keys ON, pre-cheat turnfault-seq1 init, 2M), only seed 7->13. Together with -s2 this answers recipe-vs-lottery at n=3. Prediction-if-true: tips <=0.20-0.25 with floors held. Prediction-if-false: park near 0.25+ = lottery. Strongest alternative: in-band but over the 0.20 m5 bar.

**gate**: PASS = eval_amp_m5 yaw tips <=0.20-0.25 both signs (m5-bar-clean at <=0.20) AND safety floors held (own-cfg DR-0 gait_valid >=9/12, push/fault sections PASS). Grid read across s2/s3 + original: 3/3 in-band = recipe confirmed; 1/3 = lottery. Judge tips from the m5 yaw section (hazards zeroed), never DR-0 panel medians (contamination gotcha).

**verdict**: Result: recipe FAILS to reproduce on this third seed -- not on tip-tracking (which generalizes fine) but on SAFETY. Evidence: eval_amp_m5 tips 0.218/0.228 are actually IN-BAND (<=0.25, same ballpark as s2's 0.207/0.228 and the seed=7 original's bar-clean 0.162/0.184) -- 3/3 seeds now land in-band on tip-tracking, that part of the recipe IS reproducible. But own-cfg DR-0 gait_valid is only 7/12 (det 5/6 incl. one genuine tilt_roll FALL at roll peak 30.2deg mid-episode, video-confirmed toppled; sto 2/6 with THREE separate sacrificed-leg episodes [0],[3],[5]) -- badly under the run's own >=9/12 safety floor and far worse than s2/original's clean 12/12. m5 fault section also FAILS here (gait_valid 9/12 vs bar 10, three sacrificed legs [1,2,5] vs s2's two and the original's two) though push section still PASSES clean. Why: this is a genuine seed-dependent SAFETY basin, not a tip-tracking regression -- the turn_in_place_frac=0.5 curriculum's turn-accuracy gain generalizes across seeds (3/3 in-band) but this particular basin (seed13) trades away general walk robustness (more falls/sacrificed legs even in the hazard-zeroed walk section) that the other two seeds kept intact. Next: do not promote the tipfrac05 recipe to M5-candidate status on tip-tracking evidence alone -- safety-floor seed variance (1/3 badly unsafe) is now the dominant open risk on this lineage, ahead of the already-flagged hold/forward income-repricing lever. A 4th seed would help distinguish 'seed13 is a true outlier' from 'roughly 1-in-3 basins are unsafe by default'.

