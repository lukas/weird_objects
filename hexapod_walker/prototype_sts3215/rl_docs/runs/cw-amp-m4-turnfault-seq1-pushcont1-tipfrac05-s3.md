# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T06:47:11+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: h56m3nn7

**hypothesis**: Plain English: second seed twin of the tipfrac05 M5-suite pass (see -s2): same exact config (50% turn-episode exposure, composed turn+push+fault, pricing keys ON, pre-cheat turnfault-seq1 init, 2M), only seed 7->13. Together with -s2 this answers recipe-vs-lottery at n=3. Prediction-if-true: tips <=0.20-0.25 with floors held. Prediction-if-false: park near 0.25+ = lottery. Strongest alternative: in-band but over the 0.20 m5 bar.

**gate**: PASS = eval_amp_m5 yaw tips <=0.20-0.25 both signs (m5-bar-clean at <=0.20) AND safety floors held (own-cfg DR-0 gait_valid >=9/12, push/fault sections PASS). Grid read across s2/s3 + original: 3/3 in-band = recipe confirmed; 1/3 = lottery. Judge tips from the m5 yaw section (hazards zeroed), never DR-0 panel medians (contamination gotcha).

