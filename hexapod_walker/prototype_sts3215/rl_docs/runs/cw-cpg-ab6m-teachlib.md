# cw-cpg-ab6m-teachlib

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T20:43:33+00:00

**pod**: hexapod-mjx-train-3

**steps**: 6000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: buvr417c

**hypothesis**: Plain English: the control arm of a 2-run A/B asking whether the CPG-searched gait's ~15-20% deficit as an AMP style source (cw-cpg-teacherfork-ab-cpgv1, WORSE-BUT-WALKING at 2M) persists when both style sources get 3x budget, or was an early-training artifact. This arm: the already-PASSED style05 recipe (teacher_v2 library), byte-identical, at 6M instead of 2M. Prediction-if-true (deficit is real): this arm's det margins stay >=10% ahead of the cpg_v1 sibling at 6M. Prediction-if-false: the sibling matches/inverts. Strongest alternative: both arms ceiling the bars at 6M and margins become indistinguishable — then judge the pair on slip and gait-quality margins. cpg STATUS Next item (second adoption data point), funded now that GPUs are free.

**gate**: Joint A/B read vs cw-cpg-ab6m-cpglib at 6M, DR-0 gate harness: compare det/sto gait_valid, fwd travel med, prog med, slip med. NO-SWAP (adoption fork closes) if this teacher arm leads det fwd/prog by >=10% with slip not worse; ADOPTION-LIVE if cpg arm reaches parity or better on det fwd/prog/gait with slip within +0.3; if both ceiling, judge on slip + gait margins. Absolute floor for either arm: det gait_valid >=5/6, net fwd med >=0.10m/15s, no crouch collapse.

**verdict**: The teacher-library control arm shows the scripted teacher no longer leads the CPG library at matched 6M budget -- its own NO-SWAP branch does NOT fire. DR-0 gate: det prog med 1.27 / fwd med 0.71m / slip med 2.58, sto prog 0.88 / slip 3.46, gait_valid 12/12, zero terms, video-clean six-leg cycling, floor met -- a healthy run, but the cpg_v1 sibling cw-cpg-ab6m-cpglib matches or edges it on every axis (det 1.28/0.76m/2.53; sto 0.96/3.10). Prediction-if-false branch confirmed: the 2M-era deficit was an early-training artifact, second independent pair agreeing with the 8M acq1b/budget2 read. Consequence recorded in the sibling's PASS: cpg_v1 stays co-equal-or-better as an AMP style source, no forced swap (n=6 noise edge). Checkup SUSPECT at 20:55 was stale -- the run had simply finished (W&B state=finished, 6.03M steps). No follow-up needed; cpg track GREEN, adoption question now double-answered.

