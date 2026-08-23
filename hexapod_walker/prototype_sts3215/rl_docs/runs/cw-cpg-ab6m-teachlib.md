# cw-cpg-ab6m-teachlib

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T20:43:33+00:00

**pod**: hexapod-mjx-train-3

**steps**: 6000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**hypothesis**: Plain English: the control arm of a 2-run A/B asking whether the CPG-searched gait's ~15-20% deficit as an AMP style source (cw-cpg-teacherfork-ab-cpgv1, WORSE-BUT-WALKING at 2M) persists when both style sources get 3x budget, or was an early-training artifact. This arm: the already-PASSED style05 recipe (teacher_v2 library), byte-identical, at 6M instead of 2M. Prediction-if-true (deficit is real): this arm's det margins stay >=10% ahead of the cpg_v1 sibling at 6M. Prediction-if-false: the sibling matches/inverts. Strongest alternative: both arms ceiling the bars at 6M and margins become indistinguishable — then judge the pair on slip and gait-quality margins. cpg STATUS Next item (second adoption data point), funded now that GPUs are free.

**gate**: Joint A/B read vs cw-cpg-ab6m-cpglib at 6M, DR-0 gate harness: compare det/sto gait_valid, fwd travel med, prog med, slip med. NO-SWAP (adoption fork closes) if this teacher arm leads det fwd/prog by >=10% with slip not worse; ADOPTION-LIVE if cpg arm reaches parity or better on det fwd/prog/gait with slip within +0.3; if both ceiling, judge on slip + gait margins. Absolute floor for either arm: det gait_valid >=5/6, net fwd med >=0.10m/15s, no crouch collapse.

