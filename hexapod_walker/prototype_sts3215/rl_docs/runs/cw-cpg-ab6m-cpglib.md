# cw-cpg-ab6m-cpglib

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:45:22+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**wandb_id**: il204xrp

**hypothesis**: Plain English: the treatment arm of a 2-run A/B asking whether the CPG-searched gait's ~15-20% deficit as an AMP style source is budget-limited or a real quality gap. This arm: the teacherfork-ab-cpgv1 recipe (cpg_v1.npz motion library, everything else identical to style05), byte-identical, at 6M instead of 2M. Prediction-if-true (budget-limited): det fwd/prog/gait margins close to parity with the teacher-lib sibling at 6M — the adoption fork stays open and the next step is a full walk-gate candidate trained on cpg_v1. Prediction-if-false: the ~15-20% det deficit persists or widens — adoption fork closes NO-SWAP with two independent data points. Strongest alternative: both arms ceiling the bars, margins indistinguishable — judge on slip (cpg_v1 clips measured lower slip/m than teacher_v2, so a slip win at behavior parity would still argue for adoption). cpg STATUS Next item (second adoption data point).

**gate**: Joint A/B read vs cw-cpg-ab6m-teachlib at 6M, DR-0 gate harness: ADOPTION-LIVE if this arm reaches parity or better on det fwd/prog/gait_valid with slip within +0.3 of the teacher arm; NO-SWAP if a >=10% det deficit persists; if both ceiling, judge on slip + gait margins. Absolute floor: det gait_valid >=5/6, net fwd med >=0.10m/15s, no crouch collapse, disc unsaturated.

