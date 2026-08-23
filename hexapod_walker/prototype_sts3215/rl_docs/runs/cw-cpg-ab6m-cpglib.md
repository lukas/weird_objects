# cw-cpg-ab6m-cpglib

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T20:45:22+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**wandb_id**: il204xrp

**hypothesis**: Plain English: the treatment arm of a 2-run A/B asking whether the CPG-searched gait's ~15-20% deficit as an AMP style source is budget-limited or a real quality gap. This arm: the teacherfork-ab-cpgv1 recipe (cpg_v1.npz motion library, everything else identical to style05), byte-identical, at 6M instead of 2M. Prediction-if-true (budget-limited): det fwd/prog/gait margins close to parity with the teacher-lib sibling at 6M — the adoption fork stays open and the next step is a full walk-gate candidate trained on cpg_v1. Prediction-if-false: the ~15-20% det deficit persists or widens — adoption fork closes NO-SWAP with two independent data points. Strongest alternative: both arms ceiling the bars, margins indistinguishable — judge on slip (cpg_v1 clips measured lower slip/m than teacher_v2, so a slip win at behavior parity would still argue for adoption). cpg STATUS Next item (second adoption data point).

**gate**: Joint A/B read vs cw-cpg-ab6m-teachlib at 6M, DR-0 gate harness: ADOPTION-LIVE if this arm reaches parity or better on det fwd/prog/gait_valid with slip within +0.3 of the teacher arm; NO-SWAP if a >=10% det deficit persists; if both ceiling, judge on slip + gait margins. Absolute floor: det gait_valid >=5/6, net fwd med >=0.10m/15s, no crouch collapse, disc unsaturated.

**verdict**: The CPG-searched gait library is confirmed as a full-parity AMP style source at matched 6M budget -- the joint A/B's ADOPTION-LIVE branch fires on this second independent data point. DR-0 gate: det prog med 1.28 / fwd med 0.76m / slip med 2.53 vs teacher-lib sibling cw-cpg-ab6m-teachlib 1.27 / 0.71m / 2.58; sto prog 0.96 / slip 3.10 vs 0.88 / 3.46 -- parity-or-better on every axis, slip LOWER in both modes, gait_valid 12/12, zero terms, video-clean six-leg cycling (no crouch, no flag leg). Floor met with wide margin. The 2M-era ~15-20% deficit (teacherfork-ab-cpgv1) was budget-limited, exactly as the earlier 8M pair (acq1b 1.35/0.77 vs budget2 1.21/0.71) suggested; two independent matched-budget pairs now agree in direction. Per the 08-23 adoption ruling: cpg_v1.npz stays a co-equal (now slightly favored -- lower slip both modes, better sto prog) AMP style source; individual det deltas still sit at n=6 noise edge so no forced teacher_v2 swap, but future amp arms may pre-register cpg_v1 with confidence. cpg track remains GREEN; this closes the second-adoption-data-point Next item.

