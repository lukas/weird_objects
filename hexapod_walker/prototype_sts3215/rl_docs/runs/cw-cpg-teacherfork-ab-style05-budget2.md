# cw-cpg-teacherfork-ab-style05-budget2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T06:10:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: bh6qze4z

**hypothesis**: Matched control for cpg/STATUS.md Next item 3's second data point (see cw-cpg-teacherfork-ab-cpgv1-acq1b, a concurrent cycle's +6M continuation of the CPG-library arm): the teacherfork-ab A/B compared cpgv1 vs style05 at 2M each and found cpgv1 WORSE-BUT-WALKING (progress_ratio 0.99 vs 1.16, fwd dist 0.59 vs 0.69m/15s). Continuing cpgv1 alone without also continuing its teacher_v2 twin the same extra budget would confound 'CPG gap closes with budget' with 'style05 also keeps improving' -- this run applies the IDENTICAL +6M single-lever continuation (--init-from-source, same reward/env/motion-lib=teacher_v2.npz, no other change) to style05 so both sides of the A/B are read at the same 8M total budget.

**gate**: Read jointly against cw-cpg-teacherfork-ab-cpgv1-acq1b at the SAME 8M total budget. If style05-budget2's own progress_ratio/fwd-dist stay ~flat vs its own 2M numbers (1.16/0.69) while cpgv1-acq1b closes toward them, that is real gap-closing under budget. If style05-budget2 ALSO improves by a similar margin, the apparent 2M gap was partly an undertraining artifact on both sides, not a CPG-vs-teacher quality gap -- either reading answers the adoption question, no further swap decision without this pair.

