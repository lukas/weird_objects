# cw-cpg-teacherfork-ab-style05-budget2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T06:10:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: bh6qze4z

**hypothesis**: Matched control for cpg/STATUS.md Next item 3's second data point (see cw-cpg-teacherfork-ab-cpgv1-acq1b, a concurrent cycle's +6M continuation of the CPG-library arm): the teacherfork-ab A/B compared cpgv1 vs style05 at 2M each and found cpgv1 WORSE-BUT-WALKING (progress_ratio 0.99 vs 1.16, fwd dist 0.59 vs 0.69m/15s). Continuing cpgv1 alone without also continuing its teacher_v2 twin the same extra budget would confound 'CPG gap closes with budget' with 'style05 also keeps improving' -- this run applies the IDENTICAL +6M single-lever continuation (--init-from-source, same reward/env/motion-lib=teacher_v2.npz, no other change) to style05 so both sides of the A/B are read at the same 8M total budget.

**gate**: Read jointly against cw-cpg-teacherfork-ab-cpgv1-acq1b at the SAME 8M total budget. If style05-budget2's own progress_ratio/fwd-dist stay ~flat vs its own 2M numbers (1.16/0.69) while cpgv1-acq1b closes toward them, that is real gap-closing under budget. If style05-budget2 ALSO improves by a similar margin, the apparent 2M gap was partly an undertraining artifact on both sides, not a CPG-vs-teacher quality gap -- either reading answers the adoption question, no further swap decision without this pair.

**verdict**: The control run settles the CPG-vs-teacher adoption question: the scripted-teacher side does NOT improve much with the same extra budget, so the CPG library's catch-up is real. With the identical +6M continuation, style05's det numbers stayed ~flat vs its own 2M reading (progress_ratio med 1.16->1.21, fwd 0.69->0.71m — inside eval noise) while the CPG twin (cw-cpg-teacherfork-ab-cpgv1-acq1b, verdicted PASS this same cycle) moved 0.99->1.35 / 0.59->0.77m. Sto improved on BOTH sides (0.58->0.95 here, 0.58->0.90 there) — that axis was undertrained for both, no differentiator. Joint answer per this run's own pre-registered gate: the 2M det gap was CPG-side undertraining, and at matched 8M budget cpg_v1.npz is AT LEAST CO-EQUAL to teacher_v2 as an AMP style source (det deltas favor CPG but sit at the edge of n=6 noise; sto slip slightly favors teacher, 3.15 vs 3.57 — no honest superiority claim either way). Gait_valid 6/6 det+sto, zero terminations, video clean (upright six-leg cycling). Adoption decision: cpg_v1.npz is promoted to a co-equal alternative AMP style source — no forced teacher_v2 swap (no evidence of superiority), no joystick slip-bar recalibration; future amp arms may pre-register either library. This completes the tracks.json cpg gate's 'teacher adoption is A/B-tested' clause.

