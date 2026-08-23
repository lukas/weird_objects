# cw-cpg-teacherfork-ab-cpgv1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T06:02:07+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**hypothesis**: Plain English: cpgv1's own 2M discovery run was still improving when it stopped (reward quarters 38.9/101.9/157.4/185.8, still rising at the end) and it landed WORSE-BUT-WALKING vs the scripted-teacher-library style05 champion (det progress_ratio med 0.99 vs 1.16, fwd dist med 0.59 vs 0.69m/15s; sto already matched/better). Per the 08-21 ruling (bad-but-improving eval + rising reward = continue, don't verdict a lineage dead), this single-lever continuation (--init-from-source, same reward/env/motion-lib, +6M steps, no other change) tests whether the CPG-motion-library gap closes with more budget or is a real ceiling of this exact clip set.

**gate**: DR-0 gate (own harness cfg, matched to the source run) det progress_ratio med and fwd_dist med vs this SAME source run's own recorded numbers (0.99 / 0.59m) and vs style05's target (1.16 / 0.69m), zero falls/sacrificed legs preserved (currently 6/6 both modes). CLOSES-GAP = det progress_ratio med >= 1.10 AND fwd dist med >= 0.65m (within ~10% of style05, clearly better than the 2M reading) with gait_valid/falls unchanged. PARTIAL = meaningfully better than 0.99/0.59 but short of that band. CEILING = no better than 0.99/0.59 within noise despite 3x the budget -- names the CPG clip set itself (not undertraining) as the reason it underperforms teacher_v2 as an AMP style source, closing this side-question without further budget.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

