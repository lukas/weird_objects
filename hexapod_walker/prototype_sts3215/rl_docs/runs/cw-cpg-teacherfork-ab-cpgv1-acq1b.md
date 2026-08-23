# cw-cpg-teacherfork-ab-cpgv1-acq1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T06:03:00+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**wandb_id**: tccgmsx6

**hypothesis**: Plain English: cpgv1's own 2M discovery run was still improving when it stopped (reward quarters 38.9/101.9/157.4/185.8, still rising at the end) and it landed WORSE-BUT-WALKING vs the scripted-teacher-library style05 champion (det progress_ratio med 0.99 vs 1.16, fwd dist med 0.59 vs 0.69m/15s; sto already matched/better). Per the 08-21 ruling (bad-but-improving eval + rising reward = continue, don't verdict a lineage dead), this single-lever continuation (--init-from-source, same reward/env/motion-lib, +6M steps, no other change) tests whether the CPG-motion-library gap closes with more budget or is a real ceiling of this exact clip set.

**gate**: DR-0 gate (own harness cfg, matched to the source run) det progress_ratio med and fwd_dist med vs this SAME source run's own recorded numbers (0.99 / 0.59m) and vs style05's target (1.16 / 0.69m), zero falls/sacrificed legs preserved (currently 6/6 both modes). CLOSES-GAP = det progress_ratio med >= 1.10 AND fwd dist med >= 0.65m (within ~10% of style05, clearly better than the 2M reading) with gait_valid/falls unchanged. PARTIAL = meaningfully better than 0.99/0.59 but short of that band. CEILING = no better than 0.99/0.59 within noise despite 3x the budget -- names the CPG clip set itself (not undertraining) as the reason it underperforms teacher_v2 as an AMP style source, closing this side-question without further budget.

**verdict**: The CPG motion library's weak 2M showing was undertraining, not a defect of the clip set: with +6M more steps the same run now beats the scripted-teacher style05 champion's own numbers on the DR-0 gate. CLOSES-GAP band met decisively — det progress_ratio med 1.35 (gate >=1.10; parent-at-2M 0.99; style05 target 1.16) and fwd dist med 0.77m (gate >=0.65m; parent 0.59; style05 0.69), sto prog med 0.90 vs parent's 0.58, gait_valid 6/6 det+sto, zero terminations, video-reviewed frame strips clean (upright six-leg cycling, no flag legs/crouch/drag). Reward and eval rose together (quarters 123.8->243.8->271.3->302.5) — aligned, textbook 08-21 continuation payoff. Residual: sto slip med 3.57 is above the 1.4-2.9 teacher det band (improved vs the budget2 sibling's 4.14); det slip 2.49 is in-band. Next: the adoption A/B is NOT decided by this run alone — the matched control cw-cpg-teacherfork-ab-style05-budget2 (+6M on the style05 side) is still training; the joint read at matched 8M budget decides whether cpg_v1.npz becomes a first-class AMP style source alternative or teacher_v2 also moves up with budget.

