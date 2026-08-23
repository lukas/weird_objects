# cw-cpg-teacherfork-ab-cpgv1-6m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T12:03:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**hypothesis**: Plain English: we are deciding whether the gait found by direct CPG search is as good a style example for AMP training as the hand-scripted teacher; the first A/B at 2M showed the CPG library walking but ~15-20% softer on det margins (prog 0.99 vs 1.16, slip 2.46 vs 1.88, fwd 0.59 vs 0.69m) -- this arm re-runs the SAME cpgv1 recipe at 3x budget (6M, only --steps changed) with a teacher_v2 twin at matched budget, to tell whether that det gap is a fixed property of the style target or an early-training transient. Prediction-if-true (gap is transient): cpgv1-6m closes to parity with teacher-6m (det prog >=0.95x of the twin, det slip <=1.15x, fwd within 0.1m). Prediction-if-false (gap is real): gap persists or widens at matched budget -> teacher_v2 confirmed the better style source; adoption fork closes with no swap, cpg_v1 stays a diversity resource. Strongest alternative: 6M of this fragile minimal-reward discovery recipe degrades BOTH arms (crouch collapse or gait_valid loss) -- then budget, not library, dominates and the 2M reads stand as the honest comparison.

**gate**: Read JOINTLY with cw-cpg-teacherfork-ab-teacher6m on the DR-0 gate harness (walk det+sto n=6 each). PARITY/PASS-for-adoption-question = gait_valid 6/6 det+sto, no crouch collapse (env/height_err_mm stays <40mm), det prog med >=0.95x the teacher6m twin AND det slip med <=1.15x twin. GAP-CONFIRMED = walking clean but det prog <0.95x or slip >1.15x twin (adoption fork closes, no teacher swap). INVALID/budget-dominates = either arm loses gait_valid or collapses to crouch -- 2M remains the comparison point, no further budget arms.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

