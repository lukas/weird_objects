# cw-dep-bcgait1-hard1-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-12T09:04:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-dep-bcgait1-hard1

**hypothesis**: Plain English: take the new tall-walking champion (the first one that stopped crouching and leg-splaying) and check whether it still walks fine on floors with more or less grip than the sim default -- exactly the DR/tipped-start retention panel every prior dep-line champion (vref1-r1, tip1) went through before being trusted, and bcgait1-hard1 has never been exposed to it. One variable vs cw-dep-bcgait1-hard1 itself (warm-started from ITS OWN checkpoint, not the 2M parent): dr.friction_scale 1.0 (implicit) -> 0.4-1.6x range (the same envelope proven free on the vref1-r1/tip1 lineage), reward.k_current=0 per the standing hardware-arm rule. Prediction-if-true: own-cfg det+sto gait_valid >=5/6, zero new falls, slip/m within bcgait1-hard1's own band, probe_tall_wall height still in the -20..+6mm band -- friction composes free like it did on the crouch-gait lineage, one less unknown before this checkpoint is a Gate-0 candidate. Prediction-if-false: the taller stance (less base-of-support redundancy than the old crouch) is more friction-sensitive -- height re-drifts toward the crouch OR gait_valid regresses -- flag before any further hardening.

**gate**: PASS if det+sto gait_valid >= 5/6, zero new termination class, slip/m det<=1.8 sto<=2.0 (bcgait1-hard1's own band) AND probe_tall_wall height stays >= -20mm (no crouch re-drift). FAIL if height re-drifts toward -70mm or gait_valid/falls regress vs the matched frozen bcgait1-hard1 baseline.

