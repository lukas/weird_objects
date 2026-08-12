# cw-dep-bcgait1-hard1-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-12T09:04:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: r13ga9s9

**hardware_ready**: False

**hypothesis**: Plain English: take the new tall-walking champion (the first one that stopped crouching and leg-splaying) and check whether it still walks fine on floors with more or less grip than the sim default -- exactly the DR/tipped-start retention panel every prior dep-line champion (vref1-r1, tip1) went through before being trusted, and bcgait1-hard1 has never been exposed to it. One variable vs cw-dep-bcgait1-hard1 itself (warm-started from ITS OWN checkpoint, not the 2M parent): dr.friction_scale 1.0 (implicit) -> 0.4-1.6x range (the same envelope proven free on the vref1-r1/tip1 lineage), reward.k_current=0 per the standing hardware-arm rule. Prediction-if-true: own-cfg det+sto gait_valid >=5/6, zero new falls, slip/m within bcgait1-hard1's own band, probe_tall_wall height still in the -20..+6mm band -- friction composes free like it did on the crouch-gait lineage, one less unknown before this checkpoint is a Gate-0 candidate. Prediction-if-false: the taller stance (less base-of-support redundancy than the old crouch) is more friction-sensitive -- height re-drifts toward the crouch OR gait_valid regresses -- flag before any further hardening.

**gate**: PASS if det+sto gait_valid >= 5/6, zero new termination class, slip/m det<=1.8 sto<=2.0 (bcgait1-hard1's own band) AND probe_tall_wall height stays >= -20mm (no crouch re-drift). FAIL if height re-drifts toward -70mm or gait_valid/falls regress vs the matched frozen bcgait1-hard1 baseline.

**verdict**: PASS: tall-walking champion (bcgait1-hard1) tolerates 0.4-1.6x friction DR with zero degradation. gait_valid 6/6 all 4 passes (gate+owncfg, det+sto), zero falls/terminations, slip/m 1.30-1.52 (within 1.8/2.0 bar, matches parent's own 1.32-1.33), probe_tall_wall height -7.9mm (vs parent -12.7mm on a matched re-probe -- actually taller, no crouch re-drift). Video clean six-leg cycling both modes, no flag-leg. Minor watch item: leg-yaw limit margin narrows slightly under this DR (0.82-1.45deg vs parent's matched-probe 1.37-1.90deg) -- still positive/not pinned, continuing the 17->2.3->~1deg narrowing trend since bc1-hard1's own hardening; not gate-breaking here but worth tracking if further DR axes stack.

