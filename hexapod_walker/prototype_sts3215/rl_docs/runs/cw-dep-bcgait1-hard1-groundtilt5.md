# cw-dep-bcgait1-hard1-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-12T09:03:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: mwiqp3ui

**hardware_ready**: False

**hypothesis**: Plain English: the tall-walking champion has never trained on an actually-sloped floor (real concrete/rug is not perfectly flat), only the flat DR-0.35 default -- check whether its taller, less-splayed stance (less lateral base-of-support than the old crouch) still tolerates a 5deg slope the way the old crouch-lineage did. One variable vs cw-dep-bcgait1-hard1 itself (warm-started from ITS OWN checkpoint): dr.ground_tilt_deg 0 -> 5.0 (the dose already validated free on the plain walk lineage), reward.k_current=0 per the standing hardware-arm rule. Prediction-if-true: own-cfg det+sto gait_valid >=5/6, zero new falls, slip/m within bcgait1-hard1's own band, probe_tall_wall height stays >= -20mm -- slope composes free like on every other walk lineage. Prediction-if-false: the taller stance is measurably less slope-tolerant (falls or height re-drifts toward the crouch, seeking a wider lateral base) -- flag before hardware, and the crouch-vs-tall tradeoff needs an explicit slope budget.

**gate**: PASS if det+sto gait_valid >= 5/6, zero new termination class, slip/m det<=1.8 sto<=2.0 (bcgait1-hard1's own band) AND probe_tall_wall height stays >= -20mm (no crouch re-drift). FAIL if height re-drifts toward -70mm or gait_valid/falls regress vs the matched frozen bcgait1-hard1 baseline.

**verdict**: PASS: tall-walking champion (bcgait1-hard1) tolerates a 5deg sloped floor with zero degradation. gait_valid 6/6 all 4 passes (gate+owncfg, det+sto), zero falls/terminations, slip/m 1.39-1.54 (within 1.8/2.0 bar), probe_tall_wall height -8.5mm (matches parent's own -8.5..-9.8mm exactly, no crouch re-drift). Video clean six-leg cycling both modes, no flag-leg. Minor watch item: leg-yaw limit margin narrows further under this DR (0.16-1.22deg vs parent's matched-probe 1.37-1.90deg, one seed close to zero though still positive) -- same narrowing trend as -fric, slightly more pronounced; not gate-breaking but flag before stacking more DR axes on this stance.

