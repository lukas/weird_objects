# cw-dep-bcgait1-hard1-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-12T09:01:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-dep-bcgait1-hard1

**hypothesis**: Plain English: the tall-walking champion has never trained on an actually-sloped floor (real concrete/rug is not perfectly flat), only the flat DR-0.35 default -- check whether its taller, less-splayed stance (less lateral base-of-support than the old crouch) still tolerates a 5deg slope the way the old crouch-lineage did. One variable vs cw-dep-bcgait1-hard1 itself (warm-started from ITS OWN checkpoint): dr.ground_tilt_deg 0 -> 5.0 (the dose already validated free on the plain walk lineage), reward.k_current=0 per the standing hardware-arm rule. Prediction-if-true: own-cfg det+sto gait_valid >=5/6, zero new falls, slip/m within bcgait1-hard1's own band, probe_tall_wall height stays >= -20mm -- slope composes free like on every other walk lineage. Prediction-if-false: the taller stance is measurably less slope-tolerant (falls or height re-drifts toward the crouch, seeking a wider lateral base) -- flag before hardware, and the crouch-vs-tall tradeoff needs an explicit slope budget.

**gate**: PASS if det+sto gait_valid >= 5/6, zero new termination class, slip/m det<=1.8 sto<=2.0 (bcgait1-hard1's own band) AND probe_tall_wall height stays >= -20mm (no crouch re-drift). FAIL if height re-drifts toward -70mm or gait_valid/falls regress vs the matched frozen bcgait1-hard1 baseline.

**refused_reason**: hexapod-mjx-train-1 code marker fa562e29e4a8b4d4eaa424267712d6d700b8fa2a-dirty != local HEAD fa562e29e4a8b4d4eaa424267712d6d700b8fa2a. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

