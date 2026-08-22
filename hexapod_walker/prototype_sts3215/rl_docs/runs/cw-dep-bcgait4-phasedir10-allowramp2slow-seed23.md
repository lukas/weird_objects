# cw-dep-bcgait4-phasedir10-allowramp2slow-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T16:28:01+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir10-allowramp-seed23

**wandb_id**: f4osp5m6

**hardware_ready**: no

**hypothesis**: Plain English: loosening the foot-drag price early helped one training seed but hurt the other two; this arm tests whether those hurt seeds just need the loose allowance held LONGER before the strict price kicks in. Single change vs cw-dep-bcgait4-phasedir10-allowramp-seed23 (FAIL, 0.792x/1.217x — worse than its own no-ramp longrun23 baseline 0.818x/1.175x): reward.drag_stance_allow_ramp_steps 1.2M->2.4M (ramp now ends 1.2M AFTER the log-std anneal ends, giving the policy a post-anneal grace window to settle an honest gait at low noise before the det-calibrated 24mm allowance bites; ramp_mm stays 48). This is the 'slower ramp' half of the untried branch pre-registered in joystick STATUS Next item 3; sibling 'wider start' arms test the 64mm dose at the original schedule. Grid launched as one batch per operator 08-22. (Name: allowramp2slow — the allowramp2-slow name's snapshot tag was consumed by a phase-cap REFUSED attempt, same disambiguation precedent as allowramp-a.)

**gate**: Watcher DR-0 gate report, det mode, clone-relative rung-A: PASS = progress >=0.9x clone AND slip <=1.2x clone, zero falls, gait_valid 6/6. Compare against BOTH the seed's no-ramp baseline (longrun23: 0.818x/1.175x) and its 1.2M/48mm ramp reading (0.792x/1.217x). Prediction-if-true: both axes improve vs both baselines (schedule was the defect, not the ramp mechanism). Prediction-if-false: flat/worse vs the no-ramp baseline — the slower schedule does not rescue this seed; if the wide-start sibling also fails, the ramp lever class is CLOSED for failing seeds and the track redirects to the matched-timing stance-slip mechanism (STATUS item b) / whatever the stotight sto-grid triage indicates.

**verdict**: FAIL on pre-registered prediction-if-false: extending the drag-allow ramp 1.2M->2.4M made seed23 WORSE on both clone-relative axes (det prog 0.65x, slip 1.70x clone) vs no-ramp (0.818x/1.175x) AND 48mm/1.2M ramp (0.792x/1.217x). Zero falls, gait 6/6, clean video. Slower schedule refuted on this seed.

