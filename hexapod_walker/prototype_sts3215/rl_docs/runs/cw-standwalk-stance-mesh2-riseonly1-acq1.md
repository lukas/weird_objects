# cw-standwalk-stance-mesh2-riseonly1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T08:08:31+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly1

**wandb_id**: fbdij62l

**hypothesis**: Plain English: riseonly1 (2M, rise=1.0 goal-mix, cur1 pricing) already learns a genuinely clean rise motion (video: smooth push-up to a level plant, not the full-mix's rearing/splay pathology) but every det episode still trips over_current holding the final pose (cur_p95 2.5-2.65A, right at the actuator ceiling) -- this is the SAME 'reward still declining, task partially right' shape as the holdonly1->holdonly1-acq1 continuation that already worked (hold survival rose 0.5@1M->6/6@2M while return fell only via accumulating hot charges). This arm continues riseonly1 +8M (10M total) testing whether budget alone teaches a lower-torque way to hold the risen pose instead of fighting gravity at the ceiling. Prediction-if-true: rise panel starts landing zero-over_current valid plants (>=4/6 det) by 10M, cur_p95 drops off the 2.5-2.65A ceiling toward the honest sub-1A band. Prediction-if-false: still pinned at the current ceiling with 0/6 valid plants at 10M -- budget is not the lever for rise; escalate to a mesh-specific torque/effort shaping term or a current-threshold recalibration (the ceiling may be reachable-but-tight for ANY good-faith full-weight stance on the 3.5kg body, not just a bad gait).

**gate**: Acquisition read at 10M total: pod_eval rise panel DR-0+own-DR(0.2) det+sto n=6+6. PASS: >=4/6 det AND >=4/6 sto reach valid plant (posture-strict, no over_current/tilt term) AND cur_p95<=1.5A on passing episodes. FAIL: still pinned at the current ceiling (cur_p95>=2.5A) with 0/6 or 1/6 valid plants -- budget is not the lever, next fork is torque/effort pricing or current-threshold recalibration.

