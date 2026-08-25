# cw-standwalk-stance-mesh2-loweronly1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T08:18:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-loweronly1

**hypothesis**: Plain English: loweronly1 (2M, lower=1.0 goal-mix, cur1 pricing) hasn't converged: det episodes hold a too-tall stance and trip current, but several sto episodes already get close (near-target height, no termination) at just 2M -- suggesting the policy is on the way to a working low-current descent, not stuck in a pathological basin (video shows a genuine standing posture, not the full-mix's rearing/splay). This arm continues loweronly1 +8M (10M total) testing whether budget alone lets it commit to the actual lower target instead of parking tall. Prediction-if-true: lower panel starts landing zero-over_current valid plants (>=4/6 det) by 10M. Prediction-if-false: still pinned tall/hot at 10M -- budget is not the lever; escalate to goal.lower_height_mm mesh recalibration or a torque/effort shaping term, per the pre-registered fallback fork in STATUS.md.

**gate**: Acquisition read at 10M total: pod_eval lower panel DR-0+own-DR(0.2) det+sto n=6+6. PASS: >=4/6 det AND >=4/6 sto reach valid plant (posture-strict, no over_current/tilt term) AND cur_p95<=1.5A on passing episodes. FAIL: still pinned at the current ceiling with 0/6 or 1/6 valid plants -- budget is not the lever, next fork is goal.lower_height_mm recalibration or torque/effort pricing.

