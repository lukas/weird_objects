# cw-standwalk-stance-mesh2-loweronly1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:18:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-loweronly1

**wandb_id**: gd3dy7lc

**hypothesis**: Plain English: loweronly1 (2M, lower=1.0 goal-mix, cur1 pricing) hasn't converged: det episodes hold a too-tall stance and trip current, but several sto episodes already get close (near-target height, no termination) at just 2M -- suggesting the policy is on the way to a working low-current descent, not stuck in a pathological basin (video shows a genuine standing posture, not the full-mix's rearing/splay). This arm continues loweronly1 +8M (10M total) testing whether budget alone lets it commit to the actual lower target instead of parking tall. Prediction-if-true: lower panel starts landing zero-over_current valid plants (>=4/6 det) by 10M. Prediction-if-false: still pinned tall/hot at 10M -- budget is not the lever; escalate to goal.lower_height_mm mesh recalibration or a torque/effort shaping term, per the pre-registered fallback fork in STATUS.md.

**gate**: Acquisition read at 10M total: pod_eval lower panel DR-0+own-DR(0.2) det+sto n=6+6. PASS: >=4/6 det AND >=4/6 sto reach valid plant (posture-strict, no over_current/tilt term) AND cur_p95<=1.5A on passing episodes. FAIL: still pinned at the current ceiling with 0/6 or 1/6 valid plants -- budget is not the lever, next fork is goal.lower_height_mm recalibration or torque/effort pricing.

**verdict**: FAIL, third independent confirmation that budget-alone continuation of an isolated-mode 2M checkpoint makes things WORSE, not better (after holdonly1-acq1 and riseonly1-acq1). At 2M the parent stood too TALL (current-tripped holding an upright pose, never falling). At 10M (+8M same recipe/pricing): 0/6 det, 0/6 sto, now ALL tilt_roll/tilt_pitch (genuine falls) instead of a stable-but-hot stand -- the extra budget traded a stable pose for an unstable one, exactly like riseonly1-acq1. Reward also non-monotone/declining (-32.5/-184.9/-297.9/-173.6). CLOSES the budget-alone lever for isolated-mode continuations across all 3 modes tested (hold/rise/lower) -- do not launch a 4th same-shape continuation. Rung-3 needs a structural fix: warm-start rise/lower continuations from an honest six-foot HOLD checkpoint (per the concurrent holdload1min line, once it produces one) rather than continuing an isolated-mode checkpoint on more steps, or a torque/effort shaping term / goal.lower_height_mm recalibration for lower specifically. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_loweronly1_acq1_{gate,owncfg}/, W&B gd3dy7lc.

