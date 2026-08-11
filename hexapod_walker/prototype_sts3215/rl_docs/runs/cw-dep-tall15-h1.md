# cw-dep-tall15-h1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T21:04:43+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-dep-tall15

**wandb_id**: i7irdifw

**hypothesis**: TALL LADDER T1 (budget): identical respec of cw-dep-tall15 (ref -15, warm from tall30, charge retained) at 6M instead of 2M. If the wall is step-bound, height_err_end drops toward <=8mm with more budget; if it is a workspace/stability preference, the error plateaus at ~29mm regardless of steps and the answer comes from T5s probe instead.

**gate**: PASS: height_err_end <=8mm at -15 ref, speed >=0.028, survived 1, slip <=1.8, no park. PARTIAL (still useful): err monotonically improving at 6M -> budget is the knob, schedule 10M. FAIL: err flat ~29mm -> wall is not step-bound; prioritize T5 probe then T2/T3.

