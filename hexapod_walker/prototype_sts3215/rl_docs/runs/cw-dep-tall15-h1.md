# cw-dep-tall15-h1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T21:04:43+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-dep-tall15

**wandb_id**: i7irdifw

**hardware_ready**: False

**hypothesis**: TALL LADDER T1 (budget): identical respec of cw-dep-tall15 (ref -15, warm from tall30, charge retained) at 6M instead of 2M. If the wall is step-bound, height_err_end drops toward <=8mm with more budget; if it is a workspace/stability preference, the error plateaus at ~29mm regardless of steps and the answer comes from T5s probe instead.

**gate**: PASS: height_err_end <=8mm at -15 ref, speed >=0.028, survived 1, slip <=1.8, no park. PARTIAL (still useful): err monotonically improving at 6M -> budget is the knob, schedule 10M. FAIL: err flat ~29mm -> wall is not step-bound; prioritize T5 probe then T2/T3.

**verdict**: FAIL vs its own pre-registered T1 gate: height_err_end is NOT improving with 3x budget (2M->6M). Training curve plateaus by ~1M steps (env/height_err_mm ~36-39mm) and stays flat the remaining 5M; harness eval median is actually WORSE than the 2M parent (gate 51-58mm, own-DR 57-58mm vs tall15's own 29mm), close to the tip1 ref-0 baseline (59.9mm) -- more budget did not close the gap, if anything execution got noisier (one spinning-stall episode fwd 0.20m slip 2.66, sto-owncfg gait_valid dropped to 4/6 with two sac episodes). No park/flag-leg/falls (terms 0, safety_flags 0, videos show honest six-leg paddle gait, just at the old crouch depth). Confirms: the -44mm wall is NOT step-bound. Per the run's own gate: prioritize T5 (kinematic/stability probe, not yet built) before spending more budget on T2/T3.

