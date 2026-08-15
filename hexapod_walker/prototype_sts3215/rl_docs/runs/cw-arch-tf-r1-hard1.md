# cw-arch-tf-r1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T13:53:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**wandb_id**: t79e5h3e

**hypothesis**: Does the small causal-transformer policy trunk (2 layers, d_model 128, 4 heads, over the 16-frame history window) learn to WALK as well as the hist16 MLP champion (cw-arch-hist16-r7: det gait_valid 6/6, zero falls, prog_ratio med 1.08-1.17 at 40M) when given the SAME 40M-step budget, identical joint_walk recipe/reward/DR? One variable changed vs r7: policy trunk (flatten-MLP -> causal self-attention). The 2M discovery rung (cw-arch-tf-r1b) already showed the stack boots/trains cleanly with no leg-sacrifice cheat locked in -- this run answers whether the mechanism actually GROWS a working gait at the budget r7 needed, or whether attention over a short window is no better/worse than the flatten baseline.

**gate**: PASS = det gait_valid 6/6 (own-cfg DR0.5), zero sacrificed legs, zero det falls, prog_ratio med >=0.85 (r7's gate/threshold), video shows all six legs cycling swing/stance with no flag-leg/dragging -- matching or beating r7's own numbers. FAIL = crash/NaN, leg-sacrifice/paddle lock-in in det, or gait_valid <6/6 with a clear pathology. If zero gait emerges by 40M despite no crash: verdict is 'transformer trunk undertrained/undersized at this budget', NOT proof transformers can't walk (r7 itself needed the full 40M).

