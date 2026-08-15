# cw-arch-tf-r1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-15T13:53:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**wandb_id**: t79e5h3e

**hardware_ready**: False

**hypothesis**: Does the small causal-transformer policy trunk (2 layers, d_model 128, 4 heads, over the 16-frame history window) learn to WALK as well as the hist16 MLP champion (cw-arch-hist16-r7: det gait_valid 6/6, zero falls, prog_ratio med 1.08-1.17 at 40M) when given the SAME 40M-step budget, identical joint_walk recipe/reward/DR? One variable changed vs r7: policy trunk (flatten-MLP -> causal self-attention). The 2M discovery rung (cw-arch-tf-r1b) already showed the stack boots/trains cleanly with no leg-sacrifice cheat locked in -- this run answers whether the mechanism actually GROWS a working gait at the budget r7 needed, or whether attention over a short window is no better/worse than the flatten baseline.

**gate**: PASS = det gait_valid 6/6 (own-cfg DR0.5), zero sacrificed legs, zero det falls, prog_ratio med >=0.85 (r7's gate/threshold), video shows all six legs cycling swing/stance with no flag-leg/dragging -- matching or beating r7's own numbers. FAIL = crash/NaN, leg-sacrifice/paddle lock-in in det, or gait_valid <6/6 with a clear pathology. If zero gait emerges by 40M despite no crash: verdict is 'transformer trunk undertrained/undersized at this budget', NOT proof transformers can't walk (r7 itself needed the full 40M).

**verdict**: PASS -- the causal-transformer policy trunk (2 layers, d_model 128, 4 heads, over the 16-frame history window) GROWS a full walking gait at 40M steps, matching the hist16-MLP champion (r7) at budget parity. DR0 gate: det+sto gait_valid 6/6, zero sacrificed legs, zero falls (terms 0), prog_ratio med 1.14 det / 1.08 sto (>=0.85 bar); own-cfg DR0.5: same 6/6/6/6, 0 term, prog med 1.10/1.00. Roll behavior clean (roll_class clean/recovered every episode, peak 2.0-7.8deg, tail 0.4-1.9deg) -- NOT the universal 13-27deg takeoff-roll-transient fall pattern, and a clear improvement over this same arm's own 2M canary (which fell via roll on every det episode). Video (10-frame strips, all 12 gate eps + 12 owncfg eps) shows six feet cycling through changing contact patterns over the 15s clip, no flag leg, no static/parked gait. Slip is elevated vs r7's freshest read (med 1.60 det/1.53 sto vs r7 c4's 0.95/1.02) but within the range r7's OWN verdict already flagged as 'elevated vs contract-line champions, not gated here' (1.3-1.6) -- not a new regression, same documented economy gap this whole architecture line carries. ASSUMPTION (operator to review): no bulk_session_eval cohort was run before this verdict -- the checkpoint is obs-incompatible with eval_session/bulk_session_eval (1152 vs 72, confirmed INCOMPATIBLE by the pre-staged session eval) because this architecture line, like r7 itself, is not on the deployment-exact obs contract; treated the pre-registered 6-episode-per-mode harness gate (identical convention r7 was verdicted on) as the applicable evidence bar for this obs family, same precedent as every other arch-line walk verdict in SKILLS.md.

