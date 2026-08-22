# cw-amp-m2-freeprog-term400-stagecurric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T15:36:08+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**wandb_id**: 6movvd1y

**hypothesis**: Plain English: does a from-scratch policy stop shuffling in place if it first learns to walk forward/back only, before the harder turning+stopping+resampling joystick complexity is added? Both term400 fix-pair arms (noamp AND stdanneal) confirmed a genuine reward-shape/local-optimum problem, not exploration noise: median fwd travel stayed 0.005-0.032 m/15s (bar 0.10) while env/reward_walk_freeprog_pen sat flat regardless of std. Neither arm was ever exposed to a SIMPLE version of the task -- both trained on the full stress_mix envelope (resample every 1.75s +/- jitter, yaw commands, 15% stop fraction) from step 0, the exact failure pattern this codebase already root-caused and fixed on the joystick track's steer2-stagecurric1 lineage ('full-mix exposure from step 0 is not enough' -- that PASSED canary once staged). Single change vs cw-amp-m2-freeprog-term400-noamp: goal.walk_cmd_stage=0 ramped 0->2 via the existing sched.* engine (sched.key=goal.walk_cmd_stage, v0=0, v1=2, t0=0, t1=1.2M of the 2M budget -- forward/back-only first, then headings/circles/squares, full family+jitter only by 60%), everything else byte-identical (still no AMP, still term_penalty=400). Prediction-if-true: median fwd travel rises toward/past 0.10 m/15s and env/reward_walk_freeprog_pen actually improves over the run (not flat). Prediction-if-false: shuffle persists even on the easy forward/back-only stage0 sub-problem -- proving the defect is in walk_freeprog_score's per-tick shape itself (e.g. the SLIPWALK bank's own scripted 'stall' twin, which under this exact reward stack already scores -143 vs a real gait's +417, so the theoretical ranking is right but from-scratch PPO cannot reach the 'creep' region of policy space even on the simplest sub-task), pointing at motion-library/AMP-style guidance or a demonstration anchor next, not further envelope curricula.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-noamp (its own twin at the SAME final full-mix envelope) at matched budget: PASS = median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND zero/rare terminations held AND video shows six legs cycling with net displacement (not in-place shuffling). Secondary read regardless of PASS/FAIL: does env/reward_walk_freeprog_pen actually improve over the 4 quarters (unlike noamp's and stdanneal's flat traces) -- decides whether envelope staging is a viable lever for this reward family before any reward-shape patch (e.g. an explicit net-displacement floor).

**verdict**: FAIL — staged-command curriculum (sched goal.walk_cmd_stage 0->2 over 1.2M, verified ramping) did NOT unlock locomotion: det fwd travel med 0.02m (bar 0.10m, WORSE than noamp twin's 0.026-0.032m), gait_valid 0/6 det with sacrificed rear legs [3,5], video = sprawled statue. Reward/eval agreement: per-tick reward improved only during survival learning (-4.2->-2.7/tick by 0.6M) then FLAT; freeprog_pen flat -1.5/tick; eval flat/bad = stuck mechanism per 08-22 ruling, NOT undertraining. Even the stage-0 forward-only sub-problem never left the statue basin -> staging/exploration lever class CLOSED for this family; pre-registered fallback (net-displacement-floor reward patch, bank-first) is next.

