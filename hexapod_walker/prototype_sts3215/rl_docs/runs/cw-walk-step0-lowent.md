# cw-walk-step0-lowent

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T00:57:41+00:00

**pod**: hexapod-sweep-s6

**steps**: 4000000

**parent**: ppo_goal_cw_walk_step0.zip (md5 ea1685a4)

**wandb_id**: vkrvueqg

**checkpoint**: rl_move/sim/policies/ppo_goal_cw_walk_step0_lowent.zip

**hypothesis**: Consolidation arm, one variable vs c1 (same parent ppo_goal_cw_walk_step0.zip md5 ea1685a4, same cfg/seed): ent_coef 0.01 to 0.001. The step0 plateau (rew 554-610 since 3.1M) plus monotone std runaway (1.0 to 2.30 to 3.21 across lineage) says the entropy bonus outruns the saturated task gradient. If-true: std anneals toward 1.0 or lower, ep_rew_mean clears 620 sustained, harness det keeps gait_valid 6/6 and forward 0.10m-plus with slip no worse than parent 0.93m. If-false branches: (a) std stays at 2.0-plus = entropy not the driver; (b) std falls but rew collapses under 500 with step events dying = gait was noise-dither, reward mispriced; (c) std falls, rew flat 554-610 = reward ceiling (overspeed/step-credit caps), pointing at pricing not exploration. Strongest alt is (c); the three branches separate exploration-side from reward-side causes before any reward surgery. no-canary per operator canary note (spurious rise_flat protection truncated c1). Runs beside operator's identical-config c2 as a clean A/B on the entropy hypothesis. Snapshot 3a52183.

**gate**: DR 0 harness, 6 eps/mode det AND sto: gait_valid 12/12 AND forward dist at least 0.10m in 12/12 AND at least 2 swings/leg 12/12 AND det slip mean at most 0.93m AND final train/std at most 1.2 AND ep_rew_mean last 0.5M at least 620

**verdict**: FAIL on gate std clause only (train/std final 2.075 > 1.2); other 5 clauses PASS: gait_valid 24/24 (two evals), forward 0.23-0.47m, min 3 swings/leg, det slip mean 0.746/0.717 vs 0.93 baseline (non-overlapping ranges), ep_rew_mean last0.5M 681 >= 620. sto success pooled 10/12 vs step0 1/6 (outside noise). Zero falls/flags. NEW WALK-LINE CHAMPION (beats step0). NOT hardware-ready: DR 0, skating ~0.7m/ep persists, speed commands ignored at det. Hypothesis SUPPORTED on plateau-driver claim (clean A/B vs c2), REFUTED on std-anneal-to-1.2 sub-claim: std floors at ~2.07.

