# cw-walkcurr-pf-fwd2-swingterm800

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T17:47:53+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd2-swing

**wandb_id**: 5vucihz7

**hypothesis**: Plain English: fwd2-swing's own strongest alternative hypothesis, tested directly -- if rung-1 PPO still can't discover walking even WITH the swing-flail bonus live, the suspect shifts from 'no reachable income' to 'the term_penalty=1200 catastrophe is so large relative to every other reward term that random exploration near any fall-risk pose gets stomped before swing/step income can accumulate.' Single lever vs fwd2-swing: term_penalty 1200->800 (still >2x the bank's own scripted-gait lifetime value of +346, so a real topple is still clearly the worst outcome, just less dominant against the noisy early-exploration variance). Prediction-if-true (term_penalty was the blocker): swing/step-event income rises within 1M where fwd2-swing was flat, panel shows real stepping. Prediction-if-false: freezes/flails identically to fwd2-swing (or fwd1) -- term_penalty magnitude is not the mechanism, look at init/noise/BC-kickstart instead. Runs in parallel with fwd2-swing (not sequential) per the operator's batch-grids-not-dribbles ruling -- these are the two live hypotheses for rung-1's freeze, both decidable from the same 2M budget.

**gate**: Same rung-1 gate as fwd1/fwd2-swing: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health read: step-event/swing rate rising with walk_prog > 0 but panel short = continue; flat/frozen like fwd1 = term_penalty-magnitude hypothesis refuted too, escalate to init/BC-kickstart dig-in.

