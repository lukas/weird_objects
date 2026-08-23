# cw-walkcurr-pf-fwd2-swingterm800

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T17:47:53+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd2-swing

**wandb_id**: 5vucihz7

**hypothesis**: Plain English: fwd2-swing's own strongest alternative hypothesis, tested directly -- if rung-1 PPO still can't discover walking even WITH the swing-flail bonus live, the suspect shifts from 'no reachable income' to 'the term_penalty=1200 catastrophe is so large relative to every other reward term that random exploration near any fall-risk pose gets stomped before swing/step income can accumulate.' Single lever vs fwd2-swing: term_penalty 1200->800 (still >2x the bank's own scripted-gait lifetime value of +346, so a real topple is still clearly the worst outcome, just less dominant against the noisy early-exploration variance). Prediction-if-true (term_penalty was the blocker): swing/step-event income rises within 1M where fwd2-swing was flat, panel shows real stepping. Prediction-if-false: freezes/flails identically to fwd2-swing (or fwd1) -- term_penalty magnitude is not the mechanism, look at init/noise/BC-kickstart instead. Runs in parallel with fwd2-swing (not sequential) per the operator's batch-grids-not-dribbles ruling -- these are the two live hypotheses for rung-1's freeze, both decidable from the same 2M budget.

**gate**: Same rung-1 gate as fwd1/fwd2-swing: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health read: step-event/swing rate rising with walk_prog > 0 but panel short = continue; flat/frozen like fwd1 = term_penalty-magnitude hypothesis refuted too, escalate to init/BC-kickstart dig-in.

**verdict**: Catastrophe size is irrelevant: dropping term_penalty 1200->800 on top of the swing bonus produced a byte-similar freeze trajectory to fwd2-swing (ep_rew -2370 vs -2500, ep_len 485 vs 493, walk_prog identically 0.0, swing income flat 0.064-0.088/step, same static crouch on video, det prog -0.00 / fwd 0.01m). The twin cleanly isolates the blocker: it is NOT the -1200 termination cliff and NOT missing income — it is the dense per-step charge flow (~-4.7/step, loadslip-dominated) that both makes freezing the best reachable policy and instantly punishes every exploratory motion. hardware-ready: no. Term lever closed (keep 1200 for later rungs); next mechanism is the discovery-phase charge ramp.

