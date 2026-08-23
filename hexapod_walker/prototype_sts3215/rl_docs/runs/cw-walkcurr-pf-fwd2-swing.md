# cw-walkcurr-pf-fwd2-swing

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T17:44:08+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd1

**wandb_id**: diomgtc8

**hypothesis**: Plain English: rung-1 PPO froze because no reward income is reachable by a random-init policy (step events need 10mm along-command swings, freeprog needs real velocity) — this arm adds a direction-free swing bonus (reward.k_walk_swing=1.0, pays ANY completed >=15mm swing) so flailing itself earns, giving exploration a monotone path flail -> leg-cycling -> commanded walking. Single lever on the exact fwd1 recipe, fresh init (prior-free track; fwd1's checkpoint IS the freeze attractor). Bank-proven under exact cfg incl. the pre-registered shuffle farming attack (shuffle -430, negative; gait +409.9 on top). Prediction-if-true: env/reward_step_event and swing income rise within the first 1M (vs fwd1's flat 0.02/step), walk_prog leaves 0.0, eval panel shows real stepping. Prediction-if-false: (a) freeze again with swing rate flat = swing credit still unreachable -> exploration fix moves to init/noise, or (b) in-place shuffle farming (swing income up, prog 0) = the bank's shuffle attack realized in-policy -> reprice shuffle. Strongest alternative: term_penalty catastrophe dominance suppresses exploration regardless of income (the swingterm800 twin tests exactly that).

**gate**: Rung-1 gate (same as fwd1): C-env det fixed-forward panel — zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health read (08-21 ruling): step-event/swing rate rising with walk_prog > 0 but panel short = continue; swing rate flat ~0.02/step at 2M = lever refuted, no same-recipe continuation.

