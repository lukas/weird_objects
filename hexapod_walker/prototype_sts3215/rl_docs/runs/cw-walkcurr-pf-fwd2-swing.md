# cw-walkcurr-pf-fwd2-swing

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED_FAIL

**created**: 2026-08-23T17:44:08+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd1

**wandb_id**: diomgtc8

**hypothesis**: Plain English: rung-1 PPO froze because no reward income is reachable by a random-init policy (step events need 10mm along-command swings, freeprog needs real velocity) — this arm adds a direction-free swing bonus (reward.k_walk_swing=1.0, pays ANY completed >=15mm swing) so flailing itself earns, giving exploration a monotone path flail -> leg-cycling -> commanded walking. Single lever on the exact fwd1 recipe, fresh init (prior-free track; fwd1's checkpoint IS the freeze attractor). Bank-proven under exact cfg incl. the pre-registered shuffle farming attack (shuffle -430, negative; gait +409.9 on top). Prediction-if-true: env/reward_step_event and swing income rise within the first 1M (vs fwd1's flat 0.02/step), walk_prog leaves 0.0, eval panel shows real stepping. Prediction-if-false: (a) freeze again with swing rate flat = swing credit still unreachable -> exploration fix moves to init/noise, or (b) in-place shuffle farming (swing income up, prog 0) = the bank's shuffle attack realized in-policy -> reprice shuffle. Strongest alternative: term_penalty catastrophe dominance suppresses exploration regardless of income (the swingterm800 twin tests exactly that).

**gate**: Rung-1 gate (same as fwd1): C-env det fixed-forward panel — zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health read (08-21 ruling): step-event/swing rate rising with walk_prog > 0 but panel short = continue; swing rate flat ~0.02/step at 2M = lever refuted, no same-recipe continuation.

**verdict**: Clean FAIL on the rung-1 gate, and video shows something worse than fwd1's crouch-freeze: the robot starts upright (frame 1) then COLLAPSES flat onto a splayed belly-down pose by frame 2 and stays there the rest of the 25s episode, all 6 det episodes byte-identical (progress_ratio -0.001, forward_dist 0.021m, slip/m 9.0, height_err_end 96mm -- ~10cm off the walking height target). gait_valid is actually FALSE this time (was True/frozen-crouch on fwd1) with leg [1] sacrificed -- the swing bonus did not fix the freeze, it changed WHICH static failure basin the policy collapsed into (splayed-flat vs raised-crouch), not whether it explores real stepping. Reward quarters -85.6/-505.2/-1160.3/-1978.6, same monotonic-negative shape as fwd1 -- per the concurrent cycle's fwd1 root-cause (ep_len x negative-per-step-rate logging artifact, not real further degradation) this is expected under an unfixed freeze, not new information. Swing income (reward.k_walk_swing=1.0) alone does NOT give a random-init PPO policy a path out of the tilt-safe attractor on this recipe -- prediction-if-false (a) confirmed: swing credit stays effectively unreachable (nonzero swing_count in the report is limb micro-twitching in the collapsed pose, not gait cycling, per video). Paired with -swingterm800's identical outcome (see its own verdict), BOTH rung-1 fixes named in this arm's hypothesis are now refuted.

