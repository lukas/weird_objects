# cw-amp-m2-freeprog-term400-fixedcmd

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T16:47:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**wandb_id**: 4rnu2fh9

**hypothesis**: Plain English: test whether a from-scratch policy can learn to walk forward AT ALL once the task is the exact simplest case the SLIPWALK semantics bank itself already validates -- one fixed slow forward speed (0.05 m/s) held the whole episode, no direction changes, no stops, no turns, no mid-episode resampling. Single lever vs cw-amp-m2-freeprog-term400-noamp: replace the goal.* command config with the literal SLIPWALK_OVERRIDES fixed command; byte-identical reward stack, no AMP. See --notes on the launched run for the full writeup.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg -- eval auto-matches this run's own fixed-command training cfg). PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six-leg cycling with net forward displacement, read against the SLIPWALK bank's own gait(+417,0.22m)/creep(+108,0.16m)/stall(-143)/park(-244) reference. Prediction-if-true: task complexity (not reward shape) was the barrier -- revisit stress_mix/stagecurric with a longer held stage-0. Prediction-if-false: statue persists at the bank's own simplest case -- genuine PPO exploration/basin-barrier problem; next lever is RSI or a stance-load curriculum, not another task/dose tweak.

**verdict**: seed7: DR-0 det gait_valid 0/6 (legs 2,3,5 sacrificed EVERY episode, deterministic since the command never varies), fwd med 0.05m/15s (bar 0.10m), slip 7.14/m. sto gait_valid 5/6, fwd med 0.07m, slip 17.95/m. VERDICTED FAIL, confirms the prediction-if-false branch. Same freeprog_pen (~-1.37/tick) and gait_min (~0.29) plateau as -noamp -- the SLIPWALK bank's own literal fixed-command task (the exact setup proven to rank gait>>creep>>stall>>park) does NOT unlock real locomotion from scratch. If anything WORSE than the harder stress_mix arms (which sacrificed 0-1 legs, gait_valid 5/6 det) -- simplifying the command distribution did not just fail to help, it let MORE legs go idle/statue. Video: 3 legs held in a fixed splayed pose, the other 3 doing small in-place motion, body stationary. W&B reward fell every quarter (-161/-538/-1007/-1252), never rising. Matches seed11 (also FAIL, legs 3,4 sacrificed, gait_valid 0/6 det) almost exactly -- n=2 seeds agree: TASK-COMPLEXITY HYPOTHESIS REFUTED. The from-scratch marching/statue basin is a genuine PPO exploration/optimization pathology independent of command distribution, not a curriculum-timing artifact. Per the pre-registered branch, next lever is a structurally different mechanism (RSI from teacher_v2 motion-library poses matched to this stack, or a stance-load curriculum), not another task/dose tweak -- the whole 'is the reward-shape or the task the barrier' question for M2 freeprog is now closed on the task side.

