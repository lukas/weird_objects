# cw-amp-m2-freeprog-term400-rsi1-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T17:36:56+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**hypothesis**: Every from-scratch M2 freeprog lever tried so far (term-penalty, std-anneal, staging, task-complexity/fixedcmd, AMP style dose, k_walk_swing) changed the REWARD and all failed the same way: flat/declining training reward stuck in a ~0.03m/15s in-place-shuffle basin -- because a random from-scratch policy basically never DISCOVERS a coordinated six-leg walking state via unguided exploration, so no reward shape can select for one. This arm changes the INITIAL STATE instead: goal.walk_gait_start_frac=0.5 spawns half of episodes MID-STRIDE in the scripted tripod gait's own tall walking pose (built-in RSI-for-walk mechanism) with the walk command already active, so PPO only has to learn to SUSTAIN a gait it starts inside of, not discover one from a static crouch. This exact lever (cw-gait-rsi1, frac=0.5) was tried once before but on an OLDER pre-freeprog/pre-term400 stack (08-11) and refuted with the same freeze/statue signature -- untested on the CURRENT freeprog+term400 pricing, which did not exist then. Prediction-if-true: fwd travel clears meaningfully above the ~0.03m statue-family ceiling (ideally >=0.10m) because episodes that start mid-gait keep moving instead of re-collapsing to the statue. Prediction-if-false: even mid-gait spawns collapse back to the statue/shuffle within a few hundred steps (freeprog_pen stays pinned ~-1.4/tick) -- confirming the basin is an INCOME-not-DISCOVERY problem after all, and closing RSI as a lever for this family for good (both stacks now tested).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged against the whole statue family (noamp/style05-v2/stylew2-v2/fixedcmd/swing-noamp/swing-style05, all ~0.02-0.05m fwd): PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six legs cycling with net displacement AND freeprog_pen visibly climbs off its -1.4 to -1.5/tick floor in W&B. INFORMATIVE = fwd travel clears meaningfully above 0.05m (the family's ceiling) even short of 0.10m, or gait_valid reaches 4-5/6 for the first time in this family. Cheat check: distinguish real sustained walking from episodes that only look good for the RSI-seeded head of the episode then collapse (compare early-episode vs late-episode forward_dist_m within an episode if available, or watch the video's back half specifically).

