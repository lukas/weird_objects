# cw-walkcurr-pf-rung0-swing3-rnd1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T22:22:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-rung0-swing3

**wandb_id**: cyvk6sfu

**hypothesis**: Plain English: the rung-0 stepping task freezes into a held static pose (1x swing income = one-leg stork lean, 3x = all-six-legs airborne hover) because a motionless (or airborne) pose still collects whatever charge balance is on offer without ever needing to keep moving -- this adds a small state-novelty bonus (RND, Burda et al.) that pays extra for visiting obs the policy has not seen before and decays to ~0 once a pose is held, which should make holding still/airborne actively unprofitable relative to task reward alone. Single lever vs swing3: --rnd-coef 0.0 -> 0.02 (conservative dose; RNDVecWrapper defaults bit-exact OFF at 0, 8/8 unit tests green including the novelty-decay claim itself, test_rnd_vec.py; wrapper/wiring landed and smoke-tested this cycle, snapshot pending). Everything else byte-identical to swing3 (same k_walk_swing=0.06 rung-0 diet). Prediction-if-true: env/rnd/intrinsic_mean trends down on repeated states while env/reward_swing / gait_valid rise -- C-env det panel shows >=4/6 gait_valid with actual leg cycling, not a held pose. Prediction-if-false (static pose persists, RND intrinsic stays flat/high with clip_fraction healthy): RND state-novelty is refuted as a fix for the swing-income static-pose cheat specifically -- read jointly with the -rnd3 dose sibling and the rscale50-rnd arms (RND applied directly to the rung-1 travel diet, no swing detour) before closing the RND fallback.

**gate**: Rung-0 certification gate (same as swing3/swing9): C-env det fixed-forward panel -- zero tilt terms, six legs cycling (gait_valid) on >=4/6 det episodes, video shows rhythmic stepping (not a held static pose, airborne or grounded). Mechanism health at 2M: clip_fraction > 0.02 (optimizer stays healthy), env/rnd/intrinsic_mean should fall on repeated visits (Welford predictor loss decreasing) while behavior diversifies rather than settling into one pose. PASS = certify + launch rung-1b warm-start. FAIL on both -rnd1/-rnd3 doses = RND-on-rung-0 refuted; decide against the rscale50-rnd (RND-on-rung-1-direct) sibling reads before closing the whole RND fallback.

