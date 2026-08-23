# cw-walkcurr-pf-rung0-swing3-rnd1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:22:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-rung0-swing3

**wandb_id**: cyvk6sfu

**hypothesis**: Plain English: the rung-0 stepping task freezes into a held static pose (1x swing income = one-leg stork lean, 3x = all-six-legs airborne hover) because a motionless (or airborne) pose still collects whatever charge balance is on offer without ever needing to keep moving -- this adds a small state-novelty bonus (RND, Burda et al.) that pays extra for visiting obs the policy has not seen before and decays to ~0 once a pose is held, which should make holding still/airborne actively unprofitable relative to task reward alone. Single lever vs swing3: --rnd-coef 0.0 -> 0.02 (conservative dose; RNDVecWrapper defaults bit-exact OFF at 0, 8/8 unit tests green including the novelty-decay claim itself, test_rnd_vec.py; wrapper/wiring landed and smoke-tested this cycle, snapshot pending). Everything else byte-identical to swing3 (same k_walk_swing=0.06 rung-0 diet). Prediction-if-true: env/rnd/intrinsic_mean trends down on repeated states while env/reward_swing / gait_valid rise -- C-env det panel shows >=4/6 gait_valid with actual leg cycling, not a held pose. Prediction-if-false (static pose persists, RND intrinsic stays flat/high with clip_fraction healthy): RND state-novelty is refuted as a fix for the swing-income static-pose cheat specifically -- read jointly with the -rnd3 dose sibling and the rscale50-rnd arms (RND applied directly to the rung-1 travel diet, no swing detour) before closing the RND fallback.

**gate**: Rung-0 certification gate (same as swing3/swing9): C-env det fixed-forward panel -- zero tilt terms, six legs cycling (gait_valid) on >=4/6 det episodes, video shows rhythmic stepping (not a held static pose, airborne or grounded). Mechanism health at 2M: clip_fraction > 0.02 (optimizer stays healthy), env/rnd/intrinsic_mean should fall on repeated visits (Welford predictor loss decreasing) while behavior diversifies rather than settling into one pose. PASS = certify + launch rung-1b warm-start. FAIL on both -rnd1/-rnd3 doses = RND-on-rung-0 refuted; decide against the rscale50-rnd (RND-on-rung-1-direct) sibling reads before closing the whole RND fallback.

**verdict**: RND at rnd-coef=0.02 on the rung-0 swing3 diet does NOT certify -- 0/6 det gait_valid, all six legs sacrificed (duty 0.02-0.03), fwd_dist ~0.006m/25s, identical height_err_end_mm=116.3mm to the rung-1 fwd6-rnd02 arm's pose (same belly-sit collapse, not swing3's own one-leg-stork lean). Reward quarters declined (44.5/43.9/35.6/21.3), aligned with the flat eval per the 08-21 ruling -- genuine stuck read. Why: this is the FOURTH independent recipe (rung-1 rscale50+RND, rung-0 swing income at 1x/3x, now rung-0+RND) to converge on the exact same belly-sit pose -- strong evidence it is a structural escape in the shared base reward stack (no height-based termination exists, only tilt; k_height=2.0 apparently too cheap against the savings from dropping essentially all leg-load charges at once), not an artifact of any one mechanism (swing income, RND, or their interaction). Next: read jointly with -rnd3 (dose sibling) and the rung-1-direct rscale50-rnd1/rnd3/rnd10/rnd100 arms (concurrent cycle) before declaring RND-on-rung-0 refuted; either way the concrete next diagnostic (this cycle's STATUS.md addendum) is bisecting whether a bank-legal k_height raise alone prices the settle out, before reaching for a new height-termination or foot-contact-charge mechanism. hardware-ready: no.

