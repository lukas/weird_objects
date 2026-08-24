# cw-walkcurr-pf-fwd6-actbias1-shortep3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T03:48:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**wandb_id**: lpy9ogdp

**hypothesis**: Plain English: every one of 13 independently-designed reward-pricing/architecture/reset-pose mechanisms tried on this track converges to the same static park-stand or fall from a random init. This arm touches NONE of those -- it only shortens the TRAINING episode length (eval/gate stays at the full 25s) so a fixed 2M-step budget buys ~8x more resets/goal-resamples than every prior arm, the one axis (episode SCHEDULE) this campaign registered but never tried (see walkcurr STATUS.md 08-24 ~03:3x). Off the actbias1 base (the cleanest non-collapsing arm -- action-space zero-point already fixed, so any remaining freeze is pure reward-economics, not a sim defect). New CLI flag --training-episode-seconds (default None=bit-exact, unit-tested test_training_episode_seconds.py 4/4, smoke-verified on this exact recipe on-pod) decouples training-episode length from eval-episode length so the 25s gate stays comparable across arms. Prediction-if-true: env/walk_freeprog_score leaves the [-0.10,-0.05] dead band and trends toward/past 0 faster or further than any prior arm, and/or the 25s gate eval shows real stepping (gait_valid>=4/6, net translation) for the first time. Prediction-if-false: same static-stand-or-thrash signature at this dose too -- read jointly with the -shortep8 sibling before declaring the axis closed (dose-insensitivity, not a single-dose miss). Strongest alternative: 3s is too short for the freeprog EMA (tau=0.75s) to register a stable signal at all, confounding a real freeze with a metric artifact -- checked against the -shortep8 (8s, >10 tau windows) sibling.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel at the standard 25s EVAL horizon (unchanged) -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: env/walk_freeprog_score trend vs the dead band; env/height_err_mm stays in actbias1's healthy ~15-30mm band (no collapse regression); clip_fraction stays healthy (no crush). Read jointly with -shortep8 for the dose-response shape before closing or promoting the episode-length axis.

**verdict**: Result: FAIL -- the 3s training-episode dose (vs the implicit 25s baseline, ~8x more resets/2M budget) does NOT unfreeze rung-1; identical static splayed-crouch/thrash signature to all 14 prior refuted mechanisms. Evidence: det gate 0/6 gait_valid, prog med 0.01, fwd med 0.02m/25s, slip/m med 3.80, legs [0,2,3,5] sacrificed identically across 5/6 episodes; sto gait_valid 5/6 but slip/m med 31.56 (in-place thrash, prog med -0.01); env/walk_freeprog_score bounced in [-0.085,-0.04] the whole 2M run with no trend toward 0 (worse than the rscale50 parent's monotonic -0.10->-0.015 rise); rollout/ep_rew_mean flat 47-49 all 4 quarters; train/clip_fraction healthy 0.001->0.11 (rising, no crush); rollout/ep_len_mean saturates at exactly 75 steps (=3.0s*25Hz, confirms --training-episode-seconds worked as intended -- not a wiring bug); env/height_err_mm creeps 5->19mm, inside actbias1's non-collapse band. Contact sheet: bit-identical static splayed crouch, zero net translation, all 6 det episodes. Why: aligned FAIL per the 08-21 ruling -- reward flat, eval flat, optimizer healthy, adequate 2M budget -- genuine refutation, not misalignment/undertraining. What's next: this is ONE dose of the pre-registered 2-arm episode-length pair; -shortep8 (8.0s, still training as of this cycle) is the required sibling read before declaring the episode-length axis closed (dose-insensitivity + rules out the 'freeprog EMA needs >10 tau windows' confound this pair was designed to separate). Do not launch further episode-length doses until shortep8 reads. If shortep8 also fails with the same signature, the episode-length axis closes and OPERATOR_QUESTIONS.md q_20260824T0233Z's BC-kickstart ruling becomes the track's sole remaining lever (already flagged OPEN, awaiting operator).

