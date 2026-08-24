# cw-walkcurr-pf-fwd6-actbias1-shortep3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T03:48:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**hypothesis**: Plain English: every one of 13 independently-designed reward-pricing/architecture/reset-pose mechanisms tried on this track converges to the same static park-stand or fall from a random init. This arm touches NONE of those -- it only shortens the TRAINING episode length (eval/gate stays at the full 25s) so a fixed 2M-step budget buys ~8x more resets/goal-resamples than every prior arm, the one axis (episode SCHEDULE) this campaign registered but never tried (see walkcurr STATUS.md 08-24 ~03:3x). Off the actbias1 base (the cleanest non-collapsing arm -- action-space zero-point already fixed, so any remaining freeze is pure reward-economics, not a sim defect). New CLI flag --training-episode-seconds (default None=bit-exact, unit-tested test_training_episode_seconds.py 4/4, smoke-verified on this exact recipe on-pod) decouples training-episode length from eval-episode length so the 25s gate stays comparable across arms. Prediction-if-true: env/walk_freeprog_score leaves the [-0.10,-0.05] dead band and trends toward/past 0 faster or further than any prior arm, and/or the 25s gate eval shows real stepping (gait_valid>=4/6, net translation) for the first time. Prediction-if-false: same static-stand-or-thrash signature at this dose too -- read jointly with the -shortep8 sibling before declaring the axis closed (dose-insensitivity, not a single-dose miss). Strongest alternative: 3s is too short for the freeprog EMA (tau=0.75s) to register a stable signal at all, confounding a real freeze with a metric artifact -- checked against the -shortep8 (8s, >10 tau windows) sibling.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel at the standard 25s EVAL horizon (unchanged) -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: env/walk_freeprog_score trend vs the dead band; env/height_err_mm stays in actbias1's healthy ~15-30mm band (no collapse regression); clip_fraction stays healthy (no crush). Read jointly with -shortep8 for the dose-response shape before closing or promoting the episode-length axis.

