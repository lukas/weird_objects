# cw-walkcurr-pf-fwd6-rnd10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T22:25:53+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**hypothesis**: Plain English: dose sibling of the fwd6-rnd02 arm (running/finished on train-1) -- if a small RND novelty bonus (rnd-coef 0.02, same order as the recipe's own charges) is too weak to disturb the static-crouch local optimum, a 5x stronger bonus should show it moving even if 0.02 didn't. Same exact fwd6-rscale50 diet (crush-fixed reward scale, fixed-forward-only rung-1 task), single lever vs rnd02: --rnd-coef 0.1. Pure exploration mechanism (like --use-sde/--gru), touches no walk charge the WALKCURR_PF bank prices -- no bank re-proof required. Prediction-if-true: env/walk_freeprog_score crosses 0, det gait_valid panel shows six legs cycling with real forward travel within 2M, rnd/intrinsic_mean visibly declining as the predictor catches up to the visited-state distribution. Prediction-if-false (still frozen, clip_fraction healthy, rnd/intrinsic_mean flat/low from early on): RND-at-any-reasonable-dose is refuted on the rung-1 diet -- read jointly with the rung-0 sub-goal twin (cw-walkcurr-pf-rung0-swing3-rnd1, another cycle's arm) before reaching for the track's last-resort item (d), a brief BC kickstart. Strongest alternative: 0.1 overshoots into pure novelty-chasing (policy flails for state coverage, ep_rew_mean/task reward regresses even if legs move) -- readable on video (chaotic non-gaited flail vs rhythmic stepping) and by comparing det gait_valid/direction_err against rnd02.

**gate**: Rung-1 gate (same as every fwd6 arm): C-env det fixed-forward panel from eval_checkpoint -- prog_ratio > 0 and gait_valid on >=4/6 det episodes with visible forward travel on video (not just leg-cycling in place), env/walk_freeprog_score leaves [-0.10,-0.05] and trends toward/past 0 by 2M, clip_fraction stays healthy (>0.02, no collapse). PASS = rung-1 lands, move to rung 2 (small heading set). FAIL jointly with rnd02 (both frozen, clip_fraction healthy, low rnd/intrinsic_mean) = RND-on-rung-1 refuted at these doses -- read jointly with the rung-0+RND arm before a BC-kickstart escalation.

**refused_reason**: hexapod-mjx-train-2 already runs cw-walkcurr-pf-fwd6-rnd10 — GPU pods host exactly one run; pick a free GPU pod.

