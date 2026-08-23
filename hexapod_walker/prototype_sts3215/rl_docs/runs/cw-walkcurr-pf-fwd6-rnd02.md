# cw-walkcurr-pf-fwd6-rnd02

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T22:19:29+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: 9chzzd7a

**hypothesis**: Plain English: nine rung-1 arms (noise-scale, entropy, loadslip-bootstrap, gSDE, GRU, reward-scale x0.1) all froze into a static crouch even after the crush dig-in fixed the optimizer (fwd6-rscale50: clip_fraction healthy, freeprog trending toward 0 but never crossing it at 2M) -- and the rung-0 sub-goal fallback (swing3/swing9) ALSO froze into two different static held poses under a healthy optimizer, closing the reward-landscape-redesign route. This is the pre-registered next fallback (track STATUS: 'RND state-novelty stays the fallback if rung-0 also freezes'): add a Random Network Distillation state-novelty bonus (rnd_vec.RNDVecWrapper, built+unit-tested this cycle, 8/8 green) on top of the EXACT fwd6-rscale50 diet (crush-fixed reward scale, fixed-forward-only rung-1 task) so a random-init rollout gets paid for visiting ANY new state, not just task-aligned ones -- a static held pose stops generating fresh states so its own intrinsic income decays toward 0 (pop_rollout_stats/rnd/intrinsic_mean falling = predictor caught up with a narrow visited set) while any state-visiting policy keeps collecting it. Single lever vs the rscale50 parent: --rnd-coef 0.02 (same order of magnitude as the recipe's own small charges, e.g. k_track=0.02, k_walk_heading=0.01, at this reward scale). Pure exploration mechanism (like --use-sde/--gru), touches no walk charge the WALKCURR_PF bank prices -- no bank re-proof required. Prediction-if-true: env/walk_freeprog_score crosses 0 and det gait_valid panel shows six legs cycling with real forward travel within 2M; rnd/intrinsic_mean should visibly decline over training as the predictor catches up. Prediction-if-false (still frozen with healthy clip_fraction and low/flat rnd/intrinsic_mean from the start): the intrinsic bonus is too small relative to the crouch's own charge-avoidance income to disturb the local optimum -- the higher-dose twin (rnd10, 5x) or an even bigger dose is the next test before declaring the from-scratch MLP recipe fully refuted at 2M and reaching for a brief BC kickstart (last-resort, track-rule-brushing item (d)).

**gate**: Rung-1 gate (same as every fwd6 arm): C-env det fixed-forward panel from eval_checkpoint -- prog_ratio > 0 and gait_valid on >=4/6 det episodes with visible forward travel on video (not just leg-cycling in place), env/walk_freeprog_score leaves [-0.10,-0.05] and trends toward/past 0 by 2M, clip_fraction stays healthy (>0.02, no collapse). PASS = rung-1 lands, move to rung 2 (small heading set). FAIL on both rnd02/rnd10 with clip_fraction healthy and low rnd/intrinsic_mean = RND-at-this-dose refuted, escalate to a bigger dose or BC-kickstart per track STATUS order.

