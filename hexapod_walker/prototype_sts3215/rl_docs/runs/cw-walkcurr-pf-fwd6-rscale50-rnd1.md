# cw-walkcurr-pf-fwd6-rscale50-rnd1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:30:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: 92h67d2m

**hypothesis**: Plain English: instead of detouring through the rung-0 in-place-stepping sub-goal (which just refuted twice, static stork lean / airborne hover), try the state-novelty bonus DIRECTLY on the real rung-1 forward-walking diet -- if RND alone is enough to make a random policy stumble past the frozen splayed crouch into travel, it reaches the actual goal in one step instead of two. Single lever vs rscale50 (the crush-fixed x0.02-scale rung-1 recipe, itself already optimizer-healthy with a rising but sub-zero freeprog signal over 6M in its own -cont1 continuation): --rnd-coef 0.0 -> 0.02, fresh 2M init (not a continuation -- a clean discovery-phase read of the mechanism from scratch, matching how rscale50 itself was first tested). Prediction-if-true: env/walk_freeprog_score leaves the frozen [-0.10,-0.05] band and crosses toward/past 0 faster than rscale50's own 6M-step climb, C-env det panel shows real cmd_prog_frac>0 with legs cycling. Prediction-if-false (still frozen crouch, RND intrinsic flat/high, clip_fraction healthy): RND does not fix the rung-1 landscape directly -- read jointly with the rung0-swing3-rnd siblings (RND applied to the easier in-place sub-goal) before concluding RND is refuted as a fallback altogether.

**gate**: Rung-1 gate (same as fwd1-fwd6): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism health at 2M: clip_fraction > 0.02 (optimizer stays healthy) AND env/walk_freeprog_score trending toward/past 0 faster than the no-RND rscale50 baseline -- a clear win is freeprog crossing 0 within 2M (rscale50 alone took the full 6M and still didn't).

**verdict**: RND state-novelty at rnd_coef=0.02 on the rung-1 rscale50 diet FAILS the gate -- and being a concurrent-cycle replicate of cw-walkcurr-pf-fwd6-rnd02 (same base recipe, same coef, same 2M budget, independent init), it seed-confirms that read: this is not a fluke of one init. Evidence: DR-0 C-env det panel 0/6 gait_valid, all six legs sacrificed (duty<0.10), prog 0.00, fwd med 0.02m vs 1.25-1.5m commanded, slip/m 7.7 det / 13.0 sto; frame strip walk_det_0 shows the identical static splayed crouch in every frame after settle. W&B mirrors rnd02 near-exactly: clip_fraction healthy the whole run (0.014-0.060, no collapse), value_loss tamed (310->0.4), env/walk_freeprog_score -0.105 -> -0.0086 monotone (tied with rnd02's -0.007 for best-ever rung-1 trend) but NEVER crosses zero; rnd/intrinsic_mean decays 0.037->0.0055 exactly as designed (predictor caught up with a narrow near-static visited set). Why: mechanism works, optimizer healthy, reward aligned (freeprog trend and intrinsic decay both behave as specified) -- the 0.02 dose plus 2M budget is insufficient to escape the crouch optimum; aligned-but-undertrained-at-dose per the 08-21 ruling, matching rnd02 and the dose-insensitive rnd10 read. Next: no new same-dose arms -- the open questions are already funded: rnd100 (coef 1.0, 50x, training) answers the big-dose fork and rnd10-cont1 (+4M, INTENT, other cycle) answers the budget fork; if both fail, the track's last-resort BC-kickstart item is next.

