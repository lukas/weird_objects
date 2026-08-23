# cw-walkcurr-pf-fwd6-rnd100

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T22:42:50+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: 58wtkxxr

**hypothesis**: Plain English: rnd02 (coef 0.02) and rnd10 (coef 0.10, 5x) both FAILED this cycle with NEARLY IDENTICAL rnd/intrinsic_mean decay curves -- a 5x dose change produced almost no measurable behavior difference, so this is a dose-INSENSITIVE freeze in that range, not a dose-starved one a small bump would fix. This arm jumps an order of magnitude further, --rnd-coef 1.0 (50x rnd02, 10x rnd10), to test whether the intrinsic bonus needs to be comparable in scale to the FULL per-step task reward (not just the small rung-1 walk charges, which are themselves only 0.01-0.09 at this x0.02-scaled recipe) before it can out-compete the static crouch's own charge-avoidance income. Same exact fwd6-rscale50 diet otherwise, single lever. Pure exploration mechanism, no bank re-proof needed. Prediction-if-true: walk_freeprog_score crosses 0 within 2M, det gait_valid panel shows six legs cycling; rnd/intrinsic_mean should now show a VISIBLY DIFFERENT trajectory from the 0.02/0.10 pair (not just a rescaled copy) if the policy is actually exploring differently. Prediction-if-false (frozen again, AND rnd/intrinsic_mean still tracks the same shape as rnd02/rnd10): RND-as-a-class is refuted at bank-legal reward scales on this recipe regardless of dose -- declare the rung-1 from-scratch-MLP recipe fully refuted at 2M under every tested exploration mechanism (noise-scale, entropy, gSDE, GRU, reward-rescale, RND at 3 doses -- 12 total arms) and escalate to the track's last-resort item (d), a brief BC kickstart (which brushes the 'no BC teacher' rule but is now the only untried item on the pre-registered order). Strongest alternative: coef=1.0 overshoots so hard the policy chases pure novelty and never settles into a task-relevant gait even if it does move -- readable on video (chaotic flailing vs rhythmic stepping) and by comparing det gait_valid/slip against rnd02/rnd10.

**gate**: Same rung-1 gate as rnd02/rnd10: prog_ratio>0 + gait_valid>=4/6 det with visible forward travel on video, walk_freeprog_score crosses 0, clip_fraction stays healthy (no collapse). FAIL alongside rnd02/rnd10 (3/3 doses spanning 50x, all frozen) closes the RND lever on this recipe -- next is BC-kickstart, not a 4th dose point.

