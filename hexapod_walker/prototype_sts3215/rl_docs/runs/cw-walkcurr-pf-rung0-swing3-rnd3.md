# cw-walkcurr-pf-rung0-swing3-rnd3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:26:44+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-rung0-swing3

**wandb_id**: a0kk6nqx

**hypothesis**: Plain English: dose sibling of -rnd1 (3x the intrinsic-bonus weight, mirroring how swing9 was 3x swing3's dose) -- if 0.02 is too weak against the swing-income diet's static-pose optimum, 0.06 should show it. Single lever vs swing3: --rnd-coef 0.0 -> 0.06. Same predictions as -rnd1: if-true, intrinsic decays on repeats and gait_valid/rhythmic stepping appear; if-false on BOTH doses, RND-on-rung-0 is refuted for the swing-income static-pose cheat specifically. Strongest alternative: 0.06 overshoots into chaotic flailing that never settles (readable on video/gait_valid) rather than a clean gait -- still counts as a rung-0 PASS per the certification gate (six legs cycling, travel not required).

**gate**: Rung-0 certification gate (same as swing3/swing9/-rnd1): C-env det fixed-forward panel -- zero tilt terms, six legs cycling (gait_valid) on >=4/6 det episodes, video shows rhythmic stepping. Mechanism health: clip_fraction > 0.02, env/rnd/intrinsic_mean falling on repeats. Read jointly with -rnd1 for a dose response.

**verdict**: RND at rnd-coef=0.06 (3x the -rnd1 dose) on the rung-0 swing3 diet ALSO does NOT certify -- 0/6 det gait_valid, all six legs sacrificed (duty 0.02-0.03), fwd_dist ~0.023m/25s, height_err_end_mm=116.3mm -- the identical belly-sit collapse pose as -rnd1 and the rung-1 fwd6-rnd02 arm, unmoved by tripling the intrinsic bonus. Reward quarters here actually ROSE then fell (50.1/55.1/50.2/39.1) rather than -rnd1's monotone decline, but the terminal eval is the same 0/6 FAIL -- read as a slightly different transient path into the same attractor, not a different outcome. Why: this closes the dose axis on the rung-0+RND question specifically -- 0.02 and 0.06 both land on the exact same collapse height/pose, so RND-on-rung-0 is refuted at both bank-legal-adjacent doses tested (matches the rung-1-direct arms' pattern). Next: with all 4 tested RND arms (rung-1 rnd02, rung-0 rnd1/rnd3, cross-checked against the concurrent cycle's rung-1 rnd10/rnd100/rscale50-rnd1/rnd3) converging on the same belly-sit pose, the fallback is no longer 'try another RND dose' -- it's the height/foot-contact pricing gap named in this cycle's STATUS.md addendum (bisect k_height dose first, eval-only, before any new mechanism). hardware-ready: no.

