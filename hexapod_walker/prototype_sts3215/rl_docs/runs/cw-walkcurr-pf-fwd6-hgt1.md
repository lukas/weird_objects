# cw-walkcurr-pf-fwd6-hgt1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T23:27:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: bmj5oyfo

**hypothesis**: Plain English: every RND arm tested (rung-0 1x/3x, rung-1 0.02/0.10/1.0 doses, +4M budget) converged on the SAME static belly-sit collapse -- level attitude, chassis settled ~110-116mm low, all six legs near-zero duty -- because under the x0.02-scaled rung-1 stack the pose's only per-tick cost (the quadratic k_height charge) is tiny next to the discovery-friction charges (idle/park_duty/loadslip) it dodges by never loading a foot, and nothing ends the episode early so that tiny-but-cheaper-than-trying basin rides the full 25s. This arm adds TWO already-built, previously-unused-here mechanisms on top of the exact fwd6-rscale50 recipe (single combined lever, no new reward-shape invention): reward.walk_height_gate=1 (Gaussian-gates walk income by height error, first proven on the unrelated cw-dynrep-criticD-walkcurr4 lineage 08-18) at a LOOSE sigma=15mm (honest gait rides <6mm per calibrate_walk_height.py -- 2.5x headroom before the gate bites, deliberately looser than walkcurr4's 11mm since that bank tuned against an already-competent warm-started gait and rung-1 is from-scratch), plus safety.walk_max_height_drop_mm=60 with a 1.5s grace (belly_sit's -110mm sits ~1.8x past the cutoff) so the collapse pose gets terminated early instead of riding to truncation. Bank-proven this cycle (test_walkcurr_pf_hgt_* in test_task_semantics.py, new belly_sit scripted twin at the probe-measured -109.8mm hip=20/knee=155 pose): under the gated x0.02 stack gait/park/stall all clearly beat belly_sit, the honest gait keeps >=90% of its ungated income (light tax), the pre-existing v2e ranking (gait>>park/stall) is undisturbed, and belly_sit's episode is cut to ~43 steps vs 375 ungated (early termination fires as designed). Prediction-if-true: env/walk_freeprog_score crosses 0 and det gait_valid panel shows six legs cycling with real forward travel within 2M -- the structural escape hatch closes and rung-1 discovery proceeds. Prediction-if-false (frozen again, same belly-sit signature -- height_err_end_mm ~110-116, roll_peak low, walk_speed decaying): the height-gate+termination pairing is insufficient at these doses and the next lever is a tighter sigma/lower drop threshold, or a direct minimum-total-foot-contact charge (the other named fallback in walkcurr/STATUS.md). Strongest alternative: the termination's forfeited-episode-length is real but its ABSOLUTE reward delta is small at this dose (measured in the bank: ~-8 pts vs a ~50pt shared probe-only hold artifact) -- if that undersells the incentive in actual training (where the artifact does not exist), a from-scratch policy might still find belly-sit marginally profitable and simply crouch LESS DEEPLY (e.g. settle at 55mm, just under the 60mm cutoff) rather than truly walk; watch height_err_end_mm on the next FAIL for a shallower-but-still-static pose, which would mean the cutoff itself needs tightening, not abandoning.

**gate**: Rung-1 gate (same as every fwd6 arm): C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video (not just leg-cycling in place), env/walk_freeprog_score leaves [-0.10,-0.05] and trends toward/past 0 by 2M, clip_fraction stays healthy (no collapse). PASS = rung-1 lands, move to rung 2. FAIL with the SAME belly-sit signature (height_err_end_mm ~110+, roll_peak low, walk_speed decaying to ~0) = height-gate+termination insufficient at these doses, tighten sigma/drop or escalate to a direct foot-contact charge. FAIL with a NEW shallower static pose (height_err 40-59mm, just under the 60mm cutoff) = the termination threshold itself needs tightening, not the mechanism class.

