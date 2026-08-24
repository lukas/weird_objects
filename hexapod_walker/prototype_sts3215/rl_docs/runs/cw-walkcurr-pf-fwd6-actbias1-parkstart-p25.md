# cw-walkcurr-pf-fwd6-actbias1-parkstart-p25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T03:14:05+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**wandb_id**: ujevqqfg

**hypothesis**: Plain English: does densifying the reset distribution with tripod-lifted 'park' starting poses (no reference motion, no teacher gait, no BC -- pure procedural hip-offset perturbation, ALREADY BUILT in the codebase for a different lineage) let PPO discover an exit gradient out of the same static park-stand attractor that 10 independently-designed reward/architecture/termination mechanisms all failed to dislodge? Single lever vs actbias1 (the clean, non-collapsing zero-point-fixed base -- no idle-termination stacked, since that class is separately closed): goal.walk_park_start_frac=0.25, the exact dose historically validated on a different (AMP-lineage) walk recipe (cw-walk-parkstart-mjx-c1: 11/12 fwd>=0.40m, park-exit 11/12 met). Does not touch reward pricing (test_task_semantics.py comment confirms start-pose-diversity keys are excluded from the WALKCURR_PF bank), so no bank re-proof required. Prediction-if-true: env/walk_freeprog_score crosses zero and/or det gate shows real six-leg stepping, distinctly better than actbias1's static 3-leg tripod hold. Prediction-if-false (identical static park-stand persists despite densified park-adjacent starts): reset-state diversity is exonerated too -- the static-stand economics dominate regardless of where episodes start, and BC-kickstart genuinely is the only unexplored escalation (flagged to the operator, q_20260824T0233Z).

**gate**: Rung-1 gate: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: env/walk_freeprog_score trend vs the [-0.10,-0.05] dead band every prior rung-1 arm sat in; env/height_err_mm stays in actbias1's healthy ~15-22mm band; clip_fraction stays healthy; video shows real stepping not just park-exit-then-restand.

**verdict**: Reset-state diversity does not dislodge the static park-stand: seeding 25% of episodes from tripod-lifted park poses left the policy converging to the same in-place stand as its parent actbias1. Evidence: DR-0 gate det 0/6 gait_valid (fwd med 0.03m/25s, prog med 0.01, slip/m 8.87, sac [1,4]/[0,4,5], zero terms; det video: level upright stance with in-place jitter, zero body translation across all 10 frames); sto gait_valid 5/6 but slip/m 35.9 = noise-driven in-place thrashing, fwd 0.06m. W&B: walk_freeprog_score flat in the dead band (-0.084->-0.075), direction_err ~88deg (chance) all run, ep_len_mean RISES 64->490 (the policy learns to recover from the perturbed park starts back INTO the stand), ep_rew falls 45->7 with healthy clip_fraction 0.02->0.09 — aligned FAIL per the 08-21 rules, exactly the pre-registered prediction-if-false. Why: densified park-adjacent starts give an exit gradient, but the reward economics still price re-standing above stepping, so the policy uses the exit only to re-park. Next: the p50 dose sibling (in flight, another cycle's read) gives the final word on the parkstart class; if it also fails, reset diversity is exonerated and the BC-kickstart operator question (q_20260824T0233Z) is live per the pinned fork.

