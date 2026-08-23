# cw-walkcurr-pf-rung0-swing3-rnd3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T22:26:44+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-rung0-swing3

**wandb_id**: a0kk6nqx

**hypothesis**: Plain English: dose sibling of -rnd1 (3x the intrinsic-bonus weight, mirroring how swing9 was 3x swing3's dose) -- if 0.02 is too weak against the swing-income diet's static-pose optimum, 0.06 should show it. Single lever vs swing3: --rnd-coef 0.0 -> 0.06. Same predictions as -rnd1: if-true, intrinsic decays on repeats and gait_valid/rhythmic stepping appear; if-false on BOTH doses, RND-on-rung-0 is refuted for the swing-income static-pose cheat specifically. Strongest alternative: 0.06 overshoots into chaotic flailing that never settles (readable on video/gait_valid) rather than a clean gait -- still counts as a rung-0 PASS per the certification gate (six legs cycling, travel not required).

**gate**: Rung-0 certification gate (same as swing3/swing9/-rnd1): C-env det fixed-forward panel -- zero tilt terms, six legs cycling (gait_valid) on >=4/6 det episodes, video shows rhythmic stepping. Mechanism health: clip_fraction > 0.02, env/rnd/intrinsic_mean falling on repeats. Read jointly with -rnd1 for a dose response.

