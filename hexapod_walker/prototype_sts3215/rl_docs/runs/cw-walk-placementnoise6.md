# cw-walk-placementnoise6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T22:30:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: New 13b physics-variation axis (richer-DR wishlist item, unclaimed): per-joint hand-placement/assembly slop (dr.placement_noise_deg), off the champion at DR0. Champion-baseline-FIRST per c59 rule: measured on longdist_r2 at 3x the field default (6deg) — NOT free (det prog 0.72, slip/m 2.40 vs champion's clean 0.94-0.96 band; sto milder at 0.92/1.23). One variable off champion: add dr.placement_noise_deg=6.0 at dr-scale 0. If-true: own-cfg gv 12/12, 0 term, det med fwd >=1.1m, DR0 no-noise retention clean (slip<=1.24) — assembly-tolerance axis becomes a deployable rung (relevant: real servos get hand-assembled slightly off-center). If-false: terminations or the baseline's degraded-shuffle pattern persists even after training — axis needs a lower magnitude or isn't trainable at 6deg. Strongest alternative: passes by learning a generically slower/more conservative gait that isn't specific to the noise — compare per-episode vs a flat-noise DR0 retention pass.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.placement_noise_deg=6.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; plus DR0 no-noise retention det 6/6 gv, det slip/m med <=1.24; frames watched det

