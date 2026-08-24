# cw-walkcurr-pf-fwd6-rscale50-rnd3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:34:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: 277om3m4

**hypothesis**: Plain English: dose sibling of rscale50-rnd1 (3x the intrinsic weight) applied directly to the rung-1 travel diet -- same logic as the rung0-swing3-rnd dose pair, testing whether 0.02 is too weak against the rung-1 landscape's static-crouch optimum. Single lever vs rscale50: --rnd-coef 0.0 -> 0.06, fresh 2M init. Prediction-if-true/false mirrors rscale50-rnd1; read the 2x2 grid (rung0 vs rung1-direct x low vs mid dose) jointly -- whichever base+dose first shows freeprog/gait_valid movement decides the next warm-start lineage.

**gate**: Rung-1 gate (same as rscale50-rnd1): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism health: clip_fraction > 0.02, freeprog trend vs rscale50's own no-RND 6M baseline.

**verdict**: Stale/lost run (finished 2026-08-23T22:42, sat unprocessed ~3h with no prestage evals -- picked up this cycle since it was outside every concurrent cycle's stated scope). Dose sibling of rnd02(0.02)/rnd10(0.10)/rnd100(1.0) at 0.06, direct on the rung-1 rscale50 diet. Result: identical RND-class failure signature -- DR-0 gate det 0/6 gait_valid, ALL SIX legs sacrificed, height_err_mm 111.8-116.3 (matches the 116.3mm collapse signature exactly reproduced across rnd02/rnd10/rnd100/rscale50-rnd1/rung0-swing3-rnd1/rnd3 per STATUS), slip/m 7.89 det / 8.62 sto (high -- legs splayed and thrashing while the body sinks, visible on the contact sheet as a static splayed/sunk stance held across all 10 frames, not real walking). Mechanism health was fine (train/clip_fraction 0.037 final, not collapsed) and env/walk_freeprog_score ended slightly closer to zero than sibling doses (-0.0078 vs the -0.05..-0.10 band) but this did not translate into any gait -- reward quarters 48.9/52.0/47.2/35.3 (rises then falls, not a clean undertrained-keep-going read). Aligned FAIL per 08-21 (eval flat/bad despite a healthy optimizer). Interpolates cleanly inside the already-CLOSED RND-dose bracket (0.02/0.10/1.0 -> now 0.02/0.06/0.10/1.0, same signature at every point) -- no new lever, confirms the track's existing RND-as-a-class closure, no track-decision change. Evidence: logs/ckpt_eval/cw_walkcurr_pf_fwd6_rscale50_rnd3_gate/.

