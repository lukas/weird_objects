# cw-walk-fricvar-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-09T21:50:11+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-fricvar-dr05

**wandb_id**: s7kzsccy

**hardware_ready**: False

**hypothesis**: Seed twin of the c62 fricvar-dr05 PASS (ruling-7: seed-confirm before leaning on a recipe) — the only clean dr05 compose without a twin queued (comshift/payload/latjit twins already running or verdicted). Identical config: warm-start fricvar, dr-scale 0.5 + dr.friction_scale=0.4,1.6 override, seed 1 instead of 0. Prediction-if-true: seed 1 matches — own-cfg (DR0.5+grip spread) gv 12/12, 0 term, det prog med >=0.85, DR0 nominal retention clean (slip med <=1.24). Prediction-if-false: seed 1 charges nominal retention or slick+DR draws collapse beyond seed 0's sto tail (worst 2.48 slip churn) — friction compose is seed-fragile. Strongest alternative: both seeds pass, different slick-tail signatures — compare per-episode vs seed 0 (det fwd med 1.32m, retention slip 1.10). Parent: rl_move/sim/policies/ppo_goal_cw_walk_fricvar.zip.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.friction_scale=0.4,1.6, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: PASS - seed twin confirms fricvar-dr05 compose: own-cfg DR0.5+friction0.4-1.6x panel gv 12/12, 0 term, det med fwd 1.23m>=1.1 gate, slip 1.26 (parent 1.32m/1.22 - noise-band match); DR0 nominal retention CLEAN slip 1.15<=1.24, fwd 1.27m (parent 1.09/1.36 - same band, no erosion). Recipe confirmed, not seed luck.

