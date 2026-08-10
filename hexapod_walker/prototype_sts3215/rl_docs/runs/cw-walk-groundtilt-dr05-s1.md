# cw-walk-groundtilt-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-09T21:47:15+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-groundtilt-dr05

**wandb_id**: pkz9nall

**hardware_ready**: False

**hypothesis**: Seed twin of the c65 groundtilt-dr05 PASS (ruling-7: seed-confirm before leaning on a recipe): identical compose config (warm-start groundtilt5, dr-scale 0.5 + dr.ground_tilt_deg=5.0 override), seed 1 instead of 0. The dr05-compose class retention split is 4-1 (clean: comshift/fricvar/latjit/groundtilt vs payload eroded) — a twin tells us clean retention on the tilt axis is the recipe, not seed luck. Prediction-if-true: seed 1 matches — own-cfg (DR0.5+tilt5) gv 12/12, 0 term, det med fwd >=1.2m AND DR0 nominal retention clean (slip med <=1.24). Prediction-if-false: seed 1 charges nominal retention like payload-dr05 — tilt compose is seed-fragile, panel required before folding into any consolidation stack. Strongest alternative: both seeds pass with different sto tails — compare per-episode spread vs seed 0 (det med 1.54m, slip 1.05). Parent: rl_move/sim/policies/ppo_goal_cw_walk_groundtilt5.zip.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.ground_tilt_deg=5.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: PASS - seed twin confirms groundtilt-dr05 compose: own-cfg DR0.5+tilt u(0,5deg) panel gv 12/12, 0 term, det med fwd 1.37m>=1.2 gate, slip 1.13 (parent 1.54m/1.05 - same champion band); DR0 nominal retention CLEAN slip 1.05<=1.24, fwd 1.41m (parent 0.98/1.54). Recipe confirmed, not seed luck.

