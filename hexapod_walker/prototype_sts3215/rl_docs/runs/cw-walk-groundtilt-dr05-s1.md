# cw-walk-groundtilt-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T21:43:59+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-groundtilt-dr05

**hypothesis**: Seed twin of the c65 groundtilt-dr05 PASS (ruling-7: seed-confirm before leaning on a recipe): identical compose config (warm-start groundtilt5, dr-scale 0.5 + dr.ground_tilt_deg=5.0 override), seed 1 instead of 0. The dr05-compose class retention split is 4-1 (clean: comshift/fricvar/latjit/groundtilt vs payload eroded) — a twin tells us clean retention on the tilt axis is the recipe, not seed luck. Prediction-if-true: seed 1 matches — own-cfg (DR0.5+tilt5) gv 12/12, 0 term, det med fwd >=1.2m AND DR0 nominal retention clean (slip med <=1.24). Prediction-if-false: seed 1 charges nominal retention like payload-dr05 — tilt compose is seed-fragile, panel required before folding into any consolidation stack. Strongest alternative: both seeds pass with different sto tails — compare per-episode spread vs seed 0 (det med 1.54m, slip 1.05). Parent: rl_move/sim/policies/ppo_goal_cw_walk_groundtilt5.zip.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.ground_tilt_deg=5.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**refused_reason**: hexapod-mjx-train-8 code marker 6c0b2d152813a18c0e5c28c21b0afdac9cbd7218-dirty != local HEAD 6c0b2d152813a18c0e5c28c21b0afdac9cbd7218. Sync first: snapshot.sh --sync hexapod-mjx-train-8 (and snapshot/commit before that if the tree is dirty).

