# cw-walk-groundtilt-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T19:30:00+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: gh4trxa6

**hardware_ready**: no

**hypothesis**: Compose rung off today's groundtilt5 PASS, same recipe as payload-dr05/fricvar-dr05/latjit-dr05: does slope robustness survive general physics variation? Warm-start the slope-hardened policy (groundtilt5) and train at dr-scale 0.5 (the line's validated DR ceiling) WITH dr.ground_tilt_deg=5.0 kept (override holds tilt at the isolated-run level instead of DR0.5's scaled 1deg). Plain: walk on mild slopes AND with the usual body/servo/floor variation at the same time. Prediction-if-true: own-cfg (DR0.5+tilt5) gv 12/12, 0 term, det med fwd >=1.2m, and DR0 nominal retention holds - slope joins the composable transfer recipe. Prediction-if-false: DR variation plus slope overloads the paddle gait (terminations or prog craters beyond the isolated run's 2/6 steep-draw tail) - axes must be trained jointly from scratch or slope needs a curriculum. Strongest alternative: it passes by shrinking to the shallow-tilt draws only - per-episode fwd spread vs the isolated run will show it. Parent: rl_move/sim/policies/ppo_goal_cw_walk_groundtilt5.zip.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.ground_tilt_deg=5.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: PASS: slope robustness composes with DR0.5 — own-cfg (dr 0.5 + tilt5) det 6/6 gv, 0 term, det med fwd 1.54m, slip med 1.05 (champion band); DR0 nominal retention CLEAN (det 6/6 gv, slip med 0.98, fwd 1.54m — no payload-dr05-style erosion; compose-class retention now 4-1 vs payload). Tilt-only pass keeps the parent's honest tail: 2/6 steepest det draws shuffle at ~1/3 speed (0.71/0.63m), no falls — solid to ~3-4deg, 5deg still marginal. Frames watched det+sto: level body, six legs cycling, worst sto draw slow but upright. Paddle lineage, not hardware-ready.

