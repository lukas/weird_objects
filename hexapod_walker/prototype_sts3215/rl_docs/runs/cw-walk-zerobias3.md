# cw-walk-zerobias3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T18:59:53+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: tdantwa2

**hypothesis**: OPERATOR WISHLIST 13b/13c (richer physics, one axis per run): per-servo set_zero calibration error is a REAL defect class (the 2026-08-06 incident was exactly this). ISOLATED axis via dr.<field> override: dr-scale 0.0 with ONLY dr.joint_zero_bias_deg=3.0 (per-joint zero offset u(-3,3) deg per episode; full-DR default is 1.0, champion trained at 0) - one variable off the no-DR champion. Plain: the robot should keep walking even when each servo's zero point is calibrated a few degrees wrong. Prediction-if-true: gait holds across the bias spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - miscalibration robustness is trainable by exposure. Prediction-if-false: biased draws destabilize (tilted stance, veering, falls, progress collapse) - zero error needs online estimation, not exposure. Strongest alternative (torquedroop lesson): champion ALREADY tolerates 3deg zero bias for free - triage MUST eval parent longdist-r2 under the same bias spread BEFORE verdicting. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.joint_zero_bias_deg=3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

