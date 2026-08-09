# cw-walk-zerobias3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:59:53+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: tdantwa2

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13b/13c (richer physics, one axis per run): per-servo set_zero calibration error is a REAL defect class (the 2026-08-06 incident was exactly this). ISOLATED axis via dr.<field> override: dr-scale 0.0 with ONLY dr.joint_zero_bias_deg=3.0 (per-joint zero offset u(-3,3) deg per episode; full-DR default is 1.0, champion trained at 0) - one variable off the no-DR champion. Plain: the robot should keep walking even when each servo's zero point is calibrated a few degrees wrong. Prediction-if-true: gait holds across the bias spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - miscalibration robustness is trainable by exposure. Prediction-if-false: biased draws destabilize (tilted stance, veering, falls, progress collapse) - zero error needs online estimation, not exposure. Strongest alternative (torquedroop lesson): champion ALREADY tolerates 3deg zero bias for free - triage MUST eval parent longdist-r2 under the same bias spread BEFORE verdicting. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.joint_zero_bias_deg=3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT (gate letter-PASS). Own-cfg (DR0 + joint_zero_bias 3deg) gv 6/6 det, 0 term, det med fwd 1.28m>=1.2 gate; DR0 nominal retention clean (gv 6/6, slip 0.99<=1.24). But parent longdist-r2 under the IDENTICAL bias spread matches episode-for-episode (parent det med fwd 1.42m/slip 1.41 vs run 1.28m/1.40 -- same 4 clean draws, same 2 steep-bias craters: prog 0.54/0.37 slip 2.70/3.83 fwd 0.77/0.64 parent vs prog 0.55/0.41 slip 2.73/4.09 fwd 0.78/0.63 run, frames watched det -- pixel-identical churn-in-place). Exposure taught nothing: champion already tolerates mild per-joint zero-bias free, and ~3deg draws degrade both equally. 13b/13c calibration-exposure ladder now 0-for-6 (gainvar, imubias3, zerobias3 all NO-EFFECT by the same parent-baseline method); zero-bias fix belongs to the estimation rung (DreamWaQ concurrent estimator), not exposure arms.

