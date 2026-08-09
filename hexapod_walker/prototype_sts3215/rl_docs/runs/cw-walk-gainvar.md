# cw-walk-gainvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T19:27:32+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 26pxylxx

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run): unit-to-unit servo gain spread is a REAL property of cheap STS3215s (fit_motor_model measures joint-to-joint kp/kv scatter; a replacement servo will not match the fitted model). ISOLATED axis: dr-scale 0.0 with ONLY dr.kp_scale_pct=0.40 + dr.kv_scale_pct=0.50 (2x the full-DR defaults 0.20/0.25; champion trained at 0). One axis = servo gain spread (kp+kv are the same physical knob: unit variation). Plain: the robot should walk the same even if its 18 servos are individually stiffer or softer than the calibrated model. Prediction-if-true: gait holds across the spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - gain spread joins the transfer recipe. Prediction-if-false: soft-gain draws sag/oscillate (jitter, height loss, falls) - gain mismatch needs online adaptation, not exposure. Strongest alternative (torquedroop lesson): champion ALREADY tolerates 2x gain spread free - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.kp_scale_pct=0.40 + dr.kv_scale_pct=0.50, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

