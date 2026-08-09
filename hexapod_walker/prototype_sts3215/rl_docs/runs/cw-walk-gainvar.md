# cw-walk-gainvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T19:27:32+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 26pxylxx

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run): unit-to-unit servo gain spread is a REAL property of cheap STS3215s (fit_motor_model measures joint-to-joint kp/kv scatter; a replacement servo will not match the fitted model). ISOLATED axis: dr-scale 0.0 with ONLY dr.kp_scale_pct=0.40 + dr.kv_scale_pct=0.50 (2x the full-DR defaults 0.20/0.25; champion trained at 0). One axis = servo gain spread (kp+kv are the same physical knob: unit variation). Plain: the robot should walk the same even if its 18 servos are individually stiffer or softer than the calibrated model. Prediction-if-true: gait holds across the spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - gain spread joins the transfer recipe. Prediction-if-false: soft-gain draws sag/oscillate (jitter, height loss, falls) - gain mismatch needs online adaptation, not exposure. Strongest alternative (torquedroop lesson): champion ALREADY tolerates 2x gain spread free - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.kp_scale_pct=0.40 + dr.kv_scale_pct=0.50, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT (letter passed, exposure added nothing): own-cfg kp/kv spread 0.40/0.50 panel gv 12/12, 0 term, det med fwd 1.37m (gate >=1.2) and DR0 retention clean (slip med 0.97) — but parent longdist-r2 under the IDENTICAL spread matches episode-by-episode (parent det 1.59/1.48/1.59/1.42/0.62/0.68 vs run 1.53/1.41/1.51/1.33/0.74/0.68): mid-band gain spread was already free, and the 2 extreme-gain det draws still churn in place at slip 2.8-3.6 after 20M steps of training on them. Frames: extreme draws splay+skate, no falls; nominal draws walk normally. Servo-gain-spread exposure lever CLOSED — extremes join the untrainable calibration class (pattern now: gain, contact-stiff, friction). 13b servo-imperfection exposure record 0-for-5.

