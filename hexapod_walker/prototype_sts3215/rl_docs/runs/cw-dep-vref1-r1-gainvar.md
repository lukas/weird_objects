# cw-dep-vref1-r1-gainvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T16:51:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: d2yx7kkd

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: today's loaded-actuator finding (P0 rule 6) shows loaded settling time and gain response are less certain than sim's air-fitted defaults; this line has never tested wider per-joint kp/kv gain spread. Widens dr.kp_scale_pct 0.20->0.40 and dr.kv_scale_pct 0.25->0.50 (2x the baseline jitter, still inside the existing dr-scale=0.35 curriculum framework) as an isolated axis on the contract-exact checkpoint. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- extra gain spread composes free like the other single-axis tests. If-false: wider gain variation degrades the gait -- names an actuator-uncertainty axis worth tightening before the loaded-actuator model lands in servo_model.py. Per P0 rule 3, k_current=0.

**gate**: Own-cfg (DR0.35+kp_scale_pct=0.40+kv_scale_pct=0.50) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 baseline-gain retention clean; frames watched det

