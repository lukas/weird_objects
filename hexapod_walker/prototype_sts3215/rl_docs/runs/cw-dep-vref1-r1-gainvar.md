# cw-dep-vref1-r1-gainvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T16:47:31+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: today's loaded-actuator finding (P0 rule 6) shows loaded settling time and gain response are less certain than sim's air-fitted defaults; this line has never tested wider per-joint kp/kv gain spread. Widens dr.kp_scale_pct 0.20->0.40 and dr.kv_scale_pct 0.25->0.50 (2x the baseline jitter, still inside the existing dr-scale=0.35 curriculum framework) as an isolated axis on the contract-exact checkpoint. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- extra gain spread composes free like the other single-axis tests. If-false: wider gain variation degrades the gait -- names an actuator-uncertainty axis worth tightening before the loaded-actuator model lands in servo_model.py. Per P0 rule 3, k_current=0.

**gate**: Own-cfg (DR0.35+kp_scale_pct=0.40+kv_scale_pct=0.50) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 baseline-gain retention clean; frames watched det

**refused_reason**: hexapod-mjx-train-6 code marker 5066d61615458ad54320969d3261b5f17f930f12 != local HEAD af9809985cef73ab487b6d3d2cc390f36f6f6262. Sync first: snapshot.sh --sync hexapod-mjx-train-6 (and snapshot/commit before that if the tree is dirty).

