# cw-dep-vref1-r1-gainvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: EVALUATED

**created**: 2026-08-10T16:51:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: d2yx7kkd

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: today's loaded-actuator finding (P0 rule 6) shows loaded settling time and gain response are less certain than sim's air-fitted defaults; this line has never tested wider per-joint kp/kv gain spread. Widens dr.kp_scale_pct 0.20->0.40 and dr.kv_scale_pct 0.25->0.50 (2x the baseline jitter, still inside the existing dr-scale=0.35 curriculum framework) as an isolated axis on the contract-exact checkpoint. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- extra gain spread composes free like the other single-axis tests. If-false: wider gain variation degrades the gait -- names an actuator-uncertainty axis worth tightening before the loaded-actuator model lands in servo_model.py. Per P0 rule 3, k_current=0.

**gate**: Own-cfg (DR0.35+kp_scale_pct=0.40+kv_scale_pct=0.50) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 baseline-gain retention clean; frames watched det

**verdict**: PASS -- doubling actuator gain-spread DR (kp 0.20->0.40, kv 0.25->0.50) composes onto the contract-exact hardware base, slightly wider margin than siblings. DR0-gate det+sto 6/6 gv, 0 term, slip/m med 1.27/1.10 (det ~12% over vref1-r1 own 1.13 upper edge, inside +-20% tol; sto comfortably in-band). Own-cfg (DR0.35+override) det+sto 6/6 gv, 0 term, slip/m med 1.24/1.29, same degraded-episode pattern (det/5, sto/0-1) as PASSed torquescale/tiltnoise siblings -- curriculum-DR artifact, not new. Video shows same low-amplitude six-leg creep, body level, no flag leg/fall, craters are the known fixed-draw march-in-place stall. Names gain uncertainty as a real but still-safe axis; worth watching if a future compose stacks it with another actuator axis.

