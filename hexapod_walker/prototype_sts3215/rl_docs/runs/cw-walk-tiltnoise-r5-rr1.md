# cw-walk-tiltnoise-r5-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T05:23:49+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-tiltnoise

**wandb_id**: xhxx30eg

**hypothesis**: 5th retry of cw-walk-tiltnoise (base+r1-r4 ALL died 0-step to the fleet launch-collision storm, gotcha 13b -- never got a science result on this axis, unlike its siblings imupos15/gyrobias3 which both closed NO-EFFECT). Same isolated axis unchanged: OPERATOR WISHLIST 13c untested IMU tilt-reading NOISE (dr.tilt_noise_deg=1.5, 5x the full-DR default) off the plain champion longdist-r2, dr-scale 0.0 otherwise.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.tilt_noise_deg=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

