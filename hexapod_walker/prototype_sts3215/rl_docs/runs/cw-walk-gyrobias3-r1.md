# cw-walk-gyrobias3-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:40:17+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-gyrobias3

**wandb_id**: s5fcpoet

**hardware_ready**: False

**hypothesis**: RETRY (first attempt lost a launch-collision race amid a concurrent-cycle drain storm — worker EOFError at init, 0 steps, no science result; COMMANDS.md gotcha 13b). Same spec unchanged: isolated dr.gyro_bias_deg_s=3.0 axis off champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.gyro_bias_deg_s=3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT. Own-cfg 3deg/s gyro-rate-bias exposure scrapes the letter gate (det med fwd 1.35m>=1.2, gv 12/12, 0 term, DR0 retention clean slip 1.03) but champion under the IDENTICAL fixed 3deg/s bias (measured this triage) matches it episode-for-episode incl. the same 2/6 craters (0.60-0.70m both) -- exposure training did not fix the pattern, matching the if-false prediction. Gyro-bias joins the sensor/calibration NO-DR-exposure ladder (now 10 of that class).

