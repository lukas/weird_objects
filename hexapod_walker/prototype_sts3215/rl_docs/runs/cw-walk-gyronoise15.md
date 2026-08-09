# cw-walk-gyronoise15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T22:31:25+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 9n2bs9f8

**hypothesis**: New 13b/13c sensor-noise axis, distinct from the already-queued gyrobias3 (systematic gyro BIAS): this is gyro NOISE (dr.gyro_noise_deg_s), off the champion at DR0. Champion-baseline-FIRST per c59 rule: measured on longdist_r2 at 3x the field default (1.5deg/s) — NOT free (det prog 0.79, slip/m 2.08 vs champion's clean 0.94-0.96 band; sto milder at 0.93/1.15). One variable off champion: add dr.gyro_noise_deg_s=1.5 at dr-scale 0. If-true: own-cfg gv 12/12, 0 term, det med fwd >=1.1m, DR0 no-noise retention clean (slip<=1.24) — IMU gyro-noise tolerance becomes a deployable rung. If-false: terminations or the baseline's degraded pattern persists after training. Strongest alternative: passes via a generically slower gait, not noise-specific — compare per-episode vs flat retention.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.gyro_noise_deg_s=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; plus DR0 no-noise retention det 6/6 gv, det slip/m med <=1.24; frames watched det

