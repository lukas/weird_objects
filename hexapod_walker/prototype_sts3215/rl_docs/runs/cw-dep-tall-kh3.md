# cw-dep-tall-kh3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-11T22:07:19+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tall30

**wandb_id**: o8w10022

**hypothesis**: TALL LADDER T2a: k_height crank 3x (100->300) at ref -15, warm from tall30. T5 probe (probe_tall_wall.py, 08-11): the crouch is a stability HABIT (yaw splayed to its 35deg limit, support radius 218 vs gait 179mm; pitch/knee have 45-70deg of unused upward room) and the walking body rides -75mm mid-gait (height_err_end is a stop-window metric). Question: does a 3x quadratic height charge move the MID-GAIT posture?

**gate**: Primary metric: probe_tall_wall.py steady-state walking height (parent tall30 = -75mm). PASS: walking height >= -50mm with walk retained (speed >=0.028, survived 1, slip <=1.8, no park). Secondary: height_err_end <=8mm. Compare T2b for dose-response.

