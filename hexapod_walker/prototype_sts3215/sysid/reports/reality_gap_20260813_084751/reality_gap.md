# Hexapod Sim Reality Report

- generated: 2026-08-13T08:47:52
- git: `659df2c`
- servo params `air`: sim_model.json (6968268e879e)
- servo params `rl_move/sim/sim_model_sysid.json`: sim_model_sysid.json (8b8d2f260256)

## Headline gap metrics

| trace | metric | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|---|
| sysid_steps_loaded_v1_20260813_124252.csv | unloaded q_rmse_deg | — | 0.586 | 0.503 |
| sysid_steps_loaded_v1_20260813_124252.csv | unloaded qd_rmse_deg_s | — | 1.71 | 1.39 |
| sysid_steps_loaded_v1_20260813_124252.csv | cmd→motion latency (hw) | median 120.0 ms (p10 80.0 / p90 120.2 / max 160.1, n=48) | — | — |
| sysid_champion_stand_ground_v1_20260813_124656.csv | unloaded q_rmse_deg | — | 1.146 | 1.041 |
| sysid_champion_stand_ground_v1_20260813_124656.csv | unloaded qd_rmse_deg_s | — | 4.62 | 5.09 |

## Timing (hardware)

- **sysid_steps_loaded_v1_20260813_124252.csv**: tick median 40.0 ms (nominal 40.0, p95 40.2, late 0.07%); send→recv RTT median 8.3 ms (p95 9.9)
- **sysid_champion_stand_ground_v1_20260813_124656.csv**: tick median 40.0 ms (nominal 40.0, p95 40.2, late 0.34%); send→recv RTT median 9.3 ms (p95 9.6)

## Servo-to-servo variation (t90)

- sysid_steps_loaded_v1_20260813_124252.csv [hip]: median 339.8 ms over 2 joints, spread 5.9%
- sysid_steps_loaded_v1_20260813_124252.csv [knee]: median 419.8 ms over 2 joints, spread 4.8%

## Per-segment detail

### sysid_steps_loaded_v1_20260813_124252.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| L2_hip_lstep+3_r0 | latency=159.9 overshoot=0.0 tracking=35.1 | latency=80.0 overshoot=0.0 tracking=76.9 | latency=80.0 t90=239.9 overshoot=0.0 tracking=97.1 |
| L2_hip_lstep-3_r0 | latency=80.0 overshoot=0.0 tracking=85.0 | latency=80.0 t90=199.9 overshoot=0.003 tracking=100.0 | latency=80.0 t90=240.0 overshoot=0.009 tracking=100.2 |
| L2_hip_lstep+6_r0 | latency=120.0 overshoot=0.0 tracking=74.7 | latency=79.9 overshoot=0.0 tracking=88.7 | latency=79.9 t90=320.0 overshoot=0.0 tracking=98.5 |
| L2_hip_lstep-6_r0 | latency=80.0 t90=320.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=279.9 overshoot=0.0 tracking=100.0 | latency=80.0 t90=320.0 overshoot=0.0 tracking=99.9 |
| L2_hip_lstep+3_r1 | latency=120.0 overshoot=0.0 tracking=64.5 | latency=80.0 overshoot=0.0 tracking=76.9 | latency=80.0 t90=240.0 overshoot=0.0 tracking=97.4 |
| L2_hip_lstep-3_r1 | latency=80.0 t90=239.9 overshoot=0.0 tracking=90.8 | latency=80.0 t90=200.0 overshoot=0.003 tracking=100.0 | latency=80.0 t90=239.9 overshoot=0.0 tracking=99.9 |
| L2_hip_lstep+6_r1 | latency=120.0 overshoot=0.0 tracking=76.2 | latency=80.0 overshoot=0.0 tracking=88.3 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.5 |
| L2_hip_lstep-6_r1 | latency=80.0 t90=319.7 overshoot=0.0 tracking=96.7 | latency=80.0 t90=280.0 overshoot=0.0 tracking=100.0 | latency=80.0 t90=319.7 overshoot=0.0 tracking=99.9 |
| L2_hip_lstep+3_r2 | latency=120.0 overshoot=0.0 tracking=82.0 | latency=80.0 overshoot=0.0 tracking=85.3 | latency=80.0 t90=240.0 overshoot=0.0 tracking=97.1 |
| L2_hip_lstep-3_r2 | latency=80.1 t90=280.1 overshoot=0.0 tracking=86.6 | latency=80.1 t90=200.1 overshoot=0.003 tracking=100.0 | latency=80.1 t90=240.1 overshoot=0.001 tracking=99.9 |
| L2_hip_lstep+6_r2 | latency=120.0 overshoot=0.0 tracking=76.2 | latency=79.9 overshoot=0.0 tracking=88.3 | latency=79.9 t90=320.0 overshoot=0.0 tracking=98.6 |
| L2_hip_lstep-6_r2 | latency=80.0 t90=320.1 overshoot=0.0 tracking=95.2 | latency=80.0 t90=280.6 overshoot=0.01 tracking=100.1 | latency=80.0 t90=320.1 overshoot=0.008 tracking=100.1 |
| L2_knee_lstep+5_r0 | latency=134.7 overshoot=0.0 tracking=75.6 | latency=54.6 t90=254.7 overshoot=0.0 tracking=96.5 | latency=134.7 t90=334.6 overshoot=0.0 tracking=96.8 |
| L2_knee_lstep-5_r0 | latency=79.9 t90=279.8 overshoot=0.0 tracking=97.4 | latency=79.9 t90=240.0 overshoot=0.004 tracking=99.8 | latency=119.8 t90=319.9 overshoot=0.026 tracking=99.9 |
| L2_knee_lstep+10_r0 | latency=120.2 overshoot=0.0 tracking=83.5 | latency=80.2 t90=440.2 overshoot=0.0 tracking=90.2 | latency=120.2 t90=440.2 overshoot=0.0 tracking=94.5 |
| L2_knee_lstep-10_r0 | latency=79.2 t90=479.1 overshoot=0.0 tracking=96.7 | latency=79.2 t90=399.1 overshoot=0.003 tracking=99.9 | latency=119.0 t90=439.2 overshoot=0.028 tracking=100.0 |
| L2_knee_lstep+5_r1 | latency=160.1 overshoot=0.0 tracking=73.8 | latency=80.0 overshoot=0.0 tracking=87.8 | latency=120.0 overshoot=0.0 tracking=88.8 |
| L2_knee_lstep-5_r1 | latency=80.0 t90=320.1 overshoot=0.0 tracking=96.7 | latency=80.0 t90=240.1 overshoot=0.007 tracking=99.9 | latency=120.0 t90=320.1 overshoot=0.045 tracking=100.3 |
| L2_knee_lstep+10_r1 | latency=120.0 overshoot=0.0 tracking=84.4 | latency=80.0 t90=440.0 overshoot=0.0 tracking=90.2 | latency=120.0 t90=479.9 overshoot=0.0 tracking=94.6 |
| L2_knee_lstep-10_r1 | latency=120.2 t90=480.1 overshoot=0.283 tracking=102.0 | latency=80.2 t90=400.1 overshoot=0.0 tracking=99.1 | latency=120.2 t90=440.2 overshoot=0.0 tracking=99.1 |
| L2_knee_lstep+5_r2 | latency=160.0 overshoot=0.0 tracking=73.8 | latency=79.9 overshoot=0.0 tracking=80.6 | latency=119.8 overshoot=0.0 tracking=88.8 |
| L2_knee_lstep-5_r2 | latency=80.0 t90=319.9 overshoot=0.0 tracking=96.7 | latency=80.0 t90=240.0 overshoot=0.0 tracking=98.1 | latency=119.9 t90=319.9 overshoot=0.0 tracking=98.3 |
| L2_knee_lstep+10_r2 | latency=120.1 overshoot=0.0 tracking=85.3 | latency=80.1 t90=440.0 overshoot=0.0 tracking=90.2 | latency=120.1 t90=440.0 overshoot=0.0 tracking=94.6 |
| L2_knee_lstep-10_r2 | latency=120.0 t90=480.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=439.8 overshoot=0.0 tracking=99.3 |
| L5_hip_lstep+3_r0 | latency=119.9 overshoot=0.0 tracking=58.6 | latency=80.0 overshoot=0.0 tracking=79.9 | latency=80.0 t90=240.0 overshoot=0.0 tracking=97.1 |
| L5_hip_lstep-3_r0 | latency=80.0 overshoot=0.0 tracking=87.9 | latency=80.0 t90=200.0 overshoot=0.003 tracking=100.0 | latency=80.0 t90=239.9 overshoot=0.009 tracking=100.2 |
| L5_hip_lstep+6_r0 | latency=120.0 overshoot=0.0 tracking=74.7 | latency=80.0 overshoot=0.0 tracking=88.7 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.7 |
| L5_hip_lstep-6_r0 | latency=119.9 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=280.0 overshoot=0.0 tracking=100.0 | latency=80.0 t90=319.8 overshoot=0.007 tracking=100.1 |
| L5_hip_lstep+3_r1 | latency=119.9 overshoot=0.0 tracking=61.5 | latency=80.0 overshoot=0.0 tracking=88.9 | latency=80.0 t90=240.0 overshoot=0.0 tracking=97.1 |
| L5_hip_lstep-3_r1 | latency=80.0 t90=240.0 overshoot=0.0 tracking=90.8 | latency=80.0 t90=200.0 overshoot=0.003 tracking=100.0 | latency=80.0 t90=240.0 overshoot=0.009 tracking=100.2 |
| L5_hip_lstep+6_r1 | latency=120.0 overshoot=0.0 tracking=76.2 | latency=80.0 overshoot=0.0 tracking=89.7 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.7 |
| L5_hip_lstep-6_r1 | latency=120.0 t90=359.9 overshoot=0.0 tracking=92.3 | latency=79.8 t90=279.8 overshoot=0.0 tracking=100.0 | latency=79.8 t90=319.9 overshoot=0.0 tracking=99.9 |
| L5_hip_lstep+3_r2 | latency=120.0 overshoot=0.0 tracking=85.0 | latency=79.8 overshoot=0.0 tracking=78.9 | latency=79.8 t90=240.5 overshoot=0.0 tracking=97.1 |
| L5_hip_lstep-3_r2 | latency=80.0 t90=240.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=200.2 overshoot=0.003 tracking=100.0 | latency=80.0 t90=240.0 overshoot=0.0 tracking=99.9 |
| L5_hip_lstep+6_r2 | latency=119.9 overshoot=0.0 tracking=77.6 | latency=79.9 t90=279.7 overshoot=0.0 tracking=95.9 | latency=79.9 t90=319.8 overshoot=0.0 tracking=98.5 |
| L5_hip_lstep-6_r2 | latency=79.9 t90=359.9 overshoot=0.0 tracking=96.7 | latency=79.9 t90=279.9 overshoot=0.0 tracking=100.0 | latency=79.9 t90=320.0 overshoot=0.0 tracking=99.9 |
| L5_knee_lstep+5_r0 | latency=120.0 overshoot=0.0 tracking=82.6 | latency=80.0 overshoot=0.0 tracking=80.6 | latency=120.0 overshoot=0.0 tracking=89.2 |
| L5_knee_lstep-5_r0 | latency=120.0 t90=319.8 overshoot=0.0 tracking=94.9 | latency=80.0 t90=240.0 overshoot=0.0 tracking=94.8 | latency=120.0 t90=319.8 overshoot=0.0 tracking=94.6 |
| L5_knee_lstep+10_r0 | latency=120.0 t90=480.0 overshoot=0.0 tracking=91.4 | latency=80.0 t90=399.9 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 |
| L5_knee_lstep-10_r0 | latency=120.0 t90=439.9 overshoot=0.0 tracking=97.5 | latency=79.9 t90=400.0 overshoot=0.0 tracking=97.2 | latency=120.0 t90=439.9 overshoot=0.0 tracking=97.4 |
| L5_knee_lstep+5_r1 | latency=80.3 overshoot=0.0 tracking=86.1 | latency=80.3 overshoot=0.0 tracking=89.3 | latency=120.3 t90=360.0 overshoot=0.0 tracking=89.6 |
| L5_knee_lstep-5_r1 | latency=119.9 t90=320.1 overshoot=0.0 tracking=96.7 | latency=80.0 t90=240.0 overshoot=0.0 tracking=98.1 | latency=119.9 t90=320.1 overshoot=0.0 tracking=98.3 |
| L5_knee_lstep+10_r1 | latency=120.0 t90=480.0 overshoot=0.0 tracking=93.2 | latency=80.0 t90=399.9 overshoot=0.0 tracking=98.8 | latency=120.0 t90=440.3 overshoot=0.0 tracking=99.1 |
| L5_knee_lstep-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=399.9 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L5_knee_lstep+5_r2 | latency=80.0 overshoot=0.0 tracking=87.9 | latency=80.0 t90=280.0 overshoot=0.0 tracking=89.8 | latency=120.0 t90=359.9 overshoot=0.0 tracking=89.6 |
| L5_knee_lstep-5_r2 | latency=119.7 t90=319.9 overshoot=0.0 tracking=93.2 | latency=79.9 t90=240.0 overshoot=0.0 tracking=96.0 | latency=119.7 t90=319.9 overshoot=0.0 tracking=96.4 |
| L5_knee_lstep+10_r2 | latency=120.1 t90=480.1 overshoot=0.0 tracking=92.3 | latency=80.1 t90=400.1 overshoot=0.022 tracking=100.1 | latency=120.1 t90=440.1 overshoot=0.022 tracking=99.9 |
| L5_knee_lstep-10_r2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=97.2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.4 |

### sysid_champion_stand_ground_v1_20260813_124656.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| traj_rl_stand_20260812_145534 | rmse=2.443 | rmse=2.594 | rmse=2.174 |
