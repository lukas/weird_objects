# Hexapod Sim Reality Report

- generated: 2026-08-12T14:32:36
- git: `c186b79`
- servo params `air`: sim_model.json (6968268e879e)
- servo params `rl_move/sim/sim_model_sysid.json`: sim_model_sysid.json (8b8d2f260256)

## Headline gap metrics

| trace | metric | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|---|
| sysid_steps_air_v1_20260812_181232.csv | unloaded q_rmse_deg | — | 0.375 | 0.246 |
| sysid_steps_air_v1_20260812_181232.csv | unloaded qd_rmse_deg_s | — | 2.19 | 1.71 |
| sysid_steps_air_v1_20260812_181232.csv | cmd→motion latency (hw) | median 119.9 ms (p10 80.0 / p90 120.1 / max 120.3, n=95) | — | — |
| sysid_sines_air_v1_20260812_181941.csv | unloaded q_rmse_deg | — | 0.748 | 0.616 |
| sysid_sines_air_v1_20260812_181941.csv | unloaded qd_rmse_deg_s | — | 5.21 | 3.78 |
| sysid_servo_spread_v1_20260812_182251.csv | unloaded q_rmse_deg | — | 0.276 | 0.215 |
| sysid_servo_spread_v1_20260812_182251.csv | unloaded qd_rmse_deg_s | — | 3.29 | 3.38 |
| sysid_servo_spread_v1_20260812_182251.csv | cmd→motion latency (hw) | median 80.1 ms (p10 79.9 / p90 120.0 / max 120.2, n=108) | — | — |

## Timing (hardware)

- **sysid_steps_air_v1_20260812_181232.csv**: tick median 40.0 ms (nominal 40.0, p95 40.2, late 0.04%); send→recv RTT median 8.1 ms (p95 8.5)
- **sysid_sines_air_v1_20260812_181941.csv**: tick median 40.0 ms (nominal 40.0, p95 40.1, late 0.16%); send→recv RTT median 8.2 ms (p95 8.5)
- **sysid_servo_spread_v1_20260812_182251.csv**: tick median 40.0 ms (nominal 40.0, p95 40.2, late 0.05%); send→recv RTT median 8.0 ms (p95 8.5)

## Servo-to-servo variation (t90)

- sysid_servo_spread_v1_20260812_182251.csv [yaw]: median 400.0 ms over 6 joints, spread 3.8%
- sysid_servo_spread_v1_20260812_182251.csv [hip]: median 420.0 ms over 6 joints, spread 3.3%
- sysid_servo_spread_v1_20260812_182251.csv [knee]: median 400.0 ms over 6 joints, spread 3.7%

## Per-segment detail

### sysid_steps_air_v1_20260812_181232.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| L2_yaw_step+5_r0 | latency=91.9 t90=303.9 overshoot=0.0 tracking=96.7 | latency=42.9 t90=135.4 overshoot=0.0 tracking=91.7 | latency=67.8 t90=159.7 overshoot=0.0 tracking=96.7 |
| L2_yaw_step-5_r0 | latency=120.1 t90=319.8 overshoot=0.0 tracking=94.9 | latency=80.0 t90=279.8 overshoot=0.0 tracking=96.6 | latency=120.1 t90=319.8 overshoot=0.0 tracking=96.3 |
| L2_yaw_step+10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r0 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r0 | latency=119.9 t90=639.9 overshoot=0.0 tracking=99.3 | latency=79.7 t90=679.9 overshoot=0.0 tracking=99.2 | latency=119.9 t90=679.9 overshoot=0.0 tracking=99.1 |
| L2_yaw_step-20_r0 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.1 overshoot=0.0 tracking=99.2 | latency=120.0 t90=680.1 overshoot=0.0 tracking=99.1 |
| L2_yaw_step+5_r1 | latency=120.1 t90=308.4 overshoot=0.0 tracking=94.9 | latency=80.0 t90=199.9 overshoot=0.0 tracking=96.6 | latency=120.1 t90=319.8 overshoot=0.0 tracking=96.4 |
| L2_yaw_step-5_r1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.7 | latency=80.0 t90=280.2 overshoot=0.0 tracking=96.6 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.5 |
| L2_yaw_step+10_r1 | latency=120.1 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.1 overshoot=0.0 tracking=98.3 | latency=120.1 t90=400.1 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r1 | latency=120.2 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.2 t90=680.1 overshoot=0.0 tracking=99.1 | latency=120.2 t90=680.1 overshoot=0.0 tracking=99.2 |
| L2_yaw_step-20_r1 | latency=120.0 t90=639.8 overshoot=0.0 tracking=98.9 | latency=80.1 t90=680.1 overshoot=0.0 tracking=99.1 | latency=120.0 t90=680.1 overshoot=0.0 tracking=99.1 |
| L2_yaw_step+5_r2 | latency=119.9 t90=319.9 overshoot=0.0 tracking=94.9 | latency=79.9 t90=279.9 overshoot=0.0 tracking=96.6 | latency=119.9 t90=319.9 overshoot=0.0 tracking=96.5 |
| L2_yaw_step-5_r2 | latency=119.8 t90=319.8 overshoot=0.0 tracking=93.2 | latency=79.8 t90=279.9 overshoot=0.0 tracking=96.6 | latency=119.8 t90=319.8 overshoot=0.0 tracking=96.4 |
| L2_yaw_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r2 | latency=120.0 t90=400.1 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.1 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r2 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.1 t90=680.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.1 |
| L2_yaw_step-20_r2 | latency=119.9 t90=639.9 overshoot=0.0 tracking=98.9 | latency=79.9 t90=679.9 overshoot=0.0 tracking=99.1 | latency=119.9 t90=679.9 overshoot=0.0 tracking=99.1 |
| L2_yaw_step+5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.1 t90=280.1 overshoot=0.0 tracking=96.6 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.3 |
| L2_yaw_step-5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.1 t90=280.0 overshoot=0.0 tracking=96.6 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.3 |
| L2_yaw_step+10_r3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r3 | latency=120.2 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.2 t90=400.0 overshoot=0.0 tracking=98.4 | latency=120.2 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r3 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.1 |
| L2_yaw_step-20_r3 | latency=120.0 t90=639.9 overshoot=0.0 tracking=98.9 | latency=79.9 t90=679.9 overshoot=0.0 tracking=99.2 | latency=120.0 t90=679.9 overshoot=0.0 tracking=99.1 |
| L2_yaw_step+5_r4 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=280.0 overshoot=0.0 tracking=96.6 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.5 |
| L2_yaw_step-5_r4 | latency=119.9 t90=319.9 overshoot=0.0 tracking=93.2 | latency=79.9 t90=279.9 overshoot=0.0 tracking=96.6 | latency=119.9 t90=319.9 overshoot=0.0 tracking=96.4 |
| L2_yaw_step+10_r4 | latency=120.0 t90=439.7 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=98.4 | latency=120.0 t90=439.7 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r4 | latency=119.7 t90=439.7 overshoot=0.0 tracking=97.6 | latency=79.8 t90=399.5 overshoot=0.0 tracking=98.2 | latency=119.7 t90=439.7 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r4 | latency=120.1 t90=639.8 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.2 | latency=120.1 t90=680.0 overshoot=0.0 tracking=99.1 |
| L2_yaw_step-20_r4 | latency=120.0 t90=639.8 overshoot=0.0 tracking=98.9 | latency=79.8 t90=680.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.1 |
| L2_hip_step+5_r0 | latency=80.0 t90=280.1 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.1 overshoot=0.0 tracking=93.0 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L2_hip_step-5_r0 | latency=80.0 t90=280.1 overshoot=0.01 tracking=100.2 | latency=80.0 t90=240.1 overshoot=0.0 tracking=96.1 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L2_hip_step+10_r0 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L2_hip_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=80.0 t90=440.1 overshoot=0.0 tracking=99.2 |
| L2_hip_step+20_r0 | latency=80.1 t90=640.0 overshoot=0.127 tracking=100.6 | latency=80.1 t90=680.0 overshoot=0.013 tracking=100.0 | latency=80.1 t90=640.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r0 | latency=79.9 t90=639.8 overshoot=0.0 tracking=99.8 | latency=79.9 t90=680.0 overshoot=0.0 tracking=98.6 | latency=79.9 t90=639.8 overshoot=0.0 tracking=99.6 |
| L2_hip_step-40_r0 | latency=79.9 t90=1119.9 overshoot=0.0 tracking=99.5 | latency=79.9 t90=1239.9 overshoot=0.0 tracking=98.2 | latency=79.9 t90=1160.0 overshoot=0.0 tracking=99.8 |
| L2_hip_step+5_r1 | latency=80.1 t90=280.0 overshoot=0.01 tracking=100.2 | latency=80.1 t90=240.1 overshoot=0.001 tracking=99.8 | latency=80.1 t90=320.0 overshoot=0.0 tracking=99.9 |
| L2_hip_step-5_r1 | latency=80.2 t90=280.3 overshoot=0.0 tracking=98.4 | latency=80.2 t90=280.3 overshoot=0.0 tracking=94.6 | latency=80.2 t90=320.3 overshoot=0.0 tracking=98.3 |
| L2_hip_step+10_r1 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.1 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r1 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step+20_r1 | latency=80.1 t90=640.0 overshoot=0.127 tracking=100.6 | latency=80.1 t90=680.1 overshoot=0.013 tracking=100.0 | latency=80.1 t90=640.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r1 | latency=80.0 t90=640.0 overshoot=0.039 tracking=100.2 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.6 |
| L2_hip_step-40_r1 | latency=80.3 t90=1120.0 overshoot=0.0 tracking=99.6 | latency=80.3 t90=1240.2 overshoot=0.0 tracking=98.5 | latency=80.3 t90=1160.2 overshoot=0.02 tracking=100.0 |
| L2_hip_step+5_r2 | latency=79.8 t90=280.1 overshoot=0.009 tracking=100.2 | latency=79.8 t90=240.2 overshoot=0.015 tracking=100.1 | latency=79.8 t90=320.0 overshoot=0.0 tracking=99.9 |
| L2_hip_step-5_r2 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.0 overshoot=0.0 tracking=95.1 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L2_hip_step+10_r2 | latency=80.1 t90=400.0 overshoot=0.108 tracking=101.1 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.1 t90=440.0 overshoot=0.0 tracking=99.1 |
| L2_hip_step-10_r2 | latency=80.1 t90=400.2 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.2 overshoot=0.0 tracking=97.5 | latency=80.1 t90=440.1 overshoot=0.0 tracking=99.2 |
| L2_hip_step+20_r2 | latency=80.0 t90=640.0 overshoot=0.039 tracking=100.2 | latency=80.0 t90=680.0 overshoot=0.012 tracking=100.0 | latency=80.0 t90=640.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r2 | latency=79.8 t90=640.0 overshoot=0.0 tracking=99.8 | latency=79.8 t90=680.0 overshoot=0.0 tracking=99.2 | latency=79.8 t90=640.0 overshoot=0.0 tracking=99.6 |
| L2_hip_step-40_r2 | latency=80.0 t90=1120.2 overshoot=0.0 tracking=99.5 | latency=80.0 t90=1240.1 overshoot=0.0 tracking=98.3 | latency=80.0 t90=1160.0 overshoot=0.0 tracking=99.8 |
| L2_hip_step+5_r3 | latency=79.8 t90=240.0 overshoot=0.01 tracking=100.2 | latency=79.8 t90=240.0 overshoot=0.004 tracking=100.1 | latency=79.8 t90=319.8 overshoot=0.0 tracking=99.9 |
| L2_hip_step-5_r3 | latency=80.1 t90=279.9 overshoot=0.0 tracking=98.4 | latency=80.1 t90=279.9 overshoot=0.0 tracking=95.1 | latency=80.1 t90=319.9 overshoot=0.0 tracking=98.4 |
| L2_hip_step+10_r3 | latency=79.9 t90=400.8 overshoot=0.019 tracking=100.2 | latency=79.9 t90=400.8 overshoot=0.0 tracking=99.0 | latency=79.9 t90=439.7 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r3 | latency=79.7 t90=399.9 overshoot=0.0 tracking=99.3 | latency=79.7 t90=399.9 overshoot=0.0 tracking=98.2 | latency=79.7 t90=439.9 overshoot=0.0 tracking=99.2 |
| L2_hip_step+20_r3 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=679.8 overshoot=0.012 tracking=100.0 | latency=80.0 t90=640.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r3 | latency=80.0 t90=640.1 overshoot=0.039 tracking=100.2 | latency=80.0 t90=679.8 overshoot=0.0 tracking=98.6 | latency=80.0 t90=640.1 overshoot=0.0 tracking=99.6 |
| L2_hip_step-40_r3 | latency=80.2 t90=1120.2 overshoot=0.0 tracking=99.5 | latency=80.2 t90=1240.2 overshoot=0.0 tracking=98.3 | latency=80.2 t90=1160.2 overshoot=0.0 tracking=99.8 |
| L2_hip_step+5_r4 | latency=80.1 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=240.0 overshoot=0.012 tracking=100.1 | latency=80.1 t90=320.0 overshoot=0.006 tracking=100.1 |
| L2_hip_step-5_r4 | latency=79.7 t90=279.9 overshoot=0.01 tracking=100.2 | latency=79.7 t90=279.9 overshoot=0.0 tracking=94.6 | latency=79.7 t90=319.9 overshoot=0.0 tracking=98.3 |
| L2_hip_step+10_r4 | latency=80.0 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r4 | latency=80.0 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=80.0 t90=440.1 overshoot=0.0 tracking=99.2 |
| L2_hip_step+20_r4 | latency=79.8 t90=640.0 overshoot=0.127 tracking=100.6 | latency=79.8 t90=680.0 overshoot=0.012 tracking=100.0 | latency=79.8 t90=640.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r4 | latency=80.0 t90=640.1 overshoot=0.127 tracking=100.6 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=80.0 t90=640.1 overshoot=0.0 tracking=99.6 |
| L2_hip_step-40_r4 | latency=80.1 t90=1120.0 overshoot=0.0 tracking=99.5 | latency=80.1 t90=1240.0 overshoot=0.0 tracking=98.3 | latency=80.1 t90=1160.0 overshoot=0.0 tracking=99.8 |
| L2_knee_step+5_r0 | latency=120.1 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.2 t90=280.3 overshoot=0.0 tracking=89.9 | latency=120.1 t90=320.0 overshoot=0.0 tracking=94.4 |
| L2_knee_step-5_r0 | latency=120.2 t90=360.2 overshoot=0.0 tracking=93.2 | latency=80.2 t90=280.2 overshoot=0.0 tracking=92.5 | latency=120.2 t90=320.2 overshoot=0.0 tracking=92.7 |
| L2_knee_step+10_r0 | latency=80.0 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.8 t90=440.1 overshoot=0.0 tracking=97.4 |
| L2_knee_step-10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.8 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 |
| L2_knee_step+20_r0 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=79.8 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.7 |
| L2_knee_step+40_r0 | latency=120.0 t90=1160.1 overshoot=0.0 tracking=98.9 | latency=80.0 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=120.0 t90=1200.0 overshoot=0.0 tracking=99.7 |
| L2_knee_step+5_r1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.0 overshoot=0.0 tracking=87.7 | latency=120.0 t90=320.0 overshoot=0.0 tracking=95.9 |
| L2_knee_step-5_r1 | latency=120.0 overshoot=0.0 tracking=89.6 | latency=80.0 overshoot=0.0 tracking=89.3 | latency=120.0 overshoot=0.0 tracking=89.4 |
| L2_knee_step+10_r1 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.5 |
| L2_knee_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=94.9 | latency=79.8 t90=400.0 overshoot=0.0 tracking=95.6 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.6 |
| L2_knee_step+20_r1 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.1 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.1 overshoot=0.0 tracking=98.7 |
| L2_knee_step+40_r1 | latency=119.9 t90=1159.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=119.9 t90=1200.0 overshoot=0.0 tracking=99.7 |
| L2_knee_step+5_r2 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.1 overshoot=0.0 tracking=87.7 | latency=120.0 t90=320.0 overshoot=0.0 tracking=95.9 |
| L2_knee_step-5_r2 | latency=120.2 t90=320.2 overshoot=0.0 tracking=93.2 | latency=80.2 t90=280.2 overshoot=0.0 tracking=91.2 | latency=120.2 t90=360.2 overshoot=0.0 tracking=91.6 |
| L2_knee_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=79.8 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.2 |
| L2_knee_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.5 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.6 |
| L2_knee_step+20_r2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.6 |
| L2_knee_step+40_r2 | latency=120.0 t90=1160.0 overshoot=0.0 tracking=98.7 | latency=80.0 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=120.0 t90=1200.1 overshoot=0.0 tracking=99.7 |
| L2_knee_step+5_r3 | latency=119.9 t90=319.7 overshoot=0.0 tracking=94.9 | latency=80.0 overshoot=0.0 tracking=89.3 | latency=119.9 t90=319.7 overshoot=0.0 tracking=97.9 |
| L2_knee_step-5_r3 | latency=120.0 t90=360.0 overshoot=0.0 tracking=93.2 | latency=80.1 t90=280.0 overshoot=0.0 tracking=91.2 | latency=120.0 t90=360.0 overshoot=0.0 tracking=91.2 |
| L2_knee_step+10_r3 | latency=120.0 t90=439.8 overshoot=0.0 tracking=97.6 | latency=80.0 t90=399.8 overshoot=0.0 tracking=96.7 | latency=120.0 t90=439.8 overshoot=0.0 tracking=96.5 |
| L2_knee_step-10_r3 | latency=119.9 t90=439.9 overshoot=0.0 tracking=94.9 | latency=79.9 t90=400.0 overshoot=0.0 tracking=95.5 | latency=119.9 t90=439.9 overshoot=0.0 tracking=95.6 |
| L2_knee_step+20_r3 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.7 |
| L2_knee_step+40_r3 | latency=119.9 t90=1159.9 overshoot=0.0 tracking=98.7 | latency=79.9 t90=1279.8 overshoot=0.0 tracking=98.5 | latency=119.9 t90=1199.9 overshoot=0.0 tracking=99.5 |
| L2_knee_step+5_r4 | latency=120.3 t90=320.3 overshoot=0.0 tracking=94.9 | latency=80.0 t90=280.2 overshoot=0.0 tracking=91.2 | latency=120.3 t90=320.3 overshoot=0.007 tracking=99.5 |
| L2_knee_step-5_r4 | latency=119.6 overshoot=0.0 tracking=89.6 | latency=79.6 overshoot=0.0 tracking=89.3 | latency=119.6 t90=359.6 overshoot=0.0 tracking=89.5 |
| L2_knee_step+10_r4 | latency=120.0 t90=440.1 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.1 overshoot=0.0 tracking=97.5 |
| L2_knee_step-10_r4 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.8 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.6 |
| L2_knee_step+20_r4 | latency=120.0 t90=640.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.3 |
| L2_knee_step+40_r4 | latency=120.1 t90=1160.0 overshoot=0.0 tracking=98.4 | latency=79.8 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=120.1 t90=1200.0 overshoot=0.0 tracking=99.7 |

### sysid_sines_air_v1_20260812_181941.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| L2_yaw_sine4@0.25Hz | gain=0.9915 phase=116.2 rmse=0.528 | gain=0.9652 phase=126.5 rmse=0.559 | gain=1.0072 phase=128.1 rmse=0.566 |
| L2_yaw_sine4@0.5Hz | gain=0.9912 phase=121.0 rmse=0.912 | gain=0.9471 phase=99.6 rmse=0.755 | gain=1.0073 phase=128.6 rmse=0.972 |
| L2_yaw_sine4@1Hz | gain=1.0158 phase=128.1 rmse=1.923 | gain=1.0113 phase=95.7 rmse=1.5 | gain=1.053 phase=140.8 rmse=2.146 |
| L2_yaw_sine4@2Hz | gain=0.4471 phase=-230.5 rmse=3.631 | gain=0.3842 phase=179.1 rmse=3.516 | gain=0.3507 phase=-249.5 rmse=3.769 |
| L2_yaw_sine8@0.25Hz | gain=1.0082 phase=110.8 rmse=0.987 | gain=0.9941 phase=90.1 rmse=0.799 | gain=1.015 phase=114.7 rmse=1.021 |
| L2_yaw_sine8@0.5Hz | gain=0.9995 phase=139.3 rmse=2.089 | gain=0.9838 phase=91.7 rmse=1.378 | gain=1.011 phase=149.8 rmse=2.256 |
| L2_yaw_sine8@1Hz | gain=0.9053 phase=315.1 rmse=7.658 | gain=0.7516 phase=242.6 rmse=6.353 | gain=0.8008 phase=296.0 rmse=7.572 |
| L2_yaw_sine8@2Hz | gain=0.2093 phase=-210.4 rmse=6.166 | gain=0.1867 phase=220.1 rmse=6.419 | gain=0.1784 phase=-211.8 rmse=6.387 |
| L2_hip_sine4@0.25Hz | gain=0.9986 phase=92.0 rmse=0.421 | gain=0.9748 phase=99.7 rmse=0.44 | gain=1.0104 phase=78.1 rmse=0.349 |
| L2_hip_sine4@0.5Hz | gain=0.9849 phase=95.3 rmse=0.716 | gain=0.9552 phase=74.9 rmse=0.575 | gain=1.0029 phase=88.0 rmse=0.665 |
| L2_hip_sine4@1Hz | gain=1.0045 phase=106.5 rmse=1.61 | gain=0.9977 phase=76.8 rmse=1.183 | gain=1.0185 phase=107.2 rmse=1.611 |
| L2_hip_sine4@2Hz | gain=0.4714 phase=239.9 rmse=3.669 | gain=0.3847 phase=168.9 rmse=3.425 | gain=0.377 phase=218.2 rmse=3.69 |
| L2_hip_sine8@0.25Hz | gain=1.0093 phase=87.8 rmse=0.786 | gain=0.9953 phase=70.3 rmse=0.624 | gain=1.0112 phase=80.8 rmse=0.719 |
| L2_hip_sine8@0.5Hz | gain=0.9963 phase=115.5 rmse=1.731 | gain=0.99 phase=79.3 rmse=1.197 | gain=0.9968 phase=113.3 rmse=1.703 |
| L2_hip_sine8@1Hz | gain=0.9084 phase=299.2 rmse=7.381 | gain=0.7343 phase=207.8 rmse=5.677 | gain=0.7739 phase=263.8 rmse=6.862 |
| L2_hip_sine8@2Hz | gain=0.2446 phase=-230.0 rmse=6.379 | gain=0.176 phase=217.4 rmse=6.412 | gain=0.169 phase=-236.4 rmse=6.39 |
| L2_knee_sine4@0.25Hz | gain=0.9622 phase=146.8 rmse=0.64 | gain=0.9547 phase=130.6 rmse=0.575 | gain=0.9945 phase=152.7 rmse=0.667 |
| L2_knee_sine4@0.5Hz | gain=0.9461 phase=138.4 rmse=1.016 | gain=0.9276 phase=97.1 rmse=0.731 | gain=0.9906 phase=142.7 rmse=1.068 |
| L2_knee_sine4@1Hz | gain=0.9694 phase=136.3 rmse=2.012 | gain=0.9882 phase=86.5 rmse=1.363 | gain=1.0553 phase=147.3 rmse=2.229 |
| L2_knee_sine4@2Hz | gain=0.3827 phase=-227.3 rmse=3.473 | gain=0.3704 phase=171.0 rmse=3.46 | gain=0.3497 phase=-249.8 rmse=3.779 |
| L2_knee_sine8@0.25Hz | gain=0.9987 phase=125.3 rmse=1.103 | gain=0.9896 phase=88.0 rmse=0.775 | gain=1.0121 phase=128.8 rmse=1.144 |
| L2_knee_sine8@0.5Hz | gain=0.9883 phase=148.4 rmse=2.202 | gain=0.9736 phase=82.5 rmse=1.232 | gain=1.0013 phase=151.1 rmse=2.259 |
| L2_knee_sine8@1Hz | gain=0.8841 phase=324.4 rmse=7.672 | gain=0.8016 phase=199.0 rmse=5.315 | gain=0.8152 phase=292.0 rmse=7.098 |
| L2_knee_sine8@2Hz | gain=0.1875 phase=-209.0 rmse=6.019 | gain=0.1815 phase=211.6 rmse=6.376 | gain=0.1731 phase=-212.2 rmse=6.365 |

### sysid_servo_spread_v1_20260812_182251.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| L0_yaw_step+10_r0 | latency=79.9 t90=399.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 |
| L0_yaw_step-10_r0 | latency=52.4 overshoot=0.0 tracking=75.6 | latency=129.6 overshoot=0.0 tracking=19.0 | latency=0.0 overshoot=0.0 tracking=22.2 |
| L0_yaw_step+10_r1 | latency=40.0 t90=440.0 overshoot=0.371 tracking=100.2 | latency=79.9 t90=399.8 overshoot=0.0 tracking=99.1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.1 |
| L0_yaw_step-10_r1 | latency=40.0 t90=400.0 overshoot=0.459 tracking=102.4 | latency=80.0 t90=400.0 overshoot=0.028 tracking=100.1 | latency=120.0 t90=439.8 overshoot=0.0 tracking=100.0 |
| L0_yaw_step+10_r2 | latency=40.0 t90=400.0 overshoot=0.459 tracking=101.7 | latency=79.9 t90=400.0 overshoot=0.017 tracking=99.9 | latency=120.0 t90=440.0 overshoot=0.001 tracking=100.0 |
| L0_yaw_step-10_r2 | latency=40.0 t90=359.9 overshoot=0.284 tracking=99.1 | latency=80.0 t90=400.0 overshoot=0.02 tracking=99.9 | latency=120.0 t90=439.9 overshoot=0.0 tracking=99.9 |
| L0_hip_step+10_r0 | latency=119.9 t90=400.0 overshoot=0.0 tracking=99.3 | latency=79.9 t90=400.0 overshoot=0.0 tracking=96.5 | latency=79.9 t90=440.0 overshoot=0.0 tracking=99.2 |
| L0_hip_step-10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=96.4 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L0_hip_step+10_r1 | latency=80.0 t90=400.1 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.1 overshoot=0.015 tracking=100.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=100.0 |
| L0_hip_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=399.7 overshoot=0.0 tracking=96.5 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L0_hip_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L0_hip_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=79.9 t90=399.9 overshoot=0.0 tracking=95.7 | latency=79.9 t90=440.0 overshoot=0.0 tracking=99.2 |
| L0_knee_step+10_r0 | latency=119.9 t90=400.0 overshoot=0.371 tracking=102.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.8 | latency=119.9 t90=440.0 overshoot=0.0 tracking=97.2 |
| L0_knee_step-10_r0 | latency=120.0 t90=399.8 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.8 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L0_knee_step+10_r1 | latency=119.9 t90=440.0 overshoot=0.195 tracking=102.0 | latency=80.0 t90=399.9 overshoot=0.013 tracking=100.0 | latency=119.9 t90=440.0 overshoot=0.031 tracking=100.0 |
| L0_knee_step-10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.2 |
| L0_knee_step+10_r2 | latency=120.0 t90=400.0 overshoot=0.459 tracking=103.7 | latency=80.0 t90=400.0 overshoot=0.009 tracking=99.9 | latency=120.0 t90=439.9 overshoot=0.031 tracking=100.0 |
| L0_knee_step-10_r2 | latency=120.1 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.1 overshoot=0.0 tracking=98.3 | latency=120.1 t90=440.1 overshoot=0.0 tracking=98.2 |
| L1_yaw_step+10_r0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.1 overshoot=0.0 tracking=97.7 | latency=120.1 t90=440.1 overshoot=0.001 tracking=100.0 |
| L1_yaw_step-10_r0 | latency=80.5 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.5 t90=400.0 overshoot=0.0 tracking=94.7 | latency=120.0 t90=439.9 overshoot=0.0 tracking=96.8 |
| L1_yaw_step+10_r1 | latency=120.1 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.1 | latency=120.1 t90=440.0 overshoot=0.0 tracking=98.2 |
| L1_yaw_step-10_r1 | latency=119.9 t90=400.0 overshoot=0.0 tracking=98.4 | latency=79.9 t90=400.0 overshoot=0.0 tracking=97.2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=97.4 |
| L1_yaw_step+10_r2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.5 | latency=80.1 t90=400.0 overshoot=0.0 tracking=97.5 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.4 |
| L1_yaw_step-10_r2 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.8 |
| L1_hip_step+10_r0 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=96.5 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L1_hip_step-10_r0 | latency=120.0 t90=440.1 overshoot=0.0 tracking=94.9 | latency=80.1 t90=400.0 overshoot=0.0 tracking=94.6 | latency=80.1 t90=440.1 overshoot=0.0 tracking=99.2 |
| L1_hip_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L1_hip_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L1_hip_step+10_r2 | latency=120.0 t90=400.1 overshoot=0.0 tracking=99.3 | latency=79.9 t90=400.1 overshoot=0.0 tracking=99.1 | latency=79.9 t90=440.1 overshoot=0.0 tracking=99.2 |
| L1_hip_step-10_r2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L1_knee_step+10_r0 | latency=79.9 t90=400.0 overshoot=0.107 tracking=101.1 | latency=79.9 t90=400.0 overshoot=0.0 tracking=94.8 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.2 |
| L1_knee_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.9 | latency=120.0 t90=439.9 overshoot=0.0 tracking=99.1 |
| L1_knee_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.3 |
| L1_knee_step-10_r1 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.2 | latency=119.9 t90=440.1 overshoot=0.0 tracking=98.4 |
| L1_knee_step+10_r2 | latency=79.7 t90=400.0 overshoot=0.0 tracking=99.3 | latency=79.7 t90=400.0 overshoot=0.0 tracking=99.8 | latency=120.0 t90=440.0 overshoot=0.011 tracking=99.8 |
| L1_knee_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.019 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.3 | latency=120.0 t90=439.9 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+10_r0 | latency=119.9 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=119.9 t90=440.0 overshoot=0.0 tracking=98.4 |
| L2_yaw_step-10_r0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.4 | latency=120.1 t90=440.1 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=79.9 t90=400.0 overshoot=0.0 tracking=98.4 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L2_hip_step+10_r0 | latency=79.8 t90=400.0 overshoot=0.108 tracking=101.1 | latency=79.8 t90=400.0 overshoot=0.0 tracking=96.5 | latency=79.8 t90=439.9 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=80.0 t90=439.9 overshoot=0.0 tracking=99.2 |
| L2_hip_step+10_r1 | latency=80.1 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.1 t90=400.0 overshoot=0.005 tracking=100.0 | latency=80.1 t90=440.1 overshoot=0.0 tracking=100.0 |
| L2_hip_step-10_r1 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=97.3 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.013 tracking=100.1 | latency=80.0 t90=440.1 overshoot=0.0 tracking=100.0 |
| L2_hip_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L2_knee_step+10_r0 | latency=120.0 t90=439.8 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.9 | latency=120.0 t90=439.8 overshoot=0.0 tracking=97.2 |
| L2_knee_step-10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.6 | latency=120.0 t90=440.0 overshoot=0.0 tracking=94.7 |
| L2_knee_step+10_r1 | latency=120.1 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=96.4 | latency=120.1 t90=440.0 overshoot=0.0 tracking=96.6 |
| L2_knee_step-10_r1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=399.8 overshoot=0.0 tracking=95.5 | latency=119.9 t90=440.0 overshoot=0.0 tracking=95.6 |
| L2_knee_step+10_r2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.1 overshoot=0.0 tracking=97.3 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.3 |
| L2_knee_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=79.7 t90=400.0 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.6 |
| L3_yaw_step+10_r0 | latency=101.3 t90=381.3 overshoot=0.0 tracking=99.3 | latency=61.3 t90=421.2 overshoot=0.0 tracking=95.7 | latency=141.3 t90=461.3 overshoot=0.0 tracking=98.4 |
| L3_yaw_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=120.1 t90=440.0 overshoot=0.0 tracking=99.1 |
| L3_yaw_step+10_r1 | latency=80.0 t90=399.8 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.8 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L3_yaw_step-10_r1 | latency=80.0 t90=404.2 overshoot=0.0 tracking=98.4 | latency=80.0 t90=404.2 overshoot=0.017 tracking=100.0 | latency=120.0 t90=440.0 overshoot=0.001 tracking=100.0 |
| L3_yaw_step+10_r2 | latency=80.1 t90=400.1 overshoot=0.02 tracking=100.2 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.3 | latency=120.0 t90=440.1 overshoot=0.0 tracking=99.2 |
| L3_yaw_step-10_r2 | latency=80.0 t90=399.9 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.0 |
| L3_hip_step+10_r0 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=96.5 | latency=80.0 t90=439.7 overshoot=0.0 tracking=99.2 |
| L3_hip_step-10_r0 | latency=119.9 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.5 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L3_hip_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=439.9 overshoot=0.0 tracking=99.2 |
| L3_hip_step-10_r1 | latency=120.1 t90=440.1 overshoot=0.0 tracking=96.7 | latency=80.1 t90=400.0 overshoot=0.0 tracking=95.5 | latency=80.1 t90=440.1 overshoot=0.0 tracking=99.2 |
| L3_hip_step+10_r2 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.3 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.1 | latency=79.9 t90=439.9 overshoot=0.0 tracking=99.2 |
| L3_hip_step-10_r2 | latency=120.0 t90=439.9 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=80.0 t90=439.9 overshoot=0.0 tracking=99.2 |
| L3_knee_step+10_r0 | latency=80.0 t90=400.1 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.1 overshoot=0.0 tracking=94.8 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.2 |
| L3_knee_step-10_r0 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=98.2 |
| L3_knee_step+10_r1 | latency=79.9 t90=399.8 overshoot=0.0 tracking=99.3 | latency=79.9 t90=399.8 overshoot=0.0 tracking=98.9 | latency=120.0 t90=440.1 overshoot=0.0 tracking=99.1 |
| L3_knee_step-10_r1 | latency=80.3 t90=400.3 overshoot=0.02 tracking=100.2 | latency=80.3 t90=400.3 overshoot=0.0 tracking=99.2 | latency=120.3 t90=440.2 overshoot=0.0 tracking=99.1 |
| L3_knee_step+10_r2 | latency=80.2 t90=400.0 overshoot=0.019 tracking=100.2 | latency=80.2 t90=400.0 overshoot=0.0 tracking=99.8 | latency=120.3 t90=440.1 overshoot=0.046 tracking=100.1 |
| L3_knee_step-10_r2 | latency=80.1 t90=400.1 overshoot=0.019 tracking=99.5 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.2 | latency=120.1 t90=439.8 overshoot=0.0 tracking=99.1 |
| L4_yaw_step+10_r0 | latency=120.0 t90=391.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.7 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 |
| L4_yaw_step-10_r0 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.3 | latency=120.1 t90=440.0 overshoot=0.0 tracking=99.1 |
| L4_yaw_step+10_r1 | latency=80.1 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.1 overshoot=0.0 tracking=99.2 |
| L4_yaw_step-10_r1 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L4_yaw_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.0 |
| L4_yaw_step-10_r2 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.0 |
| L4_hip_step+10_r0 | latency=79.9 t90=399.9 overshoot=0.107 tracking=101.1 | latency=79.9 t90=399.9 overshoot=0.0 tracking=96.5 | latency=79.9 t90=440.0 overshoot=0.0 tracking=99.2 |
| L4_hip_step-10_r0 | latency=79.9 t90=400.0 overshoot=0.019 tracking=100.2 | latency=79.9 t90=400.0 overshoot=0.0 tracking=98.2 | latency=79.9 t90=439.9 overshoot=0.0 tracking=99.2 |
| L4_hip_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=100.0 | latency=80.0 t90=440.0 overshoot=0.0 tracking=100.0 |
| L4_hip_step-10_r1 | latency=79.9 t90=400.0 overshoot=0.0 tracking=99.3 | latency=79.9 t90=400.0 overshoot=0.0 tracking=98.2 | latency=79.9 t90=439.9 overshoot=0.0 tracking=99.2 |
| L4_hip_step+10_r2 | latency=79.9 t90=399.9 overshoot=0.02 tracking=100.2 | latency=79.9 t90=399.9 overshoot=0.011 tracking=100.1 | latency=79.9 t90=440.0 overshoot=0.0 tracking=100.0 |
| L4_hip_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L4_knee_step+10_r0 | latency=120.0 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.8 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.2 |
| L4_knee_step-10_r0 | latency=80.1 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.8 | latency=120.1 t90=440.1 overshoot=0.032 tracking=100.0 |
| L4_knee_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.022 tracking=100.1 | latency=120.1 t90=440.1 overshoot=0.045 tracking=100.1 |
| L4_knee_step-10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.1 |
| L4_knee_step+10_r2 | latency=80.1 t90=399.9 overshoot=0.019 tracking=100.2 | latency=80.1 t90=399.9 overshoot=0.009 tracking=99.9 | latency=120.0 t90=440.1 overshoot=0.041 tracking=100.1 |
| L4_knee_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=119.9 t90=439.8 overshoot=0.0 tracking=99.1 |
| L5_yaw_step+10_r0 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 |
| L5_yaw_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.019 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.017 tracking=100.0 | latency=119.9 t90=440.0 overshoot=0.01 tracking=100.1 |
| L5_yaw_step+10_r1 | latency=80.1 t90=400.1 overshoot=0.283 tracking=99.3 | latency=80.1 t90=400.1 overshoot=0.017 tracking=100.0 | latency=120.0 t90=440.1 overshoot=0.001 tracking=100.0 |
| L5_yaw_step-10_r1 | latency=80.1 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.1 t90=400.0 overshoot=0.017 tracking=100.0 | latency=120.1 t90=445.2 overshoot=0.006 tracking=99.9 |
| L5_yaw_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.371 tracking=99.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.5 | latency=120.0 t90=439.9 overshoot=0.0 tracking=97.4 |
| L5_yaw_step-10_r2 | latency=0.0 t90=400.0 overshoot=0.0 tracking=98.7 | latency=80.0 t90=400.0 overshoot=0.028 tracking=100.1 | latency=120.0 t90=440.0 overshoot=0.001 tracking=100.0 |
| L5_hip_step+10_r0 | latency=120.2 t90=440.3 overshoot=0.0 tracking=98.4 | latency=80.2 t90=400.3 overshoot=0.0 tracking=96.5 | latency=80.2 t90=440.3 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=95.8 | latency=80.1 t90=400.0 overshoot=0.0 tracking=95.7 | latency=80.1 t90=440.1 overshoot=0.0 tracking=99.2 |
| L5_hip_step+10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=94.9 | latency=79.8 t90=400.0 overshoot=0.0 tracking=94.7 | latency=79.8 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=79.9 t90=400.0 overshoot=0.0 tracking=99.1 | latency=79.9 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=399.9 overshoot=0.0 tracking=93.8 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_knee_step+10_r0 | latency=120.0 t90=414.1 overshoot=0.0 tracking=98.4 | latency=79.9 t90=414.1 overshoot=0.0 tracking=95.8 | latency=120.0 t90=439.9 overshoot=0.0 tracking=97.9 |
| L5_knee_step-10_r0 | latency=120.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L5_knee_step+10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.1 overshoot=0.0 tracking=99.1 |
| L5_knee_step-10_r1 | latency=120.0 t90=399.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L5_knee_step+10_r2 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=439.9 overshoot=0.0 tracking=99.1 |
| L5_knee_step-10_r2 | latency=119.9 t90=400.0 overshoot=0.0 tracking=98.4 | latency=79.9 t90=400.0 overshoot=0.0 tracking=99.2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.1 |
