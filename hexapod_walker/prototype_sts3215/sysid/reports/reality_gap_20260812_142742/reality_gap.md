# Hexapod Sim Reality Report

- generated: 2026-08-12T14:27:42
- git: `c186b79`
- servo params `air`: sim_model.json (6968268e879e)
- servo params `loaded`: sim_model_loaded.json (144d43fa5dd3)

## Headline gap metrics

| trace | metric | hardware | sim air | sim loaded |
|---|---|---|---|---|
| sysid_steps_air_v1_20260812_181232.csv | unloaded q_rmse_deg | — | 0.375 | 0.436 |
| sysid_steps_air_v1_20260812_181232.csv | unloaded qd_rmse_deg_s | — | 2.19 | 2.57 |
| sysid_steps_air_v1_20260812_181232.csv | cmd→motion latency (hw) | median 119.9 ms (p10 80.0 / p90 120.1 / max 120.3, n=95) | — | — |
| sysid_sines_air_v1_20260812_181941.csv | unloaded q_rmse_deg | — | 0.748 | 0.667 |
| sysid_sines_air_v1_20260812_181941.csv | unloaded qd_rmse_deg_s | — | 5.21 | 4.37 |
| sysid_servo_spread_v1_20260812_182251.csv | unloaded q_rmse_deg | — | 0.276 | 0.28 |
| sysid_servo_spread_v1_20260812_182251.csv | unloaded qd_rmse_deg_s | — | 3.29 | 3.52 |
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

| segment | hardware | sim air | sim loaded |
|---|---|---|---|
| L2_yaw_step+5_r0 | latency=91.9 t90=303.9 overshoot=0.0 tracking=96.7 | latency=42.9 t90=135.4 overshoot=0.0 tracking=91.7 | latency=75.8 t90=167.9 overshoot=0.0 tracking=91.7 |
| L2_yaw_step-5_r0 | latency=120.1 t90=319.8 overshoot=0.0 tracking=94.9 | latency=80.0 t90=279.8 overshoot=0.0 tracking=96.6 | latency=160.0 t90=360.0 overshoot=0.0 tracking=96.6 |
| L2_yaw_step+10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=159.8 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r0 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.1 | latency=160.0 t90=479.8 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r0 | latency=119.9 t90=639.9 overshoot=0.0 tracking=99.3 | latency=79.7 t90=679.9 overshoot=0.0 tracking=99.2 | latency=159.9 t90=719.9 overshoot=0.0 tracking=99.2 |
| L2_yaw_step-20_r0 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.1 overshoot=0.0 tracking=99.2 | latency=160.0 t90=720.0 overshoot=0.0 tracking=99.2 |
| L2_yaw_step+5_r1 | latency=120.1 t90=308.4 overshoot=0.0 tracking=94.9 | latency=80.0 t90=199.9 overshoot=0.0 tracking=96.6 | latency=160.2 t90=319.8 overshoot=0.0 tracking=96.6 |
| L2_yaw_step-5_r1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.7 | latency=80.0 t90=280.2 overshoot=0.0 tracking=96.6 | latency=160.0 t90=360.0 overshoot=0.0 tracking=96.6 |
| L2_yaw_step+10_r1 | latency=120.1 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.1 overshoot=0.0 tracking=98.3 | latency=160.1 t90=400.1 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=160.1 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r1 | latency=120.2 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.2 t90=680.1 overshoot=0.0 tracking=99.1 | latency=160.3 t90=720.2 overshoot=0.0 tracking=99.0 |
| L2_yaw_step-20_r1 | latency=120.0 t90=639.8 overshoot=0.0 tracking=98.9 | latency=80.1 t90=680.1 overshoot=0.0 tracking=99.1 | latency=160.1 t90=720.0 overshoot=0.0 tracking=99.2 |
| L2_yaw_step+5_r2 | latency=119.9 t90=319.9 overshoot=0.0 tracking=94.9 | latency=79.9 t90=279.9 overshoot=0.0 tracking=96.6 | latency=160.0 t90=360.0 overshoot=0.0 tracking=96.6 |
| L2_yaw_step-5_r2 | latency=119.8 t90=319.8 overshoot=0.0 tracking=93.2 | latency=79.8 t90=279.9 overshoot=0.0 tracking=96.6 | latency=159.8 t90=359.8 overshoot=0.0 tracking=96.6 |
| L2_yaw_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.2 | latency=160.0 t90=480.1 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r2 | latency=120.0 t90=400.1 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.1 overshoot=0.0 tracking=98.2 | latency=160.0 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r2 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.1 t90=680.0 overshoot=0.0 tracking=99.2 | latency=160.0 t90=720.0 overshoot=0.0 tracking=99.0 |
| L2_yaw_step-20_r2 | latency=119.9 t90=639.9 overshoot=0.0 tracking=98.9 | latency=79.9 t90=679.9 overshoot=0.0 tracking=99.1 | latency=159.9 t90=720.0 overshoot=0.0 tracking=99.0 |
| L2_yaw_step+5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.1 t90=280.1 overshoot=0.0 tracking=96.6 | latency=160.0 t90=359.8 overshoot=0.0 tracking=96.6 |
| L2_yaw_step-5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.1 t90=280.0 overshoot=0.0 tracking=96.6 | latency=160.1 t90=360.0 overshoot=0.0 tracking=96.6 |
| L2_yaw_step+10_r3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.3 | latency=160.0 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r3 | latency=120.2 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.2 t90=400.0 overshoot=0.0 tracking=98.4 | latency=160.2 t90=480.2 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r3 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.2 | latency=160.0 t90=720.0 overshoot=0.0 tracking=99.2 |
| L2_yaw_step-20_r3 | latency=120.0 t90=639.9 overshoot=0.0 tracking=98.9 | latency=79.9 t90=679.9 overshoot=0.0 tracking=99.2 | latency=159.9 t90=719.9 overshoot=0.0 tracking=99.2 |
| L2_yaw_step+5_r4 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=280.0 overshoot=0.0 tracking=96.6 | latency=160.0 t90=360.0 overshoot=0.0 tracking=96.6 |
| L2_yaw_step-5_r4 | latency=119.9 t90=319.9 overshoot=0.0 tracking=93.2 | latency=79.9 t90=279.9 overshoot=0.0 tracking=96.6 | latency=159.9 t90=359.9 overshoot=0.0 tracking=96.6 |
| L2_yaw_step+10_r4 | latency=120.0 t90=439.7 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=98.4 | latency=159.9 t90=479.9 overshoot=0.0 tracking=98.5 |
| L2_yaw_step-10_r4 | latency=119.7 t90=439.7 overshoot=0.0 tracking=97.6 | latency=79.8 t90=399.5 overshoot=0.0 tracking=98.2 | latency=159.7 t90=479.7 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+20_r4 | latency=120.1 t90=639.8 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.2 | latency=160.0 t90=719.9 overshoot=0.0 tracking=99.0 |
| L2_yaw_step-20_r4 | latency=120.0 t90=639.8 overshoot=0.0 tracking=98.9 | latency=79.8 t90=680.0 overshoot=0.0 tracking=99.2 | latency=160.0 t90=720.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step+5_r0 | latency=80.0 t90=280.1 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.1 overshoot=0.0 tracking=93.0 | latency=120.1 t90=320.0 overshoot=0.0 tracking=93.0 |
| L2_hip_step-5_r0 | latency=80.0 t90=280.1 overshoot=0.01 tracking=100.2 | latency=80.0 t90=240.1 overshoot=0.0 tracking=96.1 | latency=119.8 t90=320.0 overshoot=0.0 tracking=96.1 |
| L2_hip_step+10_r0 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.1 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.3 |
| L2_hip_step+20_r0 | latency=80.1 t90=640.0 overshoot=0.127 tracking=100.6 | latency=80.1 t90=680.0 overshoot=0.013 tracking=100.0 | latency=120.0 t90=680.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r0 | latency=79.9 t90=639.8 overshoot=0.0 tracking=99.8 | latency=79.9 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.7 |
| L2_hip_step-40_r0 | latency=79.9 t90=1119.9 overshoot=0.0 tracking=99.5 | latency=79.9 t90=1239.9 overshoot=0.0 tracking=98.2 | latency=119.9 t90=1199.9 overshoot=0.0 tracking=98.3 |
| L2_hip_step+5_r1 | latency=80.1 t90=280.0 overshoot=0.01 tracking=100.2 | latency=80.1 t90=240.1 overshoot=0.001 tracking=99.8 | latency=119.9 t90=320.0 overshoot=0.014 tracking=100.0 |
| L2_hip_step-5_r1 | latency=80.2 t90=280.3 overshoot=0.0 tracking=98.4 | latency=80.2 t90=280.3 overshoot=0.0 tracking=94.6 | latency=120.2 t90=320.3 overshoot=0.0 tracking=94.6 |
| L2_hip_step+10_r1 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r1 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.5 |
| L2_hip_step+20_r1 | latency=80.1 t90=640.0 overshoot=0.127 tracking=100.6 | latency=80.1 t90=680.1 overshoot=0.013 tracking=100.0 | latency=120.0 t90=680.1 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r1 | latency=80.0 t90=640.0 overshoot=0.039 tracking=100.2 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.7 |
| L2_hip_step-40_r1 | latency=80.3 t90=1120.0 overshoot=0.0 tracking=99.6 | latency=80.3 t90=1240.2 overshoot=0.0 tracking=98.5 | latency=120.2 t90=1200.2 overshoot=0.0 tracking=98.5 |
| L2_hip_step+5_r2 | latency=79.8 t90=280.1 overshoot=0.009 tracking=100.2 | latency=79.8 t90=240.2 overshoot=0.015 tracking=100.1 | latency=120.0 t90=320.0 overshoot=0.077 tracking=98.3 |
| L2_hip_step-5_r2 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.0 overshoot=0.0 tracking=95.1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.6 |
| L2_hip_step+10_r2 | latency=80.1 t90=400.0 overshoot=0.108 tracking=101.1 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r2 | latency=80.1 t90=400.2 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.2 overshoot=0.0 tracking=97.5 | latency=119.9 t90=440.1 overshoot=0.0 tracking=97.4 |
| L2_hip_step+20_r2 | latency=80.0 t90=640.0 overshoot=0.039 tracking=100.2 | latency=80.0 t90=680.0 overshoot=0.012 tracking=100.0 | latency=120.0 t90=680.0 overshoot=0.001 tracking=100.0 |
| L2_hip_step-20_r2 | latency=79.8 t90=640.0 overshoot=0.0 tracking=99.8 | latency=79.8 t90=680.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-40_r2 | latency=80.0 t90=1120.2 overshoot=0.0 tracking=99.5 | latency=80.0 t90=1240.1 overshoot=0.0 tracking=98.3 | latency=120.2 t90=1200.0 overshoot=0.0 tracking=98.3 |
| L2_hip_step+5_r3 | latency=79.8 t90=240.0 overshoot=0.01 tracking=100.2 | latency=79.8 t90=240.0 overshoot=0.004 tracking=100.1 | latency=120.0 t90=319.8 overshoot=0.0 tracking=99.8 |
| L2_hip_step-5_r3 | latency=80.1 t90=279.9 overshoot=0.0 tracking=98.4 | latency=80.1 t90=279.9 overshoot=0.0 tracking=95.1 | latency=119.9 t90=319.9 overshoot=0.0 tracking=94.6 |
| L2_hip_step+10_r3 | latency=79.9 t90=400.8 overshoot=0.019 tracking=100.2 | latency=79.9 t90=400.8 overshoot=0.0 tracking=99.0 | latency=119.9 t90=439.7 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r3 | latency=79.7 t90=399.9 overshoot=0.0 tracking=99.3 | latency=79.7 t90=399.9 overshoot=0.0 tracking=98.2 | latency=119.9 t90=439.9 overshoot=0.0 tracking=98.5 |
| L2_hip_step+20_r3 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=679.8 overshoot=0.012 tracking=100.0 | latency=120.1 t90=679.8 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r3 | latency=80.0 t90=640.1 overshoot=0.039 tracking=100.2 | latency=80.0 t90=679.8 overshoot=0.0 tracking=98.6 | latency=120.0 t90=679.8 overshoot=0.0 tracking=98.7 |
| L2_hip_step-40_r3 | latency=80.2 t90=1120.2 overshoot=0.0 tracking=99.5 | latency=80.2 t90=1240.2 overshoot=0.0 tracking=98.3 | latency=120.2 t90=1200.2 overshoot=0.0 tracking=98.3 |
| L2_hip_step+5_r4 | latency=80.1 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=240.0 overshoot=0.012 tracking=100.1 | latency=120.7 t90=320.0 overshoot=0.033 tracking=100.4 |
| L2_hip_step-5_r4 | latency=79.7 t90=279.9 overshoot=0.01 tracking=100.2 | latency=79.7 t90=279.9 overshoot=0.0 tracking=94.6 | latency=119.9 t90=319.9 overshoot=0.0 tracking=94.6 |
| L2_hip_step+10_r4 | latency=80.0 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L2_hip_step-10_r4 | latency=80.0 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.1 overshoot=0.0 tracking=97.4 |
| L2_hip_step+20_r4 | latency=79.8 t90=640.0 overshoot=0.127 tracking=100.6 | latency=79.8 t90=680.0 overshoot=0.012 tracking=100.0 | latency=119.8 t90=680.0 overshoot=0.0 tracking=100.0 |
| L2_hip_step-20_r4 | latency=80.0 t90=640.1 overshoot=0.127 tracking=100.6 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=119.8 t90=680.0 overshoot=0.0 tracking=98.7 |
| L2_hip_step-40_r4 | latency=80.1 t90=1120.0 overshoot=0.0 tracking=99.5 | latency=80.1 t90=1240.0 overshoot=0.0 tracking=98.3 | latency=120.1 t90=1200.0 overshoot=0.0 tracking=98.3 |
| L2_knee_step+5_r0 | latency=120.1 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.2 t90=280.3 overshoot=0.0 tracking=89.9 | latency=120.1 t90=320.0 overshoot=0.161 tracking=98.9 |
| L2_knee_step-5_r0 | latency=120.2 t90=360.2 overshoot=0.0 tracking=93.2 | latency=80.2 t90=280.2 overshoot=0.0 tracking=92.5 | latency=160.2 t90=360.2 overshoot=0.0 tracking=91.6 |
| L2_knee_step+10_r0 | latency=80.0 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=160.0 t90=440.1 overshoot=0.0 tracking=96.6 |
| L2_knee_step-10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.8 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.0 |
| L2_knee_step+20_r0 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=79.8 t90=680.0 overshoot=0.0 tracking=98.6 | latency=40.1 t90=680.0 overshoot=0.261 tracking=100.6 |
| L2_knee_step+40_r0 | latency=120.0 t90=1160.1 overshoot=0.0 tracking=98.9 | latency=80.0 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=120.0 t90=1200.0 overshoot=0.0 tracking=99.2 |
| L2_knee_step+5_r1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.0 overshoot=0.0 tracking=87.7 | latency=120.0 t90=320.0 overshoot=0.078 tracking=98.8 |
| L2_knee_step-5_r1 | latency=120.0 overshoot=0.0 tracking=89.6 | latency=80.0 overshoot=0.0 tracking=89.3 | latency=120.0 t90=320.0 overshoot=0.299 tracking=101.3 |
| L2_knee_step+10_r1 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.1 t90=440.1 overshoot=0.03 tracking=98.6 |
| L2_knee_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=94.9 | latency=79.8 t90=400.0 overshoot=0.0 tracking=95.6 | latency=160.0 t90=440.0 overshoot=0.0 tracking=98.6 |
| L2_knee_step+20_r1 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.1 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.1 overshoot=0.121 tracking=98.7 |
| L2_knee_step+40_r1 | latency=119.9 t90=1159.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=119.9 t90=1200.0 overshoot=0.007 tracking=100.0 |
| L2_knee_step+5_r2 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.1 overshoot=0.0 tracking=87.7 | latency=120.0 t90=320.0 overshoot=0.157 tracking=103.1 |
| L2_knee_step-5_r2 | latency=120.2 t90=320.2 overshoot=0.0 tracking=93.2 | latency=80.2 t90=280.2 overshoot=0.0 tracking=91.2 | latency=160.2 t90=360.2 overshoot=0.0 tracking=92.2 |
| L2_knee_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=79.8 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.237 tracking=100.7 |
| L2_knee_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.5 | latency=120.0 t90=440.0 overshoot=0.225 tracking=101.7 |
| L2_knee_step+20_r2 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=98.6 |
| L2_knee_step+40_r2 | latency=120.0 t90=1160.0 overshoot=0.0 tracking=98.7 | latency=80.0 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=120.0 t90=1200.1 overshoot=0.126 tracking=100.3 |
| L2_knee_step+5_r3 | latency=119.9 t90=319.7 overshoot=0.0 tracking=94.9 | latency=80.0 overshoot=0.0 tracking=89.3 | latency=80.0 t90=319.7 overshoot=0.318 tracking=100.0 |
| L2_knee_step-5_r3 | latency=120.0 t90=360.0 overshoot=0.0 tracking=93.2 | latency=80.1 t90=280.0 overshoot=0.0 tracking=91.2 | latency=160.0 t90=360.0 overshoot=0.0 tracking=97.3 |
| L2_knee_step+10_r3 | latency=120.0 t90=439.8 overshoot=0.0 tracking=97.6 | latency=80.0 t90=399.8 overshoot=0.0 tracking=96.7 | latency=120.0 t90=439.8 overshoot=0.227 tracking=98.5 |
| L2_knee_step-10_r3 | latency=119.9 t90=439.9 overshoot=0.0 tracking=94.9 | latency=79.9 t90=400.0 overshoot=0.0 tracking=95.5 | latency=159.9 t90=439.9 overshoot=0.0 tracking=95.8 |
| L2_knee_step+20_r3 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.6 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.5 |
| L2_knee_step+40_r3 | latency=119.9 t90=1159.9 overshoot=0.0 tracking=98.7 | latency=79.9 t90=1279.8 overshoot=0.0 tracking=98.5 | latency=119.9 t90=1199.9 overshoot=0.069 tracking=100.0 |
| L2_knee_step+5_r4 | latency=120.3 t90=320.3 overshoot=0.0 tracking=94.9 | latency=80.0 t90=280.2 overshoot=0.0 tracking=91.2 | latency=120.3 t90=320.3 overshoot=0.212 tracking=97.0 |
| L2_knee_step-5_r4 | latency=119.6 overshoot=0.0 tracking=89.6 | latency=79.6 overshoot=0.0 tracking=89.3 | latency=119.6 t90=319.6 overshoot=0.093 tracking=97.7 |
| L2_knee_step+10_r4 | latency=120.0 t90=440.1 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.1 overshoot=0.312 tracking=100.1 |
| L2_knee_step-10_r4 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.8 | latency=120.0 t90=440.0 overshoot=0.289 tracking=102.3 |
| L2_knee_step+20_r4 | latency=120.0 t90=640.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=680.0 overshoot=0.0 tracking=98.2 | latency=80.0 t90=680.0 overshoot=0.234 tracking=100.9 |
| L2_knee_step+40_r4 | latency=120.1 t90=1160.0 overshoot=0.0 tracking=98.4 | latency=79.8 t90=1280.0 overshoot=0.0 tracking=98.7 | latency=120.1 t90=1200.0 overshoot=0.0 tracking=99.6 |

### sysid_sines_air_v1_20260812_181941.csv

| segment | hardware | sim air | sim loaded |
|---|---|---|---|
| L2_yaw_sine4@0.25Hz | gain=0.9915 phase=116.2 rmse=0.528 | gain=0.9652 phase=126.5 rmse=0.559 | gain=0.9775 phase=200.1 rmse=0.868 |
| L2_yaw_sine4@0.5Hz | gain=0.9912 phase=121.0 rmse=0.912 | gain=0.9471 phase=99.6 rmse=0.755 | gain=0.9722 phase=181.9 rmse=1.339 |
| L2_yaw_sine4@1Hz | gain=1.0158 phase=128.1 rmse=1.923 | gain=1.0113 phase=95.7 rmse=1.5 | gain=1.0515 phase=171.0 rmse=2.588 |
| L2_yaw_sine4@2Hz | gain=0.4471 phase=-230.5 rmse=3.631 | gain=0.3842 phase=179.1 rmse=3.516 | gain=0.3688 phase=-246.0 rmse=3.816 |
| L2_yaw_sine8@0.25Hz | gain=1.0082 phase=110.8 rmse=0.987 | gain=0.9941 phase=90.1 rmse=0.799 | gain=1.0073 phase=164.3 rmse=1.45 |
| L2_yaw_sine8@0.5Hz | gain=0.9995 phase=139.3 rmse=2.089 | gain=0.9838 phase=91.7 rmse=1.378 | gain=1.0071 phase=173.9 rmse=2.596 |
| L2_yaw_sine8@1Hz | gain=0.9053 phase=315.1 rmse=7.658 | gain=0.7516 phase=242.6 rmse=6.353 | gain=0.7304 phase=303.9 rmse=7.268 |
| L2_yaw_sine8@2Hz | gain=0.2093 phase=-210.4 rmse=6.166 | gain=0.1867 phase=220.1 rmse=6.419 | gain=0.1784 phase=-213.5 rmse=6.463 |
| L2_hip_sine4@0.25Hz | gain=0.9986 phase=92.0 rmse=0.421 | gain=0.9748 phase=99.7 rmse=0.44 | gain=0.992 phase=171.7 rmse=0.751 |
| L2_hip_sine4@0.5Hz | gain=0.9849 phase=95.3 rmse=0.716 | gain=0.9552 phase=74.9 rmse=0.575 | gain=0.9721 phase=156.6 rmse=1.164 |
| L2_hip_sine4@1Hz | gain=1.0045 phase=106.5 rmse=1.61 | gain=0.9977 phase=76.8 rmse=1.183 | gain=1.048 phase=152.5 rmse=2.3 |
| L2_hip_sine4@2Hz | gain=0.4714 phase=239.9 rmse=3.669 | gain=0.3847 phase=168.9 rmse=3.425 | gain=0.3704 phase=242.5 rmse=3.808 |
| L2_hip_sine8@0.25Hz | gain=1.0093 phase=87.8 rmse=0.786 | gain=0.9953 phase=70.3 rmse=0.624 | gain=1.0078 phase=144.3 rmse=1.277 |
| L2_hip_sine8@0.5Hz | gain=0.9963 phase=115.5 rmse=1.731 | gain=0.99 phase=79.3 rmse=1.197 | gain=1.0121 phase=161.3 rmse=2.421 |
| L2_hip_sine8@1Hz | gain=0.9084 phase=299.2 rmse=7.381 | gain=0.7343 phase=207.8 rmse=5.677 | gain=0.8314 phase=325.4 rmse=7.738 |
| L2_hip_sine8@2Hz | gain=0.2446 phase=-230.0 rmse=6.379 | gain=0.176 phase=217.4 rmse=6.412 | gain=0.1969 phase=-234.3 rmse=6.435 |
| L2_knee_sine4@0.25Hz | gain=0.9622 phase=146.8 rmse=0.64 | gain=0.9547 phase=130.6 rmse=0.575 | gain=1.0012 phase=97.0 rmse=0.467 |
| L2_knee_sine4@0.5Hz | gain=0.9461 phase=138.4 rmse=1.016 | gain=0.9276 phase=97.1 rmse=0.731 | gain=1.0176 phase=117.9 rmse=0.907 |
| L2_knee_sine4@1Hz | gain=0.9694 phase=136.3 rmse=2.012 | gain=0.9882 phase=86.5 rmse=1.363 | gain=1.0345 phase=127.6 rmse=1.942 |
| L2_knee_sine4@2Hz | gain=0.3827 phase=-227.3 rmse=3.473 | gain=0.3704 phase=171.0 rmse=3.46 | gain=0.3694 phase=239.2 rmse=3.817 |
| L2_knee_sine8@0.25Hz | gain=0.9987 phase=125.3 rmse=1.103 | gain=0.9896 phase=88.0 rmse=0.775 | gain=1.0155 phase=104.2 rmse=0.957 |
| L2_knee_sine8@0.5Hz | gain=0.9883 phase=148.4 rmse=2.202 | gain=0.9736 phase=82.5 rmse=1.232 | gain=1.0144 phase=138.0 rmse=2.107 |
| L2_knee_sine8@1Hz | gain=0.8841 phase=324.4 rmse=7.672 | gain=0.8016 phase=199.0 rmse=5.315 | gain=0.8089 phase=286.5 rmse=7.308 |
| L2_knee_sine8@2Hz | gain=0.1875 phase=-209.0 rmse=6.019 | gain=0.1815 phase=211.6 rmse=6.376 | gain=0.1726 phase=-213.2 rmse=6.374 |

### sysid_servo_spread_v1_20260812_182251.csv

| segment | hardware | sim air | sim loaded |
|---|---|---|---|
| L0_yaw_step+10_r0 | latency=79.9 t90=399.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=95.7 | latency=160.4 t90=480.0 overshoot=0.0 tracking=95.8 |
| L0_yaw_step-10_r0 | latency=52.4 overshoot=0.0 tracking=75.6 | latency=129.6 overshoot=0.0 tracking=19.0 | latency=0.0 overshoot=0.0 tracking=21.1 |
| L0_yaw_step+10_r1 | latency=40.0 t90=440.0 overshoot=0.371 tracking=100.2 | latency=79.9 t90=399.8 overshoot=0.0 tracking=99.1 | latency=160.0 t90=480.0 overshoot=0.0 tracking=99.2 |
| L0_yaw_step-10_r1 | latency=40.0 t90=400.0 overshoot=0.459 tracking=102.4 | latency=80.0 t90=400.0 overshoot=0.028 tracking=100.1 | latency=159.9 t90=480.0 overshoot=0.038 tracking=100.0 |
| L0_yaw_step+10_r2 | latency=40.0 t90=400.0 overshoot=0.459 tracking=101.7 | latency=79.9 t90=400.0 overshoot=0.017 tracking=99.9 | latency=160.0 t90=480.0 overshoot=0.037 tracking=100.0 |
| L0_yaw_step-10_r2 | latency=40.0 t90=359.9 overshoot=0.284 tracking=99.1 | latency=80.0 t90=400.0 overshoot=0.02 tracking=99.9 | latency=159.9 t90=480.0 overshoot=0.056 tracking=100.2 |
| L0_hip_step+10_r0 | latency=119.9 t90=400.0 overshoot=0.0 tracking=99.3 | latency=79.9 t90=400.0 overshoot=0.0 tracking=96.5 | latency=119.9 t90=440.0 overshoot=0.0 tracking=96.4 |
| L0_hip_step-10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=96.4 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.5 |
| L0_hip_step+10_r1 | latency=80.0 t90=400.1 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.1 overshoot=0.015 tracking=100.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.9 |
| L0_hip_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=399.7 overshoot=0.0 tracking=96.5 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.6 |
| L0_hip_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.2 |
| L0_hip_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=79.9 t90=399.9 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.7 |
| L0_knee_step+10_r0 | latency=119.9 t90=400.0 overshoot=0.371 tracking=102.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.8 | latency=119.9 t90=440.0 overshoot=0.153 tracking=99.4 |
| L0_knee_step-10_r0 | latency=120.0 t90=399.8 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.8 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L0_knee_step+10_r1 | latency=119.9 t90=440.0 overshoot=0.195 tracking=102.0 | latency=80.0 t90=399.9 overshoot=0.013 tracking=100.0 | latency=119.9 t90=440.0 overshoot=0.368 tracking=103.0 |
| L0_knee_step-10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.219 tracking=98.8 |
| L0_knee_step+10_r2 | latency=120.0 t90=400.0 overshoot=0.459 tracking=103.7 | latency=80.0 t90=400.0 overshoot=0.009 tracking=99.9 | latency=120.0 t90=439.9 overshoot=0.0 tracking=100.0 |
| L0_knee_step-10_r2 | latency=120.1 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.1 overshoot=0.0 tracking=98.3 | latency=120.1 t90=440.1 overshoot=0.0 tracking=99.2 |
| L1_yaw_step+10_r0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.1 overshoot=0.0 tracking=97.7 | latency=160.2 t90=480.1 overshoot=0.0 tracking=97.5 |
| L1_yaw_step-10_r0 | latency=80.5 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.5 t90=400.0 overshoot=0.0 tracking=94.7 | latency=160.0 t90=480.0 overshoot=0.0 tracking=94.7 |
| L1_yaw_step+10_r1 | latency=120.1 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.1 | latency=160.1 t90=480.1 overshoot=0.0 tracking=98.2 |
| L1_yaw_step-10_r1 | latency=119.9 t90=400.0 overshoot=0.0 tracking=98.4 | latency=79.9 t90=400.0 overshoot=0.0 tracking=97.2 | latency=160.0 t90=480.0 overshoot=0.0 tracking=97.5 |
| L1_yaw_step+10_r2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.5 | latency=80.1 t90=400.0 overshoot=0.0 tracking=97.5 | latency=160.1 t90=480.1 overshoot=0.0 tracking=97.3 |
| L1_yaw_step-10_r2 | latency=120.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=160.0 t90=480.0 overshoot=0.0 tracking=95.4 |
| L1_hip_step+10_r0 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=96.5 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.4 |
| L1_hip_step-10_r0 | latency=120.0 t90=440.1 overshoot=0.0 tracking=94.9 | latency=80.1 t90=400.0 overshoot=0.0 tracking=94.6 | latency=120.0 t90=480.1 overshoot=0.0 tracking=94.7 |
| L1_hip_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L1_hip_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.7 |
| L1_hip_step+10_r2 | latency=120.0 t90=400.1 overshoot=0.0 tracking=99.3 | latency=79.9 t90=400.1 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.1 overshoot=0.0 tracking=99.2 |
| L1_hip_step-10_r2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=119.9 t90=440.0 overshoot=0.0 tracking=95.4 |
| L1_knee_step+10_r0 | latency=79.9 t90=400.0 overshoot=0.107 tracking=101.1 | latency=79.9 t90=400.0 overshoot=0.0 tracking=94.8 | latency=120.0 t90=440.0 overshoot=0.165 tracking=98.1 |
| L1_knee_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.9 | latency=120.0 t90=439.9 overshoot=0.021 tracking=96.0 |
| L1_knee_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.0 overshoot=0.311 tracking=101.9 |
| L1_knee_step-10_r1 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.2 | latency=119.9 t90=440.1 overshoot=0.023 tracking=98.8 |
| L1_knee_step+10_r2 | latency=79.7 t90=400.0 overshoot=0.0 tracking=99.3 | latency=79.7 t90=400.0 overshoot=0.0 tracking=99.8 | latency=120.0 t90=440.0 overshoot=0.072 tracking=100.2 |
| L1_knee_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.019 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.3 | latency=120.0 t90=439.9 overshoot=0.078 tracking=100.3 |
| L2_yaw_step+10_r0 | latency=119.9 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=159.9 t90=480.0 overshoot=0.0 tracking=95.8 |
| L2_yaw_step-10_r0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.4 | latency=160.1 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=160.0 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=160.0 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_yaw_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=160.0 t90=479.9 overshoot=0.0 tracking=98.2 |
| L2_yaw_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=79.9 t90=400.0 overshoot=0.0 tracking=98.4 | latency=160.0 t90=480.0 overshoot=0.0 tracking=98.2 |
| L2_hip_step+10_r0 | latency=79.8 t90=400.0 overshoot=0.108 tracking=101.1 | latency=79.8 t90=400.0 overshoot=0.0 tracking=96.5 | latency=120.0 t90=439.9 overshoot=0.0 tracking=96.4 |
| L2_hip_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=439.9 overshoot=0.0 tracking=97.4 |
| L2_hip_step+10_r1 | latency=80.1 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.1 t90=400.0 overshoot=0.005 tracking=100.0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=100.0 |
| L2_hip_step-10_r1 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.4 |
| L2_hip_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.013 tracking=100.1 | latency=120.0 t90=440.1 overshoot=0.017 tracking=100.2 |
| L2_hip_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.3 |
| L2_knee_step+10_r0 | latency=120.0 t90=439.8 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.9 | latency=120.0 t90=439.8 overshoot=0.142 tracking=98.8 |
| L2_knee_step-10_r0 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.6 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.5 |
| L2_knee_step+10_r1 | latency=120.1 t90=440.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=96.4 | latency=120.1 t90=440.0 overshoot=0.076 tracking=99.9 |
| L2_knee_step-10_r1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=399.8 overshoot=0.0 tracking=95.5 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.4 |
| L2_knee_step+10_r2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=97.6 | latency=80.1 t90=400.1 overshoot=0.0 tracking=97.3 | latency=80.1 t90=440.1 overshoot=0.289 tracking=102.2 |
| L2_knee_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=79.7 t90=400.0 overshoot=0.0 tracking=95.7 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.9 |
| L3_yaw_step+10_r0 | latency=101.3 t90=381.3 overshoot=0.0 tracking=99.3 | latency=61.3 t90=421.2 overshoot=0.0 tracking=95.7 | latency=141.3 t90=461.3 overshoot=0.0 tracking=95.8 |
| L3_yaw_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=160.0 t90=480.0 overshoot=0.0 tracking=99.2 |
| L3_yaw_step+10_r1 | latency=80.0 t90=399.8 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.8 overshoot=0.0 tracking=99.1 | latency=160.0 t90=479.9 overshoot=0.0 tracking=99.2 |
| L3_yaw_step-10_r1 | latency=80.0 t90=404.2 overshoot=0.0 tracking=98.4 | latency=80.0 t90=404.2 overshoot=0.017 tracking=100.0 | latency=160.0 t90=440.0 overshoot=0.043 tracking=100.1 |
| L3_yaw_step+10_r2 | latency=80.1 t90=400.1 overshoot=0.02 tracking=100.2 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.3 | latency=160.1 t90=480.1 overshoot=0.0 tracking=99.2 |
| L3_yaw_step-10_r2 | latency=80.0 t90=399.9 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.1 | latency=160.0 t90=479.9 overshoot=0.0 tracking=98.9 |
| L3_hip_step+10_r0 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=96.5 | latency=120.0 t90=439.7 overshoot=0.0 tracking=96.4 |
| L3_hip_step-10_r0 | latency=119.9 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.5 | latency=119.9 t90=440.0 overshoot=0.0 tracking=95.7 |
| L3_hip_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=439.9 overshoot=0.0 tracking=98.9 |
| L3_hip_step-10_r1 | latency=120.1 t90=440.1 overshoot=0.0 tracking=96.7 | latency=80.1 t90=400.0 overshoot=0.0 tracking=95.5 | latency=120.1 t90=440.1 overshoot=0.0 tracking=95.7 |
| L3_hip_step+10_r2 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.3 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.1 | latency=120.0 t90=439.9 overshoot=0.0 tracking=99.2 |
| L3_hip_step-10_r2 | latency=120.0 t90=439.9 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=95.7 | latency=120.0 t90=439.9 overshoot=0.0 tracking=95.7 |
| L3_knee_step+10_r0 | latency=80.0 t90=400.1 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.1 overshoot=0.0 tracking=94.8 | latency=120.1 t90=440.1 overshoot=0.142 tracking=99.8 |
| L3_knee_step-10_r0 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.1 t90=440.1 overshoot=0.0 tracking=98.0 |
| L3_knee_step+10_r1 | latency=79.9 t90=399.8 overshoot=0.0 tracking=99.3 | latency=79.9 t90=399.8 overshoot=0.0 tracking=98.9 | latency=120.0 t90=440.1 overshoot=0.0 tracking=98.9 |
| L3_knee_step-10_r1 | latency=80.3 t90=400.3 overshoot=0.02 tracking=100.2 | latency=80.3 t90=400.3 overshoot=0.0 tracking=99.2 | latency=80.3 t90=440.2 overshoot=0.279 tracking=102.0 |
| L3_knee_step+10_r2 | latency=80.2 t90=400.0 overshoot=0.019 tracking=100.2 | latency=80.2 t90=400.0 overshoot=0.0 tracking=99.8 | latency=120.3 t90=440.1 overshoot=0.0 tracking=96.8 |
| L3_knee_step-10_r2 | latency=80.1 t90=400.1 overshoot=0.019 tracking=99.5 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.2 | latency=120.1 t90=439.8 overshoot=0.301 tracking=99.3 |
| L4_yaw_step+10_r0 | latency=120.0 t90=391.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.7 overshoot=0.0 tracking=95.7 | latency=160.0 t90=440.0 overshoot=0.0 tracking=95.8 |
| L4_yaw_step-10_r0 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.3 | latency=160.1 t90=480.1 overshoot=0.0 tracking=99.2 |
| L4_yaw_step+10_r1 | latency=80.1 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.1 overshoot=0.0 tracking=99.1 | latency=159.8 t90=480.0 overshoot=0.0 tracking=99.2 |
| L4_yaw_step-10_r1 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=160.0 t90=479.9 overshoot=0.0 tracking=99.2 |
| L4_yaw_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=159.9 t90=480.0 overshoot=0.0 tracking=99.2 |
| L4_yaw_step-10_r2 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.1 | latency=159.8 t90=480.0 overshoot=0.0 tracking=99.2 |
| L4_hip_step+10_r0 | latency=79.9 t90=399.9 overshoot=0.107 tracking=101.1 | latency=79.9 t90=399.9 overshoot=0.0 tracking=96.5 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.4 |
| L4_hip_step-10_r0 | latency=79.9 t90=400.0 overshoot=0.019 tracking=100.2 | latency=79.9 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=439.9 overshoot=0.0 tracking=98.2 |
| L4_hip_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=100.0 | latency=119.9 t90=440.0 overshoot=0.0 tracking=100.0 |
| L4_hip_step-10_r1 | latency=79.9 t90=400.0 overshoot=0.0 tracking=99.3 | latency=79.9 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=439.9 overshoot=0.0 tracking=98.2 |
| L4_hip_step+10_r2 | latency=79.9 t90=399.9 overshoot=0.02 tracking=100.2 | latency=79.9 t90=399.9 overshoot=0.011 tracking=100.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.9 |
| L4_hip_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.4 |
| L4_knee_step+10_r0 | latency=120.0 t90=400.0 overshoot=0.107 tracking=101.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.8 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.9 |
| L4_knee_step-10_r0 | latency=80.1 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.8 | latency=120.1 t90=440.1 overshoot=0.413 tracking=100.0 |
| L4_knee_step+10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.022 tracking=100.1 | latency=80.0 t90=440.1 overshoot=0.32 tracking=102.1 |
| L4_knee_step-10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=119.9 t90=440.0 overshoot=0.284 tracking=102.1 |
| L4_knee_step+10_r2 | latency=80.1 t90=399.9 overshoot=0.019 tracking=100.2 | latency=80.1 t90=399.9 overshoot=0.009 tracking=99.9 | latency=120.0 t90=440.1 overshoot=0.351 tracking=101.5 |
| L4_knee_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=119.9 t90=439.8 overshoot=0.325 tracking=102.7 |
| L5_yaw_step+10_r0 | latency=80.0 t90=399.9 overshoot=0.0 tracking=99.3 | latency=80.0 t90=399.9 overshoot=0.0 tracking=95.7 | latency=159.9 t90=480.0 overshoot=0.0 tracking=95.8 |
| L5_yaw_step-10_r0 | latency=80.0 t90=400.0 overshoot=0.019 tracking=100.2 | latency=80.0 t90=400.0 overshoot=0.017 tracking=100.0 | latency=160.0 t90=480.0 overshoot=0.024 tracking=99.9 |
| L5_yaw_step+10_r1 | latency=80.1 t90=400.1 overshoot=0.283 tracking=99.3 | latency=80.1 t90=400.1 overshoot=0.017 tracking=100.0 | latency=160.1 t90=480.1 overshoot=0.045 tracking=100.1 |
| L5_yaw_step-10_r1 | latency=80.1 t90=400.0 overshoot=0.02 tracking=100.2 | latency=80.1 t90=400.0 overshoot=0.017 tracking=100.0 | latency=160.0 t90=480.1 overshoot=0.038 tracking=100.0 |
| L5_yaw_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.371 tracking=99.8 | latency=80.0 t90=400.0 overshoot=0.0 tracking=97.5 | latency=160.0 t90=480.0 overshoot=0.0 tracking=97.2 |
| L5_yaw_step-10_r2 | latency=0.0 t90=400.0 overshoot=0.0 tracking=98.7 | latency=80.0 t90=400.0 overshoot=0.028 tracking=100.1 | latency=160.0 t90=480.0 overshoot=0.051 tracking=100.1 |
| L5_hip_step+10_r0 | latency=120.2 t90=440.3 overshoot=0.0 tracking=98.4 | latency=80.2 t90=400.3 overshoot=0.0 tracking=96.5 | latency=120.2 t90=440.3 overshoot=0.0 tracking=96.4 |
| L5_hip_step-10_r0 | latency=120.1 t90=440.1 overshoot=0.0 tracking=95.8 | latency=80.1 t90=400.0 overshoot=0.0 tracking=95.7 | latency=120.1 t90=440.1 overshoot=0.0 tracking=95.7 |
| L5_hip_step+10_r1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.9 |
| L5_hip_step-10_r1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=94.9 | latency=79.8 t90=400.0 overshoot=0.0 tracking=94.7 | latency=119.9 t90=440.0 overshoot=0.0 tracking=95.4 |
| L5_hip_step+10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=97.6 | latency=79.9 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=399.9 overshoot=0.0 tracking=93.8 | latency=120.0 t90=479.9 overshoot=0.0 tracking=94.0 |
| L5_knee_step+10_r0 | latency=120.0 t90=414.1 overshoot=0.0 tracking=98.4 | latency=79.9 t90=414.1 overshoot=0.0 tracking=95.8 | latency=120.0 t90=439.9 overshoot=0.117 tracking=97.6 |
| L5_knee_step-10_r0 | latency=120.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=95.5 |
| L5_knee_step+10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.0 t90=440.1 overshoot=0.0 tracking=98.9 |
| L5_knee_step-10_r1 | latency=120.0 t90=399.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.2 | latency=160.0 t90=440.0 overshoot=0.0 tracking=97.3 |
| L5_knee_step+10_r2 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=39.9 t90=439.9 overshoot=0.16 tracking=98.7 |
| L5_knee_step-10_r2 | latency=119.9 t90=400.0 overshoot=0.0 tracking=98.4 | latency=79.9 t90=400.0 overshoot=0.0 tracking=99.2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.4 |
