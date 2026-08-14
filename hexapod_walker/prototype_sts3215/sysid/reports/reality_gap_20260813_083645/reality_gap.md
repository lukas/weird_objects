# Hexapod Sim Reality Report

- generated: 2026-08-13T08:36:46
- git: `02e1cbb`
- servo params `air`: sim_model.json (6968268e879e)
- servo params `rl_move/sim/sim_model_sysid.json`: sim_model_sysid.json (8b8d2f260256)

## Headline gap metrics

| trace | metric | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|---|
| sysid_sines_air_v2_20260813_122406.csv | unloaded q_rmse_deg | — | 0.434 | 0.247 |
| sysid_sines_air_v2_20260813_122406.csv | unloaded qd_rmse_deg_s | — | 2.94 | 2.11 |
| sysid_steps_air_L5_v1_20260813_122627.csv | unloaded q_rmse_deg | — | 0.414 | 0.329 |
| sysid_steps_air_L5_v1_20260813_122627.csv | unloaded qd_rmse_deg_s | — | 2.42 | 2.37 |
| sysid_steps_air_L5_v1_20260813_122627.csv | cmd→motion latency (hw) | median 119.9 ms (p10 80.0 / p90 120.1 / max 121.9, n=95) | — | — |

## Timing (hardware)

- **sysid_sines_air_v2_20260813_122406.csv**: tick median 40.0 ms (nominal 40.0, p95 40.3, late 0.31%); send→recv RTT median 8.3 ms (p95 9.4)
- **sysid_steps_air_L5_v1_20260813_122627.csv**: tick median 40.0 ms (nominal 40.0, p95 40.1, late 0.04%); send→recv RTT median 8.2 ms (p95 8.6)

## Per-segment detail

### sysid_sines_air_v2_20260813_122406.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| L2_yaw_sine2@0.5Hz | gain=0.9641 phase=123.3 rmse=0.471 | gain=0.8461 phase=137.6 rmse=0.515 | gain=0.9778 phase=141.1 rmse=0.524 |
| L2_yaw_sine2@1Hz | gain=1.037 phase=112.4 rmse=0.881 | gain=0.8962 phase=85.5 rmse=0.7 | gain=1.0545 phase=119.1 rmse=0.936 |
| L2_yaw_sine2@1.5Hz | gain=0.9238 phase=115.3 rmse=1.274 | gain=0.8976 phase=88.5 rmse=1.005 | gain=0.9606 phase=130.3 rmse=1.438 |
| L2_yaw_sine2@2Hz | gain=0.8672 phase=154.9 rmse=1.882 | gain=0.7405 phase=102.5 rmse=1.323 | gain=0.8095 phase=151.5 rmse=1.818 |
| L2_yaw_sine3@0.5Hz | gain=0.9853 phase=122.2 rmse=0.69 | gain=0.9159 phase=110.7 rmse=0.626 | gain=0.9966 phase=133.8 rmse=0.753 |
| L2_yaw_sine3@1Hz | gain=1.0435 phase=120.9 rmse=1.396 | gain=0.9978 phase=87.0 rmse=1.032 | gain=1.0725 phase=131.0 rmse=1.524 |
| L2_yaw_sine3@1.5Hz | gain=0.9417 phase=136.7 rmse=2.19 | gain=0.8911 phase=101.3 rmse=1.668 | gain=0.897 phase=151.8 rmse=2.356 |
| L2_yaw_sine3@2Hz | gain=0.6217 phase=243.3 rmse=3.03 | gain=0.6245 phase=112.3 rmse=2.012 | gain=0.5903 phase=206.5 rmse=2.953 |
| L2_hip_sine2@0.5Hz | gain=0.9593 phase=101.5 rmse=0.385 | gain=0.8846 phase=108.8 rmse=0.415 | gain=1.0001 phase=86.3 rmse=0.328 |
| L2_hip_sine2@1Hz | gain=1.0168 phase=89.7 rmse=0.708 | gain=0.9321 phase=67.3 rmse=0.558 | gain=1.0305 phase=84.5 rmse=0.653 |
| L2_hip_sine2@1.5Hz | gain=0.9375 phase=92.5 rmse=1.042 | gain=0.9224 phase=74.5 rmse=0.855 | gain=0.928 phase=90.7 rmse=1.014 |
| L2_hip_sine2@2Hz | gain=0.88 phase=127.6 rmse=1.655 | gain=0.7361 phase=91.1 rmse=1.196 | gain=0.7311 phase=142.3 rmse=1.723 |
| L2_hip_sine3@0.5Hz | gain=0.979 phase=97.3 rmse=0.549 | gain=0.9355 phase=87.9 rmse=0.503 | gain=1.0029 phase=84.3 rmse=0.479 |
| L2_hip_sine3@1Hz | gain=1.0096 phase=95.9 rmse=1.111 | gain=0.9907 phase=67.5 rmse=0.817 | gain=1.0155 phase=94.7 rmse=1.078 |
| L2_hip_sine3@1.5Hz | gain=0.9494 phase=114.3 rmse=1.887 | gain=0.881 phase=91.0 rmse=1.509 | gain=0.8752 phase=147.4 rmse=2.34 |
| L2_hip_sine3@2Hz | gain=0.6312 phase=218.0 rmse=2.981 | gain=0.5877 phase=125.4 rmse=2.294 | gain=0.5596 phase=198.0 rmse=2.937 |
| L2_knee_sine2@0.5Hz | gain=0.8788 phase=148.6 rmse=0.538 | gain=0.8175 phase=144.4 rmse=0.538 | gain=0.9352 phase=167.9 rmse=0.61 |
| L2_knee_sine2@1Hz | gain=0.9242 phase=125.2 rmse=0.916 | gain=0.8616 phase=84.5 rmse=0.688 | gain=1.0093 phase=129.7 rmse=0.992 |
| L2_knee_sine2@1.5Hz | gain=0.8172 phase=123.3 rmse=1.276 | gain=0.8687 phase=85.1 rmse=0.958 | gain=0.9436 phase=137.3 rmse=1.483 |
| L2_knee_sine2@2Hz | gain=0.7358 phase=160.8 rmse=1.805 | gain=0.7163 phase=92.8 rmse=1.214 | gain=0.7611 phase=157.6 rmse=1.812 |
| L2_knee_sine3@0.5Hz | gain=0.9105 phase=143.3 rmse=0.783 | gain=0.8925 phase=110.0 rmse=0.621 | gain=0.971 phase=150.6 rmse=0.837 |
| L2_knee_sine3@1Hz | gain=0.9543 phase=128.2 rmse=1.431 | gain=0.9591 phase=78.3 rmse=0.931 | gain=1.0582 phase=137.7 rmse=1.598 |
| L2_knee_sine3@1.5Hz | gain=0.8675 phase=143.2 rmse=2.206 | gain=0.869 phase=91.2 rmse=1.51 | gain=0.8818 phase=159.0 rmse=2.427 |
| L2_knee_sine3@2Hz | gain=0.5104 phase=247.7 rmse=2.847 | gain=0.5846 phase=105.1 rmse=1.908 | gain=0.5997 phase=206.4 rmse=2.981 |

### sysid_steps_air_L5_v1_20260813_122627.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| L5_yaw_step+5_r0 | latency=80.2 t90=280.2 overshoot=0.098 tracking=102.0 | latency=80.2 t90=280.2 overshoot=0.0 tracking=91.7 | latency=120.2 t90=320.3 overshoot=0.0 tracking=96.7 |
| L5_yaw_step-5_r0 | latency=80.0 t90=280.0 overshoot=0.01 tracking=98.4 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.0 |
| L5_yaw_step+10_r0 | latency=79.9 t90=399.9 overshoot=0.019 tracking=100.2 | latency=79.9 t90=399.9 overshoot=0.017 tracking=100.0 | latency=119.9 t90=439.9 overshoot=0.009 tracking=100.1 |
| L5_yaw_step-10_r0 | latency=79.9 t90=399.9 overshoot=0.02 tracking=100.2 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.3 | latency=120.0 t90=439.9 overshoot=0.0 tracking=99.0 |
| L5_yaw_step+20_r0 | latency=79.9 t90=639.9 overshoot=0.0 tracking=99.8 | latency=79.9 t90=679.9 overshoot=0.012 tracking=100.0 | latency=119.8 t90=679.9 overshoot=0.005 tracking=100.0 |
| L5_yaw_step-20_r0 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.5 | latency=80.0 t90=680.0 overshoot=0.012 tracking=100.0 | latency=120.1 t90=680.0 overshoot=0.0 tracking=99.9 |
| L5_yaw_step+5_r1 | latency=80.0 t90=280.0 overshoot=0.361 tracking=103.7 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.0 |
| L5_yaw_step-5_r1 | latency=80.1 t90=280.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=280.1 overshoot=0.007 tracking=99.7 | latency=120.2 t90=320.2 overshoot=0.004 tracking=99.8 |
| L5_yaw_step+10_r1 | latency=17.4 t90=410.8 overshoot=0.195 tracking=99.3 | latency=50.7 t90=370.7 overshoot=0.006 tracking=99.9 | latency=90.7 t90=410.8 overshoot=0.0 tracking=99.9 |
| L5_yaw_step-10_r1 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=120.1 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_yaw_step+20_r1 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.0 overshoot=0.012 tracking=100.0 | latency=120.0 t90=680.0 overshoot=0.005 tracking=100.0 |
| L5_yaw_step-20_r1 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.5 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.5 |
| L5_yaw_step+5_r2 | latency=80.0 t90=280.0 overshoot=0.273 tracking=100.2 | latency=80.0 t90=280.0 overshoot=0.021 tracking=100.2 | latency=119.8 t90=320.0 overshoot=0.01 tracking=100.1 |
| L5_yaw_step-5_r2 | latency=80.0 t90=280.0 overshoot=0.098 tracking=100.2 | latency=80.0 t90=280.0 overshoot=0.0 tracking=94.6 | latency=120.0 t90=320.0 overshoot=0.0 tracking=94.7 |
| L5_yaw_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.019 tracking=99.9 | latency=120.0 t90=440.0 overshoot=0.001 tracking=100.0 |
| L5_yaw_step-10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L5_yaw_step+20_r2 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.0 overshoot=0.012 tracking=100.0 | latency=120.1 t90=680.0 overshoot=0.005 tracking=100.0 |
| L5_yaw_step-20_r2 | latency=80.1 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.1 t90=680.0 overshoot=0.0 tracking=99.5 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.6 |
| L5_yaw_step+5_r3 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.0 overshoot=0.029 tracking=100.1 | latency=120.0 t90=320.0 overshoot=0.004 tracking=99.8 |
| L5_yaw_step-5_r3 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.0 overshoot=0.0 tracking=98.4 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.0 |
| L5_yaw_step+10_r3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L5_yaw_step-10_r3 | latency=80.0 t90=360.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.017 tracking=100.0 | latency=120.0 t90=440.0 overshoot=0.001 tracking=100.0 |
| L5_yaw_step+20_r3 | latency=80.1 t90=640.1 overshoot=0.0 tracking=99.8 | latency=80.1 t90=680.1 overshoot=0.012 tracking=100.0 | latency=120.0 t90=680.1 overshoot=0.005 tracking=100.0 |
| L5_yaw_step-20_r3 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.1 overshoot=0.039 tracking=100.1 | latency=120.0 t90=680.1 overshoot=0.005 tracking=100.0 |
| L5_yaw_step+5_r4 | latency=120.2 t90=320.3 overshoot=0.274 tracking=98.4 | latency=80.2 t90=280.2 overshoot=0.0 tracking=96.6 | latency=120.2 t90=320.3 overshoot=0.0 tracking=96.5 |
| L5_yaw_step-5_r4 | latency=80.1 t90=280.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=280.0 overshoot=0.007 tracking=99.7 | latency=120.0 t90=320.0 overshoot=0.01 tracking=100.1 |
| L5_yaw_step+10_r4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.017 tracking=99.9 | latency=120.0 t90=440.0 overshoot=0.001 tracking=100.0 |
| L5_yaw_step-10_r4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.3 | latency=79.9 t90=399.9 overshoot=0.032 tracking=100.1 | latency=119.9 t90=439.8 overshoot=0.009 tracking=100.1 |
| L5_yaw_step+20_r4 | latency=80.0 t90=639.9 overshoot=0.0 tracking=99.8 | latency=80.0 t90=679.9 overshoot=0.012 tracking=100.0 | latency=119.9 t90=679.9 overshoot=0.005 tracking=100.0 |
| L5_yaw_step-20_r4 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.1 overshoot=0.012 tracking=100.0 | latency=120.0 t90=680.1 overshoot=0.005 tracking=100.0 |
| L5_hip_step+5_r0 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=280.0 overshoot=0.0 tracking=93.0 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L5_hip_step-5_r0 | latency=120.0 t90=320.0 overshoot=0.0 tracking=93.2 | latency=80.1 overshoot=0.0 tracking=87.6 | latency=80.1 t90=320.0 overshoot=0.0 tracking=98.5 |
| L5_hip_step+10_r0 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r0 | latency=119.9 t90=439.9 overshoot=0.0 tracking=96.7 | latency=79.9 t90=399.9 overshoot=0.0 tracking=95.7 | latency=79.9 t90=439.9 overshoot=0.0 tracking=99.1 |
| L5_hip_step+20_r0 | latency=80.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 |
| L5_hip_step-20_r0 | latency=120.1 t90=680.2 overshoot=0.0 tracking=97.6 | latency=80.2 t90=680.2 overshoot=0.0 tracking=97.8 | latency=80.2 t90=640.2 overshoot=0.0 tracking=99.6 |
| L5_hip_step-40_r0 | latency=120.3 t90=1160.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=1240.0 overshoot=0.0 tracking=98.3 | latency=80.0 t90=1160.0 overshoot=0.0 tracking=99.9 |
| L5_hip_step+5_r1 | latency=120.1 t90=320.1 overshoot=0.0 tracking=98.4 | latency=80.0 t90=240.0 overshoot=0.0 tracking=98.1 | latency=80.0 t90=320.1 overshoot=0.0 tracking=98.3 |
| L5_hip_step-5_r1 | latency=121.9 t90=359.9 overshoot=0.0 tracking=93.2 | latency=79.9 t90=279.9 overshoot=0.0 tracking=91.3 | latency=79.9 t90=320.0 overshoot=0.0 tracking=98.5 |
| L5_hip_step+10_r1 | latency=120.1 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.0 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r1 | latency=120.1 t90=440.1 overshoot=0.0 tracking=96.7 | latency=80.0 t90=400.0 overshoot=0.0 tracking=94.7 | latency=80.0 t90=440.1 overshoot=0.0 tracking=99.2 |
| L5_hip_step+20_r1 | latency=80.0 t90=640.1 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 |
| L5_hip_step-20_r1 | latency=120.0 t90=680.0 overshoot=0.0 tracking=97.6 | latency=80.0 t90=680.0 overshoot=0.0 tracking=97.9 | latency=80.0 t90=640.0 overshoot=0.0 tracking=99.6 |
| L5_hip_step-40_r1 | latency=120.0 t90=1160.1 overshoot=0.0 tracking=98.8 | latency=80.0 t90=1280.1 overshoot=0.0 tracking=98.5 | latency=80.0 t90=1160.1 overshoot=0.02 tracking=100.0 |
| L5_hip_step+5_r2 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=240.1 overshoot=0.0 tracking=98.1 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L5_hip_step-5_r2 | latency=120.1 t90=320.0 overshoot=0.0 tracking=91.4 | latency=40.0 t90=280.0 overshoot=0.0 tracking=91.3 | latency=81.4 t90=320.0 overshoot=0.0 tracking=98.4 |
| L5_hip_step+10_r2 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r2 | latency=119.9 t90=439.9 overshoot=0.0 tracking=95.8 | latency=79.9 t90=400.0 overshoot=0.0 tracking=94.7 | latency=79.9 t90=439.9 overshoot=0.0 tracking=99.2 |
| L5_hip_step+20_r2 | latency=120.0 t90=640.1 overshoot=0.0 tracking=99.3 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.6 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 |
| L5_hip_step-20_r2 | latency=119.9 t90=639.7 overshoot=0.0 tracking=98.4 | latency=79.9 t90=679.9 overshoot=0.0 tracking=97.8 | latency=79.9 t90=639.7 overshoot=0.0 tracking=99.6 |
| L5_hip_step-40_r2 | latency=120.0 t90=1160.0 overshoot=0.0 tracking=98.7 | latency=80.1 t90=1240.0 overshoot=0.0 tracking=98.5 | latency=80.1 t90=1160.0 overshoot=0.019 tracking=100.0 |
| L5_hip_step+5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=240.0 overshoot=0.0 tracking=98.1 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L5_hip_step-5_r3 | latency=120.0 t90=360.0 overshoot=0.0 tracking=91.4 | latency=80.0 t90=280.0 overshoot=0.0 tracking=91.3 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.4 |
| L5_hip_step+10_r3 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=440.0 overshoot=0.0 tracking=99.1 |
| L5_hip_step-10_r3 | latency=120.2 t90=440.0 overshoot=0.0 tracking=96.7 | latency=80.1 t90=400.0 overshoot=0.0 tracking=94.6 | latency=80.1 t90=440.0 overshoot=0.0 tracking=99.2 |
| L5_hip_step+20_r3 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.9 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 |
| L5_hip_step-20_r3 | latency=120.0 t90=640.1 overshoot=0.0 tracking=98.4 | latency=80.1 t90=680.0 overshoot=0.0 tracking=97.9 | latency=80.1 t90=640.1 overshoot=0.0 tracking=99.6 |
| L5_hip_step-40_r3 | latency=120.1 t90=1160.1 overshoot=0.0 tracking=98.7 | latency=80.0 t90=1240.0 overshoot=0.0 tracking=98.5 | latency=80.0 t90=1160.1 overshoot=0.019 tracking=100.0 |
| L5_hip_step+5_r4 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=240.0 overshoot=0.0 tracking=98.5 | latency=80.1 t90=320.0 overshoot=0.0 tracking=98.3 |
| L5_hip_step-5_r4 | latency=120.0 overshoot=0.0 tracking=86.1 | latency=80.0 overshoot=0.0 tracking=89.0 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.3 |
| L5_hip_step+10_r4 | latency=119.9 t90=399.9 overshoot=0.0 tracking=98.4 | latency=80.0 t90=399.9 overshoot=0.0 tracking=98.1 | latency=80.0 t90=439.9 overshoot=0.0 tracking=99.2 |
| L5_hip_step-10_r4 | latency=119.9 t90=439.9 overshoot=0.0 tracking=95.8 | latency=79.9 t90=400.0 overshoot=0.0 tracking=94.8 | latency=79.9 t90=439.9 overshoot=0.0 tracking=99.2 |
| L5_hip_step+20_r4 | latency=120.0 t90=640.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.1 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.6 |
| L5_hip_step-20_r4 | latency=120.0 t90=679.9 overshoot=0.0 tracking=98.0 | latency=79.9 t90=679.9 overshoot=0.0 tracking=97.3 | latency=79.9 t90=639.9 overshoot=0.0 tracking=99.6 |
| L5_hip_step-40_r4 | latency=119.9 t90=1159.9 overshoot=0.0 tracking=98.7 | latency=79.9 t90=1239.9 overshoot=0.0 tracking=98.5 | latency=79.9 t90=1159.9 overshoot=0.018 tracking=100.0 |
| L5_knee_step+5_r0 | latency=119.9 t90=319.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=279.9 overshoot=0.0 tracking=89.9 | latency=119.9 t90=319.9 overshoot=0.0 tracking=94.4 |
| L5_knee_step-5_r0 | latency=120.1 t90=320.1 overshoot=0.0 tracking=96.7 | latency=80.0 t90=240.0 overshoot=0.0 tracking=96.0 | latency=120.1 t90=320.1 overshoot=0.0 tracking=96.2 |
| L5_knee_step+10_r0 | latency=119.9 t90=399.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=399.9 overshoot=0.0 tracking=99.1 | latency=119.9 t90=440.0 overshoot=0.0 tracking=99.1 |
| L5_knee_step-10_r0 | latency=120.1 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=99.2 | latency=120.1 t90=440.0 overshoot=0.0 tracking=99.3 |
| L5_knee_step+20_r0 | latency=120.0 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.5 | latency=120.0 t90=680.0 overshoot=0.0 tracking=99.6 |
| L5_knee_step+40_r0 | latency=120.0 t90=1160.1 overshoot=0.0 tracking=99.8 | latency=80.0 t90=1280.0 overshoot=0.0 tracking=97.8 | latency=120.0 t90=1199.8 overshoot=0.0 tracking=98.9 |
| L5_knee_step+5_r1 | latency=120.1 t90=320.0 overshoot=0.0 tracking=96.7 | latency=80.0 overshoot=0.0 tracking=84.7 | latency=120.1 t90=320.0 overshoot=0.0 tracking=92.7 |
| L5_knee_step-5_r1 | latency=120.1 t90=320.0 overshoot=0.0 tracking=94.9 | latency=80.0 t90=240.0 overshoot=0.0 tracking=96.5 | latency=120.1 t90=320.0 overshoot=0.0 tracking=96.2 |
| L5_knee_step+10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.3 | latency=120.0 t90=440.1 overshoot=0.0 tracking=98.2 |
| L5_knee_step-10_r1 | latency=120.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L5_knee_step+20_r1 | latency=119.9 t90=640.0 overshoot=0.0 tracking=99.8 | latency=79.8 t90=679.9 overshoot=0.0 tracking=99.1 | latency=119.9 t90=679.9 overshoot=0.0 tracking=99.1 |
| L5_knee_step+40_r1 | latency=120.0 t90=1160.0 overshoot=0.0 tracking=99.5 | latency=80.0 t90=1280.0 overshoot=0.0 tracking=98.0 | latency=120.0 t90=1200.1 overshoot=0.0 tracking=99.1 |
| L5_knee_step+5_r2 | latency=120.0 t90=320.1 overshoot=0.0 tracking=98.4 | latency=80.0 overshoot=0.0 tracking=84.1 | latency=120.0 t90=320.1 overshoot=0.0 tracking=92.7 |
| L5_knee_step-5_r2 | latency=80.0 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.0 t90=240.1 overshoot=0.0 tracking=96.5 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.9 |
| L5_knee_step+10_r2 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.0 t90=400.0 overshoot=0.0 tracking=98.2 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L5_knee_step-10_r2 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L5_knee_step+20_r2 | latency=120.2 t90=640.2 overshoot=0.0 tracking=99.3 | latency=80.2 t90=680.2 overshoot=0.0 tracking=99.1 | latency=120.2 t90=680.2 overshoot=0.0 tracking=99.1 |
| L5_knee_step+40_r2 | latency=120.0 t90=1160.0 overshoot=0.0 tracking=99.7 | latency=80.0 t90=1280.1 overshoot=0.0 tracking=98.0 | latency=120.0 t90=1200.0 overshoot=0.0 tracking=99.0 |
| L5_knee_step+5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.0 overshoot=0.0 tracking=84.1 | latency=120.0 t90=320.0 overshoot=0.0 tracking=92.7 |
| L5_knee_step-5_r3 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.7 | latency=80.0 t90=240.0 overshoot=0.0 tracking=96.5 | latency=120.0 t90=320.0 overshoot=0.0 tracking=96.4 |
| L5_knee_step+10_r3 | latency=120.0 t90=400.0 overshoot=0.0 tracking=99.3 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.1 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.4 |
| L5_knee_step-10_r3 | latency=120.0 t90=400.1 overshoot=0.0 tracking=98.4 | latency=80.0 t90=400.1 overshoot=0.0 tracking=98.3 | latency=120.0 t90=440.0 overshoot=0.0 tracking=98.2 |
| L5_knee_step+20_r3 | latency=119.9 t90=640.0 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.0 overshoot=0.0 tracking=99.6 | latency=119.9 t90=680.0 overshoot=0.0 tracking=99.6 |
| L5_knee_step+40_r3 | latency=120.1 t90=1160.0 overshoot=0.0 tracking=99.5 | latency=80.1 t90=1280.0 overshoot=0.0 tracking=98.2 | latency=120.1 t90=1200.0 overshoot=0.0 tracking=99.2 |
| L5_knee_step+5_r4 | latency=80.1 t90=320.0 overshoot=0.0 tracking=98.4 | latency=80.1 overshoot=0.0 tracking=84.1 | latency=120.1 t90=320.0 overshoot=0.0 tracking=92.7 |
| L5_knee_step-5_r4 | latency=119.9 t90=319.9 overshoot=0.0 tracking=96.7 | latency=80.0 t90=239.9 overshoot=0.0 tracking=96.5 | latency=119.9 t90=319.9 overshoot=0.0 tracking=96.5 |
| L5_knee_step+10_r4 | latency=120.0 t90=400.0 overshoot=0.0 tracking=98.4 | latency=80.1 t90=400.0 overshoot=0.0 tracking=98.3 | latency=120.0 t90=440.3 overshoot=0.0 tracking=98.4 |
| L5_knee_step-10_r4 | latency=119.9 t90=405.9 overshoot=0.0 tracking=98.4 | latency=79.9 t90=405.9 overshoot=0.0 tracking=98.2 | latency=119.9 t90=440.0 overshoot=0.0 tracking=98.2 |
| L5_knee_step+20_r4 | latency=80.0 t90=644.1 overshoot=0.0 tracking=99.8 | latency=80.0 t90=680.1 overshoot=0.0 tracking=99.1 | latency=120.0 t90=680.1 overshoot=0.0 tracking=99.0 |
| L5_knee_step+40_r4 | latency=119.9 t90=1159.9 overshoot=0.0 tracking=99.5 | latency=79.9 t90=1279.9 overshoot=0.0 tracking=98.0 | latency=119.9 t90=1199.9 overshoot=0.0 tracking=99.1 |
