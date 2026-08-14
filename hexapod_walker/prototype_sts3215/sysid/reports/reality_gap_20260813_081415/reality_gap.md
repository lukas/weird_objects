# Hexapod Sim Reality Report

- generated: 2026-08-13T08:14:15
- git: `4d26954`
- servo params `air`: sim_model.json (6968268e879e)
- servo params `rl_move/sim/sim_model_sysid.json`: sim_model_sysid.json (8b8d2f260256)

## Headline gap metrics

| trace | metric | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|---|
| sysid_champion_stand_air_v1_20260813_121343.csv | unloaded q_rmse_deg | — | 0.828 | 0.558 |
| sysid_champion_stand_air_v1_20260813_121343.csv | unloaded qd_rmse_deg_s | — | 5.81 | 11.99 |

## Timing (hardware)

- **sysid_champion_stand_air_v1_20260813_121343.csv**: tick median 40.0 ms (nominal 40.0, p95 40.2, late 0.36%); send→recv RTT median 9.2 ms (p95 9.5)

## Per-segment detail

### sysid_champion_stand_air_v1_20260813_121343.csv

| segment | hardware | sim air | sim rl_move/sim/sim_model_sysid.json |
|---|---|---|---|
| traj_rl_stand_20260812_145534 | rmse=2.37 | rmse=2.633 | rmse=2.273 |
