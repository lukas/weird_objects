# cw-walk-loadslip-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:37:09+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**hardware_ready**: no

**hypothesis**: Seed-1 twin of cw-walk-loadslip per operator ruling 7 (promotion needs multi-seed panels). Identical config to cw-walk-loadslip except --seed 1: episode-accumulated loaded-slip/m income gate (never touchdown-reset), the definitive reward-side slip test. If both seeds land the same verdict the result is seed-robust; if they diverge, the loadslip outcome is seed noise and cannot close the reward side.

**gate**: DR1.0 harness 15s own-cfg 6+6: det slip_per_m <=1.0 AND sto <=1.1, gait_valid 12/12, 0 term, progress_ratio median 0.75-1.25, speed in band; DR0 det retention

**verdict**: FAIL, CONCORDANT with seed 0 — hypothesis refuted seed-robustly. DR1.0 own-cfg det slip/m med 1.67 (s0: 1.94; champion longdist-r2: 1.06; gate ≤1.0), overspeed prog med 1.40, gv 24/24, 0 term, frames = same paddle-creep. Episode-accumulated loaded-slip stake does not stop sliding in either seed → reward side of skating CLOSED with two seeds; slip root stays sim contact/current pricing (operator calibration class).

**refused_reason**: a process for cw-walk-loadslip-s1 already exists on hexapod-mjx-train-6

