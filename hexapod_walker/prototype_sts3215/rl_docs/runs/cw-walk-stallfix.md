# cw-walk-stallfix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:07:54+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**wandb_id**: 4ascrs7t

**hypothesis**: 0-c(iii) RELIABILITY: draw-stall fix via command-conditioning. Three converging results (tilt50 dig-in: same command draws stall parent+child+both seeds; wander30: 5s resampling produced ZERO stalls in 12 eps; longdist-dr05: DR training left DR1.0 sto at 3/6 = parent) say the in-place stalls are conditioned on specific fixed commands from episode start, not physics. One variable off champion longdist-r2: add goal.walk_cmd_resample_s=5.0 to the straight 0.05-0.06 band (NO heading changes, NO stops) so training visits mid-episode command re-draws and parked-to-walking transitions. If-true: on FIXED-command eval (resample off) DR0+DR1.0 sto panels show zero stall eps (prog<0.5) and DR1.0 sto lifts to >=5/6 — stalls were an unvisited-transition artifact and resample training erases the attractor. If-false: stalls persist on fixed draws (policy still freezes when the initial command is one of the bad draws) — the defect is in initial-state/command conditioning, next lever is start-state diversity, not command schedule. Strongest alternative: resample training helps only when resampling is also present at eval (as wander30 already proved) and fixed-command behavior is untouched.

**gate**: fixed-command (no resample) DR0 + DR1.0 panels 6+6 each: zero eps with prog_ratio<0.5, gv all, 0 term, DR1.0 det agg slip/m <=1.24, DR1.0 sto >=5/6; own-cfg (resample on) DR0 retention gv 12/12 0 term; frames watched det

