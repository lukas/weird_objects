# cw-walk-parkstart

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T04:42:51+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_kgate.zip

**wandb_id**: orxkp2v1

**hypothesis**: Park persists because park-adjacent states are rare (1/6 eps, entered ~t=1s) so the exit gradient is starved; income already priced (kgate: park return 274 vs ~1250, behavior unchanged). Starting 25% of walk eps in a jittered tripod park densifies the exit gradient. If-true: park-exit eval (frac=1.0) >=10/12 fwd>=0.30+gv, standard harness park 1/6 sto -> 0/12, det[2] churn converts, fwd+gv 12/12. If-false: (a) exits synthetic parks but sto[4] persists -> start-distribution mismatch -> harvest own-park starts; (b) no exit -> motorically hard -> rung-2 load evenness. Strongest alt: park starts teach settle-competence without touching walking - distinguished by standard harness unchanged AND exit eval passing. Parent ppo_goal_cw_walk_kgate.zip md5 b703f7b0. Probe probe-walk-parkstart-b PASS. Snapshot 1594ef7.

**gate**: 15s DR0 harness 6eps/mode det AND sto: fwd >=0.40m 12/12 AND gait_valid 12/12 AND >=2 swings/leg AND 0 term AND no final-third degradation AND det fwd mean >=0.50m; park-exit eval at walk_park_start_frac=1.0: >=10/12 fwd >=0.30m AND gait_valid; retention 5s det slip/m <=1.8

**verdict**: KILLED by operator switch-over 2026-08-09: last CPU-stack training run; arm relaunched as cw-walk-parkstart-mjx on hexapod-mjx-train-1 (GPU-MJX stack, identical config). No training verdict; see the -mjx run.

