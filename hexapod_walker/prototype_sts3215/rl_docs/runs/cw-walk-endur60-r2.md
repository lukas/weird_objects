# cw-walk-endur60-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T16:01:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**hypothesis**: 60s endurance folded onto the CURRENT champion. endur60(+seed twin s1) proved long-horizon endurance is seed-robust off the OLD parent anchorgate (both seeds: 3m @60s, no gait decay), but seed-0's best-ever slip 0.887 was seed luck (s1: 1.13). One variable off champion longdist-r2 (det slip 0.94-0.96 @30s): episode-seconds 30 -> 60, identical recipe otherwise. Plain: teach the CHAMPION to walk a full minute without giving back its slip gains. Prediction-if-true: det med fwd >=2.4m @60s AND det slip/m med <=1.0 (champion band holds at 60s) - a champion-path long-walk candidate for a DR1.0 panel vs r2. Prediction-if-false: slip inflates >1.0 at the longer horizon (endurance trades against slip on this lineage too) - endurance stays a separate capability, champion stands at 30s. Strongest alternative: no learning delta at all (r2 already generalizes to 60s eval; then the arm is pure confirmation and cheap).

**gate**: DR0 60s 6+6: det+sto med fwd >=2.4m, 0 term, gv 12/12, det slip/m med <=1.0; frames watched det

