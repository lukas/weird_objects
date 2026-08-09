# cw-walk-fwdband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T11:54:55+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**hypothesis**: The robot will only ever be graded on walking straight ahead (operator ruling), yet 40% of its practice commands point sideways or backward - directions it will never be scored on. This run gives the champion walker 100% straight-ahead practice. If focused practice matters, straight-line reliability improves (progress ratio moves into the 0.75-1.25 band vs champion's 1.43 overspeed) without sliding getting worse; if nothing moves beyond noise, command mix is not a lever and the sliding-physics fix remains the only path. One variable off champion ppo_goal_cw_walk_anchorgate.zip md5 35234ddc: goal.walk_heading_max_rad=0.0 (was legacy 60/20/20 mix). A/B partner: cw-steer-fdiag (pi/4 scope). Strongest alternative: any gain is just extra forward gradient steps, not scope - the fdiag A/B at equal budget distinguishes scope-width from budget. Snapshot 33a7a45.

**gate**: own-cfg DR1.0 harness 15s 6+6 fwd: det prog_ratio median 0.75-1.25 (champ baseline 1.43), det slip_per_m <=1.11 champ baseline (not worse beyond noise), gait_valid 12/12, 0 term; DR0 det fwd >=0.55 6/6; frames watched det+sto

**verdict**: Launch aborted mid-flight: authoring cycle was killed at watcher restart after writing INTENT; no trainer process ever started on train-4, 0 steps trained. Infra fault, not policy. Requeued as cw-walk-fwdband-r1.

