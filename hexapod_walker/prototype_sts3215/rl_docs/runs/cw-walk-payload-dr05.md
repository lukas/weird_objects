# cw-walk-payload-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:52:08+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_payload50.zip

**wandb_id**: k2kqugq3

**hypothesis**: Composition rung off the fresh payload50 PASS (same move class as lowgait-dr05/wander-dr05): payload competence was proven at dr-scale 0.0 only. One variable off payload50: dr-scale 0.0 -> 0.5 (model DR at half strength) while keeping the absolute dr.mass_scale=1.0,1.5 payload override. Plain: carrying weight must survive real-world physics spread, not just nominal sim. If-true: the two robustness axes stack - own-cfg DR0.5+payload harness gv 12/12, 0 term, det median fwd >=1.1m @30s, and DR0 no-payload retention stays clean - payload becomes a deployable robustness rung. If-false: DR draws plus heavy mass overload the gait (terminations or heavy-draw squat-shuffle spreads to median draws) - payload stays a nominal-sim skill and we note the compose ceiling like strafe-dr10. Strongest alternative: it holds but slip creeps past the champion band under DR like other DR composes - then the caveat is slip, not gait.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 no-payload retention det 6/6 gv, det slip/m <=1.24; frames watched det

