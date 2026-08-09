# cw-walk-payload-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:52:08+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_payload50.zip

**wandb_id**: k2kqugq3

**hardware_ready**: no

**hypothesis**: Composition rung off the fresh payload50 PASS (same move class as lowgait-dr05/wander-dr05): payload competence was proven at dr-scale 0.0 only. One variable off payload50: dr-scale 0.0 -> 0.5 (model DR at half strength) while keeping the absolute dr.mass_scale=1.0,1.5 payload override. Plain: carrying weight must survive real-world physics spread, not just nominal sim. If-true: the two robustness axes stack - own-cfg DR0.5+payload harness gv 12/12, 0 term, det median fwd >=1.1m @30s, and DR0 no-payload retention stays clean - payload becomes a deployable robustness rung. If-false: DR draws plus heavy mass overload the gait (terminations or heavy-draw squat-shuffle spreads to median draws) - payload stays a nominal-sim skill and we note the compose ceiling like strafe-dr10. Strongest alternative: it holds but slip creeps past the champion band under DR like other DR composes - then the caveat is slip, not gait.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 no-payload retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: FAIL on the pre-registered DR0 no-payload retention clause: det slip/m 1.38 > 1.24 gate, det prog med 0.54 vs parent payload50's 0.95 (fwd 1.55 -> 1.36 m) — DR0.5 training charged nominal speed-band tracking, well beyond eval noise. The compose axis itself HELD: own-cfg DR0.5+payload gv 12/12, 0 term, det med fwd 1.36 >= 1.1, cleanest compose panel yet (DR0-payload heavy-tail crater gone: worst det ep 1.20 m vs parent's 0.74 m). Frames both conditions: normal six-leg gait, no flag leg/squat — erosion is slower+slippier, not pathology. First dr05 compose to charge nominal; check comshift-dr05/deadband-dr05/fricvar-dr05/latjit-dr05 retentions for the same erosion before requeueing payload-DR (shorter-steps or lower-dr anneal is the candidate fix if the pattern repeats). Payload capability row stays with payload50 (DR0).

