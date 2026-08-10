# cw-walk-multiaxis-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T21:31:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-multiaxis1

**wandb_id**: 4jpb15sh

**hardware_ready**: no

**hypothesis**: Compose rung off the c63 multiaxis1 PASS (widen-then-harden pattern of payload/comshift/deadband/fricvar/latjit-dr05): the 4-axis stacked recipe was proven at dr-scale 0 only. One variable off multiaxis1: dr-scale 0.0 -> 0.5 while keeping all four dr overrides. Plain: the stacked-imperfections gait must survive general physics spread too, or it stays a nominal-sim trick. If-true: own-cfg DR0.5+4-axis harness gv 12/12, 0 term, det med fwd >=1.1m @30s AND DR0 nominal retention stays clean (slip/m <=1.24) - the stacked recipe becomes the robustness-champion candidate. If-false: DR draws on top of the 4-axis stack overload the gait (terminations, or nominal retention charged like payload-dr05) - the stack composes with DR only in stages. Strongest alternative: gait holds but slip creeps past champion band under DR (fricvar-dr05 pattern) - caveat is slip, not gait. Parent: rl_move/sim/policies/ppo_goal_cw_walk_multiaxis1.zip.

**gate**: Own-cfg harness at --dr-scale 0.5 + all four dr overrides, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: FAIL (letter/near-miss). Own-cfg DR0.5+4-axis-stack harness gv 12/12, 0 term, det med fwd 1.21m (>=1.1 gate), slip med 1.13; sto med fwd 1.23m, slip 1.19 -- exposure panel clears cleanly. But the pre-registered DR0 nominal-retention cap (det slip/m <=1.24) is literally missed: det slip med 1.27 (2% over) plus one severe sto outlier (draw 4: fwd 0.79m, slip 4.39, known DR-compose sto-tail). Milder than payload-dr05s crater (slip1.38/prog0.54 there vs prog staying in-band here at 0.94/0.92) but still a real letter-miss, not noise. Reading: stacking FOUR axes simultaneously at DR0.5 costs just enough to cross the single-axis retention cap -- axis-stacking ceiling for this step budget is 4 at DR0 (multiaxis1); adding generic DR0.5 on top does not hold. No requeue at this step count; more steps or fewer simultaneous axes would be the next lever, not pursued further this line.

