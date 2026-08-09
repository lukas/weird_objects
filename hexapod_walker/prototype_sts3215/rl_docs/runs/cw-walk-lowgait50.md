# cw-walk-lowgait50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:38:22+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait40.zip

**wandb_id**: 5tvk6qid

**hypothesis**: Crouch envelope rung 4: -20/-30/-40mm all PASSED (height err <=7mm, det agg slip/m 0.92-0.96, gait intact). One variable off lowgait40: height ref -40 -> -50mm. Plain: keep lowering the body until leg workspace/clearance runs out, so stance height becomes a wide runtime knob. Prediction-if-true: gait intact at -50mm, mean end-height err <=8mm, det agg slip/m <=1.10. Prediction-if-false: knee/clearance workspace exhausts - height err stays >8mm or slip/drag rises markedly = envelope edge found (also a useful answer). Strongest alternative: height tracks but sto brittleness worsens in the deeper crouch (>1/6 stall eps).

**gate**: own-cfg DR0 15s 6+6 at -50mm: gv 12/12, 0 term, mean end-height err <=8mm, det agg slip/m <=1.10; frames watched det+sto

