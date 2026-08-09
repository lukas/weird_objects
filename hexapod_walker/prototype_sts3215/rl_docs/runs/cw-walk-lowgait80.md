# cw-walk-lowgait80

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T19:26:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-lowgait70

**wandb_id**: rdym5x9f

**hypothesis**: Crouch envelope rung 7: -20..-70mm all PASSED (-70mm c60: mean end-height err det 2.0/sto 1.9mm, det agg slip/m 1.07). One variable off lowgait70: height ref -70 -> -80mm. Plain: keep lowering the body until leg workspace/clearance runs out, so stance height becomes the widest possible runtime knob. Prediction-if-true: gait intact at -80mm, mean end-height err <=8mm, det agg slip/m <=1.15. Prediction-if-false: workspace/clearance exhausts (height err stays >8mm or slip/drag rises markedly) = envelope bottom found, a useful answer; -70mm stays the limit. Strongest alternative: height tracks but sto brittleness worsens (>1/6 stall eps). Parent: rl_move/sim/policies/ppo_goal_cw_walk_lowgait70.zip.

**gate**: own-cfg DR0 15s 6+6 at -80mm: gv 12/12, 0 term, mean end-height err <=8mm, det agg slip/m <=1.15; frames watched det+sto

