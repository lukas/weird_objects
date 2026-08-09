# cw-walk-lowgait30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T13:49:22+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait.zip

**wandb_id**: un01yho8

**hypothesis**: Crouch envelope: cw-walk-lowgait PASSED -20mm (mean end-height err ~4mm, gait intact). One variable off it: height ref -20 -> -30mm. Plain: how low can it walk before leg workspace/clearance runs out, so stance height becomes a wide runtime knob. Prediction-if-true: same gait quality at -30mm, height err <=8mm, slip ~parent. Prediction-if-false: clearance/knee workspace runs out -> height err stays >8mm or drag/slip rises markedly = -30mm is outside the envelope (also a useful answer). Strongest alternative: height tracks but sto brittleness worsens in the deeper crouch.

**gate**: own-cfg DR0 15s 6+6 at -30mm: gv 12/12, 0 term, mean end-height err <=8mm, det agg slip/m <=1.35 (parent 1.14 + noise); frames watched det+sto

