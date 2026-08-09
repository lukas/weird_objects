# cw-walk-lowgait70

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:48:23+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait60.zip

**wandb_id**: v06ypzvd

**hardware_ready**: no

**hypothesis**: Crouch envelope rung 6: -20..-60mm all PASSED (-60mm c56: height err det 3.9/sto 4.5mm, det agg slip/m 1.00). One variable off lowgait60: height ref -60 -> -70mm. Plain: keep lowering the body until leg workspace/clearance runs out, so stance height becomes the widest possible runtime knob. Prediction-if-true: gait intact at -70mm, mean end-height err <=8mm, det agg slip/m <=1.15. Prediction-if-false: workspace/clearance exhausts (height err stays >8mm or slip/drag rises markedly) = envelope bottom found, a useful answer; -60mm stays the limit. Strongest alternative: height tracks but sto brittleness worsens (>1/6 stall eps).

**gate**: own-cfg DR0 15s 6+6 at -70mm: gv 12/12, 0 term, mean end-height err <=8mm, det agg slip/m <=1.15; frames watched det+sto

**verdict**: PASS — -70mm crouch rung met: own-cfg DR0 det+sto gv 12/12, 0 term, mean end-height err det 2.0mm / sto 1.9mm (gate <=8); det agg slip/m 1.07 (gate <=1.15; parent lowgait60 1.00). One sto draw-stall ep (prog 0.38, slip/m 3.30) = known 1/6 lineage sto trait, not worse than -60mm. Frames det+sto watched: deep crouch held level, six legs cycling, no flag leg; the stall draw churns near-in-place without falling; transport still paddling-class. Height ref validated as runtime knob -20..-70mm. hardware-ready: no (contact-pricing slip root open).

