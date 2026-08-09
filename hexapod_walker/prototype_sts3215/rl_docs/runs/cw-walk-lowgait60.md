# cw-walk-lowgait60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:48:55+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait50.zip

**wandb_id**: oi6ffvrz

**hardware_ready**: no

**hypothesis**: Crouch envelope rung 5: -20..-50mm all PASSED (height err <=7mm, det agg slip/m 0.92-1.04, gait intact). One variable off lowgait50: height ref -50 -> -60mm. Plain: keep lowering the body until leg workspace/clearance runs out, so stance height becomes a wide runtime knob. Prediction-if-true: gait intact at -60mm, mean end-height err <=8mm, det agg slip/m <=1.15. Prediction-if-false: workspace/clearance exhausts - height err stays >8mm or slip/drag rises markedly = envelope bottom found (useful answer; -50mm stays the limit). Strongest alternative: height tracks but sto brittleness worsens (>1/6 stall eps). Parent: rl_move/sim/policies/ppo_goal_cw_walk_lowgait50.zip.

**gate**: own-cfg DR0 15s 6+6 at -60mm: gv 12/12, 0 term, mean end-height err <=8mm, det agg slip/m <=1.15; frames watched det+sto

**verdict**: PASS — -60mm crouch rung met: own-cfg DR0 det+sto gv 12/12, 0 term, mean end-height err det 3.9mm / sto 4.5mm (gate <=8); det agg slip/m 1.00 (gate <=1.15; parent lowgait50 1.04). One sto draw-stall ep (prog 0.39, slip/m 4.16) = known lineage 1/6 trait, not worse. Frames det+sto watched: deep crouch held level (tilt <=1.8deg), six legs cycling, no flag leg; transport still paddling-class. Height ref validated as runtime knob -20..-60mm. hardware-ready: no (contact-pricing slip root open).

