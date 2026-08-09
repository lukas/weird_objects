# cw-walk-lowgait40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T14:31:31+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait30.zip

**wandb_id**: z208fmwy

**hardware_ready**: no

**hypothesis**: Crouch envelope rung 3: lowgait (-20mm) and lowgait30 (-30mm) both PASSED with height err ~4-5mm, and -30mm slightly REDUCED det slip (0.95 vs 1.14). One variable: height ref -30 -> -40mm. Plain: keep lowering the body until leg workspace/clearance runs out, so stance height becomes a wide runtime knob; also test whether the slip improvement continues with depth. Prediction-if-true: gait intact at -40mm, height err <=8mm, det slip/m <=1.10 (parent 0.95 + noise). Prediction-if-false: workspace/clearance runs out -> height err stays >8mm or slip/drag rises markedly = envelope edge found (also a useful answer). Strongest alternative: height tracks but sto draw-stall brittleness worsens in the deeper crouch.

**gate**: own-cfg DR0 15s 6+6 at -40mm: gv 12/12, 0 term, mean end-height err <=8mm, det agg slip/m <=1.10; frames watched det+sto

**verdict**: PASS — -40mm crouch rung met: own-cfg DR0 det+sto gv 12/12, 0 term, mean end-height err det 3.8mm / sto 5.9mm (gate <=8); det agg slip/m 0.96 (gate <=1.10, = lowgait30's 0.95 within noise). One sto draw-stall ep (prog 0.25, slip/m 5.16) = same known lineage sto brittleness as lowgait/lowgait30 1/6, not worse. Frames det+sto watched: crouched, level, six legs cycling; transport still paddling-class. Envelope holds at -40mm; next rung -50mm.

