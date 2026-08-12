# cw-stand-footzsharp1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-12T12:41:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-stand-footlow2-r1

**wandb_id**: s6vt8684

**hardware_ready**: False

**hypothesis**: Close the last millimeter of the quiet stand: the current policy commands one foot 0.9mm above the floor, and under the foot-height imitation term's default 10mm normalization that hover costs ~0.008 loss — below supervision resolution (measured in the footlow2-r1 dig-in). This arm sharpens the normalization 10mm->3mm (one variable, ~11x more gradient on a 1mm hover) to test whether the residual hover is supervision-resolution-limited or a PPO-preferred equilibrium the anchor cannot price away. Prediction-if-true: foot 1 duty recovers to >=0.5 in det hold (the same policy family holds duty 0.97 at +0.4mm command in footlow1) with rise/lower untouched. Prediction-if-false: hover unchanged at sharper scale -> the <=1mm residual is accepted as cosmetic (below real-servo slop) and only the hardening arm carries forward. Strongest alternative: the sharper term destabilizes rise/lower supervision balance (a new seesaw), which the retention clauses catch.

**gate**: Judge WITH rsi start_kind labels (snapshot da367c9). PASS if det hold reaches duty>=0.5 on ALL six feet in 6/6 episodes (the foot-1 hover closes) AND all cold-start (non-rsi) det rises stay valid_plant with h_err<=5mm AND det+sto lower >=10/12 valid_plant. FAIL if foot-1 stays below duty 0.5 with commanded hover >=0.5mm (resolution hypothesis refuted -> accept residual, stop this axis), or any retained mode regresses below its clause.

**verdict**: PASS per gate — the sub-mm hover was supervision-resolution-limited, exactly prediction-if-true: sharpening bc_anchor_foot_z normalization 10mm->3mm closes the foot-1 park in det hold (all six feet duty >=0.96 in 6/6, tail 0.1deg, drag med 153mm; parent footlow2-r1 held foot-1 at duty 0.03). Cold-start det rises all valid_plant (bridge/crouch h_err <=2.8mm; the 2 rise misses at 15.0-15.8mm are rsi starts, gate-excluded, same 15mm signature as the lineage flat-stall), det+sto lower 12/12 plant with det flush sub-mm. Caveats: standard draw again sampled zero flat cold starts; sto hold shake-parks (min duty 0.26) match the parent band. Product is a LEVER (foot_z norm 3mm), not a candidate — champion candidates remain footlow2-hard1/stable1, which already reached park-free via 10M budget.

