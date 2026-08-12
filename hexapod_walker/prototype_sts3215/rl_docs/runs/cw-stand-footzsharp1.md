# cw-stand-footzsharp1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T12:41:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-stand-footlow2-r1

**wandb_id**: s6vt8684

**hypothesis**: Close the last millimeter of the quiet stand: the current policy commands one foot 0.9mm above the floor, and under the foot-height imitation term's default 10mm normalization that hover costs ~0.008 loss — below supervision resolution (measured in the footlow2-r1 dig-in). This arm sharpens the normalization 10mm->3mm (one variable, ~11x more gradient on a 1mm hover) to test whether the residual hover is supervision-resolution-limited or a PPO-preferred equilibrium the anchor cannot price away. Prediction-if-true: foot 1 duty recovers to >=0.5 in det hold (the same policy family holds duty 0.97 at +0.4mm command in footlow1) with rise/lower untouched. Prediction-if-false: hover unchanged at sharper scale -> the <=1mm residual is accepted as cosmetic (below real-servo slop) and only the hardening arm carries forward. Strongest alternative: the sharper term destabilizes rise/lower supervision balance (a new seesaw), which the retention clauses catch.

**gate**: Judge WITH rsi start_kind labels (snapshot da367c9). PASS if det hold reaches duty>=0.5 on ALL six feet in 6/6 episodes (the foot-1 hover closes) AND all cold-start (non-rsi) det rises stay valid_plant with h_err<=5mm AND det+sto lower >=10/12 valid_plant. FAIL if foot-1 stays below duty 0.5 with commanded hover >=0.5mm (resolution hypothesis refuted -> accept residual, stop this axis), or any retained mode regresses below its clause.

