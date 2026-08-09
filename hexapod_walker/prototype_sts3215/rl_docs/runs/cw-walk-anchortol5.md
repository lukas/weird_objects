# cw-walk-anchortol5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T09:44:19+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: wn7pl2c4

**hypothesis**: Pre-registered audit-driven tolerance correction (parent anchorgate shape (c), fired by c1 cycle 32): c1 satisfied the tol=10 anchor gate by CADENCE INFLATION (+23% stances, reward_step_event +24%, per-stance creep 11.7-15.7mm hugging the allowance; free slip = cadence x tol => tol=10 floor 0.80-0.94 slip/m >= the 1.0 gate; collectible factor re-audited on champion 0.82-0.93 = gate non-binding) while slip stayed 1.283. HYPOTHESIS: re-sizing the allowance to tol=5mm (income-weighted collectible factor re-audited at 0.53-0.63 on current champion behavior - the same stake magnitude that produced cycle 31's real det slip movement 1.543->1.240) restores a binding anchoring gradient; at current cadence the tol=5 floor is 0.40-0.47 slip/m, so det slip <=1.0 is reachable WITHOUT cadence change. One variable off champion ppo_goal_cw_walk_anchorgate.zip md5 35234ddc: reward.anchor_tol_mm 10->5. Prediction-if-true: DR1.0 agg slip/m det<=1.0 (sto follows or trends down), det stance count stays <=~65/ep, anchor_frac re-drops then re-climbs EARNED, speed in 0.043-0.057 band, DR0 retention holds. Prediction-if-false: cadence inflates again (det stances >=~75/ep) with slip/m flat >1.0 => the PER-TOUCHDOWN ALLOWANCE RESET is the structural defect, not the tolerance size => tolerance-correction rung CLOSED per pre-registration ('one correction, else rung closed'); escalation = displacement-gated step-event credit (0-c.2) or operator sim-pricing ruling. Also false: income starvation (fwd collapses, DR0 det fwd <0.45) => tol=5 too tight for warm start, rung closed same way. Strongest alternative: slip falls via slower speed - excluded by the speed-band clause; second alternative: det improvement is panel luck - excluded by requiring the cadence clause AND slip clause together on the fixed panel.

**gate**: DR1.0 harness 15s own-cfg 6+6: agg slip/m det<=1.0 AND sto<=1.0, gv 12/12, 0 term, det stance count <=~65/ep (cadence exploit watch); DR0 harness 15s: det fwd mean >=0.55, gv 12/12, fwd-hemisphere sto fwd>=0.40 5/5 (backward draw recorded-excluded pending operator ruling); frames: anchored stance vs paddle; W&B walk_anchor_frac must be EARNED (reward_walk held) not forfeited

**verdict**: FAIL — DR1.0 agg slip/m det 1.222 / sto 1.283 vs gate <=1.0 (champion anchorgate 1.240/1.245, c1 1.283/1.326 — deltas inside per-ep noise, NO EVIDENCE of slip change). Cadence det 65.7 stances/ep mean (one ep 81) vs champion 47.2 / c1 58.2 — clause <=~65 breached marginally; per-stance creep 11.8mm >> 5mm tol, so the policy did NOT floor-ride tol=5; instead it ATE the discount: training anchor_frac only regained 0.74 (vs 0.906 at tol=10), reward_walk 1.06/tick vs 1.29 (-18% income accepted over anchoring); reward_step_event +32% (cadence income still rising). DR0 retention PASS (det fwd 0.724, gv 12/12, 0 term, fwd-hemi sto 5/5; backward draw sto5 recorded-excluded). NOT HARDWARE-READY: feet grind 1.22 m per meter at DR1.0, same sprawly creep in frames. Hypothesis REFUTED (if-true failed); failure shape HYBRID between if-false-1 (cadence up but <75) and a third shape: binding gate + policy pays it. Tolerance rung CLOSED per pre-registration (one correction, else closed). Champion UNCHANGED (35234ddc). Escalation: displacement-gated step-event credit (0-c.2) per pre-registration.

