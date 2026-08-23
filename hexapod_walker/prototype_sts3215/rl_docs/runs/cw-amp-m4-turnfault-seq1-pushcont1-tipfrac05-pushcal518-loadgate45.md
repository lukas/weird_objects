# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-loadgate45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T12:37:41+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: agco4nu1

**hypothesis**: Plain English: the robot's feet slide because sliding only ever cost it a flat fine it could pay from other income - this arm makes the WALKING INCOME ITSELF shrink with slip (full-strength income gate on a recalibrated band), so the only way to earn full pay is to walk cleaner. Single lever vs pushcal518: reward.walk_loadslip_gate 0->1.0 with band loadslip_ok=1.5/loadslip_max=4.5 (the 08-22 calibration fix: the DEFAULT band 0.75/1.5 has factor identically 0 at the family's operating ratio ~3.6 - zero gradient, a no-information tax, bank-pinned). Dose honesty (bank test loadgate_recal_band_out_doses_the_refuted_charge): the pre-registered PARTIAL gate (g=0.5) would exert ~0.5/tick marginal pressure - the SAME dose slipexcess12 (12x additive charge, ~1/tick paid, slip unmoved 3.79) already refuted - so this launches at FULL strength: ~1.0/tick marginal (2x refuted), ~70% of walk income withheld at ratio 3.6, income floor 0.9/tick (no statue cliff; bank 5/5 green). Prediction-if-true: m5 walk det slip med <=3.5 or >=0.15 improvement. Prediction-if-false: slip unmoved again - income-coupled pricing ALSO refuted at 2x dose, pricing on the slip axis is CLOSED and the next step is a gait-level mechanism (k_walk_swing-style swing shaping) or the q_20260823T0130Z bar-tolerance ruling. Strongest alternative: slip ~3.6 is the stress_mix command schedule's floor (turn/stop segments scrub by construction), not gait sloppiness.

**gate**: Own-cfg DR-0 gate + eval_amp_m5 walk+yaw sections. PASS = 0/12 raw falls AND walk det slip med <=3.5 AND gait_valid 12/12 AND tips within 0.25 band. PARTIAL = slip improves >=0.15 vs 3.67 but misses 3.5 (income coupling works, tune band). FAIL = slip unmoved (+-0.15) or gait collapse/parks/falls - pricing CLOSED on the slip axis (additive 6x/12x + income-gate 2x-marginal all refuted); escalate to gait mechanism or bar ruling, never another price arm.

**verdict**: The full-strength income gate did NOT reduce walk slip — it made it slightly worse while reward still rose, so PRICING ON THE SLIP AXIS IS NOW CLOSED per this gate's own pre-registration (additive 6x/12x charges + 2x-marginal income gate all refuted). Single lever: loadslip INCOME gate at g=1.0 (ok=1.5/max=4.5 recalibrated band, ~1.0/tick marginal, 70% of walk income withheld at the operating ratio, bank 5/5 green). eval_amp_m5 walk: det slip med 4.072 vs parent 3.67 and bar <=3.5 (own-cfg DR-0 gate eval read 4.79) — moved the WRONG way, far outside the +-0.15 unmoved band; PARTIAL needed an improvement >=0.15. Everything else held: 0/12 raw falls det+sto, m5 walk gait_valid 12/12, tips 0.2386/0.2376 within the 0.25 band, video-clean six-leg cycling. Reward rose every quarter (37->179): the policy simply paid the tax (or restructured income) rather than slip less — the measured slip is apparently not on the policy's reachable cost gradient, the same conclusion slipexcess12's economics pointed at. Next per pre-registration: NEVER another price arm; either a gait-level mechanism (k_walk_swing-style swing shaping) or a bar ruling on the assumed 3.5 slip bar (q_20260823T0130Z amendment), preceded by a root-cause read of WHAT drives measured slip (foot-plant/gait-timing artifact vs controllable action) — noamp1 already showed the AMP style term is the current slip FLOOR-HOLDER (removing it regressed slip 3.67->3.92).

