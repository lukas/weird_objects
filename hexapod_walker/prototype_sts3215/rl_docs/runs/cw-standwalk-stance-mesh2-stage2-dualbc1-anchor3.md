# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS

**created**: 2026-08-26T19:16:32+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**wandb_id**: 09kgtd67

**hypothesis**: Plain English: now that the shared-Adam momentum leak is fixed (anchor2/-s1, isolate_update=1, both CANARY PASS on the leak-fix question), the stance-only BC anchor is no longer capped by an unwanted walk side-effect -- so the reason its OWN dose (coef=3.0, unchanged since dual1) was kept low no longer applies. This arm is anchor2s exact recipe (same dual-teacher BC init, same isolate_update=1 fix) with ONE change: train.bc_anchor_coef 3.0->6.0 (2x dose), directly targeting anchor2s own residual: hold/sto stuck at a clean TOTAL hold_min_load termination (6/6 both DR, both seeds) and lower/det weak (2-3/6 fail) while walk stayed clean. Prediction-if-true: hold/sto and lower/det termination/success move toward isolated (<=1/6 fail) on both seeds while walk stays at-or-above anchor2s clean band (no re-emergence of any anchor1-class catastrophe -- the isolate_update fix should make walk dose-INSENSITIVE to the anchor coef, since its grads are exactly zero on walk ticks and now its momentum is isolated too). Prediction-if-false: hold/sto stays an unchanged 6/6 term (the anchor coef was never the bottleneck; the stance-only signal itself, or its lookahead/tolerance params, needs redesign) OR walk regresses (isolation is imperfect / a second leak channel exists), escalating to a design dig-in rather than a further dose step.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Joint call with the seed twin. LEAK-STAYS-FIXED PASS if walk shows no anchor1-class catastrophe on either seed at the higher dose (det gait_valid >=5/6, prog_ratio >= ~0.2, no 5-leg sacrifice, no negative-prog high-slip shuffle). DOSE-WORKS if hold/sto and/or lower/det move measurably toward isolated (<=2/6 fail, i.e. real improvement over anchor2s 6/6 and 2-3/6) on both seeds without walk regressing. FAIL if hold/sto stays an unchanged 6/6 term at 2x dose (coef is not the lever) or if walk regresses toward anchor1s catastrophe (isolation broke down at higher dose).

**verdict**: CANARY PASS (LEAK-STAYS-FIXED at 2x dose; joint w/ -s1) -- DOSE-WORKS branch FAILS. Mechanism-health read: raising bc_anchor_coef 3.0->6.0 on the leak-fixed (isolate_update=1) dual-core recipe does NOT reopen the momentum-leak wreck -- walk/det stays clean both DR levels (gait_valid 6/6, prog 0.29-0.43 within/above the anchor2 modeseq1 band, zero sacrificed legs, no anchor1-class freeze/shuffle). The DOSE-WORKS behavioral question this arm was launched to answer reads FAIL: hold/sto stays an UNCHANGED total 6/6 hold_min_load termination at both DR-0 and own-DR(0.5), bit-for-bit the same signature as anchor2's coef=3.0 read -- bc_anchor_coef is confirmed NOT the lever for the stochastic hold collapse, it can be left at 3.0. lower/det and rise are unchanged mixed-fail (not this arm's target axis). Next: do not raise this coef further; the hold/sto failure needs a different lever -- the campaign's own std-anneal fix (proven on the isolated rung-7 bcanchor3-stdanneal and again this cycle on stancemix-bcchain3-stdanneal, both of which cleared hold/sto) is the analogous candidate for the dual-core stage-2 recipe.

