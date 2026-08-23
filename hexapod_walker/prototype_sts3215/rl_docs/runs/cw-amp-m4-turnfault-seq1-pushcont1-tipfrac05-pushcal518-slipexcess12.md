# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-slipexcess12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T11:46:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: 9v8vij4t

**hypothesis**: Plain English: second dose point of the slip charge — if the calibrated dose (slipexcess6) moves slip but not under the bar, doubling maps the dose-response; if 12x-per-excess also fails to move slip, price is not the lever. Single lever vs pushcal518: reward.k_loadslip_excess=12.0 at loadslip_ok=1.5 (~1.0/tick charge at the current ratio ~3.6 vs ~3/tick walk income — aggressive but survivable; term_penalty=400 anti-suicide already in the recipe per the bank's topple finding).

**gate**: Own-cfg DR-0 gate + eval_amp_m5 walk+yaw sections. PASS = 0/12 raw falls AND walk det slip med <=3.5 AND gait_valid 12/12 AND tips not worse than 0.25. PARTIAL = slip improves >=0.15 but misses 3.5. FAIL = slip unmoved, OR gait collapses/parks/falls (over-charged: prefer slipexcess6's dose), OR suicide/idle farming reappears. Joint FAIL of slipexcess6+12 = additive slip pricing refuted; next lever is partial-strength loadslip income gate.

**verdict**: Result: doubling the loaded-slip excess charge again (k_loadslip_excess=12.0, second dose point) did NOT reduce walk slip -- it went the WRONG way. Evidence: eval_amp_m5 walk section det_slip_med 3.7905, WORSE than both parent pushcal518 (3.67) and the 6x dose (3.629); still misses the <=3.5 bar by more than either. Safety and gait validity intact: own-cfg DR-0 gate 0/12 raw falls (walk+sto), m5 gait_valid 12/12, 0 terms, no parked/idle-farming pathology, video-clean six-leg gait. Yaw tips 0.2234/0.2413 -- in the same 0.20-0.25 band as the rest of the family, no yaw regression since this lever doesn't touch yaw pricing. Why: matches the pre-registered gate's FAIL condition (slip unmoved/worse); the dose-response is non-monotonic in the wrong direction, not just weak -- doubling the charge from 6x to 12x made slip 0.16 WORSE rather than better, which rules out 'just needs more dose'. JOINT FAIL of slipexcess6+12 (both single-seed, same lineage): additive per-tick loaded-slip pricing is REFUTED as the walk-slip lever at these two dose points. What's next: escalate to a structurally different lever -- a partial-strength loadslip INCOME gate (discount the walk-forward-progress reward term when loaded slip is high, rather than an additive side-charge) or root-cause why the policy's slip doesn't respond to per-tick charges (e.g. is 'slip' as measured driven by foot-plant geometry/gait timing rather than a controllable action the policy can cheaply reduce). Flagging DIG-IN for that mechanism question.

