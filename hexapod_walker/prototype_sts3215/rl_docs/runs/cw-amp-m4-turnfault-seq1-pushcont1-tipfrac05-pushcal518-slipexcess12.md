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

**verdict**: Doubling the slip charge to 12x-per-excess did not move slip at all — the policy simply pays the fine and keeps skating, so additive slip pricing at the aggressive dose is NOT the lever. Evidence: eval_amp_m5 walk section det slip med 3.79 vs bar 3.5 and vs parent pushcal518 3.67 (family band 3.62-3.82 — unmoved; the gate's pre-registered FAIL branch). The charge demonstrably fired in training: env/reward_loadslip_excess ~-0.98/tick at end (exactly the predicted ~1/tick vs ~3/tick walk income), yet training walk_loadslip_ratio only edged ~3.6->3.50 and reward still rose (quarters 33.8->156.4) — the reward optimum sits at slip ~3.5-3.8 even after paying the charge. NOT over-charged: safety and gait fully held — own-cfg DR-0 gate 0/12 raw falls (0 terms det+sto; family-signature caveats only: 1 sto sacrificed-leg ep, 2 near-stationary sto eps), m5 walk gait_valid 12/12 with det prog med 1.03, video shows clean upright tripod cycling, no park/suicide farming; yaw tips 0.2234/0.2413 within the <=0.25 no-regress leg (still over the 0.20 m5 bar, unchanged vs family). Why: the marginal cost of slipping less exceeds 1/tick — slip ~3.6 is structurally coupled to how this gait produces progress (stance micro-skate), so price nudges cannot buy it down; that is a mechanism problem, not a dose problem. Hardware-ready: no (walk slip above M5 bar). Next: per the pre-registered joint rule, if slipexcess6 (concurrent cycle) also fails to move slip, additive pricing is refuted for this axis and the next lever is the partial-strength loadslip income gate; this higher-dose zero-movement result already makes that outcome near-certain.

