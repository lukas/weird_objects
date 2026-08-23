# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-slipexcess12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T11:46:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: 9v8vij4t

**hypothesis**: Plain English: second dose point of the slip charge — if the calibrated dose (slipexcess6) moves slip but not under the bar, doubling maps the dose-response; if 12x-per-excess also fails to move slip, price is not the lever. Single lever vs pushcal518: reward.k_loadslip_excess=12.0 at loadslip_ok=1.5 (~1.0/tick charge at the current ratio ~3.6 vs ~3/tick walk income — aggressive but survivable; term_penalty=400 anti-suicide already in the recipe per the bank's topple finding).

**gate**: Own-cfg DR-0 gate + eval_amp_m5 walk+yaw sections. PASS = 0/12 raw falls AND walk det slip med <=3.5 AND gait_valid 12/12 AND tips not worse than 0.25. PARTIAL = slip improves >=0.15 but misses 3.5. FAIL = slip unmoved, OR gait collapses/parks/falls (over-charged: prefer slipexcess6's dose), OR suicide/idle farming reappears. Joint FAIL of slipexcess6+12 = additive slip pricing refuted; next lever is partial-strength loadslip income gate.

