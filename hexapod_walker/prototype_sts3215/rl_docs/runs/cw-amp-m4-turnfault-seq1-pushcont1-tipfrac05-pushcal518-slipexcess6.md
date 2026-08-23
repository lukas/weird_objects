# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-slipexcess6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T11:39:43+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: wnugw2gq

**hypothesis**: Plain English: the robot's feet slide too much while loaded because sliding costs the reward nothing; charging for it should pull walk slip under the M5 bar. Single lever vs pushcal518: enable the additive loaded-slip excess charge (reward.k_loadslip_excess=6.0 at the bank-calibrated loadslip_ok=1.5 threshold — the 08-21-calibrated V5 anti-skate stack dose, semantics tests green, never yet trained in this lineage). Training walk_loadslip_ratio runs 3.55-3.73, so the charge is ~0.5/tick vs ~3/tick walk income: real pressure, not confiscation. Dig-in finding this tests: family-wide walk det slip med ~3.6-4.3 vs bar 3.5 because slip is literally unpriced (walk_loadslip_gate=0, k_loadslip_excess=0 across the whole tipfrac05 family).

**gate**: Own-cfg DR-0 gate + eval_amp_m5 walk+yaw sections. PASS = 0/12 raw falls AND walk det slip med <=3.5 AND gait_valid 12/12 AND tips not worse than 0.25 band. PARTIAL = slip improves >=0.15 vs pushcal518's ~3.67 but misses 3.5 (read slipexcess12 for dose-response). FAIL = slip unmoved or gait collapses (gait_valid <12 / falls / parked legs) — per-tick slip charge refuted at trainable dose; next lever is loadslip income gate at partial strength, not more charge.

**verdict**: Result: the per-tick loaded-slip excess charge (k_loadslip_excess=6.0 at loadslip_ok=1.5) did not move walk slip past noise. Evidence: eval_amp_m5 walk section det_slip_med 3.629 vs parent pushcal518's 3.67 -- a 0.04 improvement, far short of the gate's own PARTIAL threshold (>=0.15) and still misses the 3.5 bar. gait_valid 12/12, 0 det/sto terms, video-clean six-leg gait (contact sheet). Own-cfg DR-0 gate also 0/12 raw falls. Why: matches the pre-registered gate's FAIL condition (slip unmoved). The charge (~0.5/tick against ~3/tick walk income per the hypothesis) is real but too weak at this dose to bite the policy's slip behavior. What's next: read jointly with slipexcess12 (2x the dose, same lineage) before concluding -- if 12x also fails to move slip, additive per-tick slip pricing is refuted as the lever entirely and the next lever is a partial-strength loadslip income GATE (multiplying the walk income term down when loaded slip is high) rather than more additive charge.

