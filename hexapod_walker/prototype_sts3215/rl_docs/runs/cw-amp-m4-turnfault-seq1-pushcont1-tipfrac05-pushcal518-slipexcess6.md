# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-slipexcess6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T11:39:43+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: wnugw2gq

**hypothesis**: Plain English: the robot's feet slide too much while loaded because sliding costs the reward nothing; charging for it should pull walk slip under the M5 bar. Single lever vs pushcal518: enable the additive loaded-slip excess charge (reward.k_loadslip_excess=6.0 at the bank-calibrated loadslip_ok=1.5 threshold — the 08-21-calibrated V5 anti-skate stack dose, semantics tests green, never yet trained in this lineage). Training walk_loadslip_ratio runs 3.55-3.73, so the charge is ~0.5/tick vs ~3/tick walk income: real pressure, not confiscation. Dig-in finding this tests: family-wide walk det slip med ~3.6-4.3 vs bar 3.5 because slip is literally unpriced (walk_loadslip_gate=0, k_loadslip_excess=0 across the whole tipfrac05 family).

**gate**: Own-cfg DR-0 gate + eval_amp_m5 walk+yaw sections. PASS = 0/12 raw falls AND walk det slip med <=3.5 AND gait_valid 12/12 AND tips not worse than 0.25 band. PARTIAL = slip improves >=0.15 vs pushcal518's ~3.67 but misses 3.5 (read slipexcess12 for dose-response). FAIL = slip unmoved or gait collapses (gait_valid <12 / falls / parked legs) — per-tick slip charge refuted at trainable dose; next lever is loadslip income gate at partial strength, not more charge.

