# cw-walk-dragstance1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T19:52:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: yifd0yu2

**hypothesis**: Teach the deployed walking champion to LIFT its feet instead of scraping them: this arm prices the scrape itself (accumulated foot travel while loaded, per stance stroke) at the size the 08-11 drag audit derived, and asks whether the habit breaks in 2M steps. Background: every prior anti-slip attempt (10+ arms) either gated INCOME (policy parks and starves) or charged per-tick slip, which the audit proved cannot separate skating from honest walking (medians overlap; the 0.5mm deadband hid 53-97% of the skate). The new per-STANCE charge separates them 3.3x on measured fingerprints and its launch bank passes (stepping > skate > park ordering; honest gait keeps ~77% of income, skaters pay ~2.5x theirs). One variable vs the parent cw-dep-vref1-r1 (warm start, identical recipe): reward.k_drag_stance=8000, allow 6mm/stance, tick floor 0.25mm. The 08-11 P0 income probe (logs/probe_walk_income/vref1_p0_*) says pricing alone was near-parity because the crouch-paddle genuinely tracks the command in sim — so the charge must make the scrape itself uneconomical without making parking the optimum, exactly what the bank certifies. Prediction-if-true: slip/m drops decisively below the parent band (target <0.6 vs 1.1-1.5) at DR0 AND own-DR 0.35 while forward distance / vel-err stay in the parent band and all six legs cycle. Prediction-if-false: the policy parks or freezes (income starvation — the anneal-up curriculum variant then becomes the only surviving route on this lever), or pays the charge and keeps skating (charge too small vs realized income; do NOT re-run with a bigger k — the audit sized it, a miss refutes the sizing method). Strongest alternative: slip drops but travel collapses toward the scripted gait's slower band — a real trade the operator must price, not a clean pass.

**gate**: 2M det+sto harness at DR0 and own-DR 0.35 vs the frozen parent under the identical eval: PASS if slip/m med <= 0.6 (parent band 1.1-1.5) AND fwd distance + vel-err within the parent band AND gait_valid 6/6 det AND zero falls AND all six feet cycle (no leg parked below 0.2 duty, no flag-leg on video). FAIL if slip/m stays in the parent band (charge farmed) OR fwd distance collapses >40% below parent (park/starve) OR any leg parks. Either FAIL closes the retrofit form of the structural charge; the anneal-up-from-scratch variant (GAIT P3 lever 2, nobc track) is the successor, not a k rung.

