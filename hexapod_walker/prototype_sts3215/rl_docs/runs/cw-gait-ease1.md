# cw-gait-ease1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T02:00:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-gait-sched1

**wandb_id**: 3pm2pqy8

**hardware_ready**: False

**hypothesis**: Teach a from-scratch, no-imitation walking network to discover stepping by making the physics easier while it learns: gravity starts at half strength (falls are slow, lifting a leg is cheap, and the anti-skate stance charge is easiest to dodge by actually stepping) and ramps back to full strength mid-run (0.4M->1.1M global steps), so the final behavior is earned under real physics. This is GAIT.md P3 lever 3 (physics easing) — the LAST untried lever on the from-scratch gait line after every pricing/schedule/RSI/speed form (levers 2, 4, 5) collapsed into the same freeze-or-skate under nominal physics from step 0. Stack is byte-identical to cw-gait-dragstance1 (full k_drag_stance=8000 stance charge from step 0) with the eased-gravity anneal as the ONE variable, driven by the existing sched.* engine onto the new default-off ease.gravity_scale key (snapshot e40a3ea, test_physics_ease.py + semantics bank green). Anneal ends at exactly 1.0 by 1.1M so every eval-harness episode (own sched clock: 375 ticks x 3072 = 1.15M global steps/ep) resets at NOMINAL physics; judge on measured behavior (fwd travel, slip/m, gait_valid), not eval reward panels.

**gate**: 2M DR0 det at nominal physics (anneal complete by 1.1M): (a) mobility: fwd travel med >=0.3 m (NOT the frozen fingerprint of dragstance1/rsi1/slowfirst1/sched1), AND (b) charge resolving: slip/m med <=3.0 OR gait_valid >=1/6. Kill signature: frozen fingerprint (fwd ~0, walk_loadslip_factor floored) persisting THROUGH the eased phase by 0.8M = easing gave exploration nothing, kill early. If PASS: first-ever from-scratch gait signal — harden with longer anneal at 20M (vel-ceiling ease is the pre-built second axis). If FAIL: GAIT P3 levers 1-5 ALL closed under honest trials; recommendation to the operator is to close the from-scratch gait line (pre-registered, nobc/STATUS.md).

**verdict**: FAIL per pre-registered gate, the false branch exactly: det fwd travel med 0.00m (bar >=0.3m), slip/m med 7.21 det / 19.19 sto, video/contact-sheet a motionless splayed crouch in all 10 frames — the identical freeze fingerprint as dragstance1/rsi1/slowfirst1/anneal1/sched1. Mechanism: walk_loadslip_factor floored at 0.04-0.09 from the FIRST W&B sample through the entire run INCLUDING the whole half-gravity eased phase (sched_value confirms gravity annealed 0.5->1.0 over 0.4M-1.1M exactly as coded); drag charge paid -6.5..-7.9/tick unresolved to the end. Easing gave exploration nothing. GAIT P3 levers 1-5 now ALL closed under honest trials — per the pre-registration, the recommendation to the operator is to CLOSE the from-scratch gait line.

