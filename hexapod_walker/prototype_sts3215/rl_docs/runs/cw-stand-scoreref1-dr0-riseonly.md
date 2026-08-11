# cw-stand-scoreref1-dr0-riseonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T01:52:29+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-scoreref1-dr0

**wandb_id**: apt28ojt

**hypothesis**: Erosion forensics, controlled variable 3. Tonight's ladder: pricing fixed (score1: cheats unpaid), crutch feasible (replay valid_plant under loaded params), DR exonerated (dr0: warm start begins ALIGNED, ref 0.65/tick, then erodes), LR exonerated (dr0-lowlr at 5e-5: same erosion, slower). The gradient direction itself points away from tracking -- something else pays for leaving. Suspect A: cross-mode interference -- 45% of updates come from LOWER episodes whose legacy income is dense (~1/tick kernel + finish); if shared weights cannot serve both modes, lower wins and rise collapses (matches rfix-warm1: lower 6/6 while rise 0). This arm: goal-mix rise=1.0 (no lower, no hold), everything else identical to scoreref1-dr0. If tracking HOLDS: interference confirmed -> mode one-hot / separate heads (already a named [CODE] item) is the fix. If it still erodes: within-rise cause, next suspect is exploration noise (std ~0.2 rad = 11deg/joint) vs the 6-deg kernel -- sigma-12-with-feet-gate is the follow-up.

**gate**: W&B env/reward_rise_ref median >=0.3/tick sustained (vs 0.02-0.04 eroded baseline) = interference CONFIRMED even if valid_plant stays low; erosion to <0.1 by mid-run = interference REFUTED. Secondary: rise_score off the 0.1 floor, harness rise >=4/6 det valid_plant. No lower-retention gate: this arm deliberately drops lower (forensic probe, not a candidate).

