# cw-uni-mix40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T00:38:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-uni-blend1-r2

**hypothesis**: Unified-joystick mix ladder rung after blend1-r2 FAIL (rise/lower success flat 0 for 18M at walk=0.7/rise=0.1/lower=0.1 — walk's 70% share + ~600-700/ep income dominated the shared actor's gradient; rise-finish income actively traded away). ONE variable vs blend1-r2: goal-mix rebalanced to walk=0.4/hold=0.1/rise=0.25/lower=0.25 off the SAME driving-champion warm start on the fixed sim. If-true: rise/lower success fracs climb off zero within the first ~5M steps and reach >=5/6 det while the warm-started walk (already converged) retains the joystick gate. If-false: fracs stay pinned at 0 -> sample share is NOT the binding constraint; the pricing is (not-lowering earns positive return, rise attempts net negative) -> next move is a rise/lower income scale audit, not more mix ladder. Strongest alternative: rise from crouch is too hard to discover from a walk prior at any mix share without curriculum/shaping.

**gate**: rise/lower success >=5/6 det each AND JOYSTICK GATE (eval_drive DR0.2, own cfg) zero in-envelope falls AND quiet hold AND VIDEO: no leg-through-floor in rise/lower; frames watched det

