# cw-gait-dragstance1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T19:44:05+00:00

**pod**: hexapod-mjx-train-9

**steps**: 2000000

**parent**: cw-omni-trans1

**wandb_id**: qv1k9f1j

**hardware_ready**: False

**hypothesis**: Charge-magnitude audit (probe_drag_audit.py 08-11): the per-tick k_drag_loaded form is structurally blind to skating - the 0.5mm deadband leaves 53-97% of learned-skater slip free and per-tick slip medians overlap the honest gaits touchdown scuff, so NO coefficient can price the skate without starving the step. Per-STANCE accumulated loaded travel separates them 3.3x (skater median 9.8mm vs gait 2.9mm). From scratch on the trans1 stack with the structural stance-slip charge at the audit operating point (k=8000 per m over a 6mm per-stance allowance, 0.25mm/tick jitter floor: skate pays ~2.5x its income, honest gait ~23%, motionless foot ~0; launch-gate bank test_drag_stance_stack_prices_skating_below_stepping PASSES), the paddle basin should finally be unprofitable at discovery time. If true: first from-scratch walker that lifts its feet (slip/m below the 1.1-1.5 paddle band, visible swing clearance). If false (parks, or paddles while paying): drag pricing is exonerated for good and the gait lever moves to RSI-for-walk.

**gate**: 2M discovery mechanism gate: PASS if env/reward_drag_stance is ACTIVE early (nonzero, proving the charge engages) and its share of |return| trends down while walk income climbs, with eval det fwd prog > 0.3 and slip/m < 1.0 (below the closed paddle band). FAIL-FAST if walk income floors at the park level, or slip/m stays in/above the paddle band while the drag charge plateaus (paying-not-fixing, the anchortol/loadslip failure class).

**verdict**: STOP - known exploit: FREEZE/PARK. The audit-tuned structural per-stance drag charge (k=8000/m, the exact pre-registered operating point) engaged from step 19 (env/reward_drag_stance -9..-7/tick, never trending toward zero) but the policy resolved it by going nearly motionless instead of stepping: forward_dist 0.002-0.02m vs cmd_dist 0.5-0.7m (progress_ratio ~0), walk_loadslip_factor floored 0.05-0.08 by step 49 and stayed there, det+sto video (gate AND owncfg) shows an IDENTICAL static splayed pose held the whole 15s episode, zero translation, in all 24 clips. This is the runs own pre-registered false-branch (parks, or paddles while paying) to the letter. Structural stance-slip charge alone is exonerated as a fix for the paddle at this magnitude; next lever is RSI-for-walk or combining the charge with income-shaping, not another k rung.

