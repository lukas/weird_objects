# cw-gait-dragstance1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-11T19:57:49+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-gait-dragstance1

**wandb_id**: kt1h2tz8

**hardware_ready**: False

**hypothesis**: Full-budget test of the audit-derived structural stance-slip charge (k_drag_stance=8000, 6mm allowance, 0.25mm floor, k_drag_loaded=0, trans1 deployment-contract stack). The 2M discovery probe proved mechanism health (charge engages at -7/tick, fps unaffected) but 2M is below the walk-from-scratch floor: every genuine gait (step0, hist16-r7) needed 40M. If the charge correctly prices skating, PPO at full budget should discover the first from-scratch gait that lifts its feet (slip/m below the 1.1-1.5 paddle band, visible swing clearance, honest progress). If it instead converges to churn-and-pay or park, the flat charge is refuted at full budget and the pre-registered next rungs are the annealed-in charge, then RSI-for-walk.

**gate**: 40M: PASS if eval det fwd prog >= 0.5 with slip/m < 1.0 AND video shows swing clearance (feet lift, stance feet hold); env/reward_drag_stance share of |return| must shrink as walk income grows. FAIL if walk income floors at park level, or slip/m stays in/above the paddle band with the drag charge plateaued (paying-not-fixing), or the leg-sacrifice fingerprint appears (per-leg duty pinned).

**verdict**: KILLED before analysis, ~1M/40M steps in: this was a pre-registered respec of cw-gait-dragstance1 (identical recipe, k_drag_stance=8000/6mm/0.25mm floor, just 40M vs 2M budget), auto-drained the instant its parents pod freed. The parents own 2M discovery result (triaged this same cycle) already hit rs own pre-registered FAIL branch verbatim: env/reward_drag_stance active at -6..-9/tick the whole run (never trending toward zero), forward_dist ~0.00m vs 0.5-0.7m commanded, and 24/24 videos (det+sto, gate+owncfg) show an IDENTICAL frozen/splayed pose with zero leg-cycling for the full episode - a KNOWN exploit (freeze/park). Per RESEARCH_RULES a known exploit is a complete verdict with no re-run with more steps; r1 is exactly that re-run (same k, same allowance, same floor, only steps changed) and its own evidence field did not actually establish the drag-charge mechanism works (it was boilerplate about hist16-dep1/omni-mirror, unrelated to this hypothesis) - it should not have been drained as hardening phase. Killed to avoid burning a 40M-step budget re-confirming an already-refuted recipe; see GAIT.md for the decisive discovery-run evidence and the next lever (RSI-for-walk / charge+shaping combo).

