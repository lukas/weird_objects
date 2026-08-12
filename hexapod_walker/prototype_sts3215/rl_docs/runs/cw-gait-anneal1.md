# cw-gait-anneal1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T04:28:32+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-gait-dragstep1

**wandb_id**: gqo83h4h

**hypothesis**: Test whether the gait-cleanup anti-skate charge only fails to teach lift-and-place because it lands on a policy that has never moved before -- dragstance1 froze into a motionless park when the charge was applied from a random init. One change vs the closed dragstance1 arm: warm-start from cw-gait-dragstep1 (a genuine from-scratch, RL-only paddler -- no imitation, already translates by skating) instead of random init, and swap its own refuted per-tick k_drag_loaded=40 charge for the audit-correct structural per-stance charge (k_drag_stance=8000/6mm-allowance/0.25mm-floor) that dragstance1 used and froze under. Prediction-if-true: with an already-mobile prior to reshape instead of nothing to build from, det gait_valid shows real six-foot stepping (swing clearance visible, slip/m trending toward/below the paddle band 1.1-1.5) within the 2M discovery budget. Prediction-if-false: the charge-health metric (env/walk_loadslip_factor or equivalent) floors early exactly as in dragstance1/rsi1/slowfirst1 regardless of the mobile starting point -- warm-start-as-curriculum is refuted too, and every no-new-code form of lever 2 (annealed-up pricing) is closed; only a true in-run coefficient scheduler (CODE, unqueued) or the BC-anchor route (out of nobc's charter) remains.

**gate**: PASS if det gait_valid shows real six-foot cycling (not frozen/parked) in ANY episode with slip/m trending toward/below 1.5 AND prog_ratio meaningfully positive -- warm-start-as-curriculum helps discovery, worth hardening. FAIL if det+sto reproduce the identical near-zero-travel/floored-charge-health fingerprint from dragstance1/rsi1/slowfirst1, or the policy keeps skating and simply eats the charge with no behavior change -- closes lever 2 in every no-new-code form; remaining nobc options are a true in-run coefficient scheduler (CODE, spec first) or closing the from-scratch gait line for now.

