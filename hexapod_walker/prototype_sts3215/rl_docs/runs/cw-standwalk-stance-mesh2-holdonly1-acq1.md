# cw-standwalk-stance-mesh2-holdonly1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T07:18:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdonly1

**hypothesis**: Plain English: hold-only at 2M taught the mesh robot to balance without falling, but it stands on THREE legs with the other three held in the air, riding hot currents just under the priced 2.0A threshold -- and the bank-probed reward already pays honest six-foot quiet standing ~3x more (1472/ep vs the 504 this policy earns), so the tripod is a LOCAL basin, not the reward optimum. This arm continues the holdonly1 checkpoint +8M (10M total, same recipe/pricing, hold=1.0 diet) to test whether budget alone anneals tripod -> six-foot plant, per the 08-21 continuation ruling (eval survival improved 0.5@1M -> 6/6 det@2M while return declined only via accumulating hot charges). Prediction-if-true: sto over_current trips (priced by term_cost) plus hot charges push feet down; det+sto hold survival rises, cur_p95 drops toward the honest 0.15-0.41A band, return climbs toward the quiet-hold bank band ~1472. Prediction-if-false: at 10M total it still stilts (3 feet aloft, p95 ~1.8A, sto OC trips) -- budget is not the lever, and the next arm is a bank-checked reward.hold_feet_load income gate. Strongest alternative: the tripod basin is entropy-locked (log-std collapsed) and neither budget nor feet pricing helps without an exploration change.

**gate**: Acquisition read at 10M total: pod_eval hold panel DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear <5mm on all legs) AND cur_p95 <= 1.0A. FAIL: still tripod-stilt or persistent sto over_current -- budget is not the lever; escalate to the pre-registered bank-checked reward.hold_feet_load arm.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

