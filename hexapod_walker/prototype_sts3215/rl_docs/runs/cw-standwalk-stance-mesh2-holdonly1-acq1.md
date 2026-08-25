# cw-standwalk-stance-mesh2-holdonly1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T07:19:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdonly1

**wandb_id**: ucmjz112

**hypothesis**: Plain English: hold-only at 2M taught the mesh robot to balance without falling, but it stands on THREE legs with the other three held in the air, riding hot currents just under the priced 2.0A threshold -- and the bank-probed reward already pays honest six-foot quiet standing ~3x more (1472/ep vs the 504 this policy earns), so the tripod is a LOCAL basin, not the reward optimum. This arm continues the holdonly1 checkpoint +8M (10M total, same recipe/pricing, hold=1.0 diet) to test whether budget alone anneals tripod -> six-foot plant, per the 08-21 continuation ruling (eval survival improved 0.5@1M -> 6/6 det@2M while return declined only via accumulating hot charges). Prediction-if-true: sto over_current trips (priced by term_cost) plus hot charges push feet down; det+sto hold survival rises, cur_p95 drops toward the honest 0.15-0.41A band, return climbs toward the quiet-hold bank band ~1472. Prediction-if-false: at 10M total it still stilts (3 feet aloft, p95 ~1.8A, sto OC trips) -- budget is not the lever, and the next arm is a bank-checked reward.hold_feet_load income gate. Strongest alternative: the tripod basin is entropy-locked (log-std collapsed) and neither budget nor feet pricing helps without an exploration change.

**gate**: Acquisition read at 10M total: pod_eval hold panel DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear <5mm on all legs) AND cur_p95 <= 1.0A. FAIL: still tripod-stilt or persistent sto over_current -- budget is not the lever; escalate to the pre-registered bank-checked reward.hold_feet_load arm.

**verdict**: Budget is not the lever: 8M more steps on the balanced-but-stilting hold policy made it WORSE, not six-footed. The parent holdonly1 survived 6/6 DR-0 det hold episodes; after the +8M continuation the same panel is 0/12 DR-0 (det: all 6 tilt_pitch — video shows the planted start pitching over and splaying) and 0/12 own-DR, with training reward declining in lockstep (quarters 1.4/-94.6/-272.4/-255.5). The hot tripod-stilt basin destabilizes under continued optimization instead of migrating to six-foot stance, exactly the gate's pre-registered FAIL clause. Next: escalate to the bank-checked reward.hold_feet_load income-gate arm (probe run this cycle; min-over-feet variant preferred per the primitive-era one-foot-shedding lesson).

