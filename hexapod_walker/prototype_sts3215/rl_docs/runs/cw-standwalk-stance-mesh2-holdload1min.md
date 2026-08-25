# cw-standwalk-stance-mesh2-holdload1min

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:01:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdonly1

**wandb_id**: n591ux2g

**hypothesis**: Can the robot learn to stand quietly on ALL SIX feet if hovering feet stop earning? holdonly1 proved balance is learnable on mesh/100Hz but converged to a hot three-foot stilt because the hold income's clearance proxy prices 15-20mm hovers as down. This arm gates hold income on MEASURED per-foot load, min-over-feet variant (reward.hold_feet_load=1 + hold_feet_load_min=1; the product form was historically defeated by one-foot shedding). Probe: honest hold unchanged 1471.6/ep, stilt +511 -> -94.5. Prediction-if-true: six-foot quiet hold emerges within 6M (valid_plant, zero OC, cur_p95<=1.0A). Prediction-if-false: policy falls/thrashes instead of planting (load gate removed the stilt shelf with no reachable path to six-foot) -- next pre-registered lever is a DR ramp (0->0.2) or hold_load_ref_n slope retune. Strongest alternative: a new under-threshold cheat (partial-load feet riding the s_i ramp) -- watch duty + hold_load_factor.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear<5mm on all legs) AND cur_p95<=1.0A. FAIL: still stilting/shedding or OC -- pre-register the DR-ramp or ref_n-slope arm before launching it.

**verdict**: Result: FAIL on the pre-registered hold gate -- the load-min income gate does NOT produce a six-foot plant from scratch (seed 0). Evidence: DR-0 gate 0/12 ok (11/12 over_current, det 6/6 identical: cur_p95 2.64A pinned at ceiling, hot_servo, duty [0.55,1.0,0.19,0.43,0.31,1.0], end_clear up to 45mm = 3-4 feet aloft, valid_plant 0/12); own-DR(0.2) same 0/12 OC; training reward declined monotonically (-102/-406/-626/-691); contact sheet shows the robot rearing progressively into a near-vertical splay before tripping current. Why: with the min-over-feet gate, income is ZERO until ALL six feet carry load -- from any partial stance the gradient toward planting the remaining feet is flat, so the policy never finds the +1472/ep honest-hold optimum (bank-verified aligned) and instead fights gravity at the current ceiling. Family read: 3/3 load-min arms now failed in three DIFFERENT basins (seed0 rear-up OC lock, s1 belly-flop freeze, warm tilt collapse) -- reward aligned, exploration/gradient-path failure. What's next (pre-registered in this run's gate): the hold_load_ref_n/slope retune -- soften the per-foot load on-ramp so lightly-loaded feet earn partial credit and planting a foot has immediate gradient; launching that dose pair this cycle alongside the sibling cycle's DR-0/entropy plan.

