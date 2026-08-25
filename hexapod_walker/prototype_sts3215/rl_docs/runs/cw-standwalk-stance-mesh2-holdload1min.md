# cw-standwalk-stance-mesh2-holdload1min

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T08:01:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdonly1

**wandb_id**: n591ux2g

**hypothesis**: Can the robot learn to stand quietly on ALL SIX feet if hovering feet stop earning? holdonly1 proved balance is learnable on mesh/100Hz but converged to a hot three-foot stilt because the hold income's clearance proxy prices 15-20mm hovers as down. This arm gates hold income on MEASURED per-foot load, min-over-feet variant (reward.hold_feet_load=1 + hold_feet_load_min=1; the product form was historically defeated by one-foot shedding). Probe: honest hold unchanged 1471.6/ep, stilt +511 -> -94.5. Prediction-if-true: six-foot quiet hold emerges within 6M (valid_plant, zero OC, cur_p95<=1.0A). Prediction-if-false: policy falls/thrashes instead of planting (load gate removed the stilt shelf with no reachable path to six-foot) -- next pre-registered lever is a DR ramp (0->0.2) or hold_load_ref_n slope retune. Strongest alternative: a new under-threshold cheat (partial-load feet riding the s_i ramp) -- watch duty + hold_load_factor.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear<5mm on all legs) AND cur_p95<=1.0A. FAIL: still stilting/shedding or OC -- pre-register the DR-ramp or ref_n-slope arm before launching it.

