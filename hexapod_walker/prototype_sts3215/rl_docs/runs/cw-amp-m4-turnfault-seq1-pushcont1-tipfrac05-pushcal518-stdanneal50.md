# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T15:26:21+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45-r2

**wandb_id**: dx18cpt8

**hypothesis**: Plain English: dose-response rung on the ONE mechanism measured to cut the amp family's loaded-foot drag — anneal the training action noise deeper (log-std-final -4.5 -> -5.0) on the unchanged pushcal518 recipe. Parent stdanneal45-r2 (PARTIAL) cut the matched probe stance-travel median 14.03 -> 9.55mm (-32%) but m5 walk det slip stayed 3.71 (bar 3.5, parent 3.67). Prediction-if-true: probe median drops toward the joystick champion's ~5.5mm regime (<=8mm) AND m5 walk det slip finally moves (<=3.5, or at least <=3.55). Prediction-if-false: probe median plateaus ~9.5 and/or slip unmoved — noise-floor mechanism saturates one rung in; remaining fork = demo-anchor mechanism (teacher v4 from the CPG contextual winner) vs the 3.5-bar amendment ruling (q_20260823T0700Z, probe/m5 dissociation noted). Strongest alternative: deeper anneal overfits det behavior and costs sto robustness (watch fault sto gait_valid + push sto terms).

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps) vs SAME-CYCLE matched parent control (pushcal518_ctrl median 14.03; anneal45 rung 9.55) + weight-movement precheck vs turnfault_seq1. PASS = 0/12 raw falls AND walk gait_valid 12/12 AND probe median <=8 AND m5 walk det slip med <=3.5 AND tips within 0.25 band. PARTIAL = probe median <=8 but slip in (3.5,3.72] unmoved — mechanism real, metric saturated: escalate the probe/m5 dissociation to a bar/metric amendment proposal instead of more dose rungs. FAIL = probe median >9.0 (no gain over the -4.5 rung) or any fall or fault sto gait_valid <=9 — noise-floor saturated or costing robustness; stop dosing, fork to teacher-v4 demo-anchor arm.

