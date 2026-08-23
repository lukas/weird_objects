# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T14:56:01+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45

**wandb_id**: y6k7de3u

**hypothesis**: Plain English: RE-RUN of the noise-floor arm — the original stdanneal45 was INVALID (never trained: the on_rollout_end log-std anneal inflated first-minibatch KL past target_kl's early-stop, 0 optimizer steps in 2M; fixed at tag exp/logstd-anneal-rollout-start-fix, smoke v2b PASS with 12/12 tensors moving). Unchanged question: this family's walk drags every loaded foot ~2x farther per stance than the joystick champion (11.49mm median vs 5.5mm, same command/geometry), all reward-side levers measured useless, and the one big mechanical difference is the FIXED train std 0.135 vs the champion's annealed -4.5; the joystick track measured (08-22 pd8 dig-in) that the noise floor is what makes honest stances drag. Single lever: --log-std-final=-4.5 on the unchanged pushcal518 recipe, now actually training. Prediction-if-true: slipdist probe stance median drops from 11.49 toward <=9mm and m5 walk det slip med reaches <=3.5 with falls/gait/tips unchanged. Prediction-if-false: median unmoved — noise floor is not the amp slip mechanism. Strongest alternative: slip is DEMO-ANCHORED (AMP style holds slip near teacher demos; next mechanism = cleaner motion library from the CPG winner as teacher v4).

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps) + WEIGHT-MOVEMENT precheck (non-log_std tensors must differ from ancestor turnfault_seq1 before any behavioral verdict). PASS = 0/12 raw falls AND walk det slip med <=3.5 AND walk gait_valid 12/12 AND tips within 0.25 band AND probe stance median <=9mm. PARTIAL = probe median drops >=2.5mm vs parent's 11.49 but m5 slip misses 3.5 — dose next (-5.0 rung). FAIL = probe median unmoved (>=10.5) AND slip unmoved (+-0.15 of 3.67) — noise-floor mechanism refuted for amp; remaining fork = demo-anchor mechanism (teacher v4) vs the 3.5-bar ruling amendment.

**verdict**: Lowering the training-time action noise is the FIRST mechanism to genuinely cut this family's loaded-foot drag, but the m5 slip metric did not follow — the gate's PARTIAL branch. Probe (hazard-free own-cfg, seed 0, 6 eps, ~900 stances): per-stance travel median 9.55mm vs matched same-conditions parent control 14.03mm (-32%; tail p90 25.8 vs 35.2; mean 11.5 vs 16.4) — every prior lever (pricing 6x/12x, income gate, swing income, demos, densification) left the median at/above parent. Yet m5 walk det slip med 3.71 vs parent 3.67 (bar 3.5) = unmoved within the ±0.15 band. WEIGHT-MOVEMENT precheck PASS: 12/12 non-log_std tensors differ from turnfault_seq1 AND pushcal518, log_std annealed to -4.46 — the r1 freeze is fixed and this arm really trained. Safety clean: 0 falls all sections, gait_valid 12/12 walk+push+fault, tips 0.2088/0.2287 slightly better than parent (0.2157/0.2351), walk video clean (six-leg cycling, level body, no flag leg). Caveats: (1) the gate's hard-coded parent probe median 11.49 came from a different probe invocation — the same-cycle matched parent rerun reads 14.03, so the matched control is the honest comparator (drop 4.48mm >= 2.5mm PARTIAL threshold); (2) probe-vs-m5 dissociation (stance travel -32%, slip/m flat) says the m5 walk slip metric is dominated by something other than loaded-stance drag (likely turn-in-place phases in stress_mix) — noted as an amendment to the 3.5-bar discussion q_20260823T0700Z, no bar changed. Next per gate: dose the anneal one rung to --log-std-final=-5.0 (launching -stdanneal50).

