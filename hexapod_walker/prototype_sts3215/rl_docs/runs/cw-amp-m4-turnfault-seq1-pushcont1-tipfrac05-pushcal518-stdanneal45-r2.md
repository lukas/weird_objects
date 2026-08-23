# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T14:56:01+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45

**wandb_id**: y6k7de3u

**hypothesis**: Plain English: RE-RUN of the noise-floor arm — the original stdanneal45 was INVALID (never trained: the on_rollout_end log-std anneal inflated first-minibatch KL past target_kl's early-stop, 0 optimizer steps in 2M; fixed at tag exp/logstd-anneal-rollout-start-fix, smoke v2b PASS with 12/12 tensors moving). Unchanged question: this family's walk drags every loaded foot ~2x farther per stance than the joystick champion (11.49mm median vs 5.5mm, same command/geometry), all reward-side levers measured useless, and the one big mechanical difference is the FIXED train std 0.135 vs the champion's annealed -4.5; the joystick track measured (08-22 pd8 dig-in) that the noise floor is what makes honest stances drag. Single lever: --log-std-final=-4.5 on the unchanged pushcal518 recipe, now actually training. Prediction-if-true: slipdist probe stance median drops from 11.49 toward <=9mm and m5 walk det slip med reaches <=3.5 with falls/gait/tips unchanged. Prediction-if-false: median unmoved — noise floor is not the amp slip mechanism. Strongest alternative: slip is DEMO-ANCHORED (AMP style holds slip near teacher demos; next mechanism = cleaner motion library from the CPG winner as teacher v4).

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps) + WEIGHT-MOVEMENT precheck (non-log_std tensors must differ from ancestor turnfault_seq1 before any behavioral verdict). PASS = 0/12 raw falls AND walk det slip med <=3.5 AND walk gait_valid 12/12 AND tips within 0.25 band AND probe stance median <=9mm. PARTIAL = probe median drops >=2.5mm vs parent's 11.49 but m5 slip misses 3.5 — dose next (-5.0 rung). FAIL = probe median unmoved (>=10.5) AND slip unmoved (+-0.15 of 3.67) — noise-floor mechanism refuted for amp; remaining fork = demo-anchor mechanism (teacher v4) vs the 3.5-bar ruling amendment.

