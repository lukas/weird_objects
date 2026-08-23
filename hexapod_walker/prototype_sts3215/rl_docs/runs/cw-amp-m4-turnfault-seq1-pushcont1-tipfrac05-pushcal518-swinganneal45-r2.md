# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-swinganneal45-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T14:54:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-swinganneal45

**wandb_id**: dmx08azb

**hypothesis**: Plain English: RE-RUN of the anneal+swing composition arm — the original swinganneal45 was INVALID (never trained, byte-identical frozen twin of stdanneal45; trainer fixed at tag exp/logstd-anneal-rollout-start-fix, smoke v2b PASS). Unchanged question: swing income (k_walk_swing=1.0) compressed the worst drag tail (p90 40->34, p95 49->39) but WORSENED the typical stance (median 11.5->13.5mm) and cost fault-sto gait validity (9/12), while the std anneal (sibling stdanneal45-r2) targets the typical stance via the train-noise floor. This arm runs BOTH to answer: (a) do the two effects compose (tail from swing + median from anneal), and (b) does the fault-sto regression follow swing or vanish when noise anneals. Prediction-if-true: probe median <=9mm AND tail p90 <=34 held, m5 walk det slip <=3.5, fault sto gait_valid back >=10. Prediction-if-false: median stays >=10.5 (anneal refuted, see sibling) or fault sto gv <=9 persists (regression is swing-intrinsic). Strongest alternative: anneal alone does everything and swing only adds the fault regression — swing retired for good. Pre-registered live cheat unchanged: single-leg-farm = FAIL regardless of return.

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps) + m5 fault section watched + WEIGHT-MOVEMENT precheck (non-log_std tensors must differ from ancestor turnfault_seq1 before any behavioral verdict). PASS = 0/12 raw falls AND walk det slip med <=3.5 AND walk gait_valid 12/12 AND tips within 0.25 band AND probe stance median <=9mm AND fault gait_valid >=10. PARTIAL = probe median drops >=2.5mm vs 11.49 but slip misses 3.5, or slip passes with fault gv 9. FAIL = median unmoved (>=10.5) or single-leg-farm or fault gv <=8 — composition refuted; sibling stdanneal45-r2 alone carries the axis.

**verdict**: Adding the swing-cycle bonus on top of the noise anneal does NOT help — the composition is worse than the anneal alone on the mechanism probe and adds back swing1's known costs, so it is NOT adopted. Probe (matched conditions): stance travel median 10.43mm vs parent control 14.03 (-3.6mm, clears the PARTIAL threshold) but WORSE than sibling anneal-alone's 9.55, and the tail barely moves (p90 33.3 vs parent 35.2; anneal-alone 25.8). m5: walk det slip 3.515 — family-best, but misses the 3.5 bar by 0.015 (parent 3.67); tip_left_err 0.2512 regresses just past the 0.25 band (parent 0.2157); fault gait_valid 11/12 with one sto episode sacrificing leg [5] — swing1's fault regression follows the swing term at reduced strength (9/12 alone, 11/12 composed with anneal). 0 falls everywhere, walk gait_valid 12/12, walk video clean. WEIGHT-MOVEMENT precheck PASS (12/12 non-log_std tensors moved vs both ancestors; log_std -4.46) — this arm really trained, and its checkup rc=1 was a benign wandb-finalization race (resolution in ledger). Composition question ANSWERED: the anneal is the active ingredient; swing income redistributes drag and taxes yaw+fault. Anneal-alone carries the axis — the dose rung (-5.0) goes to the single-lever arm only, no composed dose arm.

