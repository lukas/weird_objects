# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-swinganneal45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T14:16:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-swing1

**wandb_id**: dj2d99xh

**hypothesis**: Plain English: composition test — swing income (k_walk_swing=1.0) compressed the worst drag tail (p90 40->34, p95 49->39) but WORSENED the typical stance (median 11.5->13.5mm) and cost fault-sto gait validity (9/12), while the std anneal (sibling arm stdanneal45) targets the typical stance via the train-noise floor. This arm runs BOTH (swing1 recipe + --log-std-final=-4.5) to answer: (a) do the two effects compose (tail from swing + median from anneal), and (b) does the fault-sto regression follow swing or vanish when noise anneals (if the sacrificed-leg episodes were noise-driven under-cycling, anneal fixes them). Prediction-if-true: probe median <=9mm AND tail p90 <=34 held, m5 walk det slip <=3.5, fault sto gait_valid back >=10. Prediction-if-false: median stays >=10.5 (anneal refuted, see sibling) or fault sto gv <=9 persists (regression is swing-intrinsic — swing lever stays closed even in composition). Strongest alternative: anneal alone (sibling) does everything and swing only adds the fault regression — then swing is retired for good. Pre-registered live cheat unchanged from swing1: single-leg-farm = FAIL regardless of return.

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps) + m5 fault section watched. PASS = 0/12 raw falls AND walk det slip med <=3.5 AND walk gait_valid 12/12 AND tips within 0.25 band AND probe stance median <=9mm AND fault gait_valid >=10. PARTIAL = probe median drops >=2.5mm vs 11.49 but slip misses 3.5, or slip passes with fault gv 9 (mechanism composes, fault cost decides vs sibling). FAIL = median unmoved (>=10.5) or single-leg-farm or fault gv <=8 — composition refuted; sibling stdanneal45 alone carries the axis.

