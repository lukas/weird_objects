# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11-s23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T21:10:39+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11

**wandb_id**: qkbzobrz

**hypothesis**: Plain English: re-run the exact recipe that just gave the first full M5 pass with a different random seed, to check the win is the recipe and not luck. cw-...-phasehz11 (goal.walk_phase_hz=1.1, single lever on the phasehz05 recipe) passed every amp-m5-v1 section at seed 7; this seed-23 twin changes ONLY the seed. Prediction-if-true: full m5 PASS again (tips <=0.20, prog >=0.75, slip <=3.5). Prediction-if-false: a section drops out — the sweet spot is seed-dependent and M5 needs pass-rate evidence before declaring. Strongest alternative: eval noise (bounded by the wpm24 n=28 re-read in the gate).

**gate**: eval_amp_m5 suite (own-cfg), walk judged on --walk-per-mode 24 re-read. PASS: yaw both tips <=0.20 with 0 falls AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 or prog <0.6. Read jointly with seeds 7 (original PASS)/17/23 as a 4-seed pass-rate: >=3/4 PASS = recipe-level sweet spot confirmed, M5 gate flips; 2/4 = fragile, continue best checkpoints; <=1/4 = seed luck, 1.1 Hz is not the answer.

**verdict**: Second independent seed confirms the 1.1 Hz recipe: full amp-m5-v1 PASS on every section, contributing the decisive third pass to the 4-seed replication gate. Evidence (own-cfg eval_amp_m5, walk wpm24 n_translating=28): walk det_prog_med 0.959 / det_slip_med 3.294 (bars 0.75/3.5), yaw tips 0.1398/0.1383 (bar 0.20), push 12/12 gait-valid, fault 12/12 with det_prog 0.74, ZERO falls/terms in every section; walk det strip watched -- upright, six legs cycling, no flag leg, no crouch. Why: with seed-7 (original) and s29 (PASS, stronger) this makes >=3/4 on the pre-registered joint gate -- 1.1 Hz is a recipe-level sweet spot, M5 flips. Note s23 is the narrowest of the three passers (slip 3.294 closest to the 3.5 bar, fault slip 6.18 above the family band though unbarred); champion goes to s29. Next: M5 declared on the track; M6 hardware operator-owned. hardware-ready: sim-gate yes; physical deployment untested (M6).

