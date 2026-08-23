# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11-s23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T21:10:39+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11

**hypothesis**: Plain English: re-run the exact recipe that just gave the first full M5 pass with a different random seed, to check the win is the recipe and not luck. cw-...-phasehz11 (goal.walk_phase_hz=1.1, single lever on the phasehz05 recipe) passed every amp-m5-v1 section at seed 7; this seed-23 twin changes ONLY the seed. Prediction-if-true: full m5 PASS again (tips <=0.20, prog >=0.75, slip <=3.5). Prediction-if-false: a section drops out — the sweet spot is seed-dependent and M5 needs pass-rate evidence before declaring. Strongest alternative: eval noise (bounded by the wpm24 n=28 re-read in the gate).

**gate**: eval_amp_m5 suite (own-cfg), walk judged on --walk-per-mode 24 re-read. PASS: yaw both tips <=0.20 with 0 falls AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 or prog <0.6. Read jointly with seeds 7 (original PASS)/17/23 as a 4-seed pass-rate: >=3/4 PASS = recipe-level sweet spot confirmed, M5 gate flips; 2/4 = fragile, continue best checkpoints; <=1/4 = seed luck, 1.1 Hz is not the answer.

