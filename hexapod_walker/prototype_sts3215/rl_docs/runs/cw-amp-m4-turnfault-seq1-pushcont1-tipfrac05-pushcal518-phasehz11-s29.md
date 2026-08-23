# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11-s29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T21:14:38+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11

**wandb_id**: sdusalws

**hypothesis**: Plain English: re-run the exact recipe that just gave the first full M5 pass with a different random seed, to check the win is the recipe and not luck. cw-...-phasehz11 (goal.walk_phase_hz=1.1, single lever on the phasehz05 recipe) passed every amp-m5-v1 section at seed 7; this seed-29 twin changes ONLY the seed. Prediction-if-true: full m5 PASS again (tips <=0.20, prog >=0.75, slip <=3.5). Prediction-if-false: a section drops out — the sweet spot is seed-dependent and M5 needs pass-rate evidence before declaring. Strongest alternative: eval noise (bounded by the wpm24 n=28 re-read in the gate).

**gate**: eval_amp_m5 suite (own-cfg), walk judged on --walk-per-mode 24 re-read. PASS: yaw both tips <=0.20 with 0 falls AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 or prog <0.6. Read jointly with seeds 7 (original PASS)/17/23 as a 4-seed pass-rate: >=3/4 PASS = recipe-level sweet spot confirmed, M5 gate flips; 2/4 = fragile, continue best checkpoints; <=1/4 = seed luck, 1.1 Hz is not the answer.

**verdict**: The 1.1 Hz recipe wins again on a fresh random seed -- and this seed is the strongest M5 read in program history, sealing the >=3/4 replication gate: AMP milestone M5 (MuJoCo cross-engine transfer) is GREEN. Evidence (own-cfg eval_amp_m5, walk at wpm24 n_translating=28): walk det_prog_med 1.058 / det_slip_med 2.883 (bars 0.75/3.5; beats seed-7's 0.893/3.131), yaw tips 0.1109/0.1421 (bar 0.20), push 12/12 gait-valid 0 terms, fault 11/12 (one episode sacrificed leg 1, bar >=10/12), ZERO falls/terms across all 72+ episodes; walk det strip watched -- upright, level, six legs cycling, no flag leg. Why: with seed-7 (original PASS) + s23 (PASS, this cycle) + s29, the pre-registered 4-seed pass-rate gate reads >=3/4 whatever the concurrent s17 read returns -- the cadence sweet spot is recipe-level, not seed luck. s29 is the new M5 champion checkpoint (best on every gate axis; champions append-only). Next: amp track M5 DECLARED; M6 (hardware) is operator-owned -- track drops to WAITING-ON [operator]; fleet concentrates on walkcurr/cpg. hardware-ready: sim-gate yes; physical deployment untested (M6).

