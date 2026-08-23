# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11-s17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T21:06:55+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11

**wandb_id**: 01i9xb6g

**hypothesis**: Plain English: re-run the exact recipe that just gave the first full M5 pass with a different random seed, to check the win is the recipe and not luck. cw-...-phasehz11 (goal.walk_phase_hz=1.1, single lever on the phasehz05 recipe) passed every amp-m5-v1 section at seed 7; this seed-17 twin changes ONLY the seed. Prediction-if-true: full m5 PASS again (tips <=0.20, prog >=0.75, slip <=3.5). Prediction-if-false: a section drops out — the sweet spot is seed-dependent and M5 needs pass-rate evidence before declaring. Strongest alternative: eval noise (bounded by the wpm24 n=28 re-read in the gate).

**gate**: eval_amp_m5 suite (own-cfg), walk judged on --walk-per-mode 24 re-read. PASS: yaw both tips <=0.20 with 0 falls AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 or prog <0.6. Read jointly with seeds 7 (original PASS)/17/23 as a 4-seed pass-rate: >=3/4 PASS = recipe-level sweet spot confirmed, M5 gate flips; 2/4 = fragile, continue best checkpoints; <=1/4 = seed luck, 1.1 Hz is not the answer.

**verdict**: The 1.1 Hz phase-clock recipe reproduces its full M5 pass on an independent training seed -- seed-luck is now 2/4 refuted with two to go. m5_verdict (amp-m5-v1, wpm24): walk det_prog_med 0.983 / det_slip_med 2.661 (translating n=28) / gait 48/48 / 0 terms -- BETTER than the seed-7 base (0.893/3.131 wpm24) and the best prog yet measured at this cadence; yaw tips 0.1141/0.1655 vs base 0.1451/0.1453, 0 falls, comfortably under the 0.20 M5 bar; push 12/12 gv 0 terms; fault 11/12 gv (injected limp leg held out, other five cycling upright on video) det_fwd 0.457. Walk contact sheet clean: level body, six-leg cycling, no flag leg, no crouch. Reward healthy (quarters 44->258, still rising at 2M -- margin available if a later joint read wants continuations). Caveat: the noisy off-distribution DR-0 quick gate looked bad (det slip med 3.95, one sto sacrificed leg) but the base seed shows the same artifact (slip 7.08 there vs 3.13 on m5) -- the m5 suite is the registered instrument. Joint pass-rate stands at 2/4 PASS (seed7 base + s17); s23/s29 still training decide whether the >=3/4 sweet-spot branch fires and the M5 gate flips. Evidence: logs/ckpt_eval/..._phasehz11_s17_m5/.

