# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T20:16:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: wn17rc2x

**hypothesis**: Plain English: give the slow-metronome walker more training time to see if it learns BIGGER strides and recovers full walking speed while keeping its new turning skill. phasehz05 ended its 2M discovery budget with reward still climbing (quarterly means 43.5->108.8->185.9->216.5) and walk det_prog_med at only 0.52 (family ~0.94, m5 bar 0.75); the CPG controller proves 0.08 m/s at 0.5 Hz is physically reachable with 13-15 deg strides, so the shortfall is plausibly training budget, not physics. Continue phasehz05 from its checkpoint +4M (08-21 ruling: bad evals + rising reward = continue). Prediction-if-true: walk det_prog_med climbs toward >=0.75 with tips staying <=0.20 and slip <=3.5 -- no cadence compromise needed, 0.5 Hz becomes the M5-candidate recipe. Prediction-if-false: prog plateaus ~0.5 (stride amplitude capped by something other than budget -- delta-clamp interplay at 0.5 Hz or the demos' own stride ceiling); the cadence dose-sweep siblings then decide the operating point. Strongest alternative: longer training ERODES the tip win (basin drift under continued PPO) -- tips are re-gated, never assumed.

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: both tips <=0.20 HELD AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: prog improves >=0.10 over the 0.52 baseline without reaching 0.75, tips held <=0.20. FAIL: prog plateaus (<0.62) with tips held (budget refuted -- interior cadence or conditional-clock code work is the route) OR tips regress >0.20 (continuation erodes the win -- checkpoint stands, continuation lineage closed).

**verdict**: PARTIAL per the pre-registered branch -- +4M continuation grows stride/progress substantially WITHOUT eroding the 0.5Hz tip win; in fact every other section improved. m5 read (initial + wpm24 walk re-read, n_translating=28): walk det_prog_med 0.669 (baseline 0.52, +0.149 -- clears the PARTIAL bar of +0.10, misses the 0.75 PASS bar), det_slip_med 2.31 = BEST EVER on the lineage (parent 2.443, family band 3.47-3.83), 0 terms, gait 48/48, no sacrificed legs; yaw tips 0.1146/0.1048 -- IMPROVED from parent 0.149/0.156, best in family, the strongest-alternative 'continuation erodes the tip win' is refuted; push PASS (1 det term <=2, gv 12/12); fault PASS with first-ever spotless section (0 terms, gv 12/12, det_fwd_med 0.296). m5_pass=false solely on walk prog. Video: clean upright six-leg cycling, real stepping. Reward vs eval: reward still rising at the end of +4M (quarters 102.5/244.9/260.7/272.6) and prog is rising 0.52@2M -> 0.669@6M, NOT plateaued (FAIL branch <0.62 not triggered) -- textbook 08-21 continue case. Context: cont1@6M matches phasehz07's prog (0.669 vs 0.674) with far better slip (2.31 vs 4.48) and better tips. Next: cont2 +4M from this checkpoint (launched this cycle); read jointly with phasehz09/11 dose points (other cycle) for the cadence operating-point decision.

