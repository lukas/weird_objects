# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:16:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: wn17rc2x

**hypothesis**: Plain English: give the slow-metronome walker more training time to see if it learns BIGGER strides and recovers full walking speed while keeping its new turning skill. phasehz05 ended its 2M discovery budget with reward still climbing (quarterly means 43.5->108.8->185.9->216.5) and walk det_prog_med at only 0.52 (family ~0.94, m5 bar 0.75); the CPG controller proves 0.08 m/s at 0.5 Hz is physically reachable with 13-15 deg strides, so the shortfall is plausibly training budget, not physics. Continue phasehz05 from its checkpoint +4M (08-21 ruling: bad evals + rising reward = continue). Prediction-if-true: walk det_prog_med climbs toward >=0.75 with tips staying <=0.20 and slip <=3.5 -- no cadence compromise needed, 0.5 Hz becomes the M5-candidate recipe. Prediction-if-false: prog plateaus ~0.5 (stride amplitude capped by something other than budget -- delta-clamp interplay at 0.5 Hz or the demos' own stride ceiling); the cadence dose-sweep siblings then decide the operating point. Strongest alternative: longer training ERODES the tip win (basin drift under continued PPO) -- tips are re-gated, never assumed.

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: both tips <=0.20 HELD AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: prog improves >=0.10 over the 0.52 baseline without reaching 0.75, tips held <=0.20. FAIL: prog plateaus (<0.62) with tips held (budget refuted -- interior cadence or conditional-clock code work is the route) OR tips regress >0.20 (continuation erodes the win -- checkpoint stands, continuation lineage closed).

