# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05-cont2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T21:26:30+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05-cont1

**wandb_id**: 3nwhpjdb

**hypothesis**: Plain English: the slow-metronome (0.5 Hz) walker keeps getting better the longer it trains -- give it 4M more steps to see if walking progress reaches the M5 bar, since nothing has plateaued yet. cont1 (+4M from phasehz05) grew walk det_prog_med 0.52 -> 0.669 (wpm24, n=28) while IMPROVING yaw tips to 0.1146/0.1048 (family best; the erosion alternative is refuted at one dose) and cutting slip to a lineage-best 2.31, with reward still rising at cutoff. Trajectory 0.52@2M -> 0.669@6M needs +0.081 more for the 0.75 M5 walk bar; 08-21 ruling: rising reward + rising gate metric = continue. Single lever: budget (+4M from cont1 checkpoint, byte-identical cfg). If-true: det_prog_med >= 0.75 with tips held <= 0.20 and slip <= 3.5 -- the 0.5 Hz recipe becomes the M5-candidate, superseding the cadence-compromise question. If-false (prog lands 0.67-0.72, gain per +4M collapsing): budget scaling saturates below the bar -- the operating point moves to the phasehz09/11 dose points or command-conditional clock code work (slow clock only under turn commands; semantics-bank first). Strongest alternative: tips erode at 10M where they did not at 6M -- tips re-gated, never assumed.

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read (n_translating>=6 required). PASS: both tips <=0.20 HELD AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: prog improves >=0.05 over the 0.669 baseline without reaching 0.75, tips held. FAIL: prog <0.72 (gain per +4M collapsed from 0.149 to <0.05 -- budget route refuted, cadence interior/conditional-clock is the route) OR tips regress >0.20 (erosion at 10M -- cont1 checkpoint stands, continuation lineage closed).

