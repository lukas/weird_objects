# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz07

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T20:03:18+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: wc94hwf3

**hypothesis**: Plain English: find the metronome speed between the slow gait that turns well (0.5 Hz) and the fast gait that walks well (1.33 Hz) where the robot can do BOTH. phasehz05/cpglib proved cadence is the binding constraint on turn rate (tips 0.146-0.156, first yaw PASS in lineage) and slip (2.44-2.68, best ever) but 0.5 Hz halves walk progress (0.47-0.52 vs 0.94, bar 0.75). This arm: goal.walk_phase_hz=0.7 on the otherwise unmutated phasehz05 recipe (teacher_v2 demos; demo source proven second-order by the sibling pair). Prediction-if-true: an interior cadence keeps tips <=0.20 AND recovers walk det_prog_med >=0.75 with slip <=3.5. Prediction-if-false: tips revert toward 0.22+ faster than prog recovers (monotone trade, no feasible point) -- fork moves to command-conditional clock code work or the 0.5 Hz continuation. Strongest alternative: prog recovery is budget-limited, not cadence-limited (the -cont1 sibling tests that).

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: yaw PASS (both tips <=0.20, 0 falls) AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 held but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 either side (cadence benefit lost at this dose) or prog <0.6 with tips also degraded. Read jointly with 0.9/1.1 siblings + measured endpoints 0.5/1.333 as a 5-point dose curve; pick the feasible cadence or declare the trade monotone.

**verdict**: Half the trade recovered: at 0.7 Hz cadence the robot turns in place better than any policy in this lineage ever has (m5 yaw PASS, tips 0.1013/0.1363 vs 0.149/0.156 at 0.5 Hz and 0.216/0.235 at the 1.333 Hz parent — family-best left side) and walk progress climbs from 0.52 to 0.674, but that is still under the 0.75 bar and slip regressed for real (wpm24 det_slip_med 4.476 at n_translating=28, above the 3.5 bar AND the family 3.2-4.0 band — not thin-sample noise this time). Safety spotless: 0 falls, 0 terms every section, gait_valid 12/12 walk/push/fault and 48/48 on the wpm24 re-read, no sacrificed legs. Evidence: logs/ckpt_eval/..._phasehz07_m5/ and ..._m5_wpm24/. Dose-curve reading so far: tips are NON-monotone in cadence with the best point interior at 0.7 (so slow-clock turn authority does not require the full 0.5 Hz walk sacrifice), prog is monotone up (0.52 -> 0.674 -> 0.944), slip is worst at 0.7 (2.44 -> 4.48 -> 3.55). Pre-registered PARTIAL branch: tips <=0.20 held with prog 0.6-0.75. Next: the 0.9 sibling is now the pivotal read — it needs +0.08 prog while holding a 0.06-0.10 tip margin, and its slip number decides whether 0.7's 4.48 is a mid-cadence slip valley-wall or a fluke; judge 0.9/1.1 jointly with these three fixed points per the family gate.

