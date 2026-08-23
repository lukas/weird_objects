# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz09

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T20:07:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: k42qdh3f

**hypothesis**: Plain English: middle point of the metronome-speed sweep -- test whether 0.9 Hz keeps the newly-won turning skill while walking near full speed. See phasehz07 arm for the full dose-curve rationale (cadence proven the binding constraint on tips/slip at 0.5 Hz, cost = walk prog halved). This arm: goal.walk_phase_hz=0.9, single lever on the unmutated phasehz05 recipe. Prediction-if-true: tips <=0.20 with det_prog_med >=0.75, slip <=3.5. Prediction-if-false: tips revert at this cadence -- the feasible region, if any, is below 0.9 Hz. Strongest alternative: budget-limited prog (the -cont1 sibling tests it).

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: yaw PASS (both tips <=0.20, 0 falls) AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 held but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 either side or prog <0.6 with tips degraded. Joint 5-point dose-curve read with siblings.

**verdict**: Middle of the metronome sweep: 0.9 Hz keeps the turning win but walking stays too slow and slippery — the dose curve's trough, not its answer. Evidence (m5 suite own-cfg; walk on the pre-registered wpm24 re-read, n_translating=28): yaw tips 0.1313/0.1801 (bar 0.20, parent 0.2157/0.2351 — clean tip win), but walk det_prog_med 0.642 (bar 0.75; PARTIAL band 0.6-0.75), det_slip_med 4.739 (bar 3.5), 1 det term, gait_valid 48/48, push/fault sections pass; strips watched: upright/level, six legs cycling, deficit is quantitative (short strides, sliding) not postural. Why: joint 5-point curve read — 0.7 and 0.9 Hz are the worst-of-both region (prog 0.64-0.67, slip 4.5-4.7) while 1.1 Hz passes everything (sibling phasehz11 = first full m5 PASS); mid-cadence appears to break the stride/slip rhythm the policy can exploit at either 0.5 (grip) or 1.1 (speed). Prediction-if-false branch ('feasible region below 0.9') is REFUTED — the feasible point is ABOVE 0.9. Next: no follow-up on this dose; the lineage's next question is seed-robustness of the 1.1 Hz PASS (batch launched off phasehz11).

