# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz09

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:07:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: k42qdh3f

**hypothesis**: Plain English: middle point of the metronome-speed sweep -- test whether 0.9 Hz keeps the newly-won turning skill while walking near full speed. See phasehz07 arm for the full dose-curve rationale (cadence proven the binding constraint on tips/slip at 0.5 Hz, cost = walk prog halved). This arm: goal.walk_phase_hz=0.9, single lever on the unmutated phasehz05 recipe. Prediction-if-true: tips <=0.20 with det_prog_med >=0.75, slip <=3.5. Prediction-if-false: tips revert at this cadence -- the feasible region, if any, is below 0.9 Hz. Strongest alternative: budget-limited prog (the -cont1 sibling tests it).

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: yaw PASS (both tips <=0.20, 0 falls) AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 held but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 either side or prog <0.6 with tips degraded. Joint 5-point dose-curve read with siblings.

