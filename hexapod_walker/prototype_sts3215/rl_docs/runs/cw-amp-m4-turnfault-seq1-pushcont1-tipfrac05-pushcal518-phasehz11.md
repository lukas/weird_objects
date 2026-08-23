# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:11:02+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: voskjicp

**hypothesis**: Plain English: high end of the metronome-speed sweep -- test whether even a small slowdown from the original 1.33 Hz clock (to 1.1 Hz) buys turning accuracy without losing walking speed. See phasehz07 arm for the full dose-curve rationale. This arm: goal.walk_phase_hz=1.1, single lever on the unmutated phasehz05 recipe. Prediction-if-true: prog stays >=0.75 (close to the 1.33 Hz family's ~0.94) and tips improve at least partially (<=0.22). Prediction-if-false: tips stay at the parent's saturated 0.22-0.24 -- the cadence effect needs deeper slowing, feasibility decided by the 0.7/0.9 siblings. Strongest alternative: budget-limited prog (the -cont1 sibling tests it).

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: yaw PASS (both tips <=0.20, 0 falls) AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 held but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 either side or prog <0.6 with tips degraded. Joint 5-point dose-curve read with siblings.

