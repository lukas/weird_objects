# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz07

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:03:18+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

**wandb_id**: wc94hwf3

**hypothesis**: Plain English: find the metronome speed between the slow gait that turns well (0.5 Hz) and the fast gait that walks well (1.33 Hz) where the robot can do BOTH. phasehz05/cpglib proved cadence is the binding constraint on turn rate (tips 0.146-0.156, first yaw PASS in lineage) and slip (2.44-2.68, best ever) but 0.5 Hz halves walk progress (0.47-0.52 vs 0.94, bar 0.75). This arm: goal.walk_phase_hz=0.7 on the otherwise unmutated phasehz05 recipe (teacher_v2 demos; demo source proven second-order by the sibling pair). Prediction-if-true: an interior cadence keeps tips <=0.20 AND recovers walk det_prog_med >=0.75 with slip <=3.5. Prediction-if-false: tips revert toward 0.22+ faster than prog recovers (monotone trade, no feasible point) -- fork moves to command-conditional clock code work or the 0.5 Hz continuation. Strongest alternative: prog recovery is budget-limited, not cadence-limited (the -cont1 sibling tests that).

**gate**: eval_amp_m5 suite; judge walk on a --walk-per-mode 24 re-read if n_translating<6. PASS: yaw PASS (both tips <=0.20, 0 falls) AND walk det_prog_med >=0.75 AND det_slip_med <=3.5, 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: tips <=0.20 held but prog 0.6-0.75, or prog >=0.75 with one tip 0.20-0.22. FAIL: tips >0.22 either side (cadence benefit lost at this dose) or prog <0.6 with tips also degraded. Read jointly with 0.9/1.1 siblings + measured endpoints 0.5/1.333 as a 5-point dose curve; pick the feasible cadence or declare the trade monotone.

