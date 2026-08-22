# cw-dep-bcgait4-phasedir9-longrun17-stotight40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T16:05:19+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17

**hypothesis**: Sto-robustness dose test on the joystick lineage only DONE-gate det-mode passer: longrun17 (log-std-final=-3.2, std=0.041) passes the formal 60s session gate DET half at both DR-0 and own-DR (slip 2.30 vs cap 2.9, dir_err 34.7-37.4 vs allow 40, 0 falls) but FAILS the STO half (slip 4.0, dir_err 51-52) because sto-mode eval samples actions from the policy own trained std -- so the sto shortfall may be pure residual-noise-driven slip, fixable by annealing the final std lower than -3.2, rather than a distinct policy defect. Same fresh-reinit-from-BC-clone recipe and seed 17 as longrun17, ONLY --log-std-final lowered (dose grid, per the operator assume-and-go answer in OPERATOR_QUESTIONS q_20260822T1520Z: fix the sto gap as a policy property, not a gate exception). This is dose 2 of 3 (log-std-final=-4.0), sibling to the already-launched -3.6 dose (cw-dep-bcgait4-phasedir9-longrun17-stotight).

**gate**: eval_joystick_gate.py 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto). PASS = sto slip/m median <=2.9 (session cap) AND det stays at-or-above longrun17 own numbers (progress/slip/dir_err/speed, no regression) AND zero falls + gait_valid 6/6 in every evaluated mode. Prediction-if-true: sto slip falls monotonically as log-std-final drops, and at least one of the 3 doses clears sto slip<=2.9 without breaking det. Prediction-if-false: sto slip stays flat/similar across all 3 doses despite std dropping further -- the sto failure is NOT residual-noise-driven, redirecting investigation to the eval_checkpoint.py sto action-sampling path itself instead of more noise annealing.

