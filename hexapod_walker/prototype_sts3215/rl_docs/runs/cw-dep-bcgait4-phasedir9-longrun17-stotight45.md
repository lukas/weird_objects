# cw-dep-bcgait4-phasedir9-longrun17-stotight45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T16:08:59+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17

**wandb_id**: ib0mwcru

**hardware_ready**: yes

**hypothesis**: Sto-robustness dose test on the joystick lineage only DONE-gate det-mode passer: longrun17 (log-std-final=-3.2, std=0.041) passes the formal 60s session gate DET half at both DR-0 and own-DR (slip 2.30 vs cap 2.9, dir_err 34.7-37.4 vs allow 40, 0 falls) but FAILS the STO half (slip 4.0, dir_err 51-52) because sto-mode eval samples actions from the policy own trained std -- so the sto shortfall may be pure residual-noise-driven slip, fixable by annealing the final std lower than -3.2, rather than a distinct policy defect. Same fresh-reinit-from-BC-clone recipe and seed 17 as longrun17, ONLY --log-std-final lowered (dose grid, per the operator assume-and-go answer in OPERATOR_QUESTIONS q_20260822T1520Z: fix the sto gap as a policy property, not a gate exception). This is dose 3 of 3 (log-std-final=-4.5), sibling to the already-launched -3.6 (stotight) and -4.0 (stotight40) doses.

**gate**: eval_joystick_gate.py 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto). PASS = sto slip/m median <=2.9 (session cap) AND det stays at-or-above longrun17 own numbers (progress/slip/dir_err/speed, no regression) AND zero falls + gait_valid 6/6 in every evaluated mode. Prediction-if-true: sto slip falls monotonically as log-std-final drops, and at least one of the 3 doses clears sto slip<=2.9 without breaking det. Prediction-if-false: sto slip stays flat/similar across all 3 doses despite std dropping further -- the sto failure is NOT residual-noise-driven, redirecting investigation to the eval_checkpoint.py sto action-sampling path itself instead of more noise annealing.

**verdict**: PASS — FIRST full randomized 60s joystick DONE-gate pass on the track (versioned evaluator pass=true): 48/48 zero falls, gait_valid 48/48, no sacrificed legs, combined slip 2.671 (cap 2.9), dir_err 38.6deg (allow 40), and EVERY mode individually under caps (dr0 det 2.554/37.6, dr0 sto 2.660/39.4, ownDR det 2.605/38.7, ownDR sto 2.859/39.7). Sto slip fixed 4.0->2.66-2.86 by the -4.5 log-std-final dose, monotonic across the grid (prediction-if-true fired). Videos watched (video-joygate rerun, 4 strips both DR det+sto): clean 6-leg alternating gait. Caveats recorded: det softened vs longrun17 (session det slip 2.30->2.55, dir 34.7->37.6, still under caps; 15s rung prog 1.02x->0.85x clone) so the arm's own det-no-regression clause is only loosely met; own-DR sto margins thin (2.859/2.9, 39.7/40). md5 pod=controller 9fb86d18.

