# cw-walk-wander60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:12:41+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip

**wandb_id**: f2f2dikd

**hypothesis**: Driving endurance ladder rung 3: 30s->60s horizon (~12 command changes/ep), one variable off wander30 (PASS at 30s). Plain: prove drive-it-around competence does not decay over minute-long drives — the operator's joystick sessions are minutes, not seconds. Prediction-if-true: 60s eps hold prog_ratio median 0.85-1.15, gv, 0 term, no late-episode height sag or slip growth (first-half vs second-half slip/m within noise). Prediction-if-false: degradation accumulates with change count (parked segments after later changes, slip/m rising ep-half over ep-half) -> transition robustness is horizon-limited and the wander line needs recovery-focused training, not longer horizons. Strongest alternative: passes but change-segment slip stays ~2x straight-line (contact-pricing root, unaffected by horizon).

**gate**: own-cfg DR0 60s 6+6: gv 12/12, 0 term, prog_ratio median 0.85-1.15, no ep prog<0.5, second-half slip/m not worse than first-half beyond noise and worst-ep slip/m <=2.1; frames watched det

