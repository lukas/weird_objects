# cw-stand-footlow2-plant150-1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T15:40:55+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: p47apr70

**hypothesis**: Plain English: re-harden the stand-up/sit-down (stance) half of the download deployable on the corrected robot geometry, the same fix already proven for the walk half. The sim's shin (tibia) was re-measured at 150mm (was 128mm); the session gate now HARD-FAILS at tibia-150 (sit tilt_pitch fall) using the old 128mm-trained footlow2_hard1 stance checkpoint, while cw-dep-bcgait1-plant150-1 (identical warm-start-and-refit recipe applied to the WALK half) already PASSED core (0/6 falls DR-0+own-DR, gait_valid 6/6). This arm was blocked all of 08-22 on a red rise-family semantics bank (test_task_semantics.py RISE/SCORE/GETUP overrides) that has now been root-caused and closed (152 pass/1 known-red unrelated fastprof/4 skip/1 xfail) via two threshold recalibrations, not behavior changes: PLANT_SPEC's rise-height target was still the pre-tibia-150 108-114mm belly->plant window (measured settled height is now 131.94mm at the new geometry, recalibrated to 128-136/137mm) and getup_k_progress was recalibrated 60->200 so an honest partial rise's one-shot credit clears the extra regularizer cost of actually moving. Mechanism: warm-start from ppo_goal_cw_stand_footlow2_hard1's checkpoint, byte-identical recipe/reward/goal-mix (hold=0.1,rise=0.45,lower=0.45, the same rise_ref_belly2plant.npz reference -- its joint-angle IK tracking still produces a physically valid rise at the longer tibia per the bank's own settled-height measurement, no re-mint needed), only the plant is now the measured tibia-150 tree everywhere by default. Prediction-if-true: the policy re-adapts its stance/rise/lower geometry within budget and clears the tibia-150 session's sit-fall the parent checkpoint fails, with rise/hold/lower staying at-or-above the parent's own 128mm-era numbers (cold-start det rises valid_plant, six-foot hold no real park, lower >=10/12). Prediction-if-false: falls persist or a mode (rise/hold/lower) erodes -- geometry shift too large for a fine-tune at this budget, pointing at a fresh tibia-150 BC-anchor stance line instead of a warm continuation.

**gate**: PASS if at tibia-150: rl_move.sim.eval_session's sit/stand segments show zero falls (the specific defect this arm targets) with the walk half (cw-dep-bcgait1-plant150-1) already promoted; AND the stance semantics gate holds -- cold-start (flat/bridge/crouch) det rises valid_plant h_err<=5mm, det hold has no real park (all six feet duty>=0.5), det+sto lower >=10/12 valid_plant -- not worse than cw-stand-footlow2-hard1's own 128mm numbers. FAIL if the tibia-150 sit-fall persists, or any stance mode (rise/hold/lower) degrades below the parent's own gate numbers.

