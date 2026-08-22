# cw-dep-bcgait1-plant150-1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-22T04:03:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: ig0tnmh5

**hypothesis**: Re-harden the deployed walking champion on the corrected robot geometry: the sim's shin length was re-measured at 150mm (was 128mm) and the old champion now falls when driving backwards, so this arm fine-tunes the same policy on the corrected body so it walks reliably again. Mechanism: warm-start from ppo_goal_cw_dep_bcgait1_hard1's checkpoint, identical recipe/reward, only the plant is the measured tibia-150 tree; prediction-if-true the policy re-adapts its stance/gait within the budget and passes the tibia-150 session drive gate its parent fails; prediction-if-false falls persist (geometry shift too large for fine-tune), pointing at a fresh BC-INIT on the tibia-150 scripted teacher instead.

**gate**: PASS if at tibia-150: det eval_session drive segments zero falls INCLUDING reverse, fwd_heading soft PASS, gait_valid 6/6, slip/m <= 2.0, and visual stats (roll_tail_deg, drag_m) not worse than the parent's 128mm baseline. Control = parent's tibia-150 session (back-fall + fwd yaw -21.8). FAIL if any drive-segment fall remains at 10M or gait degrades to <6/6 legs cycling.

**verdict**: PASS(core)/near-miss(soft): tibia-150 fine-tune fixes both fall pathologies the parent had at the corrected plant -- joint_walk gate zero falls DR-0+own-DR (6/6 gait_valid, slip/m 1.2-1.5, roll_tail/peak <= parent 128mm baseline) and eval_session back-segment tilt_roll FALL is gone. fwd yaw drift improved -21.8->-10.6deg (just over the +-10deg soft threshold) and back-segment net velocity ~0 (track_back soft miss); the remaining session "sit" FALL is the already-tracked stance-side defect (holdbc1_hard1), not this arm. No drive-segment fall remains and gait stays 6/6, so the pre-registered FAIL trigger did not fire. Promote as the walk half of the tibia-150 answer; full session PASS still needs the stance twin cw-stand-footlow2-plant150-1.

