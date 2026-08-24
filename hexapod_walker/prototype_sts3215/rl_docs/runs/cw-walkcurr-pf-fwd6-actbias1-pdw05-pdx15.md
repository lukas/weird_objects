# cw-walkcurr-pf-fwd6-actbias1-pdw05-pdx15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T01:14:58+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1-pdw05

**wandb_id**: 1930yh0f

**hypothesis**: Dose sibling of actbias1-pdw05: same action-recentering fix (joint_action_bias) + same park_duty confound fix (goal.park_duty_window_s=0.5), PLUS 1.5x the per-leg contact-duty charge (reward.k_park_duty 0.08->0.12, the same bank-legal max dose already proven on hgt2-pdw05-pdx15). actbias1-pdw05 showed the confound fix fires correctly (reward_park_duty now nonzero) but at 1x dose the policy still settles into a clean, stable park-stand (det 0/6 gait_valid, 5/6 legs sacrificed) instead of walking. This tests whether the SAME park-duty class merely needs to fire harder on top of the (now healthy, non-collapsing) action-bias base pose, or whether park-duty-as-a-class is insufficient regardless of dose once stacked with the bias fix. Prediction-if-true: walk_freeprog_score crosses toward/past 0, direction_err moves off ~88deg, det gait_valid improves on video. Prediction-if-false (same static park-stand persists): park_duty-class is now closed even combined with the bias fix -- escalate to the direct minimum-total-foot-contact charge (WITH a termination, per the stagea-slip1 absorbing-state lesson) and flag BC-kickstart as the named last resort to the operator in the same entry.

**gate**: Same rung-1 gate as actbias1-pdw05: C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video, walk_freeprog_score trends toward/past 0, clip_fraction stays healthy. Additionally env/height_err_mm should stay in the actbias1 healthy ~15-22mm band (no collapse) and env/reward_park_duty should read MORE negative than pdw05's -0.005..-0.025 band (dose actually biting harder). PASS = rung-1 lands. FAIL with the same static park-stand = park_duty-class closed at every bank-legal dose combined with the bias fix; next mechanism is the direct foot-contact charge, BC-kickstart flagged to the operator in the same verdict.

