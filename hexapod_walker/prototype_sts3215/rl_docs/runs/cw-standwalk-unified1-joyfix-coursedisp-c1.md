# cw-standwalk-unified1-joyfix-coursedisp-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-29T05:37:39+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: yy42stgp

**hypothesis**: Plain English: does replacing the k_walk_course EMA-of-instantaneous-velocity course-heading term (found completely inert for this lineage -- 0/5899 active ticks in a real-checkpoint repro) with a windowed net-body-position-displacement version (reward.k_walk_course_disp, built + bank-proven this cycle, test_course_disp_semantics.py 16/16 green including a real-checkpoint replay showing the disp mechanism activates on 97% of ticks vs the EMA's near-zero on the identical rollout, net-path dir_err only 6deg vs the per-tick view's 55-62deg) actually MOVE direction_err_mean_deg during TRAINING, not just read correctly in a frozen-checkpoint replay or a scripted-teacher bank? Root-cause chain: long-s1-cont1 FAIL (03:0x) -> EMA-inert root-cause (04:3x) -> floor/tau scalar fixes closed by measurement, cmdtrack-c1's raw-tick alternative closed too (dir_err flat, slip 2.5x) -> this is the pre-registered lever (b) follow-up, DIG-IN-flagged and now built. Same warm start (long-s0's 16M PASS checkpoint), k_walk_cmd_track and k_walk_course both OFF so only the new mechanism prices course. Predict-if-true: at 2M, DR-0 det walk direction_err_mean_deg improves materially (>=15deg drop) vs long-s0's own ~55-65deg band, slip/m stays in parent band, gait_valid>=5/6, terminations<=6/90. Predict-if-false: the term fires (confirmed via reward_walk_course_disp telemetry) but dir_err does not move -- would mean the learned zigzag is priced correctly only in expectation, and PPO's per-tick credit assignment on a windowed/delayed signal is too weak relative to slip/drag's per-tick charges to move behavior; or reward/terminations destabilize from the sparser signal.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only) at 2M: training reward not collapsing (no sustained fall vs long-s0's own matched-step continuation trend); DR-0 det walk gait_valid>=5/6 no new sacrificed leg; session terminations<=6/90; reward_walk_course_disp telemetry present on >=50% of walk ticks in the DR-0 det episodes (confirms the mechanism is actually live in training, not just in the frozen-checkpoint bank replay). PASS-with-delta (direction_err_mean_deg drops >=15deg vs long-s0's ~55-65deg band) -> escalate to 2-seed acquisition. PASS-no-delta (fires but dir_err flat) -> record as activation-without-behavior-change, next lever is a dose/window-size sweep or a stronger per-tick shaping term layered on top, not a mechanism kill. FAIL by reward collapse or terminations>6/90 -> mechanism closed, revert to k_walk_course EMA as the safer default.

