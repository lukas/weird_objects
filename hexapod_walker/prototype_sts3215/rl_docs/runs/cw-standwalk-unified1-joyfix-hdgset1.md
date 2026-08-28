# cw-standwalk-unified1-joyfix-hdgset1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T18:52:27+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-unified1-joyfix-hdg-c1

**wandb_id**: cb0gytvl

**hypothesis**: Plain English: hdg-c1's single-jump full-circle heading canary FAILED both its own health bars — training reward crashed hard (Q1 +30 -> Q3 -300, still -115 at 2M) and lateral response stayed flat (probe_cmd_sensitivity max(left,right) speed_ratio 0.006, WORSE than the pin-forward parent's own 0.033, vs the 0.07 target) — exactly hdg-c1's own predicted-if-false branch. Per the joystick track's own prior staged-curriculum ruling (fb_20260822T032514) and its previously-used 'rung B' set, this canary swaps the full circle (goal.walk_heading_max_rad=pi) for the much smaller discrete heading SET goal.walk_heading_set=[0,+45deg,-45deg] (fwd + both diagonals only, never lateral/reverse), same warm start (long-s0), same everything else. Predict-if-true: at 2M, reward stays within a normal band of the long-s0 continuation trend (no >150pt sustained fall like hdg-c1's), DR-0 det walk gait_valid stays >=5/6, and the +-45deg diagonal response in probe_cmd_sensitivity (now extended with diag_fl/diag_fr response rows, see snapshot exp/cw-standwalk-unified1-joyfix-hdgset1) shows real off-axis displacement (diagonal along-command speed_ratio meaningfully above hdg-c1's cardinal-lateral 0.006, target >=0.07) while fwd stays >=0.35. Predict-if-false: even this much smaller staged set still destabilizes training or fails to move diagonal tracking — would point at the velocity-blind goal.walk_obs_body_vel=2 ref-copy observation (not heading-draw width) as the real blocker, which the concurrent velobs3 arm directly tests.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only, no behavior maturity claim): at 2M — training reward not collapsing (no sustained fall vs the long-s0 continuation trend at matched steps analogous to hdg-c1's own -300 crash), DR-0 det walk gait_valid stays >=5/6 with no new sacrificed leg, session eval terminations not exploding (>2x parent's 0/90 baseline band = any >6/90 fails), AND probe_cmd_sensitivity response matrix (extended diag_fl/diag_fr rows) on the 2M ckpt: max(diag_fl,diag_fr) speed_ratio >=0.07 (vs hdg-c1's cardinal-lateral 0.006) with fwd >=0.35. PASS -> 2-seed acquisition. FAIL with healthy training -> escalate to the walk_obs_body_vel velocity-blind-obs hypothesis (coordinate with the velobs3 arm's read) instead of a further heading-width lever.

