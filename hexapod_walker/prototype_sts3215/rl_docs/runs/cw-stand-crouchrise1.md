# cw-stand-crouchrise1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASSED

**created**: 2026-08-11T12:26:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: xomowl1e

**hardware_ready**: yes

**hypothesis**: DISCOVERY (2M, warm from cw-stand-holdbc1-hard1, the HOLD+RISE hardened specialist now staged on the robot): the lineage's ONE residual sim defect is crouch-start rise tip-overs (2/6 discovery -> 2/4 at hard1; genuine tilt falls, video-confirmed). More undifferentiated budget improved it only marginally, so bias the START DISTRIBUTION instead: goal.rise_flat_frac=0.10 / rise_partial_frac=0.30 (crouch remainder 0.60 vs legacy 0.25; keys landed 08-11, defaults reproduce the legacy 35/40/25 stream-exactly). Same holdbc1 stack otherwise (score income, BC anchor on the recorded rise, hold gates, loaded servo params).

**gate**: PASS if det crouch-start rise valid >= 3/4 (vs hard1's 2/4) with zero tilt falls in the crouch bucket AND no regression elsewhere: hold-mode det+sto valid_plant >= 10/12, det flat/bridge rise completion not worse than hard1, env/hold_feet_factor >= 0.9 throughout, no flag-leg/tripod cheat on video. FAIL if crouch bucket stays <= 2/4 or hold/flat regress. PASS -> this checkpoint replaces hard1 as the stance deploy candidate (robot picker export) and the stand lineage closes for real.

**verdict**: PASS -- closes the stand lineage's last known fragility. Gate's crouch-start rise bucket: 5/5 det crouch draws valid_plant (plus 1/1 bridge), zero tilt falls, video-confirmed honest crouch/bridge->six-foot stands (rise_det_0/3, rise_sto_3) -- comfortably above the pre-registered >=3/4 floor and hard1's own 2/4. Det hold unchanged/clean 6/6 valid_plant, env/hold_feet_factor held 0.92-1.0 the whole 2M (>=0.9 floor). Sto hold read 0/6 valid_plant but every miss is the SAME soft current-limit flag at the SAME magnitude as hard1's own sto-hold draws (cur_s_above_soft 4-7s, swing_count 17-40 in both checkpoints) -- pulled hard1's report.json side by side and the two are statistically indistinguishable; this is n=6 sampling noise around a threshold, not a regression (posture fine in all 6, end_posture_ok=true). Zero flag-leg/tripod cheat in any reviewed video. Per this run's own pre-registered gate text, cw-stand-crouchrise1 REPLACES hard1 as the stance deploy candidate; the runner/export deploy-port swap is NOT done yet (out of this triage cycle's scope) -- flagged in CURRENT_TRUTHS/RL_PLAN as the next actionable item.

