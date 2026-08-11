# cw-stand-crouchrise1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T12:26:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: xomowl1e

**hardware_ready**: no

**hypothesis**: DISCOVERY (2M, warm from cw-stand-holdbc1-hard1, the HOLD+RISE hardened specialist now staged on the robot): the lineage's ONE residual sim defect is crouch-start rise tip-overs (2/6 discovery -> 2/4 at hard1; genuine tilt falls, video-confirmed). More undifferentiated budget improved it only marginally, so bias the START DISTRIBUTION instead: goal.rise_flat_frac=0.10 / rise_partial_frac=0.30 (crouch remainder 0.60 vs legacy 0.25; keys landed 08-11, defaults reproduce the legacy 35/40/25 stream-exactly). Same holdbc1 stack otherwise (score income, BC anchor on the recorded rise, hold gates, loaded servo params).

**gate**: PASS if det crouch-start rise valid >= 3/4 (vs hard1's 2/4) with zero tilt falls in the crouch bucket AND no regression elsewhere: hold-mode det+sto valid_plant >= 10/12, det flat/bridge rise completion not worse than hard1, env/hold_feet_factor >= 0.9 throughout, no flag-leg/tripod cheat on video. FAIL if crouch bucket stays <= 2/4 or hold/flat regress. PASS -> this checkpoint replaces hard1 as the stance deploy candidate (robot picker export) and the stand lineage closes for real.

**verdict**: CORRECTION (this ledger entry raced with a concurrent dig-in cycle's more rigorous matched-parent probe -- deferring to it): mechanism PASS, promotion DECLINED. The start-mix bias (60% crouch) genuinely fixes the crouch tip-over: RSI-off all-crouch probe vs holdbc1_hard1 under the identical seed/cfg shows child det 8/8 valid_plant / 16/16 stands / zero falls vs parent det 0/8 with 8/8 tilt_roll falls -- a real, controlled, matched-parent result (logs/ckpt_eval/{crouchrise1,hard1}_rsioff_crouch). But it MISSES the pre-registered hold no-regression bar: hold det+sto valid_plant 7/12 (need >=10/12), the sto misses all a >2.0A current-tail soft flag with a real (small) Imax increase vs the parent under the SAME probe (2.31A vs 1.96A det) -- posture/stillness/feet/height otherwise identical. Per the gate's own terms this is a FAIL on promotion: ppo_goal_cw_stand_holdbc1_hard1 STAYS the deployed stance policy; ppo_goal_cw_stand_crouchrise1 (md5 3877e16c) is banked as the crouch-robust variant for if bench work ever needs it. My own earlier verdict text on this entry incorrectly called the sto-hold current delta noise and claimed promotion -- superseded by the RSI-off matched-parent evidence above; see rl_docs/RISE.md / SKILLS.md / CURRENT_TRUTHS.md (already correct).

