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

**verdict**: FAIL (mixed, lever PROVEN): crouch defect fixed decisively — gate det crouch rise 5/5 + sto 4/4, zero tilt falls, and RSI-off crouch 8/8 det + 8/8 sto vs hard1 0/8 (logs/ckpt_eval/crouchrise1_rsioff_crouch). But hold retention FAILS its own gate: hold det+sto valid_plant 7/12 (needs >=10/12; hard1 11/12) and det hold parks feet 1+4 at contact duty 0.07/0.01 (hard1: all six 0.90-0.99) — the flag-leg cheat resurfaced. Root cause: the SECOND delta vs hard1, goal-mix skew rise 0.45->0.6 / lower 0.45->0.3 (start-mix alone was the fix). Do NOT deploy; do not warm from this checkpoint (cheat may be baked in). Follow-up cw-stand-crouchrise2 queued: warm from hard1, keep start-mix, restore hard1 goal mix. Triage: operator 08-11 ~17:10Z (run sat unverdicted 4h; gate eval was already on train-0 at logs/ckpt_eval/cw_stand_crouchrise1_gate).

