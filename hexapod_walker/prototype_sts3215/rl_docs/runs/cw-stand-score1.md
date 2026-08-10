# cw-stand-score1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T22:39:24+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-plantgate1

**wandb_id**: ualg0m8i

**hypothesis**: Gates leak; income routing does not. plantgate1 (FAIL) proved a multiplicative PLANT_SPEC gate still leaves the flag-leg cheat ~60% of the height income, and it wins even warm-started from the honest champion. This arm REPLACES the rise income: height progress/milestones/finish/kernel are zeroed (reward.rise_score_income=1) and the only rise income is a progress ratchet + post-ramp hold pay on stand-score S = height-kernel x feet-down^2 x HARD no-flag x plant geometry, plus a ramp-weighted airborne-feet rent that prices the penalty-dodge (cheats escaped reward_height by lofting the torso). A flag-leg stand now earns ~nothing and pays rent; the exact walk-start plant is the only paid state. No reference tracking - the score must carry the ordering alone (operator: no waypoints unless data hardcore disagrees).

**gate**: posture-strict harness at plant height [108,114]mm: rise >=4/6 det with end_posture_ok AND lower retains >=5/6 by 2M; W&B rise_score rising and reward_rise_score_hold>0 on a real fraction of episodes; VIDEO: feet-loaded stand, no flag-leg/tripod, no leg-through-floor. Early call if rise_score flat ~0 at 1.5M.

