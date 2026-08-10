# cw-stand-score1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-10T22:39:24+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-plantgate1

**wandb_id**: ualg0m8i

**hardware_ready**: False

**hypothesis**: Gates leak; income routing does not. plantgate1 (FAIL) proved a multiplicative PLANT_SPEC gate still leaves the flag-leg cheat ~60% of the height income, and it wins even warm-started from the honest champion. This arm REPLACES the rise income: height progress/milestones/finish/kernel are zeroed (reward.rise_score_income=1) and the only rise income is a progress ratchet + post-ramp hold pay on stand-score S = height-kernel x feet-down^2 x HARD no-flag x plant geometry, plus a ramp-weighted airborne-feet rent that prices the penalty-dodge (cheats escaped reward_height by lofting the torso). A flag-leg stand now earns ~nothing and pays rent; the exact walk-start plant is the only paid state. No reference tracking - the score must carry the ordering alone (operator: no waypoints unless data hardcore disagrees).

**gate**: posture-strict harness at plant height [108,114]mm: rise >=4/6 det with end_posture_ok AND lower retains >=5/6 by 2M; W&B rise_score rising and reward_rise_score_hold>0 on a real fraction of episodes; VIDEO: feet-loaded stand, no flag-leg/tripod, no leg-through-floor. Early call if rise_score flat ~0 at 1.5M.

**verdict**: FAIL — known exploit recurs. rise 0/12 valid_plant/end_posture_ok at DR0 gate AND 0/12 at own-DR0.2, every start_kind (flat/bridge/crouch): flag-leg (plant_fail includes feet_down+no_flag, one leg 40-150mm off ground) on all 12 episodes, identical pathology to cw-stand-b2p1 and cw-stand-plantgate1. env/rise_score stayed flat ~0.01-0.02 the entire 2M steps (matches the run's own pre-registered early-stop trigger). hold/track unaffected (hold 6/6 det end_posture_ok). Score-income routing (3rd distinct reward-restructuring mechanism) does not stop the cheat even warm-started from the honest stance champion — reward-shaping-only fixes for rise now look exhausted.

