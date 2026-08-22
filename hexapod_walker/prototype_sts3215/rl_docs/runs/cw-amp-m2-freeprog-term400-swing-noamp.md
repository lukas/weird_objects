# cw-amp-m2-freeprog-term400-swing-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:14:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**wandb_id**: a88hxqac

**hypothesis**: Give the from-scratch statue basin a genuinely accessible income gradient: reward.k_walk_swing pays a one-shot bonus for ANY completed lift-off->airborne->touchdown swing of >=15mm, in ANY direction (unlike k_step_event, which only pays swings that already project >=10mm along the commanded direction -- useless to a policy that has never lifted a foot at all). Every other lever for this statue is now closed: term_penalty (fixed suicide only), std-anneal (made the SAME basin more regular, not less stuck), stage curriculum (0/6 gait_valid even at stage-0 forward-only), style-weight dose 0.5x-2.0x (both statued, indistinguishable from -noamp), command complexity (the bank's own literal simplest fixed-command case still statues, cw-amp-m2-freeprog-term400-fixedcmd{,-seed11}), and RSI-for-walk (goal.walk_gait_start_frac, refuted for this exact failure signature by cw-gait-rsi1 08-12 -- not re-tested). Single lever vs cw-amp-m2-freeprog-term400-noamp (FAIL, statue): reward.k_walk_swing=1.0. Bank-checked BEFORE this launch (test_task_semantics.py SLIPWALK_SWING_OVERRIDES, 11/11 new+existing PASS, zero regressions vs the 152-pass/1-known-fastprof-red baseline): real gait/creep income rises ~11-23% (558->622, 103->126) while every stationary twin (stall, park, skate) and a realistic farming twin I added (shuffle: a genuine coordinated six-leg tripod gait reversing direction every 0.6s, ~0 net travel despite real strides both ways) stay priced BELOW park -- so the mechanism does not reopen a known stationary-cheat exploit. Pre-registered NEW cheat to watch (no scripted twin could be made to trigger it, so this is a live-monitoring flag, not a bank-cleared risk): a single-leg farm pattern (one leg cycling in place, five legs sacrificed/static, env/reward_swing consistently >0 while det fwd travel stays ~0) -- if seen, it must be banked and fixed (e.g. gate k_walk_swing by the SAME six-leg walk_gait_gate MIN factor already used for velocity income) before any follow-up dose arm. Prediction-if-true: det fwd travel clears meaningfully above the 0.02-0.03m statue floor (even short of the 0.10m PASS bar counts as informative -- ANY real six-leg cycling with net travel v.s. every prior arm's 0/6 gait_valid) and env/reward_walk_freeprog_pen finally leaves its -1.4 to -1.5/tick floor. Prediction-if-false: statue persists identically (reward_swing stays ~0, meaning even a direction-agnostic bonus finds no exploitable gradient from this exact initialization) -- then the exploration barrier is deeper than any per-tick income shape and the next lever is a structural one (BC-init is off-limits by the track's from-scratch charter; next would be a stance-load/support-polygon curriculum or reducing to a 2-leg/4-leg sub-task).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-noamp AND the fixedcmd-seed11/style05-v2/stylew2-v2 statue family at matched budget: PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six legs cycling with net displacement. INFORMATIVE (not a full PASS, still worth a follow-up dose/budget arm) = det fwd travel clears meaningfully above 0.03m (the whole statue family's ceiling) OR gait_valid > 0/6 for the first time in this family, even short of the 0.10m bar. Cheat check (must hold for either PASS/INFORMATIVE to count): no single-leg-farm signature (env/reward_swing driving positive return while <=2 legs actually move and det fwd stays ~0.02-0.03m) -- if seen, FAIL regardless of reward_swing/return, and bank+fix the gate-by-six-leg-MIN mechanism before any follow-up.

