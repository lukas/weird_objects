# cw-amp-m2-freeprog-term400-fixedcmd-seed11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T16:37:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**wandb_id**: i9iets9t

**hardware_ready**: False

**hypothesis**: Plain English: test whether a from-scratch policy can learn to walk forward AT ALL once the task is the exact simplest case the SLIPWALK semantics bank itself already validates -- one fixed slow forward speed (0.05 m/s) held the whole episode, no direction changes, no stops, no turns, no mid-episode resampling. Single lever vs cw-amp-m2-freeprog-term400-noamp: replace the goal.* command config with the literal SLIPWALK_OVERRIDES fixed command; byte-identical reward stack, no AMP. See --notes on the launched run for the full writeup.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg -- eval auto-matches this run's own fixed-command training cfg). PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six-leg cycling with net forward displacement, read against the SLIPWALK bank's own gait(+417,0.22m)/creep(+108,0.16m)/stall(-143)/park(-244) reference. Prediction-if-true: task complexity (not reward shape) was the barrier -- revisit stress_mix/stagecurric with a longer held stage-0. Prediction-if-false: statue persists at the bank's own simplest case -- genuine PPO exploration/basin-barrier problem; next lever is RSI or a stance-load curriculum, not another task/dose tweak.

**verdict**: FAIL, closes the task-complexity branch decisively: even the SLIPWALK bank's own literal simplest fixed-command case (0.05 m/s forward, no turns/stops/resampling) statues. DR-0 det: fwd travel 0.02-0.03m/15s (bar 0.10m) all 6 episodes, gait_valid 0/6, legs [3,4] sacrificed every episode, slip 6.43/m; sto: fwd 0.03-0.10m, gait_valid 2/6, slip 16-22/m. W&B reward fell every quarter (-152/-540/-1032/-1260), never rising -- genuine not-learning per the 08-21 rule, not misalignment-continue. Video/contact-sheet confirms near-frozen sprawled stance across all 10 frames, no visible net translation. Combined with the already-closed command-complexity ladder (stress_mix -> stagecurric -> this literal fixed-command floor), task complexity is definitively NOT the barrier for the freeprog/term400 statue -- matches the run's own pre-registered prediction-if-false: genuine PPO exploration/basin-barrier problem. RSI-for-walk (goal.walk_gait_start_frac) also closed for this failure signature without spending GPU: cw-gait-rsi1 (08-12) already tested the identical mechanism on an analogous from-scratch dragstance-pricing statue and got the SAME loadslip-collapse/near-zero-step-event/uniform-march FAIL. Next lever (bank-built this cycle, zero GPU spent yet): reward.k_walk_swing, an existing direction-agnostic one-shot swing-completion bonus never armed for this family -- SLIPWALK_SWING_OVERRIDES bank (11/11 new+existing PASS) confirms it raises real gait/creep income ~11-23% while a realistic farming twin (shuffle: genuine six-leg strides reversing direction every 0.6s, ~0 net travel) stays priced BELOW park, so it does not reopen a stationary/farming exploit. Launching a matched noamp+style05-v2 pair with this one lever added.

