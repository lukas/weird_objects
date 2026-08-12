# cw-mt-widen2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T22:45:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-mt-widen1

**wandb_id**: 526rchp8

**hardware_ready**: False

**hypothesis**: Give the walking robot the SAME full training budget the from-scratch generalists got, and see if it finally learns to stop and to turn on command; this arm settles whether widen1's deaf-to-new-commands result was just a too-short fine-tune (2M) or a real representation limit. Continues cw-mt-widen1's checkpoint (a2 walking prior + 2M widened) to the b2-matched 20M budget, recipe unchanged — budget is the only variable. Prediction-if-true (budget story): stop segments actually stop (signed-probe stop-hold speed collapses toward 0) and yaw responds sign-correct both directions, while gait_valid retention holds — staged widening becomes the wave-2 mainline, beating b2's 0.51-prog/9-fall shortfall. Prediction-if-false (representation story): walking stays clean but velocity/yaw remain command-INVARIANT at the matched budget (stop-hold speed still ~0.06, tip differential ~0) — budget hypothesis dead, the pre-registered representation lever (obs.history_frames arm per MULTITASK.md later-waves) is next, no further budget/width variants on this recipe. Strongest alternative (forgetting-under-budget): long training on the widened distribution eventually erodes the walking prior into the b2-style compromise gait — that shows up as gait_valid/prog decay and routes to a command-width curriculum, not representation.

**gate**: At 20M, judged vs the b2-matched baselines: PASS = gate(DR0) det gait_valid >=4/6 AND det prog med >=0.6 AND signed probe (probe_signed_yaw, speed 0.05/wz-max 0.15, cfg walk_yaw_cmd=1 + walk_obs_body_vel=2) stop-hold speed_med <=0.02 m/s AND tip yaw differential (tip-left minus tip-right wz) >= +0.10 with eval_yaw falls <=2 -> staged widening is the wave-2 mainline; next rung per track doc. FAIL(no-acquisition) = gait retained (gait_valid >=4/6) but stop-hold speed_med >0.04 OR tip differential <0.05 -> budget lever dead, queue the obs.history_frames representation arm, no further budget/width variants. FAIL(forgetting) = gait_valid <=1/6 or det prog med <0.3 -> command-width curriculum lever. Report slip_per_m/roll_tail vs a2 (1.38-1.50 / 0.5-1.0deg) and vs widen1 (1.92-2.19 / 0.5-0.9deg) in the verdict regardless; eval_yaw + probe_signed_yaw are NOT auto-staged — run both at triage.

**verdict**: FAIL(no-acquisition) per pre-registered gate, decisively — the budget confound from widen1 is now closed. At the b2-matched 20M budget the a2 walking prior stays fully intact (gate(DR0) det gait_valid 6/6, prog med 1.54; own-DR0.2 6/6, prog med 1.68; zero terminations, zero sacrificed legs, roll_tail 0.4-1.0deg, video-confirmed genuine six-leg cycling) but NEITHER new command is acquired: probe_signed_yaw stop-hold speed_med 0.0417 m/s vs fwd-hold 0.0688 (needed <=0.02, and >0.04 trips the FAIL branch outright), tip-left/tip-right yaw differential 0.0032 (needed >=0.10) -- weaker sign-differentiation than widen1's already-failing 0.017, i.e. still no directional yaw control. eval_yaw agrees: turn |wz_err| med 0.122 (gate<=0.10), hold |wz| med 0.107 (gate<=0.05), 0 falls. Per the gate's own FAIL(no-acquisition) branch (gait retained AND stop-hold speed>0.04 OR tip differential<0.05, both true here): budget lever is DEAD -- 20M of the exact b2-matched budget does not let a staged-widened policy learn new commands, so this is not a too-short-fine-tune story. No further budget/width variants on this recipe. Next: obs.history_frames representation lever per the pre-registered plan (cw-mt-b-hist16-1 queued same cycle).

