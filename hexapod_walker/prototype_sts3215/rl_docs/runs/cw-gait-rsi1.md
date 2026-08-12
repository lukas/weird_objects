# cw-gait-rsi1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T03:30:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-gait-dragstance1

**wandb_id**: x6w5e5d9

**hardware_ready**: False

**hypothesis**: Teach a from-scratch (no imitation, no BC anchor) walking network to lift its feet by starting some episodes mid-stride inside the tall scripted gait, instead of always starting standing still. cw-gait-dragstance1 showed that pricing the skate structurally (a per-stance drag charge, correctly sized by audit) is not enough alone -- from scratch the network just freezes/parks rather than discovering real stepping. The RSI mechanism that already unstuck exploration for stand-up (rise-RSI) has an existing, unbuilt-for-nobc but landed-for-hw walk-mode twin (goal.walk_gait_start_frac, from hw's tall-ladder T6): with probability f, an episode spawns mid-stride in the scripted TripodGait's tall pose instead of standing still. ONE variable vs cw-gait-dragstance1: goal.walk_gait_start_frac=0.5. Prediction-if-true: at least some fraction of episodes discover honest stepping from the injected mid-stride states (visible swing clearance, slip/m below the 1.1-1.5 paddle band, det gait_valid with all six feet cycling) even though standard starts still might not. Prediction-if-false: the policy dives straight back to the frozen/parked pose even from an injected mid-stride start (matching hw's tall-rsi1 result, where the SAME mechanism on a warm-started tall walker actively dove back to its crouch) -- meaning state injection doesn't help gait discovery any more than it helped tall posture, and the next lever for nobc's gait-from-scratch line is BC-INIT (now hw-proven to work in cw-dep-bcgait1) or an annealed-up drag charge.

**gate**: PASS (informative either way, per nobc's own queue item 2): if det gait_valid shows real six-foot cycling (not frozen/parked) in ANY episode with slip/m below the paddle band -- state injection helps discovery, worth a wider dose. FAIL if every det episode reproduces the identical freeze/park fingerprint from cw-gait-dragstance1 regardless of start state -- RSI-for-walk is refuted for from-scratch gait too, matching hw's tall-rsi1 null; next lever is BC-INIT or annealed charge, not more RSI dose.

**verdict**: FAIL, matching the pre-registered false branch. Gate: det+sto 0/6 both DR0 and own-DR0.5, prog_ratio 0.00, slip/m 6-18 (way above the 1.1-1.5 paddle band the PASS branch required); gait_valid 6/6 only means no single sacrificed leg, not real stepping -- video is uniform marching-in-place, zero net floor travel, all 24 clips. Training mechanism is IDENTICAL to cw-gait-dragstance1's freeze/park: env/walk_loadslip_factor collapses 0.51->0.03-0.07 by step 49 and stays floored, env/reward_drag_stance sits at -7..-8.5/tick unresolved the whole run, env/reward_step_event stays ~0.013-0.019 (near zero). Mid-stride RSI spawns did not give the policy anything to build on -- it drifts back to minimal-motion within the first ~1% of training regardless of start state. RSI-for-walk (P3 lever 4) is CLOSED for nobc's gait-from-scratch line, matching hw's tall-rsi1 null exactly as the hypothesis predicted-if-false.

