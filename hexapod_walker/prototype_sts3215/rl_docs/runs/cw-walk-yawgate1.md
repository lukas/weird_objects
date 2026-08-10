# cw-walk-yawgate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T15:26:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 12000000

**parent**: cw-walk-yawcmd1-rr1

**wandb_id**: nrp989dr

**hardware_ready**: False

**hypothesis**: Queue-0 named next arm (yaw income gating). yawcmd1 failed its yaw gate with walk_yaw_err FLAT across both seeds - the pre-registered if-false: the Gaussian yaw kernel pays heading-hold income at wz_ref~0 without requiring achieved tracking, so turning never has to emerge (same exploit class the linear kernel had before walk_kernel_prog_gate). One variable vs yawcmd1-rr1: reward.walk_yaw_kernel_gate=1.0 gates yaw income on achieved |wz| (smoke_yaw_quad covers the gated path). Prediction-if-true: turn-segment |wz_err| med drops from 0.24 toward <=0.10 rad/s and right turns become tracked, with JOYSTICK GATE + forward band retained. Prediction-if-false: |wz_err| stays ~0.24 - free income was not the binding exploit; escalate to a dedicated turn-in-place curriculum arm. Strongest alternative: turning emerges but linear driving erodes (kernel competition) - then ladder k_walk_yaw down, not off.

**gate**: eval_yaw.py own-cfg: commanded-turn |wz_err| med <=0.10 rad/s AND wz_ref=0 |wz| med <=0.05 AND JOYSTICK GATE @DR0.2 0 in-envelope falls AND forward det prog within joyjit-dr05-c1 band, slip <=1.25; frames watched det

**verdict**: FAIL on the yaw-tracking clauses of its own gate (escalation from yawcmd1's income-gate fix also fails); linear-driving retention PASSES. eval_yaw.py own-cfg (goal.walk_yaw_cmd=1, wz-max 0.3): turn |wz_err| med 0.236 rad/s (gate <=0.10), hold |wz| med 0.104 (gate <=0.05), 0 falls -- essentially unchanged from ungated yawcmd1-rr1 (0.239/0.099) and yawcmd1-s1 (0.242/0.087). Training telemetry env/walk_yaw_err flat 0.135->0.133 across all 4 quarters despite reward_walk_yaw dropping 0.67->0.50 (confirms the gate multiplier is wired correctly and IS suppressing free income -- not an implementation bug -- the policy still doesn't turn even when parking earns ~0). Retention intact: JOYSTICK GATE (eval_drive DR0.2) 0 in-envelope falls across full panel+flip-stress; own-cfg DR0.5 det harness gv 6/6, 0 term, prog med 0.97 slip med 1.42 (parent joyjit-dr05-c1 band 0.94/1.38, within noise); own-DR0 gate report gv 12/12 (det+sto), prog med 0.98-1.00, slip 1.22-1.25. Frames (det, gated mode): standard low-amplitude six-leg creep, no flag leg, no new pathology. ROOT CAUSE (kernel economics, not sim): k_walk_yaw peak income 1.0 vs r_walk kernel peak 2.0 + r_prog peak 1.25 -- turning perturbs vx/vy tracking, so trading walk-kernel income for yaw-kernel income is a losing bet at current weights; closing the free-income loophole was necessary but not sufficient because the *marginal* incentive to actually turn is still economically dominated by not turning. NEXT: raise reward.k_walk_yaw substantially (not just gate it) so tracking a nonzero wz_ref competes with the walk kernel instead of being dominated by it -- one-variable respec queued (cw-walk-yawgate2, k_walk_yaw 1.0->2.5, gate stays on). If that also fails, escalate to WISHLIST item 3's dedicated turn-in-place curriculum (separate goal mode, linear speed zeroed during commanded turns) rather than more weight tuning.

