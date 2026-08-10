# cw-walk-yawgate2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T16:06:50+00:00

**pod**: hexapod-mjx-train-8

**steps**: 12000000

**parent**: cw-walk-yawgate1

**wandb_id**: hoq9grxn

**hardware_ready**: False

**hypothesis**: Escalation off cw-walk-yawgate1 FAIL (income-gate fix correctly suppressed free heading-hold income (reward_walk_yaw 0.67->0.50) but turn |wz_err| stayed flat 0.236 vs 0.10 gate -- root cause is kernel economics, not a leftover loophole: k_walk_yaw peak income (1.0) is dominated by the walk-speed kernel (K_WALK=2.0) + progress kernel (up to 1.25), so trading velocity-tracking accuracy for yaw-tracking accuracy is a losing bet at current weights, even with the free-income path closed. One variable vs yawgate1: reward.k_walk_yaw 1.0->2.5 (raises the ceiling on yaw income closer to walk-kernel parity), walk_yaw_kernel_gate stays ON. If-true: turn-segment |wz_err| med drops toward <=0.10 rad/s with JOYSTICK GATE + forward-band retention held. If-false: |wz_err| still ~0.24 even with 2.5x income -- turning is blocked by something structural (gait-timing conflict, insufficient exploration, or obs/action bandwidth), not by price; escalate to WISHLIST item 3's dedicated turn-in-place curriculum (separate goal mode, linear speed forced toward 0 during commanded turns) instead of further weight tuning.

**gate**: eval_yaw.py own-cfg (goal.walk_yaw_cmd=1, wz-max 0.3): commanded-turn |wz_err| med <=0.10 rad/s AND wz_ref=0 |wz| med <=0.05 AND JOYSTICK GATE (eval_drive DR0.2) 0 in-envelope falls AND own-cfg DR0.5 det forward prog within joyjit-dr05-c1 band (prog med ~0.9-1.0, slip <=1.5); frames watched det

**verdict**: FAIL on yaw-tracking clauses, same signature as yawgate1 despite raising reward.k_walk_yaw 1.0->2.5 (income ceiling was NOT the binding constraint). eval_yaw.py own-cfg: turn |wz_err| med 0.233 rad/s (gate <=0.10, yawgate1 was 0.236 -- no improvement), hold |wz| med 0.091 (gate <=0.05, yawgate1 0.104), 0 falls. Per-scenario breakdown shows a directional bias, not a price problem: arc-left err 0.07-0.21 (small, command near the policys own ~+0.1 rad/s left drift) vs arc-right/tip-right err 0.23-0.37 (command fights the drift) -- error scales with distance from a fixed left-yaw bias in EVERY scenario including pure turn-in-place (tip-left/right), so the 2.5x income raise did not change behavior at all. Retention clean: JOYSTICK GATE (eval_drive DR0.2) PASS 0 falls; DR0 own-cfg gv 12/12 det+sto, prog med 0.98-1.00, slip 1.23-1.26; DR0.5 own-cfg det gv 6/6, prog med 0.95, slip med 1.56 (vs yawgate1 1.42 -- within eval noise, single-seed 6ep). Video (det, gated mode): clean six-leg creep, no flag leg, no new pathology. ROOT CAUSE: confirms yawgate1s own predicted if-false -- turning is blocked by something structural (a directional gait/actuation bias), not by kernel economics; raising the price further is not expected to help. NEXT (per yawgate1s pre-registered escalation, RL_PLAN queue-0): stop tuning k_walk_yaw, move to WISHLIST item 3s dedicated turn-in-place curriculum (decouple linear-speed sampling from yaw-rate sampling so turning is trained without competing walk-kernel pressure) -- this needs a small CODE change to _sample_walk goal generator (correlate vx draw with |wz| draw), not a launchable knob; flagging as a CODE backlog item rather than rushing a patch mid-hardware-window.

