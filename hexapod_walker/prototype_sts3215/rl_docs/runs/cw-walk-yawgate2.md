# cw-walk-yawgate2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T16:06:50+00:00

**pod**: hexapod-mjx-train-8

**steps**: 12000000

**parent**: cw-walk-yawgate1

**hypothesis**: Escalation off cw-walk-yawgate1 FAIL (income-gate fix correctly suppressed free heading-hold income (reward_walk_yaw 0.67->0.50) but turn |wz_err| stayed flat 0.236 vs 0.10 gate -- root cause is kernel economics, not a leftover loophole: k_walk_yaw peak income (1.0) is dominated by the walk-speed kernel (K_WALK=2.0) + progress kernel (up to 1.25), so trading velocity-tracking accuracy for yaw-tracking accuracy is a losing bet at current weights, even with the free-income path closed. One variable vs yawgate1: reward.k_walk_yaw 1.0->2.5 (raises the ceiling on yaw income closer to walk-kernel parity), walk_yaw_kernel_gate stays ON. If-true: turn-segment |wz_err| med drops toward <=0.10 rad/s with JOYSTICK GATE + forward-band retention held. If-false: |wz_err| still ~0.24 even with 2.5x income -- turning is blocked by something structural (gait-timing conflict, insufficient exploration, or obs/action bandwidth), not by price; escalate to WISHLIST item 3's dedicated turn-in-place curriculum (separate goal mode, linear speed forced toward 0 during commanded turns) instead of further weight tuning.

**gate**: eval_yaw.py own-cfg (goal.walk_yaw_cmd=1, wz-max 0.3): commanded-turn |wz_err| med <=0.10 rad/s AND wz_ref=0 |wz| med <=0.05 AND JOYSTICK GATE (eval_drive DR0.2) 0 in-envelope falls AND own-cfg DR0.5 det forward prog within joyjit-dr05-c1 band (prog med ~0.9-1.0, slip <=1.5); frames watched det

