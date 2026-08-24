# cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T02:19:10+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr7

**hypothesis**: Plain English: joyfullcurr7 (stop-speed charge, no grace) cut the stop-tick creep a lot but plateaued short of the cert bar, AND its held-out joygate falls got WORSE, and every single fall was over_current (actuator safety trip from hard braking) -- the charge prices the unavoidable deceleration transient right after a stop command exactly as harshly as sustained creep. Single-lever fix vs joyfullcurr7: reward.walk_stop_grace_s=0.4 linearly ramps the charge's multiplier 0->1 over the first 0.4s of a stop segment (built+bank-tested this cycle, test_walk_stop_grace.py 4/4 + existing stopcharge bank 3/3 unaffected), so the physically-necessary transient pays little/nothing while sustained post-transient creep still pays the full charge (same k_walk_stop_charge=1.0 dose, unchanged). Same base parent as joyfullcurr7 (ppo_goal_cw_arch_hist16_dep1_c1.zip, inherited via --from, NOT joyfullcurr7's own regressed checkpoint). If-true: b1 stop cert clears (or gets much closer) AND joygate falls drop back toward/below joyfullcurr6's 8/48 with over_current no longer dominant. If-false (falls stay high/over_current-dominated): the transient itself isn't the cause and a torque/current-rate charge is the next lever, not stop-speed pricing at all; if plateau persists but over_current clears, dose (not the transient) was the true blocker and a grace+dose combination is next.

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past side90_60s (cmd_prog>=0.65, slip/m<=2.0, 0 falls); joygate falls <= joyfullcurr6's 8/48 with over_current no longer the dominant term_reason; DR0+ownDR walk gates stay >=5/6 gait_valid 0 systemic term regression; video all six feet cycling

