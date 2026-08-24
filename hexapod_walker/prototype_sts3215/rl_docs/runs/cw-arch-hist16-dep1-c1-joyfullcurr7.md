# cw-arch-hist16-dep1-c1-joyfullcurr7

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T00:15:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr6

**wandb_id**: 6l0afxr1

**hypothesis**: Plain English: same operator-ordered full-circle joystick ladder as joyfullcurr6, with the one measured defect fixed - the robot now gets explicitly charged for moving when told to STOP, so the curriculum can promote past the rung where its predecessor sat for 40M steps. joyfullcurr6 froze at b1 because its cert stop bar (mean stop-tick speed <= 0.015 m/s) had NO matching reward term: on stop ticks every walk term except the shallow Gaussian kernel is s_ref-guarded, and PPO converged to a 0.04 m/s creep priced as near-optimal (dig-in 08-24, corrected root cause - k_walk_freeprog was never active). New reward.k_walk_stop_charge=1.0 (scale 0.015 = the cert bar, cap 4.0, turn-in-place exempt, default-off key, bank-proven: still_charged > creep_charged by >300/ep under this exact stack, stillness untaxed). Single-mechanism delta vs joyfullcurr6; same parent ppo_goal_cw_arch_hist16_dep1_c1.zip (NOT the regressed joyfullcurr6 checkpoint - it fell 8/48 on the joygate). If-true: b1 stop cert clears and the frontier climbs into side90/rear buckets. If-false: b1 stop_speed still >0.015 with the charge active (deeper defect - e.g. decel-transient dominates the mean and the bar itself needs a settle grace), or the stop charge trades away walking quality (watch b0/b1 cmd_prog and slip vs joyfullcurr6's own certs).

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past side90_60s (cmd_prog>=0.65, slip/m<=2.0, 0 falls); FINAL unchanged from joyfullcurr6: full-circle 60s drive eval DR0.5, 0 falls, rear/side followed, slip not exploding vs parent; DR0+ownDR walk gates 6/6 gait_valid 0 term; video all six feet cycling

