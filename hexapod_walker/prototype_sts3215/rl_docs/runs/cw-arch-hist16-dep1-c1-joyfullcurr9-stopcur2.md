# cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T04:37:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace

**wandb_id**: ft3d73yx

**hypothesis**: Plain English: the robot keeps falling on stop commands because it freezes by stiffly fighting the ground, drawing sustained actuator current until the 2.5A/0.8s safety cutoff trips -- so we now charge the actuator CURRENT itself on stop ticks instead of only charging residual body speed. joyfullcurr7 (speed charge) and joyfullcurr8 (+0.4s grace) both ended with 100% of joygate falls = over_current; re-timing a speed charge cannot touch a sustained-current trip by construction (trip needs >2.5A held 0.8s through a 0.1s LPF -- an isometric fight, not a braking transient). New mechanism (built+bank-proven this cycle, test_walk_stop_current.py 5/5 incl. a scripted fight pose that reproduces the over_current termination exactly and is charged ~-4.7/tick while the relaxed plant stance pays ~0; grace 4/4 + stopcharge 3/3 unaffected; tag exp/cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur): reward.k_walk_stop_current=2.0 charges per-servo current quadratically above 1.5A headroom on stop ticks only, sharing the 0.4s grace timer so braking current pays little. Speed charge k=1.0 + grace 0.4 inherited unchanged; same base parent checkpoint. Dose sibling stopcur6 (k=6.0) brackets whether the charge must exceed the speed-charge cap (4.0/tick) to win. If-true: joygate falls drop toward/below 8/48 with over_current no longer dominant AND b1 stop cert clears. If-false at BOTH doses (falls stay over_current-dominated): the fight is not reward-driven -- suspect the cert bar itself (0.015 m/s may be unreachable without stiffness) or the sim current model, and audit before any further stop-pricing arm.

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past b1; joygate falls <= joyfullcurr6's 8/48 with over_current no longer the dominant term_reason; DR0+ownDR walk gates stay >=5/6 gait_valid with no systemic term regression; video all six feet cycling

