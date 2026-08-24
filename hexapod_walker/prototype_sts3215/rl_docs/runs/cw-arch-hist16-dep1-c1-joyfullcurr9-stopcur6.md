# cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T04:39:10+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace

**wandb_id**: eponx2n1

**hypothesis**: Plain English: high-dose sibling of stopcur2 -- if a mild actuator-current charge on stop ticks is not enough to stop the robot from stiffly fighting the ground until the over_current safety cutoff, this arm makes the near-trip fight decisively more expensive than anything the stop-speed charge can pay for. The speed charge caps at 4.0/tick; at k=2.0 a single servo at the 2.5A trip pays only 2.0/tick, so the trained policy could still net-profit from a hard frozen brake. k=6.0 prices one trip-level servo at 6.0/tick (cap 24/tick), strictly above the speed-charge cap, so no braking strategy that holds near-trip current can be reward-optimal. Same bank proof as stopcur2 (test_walk_stop_current.py 5/5; ranking relaxed-still > fight/creep/walk holds at any k>0 since the charge only fires on stop-tick current excess). Risk this arm measures: over-taxing stop-tick current may make the policy avoid crisp stops (slow creepy settling to dodge braking current), visible as stop_speed_m_s regression vs stopcur2. If-true: joygate falls <= 8/48, over_current no longer dominant, stop cert clears. If-false at BOTH doses: fight is not reward-driven -- audit the cert bar / sim current model before more stop-pricing arms.

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past b1; joygate falls <= joyfullcurr6's 8/48 with over_current no longer the dominant term_reason; DR0+ownDR walk gates stay >=5/6 gait_valid with no systemic term regression; video all six feet cycling

