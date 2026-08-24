# cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-24T04:39:10+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace

**wandb_id**: eponx2n1

**hypothesis**: Plain English: high-dose sibling of stopcur2 -- if a mild actuator-current charge on stop ticks is not enough to stop the robot from stiffly fighting the ground until the over_current safety cutoff, this arm makes the near-trip fight decisively more expensive than anything the stop-speed charge can pay for. The speed charge caps at 4.0/tick; at k=2.0 a single servo at the 2.5A trip pays only 2.0/tick, so the trained policy could still net-profit from a hard frozen brake. k=6.0 prices one trip-level servo at 6.0/tick (cap 24/tick), strictly above the speed-charge cap, so no braking strategy that holds near-trip current can be reward-optimal. Same bank proof as stopcur2 (test_walk_stop_current.py 5/5; ranking relaxed-still > fight/creep/walk holds at any k>0 since the charge only fires on stop-tick current excess). Risk this arm measures: over-taxing stop-tick current may make the policy avoid crisp stops (slow creepy settling to dodge braking current), visible as stop_speed_m_s regression vs stopcur2. If-true: joygate falls <= 8/48, over_current no longer dominant, stop cert clears. If-false at BOTH doses: fight is not reward-driven -- audit the cert bar / sim current model before more stop-pricing arms.

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past b1; joygate falls <= joyfullcurr6's 8/48 with over_current no longer the dominant term_reason; DR0+ownDR walk gates stay >=5/6 gait_valid with no systemic term regression; video all six feet cycling

**verdict**: Result: the k_walk_stop_current=6.0 dose (2x the speed-charge cap) DID achieve its primary design goal -- joygate falls crashed from joyfullcurr8's 14/48 to 1/48 (best-ever in this stop-pricing lineage, previous best was joyfullcurr6's 8/48), slip/m improved to 2.317 (cap 2.9), and dir_err 45.39deg -- but it did NOT clear the ledger's other two named bars and introduced a new video-confirmed pathology. Evidence: gate_verdict.json falls_total=1/48 (the sole survivor still over_current, dr0p5); walkcurr/frontier stayed pinned at 1 all 40M steps (b1 stop_speed_m_s oscillated 0.034-0.047, never trending toward the 0.015 cert bar -- this specific lever prices actuator current, not the cert's body-speed metric, so no reason it should move this axis); own-cfg (DR 0.5) det gait_valid CRASHED to 2/6 (was 5/6 on the stopgrace parent), gate DR0 det also dropped to 4/6 -- both from a NEW, consistent leg-3 sacrifice (sac=[3] in 4/6 owncfg-det + 2/6 gate-det episodes), video-confirmed (walk_det_1.png/.mp4: leg held rigidly aloft the whole clip vs walk_det_0.png's clean 6-leg cycle) -- a real trade the k=6 dose introduces, not noise. Reward quarters 848.8/908.1/855.4/811.4 (peaks Q2, declines) = aligned per 08-21, not a keep-training case. Why: at 2x the speed-charge cap the policy found a cheaper way to avoid the current charge on one specific joint (immobilize it) rather than moving the whole gait to lower peak current, especially exposed under DR. What's next: joint dose read with the k=2.0 sibling (cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2, owned by a concurrent cycle) decides whether the lower dose keeps the over_current win without the leg-sacrifice trade; if k=2 also sacrifices a leg, the fix needs a per-leg-fairness or smoothness term alongside the current charge, not a dose retune. Champion unchanged (stotight45-seed13); this V6 hardening lineage remains operator-ordered polish (fb_20260823T220651_5c66e3), not the core DONE gate.

