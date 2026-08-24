# cw-arch-hist16-dep1-c1-joyfullcurr10-chg4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T06:37:05+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: nq6a26vc

**hypothesis**: Plain English: high-dose sibling of joyfullcurr10-chg2. joyfullcurr9-stopcur2 proved the actuator-current stop charge (k=2.0) kills the over_current fall pathology (14/48->1/48) but leaves the walkcurr b1 cert (stop_speed_m_s<=0.015) completely unmoved -- creep plateaued 0.027-0.048 m/s the whole 40M run because the current charge only fires near the 2.5A trip and never touches a gentle low-current creep. This arm quadruples the stop-speed charge (k_walk_stop_charge 1.0->4.0, matching the charge's own existing dose-cap ceiling) on top of the unchanged, proven k_walk_stop_current=2.0, same base parent, same recipe otherwise -- brackets whether -chg2's dose (if it moves the metric at all) needs to go further, or whether the creep is dose-insensitive entirely (mirroring the original k_walk_stop_charge 0->1.0 dose step, which reduced creep once then plateaued rather than continuing down with more training). If-true: stop_speed_m_s drops toward/below 0.015 and frontier promotes past b1, without reopening over_current (falls stay near stopcur2's 1/48). If-false at BOTH doses (this and -chg2): the residual creep is dose-insensitive -- close the stop-speed-charge-dose lever for good and audit whether the 0.015 m/s cert bar is physically achievable (actuator/contact settle-time floor) before any further stop-pricing mechanism.

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past b1; joygate falls stay <= stopcur2's 1/48 with over_current not the dominant term_reason; DR0+ownDR walk gates stay >=5/6 gait_valid with no systemic term regression; video all six feet cycling

**verdict**: Quadrupling the stop-speed charge (k_walk_stop_charge 1.0->4.0, same k_walk_stop_current=2.0) is WORSE than chg2's 2x dose on every axis and still does not move the cert. In-training stop_speed_m_s finishes 0.0426 m/s (cap 0.015, unchanged band). Held-out 60s joygate: falls climbed to 8/48 (vs stopcur2's 1/48, chg2's 4/48), slip/m 3.443 now OVER the 2.9 cap, dir_err 55.56deg (allow 40). Own-DR(0.5) det gate produced an ACTUAL FALL (video-confirmed progressive tilt_roll topple, walk_det_1: roll climbs 15.5->27.6->73.0->85.5deg then off-camera; leg 0 sacrificed) plus the same leg-lock pattern chg2 showed elsewhere -- gait_valid 5/6 with 1 real term, own-DR sto also shows two near-zero-progress episodes (slip 32-61/m, thrash). Reward quarters 713/722/572/394 (peaks Q1-2, declines steeply, steeper than chg2's) -- aligned per 08-21, not undertrained. Confirms and sharpens chg2's finding: raising the stop-SPEED charge dose past what the stop-CURRENT charge already prices is a strictly-worse, monotonically-regressing lever on the cert-adjacent axes with zero cert benefit at any tested dose (0/1/2/4x). Closes the k_walk_stop_charge dose-escalation lever for good, both arms. Evidence: logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr10_chg4_{gate,owncfg,joygate}/ (walk_det_1.mp4/.png video-confirmed fall), W&B run nq6a26vc.

