# cw-arch-hist16-dep1-c1-joyfullcurr10-chg2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T06:34:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: zi9qtmgu

**hypothesis**: Plain English: joyfullcurr9-stopcur2 proved the actuator-current stop charge (k=2.0) kills the over_current fall pathology (14/48->1/48) but does nothing for the walkcurr b1 cert -- in-training stop_speed_m_s plateaued 0.027-0.048 m/s the whole 40M run, identical to every prior arm, because the current charge only fires near the 2.5A trip (observed max current stayed 1.1-1.4A) and is a no-op against a gentle low-current creep. The STATUS.md caution against raising k_walk_stop_charge was written before this safety net existed and was reasoned, not dose-tested past k=1.0 -- its feared failure mode (raising the speed charge intensifies hard-braking -> over_current) is now directly priced out by the proven current charge. This arm doubles the stop-speed charge (k_walk_stop_charge 1.0->2.0) on top of the unchanged, proven k_walk_stop_current=2.0, same base parent (ppo_goal_cw_arch_hist16_dep1_c1.zip), same recipe otherwise. If-true: walkcurr/b1_front45_20s/stop_speed_m_s trends measurably below the 0.027-0.048 plateau (ideally <=0.015, promoting frontier past b1) WITHOUT joygate over_current falls climbing back toward the pre-stopcur2 13-14/48 level. If-false (matches sibling -chg4): creep stays pinned in the same band regardless of price -- the residual creep is a physical/settle-time floor, not reward-price-reachable; escalate to auditing whether the 0.015 m/s cert bar itself is achievable given actuator/contact settle dynamics, not further stop-pricing dose.

**gate**: walkcurr V6 b1 cert clears its stop check (stop_speed_m_s <= 0.015) and frontier promotes past b1; joygate falls stay <= stopcur2's 1/48 with over_current not the dominant term_reason; DR0+ownDR walk gates stay >=5/6 gait_valid with no systemic term regression; video all six feet cycling

**verdict**: Doubling the stop-speed charge (k_walk_stop_charge 1.0->2.0, on top of the proven k_walk_stop_current=2.0) does NOT move the walkcurr b1 cert and ACTIVELY REGRESSES the axis stopcur2 had just fixed. Evidence: in-training walkcurr/b1_front45_20s/stop_speed_m_s finishes at 0.0416 m/s (cap 0.015), same 0.027-0.048 band as every prior dose (0/1/2x) across 80 cert rounds -- frontier stuck at b1, no trend. Held-out 60s joygate: falls climbed 1/48 (stopcur2) -> 4/48, slip/m 2.626 (still under 2.9 cap) but dir_err 47.93deg now misses the 40deg allow (stopcur2 passed at 42.83). Own-DR(0.5) det gait crashed 5/6 (stopcur2, 1/6 leg-3 sac) -> 2/6 (3/6 episodes sacrifice leg 3, video-confirmed rigid held-aloft leg, slip 1.62-6.85) -- the SAME leg-lock shortcut stopcur6 found at k=6.0 now reappears at a lower dose once the speed charge (not just current charge) is doubled. Reward quarters 805/826/740/664 (peaks Q2, declines) = aligned per 08-21, not undertrained -- the charge bites, the policy just can't buy the cert without a new mechanism. This closes the k_walk_stop_charge dose-escalation lever for good (matches the pre-registered if-false exactly, plus a new dose-dependent leg-lock regression finding stopcur2 didn't have). Evidence: logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr10_chg2_{gate,owncfg,joygate}/, W&B run zi9qtmgu.

