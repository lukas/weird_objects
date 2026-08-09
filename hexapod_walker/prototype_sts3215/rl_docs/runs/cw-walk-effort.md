# cw-walk-effort

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T06:47:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**wandb_id**: 9rtpws1h

**hypothesis**: Paddling/skating persists because physical effort is priced at ~4% of velocity income (current -13, drag -4, action -29 vs +1250/ep, measured cycle 27); pricing walk-mode effort at ~18% of income (k_walk_effort=1.2, per-tick -k x mean servo current) makes anchored stance the cheaper way to keep velocity income. If-true: DR1.0 agg slip/m det<=1.0 AND sto<=1.0 (champion baseline 1.543/1.295) with gait retained (DR0 det fwd mean >=0.55, gv 12/12 at DR0+DR1.0, 0 term). If-false: (i) slip/m stays >=1.2 with the charge simply paid -> effort cannot reach stance anchoring, escalate to the review's phase-prior rung; or (ii) gait collapses (DR0 det fwd mean <0.45 or gv failures) -> k oversized (one recalibrated retry max, from the scale audit; no blind k iteration). Strongest alternative: current drops WITHOUT slip dropping (lighter/slower paddling) - distinguished by slip/m vs mean-current trends moving independently.

**gate**: DR1.0 harness 15s own-cfg 6+6: agg slip/m det<=1.0 AND sto<=1.0, gv 12/12, 0 term; DR0 harness 15s: det fwd mean >=0.55, gv 12/12, forward-hemisphere sto fwd>=0.40 5/5 (backward draw sto[5] recorded but excluded pending operator ruling, cycle 27); frames: visible stance anchoring vs champion paddling

**verdict**: FAIL. DR1.0 gate: agg slip/m det 1.484 (champion 1.543, delta -0.06 ~4% inside noise) sto 1.753 (champion 1.295, worse); gv 11/12 (sto5 sac leg1); 1 over_current termination (sto0) vs required 0. DR0: det fwd mean 0.722 >=0.55 ok (champion 0.745), fwd-hemisphere sto 5/5 ok, gv 11/12, det slip/m 1.282 vs champion 1.180 (no improvement). Training: reward_effort flat -0.601->-0.595 Q1->Q4, reward_current/drag unchanged - the charge was simply paid. Frames (8 strips watched, both DR levels): same sprawly paddling gait as champion, no visible stance anchoring. HYPOTHESIS REFUTED, pre-registered if-false branch (i): effort pricing at 18pct of income cannot reach stance anchoring -> escalate to review phase-prior rung. NOT HARDWARE-READY (skating unchanged, over_current at DR1.0). Champion unchanged.

