# cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace-hi

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T02:29:02+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr7

**wandb_id**: gscw2nh1

**hypothesis**: Dose sibling of cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace (grace_s=0.4): tests a LONGER grace window, reward.walk_stop_grace_s=0.8, on the exact same single-lever recipe/parent. Read the 3-point bracket (0.2/0.4/0.8) jointly against joyfullcurr7 (0.0, the FAIL baseline): the over_current fall count and the b1 stop-cert speed should move monotonically with grace length if the transient-relief mechanism is real; a non-monotonic or flat-at-every-dose result means grace length isn't the operative variable.

**gate**: Same as joyfullcurr8-stopgrace: walkcurr V6 b1 cert clears its stop check and frontier promotes past side90_60s; joygate falls <= joyfullcurr6's 8/48 with over_current no longer dominant; DR0+ownDR walk gates stay >=5/6 gait_valid; video all six feet cycling

**verdict**: Orphaned run recovered (launched 08-24 ~02:2x as part of a 3-point grace-dose bracket alongside the base -stopgrace 0.4s [already FAIL-verdicted 08-24 04:11] and -lo 0.2s; finished 40M steps 08-24 but never triaged -- picked up this cycle since evals were already pre-staged and sitting complete). This is the HIGH dose (grace_s=0.8, 2x the base). Held-out 60s joygate: 12/48 falls (BEST of the 3-point bracket: lo 19/48, base 14/48, hi 12/48), slip/m med 2.242 (cap 2.9, PASSES), direction_err med 38.72deg (allow 40, PASSES -- first arm in this lineage to clear BOTH slip and direction), gait_valid_frac 0.9375. Still FAIL overall (zero_falls and gait_valid_all both required, neither cleared) but this is a genuine, monotonic DOSE-RESPONSE across the bracket: more stop-charge grace = fewer falls AND tighter direction tracking (lo 47.6deg -> base 40.2deg -> hi 38.7deg; lo 19 falls -> base 14 -> hi 12). DR0 30s walk battery is clean (0/12 terminations, video shows upright six-leg cycling, good posture -- much less crouched than the effort-pricing lineage). This nuances the base run's own verdict ('settle-grace softens slip/dir but does not fix over_current, next lever is a direct current charge not stop-speed pricing'): grace dose DOES help monotonically, just not enough alone at any tested dose to clear the strict zero-falls bar -- consistent with (not contradicting) that verdict's redirect, since the already-launched movecur1 family (a direct current-dwell charge, on top of a similar recipe) is the compounding next step, not a reason to chase a higher grace dose in isolation. Evidence: logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr8_stopgrace_hi_{gate,joygate}/, W&B gscw2nh1.

