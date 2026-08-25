# cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace-lo

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T02:27:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr7

**wandb_id**: 9ftrzr8y

**hypothesis**: Dose sibling of cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace (grace_s=0.4): tests a SHORTER grace window, reward.walk_stop_grace_s=0.2, on the exact same single-lever recipe/parent. Read the pair jointly: if 0.2 already clears over_current and gets closer to the b1 cert bar, a short grace is enough (transient is brief); if 0.2 still shows over_current-dominated falls while 0.4 doesn't, the transient needs more settle time; if BOTH still show over_current, the transient-relief hypothesis itself needs revisiting (a torque/current-rate charge instead of stop-speed pricing) regardless of grace length.

**gate**: Same as joyfullcurr8-stopgrace: walkcurr V6 b1 cert clears its stop check and frontier promotes past side90_60s; joygate falls <= joyfullcurr6's 8/48 with over_current no longer dominant; DR0+ownDR walk gates stay >=5/6 gait_valid; video all six feet cycling

**verdict**: Orphaned run recovered (same 3-point grace-dose bracket as -hi, launched 08-24 ~02:2x, finished 40M steps, never triaged until this cycle). This is the LOW dose (grace_s=0.2, half the base 0.4s). Held-out 60s joygate: 19/48 falls (WORST of the bracket: lo 19 -> base 14 -> hi 12), slip/m med 2.333 (cap 2.9, passes), direction_err med 47.64deg (allow 40, FAILS -- worse than base's 40.17 and hi's 38.72), gait_valid_frac 0.917 (worst of the 3). DR0 30s walk battery clean (0/12 terminations). Confirms the dose-response direction from the -hi sibling: LESS grace is monotonically worse on every axis (falls, direction, gait-valid), not just noise -- the full bracket now reads lo(worst) < base < hi(best), a clean monotonic result. FAIL, no promotion; the redirect (movecur1's current-dwell charge, already running) supersedes chasing this axis further. Evidence: logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr8_stopgrace_lo_{gate,joygate}/, W&B 9ftrzr8y.

