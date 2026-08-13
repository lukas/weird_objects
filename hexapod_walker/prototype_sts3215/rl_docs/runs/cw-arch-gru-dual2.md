# cw-arch-gru-dual2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-13T20:15:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-gru-dual1

**wandb_id**: 6qar7v8z

**hypothesis**: The DAgger redistillation (operator, 08-13, local Mac) put genuine rise into the dual-core BC init for the FIRST time -- matched n=12/seed=1 recheck det 3/12 with real non-crouch wins (bridge 2/4, flat 1/3; two more bridge eps reached height within 2-3mm and failed only end-posture), vs effectively none in the old dual BC parent -- at the cost of lower (collapsed 0/6 det+sto, the bc2 DAgger fingerprint; hold 6/6 retained, walk gait honest gv 6/6 prog 1.03 with only vel_err 0.034 vs 0.030). dual1 proved this exact recipe (10M, stance anchors ON / walk-tick anchor OFF) lifts a NO-rise BC init to 7/12 det rise AND builds lower to 6/6 from a weaker starting point. ONE variable vs dual1: the BC parent (ppo_goal_cw_gru_dual_bc_dagger1.zip md5 b5167c10, pushed to all train pods). Prediction-if-true: rise recheck det >=8/12 with >=2 non-crouch wins, lower rebuilt >=4/6, walk retained with tracking polished back inside the success bar. Prediction-if-false: see gate FAIL branches (each routes to the rise-only-DAgger variant distill, an operator/local lever -- no coefficient arm).

**gate**: 10M forensics det+sto @DR0 gate cfg (identical to dual1) PLUS the matched n=12/seed=1 DR0 rise recheck (the corrective method used on dual1/hfloor1). PASS if rise recheck det >=8/12 with >=2 non-crouch (bridge or flat) wins (beats dual1's matched 7/12 det with 2 non-crouch) AND lower REBUILDS det >=4/6 (the init has lower collapsed 0/6) AND hold det >=4/6 AND walk gait_valid >=5/6 with prog_ratio med >=0.80 (freeze/paddle fingerprint absent). FAIL if rise recheck det <=7/12 (the DAgger-init rise gains do not survive RL -> next lever is the operator's rise-only-DAgger variant distill (local), NOT a coefficient/budget variant) OR lower stays <4/6 det (RL+anchors cannot rebuild from a COLLAPSED BC lower, unlike dual1's weak-but-present one -> the variant distill keeping lower BC-only becomes REQUIRED before any further dual-core arm) OR walk freezes/parks (prog <0.30 or sacrificed legs: dual-core isolation regression, escalate -- do not iterate).

