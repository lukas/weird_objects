# cw-arch-gru-dual2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T20:15:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-gru-dual1

**wandb_id**: 6qar7v8z

**hardware_ready**: False

**hypothesis**: The DAgger redistillation (operator, 08-13, local Mac) put genuine rise into the dual-core BC init for the FIRST time -- matched n=12/seed=1 recheck det 3/12 with real non-crouch wins (bridge 2/4, flat 1/3; two more bridge eps reached height within 2-3mm and failed only end-posture), vs effectively none in the old dual BC parent -- at the cost of lower (collapsed 0/6 det+sto, the bc2 DAgger fingerprint; hold 6/6 retained, walk gait honest gv 6/6 prog 1.03 with only vel_err 0.034 vs 0.030). dual1 proved this exact recipe (10M, stance anchors ON / walk-tick anchor OFF) lifts a NO-rise BC init to 7/12 det rise AND builds lower to 6/6 from a weaker starting point. ONE variable vs dual1: the BC parent (ppo_goal_cw_gru_dual_bc_dagger1.zip md5 b5167c10, pushed to all train pods). Prediction-if-true: rise recheck det >=8/12 with >=2 non-crouch wins, lower rebuilt >=4/6, walk retained with tracking polished back inside the success bar. Prediction-if-false: see gate FAIL branches (each routes to the rise-only-DAgger variant distill, an operator/local lever -- no coefficient arm).

**gate**: 10M forensics det+sto @DR0 gate cfg (identical to dual1) PLUS the matched n=12/seed=1 DR0 rise recheck (the corrective method used on dual1/hfloor1). PASS if rise recheck det >=8/12 with >=2 non-crouch (bridge or flat) wins (beats dual1's matched 7/12 det with 2 non-crouch) AND lower REBUILDS det >=4/6 (the init has lower collapsed 0/6) AND hold det >=4/6 AND walk gait_valid >=5/6 with prog_ratio med >=0.80 (freeze/paddle fingerprint absent). FAIL if rise recheck det <=7/12 (the DAgger-init rise gains do not survive RL -> next lever is the operator's rise-only-DAgger variant distill (local), NOT a coefficient/budget variant) OR lower stays <4/6 det (RL+anchors cannot rebuild from a COLLAPSED BC lower, unlike dual1's weak-but-present one -> the variant distill keeping lower BC-only becomes REQUIRED before any further dual-core arm) OR walk freezes/parks (prog <0.30 or sacrificed legs: dual-core isolation regression, escalate -- do not iterate).

**verdict**: FAIL per pre-registered branch 1 (n=12/seed=1 DR0 rise recheck det 5/12 <=7/12, ZERO non-crouch: bridge 0/4 flat 0/3 crouch 5/5) after canary AUTO-STOP at 3.18M/10M -- the first true protected-skill catch: the DAgger init passed rise_bridge 2/2 at the baseline probe (first init in this lineage to do so) and every probe from 1M on read 0/2. DIG-IN OBSERVATIONS: matched-parent control (dagger1 init, same seed/cfg/pod) det 3/12 with ALL wins non-crouch (bridge 2/4, flat 1/3, crouch 0/5) -- reproduces the operator local numbers exactly. So RL did not merely erode rise, it SWAPPED the profile back to the crouch attractor within 1M steps (init crouch 0/5->5/5, init non-crouch 3/7->0/7, matching hfloor1 endpoint profile); rise videos show honest low-crouch stalls, no cheat. Gate eval @3.18M: lower REBUILT det+sto 6/6 from the COLLAPSED (0/6) init (drag 134mm, roll_tail 0.2deg) -- refutes the "RL+anchors cannot rebuild collapsed lower" fear, so the variant distill is NOT required to keep lower BC-only; hold det 6/6 (drag 77mm, roll_tail 0.1deg); walk gv 6/6 det, zero sacrificed legs, prog med 0.61 / vel_err 0.046 = dual1 own mid-run transient at 3.2M (0.60/0.048, recovered to 0.95/0.043 by 10M) -- NOT a dual-core isolation regression, escalate branch does not fire. INTERPRETATION: PPO+stance-anchors spend the rare hard-start demo competence first; more budget would not protect it (gone by 1M; dual1 full 10M from scratch reached only 2 non-crouch wins). VERDICT: the DAgger-init rise gains do not survive warm-RL; next lever per pre-registration = the operator rise-only-DAgger variant distill (option b, local), NOT a coefficient/budget arm. NOTE for the transitions arms warm-start order (trans-dagger1 > dual2 > dagger1 BC): dual2 artifact is a 3.18M mid-transient checkpoint whose walk (0.61/0.046) is WORSE than the dagger1 BC init (1.03/0.034) and whose rise is crouch-only -- re-judge that slot. Evidence: logs/ckpt_eval/cw_arch_gru_dual2_{gate,owncfg,rise12}, gru_dual_bc_dagger1_rise12_pod (control).

