# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T06:51:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit

**wandb_id**: t8luzp7b

**hypothesis**: Plain sentence: give the stance core its own separate exploration-noise knob (log_std_b) so cooling it to -4.0 (which fixes hold) no longer also cools the walk core — THIS time with the knob actually wired: the first attempt (anchor6-logstdsplit, CANARY FAIL - INFRASTRUCTURE) never built log_std_b because the plain --init-from load dropped the flag, and it silently annealed the shared log_std instead. Fixed in commit 4fe10154 (enable_log_std_split retrofit on warm starts + fail-closed --log-std-anneal-core); this is the identical arm relaunched on the fixed code. Prediction-if-true: hold/sto DR-0 termination collapses like anchor4-stdanneal's own result (6/6 -> 0-2/6) AND walk stays gait_valid >=5/6 with prog_ratio 0.2-0.4 (no leg-sacrifice freeze) on both seeds. Prediction-if-false: walk still degrades even with its own log_std genuinely untouched — that would, for the first time on real evidence, refute the exploration-noise-starvation theory and point the dig-in at the shared critic/trunk. Strongest alternative: the anchor-loss/critic interaction breaks walk regardless of std routing.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. WIRING CHECK FIRST (new, from the anchor6 forensics): the train log must show the '[mjx-train] --gru-dual-log-std-split: retrofitted log_std_b' line and the finished checkpoint's policy.pth must contain a log_std_b tensor with log_std_b != log_std (anneal hit b only, walk's log_std free-trained away from -4.0) — if this fails the arm is another INFRASTRUCTURE fail, fix before any interpretation. Then same joint panel as anchor2/3/4-stdanneal/5-stdmild: FULL PASS = WALK-SURVIVES (det gait_valid >=5/6 both seeds, no 3+-leg-sacrifice freeze, prog_ratio >=~0.2) AND HOLD-HELPS-FULL (hold/sto DR-0 termination <=2/6) on BOTH seeds. PARTIAL if hold improves less but beats the 6/6 baseline while walk survives -> tuned-anneal follow-up. FAIL if walk shows the anchor4-class catastrophe on either seed WITH the wiring check green — that is the genuine refutation of the exploration-noise theory the invalid anchor6 pair could not deliver -> dig-in moves to shared critic/trunk.

**verdict**: CANARY FAIL - MECHANISM: per-core log_std split (retrofit-fixed wiring) does NOT rescue walk from the anchor4-class collapse on this joint 2-seed call -> exploration-noise theory REFUTED. Evidence: wiring check green both seeds (prior cycle: log_std_b tensor annealed to -4.0/std 0.018, distinct from walk's untouched log_std ~-1.49/std 0.225 -- checkpoint-verified, not inferred). Seed0 (this run) WALK-SURVIVES clean (det gait_valid 6/6, prog med 0.23, 0 sacrificed legs, slip med 6.6) AND HOLD-HELPS-FULL (hold/sto term 2/6, hold/det term 2/6) -- a clean pass in isolation. But twin seed1 (fix-s1) shows the anchor4-class catastrophe (det gait_valid 0/6, 3-5 legs sacrificed, prog med 0.01, video-confirmed static splayed-leg freeze) with the SAME wiring green. Per this arm's pre-registered gate text, FAIL-with-wiring-green on either seed is the decisive joint result: giving walk its own log_std slice does not fix the freeze -> exploration-noise theory for the anchor4-class collapse REFUTED. Root cause is NOT per-mode action-noise coupling; dig-in moves to the shared critic/trunk (gate's own next-step clause). Do not launch further log_std-split variants.

