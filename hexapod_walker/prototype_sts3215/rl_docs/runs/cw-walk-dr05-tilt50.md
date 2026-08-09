# cw-walk-dr05-tilt50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:41:13+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_dr05_r1.zip

**wandb_id**: 4blvzgkw

**hardware_ready**: no

**hypothesis**: 0-c(i) STABILITY, tilt-price half (split one-variable arm off cw-walk-dr05-r1, md5 13a668ea). Plain: under randomized physics + action noise the walker occasionally lets its body tip far (to ~11 deg) and the gait degenerates (flag leg, slip blowouts) — tipping is nearly free today (quad k_roll/k_pitch=10 => ~0.3/tick at 10 deg vs ~2-3/tick walk income; kernel flat past ~3 deg). One variable vs parent: reward.k_roll=50, reward.k_pitch=50 (1.5/tick at 10 deg, 0.03/tick at 1.5 deg — normal gait unaffected per scale audit). Prediction-if-true: DR0.5 sto blowout episodes disappear (gv 12/12, no ep slip/m>3; parent had 3 such eps + gv 5/6 sto) with det gait/income intact. Prediction-if-false: blowouts persist (tilt is a downstream symptom of the flag-leg failure, not the driver) OR speed/prog collapses (pricing suppresses leg swings). Strongest alternative: walks flatter but still slides — slip unchanged, gv unchanged. (Retry 2: pod race on t4, stale SHA on t9.)

**gate**: own-cfg DR0.5 6+6: 0 term, gv 12/12, no ep slip/m>3 (parent 3 eps), det agg slip/m <=1.3 (parent 1.56); DR0 det retention along>=0.55 gv 6/6; frames watched det+sto

**verdict**: DIG-IN (0-c(i) fork decider). OBSERVATIONS: own-cfg DR0.5 6+6 vs named baseline dr05-r1@DR0.5 (logs/ckpt_eval/cw_walk_dr05_r1_dr05): det slip/m per-ep [1.17,1.06,1.24,0.89,0.98,2.95] vs parent [1.27,1.23,1.33,0.95,0.94,3.66] — det blowout (>3) eliminated, median 1.12 vs 1.25; gv 12/12 + no sacrificed leg vs parent 11/12 with flag leg [5]; 0 term both. BUT sto blowouts persist on the SAME fixed draws as the parent (ep0 5.06/prog 0.30, ep1 27.84/prog 0.04 vs parent 3.74/23.3) — these are in-place stalls, not tip-overs. DR0 retention strong: det gv 6/6, slip/m med 1.066 (best of today's batch), along med 0.75. Frames det DR0.5+DR0: level six-leg cycling, no flag legs. Gate legs: 0-term PASS, gv PASS, DR0 retention PASS; 'no ep slip/m>3' FAIL (2 sto eps), det agg ≤1.3 FAIL (1.38, driven by one 2.95 ep). INTERPRETATION: pricing tilt 5x fixed the tilt-mediated degeneration (det blowout + flag leg gone) but the sto blowout metric is dominated by draw-specific stalls, which are not tipping-driven — the same two command draws stall parent and child, both seeds of the champion lineage too. Root cause chain: blowout metric <- stalls <- specific command draws <- policy, NOT tilt pricing. VERDICT: FAIL on pre-registered gate (prediction-if-true 'blowouts disappear' not met); genuine partial win on the tilt axis. hardware-ready: no. HYPOTHESIS STATUS: partially refuted — tilt pricing hardens deterministic behavior under DR but cannot remove draw-stalls; 0-c(i) tilt lever is worth folding into the champion line, stalls move to 0-c(iii) RELIABILITY as their own front.

