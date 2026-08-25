# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T21:32:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**hypothesis**: Seed twin of meshref-tuckclock1 (only --seed 0->1 differs; same single lever train.bc_anchor_flat_time_indexed=1 vs the meshref parent): does the flat-start absolute-script-clock anchor work robustly across seeds, or was seed-0 luck? Plain story: a probe measured that the honest scripted tuck-then-press ON ITS OWN CLOCK is the reward optimum (+2021, 0.575A, plant_ok) under the exact launched pricing where every taught behavior scores -50..-770 -- this pair tests whether anchoring flat starts to that clock lets PPO find it. Judged jointly with meshref-tuckclock1 per its pre-registered gate. Same joint-pair discipline as every mechanism hedge this campaign.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same joint-pair gate as meshref-tuckclock1 (see that run's doc): PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant with genuine duty>0/swing_count>0 sub-over_current tuck-then-press AND non-flat kinds >= meshref parent (5/6+4/6) -> 8M acquisition grid + stancemix port. PARTIAL if >=2/6 flat valid per seed or genuine swinging tuck with h_err 10-40mm while non-flat holds -> extend budget. FAIL if flat stays 0-1/12 -> clock-target semantics refuted; next suspect is PPO/anchor interaction (read train/bc_anchor_loss_rise: high+freeze = dose coef; low+freeze = emission bug).

