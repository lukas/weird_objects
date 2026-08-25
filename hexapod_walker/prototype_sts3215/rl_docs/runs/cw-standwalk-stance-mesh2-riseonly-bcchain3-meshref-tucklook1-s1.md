# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tucklook1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T20:32:40+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tucklook1

**wandb_id**: zxnx3fb3

**hypothesis**: Seed twin of meshref-tucklook1 (only --seed 0->1 differs, same script-index-floor lever pair: tuck_exempt=1 + tuck_lookahead_s=1.25): does the fix work robustly across seeds, or was seed-0 luck? Same joint-pair discipline as every mechanism hedge this campaign (meshref-s1, flatmix70-s1, tuckfloor0-s1, tuckexempt0-s1, tuckrise45-s1). Judged jointly with meshref-tucklook1 per its own pre-registered gate.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same joint-pair gate as meshref-tucklook1 (see that run's doc): MECHANISM-HEALTH CANARY ONLY. PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant with genuine duty>0 AND swing_count>0 tuck-then-press motion AND standard-gate non-flat kinds at-or-above the meshref parent -> promote to an 8M acquisition grid + port into stancemix. PARTIAL if flat-probe shows >=2/6 valid per seed or any genuine swinging tuck motion short of valid_plant, while non-flat holds >= the meshref parent -> dose tuck_lookahead_s up/down. FAIL if flat stays 0-1/12 valid or non-flat regresses below the meshref parent -> the script-index floor is refuted too.

