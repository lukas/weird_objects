# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tucklook1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T20:28:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 6ewwghw3

**hypothesis**: Does keying the BC-anchor's anti-freeze floor to SCRIPT INDEX instead of ACHIEVED HEIGHT let the mesh-native ref's tuck segment train without freezing? Prior chain: the height-floor (min_h_ahead_mm=8) skips the whole height-flat tuck (meshref/flatmix70, jumps straight to ramp_i0 from any flat/tuck state); turning it off in-tuck only (tuck_exempt, tuckfloor0/tuckexempt0, 4/4 seeds) collapsed into a total duty=0 freeze because the bare 0.25s time lookahead barely moves the matched index through the tuck's low-pose-delta trajectory -> near-zero anchor gradient; and editing the REF's own height content (tuckrise15/45, --tuck-rise-mm) was refuted by construction (achieved height always lags into the press segment regardless of dose). New lever (this run): keep tuck_exempt=1 (floor off in-tuck, unchanged) but ALSO widen the lookahead itself to a fixed 1.25s SCRIPT-INDEX offset from the current match whenever the match is still inside the tuck (train.bc_anchor_tuck_lookahead_s, new cfg, default 0=off/bit-exact, unit-tested test_bc_anchor.py) -- this can never get stuck the way the height floor did because it advances relative to wherever the match CURRENTLY sits, never relative to achieved height. Single lever vs the meshref parent (which already has tuck_exempt off, since it never needed it -- this canary adds tuck_exempt=1 + tuck_lookahead_s=1.25 together, the exact pairing the tuckexempt0 dig-in named as the surviving design).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, judged jointly with the seed-1 twin as a pair. Primary evidence = the flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto n=6+6, DR-0, run ON THE POD) + the standard DR-0 gate for non-flat kinds, vs BOTH baselines: tuckfloor0/tuckexempt0 (0/12 flat valid, duty=0 total freeze; standard gate 0/6+0/6) and the meshref parent (det 5/6 + sto 4/6 valid, oc 3/12, no freezes). PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant with genuine duty>0 AND swing_count>0 tuck-then-press motion (not a static drag-fold) AND standard-gate non-flat kinds at-or-above the meshref parent -> promote to an 8M acquisition grid + port into stancemix. PARTIAL if flat-probe shows >=2/6 valid per seed or any genuine swinging tuck motion short of valid_plant, while non-flat holds >= the meshref parent -> dose tuck_lookahead_s up/down (try 0.75/2.0). FAIL if flat stays 0-1/12 valid (freeze OR the never-tucks 2.64A press-up with zero swings) or non-flat regresses below the meshref parent -> the script-index floor is refuted too; next suspect is reward/termination pricing of the tuck phase itself, not more bc_anchor plumbing.

