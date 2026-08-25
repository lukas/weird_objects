# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt0-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T19:14:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0-s1

**wandb_id**: zigatklw

**hypothesis**: Seed twin of tuckexempt0 (train.bc_anchor_min_h_ahead_mm restored 0->8 + new train.bc_anchor_min_h_tuck_exempt_i0=1, gating the height-floor off only inside the mesh ref's tuck segment < ramp_i0): does the tuck-exempt fix work robustly across seeds, or was tuckexempt0 seed luck? Same joint-pair discipline as every mechanism hedge this campaign (meshref-s1, flatmix70-s1, tuckfloor0-s1). Judged jointly with tuckexempt0 per its own pre-registered gate.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same flat-pinned probe + standard DR-0 gate as tuckexempt0; joint pair read against tuckfloor0/-s1 (0/12 flat valid, 0/12 oc, duty=0 freeze; standard gate 0/6+0/6) and the meshref parent (det 5/6 + sto 4/6, oc 3/12, no freezes). PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant (genuine duty>0 tuck motion, not a freeze) AND standard-gate non-flat kinds at-or-above the meshref parent -> promote to an 8M acquisition grid + port into stancemix. PARTIAL if flat-probe improves over tuckfloor0's 0/12 while non-flat holds >= tuckfloor0's own floor -> extend budget. FAIL if flat stays 0-1/12 valid with either the original 2.64A press-up or the same duty=0 freeze, or non-flat regresses below the meshref parent -> tuck-segment start curriculum next, not more anchor plumbing.

