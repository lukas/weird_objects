# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-fullpace2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS

**created**: 2026-08-25T17:19:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 3kgxqusz

**hypothesis**: Does the rise still need the slowed-down copying schedule now that the demonstration itself is one the motors can afford? The half-pace anchor chain (lookahead 0.25s / min_h 8mm) was dosed entirely against the OLD infeasible primitive-extracted ref, where slowing the pursuit compensated for a demonstration paced above the servo velocity limit; the dose curve may not transfer to the mesh-native quasi-static ref (feasible at 0.53A open-loop). Exact meshref recipe with ONLY the chain pace reverted to full-pace (bc_anchor_lookahead_s 0.25->0.5, bc_anchor_min_h_ahead_mm 8->15). RELAUNCH of the lost fullpace2 (the 17:06Z fullpace launch omitted these two overrides and was killed as a meshref-s0 duplicate; the claimed fullpace2 relaunch never landed in ledger/W&B/pods). Judged jointly with -s1 against the meshref s0/s1 canary pair. Prediction-if-true: matches or beats meshref det 5/6 with same-or-cooler currents -- pace was only compensating for ref infeasibility. Prediction-if-false: regresses vs meshref (more pins/freezes) -- half-pace is a real requirement independent of ref content.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, DR-0 rise det+sto n=6+6 (same harness as meshref/slowchain), judged jointly with -s1 as a pass-rate vs the meshref s0/s1 pair. PASS: det>=5/6 AND sto>=4/6 valid_plant with valid-episode currents at-or-below the meshref band (no episode above 2.25A) and over_current terms <=3/12 -- full pace transfers; prefer it (one less tuned knob), re-dose future arms against the new ref. PARTIAL: det 4/6 or mixed read -- keep half-pace default, note subclass. FAIL: det<=3/6 or oc terms >3/12 or any all-six-leg freeze (cross-check height_err_end_mm) -- half-pace required independent of ref feasibility; axis closed, keep 0.25s/8mm.

**verdict**: CANARY PASS (PARTIAL branch — mechanism healthy, but full pace is NOT preferred; keep half-pace default). Full-pace chain on the mesh-native ref trains without collapse to the full 2M budget, but the gate lands in the pre-registered PARTIAL/mixed-read branch: DR-0 rise det 4/6 (vs meshref half-pace pair's 5/6 both seeds — the regression is a NEW bridge det failure, splayed-front press-up pinned 2.64A at h_err 73mm, absent in both half-pace seeds) + sto 5/6 (one better), oc 3/12 (equal), one valid sto episode at cur_p95 2.26A just above the 2.25A band, no freezes. Decision per gate: keep half-pace (bc_anchor_lookahead_s=0.25 / min_h_ahead_mm=8) as default pacing for the mesh ref; the joint pass-rate with -fullpace2-s1 (still training) can at best TIE the meshref pair on det, so the preference cannot flip — s1 completes the record only. Watcher SUSPECT was a false alarm: clean budget-complete exit at 2,031,616 steps, W&B 3kgxqusz state=finished. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_fullpace2_gate/.

