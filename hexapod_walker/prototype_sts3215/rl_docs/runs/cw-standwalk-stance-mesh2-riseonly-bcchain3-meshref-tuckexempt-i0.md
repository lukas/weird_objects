# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt-i0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - INFRASTRUCTURE

**created**: 2026-08-25T19:10:48+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0

**wandb_id**: j9k3h490

**hypothesis**: Does gating the BC-anchor height-floor OFF only while the state-aligned match is still inside the mesh ref's own height-flat tuck segment (ticks < ramp_i0=245) let flat starts learn to tuck-then-press, while restoring the floor unchanged in the press segment keeps the anti-freeze role that tuckfloor0/-s1 proved load-bearing? tuckfloor0 (floor OFF everywhere, min_h_ahead_mm=0) CANARY FAIL-MECHANISM'd both seeds: flat probe collapsed to a total duty=0 freeze (0/12 valid, 0/12 over_current, height stuck 79-86mm) AND previously-clean bridge/crouch/rsi starts on the standard DR-0 gate collapsed too (5/6+4/6 -> 0/6+0/6), proving the floor's original 08-12 anti-stall purpose is load-bearing in the press segment even though it's wrong in the tuck segment. Single new lever, code-level (train.bc_anchor_min_h_tuck_exempt_i0, default 0=legacy bit-exact, 4 new unit tests green 59/59 in test_bc_anchor.py, snapshot exp/cw-standwalk-tuckexempt-i0): restore min_h_ahead_mm to 8 (the meshref/flatmix70 value) but suppress the floor while bc_j < ref['ramp_i0'], so tuck ticks get pure time-lookahead pursuit (genuine tuck tracking) and press ticks keep the exact proven anti-freeze floor unchanged. Prediction-if-true: flat-pinned probe shows visible tuck motion (feet sweep to plant footprint, duty_cycle>0 early) converting toward the meshref canary's non-flat cur_p95 band (0.7-2.2A), with bridge/crouch/rsi NOT collapsing back to freeze/2.64A. Prediction-if-false: flat still freezes or re-pins at 2.64A with no tuck motion -> the floor-vs-content split is refuted and the tuck-segment defect is in the reference's OWN posture content (splayed initial pose), not the anchor's floor logic.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY, judged jointly with -s1. Primary evidence = flat-pinned probe ON THE POD (eval_checkpoint --cfg-set goal.rise_flat_frac=1.0 --cfg-set goal.rise_partial_frac=0.0 --cfg-set goal.rise_rsi_frac=0.0 --cfg-set safety.max_delta_q_deg=1.5, det+sto n=6+6, DR-0). PASS = flat det>=4/6 AND sto>=4/6 valid_plant with visible tuck motion on video (feet sweep to plant footprint / duty_cycle>0 before ramp_i0-equivalent time), zero over_current, AND standard DR-0 gate shows non-flat kinds NOT regressed vs the meshref canary pair (det 5/6 + sto 4/6, valid-ep cur_p95 in 0.7-2.2A band) -> fund 3-seed 8M acquisition. PARTIAL = flat-pinned improves over tuckfloor0's 0/12 (>=2/6 flat valid on either pass, or genuine tuck motion visible even without full valid_plant) WITHOUT re-collapsing bridge/crouch/rsi below tuckfloor0's own floor (worse than 0/6+0/6 on standard gate) -> extend budget or tune ramp_i0 margin. FAIL = flat still 0-1/12 valid with the same signature (either the 2.64A never-tucks press-up OR the duty=0 freeze) AND no tuck motion on video -> the floor/exempt-index split is refuted; next is direct ref-content edit (reshape the tuck segment's initial splayed posture) or a tuck-phase curriculum (start distribution weighted toward mid-tuck states).

**verdict**: CANARY FAIL - INFRASTRUCTURE: exact duplicate launch, not a science result. Bit-identical (seed=0, same cfg incl. train.bc_anchor_min_h_ahead_mm=8 + train.bc_anchor_min_h_tuck_exempt_i0=1, lookahead_s=0.25) to the concurrent cycle's cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt0 (created 19:08:39, 2min before mine at 19:10:48, same recipe/hypothesis). Launched believing the pre-registered tuckexempt-i0 fix had never landed (backlog empty, all 12 pods free at capacity check, and the STATUS.md entry I first read was the stale 18:1x one); the concurrent cycle's own launch simply wasn't reflected in what I'd read yet. Killed cleanly on train-1 (no live process found -- already exited/killed in time), no information lost. tuckexempt0/-s1 (train-0/train-2, launched by the concurrent cycle) is the pair of record; do not relaunch.

