# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T18:13:39+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**hypothesis**: Does turning OFF the BC anchor's height-floor lookahead let the robot finally learn to tuck its sprawled legs before pressing up from flat? The flatmix70 dig-in found the defect the whole rung-8/9 grid was circling: the state-aligned anchor's min_h_ahead_mm=8 floor requires the target ref tick to be >=8mm above the CURRENT chassis height, but the mesh scripted ref's entire tuck phase (ticks 0-245, 4.9s of 10.5s) is height-flat (-0.1..+1.6mm, belly carries the mass by design) - so from any flat/tuck state the anchor target jumps to the PRESS phase (tick 257+), supervising press-from-sprawl on every flat start. Exposure (flatmix70: 0/12 flat-pinned), budget (8M grid), pace (fullpace pair), and anchor dose all failed because they multiply or reschedule the WRONG supervision. Single config lever, defect-removal: train.bc_anchor_min_h_ahead_mm 8->0 (the code default; never run on the mesh ref - only 15/8/4/2 ever used), everything else the exact meshref canary recipe. The floor's own stall pathology does not apply here: the tuck sweeps ~90deg mean joint angle so nearest-q pursuit advances on q-distance, and the press phase is height-steep so the 0.25s time lookahead suffices there. Prediction-if-true: flat starts show visible tuck (feet sweep to plant footprint before the body lifts) and convert from the 2.64A press-up pin. Prediction-if-false: flat stays 0-1/12 with no tuck motion - the anchor pursuit itself stalls in the tuck or the defect is elsewhere; next is a code-level phase-capped floor (floor active only at/after ramp_i0) or tuck-segment curriculum.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. 2M canary, judged jointly with -s1. Primary evidence = flat-pinned probe ON THE POD (eval_checkpoint --cfg-set goal.rise_flat_frac=1.0 --cfg-set goal.rise_partial_frac=0.0 --cfg-set goal.rise_rsi_frac=0.0 --cfg-set safety.max_delta_q_deg=1.5, det+sto n=6+6, DR-0; artifacts like logs/ckpt_eval/flatmix70_fpin_det). PASS = flat det>=4/6 AND sto>=4/6 valid_plant, zero over_current, h_err_end<=10mm on valid episodes, AND standard DR-0 gate shows non-flat kinds not regressed vs the meshref canary pair (det 5/6 + sto 4/6, valid-ep cur_p95 in 0.7-2.2A band) -> fund 3-seed 8M acquisition with floor=0. PARTIAL = flat-pinned >=2/6 valid OR over_current rate halves vs flatmix70's 12/12 with genuine tuck motion visible on the flat strips (feet sweep inward before body lift) -> mechanism works but slow; extend budget or combine floor=0 with flatmix. FAIL = flat-pinned still 0-1/12 valid with the same 2.64A zero-swing press-up and NO tuck motion on video -> floor removal refuted; next rung is the code-level phase-capped floor or tuck-segment curriculum. Also watch for NEW press-phase stalls (the floor existed to prevent pursuit pinning) - a freeze at low current mid-press counts against PASS regardless of counts.

