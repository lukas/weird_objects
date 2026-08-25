# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T18:16:22+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0

**hypothesis**: Seed-1 hedge for tuckfloor0 (turn OFF the BC anchor height-floor so flat starts are supervised through the ref's tuck segment instead of being anchored past it): is the floor-skip fix's effect recipe-level or seed-luck? Exact tuckfloor0 spec (meshref 2M canary recipe, single lever train.bc_anchor_min_h_ahead_mm 8->0), only the seed changed, mirroring every mechanism-canary hedge this campaign (meshref-s1, flatmix70-s1). Root cause being tested (measured in the flatmix70-s1 dig-in): rise_ref_mesh_scripted.npz tucks for ticks 0-245 at exactly h=0.0mm before pressing 0->83mm; the min_h_ahead=8mm floor makes the anchor target jump from any h~0 state to tick ~258 (press phase), so flat starts were never supervised to tuck — prediction-if-true: flat det/sto starts now tuck first (like the 0.53A scripted replay) and the flat-pinned probe converts in BOTH seeds; prediction-if-false: flat stays a 2.64A press-up (anchor supervision not the binding constraint -> tuck-segment start curriculum next) or press-phase stalls return on non-flat kinds (the floor's anti-stall was load-bearing -> code the tuck-exempt floor, active only at/after ramp_i0).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, judged JOINTLY with tuckfloor0 (seed 0) as a pair. Primary evidence = flat-pinned probe ON THE POD (eval_checkpoint --cfg-set goal.rise_flat_frac=1.0 --cfg-set goal.rise_partial_frac=0.0 --cfg-set goal.rise_rsi_frac=0.0, det+sto n=6+6, DR-0) + the standard DR-0 gate for non-flat kinds, vs baselines: flatmix70-s1 flat probe 0/12 valid 12/12 oc@2.64A; meshref canary pair det 5/6 sto 4/6 oc 3/12. Joint pair read: PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant with oc<=2/12 AND standard-gate non-flat kinds not regressed vs the meshref pair (bridge/rsi valid with cur in the 0.7-2.2A band) -> promote 8M + port floor0 into stancemix. PARTIAL if flat-probe >=2/6 valid per seed or oc halves (<=6/12) with genuine tuck motion on video -> floor removal works but pursuit underpowered; code the tuck-exempt floor (floor only at/after ramp_i0) next. FAIL if flat probe stays 0-1/12 with the same 2.64A never-tucks press-up -> anchor supervision refuted as the binding constraint, next is tuck-segment start curriculum; if instead non-flat kinds collapse into freezes/stalls -> the floor was load-bearing, code the tuck-exempt floor. Watch the det strips: a valid flat episode must show tuck-then-press, not a lucky splayed balance.

