# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T17:04:23+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**hypothesis**: Does giving the mesh-native-reference rise recipe its full 8M acquisition budget finish the job its 2M canary nearly completed — flat-prone and rsi starts included? The canary pair (meshref/meshref-s1, CANARY PASS 08-25) proved the scripted 0.53A-feasible reference unpins deep starts (det 5/6 + sto 4/6 valid_plant vs slowchain 3/6+2/6, valid-ep cur_p95 median 1.19-1.36A), leaving only the flat-prone start and 2 rsi-sto episodes over_currenting mid-rise (splayed front leg never tucks). At 2M the std had already annealed to 0.018; an 8M schedule anneals slower, giving mid-training exploration to learn the tuck. Exact canary recipe (mesh model, scripted ref, half-pace chain lookahead 0.25s/min_h 8mm, anchor coef 3.0, from scratch), ONLY steps 2M->8M. Prediction-if-true: flat/rsi failures convert, zero over_current, full PASS bar. Prediction-if-false: same 3/12 over_current residue at 8M — budget/exploration is not the flat-start lever; the tuck phase needs its own mechanism (start-mix weighting or tuck-phase anchor dose), pre-registered as the next rung.

**gate**: DR-0 rise gate, det+sto n=6+6, same harness as the canary pair; judge the 3-seed grid (s0/s1/s2) jointly as a pass-rate. PASS (per seed): det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms; grid PASS = >=2/3 seeds pass -> recipe is the standwalk rise champion path, proceed to stancemix (rise+lower) distillation prep. PARTIAL: seeds beat the canary pair (det>5/6 or sto>4/6 or over_current terms <3/12 or flat-start det/0 converts on >=2 seeds) short of the full bar -> budget helps; examine which start kinds remain and pre-register the targeted mechanism. FAIL: counts/currents plateau at canary levels (det 5/6, sto 4/6, 3/12 over_current) across all 3 seeds with anchor loss converged -> budget refuted for the flat-start residue; next rung is a targeted tuck mechanism (start-mix weighting toward flat, or tuck-phase anchor dose), never more undirected budget. Always cross-check height_err_end_mm; current/terms alone are gameable.

