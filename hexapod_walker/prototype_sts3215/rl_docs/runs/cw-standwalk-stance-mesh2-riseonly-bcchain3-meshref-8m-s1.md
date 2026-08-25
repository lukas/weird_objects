# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T17:08:43+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: q5fttoat

**hypothesis**: Seed-1 member of the meshref-8m acquisition pass-rate grid (see cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m): does the full 8M budget convert the flat-prone/rsi over_current residue the 2M canary pair left, with everything else already working? Exact canary recipe, only steps 2M->8M and seed changed; judged jointly with s0/s2 as a 3-seed pass-rate.

**gate**: Same pre-registered grid gate as cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m: DR-0 rise det+sto n=6+6; per-seed PASS = det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current; grid PASS = >=2/3 seeds -> proceed to stancemix distillation prep. PARTIAL = beats canary counts/terms short of the bar. FAIL = plateau at canary levels across all seeds -> budget refuted, next rung is a targeted tuck mechanism. Cross-check height_err_end_mm.

**verdict**: 8M acquisition, seed 1 of the meshref rise-ref grid, PLATEAUS at its own 2M canary exactly -- no seed-level gain from 4x budget. DR-0 gate det 5/6 + sto 4/6 valid_plant (identical counts to the meshref-s1 2M canary), over_current 3/12 (1 det:flat + 2 sto:rsi -- the SAME start-kind failure set the canary named), and 2 of the 5 valid det episodes (both bridge, cur_p95 2.27A/1.97A) exceed the gate's <=1.5A-on-every-valid-episode clause, so this seed clears neither the strict PASS bar nor even the PARTIAL 'beats canary' bar on its own. own-DR(0.2): det 3/6 (bridge/flat/rsi over_current) + sto 6/6 clean. Video (rise_det_0 flat, rise_sto_4/5 rsi) shows the same qualitative story as every rung-8/9 sibling: legs tuck, body lifts toward a splayed-but-planted stand, then trips current mid-hold -- genuine progress, not a freeze, but budget-invariant for this seed. Training reward is strongly rising (quarters -18/-167/+340/+1041, ep_rew_mean 1515) -- per the 08-21 ruling this alone doesn't disqualify, but paired with a flat eval it means realign-or-more-budget is not obviously the next move for THIS seed. Concurrent cycle's seed-0 (acq8m) read a small gain instead (oc 2/12<3/12, sto 5/6). Grid is judged jointly (>=2/3 seeds at the strict bar); seed-2 (meshref-8m-s2) still training -- do not close the grid without it. Individually this run is closer to the gate's own FAIL description ('plateau at canary levels') than to PARTIAL, but the joint call waits on s2.

