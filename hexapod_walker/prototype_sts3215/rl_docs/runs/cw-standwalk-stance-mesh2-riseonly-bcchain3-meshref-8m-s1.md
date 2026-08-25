# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T17:08:43+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: q5fttoat

**hypothesis**: Seed-1 member of the meshref-8m acquisition pass-rate grid (see cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m): does the full 8M budget convert the flat-prone/rsi over_current residue the 2M canary pair left, with everything else already working? Exact canary recipe, only steps 2M->8M and seed changed; judged jointly with s0/s2 as a 3-seed pass-rate.

**gate**: Same pre-registered grid gate as cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m: DR-0 rise det+sto n=6+6; per-seed PASS = det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current; grid PASS = >=2/3 seeds -> proceed to stancemix distillation prep. PARTIAL = beats canary counts/terms short of the bar. FAIL = plateau at canary levels across all seeds -> budget refuted, next rung is a targeted tuck mechanism. Cross-check height_err_end_mm.

