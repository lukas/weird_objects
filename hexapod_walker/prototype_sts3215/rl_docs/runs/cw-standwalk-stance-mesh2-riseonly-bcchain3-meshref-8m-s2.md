# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T17:12:46+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 6pwzsqht

**hypothesis**: Seed-2 member of the meshref-8m acquisition pass-rate grid (see cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m): does the full 8M budget convert the flat-prone/rsi over_current residue the 2M canary pair left, with everything else already working? Exact canary recipe, only steps 2M->8M and seed changed; judged jointly with s0/s1 as a 3-seed pass-rate.

**gate**: Same pre-registered grid gate as cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-8m: DR-0 rise det+sto n=6+6; per-seed PASS = det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current; grid PASS = >=2/3 seeds -> proceed to stancemix distillation prep. PARTIAL = beats canary counts/terms short of the bar. FAIL = plateau at canary levels across all seeds -> budget refuted, next rung is a targeted tuck mechanism. Cross-check height_err_end_mm.

**verdict**: FAIL per the grid's pre-registered per-seed bar (budget refuted branch): 8M seed-2 is eval-identical to the 2M canary pair — DR-0 rise det 5/6 + sto 4/6 valid_plant, over_current 3/12 (det flat + 2 sto rsi, all pinned 2.64A), valid bridge episodes at cur_p95 2.24-2.33A (>1.5A clause). Video confirms the same splayed-front press-up: front legs never tuck under, press against the extended lever arm, body stuck at h_err 20-25mm. Training reward 1134 and rising, so per the 08-21 ruling this is reward-eval misalignment on the residual subclass, not non-learning — but budget is now refuted at 2M/5M/8M across 3 seeds (s0 PARTIAL oc 2/12 is the only, marginal, improvement). With s2 failing the full bar, the grid's >=2/3-seed PASS is arithmetically impossible regardless of s1; the pre-registered FAIL route stands: attack the ref's tuck segment (start-mix weighting toward flat, or tuck-phase anchor dose). Grid-level verdict + FAIL-route funding belong to the -8m-s1 cycle. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_8m_s2_gate/, W&B 6pwzsqht.

