# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-flatmix70-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-25T17:47:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-flatmix70

**wandb_id**: xz8urts2

**hypothesis**: Seed-1 hedge for flatmix70 (double flat-start exposure 0.35->0.70 within the non-rsi half, single lever, from-scratch): is the flat-exposure lever's effect (if any) recipe-level or seed-luck? Exact flatmix70 recipe, only seed changed, mirroring every other mechanism canary this campaign hedged (bcanchor3-s1, loweronly-bcchain3-s1, meshref-s1). Judged jointly with flatmix70 as a 2-seed pair before any 8M acquisition commitment.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same flat-pinned probe as flatmix70 (--cfg-set goal.rise_flat_frac=1.0 --cfg-set goal.rise_partial_frac=0.0 --cfg-set goal.rise_rsi_frac=0.0, det+sto n=6+6, DR-0) plus the standard DR-0 gate for non-flat kinds. Joint pair read: PASS if BOTH seeds clear flat det>=4/6 AND sto>=4/6 valid_plant with zero over_current and non-flat kinds not regressed -> promote 8M + port into stancemix. PARTIAL if seeds disagree or both land in the PARTIAL band (>=2/6 flat valid or over_current halved) -> dose flat higher or promote with caution. FAIL if both plateau at 0-1/12 flat valid with the same never-tucks press-up signature -> exposure refuted as the lever regardless of seed, next is ref-content/phase treatment (tuck-phase anchor dose or tuck-segment curriculum).

**verdict**: CANARY FAIL - MECHANISM: seed-1 replicates flatmix70's refutation exactly. Flat-pinned probe (rise_flat_frac=1.0/partial=0/rsi=0, det+sto n=6+6, DR-0): 0/12 valid_plant, every episode over_current at 2.64A, height_err_end 25-77mm -- same never-tucks signature. Standard DR-0 gate: det 0/6 (flat 0/4, rsi 0/2), sto 4/6 (rsi 2/2, bridge 2/2, flat 0/2); own-DR det 1/6 + sto 0/6 (bridge/rsi/flat all over_current). Training reward falls every quarter (11.3/3.0/-49.4/-307.1, ep_rew_mean -395), same shape as seed 0. Joint pair result: 2/2 seeds CANARY FAIL - MECHANISM, 0/24 combined flat-pinned valid_plant -- exposure is decisively refuted as the tuck lever, not a seed fluke. Rung-9's next lever per its own pre-registered fallback is ref-content/phase treatment (tuck-phase anchor dose or tuck-segment curriculum edit of rise_ref_mesh_scripted.npz), not more mix dosing.

