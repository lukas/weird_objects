# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-26T14:13:59+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

**wandb_id**: g6hghec7

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1 (identical recipe, seed 1) -- same run for the cross-seed pass-rate reading the joint-call convention requires before promoting or refuting the stance-only/walk-off bc_anchor fallback.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same as anchor1 seed 0; joint call reads both seeds together.

**verdict**: CANARY FAIL - MECHANISM (joint call with seed0 twin, cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1): same fallback (coef=3.0, walk_anchor=0.0), same collapse, cross-seed replicated. NOTE: this seed's original attempt was mistakenly recorded CANARY FAIL - INFRASTRUCTURE / KILLED by an earlier pass of this same cycle acting on a stale 'deadlock' read; fresh W&B evidence this cycle (state=finished, global_step 2031616, full 787-row history, real matching-size exported checkpoint identical to seed0's) shows the ORIGINAL run actually completed cleanly -- the exact same false-positive shape already documented for modeseq1-s1r one lineage step earlier. The infra-retry it spawned (-anchor1-s1-r1) was killed this cycle at ~1.05M/1M-target steps (superseded, no skill data) and its own ledger entry corrected. This verdict uses the ORIGINAL anchor1-s1 checkpoint's real gate/own-DR reads. Both seeds, DR-0 + own-DR 0.5, det+sto, real per-episode report.json. hold: sto 0/6 success at BOTH DR (matches parent's 6/6 TERM exactly); det 4/6 at DR-0 collapsing to 2/6 at own-DR -- never isolated both det+sto. lower: DR-0 sto reaches isolated (5/6, 1/6 fail) but its own det companion is only 4/6 (2/6 fail) same seed same DR, and own-DR regresses both to 2/6det+3/6sto -- gate needs BOTH cells isolated, never achieved. walk: 0/6 success in all 4 cells, but a DIFFERENT severe pathology than seed0's leg-sacrifice freeze -- gait_valid nominally True (legs cycling) yet progress_ratio NEGATIVE (-0.02 to -0.03) with slip/m 44-49 (~15x the 2.9 joystick cap), i.e. shuffling/skating in place with zero net translation (contact sheet walk_det_0: static crouch, no visible displacement across the 30s strip) -- as catastrophic as seed0's freeze, just a different failure shape, which itself argues against a single clean fixable bug and for a structural gradient-isolation problem. Reward: Q1 +38.6 -> Q3 trough -456.1 -> Q4 -121.5, same aligned bad-and-stuck shape as seed0. Joint verdict: FAIL on every gate clause, cross-seed, with two DIFFERENT walk failure modes -- escalate to the routing/gradient-isolation dig-in per the gate's own pre-registered text; do not fund a third anchor dose/config first. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_anchor1_s1_{gate,owncfg}/ (this run), _anchor1_{gate,owncfg}/ (twin), W&B g6hghec7/sqprmus4.

