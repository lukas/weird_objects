# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T14:10:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1

**wandb_id**: sqprmus4

**hypothesis**: Plain English: the bare RL fine-tune from the dual-teacher BC init (cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1/-s1) destroyed the stance skills instead of holding them -- hold TERMINATES in 24/24 episodes cross-seed (det+sto, hold_min_load/hold_low_height, both seeds 6/6+6/6) and lower fails height-tracking (0/6 success det+sto both seeds, several over_current terms) while walk stayed a weak crawl (prog_ratio 0.19-0.38, dir_err 47-91deg, far below the BC-parents own ~2600-2700 return) -- exactly the FAIL branch that runs gate pre-registered. This arm applies the cw-arch-gru-dual1-proven fix: turn ON the mesh-canonical stance BC-anchor bundle (rise/hold/lower state-aligned + stratified + IK-lower + rise-ref pursuit, coef=3.0, the same footlow2/acq8m bundle already proven to hold stance under RL) while explicitly keeping the WALK-tick anchor OFF (train.bc_anchor_walk=0.0) so PPO gradients on stance ticks get pulled back toward the proven teacher while walk ticks train unconstrained through their own dedicated dual-core head -- the identical anchor2-style recipe that let cw-arch-gru-dual1 keep real walk translation (prog_ratio 0.95) AND recover hold/lower to champion level after the plain shared-trunk anchor had frozen walk. Same base recipe/env cfg as modeseq1, same seed pairing (0 and 1), only the bc_anchor_* bundle added.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (same scope as the parent pair). 2M canary, own-cfg gate+owncfg pod_eval auto, judged jointly across both seeds. PASS if hold AND lower termination/failure collapses from majority (this arms parent: hold 6/6+6/6 TERM, lower 0/6+0/6 success) to isolated (<=1/6 term each of det/sto) on BOTH seeds, AND walk keeps or improves its (currently weak, prog_ratio 0.19-0.38) translation on video -- no new freeze/paddle. FAIL if hold/lower remain majority-term/majority-fail at the same signature even with the anchor active: that would mean the dual-core routing itself leaks stance-destructive gradients rather than the walk-fine-tune being merely unconstrained, escalating to a routing/gradient-isolation dig-in (do NOT fund a 3rd stance-anchor dose/config before that audit). Read reward trend alongside eval per the 08-21 ruling.

