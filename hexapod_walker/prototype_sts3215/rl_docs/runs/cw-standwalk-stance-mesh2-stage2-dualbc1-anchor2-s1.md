# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS

**created**: 2026-08-26T17:01:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1

**wandb_id**: wajl5mrl

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2 (identical recipe, seed 1) -- cross-seed replication for the joint leak-fix call. Plain English: tests whether dropping the shared-Adam momentum leak (train.bc_anchor_isolate_update=1, commit 2f585a97) stops the stance-only anchor from wrecking walk, which failed in two DIFFERENT seed-dependent ways on -anchor1/-anchor1-s1 exactly as optimizer-momentum noise predicts.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same as anchor2 seed 0; joint call reads both seeds together (LEAK-FIX PASS / FULL PASS / FAIL-A / FAIL-B branches as registered there).

**verdict**: CANARY PASS (LEAK-FIX CONFIRMED, joint w/ seed0; overall FAIL-B, stance dose next -- not a full pass). Plain English: same leak-fix recipe, seed 1 -- walk mechanism is EVEN CLEANER here than seed0: det gait_valid 6/6 both DR, prog_ratio 0.32-0.37 (matches modeseq1s own band), and sto shows ZERO leg sacrifice at either DR (gait_valid 6/6 both DR0/DR0.5, vs seed0s partial [2,5]/[2,4,5]) -- confirms the isolate_update fix cross-seed, no anchor1-class catastrophe anywhere. slip/m 3.7-4.1 det / 22-28 sto. NOT a full pass, matching seed0 exactly: hold/sto is a clean TOTAL hold_min_load termination in all 6/6 episodes at BOTH DR settings (identical unchanged-parent signature to seed0 and to anchor1 itself). hold/det mixed (2-4/6 fail, worst cell 4/6 fail at own-DR), lower/det weak (2-3/6 fail), lower/sto mostly fine (4-5/6 success). Video: walk_det real cycling gait, hold_sto controlled upright early-stop (min-load trip), not a fall. Joint call with seed0: LEAK-FIX PASS on BOTH seeds (fires the anchors own prediction-if-true), overall result is the gates pre-registered FAIL-B (walk fixed, stance still majority-fail sto). Next: raise train.bc_anchor_coef now that leak-capping is gone, or fund a longer acquisition on this exact recipe, targeting hold/lower -- do not change architecture again before that dose test. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_anchor2_s1_{gate,owncfg}/, W&B wajl5mrl.

